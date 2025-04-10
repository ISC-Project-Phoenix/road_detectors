import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
import time


class YoloSubscriberNode(Node):
    def __init__(self):
        super().__init__('yolo_subscriber_node')

        # Load your trained YOLO model (update the path as needed)
        self.model = YOLO('/home/isc-learning2/Documents/ws_redtoo/src/road_detectors/obj_detector_ai/obj_detector_ai/weights/bestJ.pt')
        self.model.conf = 0.75  # Confidence threshold

        # Define drawing colors in BGR format
        self.left_color = (255, 0, 0)    # Blue for left boundary
        self.center_color = (0, 255, 0)  # Green for centerline
        self.right_color = (0, 0, 255)   # Red for right boundary

        # Define smoothing parameters
        self.alpha = 0.2
        self.smoothed_left_poly_coeff = None
        self.smoothed_center_poly_coeff = None
        self.smoothed_right_poly_coeff = None

        # Create ROS2 Image Publisher (processed output)
       
       
        self.publisher = self.create_publisher(Image, 'processed_frames', 10)
        self.poly_coeff_publisher = self.create_publisher(Float32MultiArray, '/road/polynomial', 5)

        # Subscribe to the ROS2 image topic (change the topic as required)
        self.subscription = self.create_subscription(Image, '/camera/mid/rgb/image_color', self.image_callback, 10)
        self.subscription = self.create_subscription(CompressedImage, '/camera/mid/rgb/compressed',self.image_callback, 10)

        # OpenCV Bridge
        self.bridge = CvBridge()

        self.get_logger().info("Lane Detection Node Started.")

    def get_boundary_points(self, binary_mask, step=5):
        """
        Extract left, center, and right boundary points from a binary mask.
        """
        left_points, center_points, right_points = [], [], []
        h, w = binary_mask.shape
        for y in range(0, h, step):
            x_coords = np.where(binary_mask[y, :] > 0)[0]
            if x_coords.size > 0:
                left_x = x_coords[0]
                right_x = x_coords[-1]
                center_x = int((left_x + right_x) / 2)
                left_points.append((left_x, y))
                center_points.append((center_x, y))
                right_points.append((right_x, y))
        return np.array(left_points), np.array(center_points), np.array(right_points)

    def fit_polynomial(self, points, smoothed_coeff):
        if points.shape[0] >= 3:
            try:
                # Fit a 2nd-degree polynomial using the y-coordinates as the independent variable
                poly_coeff = np.polyfit(points[:, 1], points[:, 0], deg=2)
                if smoothed_coeff is None:
                    smoothed_coeff = poly_coeff
                else:
                    smoothed_coeff = self.alpha * poly_coeff + (1 - self.alpha) * smoothed_coeff

                # Publish polynomial coefficients
                coeff_msg = Float32MultiArray()
                coeff_msg.data = smoothed_coeff.tolist()
                self.poly_coeff_publisher.publish(coeff_msg)

                self.get_logger().info(f"Published Polynomial Coefficients: {smoothed_coeff}")
                return np.poly1d(smoothed_coeff), smoothed_coeff
            except Exception as e:
                self.get_logger().warn(f"Polynomial fitting error: {e}")
        return None, smoothed_coeff

    def image_callback(self, msg):
        """Process frames from ROS2 topic."""
        # Convert ROS2 Image message to an OpenCV image (BGR)
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if frame is None or frame.shape[0] == 0 or frame.shape[1] == 0:
            self.get_logger().warn("Received an empty or invalid frame; skipping processing.")
            return

        # --- Use Full Frame (No Cropping) ---
        cropped_frame = frame  # Using the full frame directly

        # Run YOLO segmentation on the full frame
        start = time.time()
        results = self.model(cropped_frame)
        end = time.time()

        # Create a blank binary mask (grayscale)
        binary_mask = np.zeros(cropped_frame.shape[:2], dtype=np.uint8)

        # Process each detection result from YOLO
        for result in results:
            if result.masks is not None:
                self.get_logger().info(f"Masks detected: {len(result.masks.xy)}")
                for mask, clsa in zip(result.masks.xy, result.boxes.cls):
                    points = np.array(mask, dtype=np.int32)
                    # Check class index (0 assumed to be "Road") and ensure mask has enough points
                    if int(clsa) == 0 and points.shape[0] >= 3:
                        points = points.reshape((-1, 1, 2))
                        cv2.fillPoly(binary_mask, [points], 255)

        # Log unique values in the binary mask for debugging
        unique_vals = np.unique(binary_mask)
        self.get_logger().info(f"Unique mask values: {unique_vals}")

        # Perform morphological closing to refine the mask
        kernel = np.ones((5, 5), np.uint8)
        binary_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_CLOSE, kernel)

        # Extract boundary points from the mask
        left_points, center_points, right_points = self.get_boundary_points(binary_mask, step=5)

        # Fit polynomials for left, right, and center boundaries
        left_poly_func, self.smoothed_left_poly_coeff = self.fit_polynomial(left_points, self.smoothed_left_poly_coeff)
        right_poly_func, self.smoothed_right_poly_coeff = self.fit_polynomial(right_points, self.smoothed_right_poly_coeff)
        center_poly_func, self.smoothed_center_poly_coeff = self.fit_polynomial(center_points, self.smoothed_center_poly_coeff)

        # --- Calculate the Curvature of the Center Polynomial ---
        # For a curve x = f(y), curvature at y is:
        #   κ = |f''(y)| / (1 + (f'(y))^2)^(3/2)
        if center_poly_func is not None:
            # Use the bottom of the image (maximum y value) for evaluation
            y_eval = binary_mask.shape[0] - 1
            first_deriv = center_poly_func.deriv(m=1)
            second_deriv = center_poly_func.deriv(m=2)
            curvature = abs(second_deriv(y_eval)) / (1 + first_deriv(y_eval)**2) ** 1.5
            curvature_text = f"Curvature: {curvature:.4f}"
        else:
            curvature_text = "Curvature: N/A"

        # Create an overlay image from the full frame
        overlay = cropped_frame.copy()
        # Optionally draw center points (for demonstration)
        if center_poly_func:
            for y in range(0, binary_mask.shape[0]):
                x_center = int(center_poly_func(y))
                if 0 <= x_center < binary_mask.shape[1]:
                    cv2.circle(overlay, (x_center, y), 2, self.center_color, -1)

        # Display FPS on the overlay image
        fps_text = f'{1 / (end - start):.2f} FPS'
        cv2.putText(overlay, fps_text, (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        # Publish the overlay image via ROS2 topic
        ros_image = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        self.publisher.publish(ros_image)

        # --- Blend the Segmentation Mask and Overlay in a Single Window ---
        colored_mask = cv2.applyColorMap(binary_mask, cv2.COLORMAP_JET)
        combined = cv2.addWeighted(overlay, 0.7, colored_mask, 0.3, 0)

        # Overlay the curvature text on the combined image (in red font)
        cv2.putText(combined, curvature_text, (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow("Combined", combined)

        # Check for key press; if 'q' is pressed, close windows and shutdown ROS2
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = YoloSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()


# JARED/TEAM WILL DELETE ALL NEEDED CODE BELOW SOON.

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import CompressedImage, Image
# from std_msgs.msg import Float32MultiArray
# from cv_bridge import CvBridge
# import cv2
# from ultralytics import YOLO
# import numpy as np
# import time


# class YoloSubscriberNode(Node):
#     def __init__(self):
#         super().__init__('yolo_subscriber_node')

#         # Load your trained YOLO model (update the path as needed)
#         self.model = YOLO('/home/isc-learning2/Documents/ws_redtoo/src/road_detectors/obj_detector_ai/obj_detector_ai/weights/bestJ.pt')
#         self.model.conf = 0.75  # Confidence threshold

#         # Define drawing colors in BGR format
#         self.left_color = (255, 0, 0)    # Blue for left boundary
#         self.center_color = (0, 255, 0)  # Green for centerline
#         self.right_color = (0, 0, 255)   # Red for right boundary

#         # Define smoothing parameters
#         self.alpha = 0.2
#         self.smoothed_left_poly_coeff = None
#         self.smoothed_center_poly_coeff = None
#         self.smoothed_right_poly_coeff = None

#         # Create ROS2 Image Publisher (processed output)
#         self.publisher = self.create_publisher(Image, 'processed_frames', 10)
#         self.poly_coeff_publisher = self.create_publisher(Float32MultiArray, '/road/polynomial', 5)

#         # Subscribe to the ROS2 image topic (change the topic as required)
#         self.subscription = self.create_subscription(Image, '/camera/mid/rgb/image_color', self.image_callback, 10)

#         # OpenCV Bridge
#         self.bridge = CvBridge()

#         self.get_logger().info("Lane Detection Node Started.")

#     def get_boundary_points(self, binary_mask, step=5):
#         """
#         Extract left, center, and right boundary points from a binary mask.
#         """
#         left_points, center_points, right_points = [], [], []
#         h, w = binary_mask.shape
#         for y in range(0, h, step):
#             x_coords = np.where(binary_mask[y, :] > 0)[0]
#             if x_coords.size > 0:
#                 left_x = x_coords[0]
#                 right_x = x_coords[-1]
#                 center_x = int((left_x + right_x) / 2)
#                 left_points.append((left_x, y))
#                 center_points.append((center_x, y))
#                 right_points.append((right_x, y))
#         return np.array(left_points), np.array(center_points), np.array(right_points)

#     def fit_polynomial(self, points, smoothed_coeff):
#         if points.shape[0] >= 3:
#             try:
#                 # Fit a 2nd-degree polynomial using the y-coordinates as the independent variable
#                 poly_coeff = np.polyfit(points[:, 1], points[:, 0], deg=2)
#                 if smoothed_coeff is None:
#                     smoothed_coeff = poly_coeff
#                 else:
#                     smoothed_coeff = self.alpha * poly_coeff + (1 - self.alpha) * smoothed_coeff

#                 # Publish polynomial coefficients
#                 coeff_msg = Float32MultiArray()
#                 coeff_msg.data = smoothed_coeff.tolist()
#                 self.poly_coeff_publisher.publish(coeff_msg)

#                 self.get_logger().info(f"Published Polynomial Coefficients: {smoothed_coeff}")
#                 return np.poly1d(smoothed_coeff), smoothed_coeff
#             except Exception as e:
#                 self.get_logger().warn(f"Polynomial fitting error: {e}")
#         return None, smoothed_coeff

#     def image_callback(self, msg):
#         """Process frames from ROS2 topic."""
#         frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#         frame_height, frame_width, _ = frame.shape

#         # Crop to the bottom 45% of the frame
#         # crop_y_start = int(frame_height * 0.45)
#         crop_y_start = int(frame_height * 0.45)

#         cropped_frame = frame[crop_y_start:, :]

#         # Run YOLO segmentation
#         start = time.time()
#         results = self.model(cropped_frame)
#         end = time.time()

#         # Create a blank binary mask (grayscale)
#         binary_mask = np.zeros(cropped_frame.shape[:2], dtype=np.uint8)

#         # Process each detection result from YOLO
#         for result in results:
#             if result.masks is not None:
#                 self.get_logger().info(f"Masks detected: {len(result.masks.xy)}")
#                 for mask, clsa in zip(result.masks.xy, result.boxes.cls):
#                     points = np.array(mask, dtype=np.int32)
#                     # Check class index (0 here assumed to be "Road") and ensure the mask has enough points
#                     if int(clsa) == 0 and points.shape[0] >= 3:
#                         points = points.reshape((-1, 1, 2))
#                         cv2.fillPoly(binary_mask, [points], 255)

#         # Log unique values in the binary mask for debugging
#         unique_vals = np.unique(binary_mask)
#         self.get_logger().info(f"Unique mask values: {unique_vals}")

#         # Perform morphological closing to refine the mask
#         kernel = np.ones((5, 5), np.uint8)
#         binary_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_CLOSE, kernel)

#         # Extract boundary points from the mask
#         left_points, center_points, right_points = self.get_boundary_points(binary_mask, step=5)

#         # Fit polynomials for left, right, and center boundaries
#         left_poly_func, self.smoothed_left_poly_coeff = self.fit_polynomial(left_points, self.smoothed_left_poly_coeff)
#         right_poly_func, self.smoothed_right_poly_coeff = self.fit_polynomial(right_points, self.smoothed_right_poly_coeff)
#         center_poly_func, self.smoothed_center_poly_coeff = self.fit_polynomial(center_points, self.smoothed_center_poly_coeff)

#         # Create an overlay image from the cropped frame
#         overlay = cropped_frame.copy()
#         for y in range(0, binary_mask.shape[0]):
#             if left_poly_func:
#                 x_left = int(left_poly_func(y))
#                 # Optionally, draw left boundary if desired.
#             if right_poly_func:
#                 x_right = int(right_poly_func(y))
#                 # Optionally, draw right boundary if desired.
#             if center_poly_func:
#                 x_center = int(center_poly_func(y))
#                 if 0 <= x_center < binary_mask.shape[1]:
#                     cv2.circle(overlay, (x_center, y), 2, self.center_color, -1)

#         # Display FPS on the overlay
#         fps_text = f'{1 / (end - start):.2f} FPS'
#         cv2.putText(overlay, fps_text, (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 1)

#         # Publish the overlay image via ROS2 topic
#         ros_image = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
#         self.publisher.publish(ros_image)

#         # --- Blending into One Window ---
#         # Apply a colormap to the binary mask to enhance visualization
#         colored_mask = cv2.applyColorMap(binary_mask, cv2.COLORMAP_JET)
#         # Blend the overlay and colored mask with 70% and 30% weights respectively
#         combined = cv2.addWeighted(overlay, 0.7, colored_mask, 0.3, 0)
#         cv2.imshow("Combined", combined)

#         # Check for key press; if 'q' is pressed, close windows and shutdown ROS2
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             cv2.destroyAllWindows()
#             rclpy.shutdown()

# def main(args=None):
#     rclpy.init(args=args)
#     yolo_subscriber_node = YoloSubscriberNode()
#     try:
#         rclpy.spin(yolo_subscriber_node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         yolo_subscriber_node.destroy_node()
#         rclpy.shutdown()
#         cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main()


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import CompressedImage
# from sensor_msgs.msg import Image
# from std_msgs.msg import Float32MultiArray
# from cv_bridge import CvBridge
# import cv2
# from ultralytics import YOLO
# import numpy as np
# import time


# class YoloSubscriberNode(Node):
#     def __init__(self):
#         super().__init__('yolo_subscriber_node')

#         # **
#         # (Substitute with your trained YOLO model)
#         # **
#         self.model = YOLO('/home/isc-learning2/Documents/ws_redtoo/src/road_detectors/obj_detector_ai/obj_detector_ai/weights/bestJ.pt')  # Example: '/home/user/best.pt'
#         self.model.conf = 0.75  # Confidence threshold

#         # Define drawing colors in BGR format
#         self.left_color = (255, 0, 0)    # Blue for left boundary
#         self.center_color = (0, 255, 0)  # Green for centerline
#         self.right_color = (0, 0, 255)   # Red for right boundary

#         # Define smoothing parameters
#         self.alpha = 0.2
#         self.smoothed_left_poly_coeff = None
#         self.smoothed_center_poly_coeff = None
#         self.smoothed_right_poly_coeff = None

#         # ROS2 Image Subscriber (input frames)

#        # self.it = ImageTransport(self)

#         # ROS2 Image Publisher (processed output)
#         self.publisher = self.create_publisher(Image, 'processed_frames', 10)
#         self.poly_coeff_publisher = self.create_publisher(Float32MultiArray, '/road/polynomial', 5)

#         # TODO: Figure out how to subscribe correctly to compressed image
#         # self.subscription = self.create_subscription(CompressedImage, '/camera/mid/rgb/compressed',self.image_callback, 10)
#         #self.it.subscribe('/camera/mid/rgb', self.listener_callback, 'compressed')

#         self.subscription = self.create_subscription(Image,  '/camera/mid/rgb/image_color', self.image_callback, 10)


#         # this one publishes the video in this repo
#         #self.subscription = self.create_subscription(Image, '/video_frames' ,self.image_callback, 10)

#         # OpenCV Bridge
#         self.bridge = CvBridge()

#         self.get_logger().info("Lane Detection Node Started.")

#     # for the publisher node DONT USE
#     def listener_callback(self, msg):

#         # Convert compressed image data to a numpy array
#         np_arr = np.frombuffer(msg.data, np.uint8)

#         # Decode the numpy array to an OpenCV image
#         frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

#         #frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#         #start = self.get_clock().now().to_msg().sec_nanosec  # Start time

#         results = self.model(frame)

#         #end = self.get_clock().now().to_msg().sec_nanosec  # End time
#         #fps = 1 / ((end[0] - start[0]) + (end[1] - start[1]) / 1e9)

#         overlay = frame.copy()

#         for result in results:
#             if result.masks is not None:
#                 for mask, clsa in zip(result.masks.xy, result.boxes.cls):
#                     points = np.array(mask, dtype=np.int32).reshape((-1, 1, 2))
#                     color = self.class_colors.get(int(clsa), (255, 255, 255, 100))
#                     if len(points) >= 3:
#                         cv2.fillPoly(overlay, [points], color[:3])  # Ignore alpha

#         frame = cv2.addWeighted(overlay, 0.5, frame, 0.5, 0)

#         # Display FPS
#         #cv2.putText(frame, f'{fps:.2f} FPS', (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 1)

#         # Convert back to ROS2 Image message and publish
#         ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
#         self.publisher.publish(ros_image)

#     def get_boundary_points(self, binary_mask, step=5):
#         """
#         Extract left, center, and right boundary points from a binary mask.
#         """
#         left_points, center_points, right_points = [], [], []
#         h, w = binary_mask.shape
#         for y in range(0, h, step):
#             x_coords = np.where(binary_mask[y, :] > 0)[0]
#             if x_coords.size > 0:
#                 left_x = x_coords[0]
#                 right_x = x_coords[-1]
#                 center_x = int((left_x + right_x) / 2)
#                 left_points.append((left_x, y))
#                 center_points.append((center_x, y))
#                 right_points.append((right_x, y))
#         return np.array(left_points), np.array(center_points), np.array(right_points)

#     def fit_polynomial(self, points, smoothed_coeff):
#         if points.shape[0] >= 3:
#             try:
#                 poly_coeff = np.polyfit(points[:, 1], points[:, 0], deg=2)  # Fit a 2nd-degree polynomial
#                 if smoothed_coeff is None:
#                     smoothed_coeff = poly_coeff
#                 else:
#                     smoothed_coeff = self.alpha * poly_coeff + (1 - self.alpha) * smoothed_coeff

#                 # Create a ROS2 message and publish coefficients
#                 coeff_msg = Float32MultiArray()
#                 coeff_msg.data = smoothed_coeff.tolist()
#                 self.poly_coeff_publisher.publish(coeff_msg)

#                 # Log the coefficients
#                 self.get_logger().info(f"Published Polynomial Coefficients: {smoothed_coeff}")

#                 return np.poly1d(smoothed_coeff), smoothed_coeff
#             except Exception as e:
#                 self.get_logger().warn(f"Polynomial fitting error: {e}")
#         return None, smoothed_coeff

#     # for the ROS2 frames. 
#     def image_callback(self, msg):
#         """Process frames from ROS2 topic."""
#         frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#         # unnesscary for CV
#         frame_height, frame_width, _ = frame.shape

#         # Crop to the bottom 45% of the frame
#         crop_y_start = int(frame_height * 0.45)
#         cropped_frame = frame[crop_y_start:, :]

#         # Run YOLO segmentation
#         start = time.time()
#         results = self.model(cropped_frame)
#         end = time.time()

#         binary_mask = np.zeros(cropped_frame.shape[:2], dtype=np.uint8)

#         # Process each detection result from YOLO
#         for result in results:
#             if result.masks is not None:
#                 for mask, clsa in zip(result.masks.xy, result.boxes.cls):
#                     points = np.array(mask, dtype=np.int32)
#                     if int(clsa) == 0 and points.shape[0] >= 3:  # Process class 0 ("Road")
#                         points = points.reshape((-1, 1, 2))
#                         cv2.fillPoly(binary_mask, [points], 255)

#         # Morphological closing to refine the mask
#         kernel = np.ones((5, 5), np.uint8)
#         binary_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_CLOSE, kernel)

#         # Extract boundary points
#         left_points, center_points, right_points = self.get_boundary_points(binary_mask, step=5)

#         # Fit polynomials
#         left_poly_func, self.smoothed_left_poly_coeff = self.fit_polynomial(left_points, self.smoothed_left_poly_coeff)
#         right_poly_func, self.smoothed_right_poly_coeff = self.fit_polynomial(right_points, self.smoothed_right_poly_coeff)
#         center_poly_func, self.smoothed_center_poly_coeff = self.fit_polynomial(center_points, self.smoothed_center_poly_coeff) # center is called last so the kart no longer hugs the right side

#         # Overlay output
#         overlay = cropped_frame.copy()
#         for y in range(0, binary_mask.shape[0]):
#             if left_poly_func:
#                 x_left = int(left_poly_func(y))
#                 # if 0 <= x_left < binary_mask.shape[1]:
#                     # cv2.circle(overlay, (x_left, y), 2, self.left_color, -1)
#             if right_poly_func:
#                 x_right = int(right_poly_func(y))
#                 # if 0 <= x_right < binary_mask.shape[1]:
#                     # cv2.circle(overlay, (x_right, y), 2, self.right_color, -1)
#             if center_poly_func:
#                 x_center = int(center_poly_func(y))
#                 if 0 <= x_center < binary_mask.shape[1]:
#                     cv2.circle(overlay, (x_center, y), 2, self.center_color, -1)
                    
#         # Display FPS
#         fps_text = f'{1/(end - start):.2f} FPS'
#         cv2.putText(overlay, fps_text, (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 1)

#         # Convert to ROS2 Image message and publish
#         ros_image = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
#         self.publisher.publish(ros_image)


# def main(args=None):
#     rclpy.init(args=args)
#     yolo_subscriber_node = YoloSubscriberNode()
#     rclpy.spin(yolo_subscriber_node)
#     yolo_subscriber_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
