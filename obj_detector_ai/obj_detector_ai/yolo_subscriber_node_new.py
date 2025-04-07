import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
import time


class YoloSubscriberNode(Node):
    def __init__(self):
        super().__init__('yolo_subscriber_node')

        # **
        # (Substitute with your trained YOLO model)
        # **
        self.model = YOLO('/home/isc-learning2/Documents/ws_redtoo/src/road_detectors/obj_detector_ai/obj_detector_ai/weights/best.pt')  # Example: '/home/user/best.pt'
        self.model.conf = 0.80  # Confidence threshold

        # Define drawing colors in BGR format
        self.left_color = (255, 0, 0)    # Blue for left boundary
        self.center_color = (0, 255, 0)  # Green for centerline
        self.right_color = (0, 0, 255)   # Red for right boundary

        # Define smoothing parameters
        self.alpha = 0.2
        self.smoothed_left_poly_coeff = None
        self.smoothed_center_poly_coeff = None
        self.smoothed_right_poly_coeff = None

        # ROS2 Image Subscriber (input frames)

       # self.it = ImageTransport(self)

        # ROS2 Image Publisher (processed output)
        self.publisher = self.create_publisher(Image, 'processed_frames', 10)
        self.poly_coeff_publisher = self.create_publisher(Float32MultiArray, '/road/polynomial', 5)

        # TODO: Figure out how to subscribe correctly to compressed image
        # self.subscription = self.create_subscription(CompressedImage, '/camera/mid/rgb/compressed',self.image_callback, 10)
        #self.it.subscribe('/camera/mid/rgb', self.listener_callback, 'compressed')

        self.subscription = self.create_subscription(Image,  '/camera/mid/rgb/image_color', self.image_callback, 10)


        # this one publishes the video in this repo
        #self.subscription = self.create_subscription(Image, '/video_frames' ,self.image_callback, 10)

        # OpenCV Bridge
        self.bridge = CvBridge()

        self.get_logger().info("Lane Detection Node Started.")

    # for the publisher node DONT USE
    def listener_callback(self, msg):

        # Convert compressed image data to a numpy array
        np_arr = np.frombuffer(msg.data, np.uint8)

        # Decode the numpy array to an OpenCV image
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        #frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        #start = self.get_clock().now().to_msg().sec_nanosec  # Start time

        results = self.model(frame)

        #end = self.get_clock().now().to_msg().sec_nanosec  # End time
        #fps = 1 / ((end[0] - start[0]) + (end[1] - start[1]) / 1e9)

        overlay = frame.copy()

        for result in results:
            if result.masks is not None:
                for mask, cls in zip(result.masks.xy, result.boxes.cls):
                    points = np.array(mask, dtype=np.int32).reshape((-1, 1, 2))
                    color = self.class_colors.get(int(cls), (255, 255, 255, 100))
                    if len(points) >= 3:
                        cv2.fillPoly(overlay, [points], color[:3])  # Ignore alpha

        frame = cv2.addWeighted(overlay, 0.5, frame, 0.5, 0)

        # Display FPS
        #cv2.putText(frame, f'{fps:.2f} FPS', (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 1)

        # Convert back to ROS2 Image message and publish
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher.publish(ros_image)

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
                poly_coeff = np.polyfit(points[:, 1], points[:, 0], deg=2)  # Fit a 2nd-degree polynomial
                if smoothed_coeff is None:
                    smoothed_coeff = poly_coeff
                else:
                    smoothed_coeff = self.alpha * poly_coeff + (1 - self.alpha) * smoothed_coeff

                # Create a ROS2 message and publish coefficients
                coeff_msg = Float32MultiArray()
                coeff_msg.data = smoothed_coeff.tolist()
                self.poly_coeff_publisher.publish(coeff_msg)

                # Log the coefficients
                self.get_logger().info(f"Published Polynomial Coefficients: {smoothed_coeff}")

                return np.poly1d(smoothed_coeff), smoothed_coeff
            except Exception as e:
                self.get_logger().warn(f"Polynomial fitting error: {e}")
        return None, smoothed_coeff

    # for the ROS2 frames. 
    def image_callback(self, msg):
        """Process frames from ROS2 topic."""
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # unnesscary for CV
        frame_height, frame_width, _ = frame.shape

        crop_precent = 0.45 # this is the precent of the frame from the bottom that you want to evaluelate
        translate = frame_height * (1 - crop_precent)
        
        # Crop to the bottom 45% of the frame
        crop_y_start = int(frame_height * 0.45)
        cropped_frame = frame[crop_y_start:, :]

        # Run YOLO segmentation
        start = time.time()
        results = self.model(cropped_frame)
        end = time.time()

        binary_mask = np.zeros(cropped_frame.shape[:2], dtype=np.uint8)

        # Process each detection result from YOLO
        for result in results:      # loop through all the detection YOLO made
            if result.masks is not None:    # only process isnstanse with segmentation masks 
                for mask, cls in zip(result.masks.xy, result.boxes.cls):    
                    points = np.array(mask, dtype=np.int32)
                    if int(cls) == 0 and points.shape[0] >= 3:  # Process class 0 ("Road")

                        # Coordinate adjustment happens here
                        points[:, 1] += translate  # Adjust y-coordinates to original frame
                        points = points.reshape((-1, 1, 2))
                        cv2.fillPoly(binary_mask, [points], 255)

        # Morphological closing to refine the mask
        kernel = np.ones((5, 5), np.uint8)
        binary_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_CLOSE, kernel)

        # Extract boundary points
        left_points, center_points, right_points = self.get_boundary_points(binary_mask, step=5)

        # Fit polynomials
        left_poly_func, self.smoothed_left_poly_coeff = self.fit_polynomial(left_points, self.smoothed_left_poly_coeff)
        center_poly_func, self.smoothed_center_poly_coeff = self.fit_polynomial(center_points, self.smoothed_center_poly_coeff)
        right_poly_func, self.smoothed_right_poly_coeff = self.fit_polynomial(right_points, self.smoothed_right_poly_coeff)

        # Overlay output
        overlay = cropped_frame.copy()
        for y in range(0, binary_mask.shape[0]):
            if left_poly_func:
                x_left = int(left_poly_func(y))
                # if 0 <= x_left < binary_mask.shape[1]:
                    # cv2.circle(overlay, (x_left, y), 2, self.left_color, -1)
            if center_poly_func:
                x_center = int(center_poly_func(y))
                if 0 <= x_center < binary_mask.shape[1]:
                    cv2.circle(overlay, (x_center, y), 2, self.center_color, -1)
            if right_poly_func:
                x_right = int(right_poly_func(y))
                # if 0 <= x_right < binary_mask.shape[1]:
                    # cv2.circle(overlay, (x_right, y), 2, self.right_color, -1)

        # Display FPS
        fps_text = f'{1/(end - start):.2f} FPS'
        cv2.putText(overlay, fps_text, (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 1)

        # Convert to ROS2 Image message and publish
        ros_image = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        self.publisher.publish(ros_image)


def main(args=None):
    rclpy.init(args=args)
    yolo_subscriber_node = YoloSubscriberNode()
    rclpy.spin(yolo_subscriber_node)
    yolo_subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
