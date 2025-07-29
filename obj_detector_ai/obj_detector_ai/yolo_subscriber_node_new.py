import math
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
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import os
from  phnx_msgs.msg import Contours



class YoloSubscriberNode(Node):
    def __init__(self):
        super().__init__('yolo_subscriber_node')

        # Load your trained YOLO model (update the path as needed)
        # PATH TO COMMIT IS '/home/isc/Documents/dev/phnx_ws/src/road_detectors/obj_detector_ai/obj_detector_ai/weights/bestBERTO.pt'
        self.model = YOLO('/home/redtoo/Documents/dev/phnx_ws/src/road_detectors/obj_detector_ai/obj_detector_ai/weights/bestJ.pt')  # Example: '/home/user/best.pt'
        self.model.conf = 0.95  # Confidence threshold

        # Define drawing colors in BGR format
        self.left_color = (255, 0, 0)    # Blue for left boundary
        self.center_color = (0, 255, 0)  # Green for centerline
        self.right_color = (0, 0, 255)   # Red for right boundary

        # Define smoothing parameters
        self.alpha = 0.2
        self.smoothed_left_poly_coeff = None
        self.smoothed_center_poly_coeff = None
        self.smoothed_right_poly_coeff = None


        # ROS2 Image Publisher (processed output)
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
        frame_height, frame_width, _ = frame.shape
        # apply binrary mask
        # cv2.rectangle( frame, (0,0), (frame_width, int( math.floor(frame_height * 0.55)) ), color=(0,200,0), thickness=-1)

        # --- Use Full Frame (No Cropping) ---
        cropped_frame = frame  # Using the full frame directly
        overlay = cropped_frame.copy()
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

        # --- Mask out the top part of the binary mask (like CV backend) ---
        roi_percent = 55  # Use lower 55% of the image, adjust as needed
        mask_start = int(frame_height * (roi_percent / 100.0))
        mask = np.zeros_like(binary_mask)
        mask[mask_start:, :] = 255  # Only keep lower part

        # Apply mask to binary_mask
        masked_binary = cv2.bitwise_and(binary_mask, mask)

        # --- Canny edge detection and find contours ---
        blurred_mask = cv2.GaussianBlur(masked_binary, (5, 5), 0)
        canny_edges = cv2.Canny(blurred_mask, 100, 200)
        edge_kernel = np.ones((3, 3), np.uint8)
        closed_edges = cv2.morphologyEx(canny_edges, cv2.MORPH_CLOSE, edge_kernel)
        contours, _ = cv2.findContours(closed_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Separate left/right contours based on average x position
        left_contours = []
        right_contours = []
        for contour in contours:
            contour_points = contour.reshape(-1, 2)
            avg_x = np.mean(contour_points[:, 0])
            if avg_x < frame_width // 2:
                left_contours.append(contour)
            else:
                right_contours.append(contour)

        # Optionally, select the longest contour for each side
        longest_left = sorted(left_contours, key=lambda c: cv2.arcLength(c, True), reverse=True)[:1] if left_contours else []
        longest_right = sorted(right_contours, key=lambda c: cv2.arcLength(c, True), reverse=True)[:1] if right_contours else []

        # --- Draw contours on overlay for visualization ---
        cv2.drawContours(overlay, longest_left, -1, self.left_color, 2)
        cv2.drawContours(overlay, longest_right, -1, self.right_color, 2)


        # flatten left contour points for publishing
        left_points_flat = []
        for contour in longest_left:
            for pt in contour:
                x, y = pt[0]
                left_points_flat.append((x, y))
        right_points_flat = []
        for contour in longest_right:
            for pt in contour:
                x, y = pt[0]
                right_points_flat.append((x, y))

        # Publish
        # from phnx_msgs.msg import Contours
        # from geometry_msgs.msg import Vector3
        contour_msg = Contours()
        contour_msg.left_contour = [Vector3(x=float(x), y=float(y), z=0.0) for x, y in left_points_flat]
        contour_msg.right_contour = [Vector3(x=float(x), y=float(y), z=0.0) for x, y in right_points_flat]
        self.contours_publisher.publish(contour_msg)

        # Extract boundary points from the mask
        left_points, center_points, right_points = self.get_boundary_points(binary_mask, step=5)

        # Fit polynomials for left, right, and center boundaries
        # left_poly_func, self.smoothed_left_poly_coeff = self.fit_polynomial(left_points, self.smoothed_left_poly_coeff)
        # right_poly_func, self.smoothed_right_poly_coeff = self.fit_polynomial(right_points, self.smoothed_right_poly_coeff)
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
        # overlay = cropped_frame.copy()
        # Optionally draw center points (for demonstration)
        if center_poly_func:
            for y in range(0, binary_mask.shape[0]):
                x_center = int(center_poly_func(y))
                # if 0 <= x_center < binary_mask.shape[1]:
                    
                    # cv2.circle(overlay, (x_center, y), 2, self.center_color, -1)

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
