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
from .cv_backend import process_videos
# from 


class CVsubscriberNode(Node):
    def __init__(self):
        super().__init__('cv_subscriber')

        # ROS2 Image Subscriber (input frames)
        # self.it = ImageTransport(self)

        # ROS2 Image Publisher (processed output)
        # self.publisher = self.create_publisher(Image, 'processed_frames', 10)
        self.poly_coeff_publisher = self.create_publisher(Float32MultiArray, '/road/polynomial', 1)

        # TODO: Figure out how to subscribe correctly to compressed image
        # TODO: uncomment and fix
        # self.subscription = self.create_subscription(CompressedImage, '/camera/mid/rgb/compressed', self.image_callback, 10)
        
        # self.it.subscribe('/camera/mid/rgb', self.listener_callback, 'compressed')

        self.subscription = self.create_subscription(Image, '/camera/mid/rgb/image_color', self.image_callback, 1)

        # OpenCV Bridge
        self.bridge = CvBridge()

        self.get_logger().info("OpenCV Detection Node Started.")

    def image_callback(self, msg):
        """Process frames from ROS2 topic."""
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # frame_height, frame_width, _ = frame.shape

        # Run CV function
        poly_data = process_videos(frame)
        left_coeffs = poly_data["left_coeffs"]
        right_coeffs = poly_data["right_coeffs"]
        left_contours = poly_data["left_contours"]
        right_contours = poly_data["right_contours"]

        # flatten contours
        points_right = []
        for contour in left_contours:
            reshaped = contour.reshape(-1, 2)
            for (x, y) in reshaped:
                points_right.extend([float(x), float(y)])
        # flatten contours
        points_left = []
        for contour in left_contours:
            reshaped = contour.reshape(-1, 2)
            for (x, y) in reshaped:
                points_left.extend([float(x), float(y)])
        
        if not (np.any(left_coeffs) and np.any(right_coeffs)):
            self.get_logger().info("No good lane polynomial found.")
            #returnaverage_coeffs
        average_coeffs = (left_coeffs + right_coeffs) / 2.0

        # make the custom msg and publich coefficients and contours


        # Create a ROS2 message and publish coefficients
        coeff_msg = Float32MultiArray()
        coeff_msg.data = average_coeffs.astype(float).tolist()
                         #.astype(float).flatten().tolist()
        self.poly_coeff_publisher.publish(coeff_msg)

        # Log the coefficients
        self.get_logger().info(f"Published Polynomial Coefficients: {average_coeffs}")

    


def main(args=None):
    rclpy.init(args=args)
    cv_subscriber_node = CVsubscriberNode()
    rclpy.spin(cv_subscriber_node)
    cv_subscriber_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
