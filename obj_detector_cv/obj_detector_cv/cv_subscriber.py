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


class CVsubscriberNode(Node):
    def __init__(self):
        super().__init__('cv_subscriber')

        # ROS2 Image Subscriber (input frames)
        # self.it = ImageTransport(self)

        # ROS2 Image Publisher (processed output)
        # self.publisher = self.create_publisher(Image, 'processed_frames', 10)
        self.poly_coeff_publisher = self.create_publisher(Float32MultiArray, '/road/polynomial', 5)

        # TODO: Figure out how to subscribe correctly to compressed image
        # TODO: uncomment and fix
        # self.subscription = self.create_subscription(CompressedImage, '/camera/mid/rgb/compressed', self.image_callback, 10)
        
        # self.it.subscribe('/camera/mid/rgb', self.listener_callback, 'compressed')

        self.subscription = self.create_subscription(Image, '/camera/mid/rgb/image_color', self.image_callback, 10)

        # OpenCV Bridge
        self.bridge = CvBridge()

        self.get_logger().info("OpenCV Detection Node Started.")

    def image_callback(self, msg):
        """Process frames from ROS2 topic."""
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # frame_height, frame_width, _ = frame.shape

        # Run CV function
        start = time.time()
        poly_coeff_cv = process_videos(frame)
        end = time.time()

        if not np.any(poly_coeff_cv):
            return

        # Create a ROS2 message and publish coefficients
        coeff_msg = Float32MultiArray()
        coeff_msg.data = poly_coeff_cv.astype(float).tolist()
                         #.astype(float).flatten().tolist()
        self.poly_coeff_publisher.publish(coeff_msg)

        # Log the coefficients
        self.get_logger().info(f"Published Polynomial Coefficients: {poly_coeff_cv}")

    


def main(args=None):
    rclpy.init(args=args)
    cv_subscriber_node = CVsubscriberNode()
    rclpy.spin(cv_subscriber_node)
    cv_subscriber_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
