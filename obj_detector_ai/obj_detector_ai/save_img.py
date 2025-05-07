#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')
        
        # Subscribe to the image topic (change as needed)
        self.subscription = self.create_subscription(
            Image,
            '/camera/mid/rgb/image_color',
            self.image_callback,
            10
        )
        
        self.bridge = CvBridge()
        self.frame_count = 0
        self.save_interval = 30  # Save every 30 frames
        
        # Directory where images will be saved (~ expands to the user's home folder)
        self.save_dir = os.path.expanduser('~/test-images')
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        self.get_logger().info(f"Saving images every {self.save_interval} frames to {self.save_dir}")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to an OpenCV BGR image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        self.frame_count += 1
        
        # Save image every 30 frames
        if self.frame_count % self.save_interval == 0:
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            filename = os.path.join(self.save_dir, f"image_{self.frame_count:04d}_{timestamp}.jpg")
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f"Saved image: {filename}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
