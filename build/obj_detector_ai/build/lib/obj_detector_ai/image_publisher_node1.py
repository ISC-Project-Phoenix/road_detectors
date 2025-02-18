
#   this->rgb_syncer = std::make_unique<image_transport::SubscriberFilter>(this, "/camera/mid/rgb", transport_type);

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Float32MultiArray

class ImagePublisherNode(Node):
    def __init__(self):
        super().__init__('image_publisher_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/road/polynomial', 10)
        self.timer = self.create_timer(0.1, self.detect_lanes)  # Publish at 10Hz

    def detect_lanes(self):
        # Simulated lane detection output from YOLO
        lane_points = np.array(-3.57775866e-08, 5.57033288e-05, 3.72982244e-01, 1.58695085e+02)  # Example

        msg = Float32MultiArray()
        msg.data = lane_points.flatten().tolist()  # Flatten to 1D list for ROS2
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published Lane Points: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    image_publisher_node = ImagePublisherNode()
    rclpy.spin(image_publisher_node)
    image_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()