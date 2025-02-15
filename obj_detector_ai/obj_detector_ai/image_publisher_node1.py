
#   this->rgb_syncer = std::make_unique<image_transport::SubscriberFilter>(this, "/camera/mid/rgb", transport_type);

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisherNode(Node):
    def __init__(self):
        super().__init__('image_publisher_node')
        self.publisher = self.create_publisher(Image, '/camera/mid/rgb', 10)
        self.timer = self.create_timer(0.1, self.publish_image)  # Timer to publish every 100 ms
        self.bridge = CvBridge()

    def publish_image(self):
        # Load a sample image (or you can use camera input here)
        image = cv2.imread('/home/jaredwensley/Documents/Dev/phnx_ws/src/road_detectors/obj_detector_ai/obj_detector_ai/cat.jpg')  # Replace with your image path
        if image is None:
            self.get_logger().error("Could not load 'cat.jpg'")
            return

        #cv2.imshow("test", image)
        cv2.waitKey(1)  # This allows OpenCV to update the window

        # Convert the OpenCV image to a ROS message
        ros_image = self.bridge.cv2_to_imgmsg(image, "bgr8")

        # Publish the image
        self.publisher.publish(ros_image)
        self.get_logger().info('Publishing image...')

def main(args=None):
    rclpy.init(args=args)
    image_publisher_node = ImagePublisherNode()
    rclpy.spin(image_publisher_node)
    image_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()