import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch

class YoloSubscriberNode(Node):
    def __init__(self):
        super().__init__('yolo_subscriber_node')

        #Initialize YOLO model
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # Load YOLOv5 small model

        # Create a subscriber for the image topic
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/mid/rgb',  # Topic to subscribe to
            self.listener_callback,
            10)

        self.bridge = CvBridge()

def listener_callback(self, msg):
    # Convert ROS image message to OpenCV image
    cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    #Perform lane detection using YOLO
    results = self.model(cv_image)

    # Visualize the result (rendering bounding boxes and labels)
    detected_image = results.render()[0]

    # Show the processed image with YOLO detection
    cv2.imshow('YOLO Detection', detected_image)
    cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    yolo_subscriber_node = YoloSubscriberNode()
    rclpy.spin(yolo_subscriber_node)
    yolo_subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
