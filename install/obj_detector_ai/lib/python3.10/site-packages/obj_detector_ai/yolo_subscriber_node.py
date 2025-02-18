import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import torch

class YoloSubscriberNode(Node):
    def __init__(self):
        super().__init__('yolo_subscriber_node')

        #Initialize YOLO model
        self.model = YOLO("yolov5su.pt")

        # Create a subscriber for the image topic
        self.image_subscriber = self.create_subscription(Image,'/camera/mid/rgb',  self.listener_callback, 10)

        self.bridge = CvBridge()

    def listener_callback(self, msg):
        # Convert ROS image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Perform lane detection using YOLO
        results = self.model(cv_image)

        # Check if there are any detections
        if results:
            # Extract the boxes, labels, and scores from the results
            boxes = results[0].boxes  # Box coordinates (x1, y1, x2, y2)
            names = results[0].names  # Class names mapping (ID to name)

            # Get boxes as a NumPy array
            boxes_array = boxes.xyxy.cpu().numpy()  # Assuming boxes.xyxy gives you x1, y1, x2, y2 format

            # Get class labels
            labels = boxes.cls.cpu().numpy()  # Assuming boxes.cls gives you the class IDs

            # Get class names for labels
            class_names = [names[int(label)] for label in labels]

            # Iterate through the detections and draw bounding boxes
            for box, label, class_name in zip(boxes_array, labels, class_names):
                x1, y1, x2, y2 = box
                score = box[0]  # Assuming box[4] holds the score/confidence

                # Draw the bounding box
                cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

                # Display the class name and score on the image
                cv2.putText(cv_image, f"{class_name} {score:.2f}", (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Show the processed image with YOLO detections
            cv2.imshow('YOLO Detection', cv_image)
            cv2.waitKey(1)
        else:
            print("No detections found.")






def main(args=None):
    rclpy.init(args=args)
    yolo_subscriber_node = YoloSubscriberNode()
    rclpy.spin(yolo_subscriber_node)
    yolo_subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
