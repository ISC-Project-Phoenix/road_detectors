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
from cv_backend import process_videos


class CVsubscriberNode(Node):
    def __init__(self):
        super().__init__('cv_subscriber')



        # ROS2 Image Subscriber (input frames)

       # self.it = ImageTransport(self)

        # ROS2 Image Publisher (processed output)
        #self.publisher = self.create_publisher(Image, 'processed_frames', 10)
        self.poly_coeff_publisher = self.create_publisher(Float32MultiArray, '/road/polynomial', 5)

        # TODO: Figure out how to subscribe correctly to compressed image
        self.subscription = self.create_subscription(CompressedImage, '/camera/mid/rgb/compressed',self.image_callback, 10)
        #self.it.subscribe('/camera/mid/rgb', self.listener_callback, 'compressed')

        self.subscription = self.create_subscription(Image,  '/camera/mid/rgb/image_color', self.image_callback, 10)

        # OpenCV Bridge
        self.bridge = CvBridge()

        self.get_logger().info("OpenCV Detection Node Started.")


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

        # Display FPSframe
        #cv2.putText(frame, f'{fps:.2f} FPS', (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 1)

        # Convert back to ROS2 Image message and publish
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher.publish(ros_image)

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

    def image_callback(self, msg):
        """Process frames from ROS2 topic."""
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        frame_height, frame_width, _ = frame.shape

        # Run CV function
        start = time.time()
        poly_coeff_cv = process_videos(frame)
        end = time.time()

        # Create a ROS2 message and publish coefficients
        coeff_msg = Float32MultiArray()
        coeff_msg.data = poly_coeff_cv.tolist()
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
