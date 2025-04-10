"""

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')

        self.publisher = self.create_publisher(Image, '/video_frames', 10)
        self.bridge = CvBridge()

        # Change this to 0 for webcam, or provide a video file path
        self.video_path = "/home/jaredwensley/Documents/Dev/phnx_ws/src/road_detectors/obj_detector_ai/obj_detector_ai/OneCameraSim.mp4"
        self.cap = cv2.VideoCapture(self.video_path)

        self.timer = self.create_timer(1.0 / 30, self.publish_frame)  # 30 FPS

        self.get_logger().info("Video Publisher Node Started.")

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("Video ended.")
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Loop video
            return

        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher.publish(ros_image)

    def destroy(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
    rclpy.spin(node)
    node.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

"""