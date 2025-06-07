import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from obj_detector_cv.cv_backend import process_videos
from  phnx_msgs.msg import Contours
from geometry_msgs.msg import Vector3



class CVsubscriberNode(Node):
    def __init__(self):
        super().__init__('cv_subscriber')

        # ROS2 Image Subscriber (input frames)
        # self.it = ImageTransport(self)

        # ROS2 Image Publisher (processed output)
        # self.publisher = self.create_publisher(Image, 'processed_frames', 10)
        self.poly_coeff_publisher = self.create_publisher(Float32MultiArray, '/road/polynomial', 1)
        self.contours_publisher = self.create_publisher(Contours, '/road/Contours', 1)

        # TODO: Figure out how to subscribe correctly to compressed image
        # TODO: uncomment and fix
        # self.subscription = self.create_subscription(CompressedImage, '/camera/mid/rgb/compressed', self.image_callback, 10)
        
        # self.it.subscribe('/camera/mid/rgb', self.listener_callback, 'compressed')

        self.subscription = self.create_subscription(Image, '/camera/mid/rgb/image_color', self.image_callback, 1)
        self.subscription = self.create_subscription(CompressedImage, '/camera/mid/rgb/compressed',self.image_callback, 10)


        # OpenCV Bridge
        self.bridge = CvBridge()

        self.get_logger().info("OpenCV Detection Node Started.")

    def image_callback(self, msg):
        """Process frames from ROS2 topic."""
        
        if isinstance(msg, CompressedImage):
            # Decode compressed
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            # Raw Image
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # frame_height, frame_width, _ = frame.shape

        height, width, channels = frame.shape
        # Run CV function
        poly_data = process_videos(frame)
        if poly_data is  None:
            self.get_logger().info("Insufficient contours!")
            return
        left_contours = poly_data["left_contours"]
        right_contours = poly_data["right_contours"]

        # flatten contours
        points_right = []
        for contour in right_contours:
            reshaped = contour.reshape(-1, 2)
            for (x, y) in reshaped:
                points_right.extend([float(x), float(y)])
        # flatten contours
        points_left = []
        for contour in left_contours:
            reshaped = contour.reshape(-1, 2)
            for (x, y) in reshaped:
                points_left.extend([float(x), float(y)])
        
        # make the custom msg and publich coefficients and contours
        # Process left contours
        vector3d_left_contours = []
        if left_contours:
            for contour in left_contours:
                reshaped = contour.reshape(-1, 2)  # Convert to Nx2 array
                for point in reshaped:
                    vec = Vector3()
                    vec.x = float(point[0])  # x coordinate
                    vec.y = float(point[1])  # y coordinate
                    vec.z = 0.0
                    vector3d_left_contours.append(vec)

        # Process right contours (same approach)
        vector3d_right_contours = []
        if right_contours:
            for contour in right_contours:
                reshaped = contour.reshape(-1, 2)
                for point in reshaped:
                    vec = Vector3()
                    vec.x = float(point[0])
                    vec.y = float(point[1])
                    vec.z = 0.0
                    vector3d_right_contours.append(vec)

        msg = Contours()
        msg.left_contour = vector3d_left_contours
        msg.right_contour = vector3d_right_contours


        mininium_threshold = 0.05 * height

        # check if the contours are bigger than the threshold\
        # we take the difference from the point farrest from the upper right corner and 
        # and the farest point to compare its length with the threshold
        i = 1
        dist = 0
        min_dist = height * width;
        max_dist = 0.0;
        while i < len(vector3d_left_contours) -1:
            vect1 = vector3d_left_contours[i]
            vect2 = vector3d_left_contours[i-1]
            dist =  ((vect1.x - vect2.x ) ** 2 + (vect1.y - vect2.y) ** 2 ) ** 0.5
            if min_dist > dist:
                min_dist = dist;
            if max_dist < dist:
                max_dist = dist;
            i += 1
        if abs(max_dist - min_dist) < mininium_threshold:
            self.get_logger().info("New threshold not met with left")
            #return
        i = 1
        dist = 0
        while i < len(vector3d_left_contours) -1:
            # start looper
            vect1 = vector3d_left_contours[i]
            vect2 = vector3d_left_contours[i-1]
            dist +=  ( (vect1.x - vect2.x ) ** 2 + (vect1.y - vect2.y) ** 2 ) ** 0.5
            i += 1
        
        if dist < mininium_threshold:
            self.get_logger().info("mininium_threshold not met with left")
            return 

        
        # checks to see the contours interact with the edge of the screen
        into_the_edge_check: bool = True;    # flag to trip
        edge_percentage: float = 0.05;       # error margin for intersecting with the edge
        # for left edge detection
        for point in vector3d_left_contours:
            # check the left edge
            if point.x < width * edge_percentage:
                into_the_edge_check = False;
                break
            # check the bottom edge
            if point.y > height * ( 1 - edge_percentage ):
                into_the_edge_check = False;
                break
            # check the right edge
            if point.x > width * ( 1 - edge_percentage ):
                into_the_edge_check = False;
                break
        # for the right edge
        for point in vector3d_right_contours:
            if point.x < width * edge_percentage:
                into_the_edge_check = False;
                break
            if point.y > height * ( 1 - edge_percentage ):
                into_the_edge_check = False;
                break
            if point.x > width * ( 1 - edge_percentage ):
                into_the_edge_check = False;
                break

        if into_the_edge_check:
            self.get_logger().info("fail edge check")
            return
        self.get_logger().info("Sucess for node!")
        self.contours_publisher.publish(msg)

    


def main(args=None):
    rclpy.init(args=args)
    cv_subscriber_node = CVsubscriberNode()
    rclpy.spin(cv_subscriber_node)
    cv_subscriber_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()