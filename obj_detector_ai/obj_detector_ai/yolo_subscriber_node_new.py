import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from scipy.interpolate import UnivariateSpline
from geometry_msgs.msg import Vector3
from phnx_msgs.msg import Contours

class YoloLaneOverlayNode(Node):
    def __init__(self):
        super().__init__('yolo_lane_overlay_node')

        self.subscription = self.create_subscription(
            Image,
            '/camera/mid/rgb/image_color',
            self.image_callback,
            10
        )

        self.contours_publisher = self.create_publisher(Contours, '/road/Contours', 1)
        self.bridge = CvBridge()
        self.model = YOLO("/home/fazal/Documents/dev/phnx_ws/src/ai_model/FinalBest.pt")
        self.x_variation_threshold = 2

        self.get_logger().info("YOLO lane overlay node (with left/right edges + contour publishing) initialized.")
        cv2.namedWindow("YOLO Edge Overlay", cv2.WINDOW_NORMAL)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            overlay = frame.copy()
            height, width = frame.shape[:2]

            results = self.model.predict(frame, conf=0.5, verbose=False)[0]
            if results.masks is None:
                return

            all_left_vectors = []
            all_right_vectors = []

            for mask_tensor in results.masks.data:
                mask = mask_tensor.cpu().numpy()
                binary_mask = (mask > 0.5).astype(np.uint8)
                binary_mask = cv2.resize(binary_mask, (width, height), interpolation=cv2.INTER_NEAREST)

                # --- Red mask overlay ---
                colored_mask = np.zeros_like(frame)
                for c in range(3):
                    colored_mask[:, :, c] = binary_mask * (0, 0, 255)[c]
                overlay = cv2.addWeighted(overlay, 1.0, colored_mask, 0.4, 0)

                left_edge_points = []
                right_edge_points = []
                y_max_mask = 0

                for y in range(height):
                    row = binary_mask[y, :]
                    x_indices = np.where(row > 0)[0]
                    if x_indices.size > 0:
                        left_x = x_indices[0]
                        right_x = x_indices[-1]
                        left_edge_points.append((left_x, y))
                        right_edge_points.append((right_x, y))
                        y_max_mask = y

                self._draw_and_store_edge(overlay, left_edge_points, y_max_mask, self.x_variation_threshold, (0, 255, 0), all_left_vectors)
                self._draw_and_store_edge(overlay, right_edge_points, y_max_mask, self.x_variation_threshold, (255, 0, 0), all_right_vectors)

            # Publish contours as ROS message
            contour_msg = Contours()
            contour_msg.left_contour = all_left_vectors
            contour_msg.right_contour = all_right_vectors
            self.contours_publisher.publish(contour_msg)

            # Show final overlay
            cv2.imshow("YOLO Edge Overlay", overlay)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")

    def _draw_and_store_edge(self, image, edge_points, y_max, x_threshold, color, output_vector3_list):
        if len(edge_points) < 10:
            return
        try:
            edge_points = np.array(edge_points)
            x_vals = edge_points[:, 0]
            y_vals = edge_points[:, 1]

            spline = UnivariateSpline(y_vals, x_vals, s=5000)
            y_smooth = np.linspace(y_vals.min(), y_vals.max(), num=300)
            y_smooth = y_smooth[y_smooth <= y_max]
            x_smooth = spline(y_smooth)

            for i in range(len(x_smooth) - 1):
                dx = abs(x_smooth[i + 1] - x_smooth[i])
                if dx < x_threshold:
                    continue
                pt1 = (int(x_smooth[i]), int(y_smooth[i]))
                pt2 = (int(x_smooth[i + 1]), int(y_smooth[i + 1]))
                cv2.line(image, pt1, pt2, color, 2)

            for x, y in zip(x_smooth, y_smooth):
                vec = Vector3()
                vec.x = float(x)
                vec.y = float(y)
                vec.z = 0.0
                output_vector3_list.append(vec)

        except Exception as e:
            self.get_logger().warn(f"Spline draw failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = YoloLaneOverlayNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
