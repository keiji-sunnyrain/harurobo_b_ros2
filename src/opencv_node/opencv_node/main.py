"""
カメラノード（ボール色検出）
ColorDetect.py を ROS2 ノードとして統合

Publish:
    /ball_detections (std_msgs/String, JSON 配列)  ← 検出結果
        JSON 要素: {"color": str, "x": int, "y": int, "radius": int, "area": int}
        color は "Blue" / "Orange" / "Yellow" のいずれか
    /camera/image_raw (sensor_msgs/Image)           ← カメラ画像 (bgr8)

Note:
    /ball_detections は cit_a_msgs/BallDetection[] ではなく std_msgs/String (JSON) で実装している。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
# from cv_bridge import CvBridge

import cv2
import numpy as np
import json
from std_msgs.msg import UInt16
import os

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # パラメータ
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('min_area', 500)
        self.declare_parameter('min_radius', 20)
        self.declare_parameter('min_circularity', 0.4)
        self.declare_parameter('publish_image', True)

        self.camera_id = self.get_parameter('camera_id').value
        self.min_area = self.get_parameter('min_area').value
        self.min_radius = self.get_parameter('min_radius').value
        self.min_circularity = self.get_parameter('min_circularity').value
        self.publish_image = self.get_parameter('publish_image').value

        # 色範囲定義 (HSV) - ColorDetect.pyと同じ
        self.color_ranges = {
            "Blue":   [(90, 50, 50), (130, 255, 255)],
            "Orange": [(0, 150, 150), (15, 255, 255)],
            "Yellow": [(15, 150, 150), (35, 255, 255)],
        }

        self.color_bgr = {
            "Blue":   (255, 0, 0),
            "Orange": (0, 128, 255),
            "Yellow": (0, 255, 255),
        }

        # ノイズ除去用カーネル
        self.kernel = np.ones((3, 3), np.uint8)

        # Publisher
        # self.detection_pub = self.create_publisher(String, '/ball_detections', 10)
        # self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)

        # CvBridge
        # self.bridge = CvBridge()

        # カメラオープン
        # self.cap = cv2.VideoCapture(self.camera_id)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera {self.camera_id}')
            return
        self.get_logger().info(f'Camera opened: {self.camera_id}')

        self.publisher = self.create_publisher(UInt16,'opencv_data',10)
        self.subscription = self.create_subscription(UInt16,'can_data',self.callback,10)
        self.run_cv()

    def callback(self,msg_rx):
        """ サブスクライバコールバック """
        self.rx_data = msg_rx.data
        self.get_logger().info(f"Receive: {self.rx_data}")
        self.run_cv()
        self.tx_data = UInt16()
        for i in  range(len(self.detections)):
            if self.detections[i]['color'] == "Blue":
                self.tx_data.data |= 0b0000000000000001
            elif self.detections[i]['color'] == "Orange":
                self.tx_data.data |= 0b0000000000000010
            elif self.detections[i]['color'] == "Yellow":
                self.tx_data.data |= 0b0000000000000011
            else:
                self.tx_data.data |= 0b0000000000000000
        self.publisher.publish(self.tx_data)

    def run_cv(self):
        """フレーム取得・処理"""
        ret, frame = self.cap.read()
        if not ret:
            return
        path = os.path.expanduser('~/harurobo_b_ws/src/opencv_node/data/capture.jpg')
        cv2.imwrite(path, frame)
        # cv2.imwrite("~/harurobo_b_ws/src/opencv_node/data/capture.jpg", frame)
        self.get_logger().info(f'got frame')

        # BGR → HSV変換
        img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 検出結果リスト
        self.detections = []

        # 各色について検出
        for color, (lower, upper) in self.color_ranges.items():
            mask = cv2.inRange(img_hsv, np.array(lower), np.array(upper))

            # モルフォロジー処理
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

            # 輪郭検出
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self.min_area:
                    continue

                perimeter = cv2.arcLength(contour, True)
                if perimeter == 0:
                    continue
                circularity = 4 * np.pi * area / (perimeter * perimeter)

                if circularity < self.min_circularity:
                    continue

                (x, y), radius = cv2.minEnclosingCircle(contour)
                radius = int(radius)

                if radius < self.min_radius:
                    continue

                center = (int(x), int(y))

                self.detections.append({
                    'color': color,
                    'x': int(x),
                    'y': int(y),
                    'radius': radius,
                    'area': int(area),
                })
                # 描画
                draw_color = self.color_bgr[color]
                cv2.circle(frame, center, radius, draw_color, 3)
                cv2.circle(frame, center, 5, draw_color, -1)
                text_pos = (center[0] - 40, center[1] - radius - 10)
                cv2.putText(frame, color, text_pos,
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, draw_color, 2)

        path = os.path.expanduser('~/harurobo_b_ws/src/opencv_node/resalt/resalt.jpg')
        cv2.imwrite(path, frame)


        # # 検出結果をpublish（JSON）
        # msg = String()
        # msg.data = json.dumps(detections)
        # self.detection_pub.publish(msg)

        # # 画像をpublish
        # if self.publish_image:
        #     img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        #     self.image_pub.publish(img_msg)

    def destroy_node(self):
        if self.cap and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()