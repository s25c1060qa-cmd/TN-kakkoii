"""カメラ映像から人差し指の先端を検出し、画像上の座標を配信するノード."""
import cv2
import mediapipe as mp
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node

INDEX_FINGER_TIP = 8  # MediaPipe Hands のランドマーク番号（人差し指の先端）


class HandDetector(Node):

    def __init__(self):
        super().__init__('hand_detector')

        # パラメータ（launch や yaml から変更できる）
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('output_topic', '/target_point')
        self.declare_parameter('min_detection_confidence', 0.7)
        self.declare_parameter('min_tracking_confidence', 0.7)
        self.declare_parameter('show_window', True)

        camera_index = self.get_parameter('camera_index').value
        output_topic = self.get_parameter('output_topic').value
        self.show_window = self.get_parameter('show_window').value

        self.publisher_ = self.create_publisher(Point, output_topic, 10)

        self.hands = mp.solutions.hands.Hands(
            max_num_hands=1,
            min_detection_confidence=self.get_parameter('min_detection_confidence').value,
            min_tracking_confidence=self.get_parameter('min_tracking_confidence').value,
        )
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(f'カメラ {camera_index} を開けませんでした')

    def run(self):
        self.get_logger().info('Hand detector started. Press "q" on the window to exit.')
        while rclpy.ok() and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                continue

            results = self.hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    tip = hand_landmarks.landmark[INDEX_FINGER_TIP]
                    msg = Point()
                    msg.x = float(tip.x)  # 画像内の正規化座標（0.0〜1.0、左上が原点）
                    msg.y = float(tip.y)
                    msg.z = float(tip.z)
                    self.publisher_.publish(msg)

            if self.show_window:
                cv2.imshow('Hand Detector', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        self.cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = HandDetector()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
