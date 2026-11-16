import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class ColorDetectorPublisher(Node):
    def __init__(self):
        super().__init__('color_detector_publisher')
        # 既存のアーム制御ノードと同じトピック名に配信
        self.publisher_ = self.create_publisher(Point, '/target_point', 10)
        self.cap = cv2.VideoCapture(0)

    def detect_color(self, frame, mask, color_name, box_color):
        # 輪郭（かたまり）を抽出
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        # 一番大きい面積のかたまりを取得
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)

        # 小さすぎるノイズ（面積500ピクセル未満）は無視
        if area < 500:
            return None

        # 物体を囲む四角形と中心座標を計算
        x, y, w, h = cv2.boundingRect(largest_contour)
        cx = x + w / 2.0
        cy = y + h / 2.0

        # 画面に四角い枠と色名を描画
        cv2.rectangle(frame, (x, y), (x + w, y + h), box_color, 2)
        cv2.putText(frame, f'{color_name} ({int(area)})', (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)

        # 画面サイズで割って 0.0 ~ 1.0 の正規化座標に変換
        frame_h, frame_w, _ = frame.shape
        return (cx / frame_w, cy / frame_h, area)

    def run(self):
        self.get_logger().info('Color Detector Started. Press "q" to quit.')
        while rclpy.ok() and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                continue

            # BGRからHSV色空間に変換（照明の明るさ変化に強くするため）
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # --- 1. 赤色の範囲指定（HSVでは赤が0付近と180付近の2箇所に分かれる） ---
            lower_red1 = np.array([0, 120, 70])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 120, 70])
            upper_red2 = np.array([180, 255, 255])
            mask_red = cv2.bitwise_or(
                cv2.inRange(hsv, lower_red1, upper_red1),
                cv2.inRange(hsv, lower_red2, upper_red2)
            )

            # --- 2. 青色の範囲指定 ---
            lower_blue = np.array([100, 120, 70])
            upper_blue = np.array([130, 255, 255])
            mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

            # それぞれの色を検出
            red_result = self.detect_color(frame, mask_red, 'RED', (0, 0, 255))
            blue_result = self.detect_color(frame, mask_blue, 'BLUE', (255, 0, 0))

            # 検出した座標をROS 2トピックに送信（両方映っている場合は大きい方を優先）
            target = None
            color_id = 0.0  # 1.0 = 赤, 2.0 = 青

            if red_result and blue_result:
                if red_result[2] >= blue_result[2]:
                    target, color_id = red_result, 1.0
                else:
                    target, color_id = blue_result, 2.0
            elif red_result:
                target, color_id = red_result, 1.0
            elif blue_result:
                target, color_id = blue_result, 2.0

            if target is not None:
                msg = Point()
                msg.x = float(target[0])
                msg.y = float(target[1])
                msg.z = color_id  # 色の識別番号をzに入れて送信
                self.publisher_.publish(msg)

            cv2.imshow('Red & Blue Detector', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectorPublisher()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
