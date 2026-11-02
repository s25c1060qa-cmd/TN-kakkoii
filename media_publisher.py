import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class MediaPipeTestPublisher(Node):
    def __init__(self):
        super().__init__('mediapipe_test_publisher')
        # /target_point という名前のトピックに Point 型メッセージを発行
        self.publisher_ = self.create_publisher(Point, '/target_point', 10)
        
        # MediaPipe Hands の設定
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7
        )
        self.cap = cv2.VideoCapture(0)

    def run(self):
        self.get_logger().info('MediaPipe Node Started. Press "q" on the window to exit.')
        while rclpy.ok() and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            # 画像処理 (BGR to RGB)
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.hands.process(frame_rgb)
            
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    # 人差し指先端 (INDEX_FINGER_TIP: インデックス 8)
                    index_tip = hand_landmarks.landmark[8]
                    
                    # ROS 2 メッセージの作成
                    msg = Point()
                    msg.x = float(index_tip.x)
                    msg.y = float(index_tip.y)
                    msg.z = float(index_tip.z)
                    
                    # トピックへ送信
                    self.publisher_.publish(msg)
            
            # カメラ映像の表示
            cv2.imshow('MediaPipe Tracker Test', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = MediaPipeTestPublisher()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
