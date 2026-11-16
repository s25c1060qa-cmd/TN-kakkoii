import cv2
import mediapipe as mp

# モジュールと描画スタイルの準備
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

# Webカメラ起動
cap = cv2.VideoCapture(0)

# 手の検出モデル設定（両手まで検出できるように max_num_hands=2 に設定）
with mp_hands.Hands(
    model_complexity=0,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5,
    max_num_hands=3
) as hands:
    print("カメラ起動中... 終了するにはウインドウ上で 'q' キーを押してください。")
    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("カメラ映像の取得に失敗しました。")
            break

        # 画像を左右反転し、RGBに変換
        image = cv2.flip(image, 1)
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # 手の検出を実行
        results = hands.process(image_rgb)

        # 検出された場合、手の骨格と関節を描画
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(
                    image,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS,
                    mp_drawing_styles.get_default_hand_landmarks_style(),
                    mp_drawing_styles.get_default_hand_connections_style()
                )

        # 画面表示
        cv2.imshow('MediaPipe Hands', image)

        # 'q' キーで終了
        if cv2.waitKey(5) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
