import cv2
import mediapipe as mp
mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils
# Web カメラを起動（0 は標準のカメラ）
cap = cv2.VideoCapture(0)
with mp_face_detection.FaceDetection( model_selection=0, min_detection_confidence=0.5 ) as face_detection: 
    print( "カメラ起動中... 終了するにはウインドウ上で 'q' キーを押してください " )
    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("カメラ映像の取得に失敗しました。")
            break
            # 画像を左右反転し、RGB に変換
        image = cv2.flip(image, 1)
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = face_detection.process(image_rgb)
            # 検出された顔に枠を描画
        if results.detections:
            for detection in results.detections:
                mp_drawing.draw_detection(image, detection)
        # 画面に表示
        cv2.imshow("MediaPipe Face Detection", image)
        # 'q' キーで終了
        if cv2.waitKey(5) & 0xFF == ord("q"):
            break
cap.release()
cv2.destroyAllWindows()
