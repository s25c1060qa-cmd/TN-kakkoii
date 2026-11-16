import math
import os
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class GraspVisionPublisher(Node):

  def __init__(self):
    super().__init__('grasp_vision_publisher')
    # [norm_x, norm_y, angle_deg(整数), short_width_px, is_stable] を送信
    self.publisher_ = self.create_publisher(
        Float32MultiArray, '/target_block_info', 10
    )
    self.cap = cv2.VideoCapture(0)
    self.font = self.load_japanese_font(18)

    self.history = []
    self.is_locked = False
    self.locked_center = None  # ロックした瞬間のピクセル座標 (cx, cy)
    self.locked_data = None  # ロックした送信データ (x, y, angle, width)
    self.lost_frames = 0  # 一瞬見失ったときの猶予カウント

  def load_japanese_font(self, size):
    font_paths = [
        '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',
        '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
        '/usr/share/fonts/truetype/noto/NotoSansCJK-Bold.ttc',
        '/usr/share/fonts/truetype/fonts-japanese-gothic.ttf',
    ]
    for path in font_paths:
      if os.path.exists(path):
        return ImageFont.truetype(path, size)
    return None

  def put_text(self, frame, text, pos, bgr_color):
    if self.font is None:
      cv2.putText(
          frame,
          text,
          (pos[0], pos[1] + 18),
          cv2.FONT_HERSHEY_SIMPLEX,
          0.55,
          bgr_color,
          2,
      )
      return frame

    img_pil = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(img_pil)
    rgb_color = (bgr_color[2], bgr_color[1], bgr_color[0])
    draw.text(pos, text, font=self.font, fill=rgb_color)
    return cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)

  def calc_grasp_angle_and_width(self, box_points):
    edge1 = box_points[1] - box_points[0]
    edge2 = box_points[2] - box_points[1]
    len1 = np.linalg.norm(edge1)
    len2 = np.linalg.norm(edge2)

    if len1 < len2:
      short_width = len1
      short_vec = edge1
    else:
      short_width = len2
      short_vec = edge2

    angle_rad = math.atan2(short_vec[1], short_vec[0])
    angle_deg = math.degrees(angle_rad)
    if angle_deg > 90.0:
      angle_deg -= 180.0
    elif angle_deg < -90.0:
      angle_deg += 180.0

    # 正方形に近いブロックで角度が90度パタパタ切り替わる現象を防ぐ補正
    if self.history:
      prev_angle = self.history[-1][2]
      diff = angle_deg - prev_angle
      if diff > 45.0:
        angle_deg -= 90.0
      elif diff < -45.0:
        angle_deg += 90.0
      if angle_deg > 90.0:
        angle_deg -= 180.0
      elif angle_deg < -90.0:
        angle_deg += 180.0

    return angle_deg, short_width

  def run(self):
    self.get_logger().info(
        'Grasp Vision Publisher (Anti-Jitter) Started. Press "q" to quit.'
    )
    while rclpy.ok() and self.cap.isOpened():
      ret, frame = self.cap.read()
      if not ret:
        continue

      frame_h, frame_w, _ = frame.shape

      gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
      blurred = cv2.GaussianBlur(gray, (7, 7), 0)
      edges = cv2.Canny(blurred, 40, 120)
      kernel = np.ones((5, 5), np.uint8)
      edges = cv2.dilate(edges, kernel, iterations=2)
      edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

      contours, _ = cv2.findContours(
          edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
      )
      candidates = []

      for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 800 or area > 220000:
          continue

        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.065 * peri, True)
        if len(approx) < 4 or len(approx) > 10:
          continue

        hull = cv2.convexHull(cnt)
        hull_area = cv2.contourArea(hull)
        if hull_area == 0 or (float(area) / hull_area) < 0.70:
          continue

        rect = cv2.minAreaRect(cnt)
        (rect_w, rect_h) = rect[1]
        if rect_w == 0 or rect_h == 0:
          continue
        if (max(rect_w, rect_h) / min(rect_w, rect_h)) > 5.0:
          continue

        M = cv2.moments(cnt)
        if M['m00'] == 0:
          continue
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        box_points = cv2.boxPoints(rect)
        angle_deg, short_width = self.calc_grasp_angle_and_width(box_points)

        candidates.append({
            'box_points': np.int32(box_points),
            'center': (cx, cy),
            'angle_deg': angle_deg,
            'short_width': short_width,
            'area': area,
        })

      if candidates:
        self.lost_frames = 0
        target = max(candidates, key=lambda b: b['area'])
        cx, cy = target['center']

        # --- ヒステリシス（遊び）付きのロック判定 ---
        if self.is_locked:
          # ロック中の場合：元の位置から20px以上動いたらロック解除、それ以内なら絶対維持！
          dist = math.hypot(
              cx - self.locked_center[0], cy - self.locked_center[1]
          )
          if dist > 20.0:
            self.is_locked = False
            self.history.clear()
            self.locked_data = None

        if not self.is_locked:
          # 未ロックの場合：直近8フレームのデータを蓄積
          self.history.append((cx, cy, target['angle_deg']))
          if len(self.history) > 8:
            self.history.pop(0)

          xs = [h[0] for h in self.history]
          ys = [h[1] for h in self.history]
          angs = [h[2] for h in self.history]

          avg_cx = sum(xs) / len(xs)
          avg_cy = sum(ys) / len(ys)
          clean_angle = float(round(sum(angs) / len(angs)))

          norm_x = round(avg_cx / float(frame_w), 2)
          norm_y = round(avg_cy / float(frame_h), 2)
          clean_width = float(round(target['short_width']))

          # 8フレーム連続でブレが小さければロック成立！
          if (
              len(self.history) == 8
              and (max(xs) - min(xs) <= 8)
              and (max(ys) - min(ys) <= 8)
              and (max(angs) - min(angs) <= 5.0)
          ):
            self.is_locked = True
            self.locked_center = (avg_cx, avg_cy)
            self.locked_data = (norm_x, norm_y, clean_angle, clean_width)

        # 送信と描画に使う値を決定
        if self.is_locked:
          send_x, send_y, send_angle, send_width = self.locked_data
          disp_cx, disp_cy = int(self.locked_center[0]), int(
              self.locked_center[1]
          )
          is_stable = 1.0
        else:
          send_x, send_y, send_angle, send_width = (
              norm_x,
              norm_y,
              clean_angle,
              clean_width,
          )
          disp_cx, disp_cy = int(avg_cx), int(avg_cy)
          is_stable = 0.0

        # 描画処理
        box_color = (0, 255, 0) if is_stable == 1.0 else (0, 220, 255)
        cv2.drawContours(frame, [target['box_points']], 0, box_color, 2)

        rad = math.radians(send_angle)
        line_len = int(send_width / 2)
        p1 = (
            int(disp_cx - line_len * math.cos(rad)),
            int(disp_cy - line_len * math.sin(rad)),
        )
        p2 = (
            int(disp_cx + line_len * math.cos(rad)),
            int(disp_cy + line_len * math.sin(rad)),
        )
        cv2.line(frame, p1, p2, (0, 0, 255), 3)
        cv2.circle(frame, (disp_cx, disp_cy), 5, (255, 0, 0), -1)

        status = '安定(ロック中)' if is_stable == 1.0 else '計測中...'
        info_text = (
            f'位置:({send_x:.2f},{send_y:.2f}) 角度:{int(send_angle)}°'
            f' 幅:{int(send_width)}px [{status}]'
        )
        frame = self.put_text(frame, info_text, (10, 15), box_color)

        msg = Float32MultiArray()
        msg.data = [send_x, send_y, send_angle, send_width, is_stable]
        self.publisher_.publish(msg)

      else:
        # 影やノイズで一瞬だけ見失っても10フレーム（約0.3秒）はロックを維持する
        self.lost_frames += 1
        if self.lost_frames > 10:
          self.history.clear()
          self.is_locked = False
          self.locked_data = None

      cv2.imshow('Grasp Vision System', frame)
      if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    self.cap.release()
    cv2.destroyAllWindows()


def main(args=None):
  rclpy.init(args=args)
  node = GraspVisionPublisher()
  try:
    node.run()
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()
