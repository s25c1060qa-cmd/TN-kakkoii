# kakkoii_hand_detector

カメラ映像から MediaPipe Hands で人差し指の先端（ランドマーク8）を検出し、画像上の座標を配信します。
CRANE-X7 には依存しないので、手の位置を使う他のロボットでもそのまま使えます。

## ノード

`hand_detector`

## 入出力トピック

| 種類 | トピック名 | 型 | 内容 |
|---|---|---|---|
| Publish | `/target_point` | `geometry_msgs/msg/Point` | 人差し指先端の座標。`x`, `y` は画像内の正規化座標（0.0〜1.0、左上が原点）、`z` は MediaPipe の相対深度 |

手が映っていないときは何も配信しません。

## パラメータ

| 名前 | 既定値 | 内容 |
|---|---|---|
| `camera_index` | `0` | 使うカメラの番号（`ls /dev/video*` で確認） |
| `output_topic` | `/target_point` | 配信先トピック |
| `min_detection_confidence` | `0.7` | 検出のしきい値 |
| `min_tracking_confidence` | `0.7` | 追跡のしきい値 |
| `show_window` | `true` | カメラ映像のウィンドウを表示するか |

## 依存

MediaPipe と OpenCV は pip で入れます。Ubuntu 24.04 ではシステムの Python に直接入れられないため、venv を使います。

```bash
cd ~/ros2_ws
python3 -m venv env --system-site-packages
source env/bin/activate
pip install mediapipe opencv-python
```

## 起動例

venv の MediaPipe を使うため、`ros2 run` ではなく `python3 -m` で起動します。

```bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws/env/bin/activate
python3 -m kakkoii_hand_detector.hand_detector_node

# カメラ番号を変える場合
python3 -m kakkoii_hand_detector.hand_detector_node --ros-args -p camera_index:=2
```

カメラのウィンドウで `q` を押すと終了します。

## ライセンス

Apache License 2.0（[LICENSE](./LICENSE)）
