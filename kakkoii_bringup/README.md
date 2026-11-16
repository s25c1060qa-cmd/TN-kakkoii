# kakkoii_bringup

TN-kakkoii のノードをまとめて起動する launch と、チーム全体の設定ファイルです。

## 起動するもの

| パッケージ | ノード | 役割 |
|---|---|---|
| kakkoii_arm | `pick_and_place` | ブロックをつかんで置く |
| kakkoii_decision | `decision` | 手の位置から置く位置を決める |

手の認識（kakkoii_hand_detector）は MediaPipe を venv に入れているため、この launch には含めていません。別のターミナルで起動してください。

## 起動例

```bash
# Gazebo
ros2 launch kakkoii_bringup kakkoii.launch.py use_sim_time:=true

# 実機
ros2 launch kakkoii_bringup kakkoii.launch.py

# 自分用の設定ファイルを使う
ros2 launch kakkoii_bringup kakkoii.launch.py params_file:=/path/to/my.yaml
```

## 引数

| 名前 | 既定値 | 内容 |
|---|---|---|
| `use_sim_time` | `false` | Gazebo のときは `true` |
| `params_file` | `config/kakkoii.yaml` | 全ノードのパラメータ |

## 設定ファイル

[`config/kakkoii.yaml`](./config/kakkoii.yaml) に、ピック位置・置く高さ・領域ごとの置く位置がまとまっています。
コードを触らずに動きを調整したいときはここを編集し、`colcon build` し直してください（`--symlink-install` でビルドしていれば不要）。

## ライセンス

Apache License 2.0（[LICENSE](./LICENSE)）
