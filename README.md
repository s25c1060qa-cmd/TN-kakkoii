# TN-kakkoii：人の手の動きに応じたブロック配置ロボット

**千葉工業大学 未来ロボティクス学科 ロボット設計製作論実習3 ― チーム「かっこいいチーム名」**

CRANE-X7とMediaPipeを連携させ、人間の手の位置・侵入方向に応じてブロックを動かすROS 2パッケージです。

## デモ

> デモ動画は完成後に追加します（中間発表時点ではGazebo上のシミュレーションのみ）。

## 概要

人と同じ空間で作業するロボットが増え、人の位置や動きに合わせた動作が求められています。
本プロジェクトでは、色によるブロック仕分けに加えて **人間の手の位置・侵入方向** を判断条件に取り入れ、状況に応じてブロックを配置するシステムを開発します。

最終目標は次の動作です。

- **赤ブロック（危険物）**：手から遠ざける
- **青ブロック（必要なもの）**：手に近づける

### 主な機能

- **手の認識**：MediaPipe Handsで人差し指の先端座標を検出し、ROS 2トピックで送信
- **動作の分岐**：受信した座標から手の侵入位置を8方向に分類し、ブロックの置き場所を決定
- **アーム制御**：MoveIt 2による軌道計画でCRANE-X7がブロックをピック＆プレース
- **ブロックの色識別**：OpenCVで赤・青を識別（開発中）

## システム構成

```
┌──────────────┐  画像   ┌────────────────────────┐  /target_point   ┌─────────────────────┐
│ カメラ       │ ──────▶ │ media_publisher.py     │ ───────────────▶ │ kakkoii (C++)       │
│ (Webカメラ / │         │ MediaPipe・OpenCV      │ geometry_msgs/   │ MoveIt 2で軌道計画  │
│  RealSense)  │         │ 指先座標を抽出         │ Point            │ → CRANE-X7を制御    │
└──────────────┘         └────────────────────────┘                  └─────────────────────┘
```

| ノード | ファイル | 役割 |
|---|---|---|
| `mediapipe_test_publisher` | `media_publisher.py` | カメラ映像から人差し指先端（ランドマーク8）を検出し、`/target_point` に送信 |
| `pick_and_place` | `kakkoii_ros/scripts/kakkoii.cpp` | `/target_point` を受信し、手の位置に応じた場所へブロックを移動 |

`/target_point`（`geometry_msgs/msg/Point`）の `x`, `y` は画像内の正規化座標（0.0〜1.0、左上が原点）です。

## 動作環境

| 項目 | 使用技術・機材 |
|---|---|
| OS | Ubuntu 24.04.5 LTS |
| ミドルウェア | ROS 2 Jazzy |
| ロボットアーム | CRANE-X7 |
| カメラ | Intel RealSense D435（現在はWebカメラで動作確認） |
| 画像処理 | MediaPipe、OpenCV |
| 動作計画 | MoveIt 2 |
| シミュレーション | Gazebo |
| 可視化ツール | RViz2 |

## セットアップ

### 1. ROS 2およびCRANE-X7のセットアップ

- **ROS 2**：各自でROS 2 Jazzyをインストールしてください。
- **CRANE-X7関連パッケージ**：[RT社公式リポジトリ](https://github.com/rt-net/crane_x7_ros/tree/ros2)からインストールできます。

```bash
source /opt/ros/jazzy/setup.bash

# ros2_ws は各自のワークスペース名に変更してください
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone -b jazzy https://github.com/rt-net/crane_x7_ros.git
git clone -b jazzy https://github.com/rt-net/crane_x7_description.git

# 依存パッケージのインストールとビルド
rosdep install -r -y -i --from-paths .
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

### 2. このリポジトリのクローンとビルド

```bash
cd ~/ros2_ws/src
git clone https://github.com/s25c1060qa-cmd/TN-kakkoii.git

cd ~/ros2_ws
colcon build --packages-select kakkoii_ros
source ~/ros2_ws/install/setup.bash
```

### 3. MediaPipeのインストール

Ubuntu 24.04ではシステムのPythonに直接pipでインストールできないため、仮想環境（venv）を使います。

```bash
sudo apt install python3-pip python3-venv
cd ~/ros2_ws
python3 -m venv env --system-site-packages   # ROS 2のrclpyを使えるようにする
source env/bin/activate
pip install mediapipe opencv-python
```

## 使用方法

### Gazeboで実行する場合

- **ターミナル1（Gazebo起動）**
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch crane_x7_gazebo crane_x7_with_table.launch.py
  ```
- **ターミナル2（手の認識）**
  ```bash
  source ~/ros2_ws/install/setup.bash
  source ~/ros2_ws/env/bin/activate
  python3 ~/ros2_ws/src/TN-kakkoii/media_publisher.py
  ```
- **ターミナル3（アーム制御）**
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch kakkoii_ros kakkoii.launch.py use_sim_time:='true'
  ```

カメラのウィンドウ上で `q` を押すと手の認識ノードが終了します。

### 実機で実行する場合

- **USBポートの権限を設定**
  ```bash
  sudo chmod 666 /dev/ttyUSB0
  ```
- **ターミナル1（ドライバ起動）**
  ```bash
  ros2 launch crane_x7_examples demo.launch.py port_name:=/dev/ttyUSB0
  ```
- **ターミナル2・3** はGazeboの場合と同じです（ターミナル3の `use_sim_time:='true'` は外してください）。

## 動作の詳細

アームは毎回、ピック位置（x=0.25, y=0.0 [m]）の上空で待機し、`/target_point` を受信するとブロックを掴んで、**手の位置と反対側**（手から遠ざける方向）に置きます。

| 手を検出した領域（画像上） | ブロックを置く位置 (x, y) [m] |
|---|---|
| 左下 | (0.35, -0.13) |
| 左中央 | (0.25, -0.15) |
| 左上 | (0.15, -0.13) |
| 中央下 | (0.35, 0.00) |
| 中央上 | (0.15, 0.00) |
| 右下 | (0.35, 0.13) |
| 右中央 | (0.25, 0.13) |
| 右上 | (0.15, 0.13) |

画像を横・縦それぞれ3等分（0.33 / 0.66）して領域を判定しています（中央列のみ上下2分割）。置き場所を変える場合は `kakkoii.cpp` の分岐部分を編集してください。

## ディレクトリ構成

```
TN-kakkoii/
├── README.md
├── how_to_start.md          # 班員用の起動手順メモ
├── media_publisher.py       # MediaPipeによる手の認識ノード
└── kakkoii_ros/             # アーム制御用ROS 2パッケージ
    ├── CMakeLists.txt
    ├── package.xml
    ├── launch/
    │   └── kakkoii.launch.py
    └── scripts/
        └── kakkoii.cpp
```

## 開発状況

2026年11月15日（中間発表）時点の状況です。

| 作業内容 | 進捗 |
|---|---|
| 画像処理とアームの通信テスト、初期位置の設定 | ○ 完了 |
| MediaPipeによる手の侵入検知 | ○ 完了 |
| Webカメラとの連携 | ○ 完了 |
| 手の位置による動作分岐（8分岐） | ○ 完了 |
| RealSenseとの連携 | △ 進行中 |
| ブロックの認識・色識別（赤／青） | × 未着手 |
| 色による分岐（青を近づける動作） | × 未着手 |
| ブロックの位置による分岐、手の位置による複雑な分岐 | × 未着手 |
| ブロックを置く位置の計算 | × 未着手 |
| 実機での動作調整 | × 未着手 |

○：達成　△：進行中　×：未達成

## 今後の課題

- RealSense D435によるブロックの認識と色識別（竹口・山崎）
- 色の識別による分岐、手の位置による複雑な分岐（渡辺）
- ブロックを置く位置の計算プログラム（米澤）
- 実機での動作確認と調整

## 開発メンバー

| メンバー | 担当 |
|---|---|
| 竹口昊汰 | Git・GitHubの管理、開発環境の構築 |
| 山崎大成 | MediaPipeによる手の認識プログラム、Webカメラ→RealSenseへのコード改変 |
| 米澤慧師 | ブロックの準備、発表資料作成、プログラムの改良 |
| 渡辺蓮 | ロボットアームの動作制御、分岐プログラム |

## 参考

- [rt-net/crane_x7_ros](https://github.com/rt-net/crane_x7_ros/tree/ros2)（`crane_x7_examples` の `pick_and_place.cpp` を参考にアーム制御を作成）
- S・N班 色付きブロック仕分けロボット（色識別の参考）
- [cigarette_dispenser_robot](https://github.com/2025-RobotDesign3-YFK/cigarette_dispenser_robot)（MediaPipeとCRANE-X7の連携の参考）

## ライセンス

© 2026 竹口昊汰, 山崎大成, 米澤慧師, 渡辺蓮

このリポジトリは [Apache License 2.0](./LICENSE) のもとで公開されています。

本リポジトリは RT Corporation の [crane_x7_ros](https://github.com/rt-net/crane_x7_ros) / crane_x7_examples を改変したコードを含みます（Copyright 2020 RT Corporation, Apache License 2.0）。
