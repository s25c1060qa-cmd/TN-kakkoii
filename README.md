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

役割ごとに 4 つの ROS 2 パッケージに分けています。パッケージどうしはトピックだけでつながっているので、1 つを差し替えても他はそのまま使えます。

```
┌────────────┐ 画像 ┌──────────────────────┐ /target_point ┌──────────────────┐ /place_target ┌──────────────────┐
│ カメラ     │ ───▶ │ kakkoii_hand_detector│ ────────────▶ │ kakkoii_decision │ ────────────▶ │ kakkoii_arm      │
│ (Webカメラ │      │ MediaPipeで指先検出  │ 画像上の座標  │ 置く位置を決める │ 置く位置 [m]  │ MoveIt 2で       │
│ /RealSense)│      │                      │ (0.0〜1.0)    │                  │               │ ピック＆プレース │
└────────────┘      └──────────────────────┘               └──────────────────┘               └──────────────────┘
```

| パッケージ | ノード | 役割 | 詳細 |
|---|---|---|---|
| `kakkoii_hand_detector` | `hand_detector` | カメラ映像から人差し指先端を検出し `/target_point` に送信 | [README](./kakkoii_hand_detector/README.md) |
| `kakkoii_decision` | `decision` | 手の位置から置く位置を決めて `/place_target` に送信 | [README](./kakkoii_decision/README.md) |
| `kakkoii_arm` | `pick_and_place` | `/place_target` を受け取り、ブロックをつかんでそこへ置く | [README](./kakkoii_arm/README.md) |
| `kakkoii_bringup` | － | arm と decision をまとめて起動する launch と設定 yaml | [README](./kakkoii_bringup/README.md) |

| トピック | 型 | 内容 |
|---|---|---|
| `/target_point` | `geometry_msgs/msg/Point` | 手の位置。`x`, `y` は画像内の正規化座標（0.0〜1.0、左上が原点） |
| `/place_target` | `geometry_msgs/msg/Point` | ブロックを置く位置。`x`, `y` [m]（ロボット基準） |

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
colcon build --symlink-install --packages-up-to kakkoii_bringup kakkoii_hand_detector
source ~/ros2_ws/install/setup.bash
```

### 3. 画像処理に必要なライブラリ・フォントのインストール

`ros2 run` から各プログラムを直接起動できるよう、以下の2つをインストールします（ターミナルのどのフォルダにいても実行できます）。

* **日本語表示用フォント・画像処理ツール**（`python3-pil` / `fonts-noto-cjk`）：カメラ映像に「赤」「青」「ロック完了」などの日本語を表示するために使用します。
* **MediaPipe と NumPy**（バージョン指定）：最新のバージョンではエラーになるため、動作が確認できているバージョンを指定して導入します。

```bash
sudo apt update
sudo apt install -y python3-pil fonts-noto-cjk
pip install "mediapipe<0.10.30" "numpy<2" "opencv-contrib-python<4.11" --break-system-packages
```

### 4. ビルドと画像処理プログラムの起動

必要なライブラリのインストール後、ワークスペースのルート（`~/ros2_ws`）に移動して画像処理パッケージをビルドします。

```bash
cd ~/ros2_ws
colcon build --packages-select kakkoii_vision
source ~/ros2_ws/install/setup.bash
```

#### 起動コマンド（用途に合わせて実行）
* **ブロック認識**：`ros2 run kakkoii_vision block_detector`
* **色識別**：`ros2 run kakkoii_vision color_detector`
* **手の追跡（MediaPipe）**：`ros2 run kakkoii_vision mediapipe_tracker`

## 使用方法

### Gazeboで実行する場合

- **ターミナル1（Gazebo起動）**
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch crane_x7_gazebo crane_x7_with_table.launch.py
  ```
- **ターミナル2（アーム制御＋置き位置の判断）**
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch kakkoii_bringup kakkoii.launch.py use_sim_time:=true
  ```
  `Waiting for new place target...` と出たら準備完了です。
- **ターミナル3（手の認識）**
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 run kakkoii_hand_detector hand_detector
  ```

カメラのウィンドウ上で `q` を押すと手の認識ノードが終了します。
MediaPipe を venv に入れている場合は、venv を有効にしてから `python3 -m kakkoii_hand_detector.hand_detector_node` で起動してください（`ros2 run` では venv の MediaPipe が見つかりません）。

### 実機で実行する場合

- **USBポートの権限を設定**
  ```bash
  sudo chmod 666 /dev/ttyUSB0
  ```
- **ターミナル1（ドライバ起動）**
  ```bash
  ros2 launch crane_x7_examples demo.launch.py port_name:=/dev/ttyUSB0
  ```
- **ターミナル2・3** はGazeboの場合と同じです（ターミナル2の `use_sim_time:=true` は外してください）。

### カメラなしで動作確認する場合

ターミナル3の代わりに、手の位置を手で送れます（例：右上に手がある）。

```bash
ros2 topic pub --once /target_point geometry_msgs/msg/Point "{x: 0.9, y: 0.1, z: 0.0}"
```

## 動作の詳細

アームは毎回、ピック位置（x=0.25, y=0.0 [m]）の上空で待機し、`/place_target` を受信するとブロックを掴んで、その位置に置きます。
置く位置は kakkoii_decision が **手の位置と反対側**（手から遠ざける方向）になるよう決めます。

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

画像を横・縦それぞれ3等分（0.33 / 0.66）して領域を判定しています（中央列のみ上下2分割）。
ピック位置・置く位置は [`kakkoii_bringup/config/kakkoii.yaml`](./kakkoii_bringup/config/kakkoii.yaml) で変更できます（コードの修正は不要）。

## ディレクトリ構成

```
TN-kakkoii/
├── README.md
├── LICENSE
├── how_to_start.md                  # 班員用の起動手順メモ
├── kakkoii_hand_detector/           # 手の認識（Python）
│   ├── kakkoii_hand_detector/hand_detector_node.py
│   ├── package.xml / setup.py / setup.cfg
│   ├── README.md
│   └── LICENSE
├── kakkoii_decision/                # 置く位置の判断（Python）
│   ├── kakkoii_decision/
│   │   ├── decision_node.py         # ROS 2ノード
│   │   └── region.py                # 判断ロジック（ROSに依存しない）
│   ├── test/test_region.py
│   ├── package.xml / setup.py / setup.cfg
│   ├── README.md
│   └── LICENSE
├── kakkoii_arm/                     # ピック＆プレース（C++）
│   ├── src/pick_and_place_node.cpp
│   ├── launch/pick_and_place.launch.py
│   ├── config/pick_and_place.yaml
│   ├── CMakeLists.txt / package.xml
│   ├── README.md
│   └── LICENSE
└── kakkoii_bringup/                 # まとめて起動
    ├── launch/kakkoii.launch.py
    ├── config/kakkoii.yaml
    ├── CMakeLists.txt / package.xml
    ├── README.md
    └── LICENSE
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

- [rt-net/crane_x7_ros](https://github.com/rt-net/crane_x7_ros/tree/ros2)（`crane_x7_examples` の `pick_and_place.cpp` を元に kakkoii_arm を作成）
- S・N班 色付きブロック仕分けロボット（色識別の参考）
- [cigarette_dispenser_robot](https://github.com/2025-RobotDesign3-YFK/cigarette_dispenser_robot)（MediaPipeとCRANE-X7の連携の参考）

## ライセンス

© 2026 竹口昊汰, 山崎大成, 米澤慧師, 渡辺蓮

このリポジトリは [Apache License 2.0](./LICENSE) のもとで公開されています。

本リポジトリは RT Corporation の [crane_x7_ros](https://github.com/rt-net/crane_x7_ros) / crane_x7_examples を改変したコードを含みます（Copyright 2020 RT Corporation, Apache License 2.0）。
