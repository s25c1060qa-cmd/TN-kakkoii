# how to start（班員用）

## 1. リポジトリのクローン
```
cd ~/ros2_ws/src
git clone git@github.com:s25c1060qa-cmd/TN-kakkoii.git
```

## 2. ビルド
```
cd ~/ros2_ws
colcon build --symlink-install --packages-up-to kakkoii_bringup kakkoii_hand_detector
source ~/ros2_ws/install/setup.bash
```

## 3. シミュレーション内での実行（Gazebo）
- ターミナル1（Gazebo起動）
```
source ~/ros2_ws/install/setup.bash
ros2 launch crane_x7_gazebo crane_x7_with_table.launch.py
```

- ターミナル2（アーム制御＋置き位置の判断）
```
source ~/ros2_ws/install/setup.bash
ros2 launch kakkoii_bringup kakkoii.launch.py use_sim_time:=true
```

- ターミナル3（手の認識）
```
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws/env/bin/activate
python3 -m kakkoii_hand_detector.hand_detector_node
```

## 担当ごとに触るパッケージ
| 作業 | パッケージ |
|---|---|
| 手の認識・RealSense対応 | kakkoii_hand_detector |
| 置く位置の計算・分岐 | kakkoii_decision（`region.py`） |
| アームの動き | kakkoii_arm |
| 起動方法・パラメータ | kakkoii_bringup（`config/kakkoii.yaml`） |
| ブロックの色認識（これから） | 新しく kakkoii_block_detector を作る |
