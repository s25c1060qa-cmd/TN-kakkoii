# kakkoii_arm

CRANE-X7 で、決まった位置（ピック位置）にあるブロックをつかみ、受け取った位置に置きます。
「どこに置くか」は判断せず、`/place_target` で受け取った位置にそのまま運びます。
そのため、置き場所を決める仕組みを差し替えても、このパッケージはそのまま使えます。

## ノード

`pick_and_place`

## 入出力トピック

| 種類 | トピック名 | 型 | 内容 |
|---|---|---|---|
| Subscribe | `/place_target` | `geometry_msgs/msg/Point` | ブロックを置く位置 `x`, `y` [m] |

## 動作の流れ

1. 起動時に `home` 姿勢へ移動し、グリッパを開く
2. ピック位置の上空で待機する
3. `/place_target` を受け取ったら、ブロックをつかんで持ち上げる
4. 受け取った位置の上空へ移動して下ろし、グリッパを開いて離す
5. 2 に戻る（待機中に届いた最新の 1 件だけを使う）

## パラメータ

| 名前 | 既定値 | 内容 |
|---|---|---|
| `input_topic` | `/place_target` | 置く位置の入力 |
| `lifting_height` | `0.3` | つかむ前・置いた後に上げる高さ [m] |
| `pick_x`, `pick_y`, `pick_z` | `0.25`, `0.0`, `0.12` | ブロックをつかむ位置 [m] |
| `place_z` | `0.13` | ブロックを置く高さ [m] |

既定値は [`config/pick_and_place.yaml`](./config/pick_and_place.yaml) にあります。
yaml に書く数値は、整数でも `0.0` のように小数点を付けてください（`0` と書くと型エラーになります）。

## 起動例

```bash
# Gazebo
ros2 launch kakkoii_arm pick_and_place.launch.py use_sim_time:=true

# 単体で動作確認（置く位置を手で送る）
ros2 topic pub --once /place_target geometry_msgs/msg/Point "{x: 0.15, y: 0.13, z: 0.0}"
```

## 依存

- [rt-net/crane_x7_ros](https://github.com/rt-net/crane_x7_ros)（crane_x7_moveit_config など）
- [rt-net/crane_x7_description](https://github.com/rt-net/crane_x7_description)

## ライセンス

Apache License 2.0（[LICENSE](./LICENSE)）

`src/pick_and_place_node.cpp` は RT Corporation の crane_x7_examples（`pick_and_place.cpp`、Copyright 2020 RT Corporation, Apache License 2.0）を改変したものです。
