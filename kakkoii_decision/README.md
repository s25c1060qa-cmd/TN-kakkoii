# kakkoii_decision

手の位置（画像上の座標）を受け取り、ブロックを置く位置 [m] を決めて配信します。
今は画像を 8 つの領域に分け、手と反対側にブロックを置く位置を返します。

## ノード

`decision`

## 入出力トピック

| 種類 | トピック名 | 型 | 内容 |
|---|---|---|---|
| Subscribe | `/target_point` | `geometry_msgs/msg/Point` | 手の位置（画像内の正規化座標 0.0〜1.0） |
| Publish | `/place_target` | `geometry_msgs/msg/Point` | ブロックを置く位置 `x`, `y` [m]（`z` は未使用で 0） |

## 領域と置く位置

画像を横・縦それぞれ 3 等分し（`low`=0.33 / `high`=0.66）、中央の列だけ上下 2 分割（`center_split`=0.5）します。

| 手を検出した領域 | パラメータ名 | 置く位置 (x, y) [m] |
|---|---|---|
| 左下 | `place.left_bottom` | (0.35, -0.13) |
| 左中央 | `place.left_middle` | (0.25, -0.15) |
| 左上 | `place.left_top` | (0.15, -0.13) |
| 中央下 | `place.center_bottom` | (0.35, 0.00) |
| 中央上 | `place.center_top` | (0.15, 0.00) |
| 右下 | `place.right_bottom` | (0.35, 0.13) |
| 右中央 | `place.right_middle` | (0.25, 0.13) |
| 右上 | `place.right_top` | (0.15, 0.13) |

境界ちょうどでどこにも入らない場合は `fallback`（既定 (0.25, 0.00)）を使います。
置く位置は `kakkoii_bringup/config/kakkoii.yaml` で変えられます。

## パラメータ

| 名前 | 既定値 | 内容 |
|---|---|---|
| `input_topic` | `/target_point` | 手の位置の入力 |
| `output_topic` | `/place_target` | 置く位置の出力 |
| `low`, `high` | `0.33`, `0.66` | 領域の境界 |
| `center_split` | `0.5` | 中央列の上下の境界 |
| `fallback` | `[0.25, 0.0]` | どの領域にも入らないときの置く位置 |
| `place.<領域名>` | 上の表 | 各領域の置く位置 `[x, y]` |

## 起動例

```bash
ros2 run kakkoii_decision decision
```

## 判断ロジックの改良

計算部分は ROS に依存しない [`kakkoii_decision/region.py`](./kakkoii_decision/region.py) にまとめてあります。
改良したら、ロボットを動かす前にテストで確認できます。

```bash
cd ~/ros2_ws
colcon test --packages-select kakkoii_decision
colcon test-result --verbose

# ROS なしでもその場で確認できる
cd ~/ros2_ws/src/TN-kakkoii/kakkoii_decision && python3 -m pytest test
```

## ライセンス

Apache License 2.0（[LICENSE](./LICENSE)）
