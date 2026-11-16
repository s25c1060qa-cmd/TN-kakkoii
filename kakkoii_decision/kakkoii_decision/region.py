"""手の位置（画像上の正規化座標）から、ブロックを置く位置を決める計算.

ROS に依存しない普通の Python なので、pytest で単体テストできる。
"""

# 領域名 → 置く位置 (x, y) [m]。手の位置と反対側に置く。
DEFAULT_PLACE_TABLE = {
    'left_bottom': (0.35, -0.13),
    'left_middle': (0.25, -0.15),
    'left_top': (0.15, -0.13),
    'center_bottom': (0.35, 0.00),
    'center_top': (0.15, 0.00),
    'right_bottom': (0.35, 0.13),
    'right_middle': (0.25, 0.13),
    'right_top': (0.15, 0.13),
}

# どの領域にも入らなかったとき（境界ちょうどなど）の置き位置
DEFAULT_FALLBACK = (0.25, 0.00)


def classify_region(x, y, low=0.33, high=0.66, center_split=0.5):
    """画像上の座標 (x, y) を 8 つの領域のどれかに分類する.

    x, y は 0.0〜1.0（左上が原点）。横・縦をそれぞれ 3 等分し、
    中央の列だけは上下 2 分割にする。どこにも入らなければ None。
    """
    if x < low:
        column = 'left'
    elif low < x < high:
        column = 'center'
    elif high < x:
        column = 'right'
    else:
        return None

    if column == 'center':
        if center_split < y:
            return 'center_bottom'
        if y < center_split:
            return 'center_top'
        return None

    if high < y:
        return f'{column}_bottom'
    if low < y < high:
        return f'{column}_middle'
    if y < low:
        return f'{column}_top'
    return None


def decide_place(x, y, table=None, fallback=DEFAULT_FALLBACK, **thresholds):
    """手の位置から、置く位置 (x, y) [m] と領域名を返す."""
    table = DEFAULT_PLACE_TABLE if table is None else table
    region = classify_region(x, y, **thresholds)
    if region is None or region not in table:
        return fallback, region
    return tuple(table[region]), region
