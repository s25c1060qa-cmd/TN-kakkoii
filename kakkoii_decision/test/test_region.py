import pytest

from kakkoii_decision.region import classify_region, decide_place


@pytest.mark.parametrize('x, y, region, place', [
    (0.1, 0.9, 'left_bottom', (0.35, -0.13)),
    (0.1, 0.5, 'left_middle', (0.25, -0.15)),
    (0.1, 0.1, 'left_top', (0.15, -0.13)),
    (0.5, 0.9, 'center_bottom', (0.35, 0.00)),
    (0.5, 0.1, 'center_top', (0.15, 0.00)),
    (0.9, 0.9, 'right_bottom', (0.35, 0.13)),
    (0.9, 0.5, 'right_middle', (0.25, 0.13)),
    (0.9, 0.1, 'right_top', (0.15, 0.13)),
])
def test_eight_regions(x, y, region, place):
    assert classify_region(x, y) == region
    assert decide_place(x, y) == (place, region)


def test_boundary_uses_fallback():
    # 境界ちょうどは元の kakkoii.cpp と同じく (0.25, 0.0)
    assert decide_place(0.33, 0.5) == ((0.25, 0.00), None)
    assert decide_place(0.5, 0.5) == ((0.25, 0.00), None)


def test_custom_table():
    table = {'left_top': (0.2, -0.2)}
    assert decide_place(0.1, 0.1, table=table) == ((0.2, -0.2), 'left_top')
