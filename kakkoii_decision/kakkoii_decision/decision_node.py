"""手の位置を受け取り、ブロックを置く位置を配信するノード.

入力: /target_point (geometry_msgs/Point) 画像上の正規化座標 0.0〜1.0
出力: /place_target (geometry_msgs/Point) 置く位置 x, y [m]（z は未使用で 0）
"""
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node

from kakkoii_decision.region import DEFAULT_FALLBACK, DEFAULT_PLACE_TABLE, decide_place


class DecisionNode(Node):

    def __init__(self):
        super().__init__('decision')

        self.declare_parameter('input_topic', '/target_point')
        self.declare_parameter('output_topic', '/place_target')
        self.declare_parameter('low', 0.33)
        self.declare_parameter('high', 0.66)
        self.declare_parameter('center_split', 0.5)
        self.declare_parameter('fallback', list(DEFAULT_FALLBACK))
        # 置き位置の表。yaml の place.<領域名>: [x, y] で上書きできる
        for name, xy in DEFAULT_PLACE_TABLE.items():
            self.declare_parameter(f'place.{name}', list(xy))

        self.table = {
            name: tuple(self.get_parameter(f'place.{name}').value)
            for name in DEFAULT_PLACE_TABLE
        }
        self.fallback = tuple(self.get_parameter('fallback').value)
        self.thresholds = {
            'low': self.get_parameter('low').value,
            'high': self.get_parameter('high').value,
            'center_split': self.get_parameter('center_split').value,
        }

        self.publisher_ = self.create_publisher(
            Point, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            Point, self.get_parameter('input_topic').value, self.callback, 10)
        self.get_logger().info('Decision node ready.')

    def callback(self, msg):
        (place_x, place_y), region = decide_place(
            msg.x, msg.y, table=self.table, fallback=self.fallback, **self.thresholds)
        out = Point()
        out.x = float(place_x)
        out.y = float(place_y)
        self.publisher_.publish(out)
        self.get_logger().debug(
            f'hand=({msg.x:.2f}, {msg.y:.2f}) region={region} -> place=({place_x}, {place_y})')


def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
