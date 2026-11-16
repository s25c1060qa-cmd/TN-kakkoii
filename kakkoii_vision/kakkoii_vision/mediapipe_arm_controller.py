import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, PositionConstraint, 
    OrientationConstraint, BoundingVolume, PlanningOptions
)
from shape_msgs.msg import SolidPrimitive

class MediaPipeArmController(Node):
    def __init__(self):
        super().__init__('mediapipe_arm_controller')

        # MediaPipeからの座標トピックを購読
        self.subscription = self.create_subscription(
            Point,
            '/target_point',
            self.target_callback,
            10
        )

        # MoveIt 2 の MoveGroup アクションクライアントを作成
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info('MoveIt 2のアクションサーバー接続を待機中...')
        self._action_client.wait_for_server()
        self.get_logger().info('MoveIt 2への接続成功！')

        self.is_moving = False

    def target_callback(self, msg: Point):
        if self.is_moving:
            return

        # 1. 座標のマッピング変換（安全な可動域に抑える）
        # Y軸（左右）: -0.15m ~ +0.15m
        target_y = (0.5 - msg.x) * 0.3

        # Z軸（上下）: +0.15m ~ +0.35m
        target_z = (1.0 - msg.y) * 0.2 + 0.15

        # X軸（奥行き）: 前方0.25m固定
        target_x = 0.25

        self.get_logger().info(f'目標送信 -> X:{target_x:.2f}, Y:{target_y:.2f}, Z:{target_z:.2f}')
        self.send_pose_goal(target_x, target_y, target_z)

    def send_pose_goal(self, x, y, z):
        self.is_moving = True

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 0.5

        # 【重要】計画(Plan)だけでなく実際の動作実行(Execute)を有効化
        goal_msg.planning_options.plan_only = False

        # 位置の拘束
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = 'base_link'
        pos_constraint.link_name = 'crane_x7_gripper_base_link'

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.05]  # 誤差許容範囲 5cm

        target_pose = Pose()
        target_pose.position.x = float(x)
        target_pose.position.y = float(y)
        target_pose.position.z = float(z)

        volume = BoundingVolume()
        volume.primitives.append(primitive)
        volume.primitive_poses.append(target_pose)
        pos_constraint.constraint_region = volume
        pos_constraint.weight = 1.0

        # 姿勢の拘束（手先を斜め下に向ける姿勢に固定してIK計算を安定させる）
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = 'base_link'
        ori_constraint.link_name = 'crane_x7_gripper_base_link'
        ori_constraint.orientation.x = 0.0
        ori_constraint.orientation.y = 0.707
        ori_constraint.orientation.z = 0.0
        ori_constraint.orientation.w = 0.707
        ori_constraint.absolute_x_axis_tolerance = 0.5
        ori_constraint.absolute_y_axis_tolerance = 0.5
        ori_constraint.absolute_z_axis_tolerance = 0.5
        ori_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)
        goal_msg.request.goal_constraints.append(constraints)

        # アクション送信
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('× 計画拒否（可動域外または姿勢不可）')
            self.is_moving = False
            return

        self.get_logger().info('○ 計画承認 -> 動作実行中...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info('✓ 到着完了！')
        self.is_moving = False

def main(args=None):
    rclpy.init(args=args)
    controller = MediaPipeArmController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
