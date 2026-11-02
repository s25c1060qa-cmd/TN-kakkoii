#include <cmath>
#include <thread>
#include <atomic>

#include "angles/angles.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class PickAndPlace
{
public:
  // グリッパの開閉角度
  inline static const double GRIPPER_OPEN = angles::from_degrees(60.0);
  inline static const double GRIPPER_GRASP = angles::from_degrees(20.0);
  inline static const double GRIPPER_CLOSE = 0.0;

  // ノードを受け取り、MoveGroupInterfaceやサブスクライバーを初期化する
  explicit PickAndPlace(rclcpp::Node::SharedPtr node)
  : node_(node), received_x_(0.0), command_received_(false)
  {
    move_group_arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
    move_group_arm_->setMaxVelocityScalingFactor(1.0);
    move_group_arm_->setMaxAccelerationScalingFactor(1.0);

    move_group_gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");

    // MediaPipe等からの /target_point (Point型) を受信するサブスクライバーの作成
    subscription_ = node_->create_subscription<geometry_msgs::msg::Point>(
      "/target_point", 10,
      std::bind(&PickAndPlace::command_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(node_->get_logger(), "Ready! Waiting for /target_point data...");
  }

  // アームを目標位置・姿勢（Pose）に動かす
  void move_arm_to_pose(const geometry_msgs::msg::Pose & pose)
  {
    move_group_arm_->setPoseTarget(pose);
    move_group_arm_->move();
  }

  // アームを目標位置（x, y, z [m]）・姿勢（roll, pitch, yaw [deg]）に動かす
  void control_arm(
    const double x, const double y, const double z,
    const double roll, const double pitch, const double yaw)
  {
    geometry_msgs::msg::Pose target_pose;
    tf2::Quaternion q;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    q.setRPY(angles::from_degrees(roll), angles::from_degrees(pitch), angles::from_degrees(yaw));
    target_pose.orientation = tf2::toMsg(q);
    move_arm_to_pose(target_pose);
  }

  // SRDFに定義された姿勢名でアームを動かす
  void move_arm_to_named_pose(const std::string & name)
  {
    move_group_arm_->setNamedTarget(name);
    move_group_arm_->move();
  }

  // グリッパを角度[rad]を指定して開閉する
  void move_gripper_angle(const double angle)
  {
    auto joint_values = move_group_gripper_->getCurrentJointValues();
    joint_values[0] = angle;
    move_group_gripper_->setJointValueTarget(joint_values);
    move_group_gripper_->move();
  }

  // アームの関節の一部に可動制限を設定する
  void set_constraints()
  {
    moveit_msgs::msg::Constraints constraints;
    constraints.name = "arm_constraints";

    moveit_msgs::msg::JointConstraint joint_constraint;
    joint_constraint.joint_name = "crane_x7_lower_arm_fixed_part_joint";
    joint_constraint.position = 0.0;
    joint_constraint.tolerance_above = angles::from_degrees(30);
    joint_constraint.tolerance_below = angles::from_degrees(30);
    joint_constraint.weight = 1.0;
    constraints.joint_constraints.push_back(joint_constraint);

    joint_constraint.joint_name = "crane_x7_upper_arm_revolute_part_twist_joint";
    joint_constraint.position = 0.0;
    joint_constraint.tolerance_above = angles::from_degrees(30);
    joint_constraint.tolerance_below = angles::from_degrees(30);
    joint_constraint.weight = 0.8;
    constraints.joint_constraints.push_back(joint_constraint);

    move_group_arm_->setPathConstraints(constraints);
  }

  // 設定された関節可動制限をクリアする
  void clear_constraints()
  {
    move_group_arm_->clearPathConstraints();
  }

  bool is_command_received() const
  {
    return command_received_;
  }

  double get_received_x() const
  {
    return received_x_;
  }

  double get_received_y() const
  {
    return received_y_;
  }

  // 次の入力を待つためにフラグをリセットする関数
  void reset_command()
  {
    command_received_ = false;
  }

private:
  void command_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    received_x_ = msg->x;
    received_y_ = msg->y;
    command_received_ = true;
    RCLCPP_INFO(node_->get_logger(), "Received target_point -> x: %.4f, y: %.4f, z: %.4f", msg->x, msg->y, msg->z);
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MoveGroupInterface> move_group_arm_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
  
  std::atomic<double> received_x_;
  std::atomic<double> received_y_;
  std::atomic<bool> command_received_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("pick_and_place", node_options);

  // MoveGroupInterfaceのデッドロックを防ぐため、スピン処理を別スレッドで走らせる
  std::thread spin_thread([node]() { rclcpp::spin(node); });

  PickAndPlace controller(node);

  // アプローチ・退避時の高さ
  const double LIFTING_HEIGHT = 0.3;

  // 掴む位置（ピック位置）のXYZ[m]とRPY[deg]
  const double PICK_X = 0.25;
  const double PICK_Y = 0.0;
  const double PICK_Z = 0.12;
  const double PICK_ROLL = -180.0;
  const double PICK_PITCH = 0.0;
  const double PICK_YAW = -90.0;

  // 初期化動作
  controller.move_arm_to_named_pose("home");
  controller.move_gripper_angle(PickAndPlace::GRIPPER_OPEN);
  controller.set_constraints();

  // 連続実行用のメインループ
  while (rclcpp::ok()) {
    // 1. ピック位置の上空へ移動して待機状態を作る
    controller.control_arm(PICK_X, PICK_Y, LIFTING_HEIGHT, PICK_ROLL, PICK_PITCH, PICK_YAW);

    // 2. 前回の受信フラグをリセットし、新しい /target_point の入力を待機
    controller.reset_command();
    RCLCPP_INFO(node->get_logger(), "Waiting for new target point data...");
    while (rclcpp::ok() && !controller.is_command_received()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!rclcpp::ok()) {
      break;
    }

    // 3. 受け取ったx軸の値を判定
    double x_value = controller.get_received_x();
    double place_y = 0.0; // デフォルト
    double y_value = controller.get_received_y();
    double place_x = 0.25;

    if (x_value < 0.33 && 0.66 < y_value) {
      place_x = 0.40;
      place_y = -0.15; //左下を検知し、右上に移動
    } else if(x_value < 0.33 && (0.33 < y_value && y_value  < 0.66)) {
      place_x = 0.25;
      place_y = -0.15; //左真ん中を検知し、右に移動
    } else if(x_value < 0.33 && y_value < 0.33) {
      place_x = 0.10;
      place_y = -0.15; //左上を検知し、右上に移動
    } else if((0.33 < x_value && x_value <0.66) && 0.5 < y_value ) {
      place_x = 0.40;
      place_y = 0.00; //下を検知し、上に移動
    } else if((0.33 < x_value && x_value <0.66) && y_value < 0.5) {
      place_x = 0.10;
      place_y = 0.00; //上を検知し、下に移動
    } else if(0.66 < x_value && 0.66 < y_value) {
      place_x = 0.40;
      place_y = 0.15; //右下を検知し、左上に移動
    } else if(0.66 < x_value && (0.33 < y_value && y_value < 0.66)) {
      place_x = 0.25;
      place_y = 0.15; //右真ん中を検知し、左に移動
    } else if(0.66 < x_value && y_value < 0.33) {
      place_x = 0.10;
      place_y = 0.15; //右上を検知し、左上に移動
    }

    // プレース位置の設定
    const double PLACE_X = place_x;
    const double PLACE_Y = place_y;
    const double PLACE_Z = 0.13;
    const double PLACE_ROLL = -180.0;
    const double PLACE_PITCH = 0.0;
    const double PLACE_YAW = -90.0; // 定義

    // 4. ピック動作（掴みに行く）
    controller.control_arm(PICK_X, PICK_Y, PICK_Z, PICK_ROLL, PICK_PITCH, PICK_YAW);
    controller.move_gripper_angle(PickAndPlace::GRIPPER_GRASP);
    controller.control_arm(PICK_X, PICK_Y, LIFTING_HEIGHT, PICK_ROLL, PICK_PITCH, PICK_YAW);

    // 5. プレース動作（指定された左右の位置へ移動して置く - PLACE_YAWを使用）
    controller.control_arm(PLACE_X, PLACE_Y, LIFTING_HEIGHT, PLACE_ROLL, PLACE_PITCH, PLACE_YAW);
    controller.control_arm(PLACE_X, PLACE_Y, PLACE_Z, PLACE_ROLL, PLACE_PITCH, PLACE_YAW);
    controller.move_gripper_angle(PickAndPlace::GRIPPER_OPEN);
    controller.control_arm(PLACE_X, PLACE_Y, LIFTING_HEIGHT, PLACE_ROLL, PLACE_PITCH, PLACE_YAW);

    // 6. ホームに戻る
    //controller.move_arm_to_named_pose("home");
    //controller.move_gripper_angle(PickAndPlace::GRIPPER_OPEN);
  }

  // 終了動作
  controller.clear_constraints();
  
  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
