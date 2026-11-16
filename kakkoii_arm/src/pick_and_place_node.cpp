// Copyright 2020 RT Corporation
// Copyright 2026 TN-kakkoii (竹口昊汰, 山崎大成, 米澤慧師, 渡辺蓮)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// crane_x7_examples の pick_and_place.cpp を元に改変。
// /place_target (geometry_msgs/Point, x・y [m]) を受け取り、
// ピック位置のブロックをつかんで、その位置に置く。
// 「どこに置くか」は決めない（kakkoii_decision の担当）。

#include <cmath>
#include <thread>
#include <atomic>
#include <chrono>
#include <string>

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
  explicit PickAndPlace(rclcpp::Node::SharedPtr node, const std::string & topic)
  : node_(node), received_x_(0.0), received_y_(0.0), command_received_(false)
  {
    move_group_arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
    move_group_arm_->setMaxVelocityScalingFactor(1.0);
    move_group_arm_->setMaxAccelerationScalingFactor(1.0);

    move_group_gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");

    // kakkoii_decision からの置き位置 (Point型, x・y [m]) を受信する
    subscription_ = node_->create_subscription<geometry_msgs::msg::Point>(
      topic, 10,
      std::bind(&PickAndPlace::command_callback, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "Ready! Waiting for %s ...", topic.c_str());
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
    RCLCPP_INFO(node_->get_logger(), "Received place_target -> x: %.3f, y: %.3f [m]", msg->x, msg->y);
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MoveGroupInterface> move_group_arm_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;

  std::atomic<double> received_x_;
  std::atomic<double> received_y_;
  std::atomic<bool> command_received_;
};

// パラメータを読む（launch の yaml で上書きされていればその値、なければ既定値）
static double param(rclcpp::Node::SharedPtr node, const std::string & name, double def)
{
  double v = def;
  node->get_parameter_or(name, v, def);
  return v;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("pick_and_place", node_options);

  // MoveGroupInterfaceのデッドロックを防ぐため、スピン処理を別スレッドで走らせる
  std::thread spin_thread([node]() {rclcpp::spin(node);});

  const std::string default_topic = "/place_target";
  std::string topic;
  node->get_parameter_or("input_topic", topic, default_topic);
  PickAndPlace controller(node, topic);

  // アプローチ・退避時の高さ [m]
  const double LIFTING_HEIGHT = param(node, "lifting_height", 0.3);

  // 掴む位置（ピック位置）のXYZ[m]とRPY[deg]
  const double PICK_X = param(node, "pick_x", 0.25);
  const double PICK_Y = param(node, "pick_y", 0.0);
  const double PICK_Z = param(node, "pick_z", 0.12);
  const double PICK_ROLL = -180.0;
  const double PICK_PITCH = 0.0;
  const double PICK_YAW = -90.0;

  // 置く高さ[m]と姿勢[deg]（置く x, y は /place_target で受け取る）
  const double PLACE_Z = param(node, "place_z", 0.13);
  const double PLACE_ROLL = -180.0;
  const double PLACE_PITCH = 0.0;
  const double PLACE_YAW = -90.0;

  // 初期化動作
  controller.move_arm_to_named_pose("home");
  controller.move_gripper_angle(PickAndPlace::GRIPPER_OPEN);
  controller.set_constraints();

  // 連続実行用のメインループ
  while (rclcpp::ok()) {
    // 1. ピック位置の上空へ移動して待機状態を作る
    controller.control_arm(PICK_X, PICK_Y, LIFTING_HEIGHT, PICK_ROLL, PICK_PITCH, PICK_YAW);

    // 2. 前回の受信フラグをリセットし、新しい置き位置を待つ
    controller.reset_command();
    RCLCPP_INFO(node->get_logger(), "Waiting for new place target...");
    while (rclcpp::ok() && !controller.is_command_received()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!rclcpp::ok()) {
      break;
    }

    const double PLACE_X = controller.get_received_x();
    const double PLACE_Y = controller.get_received_y();

    // 3. ピック動作（掴みに行く）
    controller.control_arm(PICK_X, PICK_Y, PICK_Z, PICK_ROLL, PICK_PITCH, PICK_YAW);
    controller.move_gripper_angle(PickAndPlace::GRIPPER_GRASP);
    controller.control_arm(PICK_X, PICK_Y, LIFTING_HEIGHT, PICK_ROLL, PICK_PITCH, PICK_YAW);

    // 4. プレース動作（受け取った位置へ移動して置く）
    controller.control_arm(PLACE_X, PLACE_Y, LIFTING_HEIGHT, PLACE_ROLL, PLACE_PITCH, PLACE_YAW);
    controller.control_arm(PLACE_X, PLACE_Y, PLACE_Z, PLACE_ROLL, PLACE_PITCH, PLACE_YAW);
    controller.move_gripper_angle(PickAndPlace::GRIPPER_OPEN);
    controller.control_arm(PLACE_X, PLACE_Y, LIFTING_HEIGHT, PLACE_ROLL, PLACE_PITCH, PLACE_YAW);
  }

  // 終了動作
  controller.clear_constraints();

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
