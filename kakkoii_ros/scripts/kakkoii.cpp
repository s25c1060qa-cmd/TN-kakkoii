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
//座標の番号を指定して持ち上げる関数。（真ん中が０で上から時計回りに１〜８,上空0、掴む1)
  void move_place(int xi,int yi, int h){
    double x = 0.25;
    double y = 0.0;
    if(xi==0 && yi ==0){//真ん中
      x = 0.25;
      y = -0;
    }else if(xi ==0 && yi ==1){//上
      x = 0.35;
      y = 0.0;
    }else if(xi == 1 && yi == 1){//右上
      x = 0.35;
      y = -0.13;
    }else if(xi == 1 && yi == 0){//右
      x = 0.25;
      y = -0.13; 
    }else if(xi == 1 && yi == -1){//右下
      x = 0.15;
      y = -0.13;
    }else if(xi == 0 && yi == -1){//下
      x = 0.15;
      y = 0.00;
    }else if(xi == -1 && yi == -1){//左下
      x = 0.15;
      y = 0.13;
    }else if(xi ==  -1 && yi == 0){//左
      x = 0.25;
      y = 0.13;
    }else if(xi == -1 && yi == 1){//左上
      x = 0.35;
      y = 0.13;
    }else{//真ん中
      x = 0.25;
      y = 0.00;
    }
    if(h == 0){
      control_arm(x,y,0.30,-180.0,0,-90.0);//ここで監視する高さを設定
    }else{
      control_arm(x,y,0.12,-180.0,0,-90.0);//ここで掴む高さを設定
    }
  }

private:
  void command_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    received_x_ = msg->x;
    received_y_ = msg->y;
    command_received_ = true;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MoveGroupInterface> move_group_arm_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
  
  std::atomic<double> received_x_;
  std::atomic<double> received_y_;
  std::atomic<bool> command_received_;
};

//検知した座標によって番号を返す関数。(真ん中上から時計回りに、1,2,3,4,5,6,7,8)
int fing_place(double x, double y){
  double nx = x -0.5;
  double ny = -(y -0.5);
//ラジアンで取得
  double rad = std::atan2(ny,nx);
//度数法で取得
  double deg = rad * 180.0 / 3.141592;
//条件分岐
  if(67.5 <= deg && deg <112.5){
    return 1;
  }else if(22.5 <= deg && deg <67.5){
    return 2;
  }else if(-22.5 <= deg && deg < 22.5){
    return 3;
  }else if(-67.5 <= deg && deg < -22.5){
    return 4;
  }else if(-112.5 <= deg && deg < -67.5){
    return 5;
  }else if(-157.5 <= deg && deg < -112.5){
    return 6;
  }else if(112.5 <= deg && deg <157.5){
    return 8;
  }else{
    return 7;
  }
}


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
  //const double PICK_Z = 0.12;
  const double PICK_ROLL = -180.0;
  const double PICK_PITCH = 0.0;
  const double PICK_YAW = -90.0;
  //ロボットの移動先
  int mx = 0;
  int my = 0;
  //ロボットの現在位置
  int dx = 0;
  int dy = 0;
  // 初期化動作
  controller.move_arm_to_named_pose("home");
  controller.move_gripper_angle(PickAndPlace::GRIPPER_OPEN);
  controller.set_constraints();
  controller.control_arm(PICK_X, PICK_Y, LIFTING_HEIGHT, PICK_ROLL, PICK_PITCH, PICK_YAW);
  // 連続実行用のメインループ
  while (rclcpp::ok()) {
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
    double y_value = controller.get_received_y();
  
  if (fing_place(x_value,y_value)==6){
      mx += 1; my += 1;
      RCLCPP_INFO(node->get_logger(),"左下");
    } else if(fing_place(x_value,y_value)==7) {
      mx += 1; my+=0;
      RCLCPP_INFO(node->get_logger(),"左");
    } else if(fing_place(x_value,y_value) == 8) {
      mx += 1;my +=-1;
      RCLCPP_INFO(node->get_logger(),"左上");
    } else if(fing_place(x_value,y_value)==5 ) {
      mx += 0;my +=1;
      RCLCPP_INFO(node->get_logger(),"下");
    } else if(fing_place(x_value,y_value) == 1) {
      mx += 0; my += -1;
      RCLCPP_INFO(node->get_logger(),"上");
    } else if(fing_place(x_value,y_value)==4) {
      mx += -1; my += 1;
      RCLCPP_INFO(node->get_logger(),"右下");
    } else if(fing_place(x_value,y_value)==3) {
      mx += -1; my += 0;
      RCLCPP_INFO(node->get_logger(),"右");
    } else if(fing_place(x_value,y_value)==2) {
      mx += -1;my += -1;
      RCLCPP_INFO(node->get_logger(),"右上");
    }
    if(mx > 1){
      mx = -(mx -1);
    }
    if(mx < -1){
      mx = -(mx +1);
    }
    if(my > 1){
      my = -(my -1);
    }
    if(my < -1){
      my = -(my +1);
    }
    // 4. ピック動作（掴みに行く）
    controller.move_place(dx,dy,1);
    controller.move_gripper_angle(PickAndPlace::GRIPPER_GRASP);
    controller.move_place(dx,dy,0);
    // 5. プレース動作（指定された左右の位置へ移動して置く - PLACE_YAWを使用）
    controller.move_place(mx,my,0);
    controller.move_place(mx,my,1);
    controller.move_gripper_angle(PickAndPlace::GRIPPER_OPEN);
    controller.move_place(mx,my,0);

    dx =mx; dy =my;
  }

  // 終了動作
  controller.clear_constraints();

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
