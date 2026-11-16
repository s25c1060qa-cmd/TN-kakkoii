from crane_x7_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='Gazebo で動かすときは true にする',
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('kakkoii_arm'), 'config', 'pick_and_place.yaml']),
        description='ピック位置などのパラメータ yaml',
    )

    description_loader = RobotDescriptionLoader()
    moveit_config = MoveItConfigsBuilder('crane_x7').to_moveit_configs()
    moveit_config.robot_description = {
        'robot_description': description_loader.load(),
    }

    params_file = LaunchConfiguration('params_file')

    pick_and_place_node = Node(
        package='kakkoii_arm',
        executable='pick_and_place',
        name='pick_and_place',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            params_file,
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        pick_and_place_node,
    ])
