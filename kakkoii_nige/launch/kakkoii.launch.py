from crane_x7_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description=('Set true when using the gazebo simulator.'),
    )

    description_loader = RobotDescriptionLoader()

    moveit_config = MoveItConfigsBuilder('crane_x7').to_moveit_configs()
    moveit_config.robot_description = {
        'robot_description': description_loader.load(),
    }

    kakkoii_node = Node(
        package='kakkoii_ros',
        executable='kakkoii',
        name='pick_and_place',
        output='screen',
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
            kakkoii_node,
        ]
    )
