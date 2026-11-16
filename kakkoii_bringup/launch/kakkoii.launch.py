"""アーム制御（kakkoii_arm）と置き位置の判断（kakkoii_decision）をまとめて起動する.

手の認識（kakkoii_hand_detector）は MediaPipe を venv に入れているため、
別のターミナルで起動する（README 参照）。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false', choices=['true', 'false'],
        description='Gazebo で動かすときは true にする')
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('kakkoii_bringup'), 'config', 'kakkoii.yaml']),
        description='チーム全体のパラメータ yaml')

    arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('kakkoii_arm'), 'launch', 'pick_and_place.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items(),
    )

    decision = Node(
        package='kakkoii_decision',
        executable='decision',
        name='decision',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        arm,
        decision,
    ])
