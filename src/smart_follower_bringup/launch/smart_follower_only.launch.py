from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    robot_ns = LaunchConfiguration('robot_ns')
    bringup_share = get_package_share_directory('smart_follower_bringup')
    control_share = get_package_share_directory('smart_follower_control')

    perception_params = os.path.join(bringup_share, 'config', 'perception_params.yaml')
    control_params = os.path.join(control_share, 'config', 'control_params.yaml')

    nodes = GroupAction([
        PushRosNamespace(robot_ns),
        Node(package='smart_follower_perception', executable='perception_node', parameters=[perception_params]),
        Node(package='smart_follower_control', executable='follower_controller_node', parameters=[control_params]),
        Node(package='smart_follower_control', executable='ultrasonic_range_node', parameters=[control_params]),
        Node(package='smart_follower_control', executable='obstacle_avoidance_node', parameters=[control_params]),
        Node(package='smart_follower_control', executable='arbiter_node', parameters=[control_params]),
        Node(package='smart_follower_control', executable='keyboard_command_node', parameters=[control_params]),
    ])

    return LaunchDescription([
        DeclareLaunchArgument('robot_ns', default_value='robot1'),
        nodes,
    ])
