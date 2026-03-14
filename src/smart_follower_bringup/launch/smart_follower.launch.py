import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    robot_ns = LaunchConfiguration('robot_ns')
    bringup_robot = LaunchConfiguration('bringup_robot')
    bringup_camera = LaunchConfiguration('bringup_camera')

    wheeltec_share = get_package_share_directory('turn_on_wheeltec_robot')
    bringup_share = get_package_share_directory('smart_follower_bringup')
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(bringup_share))))
    model_dir = os.path.join(workspace_root, 'models')
    yolo_model = os.path.join(model_dir, 'yolo26n.onnx')
    reid_model = os.path.join(model_dir, 'reid_resnet50_2048.onnx')
    control_share = get_package_share_directory('smart_follower_control')

    perception_params = os.path.join(bringup_share, 'config', 'perception_params.yaml')
    control_params = os.path.join(control_share, 'config', 'control_params.yaml')

    wheeltec_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(wheeltec_share, 'launch', 'turn_on_wheeltec_robot.launch.py')),
        condition=IfCondition(bringup_robot),
    )

    wheeltec_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(wheeltec_share, 'launch', 'wheeltec_camera.launch.py')),
        condition=IfCondition(bringup_camera),
    )

    follower_group = GroupAction([
        PushRosNamespace(robot_ns),
        Node(
            package='smart_follower_perception',
            executable='perception_node',
            name='perception_node',
            output='screen',
            parameters=[perception_params, {'yolo.model_path': yolo_model, 'reid.model_path': reid_model}],
        ),
        Node(
            package='smart_follower_control',
            executable='follower_controller_node',
            name='follower_controller_node',
            output='screen',
            parameters=[control_params],
        ),
        Node(
            package='smart_follower_control',
            executable='ultrasonic_range_node',
            name='ultrasonic_range_node',
            output='screen',
            parameters=[control_params],
        ),
        Node(
            package='smart_follower_control',
            executable='obstacle_avoidance_node',
            name='obstacle_avoidance_node',
            output='screen',
            parameters=[control_params],
        ),
        Node(
            package='smart_follower_control',
            executable='arbiter_node',
            name='arbiter_node',
            output='screen',
            parameters=[control_params],
        ),
        Node(
            package='smart_follower_control',
            executable='keyboard_command_node',
            name='keyboard_command_node',
            output='screen',
            parameters=[control_params],
        ),
    ])

    return LaunchDescription([
        DeclareLaunchArgument('robot_ns', default_value='robot1'),
        DeclareLaunchArgument('bringup_robot', default_value='true'),
        DeclareLaunchArgument('bringup_camera', default_value='true'),
        wheeltec_robot,
        wheeltec_camera,
        follower_group,
    ])
