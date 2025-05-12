#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Generate launch description for cylinder landing mission."""

    pkg_share = get_package_share_directory('terrain_mapping_drone_control')
    gz_model_path = os.path.join(pkg_share, 'models')
    os.environ['PX4_GZ_MODEL_POSE'] = "0,0,0.1,0,0,0"

    px4_autopilot_path = LaunchConfiguration('px4_autopilot_path')

    px4_sitl = ExecuteProcess(
        cmd=['make', 'px4_sitl', 'gz_x500_depth_mono'],
        cwd=px4_autopilot_path,
        output='screen'
    )

    spawn_cylinder_front = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder', 'model.sdf'),
            '-name', 'cylinder_front',
            '-x', '5', '-y', '0', '-z', '0',
            '-R', '0', '-P', '0', '-Y', '0',
            '-scale', '1 1 1',
            '-static'
        ],
        output='screen'
    )

    spawn_cylinder_back = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder_short', 'model.sdf'),
            '-name', 'cylinder_back',
            '-x', '-5', '-y', '0', '-z', '0',
            '-R', '0', '-P', '0', '-Y', '0',
            '-static'
        ],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/rgb_camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/rgb_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/mono_camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/mono_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        remappings=[
            ('/rgb_camera', '/drone/front_rgb'),
            ('/rgb_camera/camera_info', '/drone/front_rgb/camera_info'),
            ('/depth_camera', '/drone/front_depth'),
            ('/depth_camera/points', '/drone/front_depth/points'),
            ('/camera_info', '/drone/front_depth/camera_info'),
            ('/mono_camera', '/drone/down_mono'),
            ('/mono_camera/camera_info', '/drone/down_mono/camera_info'),
        ],
        output='screen'
    )

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[{
            'fcu_url': 'udp://:14540@127.0.0.1:14557',
            'gcs_url': '',
            'target_system_id': 1,
            'target_component_id': 1,
            'system_id': 1,
            'component_id': 1,
            'log_output': 'screen'
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'px4_autopilot_path',
            default_value='/home/chakrireddy1974/PX4-Autopilot',
            description='Path to PX4-Autopilot directory'),
        px4_sitl,
        TimerAction(period=2.0, actions=[spawn_cylinder_front]),
        TimerAction(period=2.5, actions=[spawn_cylinder_back]),
        TimerAction(period=3.0, actions=[bridge]),
        TimerAction(period=4.0, actions=[mavros_node])
    ])

