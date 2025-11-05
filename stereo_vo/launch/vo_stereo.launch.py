#!/usr/bin/env python3
"""
Launch file for stereo visual odometry system
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    camera_driver_arg = DeclareLaunchArgument(
        'camera_driver',
        default_value='argus',
        description='Camera driver to use: "argus" or "v4l2"'
    )
    
    use_imu_arg = DeclareLaunchArgument(
        'use_imu',
        default_value='false',
        description='Whether to use IMU for VIO'
    )
    
    publish_debug_images_arg = DeclareLaunchArgument(
        'publish_debug_images',
        default_value='true',
        description='Whether to publish debug visualization images'
    )
    
    # Get package paths
    stereo_vo_share = get_package_share_directory('stereo_vo')
    params_file = os.path.join(stereo_vo_share, 'config', 'params.yaml')
    
    # Camera node (Argus)
    argus_camera_node = Node(
        package='stereo_camera_bringup',
        executable='argus_stereo_node',
        name='argus_stereo_node',
        condition=IfCondition(
            ['camera_driver == "argus"']
        ),
        parameters=[params_file],
        output='screen'
    )
    
    # Camera node (V4L2)
    v4l2_camera_node = Node(
        package='stereo_camera_bringup',
        executable='v4l2_stereo_node',
        name='v4l2_stereo_node',
        condition=IfCondition(
            ['camera_driver == "v4l2"']
        ),
        parameters=[params_file],
        output='screen'
    )
    
    # VO node
    vo_node = Node(
        package='stereo_vo',
        executable='vo_node',
        name='vo_node',
        parameters=[
            params_file,
            {'use_imu': LaunchConfiguration('use_imu')},
            {'publish_debug_images': LaunchConfiguration('publish_debug_images')}
        ],
        output='screen'
    )
    
    # IMU node (conditional)
    imu_node = Node(
        package='lsm9ds0_imu',
        executable='lsm9ds0_node',
        name='lsm9ds0_node',
        condition=IfCondition(LaunchConfiguration('use_imu')),
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('lsm9ds0_imu'),
                'config',
                'imu.yaml'
            ])
        ],
        output='screen'
    )
    
    return LaunchDescription([
        camera_driver_arg,
        use_imu_arg,
        publish_debug_images_arg,
        argus_camera_node,
        v4l2_camera_node,
        vo_node,
        imu_node,
    ])
