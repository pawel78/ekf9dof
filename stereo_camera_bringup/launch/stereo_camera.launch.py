#!/usr/bin/env python3
"""
Launch file for stereo camera
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    camera_driver_arg = DeclareLaunchArgument(
        'camera_driver',
        default_value='argus',
        description='Camera driver: "argus" or "v4l2"'
    )
    
    sensor_id_left_arg = DeclareLaunchArgument(
        'sensor_id_left',
        default_value='0',
        description='Sensor ID for left camera (Argus)'
    )
    
    sensor_id_right_arg = DeclareLaunchArgument(
        'sensor_id_right',
        default_value='1',
        description='Sensor ID for right camera (Argus)'
    )
    
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='1280',
        description='Image width'
    )
    
    height_arg = DeclareLaunchArgument(
        'height',
        default_value='720',
        description='Image height'
    )
    
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30',
        description='Frame rate'
    )
    
    # Argus stereo node
    argus_node = Node(
        package='stereo_camera_bringup',
        executable='argus_stereo_node',
        name='argus_stereo_node',
        parameters=[{
            'sensor_id_left': LaunchConfiguration('sensor_id_left'),
            'sensor_id_right': LaunchConfiguration('sensor_id_right'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'fps': LaunchConfiguration('fps'),
        }],
        output='screen'
    )
    
    # V4L2 stereo node
    v4l2_node = Node(
        package='stereo_camera_bringup',
        executable='v4l2_stereo_node',
        name='v4l2_stereo_node',
        parameters=[{
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'fps': LaunchConfiguration('fps'),
            'auto_detect': True,
        }],
        output='screen'
    )
    
    return LaunchDescription([
        camera_driver_arg,
        sensor_id_left_arg,
        sensor_id_right_arg,
        width_arg,
        height_arg,
        fps_arg,
        # Note: Use only one of these based on camera_driver parameter
        # In practice, you'd use conditional logic here
        argus_node,
    ])
