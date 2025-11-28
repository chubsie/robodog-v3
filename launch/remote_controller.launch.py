#!/usr/bin/env python3
#
# RoboDog Remote Controller launch file
# Launches Joy driver and remote controller node
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """
    Generate launch description for remote controller (joystick control).
    """
    
    # Get the package share directory
    pkg_share = FindPackageShare(package='robodog').find('robodog')
    config_file = os.path.join(pkg_share, 'config', 'controller_config.yaml')
    
    # Declare launch arguments
    declare_joy_device = DeclareLaunchArgument(
        'joy_device',
        default_value='/dev/input/js0',
        description='Path to joystick device'
    )
    
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='Log level (DEBUG, INFO, WARN, ERROR)'
    )

    # Joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': LaunchConfiguration('joy_device'),
            'deadzone': 0.1,
            'autorepeat_rate': 50.0,
            'coalesce_interval_ms': 10,
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    # Remote controller node
    remote_node = Node(
        package='robodog',
        executable='robodog_remote_controller',
        name='remote_controller',
        output='screen',
        parameters=[{
            'config_file': config_file,
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    ld = LaunchDescription([
        declare_joy_device,
        declare_log_level,
        joy_node,
        remote_node,
    ])

    return ld

