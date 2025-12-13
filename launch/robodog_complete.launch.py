#!/usr/bin/env python3
#
# Complete RoboDog V3 launch file for Jetson Orin Nano + ROS2 Humble
# Launches all necessary nodes: Joy driver, remote controller, and robot HW
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """
    Generate launch description for complete RoboDog system.
    """
    
    # Get the package share directory
    pkg_share = FindPackageShare(package='robodog').find('robodog')
    
    # Define default paths
    config_file = os.path.join(pkg_share, 'config', 'controller_config.yaml')
    
    # Declare launch arguments
    declare_joy_device = DeclareLaunchArgument(
        'joy_device',
        default_value='/dev/input/js0',
        description='Path to joystick device'
    )
    
    declare_servo_port = DeclareLaunchArgument(
        'servo_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for servo controller (e.g., /dev/ttyUSB0 or /dev/ttyACM0)'
    )
    
    declare_launch_hw = DeclareLaunchArgument(
        'launch_hw',
        default_value='true',
        description='Whether to launch the hardware node'
    )
    
    declare_launch_remote = DeclareLaunchArgument(
        'launch_remote',
        default_value='true',
        description='Whether to launch the remote controller node'
    )
    
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='Log level (DEBUG, INFO, WARN, ERROR)'
    )

    # Joy node - reads from gamepad/joystick
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.1,
            'autorepeat_rate': 50.0,  # 50Hz
            'coalesce_interval_ms': 10,
        }],
        remappings=[
            ('joy', 'joy'),
        ],
        output='screen',
    )

    # Remote controller with IK - handles robot creation internally
    # This is the proper IK-based controller, not the raw servo controller
    remote_ik_node = Node(
        package='robodog',
        executable='robodog_remote_controller',
        name='remote_controller',
        output='screen',
    )

    # Build the launch description
    ld = LaunchDescription()

    # Add joy node (always launch)
    ld.add_action(joy_node)
    
    # Launch IK-based remote controller (no separate hw node needed)
    ld.add_action(remote_ik_node)

    return ld

