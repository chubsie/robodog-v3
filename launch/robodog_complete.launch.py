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
            'device_id': LaunchConfiguration('joy_device'),
            'deadzone': 0.1,
            'autorepeat_rate': 50.0,  # 50Hz
            'coalesce_interval_ms': 10,
        }],
        remappings=[
            ('joy', 'joy'),
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    # Hardware node - controls robot servos
    hw_node = Node(
        package='robodog',
        executable='robodog_hw',
        name='robodog_hw',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        # Set environment variable for servo port
        shell=True,
    )

    # Remote controller node - maps joy input to robot commands
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

    # Build the launch description
    ld = LaunchDescription([
        declare_joy_device,
        declare_servo_port,
        declare_launch_hw,
        declare_launch_remote,
        declare_log_level,
    ])

    # Add joy node (always launch)
    ld.add_action(joy_node)

    # Conditionally add hardware node
    ld.add_action(
        ExecuteProcess(
            cmd=['echo', 'Not launching hardware node'],
            condition=LaunchConfiguration('launch_hw'),
        )
    )
    
    # For now, launch both nodes unconditionally
    # TODO: Make this conditional based on launch arguments
    ld.add_action(hw_node)
    ld.add_action(remote_node)

    return ld

