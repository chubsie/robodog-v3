# RPLidar A2M12 Setup Guide

## Overview

This document describes how the RPLidar A2M12 was integrated into the RoboDog ROS2 project.

## Hardware Information

| Property | Value |
|----------|-------|
| Model | RPLidar A2M12 |
| Serial Number | B395FA89C7E19ECABCE499F038425672 |
| Firmware Version | 1.32 |
| Hardware Revision | 6 |
| USB Chip | Silicon Labs CP210x UART Bridge |
| USB Vendor ID | 10c4 |
| USB Product ID | ea60 |
| Serial Port | `/dev/ttyUSB1` (symlinked to `/dev/rplidar`) |

## Scan Specifications

- **Scan Mode**: Sensitivity
- **Sample Rate**: 16 kHz
- **Max Distance**: 16.0 m
- **Scan Frequency**: 10 Hz

## Installation Steps

### 1. Clone the sllidar_ros2 Package

The official Slamtec ROS2 driver was cloned into the workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
```

> **Note**: Do NOT use `rplidar_ros` - that is the old ROS1 driver which uses catkin. The correct ROS2 driver is `sllidar_ros2`.

### 2. Build the Package

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select sllidar_ros2 --symlink-install
```

### 3. Install udev Rules

The udev rules create a `/dev/rplidar` symlink for consistent device naming:

```bash
sudo cp ~/ros2_ws/src/sllidar_ros2/scripts/rplidar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

After this, the device appears as `/dev/rplidar` pointing to the actual ttyUSB device.

### 4. Verify Installation

Check the symlink was created:
```bash
ls -la /dev/rplidar
# Should show: /dev/rplidar -> ttyUSB1
```

## Usage

### Launch the RPLidar Node

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch sllidar_ros2 sllidar_a2m12_launch.py serial_port:=/dev/ttyUSB1
```

Or using the symlink:
```bash
ros2 launch sllidar_ros2 sllidar_a2m12_launch.py serial_port:=/dev/rplidar
```

### Published Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | 2D laser scan data |

### Check Data is Being Published

```bash
ros2 topic echo /scan --once
ros2 topic hz /scan
```

## Integration with RoboDog Launch Files

To include the RPLidar in your existing launch files, add:

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_a2m12_launch.py')
        ]),
        launch_arguments={
            'serial_port': '/dev/rplidar',
            'frame_id': 'lidar_link',
        }.items()
    )
    
    return LaunchDescription([
        sllidar_launch,
        # ... other nodes
    ])
```

## Available Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `serial_port` | `/dev/ttyUSB0` | Serial port device |
| `serial_baudrate` | `256000` | Baud rate (A2M12 uses 256000) |
| `frame_id` | `laser` | TF frame ID for the laser |
| `inverted` | `false` | Invert scan direction |
| `angle_compensate` | `true` | Enable angle compensation |
| `scan_mode` | (empty) | Scan mode (leave empty for default) |

## Troubleshooting

### Permission Denied on Serial Port

Ensure your user is in the `dialout` group:
```bash
sudo usermod -aG dialout $USER
# Log out and back in for changes to take effect
```

### Operation Timeout Error

1. Check the device is connected: `ls /dev/ttyUSB*`
2. Ensure correct serial port is specified
3. Try unplugging and reconnecting the USB cable
4. Check the motor is spinning (you should hear it)

### Device Not Found

If `/dev/rplidar` doesn't exist after udev rules:
```bash
# Reload rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Or unplug and replug the USB cable
```

## Workspace Structure

```
ros2_ws/src/
├── RoboDog_v3_ros2/      # Main robot package
├── sllidar_ros2/         # RPLidar A2M12 driver
└── transport_drivers/    # Serial communication dependency
```

## References

- [sllidar_ros2 GitHub Repository](https://github.com/Slamtec/sllidar_ros2)
- [RPLidar A2M12 Datasheet](https://www.slamtec.com/en/Lidar/A2)
