# RoboDog V3 for Jetson Orin Nano + ROS2 Humble

A complete adaptation of the RoboDog V3 repository (From Robolabhub) for Jetson Orin Nano Super 8G with ROS2 Humble and 8bitdo Ultimate controller support.

## System Requirements

- **Hardware**: Jetson Orin Nano Super 8G
- **OS**: Ubuntu 22.04 (L4T, JetPack 6.2)
- **ROS2**: Humble
- **Controller**: 8bitdo Ultimate or compatible gamepad
- **Servos**: LX16a servo motors with USB-to-Serial adapter

## Installation & Setup

### Step 1: Install ROS2 Humble

```bash
# Set locale
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources and keys
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://repo.ros2.org/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://repo.ros2.org/ubuntu/main $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop -y
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 2: Install Dependencies

```bash
# Install required build tools and dependencies
sudo apt install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    libyaml-cpp-dev \
    libasio-dev

# Initialize rosdep
sudo rosdep init
rosdep update
```

### Step 3: Create ROS2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### Step 4: Clone and Build the RoboDog Package

```bash
cd ~/ros2_ws/src
# Your RoboDog_v3_ros2 package should already be here

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build --symlink-install --packages-select robodog
source install/setup.bash
```

### Step 5: Setup 8bitdo Controller

#### Pair the 8bitdo Ultimate Controller

```bash
# Enable Bluetooth
sudo systemctl start bluetooth

# Put controller in pairing mode (hold Home + B buttons for 3 seconds)
bluetoothctl
# In bluetoothctl:
# > scan on
# Wait for "8BitDo Ultimate"
# > pair <MAC_ADDRESS>
# > trust <MAC_ADDRESS>
# > connect <MAC_ADDRESS>
# > quit
```

#### Find Joystick Device

```bash
# List input devices
ls -la /dev/input/

# Test the joystick
sudo cat /dev/input/jsX  # Replace X with the device number

# Or use jstest-gtk (GUI tool)
sudo apt install jstest-gtk
jstest-gtk
```

### Step 6: Identify Serial Port for Servos

```bash
# List serial ports
ls -la /dev/tty*

# If using USB-to-Serial adapter, it will appear as:
# /dev/ttyUSB0, /dev/ttyUSB1, etc. (FTDI, CH340)
# /dev/ttyACM0, /dev/ttyACM1, etc. (CDC)

# Test connection
dmesg | grep tty  # Check kernel messages

# Identify the correct port
lsusb  # List USB devices
```

### Step 7: Configure Serial Port Permissions

```bash
# Add your user to dialout group
sudo usermod -a -G dialout $USER
newgrp dialout

# Verify
groups $USER  # Should include 'dialout'

# Test permission
cat /dev/ttyUSB0
```

## Configuration

### Controller Configuration

Edit `config/controller_config.yaml` to customize:
- Button mappings
- Joystick sensitivity (deadzones)
- Movement parameters
- Gait speeds

Default configuration maps:
- **A button**: Stand mode
- **B button**: Walk mode  
- **X button**: Trot mode
- **Y button**: Gallop mode
- **LB**: Speed down
- **RB**: Speed up
- **Left Stick**: Forward/backward and left/right movement
- **Right Stick**: Rotation

### Serial Port Configuration

If your servo port differs from `/dev/ttyUSB0`, modify `src/Robots/RobotV3.cpp`:

```cpp
// Change this line in CreateRobotV3()
if (!lx16a->open("/dev/ttyUSB0")) {
    // Try /dev/ttyACM0 as fallback
    ...
}
```

Then rebuild:
```bash
cd ~/ros2_ws
colcon build --packages-select robodog
```

## Running RoboDog

### Launch All Components

```bash
# Terminal 1: Source the workspace
cd ~/ros2_ws
source install/setup.bash

# Launch the complete system
ros2 launch robodog robodog_complete.launch.py \
    joy_device:=/dev/input/js0 \
    servo_port:=/dev/ttyUSB0 \
    log_level:=INFO
```

### Launch Remote Controller Only

```bash
ros2 launch robodog remote_controller.launch.py \
    joy_device:=/dev/input/js0 \
    log_level:=INFO
```

### Monitor Joy Input

```bash
# In another terminal, monitor controller inputs
ros2 topic echo /joy

# Expected output from 8bitdo Ultimate:
# buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # 11 buttons
# axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 8 axes
```

### Monitor Robot Hardware

```bash
# Check servo communication
ros2 topic list  # Should show /joy

# Check logs
ros2 run robodog robodog_hw
```

## Troubleshooting

### Joy Node Issues

```bash
# If joy_node crashes or doesn't find controller:
# 1. Verify joystick connection
ls /dev/input/js*

# 2. Check permissions
ls -la /dev/input/js0  # Should be readable

# 3. Try with explicit device ID
ros2 run joy joy_node --ros-args -p device_id:=/dev/input/js0

# 4. Test with joy_test_node (if available)
ros2 run joy joy_test
```

### Serial Port Issues

```bash
# Check if port is accessible
cat /dev/ttyUSB0  # Should not hang

# Check baud rate settings
stty -a < /dev/ttyUSB0

# Use minicom to test connection
sudo apt install minicom
minicom -D /dev/ttyUSB0 -b 115200
# Press Ctrl-A, then Z for help
# Type some bytes to see if servos respond
```

### Servo Communication Issues

```bash
# Enable debug logging
ros2 run robodog robodog_hw --ros-args --log-level DEBUG

# Check which port is being used
sudo dmesg | tail -20

# Try alternate port
# Modify RobotV3.cpp and rebuild
```

### Controller Not Responding

```bash
# Check Bluetooth connection
bluetoothctl
# > devices
# > info <MAC_ADDRESS>

# Reconnect if needed
# > disconnect <MAC_ADDRESS>
# > connect <MAC_ADDRESS>
```

## Architecture Overview

### Nodes

1. **joy_node** (from `joy` package)
   - Reads gamepad input
   - Publishes to `/joy` topic
   - Runs at 50Hz (configurable)

2. **remote_controller** (robodog_remote_controller)
   - Subscribes to `/joy`
   - Maps controller inputs to robot commands
   - Reads configuration from `controller_config.yaml`
   - Publishes movement commands

3. **robodog_hw** (robodog_hw)
   - Hardware interface node
   - Controls servo motors
   - Manages robot kinematics and inverse kinematics
   - Communicates via serial port to LX16a servos

### Topics

- **`/joy`**: Joystick input (published by joy_node)
  - Fields: `buttons` array, `axes` array, `header` with timestamp

### Key Components

- **LX16a.h/cpp**: Serial communication with LX16a servo motors using ROS2 serial_driver
- **RemoteController.h/cpp**: Maps controller input to robot commands (configurable via YAML)
- **Robot.h**: Robot interface abstraction
- **RobotV3.cpp**: RoboDog V3 specific implementation
- **ServoController**: Manages multiple servo motors in parallel threads

## Hardware Connection

### Servo Motor Connection

```
LX16a Servos ←→ USB-to-Serial Adapter ←→ Jetson Orin Nano
                  (e.g., CH340, FTDI)
```

- **Servo Data Line**: TTL serial (typically 5V logic)
- **Power**: Separate 5V-6V power supply
- **Serial Port**: USB adapter on Jetson (usually `/dev/ttyUSB0`)
- **Baud Rate**: 115200 bps

### 8bitdo Controller Connection

- **Pairing**: Bluetooth (built-in on Jetson)
- **Device**: `/dev/input/js0` (or detected by joy_node)
- **Range**: ~10 meters

## Building from Source

```bash
cd ~/ros2_ws
colcon build --packages-select robodog --cmake-args -DCMAKE_BUILD_TYPE=Release
```

For debug builds with verbose output:

```bash
colcon build --packages-select robodog --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

## Performance Notes

- **Update Rate**: Joy node at 50Hz (configurable)
- **Robot Loop**: ServoController runs servo threads for smooth motion
- **Latency**: ~20-50ms from controller to servo (depends on serial port)
- **CPU Usage**: Minimal on Jetson Orin Nano (multi-threaded servo control)

## Calibration

Before first use, calibrate servos:

```bash
# 1. Power on the robot
# 2. Position all legs in neutral pose
# 3. Run calibration node (if available)
#    ros2 run robodog calibrate_servos

# 4. Or manually set offsets in config:
# Modify servo offsets in src/Robots/RobotV3.cpp CreateRobotV3() function
```

## Further Development

### Adding Custom Movements

Extend `RemoteController::joy_callback()` to implement new movement patterns or modify existing ones.

### Telemetry

Extend robot nodes to publish:
- Battery voltage
- Servo temperatures
- Joint angles
- Gait state

### Integration with Nav2

Add additional nodes for:
- ROS2 Navigation Stack (Nav2)
- SLAM mapping
- Object detection

## References

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Joy Package](http://wiki.ros.org/joy)
- [Serial Driver Package](https://github.com/ros-drivers/transport_drivers)
- [Jetson Orin Nano Documentation](https://developer.nvidia.com/embedded/jetson-orin-nano)
- [8bitdo Ultimate Controller](https://www.8bitdo.com/)

## License

MIT License - See LICENSE file

## Original Repository

Based on: [RoboDog V3 GitHub](https://github.com/robolab19/RoboDog_v3_ros2)

## Support

For issues or questions:
1. Check the troubleshooting section
2. Enable DEBUG logging
3. Post issues with detailed logs and hardware information

