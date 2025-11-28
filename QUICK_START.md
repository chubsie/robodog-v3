# RoboDog V3 Quick Start Guide

## Quick Setup (5 minutes)

### 1. Build Package

```bash
cd ~/ros2_ws
colcon build --packages-select robodog
source install/setup.bash
```

### 2. Find Your Devices

```bash
# Joystick device
ls /dev/input/js*

# Servo serial port
ls /dev/ttyUSB* /dev/ttyACM*

# Add permissions if needed
sudo usermod -a -G dialout $USER
newgrp dialout
```

### 3. Update Configuration (Optional)

Edit `src/RoboDog_v3_ros2/config/controller_config.yaml` if needed.

### 4. Launch

```bash
# Terminal 1: Start the remote controller
ros2 launch robodog remote_controller.launch.py

# Terminal 2: Monitor joy input
ros2 topic echo /joy
```

## Test Checklist

- [ ] Build succeeds: `colcon build --packages-select robodog`
- [ ] Joy node starts: `ros2 run joy joy_node`
- [ ] Controller detected: `ros2 topic echo /joy` shows values
- [ ] Remote controller starts: `ros2 run robodog robodog_remote_controller`
- [ ] Servos respond: No errors in log, servos move when commanded
- [ ] All gaits work: A/B/X/Y buttons switch modes

## Common Issues & Fixes

### "permission denied" on /dev/ttyUSB0
```bash
sudo usermod -a -G dialout $USER
newgrp dialout
```

### Joy node doesn't find controller
```bash
# Check if controller is paired
bluetoothctl
# List your devices and reconnect if needed
```

### "Could not load controller config"
```bash
# Ensure config file exists
ls src/RoboDog_v3_ros2/config/controller_config.yaml
# Or create a symlink in current directory
ln -s ~/ros2_ws/src/RoboDog_v3_ros2/config/controller_config.yaml ~/.ros/
```

### Build fails with serial_driver not found
```bash
# Ensure transport_drivers is in your workspace
cd ~/ros2_ws/src
ls transport_drivers/

# If not present, clone it
git clone https://github.com/ros-drivers/transport_drivers.git

# Rebuild
cd ~/ros2_ws
colcon build --packages-select robodog
```

## Next Steps

- See SETUP_GUIDE.md for detailed setup
- Check config/controller_config.yaml for button mapping
- Edit src/RemoteControl/RemoteController.cpp to add custom movements
- Calibrate servo offsets in src/Robots/RobotV3.cpp

## Commands Reference

```bash
# Build
colcon build --packages-select robodog

# Run remote controller with debug logging
ros2 run robodog robodog_remote_controller --ros-args --log-level DEBUG

# Monitor topics
ros2 topic echo /joy

# List running nodes
ros2 node list

# Check package info
ros2 pkg list | grep robodog

# View package contents
ros2 pkg prefix robodog
```

