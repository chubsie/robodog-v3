# RoboDog V3 - Quick Reference Card

## Hardware Information

**Jetson Orin Nano Super 8G**
- CPU: 8-core ARM64 @ up to 2.1GHz
- RAM: 8GB LPDDR5
- OS: Ubuntu 22.04 (L4T, JetPack 6.2)
- Default User: ubuntu

**Connectivity**
- Bluetooth: Built-in (for 8bitdo controller)
- USB: 1x USB 3.0, 1x USB 2.0 (for servo adapter)
- Serial: 40-pin GPIO header with UART

**Servo Controller**
- Servos: LX16a
- Communication: TTL Serial @ 115200 bps
- Adapter: USB-to-Serial (FTDI, CH340, or CDC)
- Port: `/dev/ttyUSB0` or `/dev/ttyACM0`

**Controller**
- Device: 8bitdo Ultimate
- Interface: Bluetooth
- Device Path: `/dev/input/js0` (or auto-detected)
- Buttons: 11 (indexed 0-10)
- Axes: 8 (analog sticks, triggers, D-pad)

## First-Time Setup (in order)

```
1. Source ROS2          → source /opt/ros/humble/setup.bash
2. Build RoboDog        → colcon build --packages-select robodog
3. Add to dialout       → sudo usermod -a -G dialout $USER && newgrp dialout
4. Find joystick        → ls /dev/input/js*
5. Pair controller      → bluetoothctl (Home+B button 3 sec)
6. Find servo port      → ls /dev/tty{USB,ACM}*
7. Test joy             → ros2 topic echo /joy
8. Test robot           → ros2 launch robodog remote_controller.launch.py
```

## File Locations

```
~/ros2_ws/src/RoboDog_v3_ros2/
├── include/Servos/LX16a.h           ← Serial communication
├── src/Servos/LX16a.cpp             ← Implementation
├── src/RemoteControl/RemoteController.cpp  ← Controller mapping
├── config/controller_config.yaml    ← Controller config
├── launch/remote_controller.launch.py
├── SETUP_GUIDE.md                   ← Detailed setup
├── QUICK_START.md                   ← 5-min guide
├── MIGRATION_GUIDE.md               ← Technical details
└── JETSON_TROUBLESHOOTING.md        ← Jetson issues
```

## Essential Commands

### Setup & Build
```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select robodog --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### Find Hardware
```bash
ls /dev/input/js*              # Joystick device
ls /dev/ttyUSB* /dev/ttyACM*   # Serial port
bluetoothctl devices           # Paired Bluetooth devices
```

### Launch
```bash
# Remote controller + joy
ros2 launch robodog remote_controller.launch.py

# With custom device paths
ros2 launch robodog remote_controller.launch.py \
    joy_device:=/dev/input/js0

# Full system (with hardware)
ros2 launch robodog robodog_complete.launch.py \
    servo_port:=/dev/ttyUSB0
```

### Debugging
```bash
ros2 topic echo /joy                    # Monitor controller input
ros2 topic list                         # List active topics
ros2 node list                          # List running nodes
ros2 run robodog robodog_remote_controller --ros-args --log-level DEBUG
```

## 8bitdo Button Mapping (Default)

```
          Y (Gait 3: Gallop)
          │
    LB ─ X (Gait 2: Trot)  ─ RB
    (4)  (2)              (5)
    
           A (Gait 0: Stand)
           B (Gait 1: Walk)
           │
   Back            Start
   (6)             (7)

[L-Stick]              [R-Stick]
Forward/              Rotation
Backward/
Strafe
```

## Configuration Tuning

### Edit Controller Mapping
```yaml
# config/controller_config.yaml
buttons:
  stand: 0              # Button A
  walk: 1               # Button B
  trot: 2               # Button X
  gallop: 3             # Button Y
  speed_up: 5           # RB
  speed_down: 4         # LB

sticks:
  left_deadzone: 0.15   # Adjust if drift occurs
  right_deadzone: 0.15
```

### Adjust Servo Port
```cpp
// src/Robots/RobotV3.cpp, around line 145
if (!lx16a->open("/dev/ttyUSB0")) {
    RCLCPP_WARN(..., "Trying /dev/ttyACM0");
    if (!lx16a->open("/dev/ttyACM0")) {
        return false;
    }
}
```

## Common Issues & Fixes

| Problem | Fix |
|---------|-----|
| Permission denied /dev/tty | `sudo usermod -a -G dialout $USER && newgrp dialout` |
| Joy not found | `bluetoothctl` → reconnect controller |
| Serial port not found | Check `lsusb`, try `/dev/ttyACM0` |
| Build fails (serial_driver) | `cd ~/ros2_ws && rosdep install --from-paths src -r -y` |
| Slow response | Reduce log level: `--log-level WARN` |
| High CPU | Run `sudo jetson_clocks` to lock frequencies |
| Servo timeout | Check baud rate: `stty -a < /dev/ttyUSB0` |

## System Status Checks

```bash
# Build status
cd ~/ros2_ws && colcon build --packages-select robodog --allow-overriding

# Configuration found
ros2 run robodog robodog_remote_controller --ros-args --log-level DEBUG 2>&1 | grep config

# Joy device active
ros2 topic echo /joy --once

# Node running
ros2 node info /remote_controller

# Logs
cat ~/.ros/log/*/*.log | tail -50
```

## Keyboard Shortcuts (during runtime)

```
Ctrl+C              Stop current node
Ctrl+Z              Suspend (background)
Ctrl+L              Clear screen (in terminal)
Ctrl+R              Reverse search (command history)
```

## File Editing

```bash
# Edit config (with syntax highlighting)
nano config/controller_config.yaml
vim config/controller_config.yaml

# Edit source code
nano src/RemoteControl/RemoteController.cpp
gedit src/RemoteControl/RemoteController.cpp &

# View docs
cat QUICK_START.md | less
```

## Performance Monitoring

```bash
# CPU/Memory (real-time)
htop

# Temperature (Jetson)
nvidia-smi
watch -n 1 'nvidia-smi'

# Network/USB
lsusb -v

# System load
uptime
cat /proc/loadavg
```

## Documentation Quick Links

- **New User**: Start with QUICK_START.md
- **Detailed Setup**: SETUP_GUIDE.md
- **Code Changes**: MIGRATION_GUIDE.md  
- **Jetson Issues**: JETSON_TROUBLESHOOTING.md
- **Technical Ref**: MIGRATION_GUIDE.md (All changes explained)

## Useful ROS2 Commands

```bash
# Package management
ros2 pkg list | grep robodog
ros2 pkg prefix robodog
ros2 pkg executables robodog

# Topic inspection
ros2 topic list
ros2 topic echo <topic_name>
ros2 topic info <topic_name>
ros2 topic pub <topic_name> <type> <data>

# Node inspection
ros2 node list
ros2 node info <node_name>
ros2 run <package> <executable>

# Logging
ros2 launch <package> <launch_file>
ROS_LOG_DIR=/tmp/ros_logs ros2 run ...
```

## Emergency Stop

```bash
# Immediate
Ctrl+C (in terminal)

# Kill all nodes
pkill -f "ros2 run robodog"
pkill -f "ros2 launch"

# Kill Bluetooth connection
bluetoothctl
> disconnect
> quit

# Power down servo controller
# (Cut power to servo power supply)
```

## Power Management

```bash
# Check power mode
sudo nvpmodel -q

# Switch to 15W (lower performance)
sudo nvpmodel -m 0

# Switch to 25W (higher performance)
sudo nvpmodel -m 1

# Lock GPU frequency
sudo jetson_clocks

# Show clocks
sudo jetson_clocks --show
```

## Network Debugging

```bash
# Check IP
ip addr
hostname -I

# SSH connection (if available)
ssh ubuntu@<jetson_ip>

# File transfer (SCP)
scp -r ubuntu@<jetson_ip>:/home/ubuntu/ros2_ws/install . .
```

## Version Info

```bash
# ROS2 version
ros2 --version

# ROS2 distribution
echo $ROS_DISTRO

# Python version
python3 --version

# Jetson info
cat /etc/nv_tegra_release
```

## Default Credentials (if not changed)

```
User: ubuntu
Home: /home/ubuntu
Default password: ubuntu (usually)
```

## Cleanup & Maintenance

```bash
# Remove build artifacts
cd ~/ros2_ws
rm -rf build install log

# Clear ROS logs
rm -rf ~/.ros/log/*

# Clear temporary files
rm -f /tmp/*.log
rm -rf /tmp/ros_logs

# Check disk usage
du -sh ~/ros2_ws
du -sh ~/.ros

# Rebuild from scratch
cd ~/ros2_ws
colcon build --packages-select robodog --no-cache
```

## Links & Resources

- ROS2 Humble Docs: https://docs.ros.org/en/humble/
- Joy Package: http://wiki.ros.org/joy
- Serial Driver: https://github.com/ros-drivers/transport_drivers
- Jetson Docs: https://developer.nvidia.com/embedded/jetson-orin-nano
- 8bitdo Controller: https://www.8bitdo.com/

---

**Last Updated**: November 2024
**Version**: RoboDog V3 for Jetson Orin Nano (ROS2 Humble)

Print this card and keep it by your development station for quick reference!

