# Jetson Orin Nano - RoboDog Troubleshooting Guide

## Common Jetson Orin Specific Issues

### 1. Serial Port Permission Denied

**Error**: `Permission denied (errno 13)` when accessing `/dev/ttyUSB0`

**Cause**: User is not in the `dialout` group

**Solution**:
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Apply new group membership
newgrp dialout

# Verify
groups
# Output should include: dialout
```

**Persistent Solution** (survives reboot):
```bash
# Edit /etc/udev/rules.d/99-usb.rules
sudo nano /etc/udev/rules.d/99-usb.rules

# Add these lines:
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", MODE="0666"  # FTDI
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", MODE="0666"  # CH340
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", MODE="0666"  # CP210x

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

---

### 2. Bluetooth Controller Not Detected

**Error**: `No joy devices found`

**Cause**: Bluetooth services not running or controller not paired

**Solution**:
```bash
# Check Bluetooth status
sudo systemctl status bluetooth
# If inactive, start it
sudo systemctl start bluetooth

# Check available devices
hciconfig

# If no devices, check if disabled
sudo rfkill list

# If soft-blocked, unblock
sudo rfkill unblock bluetooth

# Re-pair controller
bluetoothctl
# In bluetoothctl:
> power on
> scan on
# Wait for 8BitDo Ultimate to appear
> pair XX:XX:XX:XX:XX:XX
> trust XX:XX:XX:XX:XX:XX
> connect XX:XX:XX:XX:XX:XX
> quit
```

**Check Joy Device**:
```bash
# List joy devices
ls /dev/input/js*

# If no devices, install calibration tool
sudo apt install jstest-gtk
jstest-gtk

# Or use command line to test
cat /dev/input/js0  # Should output binary data when controller is moved
```

---

### 3. Serial Port Not Found on Jetson

**Error**: `Failed to open /dev/ttyUSB0` and `Failed to open /dev/ttyACM0`

**Cause**: 
- USB adapter not connected
- Wrong port assignment
- Driver not loaded

**Solution**:
```bash
# Check connected USB devices
lsusb
# Look for: "FTDI", "Silicon Labs", "CH340" entries

# Check all serial ports
ls -la /dev/tty* | grep -E "USB|ACM"

# Check kernel messages for USB detection
dmesg | tail -20 | grep -E "ttyUSB|ttyACM"

# If no ports appear, check USB hub
lsusb -v

# Test with minicom
sudo apt install minicom
minicom -D /dev/ttyUSB0 -b 115200
# Send data with Ctrl-A, then X to exit

# If still no connection, check dmesg for errors
dmesg | grep -i "usb\|serial" | tail -30
```

**Dynamic Port Assignment**:
Edit `src/Robots/RobotV3.cpp` to auto-detect:
```cpp
// Try multiple ports in order
std::vector<std::string> ports = {
    "/dev/ttyUSB0", "/dev/ttyUSB1",
    "/dev/ttyACM0", "/dev/ttyACM1",
    "/dev/ttyAMA0"  // Jetson GPIO UART
};

for (const auto& port : ports) {
    if (lx16a->open(port)) {
        RCLCPP_INFO(logger, "Successfully opened servo port: %s", port.c_str());
        break;
    }
}
```

---

### 4. High CPU Usage / Thermal Throttling

**Symptom**: Robot becomes slow, CPU usage > 80%

**Cause**: Jetson thermal management or inefficient servo control

**Solution**:
```bash
# Check CPU frequencies
cat /sys/devices/system/cpu/cpu*/cpufreq/cpuinfo_max_freq

# Monitor temperature
watch -n 1 'cat /sys/devices/virtual/thermal/thermal_zone*/temp | awk "{print $1/1000}"'

# Check if throttling is active
nvidia-smi dmon  # Shows GPU/CPU clock scaling

# Reduce servo update rate in launch file
ros2 launch robodog remote_controller.launch.py log_level:=WARN

# Or reduce joy_node rate
# Edit launch file, change autorepeat_rate: 50.0 → 25.0
```

**Thermal Management**:
```bash
# Improve cooling
# Option 1: Add heatsink
# Option 2: Reduce max frequency (not recommended)
sudo nvpmodel -m 0  # 15W mode (lower performance)
sudo nvpmodel -m 1  # 25W mode (higher performance)

# Check current mode
sudo nvpmodel -q
```

---

### 5. Build Fails with "Package X not found"

**Error**: `By not providing "Findserial_driver.cmake"...`

**Cause**: ROS2 package not installed or sourced incorrectly

**Solution**:
```bash
# Source ROS2 Humble setup
source /opt/ros/humble/setup.bash

# Check if transport_drivers package is in workspace
cd ~/ros2_ws
find . -name "serial_driver" -type d

# If not found, clone it
cd src
git clone https://github.com/ros-drivers/transport_drivers.git

# Check for dependencies
rosdep check --from-paths src --ignore-src

# Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# Rebuild
cd ~/ros2_ws
colcon build --packages-select robodog
```

**Check CMake Cache**:
```bash
# Clear build cache
rm -rf build install log

# Reconfigure with verbose output
colcon build --packages-select robodog --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON 2>&1 | head -100
```

---

### 6. Joy Driver Crashes on Startup

**Error**: `Segmentation fault` in joy_node

**Cause**: Incompatible joy version or device issue

**Solution**:
```bash
# Update joy package
sudo apt update
sudo apt install --only-upgrade ros-humble-joy

# Check joy version
dpkg -l | grep joy

# Test with older device_id format
ros2 run joy joy_node --ros-args -p device_id:=0  # Use device number instead of path

# Run with debug output
ROS_LOG_DIR=/tmp ros2 run joy joy_node --ros-args --log-level DEBUG 2>&1 | tee /tmp/joy.log
```

---

### 7. Jetson Clock Scalings Affecting Serial Communication

**Symptom**: Servo commands work intermittently, servo timeouts

**Cause**: Dynamic frequency scaling affects timing

**Solution**:
```bash
# Lock clocks to maximum (may increase heat)
sudo jetson_clocks

# Check if jetson_clocks is running
sudo /usr/bin/jetson_clocks --show

# Or run with fixed frequency
sudo -i
echo 1500000000 > /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq
```

---

### 8. GPIO UART Issues (if using internal UART)

**Error**: Serial communication over `/dev/ttyAMA0` fails

**Cause**: UART disabled or not configured

**Solution**:
```bash
# Check if UART is available
ls -la /dev/ttyAMA*

# If not found, enable in device tree (may require recompilation)
# Or use USB adapter (recommended)

# For debugging, check serial device driver
dmesg | grep -i uart
```

---

### 9. Memory Pressure / OOM Issues

**Symptom**: Node crashes with "Cannot allocate memory"

**Cause**: Jetson Orin Nano has limited RAM (8GB)

**Solution**:
```bash
# Check memory usage
free -h

# Monitor memory in real-time
watch -n 1 'free -h; echo "---"; top -b -n 1 | head -20'

# Reduce joy_node buffer size
# Edit launch file:
# parameters=[{ 'msg_queue_size': 1, ... }]

# Kill unnecessary background services
sudo systemctl stop snapd  # If using snap
sudo systemctl stop apport  # Error reporting

# Use swap (slower but helps)
sudo fallocate -l 4G /var/swap.img
sudo chmod 600 /var/swap.img
sudo mkswap /var/swap.img
sudo swapon /var/swap.img
```

---

### 10. Servo Temperatures Rising Rapidly

**Symptom**: Servos get hot, then reduce torque

**Cause**: Servo firmware thermal management or excessive load

**Solution**:
```bash
# Check servo temperatures
# Add to RemoteController or create monitoring node:
// uint8_t temp;
// if (lx16a->get_temp(servo_id, temp)) {
//     RCLCPP_WARN_IF(get_logger(), temp > 60, "Servo %d temp: %d°C", servo_id, temp);
// }

# Configure thermal limits
# In LX16a.cpp set_max_temp():
lx16a->set_max_temp(servo_id, 60);  // Set 60°C limit

# Improve servo cooling
# - Add heatsinks
# - Improve air circulation
# - Reduce servo load
# - Lower movement speed in config

# Verify servo voltage
# Low voltage causes heating
uint16_t vin;
if (lx16a->get_vin(servo_id, vin)) {
    RCLCPP_WARN_IF(get_logger(), vin < 5000, "Low servo voltage: %dmV", vin);
}
```

---

## Jetson Orin Nano Specifications Reference

- **CPU**: ARM64 (8-core)
- **GPU**: NVIDIA Ampere (1024 CUDA cores)
- **RAM**: 8GB LPDDR5
- **Storage**: 128GB eMMC or microSD
- **Thermals**: Fanless or active cooling option
- **Power**: 5W-15W typical, up to 25W peak
- **Interfaces**: 
  - 1x USB 3.0, 1x USB 2.0
  - 40-pin GPIO header (includes UART)
  - CSI camera connector (2x)
  - Ethernet (1Gbps)

## Performance Tuning Checklist

- [ ] Disable unnecessary services (snapd, apport, cups)
- [ ] Lock CPU frequencies with `jetson_clocks`
- [ ] Monitor thermal zone: `/sys/devices/virtual/thermal/`
- [ ] Use `-O2` or `-O3` compiler optimizations
- [ ] Profile with `nsys profile` (NVIDIA Systems Tools)
- [ ] Check serial port baud rate: `stty -a < /dev/ttyUSB0`
- [ ] Verify no other services using the serial port
- [ ] Use ROS2 thread priorities: `rcl_task_set_priority()`

## Useful Commands

```bash
# System info
uname -a
cat /etc/nv_tegra_release
cat /proc/cpuinfo

# Real-time monitoring
htop  # Better than top
iotop  # I/O usage
nethogs  # Network usage per process
iftop  # Network bandwidth

# Thermal monitoring
nvidia-smi
nvidia-smi dmon -s 50  # Update every 50ms

# Power usage
ina3221_mon  # If kernel module loaded
cat /sys/devices/virtual/thermal/thermal_zone*/temp

# Serial port test
sudo socat -d -d OPEN:/dev/ttyUSB0,b115200,raw OPEN:STDIO
# Or simpler:
stty -F /dev/ttyUSB0 ispeed 115200 ospeed 115200 cs8 -parenb -cstopb
cat < /dev/ttyUSB0 | od -c

# USB device test
lsusb -v -s BUS:DEV
usbmon  # USB protocol analyzer
```

## Support & Debugging

**Enable Verbose Logging**:
```bash
export ROS_LOG_DIR=/tmp/ros_logs
mkdir -p $ROS_LOG_DIR
ros2 run robodog robodog_remote_controller --ros-args --log-level DEBUG 2>&1 | tee debug.log
```

**Capture Detailed Output**:
```bash
# Save to file with timestamps
ros2 launch robodog remote_controller.launch.py 2>&1 | tee -a robotdog_$(date +%Y%m%d_%H%M%S).log

# Check logs
tail -f robotdog_*.log

# Search for errors
grep -i "error\|fail\|warn" robotdog_*.log
```

**System Diagnostics Bundle**:
```bash
# Create diagnostic report
mkdir -p diagnostics
uname -a > diagnostics/system.txt
cat /proc/cpuinfo >> diagnostics/system.txt
lsusb >> diagnostics/system.txt
lsb_release -a >> diagnostics/system.txt
ros2 pkg list | grep -E "robodog|joy|serial" >> diagnostics/packages.txt
```

