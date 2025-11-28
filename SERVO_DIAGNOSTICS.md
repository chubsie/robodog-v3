# Servo Controller Hz Diagnostic Tool

This tool measures the actual update frequency (Hz) of the servo controller by repeatedly querying motor positions and analyzing the timing.

## Overview

The servo controller runs in separate threads for each servo group, with each thread executing a control loop. This diagnostic tool:

1. Connects to the servos via serial port
2. Repeatedly reads the current position from a specific servo
3. Measures the time between consecutive reads
4. Calculates statistics including:
   - **Average cycle time** (in milliseconds)
   - **Frequency (Hz)** - the main metric you want
   - **Min/Max cycle times**
   - **Standard deviation** (shows consistency)

## Usage

### Basic Usage

```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
source install/setup.bash

# Run the diagnostic tool
ros2 run robodog robodog_servo_diagnostics
```

### With Custom Parameters

```bash
# Measure for 30 seconds instead of default 10
ros2 run robodog robodog_servo_diagnostics --ros-args -p measurement_duration:=30.0

# Monitor servo 5 instead of servo 1
ros2 run robodog robodog_servo_diagnostics --ros-args -p servo_id:=5

# Use a different serial port
ros2 run robodog robodog_servo_diagnostics --ros-args -p port:=/dev/ttyACM0

# Combine multiple parameters
ros2 run robodog robodog_servo_diagnostics --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p servo_id:=1 \
  -p measurement_duration:=20.0
```

## First: Scan for Connected Servos

**Before using the Hz diagnostic, first verify which servos are connected:**

```bash
# Find all connected servo IDs
ros2 run robodog robodog_servo_scanner

# This will show which IDs 1-20 are responding
# If NONE respond, see SERVO_NOT_RESPONDING.md for troubleshooting
```

Example output:
```
ID | Position | Temp | Voltage | Status
---|----------|------|---------|-------
 1 |     1024 |  28C |  6.12V  | ✓ OK
 2 |      950 |  29C |  6.11V  | ✓ OK
 3 |    N/A   |  N/A |   N/A   | ✗ No response
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `port` | string | `/dev/ttyUSB0` | Serial port where servos are connected |
| `baud_rate` | int | `115200` | Serial communication baud rate |
| `measurement_duration` | float | `10.0` | How long to measure (in seconds) |
| `servo_id` | int | `1` | Which servo to query for frequency testing |

## Example Output

```
[INFO] [servo_controller_diagnostics]: Servo Controller Hz Diagnostic Tool Started
[INFO] [servo_controller_diagnostics]: Configuration:
[INFO] [servo_controller_diagnostics]:   Port: /dev/ttyUSB0
[INFO] [servo_controller_diagnostics]:   Baud Rate: 115200
[INFO] [servo_controller_diagnostics]:   Measurement Duration: 10.0 seconds
[INFO] [servo_controller_diagnostics]:   Monitoring Servo ID: 1
[INFO] [servo_controller_diagnostics]: Serial port opened successfully

[INFO] [servo_controller_diagnostics]: ========== DIAGNOSTIC STARTED ==========
[INFO] [servo_controller_diagnostics]: Querying servo 1 position repeatedly for 10.0 seconds...

[INFO] [servo_controller_diagnostics]: Queries: 100, Last Cycle: 1.234 ms, Position: 1024
[INFO] [servo_controller_diagnostics]: Queries: 200, Last Cycle: 1.198 ms, Position: 1024
...

[INFO] [servo_controller_diagnostics]: 
========== DIAGNOSTIC RESULTS ==========
[INFO] [servo_controller_diagnostics]: Total Measurement Time: 10.001 seconds
[INFO] [servo_controller_diagnostics]: Total Queries: 1245
[INFO] [servo_controller_diagnostics]: Valid Measurements: 1244

[INFO] [servo_controller_diagnostics]: Cycle Time Statistics:
[INFO] [servo_controller_diagnostics]:   Average: 8.034 ms
[INFO] [servo_controller_diagnostics]:   Min:     2.145 ms
[INFO] [servo_controller_diagnostics]:   Max:    15.234 ms
[INFO] [servo_controller_diagnostics]:   Std Dev: 2.145 ms

[INFO] [servo_controller_diagnostics]: SERVO CONTROLLER FREQUENCY:
[INFO] [servo_controller_diagnostics]:   *** 124.47 Hz ***

[INFO] [servo_controller_diagnostics]: Position Range: 1020 to 1028
[INFO] [servo_controller_diagnostics]: ========================================
```

## Interpreting Results

### Frequency (Hz)

- **Expected Range**: 50-200+ Hz depending on:
  - Number of servos in the group
  - Serial port speed (115200 bps)
  - System load on Jetson Orin Nano

### Example Analysis

**High Frequency (150+ Hz) = Good**
- Fast communication
- Responsive servo control
- Low latency

**Medium Frequency (50-100 Hz) = Acceptable**
- Still provides smooth control
- Typical for groups with 4+ servos

**Low Frequency (<50 Hz) = Potential Issues**
- May indicate:
  - Serial port communication problems
  - High system load
  - Servo connection issues

### Standard Deviation

- **Low Std Dev (<1 ms)** = Consistent, predictable timing ✅
- **High Std Dev (>5 ms)** = Variable timing, may indicate:
  - System load spikes
  - Other processes interfering
  - Serial port issues

## Troubleshooting

### Serial Port Not Found
```
[ERROR] [servo_controller_diagnostics]: Failed to open serial port: /dev/ttyUSB0
```

**Solution**: Check serial connections and correct port:
```bash
ls /dev/tty* | grep -E "USB|ACM"
```

### No Data Collected
```
[ERROR] [servo_controller_diagnostics]: No data collected! Check servo connection.
```

**Solution**: 
- This means NO servos are responding on the serial bus
- First, use the servo scanner to diagnose which servos are connected:
  
  ```bash
  # Scan for connected servos
  ros2 run robodog robodog_servo_scanner
  
  # If no servos found, see SERVO_NOT_RESPONDING.md for full troubleshooting
  ```

- If scanner shows servos, but diagnostics still fails:
  - Verify servo ID is correct (use scanner to find actual IDs)
  - Check servo power supply
  - Test with a different servo ID

### Very Low Frequency
- Increase measurement duration to smooth out noise
- Check for USB hub interference (try direct connection)
- Monitor system load: `top` or `htop`

## Technical Details

### How It Works

The servo controller's `ThreadBody()` runs in a loop:
```cpp
while (!exit) {
    for (each servo) {
        servo->do_cycle(lazyUpdate);  // Updates position, temperature
    }
    usleep(1);  // Minimal sleep to prevent CPU spinning
}
```

Each `do_cycle()` call may query the servo for:
- Current position
- Temperature (periodically, with lazy update)
- Move commands (if pending)

The diagnostic tool measures the time between consecutive position queries to determine **actual operational frequency**.

### Why Measure Hz?

1. **Performance Validation**: Confirms your servo controller meets frequency requirements
2. **Troubleshooting**: Detects communication bottlenecks
3. **System Tuning**: Helps optimize for your Jetson hardware
4. **Motion Quality**: Higher frequency = smoother, more responsive motion

## Tips for Optimization

1. **Reduce Servo Count per Group**: Split into multiple threads if frequency is too low
2. **Check System Load**: Run `top` simultaneously to monitor CPU usage
3. **Verify Serial Connection**: Use a high-quality USB cable, avoid hubs
4. **Monitor Temperature**: High temps can reduce servo responsiveness

## Integration with Motion Control

The measured Hz directly affects:
- **Movement Smoothness**: Higher Hz = smoother motion
- **Response Time**: How quickly servos react to commands
- **Precision**: Ability to maintain accurate positions
- **Stability**: Control loop stability for complex gaits
