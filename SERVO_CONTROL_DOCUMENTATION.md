# RoboDog v3 - Servo Control System Documentation

**Date:** 2025-11-29  
**Status:** ✅ **WORKING - Real-time servo control via Joy controller**

## System Overview

The RoboDog servo control system enables real-time control of 20 LX16a servo motors via a Nintendo Pro Controller over Bluetooth.

### Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│  Nintendo Pro Controller (Bluetooth)                             │
│  16 Buttons + 6 Axes                                             │
└────────────────────────┬────────────────────────────────────────┘
                         │
                         ↓
┌─────────────────────────────────────────────────────────────────┐
│  ros2 joy_node                                                   │
│  Converts Bluetooth input → /joy topic (160-180 Hz)             │
└────────────────────────┬────────────────────────────────────────┘
                         │
                         ↓
┌─────────────────────────────────────────────────────────────────┐
│  robodog_remote_controller (ROS2 Node)                          │
│  Subscribes to /joy, maps input to servo commands               │
└────────────────────────┬────────────────────────────────────────┘
                         │
                         ↓
┌─────────────────────────────────────────────────────────────────┐
│  LX16a Serial Driver                                             │
│  Communicates with servos via TTL serial (115200 baud)          │
└────────────────────────┬────────────────────────────────────────┘
                         │
                         ↓
┌─────────────────────────────────────────────────────────────────┐
│  20x LX16a Servo Motors (TTL Serial Bus)                        │
│  All responding and under control                                │
└─────────────────────────────────────────────────────────────────┘
```

## Controller Input Mapping

### Joy Controller Axes
```
axes[0]: Left Stick X  (horizontal, -1.0 to 1.0)
axes[1]: Left Stick Y  (vertical,   -1.0 to 1.0)
axes[2]: Right Stick X (horizontal, -1.0 to 1.0)
axes[3]: Right Stick Y (vertical,   -1.0 to 1.0)
axes[4]: D-Pad X       (horizontal, -1.0 to 1.0)
axes[5]: D-Pad Y       (vertical,   -1.0 to 1.0)
```

### Servo Control Mapping (Current)
```
Left Stick Y  → Servo 1 (vertical movement)
Left Stick X  → Servo 2 (horizontal movement)
Right Stick Y → Servo 3 (vertical movement)
Right Stick X → Servo 4 (horizontal movement)
D-Pad Y       → Servo 5 (vertical movement)
D-Pad X       → Servo 6 (horizontal movement)
```

### Button Control
```
Button Minus  (-) : Toggle servo control ON/OFF
Button Plus   (+) : Speed down (reduce move time)
Button R      : Speed up (increase move time)
Button L_Stick: Speed reset to 100%
```

### Analog-to-Servo Mapping
```
Analog value: -1.0 to 1.0
Servo position: 0 to 1000

Formula: position = 500 + (analog_value × 500)

Examples:
  -1.0 → 0 (minimum)
  -0.5 → 250
   0.0 → 500 (center)
  +0.5 → 750
  +1.0 → 1000 (maximum)
```

## Performance Characteristics

### Joy Controller Publishing
- **Frequency:** 160-180 Hz
- **Latency:** 5.6-6.2 ms per message

### Servo Controller
- **Individual servo read time:** 60.36 ms (16.6 Hz per servo)
- **Full 20-servo scan:** 1147 ms (0.87 Hz per complete cycle)
- **Send delay:** 10 ms (servo processing time)
- **Read timeout:** 150 ms (Jetson optimization)

### Effective Control Rate
- **Joystick input frequency:** 160-180 Hz
- **Servo update rate:** Limited by 60ms per servo read
- **Downsampling ratio:** ~10:1 (controller publishes 10-18× faster than servo updates)

**Note:** This is the tradeoff for real-time control:
- Controller input is captured at 160-180 Hz (excellent responsiveness)
- Servo updates happen at ~16.6 Hz per servo (sufficient for smooth movement)
- System automatically buffers input during servo read cycles

## Implementation Details

### Files Modified

1. **`include/RemoteController.h`**
   - Added `#include "Servos/LX16a.h"` and `#include "io_context/io_context.hpp"`
   - Added servo controller member: `std::shared_ptr<LX16a> m_servo_controller`
   - Added servo state tracking and movement parameters
   - Added methods: `init_servo_controller()`, `move_servo()`

2. **`src/RemoteControl/RemoteController.cpp`**
   - Updated constructor to initialize servo controller before Joy subscription
   - Implemented `init_servo_controller()` to create IoContext and open serial port
   - Rewrote `joy_callback()` to:
     - Process all 14 active buttons with correct indices
     - Apply deadzone filtering to analog sticks
     - Map controller input to servo position commands
     - Handle button events for servo control toggles
   - Implemented `move_servo()` to:
     - Map analog value (-1.0 to 1.0) to servo position (0-1000)
     - Apply speed multiplier to move times
     - Send commands via LX16a.move() method

3. **`src/robodog_joy_node.cpp`**
   - Unchanged - continues to spin RemoteController node

### Key Classes & Methods

#### LX16a Serial Interface
```cpp
bool LX16a::open(const std::string port);           // Opens serial port
bool LX16a::move(uint8_t id, uint16_t angle, uint16_t time);
bool LX16a::get_pos(uint8_t id, int16_t& pos);    // Read servo position
bool LX16a::get_temp(uint8_t id, uint8_t& temp);  // Read servo temp
```

#### RemoteController Methods
```cpp
void init_servo_controller();                       // Initialize LX16a
void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
void move_servo(int servo_id, float analog_value); // Map input to servo
```

## Testing & Verification

### Test Script: `test_servo_movement.py`
```bash
cd ~/ros2_ws
source install/setup.bash
python3 test_servo_movement.py
```

**Test Sequence:**
1. Servo 1 UP (left stick Y = 1.0)
2. Servo 1 DOWN (left stick Y = -1.0)
3. Servo 2 RIGHT (left stick X = 1.0)
4. Servo 3 UP (right stick Y = 1.0)
5. All servos CENTERED (reset to 0.0)

**Verification:**
```bash
ros2 run robodog robodog_servo_scanner
```
Compare servo positions before/after test.

## Running the System

### Step 1: Start Joy Node
```bash
source ~/ros2_ws/install/setup.bash
ros2 run joy joy_node --ros-args -p dev:=/dev/input/js0
```

### Step 2: Start Remote Controller
```bash
ros2 run robodog robodog_remote_controller
```

### Step 3: Test Movement
- Move left stick up/down → servo 1 moves
- Move left stick left/right → servo 2 moves
- Move right stick up/down → servo 3 moves
- Move right stick left/right → servo 4 moves
- Press Minus (-) to toggle control ON/OFF
- Press Plus (+) to slow down
- Press R to speed up

### Step 4: Monitor Servos
```bash
ros2 run robodog robodog_servo_scanner
```

## Hardware Configuration

### Servo Hardware
- **Model:** LX16a (20 units)
- **Protocol:** TTL Serial (114200 baud, fixed)
- **Position Range:** 0-1000 raw units (0-240°)
- **Movement Time:** 10-2550 ms per command
- **Response Time:** 60.36 ms average read time

### Serial Connection
- **Port:** `/dev/ttyUSB0` (or `/dev/ttyACM0` as fallback)
- **Baud Rate:** 115200 (fixed for LX16a)
- **Interface:** USB-to-TTL serial adapter
- **Pins:** TX, RX, GND connected to servo bus

### Joy Controller
- **Device:** Nintendo Pro Controller
- **Connection:** Bluetooth
- **Device File:** `/dev/input/js0`
- **ROS2 Driver:** `joy_node`

## Troubleshooting

### Servos Not Responding
1. Check USB cable connection
2. Verify power to servo bus
3. Try alternate port: `-p port:=/dev/ttyACM0`
4. Run servo scanner to diagnose:
   ```bash
   ros2 run robodog robodog_servo_scanner
   ```

### Joy Controller Not Detected
1. Ensure Bluetooth is enabled
2. Pair Nintendo Pro Controller with Jetson
3. Check `/dev/input/js0` exists:
   ```bash
   ls -la /dev/input/js*
   ```

### Servo Movement Jerky or Unresponsive
1. Check Joy node is publishing (should be 160-180 Hz)
2. Check remote controller is receiving Joy messages
3. Increase move time to smooth movement:
   - Decrease servo speed (press Plus)
   - Adjust `m_servo_move_time` in RemoteController

## Code Examples

### Publishing Joy Command to Move Servo
```python
import rclpy
from sensor_msgs.msg import Joy

node = rclpy.create_node('test')
pub = node.create_publisher(Joy, '/joy', 10)

# Move servo 1 forward
joy_msg = Joy()
joy_msg.axes = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]  # Left Y = 1.0
joy_msg.buttons = [0] * 16
pub.publish(joy_msg)
```

### Reading Servo Position
```cpp
#include "Servos/LX16a.h"

auto lx16a = std::make_shared<LX16a>(io_ctx);
lx16a->open("/dev/ttyUSB0");

int16_t position = 0;
if (lx16a->get_pos(1, position)) {
    printf("Servo 1 position: %d\n", position);
}
```

## Future Enhancements

1. **Multi-leg Coordination**
   - Map sticks to multi-servo leg control
   - Implement gait patterns (walk, trot, gallop)

2. **Trajectory Planning**
   - Smooth movement curves between positions
   - Reduce jerkiness in servo movement

3. **Feedback Control**
   - Monitor servo temperature and load
   - Adjust movement speed based on load

4. **Advanced Button Mapping**
   - Trigger predefined poses (stand, sit, etc.)
   - Record and playback movement sequences

5. **Performance Optimization**
   - Batch servo commands to reduce serial bus traffic
   - Implement motion interpolation on Jetson

## References

- **LX16a Servo Datasheet:** 115200 baud, TTL protocol, 0-1000 position range
- **Joy Message:** 16 buttons, 6 axes (sensor_msgs/msg/Joy)
- **ROS2 Transport Drivers:** Serial communication via drivers::serial_driver

---

**Last Updated:** 2025-11-29  
**System Status:** ✅ Production Ready for Movement Control
