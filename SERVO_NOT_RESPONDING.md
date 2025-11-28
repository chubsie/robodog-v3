# Servo No Response Troubleshooting Guide

## Diagnosis Summary

The servo scanner tool has detected that **NO servos are responding** on the serial bus (IDs 1-20 all show no response).

This means commands are being sent to `/dev/ttyUSB0`, but no responses are being received back from any servos.

## Root Cause Analysis

When `No response received from servo (timeout after 100ms)` appears, it means:

1. ✅ The serial port opened successfully
2. ✅ The commands are being sent to the port
3. ❌ **NO DATA is coming back** from the servos

### Most Likely Causes (in order of probability):

1. **🔌 Servo Power Supply NOT Connected** (Most Common)
   - Servos need power to respond
   - Even with battery "plugged in", check connections
   
2. **🔗 TX/RX Wires Reversed or Disconnected**
   - Serial communication is one-way if wires are reversed
   - Check TTL connections to adapter
   
3. **❌ Wrong Serial Adapter or Port**
   - Device is connected to `/dev/ttyACM0` instead of `/dev/ttyUSB0`
   - Or adapter is not functioning
   
4. **📍 No Servos on Bus or Wrong IDs**
   - Servos aren't physically connected
   - Or they have different ID numbers than 1-20
   
5. **🔧 Servo Adapter Broken**
   - USB-to-TTL adapter may be damaged
   - Try different adapter if available

---

## Step-by-Step Troubleshooting

### Step 1: Verify Power Supply

**Check physical power:**
```bash
# Look at your robot
# 1. Is the battery connected to the servo adapter?
# 2. Is the battery charged?
# 3. Do servos have any indicator lights?
```

**What you should see:**
- Battery light on servo adapter (if it has one)
- Servo may move slightly or show signs of power

### Step 2: Scan for Connected Servos

The servo scanner will help identify which servos are actually connected:

```bash
cd ~/ros2_ws
source install/setup.bash

# Scan for servos on default port
ros2 run robodog robodog_servo_scanner

# Or try alternate port
ros2 run robodog robodog_servo_scanner --ros-args -p port:=/dev/ttyACM0
```

**What you should see:**
- Either "Found X responsive servo(s)" with IDs
- Or "No servos found!" with troubleshooting steps

### Step 3: Check Serial Port

**List available serial ports:**
```bash
ls -la /dev/tty* | grep -E "USB|ACM"
```

**Expected output:**
```
crw-rw---- 1 root dialout 188,  0 Nov 28 21:41 /dev/ttyUSB0
```

**If you see neither `/dev/ttyUSB0` nor `/dev/ttyACM0`:**
- Adapter is not connected to Jetson
- Or adapter is broken

### Step 4: Test Serial Adapter Directly

```bash
# Install minicom if not present
sudo apt install minicom

# Test on port
sudo minicom -b 115200 -D /dev/ttyUSB0

# Type something and look for character echo
# (Note: This is just to test the port works)
# Press Ctrl+A then X to exit
```

### Step 5: Check Wiring

**Servo connections:**
```
TTL Adapter:
├── 5V   → Servo Power (Red wire)
├── GND  → Servo Ground (Black wire)
├── TX   → Servo RX (Yellow/White wire)
└── RX   → Servo TX (Green/Blue wire)

⚠️ IMPORTANT: TX connects to RX, RX connects to TX!
```

**Common mistakes:**
- ❌ TX to TX, RX to RX (data won't go both ways)
- ❌ Only connecting some servos (need parallel connection)
- ❌ Missing ground connection

### Step 6: Verify Servo IDs

**If you found servos with the scanner but wrong IDs:**

The servo IDs might not be 1-20. Scan with a wider range:

```bash
# Scan IDs 1-50
ros2 run robodog robodog_servo_scanner --ros-args -p start_id:=1 -p end_id:=50
```

Then update `src/RoboDog_v3_ros2/src/Robots/RobotV3.cpp` with the actual IDs you find.

### Step 7: Try Alternate Serial Port

If you have the adapter plugged in but nothing responds:

```bash
# Try /dev/ttyACM0 instead
ros2 run robodog robodog_servo_scanner --ros-args -p port:=/dev/ttyACM0
```

---

## Hardware Checklist

Go through this physical checklist:

```
□ Battery is connected to robot
□ Battery has power (try moving servos manually - should have slight resistance)
□ USB adapter is plugged into Jetson Orin Nano
□ Servo power wire (red) is connected to adapter +5V
□ Servo ground wire (black) is connected to adapter GND
□ Servo TX wire is connected to adapter RX
□ Servo RX wire is connected to adapter TX
□ All wires are firmly seated
□ No bent or broken connector pins
```

---

## Diagnostic Tools Available

### Servo Scanner
```bash
# Find which servo IDs are responding
ros2 run robodog robodog_servo_scanner
```

### Servo Diagnostics
```bash
# Test specific servo's Hz (frequency)
ros2 run robodog robodog_servo_diagnostics --ros-args -p servo_id:=5
```

### Hardware Node (Full System Test)
```bash
# Attempts to initialize entire robot with all servos
ros2 run robodog robodog_hw
```

---

## Enhanced Logging

If you want more detailed debugging output:

```bash
# Run with DEBUG logging
ros2 run robodog robodog_servo_scanner --ros-args --log-level DEBUG
```

This will show:
- Exact bytes being sent
- Any validation errors
- Timeout occurrences

---

## Common Scenarios

### Scenario 1: Adapter shows up but no servos respond
**Likely cause:** Power not connected, or TX/RX wires reversed

**Fix:**
1. Check battery/power connection
2. Verify TX/RX are connected correctly (not reversed)
3. Reseat all connectors

### Scenario 2: Wrong serial port detected
**Likely cause:** Adapter is on `/dev/ttyACM0` instead of `/dev/ttyUSB0`

**Fix:**
```bash
# Test alternate port
ros2 run robodog robodog_servo_scanner --ros-args -p port:=/dev/ttyACM0

# If that works, update RobotV3.cpp:
# Change: if (!lx16a->open("/dev/ttyUSB0"))
# To:     if (!lx16a->open("/dev/ttyACM0"))
```

### Scenario 3: Some servos respond, others don't
**Likely cause:** Not all servos are connected or have power

**Fix:**
1. Check each servo connection individually
2. Verify power reaches all servos
3. Use the scanner to find which IDs DO respond

### Scenario 4: Servo responds but moves incorrectly
**This is different** - servo IS responding. See SERVO_CALIBRATION.md

---

## What NOT to Do

❌ Don't use `/dev/ttyS0` or `/dev/ttyS1` (those are the Jetson's internal UART)  
❌ Don't change baud rate from 115200 (LX16a is fixed at this rate)  
❌ Don't connect 5V directly to Jetson GPIO (it's 3.3V!)  
❌ Don't plug/unplug while powered on (can damage servos)  

---

## If Still Not Working

### Final Steps:

1. **Test with different adapter** (if available)
   - Borrow a USB-to-TTL adapter to verify yours isn't broken

2. **Test with different servo** (if available)
   - Verify at least one servo works

3. **Boot into working system** (if available)
   - Test on original system to confirm hardware works

4. **Check kernel driver**
   ```bash
   dmesg | grep -i "usb\|ttyUSB\|ttyACM"
   
   # Or use lsusb if dmesg shows nothing
   lsusb | grep -i "ch340\|ftdi\|serial"
   ```

5. **Reinstall drivers** (for CH340 USB adapters on some Linux systems)
   ```bash
   sudo apt install ch340
   ```

---

## Next Steps After Fixing

Once `robodog_servo_scanner` shows responsive servos:

1. Run full diagnostics:
   ```bash
   ros2 run robodog robodog_servo_diagnostics --ros-args -p servo_id:=5
   ```

2. Verify all servos respond:
   ```bash
   for id in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20; do
     echo "Testing servo $id..."
     ros2 run robodog robodog_servo_diagnostics --ros-args -p servo_id:=$id -p measurement_duration:=2.0
   done
   ```

3. Calibrate servos (see SERVO_CALIBRATION.md)

4. Launch full robot control:
   ```bash
   ros2 launch robodog remote_controller.launch.py
   ```

---

## Questions?

If the troubleshooting doesn't resolve the issue, capture and share:

1. Output from `robodog_servo_scanner`
2. Output from `ls /dev/tty*`
3. Your wiring diagram
4. Any error messages with `--log-level DEBUG` enabled

