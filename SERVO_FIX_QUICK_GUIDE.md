# Servo Communication Troubleshooting - Quick Start

## Problem
```
[ERROR] [LX16a]: get_pos: read_packet(id=5) failed
[WARN] [LX16a]: read_packet(id=5, cmd=0x1C): No response received from servo (timeout after 100ms)
```

## What This Means
The servo is **not sending any data back** to the Jetson. This is a hardware issue, not software.

## Quick Diagnosis (5 minutes)

### Step 1: Scan for Servos
```bash
cd ~/ros2_ws && source install/setup.bash
ros2 run robodog robodog_servo_scanner
```

### Step 2: Check Output
- ✅ **If you see servo IDs**: Hardware is working, servo software issue
- ❌ **If it says "No servos found"**: Hardware problem

## Hardware Checklist (Most Common Causes)

**Power:**
- [ ] Battery connected to robot?
- [ ] Servo adapter has power light (if it has one)?
- [ ] Try wiggling servo cables slightly - any movement?

**Connections:**
- [ ] Red wire (5V) → Servo adapter +5V
- [ ] Black wire (GND) → Servo adapter GND  
- [ ] Yellow wire (RX) → Servo adapter TX
- [ ] White/Blue wire (TX) → Servo adapter RX

**Port:**
- [ ] USB adapter plugged into Jetson?
- [ ] Try: `ls /dev/ttyUSB* /dev/ttyACM*`
- [ ] If you see `/dev/ttyACM0` instead of `/dev/ttyUSB0`, try:
  ```bash
  ros2 run robodog robodog_servo_scanner --ros-args -p port:=/dev/ttyACM0
  ```

## Solution Path

1. **Unplug and reseat all servo connectors** (they might be loose)

2. **Check all wires are correct colors** (TX/RX often reversed)

3. **Try wiggling battery connector** to ensure power

4. **If still failing:** 
   - Try plugging robot into different USB port on Jetson
   - Check if USB adapter LED lights up

5. **If adapter doesn't light up:**
   - Adapter might be broken
   - Ask for different USB-to-TTL adapter

## After Fix

Once `robodog_servo_scanner` shows servos:

```bash
# Test specific servo
ros2 run robodog robodog_servo_diagnostics --ros-args -p servo_id:=5

# Run full robot
ros2 launch robodog remote_controller.launch.py
```

## Full Guide

See: `SERVO_NOT_RESPONDING.md` for complete troubleshooting steps

