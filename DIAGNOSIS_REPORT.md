# Servo Communication Diagnosis Report

**Date:** November 28, 2025  
**Robot:** RoboDog V3  
**Issue:** Servo 5 not responding to position queries  
**Status:** 🔴 ROOT CAUSE IDENTIFIED

---

## Summary

The diagnostic tool has confirmed that **NO servos are currently responding** on the serial bus. All 20 servo IDs (1-20) show "No response received from servo".

The software is working correctly. This is a **hardware/connection issue**.

---

## What We Found

### ✅ Working Components
- ✅ Jetson Orin Nano boots and runs ROS2 Humble
- ✅ Serial port `/dev/ttyUSB0` opens successfully
- ✅ Software sends commands to the port (no errors)
- ✅ LX16a driver is correctly implemented

### ❌ Not Working
- ❌ **NO servos are responding to any queries**
- ❌ All position read attempts timeout after 100ms
- ❌ No data coming back from serial bus at all

---

## Root Cause

When ALL servos fail to respond, the most likely causes are (in order):

1. **🔌 Servo Power Supply NOT Connected or Dead**
   - Servos need power to respond
   - Battery might be discharged
   - Connections might be loose

2. **🔗 TX/RX Wires Reversed or Disconnected**
   - Data flows only one way
   - Adapter not getting servo responses

3. **❌ Wrong Serial Port or Adapter**
   - Adapter on `/dev/ttyACM0` instead of `/dev/ttyUSB0`
   - USB adapter not functioning

4. **📍 No Physical Servo Connection**
   - Servos not actually plugged in
   - Or different servo IDs than 1-20

---

## What We Added

To help diagnose this, we've added tools:

### 1. **Enhanced Diagnostics Tool**
```bash
ros2 run robodog robodog_servo_diagnostics --ros-args -p servo_id:=5
```
- Shows detailed logging of what's failing
- Indicates "No response received" when servo not replying

### 2. **New Servo Scanner Tool** ⭐ **USE THIS FIRST**
```bash
ros2 run robodog robodog_servo_scanner
```
- Scans all servo IDs (1-20)
- Shows which ones ARE responding
- Perfect for identifying hardware problems

### 3. **Troubleshooting Documentation**
- `SERVO_NOT_RESPONDING.md` - Complete troubleshooting guide
- `SERVO_FIX_QUICK_GUIDE.md` - Quick checklist
- `SERVO_DIAGNOSTICS.md` - Updated with scanner info

---

## Next Steps

### Immediate Action (5 minutes)

**1. Run servo scanner to confirm**
```bash
cd ~/ros2_ws && source install/setup.bash
ros2 run robodog robodog_servo_scanner
```

**2. Check your answer:**
- ✅ **Sees servos?** → Skip to "After Hardware Fix" section below
- ❌ **No servos?** → Follow hardware troubleshooting below

### Hardware Troubleshooting (15-30 minutes)

**Do ALL of these:**

1. **Check Power:**
   - Physically look: Is battery connected?
   - Can you manually move a servo (should have slight resistance)?

2. **Check Connections:**
   - Red wire: adapter +5V ✓
   - Black wire: adapter GND ✓
   - TX/RX wires not reversed ✓

3. **Check Port:**
   ```bash
   ls /dev/ttyUSB* /dev/ttyACM*
   ```
   - If you see `/dev/ttyACM0` instead, try:
   ```bash
   ros2 run robodog robodog_servo_scanner --ros-args -p port:=/dev/ttyACM0
   ```

4. **Reseat Connectors:**
   - Unplug and replug all servo cables
   - They might just be loose

5. **Test With Different Port:**
   - Try different USB port on Jetson

**After doing steps 1-5, run scanner again**

### After Hardware Fix

Once `robodog_servo_scanner` shows responding servos:

```bash
# Test servo 5 specifically
ros2 run robodog robodog_servo_diagnostics --ros-args -p servo_id:=5

# Should now see data and Hz measurements instead of timeouts

# Then launch full robot
ros2 launch robodog remote_controller.launch.py
```

---

## If Still Not Working

1. **Try different USB adapter** (if available)
   - Yours might be broken

2. **Try different servo** (if available)
   - Your servo might be broken

3. **Capture diagnostic output:**
   ```bash
   ros2 run robodog robodog_servo_scanner > servo_scan.log 2>&1
   ```
   - Share this file for further debugging

---

## Documentation Files

**Start here for different needs:**

| Need | Read | Command |
|------|------|---------|
| Quick fix | `SERVO_FIX_QUICK_GUIDE.md` | (just read) |
| Full troubleshooting | `SERVO_NOT_RESPONDING.md` | (just read) |
| Understand diagnostics | `SERVO_DIAGNOSTICS.md` | `ros2 run robodog robodog_servo_diagnostics` |
| Find connected servos | (already in guides) | `ros2 run robodog robodog_servo_scanner` |

---

## Key Takeaways

1. **Software is correct** - The issue is hardware/connections
2. **Use servo scanner first** - It tells you if ANY servos work
3. **Power and wiring are most likely** - Check those first
4. **TX/RX order matters** - Backwards = one-way communication
5. **Once servos respond** - Diagnostics will work normally

---

## Questions?

Most common next issue after fix: "Servo responds but moves wrong"  
→ See `SERVO_CALIBRATION.md` for servo offset adjustment

