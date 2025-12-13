#!/bin/bash

echo "=============================================="
echo "  8BitDo Controller Connection Fix"
echo "=============================================="
echo ""
echo "Your controller is connected but not creating"
echo "a joystick device (/dev/input/js*)."
echo ""
echo "This is a known issue with 8bitdo in Switch mode."
echo ""
echo "SOLUTION: Switch controller to X-Input mode"
echo ""
echo "Steps:"
echo "  1. Turn OFF your controller (hold Home for 3s)"
echo "  2. Remove the old pairing:"

echo ""
echo "Removing old pairing..."
bluetoothctl remove E4:17:D8:62:0A:13 2>/dev/null
sleep 2

echo ""
echo "  3. Turn ON controller in X-INPUT MODE:"
echo "     ⚠️  IMPORTANT: Hold X + START buttons while turning on"
echo "     (Not Home button - use X + START)"
echo ""
echo "  4. Put in pairing mode: Hold START for 3 seconds"
echo "     LED should flash rapidly"
echo ""
read -p "Press ENTER when controller is in pairing mode (LED flashing)..."

echo ""
echo "Scanning for controller..."

SCAN_OUTPUT=$(mktemp)
timeout 12s bluetoothctl scan on > "$SCAN_OUTPUT" 2>&1 &
SCAN_PID=$!
sleep 12
kill $SCAN_PID 2>/dev/null
wait $SCAN_PID 2>/dev/null

# Look for Xbox, 8BitDo, or Pro Controller
MAC=$(grep -iE "Xbox|8BitDo|Pro Controller|Wireless Controller" "$SCAN_OUTPUT" | grep -oE '([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}' | head -n 1)
rm "$SCAN_OUTPUT"

if [ -z "$MAC" ]; then
    echo "❌ Controller not found in scan"
    echo ""
    echo "Please ensure:"
    echo "  - Controller is ON (X + START held while powering)"
    echo "  - In pairing mode (hold START for 3s, LED flashing)"
    echo "  - Try running this script again"
    exit 1
fi

echo "✓ Found controller: $MAC"
echo ""

echo "Pairing..."
echo -e "pair $MAC\nyes\n" | bluetoothctl
sleep 3

echo "Trusting..."
bluetoothctl trust "$MAC"
sleep 1

echo "Connecting..."
bluetoothctl connect "$MAC"
sleep 4

echo ""
echo "Checking joystick device..."
sleep 2

if ls /dev/input/js* > /dev/null 2>&1; then
    JS_DEV=$(ls /dev/input/js* | head -n 1)
    echo ""
    echo "✅ SUCCESS! Joystick device created: $JS_DEV"
    echo ""
    echo "Test with: ros2 run joy joy_node --ros-args -p device_id:=0"
    echo "Then in another terminal: ros2 topic echo /joy"
else
    echo ""
    echo "⚠️  Still no /dev/input/js* device"
    echo ""
    echo "Alternative: Try D-Input mode instead"
    echo "  - Turn off controller"
    echo "  - Hold A + START while turning on"
    echo "  - Run this script again"
    echo ""
    echo "Or check if evdev works:"
    echo "  ros2 run joy joy_enumerate_devices"
fi

echo ""
