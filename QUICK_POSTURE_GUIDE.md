# Quick Posture Reference - RoboDog V3

## Accessing Postures

### Via Joy Controller

| Button | Action | Result |
|--------|--------|--------|
| **X** | Press once | Robot stands up (1.5 sec transition) |
| **Y** | Press once | Robot sits down (1.5 sec transition) |

### Posture Details

#### STAND Position
- **Height:** 0.12m (body raised)
- **Leg State:** Extended, ready for locomotion
- **Use Case:** Before moving, after sitting, idle ready state
- **Transition Time:** 1.5 seconds

#### SIT Position
- **Height:** 0.08m (body lowered)  
- **Leg State:** Folded, body resting
- **Use Case:** Resting, stable idle, power saving
- **Transition Time:** 1.5 seconds

## Step-by-Step Usage

### 1. Initialize Robot
```bash
source /home/chubsie/ros2_ws/install/setup.bash
ros2 run robodog robodog_remote_controller_ik
```

### 2. Stand Robot Up
```
Joy Controller: Press X Button
Expected: Robot smoothly raises body to standing height
Time: ~1.5 seconds
Console Output: "Robot standing"
```

### 3. Sit Robot Down
```
Joy Controller: Press Y Button
Expected: Robot smoothly lowers body to sitting height
Time: ~1.5 seconds
Console Output: "Robot sitting"
```

### 4. Return to Standing
```
Joy Controller: Press X Button (from sit position)
Expected: Robot returns to standing
Time: ~1.5 seconds
Console Output: "Robot standing"
```

## Console Output Examples

### Successful Stand Command
```
[robodog_remote_controller] [INFO] Moving to STAND position using preconfigured positions...
[robodog_remote_controller] [INFO] Robot standing
```

### Successful Sit Command
```
[robodog_remote_controller] [INFO] Moving to SIT position using preconfigured positions...
[robodog_remote_controller] [INFO] Robot sitting
```

### Partial Success (with warnings)
```
[robodog_remote_controller] [INFO] Moving to SIT position using preconfigured positions...
[robodog_remote_controller] [WARN] Failed to move leg 2 to sit position
[robodog_remote_controller] [INFO] Robot sitting
```

## Troubleshooting

### Robot doesn't respond to X or Y buttons
1. Check Joy controller is connected: `ros2 topic echo /joy | grep buttons`
2. Verify button indices: X=buttons[3], Y=buttons[2]
3. Check servos aren't disabled: Use L button to toggle servo enable

### Robot sits/stands but very slowly
1. Check temperature: Should be < 65°C
2. Verify power supply: Servos need stable voltage
3. Check for mechanical binding: Legs should move freely

### Only some legs move
1. Check which servos have errors: `ros2 topic echo /robodog/servo_status`
2. Test individual servo movement
3. Verify serial communication on `/dev/ttyUSB0` or `/dev/ttyACM0`

### Movement is jerky or irregular
1. Reduce Joy input during posture transitions
2. Let complete transition finish before next command
3. Check servo temperatures and power stability

## Advanced Notes

### Combining with Other Controls

You can mix posture commands with joystick movement:

```
1. Press X to stand
2. While standing, use left stick to move forward/backward
3. Press Y anytime to sit (even while moving)
4. Use right stick to raise/lower body height while moving
```

### Height Precision

- **Sit height:** 0.08m - legs folded, body very close to ground
- **Stand height:** 0.12m - legs extended, normal walking height
- **Movement range:** 0.05-0.25m (hard limits to prevent damage)

The system automatically clamps heights during joystick control to prevent over-extension.

### Timing Information

- **Transition time:** 1.5 seconds (both sit and stand)
- **Movement units:** Meters (Cartesian coordinates)
- **Button response:** ~10-50ms (Joy controller refresh rate)
- **Servo update rate:** ~50-100Hz

## Integration with Movement

### Recommended Workflow

```
1. Power up robot (hear servo sounds as they initialize)
2. Start joy controller: ros2 run robodog robodog_remote_controller_ik
3. Press X to stand (verify all legs respond)
4. Test joystick movement (forward/backward/strafe)
5. Press Y to sit
6. Press L to disable servos (power saving)
```

### During Testing

- Monitor console output for errors
- Watch all 4 legs move together
- Listen for smooth servo sounds (no grinding/clicking)
- Check temperatures don't exceed 65°C
- Verify L button toggles servo enable properly

## Emergency Controls

| Button | Action | Note |
|--------|--------|------|
| **Minus** | Disable movement | Emergency stop, stops all motion |
| **Plus** | Enable movement | Resume after emergency stop |
| **L** | Toggle servo enable | Disables all servos when pressed |

Use Minus button immediately if robot behaves unexpectedly.

---

**For detailed technical information, see:** `PRECONFIGURED_POSTURES.md`
