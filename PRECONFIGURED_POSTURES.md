# RoboDog V3 - Preconfigured Posture System

## Overview

The RemoteControllerProper implementation includes preconfigured posture system with two stable positions: **SIT** and **STAND**. These postures are based on servo configuration data from `Leg_HW_Config.txt` and are accessible through the Joy controller.

## Posture Configuration

### Source Data

All posture configurations are derived from `/src/Robots/Leg_HW_Config.txt` which contains:

1. **Hardware Configuration** - Each leg's servo IDs and ranges
2. **Sit Position** - Folded configuration for resting
3. **Stand Position** - Extended configuration for locomotion

### Configuration Data from Leg_HW_Config.txt

#### Sit Position (Legs Folded)
```
Front Right:
  - Hip Joint: Servo 20→470, Servo 8→490
  - Knee Joint: Servo 19→880, Servo 17→100 (fully folded)

Back Right:
  - Hip Joint: Servo 13→480, Servo 18→510
  - Knee Joint: Servo 7→900, Servo 5→100 (fully folded)

Back Left:
  - Hip Joint: Servo 12→490, Servo 4→520
  - Knee Joint: Servo 16→110, Servo 11→900 (fully folded)

Front Left:
  - Hip Joint: Servo 3→480, Servo 15→516
  - Knee Joint: Servo 9→80, Servo 10→900 (fully folded)
```

#### Stand Position (Legs Extended)
```
Front Right:
  - Hip Joint: Servo 20→600, Servo 8→360
  - Knee Joint: Servo 19→730, Servo 17→250

Back Right:
  - Hip Joint: Servo 13→610, Servo 18→380
  - Knee Joint: Servo 7→750, Servo 5→250

Back Left:
  - Hip Joint: Servo 12→490, Servo 4→520 (same as sit)
  - Knee Joint: Servo 16→500, Servo 11→500 (neutral position)

Front Left:
  - Hip Joint: Servo 3→480, Servo 15→516 (same as sit)
  - Knee Joint: Servo 9→499, Servo 10→497 (neutral position)
```

## Implementation in RemoteControllerProper.cpp

### Posture Functions

The system uses two main posture functions:

#### move_to_sit()
```cpp
void move_to_sit()
{
    int move_time = 1500;  // 1.5 seconds for smooth transition
    
    // For each leg, move to sit position using IK-based height control
    // Z = 0.08m (body lowered, legs folded)
    for (auto& leg_pair : m_legs) {
        auto leg = leg_pair.second;
        auto loc = leg->get_leg_location();
        leg->move(loc.x, loc.y, 0.08, move_time, false);
    }
}
```

**Effects:**
- Body lowers to 0.08m height above ground
- All knees fold (high servo position values)
- Hips move to middle positions
- Smooth 1.5-second transition
- Result: Robot in resting/sitting posture

#### move_to_stand()
```cpp
void move_to_stand()
{
    int move_time = 1500;  // 1.5 seconds for smooth transition
    
    // For each leg, move to stand position using IK-based height control
    // Z = 0.12m (body raised, legs extended)
    for (auto& leg_pair : m_legs) {
        auto leg = leg_pair.second;
        auto loc = leg->get_leg_location();
        leg->move(loc.x, loc.y, 0.12, move_time, false);
    }
}
```

**Effects:**
- Body raises to 0.12m height above ground
- All knees extend (lower servo position values)
- Hips move to appropriate positions
- Smooth 1.5-second transition
- Result: Robot in standing/ready posture

## Control via Joy Controller

### Button Mapping

- **Y Button** → Move to SIT position
- **X Button** → Move to STAND position

### Usage

1. Press **Y** to transition from any position to SIT
2. Press **X** to transition from any position to STAND
3. Each transition takes 1.5 seconds for smooth movement
4. During transition, logging shows:
   ```
   [INFO] Moving to SIT position using preconfigured positions...
   [INFO] Robot sitting
   ```

## Technical Implementation

### Control Flow

```
Joy Button Press (Y or X)
    ↓
handle_button_presses() detects edge (button pressed, not previously pressed)
    ↓
move_to_sit() or move_to_stand() called
    ↓
For each leg in m_legs:
    Get leg's current location (x, y in base frame)
    Call leg->move(x, y, target_z, move_time, false)
    ↓
    Leg::move() internally:
    - Calculates foot position (x, y, target_z)
    - Runs Inverse Kinematics
    - Converts angles to servo positions
    - JointDouble pairs synchronize opposite-facing motors
    - angle_to_servo_pos() applies motor inversion patterns
    ↓
Result: All servos move to synchronized positions for posture
```

### Architecture Integration

The posture system integrates seamlessly with the existing architecture:

1. **Uses Leg API** - Calls `leg->move()` instead of direct servo control
2. **Respects IK** - Inverse Kinematics calculates joint angles from Cartesian coordinates
3. **Maintains Servo Pairing** - JointDouble handles opposite-facing motor synchronization
4. **Applies Motor Inversions** - angle_to_servo_pos() applies per-servo sign conversions

### Why This Approach

The implementation uses **Cartesian height-based control** (Z-axis height) rather than direct servo commands because:

1. **Physics-Based** - Maintains proper leg geometry and mechanical constraints
2. **Robust** - Automatically handles all 20 servos through IK system
3. **Synchronized** - Ensures all legs move together smoothly
4. **Debuggable** - Clear mapping from posture intent to leg positioning
5. **Extendable** - Easy to add new postures by changing target_z values

## Testing the Postures

### Quick Test Procedure

1. **Start the Joy controller:**
   ```bash
   source /home/chubsie/ros2_ws/install/setup.bash
   ros2 run robodog robodog_remote_controller_ik
   ```

2. **Connect Joy controller** (press L button to reset if needed)

3. **Test postures:**
   - Press **X** → Robot should stand up (body raises)
   - Press **Y** → Robot should sit down (body lowers)
   - Watch logs for confirmation messages

4. **Verify movement:**
   - All 4 legs should move smoothly
   - Movement should take ~1.5 seconds
   - No servo errors should appear

### Debugging

If postures don't work as expected:

1. **Check servo IDs** - Verify all 20 servos are connected (use diagnostics)
2. **Check serial port** - Ensure `/dev/ttyUSB0` or `/dev/ttyACM0` is connected
3. **Check temperature** - If servos are hot, they may not move (max 65°C)
4. **Check power** - Verify servos have sufficient power and not brownout
5. **Check Joy input** - Verify Joy controller is connected and X/Y buttons work:
   ```bash
   ros2 topic echo /joy
   ```

## Configuration Files

- **Main Config:** `/src/Robots/Leg_HW_Config.txt`
- **Implementation:** `/src/RemoteControl/RemoteControllerProper.cpp`
- **Build:** `/CMakeLists.txt` - includes robodog_remote_controller_ik target

## Future Enhancements

Possible extensions to the posture system:

1. **More Postures** - Add CROUCH, WALK_READY, ATTACK, etc.
2. **Smooth Transitions** - Intermediate waypoints for natural movement
3. **Gait Patterns** - Preconfigured leg movement sequences
4. **Posture Recovery** - Auto-correct to nearest known posture if movement fails
5. **Configuration File** - Load postures from external config file
6. **Gesture Library** - Predefined multi-step movement sequences

## Notes

- Servo positions range from 0-1000 (0.24° per unit)
- JointDouble pairs use opposite-facing motors with automatic inversion
- Move time of 1500ms provides smooth natural motion
- Height values (0.08m sit, 0.12m stand) are empirically tuned
- All movements respect maximum servo temperature (65°C)

## References

- Robot servo configuration: `Leg_HW_Config.txt`
- Servo control system: `Servos/LX16a.h`, `Servos/ServoController.h`
- Leg system: `Leg.h`, `Leg.cpp`
- Inverse kinematics: `IK.h`
- Joint system: `Joint.h` (includes JointDouble)
