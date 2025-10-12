# Y Offset Fix Summary

## Problem

User reported that the TCP (corrected) Y value was not centered at 0 when the hand was physically centered under the gripper TCP.

## Root Cause

The hand-eye calibration matrix places the camera origin at approximately **64mm off-center from the gripper TCP in the Y direction**:

- Camera origin in TCP frame: `(13.6mm, 63.9mm, 14.1mm)`

When the hand is physically centered under the gripper:

- Camera sees it at approximately Y ≈ 0 in camera frame
- Transforms to Y ≈ 0.064m (64mm) in TCP frame
- User expected Y = 0 to mean "centered under gripper TCP" (not "centered under camera")

## Solution

Added Y offset correction alongside the existing Z offset correction in all relevant locations:

```python
hand_pos_tcp[1] -= 0.064  # Subtract 64mm Y offset (camera to TCP)
hand_pos_tcp[2] -= 0.138  # Subtract 138mm Z offset (gripper extension)
```

## Files Modified

### 1. `src/states/unified_hand_tracking_state.py`

Updated 3 locations:

- `_update_hand_tracking()` - Line ~129-130
- `_move_robot_toward_hand()` - Line ~204-205
- `_calculate_placement_pose()` - Line ~283-284

### 2. `src/hand_detection/hand_detection_module.py`

- `_draw_results()` - Line ~209-210
- Visualization now shows both raw and corrected coordinates with Y offset applied

### 3. `test_hand_tracking_coordinates.py`

- Updated to apply Y offset correction when displaying coordinates

## Expected Behavior

**Before fix:**

```
Hand physically centered under gripper TCP:
  TCP (raw):       (0.01, 0.06, 0.25)
  TCP (corrected): (0.01, 0.06, 0.11)  ← Y is 60mm off-center
```

**After fix:**

```
Hand physically centered under gripper TCP:
  TCP (raw):       (0.01, 0.06, 0.25)
  TCP (corrected): (0.01, 0.00, 0.05)  ← Y is now centered!
```

## Testing

1. Run your hand tracking with the robot static
2. Position your hand centered under the gripper TCP
3. Observe "TCP (corrected)" values in the visualization
4. Y should now be close to 0.0 when physically centered

## Long-term Solution

Recalibrate the hand-eye matrix using:

- Correct TCP link (`link_9` instead of `tool0`)
- This will eliminate the need for both Y and Z offset hacks
- Coordinates will be directly relative to the actual gripper TCP

## Note

The X value (left-right) should also be approximately correct. If there's still drift in X, that's a different issue related to the hand-eye calibration rotation component rather than translation offset.
