# Y-Axis Reflection Fix

## Problem

The robot's Y-axis movement was reflected - when the hand moved left, the robot moved right (and vice versa).

## Root Cause

The hand-eye calibration has the camera mounted with **inverted axes** relative to TCP:

- Camera X (+right in image) → TCP X (-1x) [inverted]
- Camera Y (+down in image) → TCP Y (-1x) [inverted]

This is normal for the camera mounting orientation. However, when the user stands **in front of the robot**, their left/right perspective is opposite to the robot's coordinate frame, creating a perceived "reflection" effect.

## Diagnostic Results

From `debug_y_axis_reflection.py`:

```
[Test 1] Hand moves DOWN in camera image (+Y camera):
  Camera movement: [0, +0.05, 0] (5cm down in image)
  TCP movement:    [-0.00134  -0.04998  -0.000605]
  → X: -1.3mm, Y: -50.0mm, Z: -0.6mm

[Test 2] Hand moves RIGHT in camera image (+X camera):
  Camera movement: [+0.05, 0, 0] (5cm right in image)
  TCP movement:    [-0.04998   0.001335  0.000435]
  → X: -50.0mm, Y: +1.3mm, Z: +0.4mm
```

Both X and Y axes show inversion through the hand-eye transformation.

## Solution

Added Y-axis inversion in the TCP offset before transforming to base frame:

```python
# Calculate offset in TCP frame
tcp_offset = target_pos_tcp - hand_pos_tcp

# EXPERIMENTAL: Invert Y to fix reflection (user standing in front of robot)
tcp_offset[1] = -tcp_offset[1]

# Transform to base frame (this handles the rest of the coordinate rotation)
tcp_rotation = current_tcp_matrix[:3, :3]
tcp_offset_in_base = tcp_rotation @ tcp_offset
target_position_base = current_tcp_pose[:3] + tcp_offset_in_base
```

## Changes Made

### `src/states/unified_hand_tracking_state.py`

Added Y-axis inversion in two locations:

1. **`_move_robot_toward_hand()`** - Line ~224: Inverts Y offset before transformation
2. **`_calculate_placement_pose()`** - Line ~300: Inverts Y offset for placement calculation

## Expected Behavior After Fix

**Before:**

- Hand moves LEFT (user's perspective) → Robot moves RIGHT ❌

**After:**

- Hand moves LEFT (user's perspective) → Robot moves LEFT ✅
- Hand moves RIGHT (user's perspective) → Robot moves RIGHT ✅

## Testing

1. Run hand tracking with robot in static position
2. Move your hand LEFT/RIGHT in front of camera
3. Observe if visualization shows correct Y movement direction
4. Test with robot movement enabled
5. Verify robot tracks hand in correct direction

## Alternative Solutions

If the Y-inversion doesn't fully fix the issue, consider:

1. **Invert X axis as well** - The diagnostic shows X is also inverted
2. **Recalibrate hand-eye matrix** - Use correct TCP and mounting orientation
3. **Add configuration flag** - Make axis inversions configurable

## Configuration Option (Future)

Consider adding to `system_config.py`:

```python
# Hand tracking axis inversions (for user-facing operation)
INVERT_HAND_TRACKING_X = False
INVERT_HAND_TRACKING_Y = True
```

## Frame Conventions

**TCP Frame** (gripper facing down):

- +X = Forward (along gripper approach)
- +Y = Left (across gripper)
- +Z = Down (gripper opening direction)

**Camera Frame** (RealSense):

- +X = Right (in image)
- +Y = Down (in image)
- +Z = Forward (depth into scene)

**User Perspective** (standing in front of robot):

- User's LEFT = Robot's RIGHT
- User's RIGHT = Robot's LEFT
- This creates the need for Y-axis inversion

## Notes

- The coordinate transformation (rotation matrix multiplication) is still required and correct
- The Y-axis inversion is applied **before** the transformation to base frame
- This is marked as EXPERIMENTAL and can be toggled on/off easily
- Long-term solution: Recalibrate with correct TCP and frame conventions

## Removal

To remove this fix (if you recalibrate or want different behavior):

1. Delete lines with `tcp_offset[1] = -tcp_offset[1]`
2. Delete lines with `tcp_offset_placement[1] = -tcp_offset_placement[1]`
3. Search for "EXPERIMENTAL: Invert Y" comments
