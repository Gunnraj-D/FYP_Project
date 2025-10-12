# Downward Movement Fix

## Problem

The robot kept moving downward continuously instead of settling at the target height above the hand.

## Root Cause

**The offset calculation was inverted!**

The code was calculating: `tcp_offset = target_pos_tcp - hand_pos_tcp`

This gives the direction to move the **hand** toward the target, but we were applying it to move the **TCP**!

### Why This Caused Downward Movement

**Example scenario:**

- Hand is at TCP position: `[0, 0, 0.20]` (20cm below TCP)
- Target position: `[0, 0, 0.15]` (want hand 15cm below TCP)
- Calculated offset: `[0, 0, 0.15] - [0, 0, 0.20] = [0, 0, -0.05]`
- Applied to TCP: TCP moves UP by 5cm
- **Result**: Hand (fixed in space) appears to move DOWN 5cm relative to TCP
- **Opposite of what we wanted!**

## The Fix

**Inverted the offset calculation:**

```python
# OLD (WRONG):
tcp_offset = target_pos_tcp - hand_pos_tcp  # Direction to move HAND

# NEW (CORRECT):
tcp_offset = hand_pos_tcp - target_pos_tcp  # Direction to move TCP
```

### Why the Inversion Works

We control the **TCP**, not the hand. The hand is fixed in world space.

- If hand needs to move **UP** in TCP frame → TCP needs to move **DOWN** in base frame
- If hand needs to move **DOWN** in TCP frame → TCP needs to move **UP** in base frame

By inverting the offset (`hand - target` instead of `target - hand`), we get the correct TCP movement direction.

## Changes Made

### 1. `unified_hand_tracking_state.py` - Movement calculation (line 222)

```python
# Calculate offset: hand needs to move from current position to target
# But we control TCP, not hand! So we need INVERSE offset
# If hand needs to move UP, TCP needs to move DOWN (and vice versa)
tcp_offset = hand_pos_tcp - target_pos_tcp  # Inverted: hand - target
```

### 2. Removed Distance to Target Display

Removed the distance to target calculation and display from the visualization as it was not useful. The TCP coordinates (raw and corrected) provide all the necessary information.

**File**: `src/hand_detection/hand_detection_module.py`

- Simplified visualization to show only TCP coordinates
- Removed distance calculation to DISTANCE_TO_REMAIN_M

## Expected Behavior After Fix

**Scenario: Hand is 20cm below TCP, target is 15cm below TCP**

1. System calculates: `tcp_offset = [0, 0, 0.20] - [0, 0, 0.15] = [0, 0, 0.05]`
2. TCP moves DOWN by 5cm (in TCP frame's +Z direction)
3. Hand (fixed in space) appears to move UP 5cm relative to TCP ✅
4. Hand reaches target position of 15cm below TCP ✅

## Placement Pose Note

The placement pose calculation was updated to use direct transformation:

```python
placement_pose_homogeneous = np.concatenate([placement_pose_tcp, [1.0]])
placement_in_base_homogeneous = current_tcp_matrix @ placement_pose_homogeneous
placement_pose_base = placement_in_base_homogeneous[:3]
```

This transforms the desired TCP-frame position directly to base frame coordinates.

## Testing

1. Run hand tracking with robot in static position first
2. Observe TCP (corrected) coordinates showing your hand position
3. Start hand tracking state
4. Robot should move to position TCP 15cm above your hand
5. Robot should settle and stop moving when hand is at target
6. Hold hand still for 2 seconds → state completes

## Visualization

The visualization now shows:

- **FPS**: Frame rate
- **Depth**: Hand distance from camera
- **Cam**: Camera frame coordinates (x, y, z)
- **TCP (raw)**: Before calibration offset correction
- **TCP (corrected)**: After -138mm Z offset applied

These coordinates tell you exactly where your hand is relative to the gripper TCP.

## Related Fixes

This fix works in conjunction with:

1. **Coordinate transformation** - Properly transforms TCP-frame offsets to base frame
2. **Y-axis inversion** - Fixes left/right reflection for user-facing operation
3. **Z offset hack** - Compensates for calibration with wrong TCP (-138mm)

## Files Modified

- `src/states/unified_hand_tracking_state.py` - Fixed offset calculation direction
- `src/hand_detection/hand_detection_module.py` - Simplified visualization
