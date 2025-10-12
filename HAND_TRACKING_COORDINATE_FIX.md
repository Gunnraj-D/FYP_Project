# Hand Tracking Coordinate Frame Fix

## Problem Summary

The hand tracking state was exhibiting unstable movement behavior:

1. Initially drifting upward continuously
2. After user modifications, moving downward too much
3. TCP-relative coordinates not matching expected values
4. Y coordinate not centered on gripper TCP (showed ~64mm offset)

## Root Causes Identified

### 1. **Coordinate Frame Mixing Bug** (FIXED)

**Location**: `unified_hand_tracking_state.py` lines 218-230 and 290-299

**Issue**: The code was calculating offset vectors in the TCP coordinate frame but adding them directly to positions in the base coordinate frame without proper transformation. This caused unpredictable movement because TCP and base frames have different orientations.

**Fix**: Transform TCP-relative offsets to base frame using the TCP rotation matrix:

```python
# Calculate offset in TCP frame
tcp_offset = target_pos_tcp - hand_pos_tcp

# Transform to base frame using TCP rotation
tcp_rotation = current_tcp_matrix[:3, :3]
tcp_offset_in_base = tcp_rotation @ tcp_offset

# Add transformed offset to base position
target_position_base = current_tcp_pose[:3] + tcp_offset_in_base
```

### 2. **Visualization Not Applying Calibration Hack** (FIXED)

**Location**: `hand_detection_module.py` lines 203-223

**Issue**: The visualization was showing raw TCP coordinates without the -138mm calibration offset, making it confusing to debug.

**Fix**: Added display of both raw and corrected TCP coordinates with clearer labeling and using `DISTANCE_TO_REMAIN_M` from config for target calculation.

### 3. **Hand-Eye Calibration Offset** (FIXED with temporary hacks)

**Status**: Fixed with calibration offset corrections (Y and Z offsets)

**Issue**: The hand-eye calibration matrix places the camera origin ~67mm offset from TCP (primarily 64mm in Y direction). This means:

- Camera origin in TCP frame: `(13.6mm, 63.9mm, 14.1mm)`
- When hand is physically at a position, the measured TCP coordinates include this offset
- User expected Y=0 to mean hand is centered under gripper TCP (not camera)
- The -138mm Z hack was already compensating for wrong TCP during calibration

**Diagnostic Output (before Y fix)**:

```
For hand 5cm below TCP in physical space:
  Expected TCP coords: (0.0, 0.0, 0.05)
  Actual TCP coords:   (0.0, 0.05, 0.25)  [raw]
                       (0.0, 0.05, 0.112) [with -138mm Z hack only]

  Error: Z is 200mm too large (raw), 62mm too large (corrected)
         Y is 50mm off-center
```

**Fix**: Added Y offset correction to work with existing Z correction:

```python
hand_pos_tcp[1] -= 0.064  # Subtract 64mm Y offset (camera to TCP)
hand_pos_tcp[2] -= 0.138  # Subtract 138mm Z offset (gripper extension)
```

## Changes Made

### 1. `unified_hand_tracking_state.py`

- Fixed coordinate transformation in `_move_robot_toward_hand()` (lines 218-230)
- Fixed coordinate transformation in `_calculate_placement_pose()` (lines 290-299)
- Added Y offset correction (-64mm) in all three locations alongside existing Z offset (-138mm)
- Now properly transforms TCP-frame offsets to base frame before IK solving

### 2. `hand_detection_module.py`

- Enhanced visualization to show both raw and corrected TCP coordinates
- Added Y offset correction (-64mm) alongside Z offset (-138mm) in visualization
- Added distance-to-target calculation using `DISTANCE_TO_REMAIN_M` from config
- Improved debugging information display

### 3. Created Diagnostic Tools

- `debug_hand_tcp_coordinates.py` - Analyzes hand-eye calibration matrix
- `test_hand_tracking_coordinates.py` - Real-time monitoring of hand position

## Testing & Tuning

### Immediate Testing

1. Run the updated code with your hand at a known position
2. Observe both "TCP (raw)" and "TCP (corrected)" in the visualization
3. Check if the "Dist to target" value makes sense

### Empirical Tuning (if needed)

If the coordinates are still significantly off, you can temporarily adjust:

**File**: `src/config/system_config.py`

```python
DISTANCE_TO_REMAIN_M = 0.40  # Adjust this value empirically
```

To find the right value:

1. Run `test_hand_tracking_coordinates.py`
2. Place hand where you want TCP to be
3. Note the Z value in "TCP (corrected)"
4. Set `DISTANCE_TO_REMAIN_M` to that Z value

### Long-term Solution: Recalibrate Hand-Eye Matrix

The proper fix is to recalibrate the hand-eye matrix using the correct TCP (link 9 instead of link 7/tool0). This will:

- Remove need for -138mm hack
- Provide accurate TCP-relative coordinates
- Fix the Y-axis offset issue

## Expected Behavior After Fix

1. **Coordinate Transformation**: TCP-frame vectors now correctly transform to base frame
2. **Movement Stability**: Robot should smoothly approach and settle at target height
3. **Visualization**: Shows both raw and corrected coordinates for debugging

## Hand-Eye Calibration Analysis

From diagnostic script output:

```
HAND_EYE_MATRIX translation: [13.6mm, 63.9mm, 14.1mm]
Magnitude: 66.8mm

Rotation shows:
- Camera Z-axis ≈ TCP Z-axis (0.9999 correlation)
- Camera X-axis ≈ -TCP X-axis (-0.9996)
- Camera Y-axis ≈ -TCP Y-axis (-0.9996)
```

This means the camera is mounted:

- About 67mm from TCP origin
- Roughly aligned in Z (depth = down)
- Flipped 180° in X and Y axes

## Next Steps

1. ✅ **Fixed**: Coordinate frame transformation bugs
2. ✅ **Fixed**: Visualization showing calibration hack
3. ⚠️ **Test**: Run system and observe behavior
4. 📋 **Optional**: Empirically tune `DISTANCE_TO_REMAIN_M` if needed
5. 🔄 **Recommended**: Recalibrate hand-eye matrix with correct TCP to eliminate hacks

## Files Modified

- `src/states/unified_hand_tracking_state.py` - Fixed coordinate transformations
- `src/hand_detection/hand_detection_module.py` - Enhanced visualization
- `debug_hand_tcp_coordinates.py` - New diagnostic tool
- `test_hand_tracking_coordinates.py` - New testing tool
- `HAND_TRACKING_COORDINATE_FIX.md` - This document

## Technical Notes

### Why the Calibration Hacks Exist

**Z Offset (-138mm)**: The hand-eye calibration was performed with `tool0` (link 7) as the TCP, but the actual gripper TCP is at `link 9`, which is 138mm lower. The hack subtracts this offset from the Z coordinate to compensate.

**Y Offset (-64mm)**: The camera is mounted approximately 64mm off-center from the gripper TCP in the Y direction. When the hand is centered under the gripper, the camera sees it at Y≈0, which transforms to Y≈0.064m in TCP frame. The hack subtracts this offset so Y=0 represents the hand being centered under the gripper TCP (not the camera).

Combined, these offsets transform the TCP coordinates to be relative to the actual gripper TCP position:

```python
hand_pos_tcp[1] -= 0.064  # Center on gripper TCP (Y axis)
hand_pos_tcp[2] -= 0.138  # Correct for gripper extension (Z axis)
```

### Proper Solution

Recalibrate using:

```python
# In calibration code, ensure you're using:
tcp_link = 'link_9'  # Not 'tool0' or 'link_7'
```

This will produce a hand-eye matrix that directly gives coordinates relative to the actual gripper TCP, eliminating the need for hacks.
