# Grasp Coordinate Transformation Diagnostic

## 🔴 Problem Observed

Robot attempting to move to **(0, 0, 0.225)** which is directly above the robot base center.

This causes:

- ❌ Near-singularity (robot can't efficiently reach directly above itself)
- ❌ Large joint movements (>120° rotations)
- ❌ Joint limit violations
- ❌ Dangerous collision paths

---

## 🔍 Root Cause Analysis

The position **(0, 0, 0.225)** suggests:

1. **Hand-eye calibration is wrong** → Camera-to-base transform is identity or incorrect
2. **Grasp detected at wrong location** → Network predicting center of image
3. **TCP matrix is incorrect** → Current robot pose not being used properly

---

## 📊 Diagnostic Steps

### Step 1: Check Hand-Eye Calibration

```python
# In your main code or debug script
from config import HAND_EYE_MATRIX, CAMERA_TRANSFORM_MODE

print("Camera transform mode:", CAMERA_TRANSFORM_MODE)
print("Hand-eye matrix:")
print(HAND_EYE_MATRIX)

# Expected: Should NOT be identity matrix
# Should show camera offset from TCP
```

**Expected values:**

- Camera should be offset from TCP (not at origin)
- Typically camera is 0.1-0.3m away from gripper
- Matrix should have translation components

**If matrix is identity or near-zero → Hand-eye calibration needs redoing!**

---

### Step 2: Check Grasp Detection in Camera Frame

Add logging to `grasp_detector_module.py` around line 289:

```python
# After line 289: logger.info(f"Generated camera pose: {grasp_pose_camera}")
logger.info(f"🔍 DIAGNOSTIC - Grasp in CAMERA frame:")
logger.info(f"  Position: {grasp_pose_camera[:3]}")
logger.info(f"  Expected: X and Y should be NON-ZERO if object off-center")
logger.info(f"  Z (depth) should be 0.3-1.0m typically")

# After line 304: target_position = transform_camera_to_base(...)
logger.info(f"🔍 DIAGNOSTIC - Grasp in BASE frame:")
logger.info(f"  Position: {target_position}")
logger.info(f"  Expected: Should be in robot workspace, NOT at (0,0,Z)")
logger.info(f"  Typical X: 0.3-0.7m, Y: -0.3 to 0.3m, Z: 0.1-0.5m")

# Also log TCP matrix
logger.info(f"🔍 DIAGNOSTIC - TCP matrix translation:")
logger.info(f"  TCP position: [{tcp_matrix[0,3]:.3f}, {tcp_matrix[1,3]:.3f}, {tcp_matrix[2,3]:.3f}]")
```

**What to look for:**

- Camera frame position should show object location relative to camera
- If grasp in camera frame is (0, 0, depth) → **Object detection problem**
- If grasp in camera frame is OK but base frame is (0,0,Z) → **Transform problem**

---

### Step 3: Check Current TCP Position

The robot might be in a bad starting configuration:

```python
# Check where the TCP currently is
current_joints = telemetry.get_current_joints()
tcp_matrix, tcp_pose = kinematics_solver.tcp_from_joints(current_joints)

print("Current TCP position:", tcp_pose[:3])
print("Current TCP orientation (RPY):", tcp_pose[3:])

# TCP should be somewhere reasonable, not at origin
```

---

### Step 4: Verify Camera Frame Grasp Detection

Run the visualization script to see what the network is predicting:

```bash
python visualize_grconvnet_temporal.py
```

**Check:**

- Is the grasp rectangle on the object?
- Or is it at the center of the image (0,0)?
- Quality values reasonable?

---

## 🔧 Quick Fixes

### Fix 1: Check Camera Transform Mode

```python
# In src/config/camera_config.py or via set_camera_transform_mode()
from config import set_camera_transform_mode, print_camera_transform_info

print_camera_transform_info()

# Should show:
# "Camera Transform Mode: calibrated"
# NOT "simple" or other modes
```

If not using calibrated mode:

```python
set_camera_transform_mode('calibrated')
```

---

### Fix 2: Verify Hand-Eye Matrix File

```bash
# Check if calibration file exists
ls src/hand_eye_matrix.npy

# Or in project root
ls hand_eye_matrix.npy
```

If missing → You need to run hand-eye calibration!

---

### Fix 3: Check Object Detection Quality

The network might be predicting grasps at image center (0,0) due to:

- No valid object detected
- Object too far/close
- Poor lighting
- Camera not seeing workspace

**Test:**
Place a clear, contrasting object in camera view and check visualization.

---

## 🎯 Expected vs Actual

### Expected Grasp Pose Flow:

```
1. Object detected in camera at:
   Camera frame: (0.15, -0.05, 0.45)  ← Off-center, reasonable depth

2. Transform with TCP at:
   TCP position: (0.5, 0.1, 0.4)  ← Current robot position

3. Result in base frame:
   Base frame: (0.62, 0.08, 0.35)  ← Reachable workspace position

4. IK solution:
   Joint changes: <30° per joint  ← Smooth motion
```

### Actual (Broken):

```
1. Object detected in camera at:
   Camera frame: (0.0, 0.0, 0.225)?  ← Suspicious, might be at origin

2. Transform with TCP at:
   TCP position: Unknown

3. Result in base frame:
   Base frame: (0.0, 0.0, 0.225)  ← AT ROBOT BASE! ❌

4. IK solution:
   Joint changes: >120° per joint  ← DANGEROUS ❌
   Joint limits violated  ← UNREACHABLE ❌
```

---

## 🚨 Immediate Action

**Before running again:**

1. **Add diagnostic logging** (Step 2 above)
2. **Run visualization** to verify object detection
3. **Check hand-eye matrix** exists and is not identity
4. **Verify camera transform mode** is 'calibrated'
5. **Check robot starting pose** is reasonable

**Do NOT run the robot** until you verify:

- ✅ Grasp in camera frame is reasonable (not at origin)
- ✅ Grasp in base frame is in workspace (X: 0.3-0.7m, not 0.0!)
- ✅ Joint movements are <30° per joint
- ✅ No joint limit violations

---

## 🔬 Advanced Diagnostics

### Check Transform Chain:

```python
# Add this to grasp_detector_module.py after line 304

# 1. Print grasp in camera frame
logger.info(f"Grasp in camera frame: {grasp_position_camera}")

# 2. Print TCP transformation matrix
logger.info(f"TCP matrix used for transform:")
logger.info(f"{tcp_matrix}")

# 3. Print hand-eye matrix
from config import HAND_EYE_MATRIX
logger.info(f"Hand-eye matrix:")
logger.info(f"{HAND_EYE_MATRIX}")

# 4. Manual transform test
import numpy as np
grasp_homogeneous = np.array([*grasp_position_camera, 1.0])
manual_transform = tcp_matrix @ HAND_EYE_MATRIX @ grasp_homogeneous
logger.info(f"Manual transform result: {manual_transform[:3]}")
logger.info(f"transform_camera_to_base result: {target_position}")
```

These should match! If they don't → bug in `transform_camera_to_base()`

---

## 📝 Expected Log Output (Working System)

```
🔍 DIAGNOSTIC - Grasp in CAMERA frame:
  Position: [0.123, -0.045, 0.487]  ← Object off-center ✓
  Expected: X and Y should be NON-ZERO if object off-center
  Z (depth) should be 0.3-1.0m typically ✓

🔍 DIAGNOSTIC - TCP matrix translation:
  TCP position: [0.523, 0.087, 0.412]  ← Robot arm extended ✓

🔍 DIAGNOSTIC - Grasp in BASE frame:
  Position: [0.612, 0.065, 0.334]  ← In workspace ✓
  Expected: Should be in robot workspace, NOT at (0,0,Z)
  Typical X: 0.3-0.7m, Y: -0.3 to 0.3m, Z: 0.1-0.5m ✓

Target joint angles: [-0.523, 1.234, -0.891, ...]
Joint movements: [0.15, 0.23, 0.11, ...]  ← Small changes ✓
```

---

## 🎯 Summary

**Most likely cause:** Hand-eye calibration matrix is wrong or not being loaded.

**How to verify:**

1. Check `CAMERA_TRANSFORM_MODE == 'calibrated'`
2. Check `HAND_EYE_MATRIX` is not identity
3. Add diagnostic logging to see camera vs base frame positions
4. Verify object detection is working (not predicting at image center)

**Don't run robot until target position is in workspace (X > 0.3m, not 0.0m)!**

---

## 📚 Related Files

- `src/camera_management/camera_transform_module.py` - Transformation functions
- `src/config/camera_config.py` - Hand-eye matrix configuration
- `src/object_detection/grasp_detector_module.py` - Where transform happens
- `hand_eye_matrix.npy` - Calibration file (should exist!)

