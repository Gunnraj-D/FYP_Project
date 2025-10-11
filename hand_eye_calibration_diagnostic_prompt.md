# Hand-Eye Calibration Diagnostic Prompt

## Problem Summary

I'm performing hand-eye calibration for a KUKA LBR iiwa 14 robot with a RealSense camera mounted on the TCP (Tool Center Point). The calibration is producing a translation magnitude of **4.7cm**, but I believe the camera is actually about **10cm** away from the TCP. I need help diagnosing potential issues.

## Hardware Setup

### Robot Configuration

- **Robot**: KUKA LBR iiwa 14 R820 (7-DOF collaborative robot)
- **End Effector**: Robotiq 2F-85 gripper
- **Camera**: Intel RealSense D435 (RGB-D camera)
- **Camera mounting**: Fixed to the TCP/gripper, facing downward
- **Estimated camera-to-TCP distance**: ~10cm (visual estimate)

### Calibration Target

- **Pattern**: Checkerboard with 9x6 internal corners
- **Square size**: 8mm (0.008m) - recently corrected from 0.08m
- **Physical size**: Approximately 72mm x 48mm
- **Typical position**: On table at approximately [0.4, 0.025, 0.22] meters in robot base frame

## Calibration Data

### Captured Poses

- **Total poses captured**: 32
- **Unique poses**: 31 (1 duplicate detected and removed)
- **Valid poses with checkerboard detected**: 31
- **Sessions**: 2 separate capture sessions (20251010_205126 and 20251010_215058)

### Pose Quality Metrics

**Top 15 Best Poses (by reprojection error):**

```
Rank  Pose                                  Reprojection Error (pixels)
1     20251010_215058/pose_015             0.0093
2     20251010_215058/pose_016             0.0101
3     20251010_215058/pose_012             0.0126
4     20251010_215058/pose_014             0.0136
5     20251010_215058/pose_002             0.0158
6     20251010_205126/pose_012             0.0168
7     20251010_205126/pose_013             0.0174
8     20251010_215058/pose_004             0.0174
9     20251010_215058/pose_001             0.0180
10    20251010_205126/pose_003             0.0183
11    20251010_215058/pose_003             0.0192
12    20251010_215058/pose_011             0.0194
13    20251010_205126/pose_011             0.0196
14    20251010_215058/pose_013             0.0205
15    20251010_205126/pose_015             0.0208
```

**Worst 5 Poses:**

```
27    20251010_205126/pose_009             0.0857
28    20251010_215058/pose_008             0.0954
29    20251010_215058/pose_007             0.2622
30    20251010_215058/pose_009             0.3011
31    20251010_215058/pose_005             0.4030
```

## Calibration Results

### Matrix from Top 15 Poses (Park Method)

```
Translation magnitude: 0.0476 m (4.76 cm)
Rotation determinant: 1.000000 (perfect)

Hand-Eye Matrix (Camera to TCP):
[ -0.9997 -0.0168 -0.0168  0.0069]
[  0.0218 -0.9331 -0.3589  0.0469]
[ -0.0096 -0.3591  0.9332 -0.0041]
[  0.0000  0.0000  0.0000  1.0000]
```

### Matrix from All 31 Unique Poses (Park Method)

```
Translation magnitude: 0.0471 m (4.71 cm)
Rotation determinant: 1.000000 (perfect)

Hand-Eye Matrix (Camera to TCP):
[ -0.9997 -0.0169 -0.0164  0.0064]
[  0.0216 -0.9329 -0.3594  0.0464]
[ -0.0092 -0.3597  0.9330 -0.0052]
[  0.0000  0.0000  0.0000  1.0000]
```

### Consistency

- Both calibrations produce **very similar results** (4.71cm vs 4.76cm)
- This suggests the calibration is **stable and consistent**
- The poor quality poses don't significantly affect the result

## Camera Parameters Used

### Intrinsics (from RealSense)

```python
Color stream: 640x480 @ 30fps
fx = 608.7
fy = 607.3
ppx = 320.0 (principal point X)
ppy = 240.0 (principal point Y)
```

### Approximation Used in Calibration

```python
camera_matrix = [
    [615.0,   0.0, 320.0],
    [  0.0, 615.0, 240.0],
    [  0.0,   0.0,   1.0]
]
dist_coeffs = [0, 0, 0, 0]  # Assuming no distortion
```

## Calibration Method

- **Algorithm**: OpenCV `cv2.calibrateHandEye()` with Park method (`cv2.CALIB_HAND_EYE_PARK`)
- **Convention**: Eye-in-hand (camera on robot TCP)
- **Coordinate frames**:
  - Robot base frame: Fixed reference frame
  - TCP frame: Moves with robot end effector
  - Camera frame: Fixed relative to TCP
  - Checkerboard frame: Detected in camera view using `cv2.solvePnP()`

## Sample Pose Data

### Example Good Pose (pose_015, reprojection error 0.0093):

**Joint angles (radians):**

```
[-0.360, 0.607, -0.054, -1.305, -2.919, -1.452, -1.127]
```

### Example Another Pose (pose_016, reprojection error 0.0101):

**Joint angles (radians):**

```
[-0.109, 0.704, -0.372, -1.238, -2.876, -1.730, 1.140]
```

These show **significant variation** in joint configurations, suggesting good pose diversity.

## Previous Calibration History

### Earlier Calibration (from different dataset)

- Used 13 best poses (pruned from 18)
- **Translation**: 0.280m (28cm)
- **Mean calibration error**: 0.647
- Method: Park

### Issue with Previous Data

- The 28cm result was considered too large
- Led to recalibration with new dataset
- Current dataset shows 4.7cm instead

## Potential Issues to Diagnose

### 1. Checkerboard Square Size

- **Current setting**: 8mm (0.008m)
- **History**: Was incorrectly set to 80mm (0.08m) initially, causing 10x scale error
- **Question**: Could there still be a unit conversion error somewhere?

### 2. Camera Intrinsics

- Using approximate intrinsics (fx=fy=615)
- **Question**: Should we perform proper camera calibration first?

### 3. Measurement Uncertainty

- Visual estimate of 10cm camera-to-TCP distance
- **Question**: Could the actual distance be closer to 5cm?
- **Note**: Camera lens is recessed in housing, TCP is defined at gripper flange

### 4. Coordinate Frame Conventions

- **Camera optical axis**: Assumed to be +Z direction in camera frame
- **TCP definition**: At gripper flange center
- **Question**: Are we measuring from the correct reference points?

### 5. Calibration Algorithm Behavior

- Park method showing 4.7cm with excellent consistency
- Rotation matrix is perfectly orthogonal
- Reprojection errors are sub-pixel for top poses
- **Question**: Is the algorithm correctly solving for eye-in-hand calibration?

## Data Integrity

### What We've Verified

✅ Checkerboard is detected in all 31 unique poses
✅ Reprojection errors are reasonable (best: 0.009 pixels)
✅ Joint angles vary significantly between poses (good diversity)
✅ Rotation matrix has perfect determinant (1.000000)
✅ Results are consistent between different pose subsets
✅ One duplicate was detected and removed (joint diff < 0.05 rad)

### What's Concerning

⚠️ Translation magnitude (4.7cm) is ~50% of expected (10cm)
⚠️ Large jump from previous calibration (28cm → 4.7cm)
⚠️ Using approximate camera intrinsics, not calibrated
⚠️ Assuming zero distortion coefficients

## Calibration Code Used

### PnP Solve (per pose)

```python
objp = np.zeros((54, 3), np.float32)  # 9x6 = 54 corners
objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)
objp *= 0.008  # 8mm square size

ret, rvec, tvec = cv2.solvePnP(
    objp, corners, camera_matrix, dist_coeffs
)
R_target2cam, _ = cv2.Rodrigues(rvec)
t_target2cam = tvec.flatten()
```

### Hand-Eye Calibration

```python
R_gripper2base_array = np.array(R_gripper2base_list)  # Shape: (31, 3, 3)
t_gripper2base_array = np.array(t_gripper2base_list).reshape(-1, 3, 1)  # Shape: (31, 3, 1)
R_target2cam_array = np.array(R_target2cam_list)  # Shape: (31, 3, 3)
t_target2cam_array = np.array(t_target2cam_list).reshape(-1, 3, 1)  # Shape: (31, 3, 1)

result = cv2.calibrateHandEye(
    R_gripper2base_array, t_gripper2base_array,
    R_target2cam_array, t_target2cam_array,
    method=cv2.CALIB_HAND_EYE_PARK
)

R_cam2gripper = result[0]  # Rotation: Camera to TCP
t_cam2gripper = result[1]  # Translation: Camera to TCP
```

## Questions for Diagnosis

1. **Is 4.7cm plausible?** Could the camera actually be only ~5cm from the TCP, not 10cm?

2. **Checkerboard scale**: We're using 8mm squares. Is there any way this could still be causing a scale error?

3. **Camera intrinsics**: How much error could the approximate intrinsics (fx=fy=615 vs actual fx=608.7, fy=607.3) introduce?

4. **TCP definition**: Is the TCP defined at the gripper flange, or somewhere else? Could there be a 5cm offset we're not accounting for?

5. **Algorithm correctness**: With excellent reprojection errors and perfect rotation determinant, is it possible the algorithm is correct and our visual estimate is wrong?

6. **Previous 28cm result**: Why was the previous calibration so different? Was that the incorrect one?

## What Would Help

- Validation approaches to verify the 4.7cm is correct or incorrect
- Physical measurement techniques to accurately measure camera-to-TCP distance
- Diagnostic tests to run on the calibration data
- Common pitfalls in hand-eye calibration that could explain this discrepancy
- Whether the excellent reprojection errors indicate the calibration is trustworthy despite the unexpected translation magnitude

## Additional Context

The calibration process has been iterative:

1. Initial calibration with 80mm squares → 1.4m translation (clearly wrong, 10x scale error)
2. Corrected to 8mm squares → 28cm translation (seemed too large)
3. New dataset with manual captures → 4.7cm translation (seems too small)

All three calibrations showed good rotation matrices and low reprojection errors, yet wildly different translations. This suggests a systematic issue rather than random noise.
