# Manual Calibration - Fixed Issues

## ✅ Issues Fixed

### 1. **CameraManager intrinsics attribute error**

**Problem**: Semi-automated calibration was trying to access `self.camera_manager.intrinsics` which doesn't exist.

**Root Cause**: `CameraManager` has three separate intrinsics attributes:

- `color_intrinsics` - Color stream intrinsics
- `depth_intrinsics` - Depth stream intrinsics
- `aligned_color_intrinsics` - Aligned depth intrinsics (same as color)

**Fix**: Created new manual calibration script that uses `self.camera_manager.color_intrinsics` correctly.

### 2. **Manual calibration saves images with joint positions**

**Problem**: Need to capture calibration images manually with corresponding joint angles.

**Solution**: Created `src/calibration/manual_calibration.py` that:

- ✅ Displays live camera feed with joint angles overlay
- ✅ Captures on SPACE key press
- ✅ Saves for each capture:
  - `color.png` - Color image
  - `depth.png` - Depth image
  - `joint_angles.json` - Current robot joint angles (7 DOF)
  - `metadata.json` - Timestamp, capture number, camera intrinsics
- ✅ Creates timestamped session directories (`YYYYMMDD_HHMMSS`)
- ✅ Shows capture count and current joints in real-time

### 3. **Import path fixes**

**Problem**: Scripts couldn't find modules after reorganization.

**Fix**: Updated both runner scripts:

- `src/calibration/scripts/run_semi_automated_calibration.py`
- `src/calibration/scripts/run_manual_calibration.py`

To properly add parent directories to Python path.

### 4. **CalibrationValidator removed**

**Problem**: `hand_eye_calibrator.py` was importing deleted `CalibrationValidator`.

**Fix**:

- Removed import
- Created inline validation in `_validate_calibration()`
- Simplified report generation in `_generate_calibration_report()`

## 📋 Manual Calibration Usage

### Running Manual Calibration

```bash
# With real robot
.\venv\Scripts\python.exe src\calibration\scripts\run_manual_calibration.py --mode real

# With mock data (testing)
.\venv\Scripts\python.exe src\calibration\scripts\run_manual_calibration.py --mode mock
```

### Controls

- **SPACE**: Capture current pose (image + joints)
- **Q / ESC**: Quit and finish calibration

### Visual Feedback

- Live camera feed with overlay showing:
  - Number of captures
  - Current joint angles (J1-J7)
  - Instructions
- Green flash on successful capture
- Error messages on failure

### Output Structure

```
calibration_captures/YYYYMMDD_HHMMSS/
├── pose_001/
│   ├── color.png
│   ├── depth.png
│   ├── joint_angles.json
│   └── metadata.json
├── pose_002/
│   ├── color.png
│   ├── depth.png
│   ├── joint_angles.json
│   └── metadata.json
...
```

### Saved Data Format

**joint_angles.json**:

```json
[
  -2.967, // J1
  0.215, // J2
  -2.143, // J3
  -1.161, // J4
  -0.027, // J5
  2.094, // J6
  0.894 // J7
]
```

**metadata.json**:

```json
{
  "timestamp": 1728589992.123,
  "capture_number": 1,
  "mode": "real",
  "joint_angles": [-2.967, 0.215, ...],
  "camera_intrinsics": {
    "width": 640,
    "height": 480,
    "fx": 608.7,
    "fy": 607.3,
    "ppx": 320.0,
    "ppy": 240.0
  }
}
```

## 🎯 Recommended Calibration Strategy

Based on yaw gap analysis, prioritize these orientations:

1. **Yaw 79.2°** - Fills largest 59.9° gap
2. **Yaw -95.9°** - Fills 54.4° gap
3. **Yaw -144.1°** - Fills 41.5° gap
4. **Yaw 75.0°** - Empty segment
5. **Yaw -75.0°** - Sparse coverage
6. **Yaw 135.0°** - Sparse coverage

### Tips for Good Poses

- Vary yaw angle as much as possible (prioritize gaps above)
- Maintain 0.2-0.4m distance from checkerboard
- Keep checkerboard fully visible in frame
- Avoid extreme angles that cause glare
- Ensure stable robot positions
- Capture at least 10-15 poses for good calibration

## 🔧 Next Steps After Capturing

1. **Generate matrix from captured poses**:

   ```bash
   .\venv\Scripts\python.exe src\calibration\utilities\generate_matrix_from_images.py
   ```

2. **Or generate from best poses only**:

   ```bash
   .\venv\Scripts\python.exe src\calibration\utilities\generate_matrix_best_poses.py
   ```

3. **Analyze yaw diversity**:

   ```bash
   .\venv\Scripts\python.exe src\calibration\utilities\analyze_yaw_gaps.py
   ```

4. **Update config with new matrix**:
   - Copy generated matrix to `src/hand_eye_matrix.npy`
   - Update `HAND_EYE_MATRIX` in `src/config/config.py`

## ✅ All Fixed Components

- ✅ Manual calibration script with live preview
- ✅ Joint angle capture and saving
- ✅ Correct camera intrinsics usage
- ✅ Timestamped session directories
- ✅ Comprehensive metadata saving
- ✅ Visual feedback and controls
- ✅ Error handling and cleanup
- ✅ Mock mode for testing

The manual calibration is now fully functional and ready to use!
