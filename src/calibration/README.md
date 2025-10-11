# Calibration Module

This directory contains all hand-eye calibration functionality for the KUKA LBR iiwa 14 robot with RealSense camera.

## 📁 Directory Structure

```
calibration/
├── calibration_config.py          # Configuration parameters
├── hand_eye_calibrator.py          # Core hand-eye calibration logic
├── pose_generator.py               # Robot pose generation for calibration
├── run_calibrator.py               # Main calibration runner
├── semi_automated_calibration.py   # Semi-automated workflow
├── scripts/                        # Top-level executable scripts
│   ├── run_semi_automated_calibration.py
│   └── run_manual_calibration.py
└── utilities/                      # Analysis and helper tools
    ├── analyze_hand_eye_matrix.py
    ├── analyze_yaw_gaps.py
    ├── extract_all_pose_matrices.py
    ├── generate_matrix_best_poses.py
    └── generate_matrix_from_images.py
```

## 🚀 Quick Start

### Running Calibration

From the project root directory:

```bash
# Semi-automated calibration (recommended)
.\venv\Scripts\python.exe src\calibration\scripts\run_semi_automated_calibration.py --mode real --poses 15

# Manual calibration
.\venv\Scripts\python.exe src\calibration\scripts\run_manual_calibration.py
```

### Analyzing Calibration Data

```bash
# Analyze yaw diversity gaps
.\venv\Scripts\python.exe src\calibration\utilities\analyze_yaw_gaps.py

# Extract all pose matrices
.\venv\Scripts\python.exe src\calibration\utilities\extract_all_pose_matrices.py

# Generate matrix from best poses
.\venv\Scripts\python.exe src\calibration\utilities\generate_matrix_best_poses.py
```

## 📋 Core Files

### `calibration_config.py`

Configuration parameters for calibration:

- Checkerboard specifications (9x6, 8mm squares)
- Camera intrinsics
- Workspace constraints
- Pose generation parameters

### `hand_eye_calibrator.py`

Core hand-eye calibration implementation:

- OpenCV `calibrateHandEye` wrapper
- Park method (primary)
- Quality analysis and validation
- Matrix generation and saving

### `pose_generator.py`

Generates robot poses for calibration:

- TCP-centric pose generation
- Workspace constraint validation
- IK feasibility checking
- Pose diversity optimization

### `semi_automated_calibration.py`

Semi-automated calibration workflow:

- Automatic pose generation
- Robot movement coordination
- Image capture and storage
- Data persistence and recovery

## 🔧 Utility Scripts

### `analyze_hand_eye_matrix.py`

Analyzes hand-eye calibration matrix:

- Translation magnitude validation
- Rotation matrix properties
- Physical plausibility checks

### `analyze_yaw_gaps.py`

Identifies gaps in yaw orientation coverage:

- Yaw distribution analysis
- Gap detection and prioritization
- Recommendations for new poses

### `extract_all_pose_matrices.py`

Extracts transformation matrices from all calibration poses:

- TCP position and orientation
- Joint angles
- Pose diversity metrics
- JSON export for analysis

### `generate_matrix_best_poses.py`

Generates hand-eye matrix from high-quality poses only:

- Pose quality scoring (reprojection error, diversity, workspace fitness)
- Automatic pose pruning
- Park method calibration
- Improved calibration accuracy

### `generate_matrix_from_images.py`

Generates hand-eye matrix from existing captured images:

- Loads all calibration sessions
- Processes checkerboard detections
- Computes calibration matrix
- Quality analysis

## 📊 Current Calibration Status

**Method**: Park algorithm  
**Poses used**: 13 best poses (pruned from 23 total)  
**Translation**: 0.280m (28cm camera-to-TCP distance)  
**Mean calibration error**: 0.647 (excellent)  
**Matrix location**: `src/hand_eye_matrix.npy`

## 🎯 Calibration Data

Calibration images and data are stored in:

- `calibration_captures/` - Root directory for all calibration sessions
- `calibration_captures/YYYYMMDD_HHMMSS/` - Timestamped session directories
- Each pose directory contains:
  - `color.png` - Color image of checkerboard
  - `depth.png` - Depth image
  - `joint_angles.json` - Robot joint angles
  - `metadata.json` - Capture metadata

## 📝 Pose Quality Metrics

The calibration uses multiple quality metrics:

1. **Reprojection error**: Checkerboard corner detection accuracy
2. **Corner quality**: Corner refinement quality
3. **Pose diversity**: Distance from other poses in configuration space
4. **Workspace fitness**: TCP position validity

## 🔍 Known Issues & Gaps

**Yaw Orientation Gaps** (as of latest analysis):

- Gap 1: 59.9° between 49.2° and 109.2° (priority)
- Gap 2: 54.4° between -123.2° and -68.7°
- Gap 3: 41.5° between -164.8° and -123.3°

**Recommended missing yaw angles**:

- 79.2° (highest priority)
- -95.9°
- -144.1°
- 75.0°
- -75.0°
- 135.0°

## 🧹 Recent Cleanup (2025-10-08)

Removed obsolete files:

- `prune_calibration_poses.py` (replaced by utilities/generate_matrix_best_poses.py)
- `generate_pruned_matrix.py` (duplicate functionality)
- `test_calibration_integration.py` (replaced by src/test_hand_eye_integration.py)
- `test_improved_pose_generation.py` (debug file)
- `test_camera_direction.py` (debug file)
- `debug_*.py` files (various debug scripts)
- `manual_calibration.py` (unused old approach)
- `calibration_validator.py` (unused)
- `advanced_pose_generator.py` (unused complex version)
- `sweep_hand_eye_variants.py` (experimental)

## 📚 References

- OpenCV Hand-Eye Calibration: https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#gaebfc1c9f7434196a374c382abf43439b
- Park et al. method: "Robot Sensor Calibration: Solving AX = XB on the Euclidean Group"
- KUKA LBR iiwa 14 specifications: https://www.kuka.com/en-us/products/robot-systems/industrial-robots/lbr-iiwa
