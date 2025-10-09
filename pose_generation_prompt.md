# Robot Hand-Eye Calibration Pose Generation Prompt

## Scenario Context

You are tasked with generating diverse robot poses for hand-eye calibration of a KUKA LBR iiwa 14 robot arm with a RealSense camera mounted on the TCP (Tool Center Point). The goal is to create poses that provide maximum diversity in orientation, particularly in yaw angles, while ensuring the camera can clearly see a checkerboard pattern.

## Current Calibration Setup

### Robot Configuration

- **Robot**: KUKA LBR iiwa 14 (7-DOF manipulator)
- **Camera**: Intel RealSense mounted on TCP
- **Target**: 9x6 checkerboard with 8mm square size
- **Calibration method**: Hand-eye calibration using OpenCV's Park algorithm

### Current Hand-Eye Matrix (Camera to TCP)

```
HAND_EYE_MATRIX = [
    [-0.9996,  0.0258,  0.0120,  0.0009],
    [-0.0284, -0.9361, -0.3507, -0.0683],
    [ 0.0022, -0.3509,  0.9364,  0.2714],
    [ 0.0000,  0.0000,  0.0000,  1.0000]
]
```

- Translation: 0.280m (28cm camera-to-TCP distance)
- This matrix transforms points from camera frame to TCP frame

### Checkerboard Target

- **Pattern**: 9x6 internal corners
- **Square size**: 8mm (0.008m)
- **Typical position**: Around [0.4, 0.0, 0.22] in robot base frame
- **Size**: ~72mm x 48mm

## Current Pose Analysis

### Existing Poses (23 total)

**Position Coverage:**

- X range: -0.158 to 0.665 m
- Y range: -0.443 to 0.326 m
- Z range: 0.032 to 0.803 m

**Orientation Coverage:**

- Roll range: -173.8° to 174.8° (good coverage)
- Pitch range: -48.4° to 9.6° (limited range)
- Yaw range: -179.5° to 160.2° (good range but has gaps)

### Current Yaw Distribution

The existing poses have the following yaw angles (degrees):

```
[-164.8, -167.4, -68.7, 0.5, -123.3, 26.0, -1.4, -1.4, 49.2, 1.0,
 -49.2, 160.2, 24.8, 133.0, 109.2, -179.5, 44.9, -123.2, 18.3,
 -24.9, 17.1, 18.3, 120.0]
```

**Issues Identified:**

- Largest yaw gap: 59.9° (significant gap detected)
- Some clustering around 0° and 18° yaw angles
- Missing orientations in certain yaw ranges

## Sample Current Poses

Here are examples of current valid poses:

### Pose 1 (High quality)

- **TCP Position**: [0.066, 0.296, 0.710] m
- **TCP Orientation**: Roll=-173.8°, Pitch=-48.4°, Yaw=-164.8°
- **Joint Angles**: [-2.967, 0.215, -2.143, -1.161, -0.027, 2.094, 0.894]

### Pose 2 (Good diversity)

- **TCP Position**: [0.665, -0.443, 0.032] m
- **TCP Orientation**: Roll=174.8°, Pitch=9.6°, Yaw=160.2°
- **Joint Angles**: [2.967, -0.215, 2.143, 1.161, 0.027, -2.094, -0.894]

### Pose 3 (Medium position)

- **TCP Position**: [0.400, 0.000, 0.220] m
- **TCP Orientation**: Roll=0.0°, Pitch=-20.0°, Yaw=0.0°
- **Joint Angles**: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

## Requirements for New Poses

### 1. Yaw Diversity Priority

**Primary Goal**: Generate poses with yaw angles that fill the identified gaps:

- Target yaw angles: -150°, -120°, -90°, -60°, -30°, 30°, 60°, 90°, 120°, 150°
- Avoid clustering around existing yaw values
- Ensure good distribution across the full -180° to +180° range

### 2. Position Constraints

- **X range**: -0.2 to 0.7 m (relative to robot base)
- **Y range**: -0.5 to 0.4 m (workspace limits)
- **Z range**: 0.1 to 0.8 m (above table, below ceiling)
- **Distance to target**: 0.15 to 0.5 m (optimal for checkerboard visibility)

### 3. Orientation Constraints

- **Roll**: Full range -180° to +180° (good current coverage)
- **Pitch**: -60° to +30° (expand from current -48° to +10° range)
- **Yaw**: Focus on filling gaps, especially -150° to -120°, -90° to -60°, -30° to +30°, +60° to +90°, +120° to +150°

### 4. Physical Constraints

- **Joint limits**: All 7 joints must be within KUKA LBR iiwa 14 limits
- **Collision avoidance**: TCP must not collide with table, ceiling, or robot itself
- **Reachability**: TCP must be reachable by the 7-DOF arm
- **Camera visibility**: Camera optical axis should point toward checkerboard target

### 5. Calibration Quality

- **Viewing angle**: Camera should view checkerboard at 15° to 75° incidence angle
- **Distance**: 0.2 to 0.4 m from checkerboard center for good corner detection
- **Lighting**: Avoid extreme angles that might cause glare or shadows
- **Stability**: Poses should be stable and repeatable

## Specific Yaw Gap Targets

Based on the analysis, prioritize generating poses with these yaw angles:

1. **-150° ± 10°** (fill gap between -164.8° and -123.3°)
2. **-90° ± 10°** (fill gap between -123.3° and -68.7°)
3. **-30° ± 10°** (fill gap around -30° range)
4. **+90° ± 10°** (fill gap between +49.2° and +109.2°)
5. **+150° ± 10°** (fill gap between +133.0° and +160.2°)

## Expected Output Format

For each generated pose, provide:

1. **TCP Position**: [x, y, z] in meters relative to robot base
2. **TCP Orientation**: [roll, pitch, yaw] in degrees (XYZ Euler angles)
3. **Rationale**: Brief explanation of why this pose improves diversity
4. **Yaw target**: Which specific yaw gap this pose addresses

## Quality Metrics to Achieve

- **Yaw coverage**: Reduce largest yaw gap from 59.9° to <30°
- **Yaw distribution**: Achieve standard deviation >80° for yaw angles
- **Overall diversity**: Increase orientation standard deviation while maintaining position feasibility
- **Calibration robustness**: Ensure poses contribute to well-conditioned hand-eye calibration

## Additional Context

The current hand-eye calibration shows excellent quality:

- Mean calibration error: 0.647 (very good)
- Translation magnitude: 0.280m (physically reasonable)
- Rotation matrix: Properly orthogonal

The goal is to maintain or improve this quality while significantly increasing pose diversity, particularly in yaw orientation, to create a more robust calibration dataset.

---

**Task**: Generate 10-15 new robot poses that specifically address the yaw diversity gaps identified in the current dataset, while maintaining all physical and calibration constraints.
