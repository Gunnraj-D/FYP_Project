# AI Prompt: Hand Occlusion Stabilization for Robot Handoff

## Mission

Provide a **step-by-step implementation plan** to stabilize hand tracking during partial occlusion in a robot-to-human handoff scenario. The solution must prevent Z-coordinate oscillations while maintaining X/Y tracking accuracy.

**Critical Requirement:** Your response must include **exact, copy-paste ready code modifications** with file paths, line numbers, and complete function implementations that another AI agent can implement without ambiguity.

---

## System Overview

### Hardware & Software Stack

- **Robot**: KUKA iiwa 7-DOF collaborative robot
- **Camera**: Intel RealSense D435 RGB-D (mounted on gripper end-effector)
- **Hand Detection**: MediaPipe Hand Landmarker (21 landmarks per hand)
- **Control Loop**: 10Hz (~100ms cycle time)
- **Language**: Python 3.x with numpy, OpenCV

### Coordinate Frames

```
Camera Frame (RealSense):
  +X = Right (in image)
  +Y = Down (in image)
  +Z = Forward (depth into scene)

Robot TCP Frame (gripper facing down):
  +X = Forward (gripper approach direction)
  +Y = Left (across gripper width)
  +Z = Down (gripper opening direction)

Hand-Eye Calibration:
  HAND_EYE_MATRIX = 4x4 transformation (camera → TCP)
  Translation: [13.6mm, 63.9mm, 14.1mm] (camera offset from TCP)
  Rotation: Camera axes ≈ inverted from TCP axes
```

### Current Implementation Status

#### Already Implemented ✅

1. **Hand Detection Module** (`src/hand_detection/hand_detection_module.py`):

   - MediaPipe palm detection from 7 landmark indices
   - Single-pixel depth measurement using `camera_manager.get_average_depth()`
   - Simple outlier filter (rejects if >20cm from 5-frame average)
   - Transforms camera coords → TCP coords using hand-eye matrix
   - Updates telemetry with hand position at ~30fps

2. **Hand Tracking State** (`src/states/unified_hand_tracking_state.py`):

   - Reads hand position from telemetry
   - Applies calibration offsets (Y: -64mm, Z: -138mm)
   - Target: Hand at TCP coordinates `(0, 0, 0.2)` (20cm below gripper, centered)
   - Calculates IK to move TCP toward hand
   - Dead zone: 20mm radius, 2-second stability requirement
   - Transforms TCP-frame offsets to base frame using rotation matrix

3. **Coordinate Transformations**:
   - Offset calculation: `tcp_offset = target_pos_tcp - hand_pos_tcp`
   - Rotation transform: `tcp_offset_in_base = tcp_rotation @ tcp_offset`
   - IK target: `target_position_base = current_tcp_pose[:3] + tcp_offset_in_base`

#### Configuration (`src/config/system_config.py`)

```python
DISTANCE_TO_REMAIN_M = 0.2          # Target: 20cm below TCP
HAND_STABILITY_THRESHOLD = 0.02     # Dead zone radius: 20mm
HAND_STABILITY_TIME_THRESHOLD = 2.0 # Stability duration: 2 seconds
LOOP_RATE_MS = 20                   # 50Hz main loop (but hand updates ~10Hz)
```

---

## The Problem

### Scenario Sequence

1. **Object grasped** - Gripper closes on object successfully
2. **Approach begins** - Robot moves toward human hand position
3. **Initial tracking** - Hand fully visible in camera center, tracking works perfectly
4. **Occlusion starts** - As gripper+object approach, they enter camera view and partially occlude hand
5. **Hand in periphery** - Hand may be pushed to edge of camera frame (partially visible)
6. **Z-coordinate chaos** - Height measurements jump wildly (e.g., 0.05m ↔ 0.25m)
7. **Oscillation** - Robot moves up/down continuously, never settling
8. **Handoff fails** - Never enters dead zone, never completes handoff

### Specific Symptoms

**What works fine:**

- ✅ X/Y coordinates remain relatively stable (±10mm variation)
- ✅ Full visibility tracking (hand centered, no gripper in view)
- ✅ Detection continues with partial occlusion (MediaPipe still finds hand)

**What breaks:**

- ❌ **Z-coordinate jumps**: 50-200mm oscillations between frames
- ❌ **Never reaches dead zone**: 3D distance never stays <20mm for 2 seconds
- ❌ **Vertical oscillation**: Robot bounces up/down at ~1-3 Hz
- ❌ **No convergence**: Continues indefinitely until timeout (20s)

### Root Cause Analysis

**Hypothesis - Why Z breaks but not X/Y:**

1. **Depth Measurement Sensitivity**:

   - Current: `get_average_depth(center_pixel, radius)` uses circular ROI
   - When hand partially occluded: palm center pixel may hit gripper/object edge
   - Edge pixels have invalid/noisy depth (RealSense returns 0 or extreme values)
   - Single bad frame → 20cm jump in Z coordinate

2. **Peripheral Detection Issues**:

   - MediaPipe palm centroid shifts when landmarks are occluded
   - Different landmarks visible → different centroid pixel location
   - Depth at new pixel location may be gripper surface, not hand
   - Example: Hand at 0.3m depth, gripper at 0.15m, pixel hits gripper → sudden 0.15m jump

3. **Current Filter Inadequate**:

   - 20cm outlier threshold is too large (oscillations are <20cm)
   - 5-frame averaging doesn't smooth rapid back-and-forth jumps
   - No distinction between X/Y/Z filtering (Z needs more aggressive smoothing)

4. **Feedback Loop Amplification**:
   - Bad Z measurement → robot moves wrong direction
   - Movement changes TCP position → changes hand-eye transformation
   - Next frame measures at slightly different pixel → another bad depth
   - Cycle repeats, creating sustained oscillation

### Visual Examples

**Good Case (No Occlusion):**

```
Frame N:   Hand @ (0.01, -0.02, 0.18) - stable
Frame N+1: Hand @ (0.01, -0.02, 0.18) - stable
Frame N+2: Hand @ (0.00, -0.01, 0.19) - slight variation
→ Converges to dead zone ✅
```

**Bad Case (Partial Occlusion):**

```
Frame N:   Hand @ (0.05, 0.03, 0.25) - gripper edge in view
Frame N+1: Hand @ (0.04, 0.02, 0.08) - depth jumped 17cm!
Frame N+2: Hand @ (0.06, 0.04, 0.22) - jumped back 14cm!
Frame N+3: Hand @ (0.05, 0.03, 0.11) - jumped 11cm again!
→ Never converges, robot bounces up/down ❌
```

---

## Current Code Structure

### File Hierarchy

```
src/
├── config/
│   └── system_config.py          # Configuration parameters
├── camera_management/
│   ├── camera_manager.py         # RealSense camera interface
│   └── camera_transform_module.py # Coordinate transformations
├── hand_detection/
│   └── hand_detection_module.py  # MediaPipe hand detection + filtering
└── states/
    └── unified_hand_tracking_state.py # Robot control loop
```

### Key Code Sections

#### 1. Hand Detection (`hand_detection_module.py`, lines ~280-330)

```python
# Current depth measurement (simplified)
centroid = calculate_palm_centroid(landmarks)  # MediaPipe normalized coords
palm_x = int(centroid[0] * width)
palm_y = int(centroid[1] * height)
pixel_radius = int(radius * min(width, height))

# Single measurement from circular ROI
depth = camera_manager.get_average_depth(depth_frame, (palm_x, palm_y), pixel_radius)

# Convert to 3D
vector_3d_cam = camera_manager.pixel_to_3d(palm_x, palm_y, depth)

# Transform to TCP frame
vector_3d_tcp = transform_camera_to_tcp_frame(vector_3d_cam)

# Simple outlier filter
filtered_tcp = filter_hand_position(vector_3d_tcp)  # Rejects if >20cm from average

# Update telemetry
telemetry.update_camera_vector(filtered_tcp)
```

#### 2. Outlier Filter (`hand_detection_module.py`, lines 121-167)

```python
def _filter_hand_position(self, position):
    """Reject outliers using 5-frame history."""
    # Keep last 5 positions
    if len(position_history) < 2:
        return position  # Accept if insufficient history

    avg_position = mean(position_history)
    distance_from_avg = norm(position - avg_position)

    if distance_from_avg <= 0.20:  # 20cm threshold
        position_history.append(position)
        return position
    else:
        return position_history[-1]  # Use last valid position
```

#### 3. Robot Control Loop (`unified_hand_tracking_state.py`, lines 174-250)

```python
def _move_robot_toward_hand(self, hand_position, current_time):
    # Throttle to 10Hz
    if current_time - self.last_movement_time < 0.1:
        return

    # Get hand position in TCP frame (from telemetry)
    hand_pos_tcp = np.array(hand_position)

    # Apply calibration offsets
    hand_pos_tcp[1] -= 0.064  # Y offset
    hand_pos_tcp[2] -= 0.138  # Z offset

    # Target: (0, 0, 0.2)
    target_pos_tcp = np.array([0.0, 0.0, DISTANCE_TO_REMAIN_M])

    # Calculate offset
    tcp_offset = target_pos_tcp - hand_pos_tcp

    # Transform to base frame
    tcp_rotation = current_tcp_matrix[:3, :3]
    tcp_offset_in_base = tcp_rotation @ tcp_offset
    target_position_base = current_tcp_pose[:3] + tcp_offset_in_base

    # Solve IK and send command
    target_joints = ik.solve_XYZ(target_position_base, ...)
    commands.send(SetJoints(target_joints))
```

#### 4. Depth Calculation (`camera_manager.py`, lines 242-335)

```python
def get_average_depth(self, depth_frame, center, radius):
    """Get median depth in circular region."""
    # Create circular mask
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.circle(mask, center, radius, 255, -1)

    # Extract depths in circle
    depth_array = np.asanyarray(depth_frame.get_data())
    valid_depths = depth_array[mask == 255]
    valid_depths = valid_depths[valid_depths > 0]

    # Percentile clipping (25th to 75th percentile)
    if valid_depths.size >= 10:
        lower = percentile(valid_depths, 25)
        upper = percentile(valid_depths, 75)
        clipped = valid_depths[(valid_depths >= lower) & (valid_depths <= upper)]
        valid_depths = clipped if clipped.size >= 5 else valid_depths

    # Return median in meters
    median_depth = np.median(valid_depths) * depth_scale
    return median_depth
```

---

## Constraints & Requirements

### Must Preserve

- ✅ X/Y tracking responsiveness (lateral tracking must remain accurate)
- ✅ Real-time performance (total latency <50ms added)
- ✅ Existing hand-eye calibration (no recalibration required)
- ✅ MediaPipe detection (no model retraining)
- ✅ Code modularity (clean separation of concerns)

### Success Criteria

- 🎯 Z-coordinate variation <10mm when near target (currently: 50-200mm)
- 🎯 Converges to dead zone within 3-5 seconds (currently: never)
- 🎯 Works with 30-50% hand occlusion (currently: fails)
- 🎯 No false positives (should still track when hand moves intentionally)
- 🎯 Graceful degradation (if severely occluded, hold position, don't oscillate)

### Implementation Constraints

- Maximum 3 files modified
- No new dependencies (numpy, opencv, scipy already available)
- No changes to hand-eye calibration or IK solver
- Must remain compatible with existing telemetry/command bus architecture
- Code should be production-ready (not experimental hacks)

---

## Solution Requirements

### What You Must Provide

Your response must include the following sections **exactly** as specified:

### 1. **SOLUTION OVERVIEW** (2-3 sentences)

Brief description of your approach and why it solves the problem.

### 2. **IMPLEMENTATION PLAN** (numbered steps)

High-level steps in order of implementation (e.g., "1. Add depth quality metrics, 2. Implement Z-specific filter, etc.")

### 3. **CODE MODIFICATIONS** (exact edits)

For each modification, provide:

```
FILE: src/path/to/file.py
LOCATION: Line XXX or "Add to class ClassName.__init__" or "Add new function before class ClassName"
ACTION: REPLACE / INSERT / ADD_TO_INIT / NEW_FUNCTION

--- OLD CODE (if REPLACE): ---
[exact code to replace, with enough context to be unique]

--- NEW CODE: ---
[complete new code, properly indented, ready to paste]

EXPLANATION: [one sentence why this change is needed]
```

### 4. **CONFIGURATION PARAMETERS**

List any new parameters to add to `src/config/system_config.py`:

```python
# Parameter name and default value
PARAM_NAME = value  # Description and reasoning
```

### 5. **VALIDATION TESTS**

Provide 3-5 concrete tests to verify the solution works:

```
TEST 1: [scenario description]
  Expected: [specific measurable outcome]

TEST 2: [scenario description]
  Expected: [specific measurable outcome]
```

### 6. **TUNING GUIDE**

If parameters need tuning, provide:

```
IF [observed behavior]:
  → ADJUST [parameter name] FROM [default] TO [suggested range]
  → BECAUSE [reasoning]
```

---

## Technical Context

### MediaPipe Palm Detection

- Tracks 21 hand landmarks (indices 0-20)
- Palm centroid from 7 landmarks: [0, 1, 2, 5, 9, 13, 17]
- Returns normalized coordinates (0.0-1.0 range)
- Provides confidence scores (min: 0.5)
- Works with partial occlusion (hand 30%+ visible)

### RealSense Depth Characteristics

- Depth frame: 640x480 pixels
- Depth units: Typically 0.001 (millimeters to meters conversion)
- Invalid depth: 0 or NaN
- Depth scale: `depth_frame.get_units()` returns scale factor
- Noise increases with distance (±5mm at 0.5m, ±20mm at 2m)
- Edge effects: Depth at object boundaries unreliable (±50mm)

### Current Filtering Mechanisms

**Spatial (ROI-based):**

- `get_average_depth()` uses circular mask (radius ~30-50 pixels)
- Percentile clipping (25th-75th percentile) removes outliers
- Returns median of valid pixels

**Temporal (position history):**

- 5-frame sliding window
- Rejects if >20cm from window average
- Falls back to last valid position

**Problem:** This filtering is **isotropic** (same for X, Y, Z) and **threshold-based** (binary reject/accept). Z-coordinate needs **anisotropic temporal smoothing** without binary rejection.

---

## The Occlusion Problem (Detailed)

### Failure Mode 1: Gripper Edge Interference

**Timeline:**

```
t=0s:  Hand fully visible, centered in frame
       Palm centroid at (320, 240) px → depth = 0.35m ✓

t=1s:  Gripper enters frame from top, hand still mostly visible
       Palm centroid at (330, 260) px → depth = 0.34m ✓

t=2s:  Gripper partially covers hand, hand moves to periphery
       Palm centroid at (280, 380) px (near frame edge)
       **Depth at this pixel = 0.12m** (hits gripper surface!) ❌
       Robot thinks hand is 23cm closer → moves down rapidly

t=2.1s: Robot moved down, gripper moved out of way
        Palm centroid back at (315, 250) px → depth = 0.33m
        Robot thinks hand is 21cm further → moves up rapidly

t=2.2s: Cycle repeats - oscillation established
```

### Failure Mode 2: Landmark Shift Under Occlusion

**Scenario:**

- Fingers occluded, only palm base visible
- MediaPipe centroid shifts toward wrist (different landmarks dominant)
- New centroid pixel has different depth (hand is curved surface)
- Example: Palm center at 0.30m, wrist at 0.32m → 2cm jump
- Multiple frames with different landmark visibility → erratic Z values

### Failure Mode 3: Invalid Depth Propagation

**Scenario:**

- Occlusion causes 60% of circular ROI to be invalid (depth = 0)
- Remaining 40% includes both hand (0.30m) and background (0.50m)
- Percentile clipping may select wrong population
- Median depth unstable as valid pixel set changes frame-to-frame

---

## Attempted Solutions (Do Not Suggest These)

The following have already been tried and did NOT work:

❌ **Larger outlier threshold** - Still too binary, doesn't smooth gradual jumps
❌ **Simple moving average** - Too much lag, hand can move legitimately in 5 frames
❌ **Increase ROI radius** - Makes problem worse (more gripper pixels included)
❌ **Stricter palm flatness** - Rejects valid hand poses, doesn't help Z stability
❌ **Coordinate frame changes** - Problem is in measurement, not transformation

---

## Suggested Solution Directions (Evaluate & Choose Best)

### Option A: Adaptive Temporal Filter with Quality Gating

- Separate filter for Z coordinate (more aggressive than X/Y)
- Use exponential moving average or 1-Euro filter on Z
- Gate updates based on depth quality metrics (variance, valid pixel ratio)
- When quality low → increase filter strength, when quality high → track freely

### Option B: Multi-Hypothesis Z Tracking

- Maintain multiple Z hypotheses (e.g., hand surface, hand center, hand wrist)
- Score each hypothesis based on temporal consistency
- Select most stable hypothesis over time window
- Merge hypotheses when confidence high

### Option C: Depth ROI Segmentation

- Segment circular ROI into hand vs non-hand pixels using depth clustering
- Reject pixels that are >10cm different from cluster median
- Track cluster centroid depth instead of fixed palm pixel
- Adaptive ROI that shrinks when occlusion detected

### Option D: Predictive Filter with Outlier Rejection

- Implement simple Kalman filter on Z coordinate
- Model: constant velocity or constant position
- Measurement update only if within 3σ of prediction
- Automatically rejects wild jumps while tracking smooth motion

### Option E: Hybrid: Quality Metrics + Adaptive Smoothing + Rate Limiting

- **Quality metrics**: Count valid pixels, measure depth variance in ROI
- **Adaptive smoothing**: Smooth Z more when quality is low, less when quality is high
- **Rate limiting**: Cap maximum Z change per frame (e.g., 2cm/frame max)
- **Hysteresis**: Different thresholds for entering vs exiting dead zone

---

## Your Task

**Choose the best approach** (or combine elements) and provide:

1. **Exact code modifications** following the format above
2. **Complete function implementations** (not pseudocode)
3. **Parameter values** with reasoning
4. **Integration points** clearly specified
5. **Validation tests** that can be run immediately

### Response Format Checklist

- [ ] Solution overview (why this approach)
- [ ] Step-by-step implementation plan
- [ ] File-by-file code modifications with exact line numbers or insertion points
- [ ] All new functions/classes with complete implementations
- [ ] Configuration parameters with default values
- [ ] Concrete validation tests
- [ ] Tuning guide for parameters

### Quality Standards

- Code must be **production-ready** (no TODOs or placeholders)
- All edge cases handled (None values, empty arrays, first frame, etc.)
- Performance-conscious (no nested loops over images, use vectorized ops)
- Well-commented (explain non-obvious logic)
- Logging included (at appropriate debug/info levels)
- Type hints for new functions

---

## Example of Good Response Format

```
SOLUTION OVERVIEW:
Implement One-Euro filter on Z coordinate with adaptive cutoff frequency
based on depth ROI quality. This provides low-lag smoothing when quality
is good, aggressive filtering when quality degrades during occlusion.

IMPLEMENTATION PLAN:
1. Add depth quality assessment (valid pixel ratio, depth std dev)
2. Implement One-Euro filter class for Z-coordinate smoothing
3. Add quality-based adaptive filtering in hand detection module
4. Add rate-limiting on commanded Z changes
5. Add hysteretic dead zone for stability

CODE MODIFICATIONS:

FILE: src/hand_detection/hand_detection_module.py
LOCATION: Add new class before class HandTracker (around line 40)
ACTION: NEW_CLASS

--- NEW CODE: ---
import math

class OneEuroFilter:
    """One-Euro filter for low-lag adaptive smoothing."""
    def __init__(self, min_cutoff=1.0, beta=0.007, d_cutoff=1.0):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.x_prev = None
        self.dx_prev = 0.0
        self.t_prev = None

    def __call__(self, x, t):
        if self.x_prev is None:
            self.x_prev = x
            self.t_prev = t
            return x

        dt = t - self.t_prev
        # [complete implementation...]

EXPLANATION: One-Euro filter provides adaptive smoothing based on signal velocity.

[... more modifications ...]

CONFIGURATION PARAMETERS:

# Add to src/config/system_config.py after HAND_STABILITY_TIME_THRESHOLD

# Z-coordinate filtering parameters
Z_FILTER_MIN_CUTOFF = 1.0    # Base smoothing (Hz), lower = more smooth
Z_FILTER_BETA = 0.007        # Velocity tracking gain
MIN_DEPTH_VALID_RATIO = 0.3  # Require 30% valid pixels in ROI
MAX_DEPTH_STD_DEV = 0.03     # Flag as low-quality if std dev > 30mm
MAX_Z_CHANGE_PER_FRAME = 0.02  # Rate limit: max 2cm per frame

VALIDATION TESTS:

TEST 1: Static hand with gradual occlusion
  Setup: Place hand at target position, slowly move gripper to occlude
  Expected: Z coordinate stays within ±5mm of initial value
  Expected: Robot stops moving (enters dead zone within 3s)

TEST 2: Dynamic hand with partial occlusion
  Setup: Move hand up/down slowly while gripper partially occludes
  Expected: Z coordinate tracks smooth motion with <20ms lag
  Expected: No oscillations or jumps >2cm

[... more tests ...]

TUNING GUIDE:

IF Z-coordinate has too much lag when hand moves:
  → DECREASE Z_FILTER_MIN_CUTOFF FROM 1.0 TO 0.5
  → BECAUSE lower cutoff = less smoothing = faster response

IF Z-coordinate still oscillates:
  → DECREASE MAX_Z_CHANGE_PER_FRAME FROM 0.02 TO 0.01
  → BECAUSE tighter rate limit prevents large jumps
```

---

## Additional Context

### Typical Handoff Sequence Timing

```
0-2s:   Approach phase - hand fully visible, tracking works
2-4s:   Close approach - gripper enters frame, occlusion begins
4-6s:   Stabilization - should enter dead zone (CURRENTLY FAILS HERE)
6-8s:   Stability hold - 2s dead zone timer
8s:     Handoff complete - gripper opens, releases object
```

**The 4-6s window is where the solution must work.**

### Performance Budget

- Hand detection: ~30ms per frame
- Coordinate transformation: <1ms
- IK solving: ~10ms
- **Available for filtering: ~10-15ms**

### Edge Cases to Handle

1. **First frame** - No history, filter should initialize gracefully
2. **Hand leaves frame** - Should reset filter state, not maintain stale data
3. **Complete occlusion** - No valid depth, should hold last position (not oscillate)
4. **Rapid hand movement** - Filter should track, not lag severely
5. **Near-target jitter** - Small oscillations around target should be heavily damped

---

## Deliverable

Provide a **complete, copy-paste ready implementation** that I can hand to another AI agent who will:

1. Read your exact file paths and line numbers
2. Copy your exact code blocks
3. Paste them into the specified locations
4. Verify with your exact tests
5. Tune using your exact parameter guidance

**No ambiguity allowed.** Be specific, be complete, be precise.

---

## Questions to Answer in Your Solution

1. **Which filtering approach** do you recommend and why?
2. **Where exactly** should filtering be applied (detection module, control loop, or both)?
3. **How do you detect** low-quality depth measurements?
4. **What happens** when hand is completely occluded (no valid depth)?
5. **How do you prevent** lag from making tracking feel unresponsive?
6. **What are the trade-offs** of your chosen approach?

---

## Success = Clear Implementation Path

A successful response allows an AI agent to:

- Open the specified files
- Find the exact locations (line numbers or clear markers)
- Insert/replace the provided code
- Add the configuration parameters
- Run the validation tests
- Achieve the success criteria

**No interpretation needed. No decisions left to make. Just execute.**

Thank you! Please provide your solution now.

