# AI Prompt: Hand Tracking Overshoot & Oscillation Fix at High Speed

## Mission

Provide a **step-by-step implementation plan** to eliminate overshoot and oscillation in hand tracking when the robot moves at higher speeds. The solution must account for sensing/processing delays and prevent the robot from repeatedly crossing over the target position.

**Critical Requirement:** Your response must include **exact, copy-paste ready code modifications** with file paths, line numbers, and complete function implementations that another AI agent can implement without ambiguity.

---

## System Overview

### Hardware & Software Stack

- **Robot**: KUKA iiwa 7-DOF collaborative robot
- **Camera**: Intel RealSense D435 RGB-D (mounted on end-effector)
- **Hand Detection**: MediaPipe Hand Landmarker with AdaptiveZFilter
- **Control Loop**: 10Hz update rate (~100ms cycle time)
- **Robot Communication**: OPC UA with additional latency (~20-50ms)
- **IK Solver**: Numerical inverse kinematics (TrackIK-based)

### Current Control Architecture

**Sensing → Processing → Control Loop:**

```
1. Camera captures frame                    [~33ms @ 30fps]
2. MediaPipe detects hand                   [~30ms]
3. AdaptiveZFilter smooths Z coordinate     [<1ms]
4. Transform camera → TCP → base frame      [<1ms]
5. Telemetry updated                        [<1ms]
6. Control loop reads telemetry (10Hz)      [100ms cycle]
7. IK solver calculates target joints       [~10ms]
8. Command sent via OPC UA                  [~20-50ms]
9. Robot begins motion                      [~50-100ms startup]
```

**Total latency: ~200-350ms from hand movement to robot response**

### Coordinate Frames & Transformations

```
Camera Frame (RealSense):
  +X = Right, +Y = Down, +Z = Forward (depth)

TCP Frame (gripper facing down):
  +X = Forward, +Y = Left, +Z = Down

Current Offset Calculation:
  tcp_offset = target_pos_tcp - hand_pos_tcp
  tcp_offset[0] = -tcp_offset[0]  # X inverted
  tcp_offset[1] = -tcp_offset[1]  # Y inverted
  tcp_offset[2] = -tcp_offset[2]  # Z inverted

Transform to Base:
  tcp_offset_in_base = tcp_rotation @ tcp_offset
  target_position_base = current_tcp_pose[:3] + tcp_offset_in_base
```

### Current Implementation

#### Control Loop (`unified_hand_tracking_state.py`, lines 82-111)

```python
def execute(self):
    current_time = time.time()

    # Throttle to 10Hz
    if current_time - self.last_stability_check < 0.1:
        return

    hand_position = telemetry.get_camera_vector()

    if hand_position is not None and not np.array_equal(hand_position, [0.0, 0.0, 0.0]):
        self._update_hand_tracking(hand_position)
        self._move_robot_toward_hand(hand_position, current_time)
        self._calculate_placement_pose(hand_position)
    else:
        # No hand detected - reset stability
        self.is_hand_stable = False
```

#### Movement Control (`unified_hand_tracking_state.py`, lines 210-280)

```python
def _move_robot_toward_hand(self, hand_position, current_time):
    # Stop if in dead zone (20mm radius)
    if self.is_hand_stable:
        return

    # Get current robot joints
    current_joints = telemetry.get_current_joints()
    current_tcp_matrix, current_tcp_pose = ik.tcp_from_joints(current_joints)

    # Apply Z calibration offset
    hand_pos_tcp = np.array(hand_position)
    hand_pos_tcp[2] -= 0.138  # -138mm Z offset

    # Calculate offset to target (0, 0, 0.2)
    target_pos_tcp = np.array([0.0, 0.0, DISTANCE_TO_REMAIN_M])
    tcp_offset = target_pos_tcp - hand_pos_tcp

    # Invert all axes due to camera mounting
    tcp_offset[0] = -tcp_offset[0]
    tcp_offset[1] = -tcp_offset[1]
    tcp_offset[2] = -tcp_offset[2]

    # Transform to base frame
    tcp_rotation = current_tcp_matrix[:3, :3]
    tcp_offset_in_base = tcp_rotation @ tcp_offset
    target_position_base = current_tcp_pose[:3] + tcp_offset_in_base

    # Solve IK and send command
    target_joints = ik.solve_XYZ(target_position_base, current_joints, orientation)
    if target_joints is not None:
        commands.send(SetJoints(target_joints))
```

#### Dead Zone Logic (`unified_hand_tracking_state.py`, lines 133-176)

```python
def _update_hand_tracking(self, hand_position):
    hand_pos_tcp = np.array(hand_position)
    hand_pos_tcp[2] -= 0.138  # Z offset

    target_pos_tcp = np.array([0.0, 0.0, DISTANCE_TO_REMAIN_M])

    # 3D distance check
    distance_from_target = np.linalg.norm(hand_pos_tcp - target_pos_tcp)

    # Dead zone threshold: 20mm
    dead_zone_threshold = 0.02

    if distance_from_target < dead_zone_threshold:
        if not self.is_hand_stable:
            self.is_hand_stable = True
            self.hand_stable_start_time = time.time()
        # Timer continues
    else:
        # Reset if exits dead zone
        self.is_hand_stable = False
        self.hand_stable_start_time = 0.0
```

### Configuration (`src/config/system_config.py`)

```python
DISTANCE_TO_REMAIN_M = 0.2          # Target: 20cm below TCP
HAND_STABILITY_THRESHOLD = 0.02     # Dead zone radius: 20mm
HAND_STABILITY_TIME_THRESHOLD = 2.0 # Stability duration: 2 seconds

# Z-filtering (already implemented)
Z_FILTER_ALPHA_GOOD = 0.6           # Responsive when quality good
Z_FILTER_ALPHA_POOR = 0.12          # Aggressive when quality poor
MAX_Z_CHANGE_PER_FRAME = 0.02       # 2cm max per frame
```

---

## The Problem

### Scenario

1. **Robot at rest** - Hand tracking starts
2. **Hand detected** - Robot begins moving toward hand (position error ~30cm)
3. **High-speed approach** - Robot moves at 0.1-0.2 m/s toward target
4. **Delay accumulation** - 200-350ms total delay (sensing + processing + communication)
5. **Overshoot** - Robot passes through target position before sensing it arrived
6. **Reverse direction** - Next control cycle detects robot is past target, reverses
7. **Oscillation** - Robot bounces back and forth across target position
8. **Never settles** - Cannot enter dead zone (always moving through it, not stopping in it)

### Specific Symptoms

**What works:**

- ✅ Slow approach (robot speed <0.05 m/s) - settles correctly
- ✅ Large distances (>30cm from target) - tracks smoothly
- ✅ Dead zone detection - correctly identifies when within 20mm
- ✅ Axis directions - X, Y, Z all track correctly

**What breaks at higher speeds:**

- ❌ **Overshoot**: Robot crosses target position by 3-8cm before stopping
- ❌ **Oscillation**: Bounces back and forth with period ~1-2 seconds
- ❌ **Never converges**: Distance oscillates 2-8cm, never stays <20mm for 2s
- ❌ **Autonomous drift**: Robot moves toward bottom of camera frame even with stationary hand
- ❌ **Poor settling**: Takes >15 seconds or times out (vs expected 3-5s)

### Root Cause Analysis

**1. Control Delay (Latency)**

- Robot position used for offset calculation is **200-350ms old**
- By the time command is executed, robot has moved significantly
- Example: At 0.15 m/s, robot moves **3-5cm during latency period**

**2. No Velocity Damping**

- Control law is pure proportional (P-only controller)
- No derivative term to slow down as approaching target
- Robot maintains high speed until past target, then reverses at high speed

**3. Dead Zone Hysteresis Insufficient**

- Dead zone is symmetric: enter at <20mm, exit at >20mm
- With oscillation amplitude of 3-8cm, robot never stays inside for 2 seconds
- Need different thresholds for entering vs staying in dead zone

**4. No Predictive Compensation**

- Controller assumes robot is where it was 200ms ago
- Doesn't account for commanded motion that's in progress
- Creates phase lag in control loop

### Oscillation Example Timeline

```
t=0.0s:  Hand at TCP (0, 0, 0.25), robot at (0, 0, 0.50)
         Distance: 25cm, offset = -25cm (move down 25cm)
         Command sent: target_base = current + offset

t=0.3s:  Robot started moving down at 0.15 m/s (traveled ~1cm so far)
         Hand detection sees robot at old position (lag)
         Still commanding large downward movement

t=1.0s:  Robot at (0, 0, 0.32), still moving down at 0.15 m/s
         Target is 0.25, but robot momentum carries it past

t=1.2s:  Robot reaches (0, 0, 0.22) - OVERSHOT by 3cm!
         Hand detection finally sees robot below target
         Offset = +3cm (move up)
         Commands upward movement

t=1.5s:  Robot reversing, moving up at 0.15 m/s

t=2.2s:  Robot at (0, 0, 0.27) - OVERSHOT UP by 2cm!
         Commands downward movement again

t=2.5s:  Cycle repeats - oscillation established
```

### Autonomous Drift Issue

**Observation:** Robot drifts toward bottom of camera frame even with stationary hand.

**Possible causes:**

1. **Bias in hand detection** - Palm centroid calculation has systematic error
2. **Noise integration** - Small measurement noise integrated over time
3. **Target position error** - `(0, 0, 0.2)` might not be correct target for camera frame
4. **Coordinate transformation error** - Slight error in rotation matrix compounds
5. **Gravity compensation** - Robot sags slightly under load, appears to be below target

---

## Current Code Structure

**Main Files:**

1. `src/states/unified_hand_tracking_state.py` - Control loop and movement logic
2. `src/hand_detection/hand_detection_module.py` - Hand detection with AdaptiveZFilter
3. `src/config/system_config.py` - Configuration parameters
4. `src/kinematics/kinematics_solver.py` - IK solver
5. `src/control/telemetry_store.py` - Robot state storage
6. `src/control/command_bus.py` - Command sending

**Key Methods:**

- `UnifiedHandTrackingState.execute()` - Main control loop (10Hz)
- `UnifiedHandTrackingState._move_robot_toward_hand()` - Movement calculation
- `UnifiedHandTrackingState._update_hand_tracking()` - Dead zone checking
- `AdaptiveZFilter.update()` - Z-coordinate filtering

---

## Attempted Solutions (Do Not Suggest These)

The following have already been tried:

❌ **Tighter dead zone** - Makes it harder to enter, doesn't fix overshoot
❌ **Longer stability time** - Doesn't help if never enters dead zone
❌ **More aggressive Z filtering** - Helps with noise but not overshoot
❌ **Rate limiting Z commands** - Already implemented (MAX_Z_CHANGE_PER_FRAME = 2cm), not sufficient

---

## Constraints & Requirements

### Must Preserve

- ✅ Existing hand detection and filtering (AdaptiveZFilter works well)
- ✅ Coordinate transformation logic (X/Y/Z inversions are correct)
- ✅ Dead zone concept (20mm radius, 2s duration)
- ✅ Real-time performance (<50ms added latency)
- ✅ X/Y/Z tracking accuracy when not near target

### Success Criteria

- 🎯 No overshoot >10mm when approaching target from any direction
- 🎯 Settles in dead zone within 5 seconds at normal speed
- 🎯 No oscillation (period <3 seconds) once within 10cm of target
- 🎯 No autonomous drift (should hold position when hand stationary)
- 🎯 Smooth deceleration as approaching target
- 🎯 Works at robot speeds up to 0.2 m/s

### Implementation Constraints

- Maximum 2 files modified (prefer only unified_hand_tracking_state.py)
- No new hardware/sensors required
- No changes to IK solver or hand detection
- Must be tunable via configuration parameters
- Production-ready code (handle all edge cases)

---

## Technical Details

### Current Timing Characteristics

**Latency Budget:**

```
Camera frame capture:        ~33ms (30fps)
MediaPipe processing:         ~30ms
AdaptiveZFilter + transform:  ~2ms
Telemetry update:             ~1ms
--------------------------------------
Detection to telemetry:       ~66ms

Control loop cycle:           100ms (10Hz)
IK solving:                   ~10ms
OPC UA command transmission:  ~20-50ms
Robot motion startup:         ~50-100ms
--------------------------------------
Total control latency:        180-260ms

TOTAL SYSTEM LATENCY:         ~250-325ms
```

**At 0.15 m/s robot speed, travels 3.75-4.88cm during latency!**

### Dead Zone Characteristics

**Current implementation:**

- **Entry threshold**: 20mm 3D distance from target
- **Exit threshold**: 20mm 3D distance from target (same)
- **Stability requirement**: Stay in zone for 2.0 seconds continuously
- **No hysteresis**: Entry == Exit threshold
- **Isotropic**: Same threshold for X, Y, and Z

**Problem:** With 3-8cm oscillation amplitude, robot passes through but never stays in zone.

### Robot Motion Characteristics

**KUKA iiwa behavior:**

- Default motion: Joint space velocity control
- Acceleration limit: ~0.5 m/s² typical
- Deceleration capability: ~0.8 m/s² (can stop faster than accelerate)
- Small motions (<5cm): Can execute in ~0.5-1s
- Overshoot typical with pure position commands if target changes during motion

**IK Solver:**

- Returns joint positions, not velocities
- No built-in trajectory planning
- Each command is independent (no awareness of previous commands)

---

## The Overshoot Problem (Detailed)

### Failure Mode 1: Speed-Induced Overshoot

**Timeline:**

```
t=0.0s:  Robot at (0, 0, 0.40), hand at (0, 0, 0.25), speed = 0 m/s
         Error: -15cm (need to move down)
         Command: Move to (0, 0, 0.25)

t=0.3s:  Robot at (0, 0, 0.35), accelerating, speed = 0.12 m/s
         Control loop sees robot at OLD position (0, 0, 0.40) due to latency
         Still commanding: Move to (0, 0, 0.25)

t=0.6s:  Robot at (0, 0, 0.28), speed = 0.15 m/s (high speed)
         Control loop sees robot at (0, 0, 0.35) (300ms lag)
         Error appears to be -10cm, still commanding downward

t=0.9s:  Robot crosses target at (0, 0, 0.25), speed = 0.15 m/s
         Momentum carries it to (0, 0, 0.21) before next control cycle

t=1.0s:  Control loop finally sees robot at (0, 0, 0.28) (still lagged)
         Commands further downward movement

t=1.3s:  Robot reaches (0, 0, 0.19) - OVERSHOT by 6cm!
         Control loop sees robot at (0, 0, 0.21)
         Now commands UPWARD movement

t=1.6s:  Robot reversing, moving up at 0.15 m/s

t=2.2s:  Robot at (0, 0, 0.27) - overshot UP by 2cm
         Cycle repeats...
```

### Failure Mode 2: Autonomous Y-Drift

**Observation:** Robot slowly drifts toward bottom of camera frame (increasing Y) even when hand is stationary.

**Hypothesis:**

- Integration of small measurement bias (~1-2mm/frame)
- No integral windup protection
- Target might be slightly off from hand's actual position
- Possible coordinate frame misalignment

### Failure Mode 3: Oscillation Frequency Matching Control Rate

**Observation:** Oscillation period ≈ 1-2 seconds (close to 2× control loop period)

**This suggests:** Classic discrete control system instability - sampling rate too slow for commanded velocity, creating aliasing/beat frequency.

---

## Suggested Solution Directions (Evaluate & Choose Best)

### Option A: Velocity-Damped Proportional Control (PD-like)

- Estimate robot velocity from position history
- Reduce commanded offset when velocity is high
- Gradually decrease speed as approaching target
- Add velocity term: `offset_damped = offset * (1 - k_d * |velocity|)`

### Option B: Predictive Control with Motion Compensation

- Predict where robot will be after latency period
- Use predicted position instead of current position for offset calculation
- Compensate for in-flight commands
- Model: `predicted_pos = current_pos + velocity * latency + 0.5 * accel * latency²`

### Option C: Adaptive Dead Zone with Hysteresis

- Larger entry threshold (e.g., 50mm) for initial capture
- Smaller exit threshold (e.g., 30mm) to maintain lock
- Progressive tightening: 50mm → 35mm → 20mm as velocity decreases
- Once captured, much harder to exit

### Option D: Multi-Stage Approach Strategy

- **Stage 1**: Coarse approach (>10cm away) - full speed
- **Stage 2**: Fine approach (3-10cm away) - 50% speed, proportional gain
- **Stage 3**: Settling (0-3cm away) - 20% speed, tight dead zone
- **Stage 4**: Locked (<20mm for 0.5s) - stop all motion

### Option E: Feedforward + Feedback Hybrid

- **Feedforward**: Compute expected settling position based on current trajectory
- **Feedback**: Standard proportional control
- **Blend**: Use feedforward when far, feedback when close
- **Damping**: Add artificial damping term based on approach velocity

### Option F: Simple Speed Scaling + Expanded Dead Zone

- Scale commanded offset by distance: `offset_scaled = offset * min(1.0, distance/0.1)`
- When very close (<5cm): reduce speed to 25%
- Expand dead zone to 35mm entry, 25mm exit
- Add settling time requirement: must decelerate to near-zero velocity

---

## Your Task

**Choose the best approach** (or combine elements) and provide:

1. **Exact code modifications** following the format below
2. **Complete function implementations** (not pseudocode)
3. **Parameter values** with reasoning
4. **Integration points** clearly specified
5. **Validation tests** that can be run immediately

### Response Format Required

```
SOLUTION OVERVIEW:
[2-3 sentences describing your approach and why it solves the problem]

IMPLEMENTATION PLAN:
1. [First step]
2. [Second step]
3. [etc.]

CODE MODIFICATIONS:

FILE: src/path/to/file.py
LOCATION: Line XXX or "Replace function method_name" or "Add to class __init__"
ACTION: REPLACE / INSERT / MODIFY

--- OLD CODE (if REPLACE): ---
[exact code to find and replace, with unique context]

--- NEW CODE: ---
[complete new code, properly indented, ready to paste]

EXPLANATION: [one sentence why this change is needed]

[... repeat for each modification ...]

CONFIGURATION PARAMETERS:

# Add to src/config/system_config.py after existing hand tracking params
PARAM_NAME = value  # Description and units

VALIDATION TESTS:

TEST 1: [scenario]
  Expected: [measurable outcome]

TEST 2: [scenario]
  Expected: [measurable outcome]

TUNING GUIDE:

IF [observed behavior]:
  → ADJUST [parameter] FROM [default] TO [range]
  → BECAUSE [reasoning]
```

---

## Specific Questions to Answer

1. **How do you compensate** for the 250-350ms control latency?
2. **How do you prevent overshoot** when robot has momentum?
3. **How do you detect** when robot should slow down vs maintain speed?
4. **What causes autonomous Y-drift** and how to eliminate it?
5. **Should dead zone be adaptive** (size changes based on approach conditions)?
6. **How do you handle** the case where hand moves while robot is settling?

---

## Additional Context

### Hand Detection Quality

- AdaptiveZFilter working well - Z coordinate stable
- X/Y coordinates have ~2-5mm jitter (acceptable)
- Frame rate: 30fps hand detection, 10Hz control updates
- Coordinate inversions (X, Y, Z all negated) are correct

### Robot Behavior Observations

- Smooth tracking when >20cm from target
- Abrupt direction changes when crossing target
- Speed doesn't decrease near target (maintains ~0.15 m/s)
- Sometimes "hunts" back and forth ±3-5cm around target
- Occasionally locks into stable oscillation pattern

### Performance Budget

- Current control loop: ~15ms per cycle
- Available for new logic: ~20-30ms
- Must remain real-time (<100ms total cycle time)
- Simple computations only (no optimization solvers)

---

## Success = Clear Implementation Path

A successful response allows an AI agent to:

- Open the specified files
- Find the exact locations (line numbers or clear insertion points)
- Insert/replace the provided code
- Add the configuration parameters
- Run the validation tests
- Achieve smooth settling without overshoot

**No interpretation needed. No decisions left to make. Just execute.**

---

## Deliverable Format

Provide:

1. ✅ Solution overview (why this approach beats others)
2. ✅ Step-by-step implementation plan
3. ✅ File-by-file code modifications (exact line numbers or unique context)
4. ✅ Complete function implementations (no TODOs or placeholders)
5. ✅ Configuration parameters with default values and units
6. ✅ 3-5 concrete validation tests
7. ✅ Tuning guide (if X then adjust Y)
8. ✅ Answers to all 6 questions above

### Quality Standards

- Code must be production-ready
- All edge cases handled (None values, zero velocity, etc.)
- Performance-conscious (no expensive operations in 10Hz loop)
- Well-commented (explain control theory decisions)
- Logging at appropriate levels
- Type hints for new functions

**The solution should be the simplest effective approach with biggest impact for least implementation effort.**

Thank you! Please provide your solution now.
