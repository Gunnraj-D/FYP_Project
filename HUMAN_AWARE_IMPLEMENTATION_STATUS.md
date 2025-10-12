# Human-Aware Path Planning - Current Implementation Status

## 📊 Overview

You now have a **complete architecture** for human-aware path planning, but with a **critical placeholder**: the actual RRT-Connect path planner is **not yet implemented**.

---

## ✅ What's Fully Implemented

### 1. **ZED Skeleton Tracking** ✅ 100% Complete

**File**: `src/hand_detection/zed_joint_receiver.py`

```python
# Fully working TCP receiver
receiver = ZEDJointReceiver(host='127.0.0.1', port=5005)
receiver.start()  # Background thread listening

# Get latest skeleton data (non-blocking)
frame_data = receiver.get_latest_frame()
skeleton = frame_data.skeletons[0]  # Single person

# Get any joint position (already in robot base frame!)
hand_pos = skeleton.get_joint_position('RIGHT_WRIST')  # [x, y, z] meters
```

**Status**: ✅ Fully working, tested with Unity

---

### 2. **Human Collision Model** ✅ 90% Complete

**File**: `src/kinematics/human_aware_path_planner.py` (lines 30-268)

**Class**: `HumanModelAdapter`

#### What It Does:

1. ✅ Gets skeleton data from ZED
2. ✅ Extracts critical joints (NECK, NOSE, PELVIS, etc.)
3. ✅ Creates PyBullet collision primitives:
   - **Spheres** at head joints (NECK, NOSE) - 0.20m radius
   - **Capsules** between torso joints (CHEST_SPINE ↔ PELVIS) - 0.18m radius
   - **Capsules** for arms (SHOULDER ↔ ELBOW ↔ WRIST) - 0.10m radius
   - **Spheres** at shoulders (clavicles) - 0.15m radius
4. ✅ Updates primitive positions in real-time as human moves
5. ✅ Handles missing joints gracefully

#### Example:

```python
# Creates ~9 collision bodies per person:
human_model = HumanModelAdapter(pybullet_client, config)
human_model.update_from_skeleton(skeleton)  # ZED → PyBullet primitives

# Result: PyBullet world now has collision geometries representing human
body_ids = human_model.get_all_body_ids()
# [sphere_NECK, sphere_NOSE, capsule_torso, capsule_left_upper_arm, ...]
```

**What's NOT done**: Velocity-adaptive inflation (Phase 3)

---

### 3. **Collision Distance Checking** ✅ 100% Complete

**File**: `src/kinematics/human_aware_path_planner.py` (lines 470-509)

#### Methods:

```python
# Check clearance at a specific robot configuration
clearance = planner._compute_clearance_at_config(joint_angles)
# Returns minimum distance to any human primitive (meters)

# Check entire trajectory
is_safe, min_clearance = planner.check_trajectory_safe(trajectory)
# Validates every waypoint against human model
```

#### How It Works:

```python
def _compute_clearance_at_config(self, joint_angles):
    # 1. Set robot to configuration
    for i, angle in enumerate(joint_angles):
        p.resetJointState(self.robot_id, i, angle)

    # 2. Get closest points to all human primitives
    for human_body_id in human_body_ids:
        closest_points = p.getClosestPoints(
            bodyA=self.robot_id,
            bodyB=human_body_id,
            distance=1.0
        )
        # Extract minimum distance

    return min_distance
```

**Status**: ✅ Fully working - uses PyBullet's `getClosestPoints`

---

### 4. **Speed Scaling (SSM)** ✅ 100% Complete

**File**: `src/states/human_aware_move_to_state.py` (lines 297-328)

#### SSM Zones (ISO/TS 15066):

```python
def _compute_speed_scale(self, clearance):
    if clearance >= 0.5m:
        return 1.0     # 100% speed (comfort zone)
    elif clearance >= 0.3m:
        return 0.5-1.0  # Linear ramp down (warning zone)
    elif clearance >= 0.15m:
        return 0.1-0.5  # Critical zone (very slow)
    else:
        return 0.0     # STOP
```

**Status**: ✅ Fully implemented with proper ISO/TS 15066 zones

---

### 5. **State Machine Integration** ✅ 100% Complete

**Files**:

- `src/states/human_aware_move_to_state.py`
- `src/states/human_handoff_approach_state.py`
- `src/main_debug.py` (integration)

#### Two States Available:

**State A**: `HumanAwareMoveToState` (States 7-8)

- Target: **Fixed position** you specify
- Plans once, replans if human moves
- Use: Move to workspace position while avoiding humans

**State B**: `HumanHandoffApproachState` (States 9-10) ⭐

- Target: **Human hand** (dynamic!)
- Tracks RIGHT_WRIST or LEFT_WRIST from ZED
- Approach offset: [0, 0, 0.3] = 30cm above hand
- Continuously updates target as hand moves
- **Use: Handoff workflow**

#### State Lifecycle:

```python
# 1. Enter
state.enter()
  → Initialize planner
  → Get hand position (State 9/10) or use fixed target (State 7/8)
  → Plan initial trajectory

# 2. Execute (50Hz loop)
state.execute()
  → Update hand position (if tracking hand)
  → Check if replanning needed
  → Validate trajectory safety
  → Apply speed scaling
  → Execute waypoint
  → Check if complete

# 3. Exit
state.exit()
  → Log statistics
  → Cleanup planner
```

**Status**: ✅ Fully integrated with state machine

---

## ⚠️ What's NOT Implemented (Critical!)

### **PATH PLANNER** ❌ Placeholder Only!

**File**: `src/kinematics/human_aware_path_planner.py` (lines 454-468)

#### Current Implementation:

```python
def _placeholder_plan(self, start, goal):
    """Placeholder: simple linear interpolation (replace with RRT-Connect)."""
    # TODO: Replace with actual RRT-Connect implementation
    steps = 50
    trajectory = []

    for i in range(steps + 1):
        alpha = i / steps
        waypoint = [
            start[j] + alpha * (goal[j] - start[j])
            for j in range(len(start))
        ]
        trajectory.append(waypoint)

    return trajectory
```

#### What This Means:

- 🚨 **Just linear interpolation** in joint space
- 🚨 **No obstacle avoidance** during planning
- 🚨 **Will collide with humans** if they're in the direct path
- 🚨 **Only checks clearance after planning** (not during)

#### What It SHOULD Be:

```python
# Actual RRT-Connect using pybullet_planning
from pybullet_planning import rrt_connect

def _rrt_connect_plan(self, start, goal):
    # 1. Define collision checker
    def is_collision_free(q):
        clearance = self._compute_clearance_at_config(q)
        return clearance >= self.config['hard_min_distance']

    # 2. Run RRT-Connect
    path = rrt_connect(
        start=start,
        goal=goal,
        distance_fn=joint_space_distance,
        sample_fn=sample_joint_space,
        extend_fn=joint_space_extend,
        collision_fn=is_collision_free,
        max_iterations=5000
    )

    # 3. Smooth path
    smoothed = shortcut_smooth(path)

    return smoothed
```

**Status**: ❌ **NOT IMPLEMENTED** - this is the main TODO!

---

## 🔍 Detailed Component Status

### Component Breakdown

| Component                     | Status    | Completion | Notes                                             |
| ----------------------------- | --------- | ---------- | ------------------------------------------------- |
| **ZEDJointReceiver**          | ✅ Done   | 100%       | TCP server, parsing, thread-safe buffering        |
| **HumanModelAdapter**         | ✅ Done   | 90%        | Sphere/capsule creation, lacks velocity inflation |
| **Collision Checking**        | ✅ Done   | 100%       | PyBullet getClosestPoints fully working           |
| **Speed Scaling (SSM)**       | ✅ Done   | 100%       | ISO/TS 15066 zones implemented                    |
| **HumanAwareMoveToState**     | ⚠️ Mostly | 80%        | Complete except actual path planner               |
| **HumanHandoffApproachState** | ⚠️ Mostly | 80%        | Complete except actual path planner               |
| **RRT-Connect Planner**       | ❌ TODO   | 0%         | **CRITICAL MISSING PIECE**                        |
| **Path Smoothing**            | ❌ TODO   | 0%         | Post-processing for trajectory                    |
| **Rolling Horizon**           | ❌ TODO   | 0%         | Advanced replanning strategy                      |

---

## 🎯 What Actually Happens Right Now

### If You Run State 9 (HumanHandoffApproachState)

```bash
🤖 Debug> run 9
```

**Step-by-step execution**:

1. ✅ **Enter Phase**:

   ```python
   - Gets RIGHT_WRIST position from ZED: [0.45, 0.23, 0.68]
   - Calculates target: [0.45, 0.23, 0.98] (30cm above)
   - Initializes PyBullet planning world
   - Creates human collision model (9 primitives)
   - Calls plan_trajectory()
   ```

2. ⚠️ **Planning Phase** (uses placeholder):

   ```python
   - Samples IK goal configuration
   - Calls _placeholder_plan() ← NOT RRT-Connect!
   - Returns LINEAR INTERPOLATION (50 waypoints)
   - Does NOT avoid obstacles during planning
   ```

3. ✅ **Validation Phase**:

   ```python
   - Checks each waypoint against human model
   - Computes min_clearance for the linear path
   - If any waypoint < 0.15m from human:
     → Returns is_safe=False
     → Planning "fails"
   ```

4. ✅ **Execution Phase** (if path is "safe"):
   ```python
   - Updates hand position every cycle
   - Checks trajectory safety
   - Applies speed scaling
   - Executes waypoints
   - Replans if hand moves
   ```

**Result**:

- ✅ Works if human is NOT blocking the direct path
- ❌ Fails if human is in the way (can't plan around them)
- ⚠️ Only validates clearance, doesn't actually plan detours

---

## 🔍 Architecture Diagram (Current State)

```
┌─────────────────────────────────────────────────────────────────┐
│                     ZED Unity Application                        │
│              (Sends BODY_38 skeleton @ 30-60 Hz)                │
└────────────────────────┬────────────────────────────────────────┘
                         │ TCP Port 5005
                         ↓
┌─────────────────────────────────────────────────────────────────┐
│              ZEDJointReceiver (Python)                           │
│              ✅ Background thread, thread-safe buffer            │
└────────────────────────┬────────────────────────────────────────┘
                         │ get_latest_frame()
                         ↓
┌─────────────────────────────────────────────────────────────────┐
│         HumanHandoffApproachState (State 9/10)                  │
│         ✅ Tracks hand, updates target dynamically              │
└────────────────────────┬────────────────────────────────────────┘
                         │ Calls plan_trajectory()
                         ↓
┌─────────────────────────────────────────────────────────────────┐
│              HumanAwarePathPlanner                               │
│                                                                  │
│  ┌──────────────────────────────────────────────────────┐      │
│  │   HumanModelAdapter                                   │      │
│  │   ✅ ZED skeleton → PyBullet primitives              │      │
│  │   ✅ 9 spheres + capsules per person                 │      │
│  └──────────────────┬───────────────────────────────────┘      │
│                     │                                            │
│  ┌──────────────────▼───────────────────────────────────┐      │
│  │   PyBullet Planning World (DIRECT)                   │      │
│  │   ✅ Robot model loaded                              │      │
│  │   ✅ Human collision bodies updated                  │      │
│  └──────────────────┬───────────────────────────────────┘      │
│                     │                                            │
│  ┌──────────────────▼───────────────────────────────────┐      │
│  │   Path Planning                                       │      │
│  │   ❌ _placeholder_plan() ← JUST LINEAR INTERPOLATION!│      │
│  │   ⚠️  Does NOT avoid obstacles during planning       │      │
│  └──────────────────┬───────────────────────────────────┘      │
│                     │                                            │
│  ┌──────────────────▼───────────────────────────────────┐      │
│  │   Trajectory Validation                              │      │
│  │   ✅ check_trajectory_safe()                         │      │
│  │   ✅ Checks clearance at each waypoint               │      │
│  │   ✅ Returns is_safe + min_clearance                 │      │
│  └──────────────────┬───────────────────────────────────┘      │
└────────────────────────────────────────────────────────────────┘
                         │ Returns trajectory
                         ↓
┌─────────────────────────────────────────────────────────────────┐
│         State Execute Loop (50Hz)                               │
│         ✅ Speed scaling (SSM)                                  │
│         ✅ Safety stops                                         │
│         ✅ Dynamic replanning                                   │
└────────────────────────┬────────────────────────────────────────┘
                         │ move_to_joints()
                         ↓
┌─────────────────────────────────────────────────────────────────┐
│         Robot Controller (OPC UA)                               │
│         ✅ Sends commands to robot                              │
└─────────────────────────────────────────────────────────────────┘
```

---

## 🎯 Current Behavior

### Scenario 1: Human NOT blocking path ✅ Works

```
Robot: [0, 0, 0, 0, 0, 0, 0]
Target: [0.5, 0.2, 0.6]
Human: Standing at [0.8, 0.8, 0.5] (far away)

Result:
✅ Linear interpolation from start → goal
✅ All waypoints > 0.15m from human
✅ Passes safety check
✅ Executes successfully
```

### Scenario 2: Human IN the path ❌ Fails

```
Robot: [0, 0, 0, 0, 0, 0, 0]
Target: [0.5, 0.2, 0.6]
Human: Standing at [0.3, 0.2, 0.6] (directly in path!)

Result:
❌ Linear interpolation goes THROUGH human
❌ Some waypoints < 0.15m from human
❌ Fails safety check
❌ "Planning failed" - enters wait mode
⚠️  Cannot plan detour around human
```

### Scenario 3: Human hand tracking (State 9) ✅ Partially Works

```
Robot: Current position
Target: RIGHT_WRIST + [0, 0, 0.3] = [0.45, 0.23, 0.98]
Human: RIGHT_WRIST at [0.45, 0.23, 0.68]

Result:
✅ Tracks hand position dynamically
✅ Updates target as hand moves
⚠️  Linear path to hand
✅ Works if direct path is clear
❌ Fails if body is in the way
```

---

## 📝 What Each State Actually Does

### `HumanHandoffApproachState` (State 9/10)

**Current Implementation**:

```python
def execute(self):
    # 1. Get hand position from ZED ✅
    hand_pos = skeleton.get_joint_position('RIGHT_WRIST')

    # 2. Calculate target (30cm above) ✅
    target = hand_pos + [0, 0, 0.3]

    # 3. Plan path ⚠️ Uses placeholder!
    trajectory = _placeholder_plan(current_joints, target_joints)
    # This is just: [start, start+0.02*delta, start+0.04*delta, ..., goal]

    # 4. Check if safe ✅
    is_safe, clearance = check_trajectory_safe(trajectory)
    # Validates against human collision model

    # 5. Execute if safe ✅
    if is_safe:
        for waypoint in trajectory:
            robot.move_to_joints(waypoint, speed_scale)
    else:
        # ❌ Stops - can't plan around obstacle
        logger.warning("No safe path - stopping")
```

**What's Working**:

- ✅ Hand tracking and target update
- ✅ Collision model creation
- ✅ Safety validation
- ✅ Speed scaling
- ✅ Replanning triggers

**What's NOT Working**:

- ❌ Actual obstacle avoidance planning
- ❌ Cannot plan detours around human
- ❌ Will fail if human blocks direct path

---

## 🔧 The Critical Missing Piece

### What Needs to Be Implemented

**File**: `src/kinematics/human_aware_path_planner.py`

**Replace this** (line 454):

```python
def _placeholder_plan(self, start, goal):
    # Linear interpolation ❌
    return linear_trajectory
```

**With this**:

```python
def _rrt_connect_plan(self, start, goal):
    """Actual RRT-Connect using pybullet_planning."""
    from pybullet_planning import rrt_connect

    # Define collision checker
    def is_state_valid(q):
        clearance = self._compute_clearance_at_config(q)
        return clearance >= self.config['hard_min_distance']

    # Run RRT-Connect
    path = rrt_connect(
        q1=start,
        q2=goal,
        distance=joint_space_distance,
        sample=lambda: sample_joint_configuration(),
        extend=joint_space_extend,
        collision=is_state_valid,
        iterations=self.config['max_iterations']
    )

    if path is None:
        return None

    # Smooth path
    smoothed_path = shortcut_smooth(path, is_state_valid)

    return smoothed_path
```

Then replace this call (line 353):

```python
# Line 353 - in plan_trajectory()
# OLD:
trajectory = self._placeholder_plan(start_joints, goal_joints[0])

# NEW:
trajectory = self._rrt_connect_plan(start_joints, goal_joints[0])
```

---

## 💡 Why It Works At All

**The validation saves it**:

Even though planning is just linear interpolation, the system still provides value because:

1. ✅ **Safety validation**: Checks every waypoint for collisions
2. ✅ **Dynamic updates**: Tracks hand movement in real-time
3. ✅ **Speed scaling**: Slows down near humans
4. ✅ **Emergency stops**: Prevents actual collisions
5. ✅ **Replanning**: Can try again if human moves out of the way

**But** it cannot find detours around obstacles - that requires real RRT-Connect!

---

## 📊 Current vs. Desired Behavior

### Current (With Placeholder)

```
Robot at: [0, 0, 0, 0, 0, 0, 0]
Hand at: [0.5, 0.0, 0.6]
Torso at: [0.3, 0.0, 0.8] (blocking!)

Planning:
→ Linear path from robot → hand
→ Path goes through torso
→ Validation: clearance < 0.15m ❌
→ Result: "No safe path found"
→ Robot: STOPS
```

### Desired (With RRT-Connect)

```
Robot at: [0, 0, 0, 0, 0, 0, 0]
Hand at: [0.5, 0.0, 0.6]
Torso at: [0.3, 0.0, 0.8] (blocking!)

Planning:
→ RRT-Connect samples configurations
→ Explores around torso obstacle
→ Finds detour: robot → swing wide → approach from side → hand
→ Validation: all waypoints > 0.15m ✅
→ Result: "Path found!"
→ Robot: EXECUTES safely
```

---

## 🚀 To Make It Production-Ready

### Phase 1: Core Planner (CRITICAL)

**File**: `src/kinematics/human_aware_path_planner.py`

**Task**: Replace `_placeholder_plan()` with actual RRT-Connect

**Options**:

#### Option A: Use `pybullet_planning` library

```bash
pip install pybullet-planning
```

```python
from pybullet_planning import rrt_connect, plan_joint_motion

trajectory = plan_joint_motion(
    robot=self.robot_id,
    joints=self.revolute_joint_indices,
    goal=goal_joints,
    obstacles=human_body_ids,
    self_collisions=False
)
```

#### Option B: Implement custom RRT-Connect

```python
# Custom implementation
# ~200 lines of code
# Gives full control over algorithm
```

**Estimated effort**: 2-4 hours for Option A, 1-2 days for Option B

---

### Phase 2: Testing (Important)

1. **Validate coordinate frame** (1 hour)
   - Have person hold marker at known position
   - Verify ZED reports correct position
2. **Test static human avoidance** (2 hours)
   - Person stands in various positions
   - Verify robot plans around them
3. **Test dynamic tracking** (2 hours)
   - Person moves hand slowly
   - Verify robot follows and replans

---

### Phase 3: Optimization (Optional)

- Velocity-adaptive inflation
- PRM roadmap warm-starts
- TTC prediction
- Parameter tuning

---

## 📋 Quick Reference

### What Works NOW ✅

- ZED skeleton tracking (real-time)
- Human collision model creation (spheres + capsules)
- Collision distance checking (accurate)
- Speed scaling (ISO/TS 15066)
- Safety stops and recovery
- Hand position tracking
- Dynamic target updates

### What's Placeholder ⚠️

- Path planning (linear interpolation only)
- Cannot plan detours around obstacles

### What's TODO ❌

- Actual RRT-Connect implementation
- Path smoothing
- Rolling-horizon planning

---

## 🎉 Bottom Line

You have a **90% complete system** with excellent architecture, but the **critical 10%** (RRT-Connect planner) is missing.

**Current capability**: "Move toward target and stop if human is in the way"

**Desired capability**: "Move toward target by finding a path around the human"

**To achieve desired**: Implement RRT-Connect in `_placeholder_plan()` method.

---

## 🚀 Next Steps

1. **Install pybullet_planning**: `pip install pybullet-planning`
2. **Replace placeholder** with RRT-Connect implementation
3. **Test with static human** blocking various paths
4. **Tune parameters** (step size, goal bias, etc.)
5. **Validate in real scenario** with ZED tracking

Would you like me to implement the actual RRT-Connect planner now?



