# Path Planning Architecture Design Prompt

## Context: Human-Aware Path Planning for KUKA iiwa Robot

I need architectural guidance for integrating collision-aware path planning with real-time human skeleton tracking into an existing Python-based robot control system.

---

## Current System Architecture

### Hardware

- **Robot**: KUKA LBR iiwa 14 (7-DOF collaborative robot, 14kg payload)
- **Gripper**: Robotiq 85 two-finger parallel gripper (85mm stroke)
- **Tracking**: ZED camera system with BODY_38 skeleton tracking (38 joints per person)
- **Communication**: OPC UA for robot control (20Hz update rate)

### Software Stack

- **Language**: Python 3.x
- **Physics Engine**: PyBullet (currently used for IK and collision checking)
- **Kinematics**: Custom PyBullet-based IK solver with collision-aware capabilities
- **Control Loop**: 50Hz main loop (20ms cycle time)
- **Architecture**: State machine with task sequencing

### Current Capabilities

#### 1. Kinematics Solvers

- **Base Solver** (`InverseKinematicsSolver`):

  - PyBullet-based IK with joint limit validation
  - Forward kinematics (FK) for TCP pose
  - Configurable max iterations and tolerance
  - Supports optional orientation constraints

- **Collision-Aware Solver** (`CollisionAwareKinematicsSolver`):
  - Extends base solver with table collision avoidance
  - Nullspace IK with multiple rest pose sampling
  - Pre-approach waypoint generation (vertical offset)
  - Trajectory interpolation (50 steps default)
  - Clearance scoring for IK candidate selection
  - Static table collision checking via PyBullet `getClosestPoints`

#### 2. Real-Time Human Tracking

- **ZED Joint Receiver** (`ZEDJointReceiver`):
  - TCP server receiving skeleton data from Unity/ZED SDK
  - Thread-safe frame buffering
  - 38 joints per skeleton (BODY_38_PARTS format)
  - Multiple skeleton support (up to N people)
  - Callback and polling interfaces
  - Real-time position data in Unity coordinates [x, y, z] meters

#### 3. State Machine Control

- Task sequencing (pickup, placement, etc.)
- State transitions with entry/exit hooks
- Context sharing between states
- OPC UA integration for joint commands
- Error recovery mechanisms

#### 4. Vision & Grasping

- GG-CNN2 grasp detection
- Intel RealSense D435i depth camera
- Hand-eye calibration (camera-to-TCP transform)
- Grasp pose generation with quality scoring

---

## Requirements

### Critical Human-Tracked Joints (Priority Order)

Based on safety analysis and performance considerations, monitor these specific joints:

1. **Head/Neck Region** (highest priority - avoid at all costs):

   - `NECK` - Base of neck
   - `NOSE` - Face center
   - Optional: `LEFT_EYE` or `RIGHT_EYE` - Lateral buffer for face volume

2. **Torso Core** (high priority - large obstacle):

   - `CHEST_SPINE` (aka `SPINE_2`) - Mid-chest anchor
   - `PELVIS` - Lower torso anchor
   - Together these bound the chest-abdomen volume

3. **Arms** (medium-high priority - dynamic obstacles):

   - **Each arm**: `SHOULDER`, `ELBOW`, `WRIST`
   - Focus on the **opposite arm** relative to robot's moving hand (cross-body collision risk)
   - Example: If robot moves near human's right side, track their left arm more closely

4. **Shoulder Caps** (medium priority - frequent graze points):
   - `LEFT_CLAVICLE` - Left shoulder cap
   - `RIGHT_CLAVICLE` - Right shoulder cap
   - Captures acromion region where many grazes occur

**Total**: ~10-12 joints per person for efficient real-time performance

---

## Integration Goals

### Primary Objective

Design a path planning system that:

1. **Receives** real-time human skeleton data (10-30 Hz from ZED)
2. **Plans** collision-free trajectories from current robot pose to target pose
3. **Updates** plans dynamically as humans move
4. **Executes** trajectories safely while maintaining productivity

### Performance Requirements

- **Planning Latency**: < 500ms for initial plan generation
- **Re-planning**: < 100ms for dynamic replanning
- **Control Rate**: Maintain 50Hz robot control loop
- **Safety**: Minimum clearance from human: 150-200mm (configurable)
- **Graceful Degradation**: Slow down or stop if no safe path exists

### Desired Library

- **pybullet_planning**: Preferred for integration with existing PyBullet setup
  - Already using PyBullet for kinematics
  - Familiar API for collision checking
  - Supports RRT, RRT-Connect, BiRRT variants
  - Can load URDF models directly

---

## Technical Constraints

### Robot Specifications

```python
# Joint limits (radians)
JOINT_LIMITS = {
    'A1': {'min': -2.967, 'max': 2.967},    # ±170°
    'A2': {'min': -2.094, 'max': 2.094},    # ±120°
    'A3': {'min': -2.967, 'max': 2.967},    # ±170°
    'A4': {'min': -2.094, 'max': 2.094},    # ±120°
    'A5': {'min': -2.967, 'max': 2.967},    # ±170°
    'A6': {'min': -2.094, 'max': 2.094},    # ±120°
    'A7': {'min': -3.054, 'max': 3.054},    # ±175°
}

# URDF model location
URDF_PATH = "src/resources/robot_models/kuka_with_gripper.urdf"

# Safety configuration
SAFETY_CONFIG = {
    'max_joint_velocity': 0.5,  # rad/s
    'collision_threshold': 20,   # N
    'min_distance_to_limits': 0.1,  # rad
    'emergency_deceleration': 2.0  # rad/s²
}
```

### Current Collision Avoidance Setup

```python
COLLISION_AVOIDANCE_CONFIG = {
    'pre_approach_height_offset': 0.10,  # meters
    'min_clearance_distance': 0.02,      # meters (table)
    'collision_check_distance': 0.05,    # meters
    'trajectory_interpolation_steps': 50,
    'max_ik_candidates': 5,
    'nullspace_weight': 0.1,
}

# Currently checking these robot links against table
COLLISION_CHECK_LINKS = {
    'forearm': 3,
    'wrist': 5,
    'camera': 6,
    'gripper': 7,
}
```

---

## Existing Code Structure

### Directory Layout

```
src/
├── kinematics/
│   ├── kinematics_solver.py              # Base PyBullet IK solver
│   ├── collision_aware_kinematics_solver.py  # Table collision avoidance
│   └── (new) path_planner.py             # TO BE DESIGNED
├── hand_detection/
│   ├── zed_joint_receiver.py             # Real-time skeleton tracking
│   └── ZED_JOINT_REFERENCE.md            # Joint name reference
├── states/
│   ├── state_machine.py                  # State machine controller
│   ├── move_to_state.py                  # Movement state
│   ├── grasping_state.py                 # Grasp execution
│   └── unified_hand_tracking_state.py    # Hand tracking integration
├── config/
│   └── config.py                         # All configuration constants
└── control/
    └── robot_controller.py               # OPC UA command interface
```

### Key Classes

#### Kinematics Solver Interface

```python
class InverseKinematicsSolver:
    def __init__(self, urdf_filepath, base_elements, active_links_mask, use_gui=False):
        # Loads URDF, sets up PyBullet client

    def solve_XYZ(self, target_position, current_joint_angles,
                  target_orientation=None, max_iterations=100, tolerance=1e-4):
        # Returns: np.ndarray of 7 joint angles

    def solve_tcp(self, joint_angles):
        # Forward kinematics
        # Returns: 4x4 homogeneous transform

    def tcp_from_joints(self, joint_angles_7):
        # Returns: (tcp_matrix, tcp_pose)
```

#### ZED Tracking Data Structure

```python
@dataclass
class JointData:
    joint_name: str
    x: float
    y: float
    z: float

@dataclass
class SkeletonData:
    skeleton_id: int
    joints: List[JointData]

    def get_joint_position(self, joint_name: str) -> Optional[List[float]]:
        # Returns [x, y, z] or None

@dataclass
class FrameData:
    frame: int
    skeletons: List[SkeletonData]
    timestamp: float

# Usage
receiver = ZEDJointReceiver(callback=on_frame_callback)
receiver.start()
latest_frame = receiver.get_latest_frame()
```

---

## Design Questions to Address

### 1. Architecture & Integration

**A. Component Structure**

- Should path planning be a standalone module or integrated into the collision-aware solver?
- How to structure the human model representation in PyBullet?
  - Spheres at joint positions?
  - Capsules between joints (limb segments)?
  - Bounding boxes around body regions?
- Where should the path planner live in the codebase hierarchy?

**B. Data Flow**

- How should skeleton data flow from ZED receiver → path planner → robot controller?
- Should we use a shared context/memory or publish-subscribe pattern?
- How to handle multiple skeletons (multiple people in workspace)?
- What coordinate transform is needed between ZED (Unity) and robot base frame?

### 2. Path Planning Strategy

**A. Planner Selection**

- Which algorithm from `pybullet_planning` is best for this use case?
  - RRT-Connect (fast, good for dynamic environments)?
  - BiRRT (bidirectional, faster convergence)?
  - PRM (roadmap-based, good for repeated queries)?
  - Lazy-PRM (faster initial planning)?
- Should we use different planners for different scenarios (e.g., emergency vs. normal)?

**B. Planning Modes**

- **Initial Planning**: Full path from start to goal
- **Dynamic Replanning**: When to trigger?
  - Continuous (every N frames)?
  - On significant human movement (threshold-based)?
  - When current path becomes unsafe (predictive)?
- **Execution Strategies**:
  - Plan full trajectory, then execute?
  - Rolling horizon (plan next N waypoints)?
  - Reactive (replan at each control cycle)?

### 3. Collision Representation

**A. Human Body Model**

- What geometric primitives to use for each body region?
  ```
  Head/Neck:  Sphere at NECK + NOSE? Radius?
  Torso:      Capsule CHEST_SPINE ↔ PELVIS? Or bounding box?
  Arms:       Capsule SHOULDER ↔ ELBOW ↔ WRIST? Individual spheres?
  Shoulders:  Sphere at each CLAVICLE?
  ```
- Should collision geometry inflate/deflate based on movement speed?
- How to handle occlusions or missing joint data?

**B. Safety Zones**

- **Minimum clearance**: 150mm from human body
- **Warning zone**: 300mm (slow down)
- **Comfort zone**: 500mm+ (normal speed)
- Should zones vary by body part (e.g., larger for head)?

**C. Velocity-Adaptive Safety**

- Increase safety margin if human is moving quickly?
- Decrease robot speed near humans even with safe path?
- Implement time-to-collision (TTC) predictions?

### 4. Performance Optimization

**A. Computational Budget**

- Planning budget: 500ms initial, 100ms replan
- How to balance plan quality vs. speed?
- Anytime algorithm support (improve solution over time)?
- Parallel planning (multiple threads)?

**B. Caching & Reuse**

- Cache collision geometries between frames?
- Reuse partial paths when human moves slightly?
- Pre-compute roadmaps for static environment?
- Warm-start planner with previous solution?

**C. Simplification**

- Subsample trajectory waypoints for execution?
- Reduce skeleton model complexity when far away?
- Use simplified robot model for distant checks?

### 5. Real-Time Behavior

**A. Update Strategy**

```python
# Pseudocode - which approach?

# Option 1: Continuous replanning
while executing_trajectory:
    if new_skeleton_data_available():
        replan_trajectory()

# Option 2: Threshold-based
while executing_trajectory:
    if human_moved_significantly():
        replan_trajectory()

# Option 3: Predictive
while executing_trajectory:
    if path_will_become_unsafe():
        replan_trajectory()
```

**B. Failure Handling**

- What if no collision-free path exists?
  - Stop and wait?
  - Move to safe "home" position?
  - Execute best-effort path with warnings?
- Timeout strategies for planning?
- Graceful degradation when tracking is lost?

### 6. Integration with Existing System

**A. State Machine Integration**

- Which states need path planning?
  - `MoveToState`: Always plan paths
  - `GraspingState`: Only during approach/retreat?
  - `HandTrackingState`: Plan while human is detected?
- How to signal state machine when planning fails?

**B. API Design**

```python
# Proposed interface - please critique/improve

class HumanAwarePathPlanner:
    def __init__(self, urdf_path, zed_receiver, collision_config):
        """Initialize planner with robot model and tracking source."""

    def plan_trajectory(self, start_joints, goal_pose, current_skeleton_data):
        """
        Plan collision-free trajectory.

        Args:
            start_joints: Current robot joint angles [7]
            goal_pose: Target TCP pose [x,y,z,rx,ry,rz] or [x,y,z]
            current_skeleton_data: Latest FrameData from ZED

        Returns:
            trajectory: List[List[float]] - joint angle waypoints
            metadata: Dict with planning time, clearance info, etc.
        """

    def check_trajectory_safe(self, trajectory, current_skeleton_data):
        """Check if existing trajectory is still collision-free."""

    def update_human_model(self, skeleton_data):
        """Update internal collision model with latest skeleton."""

    def get_clearance_to_human(self, joint_angles):
        """Return minimum distance to human for given robot config."""
```

**C. Configuration**

- New config parameters needed in `config.py`?
- How to tune planning aggressiveness vs. conservativeness?
- User-adjustable safety zones?

### 7. Testing & Validation

**A. Test Scenarios**

- How to unit test path planner without real hardware/tracking?
- Mock skeleton data generation for testing?
- Benchmark suite for planning performance?
- Simulation environment for validation?

**B. Metrics**

- Planning success rate
- Average planning time
- Minimum clearance achieved
- False-stop rate (stopping when unnecessary)
- Trajectory efficiency (path length, smoothness)

---

## Specific Technical Questions

### PyBullet & pybullet_planning

1. **Collision World Setup**

   - How to efficiently add/update human collision geometries in PyBullet?
   - Use separate PyBullet client for planning vs. kinematics?
   - Synchronization between planner's world and IK solver's world?

2. **pybullet_planning API**

   - Best practices for dynamic obstacle handling?
   - How to configure planner parameters (step size, goal bias, etc.)?
   - Support for differential constraints (joint velocity limits)?

3. **Performance**
   - Typical RRT-Connect planning times for 7-DOF with 10 obstacles?
   - Is GPU acceleration possible/worthwhile?
   - Memory usage for large planning sessions?

### Coordinate Transforms

4. **Frame Management**
   - ZED skeleton data is in Unity/camera coordinates
   - Robot operates in base frame
   - Current hand-eye matrix: `tcp_T_camera` (TCP → camera)
   - Need: `base_T_unity` (robot base → Unity world)
   - Where/how to define and apply this transform?

### Integration Patterns

5. **Concurrency**

   - Should path planning run in separate thread?
   - Lock-free data sharing with ZED receiver?
   - How to interrupt long planning operations?

6. **Error Propagation**
   - How should planning failures propagate to state machine?
   - Exception handling vs. return codes?
   - Logging and telemetry strategy?

---

## Deliverable Request

Please provide:

1. **High-Level Architecture Diagram**

   - Component layout
   - Data flow between modules
   - Thread/process boundaries

2. **Detailed Design Document** covering:

   - Recommended path planning algorithm and rationale
   - Human body collision model (geometric primitives, dimensions)
   - Update strategy (when to replan)
   - Integration points with existing code
   - Configuration parameters needed

3. **Implementation Roadmap**

   - Phase 1: Core planner with static human
   - Phase 2: Dynamic replanning
   - Phase 3: Velocity-adaptive safety
   - Phase 4: Optimization and tuning

4. **Pseudo-code or Code Sketches** for:

   - Main `HumanAwarePathPlanner` class
   - Skeleton → PyBullet collision geometry conversion
   - Path planning call with human avoidance
   - Trajectory validation during execution

5. **Testing Strategy**

   - Unit tests for key components
   - Integration test scenarios
   - Performance benchmarks

6. **Configuration Template**

   - New entries needed in `config.py`
   - Tunable parameters with recommended defaults

7. **Risk Assessment**
   - Potential failure modes
   - Mitigation strategies
   - Performance bottlenecks

---

## Example Usage (Desired)

```python
# In a state (e.g., MoveToState)

from kinematics.path_planner import HumanAwarePathPlanner

class MoveToState(BaseState):
    def enter(self):
        # Initialize planner
        self.planner = HumanAwarePathPlanner(
            urdf_path=URDF_FILEPATH,
            zed_receiver=self.context.zed_receiver,
            config=PATH_PLANNING_CONFIG
        )

        # Plan initial trajectory
        skeleton_data = self.context.zed_receiver.get_latest_frame()
        self.trajectory, self.metadata = self.planner.plan_trajectory(
            start_joints=self.context.current_joint_angles,
            goal_pose=self.target_pose,
            current_skeleton_data=skeleton_data
        )

        if self.trajectory is None:
            logger.error("No collision-free path found!")
            self.fail()
            return

        logger.info(f"Planned path with {len(self.trajectory)} waypoints, "
                   f"min clearance: {self.metadata['min_clearance']:.3f}m")

    def execute(self):
        # Check if trajectory still safe
        skeleton_data = self.context.zed_receiver.get_latest_frame()

        if not self.planner.check_trajectory_safe(self.trajectory, skeleton_data):
            logger.warning("Trajectory unsafe, replanning...")
            self.trajectory, self.metadata = self.planner.plan_trajectory(
                start_joints=self.context.current_joint_angles,
                goal_pose=self.target_pose,
                current_skeleton_data=skeleton_data
            )

        # Execute next waypoint
        next_waypoint = self.trajectory[self.waypoint_index]
        self.context.robot_controller.move_to_joints(next_waypoint)
        self.waypoint_index += 1

        if self.waypoint_index >= len(self.trajectory):
            self.complete()
```

---

## Additional Context

### Current Workflow (Without Path Planning)

1. Grasp detection finds target object
2. IK solver computes joint angles for grasp pose
3. Robot moves directly to target (joint-space interpolation)
4. Gripper closes, lifts object
5. Moves to placement location

**Problem**: No awareness of humans in workspace during motion.

### Desired Workflow (With Path Planning)

1. Grasp detection finds target object
2. ZED tracking provides human skeleton(s)
3. **Path planner computes collision-free trajectory** considering humans
4. Robot executes trajectory with **continuous replanning**
5. **Stops or slows down** if human enters path
6. Gripper closes, lifts object
7. **Plans return path** avoiding humans
8. Moves to placement location

---

## References & Resources

### Libraries

- **pybullet_planning**: https://github.com/caelan/pybullet-planning
- **PyBullet**: https://pybullet.org/
- **ZED SDK**: Skeleton tracking with BODY_38 format

### Papers/Concepts

- RRT-Connect for dynamic environments
- Time-to-collision (TTC) safety metrics
- Nullspace IK for redundant robots
- Human-aware motion planning

### Our Codebase

- `src/kinematics/kinematics_solver.py` - Current IK implementation
- `src/kinematics/collision_aware_kinematics_solver.py` - Table collision avoidance
- `src/hand_detection/zed_joint_receiver.py` - Skeleton tracking receiver
- `src/config/config.py` - System configuration

---

## Priority Focus Areas

Please prioritize guidance on:

1. ⭐ **Algorithm selection** (RRT variant, configuration)
2. ⭐ **Human body collision model** (specific geometry recommendations)
3. ⭐ **Replanning strategy** (when/how to update paths)
4. **Integration with existing PyBullet IK solver**
5. **Performance optimization** for real-time operation
6. **Safety zone definitions** and tuning

---

## Thank You!

I'm looking for comprehensive architectural guidance that balances:

- **Safety**: Reliable collision avoidance with humans
- **Performance**: Real-time operation at 50Hz control loop
- **Practicality**: Integrates cleanly with existing codebase
- **Robustness**: Handles edge cases (occlusions, tracking loss, planning failures)

Please feel free to ask clarifying questions or suggest alternative approaches if you see better solutions!
