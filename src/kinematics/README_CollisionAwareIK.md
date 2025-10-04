# Collision-Aware Inverse Kinematics

This module provides a collision-aware inverse kinematics solver that prevents the robot from hitting the table during motion by implementing the solutions suggested by ChatGPT 5.

## Problem Solved

The standard inverse kinematics solver returns valid joint angles for the target pose but doesn't consider:

- Collisions during the path to reach the target
- The robot sweeping through the table on the way to the goal
- Intermediate states that may collide even if the final pose is collision-free

## Solution Features

### 1. Nullspace IK with Rest Poses

- Uses PyBullet's nullspace IK with rest poses to bias solutions away from table
- Multiple rest pose configurations that keep the elbow up and away from table
- Joint limits and ranges properly configured for nullspace operation

### 2. Pre-approach Waypoints

- Generates intermediate waypoints above the target position
- Two-stage path: start → pre-approach (higher Z) → final pose
- Reduces chance of sweeping low over the table during horizontal moves

### 3. Collision Checking

- Real-time collision detection using PyBullet's `getClosestPoints`
- Checks clearance for key robot links (forearm, wrist, camera, gripper)
- Validates entire trajectory before execution

### 4. Multiple IK Candidate Sampling

- Samples multiple IK solutions with different rest poses
- Selects solution with maximum clearance from table
- Falls back to alternative solutions if collision risk detected

## Usage

### Basic Usage (Drop-in Replacement)

```python
from kinematics.collision_aware_kinematics_solver import CollisionAwareKinematicsSolver
from config.config import URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS

# Create solver (same API as standard solver)
solver = CollisionAwareKinematicsSolver(
    urdf_filepath=URDF_FILEPATH,
    base_elements=BASE_ELEMENT,
    active_links_mask=ACTIVE_LINKS,
    use_gui=False,
    table_id=table_body_id  # Optional: PyBullet body ID of table
)

# Standard IK call (now collision-aware)
joint_angles = solver.solve_XYZ(
    target_position=[0.4, 0.0, 0.5],
    current_joint_angles=[0.0, -0.5, 0.0, 1.0, 0.0, 0.5, 0.0]
)
```

### Enhanced Usage with Trajectory

```python
# Enhanced collision-aware IK with trajectory generation
final_angles, trajectory = solver.solve_XYZ_collision_aware(
    target_position=[0.4, 0.0, 0.5],
    current_joint_angles=[0.0, -0.5, 0.0, 1.0, 0.0, 0.5, 0.0],
    target_orientation=rotation_matrix,  # Optional 3x3 rotation matrix
    use_pre_approach=True  # Use pre-approach waypoint
)

# Execute trajectory safely
def execute_waypoint(joint_angles):
    # Your robot control code here
    return True  # Return success/failure

success = solver.execute_trajectory_safely(trajectory, execute_waypoint)
```

### Pose-based Targeting

```python
# Target pose as [x, y, z, rx, ry, rz]
target_pose = [0.4, 0.0, 0.5, 0, 0, -1.57]

final_angles, trajectory = solver.solve_pose_collision_aware(
    target_pose=target_pose,
    current_joint_angles=current_angles,
    use_pre_approach=True
)
```

## Configuration

The solver uses configuration parameters from `config.py`:

```python
COLLISION_AVOIDANCE_CONFIG = {
    'pre_approach_height_offset': 0.10,      # Height above target (meters)
    'min_clearance_distance': 0.02,          # Minimum clearance from table
    'collision_check_distance': 0.05,        # Distance for collision checking
    'trajectory_interpolation_steps': 50,    # Number of interpolation steps
    'max_ik_candidates': 5,                  # Maximum IK candidates to sample
    'nullspace_weight': 0.1,                 # Weight for nullspace bias
}

REST_POSES = {
    'high_elbow_1': [0.0, -1.57, 0.0, 1.57, 0.0, 1.57, 0.0],
    'high_elbow_2': [0.0, -1.2, 0.0, 1.2, 0.0, 1.2, 0.0],
    'neutral': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'safe_approach': [0.0, -0.5, 0.0, 1.0, 0.0, 0.5, 0.0],
}
```

## Key Methods

### `solve_XYZ_collision_aware(target_position, current_joint_angles, target_orientation=None, use_pre_approach=True)`

- Main collision-aware IK method
- Returns `(final_joint_angles, trajectory_waypoints)`
- Uses nullspace IK with multiple candidates
- Generates pre-approach waypoints if enabled

### `solve_pose_collision_aware(target_pose, current_joint_angles, use_pre_approach=True)`

- Pose-based collision-aware IK
- Target pose as `[x, y, z, rx, ry, rz]`
- Returns `(final_joint_angles, trajectory_waypoints)`

### `execute_trajectory_safely(trajectory, execution_callback)`

- Executes trajectory with safety checks
- Calls `execution_callback(waypoint)` for each waypoint
- Returns `True` if all waypoints executed successfully

### `_compute_clearance_score(joint_angles)`

- Computes minimum clearance distance from table
- Used for IK candidate selection
- Returns clearance in meters

## Examples

See the example files:

- `src/examples/collision_aware_ik_example.py` - Basic usage and comparison
- `src/examples/integrate_collision_aware_ik.py` - Integration examples

## Integration Steps

1. **Replace solver creation:**

   ```python
   # Old
   solver = InverseKinematicsSolver(urdf_filepath, base_elements, active_links_mask)

   # New
   solver = CollisionAwareKinematicsSolver(urdf_filepath, base_elements, active_links_mask, table_id=table_id)
   ```

2. **Update IK calls (optional):**

   ```python
   # Old
   joint_angles = solver.solve_XYZ(target_pos, current_angles)

   # New (with trajectory)
   joint_angles, trajectory = solver.solve_XYZ_collision_aware(target_pos, current_angles)
   ```

3. **Add trajectory execution:**

   ```python
   def robot_control_callback(joint_angles):
       # Send to robot controller
       return True

   success = solver.execute_trajectory_safely(trajectory, robot_control_callback)
   ```

## Benefits

- **Prevents table collisions** during robot motion
- **Minimal code changes** - drop-in replacement for existing solver
- **Configurable safety parameters** via config file
- **Real-time collision checking** during trajectory execution
- **Multiple fallback strategies** if initial solutions fail
- **Compatible with existing code** - same API as standard solver

## Requirements

- PyBullet
- NumPy
- SciPy
- Existing robot URDF file
- Optional: Table body in PyBullet for collision detection

## Notes

- The solver automatically falls back to standard IK if collision detection is not available
- Pre-approach waypoints can be disabled by setting `use_pre_approach=False`
- Collision checking is performed at each trajectory waypoint
- The solver maintains the same API as the standard `InverseKinematicsSolver` for easy integration
