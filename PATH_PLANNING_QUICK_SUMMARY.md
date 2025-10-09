# Path Planning Quick Summary

## What We Have

- **Robot**: KUKA iiwa 14 (7-DOF) with Robotiq gripper
- **Tracking**: Real-time ZED skeleton data (38 joints/person, TCP stream)
- **Kinematics**: PyBullet-based IK solver with table collision avoidance
- **Control**: 50Hz loop, state machine architecture, OPC UA communication

## What We Need

Human-aware path planning using **pybullet_planning** that:

- Plans collision-free trajectories avoiding humans
- Replans dynamically as humans move
- Maintains 50Hz control rate
- < 500ms initial planning, < 100ms replanning

## Critical Human Joints to Track (10-12 total)

1. **Head**: NECK, NOSE (±eye for buffer)
2. **Torso**: CHEST_SPINE, PELVIS
3. **Arms**: SHOULDER, ELBOW, WRIST (focus on opposite arm)
4. **Shoulders**: LEFT_CLAVICLE, RIGHT_CLAVICLE

## Key Design Questions

1. Which RRT variant (RRT-Connect, BiRRT, PRM)?
2. How to model human body (spheres, capsules, boxes)?
3. When to replan (continuous, threshold, predictive)?
4. Safety zones (150mm min, 300mm warning, 500mm comfort)?
5. How to integrate with existing collision-aware IK solver?

## Desired API

```python
planner = HumanAwarePathPlanner(urdf, zed_receiver, config)
trajectory, metadata = planner.plan_trajectory(
    start_joints, goal_pose, skeleton_data
)
is_safe = planner.check_trajectory_safe(trajectory, skeleton_data)
```

## Integration Points

- Data source: `ZEDJointReceiver.get_latest_frame()`
- Kinematics: `InverseKinematicsSolver`, `CollisionAwareKinematicsSolver`
- Execution: State machine (`MoveToState`, etc.)
- Config: Add to `config.py`

## Success Criteria

- ✅ No collisions with humans (150mm+ clearance)
- ✅ Real-time performance (< 100ms replanning)
- ✅ Graceful handling of no-path scenarios
- ✅ Clean integration with existing codebase

## Files to Reference

- Full prompt: `PATH_PLANNING_ARCHITECTURE_PROMPT.md`
- Current IK: `src/kinematics/kinematics_solver.py`
- Collision-aware: `src/kinematics/collision_aware_kinematics_solver.py`
- Tracking: `src/hand_detection/zed_joint_receiver.py`
- Config: `src/config/config.py`
