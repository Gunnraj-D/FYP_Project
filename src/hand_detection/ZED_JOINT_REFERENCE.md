# ZED SDK BODY_38 Joint Reference

This document lists all 38 joint names available from the ZED SDK body tracking system.

## Joint Names (BODY_38_PARTS Enum)

### Head & Torso

- `PELVIS` (0) - Base of spine
- `NAVAL_SPINE` (1) - Navel level
- `CHEST_SPINE` (2) - Chest level
- `NECK` (3) - Base of neck
- `HEAD` (4) - Top of head
- `NOSE` (5) - Nose
- `LEFT_EYE` (6) - Left eye
- `RIGHT_EYE` (7) - Right eye
- `LEFT_EAR` (8) - Left ear
- `RIGHT_EAR` (9) - Right ear

### Left Arm

- `LEFT_CLAVICLE` (10) - Left collarbone
- `LEFT_SHOULDER` (11) - Left shoulder
- `LEFT_ELBOW` (12) - Left elbow
- `LEFT_WRIST` (13) - Left wrist
- `LEFT_HAND` (14) - Left hand center
- `LEFT_HANDTIP` (15) - Left hand tip
- `LEFT_THUMB` (16) - Left thumb

### Right Arm

- `RIGHT_CLAVICLE` (17) - Right collarbone
- `RIGHT_SHOULDER` (18) - Right shoulder
- `RIGHT_ELBOW` (19) - Right elbow
- `RIGHT_WRIST` (20) - Right wrist
- `RIGHT_HAND` (21) - Right hand center
- `RIGHT_HANDTIP` (22) - Right hand tip
- `RIGHT_THUMB` (23) - Right thumb

### Left Leg

- `LEFT_HIP` (24) - Left hip
- `LEFT_KNEE` (25) - Left knee
- `LEFT_ANKLE` (26) - Left ankle
- `LEFT_FOOT` (27) - Left foot
- `LEFT_HEEL` (28) - Left heel
- `LEFT_FOOT_INDEX` (29) - Left foot index toe

### Right Leg

- `RIGHT_HIP` (30) - Right hip
- `RIGHT_KNEE` (31) - Right knee
- `RIGHT_ANKLE` (32) - Right ankle
- `RIGHT_FOOT` (33) - Right foot
- `RIGHT_HEEL` (34) - Right heel
- `RIGHT_FOOT_INDEX` (35) - Right foot index toe

### Additional

- `LEFT_HAND_THUMB_4` (36) - Additional left thumb joint
- `RIGHT_HAND_THUMB_4` (37) - Additional right thumb joint

## Usage in Python

```python
from hand_detection.zed_joint_receiver import ZEDJointReceiver

def on_frame(frame_data):
    for skeleton in frame_data.skeletons:
        # Get specific joint by name
        right_hand = skeleton.get_joint_by_name("RIGHT_HAND")
        if right_hand:
            print(f"Right hand at: ({right_hand.x}, {right_hand.y}, {right_hand.z})")

        # Get position as list
        left_wrist_pos = skeleton.get_joint_position("LEFT_WRIST")
        if left_wrist_pos:
            x, y, z = left_wrist_pos
            print(f"Left wrist at: ({x}, {y}, {z})")

receiver = ZEDJointReceiver(callback=on_frame)
receiver.start()
```

## Key Joints for Robot Interaction

For safety and collision avoidance, monitor these critical joints:

### Hand Positions

- `RIGHT_HAND` / `LEFT_HAND` - Center of palm
- `RIGHT_WRIST` / `LEFT_WRIST` - Wrist joint
- `RIGHT_HANDTIP` / `LEFT_HANDTIP` - Fingertip position

### Arm Positions

- `RIGHT_ELBOW` / `LEFT_ELBOW` - Elbow joints
- `RIGHT_SHOULDER` / `LEFT_SHOULDER` - Shoulder joints

### Torso (for full body tracking)

- `CHEST_SPINE` - Upper body position
- `HEAD` - Head position for safety zones

## Coordinate System

The joint positions are provided in Unity's coordinate system:

- **X**: Left (-) to Right (+)
- **Y**: Down (-) to Up (+)
- **Z**: Back (-) to Forward (+)

You may need to transform these coordinates to match your robot's coordinate system.

## Example: Workspace Monitoring

```python
def is_hand_in_robot_workspace(skeleton_data, workspace_bounds):
    """Check if either hand is in the robot's workspace."""
    hands = ['RIGHT_HAND', 'LEFT_HAND']

    for hand_name in hands:
        hand_pos = skeleton_data.get_joint_position(hand_name)
        if hand_pos:
            x, y, z = hand_pos
            if (workspace_bounds['x_min'] <= x <= workspace_bounds['x_max'] and
                workspace_bounds['y_min'] <= y <= workspace_bounds['y_max'] and
                workspace_bounds['z_min'] <= z <= workspace_bounds['z_max']):
                return True, hand_name, hand_pos

    return False, None, None
```

## Notes

- Not all joints may be available/tracked in every frame
- Joint confidence/quality is not included in the current Unity sender (could be added)
- Joint positions are in meters in Unity space
- Multiple skeletons can be tracked simultaneously (different IDs)
