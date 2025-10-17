"""
Robot configuration: kinematics, limits, safety, collision avoidance.
"""
import numpy as np

# ============================================================================
# ROBOT IDENTIFICATION
# ============================================================================

# Robot ID configuration (1-4)
# Robot ID determines namespace and node naming:
# Robot 1 -> namespace 21, nodes like R1d_Status, R1c_Joi1, etc.
# Robot 2 -> namespace 22, nodes like R2d_Status, R2c_Joi1, etc.
ROBOT_ID = 1  # Default to robot 1


def get_robot_name(robot_id: int = ROBOT_ID) -> str:
    """Get the robot name based on robot ID."""
    return f"{20 + robot_id}:robot{robot_id}"


def get_robot_namespace(robot_id: int = ROBOT_ID) -> int:
    """Get the robot namespace based on robot ID."""
    return 20 + robot_id


# ============================================================================
# KINEMATIC CHAIN
# ============================================================================

# Kinematic chain configuration for KUKA iiwa14
BASE_ELEMENT = ["base_link"]
ACTIVE_LINKS = [
    False,  # base_link - fixed
    True,   # link_1 - joint 1 (revolute)
    True,   # link_2 - joint 2 (revolute)
    True,   # link_3 - joint 3 (revolute)
    True,   # link_4 - joint 4 (revolute)
    True,   # link_5 - joint 5 (revolute)
    True,   # link_6 - joint 6 (revolute)
    True,   # link_7 - joint 7 (revolute)
    False,  # tool0 - fixed joint
    False,  # robotiq_85_base_link - fixed joint
    False   # tcp - fixed joint
]

# Joint limits for KUKA LBR iiwa 14 (in radians)
JOINT_LIMITS = {
    'A1': {'min': -2.967, 'max': 2.967},    # ±170°
    'A2': {'min': -2.094, 'max': 2.094},    # ±120°
    'A3': {'min': -2.967, 'max': 2.967},    # ±170°
    'A4': {'min': -2.094, 'max': 2.094},    # ±120°
    'A5': {'min': -2.967, 'max': 2.967},    # ±170°
    'A6': {'min': -2.094, 'max': 2.094},    # ±120°
    'A7': {'min': -3.054, 'max': 3.054},    # ±175°
}

# IK Solver Configuration
# Safety margin in degrees to prevent numerical overshoot at joint limits (0.014 rad)
IK_EPSILON_MARGIN_DEG = 0.8

# ============================================================================
# GRIPPER CONFIGURATION
# ============================================================================

GRIPPER_CONFIG = {
    'open_position': 100,
    'close_position': 20,
    'grasp_force': 50
}

# ============================================================================
# MOTION & POSES
# ============================================================================

# Pickup location for objects
PICKUP_LOCATION = {
    'position': np.array([0.0, -0.6, 0.4]),
    'approach_distance': 0.1,  # meters
    'approach_direction': np.array([0, 0, -1])  # From above
}

# Pre-pickup pose [x, y, z, rx, ry, rz] in meters and radians
PRE_PICKUP_POSE = [0.0, -0.6, 0.5, 0, 0, -1.57]

# Handoff approach pose for hand tracking
HANDOFF_APPROACH_POSE = [0.0, -0.6, 0.25, 0, 0, -1.57]

# Place operation distances
PLACE_APPROACH_DISTANCE = 0.150  # meters (150mm)
PLACE_RELEASE_DISTANCE = 0.050   # meters (50mm)

# ============================================================================
# SAFETY CONFIGURATION
# ============================================================================

SAFETY_CONFIG = {
    'max_joint_velocity': 0.5,
    'collision_threshold': 20,
    'min_distance_to_limits': 0.1,
    'emergency_deceleration': 2.0
}

# ============================================================================
# COLLISION-AWARE KINEMATICS
# ============================================================================

COLLISION_AVOIDANCE_CONFIG = {
    # Height above target for pre-approach (meters)
    'pre_approach_height_offset': 0.10,
    # Minimum clearance from table (meters)
    'min_clearance_distance': 0.02,
    # Distance for collision checking (meters)
    'collision_check_distance': 0.05,
    'trajectory_interpolation_steps': 50,     # Number of interpolation steps
    'max_ik_candidates': 5,                   # Maximum IK candidates to sample
    'nullspace_weight': 0.1,                  # Weight for nullspace bias in IK
}

# Rest poses for nullspace IK (keeps elbow up and away from table)
REST_POSES = {
    'high_elbow_1': [0.0, -1.57, 0.0, 1.57, 0.0, 1.57, 0.0],
    'high_elbow_2': [0.0, -1.2, 0.0, 1.2, 0.0, 1.2, 0.0],
    'neutral': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'safe_approach': [0.0, -0.5, 0.0, 1.0, 0.0, 0.5, 0.0],
}

# Key links for collision checking
COLLISION_CHECK_LINKS = {
    'forearm': 3,      # Link index for forearm
    'wrist': 5,        # Link index for wrist
    'camera': 6,       # Link index for camera/TCP
    'gripper': 7,      # Link index for gripper
}

# ============================================================================
# ERROR RECOVERY
# ============================================================================

ERROR_RECOVERY_CONFIG = {
    'max_retries': 3,
    'retry_delay': 1.0,
    'timeout': 20.0
}

# ============================================================================
# HUMAN BODY MODEL
# ============================================================================

HUMAN_MODEL_CONFIG = {
    # Primitive radii following ISO/TS 15066 Speed and Separation Monitoring (SSM)
    # Guidelines: Minimum protective separation = 150mm for collaborative operations
    # Formula: radius = actual_body_part_size + safety_margin
    #
    # Actual human dimensions (95th percentile male):
    #   - Head: ~100mm radius
    #   - Torso: ~150mm radius
    #   - Upper arm: ~50mm radius
    #   - Shoulder: ~70mm radius
    #
    # Safety margins applied:
    #   - Static parts (head/torso): +100mm (ISO minimum)
    #   - Dynamic parts (arms): +100mm (ISO minimum, arms move fastest)
    #   - Shoulders: +100mm (high-risk articulation point)

    # 200mm = 100mm head + 100mm safety (ISO compliant)
    'head_radius': 0.20,
    # 250mm = 150mm torso + 100mm safety (conservative)
    'torso_radius': 0.25,
    # 150mm = 50mm arm + 100mm safety (ISO minimum)
    'arm_radius': 0.15,
    # 170mm = 70mm shoulder + 100mm safety (conservative)
    'shoulder_radius': 0.17,
}

# ============================================================================
# ZED BODY TRACKING
# ============================================================================

# ZED BODY_38 joint names for tracking
TRACKED_HUMAN_JOINTS = {
    'head_neck': ['NECK', 'NOSE'],
    'torso': ['CHEST_SPINE', 'SPINE_2', 'PELVIS'],
    'left_arm': ['LEFT_SHOULDER', 'LEFT_ELBOW', 'LEFT_WRIST'],
    'right_arm': ['RIGHT_SHOULDER', 'RIGHT_ELBOW', 'RIGHT_WRIST'],
    'shoulders': ['LEFT_CLAVICLE', 'RIGHT_CLAVICLE']
}

# Manual calibration offsets for ZED joint data
# Applied to ALL received joint positions to correct Unity→Robot coordinate transform
# Adjust these values if ZED positions don't match real-world locations
ZED_MANUAL_OFFSET = {
    'x': -0.138,    # Subtract 138mm from X (meters)
    'y': 0.256,     # Add 256mm to Y (meters)
    'z': 0.05,      # Add 50mm to Z (meters)
    'enabled': True  # Set to False to disable manual offset
}
