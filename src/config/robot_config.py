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
    'position': np.array([0.39, 0.06, 0.25]),
    'approach_distance': 0.1,  # meters
    'approach_direction': np.array([0, 0, -1])  # From above
}

# Pre-pickup pose [x, y, z, rx, ry, rz] in meters and radians
PRE_PICKUP_POSE = [0.4, 0, 0.5, 0, 0, -1.57]

# Handoff approach pose for hand tracking
HANDOFF_APPROACH_POSE = [0.5, 0.2, 0.35, 0, 0, -1.57]

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
# HUMAN-AWARE PATH PLANNING
# ============================================================================

PATH_PLANNING_CONFIG = {
    # Planner algorithm
    'planner_type': 'rrt_connect',     # 'rrt_connect', 'birrt', 'prm'
    'step_size': 0.15,                 # Joint space step size (radians)
    'goal_bias': 0.3,                  # Probability of sampling goal
    'max_iterations': 2000,            # Maximum planning iterations
    'planning_timeout': 0.5,           # Initial planning timeout (seconds)
    'replan_timeout': 0.1,             # Replanning timeout (seconds)
    'smoothing_iterations': 20,        # Post-processing smoothing passes

    # Rolling horizon
    'horizon_time': 0.5,               # Plan ahead time (seconds)
    'min_replan_interval': 0.5,        # Minimum time between replans (seconds)
    'replan_threshold_position': 0.10,  # Replan if human moves >10cm
    'replan_threshold_velocity': 0.30,  # Replan if human speed >0.3m/s

    # SSM (Speed and Separation Monitoring) zones per ISO/TS 15066
    'comfort_distance': 0.50,          # ≥0.5m: normal speed (100%)
    'warning_distance': 0.30,          # ≥0.3m: reduced speed (50-100%)
    'hard_min_distance': 0.12,         # ≥0.12m: critical/stop (0-50%)
    'emergency_stop_distance': 0.08,   # <0.08m: immediate stop

    # Speed scaling
    'speed_scale_comfort': 1.0,        # 100% speed in comfort zone
    'speed_scale_warning': 0.5,        # 50% speed in warning zone
    'speed_scale_critical': 0.1,       # 10% speed near hard minimum

    # Safety behavior
    # Max time to wait when stopped (seconds)
    'max_safety_wait_time': 10.0,
    'collision_check_distance': 1.0,   # Distance to check collisions (meters)

    # ISO/TS 15066 SSM parameters
    'reaction_time': 0.2,              # System reaction time T_r (seconds)
    'stop_time_max': 1.0,              # Maximum stop time T_s (seconds)
    'intrusion_distance': 0.05,        # Intrusion tolerance C (meters)
    'position_uncertainty': 0.03,      # ZED position uncertainty Z_d (meters)
    # Robot position uncertainty Z_r (meters)
    'robot_uncertainty': 0.02,
}

# ============================================================================
# HUMAN BODY MODEL
# ============================================================================

HUMAN_MODEL_CONFIG = {
    # Primitive radii (inflated for safety margin)
    'head_radius': 0.10,               # 100mm sphere at NECK/NOSE
    'torso_radius': 0.08,              # 80mm capsule
    'arm_radius': 0.05,                # 50mm capsule
    'shoulder_radius': 0.07,           # 70mm sphere

    # Velocity-adaptive safety
    'velocity_inflation_enabled': True,
    # Additional radius = v_human * factor (s)
    'velocity_inflation_factor': 0.15,
    'max_velocity_inflation': 0.10,     # Cap additional inflation at 100mm

    # Tracking confidence
    'min_joint_confidence': 0.5,        # Drop joints below this confidence
    'tracking_timeout': 1.0,            # Keep last position for 1s after loss

    # Model limits
    'max_primitives_per_person': 15,    # Maximum collision primitives
}

# ZED BODY_38 joint names for tracking
TRACKED_HUMAN_JOINTS = {
    'head_neck': ['NECK', 'NOSE'],
    'torso': ['CHEST_SPINE', 'SPINE_2', 'PELVIS'],
    'left_arm': ['LEFT_SHOULDER', 'LEFT_ELBOW', 'LEFT_WRIST'],
    'right_arm': ['RIGHT_SHOULDER', 'RIGHT_ELBOW', 'RIGHT_WRIST'],
    'shoulders': ['LEFT_CLAVICLE', 'RIGHT_CLAVICLE']
}
