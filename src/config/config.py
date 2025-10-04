"""
Configuration file for robot hand tracking system.
Contains all system constants and parameters.
"""
import numpy as np
from pathlib import Path

# ============================================================================
# DEBUG CONFIGURATION
# ============================================================================

# Debug mode for GG-CNN2 grasp detection and frame selection
DEBUG_MODE = True

# Debug mode settings
DEBUG_CONFIG = {
    'show_live_feed': True,           # Show live camera feed in debug mode
    'frame_selection_enabled': True,  # Allow spacebar to select frames for processing
    'window_title': 'Debug Feed - Press SPACEBAR to process frame',
    'display_quality_threshold': 0.1,  # Minimum quality to show in debug display
}

# ============================================================================
# CONFIG VARIABLES
# ============================================================================

SRC_DIR = Path(__file__).parent.parent

# ============================================================================
# SYSTEM CONFIGURATION
# ============================================================================

# Main control loop rate (milliseconds)
LOOP_RATE_MS = 20  # 50Hz control loop

# ============================================================================
# OPC UA CONFIGURATION
# ============================================================================

# OPC UA Server connection
OPC_SERVER_URL = "opc.tcp://172.24.200.1:4840/"
OPC_OBJECTS_NAME = "0:Objects"

# OPC UA communication rate
OPC_UPDATE_INTERVAL_SECONDS = 0.05  # 20Hz update rate

# OPC UA Client Configuration
OPC_POLL_INTERVAL_MS = 50  # Poll interval in milliseconds (20Hz)
OPC_COMMAND_BATCH_SIZE = 10  # Maximum commands to process per loop
OPC_SKIP_REDUNDANT_WRITES = True  # Skip writes if values haven't changed
OPC_CONNECTION_TIMEOUT_SECONDS = 5.0  # Connection timeout
OPC_RECONNECT_DELAY_SECONDS = 2.0  # Delay before reconnection attempts
OPC_MAX_RECONNECT_ATTEMPTS = 5  # Maximum reconnection attempts

# OPC UA Mode Configuration
OPC_MODE = "real"  # Options: "real", "mock"
# URL for mock server (different port)
OPC_MOCK_SERVER_URL = "opc.tcp://127.0.0.1:4840/"

# ============================================================================
# ROBOT CONFIGURATION
# ============================================================================

# Robot ID configuration (1-4)
# Robot ID determines namespace and node naming:
# Robot 1 -> namespace 21, nodes like R1d_Status, R1c_Joi1, etc.
# Robot 2 -> namespace 22, nodes like R2d_Status, R2c_Joi1, etc.
# Robot 3 -> namespace 23, nodes like R3d_Status, R3c_Joi1, etc.
# Robot 4 -> namespace 24, nodes like R4d_Status, R4c_Joi1, etc.
ROBOT_ID = 1  # Default to robot 1, can be changed to 1, 2, 3, or 4


def get_robot_name(robot_id: int = ROBOT_ID) -> str:
    """Get the robot name based on robot ID."""
    return f"{20 + robot_id}:robot{robot_id}"


def get_robot_namespace(robot_id: int = ROBOT_ID) -> int:
    """Get the robot namespace based on robot ID."""
    return 20 + robot_id


# KUKA iiwa14 URDF model
URDF_FILEPATH = SRC_DIR / "resources" / "robot_models" / \
    "kuka_with_gripper.urdf"

# Kinematic chain configuration
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
# CAMERA CONFIGURATION
# ============================================================================

# Camera position relative to TCP (meters)
# For handheld camera setup - camera is essentially at TCP position
CAMERA_TRANSLATION = np.array([
    0.0,   # No forward offset
    0.0,   # No left offset
    0.0    # No vertical offset (camera held at TCP level)
])

# Camera orientation relative to TCP (degrees)
# Camera is looking down at the table, so it's rotated 180 degrees around X axis
CAMERA_ROTATION_EULER = {
    'roll': 180,    # Rotation around X axis (camera looking down)
    'pitch': 0,     # Rotation around Y axis
    'yaw': 0        # Rotation around Z axis
}

# ============================================================================
# HAND TRACKING CONFIGURATION
# ============================================================================
HAND_LOCATION_TEMP = {
    # meters in base frame (converted from mm)
    'position': np.array([0.4, 0, 0.2]),
    'approach_distance': 0.1,  # meters (converted from 100 mm)
    'approach_direction': np.array([0, 0, -1])  # From above
}

# MediaPipe hand detection model
HANDMODEL_FILEPATH = SRC_DIR / "resources" / "ml_models" / \
    "hand_landmarker.task"

# Hand tracking parameters
HAND_TRACKING_CONFIG = {
    'max_hands': 1,
    'min_confidence': 0.5,
    'palm_flatness_threshold': 0.15,
    'palm_indices': [0, 1, 2, 5, 9, 13, 17]
}

# Distance to maintain from hand (meters)
DISTANCE_TO_REMAIN_M = 0.25  # 250mm converted to meters

# How stable the hand should be to count as stable
HAND_STABILITY_THRESHOLD = 0.02  # 20mm converted to meters

# How long the hand needs to remain within the threshold to count as stable
HAND_STABILITY_TIME_THRESHOLD = 2.0  # in seconds

# ============================================================================
# OBJECT MANIPULATION CONFIGURATION
# ============================================================================
PICKUP_LOCATION = {
    # Converted from [400, 0, 200] mm to meters
    'position': np.array([0.4, 0, 0.2]),
    'approach_distance': 0.1,  # Converted from 100 mm to meters
    'approach_direction': np.array([0, 0, -1])
}

# Approximate pose for the robot to move to before starting object detection
# [x, y, z, rx, ry, rz] in meters and radians
# Converted from [400, 0, 300, 0, 0, -90] mm/deg
PRE_PICKUP_POSE = [0.4, 0, 0.5, 0, 0, -1.57]

# General pose for the robot to move to before starting hand tracking
# [x, y, z, rx, ry, rz] in meters and radians
# Converted from [500, 200, 350, 0, 0, -90] mm/deg
HANDOFF_APPROACH_POSE = [0.5, 0.2, 0.35, 0, 0, -1.57]

PLACE_APPROACH_DISTANCE = 150  # mm
PLACE_RELEASE_DISTANCE = 50    # mm

GRIPPER_CONFIG = {
    'open_position': 100,
    'close_position': 20,
    'grasp_force': 50
}

# ============================================================================
# GGCNN2 GRASP DETECTION CONFIGURATION
# ============================================================================

# GGCNN2 model path
GGCNN2_MODEL_PATH = SRC_DIR / "resources" / "ml_models" / \
    "ggcnn2_weights_cornell" / "epoch_50_cornell_statedict.pt"

# Grasp detection parameters
GRASP_DETECTION_CONFIG = {
    'min_quality_threshold': 0.5,      # Minimum grasp quality to accept
    'max_grasp_width': 100.0,         # Maximum grasp width in mm
    'min_grasp_width': 20.0,          # Minimum grasp width in mm
    'approach_height_offset': 50.0,    # Height offset for approach (mm)
    'grasp_depth_offset': 10.0,        # Depth offset for grasp (mm)
    'vertical_approach': True,         # Use vertical approach angle
    # Approach angle in degrees (vertical = -90)
    'approach_angle': -90.0,
    'frame_processing_interval': 0.5,  # Process frames every N seconds
}

# Grasp execution parameters
GRASP_EXECUTION_CONFIG = {
    'pre_grasp_delay': 1.0,           # Delay before grasping (seconds)
    'grasp_duration': 2.0,             # Time to hold grasp (seconds)
    'post_grasp_delay': 1.0,          # Delay after grasping (seconds)
    'lift_height': 100.0,              # Height to lift after grasp (mm)
    'retry_attempts': 3,               # Number of retry attempts
    'retry_delay': 2.0,                # Delay between retries (seconds)
    'grasp_generation_timeout': 10.0,  # Timeout for grasp generation (seconds)
}

# ============================================================================
# TABLE REFERENCE CONFIGURATION
# ============================================================================

# Table reference depth model parameters
TABLE_REF_UPDATE_ALPHA = 0.05         # Exponential moving average update rate
# Tolerance for table surface detection (meters)
TABLE_REF_TOLERANCE = 0.01
TABLE_REF_INITIALIZATION_FRAMES = 5   # Number of frames for initialization
TABLE_REF_MIN_DEPTH = 0.1             # Minimum valid depth (meters)
TABLE_REF_MAX_DEPTH = 3.0             # Maximum valid depth (meters)

# ============================================================================
# SAFETY CONFIGURATION
# ============================================================================
SAFETY_CONFIG = {
    'max_joint_velocity': 0.5,
    'collision_threshold': 20,
    'min_distance_to_limits': 0.1,
    'emergency_deceleration': 2.0
}

ERROR_RECOVERY_CONFIG = {
    'max_retries': 3,
    'retry_delay': 1.0,
    'timeout': 20.0
}

# ============================================================================
# LOGGING CONFIGURATION
# ============================================================================
LOGGING_CONFIG = {
    'level': 'INFO',
    'format': '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    'file': 'robot_hand_tracking.log'
}
