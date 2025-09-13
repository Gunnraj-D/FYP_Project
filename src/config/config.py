"""
Configuration file for robot hand tracking system.
Contains all system constants and parameters.
"""
import numpy as np
from pathlib import Path

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
OPC_ROBOT_NAME = "22:robot1"  # Assuming robot1 based on R1c/R1d naming

# OPC UA communication rate
OPC_UPDATE_INTERVAL_SECONDS = 0.05  # 20Hz update rate

# OPC UA Client Configuration
OPC_POLL_INTERVAL_MS = 50  # Poll interval in milliseconds (20Hz)
OPC_COMMAND_BATCH_SIZE = 10  # Maximum commands to process per loop
OPC_SKIP_REDUNDANT_WRITES = True  # Skip writes if values haven't changed
OPC_CONNECTION_TIMEOUT_SECONDS = 5.0  # Connection timeout
OPC_RECONNECT_DELAY_SECONDS = 2.0  # Delay before reconnection attempts
OPC_MAX_RECONNECT_ATTEMPTS = 5  # Maximum reconnection attempts

# ============================================================================
# ROBOT CONFIGURATION
# ============================================================================

# KUKA iiwa14 URDF model
URDF_FILEPATH = SRC_DIR / "resources" / "robot_models" / \
    "kuka_with_gripper.urdf"

# Kinematic chain configuration
BASE_ELEMENT = ["base_link"]
ACTIVE_LINKS = [
    False,  # base link - fixed
    True,   # joint 1
    True,   # joint 2
    True,   # joint 3
    True,   # joint 4
    True,   # joint 5
    True,   # joint 6
    True,   # joint 7
    False   # end-effector - fixed
]

# ============================================================================
# CAMERA CONFIGURATION
# ============================================================================

# Camera position relative to TCP (meters)
CAMERA_TRANSLATION = np.array([
    0.05,   # 50mm forward (x)
    0.03,   # 30mm left (y)
    0.02    # 20mm down (z)
])

# Camera orientation relative to TCP (degrees)
CAMERA_ROTATION_EULER = {
    'roll': 0,      # Rotation around X axis
    'pitch': -20,   # Rotation around Y axis (tilt down)
    'yaw': 0        # Rotation around Z axis
}

# ============================================================================
# HAND TRACKING CONFIGURATION
# ============================================================================
HAND_LOCATION_TEMP = {
    'position': np.array([400, 0, 200]),  # mm in base frame
    'approach_distance': 100,  # mm
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

# Distance to maintain from hand (mm)
DISTANCE_TO_REMAIN_MM = 250

# How stable the hand should be to count as stable
HAND_STABILITY_THRESHOLD = 20.0  # in mm

# How long the hand needs to remain within the threshold to count as stable
HAND_STABILITY_TIME_THRESHOLD = 2.0  # in seconds

# ============================================================================
# OBJECT MANIPULATION CONFIGURATION
# ============================================================================
PICKUP_LOCATION = {
    'position': np.array([400, 0, 200]),
    'approach_distance': 100,
    'approach_direction': np.array([0, 0, -1])
}

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
}

# Grasp execution parameters
GRASP_EXECUTION_CONFIG = {
    'pre_grasp_delay': 1.0,           # Delay before grasping (seconds)
    'grasp_duration': 2.0,             # Time to hold grasp (seconds)
    'post_grasp_delay': 1.0,          # Delay after grasping (seconds)
    'lift_height': 100.0,              # Height to lift after grasp (mm)
    'retry_attempts': 3,               # Number of retry attempts
    'retry_delay': 2.0,                # Delay between retries (seconds)
}

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
    'timeout': 30.0
}

# ============================================================================
# LOGGING CONFIGURATION
# ============================================================================
LOGGING_CONFIG = {
    'level': 'INFO',
    'format': '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    'file': 'robot_hand_tracking.log'
}
