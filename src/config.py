"""
Configuration file for robot hand tracking system.
Contains all system constants and parameters.
"""
import numpy as np
from scipy.spatial.transform import Rotation as R

# ============================================================================
# SYSTEM CONFIGURATION
# ============================================================================

# Main control loop rate (milliseconds)
LOOP_RATE_MS = 20  # 50Hz control loop

# System modes
SYSTEM_MODES = {
    'IDLE': 'Robot maintains current position',
    'TRACKING': 'Robot tracks detected hand',
    'PICKUP': 'Robot performs pickup sequence',
    'PLACE': 'Robot performs place sequence'
}

# ============================================================================
# OPC UA CONFIGURATION
# ============================================================================

# OPC UA Server connection
OPC_SERVER_URL = "opc.tcp://172.24.200.1:4840/"
OPC_OBJECTS_NAME = "0:Objects"
OPC_ROBOT_NAME = "22:robot1"  # Assuming robot1 based on R1c/R1d naming

# OPC UA communication rate
OPC_UPDATE_INTERVAL = 0.005  # 200Hz update rate

# ============================================================================
# ROBOT CONFIGURATION
# ============================================================================

# KUKA iiwa14 URDF model
URDF_FILEPATH = r"C:\Users\Raj\Documents\Year_4\FYP\FYP_Project\src\resources\models\iiwa14.urdf"

# Kinematic chain configuration
BASE_ELEMENT = ["iiwa_link_0"]
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

# Joint limits (radians)
JOINT_LIMITS = {
    'min': np.array([-2.96, -2.09, -2.96, -2.09, -2.96, -2.09, -3.05]),
    'max': np.array([2.96, 2.09, 2.96, 2.09, 2.96, 2.09, 3.05])
}

# Joint velocity limits (rad/s)
JOINT_VELOCITY_LIMITS = np.array([1.48, 1.48, 1.74, 1.74, 2.61, 2.61, 2.61])

# Workspace limits (mm)
WORKSPACE_LIMITS = {
    'min': np.array([-800, -800, 0]),
    'max': np.array([800, 800, 1300])
}

# ============================================================================
# CAMERA CONFIGURATION
# ============================================================================

# Camera transformation relative to TCP (Tool Center Point)
# This defines where the camera is mounted relative to the robot's end-effector

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

# Build homogeneous transformation matrix for camera in TCP frame
def build_camera_transform():
    """Build 4x4 homogeneous transformation matrix for camera in TCP frame."""
    # Convert rotation to matrix
    rotation = R.from_euler(
        'xyz', 
        [CAMERA_ROTATION_EULER['roll'],
         CAMERA_ROTATION_EULER['pitch'],
         CAMERA_ROTATION_EULER['yaw']],
        degrees=True
    ).as_matrix()
    
    # Build homogeneous transform
    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = CAMERA_TRANSLATION * 1000  # Convert to mm
    return T

CAMERA_IN_TCP = build_camera_transform()

# ============================================================================
# HAND TRACKING CONFIGURATION
# ============================================================================

# MediaPipe hand detection model
HANDMODEL_FILEPATH = r"C:\Users\Raj\Documents\Year_4\FYP\FYP_Project\models\hand_landmarker.task"

# Hand tracking parameters
HAND_TRACKING_CONFIG = {
    'max_hands': 1,
    'min_confidence': 0.5,
    'palm_flatness_threshold': 0.15,
    'palm_indices': [0, 1, 2, 5, 9, 13, 17]  # Landmarks for palm centroid
}

# Distance to maintain from hand (mm)
DISTANCE_TO_REMAIN = 250  # Robot stays 250mm away from detected hand

# Hand stability detection
HAND_STABILITY_THRESHOLD = 10.0  # Position change threshold (mm)
HAND_STABILITY_DURATION = 2.0    # Required stable duration (seconds)

# ============================================================================
# OBJECT MANIPULATION CONFIGURATION
# ============================================================================

# Pickup location (placeholder - should be dynamically determined)
PICKUP_LOCATION = {
    'position': np.array([400, 0, 200]),  # mm in base frame
    'approach_distance': 100,  # mm
    'approach_direction': np.array([0, 0, -1])  # From above
}

# Place location parameters
PLACE_APPROACH_DISTANCE = 150  # mm
PLACE_RELEASE_DISTANCE = 50    # mm

# Gripper configuration (if applicable)
GRIPPER_CONFIG = {
    'open_position': 100,   # mm or percentage
    'close_position': 20,   # mm or percentage
    'grasp_force': 50       # Newtons
}

# ============================================================================
# SAFETY CONFIGURATION
# ============================================================================

# Safety limits
SAFETY_CONFIG = {
    'max_joint_velocity': 0.5,     # Fraction of max velocity
    'collision_threshold': 20,      # Newtons
    'min_distance_to_limits': 0.1,  # Radians from joint limits
    'emergency_deceleration': 2.0   # rad/s^2
}

# Error recovery
ERROR_RECOVERY_CONFIG = {
    'max_retries': 3,
    'retry_delay': 1.0,  # seconds
    'timeout': 30.0      # seconds for operations
}

# ============================================================================
# LOGGING CONFIGURATION
# ============================================================================

LOGGING_CONFIG = {
    'level': 'INFO',
    'format': '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    'file': 'robot_hand_tracking.log'
}