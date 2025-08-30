# config file to hold constants
import os
import numpy as np
from scipy.spatial.transform import Rotation as R
from pathlib import Path

# Get the project root directory
PROJECT_ROOT = Path(__file__).parent.parent
SRC_DIR = Path(__file__).parent

# MAIN CONSTANTS
LOOP_RATE_MS = 20
MAX_LOOP_OVERRUN_MS = 100  # Maximum allowed loop overrun before warning

# FRI Constants
FRI_IP = "172.24.201.001"
FRI_PORT = 30200
FRI_CONNECTION_TIMEOUT = 5.0  # seconds
FRI_RETRY_ATTEMPTS = 3

# ROBOT CONSTANTS
URDF_FILEPATH = str(SRC_DIR / "resources" / "models" / "iiwa14.urdf")
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

# Joint limits in radians (from KUKA LBR iiwa 14 R820 specifications)
JOINT_LIMITS = {
    'min': [-2.967, -2.094, -2.967, -2.094, -2.967, -2.094, -3.142],
    'max': [2.967, 2.094, 2.967, 2.094, 2.967, 2.094, 3.142]
}

# Safety constants
MAX_JOINT_VELOCITY = 1.0  # rad/s
MAX_JOINT_ACCELERATION = 2.0  # rad/s²
EMERGENCY_STOP_DISTANCE = 0.1  # meters - minimum distance to maintain from obstacles

# CAMERA CONSTANTS
# Step 2: Translation (in meters)
tx = 0.05  # forward
ty = 0.03  # left
tz = 0.02  # down

# Step 3: Rotation (in degrees)
roll = 0     # around X
pitch = -20  # around Y
yaw = 0      # around Z

translation = np.array([tx, ty, tz])
rotation = R.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_matrix()

# Step 4: Build homogeneous transform
CAMERA_IN_TCP = np.eye(4)
CAMERA_IN_TCP[:3, :3] = rotation
CAMERA_IN_TCP[:3, 3] = translation

# z height that the robot should remain away from the hand in mm
DISTANCE_TO_REMAIN = 250

# HANDTRACKING CONSTANT
HANDMODEL_FILEPATH = str(PROJECT_ROOT / "models" / "hand_landmarker.task")

# Validation constants
MIN_HAND_CONFIDENCE = 0.5
MAX_HAND_DISTANCE = 2.0  # meters - maximum distance to track hand
MIN_HAND_DISTANCE = 0.1  # meters - minimum distance to track hand


def validate_config():
    """Validate configuration settings"""
    errors = []

    # Check if files exist
    if not os.path.exists(URDF_FILEPATH):
        errors.append(f"URDF file not found: {URDF_FILEPATH}")

    if not os.path.exists(HANDMODEL_FILEPATH):
        errors.append(f"Hand model file not found: {HANDMODEL_FILEPATH}")

    # Validate joint limits
    if len(JOINT_LIMITS['min']) != 7 or len(JOINT_LIMITS['max']) != 7:
        errors.append("Joint limits must have exactly 7 values")

    # Validate loop rate
    if LOOP_RATE_MS <= 0:
        errors.append("Loop rate must be positive")

    # Validate safety constants
    if MAX_JOINT_VELOCITY <= 0 or MAX_JOINT_ACCELERATION <= 0:
        errors.append("Velocity and acceleration limits must be positive")

    if len(errors) > 0:
        raise ValueError(
            f"Configuration validation failed:\n" + "\n".join(errors))

    return True

# Validate configuration on import
# Commented out to prevent hanging during import
# Uncomment the lines below if you want to validate config on startup
# try:
#     validate_config()
# except ValueError as e:
#     print(f"WARNING: Configuration validation failed: {e}")
#     print("Some features may not work correctly.")
