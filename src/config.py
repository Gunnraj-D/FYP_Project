# config file to hold constants

# MAIN CONSTANTS
LOOP_RATE_MS = 20

# FRI Constants
FRI_IP = "172.24.201.001"
FRI_PORT = 30200


# ROBOT CONSTANTS
URDF_FILEPATH = r"C:\Users\Raj\Documents\Year_4\FYP\FYP_Project\src\resources\models\iiwa14.urdf"
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

# CAMERA CONSTANTS
# CAMERA_IN_TCP = []

import numpy as np
from scipy.spatial.transform import Rotation as R

# Step 2: Translation (in meters)
tx = 0.05  # forward
ty = 0.03  # left
tz = 0.02 # down

# Step 3: Rotation (in degrees)
roll  = 0     # around X
pitch = -20    # around Y
yaw   = 0    # around Z

translation = np.array([tx, ty, tz])
rotation = R.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_matrix()

# Step 4: Build homogeneous transform
CAMERA_IN_TCP = np.eye(4)
CAMERA_IN_TCP[:3, :3] = rotation
CAMERA_IN_TCP[:3, 3] = translation

DISTANCE_TO_REMAIN = 250 # z height that the robot should remain away from the hand in mm

# HANDTRACKING CONSTANT
HANDMODEL_FILEPATH = r"C:\Users\Raj\Documents\Year_4\FYP\FYP_Project\models\hand_landmarker.task"
