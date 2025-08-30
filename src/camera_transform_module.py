# This module is meant to align the camera vector to coordinates from the robot base
from config import CAMERA_IN_TCP, DISTANCE_TO_REMAIN
import numpy as np
from scipy.spatial.transform import Rotation as R


def pose_to_homogeneous(pose):
    """
    pose: [x, y, z, roll, pitch, yaw]  (angles in radians)
    returns: 4x4 homogeneous transform matrix
    """
    x, y, z, roll, pitch, yaw = pose
    R_mat = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()

    T = np.eye(4)
    T[:3, :3] = R_mat
    T[:3, 3] = [x, y, z]
    return T


def transform_camera_to_base(camera_vector: list, tcp_pose):
    """
    Transform camera coordinates to robot base frame.

    Args:
        camera_vector: [x, y, z] coordinates in camera frame (mm)
        tcp_pose: 4x4 homogeneous transform matrix from base to TCP

    Returns:
        [x, y, z] coordinates in robot base frame (mm)
    """
    try:
        # Ensure tcp_pose is a numpy array
        if not isinstance(tcp_pose, np.ndarray):
            tcp_pose = np.array(tcp_pose)

        # Camera in base frame: base_T_cam = base_T_tcp * tcp_T_cam
        base_T_cam = tcp_pose @ CAMERA_IN_TCP

        # Convert vector to homogeneous point (subtract distance to remain)
        hand_cam = np.array([
            camera_vector[0],
            camera_vector[1],
            camera_vector[2] - DISTANCE_TO_REMAIN,
            1.0
        ])

        # Transform to base frame
        hand_base = base_T_cam @ hand_cam

        return hand_base[:3]

    except Exception as e:
        print(f"Error in coordinate transform: {e}")
        return None
