"""
Camera coordinate transformation module.
Transforms hand positions from camera frame to robot base frame.
"""
import numpy as np
from scipy.spatial.transform import Rotation as R
from typing import List
from config.config import CAMERA_TRANSLATION, CAMERA_ROTATION_EULER, DISTANCE_TO_REMAIN_MM


def build_camera_in_tcp() -> np.ndarray:
    """
    Build 4x4 homogeneous transformation matrix for camera in TCP frame
    from static parameters in config.
    """
    rotation = R.from_euler(
        'xyz',
        [CAMERA_ROTATION_EULER['roll'],
         CAMERA_ROTATION_EULER['pitch'],
         CAMERA_ROTATION_EULER['yaw']],
        degrees=True
    ).as_matrix()
    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = CAMERA_TRANSLATION * 1000.0  # meters -> mm
    return T


CAMERA_IN_TCP = build_camera_in_tcp()


def pose_to_homogeneous(pose: List[float]) -> np.ndarray:
    """
    Convert pose to homogeneous transformation matrix.
    pose: [x, y, z, roll, pitch, yaw] (mm, rad)
    """
    if len(pose) != 6:
        raise ValueError(f"Expected 6 pose values, got {len(pose)}")
    x, y, z, roll, pitch, yaw = pose
    rotation_matrix = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
    T = np.eye(4)
    T[:3, :3] = rotation_matrix
    T[:3, 3] = [x, y, z]
    return T


def homogeneous_to_pose(T: np.ndarray) -> List[float]:
    """
    Convert homogeneous transformation matrix to pose.
    returns [x, y, z, roll, pitch, yaw] (mm, rad)
    """
    if T.shape != (4, 4):
        raise ValueError(f"Expected 4x4 matrix, got {T.shape}")
    x, y, z = T[:3, 3]
    rotation_matrix = T[:3, :3]
    roll, pitch, yaw = R.from_matrix(rotation_matrix).as_euler('xyz')
    return [x, y, z, roll, pitch, yaw]


def transform_camera_to_base(camera_vector: List[float], tcp_pose: np.ndarray) -> np.ndarray:
    """
    Transform 3D point from camera frame (mm) to base frame (mm).
    """
    if len(camera_vector) != 3:
        raise ValueError(
            f"Expected 3D camera vector, got {len(camera_vector)} values")
    if tcp_pose.shape != (4, 4):
        raise ValueError(f"Expected 4x4 TCP pose matrix, got {tcp_pose.shape}")
    base_T_cam = tcp_pose @ CAMERA_IN_TCP
    adjusted_camera_vector = [
        camera_vector[0],
        camera_vector[1],
        camera_vector[2] - DISTANCE_TO_REMAIN_MM
    ]
    hand_cam = np.array([adjusted_camera_vector[0],
                        adjusted_camera_vector[1], adjusted_camera_vector[2], 1.0])
    hand_base = base_T_cam @ hand_cam
    return hand_base[:3]


def transform_base_to_camera(base_position: List[float], tcp_pose: np.ndarray) -> np.ndarray:
    """
    Transform 3D point from base frame (mm) to camera frame (mm).
    """
    if len(base_position) != 3:
        raise ValueError(
            f"Expected 3D position, got {len(base_position)} values")
    if tcp_pose.shape != (4, 4):
        raise ValueError(f"Expected 4x4 TCP pose matrix, got {tcp_pose.shape}")
    base_T_cam = tcp_pose @ CAMERA_IN_TCP
    cam_T_base = np.linalg.inv(base_T_cam)
    pos_base = np.array(
        [base_position[0], base_position[1], base_position[2], 1.0])
    pos_cam = cam_T_base @ pos_base
    return pos_cam[:3]
