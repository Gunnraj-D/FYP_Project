"""
Camera Transform Module - Coordinate frame transformations.
Handles transformations between camera frame and robot base frame.
"""
import numpy as np
from typing import List, Tuple
import logging

from config.config import CAMERA_TRANSLATION, CAMERA_ROTATION_EULER

logger = logging.getLogger(__name__)


def transform_camera_to_base(camera_position: List[float], tcp_matrix: np.ndarray) -> np.ndarray:
    """
    Transform position from camera frame to robot base frame.

    Args:
        camera_position: [x, y, z] position in camera frame (mm)
        tcp_matrix: 4x4 homogeneous transformation matrix from base to TCP

    Returns:
        Position in robot base frame (mm)
    """
    try:
        # Convert camera position to numpy array
        camera_pos = np.array(camera_position, dtype=float)

        # Create camera-to-TCP transformation matrix
        # Camera translation relative to TCP (from config)
        camera_translation = CAMERA_TRANSLATION * 1000  # Convert meters to mm

        # Camera rotation relative to TCP (from config)
        camera_rotation_euler = CAMERA_ROTATION_EULER
        camera_rotation_rad = np.radians([
            camera_rotation_euler['roll'],
            camera_rotation_euler['pitch'],
            camera_rotation_euler['yaw']
        ])

        # Create rotation matrix from Euler angles (ZYX order)
        from scipy.spatial.transform import Rotation as R
        camera_rotation_matrix = R.from_euler(
            'xyz', camera_rotation_rad).as_matrix()

        # Create camera-to-TCP transformation matrix
        camera_to_tcp = np.eye(4)
        camera_to_tcp[:3, :3] = camera_rotation_matrix
        camera_to_tcp[:3, 3] = camera_translation

        # Transform camera position to TCP frame
        camera_pos_homogeneous = np.append(camera_pos, 1.0)
        tcp_pos_homogeneous = camera_to_tcp @ camera_pos_homogeneous
        tcp_pos = tcp_pos_homogeneous[:3]

        # Transform TCP position to base frame
        # Note: tcp_matrix is in meters, but tcp_pos is in mm, so we need to convert
        tcp_pos_meters = tcp_pos / 1000.0  # Convert mm to meters
        base_pos_homogeneous = tcp_matrix @ np.append(tcp_pos_meters, 1.0)
        base_pos = base_pos_homogeneous[:3] * 1000.0  # Convert back to mm

        logger.debug(
            f"Camera position {camera_pos} -> Base position {base_pos}")
        return base_pos

    except Exception as e:
        logger.error(f"Failed to transform camera to base: {e}")
        return np.array([0.0, 0.0, 0.0])


def transform_base_to_camera(base_position: List[float], tcp_matrix: np.ndarray) -> np.ndarray:
    """
    Transform position from robot base frame to camera frame.

    Args:
        base_position: [x, y, z] position in base frame (mm)
        tcp_matrix: 4x4 homogeneous transformation matrix from base to TCP

    Returns:
        Position in camera frame (mm)
    """
    try:
        # Convert base position to numpy array
        base_pos = np.array(base_position, dtype=float)

        # Transform base position to TCP frame
        base_pos_homogeneous = np.append(base_pos, 1.0)
        tcp_pos_homogeneous = np.linalg.inv(tcp_matrix) @ base_pos_homogeneous
        tcp_pos = tcp_pos_homogeneous[:3]

        # Create TCP-to-camera transformation matrix
        camera_translation = CAMERA_TRANSLATION * 1000  # Convert meters to mm

        camera_rotation_euler = CAMERA_ROTATION_EULER
        camera_rotation_rad = np.radians([
            camera_rotation_euler['roll'],
            camera_rotation_euler['pitch'],
            camera_rotation_euler['yaw']
        ])

        from scipy.spatial.transform import Rotation as R
        camera_rotation_matrix = R.from_euler(
            'xyz', camera_rotation_rad).as_matrix()

        # Create TCP-to-camera transformation matrix
        tcp_to_camera = np.eye(4)
        # Transpose for inverse rotation
        tcp_to_camera[:3, :3] = camera_rotation_matrix.T
        tcp_to_camera[:3, 3] = -camera_rotation_matrix.T @ camera_translation

        # Transform TCP position to camera frame
        tcp_pos_homogeneous = np.append(tcp_pos, 1.0)
        camera_pos_homogeneous = tcp_to_camera @ tcp_pos_homogeneous
        camera_pos = camera_pos_homogeneous[:3]

        logger.debug(
            f"Base position {base_pos} -> Camera position {camera_pos}")
        return camera_pos

    except Exception as e:
        logger.error(f"Failed to transform base to camera: {e}")
        return np.array([0.0, 0.0, 0.0])


def get_camera_intrinsics_matrix(intrinsics) -> np.ndarray:
    """
    Get camera intrinsics matrix from RealSense intrinsics.

    Args:
        intrinsics: RealSense intrinsics object

    Returns:
        3x3 camera intrinsics matrix
    """
    try:
        K = np.array([
            [intrinsics.fx, 0, intrinsics.ppx],
            [0, intrinsics.fy, intrinsics.ppy],
            [0, 0, 1]
        ])
        return K
    except Exception as e:
        logger.error(f"Failed to get camera intrinsics: {e}")
        return np.eye(3)


def pixel_to_camera_frame(u: int, v: int, depth: float, intrinsics) -> Tuple[float, float, float]:
    """
    Convert pixel coordinates to camera frame coordinates.

    Args:
        u, v: Pixel coordinates
        depth: Depth value in meters
        intrinsics: Camera intrinsics

    Returns:
        (x, y, z) coordinates in camera frame (meters)
    """
    try:
        # Convert pixel to camera coordinates
        x = (u - intrinsics.ppx) * depth / intrinsics.fx
        y = (v - intrinsics.ppy) * depth / intrinsics.fy
        z = depth

        return (x, y, z)
    except Exception as e:
        logger.error(f"Failed to convert pixel to camera frame: {e}")
        return (0.0, 0.0, 0.0)


def camera_frame_to_pixel(x: float, y: float, z: float, intrinsics) -> Tuple[int, int]:
    """
    Convert camera frame coordinates to pixel coordinates.

    Args:
        x, y, z: Coordinates in camera frame (meters)
        intrinsics: Camera intrinsics

    Returns:
        (u, v) pixel coordinates
    """
    try:
        # Convert camera coordinates to pixel
        u = int(x * intrinsics.fx / z + intrinsics.ppx)
        v = int(y * intrinsics.fy / z + intrinsics.ppy)

        return (u, v)
    except Exception as e:
        logger.error(f"Failed to convert camera frame to pixel: {e}")
        return (0, 0)
