"""
Camera Transform Module - Coordinate frame transformations.
Handles transformations between camera frame and robot base frame.
"""
import numpy as np
from typing import List, Tuple
import logging

from config.config import CAMERA_TRANSLATION, CAMERA_ROTATION_EULER
import config.config as config_module

logger = logging.getLogger(__name__)

# Log the active camera transform mode on module load
logger.info(f"Camera Transform Mode: {config_module.CAMERA_TRANSFORM_MODE}")


def transform_camera_to_base(camera_position: List[float], tcp_matrix: np.ndarray) -> np.ndarray:
    """
    Transform position from camera frame to robot base frame using calibrated hand-eye matrix.

    Args:
        camera_position: [x, y, z] position in camera frame (meters)
        tcp_matrix: 4x4 homogeneous transformation matrix from base to TCP

    Returns:
        Position in robot base frame (meters)
    """
    try:
        # Convert camera position to numpy array
        camera_pos = np.array(camera_position, dtype=float)

        # Use hand-eye matrix for camera-to-TCP transformation (dynamically read from config)
        camera_to_tcp = config_module.HAND_EYE_MATRIX

        # Transform camera position to TCP frame
        camera_pos_homogeneous = np.append(camera_pos, 1.0)
        tcp_pos_homogeneous = camera_to_tcp @ camera_pos_homogeneous
        tcp_pos = tcp_pos_homogeneous[:3]

        logger.info(
            f"Camera->TCP transformation ({config_module.CAMERA_TRANSFORM_MODE}): {camera_pos} -> {tcp_pos}")

        # Transform TCP position to base frame
        base_pos_homogeneous = tcp_matrix @ np.append(tcp_pos, 1.0)
        base_pos = base_pos_homogeneous[:3]

        logger.info(f"TCP->Base transformation: {tcp_pos} -> {base_pos}")
        logger.debug(
            f"Camera position {camera_pos} -> Base position {base_pos}")
        return base_pos

    except Exception as e:
        logger.error(f"Failed to transform camera to base: {e}")
        return np.array([0.0, 0.0, 0.0])


def transform_base_to_camera(base_position: List[float], tcp_matrix: np.ndarray) -> np.ndarray:
    """
    Transform position from robot base frame to camera frame using calibrated hand-eye matrix.

    Args:
        base_position: [x, y, z] position in base frame (meters)
        tcp_matrix: 4x4 homogeneous transformation matrix from base to TCP

    Returns:
        Position in camera frame (meters)
    """
    try:
        # Convert base position to numpy array
        base_pos = np.array(base_position, dtype=float)

        # Transform base position to TCP frame
        base_pos_homogeneous = np.append(base_pos, 1.0)
        tcp_pos_homogeneous = np.linalg.inv(tcp_matrix) @ base_pos_homogeneous
        tcp_pos = tcp_pos_homogeneous[:3]

        # Use hand-eye matrix for TCP-to-camera transformation (inverse, dynamically read from config)
        tcp_to_camera = np.linalg.inv(config_module.HAND_EYE_MATRIX)

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
