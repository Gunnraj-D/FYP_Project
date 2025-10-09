"""
Camera Transform Module - Coordinate frame transformations.
Handles transformations between camera frame and robot base frame.

IMPORTANT - Pixel to 3D Conversion:
    For converting pixel coordinates + depth to 3D camera coordinates, 
    use CameraManager.pixel_to_3d() which uses the RealSense SDK function
    rs.rs2_deproject_pixel_to_point(). This is the canonical method and handles
    lens distortion correctly.
    
    This module focuses on higher-level coordinate frame transformations 
    (camera -> TCP -> base) rather than low-level pixel deprojection.
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

    Frame Convention:
        - HAND_EYE_MATRIX = tcp_T_camera (TCP frame with respect to camera frame)
        - tcp_matrix = base_T_tcp (Base frame with respect to TCP frame)
        - Chain: base_pos = base_T_tcp @ tcp_T_camera @ camera_pos

    Args:
        camera_position: [x, y, z] position in camera frame (meters)
        tcp_matrix: 4x4 homogeneous transformation base_T_tcp (base to TCP)

    Returns:
        Position in robot base frame (meters)
    """
    try:
        # Convert camera position to numpy array
        camera_pos = np.array(camera_position, dtype=np.float64)

        # HAND_EYE_MATRIX is tcp_T_camera: transforms camera frame → TCP frame
        tcp_T_camera = config_module.HAND_EYE_MATRIX

        # Validate matrix shapes
        if tcp_T_camera.shape != (4, 4):
            logger.error(
                f"Invalid HAND_EYE_MATRIX shape: {tcp_T_camera.shape}, expected (4, 4)")
            return np.array([0.0, 0.0, 0.0])
        if tcp_matrix.shape != (4, 4):
            logger.error(
                f"Invalid tcp_matrix shape: {tcp_matrix.shape}, expected (4, 4)")
            return np.array([0.0, 0.0, 0.0])

        # Transform camera position to TCP frame
        camera_pos_homogeneous = np.concatenate([camera_pos, [1.0]])
        tcp_pos_homogeneous = tcp_T_camera @ camera_pos_homogeneous
        tcp_pos = tcp_pos_homogeneous[:3]

        logger.debug(
            f"Camera->TCP transformation ({config_module.CAMERA_TRANSFORM_MODE}): {camera_pos} -> {tcp_pos}")

        # tcp_matrix is base_T_tcp: transforms TCP frame → base frame
        tcp_pos_homogeneous = np.concatenate([tcp_pos, [1.0]])
        base_pos_homogeneous = tcp_matrix @ tcp_pos_homogeneous
        base_pos = base_pos_homogeneous[:3]

        logger.debug(f"TCP->Base transformation: {tcp_pos} -> {base_pos}")
        return base_pos

    except Exception as e:
        logger.error(f"Failed to transform camera to base: {e}")
        return np.array([0.0, 0.0, 0.0])


def transform_base_to_camera(base_position: List[float], tcp_matrix: np.ndarray) -> np.ndarray:
    """
    Transform position from robot base frame to camera frame using calibrated hand-eye matrix.

    Frame Convention:
        - Inverse of forward chain: camera_pos = camera_T_tcp @ tcp_T_base @ base_pos
        - camera_T_tcp = inv(tcp_T_camera) = inv(HAND_EYE_MATRIX)
        - tcp_T_base = inv(base_T_tcp) = inv(tcp_matrix)

    Args:
        base_position: [x, y, z] position in base frame (meters)
        tcp_matrix: 4x4 homogeneous transformation base_T_tcp (base to TCP)

    Returns:
        Position in camera frame (meters)
    """
    try:
        # Convert base position to numpy array
        base_pos = np.array(base_position, dtype=np.float64)

        # Transform base position to TCP frame: tcp_T_base = inv(base_T_tcp)
        base_pos_homogeneous = np.concatenate([base_pos, [1.0]])
        tcp_pos_homogeneous = np.linalg.inv(tcp_matrix) @ base_pos_homogeneous
        tcp_pos = tcp_pos_homogeneous[:3]

        # Transform TCP to camera: camera_T_tcp = inv(tcp_T_camera) = inv(HAND_EYE_MATRIX)
        camera_T_tcp = np.linalg.inv(config_module.HAND_EYE_MATRIX)

        # Transform TCP position to camera frame
        tcp_pos_homogeneous = np.concatenate([tcp_pos, [1.0]])
        camera_pos_homogeneous = camera_T_tcp @ tcp_pos_homogeneous
        camera_pos = camera_pos_homogeneous[:3]

        logger.debug(
            f"Base position {base_pos} -> Camera position {camera_pos}")
        return camera_pos

    except Exception as e:
        logger.error(f"Failed to transform base to camera: {e}")
        return np.array([0.0, 0.0, 0.0])


def transform_camera_to_tcp_frame(camera_position: List[float]) -> np.ndarray:
    """
    Transform position from camera frame to TCP frame using calibrated hand-eye matrix.

    This is a simplified version of transform_camera_to_base that only goes to TCP,
    not all the way to base frame.

    Frame Convention:
        - HAND_EYE_MATRIX = tcp_T_camera
        - tcp_pos = tcp_T_camera @ camera_pos

    Args:
        camera_position: [x, y, z] position in camera frame (meters)

    Returns:
        Position in TCP frame (meters)
    """
    try:
        # Convert camera position to numpy array
        camera_pos = np.array(camera_position, dtype=np.float64)

        # HAND_EYE_MATRIX is tcp_T_camera: transforms camera frame → TCP frame
        tcp_T_camera = config_module.HAND_EYE_MATRIX

        # Transform camera position to TCP frame
        camera_pos_homogeneous = np.concatenate([camera_pos, [1.0]])
        tcp_pos_homogeneous = tcp_T_camera @ camera_pos_homogeneous
        tcp_pos = tcp_pos_homogeneous[:3]

        logger.debug(f"Camera->TCP transformation: {camera_pos} -> {tcp_pos}")
        return tcp_pos

    except Exception as e:
        logger.error(f"Failed to transform camera to TCP: {e}")
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


def camera_frame_to_pixel(x: float, y: float, z: float, intrinsics) -> Tuple[int, int]:
    """
    Convert camera frame coordinates to pixel coordinates.

    Uses pinhole camera projection: u = fx * x/z + cx, v = fy * y/z + cy

    Args:
        x, y, z: Coordinates in camera frame (meters)
        intrinsics: Camera intrinsics

    Returns:
        (u, v) pixel coordinates, or (-1, -1) if point is behind camera or at camera origin

    Note: Returns (-1, -1) sentinel value for invalid projections (z <= epsilon)
    """
    try:
        # Guard against division by zero or points behind/at the camera
        # Use small epsilon to avoid numerical instability near z=0
        Z_EPSILON = 1e-6

        if z <= Z_EPSILON:
            if z <= 0:
                logger.warning(
                    f"Cannot project point behind or at camera plane: "
                    f"point=({x:.3f}, {y:.3f}, {z:.3f}). "
                    f"Z must be > 0 for valid projection. Returning sentinel (-1, -1)."
                )
            else:
                logger.warning(
                    f"Point too close to camera origin for stable projection: "
                    f"point=({x:.3f}, {y:.3f}, {z:.6f}). "
                    f"Z={z:.6f} < epsilon={Z_EPSILON}. Returning sentinel (-1, -1)."
                )
            return (-1, -1)

        # Convert camera coordinates to pixel using pinhole camera model
        # Use round() instead of int() truncation to reduce off-by-one errors at subpixel boundaries
        u = int(round(x * intrinsics.fx / z + intrinsics.ppx))
        v = int(round(y * intrinsics.fy / z + intrinsics.ppy))

        return (u, v)

    except Exception as e:
        logger.error(
            f"Failed to convert camera frame to pixel: {e}. "
            f"Input: point=({x:.3f}, {y:.3f}, {z:.3f}). "
            f"Returning sentinel (-1, -1)."
        )
        return (-1, -1)
