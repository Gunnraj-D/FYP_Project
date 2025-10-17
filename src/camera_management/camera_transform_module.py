"""
Camera Transform Module - Coordinate frame transformations.

Handles transformations between camera frame and robot base frame for eye-in-hand setup.

COORDINATE FRAME CONVENTIONS:
    Base Frame (Robot):
        X: forward, Y: left, Z: up (right-handed)
    
    Camera Frame (Standard Computer Vision):
        X: right, Y: down, Z: away from camera (into scene)
    
    TCP Frame (Tool Center Point):
        Typically aligned with gripper/end-effector

TRANSFORMATION CHAIN:
    For eye-in-hand (camera mounted on robot):
    base_position = base_T_tcp @ tcp_T_camera @ camera_position
    
    Where:
    - tcp_T_camera (HAND_EYE_MATRIX): camera pose relative to TCP
    - base_T_tcp (tcp_matrix): TCP pose relative to base (from robot state)

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

import config as config_module

logger = logging.getLogger(__name__)

# ============================================================================
# CAMERA TO BASE FRAME TRANSFORMATIONS
# ============================================================================

def transform_camera_to_base(camera_position: List[float], tcp_matrix: np.ndarray) -> np.ndarray:
    """
    Transform position from camera frame to robot base frame using calibrated hand-eye matrix.

    Frame Convention (Eye-in-Hand):
        - HAND_EYE_MATRIX = tcp_T_camera (camera pose relative to TCP frame)
        - tcp_matrix = base_T_tcp (TCP pose relative to base frame)
        - Chain: base_pos = base_T_tcp @ tcp_T_camera @ camera_pos

    Args:
        camera_position: [x, y, z] position in camera frame (meters)
        tcp_matrix: 4x4 homogeneous transformation base_T_tcp (base to TCP)

    Returns:
        Position in robot base frame (meters)
        
    Raises:
        ValueError: If matrix shapes are invalid or transformation fails
    """
    try:
        # Convert camera position to numpy array
        camera_pos = np.array(camera_position, dtype=np.float64)
        
        if camera_pos.shape != (3,):
            raise ValueError(f"Invalid camera_position shape: {camera_pos.shape}, expected (3,)")

        # Get hand-eye transformation matrix
        tcp_T_camera = _get_tcp_to_camera_transform()
        
        # Validate matrix shapes
        if tcp_T_camera.shape != (4, 4):
            raise ValueError(
                f"Invalid HAND_EYE_MATRIX shape: {tcp_T_camera.shape}, expected (4, 4)"
            )
        if tcp_matrix.shape != (4, 4):
            raise ValueError(
                f"Invalid tcp_matrix shape: {tcp_matrix.shape}, expected (4, 4)"
            )

        # Transform camera position to TCP frame
        camera_pos_homogeneous = np.concatenate([camera_pos, [1.0]])
        tcp_pos_homogeneous = tcp_T_camera @ camera_pos_homogeneous
        tcp_pos = tcp_pos_homogeneous[:3]

        logger.debug(
            f"Camera->TCP transformation: "
            f"cam=[{camera_pos[0]:.3f}, {camera_pos[1]:.3f}, {camera_pos[2]:.3f}] -> "
            f"tcp=[{tcp_pos[0]:.3f}, {tcp_pos[1]:.3f}, {tcp_pos[2]:.3f}]"
        )

        # Transform TCP position to base frame
        # tcp_matrix is base_T_tcp: transforms TCP frame → base frame
        tcp_pos_homogeneous = np.concatenate([tcp_pos, [1.0]])
        base_pos_homogeneous = tcp_matrix @ tcp_pos_homogeneous
        base_pos = base_pos_homogeneous[:3]

        logger.debug(
            f"TCP->Base transformation: "
            f"tcp=[{tcp_pos[0]:.3f}, {tcp_pos[1]:.3f}, {tcp_pos[2]:.3f}] -> "
            f"base=[{base_pos[0]:.3f}, {base_pos[1]:.3f}, {base_pos[2]:.3f}]"
        )
        
        return base_pos

    except Exception as e:
        logger.error(f"Failed to transform camera to base: {e}")
        raise ValueError(f"Camera to base frame transformation failed: {e}")


def transform_base_to_camera(base_position: List[float], tcp_matrix: np.ndarray) -> np.ndarray:
    """
    Transform position from robot base frame to camera frame using calibrated hand-eye matrix.

    Frame Convention (Eye-in-Hand):
        - Inverse of forward chain: camera_pos = camera_T_tcp @ tcp_T_base @ base_pos
        - camera_T_tcp = inv(tcp_T_camera) = inv(HAND_EYE_MATRIX)
        - tcp_T_base = inv(base_T_tcp) = inv(tcp_matrix)

    Args:
        base_position: [x, y, z] position in base frame (meters)
        tcp_matrix: 4x4 homogeneous transformation base_T_tcp (base to TCP)

    Returns:
        Position in camera frame (meters)
        
    Raises:
        ValueError: If matrix shapes are invalid or transformation fails
    """
    try:
        # Convert base position to numpy array
        base_pos = np.array(base_position, dtype=np.float64)
        
        if base_pos.shape != (3,):
            raise ValueError(f"Invalid base_position shape: {base_pos.shape}, expected (3,)")

        # Validate tcp_matrix shape
        if tcp_matrix.shape != (4, 4):
            raise ValueError(
                f"Invalid tcp_matrix shape: {tcp_matrix.shape}, expected (4, 4)"
            )

        # Transform base position to TCP frame: tcp_T_base = inv(base_T_tcp)
        base_pos_homogeneous = np.concatenate([base_pos, [1.0]])
        tcp_T_base = np.linalg.inv(tcp_matrix)
        tcp_pos_homogeneous = tcp_T_base @ base_pos_homogeneous
        tcp_pos = tcp_pos_homogeneous[:3]

        # Get inverse hand-eye transformation: camera_T_tcp = inv(tcp_T_camera)
        tcp_T_camera = _get_tcp_to_camera_transform()
        camera_T_tcp = np.linalg.inv(tcp_T_camera)

        # Transform TCP position to camera frame
        tcp_pos_homogeneous = np.concatenate([tcp_pos, [1.0]])
        camera_pos_homogeneous = camera_T_tcp @ tcp_pos_homogeneous
        camera_pos = camera_pos_homogeneous[:3]

        logger.debug(
            f"Base->Camera transformation: "
            f"base=[{base_pos[0]:.3f}, {base_pos[1]:.3f}, {base_pos[2]:.3f}] -> "
            f"camera=[{camera_pos[0]:.3f}, {camera_pos[1]:.3f}, {camera_pos[2]:.3f}]"
        )
        
        return camera_pos

    except Exception as e:
        logger.error(f"Failed to transform base to camera: {e}")
        raise ValueError(f"Base to camera frame transformation failed: {e}")


# ============================================================================
# HAND-EYE MATRIX HANDLING
# ============================================================================


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


def _get_tcp_to_camera_transform() -> np.ndarray:
    """
    Get tcp_T_camera transformation matrix from config.
    
    Uses HAND_EYE_MATRIX directly as tcp_T_camera (gripper_T_camera).
    This is the output from cv2.CALIB_HAND_EYE_PARK method.
    
    Returns:
        4x4 homogeneous transformation matrix tcp_T_camera
        
    Raises:
        ValueError: If HAND_EYE_MATRIX is not configured or invalid
    """
    try:
        # Check if HAND_EYE_MATRIX exists
        if not hasattr(config_module, 'HAND_EYE_MATRIX'):
            raise ValueError(
                "HAND_EYE_MATRIX not found in config. "
                "Please run hand-eye calibration first."
            )
        
        tcp_T_camera = config_module.HAND_EYE_MATRIX
        
        if tcp_T_camera.shape != (4, 4):
            raise ValueError(
                f"Invalid HAND_EYE_MATRIX shape: {tcp_T_camera.shape}, expected (4, 4)"
            )
        
        logger.debug("Using HAND_EYE_MATRIX as tcp_T_camera (Park method output)")
        
        return tcp_T_camera
        
    except Exception as e:
        logger.error(f"Failed to get tcp_T_camera transform: {e}")
        raise ValueError(f"Hand-eye matrix retrieval failed: {e}")


# ============================================================================
# CAMERA INTRINSICS AND PROJECTION
# ============================================================================

def get_camera_intrinsics_matrix(intrinsics) -> np.ndarray:
    """
    Get camera intrinsics matrix from RealSense intrinsics.

    Intrinsics Matrix K:
        [[fx,  0, cx],
         [ 0, fy, cy],
         [ 0,  0,  1]]
    
    Where:
        fx, fy: focal lengths in pixels
        cx, cy: principal point (optical center) in pixels

    Args:
        intrinsics: RealSense intrinsics object

    Returns:
        3x3 camera intrinsics matrix
        
    Raises:
        ValueError: If intrinsics are invalid
    """
    try:
        K = np.array([
            [intrinsics.fx, 0, intrinsics.ppx],
            [0, intrinsics.fy, intrinsics.ppy],
            [0, 0, 1]
        ])
        
        # Validate intrinsics are reasonable
        if intrinsics.fx <= 0 or intrinsics.fy <= 0:
            raise ValueError(
                f"Invalid focal lengths: fx={intrinsics.fx}, fy={intrinsics.fy}"
            )
        
        return K
        
    except Exception as e:
        logger.error(f"Failed to get camera intrinsics: {e}")
        raise ValueError(f"Camera intrinsics retrieval failed: {e}")


def camera_frame_to_pixel(x: float, y: float, z: float, intrinsics) -> Tuple[int, int]:
    """
    Convert camera frame coordinates to pixel coordinates using pinhole projection.

    Pinhole Camera Model:
        u = fx * (x/z) + cx
        v = fy * (y/z) + cy
    
    Where (x, y, z) are in camera frame with:
        - Z pointing away from camera (into scene)
        - X pointing right
        - Y pointing down

    Args:
        x, y, z: Coordinates in camera frame (meters)
        intrinsics: RealSense camera intrinsics object

    Returns:
        (u, v) pixel coordinates
        Returns (-1, -1) if point is behind camera or at camera origin
        
    Note: 
        Points with z <= 0 cannot be projected (behind or at camera).
        Returns sentinel value (-1, -1) for invalid projections.
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
        # Use round() instead of int() truncation to reduce off-by-one errors
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


# ============================================================================
# VALIDATION UTILITIES
# ============================================================================

def validate_hand_eye_calibration() -> bool:
    """
    Validate that hand-eye calibration is properly configured.
    
    Checks:
    - HAND_EYE_MATRIX exists and has correct shape
    - Matrix is a valid homogeneous transformation (bottom row = [0,0,0,1])
    - Rotation part is orthonormal (det(R) ≈ 1)
    
    Returns:
        True if calibration is valid, False otherwise
    """
    try:
        tcp_T_camera = _get_tcp_to_camera_transform()
        
        # Check bottom row is [0, 0, 0, 1]
        expected_bottom = np.array([0, 0, 0, 1])
        if not np.allclose(tcp_T_camera[3, :], expected_bottom, atol=1e-6):
            logger.error(
                f"Invalid homogeneous matrix bottom row: {tcp_T_camera[3, :]}, "
                f"expected [0, 0, 0, 1]"
            )
            return False
        
        # Check rotation part is orthonormal
        R = tcp_T_camera[:3, :3]
        det_R = np.linalg.det(R)
        
        if not np.isclose(det_R, 1.0, atol=1e-2):
            logger.error(
                f"Rotation matrix determinant is {det_R:.4f}, expected 1.0. "
                "Hand-eye calibration may be incorrect."
            )
            return False
        
        # Check R * R^T ≈ I (orthonormality)
        I = np.eye(3)
        R_RT = R @ R.T
        
        if not np.allclose(R_RT, I, atol=1e-2):
            logger.error(
                "Rotation matrix is not orthonormal (R*R^T != I). "
                "Hand-eye calibration may be incorrect."
            )
            return False
        
        logger.info("✓ Hand-eye calibration validation passed")
        return True
        
    except Exception as e:
        logger.error(f"Hand-eye calibration validation failed: {e}")
        return False