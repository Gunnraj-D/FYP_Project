"""
Camera configuration and hand-eye calibration.
"""
import numpy as np

# ============================================================================
# CAMERA POSITION & ORIENTATION
# ============================================================================

# Camera position relative to TCP (meters)
# For handheld camera setup - camera is essentially at TCP position
CAMERA_TRANSLATION = np.array([
    0.0,   # No forward offset
    0.0,   # No left offset
    0.0    # No vertical offset (camera held at TCP level)
])

# Camera orientation relative to TCP (degrees)
# Camera is looking down at the table, rotated 180° around X axis
CAMERA_ROTATION_EULER = {
    'roll': 180,    # Rotation around X axis (camera looking down)
    'pitch': 0,     # Rotation around Y axis
    'yaw': 0        # Rotation around Z axis
}

# ============================================================================
# HAND-EYE CALIBRATION
# ============================================================================

# Camera transform mode: 'calibrated' or 'simple'
# - 'calibrated': Use full calibrated hand-eye matrix with rotation and translation
# - 'simple': Camera on TCP with 180° X rotation (pointing down), no translation offset
CAMERA_TRANSFORM_MODE = 'calibrated'

# Hand-eye transformation matrix: tcp_T_camera (Camera frame → TCP frame)
# CONVENTION: HAND_EYE_MATRIX = tcp_T_camera
#   Forward:  tcp_pos = HAND_EYE_MATRIX @ camera_pos_homogeneous
#   Inverse:  camera_pos = inv(HAND_EYE_MATRIX) @ tcp_pos_homogeneous
#
# Generated from best 15 of 18 calibration poses using Park method
# Translation: 0.0669m (6.69cm camera-to-TCP)
# Rotation: 178.5° yaw, -0.7° roll, -0.5° pitch
# Reprojection errors: Mean 3.53, Max 3.82 (excellent consistency)
# Calibration date: 2025-10-12
# Sessions: 20251012_233752 (7 poses) + 20251012_234617 (11 poses)
HAND_EYE_MATRIX_CALIBRATED = np.array([
    [-0.9996, -0.0268,  0.0084,  0.0136],
    [0.0267, -0.9996, -0.0123,  0.0639],
    [0.0087, -0.0121,  0.9999,  0.0141],
    [0.0000,  0.0000,  0.0000,  1.0000]
], dtype=np.float64)

# Simplified hand-eye matrix: tcp_T_camera (camera mounted on TCP, pointing down)
HAND_EYE_MATRIX_SIMPLE = np.array([
    [1.0,   0.0,   0.0,  0.0],    # X-axis unchanged (right)
    [0.0,  -1.0,   0.0,  0.0],    # Y-axis flipped (camera Y+ = TCP Y-)
    [0.0,   0.0,   1.0,  0.0],    # Z-axis unchanged (camera depth = TCP down)
    [0.0,   0.0,   0.0,  1.0]
], dtype=np.float64)

# Select the active hand-eye matrix based on mode
HAND_EYE_MATRIX = HAND_EYE_MATRIX_CALIBRATED if CAMERA_TRANSFORM_MODE == 'calibrated' else HAND_EYE_MATRIX_SIMPLE


def set_camera_transform_mode(mode: str):
    """
    Change the camera transform mode at runtime.

    Args:
        mode: 'calibrated' or 'simple'
    """
    global CAMERA_TRANSFORM_MODE, HAND_EYE_MATRIX
    if mode not in ['calibrated', 'simple']:
        raise ValueError(
            f"Invalid mode: {mode}. Must be 'calibrated' or 'simple'")
    CAMERA_TRANSFORM_MODE = mode
    HAND_EYE_MATRIX = HAND_EYE_MATRIX_CALIBRATED if mode == 'calibrated' else HAND_EYE_MATRIX_SIMPLE
    print(f"✅ Camera transform mode set to: {mode}")


def get_camera_transform_info() -> dict:
    """Get information about the current camera transform configuration."""
    return {
        'mode': CAMERA_TRANSFORM_MODE,
        'matrix': HAND_EYE_MATRIX,
        'translation': HAND_EYE_MATRIX[:3, 3],
        'has_rotation': not np.allclose(HAND_EYE_MATRIX[:3, :3], np.eye(3))
    }


def print_camera_transform_info():
    """Print current camera transform configuration."""
    info = get_camera_transform_info()
    print("\n" + "="*50)
    print("📷 CAMERA TRANSFORM CONFIGURATION")
    print("="*50)
    print(f"Mode: {info['mode'].upper()}")
    print(
        f"Has Rotation: {'Yes' if info['has_rotation'] else 'No (Identity)'}")
    print(f"Translation [x, y, z]: {info['translation']}")
    print(f"Translation Magnitude: {np.linalg.norm(info['translation']):.4f}m")
    print("="*50 + "\n")
