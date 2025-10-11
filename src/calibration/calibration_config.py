"""
Configuration for hand-eye calibration system.
"""
import cv2
from config import (
    ROBOT_ID, URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS,
    CAMERA_TRANSLATION, CAMERA_ROTATION_EULER
)
import numpy as np
from pathlib import Path
from typing import Dict, Any, List, Tuple
from dataclasses import dataclass, field

# Import from main config
import sys
sys.path.append(str(Path(__file__).parent.parent))


@dataclass
class CalibrationConfig:
    """Configuration parameters for hand-eye calibration."""

    # Checkerboard parameters
    # Internal corners (width, height)
    chessboard_size: Tuple[int, int] = (9, 6)
    square_size: float = 0.02  # Size of each square in meters (2 cm)

    # Calibration parameters
    num_poses: int = 15  # Number of poses to collect
    min_poses_required: int = 10  # Minimum poses needed for calibration
    max_detection_attempts: int = 10  # Max attempts per pose for corner detection

    # Camera parameters
    camera_resolution: Tuple[int, int] = (640, 480)
    camera_fps: int = 30

    # Robot workspace limits (in meters) - adjust for your setup
    workspace_limits: Dict[str, Tuple[float, float]] = field(default_factory=lambda: {
        'x': (0.2, 0.8),    # X range
        'y': (-0.4, 0.4),   # Y range
        'z': (0.3, 0.7),    # Z range
        'roll': (-np.pi, np.pi),    # Roll range
        'pitch': (-np.pi/4, np.pi/4),  # Pitch range
        'yaw': (-np.pi/2, np.pi/2)     # Yaw range
    })

    # Safety parameters
    max_joint_velocity: float = 0.5  # rad/s
    approach_velocity: float = 0.1   # rad/s for final approach
    settling_time: float = 2.0       # seconds to wait after movement

    # Calibration quality thresholds
    max_reprojection_error: float = 2.0  # pixels
    max_consistency_error: float = 0.01  # Frobenius norm
    min_corner_quality: float = 0.8      # Corner detection quality threshold

    # File paths
    calibration_data_file: str = "hand_eye_calibration_data.pkl"
    hand_eye_matrix_file: str = "hand_eye_matrix.npy"
    calibration_report_file: str = "calibration_report.txt"

    # OpenCV calibration parameters
    corner_criteria = (cv2.TERM_CRITERIA_EPS +
                       cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    calibration_method = cv2.CALIB_HAND_EYE_TSAI

    # Robot configuration (from main config)
    robot_id: int = ROBOT_ID
    urdf_filepath: str = str(URDF_FILEPATH)
    base_elements: List[str] = field(default_factory=lambda: BASE_ELEMENT)
    active_links: List[bool] = field(default_factory=lambda: ACTIVE_LINKS)

    # Camera configuration (from main config)
    camera_translation: np.ndarray = field(
        default_factory=lambda: CAMERA_TRANSLATION.copy())
    camera_rotation_euler: Dict[str, float] = field(
        default_factory=lambda: CAMERA_ROTATION_EULER)

    def __post_init__(self):
        """Validate configuration after initialization."""
        if self.num_poses < self.min_poses_required:
            raise ValueError(
                f"num_poses ({self.num_poses}) must be >= min_poses_required ({self.min_poses_required})")

        if self.chessboard_size[0] < 3 or self.chessboard_size[1] < 3:
            raise ValueError("Chessboard size must be at least 3x3")

        if self.square_size <= 0:
            raise ValueError("Square size must be positive")

    def get_workspace_center(self) -> np.ndarray:
        """Get the center of the robot workspace."""
        return np.array([
            (self.workspace_limits['x'][0] +
             self.workspace_limits['x'][1]) / 2,
            (self.workspace_limits['y'][0] +
             self.workspace_limits['y'][1]) / 2,
            (self.workspace_limits['z'][0] + self.workspace_limits['z'][1]) / 2
        ])

    def is_pose_in_workspace(self, pose: np.ndarray) -> bool:
        """Check if a pose is within workspace limits."""
        if pose.shape != (4, 4):
            return False

        position = pose[:3, 3]
        rotation = pose[:3, :3]

        # Check position limits
        for i, axis in enumerate(['x', 'y', 'z']):
            if not (self.workspace_limits[axis][0] <= position[i] <= self.workspace_limits[axis][1]):
                return False

        # Check orientation limits (convert rotation matrix to Euler angles)
        from scipy.spatial.transform import Rotation as R
        euler = R.from_matrix(rotation).as_euler('xyz')

        for i, axis in enumerate(['roll', 'pitch', 'yaw']):
            if not (self.workspace_limits[axis][0] <= euler[i] <= self.workspace_limits[axis][1]):
                return False

        return True


# Import cv2 for criteria
