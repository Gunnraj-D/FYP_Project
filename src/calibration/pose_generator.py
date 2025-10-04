"""
Pose generator for hand-eye calibration.
Generates diverse robot poses for calibration data collection.
"""
import numpy as np
from typing import List, Tuple
from scipy.spatial.transform import Rotation as R
from .calibration_config import CalibrationConfig


class PoseGenerator:
    """Generates diverse robot poses for hand-eye calibration."""

    def __init__(self, config: CalibrationConfig):
        self.config = config
        self.workspace_center = config.get_workspace_center()

    def generate_poses(self, num_poses: int) -> List[np.ndarray]:
        """
        Generate diverse robot poses for calibration.

        Args:
            num_poses: Number of poses to generate

        Returns:
            List of 4x4 transformation matrices (base -> TCP)
        """
        poses = []

        # Generate poses in a grid pattern around workspace center
        grid_size = int(np.ceil(np.sqrt(num_poses)))

        for i in range(num_poses):
            # Calculate grid position
            row = i // grid_size
            col = i % grid_size

            # Normalize to [-1, 1] range
            x_norm = (col / (grid_size - 1)) * 2 - 1 if grid_size > 1 else 0
            y_norm = (row / (grid_size - 1)) * 2 - 1 if grid_size > 1 else 0

            # Map to workspace limits
            x_range = self.config.workspace_limits['x']
            y_range = self.config.workspace_limits['y']
            z_range = self.config.workspace_limits['z']

            x = x_range[0] + (x_norm + 1) / 2 * (x_range[1] - x_range[0])
            y = y_range[0] + (y_norm + 1) / 2 * (y_range[1] - y_range[0])
            z = z_range[0] + (z_range[1] - z_range[0]) * 0.5  # Middle height

            # Generate random orientation within limits
            roll_range = self.config.workspace_limits['roll']
            pitch_range = self.config.workspace_limits['pitch']
            yaw_range = self.config.workspace_limits['yaw']

            roll = np.random.uniform(roll_range[0], roll_range[1])
            pitch = np.random.uniform(pitch_range[0], pitch_range[1])
            yaw = np.random.uniform(yaw_range[0], yaw_range[1])

            # Create rotation matrix
            rotation = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()

            # Create transformation matrix
            pose = np.eye(4)
            pose[:3, :3] = rotation
            pose[:3, 3] = [x, y, z]

            # Check if pose is valid
            if self.config.is_pose_in_workspace(pose):
                poses.append(pose)
            else:
                # Fallback to workspace center with random orientation
                pose[:3, 3] = self.workspace_center
                poses.append(pose)

        return poses

    def generate_poses_circular(self, num_poses: int, radius: float = 0.15) -> List[np.ndarray]:
        """
        Generate poses in a circular pattern around workspace center.

        Args:
            num_poses: Number of poses to generate
            radius: Radius of the circle

        Returns:
            List of 4x4 transformation matrices (base -> TCP)
        """
        poses = []

        for i in range(num_poses):
            # Calculate angle
            angle = 2 * np.pi * i / num_poses

            # Calculate position
            x = self.workspace_center[0] + radius * np.cos(angle)
            y = self.workspace_center[1] + radius * np.sin(angle)
            z = self.workspace_center[2]

            # Generate random orientation
            roll_range = self.config.workspace_limits['roll']
            pitch_range = self.config.workspace_limits['pitch']
            yaw_range = self.config.workspace_limits['yaw']

            roll = np.random.uniform(roll_range[0], roll_range[1])
            pitch = np.random.uniform(pitch_range[0], pitch_range[1])
            yaw = np.random.uniform(yaw_range[0], yaw_range[1])

            # Create rotation matrix
            rotation = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()

            # Create transformation matrix
            pose = np.eye(4)
            pose[:3, :3] = rotation
            pose[:3, 3] = [x, y, z]

            poses.append(pose)

        return poses
