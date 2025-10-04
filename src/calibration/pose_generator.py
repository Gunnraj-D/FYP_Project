"""
Pose generator for hand-eye calibration.
Generates diverse robot poses for calibration data collection.
Uses improved spatial constraints based on table height and workspace limits.
"""
import numpy as np
from typing import List, Tuple, Optional
from scipy.spatial.transform import Rotation as R
from .calibration_config import CalibrationConfig


class PoseGenerator:
    """Generates diverse robot poses for hand-eye calibration."""

    def __init__(self, config: CalibrationConfig):
        self.config = config
        self.workspace_center = config.get_workspace_center()

    def generate_poses(self, num_poses: int, target_position: Tuple[float, float, float] = (0.4, 0.025, 0.22), robot_base_z: float = 0.202) -> List[np.ndarray]:
        """
        Generate diverse robot poses for calibration targeting a specific checkerboard location.
        Uses improved spatial constraints to ensure poses are above table and within workspace.

        Args:
            num_poses: Number of poses to generate (15 recommended)
            target_position: Target position (x, y, z) in robot base frame
            robot_base_z: Robot base height (z coordinate)

        Returns:
            List of 4x4 transformation matrices (base -> TCP)
        """
        poses = []
        target_x, target_y, target_z = target_position

        # Table height and constraints
        table_height = 0.22  # Table height

        # Use the optimized 15-pose set if requesting 15 poses
        if num_poses == 15:
            pose_params = [
                {"phi": -100, "r": 0.17, "theta": 20.7, "roll":   0},
                {"phi": -100, "r": 0.23, "theta": 25.8, "roll":  45},
                {"phi": -100, "r": 0.29, "theta": 28.9, "roll": -45},
                {"phi": -50, "r": 0.17, "theta": 36.0, "roll":   0},
                {"phi": -50, "r": 0.23, "theta": 37.5, "roll":  45},
                {"phi": -50, "r": 0.29, "theta": 20.2, "roll": -45},
                {"phi":    0, "r": 0.17, "theta": 55.0, "roll":   0},
                {"phi":    0, "r": 0.23, "theta": 15.1, "roll":  45},
                {"phi":    0, "r": 0.29, "theta": 28.9, "roll": -45},
                {"phi":   50, "r": 0.17, "theta": 20.7, "roll":   0},
                {"phi":   50, "r": 0.23, "theta": 25.8, "roll":  45},
                {"phi":   50, "r": 0.29, "theta": 20.2, "roll": -45},
                {"phi":  100, "r": 0.17, "theta": 36.0, "roll":   0},
                {"phi":  100, "r": 0.23, "theta": 37.5, "roll":  45},
                {"phi":  100, "r": 0.29, "theta": 11.9, "roll": -45}
            ]

            for params in pose_params:
                pose = self._generate_pose_from_params(target_position, params)
                if pose is not None:
                    poses.append(pose)
        else:
            # Fallback to random generation for other numbers
            poses = self._generate_random_poses(
                num_poses, target_position, robot_base_z)

        return poses

    def _generate_pose_from_params(self, target_position: Tuple[float, float, float], params: dict) -> Optional[np.ndarray]:
        """Generate a single pose from spherical parameters."""
        target_x, target_y, target_z = target_position

        # Extract parameters
        phi_deg = params["phi"]
        r = params["r"]
        theta_deg = params["theta"]
        roll_deg = params["roll"]

        # Convert to radians
        phi = np.radians(phi_deg)
        theta = np.radians(theta_deg)
        roll = np.radians(roll_deg)

        # Calculate TCP position using spherical coordinates
        # p = T + r[cos(theta)cos(phi), cos(theta)sin(phi), sin(theta)]
        x = target_x + r * np.cos(theta) * np.cos(phi)
        y = target_y + r * np.cos(theta) * np.sin(phi)
        z = target_z + r * np.sin(theta)

        # Verify constraints
        if y < -0.5:  # Y >= -0.5 constraint
            return None
        if z < 0.27 or z > 0.37:  # Z in [0.27, 0.37] (5-15cm above table)
            return None

        # Generate look-at orientation
        target_vector = np.array([target_x - x, target_y - y, target_z - z])
        target_vector = target_vector / np.linalg.norm(target_vector)

        # Look-at construction: approach vector points toward target
        approach = target_vector  # Z-axis points toward target

        # Choose world up vector
        world_up = np.array([0, 0, 1])

        # Form orthonormal basis
        x_axis = np.cross(world_up, approach)
        # Handle case where approach is parallel to world_up
        if np.linalg.norm(x_axis) < 1e-6:
            x_axis = np.array([1, 0, 0])
        else:
            x_axis = x_axis / np.linalg.norm(x_axis)

        y_axis = np.cross(approach, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)

        # Base rotation matrix (approach = z_axis)
        base_rotation = np.column_stack([x_axis, y_axis, approach])

        # Apply in-plane roll about approach vector
        roll_rotation = R.from_rotvec(approach * roll).as_matrix()
        rotation = base_rotation @ roll_rotation

        # Create transformation matrix
        pose = np.eye(4)
        pose[:3, :3] = rotation
        pose[:3, 3] = [x, y, z]

        return pose

    def _generate_random_poses(self, num_poses: int, target_position: Tuple[float, float, float], robot_base_z: float) -> List[np.ndarray]:
        """Fallback random pose generation for non-15 pose requests."""
        poses = []
        target_x, target_y, target_z = target_position
        table_height = 0.22

        for i in range(num_poses):
            # Generate spherical coordinates around target
            radius = np.random.uniform(0.15, 0.30)

            # Azimuth angle: -110° to +110°
            azimuth = np.radians(np.random.uniform(-110, 110))

            # Elevation angle: ensure 5-15cm above table
            min_elevation = np.arcsin(0.05 / radius)  # 5cm above table
            max_elevation = np.arcsin(0.15 / radius)  # 15cm above table
            elevation = np.random.uniform(min_elevation, max_elevation)

            # Calculate position
            x = target_x + radius * np.cos(elevation) * np.cos(azimuth)
            y = target_y + radius * np.cos(elevation) * np.sin(azimuth)
            z = target_z + radius * np.sin(elevation)

            # Apply constraints
            y = max(y, -0.5)  # Y >= -0.5 constraint
            z = max(z, table_height + 0.05)  # At least 5cm above table

            # Generate look-at orientation
            target_vector = np.array(
                [target_x - x, target_y - y, target_z - z])
            target_vector = target_vector / np.linalg.norm(target_vector)

            # Create rotation matrix looking at target
            z_axis = target_vector
            x_axis = np.cross(z_axis, [0, 0, 1])
            if np.linalg.norm(x_axis) < 1e-6:
                x_axis = np.array([1, 0, 0])
            else:
                x_axis = x_axis / np.linalg.norm(x_axis)
            y_axis = np.cross(z_axis, x_axis)

            rotation = np.column_stack([x_axis, y_axis, z_axis])

            # Add small random roll variation
            roll = np.random.uniform(-np.pi/4, np.pi/4)
            roll_rotation = R.from_rotvec(z_axis * roll).as_matrix()
            rotation = rotation @ roll_rotation

            # Create transformation matrix
            pose = np.eye(4)
            pose[:3, :3] = rotation
            pose[:3, 3] = [x, y, z]

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
