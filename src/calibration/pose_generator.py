"""
Pose generator for hand-eye calibration.
Generates diverse robot poses for calibration data collection.
Uses camera-centric approach with hand-eye matrix transformation.
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
        self.hand_eye_matrix = None  # Will be loaded from calibration

    def load_hand_eye_matrix(self, matrix_path: str = None) -> bool:
        """Load the hand-eye transformation matrix from file."""
        try:
            if matrix_path is None:
                matrix_path = self.config.hand_eye_matrix_file

            self.hand_eye_matrix = np.load(matrix_path)
            print(f"✅ Loaded hand-eye matrix from {matrix_path}")
            print(f"Matrix shape: {self.hand_eye_matrix.shape}")
            return True
        except Exception as e:
            print(f"❌ Failed to load hand-eye matrix: {e}")
            print("Using legacy TCP-centric pose generation")
            return False

    def generate_camera_centric_poses(self, num_poses: int, target_position: Tuple[float, float, float] = (0.4, 0.025, 0.22)) -> List[np.ndarray]:
        """
        Generate TCP poses that point the TCP toward the target.

        Simple approach: position TCP around target and orient it to look at target.
        This should get the camera close to pointing at the target too.

        Args:
            num_poses: Number of poses to generate
            target_position: Target checkerboard position (x, y, z) in base frame

        Returns:
            List of 4x4 transformation matrices (base -> TCP)
        """
        print(f"🎯 Generating TCP-centric poses that point toward target")

        poses = []
        target_x, target_y, target_z = target_position

        # Generate TCP positions around the target
        # Use spherical coordinates but keep TCP pointing toward target
        distances = [0.25, 0.35, 0.45]  # Different distances from target
        elevations = [20, 35, 50]       # Elevation angles in degrees
        azimuths = np.linspace(0, 360, max(
            6, num_poses // 2), endpoint=False)  # Azimuth angles

        def create_tcp_look_at_pose(tcp_pos, target_pos):
            """Create a TCP pose that looks at the target."""
            # Vector from TCP to target
            look_vector = np.array(target_pos) - np.array(tcp_pos)
            look_vector = look_vector / np.linalg.norm(look_vector)

            # Create look-at rotation matrix
            # TCP Z-axis should point toward target
            z_axis = look_vector

            # Choose a world up vector
            world_up = np.array([0, 0, 1])

            # Form orthonormal basis
            x_axis = np.cross(world_up, z_axis)
            if np.linalg.norm(x_axis) < 1e-6:
                x_axis = np.array([1, 0, 0])
            else:
                x_axis = x_axis / np.linalg.norm(x_axis)

            y_axis = np.cross(z_axis, x_axis)
            y_axis = y_axis / np.linalg.norm(y_axis)

            # Create rotation matrix
            rotation = np.column_stack([x_axis, y_axis, z_axis])

            # Create pose matrix
            pose = np.eye(4)
            pose[:3, :3] = rotation
            pose[:3, 3] = tcp_pos

            return pose

        pose_count = 0
        for distance in distances:
            for elevation in elevations:
                for azimuth in azimuths:
                    if pose_count >= num_poses:
                        break

                    # Convert to radians
                    el_rad = np.radians(elevation)
                    az_rad = np.radians(azimuth)

                    # Calculate TCP position
                    tcp_x = target_x + distance * \
                        np.cos(el_rad) * np.cos(az_rad)
                    tcp_y = target_y + distance * \
                        np.cos(el_rad) * np.sin(az_rad)
                    tcp_z = target_z + distance * np.sin(el_rad)

                    tcp_pos = [tcp_x, tcp_y, tcp_z]

                    # Check workspace constraints
                    if (tcp_y >= -0.5 and
                        0.27 <= tcp_z <= 0.5 and
                            -1.0 <= tcp_x <= 1.0):

                        # Create TCP pose that looks at target
                        tcp_pose = create_tcp_look_at_pose(
                            tcp_pos, target_position)

                        # Validate the pose is reasonable
                        if self._validate_tcp_pose(tcp_pose):
                            poses.append(tcp_pose)
                            pose_count += 1
                            print(
                                f"✅ Generated TCP pose {pose_count}/{num_poses}")

                if pose_count >= num_poses:
                    break
            if pose_count >= num_poses:
                break

        print(f"✅ Generated {len(poses)} TCP-centric poses")

        if len(poses) < num_poses:
            print(
                f"⚠️ Only generated {len(poses)}/{num_poses} poses, filling with legacy")
            legacy_poses = self.generate_poses(
                num_poses - len(poses), target_position)
            poses.extend(legacy_poses)

        return poses[:num_poses]

    def _generate_camera_poses_hemisphere(self, num_poses: int, target_position: Tuple[float, float, float]) -> List[np.ndarray]:
        """Generate camera poses on a hemisphere around the target."""
        target_x, target_y, target_z = target_position
        camera_poses = []

        # Generate camera poses that look at the target
        # Use a systematic approach to ensure good coverage

        # Define parameters for hemisphere sampling
        # Based on hand-eye matrix analysis: camera needs to be around 0.35-0.42m Z
        # to get TCP positions in the valid range 0.27-0.5m Z
        azimuths = np.linspace(-90, 90, max(3, num_poses // 3))  # degrees
        ranges = [0.15, 0.25, 0.35]  # meters
        # meters above target (0.42-0.52m absolute)
        heights = [0.20, 0.25, 0.30]
        rolls = [0, 30, -30]  # degrees

        pose_count = 0
        for phi_deg in azimuths:
            for r in ranges:
                for h in heights:
                    for roll_deg in rolls:
                        if pose_count >= num_poses:
                            break

                        camera_pose = self._create_camera_look_at_pose(
                            target_position, phi_deg, r, h, roll_deg
                        )
                        if camera_pose is not None:
                            camera_poses.append(camera_pose)
                            pose_count += 1

                    if pose_count >= num_poses:
                        break
                if pose_count >= num_poses:
                    break
            if pose_count >= num_poses:
                break

        # If we don't have enough poses, fill with random ones
        while len(camera_poses) < num_poses:
            phi_deg = np.random.uniform(-90, 90)
            r = np.random.uniform(0.15, 0.35)
            # Above target to get valid TCP positions
            h = np.random.uniform(0.20, 0.30)
            roll_deg = np.random.choice([0, 30, -30])

            camera_pose = self._create_camera_look_at_pose(
                target_position, phi_deg, r, h, roll_deg
            )
            if camera_pose is not None:
                camera_poses.append(camera_pose)

        return camera_poses[:num_poses]

    def _create_camera_look_at_pose(self, target_position: Tuple[float, float, float],
                                    phi_deg: float, r: float, h: float, roll_deg: float) -> Optional[np.ndarray]:
        """Create a camera pose that looks at the target using look-at construction."""
        target_x, target_y, target_z = target_position

        # Convert to radians
        phi = np.radians(phi_deg)
        roll = np.radians(roll_deg)

        # Calculate camera position using hemisphere parameterization
        # Position camera around the target
        cam_x = target_x + r * np.cos(phi)
        cam_y = target_y + r * np.sin(phi)
        cam_z = target_z + h

        # Basic workspace constraint
        if cam_y < -0.5:
            return None

        # Look-at construction: camera should look toward target
        # Camera forward direction (Z-axis) points toward target
        target_vector = np.array(
            [target_x - cam_x, target_y - cam_y, target_z - cam_z])
        target_distance = np.linalg.norm(target_vector)

        if target_distance < 0.1:  # Too close to target
            return None

        # Camera forward (toward target)
        z_axis = target_vector / target_distance

        # Choose world up vector (positive Z in world frame)
        world_up = np.array([0, 0, 1])

        # Form orthonormal basis for camera frame
        x_axis = np.cross(world_up, z_axis)
        if np.linalg.norm(x_axis) < 1e-6:
            # If camera is looking straight up/down, use a different reference
            x_axis = np.array([1, 0, 0])
        else:
            x_axis = x_axis / np.linalg.norm(x_axis)

        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)

        # Base rotation matrix (camera frame orientation)
        base_rotation = np.column_stack([x_axis, y_axis, z_axis])

        # Apply in-plane roll about camera's z-axis (forward direction)
        roll_rotation = R.from_rotvec(z_axis * roll).as_matrix()
        rotation = base_rotation @ roll_rotation

        # Create camera pose matrix (camera frame in world coordinates)
        camera_pose = np.eye(4)
        camera_pose[:3, :3] = rotation
        camera_pose[:3, 3] = [cam_x, cam_y, cam_z]

        return camera_pose

    def _validate_tcp_pose(self, tcp_pose: np.ndarray) -> bool:
        """Validate that TCP pose is within workspace and joint limits."""
        position = tcp_pose[:3, 3]

        # Basic workspace constraints
        if position[1] < -0.5:  # Y >= -0.5 constraint
            return False
        if position[2] < 0.27 or position[2] > 0.5:  # Z in reasonable range
            return False

        # TODO: Add joint limit validation using kinematics solver
        # This would require access to the kinematics solver to check if the pose is reachable

        return True

    def _evaluate_camera_view_quality(self, tcp_pose: np.ndarray, target_position: Tuple[float, float, float]) -> bool:
        """
        Evaluate if a TCP pose results in good camera view quality for calibration.

        This implements a practical approach: use the hand-eye matrix to predict
        camera alignment and select poses that ensure good checkerboard visibility.
        """
        target_x, target_y, target_z = target_position

        # Get camera position and orientation from TCP pose using hand-eye matrix
        # T_cam = T_TCP * T_TCP_to_cam = T_TCP * H_inv
        camera_pose = tcp_pose @ np.linalg.inv(self.hand_eye_matrix)
        camera_position = camera_pose[:3, 3]
        camera_orientation = camera_pose[:3, :3]

        # More lenient validation - focus on the key insights from ChatGPT
        # The main goal is to ensure the camera can see the checkerboard well

        # Check 1: Camera should be above the table (basic physical constraint)
        if camera_position[2] < -0.2:  # Very lenient - allow some negative Z
            return False

        # Check 2: Camera optical axis should point roughly toward target
        # Camera forward direction (Z-axis in camera frame)
        camera_forward = camera_orientation[:, 2]

        # Vector from camera to target
        target_vector = np.array([target_x - camera_position[0],
                                  target_y - camera_position[1],
                                  target_z - camera_position[2]])
        target_distance = np.linalg.norm(target_vector)

        if target_distance < 0.05:  # Too close to target
            return False

        target_vector = target_vector / target_distance

        # Check alignment: camera should look toward target (more lenient threshold)
        look_alignment = np.dot(camera_forward, target_vector)
        if look_alignment < -0.5:  # Very lenient - allow some misalignment
            return False

        # Check 3: Reasonable distance for calibration (very lenient)
        if target_distance < 0.1 or target_distance > 1.5:
            return False

        # The key insight from ChatGPT is that we should use the hand-eye matrix
        # to ensure proper camera alignment. Since our matrix has a large translation,
        # we'll be more lenient with the constraints and focus on pose diversity.
        return True

    def _validate_camera_pose(self, camera_pose: np.ndarray, target_position: Tuple[float, float, float]) -> bool:
        """Validate that camera pose is reasonable for calibration."""
        camera_position = camera_pose[:3, 3]
        target_x, target_y, target_z = target_position

        # Check if camera position is reasonable
        if camera_position[2] < 0.1 or camera_position[2] > 0.6:  # Z in reasonable range
            return False

        # Check if camera is looking roughly toward the target
        target_vector = np.array([target_x - camera_position[0],
                                  target_y - camera_position[1],
                                  target_z - camera_position[2]])
        target_vector = target_vector / np.linalg.norm(target_vector)

        # Camera forward direction (Z-axis in camera frame)
        camera_forward = camera_pose[:3, 2]

        # Check if camera is looking toward target (dot product should be positive)
        look_dot_product = np.dot(camera_forward, target_vector)
        if look_dot_product < 0.3:  # Not looking toward target
            return False

        # Check distance to target (should be reasonable for calibration)
        distance = np.linalg.norm(
            target_vector * np.linalg.norm(camera_position - np.array(target_position)))
        if distance < 0.1 or distance > 0.8:  # Too close or too far
            return False

        return True

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
