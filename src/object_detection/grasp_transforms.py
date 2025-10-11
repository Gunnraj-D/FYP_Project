"""
Coordinate transformations for grasp poses.

Handles:
- 2D pixel coordinates → 3D camera frame
- Camera frame → Robot base frame  
- Grasp orientation composition (R_down @ R_z)
"""

import numpy as np
import logging
from typing import List, Optional
from scipy.spatial.transform import Rotation as R

from camera_management.camera_transform_module import transform_camera_to_base
from kinematics.kinematics_solver import get_facing_down_orientation
from config.config import GRASP_DETECTION_CONFIG, CAMERA_ROTATION_EULER

logger = logging.getLogger(__name__)


class GraspTransformer:
    """
    Transforms grasp poses between coordinate frames.

    Pipeline:
    1. 2D (u,v) + depth → 3D camera frame (x,y,z)
    2. Grasp angle → Orientation matrix (R_down @ R_z)
    3. Camera frame pose → Base frame pose
    """

    def __init__(self, camera_manager, kinematics_solver, telemetry):
        """
        Args:
            camera_manager: CameraManager for pixel_to_3d conversion
            kinematics_solver: KinematicsSolver for IK
            telemetry: Telemetry store for current TCP pose
        """
        self.camera_manager = camera_manager
        self.kinematics_solver = kinematics_solver
        self.telemetry = telemetry

    def grasp_2d_to_3d_pose(self, grasp_2d: dict, depth_image: np.ndarray,
                            original_depth_frame=None) -> Optional[List[float]]:
        """
        Convert 2D grasp parameters to 3D pose in camera frame.

        Args:
            grasp_2d: Dict with 'center' (u,v), 'angle' (rad), 'width' (pixels), 'quality'
            depth_image: Depth array in meters
            original_depth_frame: Optional RealSense frame for accurate depth sampling

        Returns:
            [x, y, z, roll, pitch, yaw] in camera frame (meters, radians)
            or None if conversion fails
        """
        try:
            center_u, center_v = grasp_2d["center"]
            angle = grasp_2d["angle"]

            # Apply angle offset (for jaw axis alignment)
            angle_offset_rad = GRASP_DETECTION_CONFIG.get(
                'grasp_angle_offset_rad', 0.0)
            angle += angle_offset_rad

            # Scale coordinates back to original image resolution
            h_orig = original_depth_frame.get_height(
            ) if original_depth_frame else depth_image.shape[0]
            w_orig = original_depth_frame.get_width(
            ) if original_depth_frame else depth_image.shape[1]

            scale_u = w_orig / 300.0
            scale_v = h_orig / 300.0

            center_u_scaled = center_u * scale_u
            center_v_scaled = center_v * scale_v

            # Get depth at grasp center
            depth_source = original_depth_frame if original_depth_frame else depth_image
            depth = self.camera_manager.get_average_depth(
                depth_source,
                (int(center_u_scaled), int(center_v_scaled)),
                radius=GRASP_DETECTION_CONFIG.get('depth_sample_radius', 5)
            )

            if depth <= 0:
                logger.warning(f"Invalid depth at grasp center: depth={depth}, "
                               f"center=({int(center_u_scaled)}, {int(center_v_scaled)})")
                return None

            # Convert pixel to 3D coordinates in camera frame
            x, y, z = self.camera_manager.pixel_to_3d(
                int(center_u_scaled), int(center_v_scaled), depth
            )

            # Create grasp orientation in camera frame
            # Simplified: top-down approach, yaw aligned with grasp angle
            roll = 0.0
            pitch = 0.0
            yaw = np.clip(angle, -np.pi/2, np.pi/2)  # Limit to ±90°

            pose = [x, y, z, roll, pitch, yaw]
            logger.debug(f"3D grasp pose (camera frame): {pose}")

            return pose

        except Exception as e:
            logger.error(f"Failed to convert 2D grasp to 3D pose: {e}")
            return None

    def transform_to_base_frame(self, camera_pose: List[float]) -> Optional[List[float]]:
        """
        Transform grasp pose from camera frame to robot base frame.

        Accounts for:
        - Camera mounting orientation relative to TCP
        - Current TCP pose in base frame
        - Proper rotation composition

        Args:
            camera_pose: [x,y,z, roll,pitch,yaw] in camera frame

        Returns:
            [x,y,z, roll,pitch,yaw] in base frame (meters, radians)
            or None if transformation fails
        """
        try:
            # Get current TCP pose
            current_joints = self.telemetry.get_current_joints()
            if len(current_joints) != 7:
                logger.warning("Invalid current joint positions")
                return None

            tcp_matrix, _ = self.kinematics_solver.tcp_from_joints(
                current_joints.tolist())

            # Transform position from camera to base frame
            camera_position = camera_pose[:3]
            base_position = transform_camera_to_base(
                camera_position, tcp_matrix)

            logger.info(
                f"Position: camera={camera_position} → base={base_position}")

            # Safety check: Z coordinate above workspace floor
            if base_position[2] < 0.0:
                logger.warning(f"Grasp position below workspace floor: Z={base_position[2]:.3f}m, "
                               f"clamping to 0.0m")
                base_position[2] = 0.0

            # Transform orientation
            base_orientation = self._transform_orientation(
                camera_pose[3:6], tcp_matrix)

            # Compose final base pose
            base_pose = base_position.tolist() + base_orientation.tolist()
            logger.debug(f"Transformed base frame grasp pose: {base_pose}")

            return base_pose

        except Exception as e:
            logger.error(f"Failed to transform to base frame: {e}")
            return None

    def _transform_orientation(self, grasp_rpy_camera: List[float],
                               tcp_matrix: np.ndarray) -> np.ndarray:
        """
        Transform grasp orientation from camera frame to base frame.

        Composition: Base → TCP → Camera → Grasp

        Args:
            grasp_rpy_camera: [roll, pitch, yaw] in camera frame
            tcp_matrix: 4x4 TCP transformation matrix in base frame

        Returns:
            [roll, pitch, yaw] in base frame
        """
        # Get camera mounting rotation (TCP → Camera)
        camera_mount_euler = np.radians([
            CAMERA_ROTATION_EULER['roll'],
            CAMERA_ROTATION_EULER['pitch'],
            CAMERA_ROTATION_EULER['yaw']
        ])
        tcp_to_camera_rot = R.from_euler('xyz', camera_mount_euler)

        # Get grasp orientation in camera frame
        grasp_in_camera_rot = R.from_euler('xyz', grasp_rpy_camera)

        # Get TCP orientation in base frame
        tcp_rot = R.from_matrix(tcp_matrix[:3, :3])

        # Compose: grasp_in_base = tcp_in_base * tcp_to_camera * grasp_in_camera
        grasp_in_base_rot = tcp_rot * tcp_to_camera_rot * grasp_in_camera_rot

        # Convert back to euler angles
        grasp_rpy_base = grasp_in_base_rot.as_euler('xyz')

        return grasp_rpy_base

    def compose_grasp_orientation(self, grasp_angle_rad: float) -> np.ndarray:
        """
        Compose grasp orientation from 2D angle.

        Creates R_target = R_down @ R_z:
        - R_z: Rotation around Z-axis to align with grasp angle
        - R_down: 180° rotation around X-axis (gripper pointing down)

        Args:
            grasp_angle_rad: Grasp angle in radians (already includes offset)

        Returns:
            3x3 rotation matrix
        """
        # Apply angle offset
        angle_offset_rad = GRASP_DETECTION_CONFIG.get(
            'grasp_angle_offset_rad', 0.0)
        grasp_angle_rad += angle_offset_rad

        # Create rotation matrices in BASE FRAME
        R_z = R.from_euler('z', grasp_angle_rad).as_matrix()
        R_down = get_facing_down_orientation()

        # Compose based on configuration
        compose_order = GRASP_DETECTION_CONFIG.get(
            'compose_order', 'down_then_z')

        if compose_order == 'down_then_z':
            # Recommended: align orientation in base frame, then point down
            target_orientation = R_down @ R_z
        else:
            # Alternative: point down first, then rotate in downward-pointing frame
            target_orientation = R_z @ R_down

        # Log resulting orientation
        result_rpy = R.from_matrix(target_orientation).as_euler('xyz')
        logger.info(f"🎯 Grasp orientation composition:")
        logger.info(f"   Base frame RPY: [R={np.degrees(result_rpy[0]):6.1f}°, "
                    f"P={np.degrees(result_rpy[1]):6.1f}°, Y={np.degrees(result_rpy[2]):6.1f}°]")
        logger.info(
            f"   └─ Yaw ({np.degrees(result_rpy[2]):.1f}°) = jaw closing axis")

        return target_orientation

    def pose_to_joint_angles(self, pose: List[float]) -> Optional[np.ndarray]:
        """
        Convert grasp pose to joint angles using inverse kinematics.

        Args:
            pose: [x,y,z, roll,pitch,yaw] in base frame (meters, radians)

        Returns:
            Array of 7 joint angles (radians) or None if IK fails
        """
        try:
            current_joints = self.telemetry.get_current_joints()
            if len(current_joints) != 7:
                logger.warning("Invalid current joint positions")
                return None

            # Final safety check for Z coordinate
            pose_meters = list(pose)
            if pose_meters[2] < 0.0:
                logger.warning(f"Final pose Z below workspace floor: {pose_meters[2]:.3f}m, "
                               f"clamping to 0.0m")
                pose_meters[2] = 0.0

            logger.info(f"Final pose for IK: position={pose_meters[:3]}, "
                        f"orientation={pose_meters[3:6]}")

            # Solve IK
            joint_angles = self.kinematics_solver.solve_pose(
                pose_meters, current_joints.tolist()
            )

            logger.debug(f"Joint angles: {joint_angles}")
            return joint_angles

        except Exception as e:
            logger.error(f"Failed to convert pose to joint angles: {e}")
            return None
