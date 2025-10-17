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
from config import GRASP_DETECTION_CONFIG, CAMERA_ROTATION_EULER

logger = logging.getLogger(__name__)


# ============================================================================
# GRASP TRANSFORMER CLASS
# ============================================================================

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

    # ========================================================================
    # 2D TO 3D CONVERSION
    # ========================================================================

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

        Raises:
            ValueError: If depth is invalid or conversion fails
        """
        try:
            center_u, center_v = grasp_2d["center"]
            angle = grasp_2d["angle"]

            # Apply angle offset (for jaw axis alignment)
            angle_offset_rad = GRASP_DETECTION_CONFIG.get(
                'grasp_angle_offset_rad', 0.0)
            angle += angle_offset_rad

            # Scale coordinates back to original image resolution
            scaled_u, scaled_v = self._scale_coordinates(
                center_u, center_v, depth_image, original_depth_frame
            )

            # Get depth at grasp center
            depth_m = self._get_grasp_depth(
                grasp_2d, depth_image, scaled_u, scaled_v, original_depth_frame
            )

            # Convert pixel to 3D coordinates in camera frame
            x, y, z = self.camera_manager.pixel_to_3d(
                int(scaled_u), int(scaled_v), depth_m
            )

            logger.warning(
                f"📍 pixel_to_3d: px=({int(scaled_u)},{int(scaled_v)}), "
                f"depth={depth_m:.3f}m → cam_xyz=({x:.3f}, {y:.3f}, {z:.3f})"
            )

            # Create grasp orientation in camera frame
            grasp_orientation_matrix = self.compose_grasp_orientation(angle)
            grasp_rpy_camera = R.from_matrix(
                grasp_orientation_matrix).as_euler('xyz')

            pose = [x, y, z] + grasp_rpy_camera.tolist()
            logger.debug(
                f"3D grasp pose (camera frame): pos={pose[:3]}, "
                f"ori(deg)=[{np.degrees(pose[3]):.1f}, {np.degrees(pose[4]):.1f}, {np.degrees(pose[5]):.1f}]"
            )

            return pose

        except Exception as e:
            logger.error(f"Failed to convert 2D grasp to 3D pose: {e}")
            raise ValueError(f"2D to 3D grasp conversion failed: {e}")

    def _scale_coordinates(self, center_u: float, center_v: float,
                           depth_image: np.ndarray, original_depth_frame) -> tuple:
        """Scale coordinates from 300x300 to original resolution."""
        if original_depth_frame:
            h_orig = original_depth_frame.get_height()
            w_orig = original_depth_frame.get_width()
        else:
            h_orig = depth_image.shape[0]
            w_orig = depth_image.shape[1]

        scale_u = w_orig / 300.0
        scale_v = h_orig / 300.0

        scaled_u = center_u * scale_u
        scaled_v = center_v * scale_v

        return scaled_u, scaled_v

    def _get_grasp_depth(self, grasp_2d: dict, depth_image: np.ndarray,
                         scaled_u: float, scaled_v: float, original_depth_frame) -> float:
        """
        Get depth at grasp center, preferring pre-calculated depth.

        Raises:
            ValueError: If no valid depth can be obtained
        """
        # PREFER the pre-calculated depth from postprocessor
        depth_m = grasp_2d.get('depth_m', None)
        logger.warning(f"🔍 TRANSFORM: grasp_2d depth_m = {depth_m}")

        if depth_m is not None and depth_m > 0:
            logger.warning(
                f"✅ USING PRE-CALC DEPTH: {depth_m:.3f}m from postprocessor")
            return depth_m

        # No valid pre-calculated depth - this should not happen
        raise ValueError(
            f"No valid pre-calculated depth available (got {depth_m}). "
            "Depth must be calculated by postprocessor before transformation."
        )

    # ========================================================================
    # CAMERA TO BASE FRAME TRANSFORMATION
    # ========================================================================

    def transform_to_base_frame(self, camera_pose: List[float]) -> Optional[List[float]]:
        """
        Transform grasp pose from camera frame to robot base frame.

        Args:
            camera_pose: [x,y,z, roll,pitch,yaw] in camera frame

        Returns:
            [x,y,z, roll,pitch,yaw] in base frame

        Raises:
            ValueError: If transformation fails
        """
        try:
            # Get current TCP pose from joints
            current_joints = self.telemetry.get_current_joints()
            if len(current_joints) != 7:
                logger.warning("Invalid current joint positions")
                return None

            tcp_matrix, _ = self.kinematics_solver.tcp_from_joints(
                current_joints.tolist())

            # Transform position
            camera_position = np.array(camera_pose[:3])
            base_position = transform_camera_to_base(
                camera_position, tcp_matrix)

            # Transform orientation
            base_orientation = self._transform_orientation(
                camera_pose[3:6], tcp_matrix)

            # Apply height correction offset
            from config import GRASP_DETECTION_CONFIG
            height_offset = GRASP_DETECTION_CONFIG.get(
                'grasp_height_offset', 0.0)
            base_position[2] += height_offset

            # Compose final base pose
            base_pose = base_position.tolist() + base_orientation.tolist()

            logger.debug(f"Transformed base frame grasp pose: {base_pose}")

            # Validate Z is positive (but don't fail - let grasping_state handle it)
            if base_pose[2] < 0.0:
                logger.warning(
                    f"Transformed grasp Z={base_pose[2]:.3f}m is below workspace floor (Z=0). "
                    "Will be handled by grasping_state validation.")

            return base_pose

        except Exception as e:
            logger.error(f"Failed to transform to base frame: {e}")
            raise ValueError(
                f"Camera to base frame transformation failed: {e}")

    def _build_transform_matrix(self, tcp_pose: List[float]) -> np.ndarray:
        """Build 4x4 transformation matrix from TCP pose."""
        tcp_rot = R.from_euler('xyz', tcp_pose[3:6])
        tcp_matrix = np.eye(4)
        tcp_matrix[:3, :3] = tcp_rot.as_matrix()
        tcp_matrix[:3, 3] = tcp_pose[:3]
        return tcp_matrix

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

    # ========================================================================
    # GRASP ORIENTATION COMPOSITION
    # ========================================================================

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

        # Get composition order from config
        compose_order = GRASP_DETECTION_CONFIG.get(
            'compose_order', 'down_then_z')

        if compose_order == 'down_then_z':
            # Recommended: align orientation in base frame, then point down
            target_orientation = R_down @ R_z
        elif compose_order == 'z_then_down':
            # Alternative: point down first, then rotate in downward-pointing frame
            target_orientation = R_z @ R_down
        else:
            raise ValueError(
                f"Invalid compose_order: {compose_order}. Must be 'down_then_z' or 'z_then_down'")

        # Log resulting orientation
        result_rpy = R.from_matrix(target_orientation).as_euler('xyz')
        logger.info(f"🎯 Grasp orientation composition:")
        logger.info(
            f"   Base frame RPY: [R={np.degrees(result_rpy[0]):6.1f}°, "
            f"P={np.degrees(result_rpy[1]):6.1f}°, Y={np.degrees(result_rpy[2]):6.1f}°]"
        )
        logger.info(
            f"   └─ Yaw ({np.degrees(result_rpy[2]):.1f}°) = jaw closing axis")

        return target_orientation

    # ========================================================================
    # INVERSE KINEMATICS
    # ========================================================================

    def pose_to_joint_angles(self, pose: List[float]) -> np.ndarray:
        """
        Convert grasp pose to joint angles using inverse kinematics.

        Args:
            pose: [x,y,z, roll,pitch,yaw] in base frame (meters, radians)

        Returns:
            Array of 7 joint angles (radians)

        Raises:
            ValueError: If IK fails or current joints are invalid
        """
        try:
            # Get current joint positions
            current_joints = self.telemetry.get_current_joints()
            if len(current_joints) != 7:
                raise ValueError(
                    f"Invalid current joint positions: expected 7, got {len(current_joints)}")

            # Validate pose is above workspace floor
            if pose[2] < 0.0:
                raise ValueError(
                    f"Grasp pose Z={pose[2]:.3f}m is below workspace floor (Z=0). "
                    "Cannot compute IK for invalid pose."
                )

            logger.debug(
                f"Computing IK for pose: position={pose[:3]}, "
                f"orientation(deg)=[{np.degrees(pose[3]):.1f}, {np.degrees(pose[4]):.1f}, {np.degrees(pose[5]):.1f}]"
            )

            # Solve IK
            joint_angles = self.kinematics_solver.solve_pose(
                pose, current_joints.tolist())

            if joint_angles is None:
                raise ValueError("IK solver returned None - no solution found")

            logger.debug(f"IK solution: {joint_angles}")
            return joint_angles

        except Exception as e:
            logger.error(f"Failed to convert pose to joint angles: {e}")
            raise ValueError(f"IK computation failed: {e}")
