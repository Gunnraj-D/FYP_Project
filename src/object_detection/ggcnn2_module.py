import torch
import torch.nn.functional as F
import cv2
import numpy as np
from pathlib import Path
import logging
from typing import Optional, Dict, List, Tuple
from scipy.spatial.transform import Rotation as R

from control.telemetry_store import Telemetry
from control.command_bus import CommandBus
from camera_management.camera_manager import CameraManager
from camera_management.camera_transform_module import transform_camera_to_base
from camera_management.table_reference import TableReferenceModule, TableReferenceConfig
from kinematics.kinematics_solver import InverseKinematicsSolver
from object_detection.ggcnn2 import GGCNN2
from config.config import GRASP_DETECTION_CONFIG, GRASP_EXECUTION_CONFIG

logger = logging.getLogger(__name__)


class GGcnn2Module:
    """
    Wrapper around the GGCNN2 grasp synthesis network.

    Responsibilities:
    - Preprocess depth images
    - Run inference on GGCNN2
    - Postprocess outputs into grasp parameters (center, angle, width)
    - Transform grasp pose from camera to robot base frame
    - Convert grasp pose to joint angles using inverse kinematics
    - Store results in shared telemetry
    """

    def __init__(self, model_path: str, telemetry: Telemetry, command_bus: CommandBus,
                 camera_manager: CameraManager, kinematics_solver: InverseKinematicsSolver):
        self.telemetry = telemetry
        self.command_bus = command_bus
        self.camera_manager = camera_manager
        self.kinematics_solver = kinematics_solver
        self.device = torch.device(
            "cuda" if torch.cuda.is_available() else "cpu")

        # Init model
        self.model = GGCNN2()
        state_dict = torch.load(model_path, map_location=self.device)
        self.model.load_state_dict(state_dict)
        self.model.to(self.device).eval()

        # Initialize table reference module
        table_config = TableReferenceConfig()
        self.table_reference = TableReferenceModule(table_config)

        logger.info(f"GGCNN2 model loaded from {model_path} on {self.device}")

    # ----------------------------
    # Preprocessing
    # ----------------------------
    def preprocess(self, depth_image: np.ndarray) -> torch.Tensor:
        """
        Prepare depth image for GG-CNN2.
        - Resize to 300x300
        - Normalize to [0,1]
        - Convert to torch tensor
        """
        depth = cv2.resize(depth_image, (300, 300))

        # Normalization: clip to reasonable range (you may adjust min/max)
        depth_min, depth_max = np.nanmin(depth), np.nanmax(depth)
        depth = (depth - depth_min) / (depth_max - depth_min + 1e-6)
        depth = np.clip(depth, 0.0, 1.0)

        depth_tensor = torch.from_numpy(
            depth).unsqueeze(0).unsqueeze(0).float()
        return depth_tensor.to(self.device)

    # ----------------------------
    # Inference
    # ----------------------------
    def infer(self, depth_image: np.ndarray) -> Optional[Dict]:
        """
        Run GG-CNN2 on a depth image and return best grasp candidate with joint angles.

        Returns:
            Dict containing grasp parameters and joint angles, or None if no valid grasp found
        """
        try:
            depth_tensor = self.preprocess(depth_image)

            with torch.no_grad():
                pos, cos, sin, width = self.model(depth_tensor)

            # Decode outputs
            q_img = torch.sigmoid(pos)                 # grasp quality
            ang_img = 0.5 * torch.atan2(sin, cos)      # angle [-pi/2, pi/2]
            # enforce non-negative width
            width_img = F.relu(width)

            # Pick best grasp
            grasp_2d = self.postprocess(q_img, ang_img, width_img)

            if grasp_2d is None:
                logger.warning("No valid grasp found in image")
                return None

            # Convert 2D grasp to 3D pose in camera frame
            grasp_pose_camera = self._grasp_2d_to_3d_pose(
                grasp_2d, depth_image)

            if grasp_pose_camera is None:
                logger.warning("Failed to convert 2D grasp to 3D pose")
                return None

            # Transform to robot base frame
            grasp_pose_base = self._transform_to_base_frame(grasp_pose_camera)

            if grasp_pose_base is None:
                logger.warning("Failed to transform grasp pose to base frame")
                return None

            # Convert to joint angles
            joint_angles = self._pose_to_joint_angles(grasp_pose_base)

            if joint_angles is None:
                logger.warning("Failed to convert grasp pose to joint angles")
                return None

            # Update table reference with current depth frame
            self.table_reference.update_table_reference(depth_image)

            # Compute height above table for the grasp
            grasp_height = self._compute_grasp_height(depth_image, grasp_2d)

            # Create complete grasp result
            grasp_result = {
                'grasp_2d': grasp_2d,
                'grasp_pose_camera': grasp_pose_camera,
                'grasp_pose_base': grasp_pose_base,
                'joint_angles': joint_angles.tolist(),
                'quality': grasp_2d.get('quality', 0.0),
                'grasp_height': grasp_height
            }

            # Store in telemetry
            self._store_grasp_result(grasp_result)

            logger.info(
                f"Valid grasp found with quality: {grasp_result['quality']:.3f}, "
                f"height: {grasp_height:.3f}m")
            return grasp_result

        except Exception as e:
            logger.error(f"GGCNN2 inference failed: {e}")
            return None

    # ----------------------------
    # Postprocessing
    # ----------------------------
    def postprocess(self, q_img, ang_img, width_img) -> Optional[Dict]:
        """
        Select grasp with max quality score.
        Returns dict with center (u,v), angle, width, quality.
        """
        q_np = q_img.squeeze().cpu().numpy()
        ang_np = ang_img.squeeze().cpu().numpy()
        width_np = width_img.squeeze().cpu().numpy()

        max_idx = np.unravel_index(np.argmax(q_np), q_np.shape)
        row, col = max_idx

        quality = float(q_np[row, col])

        # Only return grasp if quality is above threshold
        if quality < GRASP_DETECTION_CONFIG['min_quality_threshold']:
            return None

        return {
            "center": (col, row),
            "angle": float(ang_np[row, col]),
            "width": float(width_np[row, col]),
            "quality": quality
        }

    # ----------------------------
    # 3D Pose Conversion
    # ----------------------------
    def _grasp_2d_to_3d_pose(self, grasp_2d: Dict, depth_image: np.ndarray) -> Optional[List[float]]:
        """
        Convert 2D grasp parameters to 3D pose in camera frame.

        Returns:
            [x, y, z, roll, pitch, yaw] in camera frame (mm, rad)
        """
        try:
            center_u, center_v = grasp_2d["center"]
            angle = grasp_2d["angle"]
            width = grasp_2d["width"]

            # Scale coordinates back to original image size
            h_orig, w_orig = depth_image.shape
            scale_u = w_orig / 300.0
            scale_v = h_orig / 300.0

            center_u_scaled = center_u * scale_u
            center_v_scaled = center_v * scale_v

            # Get depth at grasp center
            depth = self.camera_manager.get_average_depth(
                depth_image, (int(center_u_scaled), int(center_v_scaled)), 5
            )

            if depth <= 0:
                logger.warning("Invalid depth at grasp center")
                return None

            # Convert pixel to 3D coordinates
            x, y, z = self.camera_manager.pixel_to_3d(
                int(center_u_scaled), int(center_v_scaled), depth
            )

            # Create grasp orientation
            # For GGCNN2, the angle represents the gripper orientation
            approach_angle_rad = np.radians(
                GRASP_DETECTION_CONFIG['approach_angle'])
            roll = 0.0
            pitch = approach_angle_rad  # Approach angle from config
            yaw = angle  # Gripper rotation from GGCNN2

            pose = [x, y, z, roll, pitch, yaw]
            logger.debug(f"3D grasp pose: {pose}")
            return pose

        except Exception as e:
            logger.error(f"Failed to convert 2D grasp to 3D pose: {e}")
            return None

    def _transform_to_base_frame(self, camera_pose: List[float]) -> Optional[List[float]]:
        """
        Transform grasp pose from camera frame to robot base frame.

        Returns:
            [x, y, z, roll, pitch, yaw] in base frame (mm, rad)
        """
        try:
            # Get current robot TCP pose
            current_joints = self.telemetry.get_current_joints()
            if len(current_joints) != 7:
                logger.warning("Invalid current joint positions")
                return None

            _, tcp_matrix = self.kinematics_solver.tcp_from_joints(
                current_joints.tolist())

            # Transform position from camera to base frame
            camera_position = camera_pose[:3]
            base_position = transform_camera_to_base(
                camera_position, tcp_matrix)

            # For orientation, we'll keep the same relative orientation
            # This is a simplified approach - you might want more sophisticated orientation handling
            orientation = camera_pose[3:6]

            base_pose = base_position.tolist() + orientation
            logger.debug(f"Base frame grasp pose: {base_pose}")
            return base_pose

        except Exception as e:
            logger.error(f"Failed to transform to base frame: {e}")
            return None

    def _pose_to_joint_angles(self, pose: List[float]) -> Optional[np.ndarray]:
        """
        Convert grasp pose to joint angles using inverse kinematics.

        Returns:
            Array of 7 joint angles (rad)
        """
        try:
            current_joints = self.telemetry.get_current_joints()
            if len(current_joints) != 7:
                logger.warning("Invalid current joint positions")
                return None

            joint_angles = self.kinematics_solver.solve_pose(
                pose, current_joints.tolist()
            )

            logger.debug(f"Joint angles: {joint_angles}")
            return joint_angles

        except Exception as e:
            logger.error(f"Failed to convert pose to joint angles: {e}")
            return None

    def _compute_grasp_height(self, depth_image: np.ndarray, grasp_2d: Dict) -> float:
        """
        Compute height of object above table for the given grasp.

        Args:
            depth_image: Current depth frame
            grasp_2d: 2D grasp parameters

        Returns:
            Height above table in meters
        """
        try:
            # Scale coordinates back to original image size
            h_orig, w_orig = depth_image.shape
            scale_u = w_orig / 300.0
            scale_v = h_orig / 300.0

            center_u, center_v = grasp_2d["center"]
            width = grasp_2d["width"]
            angle = grasp_2d["angle"]

            # Scale grasp center to original image coordinates
            center_u_scaled = int(center_u * scale_u)
            center_v_scaled = int(center_v * scale_v)

            # Create ROI mask for the grasp rectangle
            roi_mask = self.table_reference.create_grasp_roi_mask(
                grasp_center=(center_u_scaled, center_v_scaled),
                grasp_width=width,
                grasp_angle=angle,
                image_shape=(h_orig, w_orig)
            )

            # Compute height above table using ROI
            height = self.table_reference.get_height_above_table(
                depth_frame=depth_image,
                grasp_uv=(center_u_scaled, center_v_scaled),
                roi_mask=roi_mask
            )

            return height

        except Exception as e:
            logger.error(f"Failed to compute grasp height: {e}")
            return 0.0

    def _store_grasp_result(self, grasp_result: Dict):
        """
        Store grasp result in telemetry and command bus.
        """
        try:
            # Store in telemetry (you may need to extend telemetry for grasp data)
            # For now, we'll store the joint angles as target joints
            joint_angles = grasp_result['joint_angles']
            grasp_height = grasp_result.get('grasp_height', 0.0)

            # Store grasp height in telemetry
            self.telemetry.update_grasp_height(grasp_height)

            # Send joint command to robot
            from control.command_bus import SetJoints
            self.command_bus.send(SetJoints(joints=joint_angles))

            logger.info(
                f"Grasp command sent: {joint_angles}, height: {grasp_height:.3f}m")

        except Exception as e:
            logger.error(f"Failed to store grasp result: {e}")

    # ----------------------------
    # Convenience Methods
    # ----------------------------
    def process_depth_frame(self, depth_frame) -> Optional[Dict]:
        """
        Convenience method to process a depth frame from camera manager.

        Args:
            depth_frame: Depth frame from camera manager

        Returns:
            Grasp result dict or None
        """
        try:
            # Convert depth frame to numpy array
            if hasattr(depth_frame, 'get_data'):
                depth_array = np.asanyarray(depth_frame.get_data())
            else:
                depth_array = depth_frame

            return self.infer(depth_array)

        except Exception as e:
            logger.error(f"Failed to process depth frame: {e}")
            return None

    def cleanup(self):
        """Clean up resources and free memory."""
        try:
            if hasattr(self, 'model'):
                del self.model
            if hasattr(self, 'table_reference'):
                self.table_reference.reset()
                del self.table_reference
            logger.info("GGCNN2 module cleaned up")
        except Exception as e:
            logger.error(f"Error during cleanup: {e}")

    def get_table_reference_stats(self) -> Dict:
        """Get statistics about the table reference module."""
        try:
            if hasattr(self, 'table_reference'):
                return self.table_reference.get_stats()
            return {}
        except Exception as e:
            logger.error(f"Failed to get table reference stats: {e}")
            return {}
