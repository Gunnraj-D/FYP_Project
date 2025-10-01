import random
import math
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
from kinematics.kinematics_solver import InverseKinematicsSolver, get_facing_down_orientation
from object_detection.ggcnn2 import GGCNN2
from config.config import GRASP_DETECTION_CONFIG, GRASP_EXECUTION_CONFIG, DEBUG_MODE, DEBUG_CONFIG

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

        logger.info(f"GGCNN2 model loaded from {model_path} on {self.device}")

    # ----------------------------
    # Preprocessing
    # ----------------------------
    def preprocess(self, depth_image: np.ndarray) -> torch.Tensor:
        # Crop to square to avoid aspect ratio distortion
        h, w = depth_image.shape
        min_dim = min(h, w)
        start_h = (h - min_dim) // 2
        start_w = (w - min_dim) // 2
        depth = depth_image[start_h:start_h+min_dim, start_w:start_w+min_dim]

        depth = cv2.resize(depth, (300, 300))

        # Fixed normalization range (0.2m–1.2m typical for tabletop)
        depth = np.clip(depth, 200, 1200)   # mm
        depth = (depth - 200) / (1000)      # normalize [0,1]

        depth_tensor = torch.from_numpy(
            depth).unsqueeze(0).unsqueeze(0).float()
        return depth_tensor.to(self.device)

    # ----------------------------
    # Inference
    # ----------------------------

    def infer(self, depth_image: np.ndarray, original_depth_frame=None) -> Optional[Dict]:
        """
        Run GG-CNN2 on a depth image and return best grasp candidate with joint angles.

        Returns:
            Dict containing grasp parameters and joint angles, or None if no valid grasp found
        """
        try:
            # Visualize input frame if debug mode is enabled
            self._visualize_input_frame(depth_image, "GG-CNN2 Input Frame")

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

            # Visualize grasp output if debug mode is enabled
            self._visualize_grasp_output(
                depth_image, grasp_2d, "GG-CNN2 Grasp Output")

            # Convert 2D grasp to 3D pose in camera frame
            grasp_pose_camera = self._grasp_2d_to_3d_pose(
                grasp_2d, depth_image, original_depth_frame)

            if grasp_pose_camera is None:
                logger.warning("Failed to convert 2D grasp to 3D pose")
                return None

            logger.info(f"Generated camera pose: {grasp_pose_camera}")

            # CODE HERE

            # 1. Get current robot state for transformations
            current_joints = self.telemetry.get_current_joints()
            if len(current_joints) != 7:
                logger.error("Could not get current valid joint positions.")
                return None
            tcp_matrix, _ = self.kinematics_solver.tcp_from_joints(
                current_joints.tolist())

            # 2. Transform ONLY the grasp POSITION to the base frame
            grasp_position_camera = grasp_pose_camera[:3]
            target_position = transform_camera_to_base(
                grasp_position_camera, tcp_matrix)

            # 3. Define the desired grasp ORIENTATION directly in the base frame
            # This ensures the gripper always tries to point downwards.

            # Get the rotation angle from the 2D grasp detection
            grasp_angle_rad = grasp_2d["angle"]

            # Create a rotation matrix for this angle around the world Z-axis
            # This aligns the gripper with the object on the table
            from scipy.spatial.transform import Rotation as R
            R_z = R.from_euler('z', grasp_angle_rad).as_matrix()

            # Get the standard "facing down" rotation matrix from the kinematics solver
            # This points the gripper towards the table
            R_down = get_facing_down_orientation()

            # Combine them: First, point down, then rotate around Z-axis
            # The order of multiplication is important!
            target_orientation_matrix = R_z @ R_down

            # Log the resulting orientation for verification
            result_rpy = R.from_matrix(
                target_orientation_matrix).as_euler('xyz')
            logger.info(
                f"Grasp orientation - angle from GGCNN2: {np.degrees(grasp_angle_rad):.1f}°, "
                f"resulting RPY: [{np.degrees(result_rpy[0]):.1f}°, {np.degrees(result_rpy[1]):.1f}°, {np.degrees(result_rpy[2]):.1f}°]")

            # 4. Solve IK with the decoupled position and orientation
            logger.info(
                f"Attempting IK for position={target_position.tolist()}"
            )

            # Use solve_XYZ which takes a position and an orientation matrix
            joint_angles = self.kinematics_solver.solve_XYZ(
                target_position=target_position.tolist(),
                current_joint_angles=current_joints.tolist(),
                target_orientation=target_orientation_matrix
            )

            if joint_angles is None:
                logger.warning(
                    "Failed to convert grasp pose to joint angles with new method.")
                return None

            # CODE HERE

            # Reconstruct the grasp_pose_base for logging/telemetry if needed
            base_orientation_rpy = R.from_matrix(
                target_orientation_matrix).as_euler('xyz').tolist()
            grasp_pose_base = target_position.tolist() + base_orientation_rpy

            # Compute height above table for the grasp using RANSAC
            grasp_height = self._compute_grasp_height_ransac(
                depth_image, grasp_2d, original_depth_frame)
            logger.info(f"Grasp height above table: {grasp_height:.3f}m")

            # Create complete grasp result
            grasp_result = {
                'grasp_2d': grasp_2d,
                'grasp_pose_camera': grasp_pose_camera,
                'grasp_pose_base': grasp_pose_base,
                'joint_angles': joint_angles.tolist(),
                'quality': grasp_2d.get('quality', 0.0),
                'grasp_height': grasp_height
            }

            # # Transform to robot base frame
            # grasp_pose_base = self._transform_to_base_frame(grasp_pose_camera)

            # if grasp_pose_base is None:
            #     logger.warning("Failed to transform grasp pose to base frame")
            #     return None

            # logger.info(f"Transformed base pose: {grasp_pose_base}")

            # # Convert to joint angles
            # joint_angles = self._pose_to_joint_angles(grasp_pose_base)

            # if joint_angles is None:
            #     logger.warning("Failed to convert grasp pose to joint angles")
            #     return None

            # # Compute height above table for the grasp
            # grasp_height = self._compute_grasp_height(depth_image, grasp_2d)

            # Create complete grasp result
            # grasp_result = {
            #     'grasp_2d': grasp_2d,
            #     'grasp_pose_camera': grasp_pose_camera,
            #     'grasp_pose_base': grasp_pose_base,
            #     'joint_angles': joint_angles.tolist(),
            #     'quality': grasp_2d.get('quality', 0.0),
            #     'grasp_height': grasp_height
            # }

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
    # def postprocess(self, q_img, ang_img, width_img) -> Optional[Dict]:
    #     """
    #     Select grasp with max quality score.
    #     Returns dict with center (u,v), angle, width, quality.
    #     """
    #     q_np = q_img.squeeze().cpu().numpy()
    #     ang_np = ang_img.squeeze().cpu().numpy()
    #     width_np = width_img.squeeze().cpu().numpy()

    #     max_idx = np.unravel_index(np.argmax(q_np), q_np.shape)
    #     row, col = max_idx

    #     quality = float(q_np[row, col])

    #     # Only return grasp if quality is above threshold
    #     if quality < GRASP_DETECTION_CONFIG['min_quality_threshold']:
    #         return None

    #     return {
    #         "center": (col, row),
    #         "angle": float(ang_np[row, col]),
    #         "width": float(width_np[row, col]),
    #         "quality": quality
    #     }

    def postprocess(self, q_img, ang_img, width_img, depth_image=None, original_depth_frame=None, top_k=8):
        """
        Robust selection:
        - smooth quality map
        - local peak detection
        - compute quality-weighted centroid
        - score top candidates with depth-edge and width penalties
        """
        q_np = q_img.squeeze().cpu().numpy()
        ang_np = ang_img.squeeze().cpu().numpy()
        width_np = width_img.squeeze().cpu().numpy()

        # 1) Smooth quality to reduce noisy edge maxima
        q_blur = cv2.GaussianBlur(q_np, (5, 5), 2)

        # 2) Basic thresholding
        base_thresh = GRASP_DETECTION_CONFIG.get('min_quality_threshold', 0.15)
        thresh = max(base_thresh, 0.02)

        # 3) Local maxima (NMS) using dilation
        kernel = np.ones((5, 5), np.uint8)
        dilated = cv2.dilate(q_blur, kernel)
        local_max_mask = (q_blur == dilated) & (q_blur > thresh)

        # if no local maxima, use top-k global indices
        if local_max_mask.sum() == 0:
            flat_idxs = np.argpartition(q_blur.flatten(), -top_k)[-top_k:]
            cand_rows, cand_cols = np.unravel_index(flat_idxs, q_blur.shape)
        else:
            cand_rows, cand_cols = np.where(local_max_mask)
            # if too many, keep best top_k by quality
            if len(cand_rows) > top_k:
                qualities = q_blur[cand_rows, cand_cols]
                order = np.argsort(qualities)[-top_k:]
                cand_rows = cand_rows[order]
                cand_cols = cand_cols[order]

        # 4) compute quality-weighted centroid (to prefer interior peaks)
        mask = q_blur.copy()
        mask[mask < thresh] = 0.0
        if mask.sum() > 0:
            rows_idx = np.arange(q_blur.shape[0])[:, None]
            cols_idx = np.arange(q_blur.shape[1])[None, :]
            com_v = float((mask * rows_idx).sum() / mask.sum())
            com_u = float((mask * cols_idx).sum() / mask.sum())
        else:
            # fallback centroid
            com_v, com_u = q_blur.shape[0] / 2.0, q_blur.shape[1] / 2.0

        # Helpers for depth checks: we prefer original depth frame if provided
        def sample_depth_m(u_resized, v_resized):
            """
            Return depth in meters at resized image coordinates.
            Uses original_depth_frame if available, otherwise uses depth_image (numpy).
            """
            # map resized coords back to original depth image size
            if depth_image is None and original_depth_frame is None:
                return None
            # original depth dims (if numpy)
            if original_depth_frame is not None:
                # CameraManager.get_average_depth expects depth_frame-like, so use it directly
                # convert to int pixel coordinates in original frame
                h_orig = original_depth_frame.get_height()
                w_orig = original_depth_frame.get_width()
                scale_u = w_orig / 300.0
                scale_v = h_orig / 300.0
                u_orig, v_orig = int(
                    u_resized * scale_u), int(v_resized * scale_v)
                d = self.camera_manager.get_average_depth(
                    original_depth_frame, (u_orig, v_orig), radius=4)
                return float(d) if d > 0 else None
            else:
                # depth_image is numpy array (assume shape (H,W), unit: mm or meters depending on your pipeline)
                depth_np = depth_image
                h_orig, w_orig = depth_np.shape
                scale_u = w_orig / 300.0
                scale_v = h_orig / 300.0
                u_orig, v_orig = int(
                    u_resized * scale_u), int(v_resized * scale_v)
                if u_orig < 0 or u_orig >= w_orig or v_orig < 0 or v_orig >= h_orig:
                    return None
                val = depth_np[v_orig, u_orig]
                if val <= 0:
                    return None
                # if values look like millimeters (large) convert to meters
                if val > 10:  # >10 likely mm
                    return float(val) / 1000.0
                else:
                    return float(val)

        # intrinsics for width->meters conversion
        intr = getattr(self.camera_manager, "intrinsics", None)
        fx = intr.fx if (intr is not None and hasattr(intr, "fx")) else None

        # gripper width min/max (meters). Add sensible defaults or use config
        grip_min = GRASP_EXECUTION_CONFIG.get('gripper_min_width_m', 0.02)
        grip_max = GRASP_EXECUTION_CONFIG.get('gripper_max_width_m', 0.12)

        # 5) Score candidates
        best = None
        best_score = -1e9
        h_resized, w_resized = q_blur.shape

        for r, c in zip(cand_rows, cand_cols):
            quality = float(q_blur[r, c])
            # distance to centroid (smaller = better)
            dist = np.hypot(c - com_u, r - com_v) / \
                max(h_resized, w_resized)  # normalized

            # local depth stability: sample a small patch's std or gradient
            depth_m = sample_depth_m(c, r)
            edge_penalty = 0.0
            if depth_m is not None and depth_m > 0:
                # approximate gradient magnitude around that pixel in original depth
                # fallback: sample neighbors in resized space and compute std using depth_image if available
                # Simpler: compute depth variance in small neighborhood if we have numpy depth
                if original_depth_frame is not None:
                    # use CameraManager to fetch a small patch manually (cheap)
                    # reuse get_average_depth to get single value; to estimate edge we can sample 4 offsets
                    d_center = depth_m
                    offsets = [(3, 0), (-3, 0), (0, 3), (0, -3)]
                    vals = []
                    for dx, dy in offsets:
                        d2 = sample_depth_m(c+dx, r+dy)
                        if d2 is not None:
                            vals.append(d2)
                    if len(vals) > 0:
                        depth_var = float(np.std(vals + [d_center]))
                        # normalize (penalize if variance > 0.01m)
                        edge_penalty = min(1.0, depth_var / 0.02)
                else:
                    # have numpy depth image -> compute local std in a small window
                    try:
                        depth_np = depth_image
                        h_orig, w_orig = depth_np.shape
                        su = int((c / 300.0) * w_orig)
                        sv = int((r / 300.0) * h_orig)
                        ws = 5
                        x0 = max(0, su-ws)
                        x1 = min(w_orig, su+ws)
                        y0 = max(0, sv-ws)
                        y1 = min(h_orig, sv+ws)
                        patch = depth_np[y0:y1, x0:x1]
                        patch = patch[patch > 0]
                        if patch.size > 0:
                            # patch likely mm -> convert to meters if >10
                            if patch.max() > 10:
                                patch_m = patch / 1000.0
                            else:
                                patch_m = patch
                            depth_var = float(np.std(patch_m))
                            edge_penalty = min(1.0, depth_var / 0.02)
                    except Exception:
                        edge_penalty = 0.0
            else:
                # if depth unknown, penalize slightly (safer)
                edge_penalty = 0.2

            # width penalty: convert pixel width -> meters (if fx & depth known)
            width_pixels = float(width_np[r, c])  # width in resized pixels
            # convert to original pixels
            # original width (pixels) ~ width_pixels * (orig_w / 300)
            width_m = None
            if fx is not None and depth_m is not None:
                # convert to original pixel scale first
                # use color intrinsics width (camera_manager.intrinsics.width) if needed
                # convert with simple formula: width_m = width_pixels_resized * (depth_m / fx_resized)
                # fx is for original resolution; compute fx_resized = fx * (300 / orig_w)
                if hasattr(self.camera_manager.intrinsics, "width") and self.camera_manager.intrinsics.width:
                    orig_w = self.camera_manager.intrinsics.width
                    fx_resized = fx * (300.0 / orig_w)
                else:
                    fx_resized = fx  # if we can't scale, assume square mapping
                width_m = (width_pixels * depth_m) / fx_resized
            else:
                # unknown - leave width_m None
                width_m = None

            width_penalty = 0.0
            if width_m is not None:
                if width_m < grip_min:
                    width_penalty = (grip_min - width_m) / grip_min
                elif width_m > grip_max:
                    width_penalty = (width_m - grip_max) / grip_max
                width_penalty = min(1.0, width_penalty)
            else:
                # slight penalty if unknown
                width_penalty = 0.05

            # Combine into a final score: higher better
            # weights (tunable)
            score = (
                1.0 * quality
                - 0.6 * dist
                - 0.9 * edge_penalty
                - 0.7 * width_penalty
            )

            if score > best_score:
                best_score = score
                best = {
                    "center": (int(c), int(r)),
                    "angle": float(ang_np[r, c]),
                    "width": float(width_np[r, c]),
                    "quality": float(q_blur[r, c]),
                    "score": float(score),
                    "depth_m": depth_m,
                    "width_m": width_m
                }

        # final threshold check
        if best is None or best['quality'] < GRASP_DETECTION_CONFIG.get('min_quality_threshold', 0.15):
            return None

        return best

    # ----------------------------
    # 3D Pose Conversion
    # ----------------------------

    def _grasp_2d_to_3d_pose(self, grasp_2d: Dict, depth_image: np.ndarray, original_depth_frame=None) -> Optional[List[float]]:
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
            # Use original depth frame if available, otherwise use numpy array
            depth_source = original_depth_frame if original_depth_frame is not None else depth_image
            depth = self.camera_manager.get_average_depth(
                depth_source, (int(center_u_scaled), int(center_v_scaled)), 5
            )

            if depth <= 0:
                logger.warning(
                    f"Invalid depth at grasp center: depth={depth}, "
                    f"center=({int(center_u_scaled)}, {int(center_v_scaled)}), "
                    f"depth_frame_shape={depth_image.shape if hasattr(depth_image, 'shape') else 'unknown'}")
                # Try to get depth statistics to understand the issue
                if hasattr(depth_image, 'shape'):
                    logger.warning(
                        f"Depth stats: min={np.min(depth_image)}, max={np.max(depth_image)}, "
                        f"mean={np.mean(depth_image)}, non-zero pixels={np.count_nonzero(depth_image)}")
                return None

            # Convert pixel to 3D coordinates
            x, y, z = self.camera_manager.pixel_to_3d(
                int(center_u_scaled), int(center_v_scaled), depth
            )

            # Create grasp orientation
            # Simplify orientation for better IK convergence
            roll = 0.0
            pitch = 0.0  # Approach angle from config
            # Limit yaw to reasonable range for robot reachability
            yaw = np.clip(angle, -np.pi/2, np.pi/2)  # Limit to ±90 degrees

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

            tcp_matrix, tcp_pose = self.kinematics_solver.tcp_from_joints(
                current_joints.tolist())

            logger.debug(
                f"TCP matrix type: {type(tcp_matrix)}, shape: {getattr(tcp_matrix, 'shape', 'no shape')}")
            logger.debug(f"TCP pose type: {type(tcp_pose)}")

            # Transform position from camera to base frame
            camera_position = camera_pose[:3]
            logger.debug(
                f"Camera position: {camera_position}, length: {len(camera_position)}")
            base_position = transform_camera_to_base(
                camera_position, tcp_matrix)

            # Proper orientation transformation accounting for camera mounting
            # Camera orientation relative to TCP (from config)
            from config.config import CAMERA_ROTATION_EULER
            from scipy.spatial.transform import Rotation as R

            # Get camera mounting rotation (TCP -> Camera)
            camera_mount_euler = np.radians([
                CAMERA_ROTATION_EULER['roll'],
                CAMERA_ROTATION_EULER['pitch'],
                CAMERA_ROTATION_EULER['yaw']
            ])
            tcp_to_camera_rot = R.from_euler('xyz', camera_mount_euler)

            # Get grasp orientation in camera frame
            grasp_rpy_camera = camera_pose[3:6]
            grasp_in_camera_rot = R.from_euler('xyz', grasp_rpy_camera)

            # Get TCP orientation in base frame
            tcp_rot = R.from_matrix(tcp_matrix[:3, :3])

            # Compose: Base -> TCP -> Camera -> Grasp
            # grasp_in_base = tcp_in_base * tcp_to_camera * grasp_in_camera
            grasp_in_base_rot = tcp_rot * tcp_to_camera_rot * grasp_in_camera_rot

            # Convert back to euler angles
            grasp_rpy_base = grasp_in_base_rot.as_euler('xyz')

            # Compose final base pose
            base_pose = base_position.tolist() + grasp_rpy_base.tolist()
            logger.debug(
                f"Transformed base frame grasp pose: {base_pose}")
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

            # Pose is already in meters from camera_manager.pixel_to_3d()
            # No conversion needed - use pose directly
            pose_meters = [pose[0], pose[1],
                           pose[2], pose[3], pose[4], pose[5]]

            logger.debug(
                f"Pose being passed to solve_pose: {pose_meters}, length: {len(pose_meters)}")
            logger.info(
                f"Attempting IK for pose: position={pose_meters[:3]}, orientation={pose_meters[3:6]}")
            joint_angles = self.kinematics_solver.solve_pose(
                pose_meters, current_joints.tolist()
            )

            logger.debug(f"Joint angles: {joint_angles}")
            return joint_angles

        except Exception as e:
            logger.error(f"Failed to convert pose to joint angles: {e}")
            return None

    def _compute_grasp_height_ransac(self, depth_image: np.ndarray, grasp_2d: Dict, original_depth_frame=None) -> float:
        """
        RANSAC-based table plane detection for grasp height estimation.

        Args:
            depth_image: Depth image array
            grasp_2d: Grasp parameters with 'center' key
            original_depth_frame: Optional RealSense depth frame

        Returns:
            Height above table in meters, or fallback height if plane not found
        """
        try:
            # Get current TCP matrix for transformations
            current_joints = self.telemetry.get_current_joints()
            if current_joints is None or len(current_joints) != 7:
                logger.warning("Cannot compute grasp height - invalid joints")
                return 0.0
            tcp_matrix, _ = self.kinematics_solver.tcp_from_joints(
                current_joints.tolist())

            # Get grasp center in camera frame
            center_u, center_v = grasp_2d["center"]
            h_orig, w_orig = depth_image.shape
            scale_u = w_orig / 300.0
            scale_v = h_orig / 300.0
            center_u_scaled = int(center_u * scale_u)
            center_v_scaled = int(center_v * scale_v)

            # Get depth at grasp center
            depth_source = original_depth_frame if original_depth_frame is not None else depth_image
            grasp_depth = self.camera_manager.get_average_depth(
                depth_source, (center_u_scaled, center_v_scaled), 5
            )
            if grasp_depth <= 0:
                logger.warning("Invalid grasp depth for height calculation")
                return 0.0

            # Convert grasp pixel to 3D point in camera frame
            grasp_point_cam = np.array(self.camera_manager.pixel_to_3d(
                center_u_scaled, center_v_scaled, grasp_depth
            ))

            # Build point cloud from depth image (sample points for efficiency)
            point_cloud_cam = []
            sample_step = 10  # Sample every 10th pixel
            for v in range(0, h_orig, sample_step):
                for u in range(0, w_orig, sample_step):
                    d = self.camera_manager.get_average_depth(
                        depth_source, (u, v), 1)
                    if d is not None and d > 0:
                        pt = self.camera_manager.pixel_to_3d(u, v, d)
                        point_cloud_cam.append(pt)

            if len(point_cloud_cam) < 100:
                logger.warning("Insufficient points for plane fitting")
                return grasp_depth  # Fallback to grasp depth

            point_cloud_cam = np.array(point_cloud_cam)

            # RANSAC plane fitting
            ransac_iters = 200
            ransac_thresh = 0.01  # 1 cm tolerance
            min_inliers = 100

            best_plane = None
            best_mean_depth = -np.inf

            for _ in range(ransac_iters):
                # Sample 3 random points
                ids = np.random.choice(len(point_cloud_cam), 3, replace=False)
                pts = point_cloud_cam[ids]

                v1, v2 = pts[1] - pts[0], pts[2] - pts[0]
                n = np.cross(v1, v2)
                if np.linalg.norm(n) < 1e-6:
                    continue
                n = n / np.linalg.norm(n)

                d = -np.dot(n, pts[0])
                dists = np.abs(point_cloud_cam @ n + d)
                inliers = point_cloud_cam[dists < ransac_thresh]

                if len(inliers) < min_inliers:
                    continue

                # Transform plane normal to base frame to check if horizontal
                origin_base = transform_camera_to_base([0, 0, 0], tcp_matrix)
                tip_base = transform_camera_to_base(n.tolist(), tcp_matrix)
                n_base = tip_base - origin_base
                n_base = n_base / np.linalg.norm(n_base)

                # Require plane to be close to horizontal (within 25° of Z-axis)
                if abs(np.dot(n_base, [0, 0, 1])) < 0.9:
                    continue

                # Pick the deepest (largest z in camera coords) plane as the table
                mean_z = np.mean(inliers[:, 2])
                if mean_z > best_mean_depth:
                    best_plane = (n, d)
                    best_mean_depth = mean_z

            if best_plane is None:
                logger.warning(
                    "No valid table plane found via RANSAC - using grasp depth as fallback")
                return grasp_depth

            # Compute signed distance from grasp point to plane
            n, d = best_plane
            dist = abs((np.dot(n, grasp_point_cam) + d) / np.linalg.norm(n))

            logger.info(
                f"Table plane found with {best_mean_depth:.3f}m mean depth, grasp height: {dist:.3f}m")
            return dist

        except Exception as e:
            logger.error(f"Failed to compute grasp height with RANSAC: {e}")
            return 0.0

    def _compute_grasp_height(self, depth_image: np.ndarray, grasp_2d: Dict, original_depth_frame=None) -> float:
        """
        Robust table-plane-based grasp height estimation.

        Returns perpendicular distance (meters) from the grasp point to the estimated table plane.
        Fallbacks to a local median-based depth estimate if plane fit fails.
        """
        try:
            # Parameters (tune as needed)
            # exclude inner disk (object footprint) in pixels (resized coords)
            inner_radius_px = 12
            # outer radius of sample annulus in pixels (resized coords)
            outer_radius_px = 80
            sample_count = 1200          # how many candidate pixels to sample in the annulus
            ransac_iters = 250           # RANSAC iterations
            # 1.5 cm threshold for inlier (meters)
            ransac_inlier_thresh_m = 0.015
            min_inliers_for_plane = 200  # need this many inliers to accept plane
            fallback_median_window = 40  # if plane fails, take median depth in this px window

            # get resized->orig scaling
            # your pipeline resizes to 300x300 in preprocess, so mask/grasp coords are in that resized space
            # but postprocess returns coords in resized (300x300). We'll map to original depth resolution.
            if original_depth_frame is not None:
                h_orig = original_depth_frame.get_height()
                w_orig = original_depth_frame.get_width()
            else:
                h_orig, w_orig = depth_image.shape

            # compute scale from resized (300) back to original
            scale_u = w_orig / 300.0
            scale_v = h_orig / 300.0

            center_u_resized, center_v_resized = grasp_2d["center"]
            center_u = int(center_u_resized * scale_u)
            center_v = int(center_v_resized * scale_v)

            # helper: read depth (meters) at integer pixel coords using either original_depth_frame or numpy
            def read_depth_m(u_px, v_px):
                # bounds
                if u_px < 0 or u_px >= w_orig or v_px < 0 or v_px >= h_orig:
                    return None
                if original_depth_frame is not None:
                    d = self.camera_manager.get_average_depth(
                        original_depth_frame, (u_px, v_px), radius=1)
                    if d is None or d <= 0:
                        return None
                    return float(d)
                else:
                    val = depth_image[v_px, u_px]
                    if val <= 0:
                        return None
                    # decide units: if >10 assume mm
                    if val > 10:
                        return float(val) / 1000.0
                    else:
                        return float(val)

            # Create a list of candidate pixels in annulus (in original pixel coords)
            candidates = []
            # sample uniformly in annulus in resized pixel space then map to orig coords
            for _ in range(sample_count):
                # sample radius between inner and outer
                r = math.sqrt(random.uniform(
                    inner_radius_px**2, outer_radius_px**2))
                theta = random.uniform(0, 2 * math.pi)
                u_r = int(center_u_resized + r * math.cos(theta))
                v_r = int(center_v_resized + r * math.sin(theta))
                # map to original
                u_o = int(u_r * scale_u)
                v_o = int(v_r * scale_v)
                d_m = read_depth_m(u_o, v_o)
                if d_m is not None:
                    candidates.append((u_o, v_o, d_m))

            # Need enough samples
            if len(candidates) < 30:
                # fallback: enlarge region and try median
                pts = []
                ws = fallback_median_window
                for vv in range(center_v - ws, center_v + ws + 1):
                    for uu in range(center_u - ws, center_u + ws + 1):
                        d = read_depth_m(uu, vv)
                        if d is not None:
                            pts.append(d)
                if len(pts) == 0:
                    logging.warning(
                        "No usable depth pixels for fallback median estimate")
                    return 0.0
                median_depth = float(np.median(pts))
                # compute 3D grasp point
                gx, gy, gz = self.camera_manager.pixel_to_3d(
                    center_u, center_v, median_depth)
                # fallback: report vertical distance along camera z (approx)
                # but better to return gz (height above camera) — user seems to want height above table => approximate as gz - median_depth? ambiguous
                # We'll return distance from grasp point to "table" approximated as median_depth along camera ray:
                # return table Z in meters; but prefer returning grasp height so we'll compute below
                return max(0.0, median_depth - (gz)) if False else median_depth

            # Convert candidate pixels to 3D points (camera frame)
            points = []
            for (u_px, v_px, d_m) in candidates:
                x, y, z = self.camera_manager.pixel_to_3d(u_px, v_px, d_m)
                # ensure valid
                if not (np.isfinite(x) and np.isfinite(y) and np.isfinite(z)):
                    continue
                points.append((x, y, z))
            points = np.array(points)
            if points.shape[0] < 30:
                # fallback median as above
                pts = []
                ws = fallback_median_window
                for vv in range(center_v - ws, center_v + ws + 1):
                    for uu in range(center_u - ws, center_u + ws + 1):
                        d = read_depth_m(uu, vv)
                        if d is not None:
                            pts.append(d)
                if len(pts) == 0:
                    logging.warning(
                        "No usable depth pixels for fallback median estimate")
                    return 0.0
                median_depth = float(np.median(pts))
                gx, gy, gz = self.camera_manager.pixel_to_3d(
                    center_u, center_v, median_depth)
                # here we estimate table plane as perpendicular to camera Z (simple fallback)
                # height above table = gz - median_depth_in_camera_z? best approximate:
                # but when camera_pixel_to_3d returns z as distance from camera along optical axis,
                # the approximate perpendicular distance to a horizontal table is gz - median_depth (if gz and median_depth are same axis)
                # safer: return gz - median_depth
                return max(0.0, gz - median_depth)

            # RANSAC plane fit to points
            best_plane = None
            best_inliers = 0
            best_inlier_idxs = None

            P = points  # Nx3 array

            def fit_plane_from_three(p1, p2, p3):
                # plane through p1, p2, p3 -> normal = (p2-p1) x (p3-p1)
                v1 = p2 - p1
                v2 = p3 - p1
                n = np.cross(v1, v2)
                norm = np.linalg.norm(n)
                if norm < 1e-6:
                    return None
                n = n / norm
                # plane equation: n . (X - p1) = 0 -> n.x * x + n.y * y + n.z * z + d = 0
                d = -np.dot(n, p1)
                return (n[0], n[1], n[2], d)

            for _ in range(ransac_iters):
                # pick 3 distinct random indices
                idxs = np.random.choice(P.shape[0], 3, replace=False)
                p1, p2, p3 = P[idxs[0]], P[idxs[1]], P[idxs[2]]
                plane = fit_plane_from_three(p1, p2, p3)
                if plane is None:
                    continue
                a, b, c, d = plane
                # compute point-to-plane absolute distances
                numer = np.abs(a * P[:, 0] + b * P[:, 1] + c * P[:, 2] + d)
                denom = math.sqrt(a * a + b * b + c * c)
                dists = numer / (denom + 1e-12)
                inliers = np.sum(dists < ransac_inlier_thresh_m)
                if inliers > best_inliers:
                    best_inliers = int(inliers)
                    best_plane = plane
                    best_inlier_idxs = np.where(
                        dists < ransac_inlier_thresh_m)[0]

            if best_plane is None or best_inliers < min_inliers_for_plane:
                # plane fit failed — fallback: median of lowest depths in larger patch
                pts = []
                ws = fallback_median_window
                for vv in range(center_v - ws, center_v + ws + 1):
                    for uu in range(center_u - ws, center_u + ws + 1):
                        d = read_depth_m(uu, vv)
                        if d is not None:
                            pts.append(d)
                if len(pts) == 0:
                    logging.warning(
                        "No usable depth pixels for fallback median estimate after RANSAC failure")
                    return 0.0
                # choose robust estimate of table depth: median of lower half of depths (to prefer table floor)
                pts = np.array(pts)
                pts_sorted = np.sort(pts)
                take = max(1, int(len(pts_sorted) * 0.35))
                table_depth_m = float(np.median(pts_sorted[:take]))
                # convert center grasp pixel to 3D
                grasp_depth = read_depth_m(center_u, center_v)
                if grasp_depth is None:
                    # if grasp depth missing, use table_depth (so height=0)
                    return 0.0
                gx, gy, gz = self.camera_manager.pixel_to_3d(
                    center_u, center_v, grasp_depth)
                # approximate perpendicular distance along camera z axis
                # Use difference between grasp depth (distance along camera z) and table_depth as fallback
                return max(0.0, grasp_depth - table_depth_m)

            # Use best_plane to compute perpendicular distance from grasp 3D point to plane
            a, b, c, d = best_plane

            # get the grasp point depth value in meters (use average around center for stability)
            # try to read average depth with camera_manager.get_average_depth if original_depth_frame present
            if original_depth_frame is not None:
                # ask camera manager for a small median depth around the center pixel
                g_depth = self.camera_manager.get_average_depth(
                    original_depth_frame, (center_u, center_v), radius=4)
                grasp_depth_m = float(g_depth) if (
                    g_depth is not None and g_depth > 0) else None
            else:
                raw_val = depth_image[center_v, center_u]
                if raw_val <= 0:
                    grasp_depth_m = None
                elif raw_val > 10:
                    grasp_depth_m = float(raw_val) / 1000.0
                else:
                    grasp_depth_m = float(raw_val)

            if grasp_depth_m is None:
                # if the grasp pixel has no depth, approximate grasp point by projecting camera ray with table plane intersection (skip)
                # fallback to using the mean of inlier points for point location
                if best_inlier_idxs is not None and len(best_inlier_idxs) > 0:
                    inlier_pts = P[best_inlier_idxs]
                    # approximate grasp xyz as mean inlier (not perfect but a fallback)
                    gx, gy, gz = float(np.mean(inlier_pts[:, 0])), float(
                        np.mean(inlier_pts[:, 1])), float(np.mean(inlier_pts[:, 2]))
                else:
                    return 0.0
            else:
                gx, gy, gz = self.camera_manager.pixel_to_3d(
                    center_u, center_v, grasp_depth_m)

            # compute perpendicular distance
            numer = abs(a * gx + b * gy + c * gz + d)
            denom = math.sqrt(a * a + b * b + c * c) + 1e-12
            distance_m = numer / denom

            # clamp non-negative
            distance_m = max(0.0, float(distance_m))

            return distance_m

        except Exception as e:
            logging.error(
                f"Failed to compute grasp height (plane method): {e}")
            return 0.0

    # def _compute_grasp_height(self, depth_image: np.ndarray, grasp_2d: Dict) -> float:
    #     """
    #     Compute approximate grasp height directly from depth at the grasp center pixel.
    #     This replaces table-reference logic with a simpler single-frame approach.
    #     """
    #     try:
    #         h_orig, w_orig = depth_image.shape
    #         scale_u = w_orig / 300.0
    #         scale_v = h_orig / 300.0

    #         # Scale grasp center back to original depth resolution
    #         center_u, center_v = grasp_2d["center"]
    #         center_u_scaled = int(center_u * scale_u)
    #         center_v_scaled = int(center_v * scale_v)

    #         # Bounds check
    #         if center_u_scaled < 0 or center_u_scaled >= w_orig \
    #            or center_v_scaled < 0 or center_v_scaled >= h_orig:
    #             logger.warning("Grasp center is outside depth frame bounds")
    #             return 0.0

    #         # Extract raw depth at grasp center
    #         depth_value = depth_image[center_v_scaled, center_u_scaled]

    #         if depth_value <= 0:
    #             logger.warning("Invalid depth at grasp center")
    #             return 0.0

    #         # Convert depth value to meters
    #         if hasattr(depth_image, "get_units"):  # if it's a RealSense depth frame
    #             depth_meters = depth_value * depth_image.get_units()
    #         else:
    #             # Assume depth image is in millimeters
    #             depth_meters = depth_value / 1000.0

    #         return float(depth_meters)

    #     except Exception as e:
    #         logger.error(f"Failed to compute grasp height: {e}")
    #         return 0.0

    def _store_grasp_result(self, grasp_result: Dict):
        """
        Store grasp result in telemetry.
        Note: Does NOT send movement commands - that's handled by the sequencer states.
        """
        try:
            # Store grasp poses and height in telemetry for later use by sequencer
            grasp_pose_base = grasp_result['grasp_pose_base']
            grasp_height = grasp_result.get('grasp_height', 0.0)

            # Store grasp height in telemetry
            self.telemetry.update_grasp_height(grasp_height)

            # Store grasp height as pickup_height_offset for use in placement sequencer (seq 2)
            # This ensures the object is placed at the same height it was picked up from
            self.telemetry.set_pickup_height_offset(grasp_height)

            # Store the generated grasp pose and approach pose in telemetry
            # These will be used by MoveToState in the sequencer
            self.telemetry.set_generated_grasp_pose(grasp_pose_base)

            # Calculate approach pose (offset above grasp)
            approach_pose = grasp_pose_base.copy()
            approach_offset = GRASP_DETECTION_CONFIG.get(
                'approach_height_offset', 50.0) / 1000.0  # Convert mm to m
            approach_pose[2] += approach_offset
            self.telemetry.set_generated_approach_pose(approach_pose)

            logger.info(
                f"Grasp poses stored in telemetry - Grasp: {grasp_pose_base[:3]}, Approach: {approach_pose[:3]}, height: {grasp_height:.3f}m")
            logger.info(
                f"Pickup height offset set to {grasp_height:.3f}m for placement sequencer")

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

            return self.infer(depth_array, depth_frame)

        except Exception as e:
            logger.error(f"Failed to process depth frame: {e}")
            return None

    def cleanup(self):
        """Clean up resources and free memory."""
        try:
            if hasattr(self, 'model'):
                del self.model
            logger.info("GGCNN2 module cleaned up")
        except Exception as e:
            logger.error(f"Error during cleanup: {e}")

    # ----------------------------
    # Debug Visualization Methods
    # ----------------------------
    def _visualize_input_frame(self, depth_image: np.ndarray, title: str = "GG-CNN2 Input Frame"):
        """
        Display the processed depth image that is fed into the GG-CNN2 network.

        Args:
            depth_image: Original depth image
            title: Window title for the display
        """
        if not DEBUG_MODE:
            return

        try:
            # Create a copy for visualization
            vis_image = depth_image.copy()

            # Normalize for display (convert to 8-bit)
            if vis_image.dtype != np.uint8:
                # Handle NaN values
                vis_image = np.nan_to_num(vis_image, nan=0.0)

                # Normalize to 0-255 range
                if vis_image.max() > vis_image.min():
                    vis_image = ((vis_image - vis_image.min()) /
                                 (vis_image.max() - vis_image.min()) * 255).astype(np.uint8)
                else:
                    vis_image = np.zeros_like(vis_image, dtype=np.uint8)

            # Convert to color for better visualization
            vis_image_color = cv2.applyColorMap(vis_image, cv2.COLORMAP_JET)

            # Add title text
            cv2.putText(vis_image_color, title, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            # Display the image
            cv2.imshow(title, vis_image_color)
            cv2.waitKey(1)  # Non-blocking wait

            logger.debug(f"Displayed input frame: {title}")

        except Exception as e:
            logger.error(f"Failed to visualize input frame: {e}")

    def _visualize_grasp_output(self, depth_image: np.ndarray, grasp_2d: Dict,
                                title: str = "GG-CNN2 Grasp Output"):
        """
        Display the depth image with the best grasp pose overlaid.

        Args:
            depth_image: Original depth image
            grasp_2d: 2D grasp parameters (center, angle, width, quality)
            title: Window title for the display
        """
        if not DEBUG_MODE:
            return

        try:
            # Create a copy for visualization
            vis_image = depth_image.copy()

            # Normalize for display (convert to 8-bit)
            if vis_image.dtype != np.uint8:
                # Handle NaN values
                vis_image = np.nan_to_num(vis_image, nan=0.0)

                # Normalize to 0-255 range
                if vis_image.max() > vis_image.min():
                    vis_image = ((vis_image - vis_image.min()) /
                                 (vis_image.max() - vis_image.min()) * 255).astype(np.uint8)
                else:
                    vis_image = np.zeros_like(vis_image, dtype=np.uint8)

            # Convert to color for better visualization
            vis_image_color = cv2.applyColorMap(vis_image, cv2.COLORMAP_JET)

            # Scale grasp coordinates back to original image size
            h_orig, w_orig = depth_image.shape
            scale_u = w_orig / 300.0
            scale_v = h_orig / 300.0

            center_u, center_v = grasp_2d["center"]
            center_u_scaled = int(center_u * scale_u)
            center_v_scaled = int(center_v * scale_v)

            # Scale grasp width
            grasp_width = grasp_2d["width"] * scale_u
            grasp_angle = grasp_2d["angle"]
            grasp_quality = grasp_2d["quality"]

            # Draw grasp center point
            cv2.circle(vis_image_color, (center_u_scaled,
                       center_v_scaled), 5, (0, 255, 0), -1)

            # Draw grasp rectangle
            # Calculate rectangle corners based on angle and width
            half_width = grasp_width / 2

            # Calculate the four corners of the grasp rectangle
            cos_angle = np.cos(grasp_angle)
            sin_angle = np.sin(grasp_angle)

            # Rectangle corners relative to center
            corners = np.array([
                [-half_width, -10],  # Top-left
                [half_width, -10],   # Top-right
                [half_width, 10],    # Bottom-right
                [-half_width, 10]    # Bottom-left
            ])

            # Rotate corners
            rotation_matrix = np.array([
                [cos_angle, -sin_angle],
                [sin_angle, cos_angle]
            ])

            rotated_corners = np.dot(corners, rotation_matrix.T)

            # Translate to grasp center
            rotated_corners[:, 0] += center_u_scaled
            rotated_corners[:, 1] += center_v_scaled

            # Draw the grasp rectangle
            pts = rotated_corners.astype(np.int32)
            cv2.polylines(vis_image_color, [pts], True, (0, 255, 0), 2)

            # Draw approach direction (arrow)
            arrow_length = 30
            arrow_end_u = int(center_u_scaled + arrow_length * cos_angle)
            arrow_end_v = int(center_v_scaled + arrow_length * sin_angle)
            cv2.arrowedLine(vis_image_color,
                            (center_u_scaled, center_v_scaled),
                            (arrow_end_u, arrow_end_v),
                            (255, 0, 0), 3, tipLength=0.3)

            # Add text information
            info_text = f"Quality: {grasp_quality:.3f}"
            cv2.putText(vis_image_color, info_text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            info_text2 = f"Angle: {np.degrees(grasp_angle):.1f}°"
            cv2.putText(vis_image_color, info_text2, (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            info_text3 = f"Width: {grasp_width:.1f}px"
            cv2.putText(vis_image_color, info_text3, (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # Display the image
            cv2.imshow(title, vis_image_color)
            cv2.waitKey(1)  # Non-blocking wait

            logger.debug(
                f"Displayed grasp output: {title}, Quality: {grasp_quality:.3f}")

        except Exception as e:
            logger.error(f"Failed to visualize grasp output: {e}")
