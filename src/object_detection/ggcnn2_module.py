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
from kinematics.collision_aware_kinematics_solver import CollisionAwareKinematicsSolver
from kinematics.kinematics_solver import get_facing_down_orientation
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
                 camera_manager: CameraManager, kinematics_solver: CollisionAwareKinematicsSolver):
        self.telemetry = telemetry
        self.command_bus = command_bus
        self.camera_manager = camera_manager
        self.kinematics_solver = kinematics_solver
        self.device = torch.device(
            "cuda" if torch.cuda.is_available() else "cpu")

        # Configuration parameters
        self.resize_size = 300  # Standard GGCNN2 input size
        self.depth_sample_radius = int(
            GRASP_DETECTION_CONFIG.get('depth_sample_radius', 5))

        # Init model
        self.model = GGCNN2()
        state_dict = torch.load(model_path, map_location=self.device)
        self.model.load_state_dict(state_dict)
        self.model.to(self.device).eval()

        logger.info(f"GGCNN2 model loaded from {model_path} on {self.device}")
        logger.info(
            f"Depth sample radius: {self.depth_sample_radius}, resize: {self.resize_size}x{self.resize_size}")

    # ----------------------------
    # Preprocessing
    # ----------------------------
    def preprocess(self, depth_image: np.ndarray) -> torch.Tensor:
        """
        Preprocess depth image for GGCNN2 inference.

        Args:
            depth_image: Depth image in meters

        Returns:
            Preprocessed depth tensor normalized to [0,1]
        """
        # Crop to square to avoid aspect ratio distortion
        h, w = depth_image.shape
        min_dim = min(h, w)
        start_h = (h - min_dim) // 2
        start_w = (w - min_dim) // 2
        depth = depth_image[start_h:start_h+min_dim, start_w:start_w+min_dim]

        depth = cv2.resize(depth, (300, 300))

        # Fixed normalization range for tabletop scenarios
        # All units in METERS (0.2m to 1.2m typical for tabletop grasping)
        depth = np.clip(depth, 0.2, 1.2)      # meters
        depth = (depth - 0.2) / (1.0)         # normalize [0,1]

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
            grasp_2d = self.postprocess(
                q_img, ang_img, width_img, depth_image, original_depth_frame)

            if grasp_2d is None:
                logger.warning("No valid grasp found in image")
                # Debug: Log quality statistics
                q_np = q_img.squeeze().cpu().numpy()
                logger.warning(
                    f"Quality stats - Max: {q_np.max():.3f}, Mean: {q_np.mean():.3f}, Min: {q_np.min():.3f}")
                logger.warning(
                    f"Depth image stats - Max: {depth_image.max():.3f}, Mean: {depth_image.mean():.3f}, Min: {depth_image.min():.3f}")
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
            # GGCNN2 outputs angle in range [-π/2, π/2]
            grasp_angle_rad = grasp_2d["angle"]

            # Apply configurable angle offset (for gripper finger axis alignment)
            # This compensates for:
            # 1. Gripper finger axis orientation vs. GGCNN2 angle convention
            # 2. Camera mounting orientation effects
            # 3. Any systematic rotation bias in the training data
            # Must be calibrated empirically during real-world testing
            angle_offset_rad = GRASP_DETECTION_CONFIG.get(
                'grasp_angle_offset_rad', 0.0)
            grasp_angle_rad += angle_offset_rad

            # Create rotation matrices in BASE FRAME coordinates:
            from scipy.spatial.transform import Rotation as R

            # R_z: Rotation around base frame Z-axis to align gripper with grasp angle
            # This orients the gripper to match the object's orientation on the table
            R_z = R.from_euler('z', grasp_angle_rad).as_matrix()

            # R_down: Rotation to point gripper downward (180° around X-axis)
            # This makes the gripper approach from above
            R_down = get_facing_down_orientation()

            # IMPORTANT: Rotation composition order (configurable)
            # 'down_then_z': R_down @ R_z = align with object orientation, then point down (default)
            # 'z_then_down': R_z @ R_down = point down, then rotate in local frame
            compose_order = GRASP_DETECTION_CONFIG.get(
                'compose_order', 'down_then_z')
            if compose_order == 'down_then_z':
                # Recommended: align orientation in base frame, then point down
                target_orientation_matrix = R_down @ R_z
            else:
                # Alternative: point down first, then rotate in downward-pointing frame
                target_orientation_matrix = R_z @ R_down

            # Log the resulting orientation for verification
            result_rpy = R.from_matrix(
                target_orientation_matrix).as_euler('xyz')
            logger.info(
                f"Grasp orientation - angle from GGCNN2: {np.degrees(grasp_angle_rad):.1f}°, "
                f"resulting RPY: [{np.degrees(result_rpy[0]):.1f}°, {np.degrees(result_rpy[1]):.1f}°, {np.degrees(result_rpy[2]):.1f}°]")

            # 4. Centralized Z clamp before IK
            # Ensure Z coordinate is non-negative (above workspace floor)
            if target_position[2] < 0.0:
                logger.warning(
                    f"Clamping target Z from {target_position[2]:.3f}m to 0.0m before IK")
                target_position[2] = 0.0

            # 5. Solve IK with the decoupled position and orientation
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
            All depths are expected to be in meters.
            Includes nearest-valid fallback to avoid discarding grasps over sparse holes.
            """
            # map resized coords back to original depth image size
            if depth_image is None and original_depth_frame is None:
                return None

            if original_depth_frame is not None:
                # CameraManager.get_average_depth expects depth_frame-like, so use it directly
                # convert to int pixel coordinates in original frame
                h_orig = original_depth_frame.get_height()
                w_orig = original_depth_frame.get_width()
                scale_u = w_orig / 300.0
                scale_v = h_orig / 300.0
                su = int(u_resized * scale_u)
                sv = int(v_resized * scale_v)

                # Try primary location with unified radius
                d = self.camera_manager.get_average_depth(
                    original_depth_frame, (su, sv), radius=self.depth_sample_radius)
                if d is not None and d > 0:
                    return float(d)

                # Nearest-valid fallback (3x3 search)
                for dv in (-1, 0, 1):
                    for du in (-1, 0, 1):
                        if du == 0 and dv == 0:
                            continue
                        uu, vv = su + du, sv + dv
                        if 0 <= uu < w_orig and 0 <= vv < h_orig:
                            d2 = self.camera_manager.get_average_depth(
                                original_depth_frame, (uu, vv), radius=self.depth_sample_radius)
                            if d2 is not None and d2 > 0:
                                return float(d2)
                return None
            else:
                # depth_image is numpy array in meters
                depth_np = depth_image
                h_orig, w_orig = depth_np.shape
                scale_u = w_orig / 300.0
                scale_v = h_orig / 300.0
                su = int(u_resized * scale_u)
                sv = int(v_resized * scale_v)

                # Try primary location
                if 0 <= su < w_orig and 0 <= sv < h_orig and depth_np[sv, su] > 0:
                    return float(depth_np[sv, su])

                # Nearest-valid fallback (3x3 search)
                for dv in (-1, 0, 1):
                    for du in (-1, 0, 1):
                        if du == 0 and dv == 0:
                            continue
                        uu, vv = su + du, sv + dv
                        if 0 <= uu < w_orig and 0 <= vv < h_orig and depth_np[vv, uu] > 0:
                            return float(depth_np[vv, uu])
                return None

        # Get intrinsics for width->meters conversion
        # IMPORTANT: Since we use ALIGNED depth frames (aligned to color), we must use
        # aligned_color_intrinsics because the aligned depth has the same resolution and
        # coordinate frame as the color stream
        aligned_intr = getattr(self.camera_manager,
                               "aligned_color_intrinsics", None)
        if aligned_intr is None:
            # Fallback to color_intrinsics for backward compatibility
            aligned_intr = getattr(self.camera_manager,
                                   "color_intrinsics", None)
        fx = aligned_intr.fx if (
            aligned_intr is not None and hasattr(aligned_intr, "fx")) else None
        depth_img_width = aligned_intr.width if (
            aligned_intr is not None and hasattr(aligned_intr, "width")) else None

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
                            # patch is in meters
                            depth_var = float(np.std(patch))
                            edge_penalty = min(1.0, depth_var / 0.02)
                    except Exception:
                        edge_penalty = 0.0
            else:
                # if depth unknown, penalize slightly (safer)
                edge_penalty = 0.2

            # width penalty: convert pixel width -> meters (if fx & depth known)
            width_pixels = float(width_np[r, c])  # width in resized pixels
            # convert to original pixels using DEPTH image dimensions
            width_m = None

            # Assertion: depth intrinsics should be complete or both None
            assert (fx is None) == (depth_img_width is None) or (fx is not None and depth_img_width is not None), \
                "Depth intrinsics incomplete: fx and width must both be set or both None"

            if fx is not None and depth_m is not None and depth_img_width is not None and depth_m > 0:
                # Convert pixel width to meters using depth intrinsics
                # Formula: width_m = width_pixels_resized * (depth_m / fx_resized)
                # where fx_resized = fx_depth * (300 / depth_image_width)
                # This accounts for the resize from original depth resolution to 300x300
                fx_resized = fx * (300.0 / depth_img_width)
                # Protect against division by zero
                width_m = (width_pixels * depth_m) / max(fx_resized, 1e-6)
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
                # Small penalty if unknown to avoid over-pruning good grasps
                width_penalty = 0.05

            # Combine into a final score: higher better
            # weights (tunable)
            score = (
                1.0 * quality
                - 0.6 * dist
                - 0.9 * edge_penalty
                - 0.7 * width_penalty
            )

            # CRITICAL: Only consider candidates with valid depth data
            if depth_m is not None and depth_m > 0:
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
            else:
                # Skip candidates with invalid depth
                logger.debug(
                    f"Skipping candidate at ({c}, {r}) - invalid depth: {depth_m}")

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
            [x, y, z, roll, pitch, yaw] in camera frame (meters, radians)
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
                depth_source, (int(center_u_scaled), int(
                    center_v_scaled)), self.depth_sample_radius
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
            [x, y, z, roll, pitch, yaw] in base frame (meters, radians)
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
            logger.info(f"Camera position: {camera_position}")
            logger.info(f"TCP matrix Z component: {tcp_matrix[2, 3]:.3f}m")

            base_position = transform_camera_to_base(
                camera_position, tcp_matrix)

            logger.info(f"Base position after transformation: {base_position}")

            # Safety check: Ensure Z coordinate is reasonable (above workspace floor)
            if base_position[2] < 0.0:
                logger.warning(
                    f"Grasp position below workspace floor: Z={base_position[2]:.3f}m, clamping to 0.0m")
                base_position[2] = 0.0
                logger.info(f"Base position after Z clamping: {base_position}")

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
            # Apply final safety check for Z coordinate
            pose_meters = [pose[0], pose[1],
                           pose[2], pose[3], pose[4], pose[5]]

            # CRITICAL: Ensure Z coordinate is above workspace floor
            if pose_meters[2] < 0.0:
                logger.warning(
                    f"Final pose Z below workspace floor: {pose_meters[2]:.3f}m, clamping to 0.0m")
                pose_meters[2] = 0.0

            logger.info(
                f"Final pose for IK: position={pose_meters[:3]}, orientation={pose_meters[3:6]}")
            logger.debug(
                f"Pose being passed to solve_pose: {pose_meters}, length: {len(pose_meters)}")
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
                depth_source, (center_u_scaled,
                               center_v_scaled), self.depth_sample_radius
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
                # IMPORTANT: Normals are direction vectors, so use rotation-only (no translation)
                R_tcp = tcp_matrix[:3, :3]
                n_base = (R_tcp @ n.reshape(3, 1)).reshape(3)
                n_base = n_base / (np.linalg.norm(n_base) + 1e-9)

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
                'approach_height_offset', 0.050)  # meters
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
            # Convert depth frame to numpy array IN METERS
            if hasattr(depth_frame, 'get_data'):
                # Get raw depth data
                depth_array = np.asanyarray(depth_frame.get_data())

                # CRITICAL: Convert from raw sensor units to meters
                # RealSense depth is typically in internal units that need scaling
                # depth_frame.get_units() returns the scale factor (usually 0.001 for mm->m)
                depth_units = depth_frame.get_units()
                depth_array = depth_array.astype(np.float32) * depth_units

                logger.debug(
                    f"Converted depth from raw units to meters (scale: {depth_units})")
                logger.debug(
                    f"Depth range: {depth_array[depth_array > 0].min():.3f}m to {depth_array[depth_array > 0].max():.3f}m")
            else:
                # Assume already a numpy array in meters
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
