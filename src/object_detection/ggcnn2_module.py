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
from kinematics.kinematics_solver import InverseKinematicsSolver
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

            # Transform to robot base frame
            grasp_pose_base = self._transform_to_base_frame(grasp_pose_camera)

            if grasp_pose_base is None:
                logger.warning("Failed to transform grasp pose to base frame")
                return None

            logger.info(f"Transformed base pose: {grasp_pose_base}")

            # Convert to joint angles
            joint_angles = self._pose_to_joint_angles(grasp_pose_base)

            if joint_angles is None:
                logger.warning("Failed to convert grasp pose to joint angles")
                return None

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
                logger.warning("Invalid depth at grasp center")
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

            # Simplified orientation transformation
            # For now, just use the camera orientation directly
            # This avoids complex rotation composition that might be causing issues
            grasp_rpy_camera = camera_pose[3:6]

            # Use the camera orientation directly (simplified approach)
            base_pose = base_position.tolist() + grasp_rpy_camera
            logger.debug(
                f"Correctly transformed base frame grasp pose: {base_pose}")
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

    def _compute_grasp_height(self, depth_image: np.ndarray, grasp_2d: Dict) -> float:
        """
        Compute approximate grasp height directly from depth at the grasp center pixel.
        This replaces table-reference logic with a simpler single-frame approach.
        """
        try:
            h_orig, w_orig = depth_image.shape
            scale_u = w_orig / 300.0
            scale_v = h_orig / 300.0

            # Scale grasp center back to original depth resolution
            center_u, center_v = grasp_2d["center"]
            center_u_scaled = int(center_u * scale_u)
            center_v_scaled = int(center_v * scale_v)

            # Bounds check
            if center_u_scaled < 0 or center_u_scaled >= w_orig \
               or center_v_scaled < 0 or center_v_scaled >= h_orig:
                logger.warning("Grasp center is outside depth frame bounds")
                return 0.0

            # Extract raw depth at grasp center
            depth_value = depth_image[center_v_scaled, center_u_scaled]

            if depth_value <= 0:
                logger.warning("Invalid depth at grasp center")
                return 0.0

            # Convert depth value to meters
            if hasattr(depth_image, "get_units"):  # if it's a RealSense depth frame
                depth_meters = depth_value * depth_image.get_units()
            else:
                # Assume depth image is in millimeters
                depth_meters = depth_value / 1000.0

            return float(depth_meters)

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
