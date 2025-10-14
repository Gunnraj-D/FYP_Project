"""
ADVANCED grasp detector module - orchestrates the grasp detection pipeline.

Major improvements from legacy:
1. Uses advanced preprocessing (per-channel RGB, percentile depth)
2. Uses advanced postprocessing (anti-tip fixes, multi-factor scoring)
3. Passes depth map for local depth estimation
4. Camera intrinsics integration
5. PCA-based angle correction
6. Sophisticated grasp selection

This is a lightweight wrapper that coordinates:
- Preprocessing (grasp_preprocessing.py) - IMPROVED
- Network inference (ggcnn2.py or grconvnet.py)
- Postprocessing (grasp_postprocessing.py) - COMPLETELY REWRITTEN
- Coordinate transformations (grasp_transforms.py)
- Visualization (grasp_visualization.py)
"""

import torch
import torch.nn.functional as F
import numpy as np
import logging
from typing import Optional, Dict
from pathlib import Path

from control.telemetry_store import Telemetry
from control.command_bus import CommandBus
from camera_management.camera_manager import CameraManager
from kinematics.kinematics_solver import InverseKinematicsSolver

# Import network models
from object_detection.ggcnn2 import GGCNN2
from object_detection.grconvnet import GRConvNet

# Import refactored modules (ADVANCED versions)
from object_detection.grasp_preprocessing import GraspPreprocessor
from object_detection.grasp_postprocessing import GraspPostprocessor, TemporalAngleFilter
from object_detection.grasp_transforms import GraspTransformer
from object_detection.grasp_visualization import GraspVisualizer

# Config
from config import (
    GRASP_DETECTION_CONFIG, GRASP_EXECUTION_CONFIG, DEBUG_MODE,
    GRASP_MODEL_TYPE, GRCONVNET_CONFIG, GGCNN2_MODEL_PATH, GRCONVNET_MODEL_PATH
)

logger = logging.getLogger(__name__)


class GraspDetector:
    """
    ADVANCED grasp detector with anti-tip fixes.

    Pipeline:
    1. Preprocess: depth/RGB-D → tensor (+ depth map in meters, median depth)
    2. Inference: network → quality, angle, width maps
    3. Postprocess: ADVANCED candidate selection with:
       - Local depth estimation
       - Object mask overlap
       - Border penalties
       - PCA angle correction
       - Multi-factor scoring
    4. Transform: 2D → 3D → base frame → joint angles
    5. Visualize: show results (if debug mode)

    Supports both GGCNN2 (depth-only) and GR-ConvNet (RGB-D).

    CALIBRATED FOR SCREWDRIVERS (30mm handle):
    - Width multiplier: 95.0 (set in config)
    - Min overlap: 0.25 (set in config)
    - PCA angle correction: Enabled
    """

    def __init__(self, model_path: str, telemetry: Telemetry, command_bus: CommandBus,
                 camera_manager: CameraManager, kinematics_solver: InverseKinematicsSolver):
        """
        Initialize ADVANCED grasp detector.

        Args:
            model_path: Path to model weights (overridden by config)
            telemetry: Telemetry store for robot state
            command_bus: Command bus for robot commands
            camera_manager: Camera manager for image acquisition + intrinsics
            kinematics_solver: IK solver for joint angles
        """
        self.telemetry = telemetry
        self.command_bus = command_bus
        self.camera_manager = camera_manager
        self.kinematics_solver = kinematics_solver

        # Device selection
        self.device = torch.device(
            "cuda" if torch.cuda.is_available() else "cpu")
        logger.info(f"Using device: {self.device}")

        # Model configuration
        self.model_type = GRASP_MODEL_TYPE
        self.use_rgbd = (GRASP_MODEL_TYPE == 'grconvnet')

        if self.use_rgbd:
            self.resize_size = GRCONVNET_CONFIG.get('input_size', 300)
        else:
            self.resize_size = 300  # GGCNN2 standard

        # Initialize model
        self.model = self._load_model()

        # Initialize ADVANCED pipeline components
        self.preprocessor = GraspPreprocessor(
            model_type=self.model_type,
            resize_size=self.resize_size,
            device=self.device
        )

        self.temporal_filter = TemporalAngleFilter(
            enabled=GRASP_DETECTION_CONFIG.get(
                'temporal_filter_enabled', False),
            window_size=GRASP_DETECTION_CONFIG.get('temporal_window_size', 5),
            filter_type=GRASP_DETECTION_CONFIG.get(
                'temporal_filter_type', 'circular_mean'),
            ema_alpha=GRASP_DETECTION_CONFIG.get('temporal_ema_alpha', 0.3),
            outlier_threshold_deg=GRASP_DETECTION_CONFIG.get(
                'temporal_outlier_threshold_deg', 30)
        )

        self.postprocessor = GraspPostprocessor(
            camera_manager=camera_manager,
            temporal_filter=self.temporal_filter
        )

        self.transformer = GraspTransformer(
            camera_manager=camera_manager,
            kinematics_solver=kinematics_solver,
            telemetry=telemetry
        )

        self.visualizer = GraspVisualizer(enabled=DEBUG_MODE)

        logger.info(
            f"✅ ADVANCED {self.model_type.upper()} grasp detector initialized")
        logger.info(f"   Input: {'RGB-D (4ch, per-channel norm)' if self.use_rgbd else 'Depth (1ch)'}, "
                    f"Size: {self.resize_size}x{self.resize_size}")
        logger.info(
            f"   Anti-tip fixes: Local depth, NMS, Overlap, PCA angles, Multi-factor scoring")

    def _load_model(self) -> torch.nn.Module:
        """Load and initialize grasp detection model."""
        if self.model_type == 'grconvnet':
            model = GRConvNet(
                input_channels=GRCONVNET_CONFIG.get('input_channels', 4),
                channel_size=GRCONVNET_CONFIG.get('channel_size', 32),
                input_size=self.resize_size,
                dropout=GRCONVNET_CONFIG.get('use_dropout', False),
                dropout_prob=GRCONVNET_CONFIG.get('dropout_prob', 0.0)
            )
            model_path = str(GRCONVNET_MODEL_PATH)
        else:
            model = GGCNN2()
            model_path = str(GGCNN2_MODEL_PATH)

        # Load weights
        state_dict = torch.load(
            model_path, map_location=self.device, weights_only=False)
        model.load_state_dict(state_dict)
        model.to(self.device).eval()

        logger.info(f"📦 Model loaded from {Path(model_path).name}")
        return model

    def process_depth_frame(self, depth_frame, color_frame=None) -> Optional[Dict]:
        """
        Process depth (and optional color) frame to generate grasp pose.

        This is the main entry point for the grasp detection pipeline.

        Args:
            depth_frame: Depth frame (numpy array or RealSense frame)
            color_frame: Color frame (required for GR-ConvNet)

        Returns:
            Dict with grasp result:
            - 'joint_angles': Target joint angles (7 values)
            - 'pose': Base frame pose [x,y,z, r,p,y]
            - 'quality': Grasp quality score
            - 'angle': Grasp angle in radians
            - 'width': Grasp width in meters
            - 'object_overlap': Overlap with object mask [0,1]
            - 'border_distance': Distance from border [0,1]
            or None if no valid grasp found
        """
        try:
            # 1. Convert frames to numpy arrays
            depth_array, color_array = self._extract_frame_data(
                depth_frame, color_frame)

            # 2. Run inference
            grasp_result = self.infer(depth_array, depth_frame, color_array)

            if grasp_result is None:
                logger.warning("No valid grasp found")
                return None

            logger.info(f"✅ ADVANCED grasp detected: quality={grasp_result['quality']:.3f}, "
                        f"angle={np.degrees(grasp_result.get('angle', 0)):.1f}°, "
                        f"width={grasp_result.get('width_m', 0)*1000:.1f}mm, "
                        f"overlap={grasp_result.get('object_overlap', 0):.2f}")

            return grasp_result

        except Exception as e:
            logger.error(f"Grasp detection failed: {e}", exc_info=True)
            return None

    def infer(self, depth_image: np.ndarray, original_depth_frame=None,
              color_image: Optional[np.ndarray] = None) -> Optional[Dict]:
        """
        Run full ADVANCED inference pipeline.

        Args:
            depth_image: Depth array in meters (H, W)
            original_depth_frame: Optional RealSense frame for accurate depth sampling
            color_image: Optional color array in BGR (H, W, 3) - required for GR-ConvNet

        Returns:
            Dict with grasp parameters and joint angles, or None if no valid grasp
        """
        try:
            # Visualize input
            self.visualizer.visualize_input_frame(
                depth_image, f"{self.model_type.upper()} Input")

            # 1. ADVANCED Preprocess (returns depth map + median depth)
            input_tensor, depth_resized_m, median_depth_m = self.preprocessor.preprocess(
                depth_image, color_image)

            # 2. Network inference
            with torch.no_grad():
                pos, cos, sin, width = self.model(input_tensor)

            # 3. Decode outputs
            q_img = torch.sigmoid(pos)  # Quality
            ang_img = 0.5 * torch.atan2(sin, cos)  # Angle [-π/2, π/2]

            # Width decoding (CALIBRATED multiplier from config)
            if self.model_type == 'grconvnet':
                width_multiplier = GRASP_DETECTION_CONFIG.get(
                    'width_multiplier', 95.0)
                width_img = F.relu(width) * width_multiplier
                logger.debug(
                    f"Using calibrated width multiplier: {width_multiplier}")
            else:
                width_img = F.relu(width)  # GGCNN2: direct pixel width

            # 4. ADVANCED Postprocess: anti-tip candidate selection
            grasp_2d = self.postprocessor.postprocess(
                q_img, ang_img, width_img,
                depth_image=depth_resized_m,  # Pass depth map for local depth
                original_depth_frame=original_depth_frame
            )

            if grasp_2d is None:
                logger.warning(
                    "No valid grasp candidate found (advanced filtering)")
                return None

            # Log selected grasp with advanced metrics
            pred_angle_deg = np.degrees(grasp_2d['angle'])
            overlap = grasp_2d.get('object_overlap', 0.0)
            border = grasp_2d.get('border_distance', 0.0)

            logger.info(f"📐 Selected grasp: angle={pred_angle_deg:.1f}°, "
                        f"width={grasp_2d.get('width_m', 0)*1000:.1f}mm, "
                        f"overlap={overlap:.2f}, border={border:.2f}")

            # Visualize grasp output
            self.visualizer.visualize_grasp_output(
                depth_image, grasp_2d, "ADVANCED Grasp Output")

            # 5. Transform to 3D and base frame
            grasp_result = self._transform_grasp_to_base(
                grasp_2d, depth_image, original_depth_frame)

            return grasp_result

        except Exception as e:
            logger.error(f"Inference failed: {e}", exc_info=True)
            return None

    def _extract_frame_data(self, depth_frame, color_frame):
        """Extract numpy arrays from frames."""
        # Depth
        if isinstance(depth_frame, np.ndarray):
            depth_array = depth_frame
        else:
            depth_array = np.asanyarray(depth_frame.get_data())
            if depth_array.dtype != np.float32:
                depth_units = depth_frame.get_units()
                depth_array = depth_array.astype(np.float32) * depth_units

        # Color (if needed)
        color_array = None
        if self.use_rgbd and color_frame is not None:
            if isinstance(color_frame, np.ndarray):
                color_array = color_frame
            else:
                color_array = np.asanyarray(color_frame.get_data())

        return depth_array, color_array

    def _transform_grasp_to_base(self, grasp_2d: Dict, depth_image: np.ndarray,
                                 original_depth_frame) -> Optional[Dict]:
        """Transform grasp from 2D to base frame with joint angles."""
        # 2D → 3D camera frame
        camera_pose = self.transformer.grasp_2d_to_3d_pose(
            grasp_2d, depth_image, original_depth_frame
        )
        if camera_pose is None:
            logger.warning("Failed to convert grasp to 3D pose")
            return None

        # Camera frame → Base frame
        base_pose = self.transformer.transform_to_base_frame(camera_pose)
        if base_pose is None:
            logger.warning("Failed to transform grasp to base frame")
            return None

        # Base pose → Joint angles
        joint_angles = self.transformer.pose_to_joint_angles(base_pose)
        if joint_angles is None:
            logger.warning("Failed to solve IK for grasp pose")
            return None

        # Return complete grasp result with advanced metrics
        return {
            'joint_angles': joint_angles,
            'pose': base_pose,
            'quality': grasp_2d['quality'],
            'angle': grasp_2d['angle'],
            'width': grasp_2d.get('width_m', grasp_2d['width']),
            'center': grasp_2d['center'],
            'depth_m': grasp_2d.get('depth_m'),
            # Advanced metrics
            'object_overlap': grasp_2d.get('object_overlap', 0.0),
            'border_distance': grasp_2d.get('border_distance', 0.0),
            'width_mm': grasp_2d.get('width_m', 0) * 1000
        }

    def cleanup(self):
        """Clean up resources (e.g., visualization windows)."""
        self.visualizer.cleanup()
        logger.info("ADVANCED grasp detector cleanup complete")


# Backward compatibility: Alias for existing code
GGcnn2Module = GraspDetector
