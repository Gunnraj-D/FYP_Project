"""
Main grasp detector module - orchestrates the grasp detection pipeline.

This is a lightweight wrapper that coordinates:
- Preprocessing (grasp_preprocessing.py)
- Network inference (ggcnn2.py or grconvnet.py)
- Postprocessing (grasp_postprocessing.py)
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
from kinematics.collision_aware_kinematics_solver import CollisionAwareKinematicsSolver

# Import network models
from object_detection.ggcnn2 import GGCNN2
from object_detection.grconvnet import GRConvNet

# Import refactored modules
from object_detection.grasp_preprocessing import GraspPreprocessor
from object_detection.grasp_postprocessing import GraspPostprocessor, TemporalAngleFilter
from object_detection.grasp_transforms import GraspTransformer
from object_detection.grasp_visualization import GraspVisualizer

# Config
from config.config import (
    GRASP_DETECTION_CONFIG, GRASP_EXECUTION_CONFIG, DEBUG_MODE,
    GRASP_MODEL_TYPE, GRCONVNET_CONFIG, GGCNN2_MODEL_PATH, GRCONVNET_MODEL_PATH
)

logger = logging.getLogger(__name__)


class GraspDetector:
    """
    Main grasp detector - coordinates the full grasp detection pipeline.

    Pipeline:
    1. Preprocess: depth/RGB-D → tensor
    2. Inference: network → quality, angle, width maps
    3. Postprocess: select best candidate + temporal filtering
    4. Transform: 2D → 3D → base frame → joint angles
    5. Visualize: show results (if debug mode)

    Supports both GGCNN2 (depth-only) and GR-ConvNet (RGB-D).
    """

    def __init__(self, model_path: str, telemetry: Telemetry, command_bus: CommandBus,
                 camera_manager: CameraManager, kinematics_solver: CollisionAwareKinematicsSolver):
        """
        Initialize grasp detector with all required components.

        Args:
            model_path: Path to model weights (overridden by config)
            telemetry: Telemetry store for robot state
            command_bus: Command bus for robot commands
            camera_manager: Camera manager for image acquisition
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

        # Initialize pipeline components
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

        logger.info(f"✅ {self.model_type.upper()} grasp detector initialized")
        logger.info(f"   Input: {'RGB-D (4ch)' if self.use_rgbd else 'Depth (1ch)'}, "
                    f"Size: {self.resize_size}x{self.resize_size}")

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

            logger.info(f"✅ Grasp detected: quality={grasp_result['quality']:.3f}, "
                        f"angle={np.degrees(grasp_result.get('angle', 0)):.1f}°")

            return grasp_result

        except Exception as e:
            logger.error(f"Grasp detection failed: {e}", exc_info=True)
            return None

    def infer(self, depth_image: np.ndarray, original_depth_frame=None,
              color_image: Optional[np.ndarray] = None) -> Optional[Dict]:
        """
        Run full inference pipeline.

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

            # 1. Preprocess
            input_tensor = self.preprocessor.preprocess(
                depth_image, color_image)

            # 2. Network inference
            with torch.no_grad():
                pos, cos, sin, width = self.model(input_tensor)

            # 3. Decode outputs
            q_img = torch.sigmoid(pos)  # Quality
            ang_img = 0.5 * torch.atan2(sin, cos)  # Angle [-π/2, π/2]

            # Width decoding (model-specific)
            if self.model_type == 'grconvnet':
                width_img = F.relu(width) * 150.0  # GR-ConvNet: scale by 150
            else:
                width_img = F.relu(width)  # GGCNN2: direct pixel width

            # 4. Postprocess: select best candidate
            grasp_2d = self.postprocessor.postprocess(
                q_img, ang_img, width_img, depth_image, original_depth_frame
            )

            if grasp_2d is None:
                logger.warning("No valid grasp candidate found")
                return None

            # Log selected angle
            pred_angle_deg = np.degrees(grasp_2d['angle'])
            angle_offset_deg = np.degrees(
                GRASP_DETECTION_CONFIG.get('grasp_angle_offset_rad', 0.0))

            logger.info(
                f"📐 Network predicted angle: {pred_angle_deg:.1f}° (contact line)")
            if angle_offset_deg != 0.0:
                jaw_axis_deg = pred_angle_deg + angle_offset_deg
                logger.info(f"🔄 Applying {angle_offset_deg:.1f}° offset → "
                            f"Jaw axis: {jaw_axis_deg:.1f}°")

            # Visualize grasp output
            self.visualizer.visualize_grasp_output(
                depth_image, grasp_2d, "Grasp Output")

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

        # Return complete grasp result
        return {
            'joint_angles': joint_angles,
            'pose': base_pose,
            'quality': grasp_2d['quality'],
            'angle': grasp_2d['angle'],
            'width': grasp_2d.get('width_m', grasp_2d['width']),
            'center': grasp_2d['center'],
            'depth_m': grasp_2d.get('depth_m')
        }

    def cleanup(self):
        """Clean up resources (e.g., visualization windows)."""
        self.visualizer.cleanup()
        logger.info("Grasp detector cleanup complete")


# Backward compatibility: Alias for existing code
GGcnn2Module = GraspDetector
