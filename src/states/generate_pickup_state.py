"""
Generate Pickup State - Generates grasp poses for objects using GGCNN2 module.
This state handles the process of detecting objects and generating optimal grasp poses
that are stored in telemetry for subsequent pickup operations.
"""
import logging
import time
import numpy as np
from typing import Optional, Dict, List

from states.base_state import BaseState
from states.context import StateContext
from object_detection.ggcnn2_module import GGcnn2Module
from config.config import (
    GGCNN2_MODEL_PATH,
    GRASP_DETECTION_CONFIG,
    GRASP_EXECUTION_CONFIG
)

logger = logging.getLogger(__name__)


class GeneratePickupState(BaseState):
    """
    State for generating grasp poses using GGCNN2 object detection.

    This state:
    1. Captures depth images from camera
    2. Runs GGCNN2 inference to find optimal grasp poses
    3. Validates grasp quality and feasibility
    4. Stores the best grasp pose in telemetry for pickup operations
    5. Provides feedback on grasp generation success/failure
    """

    def __init__(self, context: StateContext):
        super().__init__(context)
        self.ggcnn2_module: Optional[GGcnn2Module] = None
        self.grasp_attempts = 0
        self.max_attempts = GRASP_EXECUTION_CONFIG['retry_attempts']
        self.state_start_time = 0.0
        self.best_grasp_result: Optional[Dict] = None
        self.grasp_generation_timeout = 10.0  # seconds
        self.min_grasp_quality = GRASP_DETECTION_CONFIG['min_quality_threshold']
        self.last_frame_time = 0.0
        self.frame_processing_interval = 0.5  # Process frames every 500ms

    def enter(self):
        """Initialize GGCNN2 module and reset state variables."""
        logger.info("Entering GeneratePickupState")

        try:
            # Initialize GGCNN2 module
            self.ggcnn2_module = GGcnn2Module(
                model_path=str(GGCNN2_MODEL_PATH),
                telemetry=self.context.telemetry,
                command_bus=self.context.commands,
                camera_manager=self.context.camera,
                kinematics_solver=self.context.ik
            )

            # Reset state variables
            self.grasp_attempts = 0
            self.best_grasp_result = None
            self.state_start_time = time.time()
            self.last_frame_time = 0.0

            logger.info("GeneratePickupState initialized successfully")

        except Exception as e:
            logger.error(f"Failed to initialize GeneratePickupState: {e}")
            raise

    def execute(self):
        """Main execution loop for grasp pose generation."""
        current_time = time.time()

        # Throttle frame processing to avoid excessive computation
        if current_time - self.last_frame_time < self.frame_processing_interval:
            return

        self.last_frame_time = current_time

        try:
            # Get depth frame from camera
            color_frame, depth_frame = self.context.camera.get_frames()

            if depth_frame is None:
                logger.warning("No depth frame available for grasp generation")
                return

            # Run GGCNN2 inference to generate grasp pose
            grasp_result = self._generate_grasp_pose(depth_frame)

            if grasp_result is None:
                logger.debug("No valid grasp pose generated")
                self.grasp_attempts += 1
                return

            # Validate grasp quality
            if not self._validate_grasp_pose(grasp_result):
                logger.warning(
                    f"Generated grasp pose failed validation (quality: {grasp_result.get('quality', 0):.3f})")
                self.grasp_attempts += 1
                return

            # Store the best grasp result
            self._store_best_grasp_result(grasp_result)
            self.grasp_attempts += 1

            logger.info(
                f"Grasp pose generated successfully (attempt {self.grasp_attempts}): "
                f"Quality = {grasp_result['quality']:.3f}, "
                f"Joint angles = {grasp_result['joint_angles']}"
            )

        except Exception as e:
            logger.error(f"Error in GeneratePickupState execution: {e}")
            self.grasp_attempts += 1

    def _generate_grasp_pose(self, depth_frame) -> Optional[Dict]:
        """
        Generate a grasp pose using GGCNN2 module.

        Args:
            depth_frame: Depth image from camera

        Returns:
            Dict containing grasp parameters and joint angles, or None if failed
        """
        try:
            # Use GGCNN2 module to process depth frame
            grasp_result = self.ggcnn2_module.process_depth_frame(depth_frame)

            if grasp_result is None:
                return None

            # Ensure we have all required fields
            required_fields = ['joint_angles', 'quality', 'grasp_pose_base']
            if not all(field in grasp_result for field in required_fields):
                logger.warning("Grasp result missing required fields")
                return None

            return grasp_result

        except Exception as e:
            logger.error(f"Failed to generate grasp pose: {e}")
            return None

    def _validate_grasp_pose(self, grasp_result: Dict) -> bool:
        """
        Validate the generated grasp pose for feasibility and quality.

        Args:
            grasp_result: Dict containing grasp parameters

        Returns:
            True if grasp pose is valid, False otherwise
        """
        try:
            # Check grasp quality threshold
            quality = grasp_result.get('quality', 0.0)
            if quality < self.min_grasp_quality:
                logger.debug(
                    f"Grasp quality too low: {quality:.3f} < {self.min_grasp_quality}")
                return False

            # Check joint angles validity
            joint_angles = grasp_result.get('joint_angles')
            if joint_angles is None or len(joint_angles) != 7:
                logger.warning("Invalid joint angles in grasp result")
                return False

            # Check for reasonable joint angle values (basic sanity check)
            joint_array = np.array(joint_angles)
            if np.any(np.isnan(joint_array)) or np.any(np.isinf(joint_array)):
                logger.warning("Joint angles contain NaN or Inf values")
                return False

            # Check grasp width constraints
            grasp_2d = grasp_result.get('grasp_2d', {})
            width = grasp_2d.get('width', 0.0)
            min_width = GRASP_DETECTION_CONFIG['min_grasp_width']
            max_width = GRASP_DETECTION_CONFIG['max_grasp_width']

            if width < min_width or width > max_width:
                logger.debug(
                    f"Grasp width out of range: {width:.1f}mm (range: {min_width}-{max_width}mm)")
                return False

            # Check grasp pose base frame validity
            grasp_pose_base = grasp_result.get('grasp_pose_base')
            if grasp_pose_base is None or len(grasp_pose_base) != 6:
                logger.warning("Invalid grasp pose in base frame")
                return False

            logger.debug(
                f"Grasp pose validation passed: quality={quality:.3f}, width={width:.1f}mm")
            return True

        except Exception as e:
            logger.error(f"Error validating grasp pose: {e}")
            return False

    def _store_best_grasp_result(self, grasp_result: Dict):
        """
        Store the best grasp result in telemetry for pickup operations.

        Args:
            grasp_result: Dict containing validated grasp parameters
        """
        try:
            # Store the grasp result as the best available
            self.best_grasp_result = grasp_result.copy()

            # Store joint angles in telemetry for pickup pose
            joint_angles = grasp_result['joint_angles']
            self.context.telemetry.update_pickup_pose_joints(joint_angles)

            # Store additional grasp information in telemetry
            # (You may want to extend telemetry to store more grasp data)
            logger.info(f"Best grasp pose stored in telemetry: {joint_angles}")

        except Exception as e:
            logger.error(f"Failed to store grasp result in telemetry: {e}")

    def is_complete(self) -> bool:
        """Check if grasp pose generation is complete."""
        current_time = time.time()
        elapsed_time = current_time - self.state_start_time

        # Check timeout
        if elapsed_time > self.grasp_generation_timeout:
            logger.warning(
                f"Grasp generation timeout ({self.grasp_generation_timeout}s) reached")
            return True

        # Check if we've exceeded max attempts
        if self.grasp_attempts >= self.max_attempts:
            logger.warning(
                f"Max grasp generation attempts ({self.max_attempts}) reached")
            return True

        # Check if we have a valid grasp result
        if self.best_grasp_result is not None:
            logger.info("Valid grasp pose generated successfully")
            return True

        # Continue if we haven't found a good grasp yet
        return False

    def exit(self):
        """Clean up resources and log final results."""
        logger.info("Exiting GeneratePickupState")

        # Log final results
        if self.best_grasp_result is not None:
            quality = self.best_grasp_result.get('quality', 0.0)
            joint_angles = self.best_grasp_result.get('joint_angles', [])
            logger.info(
                f"Final grasp pose: Quality={quality:.3f}, Joints={joint_angles}")
        else:
            logger.warning("No valid grasp pose was generated")

        # Clean up GGCNN2 module and free memory
        if self.ggcnn2_module:
            self.ggcnn2_module.cleanup()
            logger.info("GGCNN2 model memory freed")

        # Reset state variables
        self.grasp_attempts = 0
        self.best_grasp_result = None
        self.state_start_time = 0.0
        self.last_frame_time = 0.0

    def get_grasp_quality(self) -> float:
        """Get the quality of the best generated grasp pose."""
        if self.best_grasp_result:
            return self.best_grasp_result.get('quality', 0.0)
        return 0.0

    def get_grasp_joint_angles(self) -> Optional[List[float]]:
        """Get the joint angles for the best generated grasp pose."""
        if self.best_grasp_result:
            return self.best_grasp_result.get('joint_angles')
        return None

    def get_grasp_pose_base(self) -> Optional[List[float]]:
        """Get the grasp pose in base frame for the best generated grasp."""
        if self.best_grasp_result:
            return self.best_grasp_result.get('grasp_pose_base')
        return None

    def get_generation_stats(self) -> Dict:
        """Get statistics about the grasp generation process."""
        elapsed_time = time.time() - self.state_start_time
        return {
            'attempts': self.grasp_attempts,
            'elapsed_time': elapsed_time,
            'success': self.best_grasp_result is not None,
            'best_quality': self.get_grasp_quality(),
            'timeout_reached': elapsed_time > self.grasp_generation_timeout
        }
