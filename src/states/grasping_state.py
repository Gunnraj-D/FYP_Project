"""
Grasping state using GGCNN2 for object detection and grasp planning.
"""
import logging
import time
from typing import Optional

from states.base_state import BaseState
from states.context import StateContext
from object_detection.ggcnn2_module import GGcnn2Module
from config.config import GGCNN2_MODEL_PATH, GRASP_EXECUTION_CONFIG

logger = logging.getLogger(__name__)


class GraspingState(BaseState):
    """
    State for detecting and grasping objects using GGCNN2.

    This state:
    1. Captures depth images from camera
    2. Runs GGCNN2 inference to find grasp poses
    3. Converts grasp poses to joint angles
    4. Executes the grasp motion
    5. Monitors grasp success
    """

    def __init__(self, context: StateContext):
        super().__init__(context)
        self.ggcnn2_module: Optional[GGcnn2Module] = None
        self.grasp_attempts = 0
        self.max_attempts = GRASP_EXECUTION_CONFIG['retry_attempts']
        self.grasp_start_time = 0.0
        self.current_grasp_result = None
        self.state_start_time = 0.0

    def enter(self):
        """Initialize GGCNN2 module and reset state variables."""
        logger.info("Entering GraspingState")

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
            self.current_grasp_result = None
            self.state_start_time = time.time()

            logger.info("GraspingState initialized successfully")

        except Exception as e:
            logger.error(f"Failed to initialize GraspingState: {e}")
            raise

    def execute(self):
        """Main execution loop for grasping."""
        try:
            # Get depth frame from camera
            color_frame, depth_frame = self.context.camera.get_frames()

            if depth_frame is None:
                logger.warning("No depth frame available")
                return

            # Run GGCNN2 inference
            grasp_result = self.ggcnn2_module.process_depth_frame(depth_frame)

            if grasp_result is None:
                logger.debug("No valid grasp found")
                return

            # Store the grasp result
            self.current_grasp_result = grasp_result
            self.grasp_attempts += 1

            logger.info(
                f"Grasp attempt {self.grasp_attempts}: Quality = {grasp_result['quality']:.3f}")

            # The grasp command is automatically sent by GGCNN2 module
            # We just need to wait for execution

        except Exception as e:
            logger.error(f"Error in GraspingState execution: {e}")
            self.grasp_attempts += 1

    def is_complete(self) -> bool:
        """Check if grasping is complete."""
        # Check if we've exceeded max attempts
        if self.grasp_attempts >= self.max_attempts:
            logger.warning(f"Max grasp attempts ({self.max_attempts}) reached")
            return True

        # Check if we have a valid grasp result
        if self.current_grasp_result is None:
            return False

        # Check if enough time has passed for grasp execution
        if self.grasp_start_time == 0.0:
            self.grasp_start_time = time.time()
            return False

        elapsed_time = time.time() - self.grasp_start_time
        required_time = GRASP_EXECUTION_CONFIG['grasp_duration']

        if elapsed_time >= required_time:
            logger.info("Grasp execution completed")
            return True

        return False

    def exit(self):
        """Clean up resources."""
        logger.info("Exiting GraspingState")

        # Clean up GGCNN2 module if needed
        if self.ggcnn2_module:
            # GGCNN2 module doesn't need explicit cleanup
            pass

        # Reset state variables
        self.grasp_attempts = 0
        self.current_grasp_result = None
        self.grasp_start_time = 0.0

    def get_grasp_quality(self) -> float:
        """Get the quality of the current grasp attempt."""
        if self.current_grasp_result:
            return self.current_grasp_result.get('quality', 0.0)
        return 0.0

    def get_grasp_joint_angles(self) -> Optional[list]:
        """Get the joint angles for the current grasp."""
        if self.current_grasp_result:
            return self.current_grasp_result.get('joint_angles')
        return None

