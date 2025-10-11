"""
Grasping state using GGCNN2 for object detection and grasp planning.
"""
import logging
import time
import numpy as np
from typing import Optional

from states.base_state import BaseState
from states.context import StateContext
from object_detection.grasp_detector_module import GGcnn2Module
from config.config import GGCNN2_MODEL_PATH, GRASP_EXECUTION_CONFIG, DEBUG_MODE, DEBUG_CONFIG

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

    def __init__(self, context: StateContext, auto_process: bool = False):
        super().__init__(context)
        self.ggcnn2_module: Optional[GGcnn2Module] = None
        self.grasp_attempts = 0
        self.max_attempts = GRASP_EXECUTION_CONFIG['retry_attempts']
        self.grasp_start_time = 0.0
        self.current_grasp_result = None
        self.state_start_time = 0.0

        # Debug mode variables
        self.debug_mode = DEBUG_MODE
        # Disable frame selection if auto_process is True (e.g., in sequencer mode)
        self.frame_selection_enabled = DEBUG_CONFIG.get(
            'frame_selection_enabled', True) and not auto_process
        self.selected_frame = None
        self.frame_selected = False

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

            # Handle debug mode with frame selection
            if self.debug_mode and self.frame_selection_enabled:
                self._handle_debug_mode(color_frame, depth_frame)
            else:
                # Normal execution - process every frame
                self._process_frame(depth_frame, color_frame)

        except Exception as e:
            logger.error(f"Error in GraspingState execution: {e}")
            self.grasp_attempts += 1

    def is_complete(self) -> bool:
        """Check if grasping is complete."""
        # Check if we've exceeded max attempts
        if self.grasp_attempts >= self.max_attempts:
            logger.warning(f"Max grasp attempts ({self.max_attempts}) reached")
            return True

        # When we have a valid grasp result, wait a bit to view the visualization
        if self.current_grasp_result is not None:
            if self.grasp_start_time == 0.0:
                self.grasp_start_time = time.time()
                logger.info(
                    "Grasp pose generated successfully - displaying for 3 seconds")
                return False

            # Wait 3 seconds to view the grasp visualization
            elapsed_time = time.time() - self.grasp_start_time
            if elapsed_time >= 3.0:
                logger.info("Grasp visualization complete - state complete")
                return True

            return False

        return False

    def exit(self):
        """Clean up resources."""
        logger.info("Exiting GraspingState")

        # Clean up GGCNN2 module if needed
        if self.ggcnn2_module:
            # GGCNN2 module doesn't need explicit cleanup
            pass

        # Clean up visualization windows if debug mode was enabled
        if self.debug_mode:
            try:
                import cv2
                cv2.destroyAllWindows()
                logger.debug("Closed visualization windows")
            except Exception as e:
                logger.warning(f"Failed to close visualization windows: {e}")

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

    def _handle_debug_mode(self, color_frame, depth_frame):
        """Handle debug mode with live feed display and frame selection."""
        try:
            import cv2

            # Convert depth frame to displayable format
            if hasattr(depth_frame, 'get_data'):
                depth_array = np.asanyarray(depth_frame.get_data())
            else:
                depth_array = depth_frame

            # Normalize depth for display
            depth_display = self._normalize_depth_for_display(depth_array)

            # Create display image with instructions
            display_image = self._create_debug_display(
                depth_display, color_frame)

            # Show the debug window
            window_title = DEBUG_CONFIG.get(
                'window_title', 'Debug Feed - Press SPACEBAR to process frame')
            cv2.imshow(window_title, display_image)

            # Check for keyboard input
            key = cv2.waitKey(1) & 0xFF

            if key == ord(' '):  # Spacebar pressed
                logger.info("Frame selected for processing!")
                self.selected_frame = depth_frame
                self.frame_selected = True
                self._process_frame(depth_frame, color_frame)
            elif key == ord('q') or key == 27:  # 'q' or ESC to quit
                logger.info("Debug mode quit requested")
                cv2.destroyAllWindows()
                return

        except Exception as e:
            logger.error(f"Error in debug mode handling: {e}")

    def _process_frame(self, depth_frame, color_frame=None):
        """Process a single frame for grasp detection."""
        try:
            # Run grasp detection (GGCNN2 or GR-ConvNet)
            grasp_result = self.ggcnn2_module.process_depth_frame(
                depth_frame, color_frame)

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
            logger.error(f"Error processing frame: {e}")

    def _normalize_depth_for_display(self, depth_array):
        """Normalize depth array for display purposes."""
        try:
            # Handle NaN values
            depth_clean = np.nan_to_num(depth_array, nan=0.0)

            # Normalize to 0-255 range
            if depth_clean.max() > depth_clean.min():
                depth_normalized = ((depth_clean - depth_clean.min()) /
                                    (depth_clean.max() - depth_clean.min()) * 255).astype(np.uint8)
            else:
                depth_normalized = np.zeros_like(depth_clean, dtype=np.uint8)

            return depth_normalized

        except Exception as e:
            logger.error(f"Error normalizing depth for display: {e}")
            return np.zeros_like(depth_array, dtype=np.uint8)

    def _create_debug_display(self, depth_display, color_frame):
        """Create debug display with instructions and frame information."""
        try:
            import cv2

            # Convert depth to color map for better visualization
            depth_colored = cv2.applyColorMap(depth_display, cv2.COLORMAP_JET)

            # Add text overlay with instructions
            instructions = [
                "DEBUG MODE - Frame Selection",
                "Press SPACEBAR to process this frame",
                "Press 'q' or ESC to quit",
                f"Frame: {self.grasp_attempts + 1}",
                f"Selected: {'Yes' if self.frame_selected else 'No'}"
            ]

            # Add text to the image
            y_offset = 30
            for i, text in enumerate(instructions):
                color = (0, 255, 0) if i == 0 else (255, 255, 255)
                thickness = 2 if i == 0 else 1
                cv2.putText(depth_colored, text, (10, y_offset),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, thickness)
                y_offset += 25

            return depth_colored

        except Exception as e:
            logger.error(f"Error creating debug display: {e}")
            return depth_display
