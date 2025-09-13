"""
Hand Tracking State - Active hand tracking and placement pose calculation.
This state continuously tracks the operator's hand and calculates the final placement pose
for object handoff operations.
"""
import logging
import time
import numpy as np
from typing import Optional

from states.base_state import BaseState
from states.context import StateContext
from hand_detection.hand_detection_module import HandTracker
from config.config import HAND_STABILITY_TIME_THRESHOLD

logger = logging.getLogger(__name__)


class HandTrackingState(BaseState):
    """
    State for active hand tracking and placement pose calculation.

    This state:
    1. Activates the HandTracker module for continuous hand detection
    2. Receives real-time hand positions and stores them in telemetry
    3. Calculates final placement pose by offsetting hand position with pickup height
    4. Transitions to next state when hand position is stable
    """

    def __init__(self, context: StateContext):
        super().__init__(context)
        self.hand_tracker: Optional[HandTracker] = None
        self.state_start_time = 0.0
        self.hand_stable_start_time = 0.0
        self.is_hand_stable = False
        self.last_hand_position = None
        self.stability_check_interval = HAND_STABILITY_TIME_THRESHOLD / \
            20.0  # Check stability 20 times per threshold period
        self.last_stability_check = 0.0

    def enter(self):
        """Initialize hand tracker and reset state variables."""
        logger.info("Entering HandTrackingState")

        try:
            # Initialize hand tracker
            self.hand_tracker = HandTracker(
                telemetry=self.context.telemetry,
                command_bus=self.context.commands,
                camera_manager=self.context.camera
            )

            # Start hand tracking
            self.hand_tracker.start()

            # Reset state variables
            self.state_start_time = time.time()
            self.hand_stable_start_time = 0.0
            self.is_hand_stable = False
            self.last_hand_position = None
            self.last_stability_check = 0.0

            logger.info("HandTrackingState initialized successfully")

        except Exception as e:
            logger.error(f"Failed to initialize HandTrackingState: {e}")
            raise

    def execute(self):
        """Main execution loop for hand tracking and pose calculation."""
        current_time = time.time()

        # Throttle stability checking
        if current_time - self.last_stability_check < self.stability_check_interval:
            return

        self.last_stability_check = current_time

        try:
            # Get current hand position from telemetry
            hand_position = self.context.telemetry.get_camera_vector()

            # Check if hand is detected
            if hand_position and hand_position != [0.0, 0.0, 0.0]:
                self._update_hand_tracking(hand_position)
                self._calculate_placement_pose(hand_position)
            else:
                # Reset stability if no hand detected
                self.is_hand_stable = False
                self.hand_stable_start_time = 0.0
                logger.debug("No hand detected, resetting stability")

        except Exception as e:
            logger.error(f"Error in HandTrackingState execution: {e}")

    def _update_hand_tracking(self, hand_position):
        """Update hand tracking state and check stability."""
        try:
            # Store live hand pose in telemetry
            # Convert from camera vector format to pose format [x, y, z, rx, ry, rz]
            # Add rotation components (assuming no rotation)
            live_hand_pose = hand_position + [0.0, 0.0, 0.0]
            self.context.telemetry.set_live_hand_pose(live_hand_pose)

            # Check hand stability
            if self.last_hand_position is not None:
                # Calculate distance moved
                distance = np.linalg.norm(
                    np.array(hand_position) - np.array(self.last_hand_position)
                )

                # Check if hand is stable (within threshold)
                if distance < HAND_STABILITY_THRESHOLD:
                    if not self.is_hand_stable:
                        self.is_hand_stable = True
                        self.hand_stable_start_time = time.time()
                        logger.info("Hand detected as stable")
                else:
                    # Reset stability if hand moved too much
                    self.is_hand_stable = False
                    self.hand_stable_start_time = 0.0
                    logger.debug(
                        f"Hand moved {distance:.1f}mm, resetting stability")

            self.last_hand_position = hand_position.copy()

        except Exception as e:
            logger.error(f"Error updating hand tracking: {e}")

    def _calculate_placement_pose(self, hand_position):
        """Calculate final placement pose for object handoff."""
        try:
            # Get pickup height offset from telemetry
            pickup_height_offset = self.context.telemetry.get_pickup_height_offset()

            # Convert hand position to meters if needed (assuming input is in mm)
            hand_pos_mm = np.array(hand_position)

            # Calculate placement pose by offsetting hand position upward
            # This ensures the robot places the object at a safe height above the palm
            placement_pose = hand_pos_mm.copy()
            placement_pose[2] += pickup_height_offset * \
                1000.0  # Convert meters to mm

            # Add rotation components (maintain same orientation as handoff approach)
            full_placement_pose = list(
                placement_pose) + [0.0, 0.0, -90.0]  # [x, y, z, rx, ry, rz]

            # Store calculated handoff pose in telemetry
            self.context.telemetry.set_calculated_handoff_pose(
                full_placement_pose)

            logger.debug(f"Placement pose calculated: {full_placement_pose}")
            logger.debug(
                f"Hand position: {hand_position}, Height offset: {pickup_height_offset:.3f}m")

        except Exception as e:
            logger.error(f"Error calculating placement pose: {e}")

    def is_complete(self) -> bool:
        """Check if hand tracking is complete (hand stable for required duration)."""
        current_time = time.time()

        # Check if hand has been stable for the required duration
        if self.is_hand_stable:
            stable_duration = current_time - self.hand_stable_start_time
            if stable_duration >= HAND_STABILITY_TIME_THRESHOLD:
                logger.info(
                    f"Hand tracking complete - hand stable for {stable_duration:.1f}s")
                return True

        # Check timeout (safety measure)
        elapsed_time = current_time - self.state_start_time
        timeout_threshold = HAND_STABILITY_TIME_THRESHOLD * \
            15  # 15x the stability threshold
        if elapsed_time > timeout_threshold:
            logger.warning("Hand tracking timeout reached")
            return True

        return False

    def exit(self):
        """Clean up hand tracker and log final results."""
        logger.info("Exiting HandTrackingState")

        # Stop hand tracker
        if self.hand_tracker:
            self.hand_tracker.stop()
            logger.info("Hand tracker stopped")

        # Log final placement pose
        try:
            placement_pose = self.context.telemetry.get_calculated_handoff_pose()
            if placement_pose and placement_pose != [0.0] * 6:
                logger.info(f"Final placement pose: {placement_pose}")
            else:
                logger.warning("No valid placement pose was calculated")
        except Exception as e:
            logger.error(f"Error retrieving final placement pose: {e}")

        # Reset state variables
        self.hand_tracker = None
        self.state_start_time = 0.0
        self.hand_stable_start_time = 0.0
        self.is_hand_stable = False
        self.last_hand_position = None
        self.last_stability_check = 0.0

    def get_tracking_stats(self) -> dict:
        """Get statistics about the hand tracking process."""
        current_time = time.time()
        elapsed_time = current_time - self.state_start_time

        stable_duration = 0.0
        if self.is_hand_stable:
            stable_duration = current_time - self.hand_stable_start_time

        return {
            'elapsed_time': elapsed_time,
            'is_hand_stable': self.is_hand_stable,
            'stable_duration': stable_duration,
            'hand_detected': self.last_hand_position is not None,
            'current_hand_position': self.last_hand_position
        }
