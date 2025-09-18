import logging
import numpy as np

from states.context import StateContext
from .base_state import BaseState
from control.command_bus import SetJoints
from camera_management.camera_transform_module import transform_camera_to_base

from config.config import HAND_STABILITY_TIME_THRESHOLD

logger = logging.getLogger(__name__)


class TrackingState(BaseState):
    """Tracking State - Follows the operator's hand until stability timeout"""

    def __init__(self, context: StateContext):
        super().__init__(context=context)

    def enter(self):
        logger.info("Entering TRACKING state")
        if self.context.hand_tracker is not None:
            self.context.hand_tracker.start()

    def execute(self):
        camera_vector = self.context.telemetry.get_camera_vector()

        # Skip if no hand detected
        if camera_vector == [0.0, 0.0, 0.0]:
            logger.debug("No hand detected")
            return

        current_7 = self.context.telemetry.get_current_joints()

        tcp_pose = self.context.ik.solve_tcp(current_7)

        # Transform hand position from camera to base frame
        hand_position_base = transform_camera_to_base(camera_vector, tcp_pose)

        # Solve inverse kinematics for target position
        target_7 = self.context.ik.solve_XYZ(
            hand_position_base, current_7
        )

        # Send command with latest coalesced joint target
        self.context.commands.send(SetJoints(list(target_7)))

        logger.debug(f"Target joints updated: {target_7}")

    def exit(self):
        logger.info("Exiting TRACKING state")

    def is_complete(self) -> bool:
        return self.context.telemetry.is_hand_stable(HAND_STABILITY_TIME_THRESHOLD)
