import logging
import numpy as np

from states.context import StateContext
from .base_state import BaseState
from control.command_bus import SetJoints

logger = logging.getLogger(__name__)


class MoveToState(BaseState):
    """Move To State - Moves robot to defined state, then completes."""

    def __init__(self, context: StateContext, target_location=None, pose_from_telemetry=None):
        super().__init__(context=context)
        # mm in base frame (legacy support)
        self.target_location = target_location
        # Key to retrieve pose from telemetry
        self.pose_from_telemetry = pose_from_telemetry
        self.started_motion = False
        self.target_joint_angles = None  # computed on enter/first execute

    def enter(self):
        logger.info("Entering MOVE_TO state")

    def execute(self):
        if self.started_motion:
            return

        # Get target location from telemetry or use provided location
        if self.pose_from_telemetry:
            target_pose = self._get_pose_from_telemetry()
            if target_pose is None:
                logger.error(
                    f"Failed to retrieve pose from telemetry key: {self.pose_from_telemetry}")
                return
            # Extract XYZ coordinates from pose [x, y, z, rx, ry, rz]
            target_location = tuple(target_pose[:3])
        else:
            target_location = self.target_location

        if target_location is None:
            logger.error("No target location specified")
            return

        self.target_joint_angles = self.context.ik.solve_XYZ(
            target_location, self.context.telemetry.get_current_joints())
        self.context.commands.send(SetJoints(list(self.target_joint_angles)))
        self.started_motion = True

    def _get_pose_from_telemetry(self):
        """Get pose from telemetry based on the specified key."""
        if self.pose_from_telemetry == 'generated_grasp_pose':
            return self.context.telemetry.get_generated_grasp_pose()
        elif self.pose_from_telemetry == 'generated_approach_pose':
            return self.context.telemetry.get_generated_approach_pose()
        elif self.pose_from_telemetry == 'calculated_handoff_pose':
            return self.context.telemetry.get_calculated_handoff_pose()
        else:
            logger.error(
                f"Unknown telemetry pose key: {self.pose_from_telemetry}")
            return None

    def exit(self):
        logger.info("Exiting MOVE_TO state")

    def is_complete(self) -> bool:
        current = self.context.telemetry.get_current_joints()
        return np.allclose(current, self.target_joint_angles, atol=1e-2)
