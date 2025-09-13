import logging
import numpy as np

from states.context import StateContext
from base_state import BaseState
from control.command_bus import SetJoints

logger = logging.getLogger(__name__)


class MoveToState(BaseState):
    """Move To State - Moves robot to defined state, then completes."""

    def __init__(self, context: StateContext, target_location: tuple[int, int, int]):
        super().__init__(context=context)
        self.target_location = target_location  # mm in base frame
        self.started_motion = False
        self.target_joint_angles = None  # computed on enter/first execute

    def enter(self):
        logger.info("Entering MOVE_TO state")

    def execute(self):
        if self.started_motion:
            return

        self.target_joint_angles = self.context.ik.solve_XYZ(
            self.target_location, self.context.telemetry.get_current_joints())
        self.context.commands.send(SetJoints(list(self.target_joint_angles)))
        self.started_motion = True

    def exit(self):
        logger.info("Exiting MOVE_TO state")

    def is_complete(self) -> bool:
        current = self.context.telemetry.get_current_joints()
        return np.allclose(current, self.target_joint_angles, atol=1e-2)
