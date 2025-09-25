import logging
from states.context import StateContext
from .base_state import BaseState
from control.command_bus import SetGripper

logger = logging.getLogger(__name__)


class GripperControlState(BaseState):
    """Gripper Control State - Handles gripper function."""

    def __init__(self, context: StateContext, action: str):
        super().__init__(context=context)

        # TODO: use enums for gripper state?
        if action not in ("open", "close"):
            raise ValueError(f"Invalid gripper action: {action}")

        self.action = action
        self.sent = False

    def enter(self):
        logger.info("Entering GRIPPER_CONTROL state")

    def execute(self):
        if self.sent:
            return
        # Set target status in telemetry before sending command
        self.context.telemetry.update_target_gripper_status(self.action)
        self.context.commands.send(SetGripper(self.action))
        self.sent = True

    def exit(self):
        logger.info("Exiting GRIPPER_CONTROL state")

    def is_complete(self) -> bool:
        return self.context.telemetry.get_current_gripper_status() == self.action
