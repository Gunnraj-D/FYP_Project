import logging
import time
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
        self.send_time = None

    def enter(self):
        logger.info("Entering GRIPPER_CONTROL state")

    def execute(self):
        if self.sent:
            return
        # Set target status in telemetry before sending command
        self.context.telemetry.update_target_gripper_status(self.action)
        self.context.commands.send(SetGripper(self.action))
        self.sent = True
        self.send_time = time.time()
        logger.info(f"📤 Sent gripper command: {self.action}")

    def exit(self):
        logger.info("Exiting GRIPPER_CONTROL state")

    def is_complete(self) -> bool:
        # Wait 2.5 seconds after sending command
        if not self.sent or self.send_time is None:
            return False  # Command not sent yet

        elapsed_time = time.time() - self.send_time
        is_done = elapsed_time >= 2.5

        if is_done:
            logger.info(
                f"✅ Gripper wait completed: {self.action} (waited {elapsed_time:.2f}s)")

        return is_done
