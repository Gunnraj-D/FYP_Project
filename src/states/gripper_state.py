import logging
from typing import Optional
import numpy as np

from shared_state import SharedState
from camera_manager import CameraManager
from opc_client import OPCClient
from kinematics_solver import InverseKinematicsSolver
from hand_detection_module import HandTracker

from states_enum import States
from base_state import BaseState

logger = logging.getLogger(__name__)


class GripperControlState(BaseState):
    """Gripper Control State - Handles gripper function. """

    def __init__(self, shared_state: SharedState, camera_manager: CameraManager,
                 opc_client: OPCClient, kinematics_solver: InverseKinematicsSolver,
                 action: str):

        super().__init__(shared_state, camera_manager,
                         opc_client, kinematics_solver)

        if action not in ("open", "close"):
            raise ValueError(f"Action type requested for Gripper State does not exist! Action requested: {action}")

        self.action = action
        self.started_gripper = False

    def enter(self):
        logger.info("Entering GRIPPER_CONTROL state")

    def execute(self):
        if self.started_gripper:
            return

        self.shared_state.update_target_gripper_status(self.action)
        self.started_gripper = True

    def exit(self):
        logger.info("Exiting GRIPPER_CONTROL state")

    def is_complete(self) -> bool:
        if self.shared_state.get_current_gripper_status() == self.shared_state.get_target_gripper_status():
            return True

        return False
