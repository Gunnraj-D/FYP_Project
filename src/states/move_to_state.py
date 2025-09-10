import logging
from abc import ABC, abstractmethod
from typing import Optional, List
import numpy as np

from shared_state import SharedState
from camera_manager import CameraManager
from opc_client import OPCClient
from kinematics_solver import InverseKinematicsSolver

from states_enum import States
from base_state import BaseState

logger = logging.getLogger(__name__)


class MoveToState(BaseState):
    """Move To State - Moves robot to defined state, then completes."""

    def __init__(self, shared_state: SharedState, camera_manager: CameraManager,
                 opc_client: OPCClient, kinematics_solver: InverseKinematicsSolver,
                 target_location: tuple[int, int, int]):

        super().__init__(shared_state, camera_manager,
                       opc_client, kinematics_solver)
        
        self.target_joint_angles = self.kinematics_solver.solve_XYZ(target_location, self.shared_state.get_current_joints())
        self.started_motion = False
        self.reached_target = False

    def enter(self):
        logger.info("Entering MOVE_TO state")

    def execute(self):
        if self.started_motion:
            current_joint_angles = self.shared_state.get_current_joints()
            # check if joints are close enough
            if np.allclose(current_joint_angles, self.target_joint_angles, atol=1e-2):
                self.reached_target = True

        self.shared_state.update_target_joints(self.target_joint_angles)
        self.started_motion = True
        
    def exit(self):
        logger.info("Exiting MOVE_TO state")

    def is_complete(self) -> bool:
        if self.reached_target:
            return True
        
        return False
