import logging
from typing import Optional
import numpy as np

from shared_state import SharedState
from camera_manager import CameraManager
from opc_client import OPCClient
from kinematics_solver import InverseKinematicsSolver
from hand_detection_module import HandTracker
from camera_transform_module import transform_camera_to_base

from states_enum import States
from base_state import BaseState

logger = logging.getLogger(__name__)


class TrackingState(BaseState):
    """Tracking State - Will follow operator's hand until hand stops moving for set period"""

    def __init__(self, shared_state: SharedState, camera_manager: CameraManager,
                 opc_client: OPCClient, kinematics_solver: InverseKinematicsSolver,
                 hand_tracker: HandTracker):

        super().__init__(shared_state, camera_manager,
                         opc_client, kinematics_solver)

        self.hand_tracker = hand_tracker
        self.stability_threshold = 2.0

    def enter(self):
        logger.info("Entering TRACKING state")
        self.hand_tracker.start()

    def execute(self):
        camera_vector = self.shared_state.get_camera_vector()

        # Skip if no hand detected
        if camera_vector == [0.0, 0.0, 0.0]:
            logger.debug("No hand detected")
            return

        current_joints = self.shared_state.get_current_joints()

        # Calculate current TCP pose
        tcp_pose = self.kinematics_solver.solve_tcp(current_joints)

        # Transform hand position from camera to base frame
        hand_position_base = transform_camera_to_base(camera_vector, tcp_pose)

        # Solve inverse kinematics for target position

        # ? Could this use the jacobian instead?
        target_joints_full = self.kinematics_solver.solve_XYZ(
            hand_position_base, current_joints
        )

        # Remove base and end-effector joints
        target_joints = target_joints_full[1:8]

        # Update shared state with target joints
        self.shared_state.update_target_joints(target_joints)

        logger.debug(f"Target joints updated: {target_joints}")

    def exit(self):
        logger.info("Exiting TRACKING state")

    def is_complete(self) -> bool:
        if self.shared_state.is_hand_stable(self.stability_threshold):
            return True
        
        return False
