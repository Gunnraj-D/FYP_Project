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

        # Get current joints with fallback to default values
        current_joints = self.context.telemetry.get_current_joints()
        logger.info(f"current joints: {current_joints}")
        if current_joints is None or len(current_joints) == 0:
            logger.warning(
                "Current joints not available, using default home position")
            current_joints = np.array([0.0] * 7, dtype=float)

        # Debug: Print values to understand the issue
        logger.info(f"Target location: {target_location}")
        logger.info(f"Current joints: {current_joints}")
        logger.info(f"Current joints type: {type(current_joints)}")
        logger.info(
            f"Current joints shape: {current_joints.shape if hasattr(current_joints, 'shape') else 'No shape'}")

        logger.info(f"Target location: {target_location}")
        self.target_joint_angles = self.context.ik.solve_XYZ(
            target_location, current_joints)
        if self.target_joint_angles is None:
            logger.error(
                f"IK solver failed for target location: {target_location}")
            # Set to current to avoid infinite loop
            self.target_joint_angles = current_joints
            return
        self.context.commands.send(
            SetJoints(list(self.target_joint_angles)))
        self.started_motion = True
        # except Exception as e:
        #     logger.error(f"IK solver error: {e}")
        #     self.target_joint_angles = current_joints  # Set to current to avoid infinite loop
        #     return

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
        if self.target_joint_angles is None:
            logger.warning("Target joint angles not set, state not complete")
            return False

        if current is None:
            logger.warning("Current joints not available, state not complete")
            return False

        # Convert to numpy arrays for comparison
        current_np = np.array(current)
        target_np = np.array(self.target_joint_angles)

        is_close = np.allclose(current_np, target_np, atol=1e-2)
        logger.info(f"Current joints: {current_np}")
        logger.info(f"Target joints: {target_np}")
        logger.info(f"Are close: {is_close}")

        return is_close
