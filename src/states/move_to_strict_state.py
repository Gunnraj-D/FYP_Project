"""
MoveToStrict State - Moves robot to exact pose with strict orientation control.

This state uses the solve_pose() method from the IK solver to achieve precise
position and orientation control. It's designed for tasks where exact orientation
matters, such as grasping operations.
"""

import logging
import numpy as np
from scipy.spatial.transform import Rotation as R

from states.context import StateContext
from .base_state import BaseState
from control.command_bus import SetJoints
from kinematics.kinematics_solver import get_facing_down_orientation

logger = logging.getLogger(__name__)


class MoveToStrictState(BaseState):
    """
    Move To Strict State - Moves robot to exact pose with strict orientation control.

    This state uses solve_pose() for precise 6-DOF control, making it ideal for
    grasping operations where exact orientation matters.
    """

    def __init__(self, context: StateContext, target_pose=None, pose_from_telemetry=None,
                 enforce_face_down=False):
        """
        Initialize MoveToStrictState.

        Args:
            context: State context with shared resources
            target_pose: Target pose [x, y, z, rx, ry, rz] in base frame (meters, radians)
            pose_from_telemetry: Key to retrieve pose from telemetry
            enforce_face_down: If True, override orientation with face-down matrix
        """
        super().__init__(context=context)

        # Target pose in base frame [x, y, z, rx, ry, rz]
        self.target_pose = target_pose
        # Key to retrieve pose from telemetry
        self.pose_from_telemetry = pose_from_telemetry
        # Whether to enforce face-down orientation
        self.enforce_face_down = enforce_face_down

        # State tracking
        self.started_motion = False
        self.target_joint_angles = None
        self._log_counter = 0

    def enter(self):
        """Initialize the state."""
        logger.info("Entering MOVE_TO_STRICT state")

    def execute(self):
        """Execute the strict pose movement."""
        if self.started_motion:
            return

        # Get target pose from telemetry or use provided pose
        target_pose = self._get_target_pose()
        if target_pose is None:
            logger.error("No valid target pose available")
            return

        # Apply face-down enforcement if requested
        if self.enforce_face_down:
            target_pose = self._apply_face_down_enforcement(target_pose)
            logger.info("🔒 Applied face-down orientation enforcement")

        # Get current joints
        current_joints = self.context.telemetry.get_current_joints()
        if current_joints is None or len(current_joints) == 0:
            logger.warning(
                "Current joints not available, using default home position")
            current_joints = np.array([0.0] * 7, dtype=float)

        logger.info(
            f"Target pose: pos={target_pose[:3]}, ori(deg)={np.degrees(target_pose[3:6])}")
        logger.info(f"Current joints: {current_joints}")

        # Solve IK using solve_pose for precise 6-DOF control
        try:
            self.target_joint_angles = self.context.ik.solve_pose(
                target_pose, current_joints.tolist()
            )

            if self.target_joint_angles is None:
                logger.error("IK solver failed - no solution found")
                self.target_joint_angles = current_joints
                return

            logger.info(f"IK solution: {self.target_joint_angles}")

        except Exception as e:
            logger.error(f"IK solver error: {e}")
            self.target_joint_angles = current_joints
            return

        # Send joint command
        self.context.commands.send(SetJoints(list(self.target_joint_angles)))
        self.started_motion = True

    def _get_target_pose(self):
        """Get target pose from telemetry or return provided pose."""
        if self.pose_from_telemetry:
            return self._get_pose_from_telemetry()
        else:
            return self.target_pose

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

    def _apply_face_down_enforcement(self, pose):
        """
        Apply face-down orientation enforcement to a pose.

        Args:
            pose: [x, y, z, rx, ry, rz] pose in base frame

        Returns:
            Modified pose with face-down orientation
        """
        # Keep the position, replace orientation with face-down
        face_down_matrix = get_facing_down_orientation()
        face_down_rpy = R.from_matrix(face_down_matrix).as_euler('xyz')

        # Return [x, y, z, rx, ry, rz] with face-down orientation
        return list(pose[:3]) + list(face_down_rpy)

    def is_complete(self) -> bool:
        """Check if the movement is complete."""
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

        # Log only every 50 cycles to reduce spam
        self._log_counter += 1
        if self._log_counter % 50 == 0 or is_close:
            logger.info(f"Current joints: {current_np}")
            logger.info(f"Target joints: {target_np}")
            logger.info(f"Are close: {is_close}")

        return is_close

    def exit(self):
        """Clean up the state."""
        logger.info("Exiting MOVE_TO_STRICT state")

    @classmethod
    def with_face_down_enforcement(cls, context: StateContext, target_pose):
        """
        Factory method to create MoveToStrictState with face-down enforcement.

        Args:
            context: State context
            target_pose: Target pose [x, y, z, rx, ry, rz] (orientation will be overridden)

        Returns:
            MoveToStrictState instance with face-down enforcement
        """
        return cls(context, target_pose=target_pose, enforce_face_down=True)

    @classmethod
    def from_telemetry_with_face_down(cls, context: StateContext, pose_key):
        """
        Factory method to create MoveToStrictState from telemetry with face-down enforcement.

        Args:
            context: State context
            pose_key: Telemetry key for pose ('generated_grasp_pose', etc.)

        Returns:
            MoveToStrictState instance with face-down enforcement
        """
        return cls(context, pose_from_telemetry=pose_key, enforce_face_down=True)

    @classmethod
    def with_custom_pose(cls, context: StateContext, target_pose, enforce_face_down=False):
        """
        Factory method to create MoveToStrictState with custom pose.

        Args:
            context: State context
            target_pose: Target pose [x, y, z, rx, ry, rz]
            enforce_face_down: Whether to enforce face-down orientation

        Returns:
            MoveToStrictState instance
        """
        return cls(context, target_pose=target_pose, enforce_face_down=enforce_face_down)
