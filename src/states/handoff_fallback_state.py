"""
Handoff Fallback State - Fallback behavior for hand tracking failures.

This state is activated when the hand tracking state fails due to persistent
hand occlusion. It moves the robot to 15cm above the operator's palm as a
simple fallback placement strategy.

Behavior:
1. Captures current skeleton snapshot from ZED receiver
2. Computes palm position (midpoint between wrist and fingertips)
3. Adds 15cm vertical offset (Z-axis)
4. Solves IK and sends single movement command
5. Completes when robot reaches target or timeout
"""
import logging
import time
import numpy as np
from typing import Optional

from states.base_state import BaseState
from states.context import StateContext
from control.command_bus import SetJoints
from kinematics.kinematics_solver import get_facing_down_orientation
from hand_detection.skeleton_data_provider import SkeletonDataProvider

logger = logging.getLogger(__name__)


class HandoffFallbackState(BaseState):
    """
    Fallback state for handling hand tracking failures due to occlusion.

    Captures a snapshot of the operator's hand from ZED skeleton tracking,
    computes palm position, and moves robot to 15cm above the palm.
    """

    def __init__(self, context: StateContext,
                 failure_reason: Optional[str] = None,
                 height_offset: float = 0.15,
                 hand_side: str = 'RIGHT',
                 timeout: float = 10.0,
                 position_threshold: float = 0.05):
        """
        Initialize the fallback state.

        Args:
            context: Shared state context
            failure_reason: Optional description of why fallback was triggered
            height_offset: Vertical offset above palm in meters (default: 0.15m = 15cm)
            hand_side: Which hand to track ('RIGHT' or 'LEFT')
            timeout: Maximum time to wait for movement completion (seconds)
            position_threshold: Distance threshold to consider "arrived" (meters)
        """
        super().__init__(context)
        self.failure_reason = failure_reason or "Hand occlusion detected"
        self.height_offset = height_offset
        self.hand_side = hand_side
        self.timeout = timeout
        self.position_threshold = position_threshold

        # Joint names for palm computation
        self.wrist_joint_name = f'{hand_side}_WRIST'
        self.handtip_joint_name = f'{hand_side}_HANDTIP'

        # Skeleton data provider
        self.skeleton_provider = SkeletonDataProvider(context.zed_receiver)

        # State tracking
        self.entry_time = 0.0
        self.palm_position = None
        self.target_position = None
        self.target_joints = None
        self._is_complete = False
        self._failed = False

    def enter(self):
        """
        Enter the fallback state, capture snapshot, and command movement.

        Steps:
        1. Log failure reason
        2. Capture skeleton snapshot from ZED
        3. Compute palm position
        4. Add vertical offset
        5. Solve IK
        6. Send movement command
        """
        self.entry_time = time.time()
        logger.warning("=" * 80)
        logger.warning("⚠️  ENTERING HANDOFF FALLBACK STATE")
        logger.warning(f"Reason: {self.failure_reason}")
        logger.warning("=" * 80)

        # Log data source information
        data_info = self.skeleton_provider.get_data_source_info()
        logger.info(f"Data source: {data_info['data_source']}")
        if data_info['use_static_data']:
            logger.info(
                f"Using static skeleton data with {data_info['static_joints_count']} joints")
        else:
            logger.info("Using live ZED skeleton data")

        # Capture skeleton snapshot and compute palm position
        if not self._capture_palm_position():
            logger.error("Failed to capture palm position - fallback aborted")
            self._failed = True
            return

        logger.info(f"Palm position captured: {self.palm_position}")
        logger.info(
            f"Target position (15cm above palm): {self.target_position}")

        # Solve IK to get target joint configuration
        if not self._solve_ik_for_target():
            logger.error("Failed to solve IK - fallback aborted")
            self._failed = True
            return

        logger.info("IK solution found, sending movement command...")

        # Send movement command
        try:
            self.context.commands.send(SetJoints(self.target_joints))
            logger.info("Movement command sent successfully")
        except Exception as e:
            logger.error(f"Failed to send movement command: {e}")
            self._failed = True
            return

        logger.info(
            f"Fallback movement initiated - waiting for completion (timeout: {self.timeout}s)")

    def execute(self):
        """
        Monitor movement progress and check for completion or timeout.
        """
        # If already failed, don't do anything
        if self._failed:
            return

        # Check if we've reached the target position
        if self._check_reached_target():
            logger.info("Reached target position above palm!")
            self._is_complete = True
            return

        # Check for timeout
        elapsed = time.time() - self.entry_time
        if elapsed > self.timeout:
            logger.warning(
                f"Fallback movement timeout ({self.timeout}s) - completing anyway")
            self._is_complete = True
            return

        # Log periodic status updates
        if int(elapsed) % 2 == 0 and int(elapsed) > 0:
            distance = self._get_distance_to_target()
            logger.info(
                f"Fallback active for {elapsed:.1f}s, distance to target: {distance*1000:.1f}mm")

    def is_complete(self) -> bool:
        """
        Check if fallback state is complete.

        Returns:
            True if movement complete, failed, or timeout reached
        """
        return self._is_complete or self._failed

    def exit(self):
        """Exit the fallback state."""
        elapsed = time.time() - self.entry_time
        logger.warning(f"Exiting HandoffFallbackState after {elapsed:.1f}s")

        if self._failed:
            logger.error("Fallback state exited with failure")
        else:
            logger.info("Fallback state completed successfully")

        if self.palm_position is not None:
            logger.info(f"  Final palm position: {self.palm_position}")
            logger.info(f"  Target position: {self.target_position}")

            # Log final distance to target
            distance = self._get_distance_to_target()
            logger.info(f"  Final distance to target: {distance*1000:.1f}mm")

    def _capture_palm_position(self) -> bool:
        """
        Capture skeleton snapshot and compute palm position.

        Returns:
            True if successful, False otherwise
        """
        frame_data = self.skeleton_provider.get_latest_frame()

        if not frame_data or not frame_data.skeletons:
            logger.error("No skeleton data available")
            return False

        # Get first skeleton (assume single person)
        skeleton = frame_data.skeletons[0]

        # Extract wrist and fingertip positions
        wrist_position = skeleton.get_joint_position(self.wrist_joint_name)
        handtip_position = skeleton.get_joint_position(self.handtip_joint_name)

        if wrist_position is None:
            logger.error(f"{self.wrist_joint_name} not detected in snapshot")
            return False

        if handtip_position is None:
            logger.error(f"{self.handtip_joint_name} not detected in snapshot")
            return False

        # Compute palm position as midpoint between wrist and fingertips
        wrist_pos = np.array(wrist_position)
        handtip_pos = np.array(handtip_position)
        palm_position = (wrist_pos + handtip_pos) / 2.0

        # Store palm position
        self.palm_position = palm_position

        # Compute target position: palm + vertical offset
        self.target_position = palm_position.copy()
        # Add vertical offset in Z-axis
        self.target_position[2] += self.height_offset

        logger.info(f"  Wrist: {wrist_pos}")
        logger.info(f"  Fingertip: {handtip_pos}")
        logger.info(f"  Palm (computed): {palm_position}")

        return True

    def _solve_ik_for_target(self) -> bool:
        """
        Solve IK to get joint configuration for target position.

        Returns:
            True if IK solution found, False otherwise
        """
        if self.target_position is None:
            logger.error("Target position not set")
            return False

        # Get current joint configuration
        current_joints = self.context.telemetry.get_current_joints()
        if current_joints is None:
            logger.error("Cannot get current joint configuration")
            return False

        # Solve IK for target position with facing-down orientation
        target_joints = self.context.ik.solve_XYZ(
            target_pos=self.target_position,
            current_q=current_joints,
            orientation=get_facing_down_orientation()
        )

        if target_joints is None:
            logger.error(
                f"IK solver failed for target position {self.target_position}")
            return False

        self.target_joints = list(target_joints)
        logger.info(f"IK solution: {self.target_joints}")
        return True

    def _check_reached_target(self) -> bool:
        """
        Check if robot has reached the target position.

        Returns:
            True if within position threshold, False otherwise
        """
        if self.target_position is None:
            return False

        distance = self._get_distance_to_target()
        return distance < self.position_threshold

    def _get_distance_to_target(self) -> float:
        """
        Get current distance to target position.

        Returns:
            Distance in meters, or infinity if cannot compute
        """
        if self.target_position is None:
            return float('inf')

        try:
            current_joints = self.context.telemetry.get_current_joints()
            if current_joints is None:
                return float('inf')

            tcp_matrix, tcp_pose = self.context.ik.tcp_from_joints(
                current_joints)
            current_position = np.array(tcp_pose[:3])

            distance = np.linalg.norm(current_position - self.target_position)
            return float(distance)
        except Exception as e:
            logger.error(f"Error computing distance to target: {e}")
            return float('inf')
