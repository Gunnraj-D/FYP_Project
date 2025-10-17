"""
Human Handoff Approach State

Moves robot toward the human's right hand using snapshot-based collision-aware path planning.
The target is computed once as ~30cm above the tracked right hand position using a stable snapshot.

This state:
1. Captures a stable snapshot of human skeleton from ZED tracking
2. Calculates approach position (offset above hand)
3. Plans trajectory once using human-aware path planning to avoid body collision
4. Executes pre-planned trajectory without replanning
5. Completes when trajectory execution is finished

Typical workflow:
    HumanHandoffApproachState → UnifiedHandTrackingState → HandoffState
"""

import logging
import time
import numpy as np
from typing import List, Optional, Tuple, Dict, Any

from .base_state import BaseState
from .context import StateContext
from .states_enum import States

logger = logging.getLogger(__name__)


class HumanHandoffApproachState(BaseState):
    """
    Approaches human's right hand with collision avoidance using snapshot-based planning.

    Captures a stable snapshot of human skeleton, plans trajectory once, and executes
    without dynamic replanning.
    """

    def __init__(self, context: StateContext,
                 approach_offset: List[float] = [0.0, 0.0, 0.30],
                 hand_side: str = 'RIGHT'):
        """
        Initialize human handoff approach state.

        Args:
            context: Shared context with robot controller, ZED receiver, etc.
            approach_offset: Offset from palm position [x, y, z] in meters
                           Default: [0, 0, 0.3] = 30cm above palm
            hand_side: Which hand to track ('RIGHT' or 'LEFT')
        """
        super().__init__(context=context)

        self.approach_offset = np.array(approach_offset)
        self.hand_side = hand_side
        self.wrist_joint_name = f'{hand_side}_WRIST'
        self.handtip_joint_name = f'{hand_side}_HANDTIP'

        # Planning components
        self.planner = None
        self.trajectory = None
        self.trajectory_metadata = None

        # Snapshot data (captured once)
        self.snapshot_hand_position = None
        self.snapshot_target_position = None
        self.snapshot_skeleton = None

        # Execution state
        self.waypoint_index = 0
        self._is_complete = False
        self._failed = False
        self._failure_reason = None

        # Statistics
        self.planning_time = 0
        self.trajectory_waypoints = 0

    def enter(self):
        """Capture snapshot, initialize planner, and plan trajectory once."""
        logger.info(f"Entering {self.name}")
        logger.info(
            f"Tracking {self.hand_side} hand (palm area) with offset {self.approach_offset}")

        # Check if ZED receiver is available
        if not self.context.zed_receiver:
            logger.error("ZED receiver not available - cannot track hand!")
            self._failed = True
            self._failure_reason = "ZED receiver not available"
            return

        # Capture stable snapshot of human skeleton
        if not self._capture_snapshot():
            logger.error("Cannot capture stable skeleton snapshot - aborting")
            self._failed = True
            self._failure_reason = "Snapshot capture failed"
            return

        logger.info(f"Snapshot hand position: {self.snapshot_hand_position}")
        logger.info(
            f"Snapshot target (approach) position: {self.snapshot_target_position}")

        # Import here to avoid circular dependencies
        from kinematics.human_aware_path_planner import HumanAwarePathPlanner
        from config import PATH_PLANNING_CONFIG, URDF_FILEPATH

        # Initialize planner
        try:
            self.planner = HumanAwarePathPlanner(
                urdf_path=str(URDF_FILEPATH),
                zed_receiver=self.context.zed_receiver,
                ik_solver=self.context.ik,
                config=PATH_PLANNING_CONFIG
            )
            logger.info("Path planner initialized")
        except Exception as e:
            logger.error(f"Failed to initialize path planner: {e}")
            self._failed = True
            self._failure_reason = "Planner initialization failed"
            return

        # Plan trajectory once to approach position
        self._plan_trajectory_to_snapshot_target()

        if self.trajectory is None:
            logger.error("Trajectory planning failed")
            self._failed = True
            self._failure_reason = "Planning failed"

    def execute(self):
        """Execute pre-planned trajectory without replanning."""
        # Check if trajectory is complete
        if self.trajectory is None or self.waypoint_index >= len(self.trajectory):
            logger.info("Trajectory execution complete!")
            self._complete_motion()
            return

        # Execute next waypoint
        next_waypoint = self.trajectory[self.waypoint_index]

        try:
            # Use OPC client to send joint commands
            from control.command_bus import SetJoints
            self.context.commands.send(SetJoints(next_waypoint))

            # Log progress periodically
            if self.waypoint_index % 10 == 0:
                progress_pct = (self.waypoint_index /
                                len(self.trajectory)) * 100
                logger.info(f"Waypoint {self.waypoint_index}/{len(self.trajectory)} "
                            f"({progress_pct:.0f}% complete)")

            self.waypoint_index += 1

        except Exception as e:
            logger.error(f"Error executing waypoint: {e}")
            self._failed = True
            self._failure_reason = f"Execution error: {e}"

    def exit(self):
        """Cleanup when leaving state."""
        logger.info(f"Exiting {self.name}")

        # Log statistics
        logger.info(f"Handoff approach statistics:")
        logger.info(f"  Planning time: {self.planning_time:.3f}s")
        logger.info(f"  Trajectory waypoints: {self.trajectory_waypoints}")
        logger.info(f"  Waypoints executed: {self.waypoint_index}")

        # Cleanup planner
        if self.planner:
            self.planner.cleanup()

    def _capture_snapshot(self) -> bool:
        """
        Capture a stable snapshot of the human skeleton.

        Computes palm position as midpoint between wrist and fingertips.

        Returns:
            True if snapshot captured successfully, False otherwise
        """
        frame_data = self.context.zed_receiver.get_latest_frame()

        if not frame_data or not frame_data.skeletons:
            logger.error("No skeleton data available")
            return False

        # Get first skeleton (assume single person)
        self.snapshot_skeleton = frame_data.skeletons[0]

        # Extract wrist and fingertip positions to compute palm
        wrist_position = self.snapshot_skeleton.get_joint_position(
            self.wrist_joint_name)
        handtip_position = self.snapshot_skeleton.get_joint_position(
            self.handtip_joint_name)

        if wrist_position is None:
            logger.error(f"{self.wrist_joint_name} not detected in snapshot")
            return False

        if handtip_position is None:
            logger.error(
                f"{self.handtip_joint_name} not detected in snapshot")
            return False

        # Compute palm position as midpoint between wrist and fingertips
        wrist_pos = np.array(wrist_position)
        handtip_pos = np.array(handtip_position)
        palm_position = (wrist_pos + handtip_pos) / 2.0

        # Store snapshot data
        self.snapshot_hand_position = palm_position

        # Calculate approach position (offset above palm)
        self.snapshot_target_position = self.snapshot_hand_position + self.approach_offset

        logger.info(
            f"Snapshot captured: {len(self.snapshot_skeleton.joints)} joints detected")
        logger.info(f"  Wrist: {wrist_pos}")
        logger.info(f"  Fingertip: {handtip_pos}")
        logger.info(f"  Palm (computed): {palm_position}")

        return True

    def _plan_trajectory_to_snapshot_target(self):
        """Plan trajectory to snapshot target position."""
        if self.snapshot_target_position is None:
            logger.error("No snapshot target position")
            return

        start_time = time.time()

        # Get current robot state
        current_joints = self.context.telemetry.get_current_joints()

        # Get current TCP orientation to extract Z-rotation
        from kinematics.kinematics_solver import get_facing_down_orientation_with_z_rotation
        from scipy.spatial.transform import Rotation as R

        # Get current TCP pose using IK solver
        _, current_tcp_pose = self.context.ik.tcp_from_joints(current_joints)
        current_orientation = R.from_euler('xyz', current_tcp_pose[3:])

        # Extract current Z-rotation (yaw) to maintain it
        current_euler = current_orientation.as_euler('xyz')
        current_z_rotation = current_euler[2]  # Yaw angle

        logger.info(
            f"Maintaining current Z-rotation: {np.degrees(current_z_rotation):.1f}°")

        # Create facing-down orientation with current Z-rotation preserved
        target_orientation = get_facing_down_orientation_with_z_rotation(
            current_z_rotation)

        # Convert to pose format
        euler = R.from_matrix(target_orientation).as_euler('xyz')
        target_pose = list(self.snapshot_target_position) + list(euler)

        # Call planner
        try:
            self.trajectory, self.trajectory_metadata = self.planner.plan_trajectory(
                start_joints=current_joints,
                goal_pose=target_pose,
                use_pre_approach=False  # Already at approach height
            )

            self.planning_time = time.time() - start_time

            if self.trajectory:
                self.trajectory_waypoints = len(self.trajectory)
                self.waypoint_index = 0

                logger.info(f"Trajectory planning successful: "
                            f"{self.trajectory_waypoints} waypoints, "
                            f"target at {self.snapshot_target_position}, "
                            f"planning time: {self.planning_time:.3f}s")
            else:
                logger.error(
                    f"Planning failed: {self.trajectory_metadata.get('reason', 'unknown')}")

        except Exception as e:
            logger.error(f"Planning exception: {e}")
            self.trajectory = None
            self.trajectory_metadata = {'success': False, 'reason': str(e)}

    def _complete_motion(self):
        """Complete the motion successfully."""
        logger.info(f"Handoff approach complete!")
        logger.info(f"  Snapshot palm position: {self.snapshot_hand_position}")
        logger.info(
            f"  Snapshot target position: {self.snapshot_target_position}")
        logger.info(
            f"  Waypoints executed: {self.waypoint_index}/{self.trajectory_waypoints}")
        self._is_complete = True

    def is_complete(self) -> bool:
        """Check if the state has completed successfully or failed."""
        return self._is_complete or self._failed
