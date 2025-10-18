"""
Human Handoff Approach State

Moves robot toward the human's right hand using collision-aware path planning.
The target is dynamically computed as ~30cm above the tracked right hand position.

This state:
1. Gets human hand position from ZED skeleton tracking
2. Calculates approach position (offset above hand)
3. Uses human-aware path planning to avoid body collision
4. Continuously updates target as hand moves
5. Completes when within threshold of hand approach position

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
    Approaches human's right hand with collision avoidance.

    Dynamically tracks hand position and plans collision-free path
    to approach position above the hand.
    """

    def __init__(self, context: StateContext,
                 approach_offset: List[float] = [0.0, 0.0, 0.15],
                 position_threshold: float = 0.05,
                 hand_joint_name: str = 'RIGHT_WRIST'):
        """
        Initialize human handoff approach state.

        Args:
            context: Shared context with robot controller, ZED receiver, etc.
            approach_offset: Offset from hand position [x, y, z] in meters
                           Default: [0, 0, 0.3] = 30cm above hand
            position_threshold: Distance threshold to consider "arrived" (meters)
            hand_joint_name: Which joint to track ('RIGHT_WRIST' or 'LEFT_WRIST')
        """
        super().__init__(context=context)

        self.approach_offset = np.array(approach_offset)
        self.position_threshold = position_threshold
        self.hand_joint_name = hand_joint_name

        # Finalization hold to avoid oscillatory replanning when robot is still catching up
        self._final_hold_active = False
        self._final_hold_start_time = 0.0
        self._final_hold_last_distance = float('inf')
        self._final_hold_max_time = 2.0  # seconds to allow robot to settle at last waypoint
        # Track last planned target and replan time for proper gating
        self.last_planned_target = None
        self.last_replan_time = 0.0

        # Planning components
        self.planner = None
        self.trajectory = None
        self.trajectory_metadata = None

        # Target tracking
        self.current_hand_position = None
        self.current_target_position = None
        self.last_target_update_time = 0

        # Execution state
        self.waypoint_index = 0
        self.last_replan_time = 0
        self._is_complete = False
        self._failed = False
        self._failure_reason = None

        # Safety state
        self.current_clearance = float('inf')
        self.current_speed_scale = 1.0
        self.stopped_for_safety = False
        self.wait_start_time = None

        # Statistics
        self.replan_count = 0
        self.total_planning_time = 0
        self.target_update_count = 0

    def enter(self):
        """Initialize planner and track initial hand position."""
        logger.info(f"Entering {self.name}")
        logger.info(
            f"Tracking {self.hand_joint_name} with offset {self.approach_offset}")

        # Check if ZED receiver is available
        if not self.context.zed_receiver:
            logger.error("ZED receiver not available - cannot track hand!")
            self._failed = True
            self._failure_reason = "ZED receiver not available"
            return

        # Get initial hand position
        if not self._update_target_from_hand():
            logger.error("Cannot detect human hand - aborting")
            self._failed = True
            self._failure_reason = "Hand not detected"
            return

        logger.info(f"Initial hand position: {self.current_hand_position}")
        logger.info(
            f"Initial target (approach) position: {self.current_target_position}")

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

        # Plan initial trajectory to approach position
        self._plan_trajectory_to_target(is_initial=True)

        if self.trajectory is None:
            logger.error("Initial planning failed")
            self._failed = True
            self._failure_reason = "Initial planning failed"

    def execute(self):
        """Execute trajectory with dynamic target updates and replanning."""
        if self.stopped_for_safety:
            self._handle_safety_stop()
            return

        # 1. Update target position from hand tracking
        target_moved = self._update_target_from_hand()

        if not target_moved and self.current_target_position is None:
            logger.warning("Lost hand tracking")
            self._stop_for_safety()
            return

        # 2. Check if we've reached the target
        if self._check_if_reached_target():
            logger.info("Reached approach position!")
            self._complete_motion()
            return

        # 3. Check if target moved significantly (hand moved)
        if self._should_replan_for_target_update():
            logger.info(
                "Hand moved significantly - replanning to new position")
            self._plan_trajectory_to_target(is_initial=False)

            if self.trajectory is None:
                logger.warning("Replanning failed - stopping for safety")
                self._stop_for_safety()
                return

        # 4. Execute current trajectory
        if self.trajectory is None or self.waypoint_index >= len(self.trajectory):
            # Trajectory complete - check if we've reached target
            if self._check_if_reached_target():
                logger.info("Trajectory complete and target reached!")
                self._complete_motion()
                return

            # Not at target yet - check if target moved significantly
            if self._should_replan_for_target_update():
                logger.info("Trajectory complete, target moved - replanning")
                self._plan_trajectory_to_target(is_initial=False)
                return
            else:
                # Target hasn't moved much - but we haven't reached it yet.
                # Before replanning, hold at the final waypoint and allow the robot to physically catch up.
                current_tcp = self._get_current_tcp_position()
                distance = np.linalg.norm(
                    current_tcp - self.current_target_position) if self.current_target_position is not None else float('inf')

                # Initialize final hold on first pass
                if not self._final_hold_active:
                    logger.info(
                        f"Trajectory complete, target stable but not reached (distance: {distance:.3f}m) - holding to settle")
                    self._final_hold_active = True
                    self._final_hold_start_time = time.time()
                    self._final_hold_last_distance = distance
                    # Re-send last waypoint to ensure controller is targeting it
                    if self.trajectory and len(self.trajectory) > 0:
                        from control.command_bus import SetJoints
                        last_wp = self.trajectory[-1]
                        self.context.commands.send(SetJoints(last_wp))
                    return

                # During hold: check if we reached or improved
                hold_elapsed = time.time() - self._final_hold_start_time
                if distance <= self.position_threshold:
                    logger.info(
                        "Final hold: target reached during settle window")
                    self._complete_motion()
                    return

                # If improving, keep holding
                if distance < self._final_hold_last_distance - 0.005:  # 5mm improvement
                    self._final_hold_last_distance = distance
                    return

                # Timeout: proceed to replan, but gate by min interval or target delta
                if hold_elapsed >= self._final_hold_max_time:
                    from config import PATH_PLANNING_CONFIG
                    min_interval = PATH_PLANNING_CONFIG.get(
                        'min_replan_interval', 1.0)
                    delta_for_replan = PATH_PLANNING_CONFIG.get(
                        'replan_threshold_position', 0.08)
                    now = time.time()
                    if self.last_planned_target is None:
                        self.last_planned_target = self.current_target_position.copy()
                    target_delta = np.linalg.norm(
                        self.current_target_position - self.last_planned_target)

                    if (now - self.last_replan_time) >= min_interval or target_delta >= delta_for_replan:
                        logger.info(
                            f"Final hold timeout ({hold_elapsed:.1f}s), replanning (distance: {distance:.3f}m, Δtarget={target_delta:.3f}m)")
                        self._final_hold_active = False
                        self._plan_trajectory_to_target(is_initial=False)
                        self.last_planned_target = self.current_target_position.copy()
                        self.last_replan_time = now
                        return
                    # Else, resend final waypoint to induce small adjustment and keep holding
                    if self.trajectory and len(self.trajectory) > 0:
                        from control.command_bus import SetJoints
                        self.context.commands.send(
                            SetJoints(self.trajectory[-1]))
                    return
                # Keep holding otherwise
                return

        # 5. Validate current trajectory segment is still safe
        remaining_trajectory = self.trajectory[self.waypoint_index:]
        is_safe, clearance = self.planner.check_trajectory_safe(
            remaining_trajectory)
        self.current_clearance = clearance

        if not is_safe:
            logger.warning(
                f"Trajectory unsafe (clearance: {clearance:.3f}m) - human moved into path!")
            logger.info("Replanning to avoid new obstacle position...")

            # Try to replan around the obstacle
            self._plan_trajectory_to_target(is_initial=False)

            if self.trajectory is None:
                # Replanning failed - no safe path exists
                logger.error("Cannot find safe path around human - stopping!")
                self._stop_for_safety()
                return
            else:
                # Successfully replanned - reset to start of new trajectory
                logger.info(
                    f"Replanned around obstacle: {len(self.trajectory)} waypoints")
                self.waypoint_index = 0
                return

        # 6. Compute speed scaling based on clearance
        speed_scale = self._compute_speed_scale(clearance)
        self.current_speed_scale = speed_scale

        if speed_scale < 0.1:
            logger.warning(f"Human very close ({clearance:.3f}m) - stopping")
            self._stop_for_safety()
            return

        # 7. Execute next waypoint
        next_waypoint = self.trajectory[self.waypoint_index]

        try:
            # Use OPC client to send joint commands
            from control.command_bus import SetJoints
            self.context.commands.send(SetJoints(next_waypoint))
            success = True  # Command sent successfully

            if not success:
                logger.error("Failed to execute waypoint")
                self._failed = True
                self._failure_reason = "Waypoint execution failed"
                return

            # Log progress periodically
            if self.waypoint_index % 10 == 0:
                distance_to_target = np.linalg.norm(
                    self._get_current_tcp_position() - self.current_target_position
                )
                logger.debug(f"Waypoint {self.waypoint_index}/{len(self.trajectory)}, "
                             f"distance to target: {distance_to_target:.3f}m, "
                             f"clearance: {clearance:.3f}m, speed: {speed_scale*100:.0f}%")

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
        logger.info(f"  Target updates: {self.target_update_count}")
        logger.info(f"  Replans: {self.replan_count}")
        logger.info(f"  Total planning time: {self.total_planning_time:.3f}s")
        logger.info(f"  Final clearance: {self.current_clearance:.3f}m")

        # Cleanup planner
        if self.planner:
            self.planner.cleanup()

        # Reset final hold flags
        self._final_hold_active = False
        self._final_hold_start_time = 0.0
        self._final_hold_last_distance = float('inf')

    def _update_target_from_hand(self) -> bool:
        """
        Update target position from current hand tracking data.

        Returns:
            True if hand was detected and target updated, False otherwise
        """
        # Use SkeletonDataProvider to get data (static or live)
        from hand_detection.skeleton_data_provider import SkeletonDataProvider
        skeleton_provider = SkeletonDataProvider(self.context.zed_receiver)
        frame_data = skeleton_provider.get_latest_frame()

        if not frame_data or not frame_data.skeletons:
            logger.debug("No skeleton data available")
            return False

        skeleton = frame_data.skeletons[0]  # Single person
        hand_position = skeleton.get_joint_position(self.hand_joint_name)

        if hand_position is None:
            logger.debug(f"{self.hand_joint_name} not detected")
            return False

        # Update hand position
        self.current_hand_position = np.array(hand_position)

        # Calculate approach position (offset above hand)
        new_target = self.current_hand_position + self.approach_offset

        # Check if this is a significant update
        if self.current_target_position is not None:
            movement = np.linalg.norm(
                new_target - self.current_target_position)
            if movement > 0.01:  # More than 1cm movement
                logger.debug(f"Hand moved {movement:.3f}m")
                self.target_update_count += 1

        self.current_target_position = new_target
        self.last_target_update_time = time.time()

        return True

    def _should_replan_for_target_update(self) -> bool:
        """Check if target moved enough to warrant replanning."""
        from config import PATH_PLANNING_CONFIG

        # Rate limit replanning
        current_time = time.time()
        min_replan_interval = PATH_PLANNING_CONFIG.get(
            'min_replan_interval', 0.5)
        if current_time - self.last_replan_time < min_replan_interval:
            return False

        # Check if we have a previous trajectory
        if self.trajectory is None:
            return True

        # Get target position at time of last plan
        # If hand has moved significantly, replan
        # For now, replan if we've received multiple target updates
        # TODO: Could be more sophisticated - track target velocity, predict position
        # With smoothing enabled, we can tolerate more updates before replanning

        # Replan every 10 hand updates (was 5)
        return self.target_update_count > 10

    def _check_if_reached_target(self) -> bool:
        """Check if robot has reached the approach position."""
        if self.current_target_position is None:
            return False

        current_tcp = self._get_current_tcp_position()
        distance = np.linalg.norm(current_tcp - self.current_target_position)

        reached = distance < self.position_threshold

        if reached:
            logger.info(
                f"Reached target! Distance: {distance:.3f}m (threshold: {self.position_threshold:.3f}m)")

        return reached

    def _get_current_tcp_position(self) -> np.ndarray:
        """Get current TCP position from forward kinematics."""
        current_joints = self.context.telemetry.get_current_joints()
        tcp_matrix, tcp_pose = self.context.ik.tcp_from_joints(current_joints)
        return np.array(tcp_pose[:3])  # [x, y, z]

    def _plan_trajectory_to_target(self, is_initial: bool = False):
        """Plan trajectory to current target position."""
        if self.current_target_position is None:
            logger.error("No target position to plan to")
            return

        start_time = time.time()

        # Get current robot state
        current_joints = self.context.telemetry.get_current_joints()

        # IMPROVED: Use position-only target to exploit multi-yaw IK search
        # This allows the IK solver to try multiple yaw angles and find the best
        # reachable configuration, dramatically improving convergence success rate.
        # The solver will automatically maintain facing-down orientation while
        # searching through yaw redundancy (12 yaw samples × 5 position samples = 60 IK trials).
        target_position = list(self.current_target_position)  # [x, y, z] only

        logger.debug(
            f"Planning to position {target_position} with yaw freedom for robust IK")

        # Call planner with position-only target
        try:
            self.trajectory, self.trajectory_metadata = self.planner.plan_trajectory(
                start_joints=current_joints,
                goal_pose=target_position,  # 3-DOF position only - exploits yaw redundancy!
                use_pre_approach=False  # Already at approach height
            )

            planning_time = time.time() - start_time
            self.total_planning_time += planning_time

            if self.trajectory:
                self.waypoint_index = 0
                if not is_initial:
                    self.replan_count += 1
                    self.target_update_count = 0  # Reset counter after replan

                logger.info(f"{'Initial plan' if is_initial else 'Replan'} successful: "
                            f"{len(self.trajectory)} waypoints, "
                            f"target at {self.current_target_position}, "
                            f"planning time: {planning_time:.3f}s")
            else:
                logger.warning(
                    f"Planning failed: {self.trajectory_metadata.get('reason', 'unknown')}")

        except Exception as e:
            logger.error(f"Planning exception: {e}")
            self.trajectory = None
            self.trajectory_metadata = {'success': False, 'reason': str(e)}

    def _compute_speed_scale(self, clearance: float) -> float:
        """Compute speed scaling based on clearance (SSM)."""
        from config import PATH_PLANNING_CONFIG

        comfort_dist = PATH_PLANNING_CONFIG.get('comfort_distance', 0.5)
        warning_dist = PATH_PLANNING_CONFIG.get('warning_distance', 0.3)
        hard_min_dist = PATH_PLANNING_CONFIG.get('hard_min_distance', 0.15)

        if clearance >= comfort_dist:
            return 1.0
        elif clearance >= warning_dist:
            ratio = (clearance - warning_dist) / (comfort_dist - warning_dist)
            return 0.5 + 0.5 * ratio
        elif clearance >= hard_min_dist:
            ratio = (clearance - hard_min_dist) / \
                (warning_dist - hard_min_dist)
            return 0.1 + 0.4 * ratio
        else:
            return 0.0

    def _stop_for_safety(self):
        """Stop robot for safety."""
        logger.warning("SAFETY STOP - Unsafe condition detected")

        try:
            # Stop robot by sending current position as target
            current_joints = self.context.telemetry.get_current_joints()
            from control.command_bus import SetJoints
            self.context.commands.send(SetJoints(current_joints))
            self.stopped_for_safety = True
            self.wait_start_time = time.time()
        except Exception as e:
            logger.error(f"Failed to stop robot: {e}")
            self._failed = True
            self._failure_reason = "Safety stop failed"

    def _handle_safety_stop(self):
        """Handle state when stopped for safety."""
        from config import PATH_PLANNING_CONFIG

        # Check if we've been waiting too long
        max_wait_time = PATH_PLANNING_CONFIG.get('max_safety_wait_time', 10.0)
        if time.time() - self.wait_start_time > max_wait_time:
            logger.error("Safety wait timeout - aborting")
            self._failed = True
            self._failure_reason = "Safety wait timeout"
            return

        # Check clearance from current position
        current_joints = self.context.telemetry.get_current_joints()
        clearance = self.planner.get_clearance_to_human(current_joints)

        if clearance > PATH_PLANNING_CONFIG.get('warning_distance', 0.3):
            logger.info(f"Clearance improved ({clearance:.3f}m) - resuming")
            self.stopped_for_safety = False
            self.wait_start_time = None
            # Will replan on next execute

    def _complete_motion(self):
        """Complete the motion successfully."""
        logger.info(f"Handoff approach complete!")
        logger.info(f"  Final hand position: {self.current_hand_position}")
        logger.info(
            f"  Final TCP position: {self._get_current_tcp_position()}")
        logger.info(f"  Final clearance: {self.current_clearance:.3f}m")
        logger.info(f"  Replans: {self.replan_count}")
        self._is_complete = True

    def is_complete(self) -> bool:
        """Check if the state has completed successfully or failed."""
        return self._is_complete or self._failed
