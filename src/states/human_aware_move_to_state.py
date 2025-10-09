"""
Human-Aware Move To State

Executes collision-free motion planning that avoids a tracked human.
Uses RRT-Connect path planning with rolling-horizon replanning and 
ISO/TS 15066 Speed and Separation Monitoring (SSM).

This state:
1. Plans collision-free trajectory avoiding the tracked person
2. Executes trajectory with continuous safety monitoring
3. Replans dynamically as the human moves
4. Applies speed scaling based on human proximity
5. Stops if no safe path exists

Usage:
    state = HumanAwareMoveToState(
        target_position=[x, y, z],
        target_orientation=rotation_matrix,  # Optional
        context=shared_context
    )
"""

import logging
import time
import numpy as np
from typing import List, Optional, Tuple, Dict, Any

from .base_state import BaseState
from .context import StateContext
from .states_enum import States

logger = logging.getLogger(__name__)


class HumanAwareMoveToState(BaseState):
    """
    State for human-aware motion planning and execution.

    Integrates with ZEDJointReceiver for real-time skeleton tracking
    and uses path planning to avoid collisions with the tracked person.
    """

    def __init__(self, context: StateContext, target_position: List[float],
                 target_orientation: Optional[np.ndarray] = None,
                 use_pre_approach: bool = True):
        """
        Initialize human-aware move state.

        Args:
            context: Shared context with robot controller, ZED receiver, etc.
            target_position: Target TCP position [x, y, z] in meters
            target_orientation: Optional 3x3 rotation matrix for TCP orientation
            use_pre_approach: Whether to use pre-approach waypoint
        """
        super().__init__(context=context)

        self.target_position = target_position
        self.target_orientation = target_orientation
        self.use_pre_approach = use_pre_approach

        # Planning components
        self.planner = None
        self.trajectory = None
        self.trajectory_metadata = None

        # Execution state
        self.waypoint_index = 0
        self.last_replan_time = 0
        self.last_human_position = None
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

    def enter(self):
        """Initialize planner and generate initial trajectory."""
        logger.info(f"Entering {self.name}")
        logger.info(f"Target: pos={self.target_position}, "
                    f"orientation={'specified' if self.target_orientation is not None else 'any'}")

        # Import here to avoid circular dependencies
        from kinematics.human_aware_path_planner import HumanAwarePathPlanner
        from config.config import PATH_PLANNING_CONFIG, URDF_FILEPATH

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

        # Check if human is detected
        frame_data = self.context.zed_receiver.get_latest_frame()
        person_detected = frame_data and len(frame_data.skeletons) > 0

        if person_detected:
            logger.info("Human detected - planning collision-free trajectory")
        else:
            logger.info("No human detected - planning direct trajectory")

        # Plan initial trajectory
        self._plan_trajectory(is_initial=True)

        if self.trajectory is None:
            if person_detected:
                logger.warning("No safe path found - person blocking target")
                self._enter_wait_mode()
            else:
                logger.error("Planning failed with no obstacles")
                self._failed = True
                self._failure_reason = "Initial planning failed"

    def execute(self):
        """Execute trajectory with dynamic replanning and safety monitoring."""
        if self.stopped_for_safety:
            self._handle_safety_stop()
            return

        if self.trajectory is None or self.waypoint_index >= len(self.trajectory):
            # Trajectory complete
            self._complete_motion()
            return

        # 1. Check if replanning is needed
        if self._should_replan():
            logger.info("Triggering replan due to human motion")
            self._plan_trajectory(is_initial=False)

            if self.trajectory is None:
                logger.warning("Replanning failed - stopping for safety")
                self._stop_for_safety()
                return

        # 2. Validate current trajectory segment is still safe
        remaining_trajectory = self.trajectory[self.waypoint_index:]
        is_safe, clearance = self.planner.check_trajectory_safe(
            remaining_trajectory)
        self.current_clearance = clearance

        if not is_safe:
            logger.warning(
                f"Trajectory unsafe (clearance: {clearance:.3f}m) - stopping!")
            self._stop_for_safety()
            return

        # 3. Compute speed scaling based on clearance
        speed_scale = self._compute_speed_scale(clearance)
        self.current_speed_scale = speed_scale

        if speed_scale < 0.1:
            logger.warning(f"Human very close ({clearance:.3f}m) - stopping")
            self._stop_for_safety()
            return

        # 4. Execute next waypoint
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

            # Log clearance info periodically
            if self.waypoint_index % 10 == 0:
                logger.debug(f"Waypoint {self.waypoint_index}/{len(self.trajectory)}, "
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
        logger.info(f"Motion statistics:")
        logger.info(f"  Replans: {self.replan_count}")
        logger.info(f"  Total planning time: {self.total_planning_time:.3f}s")
        logger.info(f"  Final clearance: {self.current_clearance:.3f}m")

        # Cleanup planner
        if self.planner:
            self.planner.cleanup()

    def _plan_trajectory(self, is_initial: bool = False):
        """Plan or replan trajectory to target."""
        start_time = time.time()

        # Get current robot state
        current_joints = self.context.telemetry.get_current_joints()

        # Compose target pose
        if self.target_orientation is not None:
            # Convert orientation to euler angles for planner
            from scipy.spatial.transform import Rotation as R
            euler = R.from_matrix(self.target_orientation).as_euler('xyz')
            target_pose = list(self.target_position) + list(euler)
        else:
            target_pose = self.target_position

        # Call planner
        try:
            self.trajectory, self.trajectory_metadata = self.planner.plan_trajectory(
                start_joints=current_joints,
                goal_pose=target_pose,
                use_pre_approach=self.use_pre_approach
            )

            planning_time = time.time() - start_time
            self.total_planning_time += planning_time

            if self.trajectory:
                self.waypoint_index = 0
                if not is_initial:
                    self.replan_count += 1

                logger.info(f"{'Initial plan' if is_initial else 'Replan'} successful: "
                            f"{len(self.trajectory)} waypoints, "
                            f"min clearance: {self.trajectory_metadata.get('min_clearance', 'N/A'):.3f}m, "
                            f"planning time: {planning_time:.3f}s")
            else:
                logger.warning(
                    f"Planning failed: {self.trajectory_metadata.get('reason', 'unknown')}")

        except Exception as e:
            logger.error(f"Planning exception: {e}")
            self.trajectory = None
            self.trajectory_metadata = {'success': False, 'reason': str(e)}

    def _should_replan(self) -> bool:
        """Determine if replanning is needed based on human motion."""
        from config.config import PATH_PLANNING_CONFIG

        # Rate limit replanning
        current_time = time.time()
        min_replan_interval = PATH_PLANNING_CONFIG.get(
            'min_replan_interval', 0.5)
        if current_time - self.last_replan_time < min_replan_interval:
            return False

        # Check if human moved significantly
        frame_data = self.context.zed_receiver.get_latest_frame()

        if not frame_data or not frame_data.skeletons:
            # No human - no need to replan
            return False

        skeleton = frame_data.skeletons[0]

        # Use torso position as reference for human movement
        torso_pos = skeleton.get_joint_position('PELVIS')
        if torso_pos is None:
            torso_pos = skeleton.get_joint_position('CHEST_SPINE')

        if torso_pos is None:
            return False  # Can't determine position

        # Check if moved beyond threshold
        if self.last_human_position is not None:
            movement = np.linalg.norm(
                np.array(torso_pos) - np.array(self.last_human_position)
            )
            threshold = PATH_PLANNING_CONFIG.get(
                'replan_threshold_position', 0.1)

            if movement > threshold:
                logger.debug(
                    f"Human moved {movement:.3f}m (threshold: {threshold:.3f}m)")
                self.last_replan_time = current_time
                self.last_human_position = torso_pos
                return True
        else:
            # First time tracking
            self.last_human_position = torso_pos

        return False

    def _compute_speed_scale(self, clearance: float) -> float:
        """
        Compute speed scaling factor based on human clearance.
        Implements Speed and Separation Monitoring (SSM) per ISO/TS 15066.

        Args:
            clearance: Minimum distance to human (meters)

        Returns:
            Speed scale factor [0.0, 1.0]
        """
        from config.config import PATH_PLANNING_CONFIG

        comfort_dist = PATH_PLANNING_CONFIG.get('comfort_distance', 0.5)
        warning_dist = PATH_PLANNING_CONFIG.get('warning_distance', 0.3)
        hard_min_dist = PATH_PLANNING_CONFIG.get('hard_min_distance', 0.15)

        if clearance >= comfort_dist:
            # Comfort zone - full speed
            return 1.0
        elif clearance >= warning_dist:
            # Warning zone - linear ramp from full to reduced
            # Interpolate between 1.0 (at comfort) and 0.5 (at warning)
            ratio = (clearance - warning_dist) / (comfort_dist - warning_dist)
            return 0.5 + 0.5 * ratio
        elif clearance >= hard_min_dist:
            # Critical zone - linear ramp from reduced to stop
            # Interpolate between 0.5 (at warning) and 0.1 (at hard_min)
            ratio = (clearance - hard_min_dist) / \
                (warning_dist - hard_min_dist)
            return 0.1 + 0.4 * ratio
        else:
            # Too close - stop
            return 0.0

    def _stop_for_safety(self):
        """Stop robot for safety and enter waiting mode."""
        logger.warning("SAFETY STOP - Human too close or trajectory unsafe")

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
        from config.config import PATH_PLANNING_CONFIG

        # Check if we've been waiting too long
        max_wait_time = PATH_PLANNING_CONFIG.get('max_safety_wait_time', 10.0)
        if time.time() - self.wait_start_time > max_wait_time:
            logger.error("Safety wait timeout - aborting motion")
            self._failed = True
            self._failure_reason = "Safety wait timeout"
            return

        # Check if human has moved away
        frame_data = self.context.zed_receiver.get_latest_frame()

        if not frame_data or not frame_data.skeletons:
            # Human left - try to replan
            logger.info("Human left workspace - attempting to resume")
            self._resume_from_safety_stop()
            return

        # Check clearance from current position
        current_joints = self.context.telemetry.get_current_joints()
        clearance = self.planner.get_clearance_to_human(current_joints)

        if clearance > PATH_PLANNING_CONFIG.get('warning_distance', 0.3):
            # Safe to resume - replan from current position
            logger.info(
                f"Human moved away (clearance: {clearance:.3f}m) - resuming")
            self._resume_from_safety_stop()
        else:
            # Still too close - continue waiting
            if int(time.time() - self.wait_start_time) % 2 == 0:  # Log every 2s
                logger.debug(
                    f"Waiting for human to move away (clearance: {clearance:.3f}m)")

    def _resume_from_safety_stop(self):
        """Resume motion after safety stop."""
        logger.info("Resuming motion after safety stop")
        self.stopped_for_safety = False
        self.wait_start_time = None

        # Replan from current position
        self._plan_trajectory(is_initial=False)

        if self.trajectory is None:
            logger.warning("Failed to replan after safety stop")
            self._stop_for_safety()

    def _enter_wait_mode(self):
        """Enter waiting mode when no path is available."""
        logger.info("Entering wait mode - no safe path available")
        self.stopped_for_safety = True
        self.wait_start_time = time.time()

    def _complete_motion(self):
        """Complete the motion successfully."""
        logger.info(f"Motion to target complete!")
        logger.info(f"  Final clearance: {self.current_clearance:.3f}m")
        logger.info(f"  Replans: {self.replan_count}")
        self._is_complete = True

    def is_complete(self) -> bool:
        """Check if the state has completed successfully or failed."""
        return self._is_complete or self._failed
