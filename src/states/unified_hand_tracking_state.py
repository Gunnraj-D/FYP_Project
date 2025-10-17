"""
Unified Hand Tracking State - Active hand tracking with robot movement control.
This state combines hand tracking with robot movement control using the command bus.
The robot moves toward the hand centroid with a specified height offset, and calculates
placement pose when the hand remains stable for the required duration.

Features:
- Intelligent hand occlusion detection using MediaPipe confidence scores
- 2-second grace period for temporary occlusions
- Automatic failure handling for persistent occlusion
"""
import logging
import time
import numpy as np
from typing import Optional

from kinematics.kinematics_solver import get_facing_down_orientation
from states.base_state import BaseState
from states.context import StateContext
from control.command_bus import SetJoints
from hand_detection.hand_detection_module import HandTracker
from hand_detection.hand_occlusion_detector import (
    HandOcclusionDetector,
    OcclusionConfig,
    OcclusionState as OcclusionStateEnum
)
from camera_management.camera_transform_module import transform_camera_to_base
from config import (
    HAND_STABILITY_TIME_THRESHOLD,
    HAND_STABILITY_THRESHOLD,
    HAND_OCCLUSION_CONFIG,
    DISTANCE_TO_REMAIN_M,
    CONTROL_ESTIMATED_LATENCY_S,
    MAX_POSITION_CHANGE_PER_CYCLE,
    MAX_AXIAL_CHANGE_PER_CYCLE,
    STAGED_FULL_SPEED_DISTANCE,
    STAGED_FINE_DISTANCE,
    VELOCITY_DAMPING_K,
    DEAD_ZONE_ENTRY_M,
    DEAD_ZONE_EXIT_M,
    STABLE_VELOCITY_THRESHOLD,
    MIN_VELOCITY_DT
)

logger = logging.getLogger(__name__)


class UnifiedHandTrackingState(BaseState):
    """
    Unified state for hand tracking with robot movement control.

    This state:
    1. Activates the HandTracker module for continuous hand detection
    2. Moves the robot toward the hand centroid using command bus
    3. Maintains DISTANCE_TO_REMAIN_M height above the hand
    4. Implements dead zone - no movement when within stability threshold
    5. Calculates placement pose when hand remains stable for threshold time
    6. Uses command bus for all robot movement commands
    """

    def __init__(self, context: StateContext):
        super().__init__(context)
        self.hand_tracker: Optional[HandTracker] = None
        self.state_start_time = 0.0
        self.hand_stable_start_time = 0.0
        self.is_hand_stable = False
        self.last_hand_position = None
        self.stability_check_interval = HAND_STABILITY_TIME_THRESHOLD / \
            20.0  # Check 20 times per threshold
        self.last_stability_check = 0.0
        self.last_movement_time = 0.0
        self.movement_interval = 0.1  # 10Hz movement updates

        # Predictive control attributes
        self.last_tcp_pose: Optional[np.ndarray] = None
        self.last_tcp_time: Optional[float] = None
        self.last_estimated_tcp_velocity: np.ndarray = np.zeros(3)

        # Occlusion detection
        occlusion_config = OcclusionConfig(**HAND_OCCLUSION_CONFIG)
        self.occlusion_detector = HandOcclusionDetector(occlusion_config)
        self.occlusion_failed = False

        # Single-move strategy: move once to deadzone center, wait, then check if reposition needed
        self.is_moving_to_target = False
        self.waiting_for_move_completion = False
        self.move_start_time = 0.0
        self.move_completion_wait_time = 1.0  # Wait 1 second for move to complete

    def enter(self):
        """Initialize hand tracker and reset state variables."""
        print("=" * 80)
        print("🎬 ENTER METHOD CALLED - UnifiedHandTrackingState")
        print("=" * 80)
        logger.info("🎬 Entering UnifiedHandTrackingState")

        try:
            # Initialize hand tracker
            print("Step 1: Creating HandTracker instance...")
            logger.info("Creating HandTracker instance...")

            self.hand_tracker = HandTracker(
                telemetry=self.context.telemetry,
                command_bus=self.context.commands,
                camera_manager=self.context.camera
            )
            print("Step 2: HandTracker instance created ✓")
            logger.info("HandTracker instance created")

            # Start hand tracking
            print("Step 3: Starting hand tracker...")
            logger.info("Starting hand tracker...")

            self.hand_tracker.start()

            print("Step 4: Hand tracker started ✓")
            logger.info("Hand tracker started")

            # Reset state variables
            print("Step 5: Resetting state variables...")
            self.state_start_time = time.time()
            self.hand_stable_start_time = 0.0
            self.is_hand_stable = False
            self.last_hand_position = None
            self.last_stability_check = 0.0
            self.last_movement_time = 0.0

            # Reset occlusion detector
            self.occlusion_detector.reset()
            self.occlusion_failed = False

            print("✅ UnifiedHandTrackingState initialized successfully")
            logger.info("✅ UnifiedHandTrackingState initialized successfully")

        except Exception as e:
            print(f"❌ EXCEPTION DURING INIT: {e}")
            logger.error(
                f"❌ Failed to initialize UnifiedHandTrackingState: {e}", exc_info=True)
            import traceback
            traceback.print_exc()
            raise

    def execute(self):
        """
        Single-move strategy:
        1. Check if hand is detected
        2. If outside deadzone and not currently moving, make ONE move to deadzone center
        3. Wait for move to complete
        4. Check if still in deadzone, if not, repeat step 2
        """
        current_time = time.time()

        # Throttle to 10Hz
        if current_time - self.last_stability_check < 0.1:
            return
        self.last_stability_check = current_time

        # Update occlusion detection
        occlusion_data = self.hand_tracker.get_occlusion_data()
        occlusion_status = self.occlusion_detector.update(
            confidence=occlusion_data['confidence'],
            landmark_count=occlusion_data['landmark_count'],
            hand_position=occlusion_data['hand_position']
        )

        # Update GUI with occlusion status
        self._update_gui_occlusion_status(occlusion_status)

        # Check for occlusion failure
        if occlusion_status.state == OcclusionStateEnum.OCCLUDED_FAILED:
            logger.warning(
                f"Hand occlusion detected: {occlusion_status.reason}")
            self.occlusion_failed = True
            return

        # Log temporary occlusion (informational)
        if occlusion_status.state == OcclusionStateEnum.TEMPORARILY_OCCLUDED:
            logger.info(
                f"Hand temporarily occluded: {occlusion_status.reason}")

        hand_position = self.context.telemetry.get_camera_vector()

        # Guard: invalid detection
        if hand_position is None or np.array_equal(hand_position, [0.0, 0.0, 0.0]):
            # No hand detected - reset stability flags
            self.is_hand_stable = False
            self.hand_stable_start_time = 0.0
            self.is_moving_to_target = False
            self.waiting_for_move_completion = False
            return

        # Update hand tracking and check deadzone status
        try:
            self._update_hand_tracking(hand_position, current_time)
        except Exception as e:
            logger.exception("Error in _update_hand_tracking: %s", e)
            return

        # Single-move logic
        if self.waiting_for_move_completion:
            # Wait for move to complete
            if current_time - self.move_start_time >= self.move_completion_wait_time:
                logger.info("Move completion wait finished, checking position")
                self.waiting_for_move_completion = False
                self.is_moving_to_target = False
            return

        # If not in deadzone and not currently moving, make a move
        if not self.is_hand_stable and not self.is_moving_to_target:
            try:
                self._make_single_move_to_deadzone(hand_position, current_time)
            except Exception as e:
                logger.exception(
                    "Error in _make_single_move_to_deadzone: %s", e)
                return

        # Always update placement pose calculation
        try:
            self._calculate_placement_pose(hand_position)
        except Exception:
            # Non-critical - log and continue
            logger.exception("_calculate_placement_pose failed")

    def _update_hand_tracking(self, hand_position, current_time: float):
        """
        Hysteretic dead-zone plus velocity check.
        - hand_position: reported in TCP frame (meters)
        - Uses DEAD_ZONE_ENTRY_M and DEAD_ZONE_EXIT_M for hysteresis.
        - Requires robot velocity magnitude < STABLE_VELOCITY_THRESHOLD to declare stable.
        """
        # Store live hand pose in telemetry
        live_hand_pose = hand_position + [0.0, 0.0, 0.0]
        self.context.telemetry.set_live_hand_pose(live_hand_pose)

        # Apply Z calibration offset (keep same as original)
        hand_pos_tcp = np.array(hand_position, dtype=float)
        hand_pos_tcp[2] -= 0.138  # Z offset

        # Target in TCP coordinates
        target_pos_tcp = np.array(
            [0.0, 0.0, DISTANCE_TO_REMAIN_M], dtype=float)

        distance_from_target = float(
            np.linalg.norm(hand_pos_tcp - target_pos_tcp))

        # Estimate robot velocity (base frame) for velocity-based stability requirement
        current_joints = self.context.telemetry.get_current_joints()
        if current_joints is not None:
            current_tcp_matrix, current_tcp_pose = self.context.ik.tcp_from_joints(
                current_joints)
            current_pos_base = np.array(current_tcp_pose[:3], dtype=float)

            last_pose = getattr(self, "last_tcp_pose", None)
            last_time = getattr(self, "last_tcp_time", None)
            robot_vel = self.__estimate_robot_velocity(
                current_pos_base, current_time, last_pose, last_time)

            # Update history
            self.last_tcp_pose = current_pos_base.copy()
            self.last_tcp_time = current_time
            self.last_estimated_tcp_velocity = robot_vel
        else:
            robot_vel = getattr(
                self, "last_estimated_tcp_velocity", np.zeros(3))

        robot_speed = float(np.linalg.norm(robot_vel))

        # Hysteresis thresholds from config
        entry = DEAD_ZONE_ENTRY_M
        exit_ = DEAD_ZONE_EXIT_M
        vel_thresh = STABLE_VELOCITY_THRESHOLD

        if self.is_hand_stable:
            # We are currently stable; require a tighter threshold to exit
            if distance_from_target > exit_ or robot_speed > vel_thresh:
                # Exited stable region
                self.is_hand_stable = False
                self.hand_stable_start_time = 0.0
                logger.debug(
                    "Hand exited stable zone: dist=%.4fm speed=%.4fm/s", distance_from_target, robot_speed)
            else:
                # Still within exit threshold and slow enough; check stable time
                if self.hand_stable_start_time == 0.0:
                    self.hand_stable_start_time = current_time
                # keep stable until timeout resets
        else:
            # Not currently stable; use larger entry threshold to capture
            if distance_from_target < entry and robot_speed < vel_thresh:
                # Entering stable
                self.is_hand_stable = True
                self.hand_stable_start_time = current_time
                logger.info("Hand entered stable zone: dist=%.4fm speed=%.4fm/s",
                            distance_from_target, robot_speed)
            else:
                # remain unstable
                self.is_hand_stable = False
                self.hand_stable_start_time = 0.0

        self.last_hand_position = hand_position.copy()

    def _transform_camera_to_tcp(self, camera_position, tcp_matrix):
        """Transform position from camera frame to TCP frame using hand-eye matrix."""
        try:
            # Use the calibrated hand-eye matrix to transform camera to TCP
            from camera_management.camera_transform_module import transform_camera_to_tcp_frame
            return transform_camera_to_tcp_frame(camera_position)
        except Exception as e:
            logger.error(f"Error transforming camera to TCP: {e}")
            return np.array([0.0, 0.0, 0.0])

    def _make_single_move_to_deadzone(self, hand_position, current_time: float):
        """
        Make a single move directly to the center of the deadzone.
        After sending command, wait for completion before making another move.
        """
        # Get current robot joints
        current_joints = self.context.telemetry.get_current_joints()
        if current_joints is None:
            logger.warning(
                "_make_single_move_to_deadzone: current_joints is None")
            return

        current_tcp_matrix, current_tcp_pose = self.context.ik.tcp_from_joints(
            current_joints)
        current_pos_base = np.array(current_tcp_pose[:3], dtype=float)

        # Apply Z calibration offset to hand (in TCP frame)
        hand_pos_tcp = np.array(hand_position, dtype=float)
        hand_pos_tcp[2] -= 0.138  # -138mm Z offset

        # Target position in TCP frame (center of deadzone = [0, 0, DISTANCE_TO_REMAIN_M])
        target_pos_tcp = np.array(
            [0.0, 0.0, DISTANCE_TO_REMAIN_M], dtype=float)

        # Compute offset in TCP frame
        tcp_offset_tcp = target_pos_tcp - hand_pos_tcp
        # Invert all axes (preserves original behavior)
        tcp_offset_tcp = -tcp_offset_tcp

        # Transform tcp_offset to base frame (vector)
        tcp_rotation = np.array(current_tcp_matrix[:3, :3], dtype=float)
        tcp_offset_in_base = tcp_rotation @ tcp_offset_tcp

        # Calculate desired target position in base frame
        desired_target_base = current_pos_base + tcp_offset_in_base

        distance = float(np.linalg.norm(tcp_offset_in_base))
        logger.info(
            f"🎯 Making single move to deadzone center (distance: {distance*1000:.1f}mm)")

        # Solve IK for target position
        target_position = desired_target_base.tolist()  # [x, y, z] only

        # Use comprehensive search with yaw freedom
        solutions = self.context.ik.solve_with_position_and_yaw_search(
            target_pos=target_position,
            current_q=current_joints,
            position_samples=5,      # Try 5 position perturbations
            yaw_samples=12,          # Try 12 yaw angles per position
            xy_perturbation=0.02,    # 2cm XY perturbation
            z_perturbation=0.03,     # 3cm Z perturbation
            max_iter=150,            # More iterations for better convergence
            tol=1e-3
        )

        if solutions:
            # Use the best solution (first in sorted list)
            best_solution, quality, perturbed_pos, yaw = solutions[0]
            target_joints = best_solution[:7]  # Take first 7 joints

            logger.info(
                f"✅ IK solution found (quality={quality:.3f}), sending move command")

            # Send the joint command
            self.context.commands.send(SetJoints(list(target_joints)))

            # Set flags to wait for move completion
            self.is_moving_to_target = True
            self.waiting_for_move_completion = True
            self.move_start_time = current_time
        else:
            logger.warning("❌ IK solve failed - no solutions found")
            self.is_moving_to_target = False

    def __estimate_robot_velocity(self, current_pos: np.ndarray, current_time: float, last_pos: Optional[np.ndarray], last_time: Optional[float]) -> np.ndarray:
        """
        Estimate robot TCP linear velocity (base frame) from two samples.
        Returns 3-vector (m/s). Robust to small dt by returning zeros if dt too small.
        """
        if last_pos is None or last_time is None:
            return np.zeros(3, dtype=float)
        dt = current_time - last_time
        if dt < MIN_VELOCITY_DT:
            return np.zeros(3, dtype=float)
        vel = (current_pos - last_pos) / dt
        # clamp absurd velocities to safe maximum (defensive programming)
        MAX_REASONABLE_VEL = 2.0  # m/s, extremely conservative
        vel = np.clip(vel, -MAX_REASONABLE_VEL, MAX_REASONABLE_VEL)
        return vel

    def __clamp_delta(self, delta: np.ndarray, max_total: float, max_axis: float) -> np.ndarray:
        """
        Clamp the position delta vector by total magnitude and per-axis magnitude.
        Returns clamped delta.
        """
        # Per-axis clamp
        delta = np.clip(delta, -abs(max_axis), abs(max_axis))
        mag = float(np.linalg.norm(delta))
        if mag > max_total:
            delta = delta * (max_total / mag)
        return delta

    def _calculate_placement_pose(self, hand_position):
        """Calculate final placement pose for object handoff using TCP-relative coordinates."""
        try:
            # Get current robot position for transformation
            current_joints = self.context.telemetry.get_current_joints()
            if current_joints is None or len(current_joints) != 7:
                logger.warning(
                    "Invalid current joints, cannot calculate placement pose")
                return

            # Calculate current TCP pose
            current_tcp_matrix, current_tcp_pose = self.context.ik.tcp_from_joints(
                current_joints)

            # hand_position is now already in TCP frame (from hand detection module)
            hand_pos_tcp = np.array(hand_position)

            # ========================================================================
            # TEMPORARY HACK: Compensate for calibration offsets
            # TODO: Remove after recalibrating hand-eye matrix with correct TCP
            # Hand-eye matrix already accounts for camera X/Y offset from TCP
            # Subtract 138mm Z offset (gripper extension)
            hand_pos_tcp[2] -= 0.138
            # ========================================================================

            # Get pickup height offset from telemetry
            pickup_height_offset = self.context.telemetry.get_pickup_height_offset()

            # Calculate placement pose in TCP frame: hand position + height offset
            placement_pose_tcp = hand_pos_tcp.copy()
            placement_pose_tcp[2] += pickup_height_offset

            # Convert TCP-relative placement pose to base frame
            # For placement, we're calculating where TCP should be, not chasing a target
            # So we transform the desired TCP-frame position to base frame directly
            placement_pose_homogeneous = np.concatenate(
                [placement_pose_tcp, [1.0]])
            placement_in_base_homogeneous = current_tcp_matrix @ placement_pose_homogeneous
            placement_pose_base = placement_in_base_homogeneous[:3]

            # Add rotation components using the standard facing-down orientation
            from scipy.spatial.transform import Rotation as R
            facing_down_matrix = get_facing_down_orientation()
            facing_down_rpy = R.from_matrix(facing_down_matrix).as_euler('xyz')

            # [x, y, z, rx, ry, rz] in radians
            full_placement_pose = list(
                placement_pose_base) + list(facing_down_rpy)

            # Store calculated handoff pose in telemetry
            self.context.telemetry.set_calculated_handoff_pose(
                full_placement_pose)

            logger.debug(f"Placement pose calculated: {full_placement_pose}")
            logger.debug(
                f"Hand (TCP): {hand_pos_tcp}, Height offset: {pickup_height_offset:.3f}m")

        except Exception as e:
            logger.error(f"Error calculating placement pose: {e}")

    def is_complete(self) -> bool:
        """
        Check if hand tracking is complete (hand stable for required duration).
        Also returns True if occlusion caused failure.
        """
        current_time = time.time()

        # Check for occlusion failure
        if self.occlusion_failed:
            logger.warning("Hand tracking failed due to occlusion")
            return True

        # Check if hand has been stable for the required duration
        if self.is_hand_stable:
            stable_duration = current_time - self.hand_stable_start_time
            if stable_duration >= HAND_STABILITY_TIME_THRESHOLD:
                logger.info(
                    f"Hand tracking complete - hand stable for {stable_duration:.1f}s")
                return True

        # Check timeout (safety measure)
        elapsed_time = current_time - self.state_start_time
        timeout_threshold = 20.0  # 20 seconds timeout
        if elapsed_time > timeout_threshold:
            logger.warning("Hand tracking timeout reached")
            return True

        return False

    def did_fail(self) -> bool:
        """
        Check if the hand tracking state failed due to occlusion.
        This can be used by the sequencer to determine if fallback is needed.
        """
        return self.occlusion_failed

    def exit(self):
        """Clean up hand tracker and log final results."""
        logger.info("Exiting UnifiedHandTrackingState")

        # Stop hand tracker
        if self.hand_tracker:
            self.hand_tracker.stop()
            logger.info("Hand tracker stopped")

        # Log final placement pose
        try:
            placement_pose = self.context.telemetry.get_calculated_handoff_pose()
            if placement_pose and placement_pose != [0.0] * 6:
                logger.info(f"Final placement pose: {placement_pose}")
            else:
                logger.warning("No valid placement pose was calculated")
        except Exception as e:
            logger.error(f"Error retrieving final placement pose: {e}")

        # Reset state variables
        self.hand_tracker = None
        self.state_start_time = 0.0
        self.hand_stable_start_time = 0.0
        self.is_hand_stable = False
        self.last_hand_position = None
        self.last_stability_check = 0.0
        self.last_movement_time = 0.0

        # Log occlusion statistics
        occlusion_stats = self.occlusion_detector.get_statistics()
        logger.info(f"Occlusion detection stats: {occlusion_stats}")

    def _update_gui_occlusion_status(self, occlusion_status):
        """Update the hand tracker GUI with current occlusion status."""
        try:
            # Map occlusion state to display text and color
            state_mapping = {
                OcclusionStateEnum.VISIBLE: ("TRACKING", (0, 255, 0)),  # Green
                # Orange
                OcclusionStateEnum.TEMPORARILY_OCCLUDED: ("OCCLUDED", (0, 165, 255)),
                # Red
                OcclusionStateEnum.OCCLUDED_FAILED: ("FAILED", (0, 0, 255)),
                OcclusionStateEnum.NO_HAND: ("NO HAND", (0, 0, 255))  # Red
            }

            status_text, status_color = state_mapping.get(
                occlusion_status.state, ("UNKNOWN", (128, 128, 128))
            )

            # Add reason to status text if available
            if hasattr(occlusion_status, 'reason') and occlusion_status.reason:
                status_text += f" ({occlusion_status.reason})"

            # Update hand tracker display
            self.hand_tracker.set_occlusion_status(
                status_text=status_text,
                status_color=status_color,
                time_in_state=occlusion_status.time_in_state
            )

        except Exception as e:
            logger.debug(f"Failed to update GUI occlusion status: {e}")

    def get_tracking_stats(self) -> dict:
        """Get statistics about the hand tracking process."""
        current_time = time.time()
        elapsed_time = current_time - self.state_start_time

        stable_duration = 0.0
        if self.is_hand_stable:
            stable_duration = current_time - self.hand_stable_start_time

        return {
            'elapsed_time': elapsed_time,
            'is_hand_stable': self.is_hand_stable,
            'stable_duration': stable_duration,
            'hand_detected': self.last_hand_position is not None,
            'current_hand_position': self.last_hand_position
        }
