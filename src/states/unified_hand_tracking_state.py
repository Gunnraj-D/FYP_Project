"""
Unified Hand Tracking State - Active hand tracking with robot movement control.
This state combines hand tracking with robot movement control using the command bus.
The robot moves toward the hand centroid with a specified height offset, and calculates
placement pose when the hand remains stable for the required duration.
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
from camera_management.camera_transform_module import transform_camera_to_base
from config import (
    HAND_STABILITY_TIME_THRESHOLD,
    HAND_STABILITY_THRESHOLD,
    DISTANCE_TO_REMAIN_M
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

    def enter(self):
        """Initialize hand tracker and reset state variables."""
        logger.info("Entering UnifiedHandTrackingState")

        try:
            # Initialize hand tracker
            self.hand_tracker = HandTracker(
                telemetry=self.context.telemetry,
                command_bus=self.context.commands,
                camera_manager=self.context.camera
            )

            # Start hand tracking
            self.hand_tracker.start()

            # Reset state variables
            self.state_start_time = time.time()
            self.hand_stable_start_time = 0.0
            self.is_hand_stable = False
            self.last_hand_position = None
            self.last_stability_check = 0.0
            self.last_movement_time = 0.0

            logger.info("UnifiedHandTrackingState initialized successfully")

        except Exception as e:
            logger.error(f"Failed to initialize UnifiedHandTrackingState: {e}")
            raise

    def execute(self):
        """Main execution loop for hand tracking and robot movement."""
        current_time = time.time()

        # Throttle stability checking
        if current_time - self.last_stability_check < self.stability_check_interval:
            return

        self.last_stability_check = current_time

        try:
            # Get current hand position from telemetry
            hand_position = self.context.telemetry.get_camera_vector()

            # Check if hand is detected
            if hand_position is not None and not np.array_equal(hand_position, [0.0, 0.0, 0.0]):
                self._update_hand_tracking(hand_position)
                self._move_robot_toward_hand(hand_position, current_time)
                self._calculate_placement_pose(hand_position)
            else:
                # Reset stability if no hand detected
                self.is_hand_stable = False
                self.hand_stable_start_time = 0.0
                logger.debug("No hand detected, resetting stability")

        except Exception as e:
            logger.error(f"Error in UnifiedHandTrackingState execution: {e}")

    def _update_hand_tracking(self, hand_position):
        """Update hand tracking state and check stability based on dead zone relative to TCP."""
        try:
            # Store live hand pose in telemetry
            # Convert from camera vector format to pose format [x, y, z, rx, ry, rz]
            live_hand_pose = hand_position + [0.0, 0.0, 0.0]
            self.context.telemetry.set_live_hand_pose(live_hand_pose)

            # hand_position is now already in TCP frame (from hand detection module)
            hand_pos_tcp = np.array(hand_position)

            # ========================================================================
            # TEMPORARY HACK: Compensate for tool0 vs tcp calibration mismatch
            # TODO: Remove after recalibrating hand-eye matrix with correct TCP
            #
            # Issue: Hand-eye calibration was done with tool0 (link 7) as TCP
            #        Now using actual tcp (link 9) = 138mm lower
            # Fix: Add 138mm Z offset to hand position in TCP frame
            hand_pos_tcp[2] -= 0.138  # Subtract 138mm gripper extension offset
            logger.debug(
                f"⚠️ TEMP: Applied -138mm Z offset for calibration mismatch (hand Z: {hand_pos_tcp[2]:.3f}m)")
            # ========================================================================

            # Dead zone is relative to TCP: hand should be at [0, 0, DISTANCE_TO_REMAIN_M] in TCP frame
            target_pos_tcp = np.array([0.0, 0.0, DISTANCE_TO_REMAIN_M])

            # Calculate 3D distance from hand to target position in TCP frame
            distance_from_target = np.linalg.norm(
                hand_pos_tcp - target_pos_tcp)

            # Use 2cm threshold for dead zone (can be adjusted)
            dead_zone_threshold = 0.02  # 2cm

            # Check if hand is within dead zone
            if distance_from_target < dead_zone_threshold:
                if not self.is_hand_stable:
                    self.is_hand_stable = True
                    self.hand_stable_start_time = time.time()
                    logger.info(
                        f"Hand entered dead zone (distance: {distance_from_target*1000:.1f}mm from TCP target)")
                # Hand remains stable - timer continues
            else:
                # Reset stability if hand exits dead zone
                if self.is_hand_stable:
                    logger.info(
                        f"Hand exited dead zone (distance: {distance_from_target*1000:.1f}mm from TCP target)")
                self.is_hand_stable = False
                self.hand_stable_start_time = 0.0

            self.last_hand_position = hand_position.copy()

        except Exception as e:
            logger.error(f"Error updating hand tracking: {e}")

    def _transform_camera_to_tcp(self, camera_position, tcp_matrix):
        """Transform position from camera frame to TCP frame using hand-eye matrix."""
        try:
            # Use the calibrated hand-eye matrix to transform camera to TCP
            from camera_management.camera_transform_module import transform_camera_to_tcp_frame
            return transform_camera_to_tcp_frame(camera_position)
        except Exception as e:
            logger.error(f"Error transforming camera to TCP: {e}")
            return np.array([0.0, 0.0, 0.0])

    def _move_robot_toward_hand(self, hand_position, current_time):
        """Move robot toward hand centroid using command bus."""
        # Stop moving if hand is in dead zone (stable)
        if self.is_hand_stable:
            logger.debug("Hand in dead zone - robot holding position")
            return

        # Throttle movement updates
        if current_time - self.last_movement_time < self.movement_interval:
            return

        try:
            # Get current robot position
            current_joints = self.context.telemetry.get_current_joints()
            if current_joints is None or len(current_joints) != 7:
                logger.warning("Invalid current joints, skipping movement")
                return

            # Calculate current TCP pose
            current_tcp_matrix, current_tcp_pose = self.context.ik.tcp_from_joints(
                current_joints)

            # hand_position is now already in TCP frame (from hand detection module)
            hand_pos_tcp = np.array(hand_position)

            # ========================================================================
            # TEMPORARY HACK: Compensate for tool0 vs tcp calibration mismatch
            # TODO: Remove after recalibrating hand-eye matrix with correct TCP
            hand_pos_tcp[2] -= 0.138  # Subtract 138mm gripper extension offset
            # ========================================================================

            # Target in TCP frame: hand should be at [0, 0, DISTANCE_TO_REMAIN_M]
            target_pos_tcp = np.array([0.0, 0.0, DISTANCE_TO_REMAIN_M])

            # Calculate distance to target in TCP frame
            distance_to_target_tcp = np.linalg.norm(
                hand_pos_tcp - target_pos_tcp)

            # Log TCP-relative distances for quality assessment
            logger.info(f"Hand (TCP): {hand_pos_tcp}")
            logger.info(f"Target (TCP): {target_pos_tcp}")
            logger.info(
                f"Distance to target (TCP): {distance_to_target_tcp*1000:.1f}mm")

            # Convert TCP-relative target to base frame for IK solving
            # The target in base frame is: current_TCP + (target_pos_tcp - hand_pos_tcp)
            tcp_offset = target_pos_tcp - hand_pos_tcp
            target_position_base = current_tcp_pose[:3] + tcp_offset

            # Clamp to workspace floor at z >= 0 (meters)
            workspace_floor_z = 0.0
            if target_position_base[2] < workspace_floor_z:
                logger.warning(
                    f"Clamping target Z from {target_position_base[2]:.3f} to workspace floor {workspace_floor_z:.3f}")
                target_position_base[2] = workspace_floor_z

            logger.info(f"Target (base): {target_position_base}")

            # Solve inverse kinematics for target position
            target_joints = self.context.ik.solve_XYZ(
                target_position_base, current_joints, get_facing_down_orientation())

            if target_joints is not None:
                # Send movement command via command bus
                self.context.commands.send(SetJoints(list(target_joints)))
                self.last_movement_time = current_time
                logger.debug(
                    f"Moving toward hand: target joints {target_joints}")
                logger.debug(
                    f"Distance to target (TCP): {distance_to_target_tcp:.3f}m")
            else:
                logger.warning(
                    "Failed to solve inverse kinematics for target position")

        except Exception as e:
            logger.error(f"Error in robot movement: {e}")

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
            # TEMPORARY HACK: Compensate for tool0 vs tcp calibration mismatch
            # TODO: Remove after recalibrating hand-eye matrix with correct TCP
            hand_pos_tcp[2] -= 0.138  # Subtract 138mm gripper extension offset
            # ========================================================================

            # Get pickup height offset from telemetry
            pickup_height_offset = self.context.telemetry.get_pickup_height_offset()

            # Calculate placement pose in TCP frame: hand position + height offset
            placement_pose_tcp = hand_pos_tcp.copy()
            placement_pose_tcp[2] += pickup_height_offset

            # Convert TCP-relative placement pose to base frame
            placement_pose_base = current_tcp_pose[:3] + \
                (placement_pose_tcp - hand_pos_tcp)

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
        """Check if hand tracking is complete (hand stable for required duration)."""
        current_time = time.time()

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
