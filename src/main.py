#!/usr/bin/env python3
"""
Main application for KUKA robot hand tracking and control system.
This module provides a safe, robust interface between hand tracking and robot control.
"""

import logging
import time
import signal
import sys
from typing import Optional, Dict, Any
import numpy as np

from hand_detection_module import HandTracker
from FRI_Interface import KukaFRIInterface
from shared_state_joints import SharedState
from config import (
    FRI_IP, FRI_PORT, LOOP_RATE_MS, MAX_LOOP_OVERRUN_MS,
    URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS
)
from camera_transform_module import pose_to_homogeneous, transform_camera_to_base
from kinematics_solver import InverseKinematicsSolver

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('robot_control.log'),
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)


class RobotControlSystem:
    """Main robot control system with safety features and error handling."""

    def __init__(self):
        self.shared_state = SharedState()
        self.hand_tracker = None
        self.robot = None
        self.kinematics_solver = None

        # System state
        self.is_running = False
        self.emergency_stop_active = False
        self.system_status = "initializing"

        # Performance tracking
        self.loop_count = 0
        self.total_loop_time = 0.0
        self.max_loop_time = 0.0
        self.min_loop_time = float('inf')

        # Safety thresholds
        self.max_consecutive_errors = 5
        self.consecutive_errors = 0
        self.last_successful_loop = time.time()

        # Signal handling
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        """Handle system signals for graceful shutdown."""
        logger.info(f"Received signal {signum}, initiating shutdown...")
        self.stop()

    def initialize(self) -> bool:
        """Initialize all system components."""
        try:
            logger.info("Initializing robot control system...")

            # Initialize hand tracker
            logger.info("Initializing hand tracker...")
            self.hand_tracker = HandTracker(self.shared_state)

            # Initialize robot interface
            logger.info("Initializing robot interface...")
            self.robot = KukaFRIInterface(FRI_IP, FRI_PORT)

            # Initialize kinematics solver
            logger.info("Initializing kinematics solver...")
            self.kinematics_solver = InverseKinematicsSolver(
                URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS
            )

            self.system_status = "initialized"
            logger.info("System initialization completed successfully")
            return True

        except Exception as e:
            logger.error(f"System initialization failed: {e}")
            self.system_status = "initialization_failed"
            return False

    def start(self) -> bool:
        """Start the robot control system."""
        if not self.initialize():
            return False

        try:
            logger.info("Starting robot control system...")

            # Start hand tracking
            self.hand_tracker.start()
            logger.info("Hand tracking started")

            # Connect to robot
            if not self.robot.connect():
                logger.error("Failed to connect to robot")
                return False
            logger.info("Robot connected successfully")

            # Start main control loop
            self.is_running = True
            self.system_status = "running"
            logger.info("Robot control system started successfully")

            return True

        except Exception as e:
            logger.error(f"Failed to start system: {e}")
            self.system_status = "startup_failed"
            return False

    def stop(self):
        """Stop the robot control system safely."""
        logger.info("Stopping robot control system...")
        self.is_running = False
        self.system_status = "stopping"

        try:
            # Stop hand tracking
            if self.hand_tracker:
                self.hand_tracker.stop()
                logger.info("Hand tracking stopped")

            # Disconnect robot
            if self.robot:
                self.robot.disconnect()
                logger.info("Robot disconnected")

        except Exception as e:
            logger.error(f"Error during shutdown: {e}")

        self.system_status = "stopped"
        logger.info("Robot control system stopped")

    def emergency_stop(self):
        """Emergency stop the system."""
        logger.critical("EMERGENCY STOP ACTIVATED")
        self.emergency_stop_active = True
        self.is_running = False

        try:
            if self.robot:
                self.robot.emergency_stop()
            self.system_status = "emergency_stop"
        except Exception as e:
            logger.error(f"Error during emergency stop: {e}")

    def _validate_hand_detection(self, camera_vector: list) -> bool:
        """Validate hand detection data."""
        if not camera_vector or len(camera_vector) != 3:
            return False

        # Check for zero vector (no detection)
        if all(v == 0.0 for v in camera_vector):
            return False

        # Check for valid numeric values
        if not all(isinstance(v, (int, float)) and np.isfinite(v) for v in camera_vector):
            return False

        # Check reasonable bounds (in mm)
        if any(abs(v) > 5000 for v in camera_vector):
            return False

        return True

    def _validate_joint_angles(self, joint_angles: list) -> bool:
        """Validate joint angles before sending to robot."""
        if not joint_angles or len(joint_angles) != 7:
            return False

        # Check for valid numeric values
        if not all(isinstance(j, (int, float)) and np.isfinite(j) for j in joint_angles):
            return False

        # Check joint limits
        if not self.kinematics_solver.validate_joint_angles(joint_angles):
            return False

        return True

    def _check_safety_conditions(self) -> bool:
        """Check all safety conditions before proceeding."""
        # Check emergency stop
        if self.emergency_stop_active:
            return False

        # Check robot connection
        if not self.robot or not self.robot.check_connection():
            logger.warning("Robot not connected")
            return False

        # Check error count
        if self.consecutive_errors >= self.max_consecutive_errors:
            logger.error("Maximum consecutive errors exceeded")
            self.emergency_stop()
            return False

        # Check data freshness
        if not self.shared_state.is_data_fresh(max_age_seconds=2.0):
            logger.warning("Hand detection data is stale")
            return False

        return True

    def _execute_control_loop(self) -> bool:
        """Execute one iteration of the control loop."""
        try:
            # Get current hand position
            camera_vector = self.shared_state.get_camera_vector()

            # Validate hand detection
            if not self._validate_hand_detection(camera_vector):
                logger.debug("Invalid hand detection data")
                return False

            # Get current robot joint positions
            current_joint_angles = self.robot.get_joint_positions()
            if current_joint_angles is None:
                logger.warning("Failed to get current joint positions")
                return False

            # Solve forward kinematics for current TCP pose
            tcp_pose = self.kinematics_solver.solve_tcp(current_joint_angles)
            if tcp_pose is None:
                logger.warning("Failed to solve forward kinematics")
                return False

            # Transform camera coordinates to robot base frame
            hand_location_in_base = transform_camera_to_base(
                camera_vector, tcp_pose)
            if hand_location_in_base is None:
                logger.warning("Failed to transform coordinates")
                return False

            # Solve inverse kinematics for target joint angles
            target_joints = self.kinematics_solver.solve_XYZ(
                hand_location_in_base, current_joint_angles
            )
            if target_joints is None:
                logger.warning("Failed to solve inverse kinematics")
                return False

            # Validate target joint angles
            if not self._validate_joint_angles(target_joints):
                logger.warning("Invalid target joint angles")
                return False

            # Check velocity and acceleration limits
            dt = LOOP_RATE_MS / 1000.0
            if not self.kinematics_solver.check_joint_velocity(
                current_joint_angles, target_joints, dt
            ):
                logger.warning("Joint velocity exceeds limits")
                return False

            if not self.kinematics_solver.check_joint_acceleration(
                current_joint_angles, target_joints, dt
            ):
                logger.warning("Joint acceleration exceeds limits")
                return False

            # Send joint commands to robot
            if not self.robot.set_joint_positions(target_joints):
                logger.warning("Failed to set joint positions")
                return False

            # Update kinematics solver with current joints
            self.kinematics_solver.update_previous_joints(current_joint_angles)

            # Update shared state
            self.shared_state.update_target_joints(target_joints)

            # Reset error count on success
            self.consecutive_errors = 0
            self.last_successful_loop = time.time()

            return True

        except Exception as e:
            logger.error(f"Error in control loop: {e}")
            self.consecutive_errors += 1
            return False

    def _log_performance_metrics(self):
        """Log performance metrics periodically."""
        if self.loop_count % 100 == 0 and self.loop_count > 0:
            avg_loop_time = self.total_loop_time / self.loop_count
            logger.info(
                f"Performance: Avg={avg_loop_time:.3f}ms, "
                f"Min={self.min_loop_time:.3f}ms, "
                f"Max={self.max_loop_time:.3f}ms, "
                f"Errors={self.consecutive_errors}"
            )

    def run(self):
        """Main control loop."""
        if not self.start():
            logger.error("Failed to start system")
            return

        logger.info("System running. Press Ctrl+C to stop.")

        try:
            while self.is_running:
                loop_start_time = time.time()

                # Check safety conditions
                if not self._check_safety_conditions():
                    time.sleep(0.1)
                    continue

                # Execute control loop
                success = self._execute_control_loop()

                # Calculate loop timing
                loop_time = (time.time() - loop_start_time) * 1000
                self.total_loop_time += loop_time
                self.max_loop_time = max(self.max_loop_time, loop_time)
                self.min_loop_time = min(self.min_loop_time, loop_time)
                self.loop_count += 1

                # Log performance metrics
                self._log_performance_metrics()

                # Check for loop overrun
                if loop_time > LOOP_RATE_MS:
                    overrun = loop_time - LOOP_RATE_MS
                    if overrun > MAX_LOOP_OVERRUN_MS:
                        logger.warning(
                            f"Loop overrun: {overrun:.1f}ms exceeds limit {MAX_LOOP_OVERRUN_MS}ms"
                        )
                    else:
                        logger.debug(f"Loop overrun: {overrun:.1f}ms")

                # Timing control
                remaining_time = LOOP_RATE_MS - loop_time
                if remaining_time > 0:
                    time.sleep(remaining_time / 1000)

        except KeyboardInterrupt:
            logger.info("Received keyboard interrupt")
        except Exception as e:
            logger.error(f"Unexpected error in main loop: {e}")
            self.emergency_stop()
        finally:
            self.stop()


def main():
    """Main entry point."""
    try:
        # Create and run the control system
        control_system = RobotControlSystem()
        control_system.run()

    except Exception as e:
        logger.critical(f"Fatal error in main: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
