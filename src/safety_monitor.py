#!/usr/bin/env python3
"""
Safety monitoring module for KUKA robot control system.
Provides comprehensive safety checks, collision detection, and emergency stop functionality.
"""

import logging
import time
import threading
from typing import List, Dict, Optional, Tuple
import numpy as np
from dataclasses import dataclass
from config import JOINT_LIMITS, MAX_JOINT_VELOCITY, MAX_JOINT_ACCELERATION, EMERGENCY_STOP_DISTANCE

logger = logging.getLogger(__name__)


@dataclass
class SafetyThresholds:
    """Configurable safety thresholds."""
    max_joint_velocity: float = MAX_JOINT_VELOCITY
    max_joint_acceleration: float = MAX_JOINT_ACCELERATION
    max_cartesian_velocity: float = 0.5  # m/s
    max_cartesian_acceleration: float = 1.0  # m/s²
    emergency_stop_distance: float = EMERGENCY_STOP_DISTANCE
    max_loop_overrun: float = 0.1  # seconds
    max_consecutive_errors: int = 5
    heartbeat_timeout: float = 2.0  # seconds


@dataclass
class SafetyStatus:
    """Current safety status information."""
    is_safe: bool = True
    emergency_stop_active: bool = False
    joint_limits_violated: bool = False
    velocity_limits_violated: bool = False
    acceleration_limits_violated: bool = False
    collision_detected: bool = False
    communication_timeout: bool = False
    last_safety_check: float = 0.0
    safety_violations: List[str] = None

    def __post_init__(self):
        if self.safety_violations is None:
            self.safety_violations = []


class SafetyMonitor:
    """Comprehensive safety monitoring system."""

    def __init__(self, thresholds: SafetyThresholds = None):
        self.thresholds = thresholds or SafetyThresholds()
        self.status = SafetyStatus()

        # Monitoring state
        self.is_monitoring = False
        self.monitor_thread = None
        self.monitor_interval = 0.01  # 10ms monitoring loop

        # Historical data for trend analysis
        self.joint_history = []
        self.velocity_history = []
        self.acceleration_history = []
        self.max_history_length = 100

        # Callbacks
        self.emergency_stop_callback = None
        self.safety_violation_callback = None

        # Thread safety
        self.lock = threading.Lock()

    def set_emergency_stop_callback(self, callback):
        """Set callback function for emergency stop events."""
        self.emergency_stop_callback = callback

    def set_safety_violation_callback(self, callback):
        """Set callback function for safety violation events."""
        self.safety_violation_callback = callback

    def start_monitoring(self):
        """Start the safety monitoring system."""
        if self.is_monitoring:
            logger.warning("Safety monitoring already active")
            return

        logger.info("Starting safety monitoring system...")
        self.is_monitoring = True
        self.monitor_thread = threading.Thread(
            target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
        logger.info("Safety monitoring started")

    def stop_monitoring(self):
        """Stop the safety monitoring system."""
        logger.info("Stopping safety monitoring system...")
        self.is_monitoring = False

        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=2.0)

        logger.info("Safety monitoring stopped")

    def _monitor_loop(self):
        """Main monitoring loop."""
        while self.is_monitoring:
            try:
                # Perform safety checks
                self._perform_safety_checks()

                # Update status
                with self.lock:
                    self.status.last_safety_check = time.time()

                # Sleep for monitoring interval
                time.sleep(self.monitor_interval)

            except Exception as e:
                logger.error(f"Error in safety monitoring loop: {e}")
                time.sleep(0.1)

    def _perform_safety_checks(self):
        """Perform all safety checks."""
        violations = []

        # Check joint limits
        if self._check_joint_limits():
            violations.append("joint_limits")

        # Check velocity limits
        if self._check_velocity_limits():
            violations.append("velocity_limits")

        # Check acceleration limits
        if self._check_acceleration_limits():
            violations.append("acceleration_limits")

        # Check for collisions
        if self._check_collision_detection():
            violations.append("collision")

        # Check communication health
        if self._check_communication_health():
            violations.append("communication")

        # Update safety status
        with self.lock:
            self.status.safety_violations = violations
            self.status.is_safe = len(violations) == 0

            # Check if emergency stop is needed
            if violations and not self.status.emergency_stop_active:
                self._trigger_emergency_stop(violations)

    def _check_joint_limits(self) -> bool:
        """Check if current joint positions violate limits."""
        if not self.joint_history:
            return False

        current_joints = self.joint_history[-1]

        for i, joint in enumerate(current_joints):
            if i < len(JOINT_LIMITS['min']):
                if joint < JOINT_LIMITS['min'][i] or joint > JOINT_LIMITS['max'][i]:
                    logger.warning(
                        f"Joint {i} limit violation: {joint} not in [{JOINT_LIMITS['min'][i]}, {JOINT_LIMITS['max'][i]}]")
                    with self.lock:
                        self.status.joint_limits_violated = True
                    return True

        with self.lock:
            self.status.joint_limits_violated = False
        return False

    def _check_velocity_limits(self) -> bool:
        """Check if joint velocities exceed limits."""
        if len(self.joint_history) < 2:
            return False

        current_joints = np.array(self.joint_history[-1])
        previous_joints = np.array(self.joint_history[-2])
        dt = self.monitor_interval

        velocities = np.abs(current_joints - previous_joints) / dt

        for i, vel in enumerate(velocities):
            if vel > self.thresholds.max_joint_velocity:
                logger.warning(
                    f"Joint {i} velocity violation: {vel:.3f} > {self.thresholds.max_joint_velocity}")
                with self.lock:
                    self.status.velocity_limits_violated = True
                return True

        with self.lock:
            self.status.velocity_limits_violated = False
        return False

    def _check_acceleration_limits(self) -> bool:
        """Check if joint accelerations exceed limits."""
        if len(self.joint_history) < 3:
            return False

        current_joints = np.array(self.joint_history[-1])
        previous_joints = np.array(self.joint_history[-2])
        prev_previous_joints = np.array(self.joint_history[-3])
        dt = self.monitor_interval

        current_velocities = (current_joints - previous_joints) / dt
        previous_velocities = (previous_joints - prev_previous_joints) / dt
        accelerations = np.abs(current_velocities - previous_velocities) / dt

        for i, acc in enumerate(accelerations):
            if acc > self.thresholds.max_joint_acceleration:
                logger.warning(
                    f"Joint {i} acceleration violation: {acc:.3f} > {self.thresholds.max_joint_acceleration}")
                with self.lock:
                    self.status.acceleration_limits_violated = True
                return True

        with self.lock:
            self.status.acceleration_limits_violated = False
        return False

    def _check_collision_detection(self) -> bool:
        """Check for potential collisions."""
        # This is a simplified collision detection
        # In a real system, you would implement more sophisticated collision detection
        # based on workspace geometry, obstacle detection, etc.

        # For now, we'll check if the robot is moving too fast in Cartesian space
        if len(self.joint_history) < 2:
            return False

        # Calculate approximate Cartesian velocity from joint changes
        current_joints = np.array(self.joint_history[-1])
        previous_joints = np.array(self.joint_history[-2])
        dt = self.monitor_interval

        # Rough estimate of Cartesian velocity (this is simplified)
        joint_velocity = np.linalg.norm(current_joints - previous_joints) / dt
        estimated_cartesian_velocity = joint_velocity * 0.1  # Rough conversion factor

        if estimated_cartesian_velocity > self.thresholds.max_cartesian_velocity:
            logger.warning(
                f"High Cartesian velocity detected: {estimated_cartesian_velocity:.3f} m/s")
            with self.lock:
                self.status.collision_detected = True
            return True

        with self.lock:
            self.status.collision_detected = False
        return False

    def _check_communication_health(self) -> bool:
        """Check communication health and timeouts."""
        current_time = time.time()

        # Check if we're receiving regular updates
        if self.joint_history:
            last_update = self.joint_history[-1][0] if isinstance(
                self.joint_history[-1], (list, np.ndarray)) else 0
            time_since_update = current_time - last_update

            if time_since_update > self.thresholds.heartbeat_timeout:
                logger.warning(
                    f"Communication timeout: {time_since_update:.2f}s since last update")
                with self.lock:
                    self.status.communication_timeout = True
                return True

        with self.lock:
            self.status.communication_timeout = False
        return False

    def _trigger_emergency_stop(self, violations: List[str]):
        """Trigger emergency stop due to safety violations."""
        logger.critical(
            f"EMERGENCY STOP TRIGGERED due to violations: {violations}")

        with self.lock:
            self.status.emergency_stop_active = True
            self.status.is_safe = False

        # Call emergency stop callback if set
        if self.emergency_stop_callback:
            try:
                self.emergency_stop_callback(violations)
            except Exception as e:
                logger.error(f"Error in emergency stop callback: {e}")

        # Call safety violation callback if set
        if self.safety_violation_callback:
            try:
                self.safety_violation_callback(violations)
            except Exception as e:
                logger.error(f"Error in safety violation callback: {e}")

    def update_joint_positions(self, joint_positions: List[float]):
        """Update current joint positions for monitoring."""
        if not isinstance(joint_positions, (list, np.ndarray)) or len(joint_positions) != 7:
            logger.warning(
                f"Invalid joint positions for safety monitoring: {joint_positions}")
            return

        current_time = time.time()

        with self.lock:
            # Add timestamp to joint data
            joint_data = [current_time] + list(joint_positions)
            self.joint_history.append(joint_data)

            # Keep history within limits
            if len(self.joint_history) > self.max_history_length:
                self.joint_history.pop(0)

            # Calculate velocities and accelerations
            if len(self.joint_history) >= 2:
                current_joints = np.array(self.joint_history[-1][1:])
                previous_joints = np.array(self.joint_history[-2][1:])
                dt = current_time - self.joint_history[-2][0]

                if dt > 0:
                    velocities = (current_joints - previous_joints) / dt
                    self.velocity_history.append(
                        [current_time] + list(velocities))

                    if len(self.velocity_history) > self.max_history_length:
                        self.velocity_history.pop(0)

            if len(self.joint_history) >= 3:
                current_joints = np.array(self.joint_history[-1][1:])
                previous_joints = np.array(self.joint_history[-2][1:])
                prev_previous_joints = np.array(self.joint_history[-3][1:])
                dt = current_time - self.joint_history[-2][0]

                if dt > 0:
                    current_velocities = (
                        current_joints - previous_joints) / dt
                    previous_velocities = (
                        previous_joints - prev_previous_joints) / dt
                    accelerations = (current_velocities -
                                     previous_velocities) / dt
                    self.acceleration_history.append(
                        [current_time] + list(accelerations))

                    if len(self.acceleration_history) > self.max_history_length:
                        self.acceleration_history.pop(0)

    def clear_emergency_stop(self):
        """Clear emergency stop condition."""
        logger.info("Clearing emergency stop condition")

        with self.lock:
            self.status.emergency_stop_active = False
            self.status.safety_violations.clear()

    def get_safety_status(self) -> SafetyStatus:
        """Get current safety status."""
        with self.lock:
            return SafetyStatus(
                is_safe=self.status.is_safe,
                emergency_stop_active=self.status.emergency_stop_active,
                joint_limits_violated=self.status.joint_limits_violated,
                velocity_limits_violated=self.status.velocity_limits_violated,
                acceleration_limits_violated=self.status.acceleration_limits_violated,
                collision_detected=self.status.collision_detected,
                communication_timeout=self.status.communication_timeout,
                last_safety_check=self.status.last_safety_check,
                safety_violations=self.status.safety_violations.copy()
            )

    def get_safety_statistics(self) -> Dict[str, float]:
        """Get safety monitoring statistics."""
        with self.lock:
            if not self.joint_history:
                return {}

            # Calculate statistics
            joint_data = np.array([joints[1:]
                                  for joints in self.joint_history])

            stats = {
                'total_checks': len(self.joint_history),
                'safety_violations': len(self.status.safety_violations),
                'max_joint_velocity': 0.0,
                'max_joint_acceleration': 0.0,
                'avg_loop_time': 0.0
            }

            if len(self.velocity_history) > 0:
                velocity_data = np.array([vels[1:]
                                         for vels in self.velocity_history])
                stats['max_joint_velocity'] = float(
                    np.max(np.abs(velocity_data)))

            if len(self.acceleration_history) > 0:
                acceleration_data = np.array(
                    [accs[1:] for accs in self.acceleration_history])
                stats['max_joint_acceleration'] = float(
                    np.max(np.abs(acceleration_data)))

            if len(self.joint_history) > 1:
                timestamps = [joints[0] for joints in self.joint_history]
                loop_times = np.diff(timestamps)
                stats['avg_loop_time'] = float(np.mean(loop_times))

            return stats

    def is_system_safe(self) -> bool:
        """Check if the system is currently safe."""
        with self.lock:
            return self.status.is_safe and not self.status.emergency_stop_active
