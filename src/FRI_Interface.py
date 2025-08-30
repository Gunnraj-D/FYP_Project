import numpy as np
import time
import logging
from typing import Optional, List, Tuple
from jadeClient.clientJVM import stopJVM
from jadeClient.lbrJadeClientCallback import LBRJadeClientCallback
from jadeClient.lbrJadeClientThread import LBRJadeClientThread
from jadeController.jointController import JointController
from config import FRI_IP, FRI_PORT, LOOP_RATE_MS, FRI_CONNECTION_TIMEOUT, FRI_RETRY_ATTEMPTS

logger = logging.getLogger(__name__)


class KukaFRIInterface:
    def __init__(self, robot_ip: str, port: int):
        self.robot_ip = robot_ip
        self.port = port
        self.client_thread = None
        self.callback = None
        self.controller = None
        self.is_connected = False
        self.current_joints = None
        self.connection_attempts = 0
        self.last_connection_time = 0
        self.connection_timeout = FRI_CONNECTION_TIMEOUT
        self.max_retry_attempts = FRI_RETRY_ATTEMPTS

        # Safety and status tracking
        self.emergency_stop_active = False
        self.last_command_time = 0
        self.command_timeout = 5.0  # seconds
        self.system_status = "disconnected"

    def connect(self) -> bool:
        """Connect to the robot with retry logic"""
        if self.is_connected:
            logger.info("Already connected to robot")
            return True

        for attempt in range(self.max_retry_attempts):
            try:
                logger.info(
                    f"Attempting to connect to robot at {self.robot_ip}:{self.port} (attempt {attempt + 1}/{self.max_retry_attempts})")

                # Create callback and controller
                self.callback = LBRJadeClientCallback()
                self.controller = JointController()

                # Set up callback functions
                self.callback.setOnStateChange(self.controller.onStateChange)
                self.callback.setOnMonitor(self.controller.monitor)
                self.callback.setOnWaitForCommand(
                    self.controller.waitForCommand)
                self.callback.setOnCommand(self.controller.command)

                # Create and start client thread
                self.client_thread = LBRJadeClientThread(
                    self.robot_ip, self.port)
                self.client_thread.addClientCallback(self.callback)
                self.client_thread.start()

                # Wait for connection with timeout
                start_time = time.time()
                while not self._check_connection_status() and (time.time() - start_time) < self.connection_timeout:
                    time.sleep(0.1)

                if self._check_connection_status():
                    self.is_connected = True
                    self.connection_attempts = 0
                    self.last_connection_time = time.time()
                    self.system_status = "connected"
                    logger.info(
                        f"Successfully connected to robot at {self.robot_ip}:{self.port}")
                    return True
                else:
                    logger.warning(
                        f"Connection attempt {attempt + 1} timed out")
                    self._cleanup_connection()

            except Exception as e:
                logger.error(f"Connection attempt {attempt + 1} failed: {e}")
                self._cleanup_connection()

                if attempt < self.max_retry_attempts - 1:
                    wait_time = 2 ** attempt  # Exponential backoff
                    logger.info(f"Waiting {wait_time} seconds before retry...")
                    time.sleep(wait_time)

        logger.error(
            f"Failed to connect after {self.max_retry_attempts} attempts")
        self.system_status = "connection_failed"
        return False

    def _check_connection_status(self) -> bool:
        """Check if the connection is actually established"""
        try:
            if self.client_thread and hasattr(self.client_thread, 'client'):
                # Try to get robot state to verify connection
                robot_state = self.client_thread.client.getRobotState()
                return robot_state is not None
        except Exception:
            pass
        return False

    def _cleanup_connection(self):
        """Clean up connection resources"""
        try:
            if self.client_thread:
                self.client_thread.stop()
                self.client_thread.join(timeout=1.0)
        except Exception as e:
            logger.warning(f"Error during connection cleanup: {e}")

        self.client_thread = None
        self.callback = None
        self.controller = None
        self.is_connected = False

    def disconnect(self):
        """Disconnect from the robot"""
        if not self.is_connected:
            logger.info("Not connected to robot")
            return

        logger.info("Disconnecting from robot...")
        self._cleanup_connection()

        try:
            stopJVM()
        except Exception as e:
            logger.warning(f"Error stopping JVM: {e}")

        self.system_status = "disconnected"
        logger.info("Disconnected from robot")

    def check_connection(self) -> bool:
        """Check if currently connected"""
        return self.is_connected and self._check_connection_status()

    def get_joint_positions(self) -> Optional[np.ndarray]:
        """
        Get current joint positions in radians

        Returns:
            numpy array of 7 joint positions or None if not connected
        """
        if not self.is_connected or not self.client_thread:
            logger.warning("Not connected to robot")
            return None

        try:
            # Access the measured joint positions from the robot state
            if hasattr(self.client_thread.client, 'getRobotState'):
                robot_state = self.client_thread.client.getRobotState()
                if robot_state:
                    joints = robot_state.getMeasuredJointPosition()
                    if joints and len(joints) == 7:
                        self.current_joints = np.array(
                            joints, dtype=np.float64)
                        return self.current_joints.copy()
                    else:
                        logger.warning(
                            f"Invalid joint data received: {joints}")
        except Exception as e:
            logger.error(f"Error getting joint positions: {e}")

        return None

    def set_joint_positions(self, target_joints: List[float], blocking: bool = False, timeout: float = 30.0) -> bool:
        """
        Set target joint positions with safety checks

        Args:
            target_joints: List/array of 7 joint positions in radians
            blocking: If True, wait until motion is complete
            timeout: Maximum time to wait if blocking=True

        Returns:
            True if command was sent successfully, False otherwise
        """
        if not self.is_connected or not self.controller:
            logger.warning("Not connected to robot")
            return False

        # Validate input
        if not isinstance(target_joints, (list, np.ndarray)) or len(target_joints) != 7:
            logger.error(
                f"Invalid target joints: expected 7 values, got {target_joints}")
            return False

        # Check for emergency stop
        if self.emergency_stop_active:
            logger.warning("Emergency stop active - ignoring joint command")
            return False

        # Validate joint values are numeric
        try:
            target_joints = [float(j) for j in target_joints]
        except (ValueError, TypeError) as e:
            logger.error(f"Invalid joint values: {e}")
            return False

        try:
            # Send command to robot
            self.controller.setTargetJointPositions(target_joints)
            self.last_command_time = time.time()

            if blocking:
                return self._wait_for_motion_completion(timeout)

            return True

        except Exception as e:
            logger.error(f"Error setting joint positions: {e}")
            return False

    def _wait_for_motion_completion(self, timeout: float) -> bool:
        """Wait for motion to complete"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self._is_motion_complete():
                return True
            time.sleep(0.01)

        logger.warning(f"Motion completion timeout after {timeout} seconds")
        return False

    def _is_motion_complete(self) -> bool:
        """Check if motion is complete"""
        try:
            if self.controller and hasattr(self.controller, 'isMotionComplete'):
                return self.controller.isMotionComplete()
        except Exception:
            pass
        return True  # Assume complete if we can't check

    def emergency_stop(self):
        """Emergency stop the robot"""
        logger.warning("EMERGENCY STOP ACTIVATED")
        self.emergency_stop_active = True

        try:
            if self.controller and hasattr(self.controller, 'emergencyStop'):
                self.controller.emergencyStop()
        except Exception as e:
            logger.error(f"Error during emergency stop: {e}")

    def clear_emergency_stop(self):
        """Clear emergency stop condition"""
        if self.emergency_stop_active:
            logger.info("Clearing emergency stop condition")
            self.emergency_stop_active = False

            try:
                if self.controller and hasattr(self.controller, 'clearEmergencyStop'):
                    self.controller.clearEmergencyStop()
            except Exception as e:
                logger.error(f"Error clearing emergency stop: {e}")

    def get_robot_status(self) -> dict:
        """Get current robot status"""
        status = {
            'connected': self.is_connected,
            'system_status': self.system_status,
            'emergency_stop': self.emergency_stop_active,
            'last_command_time': self.last_command_time,
            'current_joints': self.current_joints.tolist() if self.current_joints is not None else None
        }
        return status

    def check_connection_health(self) -> bool:
        """Check if connection is healthy"""
        if not self.is_connected:
            return False

        # Check if we can still communicate
        try:
            joints = self.get_joint_positions()
            return joints is not None
        except Exception:
            return False
