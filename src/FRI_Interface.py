import numpy as np
import time
from typing import Optional, List
from jadeClient.clientJVM import stopJVM
from jadeClient.lbrJadeClientCallback import LBRJadeClientCallback
from jadeClient.lbrJadeClientThread import LBRJadeClientThread
from jadeController.jointController import JointController
from config import FRI_IP, FRI_PORT, LOOP_RATE_MS


class KukaFRIInterface:
    def __init__(self, robot_ip: str, port: int):
        self.robot_ip = robot_ip
        self.port = port
        self.client_thread = None
        self.callback = None
        self.controller = None
        self.is_connected = False
        self.current_joints = None

    def connect(self):
        """Connect to the robot"""
        if self.is_connected:
            print("Already connected!")
            return

        try:
            # Create callback and controller
            self.callback = LBRJadeClientCallback()
            self.controller = JointController()

            # Set up callback functions
            self.callback.setOnStateChange(self.controller.onStateChange)
            self.callback.setOnMonitor(self.controller.monitor)
            self.callback.setOnWaitForCommand(self.controller.waitForCommand)
            self.callback.setOnCommand(self.controller.command)

            # Create and start client thread
            self.client_thread = LBRJadeClientThread(self.robot_ip, self.port)
            self.client_thread.addClientCallback(self.callback)
            self.client_thread.start()

            # Wait for connection
            time.sleep(5.0)
            self.is_connected = True
            print(f"Connected to robot at {self.robot_ip}:{self.port}")

        except Exception as e:
            print(f"Failed to connect: {e}")
            self.is_connected = False
            raise

    def disconnect(self):
        """Disconnect from the robot"""
        if not self.is_connected:
            return

        if self.client_thread:
            self.client_thread.stop()
            self.client_thread.join(timeout=5.0)

        stopJVM()
        self.is_connected = False
        print("Disconnected from robot")

    def get_joint_positions(self) -> Optional[np.ndarray]:
        """
        Get current joint positions in radians

        Returns:
            numpy array of 7 joint positions or None if not connected
        """
        if not self.is_connected or not self.client_thread:
            print("Not connected to robot!")
            return None

        try:
            # Access the measured joint positions from the robot state
            if hasattr(self.client_thread.client, 'getRobotState'):
                robot_state = self.client_thread.client.getRobotState()
                if robot_state:
                    joints = robot_state.getMeasuredJointPosition()
                    self.current_joints = np.array(joints)
                    return self.current_joints.copy()
        except Exception as e:
            print(f"Error getting joint positions: {e}")

        return None

    def set_joint_positions(self, target_joints: List[float], blocking: bool = False, timeout: float = 30.0):
        """
        Set target joint positions

        Args:
            target_joints: List/array of 7 joint positions in radians
            blocking: If True, wait until motion is complete
            timeout: Maximum time to wait if blocking=True
        """
        if not self.is_connected or not self.controller:
            print("Not connected to robot!")
            return False

        if len(target_joints) != 7:
            print("Must provide exactly 7 joint values!")
            return False

        try:
            target_array = np.array(target_joints)
            self.controller.setTargetJointValues(target_array)
            # print(f"Set target joints: {target_array}")

            if blocking:
                return self._wait_for_motion_complete(timeout)

            return True

        except Exception as e:
            print(f"Error setting joint positions: {e}")
            return False

    def _wait_for_motion_complete(self, timeout: float) -> bool:
        """Wait until robot reaches target position"""
        start_time = time.time()

        while time.time() - start_time < timeout:
            current = self.get_joint_positions()
            if current is not None and self.controller.targetJointValues is not None:
                diff = np.abs(current - self.controller.targetJointValues)
                if np.all(diff < 0.001):  # 1mrad tolerance
                    print("Motion complete!")
                    return True
            time.sleep(0.01)

        print("Motion timeout!")
        return False

    def stop_motion(self):
        """Stop robot motion by setting current position as target"""
        current = self.get_joint_positions()
        if current is not None:
            self.set_joint_positions(current)
            print("Motion stopped")


if __name__ == "__main__":
    # Create interface
    robot = KukaFRIInterface(robot_ip=FRI_IP, port=FRI_PORT)

    try:
        # Connect to robot
        robot.connect()

        # Get current joint positions
        current_joints = robot.get_joint_positions()
        if current_joints is not None:
            print(f"Current joints: {current_joints}")

        # Move to a new position (small movement for safety)
        if current_joints is not None:
            target_joints = current_joints.copy()
            target_joints[0] += 0.1  # Move joint 1 by 0.1 radians

            print("Moving to new position...")
            success = robot.set_joint_positions(target_joints, blocking=True)

            if success:
                final_joints = robot.get_joint_positions()
                print(f"Final joints: {final_joints}")

    finally:
        # Always disconnect
        robot.disconnect()
