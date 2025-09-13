"""
Thread-safe shared state management for robot hand tracking system.
Provides synchronized access to system state across multiple threads.
"""
from threading import RLock
from typing import List, Dict, Optional
import time
import numpy as np
from dataclasses import dataclass, field

from config.config import HAND_STABILITY_THRESHOLD


@dataclass
class _ArmState:
    target_joints: List[float] = field(default_factory=lambda: [0.0] * 7)


@dataclass
class _GripperState:
    current_status: str = ""  # "open" | "close" or "" initially
    # target_status: str   # "open" | "close" or "" initially


@dataclass
class _HandTrackingState:
    last_update_time: float
    camera_vector: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  # in mm
    hand_radius: float = 0.0
    hand_detected: bool = False
    hand_stable_time: float = 0.0
    last_hand_position: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])


@dataclass
class _ObjectPickupState:
    pickup_pose_joints: List[float] = field(default_factory=lambda: [0.0] * 7)
    object_remaining_height: float = 0.0


@dataclass
class _RobotState:
    connected: bool = False
    status_code: int = -1
    error_message: Optional[str] = None
    emergency_stop: bool = False


class Telemetry:
    """Thread-safe container for shared system state with fine-grained locks."""

    def __init__(self):
        # Locks (fine-grained)
        self._arm_lock = RLock()
        self._gripper_lock = RLock()
        self._hand_tracking_lock = RLock()
        self._object_pickup_lock = RLock()
        self._robot_lock = RLock()

        # Arm
        self._arm = _ArmState()

        # Gripper
        self._gripper = _GripperState()

        # Perception
        now = time.time()

        self._hand_tracking = _HandTrackingState(
            last_update_time=now
        )

        self._object_pickup = _ObjectPickupState()

        # Robot
        self._robot = _RobotState()

    # === Joint Position Methods === #

    def get_current_joints(self) -> np.ndarray:
        """Get current joint positions as a numpy array of len 7."""
        with self._arm_lock:
            return np.array(self._arm.current_joints, dtype=float)

    def update_current_joints(self, joint_list: List[float]):
        """Update current joint positions from robot."""
        if len(joint_list) != 7:
            raise ValueError(f"Expected 7 joint values, got {len(joint_list)}")
        with self._arm_lock:
            self._arm.current_joints = list(joint_list)

    # === Gripper Methods === #

    def get_current_gripper_status(self) -> str:
        """Get current gripper status"""
        with self._gripper_lock:
            return self._gripper.current_status

    def update_current_gripper_status(self, status: str):
        """Update current gripper status"""
        if status not in ("open", "close"):
            raise ValueError(
                f"Gripper status is not valid. Attempted to update current gripper to status: {status}"
            )
        with self._gripper_lock:
            self._gripper.current_status = status

    def get_target_gripper_status(self) -> str:
        """Get target gripper status"""
        with self._gripper_lock:
            return self._gripper.target_status

    # === Camera and Handtracking Methods === #
    def get_camera_vector(self) -> List[float]:
        """Get current camera vector."""
        with self._hand_tracking_lock:
            return list(self._hand_tracking.camera_vector)

    def update_camera_vector(self, vector: List[float]):
        """Update camera vector with hand position; maintains simple stability timer."""
        now = time.time()
        with self._hand_tracking_lock:
            self._hand_tracking.camera_vector = list(vector)
            detected = (vector != [0.0, 0.0, 0.0])
            self._hand_tracking.hand_detected = detected

            if detected:
                distance = np.linalg.norm(
                    np.array(vector, dtype=float) -
                    np.array(self._hand_tracking.last_hand_position, dtype=float)
                )
                if distance < HAND_STABILITY_THRESHOLD:
                    self._hand_tracking.hand_stable_time += now - \
                        self._hand_tracking.last_update_time
                else:
                    self._hand_tracking.hand_stable_time = 0.0
                    self._hand_tracking.last_hand_position = list(vector)
            else:
                self._hand_tracking.hand_stable_time = 0.0

            self._hand_tracking.last_update_time = now

    def is_hand_stable(self, duration_s: float = 2.0) -> bool:
        """Check if hand has been stable for specified duration."""
        with self._hand_tracking_lock:
            return self._hand_tracking.hand_stable_time >= duration_s

    def update_radius(self, radius: float):
        """Update hand palm radius (stored as radius)."""
        with self._hand_tracking_lock:
            self._hand_tracking.hand_radius = radius / 2  # preserve existing behavior

    def get_hand_radius(self) -> float:
        """Get hand palm radius."""
        with self._hand_tracking_lock:
            return self._hand_tracking.hand_radius

    # === Robot status methods === #

    def get_pickup_pose_joints(self) -> List[float]:
        with self._object_pickup_lock:
            self._object_pickup.pickup_pose_joints

    def update_pickup_pose_joints(self, joint_list: List[float]):
        with self._object_pickup_lock:
            if len(joint_list) != 7:
                raise ValueError(
                    f"Expected 7 joint values, got {len(joint_list)}")

            self._object_pickup.pickup_pose_joints = joint_list

    def get_object_remaining_height(self):
        with self._object_pickup_lock:
            return self._object_pickup.object_remaining_height

    def update_object_remaining_height(self, remaining_height: float):
        with self._object_pickup_lock:
            self._object_pickup.object_remaining_height = remaining_height

    # === Robot status methods === #

    def update_robot_status(self, status: Dict):
        """Update robot status information."""
        with self._robot_lock:
            # Only update known fields
            if 'connected' in status:
                self._robot.connected = bool(status['connected'])
            if 'status_code' in status:
                self._robot.status_code = int(status['status_code'])
            if 'error_message' in status:
                self._robot.error_message = status['error_message']

    def get_robot_status(self) -> Dict:
        """Get robot status information."""
        with self._robot_lock:
            return {
                'connected': self._robot.connected,
                'status_code': self._robot.status_code,
                'error_message': self._robot.error_message
            }

    def set_emergency_stop(self, stop: bool):
        """Set emergency stop flag."""
        with self._robot_lock:
            self._robot.emergency_stop = bool(stop)

    def is_emergency_stop(self) -> bool:
        """Check if emergency stop is active."""
        with self._robot_lock:
            return self._robot.emergency_stop

    # === Utility Methods === #

    def get_full_state(self) -> Dict:
        """Get complete system state as dictionary."""
        with self._arm_lock, self._gripper_lock, self._hand_tracking_lock, self._robot_lock:
            return {
                'target_joints': list(self._arm.target_joints),
                'current_joints': list(self._arm.current_joints),
                'camera_vector': list(self._hand_tracking.camera_vector),
                'hand_radius': self._hand_tracking.hand_radius,
                'hand_detected': self._hand_tracking.hand_detected,
                'hand_stable_time': self._hand_tracking.hand_stable_time,
                'robot_status': {
                    'connected': self._robot.connected,
                    'status_code': self._robot.status_code,
                    'error_message': self._robot.error_message
                },
                'emergency_stop': self._robot.emergency_stop
            }
