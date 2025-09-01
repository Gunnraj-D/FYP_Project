"""
Thread-safe shared state management for robot hand tracking system.
Provides synchronized access to system state across multiple threads.
"""
from threading import Lock
from typing import List, Dict, Optional, Tuple
import time
import numpy as np


class SharedState:
    """Thread-safe container for shared system state."""
    
    def __init__(self):
        # Joint positions (7 DOF for KUKA iiwa)
        self.target_joints = [0.0] * 7
        self.current_joints = [0.0] * 7
        
        # Camera/hand tracking data
        self.camera_vector = [0.0, 0.0, 0.0]  # x, y, z in mm
        self.hand_radius = 0.0  # Hand palm radius in mm
        self.hand_detected = False
        self.hand_stable_time = 0.0  # Time hand has been stable
        self.last_hand_position = [0.0, 0.0, 0.0]
        
        # Robot status
        self.robot_status = {
            'connected': False,
            'status_code': -1,
            'error_message': None
        }
        
        # System mode and control
        self.system_mode = "IDLE"  # IDLE, TRACKING, PICKUP, PLACE
        self.emergency_stop = False
        
        # Object manipulation state
        self.object_detected = False
        self.object_grasped = False
        self.grasp_pose = None
        
        # Thread safety
        self.lock = Lock()
        
        # Timing
        self.last_update_time = time.time()
        
    # Camera/Hand tracking methods
    def update_camera_vector(self, vector: List[float]):
        """Update camera vector with hand position."""
        with self.lock:
            self.camera_vector = list(vector)
            self.hand_detected = (vector != [0.0, 0.0, 0.0])
            
            # Check hand stability
            if self.hand_detected:
                distance = np.linalg.norm(
                    np.array(vector) - np.array(self.last_hand_position)
                )
                if distance < 10.0:  # Within 10mm
                    self.hand_stable_time += time.time() - self.last_update_time
                else:
                    self.hand_stable_time = 0.0
                    self.last_hand_position = list(vector)
            else:
                self.hand_stable_time = 0.0
                
            self.last_update_time = time.time()
            
    def get_camera_vector(self) -> List[float]:
        """Get current camera vector."""
        with self.lock:
            return self.camera_vector.copy()
            
    def update_radius(self, radius: float):
        """Update hand palm radius."""
        with self.lock:
            self.hand_radius = radius / 2  # Store as actual radius
            
    def get_hand_radius(self) -> float:
        """Get hand palm radius."""
        with self.lock:
            return self.hand_radius
            
    def is_hand_stable(self, duration_s: float = 2.0) -> bool:
        """Check if hand has been stable for specified duration."""
        with self.lock:
            return self.hand_stable_time >= duration_s
            
    # Joint position methods
    def update_target_joints(self, joint_list: List[float]):
        """Update target joint positions."""
        with self.lock:
            if len(joint_list) == 7:
                self.target_joints = list(joint_list)
            else:
                raise ValueError(f"Expected 7 joint values, got {len(joint_list)}")
                
    def get_target_joints(self) -> List[float]:
        """Get target joint positions."""
        with self.lock:
            return self.target_joints.copy()
            
    def update_current_joints(self, joint_list: List[float]):
        """Update current joint positions from robot."""
        with self.lock:
            if len(joint_list) == 7:
                self.current_joints = list(joint_list)
            else:
                raise ValueError(f"Expected 7 joint values, got {len(joint_list)}")
                
    def get_current_joints(self) -> List[float]:
        """Get current joint positions."""
        with self.lock:
            return self.current_joints.copy()
            
    # Robot status methods
    def update_robot_status(self, status: Dict):
        """Update robot status information."""
        with self.lock:
            self.robot_status.update(status)
            
    def get_robot_status(self) -> Dict:
        """Get robot status information."""
        with self.lock:
            return self.robot_status.copy()
            
    def is_robot_connected(self) -> bool:
        """Check if robot is connected."""
        with self.lock:
            return self.robot_status.get('connected', False)
            
    # System control methods
    def set_system_mode(self, mode: str):
        """Set system operating mode."""
        valid_modes = ["IDLE", "TRACKING", "PICKUP", "PLACE"]
        with self.lock:
            if mode in valid_modes:
                self.system_mode = mode
            else:
                raise ValueError(f"Invalid mode: {mode}")
                
    def get_system_mode(self) -> str:
        """Get current system mode."""
        with self.lock:
            return self.system_mode
            
    def set_emergency_stop(self, stop: bool):
        """Set emergency stop flag."""
        with self.lock:
            self.emergency_stop = stop
            
    def is_emergency_stop(self) -> bool:
        """Check if emergency stop is active."""
        with self.lock:
            return self.emergency_stop
            
    # Object manipulation methods
    def set_object_detected(self, detected: bool):
        """Set object detection status."""
        with self.lock:
            self.object_detected = detected
            
    def is_object_detected(self) -> bool:
        """Check if object is detected."""
        with self.lock:
            return self.object_detected
            
    def set_object_grasped(self, grasped: bool):
        """Set object grasp status."""
        with self.lock:
            self.object_grasped = grasped
            
    def is_object_grasped(self) -> bool:
        """Check if object is grasped."""
        with self.lock:
            return self.object_grasped
            
    def set_grasp_pose(self, pose: Optional[np.ndarray]):
        """Set grasp pose for object manipulation."""
        with self.lock:
            self.grasp_pose = pose
            
    def get_grasp_pose(self) -> Optional[np.ndarray]:
        """Get grasp pose."""
        with self.lock:
            return self.grasp_pose.copy() if self.grasp_pose is not None else None
            
    # Utility methods
    def get_full_state(self) -> Dict:
        """Get complete system state as dictionary."""
        with self.lock:
            return {
                'target_joints': self.target_joints.copy(),
                'current_joints': self.current_joints.copy(),
                'camera_vector': self.camera_vector.copy(),
                'hand_radius': self.hand_radius,
                'hand_detected': self.hand_detected,
                'hand_stable_time': self.hand_stable_time,
                'robot_status': self.robot_status.copy(),
                'system_mode': self.system_mode,
                'emergency_stop': self.emergency_stop,
                'object_detected': self.object_detected,
                'object_grasped': self.object_grasped,
                'has_grasp_pose': self.grasp_pose is not None
            }
            
    def reset_hand_tracking(self):
        """Reset hand tracking state."""
        with self.lock:
            self.camera_vector = [0.0, 0.0, 0.0]
            self.hand_radius = 0.0
            self.hand_detected = False
            self.hand_stable_time = 0.0
            self.last_hand_position = [0.0, 0.0, 0.0]
            
    def reset_object_state(self):
        """Reset object manipulation state."""
        with self.lock:
            self.object_detected = False
            self.object_grasped = False
            self.grasp_pose = None