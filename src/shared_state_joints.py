from threading import Lock
import numpy as np
from typing import List, Optional, Tuple
import logging
import time

logger = logging.getLogger(__name__)

class SharedState:
    def __init__(self):
        self.target_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.radius = 0
        self.camera_vector = [0, 0, 0]
        self.lock = Lock()
        
        # Safety and status tracking
        self.system_status = "initialized"
        self.last_update_time = 0.0
        self.error_count = 0
        self.max_errors = 10
        
        # Hand tracking quality metrics
        self.hand_confidence = 0.0
        self.hand_detection_stable = False
        self.consecutive_detections = 0
        self.min_stable_detections = 5

    def update_camera_vector(self, vector: List[float], confidence: float = 0.0):
        """Update camera vector with validation"""
        if not isinstance(vector, (list, np.ndarray)) or len(vector) != 3:
            logger.warning(f"Invalid camera vector format: {vector}")
            return False
            
        # Validate vector values
        if any(not isinstance(v, (int, float)) for v in vector):
            logger.warning(f"Invalid camera vector values: {vector}")
            return False
            
        with self.lock:
            self.camera_vector = vector.copy()
            self.hand_confidence = confidence
            self.last_update_time = time.time()
            
            # Track detection stability
            if confidence > 0.5:  # Good detection
                self.consecutive_detections += 1
                if self.consecutive_detections >= self.min_stable_detections:
                    self.hand_detection_stable = True
            else:
                self.consecutive_detections = 0
                self.hand_detection_stable = False
                
        return True

    def update_target_joints(self, joint_list: List[float]):
        """Update target joints with validation"""
        if not isinstance(joint_list, (list, np.ndarray)) or len(joint_list) != 7:
            logger.warning(f"Invalid joint list format: {joint_list}")
            return False
            
        # Validate joint values
        if any(not isinstance(j, (int, float)) for j in joint_list):
            logger.warning(f"Invalid joint values: {joint_list}")
            return False
            
        with self.lock:
            self.target_joints = joint_list.copy()
            self.last_update_time = time.time()
        return True

    def update_radius(self, radius: float):
        """Update radius with validation"""
        if not isinstance(radius, (int, float)) or radius < 0:
            logger.warning(f"Invalid radius value: {radius}")
            return False
            
        with self.lock:
            self.radius = radius / 2
            self.last_update_time = time.time()
        return True

    def get_camera_vector(self) -> List[float]:
        """Get camera vector with safety check"""
        with self.lock:
            # Check if detection is stable enough
            if not self.hand_detection_stable:
                return [0, 0, 0]
            return self.camera_vector.copy()

    def get_target_joints(self) -> List[float]:
        """Get target joints"""
        with self.lock:
            return self.target_joints.copy()

    def get_radius(self) -> float:
        """Get radius"""
        with self.lock:
            return self.radius

    def get_system_status(self) -> str:
        """Get current system status"""
        with self.lock:
            return self.system_status

    def set_system_status(self, status: str):
        """Set system status"""
        with self.lock:
            self.system_status = status
            self.last_update_time = time.time()

    def increment_error_count(self) -> bool:
        """Increment error count and return True if max errors exceeded"""
        with self.lock:
            self.error_count += 1
            if self.error_count >= self.max_errors:
                self.system_status = "error_limit_exceeded"
                return True
        return False

    def reset_error_count(self):
        """Reset error count"""
        with self.lock:
            self.error_count = 0

    def is_hand_detection_stable(self) -> bool:
        """Check if hand detection is stable"""
        with self.lock:
            return self.hand_detection_stable

    def get_hand_confidence(self) -> float:
        """Get current hand detection confidence"""
        with self.lock:
            return self.hand_confidence

    def get_last_update_time(self) -> float:
        """Get last update time"""
        with self.lock:
            return self.last_update_time

    def is_data_fresh(self, max_age_seconds: float = 1.0) -> bool:
        """Check if data is fresh (updated within max_age_seconds)"""
        with self.lock:
            return (time.time() - self.last_update_time) < max_age_seconds
