import numpy as np
import time
from typing import Optional, List


class MockKukaFRIInterface:
    """Simple mock that mimics the real FRI interface"""
    
    def __init__(self, robot_ip: str = "mock", port: int = 30200):
        self.robot_ip = robot_ip
        self.port = port
        self.is_connected = False
        
        # Simulated robot state
        self.current_joints = np.array([0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0])  # Home position
        self.target_joints = self.current_joints.copy()
        self.joint_velocities = np.zeros(7)
        
        # Simulated cartesian pose [x, y, z, rx, ry, rz] (position + rotation)
        self.cartesian_pose = np.array([0.5, 0.0, 0.8, 0.0, 0.0, 0.0])
        
        # Movement simulation
        self.movement_speed = 0.1  # rad/s for joints
        self.last_update_time = time.time()
        
    def connect(self):
        """Mock connection"""
        print(f"Mock: Connected to robot at {self.robot_ip}:{self.port}")
        self.is_connected = True
        time.sleep(0.1)  # Simulate connection delay
        
    def disconnect(self):
        """Mock disconnection"""
        if self.is_connected:
            print("Mock: Disconnected from robot")
            self.is_connected = False
    
    def get_joint_positions(self) -> Optional[np.ndarray]:
        """Return current joint positions with simulated movement"""
        if not self.is_connected:
            return None
        
        self._simulate_movement()
        return self.current_joints.copy()
    
    def set_joint_positions(self, target_joints: List[float], blocking: bool = False, timeout: float = 30.0):
        """Set target joint positions"""
        if not self.is_connected:
            return False
            
        if len(target_joints) != 7:
            print("Mock: Must provide exactly 7 joint values!")
            return False
        
        self.target_joints = np.array(target_joints)
        # print(f"Mock: Set target joints: {self.target_joints}")
        
        if blocking:
            # Simulate time to reach target
            max_diff = np.max(np.abs(self.target_joints - self.current_joints))
            sim_time = max_diff / self.movement_speed
            time.sleep(min(sim_time, timeout))
            self.current_joints = self.target_joints.copy()
        
        return True
    
    def get_joint_velocities(self) -> Optional[np.ndarray]:
        """Return simulated joint velocities"""
        if not self.is_connected:
            return None
        return self.joint_velocities.copy()
    
    def get_cartesian_pose(self) -> Optional[np.ndarray]:
        """Return simulated cartesian pose"""
        if not self.is_connected:
            return None
        
        # Simple simulation: cartesian pose changes with joint 1
        self.cartesian_pose[0] = 0.5 + 0.3 * np.sin(self.current_joints[0])
        self.cartesian_pose[1] = 0.3 * np.cos(self.current_joints[0])
        
        return self.cartesian_pose.copy()
    
    def _simulate_movement(self):
        """Simulate gradual movement toward target"""
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Simulate movement toward target
        joint_diff = self.target_joints - self.current_joints
        max_step = self.movement_speed * dt
        
        for i in range(7):
            if abs(joint_diff[i]) > max_step:
                self.current_joints[i] += max_step * np.sign(joint_diff[i])
            else:
                self.current_joints[i] = self.target_joints[i]
        
        # Update velocities
        self.joint_velocities = joint_diff / dt if dt > 0 else np.zeros(7)


class RealisticMockKukaFRI(MockKukaFRIInterface):
    """More realistic mock with joint limits and physics"""
    
    def __init__(self, robot_ip: str = "mock", port: int = 30200):
        super().__init__(robot_ip, port)
        
        # KUKA LBR iiwa joint limits (approximate)
        self.joint_limits_min = np.array([-2.97, -2.09, -2.97, -2.09, -2.97, -2.09, -3.05])
        self.joint_limits_max = np.array([2.97, 2.09, 2.97, 2.09, 2.97, 2.09, 3.05])
        
        # Add some noise to make it more realistic
        self.position_noise_std = 0.001  # 1mrad noise
        
    def get_joint_positions(self) -> Optional[np.ndarray]:
        """Return joint positions with noise and limits"""
        if not self.is_connected:
            return None
            
        self._simulate_movement()
        
        # Apply joint limits
        self.current_joints = np.clip(self.current_joints, 
                                    self.joint_limits_min, 
                                    self.joint_limits_max)
        
        # Add realistic sensor noise
        noise = np.random.normal(0, self.position_noise_std, 7)
        return self.current_joints + noise
    
    def set_joint_positions(self, target_joints: List[float], blocking: bool = False, timeout: float = 30.0):
        """Set joints with limit checking"""
        if not self.is_connected:
            return False
            
        target_array = np.array(target_joints)
        
        # Check joint limits
        if np.any(target_array < self.joint_limits_min) or np.any(target_array > self.joint_limits_max):
            print("Mock: Warning - Target joints exceed limits!")
            # Clamp to limits
            target_array = np.clip(target_array, self.joint_limits_min, self.joint_limits_max)
        
        return super().set_joint_positions(target_array, blocking, timeout)