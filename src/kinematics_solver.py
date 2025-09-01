"""
Kinematics solver for KUKA iiwa robot.
Provides forward and inverse kinematics using ikpy library.
"""
from ikpy.chain import Chain
import numpy as np
from typing import List, Optional, Tuple
import logging
from config import JOINT_LIMITS, JOINT_VELOCITY_LIMITS, WORKSPACE_LIMITS

logger = logging.getLogger(__name__)


class InverseKinematicsSolver:
    """
    Kinematics solver for robot arm using ikpy.
    """
    
    def __init__(
        self, 
        urdf_filepath: str, 
        base_elements: List[str], 
        active_links_mask: List[bool]
    ):
        """
        Initialize kinematics solver.
        
        Args:
            urdf_filepath: Path to URDF file
            base_elements: List of base element names
            active_links_mask: Boolean mask for active joints
        """
        try:
            self.chain = Chain.from_urdf_file(
                urdf_filepath,
                base_elements=base_elements,
                active_links_mask=active_links_mask
            )
            logger.info(f"Kinematics solver initialized with {len(self.chain.links)} links")
            
            # Store configuration
            self.active_links_mask = active_links_mask
            self.num_joints = sum(active_links_mask) - 2  # Exclude base and end-effector
            
        except Exception as e:
            logger.error(f"Failed to initialize kinematics solver: {e}")
            raise
            
    def solve_tcp(self, joint_angles: List[float]) -> np.ndarray:
        """
        Calculate TCP pose from joint angles (forward kinematics).
        
        Args:
            joint_angles: List of joint angles including base and end-effector (radians)
            
        Returns:
            4x4 homogeneous transformation matrix
        """
        if len(joint_angles) != len(self.active_links_mask):
            raise ValueError(
                f"Expected {len(self.active_links_mask)} joint angles, "
                f"got {len(joint_angles)}"
            )
            
        try:
            tcp_matrix = self.chain.forward_kinematics(joint_angles)
            return tcp_matrix
            
        except Exception as e:
            logger.error(f"Forward kinematics failed: {e}")
            raise
            
    def solve_XYZ(
        self, 
        target_position: List[float], 
        current_joint_angles: List[float],
        target_orientation: Optional[np.ndarray] = None,
        max_iterations: int = 100,
        tolerance: float = 1e-4
    ) -> np.ndarray:
        """
        Calculate joint angles for target position (inverse kinematics).
        
        Args:
            target_position: Target [x, y, z] position in base frame (mm)
            current_joint_angles: Current joint angles for initial guess (radians)
            target_orientation: Optional 3x3 rotation matrix for orientation
            max_iterations: Maximum iterations for IK solver
            tolerance: Position tolerance for convergence
            
        Returns:
            Array of joint angles including base and end-effector
        """
        if len(current_joint_angles) != len(self.active_links_mask):
            raise ValueError(
                f"Expected {len(self.active_links_mask)} joint angles, "
                f"got {len(current_joint_angles)}"
            )
            
        try:
            # Build target matrix
            target_matrix = np.eye(4)
            target_matrix[:3, 3] = target_position
            
            if target_orientation is not None:
                target_matrix[:3, :3] = target_orientation
                
            # Solve inverse kinematics
            joint_angles = self.chain.inverse_kinematics(
                target_matrix,
                initial_position=current_joint_angles,
                max_iter=max_iterations,
                tolerance=tolerance
            )
            
            # Validate solution
            if self._validate_joint_angles(joint_angles):
                return joint_angles
            else:
                logger.warning("IK solution violates joint limits")
                return self._clamp_joint_angles(joint_angles)
                
        except Exception as e:
            logger.error(f"Inverse kinematics failed: {e}")
            raise
            
    def solve_pose(
        self,
        target_pose: List[float],
        current_joint_angles: List[float],
        max_iterations: int = 100,
        tolerance: float = 1e-4
    ) -> np.ndarray:
        """
        Calculate joint angles for target pose (position + orientation).
        
        Args:
            target_pose: [x, y, z, roll, pitch, yaw] in base frame
            current_joint_angles: Current joint angles for initial guess
            max_iterations: Maximum iterations for IK solver
            tolerance: Position tolerance for convergence
            
        Returns:
            Array of joint angles
        """
        from scipy.spatial.transform import Rotation as R
        
        # Extract position and orientation
        position = target_pose[:3]
        euler_angles = target_pose[3:6]
        
        # Convert Euler angles to rotation matrix
        rotation_matrix = R.from_euler('xyz', euler_angles).as_matrix()
        
        # Solve IK with orientation
        return self.solve_XYZ(
            position, 
            current_joint_angles,
            target_orientation=rotation_matrix,
            max_iterations=max_iterations,
            tolerance=tolerance
        )
        
    def _validate_joint_angles(self, joint_angles: np.ndarray) -> bool:
        """
        Check if joint angles are within limits.
        
        Args:
            joint_angles: Array of joint angles
            
        Returns:
            True if all angles are within limits
        """
        # Extract actual joint values (skip base and end-effector)
        actual_joints = []
        for i, active in enumerate(self.active_links_mask):
            if active and i != 0 and i != len(self.active_links_mask) - 1:
                actual_joints.append(joint_angles[i])
                
        actual_joints = np.array(actual_joints)
        
        # Check against limits
        within_min = np.all(actual_joints >= JOINT_LIMITS['min'])
        within_max = np.all(actual_joints <= JOINT_LIMITS['max'])
        
        return within_min and within_max
        
    def _clamp_joint_angles(self, joint_angles: np.ndarray) -> np.ndarray:
        """
        Clamp joint angles to limits.
        
        Args:
            joint_angles: Array of joint angles
            
        Returns:
            Clamped joint angles
        """
        clamped = joint_angles.copy()
        
        # Clamp actual joints (skip base and end-effector)
        joint_idx = 0
        for i, active in enumerate(self.active_links_mask):
            if active and i != 0 and i != len(self.active_links_mask) - 1:
                clamped[i] = np.clip(
                    joint_angles[i],
                    JOINT_LIMITS['min'][joint_idx],
                    JOINT_LIMITS['max'][joint_idx]
                )
                joint_idx += 1
                
        return clamped
        
    def calculate_jacobian(self, joint_angles: List[float]) -> np.ndarray:
        """
        Calculate Jacobian matrix at current configuration.
        
        Args:
            joint_angles: Current joint angles
            
        Returns:
            6xN Jacobian matrix
        """
        try:
            # ikpy doesn't directly expose Jacobian, so we approximate it
            # This is a simplified version - consider using a proper robotics library
            # for production use
            
            epsilon = 1e-6
            tcp_current = self.solve_tcp(joint_angles)
            position_current = tcp_current[:3, 3]
            
            jacobian = []
            
            for i, active in enumerate(self.active_links_mask):
                if active and i != 0 and i != len(self.active_links_mask) - 1:
                    # Perturb joint
                    joint_angles_perturbed = joint_angles.copy()
                    joint_angles_perturbed[i] += epsilon
                    
                    # Calculate new position
                    tcp_perturbed = self.solve_tcp(joint_angles_perturbed)
                    position_perturbed = tcp_perturbed[:3, 3]
                    
                    # Calculate derivative
                    dp_dq = (position_perturbed - position_current) / epsilon
                    jacobian.append(dp_dq)
                    
            return np.array(jacobian).T
            
        except Exception as e:
            logger.error(f"Jacobian calculation failed: {e}")
            raise
            
    def check_singularity(
        self, 
        joint_angles: List[float], 
        threshold: float = 0.01
    ) -> bool:
        """
        Check if robot is near singularity.
        
        Args:
            joint_angles: Current joint angles
            threshold: Singularity threshold for determinant
            
        Returns:
            True if near singularity
        """
        try:
            jacobian = self.calculate_jacobian(joint_angles)
            
            # Check determinant of Jacobian
            # For non-square Jacobian, use condition number
            if jacobian.shape[0] != jacobian.shape[1]:
                condition_number = np.linalg.cond(jacobian)
                return condition_number > (1.0 / threshold)
            else:
                det = np.linalg.det(jacobian)
                return abs(det) < threshold
                
        except Exception as e:
            logger.error(f"Singularity check failed: {e}")
            return False