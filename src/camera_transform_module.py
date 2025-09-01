"""
Camera coordinate transformation module.
Transforms hand positions from camera frame to robot base frame.
"""
import numpy as np
from scipy.spatial.transform import Rotation as R
from typing import List, Tuple
from config import CAMERA_IN_TCP, DISTANCE_TO_REMAIN


def pose_to_homogeneous(pose: List[float]) -> np.ndarray:
    """
    Convert pose to homogeneous transformation matrix.
    
    Args:
        pose: [x, y, z, roll, pitch, yaw] (positions in mm, angles in radians)
        
    Returns:
        4x4 homogeneous transformation matrix
    """
    if len(pose) != 6:
        raise ValueError(f"Expected 6 pose values, got {len(pose)}")
        
    x, y, z, roll, pitch, yaw = pose
    
    # Create rotation matrix from Euler angles
    rotation_matrix = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
    
    # Build homogeneous transformation matrix
    T = np.eye(4)
    T[:3, :3] = rotation_matrix
    T[:3, 3] = [x, y, z]
    
    return T


def homogeneous_to_pose(T: np.ndarray) -> List[float]:
    """
    Convert homogeneous transformation matrix to pose.
    
    Args:
        T: 4x4 homogeneous transformation matrix
        
    Returns:
        pose: [x, y, z, roll, pitch, yaw] (positions in mm, angles in radians)
    """
    if T.shape != (4, 4):
        raise ValueError(f"Expected 4x4 matrix, got {T.shape}")
        
    # Extract position
    x, y, z = T[:3, 3]
    
    # Extract rotation and convert to Euler angles
    rotation_matrix = T[:3, :3]
    euler_angles = R.from_matrix(rotation_matrix).as_euler('xyz')
    roll, pitch, yaw = euler_angles
    
    return [x, y, z, roll, pitch, yaw]


def transform_camera_to_base(
    camera_vector: List[float], 
    tcp_pose: np.ndarray
) -> np.ndarray:
    """
    Transform hand position from camera frame to robot base frame.
    
    Args:
        camera_vector: [x, y, z] position in camera frame (mm)
        tcp_pose: 4x4 homogeneous transform of TCP in base frame
        
    Returns:
        3D position in base frame (mm)
    """
    # Validate inputs
    if len(camera_vector) != 3:
        raise ValueError(f"Expected 3D camera vector, got {len(camera_vector)} values")
        
    if tcp_pose.shape != (4, 4):
        raise ValueError(f"Expected 4x4 TCP pose matrix, got {tcp_pose.shape}")
    
    # Calculate camera pose in base frame: base_T_cam = base_T_tcp * tcp_T_cam
    base_T_cam = tcp_pose @ CAMERA_IN_TCP
    
    # Adjust z-coordinate to maintain distance from hand
    adjusted_camera_vector = [
        camera_vector[0],
        camera_vector[1],
        camera_vector[2] - DISTANCE_TO_REMAIN
    ]
    
    # Convert to homogeneous coordinates
    hand_cam = np.array([
        adjusted_camera_vector[0],
        adjusted_camera_vector[1],
        adjusted_camera_vector[2],
        1.0
    ])
    
    # Transform to base frame
    hand_base = base_T_cam @ hand_cam
    
    # Return 3D position (without homogeneous coordinate)
    return hand_base[:3]


def transform_base_to_camera(
    base_position: List[float],
    tcp_pose: np.ndarray
) -> np.ndarray:
    """
    Transform position from robot base frame to camera frame.
    
    Args:
        base_position: [x, y, z] position in base frame (mm)
        tcp_pose: 4x4 homogeneous transform of TCP in base frame
        
    Returns:
        3D position in camera frame (mm)
    """
    # Validate inputs
    if len(base_position) != 3:
        raise ValueError(f"Expected 3D position, got {len(base_position)} values")
        
    if tcp_pose.shape != (4, 4):
        raise ValueError(f"Expected 4x4 TCP pose matrix, got {tcp_pose.shape}")
    
    # Calculate camera pose in base frame
    base_T_cam = tcp_pose @ CAMERA_IN_TCP
    
    # Calculate inverse transform (camera to base)
    cam_T_base = np.linalg.inv(base_T_cam)
    
    # Convert position to homogeneous coordinates
    pos_base = np.array([base_position[0], base_position[1], base_position[2], 1.0])
    
    # Transform to camera frame
    pos_cam = cam_T_base @ pos_base
    
    # Return 3D position
    return pos_cam[:3]


def calculate_tcp_from_joints(
    kinematics_solver,
    joint_angles: List[float]
) -> Tuple[np.ndarray, List[float]]:
    """
    Calculate TCP pose from joint angles.
    
    Args:
        kinematics_solver: Inverse kinematics solver instance
        joint_angles: List of 7 joint angles (radians)
        
    Returns:
        Tuple of (4x4 homogeneous matrix, [x,y,z,roll,pitch,yaw] pose)
    """
    # Add base and end-effector dummy joints
    joints_full = np.insert(joint_angles, 0, 0.0)
    joints_full = np.append(joints_full, 0.0)
    
    # Calculate forward kinematics
    tcp_matrix = kinematics_solver.solve_tcp(joints_full)
    
    # Convert to pose representation
    tcp_pose = homogeneous_to_pose(tcp_matrix)
    
    return tcp_matrix, tcp_pose


def validate_workspace_limits(
    position: np.ndarray,
    workspace_limits: dict = None
) -> bool:
    """
    Check if position is within robot workspace limits.
    
    Args:
        position: [x, y, z] position in base frame (mm)
        workspace_limits: Dictionary with 'min' and 'max' arrays
        
    Returns:
        True if position is within limits, False otherwise
    """
    if workspace_limits is None:
        # Default KUKA iiwa14 approximate workspace (mm)
        workspace_limits = {
            'min': np.array([-800, -800, 0]),
            'max': np.array([800, 800, 1300])
        }
    
    position = np.array(position)
    
    # Check if within bounds
    within_min = np.all(position >= workspace_limits['min'])
    within_max = np.all(position <= workspace_limits['max'])
    
    return within_min and within_max


def calculate_approach_position(
    target_position: np.ndarray,
    approach_distance: float = 100.0,
    approach_direction: np.ndarray = None
) -> np.ndarray:
    """
    Calculate approach position for grasping.
    
    Args:
        target_position: Target position in base frame (mm)
        approach_distance: Distance to approach from (mm)
        approach_direction: Unit vector for approach direction
        
    Returns:
        Approach position in base frame (mm)
    """
    if approach_direction is None:
        # Default approach from above
        approach_direction = np.array([0, 0, -1])
    
    # Normalize direction vector
    approach_direction = approach_direction / np.linalg.norm(approach_direction)
    
    # Calculate approach position
    approach_position = target_position - approach_distance * approach_direction
    
    return approach_position
    