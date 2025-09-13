"""
Kinematics solver for KUKA iiwa robot.
Provides forward and inverse kinematics using ikpy library.
"""
from ikpy.chain import Chain
import numpy as np
from typing import List, Optional
import logging
from scipy.spatial.transform import Rotation as R

logger = logging.getLogger(__name__)


def homogeneous_to_pose(T: np.ndarray) -> List[float]:
    """
    Convert homogeneous transform to [x,y,z,roll,pitch,yaw] (mm, rad)
    """
    if T.shape != (4, 4):
        raise ValueError(f"Expected 4x4 matrix, got {T.shape}")
    x, y, z = T[:3, 3]
    roll, pitch, yaw = R.from_matrix(T[:3, :3]).as_euler('xyz')
    return [x, y, z, roll, pitch, yaw]


class InverseKinematicsSolver:
    """
    Kinematics solver for robot arm using ikpy.
    Joint limits are integrated directly into the ikpy Chain object for robust solving.
    """

    def __init__(self, urdf_filepath: str, base_elements: List[str], active_links_mask: List[bool]):
        try:
            self.chain = Chain.from_urdf_file(
                urdf_filepath,
                base_elements=base_elements,
                active_links_mask=active_links_mask
            )
            logger.info(
                f"Kinematics chain initialized with {len(self.chain.links)} links")
            # Store masks/indices
            self.active_links_mask = active_links_mask
            # Indices of the 7 actuated robot joints (exclude base and tool fixed joints)
            self.movable_joint_indices = [
                i for i, active in enumerate(self.active_links_mask)
                if active and i > 0 and i < len(self.active_links_mask) - 1
            ]
            if len(self.movable_joint_indices) != 7:
                logger.warning(
                    f"Expected 7 movable joints, found {len(self.movable_joint_indices)} from mask."
                )
        except Exception as e:
            logger.error(f"Failed to initialize kinematics solver: {e}")
            raise

    def solve_tcp(self, joint_angles: List[float]) -> np.ndarray:
        """
        Forward kinematics for a full joint vector matching the chain length.
        Use tcp_from_joints if you have only 7 robot joints.
        """
        if len(joint_angles) != len(self.active_links_mask):
            raise ValueError(
                f"Expected {len(self.active_links_mask)} joint angles, got {len(joint_angles)}")
        try:
            return self.chain.forward_kinematics(joint_angles)
        except Exception as e:
            logger.error(f"Forward kinematics failed: {e}")
            raise

    def tcp_from_joints(self, joint_angles_7: List[float]):
        """
        Helper to compute TCP 4x4 matrix and pose from 7 joint angles (rad).
        Adds dummy base/tool joints as required by the chain.
        """
        if len(joint_angles_7) != 7:
            raise ValueError(
                f"Expected 7 joint values, got {len(joint_angles_7)}")
        joints_full = np.insert(np.asarray(
            joint_angles_7, dtype=float), 0, 0.0)
        joints_full = np.append(joints_full, 0.0)
        tcp_matrix = self.solve_tcp(joints_full)
        tcp_pose = homogeneous_to_pose(tcp_matrix)
        return tcp_matrix, tcp_pose

    def solve_XYZ(
        self,
        target_position: List[float],
        current_joint_angles: List[float],
        target_orientation: Optional[np.ndarray] = None,
        max_iterations: int = 100,
        tolerance: float = 1e-4
    ) -> np.ndarray:
        # Accept either 7-joint vector (robot joints) or full-length vector (chain)
        if len(current_joint_angles) == 7:
            initial_full = np.insert(np.asarray(
                current_joint_angles, dtype=float), 0, 0.0)
            initial_full = np.append(initial_full, 0.0)
        elif len(current_joint_angles) == len(self.active_links_mask):
            initial_full = np.asarray(current_joint_angles, dtype=float)
        else:
            raise ValueError(
                f"Expected 7 or {len(self.active_links_mask)} joint angles, got {len(current_joint_angles)}")
        try:
            target_matrix = np.eye(4)
            target_matrix[:3, 3] = target_position
            orientation_mode = None
            if target_orientation is not None:
                target_matrix[:3, :3] = target_orientation
                orientation_mode = "all"
            solution_angles_full = self.chain.inverse_kinematics(
                target_matrix,
                initial_position=initial_full,
                max_iter=max_iterations,
                tolerance=tolerance,
                orientation_mode=orientation_mode
            )
            # Compute final pose for error checking
            final_pose_matrix = self.solve_tcp(solution_angles_full)
            final_position = final_pose_matrix[:3, 3]
            positional_error = np.linalg.norm(
                final_position - np.array(target_position))
            if positional_error > tolerance:
                logger.warning(
                    f"IK solver failed to converge. error: {positional_error:.4f} > tol: {tolerance}"
                )
                raise RuntimeError(
                    "Inverse kinematics failed to find a valid solution within tolerance.")
            logger.debug(
                f"IK solution positional error: {positional_error:.4f}")
            # Return only the 7 actuated joints, in order
            solution_7 = np.asarray(solution_angles_full, dtype=float)[
                self.movable_joint_indices]
            return solution_7
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
        position = target_pose[:3]
        euler_angles = target_pose[3:6]
        rotation_matrix = R.from_euler('xyz', euler_angles).as_matrix()
        return self.solve_XYZ(
            position,
            current_joint_angles,
            target_orientation=rotation_matrix,
            max_iterations=max_iterations,
            tolerance=tolerance
        )


# Planning/motion utilities colocated with kinematics for now
def validate_workspace_limits(position: np.ndarray, workspace_limits: dict = None) -> bool:
    if workspace_limits is None:
        workspace_limits = {'min': np.array(
            [-800, -800, 0]), 'max': np.array([800, 800, 1300])}
    position = np.array(position)
    return np.all(position >= workspace_limits['min']) and np.all(position <= workspace_limits['max'])


def calculate_approach_position(target_position: np.ndarray, approach_distance: float = 100.0, approach_direction: np.ndarray = None) -> np.ndarray:
    if approach_direction is None:
        approach_direction = np.array([0, 0, -1])
    approach_direction = approach_direction / \
        np.linalg.norm(approach_direction)
    return target_position - approach_distance * approach_direction
