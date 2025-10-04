"""
Kinematics solver for KUKA iiwa robot using PyBullet (drop-in replacement for ikpy-based solver).
Provides forward and inverse kinematics and matching API:
- InverseKinematicsSolver(urdf_filepath, base_elements, active_links_mask)
- solve_XYZ(target_position, current_joint_angles, target_orientation=None, ...)
- solve_pose(target_pose, current_joint_angles, ...)
- tcp_from_joints(joint_angles_7)
- solve_tcp(joint_angles_full)
Note: Positions are expected in METERS for PyBullet.
"""
from config.config import JOINT_LIMITS
import sys
from pathlib import Path
from typing import List, Optional

import numpy as np
import logging
import pybullet as p
import pybullet_data
from scipy.spatial.transform import Rotation as R

# preserve same imports names as original file for ease of swapping
logger = logging.getLogger(__name__)

# Import joint limits from config


def homogeneous_to_pose(T: np.ndarray) -> List[float]:
    """
    Convert homogeneous transform to [x,y,z,roll,pitch,yaw] (meters, rad)
    """
    if T.shape != (4, 4):
        raise ValueError(f"Expected 4x4 matrix, got {T.shape}")
    x, y, z = T[:3, 3]
    roll, pitch, yaw = R.from_matrix(T[:3, :3]).as_euler('xyz')
    return [x, y, z, roll, pitch, yaw]


def get_facing_down_orientation() -> np.ndarray:
    """
    Returns the 3x3 rotation matrix for a tool facing straight down.
    This corresponds to a 180-degree rotation around the world's Z-axis.
    """
    return np.array([
        [1,  0,  0],
        [0, -1,  0],
        [0,  0, -1]
    ])


def validate_joint_limits(joint_angles: List[float]) -> bool:
    """
    Validate that joint angles are within specified limits.
    Returns True if all joints are within limits, False otherwise.
    """
    if len(joint_angles) != 7:
        logger.warning(f"Expected 7 joint angles, got {len(joint_angles)}")
        return False

    joint_names = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7']

    for i, (angle, joint_name) in enumerate(zip(joint_angles, joint_names)):
        limits = JOINT_LIMITS[joint_name]
        if angle < limits['min'] or angle > limits['max']:
            logger.warning(
                f"Joint {joint_name} (index {i}) angle {angle:.3f} exceeds limits [{limits['min']:.3f}, {limits['max']:.3f}]")
            return False

    return True


def clamp_joint_limits(joint_angles: List[float]) -> List[float]:
    """
    Clamp joint angles to within specified limits.
    Returns clamped joint angles.
    """
    if len(joint_angles) != 7:
        logger.warning(f"Expected 7 joint angles, got {len(joint_angles)}")
        return joint_angles

    joint_names = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7']
    clamped_angles = []

    for angle, joint_name in zip(joint_angles, joint_names):
        limits = JOINT_LIMITS[joint_name]
        clamped_angle = np.clip(angle, limits['min'], limits['max'])
        if clamped_angle != angle:
            logger.warning(
                f"Clamped joint {joint_name} from {angle:.3f} to {clamped_angle:.3f}")
        clamped_angles.append(clamped_angle)

    return clamped_angles


class InverseKinematicsSolver:
    """
    Kinematics solver wrapper using PyBullet.
    Keeps a pybullet client open for fast FK/IK queries.
    """

    def __init__(self, urdf_filepath: str, base_elements: List[str], active_links_mask: List[bool], use_gui: bool = False):
        """
        urdf_filepath: path to URDF (string)
        base_elements, active_links_mask: accepted for compatibility (not used by pybullet heavily)
        use_gui: start a GUI physics client when True (useful for debugging)
        """
        try:
            # Start PyBullet
            self.client = p.connect(p.GUI if use_gui else p.DIRECT)
            p.setAdditionalSearchPath(
                pybullet_data.getDataPath(), physicsClientId=self.client)
            p.setPhysicsEngineParameter(
                enableFileCaching=0, physicsClientId=self.client)

            # Load URDF (accept Path or str)
            flags = p.URDF_USE_INERTIA_FROM_FILE
            urdf_path_str = str(urdf_filepath)
            self.robot_id = p.loadURDF(
                urdf_path_str,
                useFixedBase=True,
                flags=flags,
                physicsClientId=self.client
            )

            # Cache joint & link info
            self.num_joints = p.getNumJoints(
                self.robot_id, physicsClientId=self.client)
            # Revolute/prismatic joints: jointType 0 = revolute, 1 = prismatic, etc.
            revolute_indices = []
            revolute_names = []
            for j in range(self.num_joints):
                info = p.getJointInfo(
                    self.robot_id, j, physicsClientId=self.client)
                joint_type = info[2]
                joint_name = info[1].decode('utf-8')
                # JOINT_REVOLUTE constant is 0
                if joint_type == p.JOINT_REVOLUTE or joint_type == p.JOINT_PRISMATIC:
                    revolute_indices.append(j)
                    revolute_names.append(joint_name)

            self.revolute_joint_indices = revolute_indices
            self.revolute_joint_names = revolute_names

            logger.info(
                f"Loaded URDF: {urdf_path_str} with {len(self.revolute_joint_indices)} active revolute/prismatic joints")

            # Attempt to find end effector link index: assume last link by default
            # You can override this by setting self.end_effector_link_index externally if needed
            self.end_effector_link_index = self.num_joints - 1

            # Compatibility fields
            self.active_links_mask = active_links_mask
            # Build a best-effort movable_joint_indices mapping (indices into a "full chain" style array).
            # If active_links_mask length matches number of revolute joints, map 1-to-1; otherwise create a trivial mapping.
            try:
                if active_links_mask and len(active_links_mask) == len(self.revolute_joint_indices):
                    self.movable_joint_indices = [
                        i for i, v in enumerate(active_links_mask) if v]
                else:
                    # fallback: first N revolute joints
                    self.movable_joint_indices = list(
                        range(len(self.revolute_joint_indices)))
            except Exception:
                self.movable_joint_indices = list(
                    range(len(self.revolute_joint_indices)))

            # Ensure we can handle typical 7-DOF iiwa
            if len(self.revolute_joint_indices) < 7:
                logger.warning(
                    "Less than 7 revolute joints detected in URDF - verify URDF and joints mapping")

        except Exception as e:
            logger.error(f"Failed to initialize PyBullet IK solver: {e}")
            raise

    def disconnect(self):
        try:
            p.disconnect(physicsClientId=self.client)
        except Exception:
            pass

    def _set_joint_states_from_list(self, joint_values: List[float]):
        """
        Set revolute joints to the provided values. joint_values length must match number of revolute joints used.
        This uses resetJointState (instant, no physics) for quick FK checks.
        """
        if len(joint_values) != len(self.revolute_joint_indices):
            raise ValueError(
                f"Expected {len(self.revolute_joint_indices)} joint values, got {len(joint_values)}")
        for idx, j in enumerate(self.revolute_joint_indices):
            p.resetJointState(self.robot_id, j,
                              joint_values[idx], physicsClientId=self.client)

    def solve_tcp(self, joint_angles: List[float]) -> np.ndarray:
        """
        Forward kinematics for a full joint vector matching the revolute_joint_indices length.
        Returns 4x4 homogeneous transform matrix (world -> end effector).
        """
        # Expect joint_angles to match revolute joints count
        if len(joint_angles) != len(self.revolute_joint_indices):
            raise ValueError(
                f"Expected {len(self.revolute_joint_indices)} joint angles, got {len(joint_angles)}")

        # Apply joints and query link state
        self._set_joint_states_from_list(list(joint_angles))
        link_state = p.getLinkState(self.robot_id, self.end_effector_link_index,
                                    computeForwardKinematics=True, physicsClientId=self.client)
        pos = np.array(link_state[4])  # worldLinkFramePosition
        # worldLinkFrameOrientation quaternion (x,y,z,w)
        orn = np.array(link_state[5])
        rot = R.from_quat(orn).as_matrix()
        T = np.eye(4)
        T[:3, :3] = rot
        T[:3, 3] = pos
        return T

    def tcp_from_joints(self, joint_angles_7: List[float]):
        """
        Helper to compute TCP 4x4 matrix and pose from 7 joint angles (rad).
        This method maps the 7 provided joint values to the first 7 revolute joints found in the URDF.
        Returns (tcp_matrix, tcp_pose) where tcp_pose is [x,y,z,roll,pitch,yaw]
        """
        if len(joint_angles_7) != 7:
            raise ValueError(
                f"Expected 7 joint values, got {len(joint_angles_7)}")
        if len(self.revolute_joint_indices) < 7:
            raise RuntimeError(
                "URDF does not expose at least 7 revolute joints. Unable to map 7-joint vector.")

        # Validate joint limits
        if not validate_joint_limits(joint_angles_7):
            logger.warning("Input joint angles violate limits, clamping...")
            joint_angles_7 = clamp_joint_limits(joint_angles_7)

        # Build full revolute joint vector: use provided 7 values for first 7 revolute joints and zeros for the rest
        full_joint_vector = np.zeros(
            len(self.revolute_joint_indices), dtype=float)
        full_joint_vector[:7] = np.asarray(joint_angles_7, dtype=float)

        tcp_matrix = self.solve_tcp(full_joint_vector.tolist())
        tcp_pose = homogeneous_to_pose(tcp_matrix)
        return tcp_matrix, tcp_pose

    def _rotation_matrix_to_quat(self, R_mat: np.ndarray) -> List[float]:
        """
        Convert 3x3 rotation matrix to quaternion in (x, y, z, w) for PyBullet (same order SciPy uses).
        """
        return R.from_matrix(R_mat).as_quat().tolist()  # SciPy returns [x, y, z, w]

    def solve_XYZ(
        self,
        target_position: List[float],
        current_joint_angles: List[float],
        target_orientation: Optional[np.ndarray] = None,
        max_iterations: int = 100,
        tolerance: float = 1e-4
    ) -> np.ndarray:
        """
        Compute IK for target position and optional orientation using PyBullet.
        - target_position: [x,y,z] in meters
        - current_joint_angles: either 7-length list or full-length matching revolute joints count
        - target_orientation: optional 3x3 rotation matrix (world->tcp)
        Returns numpy array of 7 joint targets (rad) for the robot's first 7 revolute joints.
        """
        # Normalize inputs
        if target_position is None or any(x is None for x in target_position):
            raise ValueError("Target position contains None values")

        # Convert current joints into length matching revolute joints for pybullet initial guess
        if len(current_joint_angles) == 7:
            initial_full = np.zeros(
                len(self.revolute_joint_indices), dtype=float)
            initial_full[:7] = np.asarray(current_joint_angles, dtype=float)
        elif len(current_joint_angles) == len(self.revolute_joint_indices):
            initial_full = np.asarray(current_joint_angles, dtype=float)
        else:
            # try to fallback if they provided a longer mask-like vector
            try:
                initial_full = np.asarray(current_joint_angles, dtype=float)[
                    :len(self.revolute_joint_indices)]
            except Exception:
                raise ValueError(
                    f"Expected 7 or {len(self.revolute_joint_indices)} joint angles, got {len(current_joint_angles)}")

        # Ensure constraints on iterations
        max_iterations = min(max_iterations, 200)

        # Convert orientation (3x3) to quaternion if provided
        quat = None
        if target_orientation is not None:
            if target_orientation.shape != (3, 3):
                raise ValueError(
                    "target_orientation must be 3x3 rotation matrix")
            quat = self._rotation_matrix_to_quat(target_orientation)

        # Call PyBullet IK
        try:
            sol = p.calculateInverseKinematics(
                bodyUniqueId=self.robot_id,
                endEffectorLinkIndex=self.end_effector_link_index,
                targetPosition=target_position,
                targetOrientation=quat,
                maxNumIterations=max_iterations,
                residualThreshold=tolerance,
                physicsClientId=self.client
            )
        except Exception as e:
            logger.error(f"PyBullet calculateInverseKinematics failed: {e}")
            raise

        # `sol` contains a solution for all joints (length == num_joints).
        # We extract revolute joints in the order we discovered them.
        solution_revolute = []
        for rev_idx in self.revolute_joint_indices:
            # When pybullet returns a full-length vector, selecting by joint index is safe
            try:
                solution_revolute.append(sol[rev_idx])
            except Exception:
                # If calculateInverseKinematics returned only revolute joint solutions in order,
                # fallback to using sequential extraction.
                logger.debug(
                    "Falling back to sequential extraction from IK solution.")
                # convert sol to array and take first N revolute
                arr = np.asarray(sol, dtype=float)
                solution_revolute = arr[:len(
                    self.revolute_joint_indices)].tolist()
                break

        solution_revolute = np.asarray(solution_revolute, dtype=float)

        # Return only the first 7 actuated joints (matching your robot)
        if len(solution_revolute) < 7:
            raise RuntimeError(
                "IK returned fewer than 7 revolute joint values.")
        solution_7 = solution_revolute[:7]

        # Validate joint limits
        if not validate_joint_limits(solution_7.tolist()):
            logger.warning("IK solution violates joint limits, clamping...")
            solution_7 = np.array(clamp_joint_limits(solution_7.tolist()))

        # Optional: verify positional error
        # Apply solution to robot and compute actual TCP
        try:
            self._set_joint_states_from_list(solution_revolute.tolist())
            link_state = p.getLinkState(self.robot_id, self.end_effector_link_index,
                                        computeForwardKinematics=True, physicsClientId=self.client)
            achieved_pos = np.array(link_state[4])
            pos_err = np.linalg.norm(
                achieved_pos - np.asarray(target_position))
            if pos_err > tolerance:
                logger.warning(
                    f"IK residual position error: {pos_err:.4f} (tolerance {tolerance})")
        except Exception as ex:
            logger.debug(f"FK verification failed: {ex}")

        return solution_7

    def solve_pose(
        self,
        target_pose: List[float],
        current_joint_angles: List[float],
        max_iterations: int = 100,
        tolerance: float = 1e-4
    ) -> np.ndarray:
        """
        target_pose expected as [x,y,z, rx, ry, rz] where rx,ry,rz are Euler 'xyz' angles (radians).
        """
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

    # (optional) destructor fallback
    def __del__(self):
        try:
            self.disconnect()
        except Exception:
            pass


# Example usage when run as script (quick smoke test)
if __name__ == "__main__":
    import os
    logging.basicConfig(level=logging.INFO)
    # Replace with your URDF path
    urdf = os.environ.get("URDF_FILEPATH", "src/resources/models/iiwa14.urdf")
    solver = InverseKinematicsSolver(
        urdf_filepath=urdf, base_elements=None, active_links_mask=None, use_gui=False)
    sol7 = solver.solve_XYZ([0.5, 0.0, 0.6], [0.0]*7,
                            get_facing_down_orientation())
    print("Solution (7):", sol7)
    solver.disconnect()
