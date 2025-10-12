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
from config import JOINT_LIMITS
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

# ============================================================================
# IK SOLVER CONFIGURATION
# ============================================================================
# Epsilon margin to prevent numerical overshoot at joint limits
# PyBullet's IK solver can overshoot by small amounts due to tolerance
IK_EPSILON_MARGIN_DEG = 0.8  # Safety margin in degrees (0.014 rad)


# ============================================================================
# EPSILON-CLAMPING HELPER FUNCTIONS
# ============================================================================

def _epsilon_limits(limits: dict, eps_rad: float) -> tuple:
    """
    Shrink joint limit range by epsilon margin on both sides to prevent numerical overshoot.

    Args:
        limits: Dict with 'min' and 'max' keys (radians)
        eps_rad: Safety margin in radians to shrink on each side

    Returns:
        Tuple of (lower, upper, range) with epsilon-shrunk limits
    """
    lower = limits['min'] + eps_rad
    upper = limits['max'] - eps_rad

    # Fallback if epsilon too large for this joint's range
    if upper < lower:
        mid = 0.5 * (limits['min'] + limits['max'])
        logger.warning(f"Epsilon margin {eps_rad:.4f} too large for joint range "
                       f"[{limits['min']:.4f}, {limits['max']:.4f}], using midpoint")
        lower, upper = mid, mid

    joint_range = upper - lower
    return lower, upper, joint_range


def _unwrap_to_limits(q: List[float], limits: List[dict]) -> List[float]:
    """
    Adjust angles by 2π multiples to lie inside [min, max] if wrapping helps.
    Prefers solutions closest to the middle of the joint range.

    Args:
        q: Joint angles (radians)
        limits: List of dicts with 'min' and 'max' keys

    Returns:
        Unwrapped joint angles that fit within limits
    """
    out = []
    for qi, lim in zip(q, limits):
        # If already in bounds, keep as-is
        if lim['min'] <= qi <= lim['max']:
            out.append(qi)
            continue

        # Try shifts of ±2π to fit, prefer midpoint-closest solution
        qwrap = qi
        mid = 0.5 * (lim['min'] + lim['max'])
        best_dist = abs(qi - mid)

        for k in (-2, -1, 1, 2):  # Extended range for safety
            cand = qi + 2 * np.pi * k
            if lim['min'] <= cand <= lim['max']:
                dist = abs(cand - mid)
                if dist < best_dist:
                    qwrap = cand
                    best_dist = dist

        out.append(qwrap)
    return out


def _mid_limits(limits: List[dict]) -> List[float]:
    """
    Compute midpoint of each joint's range.
    Useful for rest poses that avoid boundaries.

    Args:
        limits: List of dicts with 'min' and 'max' keys

    Returns:
        List of midpoint angles
    """
    return [(lim['min'] + lim['max']) * 0.5 for lim in limits]


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
    This corresponds to a 180-degree rotation around the world's X-axis.

    This matrix flips the Y and Z axes, making the tool point downward (-Z direction)
    while keeping the X-axis unchanged. This is the standard orientation for
    top-down grasping tasks.
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
            logger.info(
                f"Revolute joint indices: {self.revolute_joint_indices}")
            logger.info(f"Revolute joint names: {self.revolute_joint_names}")

            # Find end effector link index by name - MUST use 'tcp' for gripper tip!
            # Priority order: tcp > ee_link > flange > tool0 (last resort)
            self.end_effector_link_index = self.num_joints - 1  # Default fallback
            ee_found = False

            # Try to find 'tcp' first (most important - actual gripper tip)
            try:
                for i in range(self.num_joints):
                    info = p.getJointInfo(
                        self.robot_id, i, physicsClientId=self.client)
                    link_name = info[12].decode('utf-8')  # link name

                    # Priority: tcp is most important (gripper tip)
                    if link_name == 'tcp':
                        self.end_effector_link_index = i
                        logger.info(
                            f"✅ Found TCP (gripper tip) at link index {i}")
                        ee_found = True
                        break
                    elif not ee_found and link_name in ('ee_link', 'flange'):
                        self.end_effector_link_index = i
                        logger.info(
                            f"Found end effector '{link_name}' at index {i}")
                        ee_found = True

                # Warn if only found tool0 (missing gripper extension!)
                if not ee_found:
                    for i in range(self.num_joints):
                        info = p.getJointInfo(
                            self.robot_id, i, physicsClientId=self.client)
                        link_name = info[12].decode('utf-8')
                        if link_name == 'tool0':
                            self.end_effector_link_index = i
                            logger.warning(
                                f"⚠️ Using 'tool0' at index {i} - gripper extension NOT included in TCP!")
                            break

            except Exception as e:
                logger.debug(
                    f"Could not find end effector by name, using default: {e}")

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
        tolerance: float = 1e-3  # Relaxed from 1e-4 to handle small residuals after clamping
    ) -> np.ndarray:
        """
        Compute IK for target position and optional orientation using PyBullet.

        Uses epsilon-clamping strategy to prevent joint limit violations:
        - Joint limits are shrunk by IK_EPSILON_MARGIN_DEG (default 0.8°) before solving
        - Solution is unwrapped (2π shifts) to fit within actual joint limits
        - Falls back to hard clamping only if epsilon-clamping + unwrapping fail

        Args:
            target_position: [x,y,z] in meters
            current_joint_angles: either 7-length list or full-length matching revolute joints count
            target_orientation: optional 3x3 rotation matrix (world->tcp)
            max_iterations: maximum IK solver iterations (default 100)
            tolerance: positional error threshold in meters (default 1e-3)

        Returns:
            numpy array of 7 joint targets (rad) for the robot's first 7 revolute joints
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

        # Seed IK by setting robot to current configuration to bias solution
        # This helps PyBullet's IK solver start from a good initial guess
        try:
            self._set_joint_states_from_list(initial_full.tolist())
        except Exception as e:
            logger.debug(f"Could not seed IK with current joint state: {e}")

        # Prepare joint limits for PyBullet IK with epsilon-clamping
        # IMPORTANT: Must map limits to ALL joints in URDF, not just revolute joints
        num_joints = p.getNumJoints(self.robot_id, physicsClientId=self.client)

        joint_names = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7']
        lower_limits = []
        upper_limits = []
        joint_ranges = []
        rest_poses = []

        # Epsilon margin to prevent solver overshoot
        eps_rad = np.deg2rad(IK_EPSILON_MARGIN_DEG)

        # Build mapping of revolute joint index -> KUKA joint name robustly by name
        # This handles URDF ordering differences correctly
        revolute_to_kuka = {}
        kuka_name_order = ['joint_a1', 'joint_a2', 'joint_a3',
                           'joint_a4', 'joint_a5', 'joint_a6', 'joint_a7']
        kuka_to_Aname = {'joint_a1': 'A1', 'joint_a2': 'A2', 'joint_a3': 'A3', 'joint_a4': 'A4',
                         'joint_a5': 'A5', 'joint_a6': 'A6', 'joint_a7': 'A7'}

        # Match by actual joint name from URDF
        for idx, rev_idx in enumerate(self.revolute_joint_indices):
            name = self.revolute_joint_names[idx] if idx < len(
                self.revolute_joint_names) else None
            if name and name in kuka_name_order:
                # Map actual PyBullet joint index -> logical 'A#' name
                revolute_to_kuka[rev_idx] = kuka_to_Aname[name]
                # Stop when we've found all seven
                if len(revolute_to_kuka) == 7:
                    break

        # Fallback: if any of the expected KUKA joints were not found, fall back to first-7 mapping
        if len(revolute_to_kuka) < 7:
            logger.warning(
                "Could not map all KUKA joint names by name; falling back to first-7 revolute joints.")
            revolute_to_kuka = {}
            for i, rev_idx in enumerate(self.revolute_joint_indices[:7]):
                revolute_to_kuka[rev_idx] = joint_names[i]

        # Log the mapping for debugging
        logger.debug(f"Joint mapping: {revolute_to_kuka}")

        # Apply epsilon-shrunk limits to ALL joints in the URDF
        for joint_idx in range(num_joints):
            if joint_idx in revolute_to_kuka:
                # This is one of the 7 KUKA revolute joints - use epsilon-shrunk limits
                kuka_name = revolute_to_kuka[joint_idx]
                limits = JOINT_LIMITS[kuka_name]

                # Apply epsilon-clamping to prevent overshoot
                ll, uu, jr = _epsilon_limits(limits, eps_rad)
                lower_limits.append(ll)
                upper_limits.append(uu)
                joint_ranges.append(jr)

                # Rest pose: use the value from initial_full for the matching revolute joint index
                # Find the revolute joint array index
                try:
                    rev_array_index = self.revolute_joint_indices.index(
                        joint_idx)
                except ValueError:
                    rev_array_index = None

                if rev_array_index is not None and rev_array_index < len(initial_full):
                    rest_poses.append(float(initial_full[rev_array_index]))
                else:
                    rest_poses.append(0.0)
            else:
                # Fixed or gripper joint - use very tight limits (effectively fixed)
                lower_limits.append(-0.01)
                upper_limits.append(0.01)
                joint_ranges.append(0.02)
                rest_poses.append(0.0)

        # Call PyBullet IK with proper joint limits, rest poses, and damping
        try:
            sol = p.calculateInverseKinematics(
                bodyUniqueId=self.robot_id,
                endEffectorLinkIndex=self.end_effector_link_index,
                targetPosition=target_position,
                targetOrientation=quat,
                lowerLimits=lower_limits,
                upperLimits=upper_limits,
                jointRanges=joint_ranges,
                restPoses=rest_poses,
                maxNumIterations=max_iterations,
                residualThreshold=tolerance,
                # Damping for stability near singularities
                jointDamping=[0.1] * num_joints,
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

        # Normalize angles into principal range to make validation more predictable
        def wrap_to_pi(x):
            """Wrap angle to [-pi, pi] range."""
            return (x + np.pi) % (2 * np.pi) - np.pi

        solution_revolute = np.asarray(solution_revolute, dtype=float)
        solution_revolute = wrap_to_pi(solution_revolute)

        # Return only the first 7 actuated joints (matching your robot)
        if len(solution_revolute) < 7:
            raise RuntimeError(
                "IK returned fewer than 7 revolute joint values.")
        solution_7 = solution_revolute[:7]

        # Unwrap to actual joint limits (try 2π shifts to fit in bounds)
        # This is critical because wrap_to_pi doesn't account for asymmetric joint limits
        kuka_limits = [JOINT_LIMITS[name] for name in joint_names]
        solution_7 = np.array(_unwrap_to_limits(
            solution_7.tolist(), kuka_limits))

        # Validate joint limits (should rarely fail now with epsilon-clamping + unwrapping)
        if not validate_joint_limits(solution_7.tolist()):
            logger.warning(
                f"IK solution violates joint limits after epsilon-clamping (ε={IK_EPSILON_MARGIN_DEG}°) "
                f"and unwrapping. Clamping to hard limits. Consider increasing epsilon margin.")
            solution_7 = np.array(clamp_joint_limits(solution_7.tolist()))
        else:
            logger.debug(
                f"✓ IK solution within limits (epsilon-clamped with {IK_EPSILON_MARGIN_DEG}° margin)")

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
        # Higher default for pose (orientation convergence needs more iterations)
        max_iterations: int = 150,
        tolerance: float = 1e-3,
        verify_orientation: bool = False,
        orientation_tolerance_deg: float = 2.0
    ) -> np.ndarray:
        """
        Solve IK for full 6-DOF pose (position + orientation).

        Inherits all Phase 1 epsilon-clamping benefits from solve_XYZ.
        Orientation tracking typically requires more iterations than position-only IK.

        IMPORTANT: Target orientation must be in WORLD frame at the TCP link, not robot base frame.

        Args:
            target_pose: [x, y, z, rx, ry, rz] where:
                - x, y, z: position in meters (world frame)
                - rx, ry, rz: Euler angles in radians (world frame, 'xyz' convention)
            current_joint_angles: Either 7-length or full revolute joint array
            max_iterations: IK solver iterations (default 150, higher than position-only)
            tolerance: Position error threshold in meters (default 1e-3)
            verify_orientation: If True, check and log orientation error after solving
            orientation_tolerance_deg: Warning threshold for orientation error (default 2.0°)

        Returns:
            numpy array of 7 joint angles (rad) that achieve the target pose

        Notes:
            - Orientation uses quaternion internally (converted from Euler 'xyz')
            - Same null-space limit enforcement as solve_XYZ (epsilon-clamping active)
            - For strict orientation accuracy, consider iterative refinement or lower damping
            - If orientation error is high, try increasing max_iterations or reducing tolerance
        """
        position = target_pose[:3]
        euler_angles = target_pose[3:6]
        rotation_matrix = R.from_euler('xyz', euler_angles).as_matrix()

        # Call solve_XYZ with orientation constraint
        solution = self.solve_XYZ(
            position,
            current_joint_angles,
            target_orientation=rotation_matrix,
            max_iterations=max_iterations,
            tolerance=tolerance
        )

        # Optional: Verify orientation error
        if verify_orientation:
            try:
                # Apply solution and compute achieved orientation
                full_joints = np.zeros(len(self.revolute_joint_indices))
                full_joints[:7] = solution
                self._set_joint_states_from_list(full_joints.tolist())

                link_state = p.getLinkState(
                    self.robot_id,
                    self.end_effector_link_index,
                    computeForwardKinematics=True,
                    physicsClientId=self.client
                )

                # orientation quaternion
                achieved_quat = np.array(link_state[5])
                achieved_rot = R.from_quat(achieved_quat).as_matrix()

                # Compute orientation error (angle between rotation matrices)
                # R_error = R_target^T @ R_achieved
                R_error = rotation_matrix.T @ achieved_rot
                angle_error = np.arccos(
                    np.clip((np.trace(R_error) - 1) / 2, -1, 1))
                angle_error_deg = np.rad2deg(angle_error)

                if angle_error_deg > orientation_tolerance_deg:
                    logger.warning(
                        f"Orientation error: {angle_error_deg:.2f}° (tolerance {orientation_tolerance_deg:.1f}°). "
                        f"Consider increasing max_iterations or using iterative refinement."
                    )
                else:
                    logger.debug(
                        f"✓ Orientation error: {angle_error_deg:.3f}° (within tolerance)")

            except Exception as ex:
                logger.debug(f"Orientation verification failed: {ex}")

        return solution

    def solve_pose_iterative(
        self,
        target_pose: List[float],
        current_joint_angles: List[float],
        max_outer_iterations: int = 3,
        max_ik_iterations: int = 150,
        position_tolerance: float = 1e-3,
        orientation_tolerance_deg: float = 1.0
    ) -> np.ndarray:
        """
        Iterative pose refinement for strict position AND orientation accuracy.

        Repeatedly invokes IK from the last solution until both position and orientation
        errors are below threshold, or max outer iterations reached.

        Use this when standard solve_pose doesn't achieve sufficient orientation accuracy.

        Args:
            target_pose: [x, y, z, rx, ry, rz] in world frame
            current_joint_angles: Initial joint configuration
            max_outer_iterations: Maximum refinement iterations (default 3)
            max_ik_iterations: IK solver iterations per refinement (default 150)
            position_tolerance: Position error threshold in meters (default 1e-3)
            orientation_tolerance_deg: Orientation error threshold in degrees (default 1.0°)

        Returns:
            numpy array of 7 joint angles that achieve target pose within tolerances
        """
        position = np.array(target_pose[:3])
        euler_angles = target_pose[3:6]
        target_rot = R.from_euler('xyz', euler_angles).as_matrix()
        target_quat = R.from_matrix(target_rot).as_quat()

        solution = np.array(current_joint_angles[:7] if len(
            current_joint_angles) >= 7 else [0.0]*7)

        for iteration in range(max_outer_iterations):
            # Solve IK from current best solution
            solution = self.solve_XYZ(
                position.tolist(),
                solution.tolist(),
                target_orientation=target_rot,
                max_iterations=max_ik_iterations,
                tolerance=position_tolerance
            )

            # Check both position and orientation errors
            full_joints = np.zeros(len(self.revolute_joint_indices))
            full_joints[:7] = solution
            self._set_joint_states_from_list(full_joints.tolist())

            link_state = p.getLinkState(
                self.robot_id,
                self.end_effector_link_index,
                computeForwardKinematics=True,
                physicsClientId=self.client
            )

            achieved_pos = np.array(link_state[4])
            achieved_quat = np.array(link_state[5])
            achieved_rot = R.from_quat(achieved_quat).as_matrix()

            # Position error
            pos_error = np.linalg.norm(achieved_pos - position)

            # Orientation error (geodesic distance on SO(3))
            R_error = target_rot.T @ achieved_rot
            angle_error = np.arccos(
                np.clip((np.trace(R_error) - 1) / 2, -1, 1))
            angle_error_deg = np.rad2deg(angle_error)

            logger.debug(
                f"Iteration {iteration+1}/{max_outer_iterations}: "
                f"pos_err={pos_error*1000:.2f}mm, orient_err={angle_error_deg:.2f}°"
            )

            # Check convergence
            if pos_error <= position_tolerance and angle_error_deg <= orientation_tolerance_deg:
                logger.info(
                    f"✓ Pose converged in {iteration+1} iteration(s): "
                    f"pos_err={pos_error*1000:.2f}mm, orient_err={angle_error_deg:.2f}°"
                )
                return solution

        # Max iterations reached - log final error
        logger.warning(
            f"Iterative pose refinement reached max iterations ({max_outer_iterations}). "
            f"Final errors: pos={pos_error*1000:.2f}mm, orient={angle_error_deg:.2f}°. "
            f"Target may be infeasible or consider increasing iterations."
        )

        return solution

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
