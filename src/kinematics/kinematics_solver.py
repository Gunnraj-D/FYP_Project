"""
Streamlined PyBullet IK solver for KUKA iiwa with facing-down constraint.
"""

import numpy as np
import logging
import pybullet as p
import pybullet_data
from scipy.spatial.transform import Rotation as R
from typing import List, Optional, Tuple
import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


logger = logging.getLogger(__name__)


def get_facing_down_orientation() -> np.ndarray:
    """Returns rotation matrix for tool pointing down (-Z)."""
    return np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])


def get_facing_down_with_yaw_freedom(yaw_rad: float = 0.0) -> np.ndarray:
    """
    Returns rotation matrix for tool facing down with free yaw rotation.
    Useful for tasks where gripper rotation around vertical axis doesn't matter.
    """
    base = get_facing_down_orientation()
    cos_y, sin_y = np.cos(yaw_rad), np.sin(yaw_rad)
    yaw_rot = np.array([[cos_y, -sin_y, 0], [sin_y, cos_y, 0], [0, 0, 1]])
    return base @ yaw_rot


class InverseKinematicsSolver:
    """
    Kinematics solver wrapper using PyBullet.
    Keeps a pybullet client open for fast FK/IK queries.
    """

    def __init__(self, urdf_filepath: str, base_elements: Optional[List[str]] = None, active_links_mask: Optional[List[bool]] = None, use_gui: bool = False):
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

            if len(self.revolute_joint_indices) > 7:
                logger.info(
                    f"Detected {len(self.revolute_joint_indices) - 7} extra revolute joints (likely gripper) - IK will focus on first 7 arm joints")

            # Cache arm joint limits (min, max, range)
            self.arm_joint_limits = {}
            for i in range(7):
                joint_name = self.revolute_joint_names[i] if i < len(
                    self.revolute_joint_names) else None
                if joint_name:
                    limits = p.getJointInfo(
                        self.robot_id, self.revolute_joint_indices[i], physicsClientId=self.client)[8:11]
                    self.arm_joint_limits[joint_name] = {
                        'min': limits[0],
                        'max': limits[1],
                        'range': limits[1] - limits[0]
                    }
                else:
                    self.arm_joint_limits[f'joint_a{i+1}'] = {
                        'min': -np.pi,
                        'max': np.pi,
                        'range': 2 * np.pi
                    }

            # Compatibility fields
            self.base_elements = base_elements or []
            self.active_links_mask = active_links_mask or []
            # Build a best-effort movable_joint_indices mapping (indices into a "full chain" style array).
            # If active_links_mask length matches number of revolute joints, map 1-to-1; otherwise create a trivial mapping.
            try:
                if self.active_links_mask and len(self.active_links_mask) == len(self.revolute_joint_indices):
                    self.movable_joint_indices = [
                        i for i, v in enumerate(self.active_links_mask) if v]
                else:
                    # fallback: first N revolute joints
                    self.movable_joint_indices = list(
                        range(len(self.revolute_joint_indices)))
            except Exception:
                self.movable_joint_indices = list(
                    range(len(self.revolute_joint_indices)))

            # Ensure we can handle typical 7-DOF iiwa
            if len(self.revolute_joint_indices) < 7:
                raise ValueError(
                    f"URDF must have at least 7 revolute joints for arm kinematics. Found {len(self.revolute_joint_indices)}. Use the full arm + gripper URDF.")

        except Exception as e:
            logger.error(f"Failed to initialize PyBullet IK solver: {e}")
            raise

    def disconnect(self):
        try:
            p.disconnect(physicsClientId=self.client)
        except:
            pass

    def _set_joints(self, q: List[float]):
        """Set joint positions for FK."""
        for idx, j in enumerate(self.revolute_joint_indices[:len(q)]):
            p.resetJointState(self.robot_id, j,
                              q[idx], physicsClientId=self.client)

    def _get_tcp_transform(self, q: List[float]) -> np.ndarray:
        """Compute 4x4 TCP transform from joint angles."""
        self._set_joints(q)
        link_state = p.getLinkState(self.robot_id, self.end_effector_link_index, computeForwardKinematics=True,
                                    physicsClientId=self.client)
        pos = np.array(link_state[4])
        rot = R.from_quat(np.array(link_state[5])).as_matrix()
        T = np.eye(4)
        T[:3, :3] = rot
        T[:3, 3] = pos
        return T

    def tcp_from_joints(self, q7: List[float]) -> Tuple[np.ndarray, List[float]]:
        """Get TCP matrix and pose [x,y,z,rx,ry,rz] from 7 joint angles."""
        if len(q7) != 7:
            raise ValueError(f"Expected 7 angles, got {len(q7)}")
        T = self._get_tcp_transform(q7)
        pose = T[:3, 3].tolist() + R.from_matrix(T[:3, :3]
                                                 ).as_euler('xyz').tolist()
        return T, pose

    def _build_joint_limits(self, initial: np.ndarray) -> Tuple[List[float], List[float], List[float], List[float]]:
        lower = [0.0] * self.num_joints
        upper = [0.0] * self.num_joints
        joint_ranges = [0.0] * self.num_joints
        rest_poses = [0.0] * self.num_joints
        for j in range(self.num_joints):
            info = p.getJointInfo(
                self.robot_id, j, physicsClientId=self.client)
            joint_type = info[2]
            q_min = info[8]
            q_max = info[9]
            if joint_type == p.JOINT_FIXED:
                lower[j] = q_min
                upper[j] = q_max
                joint_ranges[j] = 0.0
                rest_poses[j] = 0.0
            elif joint_type in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
                if q_min >= q_max:
                    if joint_type == p.JOINT_REVOLUTE:
                        q_min = -np.pi
                        q_max = np.pi
                    else:
                        q_min = -0.1
                        q_max = 0.1
                lower[j] = q_min
                upper[j] = q_max
                joint_ranges[j] = q_max - q_min
                if j in self.revolute_joint_indices[:7]:
                    arm_idx = self.revolute_joint_indices[:7].index(j)
                    rest_poses[j] = initial[arm_idx]
                else:
                    rest_poses[j] = info[11] if info[11] != 0.0 else 0.0
            else:
                lower[j] = 0.0
                upper[j] = 0.0
                joint_ranges[j] = 0.0
                rest_poses[j] = 0.0
        return lower, upper, joint_ranges, rest_poses

    def solve_XYZ(self, target_pos: List[float], current_q: List[float],
                  target_rot: Optional[np.ndarray] = None, max_iter: int = 100,
                  tol: float = 1e-3) -> np.ndarray:
        """
        Solve IK for position (and optional orientation).

        Args:
            target_pos: [x, y, z] target in meters
            current_q: Current joint angles (7 or full length)
            target_rot: Optional 3x3 rotation matrix
            max_iter: Max iterations
            tol: Position tolerance in meters

        Returns:
            7-element joint angle array
        """
        # Normalize input
        if len(current_q) == 7:
            initial = np.zeros(len(self.revolute_joint_indices))
            initial[:7] = current_q
        else:
            initial = np.asarray(current_q, dtype=float)

        # Seed robot to current config
        self._set_joints(initial.tolist())

        # Build limits with epsilon-clamping
        ll, uu, jr, rp = self._build_joint_limits(initial)

        # Convert orientation to quat
        quat = None
        if target_rot is not None:
            if target_rot.shape != (3, 3):
                raise ValueError("Orientation must be 3x3 matrix")
            quat = R.from_matrix(target_rot).as_quat().tolist()

        # Solve IK
        try:
            sol = p.calculateInverseKinematics(
                bodyUniqueId=self.robot_id,
                endEffectorLinkIndex=self.end_effector_link_index,
                targetPosition=target_pos,
                targetOrientation=quat,
                lowerLimits=ll,
                upperLimits=uu,
                jointRanges=jr,
                restPoses=rp,
                maxNumIterations=min(max_iter, 200),
                residualThreshold=tol,
                jointDamping=[0.1] * self.num_joints,
                physicsClientId=self.client
            )
        except Exception as e:
            logger.error(f"IK failed: {e}")
            raise

        # Extract 7 revolute joints
        q7 = np.array([sol[self.revolute_joint_indices[i]] for i in range(7)])

        # Unwrap to actual limits
        kuka_limits = [self.arm_joint_limits[self.revolute_joint_names[i]]
                       for i in range(7)]

        # Hard clamp if needed
        for i, (angle, lim) in enumerate(zip(q7, kuka_limits)):
            if angle < lim['min'] or angle > lim['max']:
                logger.warning(
                    f"Arm joint A{i+1} out of bounds: {angle:.3f}, clamping to [{lim['min']:.3f}, {lim['max']:.3f}]")
                q7[i] = np.clip(angle, lim['min'], lim['max'])

        # Check achieved position against target for convergence
        T_achieved, achieved_pose = self.tcp_from_joints(q7.tolist())
        achieved_pos = T_achieved[:3, 3]
        pos_error = np.linalg.norm(achieved_pos - np.array(target_pos))
        if pos_error > tol:
            logger.warning(
                f"IK convergence check failed: position error {pos_error:.4f}m > tol {tol}m. Achieved: {achieved_pos}, Target: {target_pos}")

        return q7

    def solve_pose(self, target_pose: List[float], current_q: List[float],
                   max_iter: int = 150, tol: float = 1e-3) -> np.ndarray:
        """
        Solve IK for full 6-DOF pose [x, y, z, rx, ry, rz].

        Args:
            target_pose: Target in world frame
            current_q: Current joint config
            max_iter: Max iterations (higher for orientation)
            tol: Position tolerance

        Returns:
            7-element joint array
        """
        pos = target_pose[:3]
        euler = target_pose[3:6]
        rot = R.from_euler('xyz', euler).as_matrix()

        return self.solve_XYZ(pos, current_q, target_rot=rot, max_iter=max_iter, tol=tol)

    def solve_location_facing_down(self, target_pos: List[float], current_q: List[float],
                                   yaw_freedom: bool = True, yaw_guess: float = 0.0,
                                   max_iter: int = 100) -> np.ndarray:
        """
        Solve IK with gripper always facing downward.

        Args:
            target_pos: [x, y, z] target location
            current_q: Current joint angles
            yaw_freedom: If True, yaw rotation is free (gripper can spin around Z-axis)
            yaw_guess: Initial yaw estimate for the solver
            max_iter: Max iterations

        Returns:
            7-element joint array with gripper facing down
        """
        if yaw_freedom:
            rot = get_facing_down_with_yaw_freedom(yaw_guess)
        else:
            rot = get_facing_down_orientation()

        return self.solve_XYZ(target_pos, current_q, target_rot=rot, max_iter=max_iter)

    def compute_configuration_quality(self, q7: np.ndarray) -> float:
        """
        Compute quality score for a joint configuration.
        Higher score = better configuration (farther from limits, better posture).

        Args:
            q7: 7-element joint angle array

        Returns:
            Quality score (0-1, higher is better)
        """
        if len(q7) != 7:
            return 0.0

        scores = []

        # Score 1: Distance from joint limits (normalized)
        for i in range(7):
            joint_name = self.revolute_joint_names[i] if i < len(
                self.revolute_joint_names) else f'joint_a{i+1}'
            limits = self.arm_joint_limits.get(joint_name, {
                'min': -np.pi,
                'max': np.pi,
                'range': 2*np.pi
            })

            # Normalize angle to [0, 1] range within limits
            normalized = (q7[i] - limits['min']) / limits['range']
            # Distance from closest limit (0.5 = centered, 0 or 1 = at limit)
            limit_distance = 1.0 - 2.0 * abs(normalized - 0.5)
            scores.append(limit_distance)

        # Score 2: Penalize extreme joint 2 angles (elbow down = bad for table collision)
        # Joint A2 (index 1): prefer values around -0.5 to -1.5 (elbow up)
        if len(self.revolute_joint_names) > 1:
            a2_angle = q7[1]
            # Optimal range: -1.5 to -0.5 radians (elbow raised)
            if -1.5 <= a2_angle <= -0.5:
                elbow_score = 1.0
            elif -2.0 <= a2_angle < -1.5:
                elbow_score = 0.7
            elif -0.5 < a2_angle <= 0.0:
                elbow_score = 0.7
            else:
                # Positive A2 or very negative A2 = elbow pointing down (bad)
                elbow_score = 0.3
            scores.append(elbow_score * 1.5)  # Weight this score higher

        # Score 3: Prefer moderate joint velocities (smooth configurations)
        joint_velocity_penalty = np.sum(np.abs(q7)) / (7.0 * np.pi)
        smoothness_score = max(0.0, 1.0 - joint_velocity_penalty)
        scores.append(smoothness_score)

        # Weighted average
        quality = np.mean(scores)
        return np.clip(quality, 0.0, 1.0)

    def solve_with_yaw_search(self, target_pos: List[float], current_q: List[float],
                              n_yaw_samples: int = 12, max_iter: int = 150,
                              tol: float = 1e-3) -> List[Tuple[np.ndarray, float, float]]:
        """
        Search for IK solutions across multiple yaw angles.
        Exploits redundancy in 7-DOF arm for 6-DOF task (position + facing down).

        Args:
            target_pos: [x, y, z] target position
            current_q: Current joint angles (seed)
            n_yaw_samples: Number of yaw angles to sample
            max_iter: Max IK iterations per sample
            tol: Position tolerance

        Returns:
            List of (solution, quality_score, yaw_angle) tuples, sorted by quality (best first)
        """
        logger.info(
            f"Searching IK solutions with {n_yaw_samples} yaw samples for target {target_pos}")

        solutions = []

        # Sample yaw angles uniformly over 180 degrees
        # (180-360 is symmetric for most grasping tasks)
        yaw_angles = np.linspace(0, np.pi, n_yaw_samples, endpoint=False)

        for i, yaw in enumerate(yaw_angles):
            try:
                # Solve IK with this yaw angle
                rot = get_facing_down_with_yaw_freedom(yaw)
                q_solution = self.solve_XYZ(
                    target_pos=target_pos,
                    current_q=current_q,
                    target_rot=rot,
                    max_iter=max_iter,
                    tol=tol
                )

                # Verify convergence
                T_achieved, _ = self.tcp_from_joints(q_solution.tolist())
                achieved_pos = T_achieved[:3, 3]
                pos_error = np.linalg.norm(achieved_pos - np.array(target_pos))

                if pos_error <= tol * 2.0:  # Allow 2x tolerance for acceptance
                    # Compute quality score
                    quality = self.compute_configuration_quality(q_solution)
                    solutions.append((q_solution, quality, yaw))
                    logger.debug(
                        f"  Yaw {np.rad2deg(yaw):.1f}°: solution found (quality={quality:.3f}, error={pos_error*1000:.2f}mm)")
                else:
                    logger.debug(
                        f"  Yaw {np.rad2deg(yaw):.1f}°: failed convergence (error={pos_error*1000:.2f}mm)")

            except Exception as e:
                logger.debug(f"  Yaw {np.rad2deg(yaw):.1f}°: IK failed ({e})")
                continue

        # Sort by quality score (highest first)
        solutions.sort(key=lambda x: x[1], reverse=True)

        logger.info(
            f"Found {len(solutions)}/{n_yaw_samples} valid IK solutions")
        if solutions:
            best_quality = solutions[0][1]
            best_yaw = np.rad2deg(solutions[0][2])
            logger.info(
                f"Best solution: quality={best_quality:.3f}, yaw={best_yaw:.1f}°")

        return solutions

    def solve_with_position_and_yaw_search(
        self, target_pos: List[float], current_q: List[float],
        position_samples: int = 5, yaw_samples: int = 12,
        xy_perturbation: float = 0.02, z_perturbation: float = 0.03,
        max_iter: int = 150, tol: float = 1e-3
    ) -> List[Tuple[np.ndarray, float, np.ndarray, float]]:
        """
        Comprehensive IK search: sample both position perturbations AND yaw angles.

        Args:
            target_pos: [x, y, z] nominal target position
            current_q: Current joint angles
            position_samples: Number of position perturbations to try
            yaw_samples: Number of yaw angles per position
            xy_perturbation: Max XY perturbation (meters)
            z_perturbation: Max Z perturbation (meters)
            max_iter: Max IK iterations
            tol: Position tolerance

        Returns:
            List of (solution, quality, perturbed_pos, yaw) tuples, sorted by quality
        """
        logger.info(
            f"Comprehensive IK search: {position_samples} positions × {yaw_samples} yaws = {position_samples * yaw_samples} total samples")

        all_solutions = []

        # Generate position samples
        position_perturbations = []
        position_perturbations.append(
            np.array(target_pos))  # Original position first

        # Random perturbations
        np.random.seed(int(time.time() * 1000) % (2**32))
        for _ in range(position_samples - 1):
            perturbed = np.array(target_pos) + np.array([
                np.random.uniform(-xy_perturbation, xy_perturbation),
                np.random.uniform(-xy_perturbation, xy_perturbation),
                np.random.uniform(-z_perturbation, z_perturbation)
            ])
            position_perturbations.append(perturbed)

        # Try each position with yaw search
        for pos_idx, perturbed_pos in enumerate(position_perturbations):
            yaw_solutions = self.solve_with_yaw_search(
                target_pos=perturbed_pos.tolist(),
                current_q=current_q,
                n_yaw_samples=yaw_samples,
                max_iter=max_iter,
                tol=tol
            )

            # Add position info to solutions
            for solution, quality, yaw in yaw_solutions:
                all_solutions.append((solution, quality, perturbed_pos, yaw))

        # Sort by quality
        all_solutions.sort(key=lambda x: x[1], reverse=True)

        logger.info(
            f"Total valid solutions: {len(all_solutions)}/{position_samples * yaw_samples}")

        return all_solutions

    def solve_path_with_relaxation(self, start_pos: List[float], end_pos: List[float],
                                   current_q: List[float], min_z: float = 0.0,
                                   max_iterations: int = 10) -> Tuple[np.ndarray, bool]:
        """
        Alternative: Let IK solver take easiest path, then retry with lifted waypoints if needed.

        This is faster for "easy" paths, but corrects when z constraint is violated.

        Returns:
            (joint_angles, success_bool)
        """
        start_pos = np.array(start_pos)
        end_pos = np.array(end_pos)

        for iteration in range(max_iterations):
            logger.info(f"Path solve iteration {iteration + 1}")

            # Adjust end_pos z if too low
            target_end = end_pos.copy()
            target_end[2] = max(target_end[2], min_z)

            if iteration == 0:
                # First try: direct solve (might take easy path that dips)
                q_solution = self.solve_location_facing_down(
                    target_end.tolist(),
                    current_q.tolist(),
                    max_iter=100
                )
            else:
                # Retry: insert an intermediate waypoint higher up
                lift_height = min_z + 0.05 + \
                    (iteration * 0.03)  # Progressively higher
                mid_pos = (start_pos + target_end) / 2
                mid_pos[2] = max(mid_pos[2], lift_height)

                logger.info(
                    f"  Retrying with lifted waypoint at z={lift_height:.3f}")

                q_mid = self.solve_location_facing_down(
                    mid_pos.tolist(),
                    current_q.tolist(),
                    max_iter=100
                )

                q_solution = self.solve_location_facing_down(
                    target_end.tolist(),
                    q_mid.tolist(),
                    max_iter=100
                )

            # Check if solution is valid
            T, _ = self.tcp_from_joints(q_solution.tolist())
            final_z = T[2, 3]

            if final_z >= min_z - 0.01:  # 1cm tolerance
                logger.info(f"✓ Valid solution found (z={final_z:.3f})")
                return q_solution, True
            else:
                logger.warning(
                    f"  Solution violates z constraint (z={final_z:.3f})")

        logger.error(
            f"Failed to find valid path after {max_iterations} iterations")
        return q_solution, False

    def __del__(self):
        self.disconnect()


# Quick test
if __name__ == "__main__":
    import os
    logging.basicConfig(level=logging.INFO)
    # Default URDF path relative to project root (works from any subdir)
    default_urdf = os.path.join(os.path.dirname(os.path.dirname(
        __file__)), 'resources', 'robot_models', 'iiwa14_with_robotiq85.urdf')
    urdf = os.environ.get("URDF_FILEPATH", default_urdf)
    print(f"Loading URDF: {os.path.abspath(urdf)}")  # Debug: Confirm path
    # Call without extra kwargs for minimal signature
    solver = InverseKinematicsSolver(urdf_filepath=urdf, use_gui=False)
    sol7 = solver.solve_XYZ([0.5, 0.0, 0.6], [0.0]*7,
                            get_facing_down_orientation())
    print("Solution (7):", sol7)
    solver.disconnect()
