"""
Collision-aware kinematics solver for KUKA iiwa robot using PyBullet.
Implements collision avoidance techniques to prevent the robot from hitting the table
during motion by using:
1. Nullspace IK with rest poses to bias solutions away from table
2. Pre-approach waypoints above target position
3. Collision checking during trajectory interpolation
4. Multiple IK candidate sampling and selection based on clearance

This is a drop-in replacement for the standard kinematics solver with enhanced safety.
"""
import numpy as np
import logging
import pybullet as p
from typing import List, Optional, Tuple, Dict, Any
from scipy.spatial.transform import Rotation as R

from config.config import (
    JOINT_LIMITS,
    COLLISION_AVOIDANCE_CONFIG,
    REST_POSES,
    COLLISION_CHECK_LINKS
)
from .kinematics_solver import (
    InverseKinematicsSolver,
    validate_joint_limits,
    clamp_joint_limits,
    homogeneous_to_pose,
    get_facing_down_orientation
)

logger = logging.getLogger(__name__)


class CollisionAwareKinematicsSolver(InverseKinematicsSolver):
    """
    Enhanced kinematics solver with collision avoidance capabilities.
    Inherits from InverseKinematicsSolver and adds collision-aware features.
    """

    def __init__(self, urdf_filepath: str, base_elements: List[str],
                 active_links_mask: List[bool], use_gui: bool = False,
                 table_id: Optional[int] = None):
        """
        Initialize collision-aware kinematics solver.

        Args:
            urdf_filepath: Path to robot URDF file
            base_elements: Base elements for kinematic chain
            active_links_mask: Mask for active links
            use_gui: Whether to show PyBullet GUI
            table_id: PyBullet body ID of the table (for collision checking)
        """
        super().__init__(urdf_filepath, base_elements, active_links_mask, use_gui)

        self.table_id = table_id
        self.collision_config = COLLISION_AVOIDANCE_CONFIG
        self.rest_poses = REST_POSES
        self.collision_links = COLLISION_CHECK_LINKS

        # Build joint limits arrays for nullspace IK
        self._build_joint_limits_arrays()

        logger.info("Collision-aware kinematics solver initialized")

    def _build_joint_limits_arrays(self):
        """Build joint limits arrays for nullspace IK."""
        joint_names = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7']

        self.lower_limits = []
        self.upper_limits = []
        self.joint_ranges = []

        for joint_name in joint_names:
            limits = JOINT_LIMITS[joint_name]
            self.lower_limits.append(limits['min'])
            self.upper_limits.append(limits['max'])
            self.joint_ranges.append(limits['max'] - limits['min'])

    def _compute_clearance_score(self, joint_angles: List[float]) -> float:
        """
        Compute clearance score for given joint configuration.
        Higher score means better clearance from table.

        Args:
            joint_angles: Joint angles for the first 7 revolute joints

        Returns:
            Minimum clearance distance from table (meters)
        """
        if self.table_id is None:
            # If no table, return a default high score
            return 1.0

        # Set robot to the given joint configuration
        self._set_joint_states_from_list(joint_angles)

        min_distance = float('inf')

        # Check clearance for key links
        for link_name, link_idx in self.collision_links.items():
            try:
                # Get closest points between robot link and table
                closest_points = p.getClosestPoints(
                    self.robot_id,
                    self.table_id,
                    distance=self.collision_config['collision_check_distance'],
                    linkIndexA=link_idx,
                    physicsClientId=self.client
                )

                if closest_points:
                    # Get minimum distance from all contact points
                    for cp in closest_points:
                        contact_distance = cp[8]  # Contact distance
                        min_distance = min(min_distance, contact_distance)
                else:
                    # No contact points within check distance
                    min_distance = min(
                        min_distance, self.collision_config['collision_check_distance'])

            except Exception as e:
                logger.debug(f"Error checking clearance for {link_name}: {e}")
                continue

        return min_distance if min_distance != float('inf') else 0.0

    def _sample_ik_candidates(self, target_position: List[float],
                              target_orientation: Optional[np.ndarray] = None,
                              current_joint_angles: Optional[List[float]] = None) -> List[Tuple[np.ndarray, float]]:
        """
        Sample multiple IK candidates using different rest poses.

        Args:
            target_position: Target end-effector position [x, y, z]
            target_orientation: Optional target orientation (3x3 matrix)
            current_joint_angles: Current joint angles for initial guess

        Returns:
            List of (joint_angles, clearance_score) tuples
        """
        candidates = []

        # Convert orientation to quaternion if provided
        quat = None
        if target_orientation is not None:
            quat = self._rotation_matrix_to_quat(target_orientation)

        # Prepare initial guess
        if current_joint_angles is not None:
            if len(current_joint_angles) == 7:
                initial_guess = np.array(current_joint_angles)
            else:
                initial_guess = np.array(current_joint_angles[:7])
        else:
            initial_guess = np.zeros(7)

        # Try different rest poses
        rest_pose_names = list(self.rest_poses.keys())
        max_candidates = min(
            self.collision_config['max_ik_candidates'], len(rest_pose_names))

        for i in range(max_candidates):
            rest_pose_name = rest_pose_names[i % len(rest_pose_names)]
            rest_pose = self.rest_poses[rest_pose_name]

            try:
                # Compute IK with nullspace bias
                solution = p.calculateInverseKinematics(
                    bodyUniqueId=self.robot_id,
                    endEffectorLinkIndex=self.end_effector_link_index,
                    targetPosition=target_position,
                    targetOrientation=quat,
                    lowerLimits=self.lower_limits,
                    upperLimits=self.upper_limits,
                    jointRanges=self.joint_ranges,
                    restPoses=rest_pose,
                    maxNumIterations=100,
                    residualThreshold=1e-4,
                    physicsClientId=self.client
                )

                # Extract first 7 joint angles
                solution_7 = np.array(solution[:7])

                # Validate joint limits
                if validate_joint_limits(solution_7.tolist()):
                    # Compute clearance score
                    clearance_score = self._compute_clearance_score(
                        solution_7.tolist())
                    candidates.append((solution_7, clearance_score))

            except Exception as e:
                logger.debug(f"IK failed for rest pose {rest_pose_name}: {e}")
                continue

        # Sort by clearance score (highest first)
        candidates.sort(key=lambda x: x[1], reverse=True)

        return candidates

    def _interpolate_trajectory(self, start_angles: List[float],
                                end_angles: List[float]) -> List[List[float]]:
        """
        Interpolate trajectory between start and end joint angles.

        Args:
            start_angles: Starting joint angles
            end_angles: Ending joint angles

        Returns:
            List of joint angle waypoints
        """
        steps = self.collision_config['trajectory_interpolation_steps']
        trajectory = []

        for i in range(steps + 1):
            alpha = i / steps
            interpolated = np.array(start_angles) + alpha * \
                (np.array(end_angles) - np.array(start_angles))
            trajectory.append(interpolated.tolist())

        return trajectory

    def _check_trajectory_collision(self, trajectory: List[List[float]]) -> bool:
        """
        Check if trajectory has any collisions.

        Args:
            trajectory: List of joint angle waypoints

        Returns:
            True if trajectory is collision-free, False otherwise
        """
        for waypoint in trajectory:
            if not self._is_safe_configuration(waypoint):
                return False
        return True

    def _is_safe_configuration(self, joint_angles: List[float]) -> bool:
        """
        Check if a joint configuration is safe (no collision with table).

        Args:
            joint_angles: Joint angles for first 7 revolute joints

        Returns:
            True if safe, False if collision risk
        """
        clearance = self._compute_clearance_score(joint_angles)
        return clearance >= self.collision_config['min_clearance_distance']

    def solve_XYZ_collision_aware(
        self,
        target_position: List[float],
        current_joint_angles: List[float],
        target_orientation: Optional[np.ndarray] = None,
        use_pre_approach: bool = True,
        max_iterations: int = 100,
        tolerance: float = 1e-4
    ) -> Tuple[np.ndarray, List[List[float]]]:
        """
        Solve IK with collision avoidance and return trajectory.

        Args:
            target_position: Target end-effector position [x, y, z]
            current_joint_angles: Current joint angles
            target_orientation: Optional target orientation (3x3 matrix)
            use_pre_approach: Whether to use pre-approach waypoint
            max_iterations: Maximum IK iterations
            tolerance: IK tolerance

        Returns:
            Tuple of (final_joint_angles, trajectory_waypoints)
        """
        logger.info(
            f"Solving collision-aware IK for position: {target_position}")

        # Sample multiple IK candidates
        candidates = self._sample_ik_candidates(
            target_position, target_orientation, current_joint_angles
        )

        if not candidates:
            raise RuntimeError("No valid IK solutions found")

        # Select best candidate based on clearance
        best_angles, best_clearance = candidates[0]
        logger.info(
            f"Selected IK solution with clearance: {best_clearance:.4f}m")

        # Generate trajectory
        trajectory = []

        if use_pre_approach:
            # Create pre-approach waypoint above target
            pre_approach_pos = np.array(
                target_position) + np.array([0, 0, self.collision_config['pre_approach_height_offset']])

            # Find IK solution for pre-approach position
            pre_candidates = self._sample_ik_candidates(
                pre_approach_pos.tolist(), target_orientation, current_joint_angles
            )

            if pre_candidates:
                pre_approach_angles = pre_candidates[0][0]

                # Interpolate from current to pre-approach
                traj1 = self._interpolate_trajectory(
                    current_joint_angles, pre_approach_angles.tolist())

                # Interpolate from pre-approach to final
                traj2 = self._interpolate_trajectory(
                    pre_approach_angles.tolist(), best_angles.tolist())

                trajectory = traj1 + traj2[1:]  # Skip duplicate waypoint
            else:
                logger.warning(
                    "No valid pre-approach solution found, using direct trajectory")
                trajectory = self._interpolate_trajectory(
                    current_joint_angles, best_angles.tolist())
        else:
            # Direct trajectory
            trajectory = self._interpolate_trajectory(
                current_joint_angles, best_angles.tolist())

        # Check trajectory for collisions
        if not self._check_trajectory_collision(trajectory):
            logger.warning(
                "Trajectory has collision risk, attempting to find safer path")

            # Try with higher pre-approach if we used it
            if use_pre_approach:
                logger.info("Retrying with higher pre-approach position")
                return self.solve_XYZ_collision_aware(
                    target_position, current_joint_angles, target_orientation,
                    use_pre_approach=True, max_iterations=max_iterations, tolerance=tolerance
                )
            else:
                raise RuntimeError(
                    "Trajectory has collision risk and no pre-approach available")

        logger.info(
            f"Generated collision-free trajectory with {len(trajectory)} waypoints")
        return best_angles, trajectory

    def solve_pose_collision_aware(
        self,
        target_pose: List[float],
        current_joint_angles: List[float],
        use_pre_approach: bool = True,
        max_iterations: int = 100,
        tolerance: float = 1e-4
    ) -> Tuple[np.ndarray, List[List[float]]]:
        """
        Solve IK for target pose with collision avoidance.

        Args:
            target_pose: Target pose [x, y, z, rx, ry, rz] (position + euler angles)
            current_joint_angles: Current joint angles
            use_pre_approach: Whether to use pre-approach waypoint
            max_iterations: Maximum IK iterations
            tolerance: IK tolerance

        Returns:
            Tuple of (final_joint_angles, trajectory_waypoints)
        """
        position = target_pose[:3]
        euler_angles = target_pose[3:6]
        rotation_matrix = R.from_euler('xyz', euler_angles).as_matrix()

        return self.solve_XYZ_collision_aware(
            position, current_joint_angles, rotation_matrix,
            use_pre_approach, max_iterations, tolerance
        )

    def execute_trajectory_safely(self, trajectory: List[List[float]],
                                  execution_callback: callable) -> bool:
        """
        Execute trajectory with safety checks at each waypoint.

        Args:
            trajectory: List of joint angle waypoints
            execution_callback: Function to call for each waypoint (joint_angles) -> bool

        Returns:
            True if trajectory executed successfully, False otherwise
        """
        logger.info(f"Executing trajectory with {len(trajectory)} waypoints")

        for i, waypoint in enumerate(trajectory):
            # Check safety before executing waypoint
            if not self._is_safe_configuration(waypoint):
                logger.error(f"Unsafe configuration detected at waypoint {i}")
                return False

            # Execute waypoint
            try:
                success = execution_callback(waypoint)
                if not success:
                    logger.error(f"Failed to execute waypoint {i}")
                    return False
            except Exception as e:
                logger.error(f"Error executing waypoint {i}: {e}")
                return False

        logger.info("Trajectory executed successfully")
        return True


# Example usage and testing
if __name__ == "__main__":
    import os
    import logging

    logging.basicConfig(level=logging.INFO)

    # Load URDF
    urdf_path = os.environ.get(
        "URDF_FILEPATH", "src/resources/robot_models/kuka_with_gripper.urdf")

    # Create solver
    solver = CollisionAwareKinematicsSolver(
        urdf_filepath=urdf_path,
        base_elements=None,
        active_links_mask=None,
        use_gui=False,
        table_id=None  # No table for this test
    )

    # Test collision-aware IK
    target_pos = [0.5, 0.0, 0.6]
    current_angles = [0.0] * 7
    target_orientation = get_facing_down_orientation()

    try:
        final_angles, trajectory = solver.solve_XYZ_collision_aware(
            target_pos, current_angles, target_orientation
        )
        print(f"Final joint angles: {final_angles}")
        print(f"Trajectory length: {len(trajectory)} waypoints")

    except Exception as e:
        print(f"Error: {e}")

    finally:
        solver.disconnect()
