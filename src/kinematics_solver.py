from ikpy.chain import Chain
import numpy as np
import logging
from typing import List, Optional, Tuple
from config import JOINT_LIMITS, MAX_JOINT_VELOCITY, MAX_JOINT_ACCELERATION

logger = logging.getLogger(__name__)


class InverseKinematicsSolver:
    def __init__(self, urdf_filepath: str, base_elements: List[str], active_links_mask: List[bool]):
        try:
            self.chain = Chain.from_urdf_file(
                urdf_filepath,
                base_elements=base_elements,
                active_links_mask=active_links_mask
            )
            logger.info(
                f"Kinematics solver initialized with URDF: {urdf_filepath}")
        except Exception as e:
            logger.error(f"Failed to initialize kinematics solver: {e}")
            raise

        self.joint_limits = JOINT_LIMITS
        self.max_velocity = MAX_JOINT_VELOCITY
        self.max_acceleration = MAX_JOINT_ACCELERATION
        self.previous_joints = None

    def validate_joint_angles(self, joint_angles: List[float]) -> bool:
        """Validate joint angles against limits"""
        if len(joint_angles) != 7:
            logger.warning(f"Expected 7 joint angles, got {len(joint_angles)}")
            return False

        for i, angle in enumerate(joint_angles):
            if not isinstance(angle, (int, float)):
                logger.warning(f"Joint {i} has invalid type: {type(angle)}")
                return False

            if angle < self.joint_limits['min'][i] or angle > self.joint_limits['max'][i]:
                logger.warning(
                    f"Joint {i} angle {angle} exceeds limits [{self.joint_limits['min'][i]}, {self.joint_limits['max'][i]}]")
                return False

        return True

    def check_joint_velocity(self, current_joints: List[float], target_joints: List[float], dt: float) -> bool:
        """Check if joint velocity exceeds limits"""
        if self.previous_joints is None:
            self.previous_joints = current_joints
            return True

        velocities = np.abs(np.array(target_joints) -
                            np.array(current_joints)) / dt

        for i, vel in enumerate(velocities):
            if vel > self.max_velocity:
                logger.warning(
                    f"Joint {i} velocity {vel:.3f} exceeds limit {self.max_velocity}")
                return False

        return True

    def check_joint_acceleration(self, current_joints: List[float], target_joints: List[float], dt: float) -> bool:
        """Check if joint acceleration exceeds limits"""
        if self.previous_joints is None:
            self.previous_joints = current_joints
            return True

        current_velocities = np.array(
            current_joints) - np.array(self.previous_joints) / dt
        target_velocities = np.array(
            target_joints) - np.array(current_joints) / dt
        accelerations = np.abs(target_velocities - current_velocities) / dt

        for i, acc in enumerate(accelerations):
            if acc > self.max_acceleration:
                logger.warning(
                    f"Joint {i} acceleration {acc:.3f} exceeds limit {self.max_acceleration}")
                return False

        return True

    def solve_tcp(self, current_joint_angles: List[float]) -> Optional[np.ndarray]:
        """Solve forward kinematics to get TCP pose"""
        try:
            if not self.validate_joint_angles(current_joint_angles):
                logger.error("Invalid joint angles for forward kinematics")
                return None

            # Ensure we have the right number of joints for the chain
            chain_joints = self._prepare_joints_for_chain(current_joint_angles)

            result = self.chain.forward_kinematics(chain_joints)
            logger.debug(f"Forward kinematics solved successfully")
            return result

        except Exception as e:
            logger.error(f"Forward kinematics failed: {e}")
            return None

    def solve_XYZ(self, target_position: List[float], current_joint_angles: List[float]) -> Optional[List[float]]:
        """Solve inverse kinematics to get joint angles"""
        try:
            if not self.validate_joint_angles(current_joint_angles):
                logger.error(
                    "Invalid current joint angles for inverse kinematics")
                return None

            # Validate target position
            if len(target_position) != 3:
                logger.error(
                    f"Target position must have 3 elements, got {len(target_position)}")
                return None

            # Ensure we have the right number of joints for the chain
            chain_joints = self._prepare_joints_for_chain(current_joint_angles)

            # Solve IK
            result = self.chain.inverse_kinematics(
                target_position,
                initial_position=chain_joints
            )

            # Extract the 7 active joints (skip base and end effector)
            active_joints = result[1:8]

            # Validate the solution
            if not self.validate_joint_angles(active_joints):
                logger.warning("IK solution contains invalid joint angles")
                return None

            logger.debug(f"Inverse kinematics solved successfully")
            return active_joints

        except Exception as e:
            logger.error(f"Inverse kinematics failed: {e}")
            return None

    def _prepare_joints_for_chain(self, joint_angles: List[float]) -> List[float]:
        """Prepare joint angles for the chain (add base and end effector)"""
        # Add base joint (fixed at 0) and end effector (fixed at 0)
        chain_joints = [0.0] + joint_angles + [0.0]
        return chain_joints

    def update_previous_joints(self, joints: List[float]):
        """Update previous joint positions for velocity/acceleration checking"""
        self.previous_joints = joints.copy()

    def get_joint_limits(self) -> dict:
        """Get current joint limits"""
        return self.joint_limits.copy()

    def set_joint_limits(self, limits: dict):
        """Set new joint limits"""
        if 'min' in limits and 'max' in limits:
            if len(limits['min']) == 7 and len(limits['max']) == 7:
                self.joint_limits = limits.copy()
                logger.info("Joint limits updated")
            else:
                logger.warning("Joint limits must have exactly 7 values")
        else:
            logger.warning("Joint limits must contain 'min' and 'max' keys")
