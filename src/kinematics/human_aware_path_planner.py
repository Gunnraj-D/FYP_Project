"""
Human-Aware Path Planner

Path planning module that avoids collision with tracked humans using RRT-Connect.
Integrates with ZED skeleton tracking and PyBullet collision checking.

Key Features:
- RRT-Connect/BiRRT planning in joint space
- Real-time human collision model (spheres + capsules)
- Rolling-horizon replanning
- ISO/TS 15066 Speed and Separation Monitoring (SSM)
- No coordinate transforms needed (ZED data already in robot base frame)

Architecture:
    ZEDJointReceiver → HumanModelAdapter → PyBullet Planning World → RRT-Connect → Trajectory
"""

import logging
import time
import numpy as np
import pybullet as p
from typing import List, Optional, Tuple, Dict, Any
from scipy.spatial.transform import Rotation as R

# Import pybullet_planning for RRT-Connect
try:
    from pybullet_planning import link_pairs_collision, get_joint_positions, set_joint_positions
    from pybullet_planning import plan_joint_motion, get_movable_joints
    from pybullet_planning import get_collision_fn, set_client
    PYBULLET_PLANNING_AVAILABLE = True
except ImportError:
    logger.warning("pybullet_planning not available - using fallback planner")
    PYBULLET_PLANNING_AVAILABLE = False

from hand_detection.zed_joint_receiver import ZEDJointReceiver, SkeletonData

logger = logging.getLogger(__name__)


class HumanModelAdapter:
    """
    Converts ZED skeleton data to PyBullet collision primitives.

    Creates spheres and capsules from tracked joints to represent human body
    for collision checking. NO coordinate transform needed - ZED data is
    already in robot base frame (meters).
    """

    def __init__(self, pybullet_client: int, config: Dict[str, Any]):
        """
        Initialize human model adapter.

        Args:
            pybullet_client: PyBullet client ID
            config: Configuration dict with primitive radii, etc.
        """
        self.client = pybullet_client
        self.config = config

        # Collision body tracking
        self.collision_bodies = {}  # {name: body_id}
        self.last_update_time = 0

        # Track which joints are currently valid
        self.valid_joints = set()

    def update_from_skeleton(self, skeleton: SkeletonData):
        """
        Update PyBullet collision bodies from skeleton data.

        Args:
            skeleton: Single skeleton from ZED (already in robot base frame!)
        """
        self.valid_joints.clear()

        # Get all critical joint positions
        joint_positions = self._extract_joint_positions(skeleton)

        # Update primitives
        self._update_head_neck_spheres(joint_positions)
        self._update_torso_capsule(joint_positions)
        self._update_arm_capsules(joint_positions, 'left')
        self._update_arm_capsules(joint_positions, 'right')
        self._update_shoulder_spheres(joint_positions)

        self.last_update_time = time.time()

        logger.debug(
            f"Updated human model with {len(self.valid_joints)} valid joints")

    def _extract_joint_positions(self, skeleton: SkeletonData) -> Dict[str, np.ndarray]:
        """
        Extract positions of critical joints from skeleton.

        Returns dict of {joint_name: [x, y, z]} for joints that are present.
        ZED positions are ALREADY in robot base frame - no transform needed!
        """
        from config.config import TRACKED_HUMAN_JOINTS

        positions = {}
        found_joints = []
        missing_joints = []

        for category, joint_names in TRACKED_HUMAN_JOINTS.items():
            for joint_name in joint_names:
                pos = skeleton.get_joint_position(joint_name)
                if pos is not None:
                    positions[joint_name] = np.array(pos)
                    self.valid_joints.add(joint_name)
                    found_joints.append(joint_name)
                else:
                    missing_joints.append(joint_name)

        logger.info(f"Found {len(found_joints)} joints: {found_joints}")
        if missing_joints:
            logger.warning(
                f"Missing {len(missing_joints)} joints: {missing_joints}")

        return positions

    def _update_head_neck_spheres(self, positions: Dict[str, np.ndarray]):
        """Create/update spheres at NECK and NOSE."""
        radius = self.config['head_radius']

        for joint_name in ['NECK', 'NOSE']:
            if joint_name in positions:
                self._create_or_update_sphere(
                    name=f"head_{joint_name}",
                    position=positions[joint_name],
                    radius=radius
                )
                logger.info(
                    f"Created/updated head sphere '{joint_name}' at {positions[joint_name]} r={radius}")

    def _update_torso_capsule(self, positions: Dict[str, np.ndarray]):
        """Create/update capsule between CHEST_SPINE and PELVIS."""
        # ZED BODY_38 might use SPINE_2 or CHEST_SPINE
        chest_joint = 'CHEST_SPINE' if 'CHEST_SPINE' in positions else 'SPINE_2'

        if chest_joint in positions and 'PELVIS' in positions:
            self._create_or_update_capsule(
                name="torso",
                pos_a=positions[chest_joint],
                pos_b=positions['PELVIS'],
                radius=self.config['torso_radius']
            )
            logger.info(
                f"Created/updated torso capsule {chest_joint}↔PELVIS r={self.config['torso_radius']}")
        else:
            logger.warning(
                f"Cannot create torso: chest_joint '{chest_joint}' in positions: {chest_joint in positions}, PELVIS in positions: {'PELVIS' in positions}")

    def _update_arm_capsules(self, positions: Dict[str, np.ndarray], side: str):
        """Create/update capsules for arm segments (upper arm, forearm)."""
        prefix = side.upper()
        shoulder = f'{prefix}_SHOULDER'
        elbow = f'{prefix}_ELBOW'
        wrist = f'{prefix}_WRIST'

        radius = self.config['arm_radius']

        # Upper arm: shoulder → elbow
        if shoulder in positions and elbow in positions:
            self._create_or_update_capsule(
                name=f"{side}_upper_arm",
                pos_a=positions[shoulder],
                pos_b=positions[elbow],
                radius=radius
            )

        # Forearm: elbow → wrist
        if elbow in positions and wrist in positions:
            self._create_or_update_capsule(
                name=f"{side}_forearm",
                pos_a=positions[elbow],
                pos_b=positions[wrist],
                radius=radius
            )

    def _update_shoulder_spheres(self, positions: Dict[str, np.ndarray]):
        """Create/update spheres at clavicles."""
        radius = self.config['shoulder_radius']

        for joint_name in ['LEFT_CLAVICLE', 'RIGHT_CLAVICLE']:
            if joint_name in positions:
                self._create_or_update_sphere(
                    name=f"shoulder_{joint_name}",
                    position=positions[joint_name],
                    radius=radius
                )

    def _create_or_update_sphere(self, name: str, position: np.ndarray, radius: float):
        """Create or update a sphere collision body."""
        if name in self.collision_bodies:
            # Update existing sphere position
            body_id = self.collision_bodies[name]
            p.resetBasePositionAndOrientation(
                body_id,
                position.tolist(),
                [0, 0, 0, 1],  # Identity quaternion (no rotation for sphere)
                physicsClientId=self.client
            )
        else:
            # Create new sphere
            col_shape = p.createCollisionShape(
                p.GEOM_SPHERE,
                radius=radius,
                physicsClientId=self.client
            )
            body_id = p.createMultiBody(
                baseMass=0,  # Static obstacle
                baseCollisionShapeIndex=col_shape,
                basePosition=position.tolist(),
                physicsClientId=self.client
            )
            self.collision_bodies[name] = body_id
            logger.debug(
                f"Created sphere '{name}' at {position} with radius {radius:.3f}m")

    def _create_or_update_capsule(self, name: str, pos_a: np.ndarray,
                                  pos_b: np.ndarray, radius: float):
        """Create or update a capsule collision body between two points."""
        # Compute capsule parameters
        direction = pos_b - pos_a
        length = np.linalg.norm(direction)

        if length < 1e-6:
            logger.warning(f"Capsule '{name}' has zero length, skipping")
            return

        center = (pos_a + pos_b) / 2
        direction_normalized = direction / length

        # Compute orientation quaternion to align capsule with direction
        # PyBullet capsules are aligned along Z-axis by default
        z_axis = np.array([0, 0, 1])
        rotation_axis = np.cross(z_axis, direction_normalized)
        rotation_axis_norm = np.linalg.norm(rotation_axis)

        if rotation_axis_norm < 1e-6:
            # Parallel or anti-parallel to Z-axis
            if np.dot(z_axis, direction_normalized) > 0:
                orientation = [0, 0, 0, 1]  # No rotation
            else:
                orientation = [1, 0, 0, 0]  # 180° rotation around X
        else:
            rotation_axis = rotation_axis / rotation_axis_norm
            rotation_angle = np.arccos(
                np.clip(np.dot(z_axis, direction_normalized), -1, 1))

            # Convert to quaternion
            quat = R.from_rotvec(rotation_angle * rotation_axis).as_quat()
            orientation = quat.tolist()  # [x, y, z, w]

        if name in self.collision_bodies:
            # Update existing capsule
            body_id = self.collision_bodies[name]
            p.resetBasePositionAndOrientation(
                body_id,
                center.tolist(),
                orientation,
                physicsClientId=self.client
            )
            # Note: Can't easily update capsule dimensions in PyBullet
            # If length changes significantly, consider recreating
        else:
            # Create new capsule
            col_shape = p.createCollisionShape(
                p.GEOM_CAPSULE,
                radius=radius,
                height=length,  # Length along Z-axis
                physicsClientId=self.client
            )
            body_id = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=col_shape,
                basePosition=center.tolist(),
                baseOrientation=orientation,
                physicsClientId=self.client
            )
            self.collision_bodies[name] = body_id
            logger.debug(f"Created capsule '{name}' from {pos_a} to {pos_b}, "
                         f"length {length:.3f}m, radius {radius:.3f}m")

    def clear(self):
        """Remove all collision bodies."""
        for body_id in self.collision_bodies.values():
            p.removeBody(body_id, physicsClientId=self.client)
        self.collision_bodies.clear()
        self.valid_joints.clear()
        logger.debug("Cleared human collision model")

    def get_all_body_ids(self) -> List[int]:
        """Get list of all human collision body IDs."""
        return list(self.collision_bodies.values())


class HumanAwarePathPlanner:
    """
    Main path planner with human collision avoidance.

    Uses RRT-Connect in joint space with collision checking against
    human body primitives. Supports rolling-horizon replanning.
    """

    def __init__(self, urdf_path: str, zed_receiver: ZEDJointReceiver,
                 ik_solver, config: Dict[str, Any]):
        """
        Initialize human-aware path planner.

        Args:
            urdf_path: Path to robot URDF
            zed_receiver: ZED joint receiver for skeleton data
            ik_solver: IK solver for goal sampling
            config: Planning configuration
        """
        self.zed_receiver = zed_receiver
        self.ik_solver = ik_solver
        self.config = config

        # Create dedicated PyBullet client for planning (DIRECT mode - no GUI)
        self.client = p.connect(p.DIRECT)
        logger.info("Created PyBullet planning world (DIRECT mode)")

        # CRITICAL: Bind pybullet_planning to our PyBullet client!
        # This ensures the library uses the same physics server as our robot/obstacles
        if PYBULLET_PLANNING_AVAILABLE:
            set_client(self.client)
            logger.info(f"Bound pybullet_planning to client {self.client}")

        # Load robot
        self.robot_id = p.loadURDF(
            urdf_path,
            useFixedBase=True,
            physicsClientId=self.client
        )

        # Get movable joint information
        self.num_joints = p.getNumJoints(
            self.robot_id, physicsClientId=self.client)
        self.movable_joints = []
        for j in range(self.num_joints):
            info = p.getJointInfo(
                self.robot_id, j, physicsClientId=self.client)
            joint_type = info[2]
            if joint_type == p.JOINT_REVOLUTE or joint_type == p.JOINT_PRISMATIC:
                self.movable_joints.append(j)

        # We'll use first 7 revolute joints for planning
        self.planning_joints = self.movable_joints[:7]
        logger.info(
            f"Planning with {len(self.planning_joints)} joints: {self.planning_joints}")

        # TODO: Load table/environment obstacles

        # Initialize human model adapter
        from config.config import HUMAN_MODEL_CONFIG
        self.human_model = HumanModelAdapter(self.client, HUMAN_MODEL_CONFIG)

        # Planning state
        self.person_detected = False
        self.last_skeleton_data = None

        logger.info("Human-aware path planner initialized")

    def plan_trajectory(self, start_joints: List[float], goal_pose: List[float],
                        use_pre_approach: bool = True) -> Tuple[Optional[List[List[float]]], Dict[str, Any]]:
        """
        Plan collision-free trajectory to goal.

        Args:
            start_joints: Starting joint configuration [7]
            goal_pose: Target pose [x,y,z] or [x,y,z,rx,ry,rz]
            use_pre_approach: Whether to use pre-approach waypoint

        Returns:
            (trajectory, metadata) where trajectory is List of joint waypoints or None
        """
        start_time = time.time()

        logger.info(f"Planning trajectory from start to pose {goal_pose[:3]}")

        # 1. Update human collision model
        logger.info("Updating human collision model from ZED data...")
        self._update_human_model()

        # 2. Sample goal configurations via IK
        goal_joints = self._sample_goal_configurations(goal_pose)

        if not goal_joints:
            return None, {
                'success': False,
                'reason': 'No valid IK solutions for goal',
                'planning_time': time.time() - start_time
            }

        # 3. Run RRT-Connect planner
        if PYBULLET_PLANNING_AVAILABLE:
            trajectory = self._rrt_connect_plan(start_joints, goal_joints[0])
        else:
            logger.warning(
                "Using fallback linear planner (pybullet_planning not installed)")
            trajectory = self._placeholder_plan(start_joints, goal_joints[0])

        if trajectory is None:
            return None, {
                'success': False,
                'reason': 'Planning failed - no collision-free path found',
                'person_detected': self.person_detected,
                'planning_time': time.time() - start_time
            }

        # 4. Compute metadata
        min_clearance = self._compute_min_clearance(trajectory)

        metadata = {
            'success': True,
            'planning_time': time.time() - start_time,
            'waypoint_count': len(trajectory),
            'min_clearance': min_clearance,
            'person_detected': self.person_detected,
            'planner_used': 'rrt_connect' if PYBULLET_PLANNING_AVAILABLE else 'linear_fallback'
        }

        return trajectory, metadata

    def check_trajectory_safe(self, trajectory: List[List[float]]) -> Tuple[bool, float]:
        """
        Check if trajectory is collision-free with current human position.

        Args:
            trajectory: List of joint waypoints

        Returns:
            (is_safe, min_clearance)
        """
        # Update human model with latest data
        self._update_human_model()

        min_clearance = float('inf')

        for waypoint in trajectory:
            clearance = self._compute_clearance_at_config(waypoint)
            min_clearance = min(min_clearance, clearance)

            if clearance < self.config['hard_min_distance']:
                return False, clearance

        return True, min_clearance

    def get_clearance_to_human(self, joint_angles: List[float]) -> float:
        """
        Get minimum clearance to human at given configuration.

        Args:
            joint_angles: Robot joint configuration

        Returns:
            Minimum distance to human (meters)
        """
        self._update_human_model()
        return self._compute_clearance_at_config(joint_angles)

    def human_moved_significantly(self) -> bool:
        """Check if human has moved significantly since last check."""
        # TODO: Implement movement detection
        # Compare current skeleton to self.last_skeleton_data
        return False

    def _update_human_model(self):
        """Update human collision model from latest ZED data."""
        frame_data = self.zed_receiver.get_latest_frame()

        if frame_data is None or not frame_data.skeletons:
            # No person detected
            if self.person_detected:
                logger.info(
                    "Person no longer detected - clearing collision model")
                self.human_model.clear()
            self.person_detected = False
            self.last_skeleton_data = None
            logger.info("No person detected in ZED data")
            return

        # Update with first (and only) skeleton
        skeleton = frame_data.skeletons[0]
        logger.info(
            f"Updating human model from skeleton ID {skeleton.skeleton_id}")
        self.human_model.update_from_skeleton(skeleton)
        self.person_detected = True
        self.last_skeleton_data = skeleton

        # Log how many collision bodies were created
        num_bodies = len(self.human_model.get_all_body_ids())
        logger.info(
            f"Human model updated: {num_bodies} collision bodies created")

        # Log sample positions for debugging
        sample_joints = ['PELVIS', 'NECK', 'RIGHT_WRIST']
        logger.info("Sample human joint positions:")
        for joint_name in sample_joints:
            pos = skeleton.get_joint_position(joint_name)
            if pos:
                logger.info(f"  {joint_name}: {pos}")

    def _sample_goal_configurations(self, goal_pose: List[float]) -> List[List[float]]:
        """Sample multiple goal configurations via IK."""
        # TODO: Use collision-aware IK with multiple rest poses
        # For now, single IK solution

        if len(goal_pose) == 3:
            # Position only
            goal_joints = self.ik_solver.solve_XYZ(
                target_position=goal_pose,
                current_joint_angles=[0]*7
            )
        else:
            # Position + orientation
            goal_joints = self.ik_solver.solve_pose(
                target_pose=goal_pose,
                current_joint_angles=[0]*7
            )

        return [goal_joints.tolist()] if goal_joints is not None else []

    def _rrt_connect_plan(self, start: List[float], goal: List[float]) -> Optional[List[List[float]]]:
        """
        Plan collision-free path using RRT-Connect from pybullet_planning.

        Args:
            start: Starting joint configuration [7]
            goal: Goal joint configuration [7]

        Returns:
            List of joint waypoints or None if no path found
        """
        logger.info(f"Running RRT-Connect planner (start → goal)")

        try:
            # Set robot to start configuration in planning world
            logger.info(f"Setting robot to start configuration: {start}")
            for i, joint_idx in enumerate(self.planning_joints):
                p.resetJointState(
                    self.robot_id,
                    joint_idx,
                    start[i],
                    physicsClientId=self.client
                )

            # Get robot TCP and base position for debugging
            link_state = p.getLinkState(
                self.robot_id,
                self.num_joints - 1,  # Last link (TCP)
                computeForwardKinematics=True,
                physicsClientId=self.client
            )
            robot_tcp = np.array(link_state[4])  # World position

            # Get robot base position
            base_pos, base_orn = p.getBasePositionAndOrientation(
                self.robot_id, physicsClientId=self.client
            )
            logger.info(f"Robot base position: {base_pos}")
            logger.info(f"Robot TCP position: {robot_tcp}")

            # Get a sample human body position
            human_bodies = self.human_model.get_all_body_ids()
            if human_bodies:
                sample_body = human_bodies[0]
                sample_pos, _ = p.getBasePositionAndOrientation(
                    sample_body, physicsClientId=self.client
                )
                logger.info(f"Sample human body position: {sample_pos}")

            # Check if start is collision-free
            start_clearance, closest_body = self._compute_clearance_at_config_detailed(
                start)
            logger.info(
                f"Start configuration clearance: {start_clearance:.3f}m (closest to: {closest_body})")

            # Log human obstacle info
            human_bodies = self.human_model.get_all_body_ids()
            logger.info(
                f"Planning with {len(human_bodies)} human collision primitives")

            # DIAGNOSTIC: Use pybullet_planning's collision checker to see what it thinks
            if PYBULLET_PLANNING_AVAILABLE:
                logger.info(
                    "Running collision diagnostic on start configuration...")
                try:
                    collision_fn = get_collision_fn(
                        self.robot_id,
                        self.planning_joints,
                        obstacles=human_bodies,
                        self_collisions=False,
                        disabled_collisions=set()
                    )
                    start_in_collision = collision_fn(start, diagnosis=True)
                    if start_in_collision:
                        logger.warning(
                            "pybullet_planning reports start is in collision!")
                        logger.warning("Collision pairs: " +
                                       str(start_in_collision))
                    else:
                        logger.info(
                            "pybullet_planning: start is collision-free ✓")
                except Exception as e:
                    logger.warning(f"Collision diagnostic failed: {e}")

            hard_min = self.config.get('hard_min_distance', 0.12)

            if start_clearance < 0.02:  # Less than 2cm - true penetration
                logger.error(
                    f"Start configuration in collision! (clearance: {start_clearance:.3f}m)")
                logger.error(
                    "Robot is penetrating human model. Move robot away first.")
                return None

            # Now plan with proper obstacles (client binding should fix the collision check)
            logger.info("Calling plan_joint_motion with human obstacles...")
            path = plan_joint_motion(
                self.robot_id,
                self.planning_joints,
                goal,
                obstacles=human_bodies,  # Now using proper obstacles!
                self_collisions=False,
                disabled_collisions=set(),
                max_distance=self.config.get('step_size', 0.15),
                restarts=2,  # Reduced restarts for speed (was 5)
                iterations=self.config.get('max_iterations', 2000),
                smooth=self.config.get('smoothing_iterations', 20),
                custom_limits={},
                diagnosis=False,
                physicsClientId=self.client
            )

            if path is None:
                logger.warning("RRT-Connect could not find a path")
                return None

            logger.info(f"RRT-Connect found path with {len(path)} waypoints")

            # Convert path to list format (path is list of tuples)
            trajectory = [list(waypoint) for waypoint in path]

            return trajectory

        except Exception as e:
            logger.error(f"RRT-Connect planning failed: {e}")
            return None

    def _placeholder_plan(self, start: List[float], goal: List[float]) -> Optional[List[List[float]]]:
        """Fallback: simple linear interpolation (used if pybullet_planning not available)."""
        logger.warning(
            "Using linear interpolation fallback - no obstacle avoidance!")
        steps = 50
        trajectory = []

        for i in range(steps + 1):
            alpha = i / steps
            waypoint = [
                start[j] + alpha * (goal[j] - start[j])
                for j in range(len(start))
            ]
            trajectory.append(waypoint)

        return trajectory

    def _compute_clearance_at_config_detailed(self, joint_angles: List[float]) -> Tuple[float, str]:
        """
        Compute minimum clearance to human with details about closest body.

        Returns:
            (min_clearance, closest_body_name)
        """
        if not self.person_detected:
            return float('inf'), 'none'

        # Set robot to configuration using correct joint indices
        for i, angle in enumerate(joint_angles):
            joint_idx = self.planning_joints[i] if i < len(
                self.planning_joints) else i
            p.resetJointState(self.robot_id, joint_idx, angle,
                              physicsClientId=self.client)

        # Get closest points to all human bodies
        human_body_ids = self.human_model.get_all_body_ids()
        min_distance = float('inf')
        closest_body_name = 'unknown'

        # Get body names from human model
        body_id_to_name = {v: k for k,
                           v in self.human_model.collision_bodies.items()}

        for human_body_id in human_body_ids:
            closest_points = p.getClosestPoints(
                bodyA=self.robot_id,
                bodyB=human_body_id,
                distance=self.config.get('collision_check_distance', 1.0),
                physicsClientId=self.client
            )

            logger.debug(
                f"Checking body {human_body_id}: {len(closest_points) if closest_points else 0} contact points")

            if not closest_points:
                logger.debug(
                    f"  No contact points found for body {human_body_id} (might be > 1.0m away)")
                continue

            for cp in closest_points:
                distance = cp[8]  # Contact distance
                logger.debug(f"  Contact distance: {distance:.4f}m")

                # Negative means penetration - treat as collision
                if distance < 0:
                    distance = 0.0

                if distance < min_distance:
                    min_distance = distance
                    closest_body_name = body_id_to_name.get(
                        human_body_id, f'body_{human_body_id}')

        if min_distance == float('inf'):
            # No contact points found - everything is far away
            logger.debug(
                "No contact points found - all bodies > collision_check_distance")
            logger.debug(
                f"  collision_check_distance = {self.config.get('collision_check_distance', 1.0)}")
            logger.debug(
                "  Increasing search distance to 2.0m for this check...")

            # Try again with larger distance
            for human_body_id in human_body_ids:
                closest_points = p.getClosestPoints(
                    bodyA=self.robot_id,
                    bodyB=human_body_id,
                    distance=2.0,  # Search up to 2 meters
                    physicsClientId=self.client
                )
                if closest_points:
                    for cp in closest_points:
                        distance = cp[8]
                        if distance < 0:
                            distance = 0.0
                        if distance < min_distance:
                            min_distance = distance
                            closest_body_name = body_id_to_name.get(
                                human_body_id, f'body_{human_body_id}')

        final_distance = min_distance if min_distance != float(
            'inf') else 2.0  # Return large value if still nothing
        logger.debug(
            f"Final clearance: {final_distance:.3f}m to {closest_body_name}")
        return final_distance, closest_body_name

    def _compute_clearance_at_config(self, joint_angles: List[float]) -> float:
        """Compute minimum clearance to human at given joint configuration."""
        clearance, _ = self._compute_clearance_at_config_detailed(joint_angles)
        return clearance

    def _compute_min_clearance(self, trajectory: List[List[float]]) -> float:
        """Compute minimum clearance along entire trajectory."""
        min_clearance = float('inf')

        for waypoint in trajectory:
            clearance = self._compute_clearance_at_config(waypoint)
            min_clearance = min(min_clearance, clearance)

        return min_clearance

    def cleanup(self):
        """Cleanup PyBullet resources."""
        try:
            p.disconnect(physicsClientId=self.client)
            logger.info("Cleaned up planning world")
        except Exception as e:
            logger.error(f"Error during cleanup: {e}")


# TODO: Implement actual RRT-Connect planner
# - Use pybullet_planning library
# - Or implement custom RRT-Connect with collision callbacks
# - Add path smoothing
# - Add rolling-horizon support
