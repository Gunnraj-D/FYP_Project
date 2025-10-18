"""
Human-Aware Path Planner - FIXED VERSION

Key fixes applied:
1. TCP link found by name (not last joint)
2. IK solutions truncated to planning DOF
3. Collision detection forced before checks
4. GUI mode configurable
5. ZED data staleness validation
6. Improved capsule orientation handling
7. Added self-collision support
"""

from config import TRACKED_HUMAN_JOINTS, HUMAN_MODEL_CONFIG
from pybullet_planning import get_collision_fn, set_client
from pybullet_planning import plan_joint_motion, get_movable_joints
from pybullet_planning import link_pairs_collision, get_joint_positions, set_joint_positions
from hand_detection.zed_joint_receiver import ZEDJointReceiver, SkeletonData
from hand_detection.skeleton_data_provider import SkeletonDataProvider
import logging
import time
import numpy as np
import pybullet as p
from typing import List, Optional, Tuple, Dict, Any
from scipy.spatial.transform import Rotation as R

logger = logging.getLogger(__name__)


class HumanModelAdapter:
    """
    Converts ZED skeleton data to PyBullet collision primitives.
    """

    def __init__(self, pybullet_client: int, config: Dict[str, Any]):
        self.client = pybullet_client
        self.config = config
        self.collision_bodies = {}
        self._body_meta = {}
        self.last_update_time = 0
        self.valid_joints = set()

    def update_from_skeleton(self, skeleton: SkeletonData):
        """Update PyBullet collision bodies from skeleton data."""
        self.valid_joints.clear()
        joint_positions = self._extract_joint_positions(skeleton)

        self._update_head_neck_spheres(joint_positions)
        self._update_torso_capsule(joint_positions)
        self._update_arm_capsules(joint_positions, 'left')
        self._update_arm_capsules(joint_positions, 'right')
        self._update_shoulder_spheres(joint_positions)

        self.last_update_time = time.time()
        logger.debug(
            f"Updated human model with {len(self.valid_joints)} valid joints")

    def _extract_joint_positions(self, skeleton: SkeletonData) -> Dict[str, np.ndarray]:
        """Extract positions of critical joints from skeleton."""
        positions = {}
        found_joints = []
        missing_joints = []

        for _, joint_names in TRACKED_HUMAN_JOINTS.items():
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

    def _update_torso_capsule(self, positions: Dict[str, np.ndarray]):
        """Create/update capsule between CHEST_SPINE and PELVIS."""
        chest_joint = 'SPINE_2'
        if chest_joint in positions and 'PELVIS' in positions:
            self._create_or_update_capsule(
                name="torso",
                pos_a=positions[chest_joint],
                pos_b=positions['PELVIS'],
                radius=self.config['torso_radius']
            )

    def _update_arm_capsules(self, positions: Dict[str, np.ndarray], side: str):
        """Create/update capsules for arm segments."""
        prefix = side.upper()
        shoulder = f'{prefix}_SHOULDER'
        elbow = f'{prefix}_ELBOW'
        wrist = f'{prefix}_WRIST'
        radius = self.config['arm_radius']

        if shoulder in positions and elbow in positions:
            self._create_or_update_capsule(
                name=f"{side}_upper_arm",
                pos_a=positions[shoulder],
                pos_b=positions[elbow],
                radius=radius
            )

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
        existing = self.collision_bodies.get(name)
        if existing is not None:
            meta = self._body_meta.get(existing, {})
            if meta and abs(meta.get('radius', 0.0) - radius) > 1e-6:
                p.removeBody(existing, physicsClientId=self.client)
                del self.collision_bodies[name]
                del self._body_meta[existing]
            else:
                p.resetBasePositionAndOrientation(
                    existing, position.tolist(), [0, 0, 0, 1],
                    physicsClientId=self.client
                )
                self._body_meta[existing]['position'] = position.tolist()
                return

        col_shape = p.createCollisionShape(
            p.GEOM_SPHERE, radius=radius, physicsClientId=self.client)
        body_id = p.createMultiBody(
            baseMass=0, baseCollisionShapeIndex=col_shape,
            basePosition=position.tolist(), physicsClientId=self.client
        )
        self.collision_bodies[name] = body_id
        self._body_meta[body_id] = {
            'type': 'sphere', 'radius': radius, 'position': position.tolist()
        }

    def _create_or_update_capsule(self, name: str, pos_a: np.ndarray,
                                  pos_b: np.ndarray, radius: float):
        """Create or update a capsule collision body - FIXED ORIENTATION."""
        direction = pos_b - pos_a
        length = float(np.linalg.norm(direction))

        if length < 1e-6:
            logger.warning(f"Capsule '{name}' has zero length, skipping")
            return

        center = (pos_a + pos_b) / 2.0
        direction_normalized = direction / length

        # FIXED: Improved orientation computation
        z_axis = np.array([0.0, 0.0, 1.0])
        rotation_axis = np.cross(z_axis, direction_normalized)
        rotation_axis_norm = np.linalg.norm(rotation_axis)

        if rotation_axis_norm < 1e-6:
            if np.dot(z_axis, direction_normalized) > 0:
                orientation = [0, 0, 0, 1]
            else:
                # Anti-parallel: rotate 180° around perpendicular axis
                if abs(direction_normalized[0]) < 0.9:
                    perp_axis = np.array([1.0, 0.0, 0.0])
                else:
                    perp_axis = np.array([0.0, 1.0, 0.0])
                orientation = R.from_rotvec(
                    np.pi * perp_axis).as_quat().tolist()
        else:
            rotation_axis = rotation_axis / rotation_axis_norm
            rotation_angle = float(
                np.arccos(np.clip(np.dot(z_axis, direction_normalized), -1.0, 1.0)))
            orientation = R.from_rotvec(
                rotation_angle * rotation_axis).as_quat().tolist()

        existing = self.collision_bodies.get(name)
        if existing is not None:
            meta = self._body_meta.get(existing, {})
            prev_h = meta.get('height')
            prev_r = meta.get('radius')
            if prev_r is None or prev_h is None or abs(prev_r - radius) > 1e-6 or abs(prev_h - length) > 1e-6:
                p.removeBody(existing, physicsClientId=self.client)
                del self.collision_bodies[name]
                del self._body_meta[existing]
            else:
                p.resetBasePositionAndOrientation(
                    existing, center.tolist(), orientation, physicsClientId=self.client
                )
                self._body_meta[existing]['position'] = center.tolist()
                return

        col_shape = p.createCollisionShape(
            p.GEOM_CAPSULE, radius=radius, height=length, physicsClientId=self.client
        )
        body_id = p.createMultiBody(
            baseMass=0, baseCollisionShapeIndex=col_shape,
            basePosition=center.tolist(), baseOrientation=orientation,
            physicsClientId=self.client
        )
        self.collision_bodies[name] = body_id
        self._body_meta[body_id] = {
            'type': 'capsule', 'radius': radius, 'height': length,
            'position': center.tolist()
        }

    def clear(self):
        """Remove all collision bodies."""
        for body_id in self.collision_bodies.values():
            p.removeBody(body_id, physicsClientId=self.client)
        self.collision_bodies.clear()
        self._body_meta.clear()
        self.valid_joints.clear()

    def get_all_body_ids(self) -> List[int]:
        """Get list of all human collision body IDs."""
        return list(self.collision_bodies.values())


class HumanAwarePathPlanner:
    """Main path planner with human collision avoidance - FIXED VERSION."""

    def __init__(self, urdf_path: str, zed_receiver: ZEDJointReceiver,
                 ik_solver, config: Dict[str, Any]):
        self.zed_receiver = zed_receiver
        self.ik_solver = ik_solver
        self.config = config
        # Unified skeleton provider (supports static or ZED based on config)
        self.skeleton_provider = SkeletonDataProvider(self.zed_receiver)

        # GUI is configurable
        use_gui = config.get('planning_gui', False)
        mode = p.GUI if use_gui else p.DIRECT
        self.client = p.connect(mode)

        if use_gui:
            logger.info("Created PyBullet planning world (GUI mode)")
            p.resetDebugVisualizerCamera(
                cameraDistance=2.0, cameraYaw=45, cameraPitch=-20,
                cameraTargetPosition=[0.5, 0, 0.5],
                physicsClientId=self.client
            )
        else:
            logger.info(
                "Created PyBullet planning world (DIRECT mode - headless)")

        set_client(self.client)

        # Load robot
        self.robot_id = p.loadURDF(
            urdf_path, useFixedBase=True,
            flags=p.URDF_USE_SELF_COLLISION,
            physicsClientId=self.client
        )
        logger.info(f"Loaded robot URDF (ID={self.robot_id})")

        # Get movable joints
        self.num_joints = p.getNumJoints(
            self.robot_id, physicsClientId=self.client)
        self.movable_joints = []
        self.joint_limits = {}

        logger.info("Analyzing robot joints:")
        for j in range(self.num_joints):
            info = p.getJointInfo(
                self.robot_id, j, physicsClientId=self.client)
            joint_name = info[1].decode('utf-8')
            joint_type = info[2]

            if joint_type in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
                self.movable_joints.append(j)
                lower, upper = info[8], info[9]

                if lower > upper or (lower == 0.0 and upper == 0.0):
                    lower, upper = -np.pi, np.pi

                self.joint_limits[j] = (lower, upper)
                logger.info(
                    f"  Joint {j} ({joint_name}): movable, limits [{lower:.2f}, {upper:.2f}]")
            else:
                logger.info(f"  Joint {j} ({joint_name}): fixed (skipped)")

        # Select planning joints (first 7 movable = arm joints)
        n_plan = min(self.config.get('planning_dof', 7),
                     len(self.movable_joints))
        self.planning_joints = self.movable_joints[:n_plan]
        logger.info(
            f"Planning with {len(self.planning_joints)} joints: {self.planning_joints}")

        self.tcp_link_idx = None
        for i in range(self.num_joints):
            link_info = p.getJointInfo(
                self.robot_id, i, physicsClientId=self.client)
            link_name = link_info[12].decode('utf-8')  # Child link name
            if link_name == 'tcp':
                self.tcp_link_idx = i
                logger.info(f"Found TCP link at joint index {i}")
                break

        if self.tcp_link_idx is None:
            logger.warning(
                "Could not find 'tcp' link, using last movable joint as TCP")
            self.tcp_link_idx = self.planning_joints[-1]

        # Initialize human model
        self.human_model = HumanModelAdapter(self.client, HUMAN_MODEL_CONFIG)
        self.person_detected = False
        self.last_skeleton_data = None

        logger.info("Human-aware path planner initialized successfully")

    def plan_trajectory(self, start_joints: List[float], goal_pose: List[float],
                        use_pre_approach: bool = True) -> Tuple[Optional[List[List[float]]], Dict[str, Any]]:
        """Plan collision-free trajectory to goal."""
        start_time = time.time()
        logger.info(f"Planning trajectory to pose {goal_pose[:3]}")

        # Update human model
        self._update_human_model()

        # Sample goal configurations
        goal_joints = self._sample_goal_configurations(goal_pose)
        if not goal_joints:
            return None, {
                'success': False,
                'reason': 'No valid IK solutions',
                'planning_time': time.time() - start_time
            }

        # Plan with RRT-Connect
        trajectory = self._rrt_connect_plan(start_joints, goal_joints[0])

        if trajectory is None:
            return None, {
                'success': False,
                'reason': 'Planning failed',
                'person_detected': self.person_detected,
                'planning_time': time.time() - start_time
            }

        min_clearance = self._compute_min_clearance(trajectory)

        metadata = {
            'success': True,
            'planning_time': time.time() - start_time,
            'waypoint_count': len(trajectory),
            'min_clearance': min_clearance,
            'person_detected': self.person_detected,
            'planner_used': 'rrt_connect'
        }

        return trajectory, metadata

    def get_clearance_to_human(self, joint_angles: List[float]) -> float:
        """Get minimum clearance to human at given configuration."""
        self._update_human_model()
        return self._compute_clearance_at_config(joint_angles)

    def human_moved_significantly(self, threshold: float = 0.10) -> bool:
        """Check if human moved more than threshold."""
        if not self.person_detected or self.last_skeleton_data is None:
            return False

        current_frame = self.zed_receiver.get_latest_frame()
        if current_frame is None or not current_frame.skeletons:
            return False

        current_skeleton = current_frame.skeletons[0]
        key_joints = ['RIGHT_WRIST', 'LEFT_WRIST', 'RIGHT_ELBOW', 'LEFT_ELBOW']

        for joint_name in key_joints:
            old_pos = self.last_skeleton_data.get_joint_position(joint_name)
            new_pos = current_skeleton.get_joint_position(joint_name)

            if old_pos is not None and new_pos is not None:
                delta = np.linalg.norm(np.array(new_pos) - np.array(old_pos))
                if delta > threshold:
                    logger.info(
                        f"Human moved: {joint_name} delta={delta:.3f}m")
                    return True

        return False

    def _update_human_model(self):
        """Update human collision model with staleness check (supports static skeleton)."""
        # Prefer unified provider which respects PATH_PLANNING_CONFIG (static/live)
        frame_data = self.skeleton_provider.get_latest_frame(
        ) if self.skeleton_provider else None

        if frame_data is None or not frame_data.skeletons:
            if self.person_detected:
                logger.info("Person no longer detected")
                self.human_model.clear()
            self.person_detected = False
            self.last_skeleton_data = None
            return

        # FIXED: Check data freshness (skip for static provider which always returns fresh timestamp)
        from config import PATH_PLANNING_CONFIG
        use_static = PATH_PLANNING_CONFIG.get(
            'use_static_skeleton_data', False)
        current_time = time.time()
        frame_age = current_time - \
            getattr(frame_data, 'timestamp', current_time)

        if not use_static and frame_age > 0.5:  # 500ms threshold for live data
            logger.warning(f"Stale skeleton data! Frame age: {frame_age:.3f}s")
            self.human_model.clear()
            self.person_detected = False
            return

        skeleton = frame_data.skeletons[0]
        self.human_model.update_from_skeleton(skeleton)
        self.person_detected = True
        self.last_skeleton_data = skeleton

        num_bodies = len(self.human_model.get_all_body_ids())
        logger.info(f"Human model updated: {num_bodies} collision bodies")

    def _sample_goal_configurations(self, goal_pose: List[float],
                                    max_configs: int = 30) -> List[List[float]]:
        """
        Sample goal configurations using multi-yaw and position search.
        IMPROVED: Searches yaw redundancy + position perturbations for robust IK.

        Args:
            goal_pose: Target pose [x,y,z] or [x,y,z,rx,ry,rz]
            max_configs: Maximum number of configurations to return

        Returns:
            List of valid joint configurations, sorted by quality (best first)
        """
        goal_configs = []
        n_plan_joints = len(self.planning_joints)

        # Get configuration from path planning config
        n_yaw_samples = self.config.get('ik_yaw_samples', 12)
        n_position_samples = self.config.get('ik_position_samples', 5)
        xy_perturb = self.config.get('ik_xy_perturbation', 0.02)
        z_perturb = self.config.get('ik_z_perturbation', 0.03)
        max_iter = self.config.get('ik_max_iterations', 200)

        # Get current joint state as seed (use zeros if not available)
        try:
            current_joints = [p.getJointState(self.robot_id, j, physicsClientId=self.client)[0]
                              for j in self.planning_joints]
        except:
            current_joints = [0.0] * 7

        logger.info(
            f"Sampling IK solutions: {n_position_samples} positions × {n_yaw_samples} yaws = {n_position_samples * n_yaw_samples} trials")

        # Handle position-only vs full pose
        if len(goal_pose) == 3:
            # Position only - use comprehensive search with yaw freedom
            target_pos = goal_pose

            solutions = self.ik_solver.solve_with_position_and_yaw_search(
                target_pos=target_pos,
                current_q=current_joints,
                position_samples=n_position_samples,
                yaw_samples=n_yaw_samples,
                xy_perturbation=xy_perturb,
                z_perturbation=z_perturb,
                max_iter=max_iter,
                tol=1e-3
            )

            # Extract joint configurations (truncate to planning DOF)
            for solution, quality, perturbed_pos, yaw in solutions[:max_configs]:
                config = solution[:n_plan_joints].tolist()
                goal_configs.append(config)

                # Log best few solutions
                if len(goal_configs) <= 3:
                    logger.info(
                        f"  Solution {len(goal_configs)}: quality={quality:.3f}, "
                        f"yaw={np.rad2deg(yaw):.1f}°, "
                        f"pos_offset=[{perturbed_pos[0]-target_pos[0]:.3f}, "
                        f"{perturbed_pos[1]-target_pos[1]:.3f}, "
                        f"{perturbed_pos[2]-target_pos[2]:.3f}]m"
                    )

        else:
            # Full pose with orientation constraint - less freedom, but still try perturbations
            logger.warning(
                "Full 6-DOF pose provided - limited yaw freedom. Consider using position-only targets for better IK success.")

            # Try original pose
            try:
                base_solution = self.ik_solver.solve_pose(
                    target_pose=goal_pose,
                    current_q=current_joints,
                    max_iter=max_iter
                )
                if base_solution is not None:
                    quality = self.ik_solver.compute_configuration_quality(
                        base_solution)
                    goal_configs.append(
                        (base_solution[:n_plan_joints].tolist(), quality))
            except Exception as e:
                logger.debug(f"Base pose IK failed: {e}")

            # Try position perturbations with same orientation
            for i in range(n_position_samples - 1):
                perturbed_pose = goal_pose.copy()
                perturbed_pose[0] += np.random.uniform(-xy_perturb, xy_perturb)
                perturbed_pose[1] += np.random.uniform(-xy_perturb, xy_perturb)
                perturbed_pose[2] += np.random.uniform(-z_perturb, z_perturb)

                try:
                    solution = self.ik_solver.solve_pose(
                        target_pose=perturbed_pose,
                        current_q=current_joints,
                        max_iter=max_iter
                    )
                    if solution is not None:
                        quality = self.ik_solver.compute_configuration_quality(
                            solution)
                        goal_configs.append(
                            (solution[:n_plan_joints].tolist(), quality))
                except Exception as e:
                    logger.debug(f"Perturbed pose IK failed: {e}")

            # Sort by quality and extract configs
            goal_configs.sort(key=lambda x: x[1], reverse=True)
            goal_configs = [cfg for cfg, _ in goal_configs[:max_configs]]

        logger.info(
            f"✓ Generated {len(goal_configs)} valid goal configurations")

        if len(goal_configs) == 0:
            logger.error(
                "❌ NO VALID IK SOLUTIONS FOUND! Target may be unreachable.")

        return goal_configs

    def _rrt_connect_plan(self, start: List[float],
                          goal: List[float]) -> Optional[List[List[float]]]:
        """
        Plan using RRT-Connect with inflated human model for safety.

        The human model bodies are pre-inflated with safety margins, so
        the planner naturally maintains safe distances by avoiding collisions
        with the inflated obstacles.
        """
        logger.info(
            "Running RRT-Connect planner with safety-aware collision checking")

        try:
            # Set start configuration
            for i, joint_idx in enumerate(self.planning_joints):
                p.resetJointState(
                    self.robot_id, joint_idx, start[i],
                    physicsClientId=self.client
                )

            p.performCollisionDetection(physicsClientId=self.client)

            # Get TCP position for debugging
            link_state = p.getLinkState(
                self.robot_id, self.tcp_link_idx,
                computeForwardKinematics=True,
                physicsClientId=self.client
            )
            logger.info(f"Start TCP position: {link_state[4]}")

            # Check start clearance
            start_clearance, closest = self._compute_clearance_at_config_detailed(
                start)
            logger.info(
                f"Start clearance: {start_clearance:.3f}m (closest: {closest})")

            if start_clearance < 0.02:
                logger.error("Start configuration in collision!")
                return None

            # Get human obstacles (already inflated with safety margins)
            human_bodies = self.human_model.get_all_body_ids()
            logger.info(f"Planning with {len(human_bodies)} human obstacles")
            from config import HUMAN_MODEL_CONFIG
            arm_r = int(HUMAN_MODEL_CONFIG.get('arm_radius', 0.12) * 1000)
            head_r = int(HUMAN_MODEL_CONFIG.get('head_radius', 0.20) * 1000)
            logger.info(
                f"Human model inflated with safety margins: arms={arm_r}mm, head={head_r}mm")

            # Randomize RRT seed
            np.random.seed(int(time.time() * 1000) % (2**32))

            # Plan - the inflated obstacles enforce safety distances automatically
            path = plan_joint_motion(
                self.robot_id,
                self.planning_joints,
                goal,
                obstacles=human_bodies,
                self_collisions=self.config.get('check_self_collision', False),
                disabled_collisions=set(),
                custom_limits={},
                max_distance=self.config.get('step_size', 0.15),
                restarts=5,
                iterations=self.config.get('max_iterations', 2000),
                smooth=self.config.get('smoothing_iterations', 20),
                diagnosis=False,
                physicsClientId=self.client
            )

            if path is None:
                logger.warning("RRT-Connect could not find path")
                return None

            logger.info(f"Path found with {len(path)} waypoints")
            trajectory = [list(waypoint) for waypoint in path]

            # SAFETY CHECK: Verify smoothed path maintains minimum clearances
            # Smoothing can bring waypoints closer to obstacles, so we validate
            if self.person_detected:
                min_clearance = self._compute_min_clearance(trajectory)
                safety_threshold = 0.01  # 10mm - account for numerical precision

                if min_clearance < safety_threshold:
                    logger.error(
                        f"Smoothed path violates safety! Clearance {min_clearance*1000:.1f}mm "
                        f"< {safety_threshold*1000:.1f}mm threshold. Rejecting path."
                    )
                    return None
                else:
                    logger.info(
                        f"Path validated: minimum clearance {min_clearance*1000:.0f}mm")

            return trajectory

        except Exception as e:
            logger.error(f"RRT-Connect planning failed: {e}")
            import traceback
            traceback.print_exc()
            return None

    def _compute_clearance_at_config_detailed(self, joint_angles: List[float]) -> Tuple[float, str]:
        """Compute clearance with contact detection - FIXED."""
        if not self.person_detected:
            return float('inf'), 'none'

        if len(joint_angles) != len(self.planning_joints):
            raise ValueError(
                f"joint_angles length ({len(joint_angles)}) != planning_joints ({len(self.planning_joints)})"
            )

        # Set configuration
        for i, angle in enumerate(joint_angles):
            joint_idx = self.planning_joints[i]
            p.resetJointState(self.robot_id, joint_idx, angle,
                              physicsClientId=self.client)

        # FIXED: Force collision detection
        p.performCollisionDetection(physicsClientId=self.client)

        human_body_ids = self.human_model.get_all_body_ids()
        body_id_to_name = {v: k for k,
                           v in self.human_model.collision_bodies.items()}

        # Check for actual collisions (penetration)
        in_collision = False
        collision_depth = 0.0
        collision_body_name = 'none'

        for human_body_id in human_body_ids:
            contacts = p.getContactPoints(
                bodyA=self.robot_id,
                bodyB=human_body_id,
                physicsClientId=self.client
            )

            if contacts:
                for contact in contacts:
                    contact_dist = contact[8]
                    if contact_dist < 0:  # Penetration
                        in_collision = True
                        penetration = abs(contact_dist)
                        if penetration > collision_depth:
                            collision_depth = penetration
                            collision_body_name = body_id_to_name.get(
                                human_body_id, f'body_{human_body_id}'
                            )

        if in_collision:
            logger.error(
                f"Collision! Penetration: {collision_depth*1000:.1f}mm with {collision_body_name}")
            return 0.0, collision_body_name

        # Find minimum separation distance
        min_distance = float('inf')
        closest_body_name = 'unknown'

        for human_body_id in human_body_ids:
            body_name = body_id_to_name.get(
                human_body_id, f'body_{human_body_id}')

            closest_points = p.getClosestPoints(
                bodyA=self.robot_id,
                bodyB=human_body_id,
                distance=1.0,
                physicsClientId=self.client
            )

            if closest_points:
                for cp in closest_points:
                    distance = cp[8]
                    if distance > 0 and distance < min_distance:
                        min_distance = distance
                        closest_body_name = body_name

        final_distance = min_distance if min_distance != float('inf') else 1.0
        return final_distance, closest_body_name

    def _compute_clearance_at_config(self, joint_angles: List[float]) -> float:
        """Compute minimum clearance to human."""
        clearance, _ = self._compute_clearance_at_config_detailed(joint_angles)
        return clearance

    def _compute_min_clearance(self, trajectory: List[List[float]]) -> float:
        """Compute minimum clearance along trajectory."""
        min_clearance = float('inf')
        for waypoint in trajectory:
            clearance = self._compute_clearance_at_config(waypoint)
            min_clearance = min(min_clearance, clearance)
        return min_clearance

    def check_trajectory_safe(self, trajectory: List[List[float]]) -> Tuple[bool, float]:
        """Check if trajectory is safe - UPDATED WITH FRESHNESS CHECK."""
        if not trajectory or len(trajectory) == 0:
            return True, float('inf')

        # Validate waypoint lengths
        for idx, wp in enumerate(trajectory):
            if len(wp) != len(self.planning_joints):
                raise ValueError(
                    f"Waypoint {idx} length {len(wp)} != planning DOF {len(self.planning_joints)}"
                )

        # CRITICAL: Update human model from latest ZED data
        self._update_human_model()

        if not self.person_detected:
            return True, float('inf')

        min_clearance = self._compute_min_clearance(trajectory)
        emergency_stop_dist = self.config.get('emergency_stop_distance', 0.15)
        is_safe = min_clearance >= emergency_stop_dist

        if not is_safe:
            logger.warning(
                f"Trajectory UNSAFE! Clearance {min_clearance:.3f}m < {emergency_stop_dist:.3f}m"
            )
        else:
            logger.debug(f"Trajectory safe: clearance {min_clearance:.3f}m")

        return is_safe, min_clearance

    def cleanup(self):
        """Cleanup PyBullet resources."""
        try:
            p.disconnect(physicsClientId=self.client)
            logger.info("Cleaned up planning world")
        except Exception as e:
            logger.error(f"Error during cleanup: {e}")
