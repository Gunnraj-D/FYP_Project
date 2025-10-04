"""
Integrated Hand-Eye Calibration System for KUKA iiwa14 with RealSense Camera.

This module provides a complete hand-eye calibration solution that integrates with
the existing robot control and camera management systems.
"""
from .calibration_validator import CalibrationValidator
from .pose_generator import PoseGenerator
from .calibration_config import CalibrationConfig
from IO_handling.opc_client_factory import OPCClientFactory
from IO_handling.opc_client import OPCClient, OPCConfig
from control.telemetry_store import Telemetry
from control.command_bus import CommandBus, SetJoints
from kinematics.kinematics_solver import InverseKinematicsSolver
from camera_management.camera_manager import CameraManager, CameraConfig
import cv2
import numpy as np
import pickle
import time
import logging
from typing import List, Tuple, Optional, Dict, Any
import json
from pathlib import Path

# Import system components
import sys
sys.path.append(str(Path(__file__).parent.parent))


logger = logging.getLogger(__name__)


class HandEyeCalibrator:
    """Integrated hand-eye calibration system."""

    def __init__(self, config: CalibrationConfig = None):
        """
        Initialize hand-eye calibrator.

        Args:
            config: Calibration configuration (uses default if None)
        """
        self.config = config or CalibrationConfig()

        # Initialize components
        self.camera_manager = None
        self.kinematics_solver = None
        self.robot_client = None
        self.pose_generator = None
        self.validator = CalibrationValidator(self.config)

        # Calibration data
        self.calibration_data = {
            'R_gripper2base': [],
            't_gripper2base': [],
            'R_target2cam': [],
            't_target2cam': [],
            'reprojection_errors': [],
            'poses_used': []
        }

        # Hand-eye transformation matrix
        self.H_cam2tcp = None

        # State tracking
        self.is_initialized = False
        self.calibration_complete = False

    def initialize(self, opc_mode: str = "mock") -> bool:
        """
        Initialize all system components.

        Args:
            opc_mode: "real" or "mock" for robot communication

        Returns:
            True if initialization successful, False otherwise
        """
        try:
            logger.info("Initializing hand-eye calibration system...")

            # Initialize camera
            camera_config = CameraConfig(
                color_width=self.config.camera_resolution[0],
                color_height=self.config.camera_resolution[1],
                color_fps=self.config.camera_fps,
                depth_width=self.config.camera_resolution[0],
                depth_height=self.config.camera_resolution[1],
                depth_fps=self.config.camera_fps
            )
            self.camera_manager = CameraManager(camera_config)

            if not self.camera_manager.initialize():
                logger.error("Failed to initialize camera")
                return False

            # Initialize kinematics solver
            self.kinematics_solver = InverseKinematicsSolver(
                urdf_filepath=self.config.urdf_filepath,
                base_elements=self.config.base_elements,
                active_links_mask=self.config.active_links,
                use_gui=False
            )

            # Initialize robot communication
            command_bus = CommandBus()
            telemetry = Telemetry()

            opc_config = OPCConfig()
            opc_config.robot_id = self.config.robot_id

            if opc_mode == "mock":
                from IO_handling.mock_opc_client import MockOPCClient
                self.robot_client = MockOPCClient(
                    command_bus, telemetry, opc_config)
            else:
                self.robot_client = OPCClient(
                    command_bus, telemetry, opc_config)

            self.robot_client.start()

            # Wait for robot connection
            max_wait_time = 10.0
            start_time = time.time()
            while not self.robot_client.is_connected() and (time.time() - start_time) < max_wait_time:
                time.sleep(0.1)

            if not self.robot_client.is_connected():
                logger.error("Failed to connect to robot")
                return False

            # Initialize pose generator
            self.pose_generator = PoseGenerator(
                self.config, self.kinematics_solver)

            self.is_initialized = True
            logger.info("Hand-eye calibration system initialized successfully")
            return True

        except Exception as e:
            logger.error(f"Failed to initialize calibration system: {e}")
            return False

    def run_calibration(self, num_poses: int = None, manual: bool = False) -> bool:
        """
        Run the complete hand-eye calibration process.

        Args:
            num_poses: Number of poses to collect (uses config default if None)

        Returns:
            True if calibration successful, False otherwise
        """
        if not self.is_initialized:
            logger.error("System not initialized. Call initialize() first.")
            return False

        try:
            num_poses = num_poses or self.config.num_poses
            logger.info(
                f"Starting hand-eye calibration with {num_poses} poses...")

            if manual:
                # Manual capture loop with live preview
                if not self._collect_calibration_data_manual(num_poses):
                    logger.error(
                        "Failed to collect calibration data (manual mode)")
                    return False
            else:
                # Auto-generated poses
                poses = self.pose_generator.generate_calibration_poses(
                    num_poses)
                if len(poses) < self.config.min_poses_required:
                    logger.error(
                        f"Insufficient valid poses generated: {len(poses)} < {self.config.min_poses_required}")
                    return False

                # Collect calibration data
                if not self._collect_calibration_data(poses):
                    logger.error("Failed to collect calibration data")
                    return False

            # Perform hand-eye calibration
            if not self._perform_hand_eye_calibration():
                logger.error("Hand-eye calibration failed")
                return False

            # Validate calibration
            validation_results = self._validate_calibration()

            # Generate report
            self._generate_calibration_report(validation_results)

            self.calibration_complete = True
            logger.info("Hand-eye calibration completed successfully")
            return True

        except Exception as e:
            logger.error(f"Calibration failed: {e}")
            return False

    def _collect_calibration_data(self, poses: List[np.ndarray]) -> bool:
        """Collect calibration data by moving robot through poses."""
        logger.info(f"Collecting calibration data for {len(poses)} poses...")

        collected = 0
        total_poses = len(poses)

        for i, pose in enumerate(poses):
            logger.info(f"Pose {i+1}/{total_poses}: Collecting data...")

            try:
                # Move robot to pose
                if not self._move_robot_to_pose(pose):
                    logger.warning(
                        f"Failed to move to pose {i+1}, skipping...")
                    continue

                # Wait for robot to settle
                time.sleep(self.config.settling_time)

                # Attempt to detect checkerboard
                success, data = self._capture_pose_data(pose, i)
                if success:
                    # Store calibration data
                    self.calibration_data['R_gripper2base'].append(
                        data['R_gripper2base'])
                    self.calibration_data['t_gripper2base'].append(
                        data['t_gripper2base'])
                    self.calibration_data['R_target2cam'].append(
                        data['R_target2cam'])
                    self.calibration_data['t_target2cam'].append(
                        data['t_target2cam'])
                    self.calibration_data['reprojection_errors'].append(
                        data['reprojection_error'])
                    self.calibration_data['poses_used'].append(pose)

                    collected += 1
                    logger.info(
                        f"Pose {i+1} collected successfully (total: {collected})")
                else:
                    logger.warning(f"Failed to capture data for pose {i+1}")

            except Exception as e:
                logger.error(f"Error collecting pose {i+1}: {e}")
                continue

        logger.info(f"Collected {collected}/{total_poses} poses")
        return collected >= self.config.min_poses_required

    def _collect_calibration_data_manual(self, target_poses: int) -> bool:
        """Manual capture: show live feed, press SPACE to capture a sample.

        User can move the robot manually. On SPACE, we detect the checkerboard,
        solve PnP, read current joints, compute TCP, and append calibration data.
        Press 'q' to finish early.
        """
        logger.info("Manual capture mode: Press SPACE to capture, 'q' to quit")

        # Prepare session directory to save captures and metadata
        base_dir = Path("calibration_captures")
        session_dir = base_dir / time.strftime("%Y%m%d_%H%M%S")
        try:
            session_dir.mkdir(parents=True, exist_ok=True)
            logger.info(f"Saving captures to: {session_dir}")
        except Exception as e:
            logger.error(
                f"Failed to create capture directory {session_dir}: {e}")
            # Continue without saving if directory creation fails
            session_dir = None

        collected = 0
        first_tcp: Optional[np.ndarray] = None

        while collected < target_poses:
            try:
                color_frame, depth_frame = self.camera_manager.get_frames()
                if color_frame is None:
                    cv2.waitKey(1)
                    continue

                # Try detect corners for overlay
                ret, corners = self._detect_checkerboard_corners(color_frame)
                display = color_frame.copy()
                if ret and corners is not None:
                    cv2.drawChessboardCorners(
                        display, self.config.chessboard_size, corners, ret)

                # HUD text
                msg1 = f"Manual capture: {collected}/{target_poses} | SPACE=capture, q=quit"
                cv2.putText(display, msg1, (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                if not ret:
                    cv2.putText(display, "Checkerboard NOT detected", (10, 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                else:
                    cv2.putText(display, "Checkerboard detected", (10, 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                cv2.imshow("Calibration - Manual Capture", display)
                key = cv2.waitKey(1) & 0xFF

                if key == ord('q'):
                    break

                if key == 32:  # SPACE
                    if not ret or corners is None:
                        logger.warning(
                            "Capture skipped: checkerboard not detected")
                        continue

                    # Ensure intrinsics
                    if not self.camera_manager.intrinsics:
                        logger.error("Camera intrinsics not available")
                        continue

                    # Solve PnP
                    success_pnp, rvec, tvec = cv2.solvePnP(
                        self._get_object_points(),
                        corners,
                        self._get_camera_matrix(),
                        self._get_distortion_coeffs()
                    )
                    if not success_pnp:
                        logger.warning(
                            "PnP solve failed; try a different angle")
                        continue

                    R_target2cam, _ = cv2.Rodrigues(rvec)

                    # Read current joints and compute TCP
                    current_joints = self.robot_client.telemetry.get_current_joints()
                    if current_joints is None or len(current_joints) == 0:
                        logger.error(
                            "Failed to read current joints; skipping capture")
                        continue
                    tcp_matrix, _ = self.kinematics_solver.tcp_from_joints(
                        current_joints)

                    # Build relative gripper->base motion versus first capture
                    if first_tcp is None:
                        first_tcp = tcp_matrix.copy()
                        R_gripper2base = np.eye(3)
                        t_gripper2base = np.zeros(3)
                    else:
                        rel_transform = np.linalg.inv(first_tcp) @ tcp_matrix
                        R_gripper2base = rel_transform[:3, :3]
                        t_gripper2base = rel_transform[:3, 3]

                    reprojection_error = self._compute_reprojection_error(
                        corners, rvec, tvec, self._get_camera_matrix(), self._get_distortion_coeffs()
                    )

                    # Enforce reprojection error threshold
                    if reprojection_error > self.config.max_reprojection_error:
                        logger.warning(
                            f"Capture rejected: reprojection error {reprojection_error:.3f}px > threshold {self.config.max_reprojection_error:.3f}px")
                        overlay = color_frame.copy()
                        cv2.putText(overlay, f"Rejected: reproj {reprojection_error:.2f}px > {self.config.max_reprojection_error:.2f}px",
                                    (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                        cv2.imshow("Calibration - Manual Capture", overlay)
                        cv2.waitKey(600)
                        continue

                    # Append accepted sample
                    self.calibration_data['R_gripper2base'].append(
                        R_gripper2base)
                    self.calibration_data['t_gripper2base'].append(
                        t_gripper2base)
                    self.calibration_data['R_target2cam'].append(R_target2cam)
                    self.calibration_data['t_target2cam'].append(
                        tvec.flatten())
                    self.calibration_data['reprojection_errors'].append(
                        reprojection_error)
                    self.calibration_data['poses_used'].append(tcp_matrix)

                    # Save capture artifacts (image and metadata) to session folder
                    try:
                        if session_dir is not None:
                            sample_idx = collected + 1
                            prefix = f"sample_{sample_idx:02d}"
                            # Save color image
                            cv2.imwrite(
                                str(session_dir / f"{prefix}_color.png"), color_frame)
                            # Save numpy data
                            np.save(session_dir /
                                    f"{prefix}_corners.npy", corners)
                            np.save(session_dir / f"{prefix}_rvec.npy", rvec)
                            np.save(session_dir / f"{prefix}_tvec.npy", tvec)
                            np.save(session_dir /
                                    f"{prefix}_tcp.npy", tcp_matrix)
                            np.save(
                                session_dir / f"{prefix}_R_gripper2base.npy", R_gripper2base)
                            np.save(
                                session_dir / f"{prefix}_t_gripper2base.npy", t_gripper2base)
                            # Save metadata JSON
                            meta = {
                                "timestamp": time.time(),
                                "reprojection_error_px": float(reprojection_error),
                                "chessboard_size": list(self.config.chessboard_size),
                                "square_size_m": float(self.config.square_size),
                                "camera_resolution": list(self.config.camera_resolution),
                            }
                            with open(session_dir / f"{prefix}_meta.json", "w", encoding="utf-8") as f:
                                json.dump(meta, f, indent=2)
                    except Exception as e:
                        logger.warning(
                            f"Failed to save capture artifacts: {e}")

                    collected += 1
                    logger.info(
                        f"Captured sample {collected}/{target_poses} | reproj={reprojection_error:.3f}px (accepted)")

            except KeyboardInterrupt:
                break
            except Exception as e:
                logger.error(f"Manual capture error: {e}")
                time.sleep(0.2)
                continue

        cv2.destroyWindow("Calibration - Manual Capture")

        if collected < self.config.min_poses_required:
            logger.error(
                f"Only collected {collected} samples (< {self.config.min_poses_required})")
            return False
        return True

    def _move_robot_to_pose(self, pose: np.ndarray) -> bool:
        """Move robot to specified pose."""
        try:
            # Convert pose to joint angles
            position = pose[:3, 3]
            orientation = pose[:3, :3]

            # Get current joint positions
            current_joints = self.robot_client.telemetry.get_current_joints()
            if current_joints is None or len(current_joints) == 0:
                current_joints = [0.0] * 7

            # Solve inverse kinematics
            target_joints = self.kinematics_solver.solve_XYZ(
                position, current_joints, target_orientation=orientation
            )

            # Send joint command
            command = SetJoints(target_joints.tolist())
            self.robot_client.command_bus.send_command(command)

            # Wait for movement to complete
            time.sleep(1.0)

            return True

        except Exception as e:
            logger.error(f"Failed to move robot to pose: {e}")
            return False

    def _capture_pose_data(self, pose: np.ndarray, pose_index: int) -> Tuple[bool, Optional[Dict]]:
        """Capture calibration data for a single pose."""
        for attempt in range(self.config.max_detection_attempts):
            try:
                # Get camera frames
                color_frame, depth_frame = self.camera_manager.get_frames()
                if color_frame is None:
                    logger.warning(
                        f"Failed to get camera frame (attempt {attempt+1})")
                    time.sleep(0.5)
                    continue

                # Detect checkerboard corners
                success, corners = self._detect_checkerboard_corners(
                    color_frame)
                if not success:
                    logger.warning(
                        f"Checkerboard not detected (attempt {attempt+1})")
                    time.sleep(0.5)
                    continue

                # Get camera intrinsics
                if not self.camera_manager.intrinsics:
                    logger.error("Camera intrinsics not available")
                    return False, None

                # Compute target pose in camera frame
                success_pnp, rvec, tvec = cv2.solvePnP(
                    self._get_object_points(),
                    corners,
                    self._get_camera_matrix(),
                    self._get_distortion_coeffs()
                )

                if not success_pnp:
                    logger.warning(f"PnP solve failed (attempt {attempt+1})")
                    time.sleep(0.5)
                    continue

                # Convert to rotation matrix
                R_target2cam, _ = cv2.Rodrigues(rvec)

                # Get current robot pose
                current_joints = self.robot_client.telemetry.get_current_joints()
                if current_joints is None or len(current_joints) == 0:
                    logger.error("Failed to get current joint positions")
                    return False, None

                # Compute current TCP pose
                tcp_matrix, _ = self.kinematics_solver.tcp_from_joints(
                    current_joints)

                # For hand-eye calibration, we need relative poses
                if pose_index == 0:
                    # First pose is reference (identity)
                    R_gripper2base = np.eye(3)
                    t_gripper2base = np.zeros(3)
                else:
                    # Relative transformation from first pose
                    first_pose = self.calibration_data['poses_used'][0]
                    rel_transform = np.linalg.inv(first_pose) @ tcp_matrix
                    R_gripper2base = rel_transform[:3, :3]
                    t_gripper2base = rel_transform[:3, 3]

                # Compute reprojection error
                reprojection_error = self._compute_reprojection_error(
                    corners, rvec, tvec, self._get_camera_matrix(), self._get_distortion_coeffs()
                )

                data = {
                    'R_gripper2base': R_gripper2base,
                    't_gripper2base': t_gripper2base,
                    'R_target2cam': R_target2cam,
                    't_target2cam': tvec.flatten(),
                    'reprojection_error': reprojection_error
                }

                return True, data

            except Exception as e:
                logger.error(
                    f"Error capturing pose data (attempt {attempt+1}): {e}")
                time.sleep(0.5)
                continue

        return False, None

    def _detect_checkerboard_corners(self, image: np.ndarray) -> Tuple[bool, Optional[np.ndarray]]:
        """Detect checkerboard corners in image."""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(
            gray,
            self.config.chessboard_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        if ret:
            # Refine corner positions
            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), self.config.corner_criteria
            )

        return ret, corners

    def _get_object_points(self) -> np.ndarray:
        """Get 3D object points for checkerboard."""
        objp = np.zeros(
            (self.config.chessboard_size[0] * self.config.chessboard_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.config.chessboard_size[0],
                               0:self.config.chessboard_size[1]].T.reshape(-1, 2)
        objp *= self.config.square_size
        return objp

    def _get_camera_matrix(self) -> np.ndarray:
        """Get camera intrinsic matrix."""
        intrinsics = self.camera_manager.intrinsics
        return np.array([
            [intrinsics.fx, 0, intrinsics.ppx],
            [0, intrinsics.fy, intrinsics.ppy],
            [0, 0, 1]
        ])

    def _get_distortion_coeffs(self) -> np.ndarray:
        """Get camera distortion coefficients."""
        return np.array(self.camera_manager.intrinsics.coeffs)

    def _compute_reprojection_error(self, corners: np.ndarray, rvec: np.ndarray, tvec: np.ndarray,
                                    camera_matrix: np.ndarray, dist_coeffs: np.ndarray) -> float:
        """Compute reprojection error for pose."""
        obj_points = self._get_object_points()
        img_points_proj, _ = cv2.projectPoints(
            obj_points, rvec, tvec, camera_matrix, dist_coeffs)
        error = cv2.norm(corners, img_points_proj,
                         cv2.NORM_L2) / len(img_points_proj)
        return error

    def _perform_hand_eye_calibration(self) -> bool:
        """Perform hand-eye calibration using relative motion pairs (A2/B4)."""
        try:
            logger.info(
                "Performing hand-eye calibration (relative motions A2/B4)...")

            poses_used = self.calibration_data['poses_used']
            R_target2cam = self.calibration_data['R_target2cam']
            t_target2cam = self.calibration_data['t_target2cam']

            if len(poses_used) < self.config.min_poses_required:
                logger.error(
                    f"Insufficient data for calibration: {len(poses_used)} poses")
                return False

            # Build relative motions between consecutive samples
            def to_h(R: np.ndarray, t: np.ndarray) -> np.ndarray:
                H = np.eye(4)
                H[:3, :3] = R
                H[:3, 3] = t.reshape(3)
                return H

            Tg = [np.array(P) for P in poses_used]  # base->tcp per sample
            T_t2c = [to_h(np.array(R), np.array(t))
                     for R, t in zip(R_target2cam, t_target2cam)]  # target->cam
            T_c2t = [np.linalg.inv(T) for T in T_t2c]  # cam->target

            # A2: inv(Tg_{i+1}) @ Tg_i
            A_list = [np.linalg.inv(Tg[i+1]) @ Tg[i]
                      for i in range(len(Tg) - 1)]
            # B4: inv(T_c2t_{i+1}) @ T_c2t_i
            B_list = [np.linalg.inv(T_c2t[i+1]) @ T_c2t[i]
                      for i in range(len(T_c2t) - 1)]

            def split_RT(T_list):
                R_list = [T[:3, :3] for T in T_list]
                t_list = [T[:3, 3] for T in T_list]
                return R_list, t_list

            Rg, tg = split_RT(A_list)
            Rt, tt = split_RT(B_list)

            # Perform hand-eye calibration
            R_cam2tcp, t_cam2tcp = cv2.calibrateHandEye(
                Rg, tg, Rt, tt, method=self.config.calibration_method
            )

            # Create hand-eye transformation matrix
            self.H_cam2tcp = np.eye(4)
            self.H_cam2tcp[:3, :3] = R_cam2tcp
            self.H_cam2tcp[:3, 3] = t_cam2tcp.flatten()

            logger.info("Hand-eye calibration completed successfully (A2/B4)")
            logger.info(f"Hand-eye transformation matrix:\n{self.H_cam2tcp}")

            return True

        except Exception as e:
            logger.error(f"Hand-eye calibration failed (A2/B4): {e}")
            return False

    def _validate_calibration(self) -> Dict[str, Any]:
        """Validate calibration quality."""
        logger.info("Validating calibration quality...")

        validation_results = self.validator.validate_calibration_quality(
            self.calibration_data['R_gripper2base'],
            self.calibration_data['t_gripper2base'],
            self.calibration_data['R_target2cam'],
            self.calibration_data['t_target2cam'],
            self.H_cam2tcp,
            self.calibration_data['reprojection_errors']
        )

        return validation_results

    def _generate_calibration_report(self, validation_results: Dict[str, Any]) -> None:
        """Generate calibration report and save results."""
        try:
            # Save calibration data
            self._save_calibration_data()

            # Save hand-eye matrix
            if self.H_cam2tcp is not None:
                np.save(self.config.hand_eye_matrix_file, self.H_cam2tcp)
                logger.info(
                    f"Hand-eye matrix saved to {self.config.hand_eye_matrix_file}")

            # Generate and save report
            report = self.validator.generate_calibration_report(
                validation_results, self.H_cam2tcp, self.config.calibration_report_file
            )

            # Create validation plots
            self.validator.create_validation_plots(
                self.calibration_data['reprojection_errors'],
                [np.linalg.norm(A @ self.H_cam2tcp - self.H_cam2tcp @ B, 'fro')
                 for A, B in zip(
                     [np.eye(4)
                      for _ in self.calibration_data['R_gripper2base']],
                     [np.eye(4) for _ in self.calibration_data['R_target2cam']]
                )],
                f"calibration_validation_plots.png"
            )

            logger.info("Calibration report and data saved successfully")

        except Exception as e:
            logger.error(f"Failed to generate calibration report: {e}")

    def _save_calibration_data(self) -> None:
        """Save calibration data to file."""
        try:
            with open(self.config.calibration_data_file, 'wb') as f:
                pickle.dump(self.calibration_data, f)
            logger.info(
                f"Calibration data saved to {self.config.calibration_data_file}")
        except Exception as e:
            logger.error(f"Failed to save calibration data: {e}")

    def load_calibration_data(self, filepath: str) -> bool:
        """Load calibration data from file."""
        try:
            with open(filepath, 'rb') as f:
                self.calibration_data = pickle.load(f)
            logger.info(f"Calibration data loaded from {filepath}")
            return True
        except Exception as e:
            logger.error(f"Failed to load calibration data: {e}")
            return False

    def get_hand_eye_matrix(self) -> Optional[np.ndarray]:
        """Get the hand-eye transformation matrix."""
        return self.H_cam2tcp

    def cleanup(self) -> None:
        """Clean up system resources."""
        try:
            if self.camera_manager:
                self.camera_manager.cleanup()

            if self.kinematics_solver:
                self.kinematics_solver.disconnect()

            if self.robot_client:
                self.robot_client.stop()

            logger.info("Calibration system cleanup complete")

        except Exception as e:
            logger.error(f"Error during cleanup: {e}")

    def __del__(self):
        """Destructor to ensure cleanup."""
        self.cleanup()
