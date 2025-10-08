"""
Semi-automated hand-eye calibration with live preview and manual capture.
"""
import cv2
import numpy as np
import time
import logging
import pickle
import json
from pathlib import Path
from typing import List, Tuple, Optional, Dict, Any
import threading
import queue
import asyncio

from .calibration_config import CalibrationConfig
from .hand_eye_calibrator import HandEyeCalibrator
from .pose_generator import PoseGenerator
from IO_handling.opc_client_factory import OPCClientFactory
from control.telemetry_store import Telemetry
from control.command_bus import CommandBus, SetJoints
from kinematics.collision_aware_kinematics_solver import CollisionAwareKinematicsSolver
from camera_management.camera_manager import CameraManager

# Add src directory to path
import sys
sys.path.append(str(Path(__file__).parent.parent))

logger = logging.getLogger(__name__)


class SemiAutomatedCalibrator:
    """Semi-automated hand-eye calibration with live preview and manual capture."""

    def __init__(self, config: CalibrationConfig = None):
        self.config = config or CalibrationConfig()
        self.pose_generator = PoseGenerator(self.config)

        # Try to load existing hand-eye matrix for improved pose generation
        if self.pose_generator.load_hand_eye_matrix():
            print("🎯 Using camera-centric pose generation with hand-eye matrix")
        else:
            print("⚠️ Using legacy TCP-centric pose generation")

        # System components
        self.camera_manager = None
        self.kinematics_solver = None
        self.robot_client = None

        # Mock server management
        self.mock_server = None
        self.mock_server_thread = None
        self._shutdown_event = threading.Event()

        # Calibration data
        self.calibration_data = {
            'poses_used': [],
            'R_gripper2base': [],
            't_gripper2base': [],
            'R_target2cam': [],
            't_target2cam': [],
            'reprojection_errors': [],
            'joint_angles': [],
            'capture_metadata': []
        }

        # Live preview
        self.preview_running = False
        self.preview_thread = None
        self.frame_queue = queue.Queue(maxsize=1)

        # Target checkerboard position
        self.target_position = (0.4, 0.025, 0.22)  # (x, y, z) in base frame
        self.robot_base_z = 0.202  # Robot base height

    def start_mock_server(self):
        """Start mock server in background thread with proper event loop."""
        if self.mock_server is None:
            print("🚀 Starting mock OPC UA server...")

            # Create a new event loop for the server thread
            def run_server():
                try:
                    loop = asyncio.new_event_loop()
                    asyncio.set_event_loop(loop)

                    async def start_server():
                        from IO_handling.mock_opc_server import MockOPCServer
                        from config.config import ROBOT_ID
                        self.mock_server = MockOPCServer(
                            url="opc.tcp://127.0.0.1:4840/", robot_id=ROBOT_ID)
                        await self.mock_server.initialize()
                        await self.mock_server.start_server()
                        print("✅ Mock OPC UA server ready")

                        # Keep the server running
                        try:
                            while self.mock_server and not self._shutdown_event.is_set():
                                await asyncio.sleep(0.1)
                        except Exception as e:
                            print(f"❌ Server loop error: {e}")
                        finally:
                            if self.mock_server:
                                await self.mock_server.stop_server()
                                print("✅ Mock OPC UA server stopped")

                    loop.run_until_complete(start_server())
                    loop.close()

                except Exception as e:
                    print(f"❌ Mock server thread error: {e}")

            self.mock_server_thread = threading.Thread(
                target=run_server,
                name="MockOPCServer",
                daemon=True
            )
            self.mock_server_thread.start()

            # Wait for server to start and be ready
            time.sleep(3)
            print("✅ Mock server ready")

    async def stop_mock_server(self):
        """Stop the mock OPC UA server."""
        if self.mock_server:
            try:
                self._shutdown_event.set()
                await self.mock_server.stop_server()
                print("✅ Mock OPC UA server stopped")
            except Exception as e:
                print(f"❌ Error stopping mock server: {e}")
            finally:
                self.mock_server = None

    def initialize(self, opc_mode: str = "mock") -> bool:
        """Initialize the calibration system."""
        try:
            logger.info("Initializing semi-automated calibration system...")

            # Start mock server if in mock mode
            if opc_mode == "mock":
                self.start_mock_server()

            # Initialize camera
            self.camera_manager = CameraManager()
            if not self.camera_manager.initialize():
                logger.error("Failed to initialize camera")
                return False

            # Initialize collision-aware kinematics solver
            self.kinematics_solver = CollisionAwareKinematicsSolver(
                self.config.urdf_filepath,
                self.config.base_elements,
                self.config.active_links,
                use_gui=False,
                table_id=None  # No table collision detection during calibration
            )

            # Initialize robot client
            from control.command_bus import CommandBus
            from control.telemetry_store import Telemetry

            command_bus = CommandBus()
            telemetry = Telemetry()

            if opc_mode == "mock":
                # Give mock server a bit more time to be fully ready
                time.sleep(2)
                opc_factory = OPCClientFactory()
                self.robot_client = opc_factory.create_client(
                    command_bus=command_bus,
                    telemetry=telemetry,
                    mode=opc_mode
                )
                self.robot_client.start()
                print("🔧 Using mock robot client with mock server")
            else:
                opc_factory = OPCClientFactory()
                self.robot_client = opc_factory.create_client(
                    command_bus=command_bus,
                    telemetry=telemetry,
                    mode=opc_mode
                )
                self.robot_client.start()
                # Give the client a moment to initialize
                time.sleep(1.0)

            logger.info(
                "Semi-automated calibration system initialized successfully")
            return True

        except Exception as e:
            logger.error(f"Failed to initialize calibration system: {e}")
            return False

    def run_calibration(self, num_poses: int = 15) -> bool:
        """Run semi-automated calibration with live preview."""
        try:
            logger.info(
                f"Starting semi-automated calibration with {num_poses} poses")

            # Generate poses targeting the checkerboard
            if self.pose_generator.hand_eye_matrix is not None:
                # Use camera-centric approach with hand-eye matrix
                poses = self.pose_generator.generate_camera_centric_poses(
                    num_poses,
                    target_position=self.target_position
                )
            else:
                # Fallback to legacy TCP-centric approach
                poses = self.pose_generator.generate_poses(
                    num_poses,
                    target_position=self.target_position,
                    robot_base_z=self.robot_base_z
                )

            logger.info(
                f"Generated {len(poses)} poses targeting checkerboard at {self.target_position}")

            # Start live preview
            self.start_live_preview()

            # Execute calibration poses
            success_count = 0
            for i, pose in enumerate(poses):
                logger.info(f"Executing pose {i+1}/{len(poses)}")

                # Move robot to pose
                if not self._move_robot_to_pose(pose):
                    logger.warning(f"Failed to move to pose {i+1}, skipping")
                    continue

                # Wait for manual capture
                capture_success = self._wait_for_manual_capture(
                    i+1, len(poses))

                if capture_success:
                    success_count += 1
                    logger.info(f"Successfully captured pose {i+1}")
                else:
                    logger.info(f"Skipped pose {i+1}")

            # Stop live preview
            self.stop_live_preview()

            if success_count < self.config.min_poses_required:
                logger.error(
                    f"Insufficient captures: {success_count} < {self.config.min_poses_required}")
                return False

            # Perform hand-eye calibration
            logger.info("Performing hand-eye calibration...")
            calibrator = HandEyeCalibrator(self.config)
            calibrator.calibration_data = self.calibration_data

            if not calibrator._perform_hand_eye_calibration():
                logger.error("Hand-eye calibration failed")
                return False

            # Save results
            self._save_calibration_results(calibrator.H_cam2tcp)

            logger.info("Semi-automated calibration completed successfully!")
            return True

        except Exception as e:
            logger.error(f"Calibration failed: {e}")
            return False
        finally:
            self.stop_live_preview()

    def start_live_preview(self):
        """Start live camera preview in separate thread."""
        self.preview_running = True
        self.preview_thread = threading.Thread(
            target=self._preview_loop, daemon=True)
        self.preview_thread.start()

    def stop_live_preview(self):
        """Stop live camera preview."""
        self.preview_running = False
        if self.preview_thread:
            self.preview_thread.join(timeout=2.0)
        cv2.destroyAllWindows()

    def _preview_loop(self):
        """Live preview loop showing camera feed with checkerboard detection."""
        while self.preview_running:
            try:
                # Get camera frames
                color_frame, depth_frame = self.camera_manager.get_frames()
                if color_frame is None:
                    continue

                # Detect checkerboard
                success, corners = self._detect_checkerboard_corners(
                    color_frame)

                # Create display frame
                display_frame = color_frame.copy()

                # Draw checkerboard detection status
                if success:
                    cv2.drawChessboardCorners(
                        display_frame, self.config.chessboard_size, corners, success)
                    cv2.putText(display_frame, "CHECKERBOARD DETECTED", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                else:
                    cv2.putText(display_frame, "NO CHECKERBOARD", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                # Add instructions
                cv2.putText(display_frame, "SPACEBAR: Capture | S: Skip | ESC: Exit",
                            (10, display_frame.shape[0] - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                # Show frame
                cv2.imshow("Calibration Preview", display_frame)

                # Handle key presses
                key = cv2.waitKey(1) & 0xFF
                if key == 27:  # ESC
                    self.preview_running = False
                    break
                elif key == ord(' '):  # SPACEBAR
                    if not self.frame_queue.full():
                        self.frame_queue.put(
                            (color_frame, depth_frame, success, corners))
                elif key == ord('s'):  # Skip
                    if not self.frame_queue.full():
                        self.frame_queue.put(None)  # Signal to skip

            except Exception as e:
                logger.error(f"Preview loop error: {e}")
                time.sleep(0.1)

    def _wait_for_manual_capture(self, pose_num: int, total_poses: int) -> bool:
        """Wait for manual capture with live preview."""
        logger.info(f"Waiting for capture of pose {pose_num}/{total_poses}")
        logger.info("Press SPACEBAR to capture, S to skip, ESC to exit")

        while self.preview_running:
            try:
                # Wait for user input
                result = self.frame_queue.get(timeout=0.1)

                if result is None:
                    logger.info("Pose skipped by user")
                    return False

                color_frame, depth_frame, success, corners = result

                if not success:
                    logger.warning("Checkerboard not detected, try again")
                    continue

                # Capture the data
                if self._capture_pose_data(pose_num, color_frame, depth_frame, corners):
                    logger.info(f"Successfully captured pose {pose_num}")
                    return True
                else:
                    logger.warning("Failed to capture pose data, try again")
                    continue

            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"Error in manual capture: {e}")
                return False

        return False

    def _move_robot_to_pose(self, pose: np.ndarray) -> bool:
        """Move robot to specified pose."""
        try:
            # Convert pose to joint angles
            position = pose[:3, 3]
            orientation = pose[:3, :3]

            print(
                f"🎯 Target position: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]")
            print(f"🎯 Target orientation: {orientation}")

            # Get current joint positions
            current_joints = self.robot_client.telemetry.get_current_joints()
            if current_joints is None or len(current_joints) == 0:
                current_joints = [0.0] * 7
            print(f"📍 Current joints: {current_joints}")

            # Solve inverse kinematics
            target_joints = self.kinematics_solver.solve_XYZ(
                position, current_joints, target_orientation=orientation
            )

            # Send joint command
            command = SetJoints(target_joints.tolist())
            print(f"🤖 Sending joint command: {target_joints.tolist()}")
            self.robot_client.command_bus.send(command)
            print("✅ Joint command sent to robot")

            # Wait for movement to complete
            print(
                f"⏳ Waiting {self.config.settling_time}s for robot to settle...")
            time.sleep(self.config.settling_time)

            return True

        except Exception as e:
            logger.error(f"Failed to move robot to pose: {e}")
            return False

    def _detect_checkerboard_corners(self, frame):
        """Detect checkerboard corners in the frame."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        success, corners = cv2.findChessboardCorners(
            gray,
            self.config.chessboard_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        if success:
            # Refine corner positions
            criteria = (cv2.TERM_CRITERIA_EPS +
                        cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria)

        return success, corners

    def _capture_pose_data(self, pose_num: int, color_frame, depth_frame, corners) -> bool:
        """Capture calibration data for current pose."""
        try:
            # Get current robot pose
            current_joints = self.robot_client.telemetry.get_current_joints()
            if current_joints is None:
                logger.error("Failed to get current joint angles")
                return False

            # Calculate current TCP pose
            tcp_matrix, tcp_pose = self.kinematics_solver.tcp_from_joints(
                current_joints)

            # Get camera intrinsics
            if not self.camera_manager.intrinsics:
                logger.error("Camera intrinsics not available")
                return False

            # Compute target pose in camera frame
            success_pnp, rvec, tvec = cv2.solvePnP(
                self._get_object_points(),
                corners,
                self._get_camera_matrix(),
                self._get_distortion_coeffs()
            )

            if not success_pnp:
                logger.error("PnP solve failed")
                return False

            # Convert to rotation matrix
            R_target2cam, _ = cv2.Rodrigues(rvec)
            t_target2cam = tvec.flatten()

            # Calculate reprojection error
            reprojection_error = self._calculate_reprojection_error(
                corners, R_target2cam, t_target2cam
            )

            # Store calibration data
            self.calibration_data['poses_used'].append(tcp_matrix.tolist())
            self.calibration_data['R_gripper2base'].append(
                tcp_matrix[:3, :3].tolist())
            self.calibration_data['t_gripper2base'].append(
                tcp_matrix[:3, 3].tolist())
            self.calibration_data['R_target2cam'].append(R_target2cam.tolist())
            self.calibration_data['t_target2cam'].append(t_target2cam.tolist())
            self.calibration_data['reprojection_errors'].append(
                reprojection_error)
            self.calibration_data['joint_angles'].append(
                current_joints.tolist())

            # Store capture metadata
            metadata = {
                'pose_number': pose_num,
                'timestamp': time.time(),
                'reprojection_error': reprojection_error,
                'tcp_position': tcp_pose[:3],  # tcp_pose is already a list
                'tcp_orientation': tcp_pose[3:]  # tcp_pose is already a list
            }
            self.calibration_data['capture_metadata'].append(metadata)

            # Save individual capture data
            self._save_capture_data(
                pose_num, color_frame, depth_frame, current_joints.tolist(), metadata)

            return True

        except Exception as e:
            logger.error(f"Failed to capture pose data: {e}")
            return False

    def _get_object_points(self):
        """Get 3D object points for checkerboard."""
        objp = np.zeros(
            (self.config.chessboard_size[0] * self.config.chessboard_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.config.chessboard_size[0],
                               0:self.config.chessboard_size[1]].T.reshape(-1, 2)
        objp *= self.config.square_size
        return objp

    def _get_camera_matrix(self):
        """Get camera intrinsic matrix."""
        intrinsics = self.camera_manager.intrinsics
        return np.array([
            [intrinsics.fx, 0, intrinsics.ppx],
            [0, intrinsics.fy, intrinsics.ppy],
            [0, 0, 1]
        ])

    def _get_distortion_coeffs(self):
        """Get camera distortion coefficients."""
        intrinsics = self.camera_manager.intrinsics
        return np.array([intrinsics.coeffs])

    def _calculate_reprojection_error(self, corners, R, t):
        """Calculate reprojection error for corners."""
        obj_points = self._get_object_points()
        img_points_proj, _ = cv2.projectPoints(
            obj_points, R, t, self._get_camera_matrix(), self._get_distortion_coeffs()
        )
        error = cv2.norm(corners, img_points_proj,
                         cv2.NORM_L2) / len(img_points_proj)
        return error

    def _save_capture_data(self, pose_num: int, color_frame, depth_frame, joint_angles, metadata):
        """Save individual capture data for recovery."""
        try:
            # Create timestamped session directory if it doesn't exist
            if not hasattr(self, '_session_dir') or self._session_dir is None:
                import time
                base_dir = Path("calibration_captures")
                self._session_dir = base_dir / time.strftime("%Y%m%d_%H%M%S")
                self._session_dir.mkdir(parents=True, exist_ok=True)
                print(f"📁 Saving captures to: {self._session_dir}")

            # Create capture directory within session
            capture_dir = self._session_dir / f"pose_{pose_num:03d}"
            capture_dir.mkdir(parents=True, exist_ok=True)

            # Save images
            cv2.imwrite(str(capture_dir / "color.png"), color_frame)
            if depth_frame is not None:
                # Convert depth frame to numpy array if it's a RealSense frame
                if hasattr(depth_frame, 'get_data'):
                    depth_array = np.asanyarray(depth_frame.get_data())
                else:
                    depth_array = depth_frame
                cv2.imwrite(str(capture_dir / "depth.png"), depth_array)

            # Save joint angles
            with open(capture_dir / "joint_angles.json", 'w') as f:
                json.dump(joint_angles, f, indent=2)

            # Save metadata
            with open(capture_dir / "metadata.json", 'w') as f:
                json.dump(metadata, f, indent=2)

            logger.info(f"Saved capture data for pose {pose_num}")

        except Exception as e:
            logger.error(f"Failed to save capture data: {e}")

    def _save_calibration_results(self, hand_eye_matrix):
        """Save final calibration results."""
        try:
            # Save hand-eye matrix
            np.save(self.config.hand_eye_matrix_file, hand_eye_matrix)

            # Save calibration data
            with open(self.config.calibration_data_file, 'wb') as f:
                pickle.dump(self.calibration_data, f)

            # Generate report
            self._generate_calibration_report(hand_eye_matrix)

            logger.info("Calibration results saved successfully")

        except Exception as e:
            logger.error(f"Failed to save calibration results: {e}")

    def _generate_calibration_report(self, hand_eye_matrix):
        """Generate calibration report."""
        try:
            report_lines = [
                "=" * 60,
                "SEMI-AUTOMATED HAND-EYE CALIBRATION REPORT",
                "=" * 60,
                f"Target Position: {self.target_position}",
                f"Robot Base Height: {self.robot_base_z}m",
                f"Number of Captures: {len(self.calibration_data['poses_used'])}",
                f"Chessboard Size: {self.config.chessboard_size}",
                f"Square Size: {self.config.square_size}m",
                "",
                "REPROJECTION ERRORS:",
                f"  Mean Error: {np.mean(self.calibration_data['reprojection_errors']):.3f} pixels",
                f"  Max Error: {np.max(self.calibration_data['reprojection_errors']):.3f} pixels",
                f"  Std Error: {np.std(self.calibration_data['reprojection_errors']):.3f} pixels",
                "",
                "HAND-EYE TRANSFORMATION MATRIX (Camera to TCP):",
                "Rotation Matrix:",
                f"  {hand_eye_matrix[:3, 0]}",
                f"  {hand_eye_matrix[:3, 1]}",
                f"  {hand_eye_matrix[:3, 2]}",
                "Translation Vector:",
                f"  {hand_eye_matrix[:3, 3]}",
                "=" * 60
            ]

            with open(self.config.calibration_report_file, 'w') as f:
                f.write('\n'.join(report_lines))

            logger.info(
                f"Calibration report saved to {self.config.calibration_report_file}")

        except Exception as e:
            logger.error(f"Failed to generate calibration report: {e}")

    def cleanup(self):
        """Clean up system resources."""
        try:
            self.stop_live_preview()

            if self.camera_manager:
                self.camera_manager.cleanup()

            if self.kinematics_solver:
                self.kinematics_solver.disconnect()

            if self.robot_client:
                self.robot_client.stop()

            # Stop mock server if running
            if self.mock_server:
                try:
                    self._shutdown_event.set()
                    # Wait for server thread to finish
                    if self.mock_server_thread and self.mock_server_thread.is_alive():
                        self.mock_server_thread.join(timeout=2.0)
                        if self.mock_server_thread.is_alive():
                            print("⚠️ Mock server thread did not stop gracefully")
                    self.mock_server = None
                    self.mock_server_thread = None
                except Exception as e:
                    print(f"❌ Error stopping mock server: {e}")

            logger.info("Semi-automated calibration cleanup complete")

        except Exception as e:
            logger.error(f"Error during cleanup: {e}")


def main():
    """Main function to run semi-automated calibration."""
    import argparse

    parser = argparse.ArgumentParser(
        description='Semi-automated Hand-Eye Calibration')
    parser.add_argument('--mode', choices=['real', 'mock'], default='mock',
                        help='Robot communication mode')
    parser.add_argument('--poses', type=int, default=15,
                        help='Number of poses to capture')
    parser.add_argument('--target-x', type=float, default=0.4,
                        help='Target checkerboard X position')
    parser.add_argument('--target-y', type=float, default=0.025,
                        help='Target checkerboard Y position')
    parser.add_argument('--target-z', type=float, default=0.22,
                        help='Target checkerboard Z position')
    parser.add_argument('--robot-base-z', type=float, default=0.202,
                        help='Robot base height (Z coordinate)')

    args = parser.parse_args()

    # Configure logging
    logging.basicConfig(
        level=logging.WARNING,
        format='%(levelname)s - %(message)s'
    )

    # Reduce verbose logging from various modules
    logging.getLogger('pybullet').setLevel(logging.ERROR)
    logging.getLogger('IO_handling').setLevel(logging.ERROR)
    logging.getLogger('asyncua').setLevel(logging.ERROR)
    logging.getLogger('camera_management').setLevel(logging.ERROR)
    logging.getLogger('kinematics').setLevel(logging.ERROR)

    # Suppress specific OPC warnings
    logging.getLogger('IO_handling.opc_client').setLevel(logging.CRITICAL)

    # Create calibrator
    calibrator = SemiAutomatedCalibrator()
    calibrator.target_position = (args.target_x, args.target_y, args.target_z)
    calibrator.robot_base_z = args.robot_base_z

    try:
        # Initialize system
        if not calibrator.initialize(opc_mode=args.mode):
            print("❌ Failed to initialize calibration system")
            return 1

        # Run calibration
        if calibrator.run_calibration(num_poses=args.poses):
            print("✅ Semi-automated calibration completed successfully!")
            return 0
        else:
            print("❌ Semi-automated calibration failed!")
            return 1

    except KeyboardInterrupt:
        print("\n👋 Calibration interrupted by user")
        return 0
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        return 1
    finally:
        calibrator.cleanup()


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
