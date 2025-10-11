#!/usr/bin/env python3
"""
Manual hand-eye calibration with real-time joint position saving.

This script allows manual capture of calibration images by pressing SPACE.
Each capture saves:
- Color image
- Depth image  
- Joint angles (from current robot position)
- Metadata

Press SPACE to capture
Press Q to quit
"""
import cv2
import numpy as np
import json
from pathlib import Path
import time
import logging
import sys
from typing import Optional
import pyrealsense2 as rs

# Add src to path FIRST before any local imports
sys.path.append(str(Path(__file__).parent.parent))

# Local imports AFTER path setup - do not move these above sys.path.append!
from calibration.calibration_config import CalibrationConfig  # noqa: E402
from IO_handling.opc_client import OPCConfig  # noqa: E402
from IO_handling.opc_client_factory import OPCClientFactory  # noqa: E402
from control.command_bus import CommandBus  # noqa: E402
from control.telemetry_store import Telemetry  # noqa: E402
from camera_management.camera_manager import CameraManager, CameraConfig  # noqa: E402


logging.basicConfig(level=logging.WARNING)  # Reduce logging noise
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)  # Keep our own logs at INFO level


class ManualCalibration:
    """Manual calibration with live camera feed and manual capture."""

    def __init__(self, mode: str = "real"):
        """
        Initialize manual calibration.

        Args:
            mode: "real" for real robot, "mock" for simulation
        """
        self.mode = mode
        self.config = CalibrationConfig()

        # Create timestamped session directory
        self.session_dir = Path("calibration_captures") / \
            time.strftime("%Y%m%d_%H%M%S")
        self.session_dir.mkdir(parents=True, exist_ok=True)

        # Initialize components
        self.camera_manager = None
        self.robot_client = None  # Contains telemetry and command_bus

        # Capture counter
        self.capture_count = 0

        logger.info(f"Manual calibration initialized in {mode} mode")
        logger.info(f"Session directory: {self.session_dir}")

    def initialize(self) -> bool:
        """Initialize all components."""
        try:
            # Initialize camera
            logger.info("Initializing camera...")
            camera_config = CameraConfig()
            self.camera_manager = CameraManager(camera_config)
            if not self.camera_manager.initialize():
                logger.error("Failed to initialize camera")
                return False
            logger.info("✅ Camera initialized")

            if self.mode == "real":
                # Initialize robot connection
                logger.info("Initializing robot connection...")
                opc_config = OPCConfig()

                # Create local telemetry and command bus (matching semi_automated pattern)
                # These will be accessed through robot_client, not as instance variables
                command_bus = CommandBus()
                telemetry = Telemetry()

                # Create OPC client with the telemetry instance
                opc_factory = OPCClientFactory()
                self.robot_client = opc_factory.create_client(
                    command_bus=command_bus,
                    telemetry=telemetry,
                    mode="real"
                )

                # Start the OPC client background thread
                self.robot_client.start()

                # Wait for connection and first telemetry update
                logger.info("Waiting for robot connection and telemetry...")
                time.sleep(3.0)

                # Verify we're getting joint data (through robot_client.telemetry)
                test_joints = self.robot_client.telemetry.get_current_joints()
                if np.allclose(test_joints, 0.0):
                    logger.warning(
                        "⚠️ Joint telemetry may not be connected yet")
                    logger.info("Waiting additional time for telemetry...")
                    time.sleep(3.0)
                    test_joints = self.robot_client.telemetry.get_current_joints()

                logger.info(f"Current joints: {test_joints}")

                if not np.allclose(test_joints, 0.0):
                    logger.info("✅ Robot connection and telemetry established")
                else:
                    logger.warning(
                        "⚠️ Still not receiving joint data - continuing anyway")
            else:
                logger.info("Mock mode - no robot connection")

            return True

        except Exception as e:
            logger.error(f"Initialization failed: {e}")
            return False

    def get_current_joints(self) -> Optional[np.ndarray]:
        """Get current joint angles from robot."""
        if self.mode == "mock":
            # Return mock joint angles
            return np.array([0.0, 0.5, 0.0, -1.0, 0.0, 1.5, 0.0])

        # Access telemetry through robot_client (same instance being updated)
        if self.robot_client is None:
            logger.warning("Robot client not initialized")
            return None

        try:
            joints = self.robot_client.telemetry.get_current_joints()
            if joints is not None and len(joints) == 7:
                return joints
            else:
                logger.warning("Invalid joint data")
                return None
        except Exception as e:
            logger.error(f"Failed to get joints: {e}")
            return None

    def capture_pose(self) -> bool:
        """Capture current pose (image + joints)."""
        try:
            # Force a fresh read from telemetry
            # Wait a moment to ensure latest update
            time.sleep(0.1)

            # Get current joint angles
            joint_angles = self.get_current_joints()
            if joint_angles is None:
                logger.error("Failed to get joint angles")
                return False

            # Verify joints are not all zeros (would indicate no telemetry)
            if np.allclose(joint_angles, 0.0):
                logger.warning(
                    "⚠️ Joint angles are all zeros - telemetry may not be updating")
                logger.warning("   Trying to read again...")
                time.sleep(0.5)
                joint_angles = self.get_current_joints()

                if np.allclose(joint_angles, 0.0):
                    logger.error(
                        "❌ Still receiving zero joints - capture may be invalid!")
                    # Continue anyway to save the image, but warn user

            # Get camera frames
            color_frame, depth_frame = self.camera_manager.get_frames()
            if color_frame is None or depth_frame is None:
                logger.error("Failed to get camera frames")
                return False

            # Create pose directory
            self.capture_count += 1
            pose_dir = self.session_dir / f"pose_{self.capture_count:03d}"
            pose_dir.mkdir(parents=True, exist_ok=True)

            # Save color image
            color_path = pose_dir / "color.png"
            cv2.imwrite(str(color_path), color_frame)

            # Save depth image
            depth_array = np.asanyarray(depth_frame.get_data())
            depth_path = pose_dir / "depth.png"
            cv2.imwrite(str(depth_path), depth_array)

            # Save joint angles
            joints_path = pose_dir / "joint_angles.json"
            with open(joints_path, 'w') as f:
                json.dump(joint_angles.tolist(), f, indent=2)

            # Save metadata
            metadata = {
                'timestamp': time.time(),
                'capture_number': self.capture_count,
                'mode': self.mode,
                'joint_angles': joint_angles.tolist(),
                'camera_intrinsics': {
                    'width': self.camera_manager.color_intrinsics.width,
                    'height': self.camera_manager.color_intrinsics.height,
                    'fx': self.camera_manager.color_intrinsics.fx,
                    'fy': self.camera_manager.color_intrinsics.fy,
                    'ppx': self.camera_manager.color_intrinsics.ppx,
                    'ppy': self.camera_manager.color_intrinsics.ppy
                } if self.camera_manager.color_intrinsics else None
            }

            metadata_path = pose_dir / "metadata.json"
            with open(metadata_path, 'w') as f:
                json.dump(metadata, f, indent=2)

            logger.info(f"✅ Captured pose {self.capture_count} to {pose_dir}")
            logger.info(f"   Joint angles: {joint_angles}")

            return True

        except Exception as e:
            logger.error(f"Failed to capture pose: {e}")
            return False

    def run(self):
        """Run manual calibration loop."""
        logger.info("\n" + "="*60)
        logger.info("Manual Hand-Eye Calibration")
        logger.info("="*60)
        logger.info("Instructions:")
        logger.info("  - Move robot to different poses manually")
        logger.info("  - Press SPACE to capture current pose")
        logger.info("  - Press Q to quit and finish calibration")
        logger.info("  - Aim for diverse orientations (especially yaw)")
        logger.info("="*60 + "\n")

        # Create window
        cv2.namedWindow('Manual Calibration', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Manual Calibration', 1280, 720)

        try:
            while True:
                # Get frame
                color_frame, _ = self.camera_manager.get_frames()
                if color_frame is None:
                    logger.warning("No frame received")
                    time.sleep(0.1)
                    continue

                # Get current joint angles for display
                joints = self.get_current_joints()

                # Draw info overlay
                display_frame = color_frame.copy()

                # Detect checkerboard
                gray = cv2.cvtColor(color_frame, cv2.COLOR_BGR2GRAY)
                success, corners = cv2.findChessboardCorners(
                    gray,
                    self.config.chessboard_size,
                    cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
                )

                # Draw checkerboard detection status
                font = cv2.FONT_HERSHEY_SIMPLEX
                y_offset = 30

                if success:
                    # Refine corners for better visualization
                    criteria = (cv2.TERM_CRITERIA_EPS +
                                cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                    corners = cv2.cornerSubPix(
                        gray, corners, (11, 11), (-1, -1), criteria)

                    # Draw detected corners
                    cv2.drawChessboardCorners(
                        display_frame, self.config.chessboard_size, corners, success)

                    cv2.putText(display_frame, "CHECKERBOARD DETECTED", (10, y_offset),
                                font, 1, (0, 255, 0), 2)
                else:
                    cv2.putText(display_frame, "NO CHECKERBOARD", (10, y_offset),
                                font, 1, (0, 0, 255), 2)

                y_offset += 40

                # Add capture count
                cv2.putText(display_frame, f"Captures: {self.capture_count}",
                            (10, y_offset), font, 0.8, (255, 255, 0), 2)
                y_offset += 35

                # Add joint angles with live update indicator
                if joints is not None:
                    # Show if joints are changing (not all zeros and not identical to last frame)
                    if not hasattr(self, '_last_displayed_joints'):
                        self._last_displayed_joints = joints.copy()

                    joints_changed = not np.allclose(
                        joints, self._last_displayed_joints, atol=0.001)
                    status_color = (0, 255, 0) if joints_changed else (
                        128, 128, 128)
                    status_text = "LIVE" if joints_changed else "STATIC"

                    cv2.putText(display_frame, f"Joints [{status_text}]:",
                                (10, y_offset), font, 0.7, status_color, 2)
                    y_offset += 30

                    cv2.putText(display_frame, f"J1-J3: [{joints[0]:.3f}, {joints[1]:.3f}, {joints[2]:.3f}]",
                                (10, y_offset), font, 0.6, (255, 255, 255), 1)
                    y_offset += 30
                    cv2.putText(display_frame, f"J4-J6: [{joints[3]:.3f}, {joints[4]:.3f}, {joints[5]:.3f}]",
                                (10, y_offset), font, 0.6, (255, 255, 255), 1)
                    y_offset += 30
                    cv2.putText(display_frame, f"J7: {joints[6]:.3f}",
                                (10, y_offset), font, 0.6, (255, 255, 255), 1)

                    self._last_displayed_joints = joints.copy()

                # Add instructions
                cv2.putText(display_frame, "SPACE: Capture | Q: Quit",
                            (10, display_frame.shape[0] - 20), font, 0.8, (0, 255, 255), 2)

                # Show frame
                cv2.imshow('Manual Calibration', display_frame)

                # Handle key presses
                key = cv2.waitKey(1) & 0xFF

                if key == ord(' '):
                    # Capture pose
                    logger.info("📸 Capturing pose...")
                    if self.capture_pose():
                        # Flash screen green
                        flash = np.zeros_like(display_frame)
                        flash[:, :, 1] = 255
                        cv2.imshow('Manual Calibration', cv2.addWeighted(
                            display_frame, 0.5, flash, 0.5, 0))
                        cv2.waitKey(200)
                    else:
                        logger.error("❌ Capture failed!")

                elif key == ord('q') or key == ord('Q') or key == 27:  # Q or ESC
                    logger.info("Quitting manual calibration...")
                    break

        finally:
            cv2.destroyAllWindows()

            logger.info("\n" + "="*60)
            logger.info(f"Manual calibration complete!")
            logger.info(f"Captured {self.capture_count} poses")
            logger.info(f"Data saved to: {self.session_dir}")
            logger.info("="*60)

    def cleanup(self):
        """Cleanup resources gracefully."""
        try:
            # Stop OPC client first (this stops telemetry too)
            if self.robot_client:
                logger.info("Stopping OPC client...")
                self.robot_client.stop()
                time.sleep(0.5)  # Give it time to stop gracefully

            # Stop camera
            if self.camera_manager:
                logger.info("Stopping camera...")
                self.camera_manager.cleanup()

            logger.info("✅ Cleanup complete")

        except KeyboardInterrupt:
            logger.info("Cleanup interrupted - forcing shutdown")
        except Exception as e:
            logger.warning(f"Cleanup error (non-critical): {e}")


def main():
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(description="Manual hand-eye calibration")
    parser.add_argument('--mode', type=str, default='real', choices=['real', 'mock'],
                        help='Calibration mode (real or mock)')

    args = parser.parse_args()

    calibrator = ManualCalibration(mode=args.mode)

    try:
        if not calibrator.initialize():
            logger.error("Failed to initialize calibrator")
            return 1

        calibrator.run()

        return 0

    except KeyboardInterrupt:
        logger.info("\n👋 Interrupted by user")
        return 0

    except Exception as e:
        logger.error(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1

    finally:
        calibrator.cleanup()


if __name__ == "__main__":
    sys.exit(main())
