"""
Example script demonstrating GG-CNN2 visual debugging mode with RealSense camera.

This script shows how to enable and use the visual debugging feature
for the GG-CNN2 grasp detection system using live camera feed.

Usage:
    python debug_ggcnn2_example.py

Controls:
    SPACEBAR - Capture current frame and run GG-CNN2 inference
    ESC or 'q' - Exit the application

Make sure to set DEBUG_MODE = True in config.py before running.
"""

from config import DEBUG_MODE, DEBUG_CONFIG, URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS, PRE_PICKUP_POSE
from object_detection.grasp_detector import GraspDetector as GGcnn2Module
from control.telemetry_store import Telemetry
from control.command_bus import CommandBus
from camera_management.camera_manager import CameraManager
from kinematics.kinematics_solver import InverseKinematicsSolver
import sys
import os
import numpy as np
import cv2
import logging
from pathlib import Path

# Add src directory to path
src_dir = Path(__file__).parent.parent
sys.path.insert(0, str(src_dir))

# Add project root to path for imports
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

# Ensure we can import from src
os.environ['PYTHONPATH'] = str(
    src_dir) + os.pathsep + os.environ.get('PYTHONPATH', '')


# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def create_components():
    """
    Create system components for real camera usage.
    """
    # Initialize telemetry
    telemetry = Telemetry()

    # For debug purposes, initialize with 7 joint angles (matching KUKA iiwa)
    # In a real system, this would come from the robot
    debug_joints = [0.0] * 7  # 7 movable joints for KUKA iiwa
    telemetry._arm.current_joints = debug_joints
    telemetry._arm.target_joints = debug_joints

    # Initialize command bus
    command_bus = CommandBus()

    # Initialize camera manager (this will connect to RealSense)
    camera_manager = CameraManager()

    # Initialize the camera
    if not camera_manager.initialize():
        raise RuntimeError(
            "Failed to initialize RealSense camera. Make sure it's connected!")

    # Initialize kinematics solver
    kinematics_solver = InverseKinematicsSolver(
        URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS)

    return telemetry, command_bus, camera_manager, kinematics_solver


def display_camera_feed(camera_manager, window_name=None):
    """
    Display live camera feed with instructions using new debug configuration.

    Returns:
        tuple: (color_frame, depth_frame) when spacebar is pressed, None otherwise
    """
    # Use debug config for window name and settings
    if window_name is None:
        window_name = DEBUG_CONFIG.get(
            'window_title', 'Debug Feed - Press SPACEBAR to process frame')

    show_live_feed = DEBUG_CONFIG.get('show_live_feed', True)
    frame_selection_enabled = DEBUG_CONFIG.get('frame_selection_enabled', True)

    if not show_live_feed:
        print("Live feed display is disabled in debug configuration")
        return None, None

    print("Camera feed started. Press SPACEBAR to capture frame, ESC or 'q' to exit")
    if frame_selection_enabled:
        print("Frame selection is ENABLED - you can choose which frames to process")
    else:
        print("Frame selection is DISABLED - all frames will be processed automatically")

    while True:
        try:
            # Get frames from camera
            color_frame, depth_frame = camera_manager.get_frames()

            if color_frame is None or depth_frame is None:
                print("No frames received from camera")
                continue

            # Convert frames to numpy arrays for display
            if hasattr(color_frame, 'get_data'):
                color_array = np.asanyarray(color_frame.get_data())
            else:
                color_array = color_frame

            if hasattr(depth_frame, 'get_data'):
                depth_array = np.asanyarray(depth_frame.get_data())
            else:
                depth_array = depth_frame

            # Create display image (combine color and depth)
            # Resize depth to match color frame
            if color_array.shape[:2] != depth_array.shape:
                depth_resized = cv2.resize(
                    depth_array, (color_array.shape[1], color_array.shape[0]))
            else:
                depth_resized = depth_array

            # Normalize depth for display
            if depth_resized.dtype != np.uint8:
                depth_normalized = cv2.normalize(
                    depth_resized, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                depth_colored = cv2.applyColorMap(
                    depth_normalized, cv2.COLORMAP_JET)
            else:
                depth_colored = cv2.applyColorMap(
                    depth_resized, cv2.COLORMAP_JET)

            # Create side-by-side display
            display_image = np.hstack([color_array, depth_colored])

            # Add instructions text using debug config
            instructions = [
                "DEBUG MODE - Frame Selection",
                "Press SPACEBAR to capture frame",
                "Press ESC or 'q' to exit",
                f"Frame Selection: {'Enabled' if frame_selection_enabled else 'Disabled'}",
                "Left: Color | Right: Depth"
            ]

            y_offset = 30
            for i, text in enumerate(instructions):
                color = (0, 255, 0) if i == 0 else (255, 255, 255)
                thickness = 2 if i == 0 else 1
                cv2.putText(display_image, text, (10, y_offset),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, thickness)
                y_offset += 25

            # Display the image
            cv2.imshow(window_name, display_image)

            # Check for key presses
            key = cv2.waitKey(1) & 0xFF

            if key == ord(' '):  # Spacebar pressed
                print("Frame captured! Running GG-CNN2 inference...")
                return color_frame, depth_frame
            elif key == 27 or key == ord('q'):  # ESC or 'q' pressed
                print("Exiting camera feed...")
                return None, None

        except Exception as e:
            logger.error(f"Error in camera feed: {e}")
            print(f"Camera feed error: {e}")
            return None, None


def demonstrate_debug_config():
    """Demonstrate the new debug configuration features."""
    print("\nDebug Configuration Features:")
    print("-" * 30)

    if DEBUG_MODE:
        print("✓ Debug mode is ENABLED")

        # Show current debug configuration
        print("\nCurrent Debug Settings:")
        for key, value in DEBUG_CONFIG.items():
            status = "✓" if value else "✗"
            print(f"  {status} {key}: {value}")

        # Show what features are available
        print("\nAvailable Features:")
        if DEBUG_CONFIG.get('show_live_feed', True):
            print("  ✓ Live camera feed display")
        else:
            print("  ✗ Live camera feed display (disabled)")

        if DEBUG_CONFIG.get('frame_selection_enabled', True):
            print("  ✓ Frame selection with spacebar")
        else:
            print("  ✗ Frame selection (disabled - all frames processed)")

        print(
            f"  ✓ Window title: '{DEBUG_CONFIG.get('window_title', 'Default')}'")
        print(
            f"  ✓ Quality threshold: {DEBUG_CONFIG.get('display_quality_threshold', 0.1)}")

        print("\nHow to modify debug settings:")
        print("  - Edit DEBUG_CONFIG in src/config/config.py")
        print("  - Or modify settings at runtime (see example below)")

        # Show example of runtime modification
        print("\nExample: Modifying debug settings at runtime")
        print("  DEBUG_CONFIG['show_live_feed'] = False  # Disable live feed")
        print(
            "  DEBUG_CONFIG['frame_selection_enabled'] = False  # Auto-process all frames")
        print(
            "  DEBUG_CONFIG['window_title'] = 'My Custom Title'  # Custom window title")
    else:
        print("✗ Debug mode is DISABLED")
        print("  Set DEBUG_MODE = True in config.py to enable features")


def main():
    """Main function to demonstrate GG-CNN2 debugging with real camera."""

    print("GG-CNN2 Visual Debugging Example with RealSense Camera")
    print("=" * 55)

    # Demonstrate debug configuration
    demonstrate_debug_config()

    # Check if debug mode is enabled
    if not DEBUG_MODE:
        print("\nWARNING: DEBUG_MODE is set to False in config.py")
        print("Please set DEBUG_MODE = True to see visualizations")
        print("Continuing without visualizations...")
    else:
        print("\nYou will see debug windows when processing frames:")
        print("1. 'GG-CNN2 Input Frame' - Shows the processed depth image")
        print("2. 'GG-CNN2 Grasp Output' - Shows the detected grasp pose")

    try:
        # Create system components
        telemetry, command_bus, camera_manager, kinematics_solver = create_components()
        pickup_pose_joints = kinematics_solver.solve_pose(
            PRE_PICKUP_POSE,
            telemetry.get_current_joints()
        )
        telemetry.update_current_joints(pickup_pose_joints.tolist())

        # Initialize GGCNN2 module
        model_path = src_dir / "resources" / "ml_models" / \
            "ggcnn2_weights_cornell" / "epoch_50_cornell_statedict.pt"

        if not model_path.exists():
            print(f"ERROR: Model file not found at {model_path}")
            print("Please ensure the GG-CNN2 model is available")
            return

        ggcnn2_module = GGcnn2Module(
            model_path=str(model_path),
            telemetry=telemetry,
            command_bus=command_bus,
            camera_manager=camera_manager,
            kinematics_solver=kinematics_solver
        )

        print("\nStarting camera feed...")
        print("Make sure your RealSense camera is connected!")

        # Display camera feed and wait for frame capture
        while True:
            color_frame, depth_frame = display_camera_feed(camera_manager)

            if color_frame is None or depth_frame is None:
                # User pressed ESC or 'q'
                break

            print("Processing captured frame...")

            # Run GG-CNN2 inference (this will trigger visualizations if debug mode is enabled)
            # Pass the original depth frame, not the numpy array
            grasp_result = ggcnn2_module.process_depth_frame(depth_frame)

            if grasp_result:
                print("Grasp detection successful!")
                print(f"Grasp quality: {grasp_result['quality']:.3f}")
                print(f"Grasp center: {grasp_result['grasp_2d']['center']}")
                print(
                    f"Grasp angle: {np.degrees(grasp_result['grasp_2d']['angle']):.1f}°")
                print(
                    f"Grasp width: {grasp_result['grasp_2d']['width']:.1f}px")

                if DEBUG_MODE:
                    print("Check the debug windows for visualization!")
                    print("Press any key in the debug windows to continue...")
                    cv2.waitKey(0)
            else:
                print("No valid grasp found in this frame")

            print(
                "\nPress SPACEBAR in camera feed to capture another frame, or ESC/'q' to exit")

        # Clean up
        print("Cleaning up...")
        ggcnn2_module.cleanup()
        camera_manager.cleanup()
        cv2.destroyAllWindows()

    except Exception as e:
        logger.error(f"Error in debug example: {e}")
        print(f"Error: {e}")
        cv2.destroyAllWindows()

        # Try to clean up camera if it was initialized
        try:
            if 'camera_manager' in locals() and camera_manager.is_initialized:
                camera_manager.cleanup()
        except:
            pass

    print("Debug example completed")


if __name__ == "__main__":
    main()
