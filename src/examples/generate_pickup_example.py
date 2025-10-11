"""
Example demonstrating GeneratePickupState usage.
Shows how to use the state to generate grasp poses for objects.
"""
import time
import logging
from control.command_bus import CommandBus
from control.telemetry_store import Telemetry
from camera_management.camera_manager import CameraManager, CameraConfig
from IO_handling.opc_client import OPCClient, OPCConfig
from kinematics.kinematics_solver import InverseKinematicsSolver
from states.context import StateContext
from states.generate_pickup_state import GeneratePickupState
from config import (
    URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS,
    OPC_SERVER_URL, OPC_OBJECTS_NAME, ROBOT_ID, get_robot_name
)

# Setup logging
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


def main():
    """Example of GeneratePickupState usage."""

    # Initialize shared components
    telemetry = Telemetry()
    command_bus = CommandBus()

    # Initialize camera manager
    camera_config = CameraConfig()
    camera_manager = CameraManager(camera_config)

    # Initialize OPC client
    opc_config = OPCConfig()
    opc_client = OPCClient(command_bus, telemetry, opc_config)

    # Initialize kinematics solver
    kinematics_solver = InverseKinematicsSolver(
        URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS
    )

    try:
        # Initialize camera
        if not camera_manager.initialize():
            logger.error("Failed to initialize camera")
            return

        # Create state context
        context = StateContext(
            telemetry=telemetry,
            commands=command_bus,
            camera=camera_manager,
            opc=opc_client,
            ik=kinematics_solver
        )

        # Create and enter the GeneratePickupState
        generate_pickup_state = GeneratePickupState(context)

        logger.info("Starting grasp pose generation...")
        generate_pickup_state.enter()

        # Run the state until completion
        start_time = time.time()
        max_runtime = 30.0  # Maximum 30 seconds

        while not generate_pickup_state.is_complete():
            generate_pickup_state.execute()

            # Check for timeout
            if time.time() - start_time > max_runtime:
                logger.warning("Maximum runtime exceeded")
                break

            # Small delay to prevent excessive CPU usage
            time.sleep(0.1)

        # Get results
        stats = generate_pickup_state.get_generation_stats()
        logger.info(f"Generation completed: {stats}")

        if generate_pickup_state.get_grasp_joint_angles() is not None:
            logger.info("✅ Valid grasp pose generated!")
            logger.info(
                f"Joint angles: {generate_pickup_state.get_grasp_joint_angles()}")
            logger.info(
                f"Quality: {generate_pickup_state.get_grasp_quality():.3f}")

            # The grasp pose is now stored in telemetry
            pickup_joints = telemetry.get_pickup_pose_joints()
            logger.info(f"Pickup pose stored in telemetry: {pickup_joints}")
        else:
            logger.warning("❌ No valid grasp pose was generated")

        # Exit the state
        generate_pickup_state.exit()

    except KeyboardInterrupt:
        logger.info("Received keyboard interrupt")
    except Exception as e:
        logger.error(f"Error: {e}")
    finally:
        # Cleanup
        if 'camera_manager' in locals():
            camera_manager.cleanup()
        logger.info("Example completed")


if __name__ == "__main__":
    main()
