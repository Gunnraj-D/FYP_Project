"""
Example demonstrating the integrated robot control system.
Shows how to use the new integrated system with proper state machine integration.
"""
from integrated_robot_control_system import IntegratedRobotControlSystem, SystemMode
import time
import logging
import sys
from pathlib import Path

# Add src directory to path for imports
sys.path.append(str(Path(__file__).parent.parent))


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def demonstrate_integrated_system():
    """Demonstrate the integrated robot control system."""
    logger.info("=== Integrated Robot Control System Demonstration ===")

    # Create system instance
    system = IntegratedRobotControlSystem()

    try:
        # Initialize camera
        logger.info("Initializing camera...")
        if not system.camera_manager.initialize():
            logger.error("Failed to initialize camera")
            return False

        # Start hand tracker
        logger.info("Starting hand tracker...")
        system.hand_tracker.start()

        # Start OPC UA communication
        logger.info("Starting OPC UA communication...")
        system.opc_client.start()

        # Wait for system to stabilize
        logger.info("Waiting for system to stabilize...")
        time.sleep(2)

        # Demonstrate system status
        logger.info("\n--- System Status ---")
        status = system.get_system_status()
        logger.info(f"System Status: {status}")

        # Demonstrate mode switching
        logger.info("\n--- Mode Switching Demonstration ---")

        # Switch to tracking mode
        logger.info("Switching to TRACKING mode...")
        system.set_mode(SystemMode.TRACKING)
        time.sleep(2)
        logger.info(f"Current state: {system.get_current_state()}")
        logger.info(f"Current mode: {system.get_current_mode()}")

        # Switch to pickup mode
        logger.info("Switching to PICKUP mode...")
        system.set_mode(SystemMode.PICKUP)
        time.sleep(2)
        logger.info(f"Current state: {system.get_current_state()}")
        logger.info(f"Current mode: {system.get_current_mode()}")

        # Switch to placement mode
        logger.info("Switching to PLACEMENT mode...")
        system.set_mode(SystemMode.PLACEMENT)
        time.sleep(2)
        logger.info(f"Current state: {system.get_current_state()}")
        logger.info(f"Current mode: {system.get_current_mode()}")

        # Switch to complete task mode
        logger.info("Switching to COMPLETE_TASK mode...")
        system.set_mode(SystemMode.COMPLETE_TASK)
        time.sleep(2)
        logger.info(f"Current state: {system.get_current_state()}")
        logger.info(f"Current mode: {system.get_current_mode()}")

        # Return to idle
        logger.info("Returning to IDLE mode...")
        system.set_mode(SystemMode.IDLE)
        time.sleep(1)
        logger.info(f"Current state: {system.get_current_state()}")
        logger.info(f"Current mode: {system.get_current_mode()}")

        # Demonstrate task orchestrator status
        logger.info("\n--- Task Orchestrator Status ---")
        orchestrator_status = system.task_orchestrator.get_status()
        logger.info(f"Orchestrator Status: {orchestrator_status}")

        # Demonstrate emergency stop
        logger.info("\n--- Emergency Stop Demonstration ---")
        logger.info("Activating emergency stop...")
        system.emergency_stop()
        logger.info(
            f"Current mode after emergency stop: {system.get_current_mode()}")
        logger.info(f"Error message: {system.error_message}")

        # Demonstrate system reset
        logger.info("\n--- System Reset Demonstration ---")
        logger.info("Resetting system...")
        system.reset_system()
        logger.info(f"Current mode after reset: {system.get_current_mode()}")
        logger.info(f"Error message after reset: {system.error_message}")

        logger.info("=== Demonstration Complete ===")
        return True

    except Exception as e:
        logger.error(f"Demonstration failed: {e}")
        return False
    finally:
        # Clean up
        logger.info("Cleaning up system...")
        system.stop()


def demonstrate_state_machine_integration():
    """Demonstrate the state machine integration."""
    logger.info("\n=== State Machine Integration Demonstration ===")

    # Create system instance
    system = IntegratedRobotControlSystem()

    try:
        # Initialize camera
        if not system.camera_manager.initialize():
            logger.error("Failed to initialize camera")
            return False

        # Start hand tracker
        system.hand_tracker.start()

        # Start OPC UA communication
        system.opc_client.start()

        # Wait for system to stabilize
        time.sleep(2)

        # Demonstrate state machine operations
        logger.info("--- State Machine Operations ---")

        # Get current state
        current_state = system.get_current_state()
        logger.info(f"Current state: {current_state}")

        # Step the state machine
        logger.info("Stepping state machine...")
        system.state_machine.step()
        logger.info(f"State after step: {system.get_current_state()}")

        # Demonstrate state transition
        logger.info("Transitioning to pickup mode...")
        system.set_mode(SystemMode.PICKUP)

        # Step through a few cycles
        for i in range(5):
            logger.info(f"Step {i+1}:")
            system.state_machine.step()
            logger.info(f"  Current state: {system.get_current_state()}")
            logger.info(
                f"  State complete: {system.state_machine.current_state.is_complete()}")
            time.sleep(0.5)

        logger.info("=== State Machine Integration Demonstration Complete ===")
        return True

    except Exception as e:
        logger.error(f"State machine demonstration failed: {e}")
        return False
    finally:
        # Clean up
        system.stop()


def demonstrate_configuration_integration():
    """Demonstrate the configuration integration."""
    logger.info("\n=== Configuration Integration Demonstration ===")

    # Import configuration
    from config.config import (
        GRASP_DETECTION_CONFIG, GRASP_EXECUTION_CONFIG,
        HAND_STABILITY_THRESHOLD, HAND_STABILITY_TIME_THRESHOLD,
        CAMERA_TRANSLATION, CAMERA_ROTATION_EULER
    )

    logger.info("--- Configuration Parameters ---")
    logger.info(f"Grasp Detection Config: {GRASP_DETECTION_CONFIG}")
    logger.info(f"Grasp Execution Config: {GRASP_EXECUTION_CONFIG}")
    logger.info(f"Hand Stability Threshold: {HAND_STABILITY_THRESHOLD}mm")
    logger.info(
        f"Hand Stability Time Threshold: {HAND_STABILITY_TIME_THRESHOLD}s")
    logger.info(f"Camera Translation: {CAMERA_TRANSLATION}")
    logger.info(f"Camera Rotation: {CAMERA_ROTATION_EULER}")

    # Demonstrate configuration usage in states
    logger.info("\n--- Configuration Usage in States ---")

    # Create system and context
    system = IntegratedRobotControlSystem()

    # Show how states use configuration
    from states.generate_pickup_state import GeneratePickupState
    from states.hand_tracking_state import HandTrackingState

    # Generate pickup state uses configuration
    pickup_state = GeneratePickupState(system.context)
    logger.info(
        f"GeneratePickupState timeout: {pickup_state.grasp_generation_timeout}s")
    logger.info(
        f"GeneratePickupState min quality: {pickup_state.min_grasp_quality}")
    logger.info(
        f"GeneratePickupState frame interval: {pickup_state.frame_processing_interval}s")

    # Hand tracking state uses configuration
    hand_state = HandTrackingState(system.context)
    logger.info(
        f"HandTrackingState stability interval: {hand_state.stability_check_interval}s")

    logger.info("=== Configuration Integration Demonstration Complete ===")
    return True


def main():
    """Main demonstration function."""
    try:
        logger.info("Starting Integrated System Examples")

        # Demonstrate integrated system
        success1 = demonstrate_integrated_system()

        # Demonstrate state machine integration
        success2 = demonstrate_state_machine_integration()

        # Demonstrate configuration integration
        success3 = demonstrate_configuration_integration()

        if success1 and success2 and success3:
            logger.info("All demonstrations completed successfully!")
            return 0
        else:
            logger.error("Some demonstrations failed")
            return 1

    except Exception as e:
        logger.error(f"Demonstration failed: {e}")
        return 1


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
