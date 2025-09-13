"""
Main control loop for robot arm hand tracking system.
Refactored with modular architecture, shared resource management, and state machine design.
"""
import time
import logging
from typing import Optional

# Core modules
from control.telemetry_store import Telemetry
from control.command_bus import CommandBus
from camera_management.camera_manager import CameraManager, CameraConfig
from IO_handling.opc_client import OPCClient, OPCConfig
from hand_detection.hand_detection_module import HandTracker, HandTrackingConfig
from object_detection.ggcnn2_module import GGcnn2Module
from states.state_machine import StateMachine
from kinematics.kinematics_solver import InverseKinematicsSolver

# Configuration
from config.config import (
    LOOP_RATE_MS, URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS,
    OPC_SERVER_URL, OPC_OBJECTS_NAME, OPC_ROBOT_NAME, GGCNN2_MODEL_PATH
)

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class RobotControlSystem:
    """Main system coordinator with modular architecture."""

    def __init__(self):
        # Initialize shared state components
        self.telemetry = Telemetry()
        self.command_bus = CommandBus()

        # Initialize shared camera manager
        camera_config = CameraConfig()
        self.camera_manager = CameraManager(camera_config)

        # Initialize OPC UA client
        opc_config = OPCConfig(
            url=OPC_SERVER_URL,
            objects_name=OPC_OBJECTS_NAME,
            robot_name=OPC_ROBOT_NAME
        )
        self.opc_client = OPCClient(
            self.command_bus, self.telemetry, opc_config)

        # Initialize kinematics solver
        self.kinematics_solver = InverseKinematicsSolver(
            URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS
        )

        # Initialize hand tracker
        hand_config = HandTrackingConfig()
        self.hand_tracker = HandTracker(
            self.telemetry, self.command_bus, self.camera_manager, hand_config
        )

        # Initialize GGCNN2 module
        self.ggcnn2_module = GGcnn2Module(
            model_path=str(GGCNN2_MODEL_PATH),
            telemetry=self.telemetry,
            command_bus=self.command_bus,
            camera_manager=self.camera_manager,
            kinematics_solver=self.kinematics_solver
        )

        # Initialize state machine
        self.state_machine = StateMachine(
            self.telemetry, self.command_bus, self.camera_manager,
            self.opc_client, self.kinematics_solver
        )

        # Control flags
        self.running = False

    def start(self):
        """Start all system components."""
        logger.info("Starting Robot Hand Tracking System...")

        try:
            # Initialize camera
            if not self.camera_manager.initialize():
                raise RuntimeError("Failed to initialize camera")

            # Start hand tracker
            self.hand_tracker.start()

            # Start OPC UA communication
            self.opc_client.start()

            # Start main control loop
            self.running = True
            self.run_control_loop()

        except Exception as e:
            logger.error(f"Failed to start system: {e}")
            self.stop()
            raise

    def stop(self):
        """Stop all system components."""
        logger.info("Stopping Robot Hand Tracking System...")
        self.running = False

        # Stop components in reverse order
        self.hand_tracker.stop()
        self.opc_client.stop()
        self.camera_manager.cleanup()

        logger.info("System stopped.")

    def run_control_loop(self):
        """Main control loop using state machine."""
        loop_start_time = 0

        try:
            logger.info("Control loop started. Press Ctrl+C to stop.")

            while self.running:
                loop_start_time = time.time()

                # Update state machine
                self.state_machine.update()

                # Maintain loop rate
                self.maintain_loop_rate(loop_start_time)

        except KeyboardInterrupt:
            logger.info("Stopping due to keyboard interrupt...")
        except Exception as e:
            logger.error(f"Error in control loop: {e}")
        finally:
            self.stop()

    def maintain_loop_rate(self, loop_start_time):
        """Maintain consistent loop rate."""
        elapsed_ms = (time.time() - loop_start_time) * 1000
        remaining_ms = LOOP_RATE_MS - elapsed_ms

        if remaining_ms > 0:
            time.sleep(remaining_ms / 1000)
        else:
            logger.warning(
                f"Loop exceeded {LOOP_RATE_MS}ms by {-remaining_ms:.1f}ms")

    def set_mode(self, mode: str):
        """Set the operating mode of the system."""
        valid_modes = ["IDLE", "TRACKING", "PICKUP", "PLACE", "GRASPING"]
        if mode in valid_modes:
            # Note: You'll need to implement state switching logic in your state machine
            logger.info(f"Mode changed to: {mode}")
        else:
            logger.error(f"Invalid mode: {mode}")

    def get_current_state(self) -> str:
        """Get the current state name."""
        return self.state_machine.get_current_state_name()

    def is_robot_connected(self) -> bool:
        """Check if robot is connected."""
        return self.opc_client.is_connected()

    def get_system_status(self) -> dict:
        """Get complete system status."""
        return {
            'running': self.running,
            'current_state': self.get_current_state(),
            'robot_connected': self.is_robot_connected(),
            'camera_ready': self.camera_manager.is_ready(),
            'opc_status': self.opc_client.get_status(),
            'system_state': self.telemetry.get_full_state()
        }


def main():
    """Main entry point for the application."""
    system = RobotControlSystem()

    try:
        # Start in tracking mode
        system.set_mode("TRACKING")
        system.start()

    except Exception as e:
        logger.error(f"System error: {e}")
        system.stop()


if __name__ == "__main__":
    main()
