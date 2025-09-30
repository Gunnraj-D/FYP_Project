"""
Integrated Robot Control System - Complete system integration.
This system properly integrates all components using the current state machine
and task orchestration implementations.
"""
import time
import logging
from typing import Optional, Dict, Any
from enum import Enum

# Core modules
from control.telemetry_store import Telemetry
from control.command_bus import CommandBus
from camera_management.camera_manager import CameraManager, CameraConfig
from IO_handling.opc_client_factory import OPCClientFactory, OPCConfig
from hand_detection.hand_detection_module import HandTracker, HandTrackingConfig
from object_detection.ggcnn2_module import GGcnn2Module
from kinematics.kinematics_solver import InverseKinematicsSolver
from states.state_machine import StateMachine
from states.context import StateContext
from states.task_orchestrator import TaskOrchestrator, TaskPhase
from states.base_state import BaseState

# Configuration
from config.config import (
    LOOP_RATE_MS, URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS,
    OPC_SERVER_URL, OPC_OBJECTS_NAME, GGCNN2_MODEL_PATH, ROBOT_ID, get_robot_name
)

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class SystemMode(Enum):
    """System operating modes."""
    IDLE = "idle"
    TRACKING = "tracking"
    PICKUP = "pickup"
    PLACEMENT = "placement"
    COMPLETE_TASK = "complete_task"
    ERROR = "error"


class IdleState(BaseState):
    """Idle state - system ready but not executing tasks."""

    def __init__(self, context: StateContext):
        super().__init__(context)
        self.enter_time = 0.0

    def enter(self):
        logger.info("Entering IDLE state")
        self.enter_time = time.time()

    def execute(self):
        # In idle state, we just monitor system status
        pass

    def is_complete(self) -> bool:
        # Idle state never completes automatically
        return False

    def exit(self):
        logger.info("Exiting IDLE state")


class IntegratedRobotControlSystem:
    """
    Integrated robot control system with proper state machine integration.

    This system:
    1. Initializes all components with proper configuration
    2. Uses the current state machine implementation
    3. Integrates task orchestration for complete workflows
    4. Provides unified control interface
    """

    def __init__(self, opc_mode: str = None):
        # Initialize shared state components
        self.telemetry = Telemetry()
        self.command_bus = CommandBus()

        # Initialize shared camera manager
        camera_config = CameraConfig()
        self.camera_manager = CameraManager(camera_config)

        # Initialize OPC UA client using factory
        self.opc_client = OPCClientFactory.create_client(
            self.command_bus, self.telemetry, None, opc_mode)

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

        # Create shared context
        self.context = StateContext(
            telemetry=self.telemetry,
            commands=self.command_bus,
            camera=self.camera_manager,
            opc=self.opc_client,
            ik=self.kinematics_solver,
            hand_tracker=self.hand_tracker
        )

        # Initialize state machine with idle state
        self.idle_state = IdleState(self.context)
        self.state_machine = StateMachine(self.idle_state)

        # Initialize task orchestrator
        self.task_orchestrator = TaskOrchestrator(
            self.state_machine, self.context)

        # System state
        self.running = False
        self.current_mode = SystemMode.IDLE
        self.error_message = ""

        logger.info("IntegratedRobotControlSystem initialized successfully")

    def start(self):
        """Start all system components."""
        logger.info("Starting Integrated Robot Control System...")

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
            self.error_message = str(e)
            self.current_mode = SystemMode.ERROR
            self.stop()
            raise

    def stop(self):
        """Stop all system components."""
        logger.info("Stopping Integrated Robot Control System...")
        self.running = False

        # Stop components in reverse order with error handling
        try:
            if hasattr(self, 'hand_tracker') and self.hand_tracker:
                self.hand_tracker.stop()
        except Exception as e:
            logger.warning(f"Error stopping hand tracker: {e}")

        try:
            if hasattr(self, 'opc_client') and self.opc_client:
                self.opc_client.stop()
        except Exception as e:
            logger.warning(f"Error stopping OPC client: {e}")

        try:
            if hasattr(self, 'camera_manager') and self.camera_manager:
                self.camera_manager.cleanup()
        except Exception as e:
            logger.warning(f"Error cleaning up camera manager: {e}")

        logger.info("System stopped.")

    def run_control_loop(self):
        """Main control loop using integrated state machine."""
        loop_start_time = 0.0

        try:
            logger.info("Control loop started. Press Ctrl+C to stop.")

            while self.running:
                loop_start_time = time.time()

                # Update state machine
                self.state_machine.step()

                # Update task orchestrator if active
                if self.current_mode in [SystemMode.PICKUP, SystemMode.PLACEMENT, SystemMode.COMPLETE_TASK]:
                    self.task_orchestrator.step()

                # Maintain loop rate
                self.maintain_loop_rate(loop_start_time)

        except KeyboardInterrupt:
            logger.info("Stopping due to keyboard interrupt...")
        except Exception as e:
            logger.error(f"Error in control loop: {e}")
            self.error_message = str(e)
            self.current_mode = SystemMode.ERROR
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

    def set_mode(self, mode: SystemMode):
        """Set the operating mode of the system."""
        logger.info(f"Setting system mode to: {mode.value}")

        try:
            if mode == SystemMode.IDLE:
                self._set_idle_mode()
            elif mode == SystemMode.TRACKING:
                self._set_tracking_mode()
            elif mode == SystemMode.PICKUP:
                self._set_pickup_mode()
            elif mode == SystemMode.PLACEMENT:
                self._set_placement_mode()
            elif mode == SystemMode.COMPLETE_TASK:
                self._set_complete_task_mode()
            else:
                raise ValueError(f"Invalid mode: {mode}")

            self.current_mode = mode
            self.error_message = ""

        except Exception as e:
            logger.error(f"Failed to set mode {mode.value}: {e}")
            self.error_message = str(e)
            self.current_mode = SystemMode.ERROR

    def _set_idle_mode(self):
        """Set system to idle mode."""
        # Reset task orchestrator
        self.task_orchestrator.reset()

        # Transition to idle state
        self.state_machine.transition(self.idle_state)

    def _set_tracking_mode(self):
        """Set system to hand tracking mode."""
        # Ensure hand tracker is running
        if not self.hand_tracker.is_running:
            self.hand_tracker.start()

        # Stay in idle state but with hand tracking active
        self.state_machine.transition(self.idle_state)

    def _set_pickup_mode(self):
        """Set system to pickup mode."""
        logger.info("Starting pickup task sequence")
        self.task_orchestrator.start_pickup_task()

    def _set_placement_mode(self):
        """Set system to placement mode."""
        logger.info("Starting placement task sequence")
        self.task_orchestrator.start_placement_task()

    def _set_complete_task_mode(self):
        """Set system to complete task mode."""
        logger.info("Starting complete object manipulation task")
        self.task_orchestrator.start_complete_task()

    def get_current_state(self) -> str:
        """Get the current state name."""
        return self.state_machine.get_current_state_name()

    def get_current_mode(self) -> SystemMode:
        """Get the current system mode."""
        return self.current_mode

    def is_robot_connected(self) -> bool:
        """Check if robot is connected."""
        return self.opc_client.is_connected()

    def get_system_status(self) -> Dict[str, Any]:
        """Get complete system status."""
        status = {
            'running': self.running,
            'current_mode': self.current_mode.value,
            'current_state': self.get_current_state(),
            'robot_connected': self.is_robot_connected(),
            'camera_ready': self.camera_manager.is_ready(),
            'opc_status': self.opc_client.get_status(),
            'hand_tracker_running': self.hand_tracker.is_running if hasattr(self.hand_tracker, 'is_running') else False,
            'error_message': self.error_message,
            'system_state': self.telemetry.get_full_state()
        }

        # Add task orchestrator status if active
        if self.current_mode in [SystemMode.PICKUP, SystemMode.PLACEMENT, SystemMode.COMPLETE_TASK]:
            status['task_orchestrator'] = self.task_orchestrator.get_status()

        return status

    def emergency_stop(self):
        """Emergency stop the system."""
        logger.warning("EMERGENCY STOP ACTIVATED")

        # Send emergency stop command
        from control.command_bus import EmergencyStop
        self.command_bus.send(EmergencyStop(active=True))

        # Set error mode
        self.current_mode = SystemMode.ERROR
        self.error_message = "Emergency stop activated"

        # Stop task orchestrator
        self.task_orchestrator.reset()

    def reset_system(self):
        """Reset the system to idle state."""
        logger.info("Resetting system to idle state")

        # Reset task orchestrator
        self.task_orchestrator.reset()

        # Clear error state
        self.error_message = ""

        # Set to idle mode
        self.current_mode = SystemMode.IDLE

        # Transition to idle state
        self.state_machine.transition(self.idle_state)


def main():
    """Main entry point for the integrated system."""
    system = IntegratedRobotControlSystem()

    try:
        # Start the system
        system.start()

    except Exception as e:
        logger.error(f"System error: {e}")
        system.stop()


if __name__ == "__main__":
    main()
