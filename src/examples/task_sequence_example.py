"""
Example demonstrating the usage of pickup and placement task sequences.
This example shows how to integrate the new task sequencers into a robot control system.
"""
from kinematics.kinematics_solver import InverseKinematicsSolver
from IO_handling.opc_client import OPCClient
from camera_management.camera_manager import CameraManager
from control.command_bus import CommandBus
from control.telemetry_store import Telemetry
from states.base_state import BaseState
from states.task_orchestrator import TaskOrchestrator, TaskPhase
from states.state_machine import StateMachine
from states.context import StateContext
import logging
import time
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


class ExampleState(BaseState):
    """Example state for demonstration purposes."""

    def __init__(self, context: StateContext, name: str = "ExampleState"):
        super().__init__(context)
        self.name = name
        self.execution_count = 0

    def execute(self):
        self.execution_count += 1
        logger.info(f"{self.name} executed {self.execution_count} times")

    def is_complete(self) -> bool:
        return self.execution_count >= 3  # Complete after 3 executions


def create_mock_context() -> StateContext:
    """Create a mock context for demonstration purposes."""
    # In a real implementation, these would be actual initialized components
    telemetry = Telemetry()
    command_bus = CommandBus()

    # Mock camera manager (would be initialized in real implementation)
    camera_manager = None  # CameraManager()

    # Mock OPC client (would be initialized in real implementation)
    opc_client = None  # OPCClient()

    # Mock kinematics solver (would be initialized in real implementation)
    ik_solver = None  # InverseKinematicsSolver()

    return StateContext(
        telemetry=telemetry,
        commands=command_bus,
        camera=camera_manager,
        opc=opc_client,
        ik=ik_solver
    )


def demonstrate_task_orchestrator():
    """Demonstrate the task orchestrator functionality."""
    logger.info("=== Task Orchestrator Demonstration ===")

    # Create mock context and state machine
    context = create_mock_context()
    initial_state = ExampleState(context, "InitialState")
    state_machine = StateMachine(initial_state)

    # Create task orchestrator
    orchestrator = TaskOrchestrator(state_machine, context)

    # Demonstrate status checking
    status = orchestrator.get_status()
    logger.info(f"Initial status: {status}")

    # Start pickup task
    logger.info("\n--- Starting Pickup Task ---")
    orchestrator.start_pickup_task()

    # Simulate some steps (in real implementation, this would be in a control loop)
    for i in range(5):
        orchestrator.step()
        status = orchestrator.get_status()
        logger.info(f"Step {i+1}: {status}")
        time.sleep(0.1)

    # Start placement task
    logger.info("\n--- Starting Placement Task ---")
    orchestrator.start_placement_task()

    # Simulate more steps
    for i in range(5):
        orchestrator.step()
        status = orchestrator.get_status()
        logger.info(f"Step {i+1}: {status}")
        time.sleep(0.1)

    # Demonstrate complete task workflow
    logger.info("\n--- Starting Complete Task Workflow ---")
    orchestrator.reset()
    orchestrator.start_complete_task()

    # Simulate complete workflow
    for i in range(10):
        orchestrator.step()
        status = orchestrator.get_status()
        logger.info(f"Complete Task Step {i+1}: {status}")
        time.sleep(0.1)

        # Stop if complete
        if status['is_complete']:
            logger.info("Complete task workflow finished!")
            break

    logger.info("=== Task Orchestrator Demonstration Complete ===")


def demonstrate_individual_sequencers():
    """Demonstrate individual task sequencers."""
    logger.info("\n=== Individual Sequencer Demonstration ===")

    # Create mock context and state machine
    context = create_mock_context()
    initial_state = ExampleState(context, "InitialState")
    state_machine = StateMachine(initial_state)

    # Demonstrate pickup sequencer
    logger.info("\n--- Pickup Sequencer ---")
    from states.pickup_task_sequencer import create_pickup_sequencer

    pickup_sequencer = create_pickup_sequencer(state_machine, context)
    description = pickup_sequencer.get_sequence_description()
    logger.info("Pickup sequence:")
    for step in description:
        logger.info(f"  {step}")

    progress = pickup_sequencer.get_progress()
    logger.info(f"Pickup progress: {progress}")

    # Demonstrate placement sequencer
    logger.info("\n--- Placement Sequencer ---")
    from states.placement_task_sequencer import create_placement_sequencer

    placement_sequencer = create_placement_sequencer(state_machine, context)
    description = placement_sequencer.get_sequence_description()
    logger.info("Placement sequence:")
    for step in description:
        logger.info(f"  {step}")

    progress = placement_sequencer.get_progress()
    logger.info(f"Placement progress: {progress}")

    logger.info("=== Individual Sequencer Demonstration Complete ===")


def main():
    """Main demonstration function."""
    try:
        logger.info("Starting Task Sequence Examples")

        # Demonstrate individual sequencers
        demonstrate_individual_sequencers()

        # Demonstrate task orchestrator
        demonstrate_task_orchestrator()

        logger.info("All demonstrations completed successfully!")

    except Exception as e:
        logger.error(f"Demonstration failed: {e}")
        raise


if __name__ == "__main__":
    main()
