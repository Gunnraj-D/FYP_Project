"""
Task Orchestrator - Coordinates pickup and placement task sequences.
This orchestrator manages the complete object manipulation workflow.
"""
import logging
from enum import Enum
from typing import Optional

from states.base_state import BaseState
from states.context import StateContext
from states.state_machine import StateMachine
from states.pickup_task_sequencer import PickupTaskSequencer, create_pickup_sequencer
from states.placement_task_sequencer import PlacementTaskSequencer, create_placement_sequencer

logger = logging.getLogger(__name__)


class TaskPhase(Enum):
    """Enumeration of task phases."""
    IDLE = "idle"
    PICKUP = "pickup"
    PLACEMENT = "placement"
    COMPLETE = "complete"
    ERROR = "error"


class TaskOrchestrator:
    """
    Orchestrator for managing complete object manipulation tasks.

    This orchestrator coordinates between pickup and placement sequences,
    managing the overall workflow and state transitions.
    """

    def __init__(self, state_machine: StateMachine, context: StateContext):
        self.state_machine = state_machine
        self.context = context
        self.current_phase = TaskPhase.IDLE
        self.pickup_sequencer: Optional[PickupTaskSequencer] = None
        self.placement_sequencer: Optional[PlacementTaskSequencer] = None
        self.error_message = ""

        logger.info("TaskOrchestrator initialized")

    def start_pickup_task(self):
        """Start the pickup task sequence."""
        try:
            logger.info("Starting pickup task sequence")
            self.current_phase = TaskPhase.PICKUP
            self.error_message = ""

            # Create and initialize pickup sequencer
            self.pickup_sequencer = create_pickup_sequencer(
                self.state_machine, self.context)

            # Set up state completion callback
            self.state_machine.on_state_completion = self._on_pickup_state_completion

            logger.info("Pickup task sequence started")

        except Exception as e:
            logger.error(f"Failed to start pickup task: {e}")
            self.current_phase = TaskPhase.ERROR
            self.error_message = f"Pickup task initialization failed: {e}"

    def start_placement_task(self):
        """Start the placement task sequence."""
        try:
            logger.info("Starting placement task sequence")
            self.current_phase = TaskPhase.PLACEMENT
            self.error_message = ""

            # Create and initialize placement sequencer
            self.placement_sequencer = create_placement_sequencer(
                self.state_machine, self.context)

            # Set up state completion callback
            self.state_machine.on_state_completion = self._on_placement_state_completion

            logger.info("Placement task sequence started")

        except Exception as e:
            logger.error(f"Failed to start placement task: {e}")
            self.current_phase = TaskPhase.ERROR
            self.error_message = f"Placement task initialization failed: {e}"

    def start_complete_task(self):
        """Start the complete pickup and placement workflow."""
        try:
            logger.info("Starting complete object manipulation task")
            self.start_pickup_task()

        except Exception as e:
            logger.error(f"Failed to start complete task: {e}")
            self.current_phase = TaskPhase.ERROR
            self.error_message = f"Complete task initialization failed: {e}"

    def step(self):
        """Execute one step of the current task."""
        if self.current_phase == TaskPhase.IDLE:
            return
        elif self.current_phase == TaskPhase.PICKUP and self.pickup_sequencer:
            self.pickup_sequencer.step()
        elif self.current_phase == TaskPhase.PLACEMENT and self.placement_sequencer:
            self.placement_sequencer.step()
        elif self.current_phase == TaskPhase.ERROR:
            logger.warning(f"Task in error state: {self.error_message}")

    def _on_pickup_state_completion(self):
        """Callback when pickup state completes."""
        try:
            if self.pickup_sequencer and len(self.pickup_sequencer.task_queue) == 0:
                logger.info("Pickup task sequence completed successfully")
                # Automatically transition to placement task
                self.start_placement_task()
            else:
                # Continue with next state in pickup sequence
                self.pickup_sequencer.queue_next_task()

        except Exception as e:
            logger.error(f"Error in pickup state completion: {e}")
            self.current_phase = TaskPhase.ERROR
            self.error_message = f"Pickup state completion error: {e}"

    def _on_placement_state_completion(self):
        """Callback when placement state completes."""
        try:
            if self.placement_sequencer and len(self.placement_sequencer.task_queue) == 0:
                logger.info("Placement task sequence completed successfully")
                self.current_phase = TaskPhase.COMPLETE
            else:
                # Continue with next state in placement sequence
                self.placement_sequencer.queue_next_task()

        except Exception as e:
            logger.error(f"Error in placement state completion: {e}")
            self.current_phase = TaskPhase.ERROR
            self.error_message = f"Placement state completion error: {e}"

    def get_status(self) -> dict:
        """Get current status of the orchestrator."""
        status = {
            'current_phase': self.current_phase.value,
            'error_message': self.error_message,
            'is_complete': self.current_phase == TaskPhase.COMPLETE,
            'has_error': self.current_phase == TaskPhase.ERROR
        }

        # Add phase-specific status
        if self.current_phase == TaskPhase.PICKUP and self.pickup_sequencer:
            status['pickup_progress'] = self.pickup_sequencer.get_progress()
        elif self.current_phase == TaskPhase.PLACEMENT and self.placement_sequencer:
            status['placement_progress'] = self.placement_sequencer.get_progress()

        return status

    def reset(self):
        """Reset the orchestrator to idle state."""
        logger.info("Resetting task orchestrator")
        self.current_phase = TaskPhase.IDLE
        self.pickup_sequencer = None
        self.placement_sequencer = None
        self.error_message = ""
        self.state_machine.on_state_completion = None

    def get_current_task_description(self) -> str:
        """Get description of current task phase."""
        if self.current_phase == TaskPhase.PICKUP:
            return "Object pickup sequence"
        elif self.current_phase == TaskPhase.PLACEMENT:
            return "Object placement sequence"
        elif self.current_phase == TaskPhase.COMPLETE:
            return "Task completed successfully"
        elif self.current_phase == TaskPhase.ERROR:
            return f"Error: {self.error_message}"
        else:
            return "Idle - no active task"
