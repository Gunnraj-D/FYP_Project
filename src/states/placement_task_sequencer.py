"""
Placement Task Sequencer - Manages the complete object placement sequence.
This sequencer orchestrates the states required to track hand, approach, and place object.
"""
import logging
from typing import List
from collections import deque

from states.base_state import BaseState
from states.context import StateContext
from states.state_machine import StateMachine
from states.task_sequencer import TaskSequencer
from states.move_to_state import MoveToState
from states.gripper_state import GripperControlState
from states.hand_tracking_state import HandTrackingState
from config.config import HANDOFF_APPROACH_POSE

logger = logging.getLogger(__name__)


class PlacementTaskSequencer(TaskSequencer):
    """
    Task sequencer for object placement operations.

    Sequence:
    1. MoveToState(pose=HANDOFF_APPROACH_POSE)
    2. HandTrackingState()
    3. MoveToState(pose_from_telemetry='calculated_handoff_pose')
    4. GripperControlState(action='open')
    """

    def __init__(self, state_machine: StateMachine, context: StateContext):
        # Create the sequence of states for placement task
        placement_states = self._create_placement_sequence(context)

        # Initialize parent TaskSequencer
        super().__init__(state_machine, placement_states)

        self.context = context
        logger.info("PlacementTaskSequencer initialized with {} states".format(
            len(placement_states)))

    def _create_placement_sequence(self, context: StateContext) -> List[BaseState]:
        """
        Create the sequence of states for the placement task.

        Args:
            context: State context containing shared resources

        Returns:
            List of states in execution order
        """
        states = []

        # 1. Move to handoff approach position
        # Extract XYZ coordinates
        handoff_location = tuple(HANDOFF_APPROACH_POSE[:3])
        states.append(MoveToState(context, target_location=handoff_location))

        # 2. Track hand and calculate placement pose
        states.append(HandTrackingState(context))

        # 3. Move to calculated handoff pose
        states.append(MoveToState(
            context, pose_from_telemetry='calculated_handoff_pose'))

        # 4. Open gripper to release object
        states.append(GripperControlState(context, action='open'))

        logger.info(
            "Created placement sequence with {} states".format(len(states)))
        return states

    def get_sequence_description(self) -> List[str]:
        """Get a human-readable description of the placement sequence."""
        return [
            "1. Move to handoff approach position",
            "2. Track operator's hand and calculate placement pose",
            "3. Move to calculated handoff pose",
            "4. Open gripper to release object"
        ]

    def get_current_step(self) -> int:
        """Get the current step number (1-based)."""
        return len(self.get_sequence_description()) - len(self.task_queue)

    def get_progress(self) -> dict:
        """Get progress information about the placement sequence."""
        total_steps = len(self.get_sequence_description())
        current_step = self.get_current_step()

        return {
            'total_steps': total_steps,
            'current_step': current_step,
            'progress_percent': (current_step / total_steps) * 100,
            'remaining_steps': len(self.task_queue),
            'is_complete': len(self.task_queue) == 0
        }


def create_placement_sequencer(state_machine: StateMachine, context: StateContext) -> PlacementTaskSequencer:
    """
    Factory function to create a placement task sequencer.

    Args:
        state_machine: State machine to manage state transitions
        context: Shared state context

    Returns:
        Initialized PlacementTaskSequencer
    """
    return PlacementTaskSequencer(state_machine, context)
