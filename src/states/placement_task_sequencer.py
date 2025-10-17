"""
Placement Task Sequencer - Manages the complete object placement sequence.

This sequencer orchestrates the states required to track hand, approach, and place object.

Features:
- Intelligent occlusion failure handling
- Automatic fallback when hand tracking fails
- Graceful degradation for operator safety
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
from states.unified_hand_tracking_state import UnifiedHandTrackingState
from states.handoff_fallback_state import HandoffFallbackState
from config import HANDOFF_APPROACH_POSE

logger = logging.getLogger(__name__)


class PlacementTaskSequencer(TaskSequencer):
    """
    Task sequencer for object placement operations.

    Sequence:
    1. MoveToState(pose=HANDOFF_APPROACH_POSE)
    2. UnifiedHandTrackingState()
    3. MoveToState(pose_from_telemetry='calculated_handoff_pose')
    4. GripperControlState(action='open')
    """

    def __init__(self, state_machine: StateMachine, context: StateContext):
        # Create the sequence of states for placement task
        placement_states = self._create_placement_sequence(context)

        # Initialize parent TaskSequencer
        super().__init__(state_machine, placement_states)

        self.context = context
        self.hand_tracking_failed = False  # Track if hand tracking failed
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
        # Note: This state may fail due to occlusion
        states.append(UnifiedHandTrackingState(context))

        # 3. Move to calculated handoff pose
        # Note: This step is skipped if hand tracking fails
        states.append(MoveToState(
            context, pose_from_telemetry='calculated_handoff_pose'))

        # 4. Open gripper to release object
        # Note: This step is skipped if hand tracking fails
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
            'is_complete': len(self.task_queue) == 0,
            'hand_tracking_failed': self.hand_tracking_failed
        }

    def execute(self):
        """
        Execute the next state in the sequence.

        Overridden to handle hand tracking failures and trigger fallback.
        """
        # If no tasks remain, we're done
        if not self.task_queue:
            return

        # Get the current state
        current_state = self.task_queue[0]

        # Execute the current state
        current_state.execute()

        # Check if current state is complete
        if current_state.is_complete():
            logger.info(f"State {type(current_state).__name__} completed")

            # Special handling for UnifiedHandTrackingState
            if isinstance(current_state, UnifiedHandTrackingState):
                # Check if hand tracking failed due to occlusion
                if hasattr(current_state, 'did_fail') and current_state.did_fail():
                    logger.warning("Hand tracking failed due to occlusion")
                    self.hand_tracking_failed = True

                    # Exit the current state
                    current_state.exit()
                    self.task_queue.popleft()

                    # Clear remaining placement tasks (we can't place without hand tracking)
                    logger.info(
                        f"Clearing {len(self.task_queue)} remaining placement tasks")
                    self.task_queue.clear()

                    # Add fallback state to handle the failure
                    fallback_state = HandoffFallbackState(
                        self.context,
                        failure_reason="Hand occlusion detected during placement"
                    )
                    self.task_queue.append(fallback_state)

                    # Enter the fallback state immediately
                    fallback_state.enter()
                    logger.info("Transitioned to HandoffFallbackState")
                    return

            # Normal completion - exit and move to next state
            current_state.exit()
            self.task_queue.popleft()

            # If there's a next state, enter it
            if self.task_queue:
                next_state = self.task_queue[0]
                next_state.enter()
                logger.info(f"Transitioned to {type(next_state).__name__}")


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
