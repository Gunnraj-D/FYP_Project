"""
Placement Task Sequencer - Manages the complete object placement sequence.

This sequencer orchestrates the states required to track hand, approach, and place object.

Sequence:
1. HumanHandoffApproachState - Collision-aware approach to hand
2. UnifiedHandTrackingState - Track hand for precise placement
3. If occluded -> HandoffFallbackState (operator-controlled placement)
4. Move down to place object
5. Open gripper to release
6. Move back up to safe position

Features:
- Intelligent occlusion failure handling
- Automatic fallback when hand tracking fails
- Graceful degradation for operator safety
"""
import logging
from typing import List
from collections import deque
import numpy as np

from states.base_state import BaseState
from states.context import StateContext
from states.state_machine import StateMachine
from states.task_sequencer import TaskSequencer
from states.move_to_state import MoveToState
from states.gripper_state import GripperControlState
from states.unified_hand_tracking_state import UnifiedHandTrackingState
from states.handoff_fallback_state import HandoffFallbackState
from states.human_handoff_approach_state import HumanHandoffApproachState
from config import HANDOFF_APPROACH_POSE, PLACE_APPROACH_DISTANCE, PLACE_RELEASE_DISTANCE

logger = logging.getLogger(__name__)


class PlacementTaskSequencer(TaskSequencer):
    """
    Task sequencer for object placement operations.

    Sequence:
    1. HumanHandoffApproachState - Collision-aware approach to hand
    2. UnifiedHandTrackingState - Track hand for precise placement
    3. If occluded -> HandoffFallbackState (operator-controlled placement)
    4. Move down to place object
    5. Open gripper to release
    6. Move back up to safe position
    """

    def __init__(self, state_machine: StateMachine, context: StateContext):
        # Create the sequence of states for placement task
        placement_states = self._create_placement_sequence(context)

        # Initialize parent TaskSequencer
        super().__init__(state_machine, placement_states)

        self.context = context
        self.hand_tracking_failed = False  # Track if hand tracking failed
        self.used_fallback = False  # Track if fallback was used
        self.placement_position = None  # Store final placement position
        logger.info("PlacementTaskSequencer initialized with {} states".format(
            len(placement_states)))

    def _create_placement_sequence(self, context: StateContext) -> List[BaseState]:
        """
        Create the initial sequence of states for the placement task.

        Args:
            context: State context containing shared resources

        Returns:
            List of states in execution order

        Note: Steps 1-2 only. Steps 3-6 are added dynamically after 
        hand tracking or fallback completes.
        """
        states = []

        # 1. Human-aware approach to hand position (collision-aware)
        states.append(HumanHandoffApproachState(
            context=context,
            approach_offset=[0.0, 0.0, 0.15],  # 15cm above hand
            position_threshold=0.05,
            hand_joint_name='RIGHT_WRIST'
        ))

        # 2. Track hand for precise placement position
        # This may fail due to occlusion -> triggers fallback in execute()
        states.append(UnifiedHandTrackingState(context))

        # Steps 3-6 are added dynamically after step 2 completes:
        # 3. Move down to place (added in execute() after tracking/fallback)
        # 4. Open gripper (added in execute())
        # 5. Move back up (added in execute())

        logger.info(
            f"Created initial placement sequence with {len(states)} states")
        return states

    def get_sequence_description(self) -> List[str]:
        """Get a human-readable description of the placement sequence."""
        desc = [
            "1. Collision-aware approach to operator's hand",
            "2. Track hand for precise placement (or fallback if occluded)",
            "3. Move down to placement position",
            "4. Open gripper to release object",
            "5. Move back up to safe position"
        ]

        if self.used_fallback:
            desc[1] = "2. [FALLBACK] Operator-controlled placement"

        return desc

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

    def _add_placement_completion_states(self):
        """
        Add the final placement states (move down, open gripper, move up).
        Called after hand tracking or fallback completes successfully.
        """
        # Get current TCP position
        current_joints = self.context.telemetry.get_current_joints()
        tcp_matrix, tcp_pose = self.context.ik.tcp_from_joints(current_joints)
        current_position = np.array(tcp_pose[:3])

        # 3. Move down to place object (lower Z by placement distance)
        place_position = current_position.copy()
        place_position[2] -= PLACE_RELEASE_DISTANCE  # Move down 5cm to place

        logger.info(
            f"Adding placement move: from {current_position} to {place_position}")
        self.task_queue.append(MoveToState(
            self.context,
            target_location=tuple(place_position)
        ))

        # 4. Open gripper to release object
        self.task_queue.append(GripperControlState(
            self.context,
            action='open'
        ))

        # 5. Move back up to safe position
        retreat_position = place_position.copy()
        retreat_position[2] += PLACE_APPROACH_DISTANCE  # Move up 15cm

        logger.info(
            f"Adding retreat move: from {place_position} to {retreat_position}")
        self.task_queue.append(MoveToState(
            self.context,
            target_location=tuple(retreat_position)
        ))

        logger.info(
            "Added 3 placement completion states (move down, open gripper, move up)")

    def execute(self):
        """
        Execute the next state in the sequence.

        Handles:
        - Normal state transitions
        - UnifiedHandTrackingState success -> add placement states
        - UnifiedHandTrackingState failure -> insert fallback, then add placement states
        - HandoffFallbackState success -> add placement states
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
                    logger.warning(
                        "⚠️ Hand tracking failed due to occlusion - switching to fallback")
                    self.hand_tracking_failed = True
                    self.used_fallback = True

                    # Exit the current state
                    current_state.exit()
                    self.task_queue.popleft()

                    # Insert fallback state
                    fallback_state = HandoffFallbackState(
                        self.context,
                        failure_reason="Hand occlusion detected - operator will guide placement"
                    )
                    self.task_queue.appendleft(fallback_state)

                    # Enter the fallback state immediately
                    fallback_state.enter()
                    logger.info("✓ Transitioned to HandoffFallbackState")
                    return
                else:
                    # Hand tracking succeeded - add placement completion states
                    logger.info(
                        "✓ Hand tracking succeeded - adding placement completion states")
                    current_state.exit()
                    self.task_queue.popleft()

                    # Add the remaining states (move down, open gripper, move up)
                    self._add_placement_completion_states()

                    # Enter next state if available
                    if self.task_queue:
                        next_state = self.task_queue[0]
                        next_state.enter()
                        logger.info(
                            f"Transitioned to {type(next_state).__name__}")
                    return

            # Special handling for HandoffFallbackState
            if isinstance(current_state, HandoffFallbackState):
                # Fallback completed - add placement completion states
                logger.info(
                    "✓ Fallback completed - adding placement completion states")
                current_state.exit()
                self.task_queue.popleft()

                # Add the remaining states (move down, open gripper, move up)
                self._add_placement_completion_states()

                # Enter next state if available
                if self.task_queue:
                    next_state = self.task_queue[0]
                    next_state.enter()
                    logger.info(f"Transitioned to {type(next_state).__name__}")
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
