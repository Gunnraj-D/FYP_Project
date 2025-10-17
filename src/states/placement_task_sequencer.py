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
    Orchestrates the sequence of states for a human-assisted object placement task.

    This sequencer manages the flow from approaching the operator's hand to placing the
    object, including a fallback mechanism if visual hand tracking is lost.

    Sequence:
    1.  **HumanHandoffApproachState**: Safely moves the robot arm towards the operator's hand.
    2.  **UnifiedHandTrackingState**: Tracks the hand to determine the precise placement target.
        - **On Success**: Proceeds to the final placement steps.
        - **On Failure (e.g., occlusion)**: Transitions to the fallback state.
    3.  **HandoffFallbackState (Optional)**: If tracking fails, allows operator-guided control.
    4.  **Final Placement**: Once a target is set (by tracking or fallback), it executes:
        a. Move down to the placement surface.
        b. Open the gripper to release the object.
        c. Retreat to a safe position.
    """

    def __init__(self, state_machine: StateMachine, context: StateContext):
        """
        Initializes the sequencer with the first two states of the placement task.
        """
        self.context = context
        # Flag to indicate if visual tracking failed.
        self.hand_tracking_failed = False
        # Flag to indicate if the fallback state was used.
        self.used_fallback = False

        # The full sequence is built dynamically. Start with the first two states.
        initial_states = self._create_initial_sequence()

        super().__init__(state_machine, initial_states)
        logger.info(
            f"PlacementTaskSequencer initialized with {len(initial_states)} initial states.")

    def _create_initial_sequence(self) -> List[BaseState]:
        """Creates the initial part of the state sequence."""
        return [
            # 1. Approach the operator's hand.
            HumanHandoffApproachState(
                context=self.context,
                approach_offset=[0.0, 0.0, 0.15],  # 15cm above hand
                position_threshold=0.05,
                hand_joint_name='RIGHT_WRIST'
            ),
            # 2. Track the hand to find the placement spot.
            UnifiedHandTrackingState(self.context)
        ]

    def queue_next_task(self):
        """
        Override default to inject fallback/final steps based on the state that just completed.

        This method is called by the StateMachine when the CURRENT state reports is_complete().
        We inspect that completed state to decide what to enqueue next before transitioning.
        """
        # The state that just completed (StateMachine has not transitioned yet)
        completed_state = self.state_machine.current_state

        # Decide dynamic additions based on which state completed
        should_add_final_steps = False

        if isinstance(completed_state, UnifiedHandTrackingState):
            if hasattr(completed_state, 'did_fail') and completed_state.did_fail():
                self.hand_tracking_failed = True
                self.used_fallback = True
                failure_reason = (
                    "Hand tracking timeout. Operator will guide placement."
                    if getattr(completed_state, 'timed_out', False)
                    else "Hand occlusion detected. Operator will guide placement."
                )
                # Run fallback next
                fallback_state = HandoffFallbackState(
                    self.context, failure_reason=failure_reason)
                self.task_queue.appendleft(fallback_state)
            else:
                # Success path: add final placement actions
                should_add_final_steps = True

        elif isinstance(completed_state, HandoffFallbackState):
            # After fallback, proceed to final placement actions
            should_add_final_steps = True

        # Add final steps if flagged
        if should_add_final_steps:
            self._add_placement_completion_states()

        # Transition to next state if available; else exit the completed state
        if self.task_queue:
            next_state = self.task_queue.popleft()
            self.state_machine.transition(next_state)
        else:
            # No further states: ensure the completed state is properly exited
            try:
                completed_state.exit()
            except Exception:
                logger.exception("Error while exiting final state")
            logger.info("Task complete — no more states in sequence.")

    def execute(self):
        """
        Executes the current state and handles transitions upon its completion.

        This method contains the core logic for the sequence, including the
        conditional transition to the fallback state and the dynamic addition
        of the final placement steps.
        """
        if not self.task_queue:
            return  # Sequence is complete.

        current_state = self.task_queue[0]
        current_state.execute()

        # If the state is still running, wait for the next execution cycle.
        if not current_state.is_complete():
            return

        # --- The current state has just finished. Handle the transition. ---
        logger.info(f"State '{type(current_state).__name__}' has completed.")
        current_state.exit()
        completed_state = self.task_queue.popleft()

        should_add_final_steps = False

        # --- Logic for specific state completions ---

        # Case 1: UnifiedHandTrackingState just finished.
        if isinstance(completed_state, UnifiedHandTrackingState):
            # Check if it failed (e.g., due to occlusion).
            if hasattr(completed_state, 'did_fail') and completed_state.did_fail():
                # Decide reason: timeout vs occlusion
                failure_reason = "Hand tracking timeout. Operator will guide placement." if getattr(
                    completed_state, 'timed_out', False) else "Hand occlusion detected. Operator will guide placement."
                logger.warning(
                    f"Hand tracking failure detected: {failure_reason}")
                self.hand_tracking_failed = True
                self.used_fallback = True
                # Insert the fallback state at the front of the queue to run next.
                fallback_state = HandoffFallbackState(
                    self.context,
                    failure_reason=failure_reason
                )
                self.task_queue.appendleft(fallback_state)
            else:
                # It succeeded, so we're ready for the final placement steps.
                logger.info(
                    "Hand tracking succeeded. Queuing final placement actions.")
                should_add_final_steps = True

        # Case 2: HandoffFallbackState just finished.
        elif isinstance(completed_state, HandoffFallbackState):
            # The operator has finished positioning, so we're ready for final placement.
            logger.info(
                "Operator-controlled fallback completed. Queuing final placement actions.")
            should_add_final_steps = True

        # --- Add final placement states if triggered by a successful hand-off ---
        if should_add_final_steps:
            self._add_placement_completion_states()

        # --- Transition to the next state in the queue ---
        if self.task_queue:
            next_state = self.task_queue[0]
            logger.info(
                f"Transitioning to next state: '{type(next_state).__name__}'.")
            next_state.enter()
        else:
            logger.info("✅ Placement sequence has completed successfully.")

    def _add_placement_completion_states(self):
        """
        Adds the final states to the queue: move down, open gripper, and move up.
        """
        logger.info("🔧 Adding final placement states to the queue...")
        current_joints = self.context.telemetry.get_current_joints()
        if not current_joints:
            logger.error(
                "Cannot add completion states: current joint data is unavailable.")
            return

        _, tcp_pose = self.context.ik.tcp_from_joints(current_joints)
        current_position = np.array(tcp_pose[:3])

        # Define the three final positions/actions.
        place_position = current_position - \
            np.array([0, 0, PLACE_RELEASE_DISTANCE])
        retreat_position = current_position + \
            np.array([0, 0, PLACE_APPROACH_DISTANCE - PLACE_RELEASE_DISTANCE])

        # Create the state sequence.
        final_states = [
            # 1. Move down to place the object.
            MoveToState(self.context, target_location=tuple(place_position)),
            # 2. Open the gripper.
            GripperControlState(self.context, action='open'),
            # 3. Move back up to a safe retreat position.
            MoveToState(self.context, target_location=tuple(retreat_position))
        ]

        # Add the states to the end of the queue.
        self.task_queue.extend(final_states)
        logger.info(f"Added {len(final_states)} final states to the queue.")

    def get_sequence_description(self) -> List[str]:
        """Returns a human-readable list of the sequence steps."""
        desc = [
            "1. Approach operator's hand",
            "2. Track hand for precise placement",
            "3. Move down to placement position",
            "4. Open gripper to release object",
            "5. Move up to a safe position"
        ]
        if self.used_fallback:
            desc[1] = "2. [FALLBACK] Guide robot with operator control"
        return desc

    def get_progress(self) -> dict:
        """Returns a dictionary with progress information."""
        total_steps = len(self.get_sequence_description())
        # current_step is total steps minus remaining tasks, but capped at total_steps.
        current_step_num = min(
            total_steps, total_steps - len(self.task_queue) + 1)
        is_complete = not self.task_queue

        return {
            'total_steps': total_steps,
            'current_step': current_step_num if not is_complete else total_steps,
            'progress_percent': (current_step_num / total_steps) * 100 if total_steps > 0 else 100,
            'is_complete': is_complete,
            'used_fallback': self.used_fallback
        }


def create_placement_sequencer(state_machine: StateMachine, context: StateContext) -> PlacementTaskSequencer:
    """Factory function to create a PlacementTaskSequencer."""
    return PlacementTaskSequencer(state_machine, context)
