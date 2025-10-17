"""
Simple Placement Task Sequencer - Hardcoded, straightforward implementation.
No complex callbacks or dynamic state injection - just a linear sequence.
"""
import logging
import numpy as np
from typing import List, Optional
from collections import deque

from states.base_state import BaseState
from states.context import StateContext
from states.state_machine import StateMachine
from states.move_to_state import MoveToState
from states.gripper_state import GripperControlState
from states.unified_hand_tracking_state import UnifiedHandTrackingState
from states.handoff_fallback_state import HandoffFallbackState
from states.human_handoff_approach_state import HumanHandoffApproachState
from config import PLACE_APPROACH_DISTANCE, PLACE_RELEASE_DISTANCE

logger = logging.getLogger(__name__)


class SimplePlacementSequencer:
    """
    Dead simple placement sequencer with explicit state management.
    No inheritance, no complex callbacks - just straightforward execution.
    """

    def __init__(self, state_machine: StateMachine, context: StateContext):
        self.state_machine = state_machine
        self.context = context

        # Sequence control
        self.current_step = 0
        self.total_steps = 0
        self.is_complete_flag = False

        # State tracking
        self.hand_tracking_state: Optional[UnifiedHandTrackingState] = None
        self.fallback_state: Optional[HandoffFallbackState] = None
        self.used_fallback = False

        # Create states upfront
        self.approach_state = HumanHandoffApproachState(
            context=context,
            approach_offset=[0.0, 0.0, 0.15],  # 15cm above hand
            position_threshold=0.05,
            hand_joint_name='RIGHT_WRIST'
        )

        logger.info("SimplePlacementSequencer initialized")

    def start(self):
        """Start the sequence by entering the first state."""
        logger.info("🚀 Starting simple placement sequence")
        self.current_step = 1
        self.total_steps = 5  # approach, track, move down, open, move up

        # Start with approach state
        logger.info("Step 1/5: Approaching human hand")
        self.state_machine.current_state = self.approach_state
        self.approach_state.enter()

    def step(self):
        """Execute one step of the sequencer."""
        if self.is_complete_flag:
            return

        # Execute current state
        current_state = self.state_machine.current_state
        if current_state is None:
            logger.error("No current state!")
            return

        current_state.execute()

        # Check if current state is complete
        if not current_state.is_complete():
            return

        # State completed - transition to next
        self._handle_state_completion(current_state)

    def _handle_state_completion(self, completed_state: BaseState):
        """Handle completion of a state and transition to next."""
        logger.info(f"✅ Completed: {type(completed_state).__name__}")

        # Exit the completed state
        try:
            completed_state.exit()
        except Exception as e:
            logger.error(f"Error exiting state: {e}")

        # Determine next state based on what just completed
        if isinstance(completed_state, HumanHandoffApproachState):
            # Step 1 done -> Start hand tracking
            self.current_step = 2
            logger.info("Step 2/5: Tracking hand for placement position")
            self.hand_tracking_state = UnifiedHandTrackingState(self.context)
            self.state_machine.current_state = self.hand_tracking_state
            self.hand_tracking_state.enter()

        elif isinstance(completed_state, UnifiedHandTrackingState):
            # Step 2 done -> Check if it failed
            if hasattr(completed_state, 'did_fail') and completed_state.did_fail():
                # Failed - go to fallback
                failure_reason = (
                    "Hand tracking timeout. Please guide the robot to placement position."
                    if getattr(completed_state, 'timed_out', False)
                    else "Hand occluded. Please guide the robot to placement position."
                )
                logger.warning(f"⚠️ Hand tracking failed: {failure_reason}")
                logger.info("Step 2b/5: Using operator fallback mode")
                self.used_fallback = True
                self.fallback_state = HandoffFallbackState(
                    self.context, failure_reason=failure_reason)
                self.state_machine.current_state = self.fallback_state
                self.fallback_state.enter()
            else:
                # Success - proceed to placement
                logger.info("✅ Hand tracking succeeded")
                self._start_final_placement()

        elif isinstance(completed_state, HandoffFallbackState):
            # Fallback done -> proceed to placement
            logger.info("✅ Operator guidance complete")
            self._start_final_placement()

        elif isinstance(completed_state, MoveToState):
            # One of the final movement states completed
            self._continue_final_placement()

        elif isinstance(completed_state, GripperControlState):
            # Gripper state completed
            self._continue_final_placement()

    def _start_final_placement(self):
        """Start the final placement sequence (move down, open gripper, move up)."""
        logger.info("🔧 Starting final placement sequence")
        self.current_step = 3

        # Get current position
        current_joints = self.context.telemetry.get_current_joints()
        if current_joints is None or (isinstance(current_joints, np.ndarray) and current_joints.size == 0):
            logger.error("Cannot get current joints for placement!")
            self.is_complete_flag = True
            return

        _, tcp_pose = self.context.ik.tcp_from_joints(current_joints)
        current_position = np.array(tcp_pose[:3])

        # Get object height from telemetry (set during pickup)
        object_height = self.context.telemetry.get_grasp_height()
        if object_height <= 0:
            logger.warning(
                f"Object height not available ({object_height}), using default 0.05m")
            object_height = 0.05  # Default 5cm if not available

        # Calculate placement positions
        # Move down by object height + PLACE_RELEASE_DISTANCE to place the object
        place_distance = object_height + PLACE_RELEASE_DISTANCE
        self.place_position = current_position - \
            np.array([0, 0, place_distance])
        # Move up by PLACE_APPROACH_DISTANCE to retreat
        self.retreat_position = current_position + \
            np.array([0, 0, PLACE_APPROACH_DISTANCE])

        logger.info(f"Current position: {current_position}")
        logger.info(f"Object height: {object_height:.3f}m")
        logger.info(
            f"Place position (down {place_distance:.3f}m): {self.place_position}")
        logger.info(
            f"Retreat position (up {PLACE_APPROACH_DISTANCE:.3f}m): {self.retreat_position}")

        # Create final states
        self.final_states = [
            MoveToState(self.context, target_location=tuple(
                self.place_position)),  # Move down
            GripperControlState(self.context, action='open'),  # Open gripper
            MoveToState(self.context, target_location=tuple(
                self.retreat_position))  # Move up
        ]
        self.final_state_index = 0

        # Start first final state
        logger.info("Step 3/5: Moving down to placement position")
        next_state = self.final_states[self.final_state_index]
        self.state_machine.current_state = next_state
        next_state.enter()

    def _continue_final_placement(self):
        """Continue through the final placement states."""
        self.final_state_index += 1

        if self.final_state_index >= len(self.final_states):
            # All final states complete
            logger.info("✅✅✅ Placement sequence complete!")
            self.current_step = 5
            self.is_complete_flag = True
            return

        # Move to next final state
        self.current_step = 3 + self.final_state_index
        step_names = ["Moving down", "Opening gripper", "Retreating up"]
        if self.final_state_index < len(step_names):
            logger.info(
                f"Step {self.current_step}/5: {step_names[self.final_state_index]}")

        next_state = self.final_states[self.final_state_index]
        self.state_machine.current_state = next_state
        next_state.enter()

    def is_complete(self) -> bool:
        """Check if the entire sequence is complete."""
        return self.is_complete_flag

    def get_progress(self) -> dict:
        """Get progress information."""
        return {
            'total_steps': self.total_steps,
            'current_step': self.current_step,
            'progress_percent': (self.current_step / self.total_steps) * 100 if self.total_steps > 0 else 100,
            'is_complete': self.is_complete_flag,
            'used_fallback': self.used_fallback
        }

    def get_sequence_description(self) -> List[str]:
        """Get human-readable sequence description."""
        desc = [
            "1. Approach operator's hand",
            "2. Track hand for placement position",
            "3. Move down to placement surface",
            "4. Open gripper to release object",
            "5. Retreat to safe position"
        ]
        if self.used_fallback:
            desc[1] = "2. [FALLBACK] Operator guides placement"
        return desc


def create_simple_placement_sequencer(state_machine: StateMachine, context: StateContext) -> SimplePlacementSequencer:
    """Factory function to create a SimplePlacementSequencer."""
    sequencer = SimplePlacementSequencer(state_machine, context)
    sequencer.start()  # Auto-start the sequence
    return sequencer
