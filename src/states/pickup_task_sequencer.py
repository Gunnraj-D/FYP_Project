"""
Pickup Task Sequencer - Manages the complete object pickup sequence.
This sequencer orchestrates the states required to detect, approach, grasp, and lift an object.
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
from states.grasping_state import GraspingState
from config import PICKUP_LOCATION

logger = logging.getLogger(__name__)


class PickupTaskSequencer(TaskSequencer):
    """
    Task sequencer for object pickup operations.

    Sequence:
    1. MoveToState(pose=PICKUP_LOCATION )
    2. GraspingState()
    3. GripperControlState(action='open')
    4. MoveToState(pose_from_telemetry='generated_approach_pose')
    5. MoveToState(pose_from_telemetry='generated_grasp_pose')
    6. GripperControlState(action='close')
    7. MoveToState(pose_from_telemetry='generated_approach_pose')
    """

    def __init__(self, state_machine: StateMachine, context: StateContext):
        # Set Z offset before creating sequence (it's needed during sequence creation)
        self.z_offset = 0.2  # 20cm above grasp position

        # Create the sequence of states for pickup task
        pickup_states = self._create_pickup_sequence(context)

        # Initialize parent TaskSequencer
        super().__init__(state_machine, pickup_states)

        self.context = context
        logger.info("PickupTaskSequencer initialized with {} states".format(
            len(pickup_states)))

    def _create_pickup_sequence(self, context: StateContext) -> List[BaseState]:
        """
        Create the sequence of states for the pickup task.

        Args:
            context: State context containing shared resources

        Returns:
            List of states in execution order
        """
        states = []

        # Get Z offset (default 8cm)
        # z_offset = getattr(self, 'z_offset', 0.2)

        states.append(MoveToState(
            context, target_location=PICKUP_LOCATION["position"]))

        # 2. Generate pickup pose using GGCNN2/GR-ConvNet
        # Pass z_offset so GraspingState can create approach pose correctly
        states.append(GraspingState(context, approach_z_offset=self.z_offset))

        # 3. Open gripper
        states.append(GripperControlState(context, action='open'))

        # 4. Move to approach pose (above grasp position)
        # Z offset is applied by GraspingState when it stores the approach pose
        states.append(MoveToState(
            context, pose_from_telemetry='generated_approach_pose'))

        # 5. Move to grasp pose (final grasping position)
        states.append(MoveToState(
            context, pose_from_telemetry='generated_grasp_pose'))

        # 6. Close gripper to grasp object
        states.append(GripperControlState(context, action='close'))

        # 7. Move back to approach pose (lift object)
        # Apply Z offset to approach pose in telemetry (already done above, but ensuring consistency)
        states.append(MoveToState(
            context, pose_from_telemetry='generated_approach_pose'))

        logger.info(
            "Created pickup sequence with {} states".format(len(states)))
        logger.info(
            f"ℹ️ Approach Z offset ({self.z_offset*1000:.0f}mm) will be applied by GraspingState when storing poses")
        return states

    def get_sequence_description(self) -> List[str]:
        """Get a human-readable description of the pickup sequence."""
        return [
            "1. Move to pre-pickup position",
            "2. Generate grasp pose using GGCNN2",
            "3. Open gripper",
            "4. Move to approach pose (above object)",
            "5. Move to grasp pose (grasp object)",
            "6. Close gripper",
            "7. Lift object to approach pose"
        ]

    def get_current_step(self) -> int:
        """Get the current step number (1-based)."""
        return len(self.get_sequence_description()) - len(self.task_queue)

    def get_progress(self) -> dict:
        """Get progress information about the pickup sequence."""
        total_steps = len(self.get_sequence_description())
        current_step = self.get_current_step()

        return {
            'total_steps': total_steps,
            'current_step': current_step,
            'progress_percent': (current_step / total_steps) * 100,
            'remaining_steps': len(self.task_queue),
            'is_complete': len(self.task_queue) == 0
        }


def create_pickup_sequencer(state_machine: StateMachine, context: StateContext, z_offset: float = 0.0) -> PickupTaskSequencer:
    """
    Factory function to create a pickup task sequencer.

    Args:
        state_machine: State machine to manage state transitions
        context: Shared state context
        z_offset: Z-axis offset in meters to compensate for larger tools (default: 0.0m = DISABLED)
                  Set to 0.0 to use calibrated hand-eye matrix without artificial offsets

    Returns:
        Initialized PickupTaskSequencer with Z offset applied
    """
    sequencer = PickupTaskSequencer(state_machine, context)
    sequencer.z_offset = z_offset
    return sequencer
