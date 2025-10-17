"""
Handoff Fallback State - Placeholder for handling hand occlusion failures.

This state is activated when the hand tracking state fails due to persistent
hand occlusion. It provides a safe fallback behavior for the placement task.

PLACEHOLDER IMPLEMENTATION:
This is a placeholder state that currently performs basic error handling.
Future implementations could include:
- Retry logic with different approach angles
- Voice/audio prompts to the operator
- Moving to a designated drop-off zone
- Returning object to pickup location
- Waiting at a safe position for operator intervention
"""
import logging
import time
from typing import Optional

from states.base_state import BaseState
from states.context import StateContext

logger = logging.getLogger(__name__)


class HandoffFallbackState(BaseState):
    """
    Fallback state for handling hand tracking failures due to occlusion.

    Current Behavior (PLACEHOLDER):
    - Logs the failure
    - Provides basic status information
    - Completes immediately (allowing sequencer to handle next steps)

    Future Enhancements:
    - Implement retry with timeout
    - Move to safe waiting position
    - Provide operator feedback (audio/visual)
    - Alternative handoff strategies
    """

    def __init__(self, context: StateContext, failure_reason: Optional[str] = None):
        """
        Initialize the fallback state.

        Args:
            context: Shared state context
            failure_reason: Optional description of why fallback was triggered
        """
        super().__init__(context)
        self.failure_reason = failure_reason or "Hand occlusion detected"
        self.entry_time = 0.0
        # Time to wait before completing (seconds)
        self.fallback_duration = 2.0

    def enter(self):
        """Enter the fallback state and log the failure."""
        self.entry_time = time.time()
        logger.warning("=" * 80)
        logger.warning("⚠️  ENTERING HANDOFF FALLBACK STATE")
        logger.warning(f"Reason: {self.failure_reason}")
        logger.warning("=" * 80)

        # Log current robot state for debugging
        try:
            current_joints = self.context.telemetry.get_current_joints()
            if current_joints:
                logger.info(f"Current joint configuration: {current_joints}")

            hand_position = self.context.telemetry.get_camera_vector()
            logger.info(f"Last known hand position: {hand_position}")
        except Exception as e:
            logger.error(f"Error logging robot state: {e}")

        # TODO: Implement actual fallback behavior here
        # Examples:
        # - Move to safe waiting position
        # - Play audio prompt: "Please present your hand clearly"
        # - Return to approach position for retry
        # - Move to designated drop-off zone

        logger.info("PLACEHOLDER: Fallback state entered but no action taken")
        logger.info(f"Will complete after {self.fallback_duration}s")

    def execute(self):
        """
        Execute fallback behavior (currently placeholder).

        PLACEHOLDER: This method currently does nothing.

        Future implementations could:
        - Monitor for hand reappearance
        - Execute safe positioning moves
        - Provide periodic status updates
        """
        elapsed = time.time() - self.entry_time

        # Log periodic status updates
        if int(elapsed) % 5 == 0 and int(elapsed) > 0:
            logger.info(f"Fallback state active for {elapsed:.1f}s")

    def is_complete(self) -> bool:
        """
        Check if fallback state is complete.

        Returns:
            True after fallback_duration seconds have elapsed
        """
        elapsed = time.time() - self.entry_time
        return elapsed >= self.fallback_duration

    def exit(self):
        """Exit the fallback state."""
        elapsed = time.time() - self.entry_time
        logger.warning(f"Exiting HandoffFallbackState after {elapsed:.1f}s")
        logger.info("PLACEHOLDER: No cleanup required")

        # TODO: Add any cleanup logic here
        # Examples:
        # - Stop audio feedback
        # - Clear warning indicators
        # - Reset hand tracking state

    def get_fallback_info(self) -> dict:
        """
        Get information about the fallback state.

        Returns:
            Dictionary with fallback status information
        """
        elapsed = time.time() - self.entry_time if self.entry_time > 0 else 0.0

        return {
            'failure_reason': self.failure_reason,
            'elapsed_time': elapsed,
            'fallback_duration': self.fallback_duration,
            'time_remaining': max(0, self.fallback_duration - elapsed),
            'is_complete': self.is_complete()
        }

