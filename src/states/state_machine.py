import logging
from .base_state import BaseState

logger = logging.getLogger(__name__)


class StateMachine:
    """Manages transitions between states."""

    def __init__(self, initial_state: BaseState, on_state_completion=None):
        self.current_state: BaseState = initial_state
        self.on_state_completion = on_state_completion
        self.current_state.enter()

    def step(self):
        """Run one cycle of the current state."""
        if self.current_state.is_complete():
            if self.on_state_completion:
                self.on_state_completion()
        else:
            self.current_state.execute()

    def transition(self, next_state: BaseState):
        """Exit current state and enter the next one."""
        logger.info(
            f"Transition: {self.current_state.name} → {next_state.name}")
        self.current_state.exit()
        self.current_state = next_state
        self.current_state.enter()

    def get_current_state_name(self) -> str:
        return self.current_state.name
