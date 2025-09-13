"""
Base state for state machine
"""
import logging
from abc import ABC, abstractmethod

from states.context import StateContext

logger = logging.getLogger(__name__)


class BaseState(ABC):
    """Abstract base class for all states."""

    def __init__(self, context: StateContext):
        self.context = context

    @property
    def name(self) -> str:
        return self.__class__.__name__

    def enter(self):
        """Called when entering this state."""
        logger.debug(f"Entering {self.__class__.__name__}")

    def exit(self):
        """Called when exiting this state."""
        logger.debug(f"Exiting {self.__class__.__name__}")

    @abstractmethod
    def execute(self):
        """Main execution logic for this state."""
        pass

    @abstractmethod
    def is_complete(self) -> bool:
        """Determine the next state based on current conditions."""
        pass
