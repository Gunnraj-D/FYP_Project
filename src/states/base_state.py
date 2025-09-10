"""
Base state for state machine
"""
import logging
from abc import ABC, abstractmethod
from typing import Optional

from shared_state import SharedState
from camera_manager import CameraManager
from opc_client import OPCClient
from kinematics_solver import InverseKinematicsSolver

from states_enum import States

logger = logging.getLogger(__name__)


class BaseState(ABC):
    """Abstract base class for all states."""

    def __init__(self, shared_state: SharedState, camera_manager: CameraManager,
                 opc_client: OPCClient, kinematics_solver: InverseKinematicsSolver):
        self.shared_state = shared_state
        self.camera_manager = camera_manager
        self.opc_client = opc_client
        self.kinematics_solver = kinematics_solver

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
