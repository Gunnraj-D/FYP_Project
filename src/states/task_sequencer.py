import logging
from collections import deque
from state_machine import StateMachine
from base_state import BaseState

logger = logging.getLogger(__name__)


class TaskSequencer:
    """Sequences a predefined list of states into the state machine."""

    def __init__(self, state_machine: StateMachine, task_states: list[BaseState]):
        self.state_machine = state_machine
        self.task_queue = deque(task_states)

    def step(self):
        """Advance state machine through queued states."""
        self.state_machine.step()

    def queue_next_task(self):
        if self.task_queue:
            next_state = self.task_queue.popleft()
            self.state_machine.transition(next_state)
        else:
            logger.info("Task complete — no more states in sequence.")
