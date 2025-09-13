# src/control/command_bus.py
from dataclasses import dataclass
from typing import List, Literal, Union, Optional
from threading import RLock

@dataclass(frozen=True)
class SetJoints:
    joints: List[float]  # len 7

@dataclass(frozen=True)
class SetGripper:
    status: Literal["open", "close"]

@dataclass(frozen=True)
class EmergencyStop:
    active: bool = True

Command = Union[SetJoints, SetGripper, EmergencyStop]

class CommandBus:
    def __init__(self):
        self._lock = RLock()
        # Keep only the latest of each type
        self._latest_joints: Optional[SetJoints] = None
        self._latest_gripper: Optional[SetGripper] = None
        self._latest_estop: Optional[EmergencyStop] = None

    def send(self, cmd: Command):
        """Send a command, replacing any previous command of the same type."""
        with self._lock:
            if isinstance(cmd, SetJoints):
                self._latest_joints = cmd
            elif isinstance(cmd, SetGripper):
                self._latest_gripper = cmd
            elif isinstance(cmd, EmergencyStop):
                self._latest_estop = cmd

    def recv_all_pending(self) -> List[Command]:
        """Get all pending commands (latest of each type) and clear them."""
        with self._lock:
            commands = []
            
            # Emergency stop has highest priority
            if self._latest_estop is not None:
                commands.append(self._latest_estop)
                self._latest_estop = None
                # Clear everything else when emergency stop
                self._latest_joints = None
                self._latest_gripper = None
                return commands
            
            # Add latest of each type (if any)
            if self._latest_joints is not None:
                commands.append(self._latest_joints)
                self._latest_joints = None
                
            if self._latest_gripper is not None:
                commands.append(self._latest_gripper)
                self._latest_gripper = None
                
            return commands

    def has_pending(self) -> bool:
        """Check if there are any pending commands."""
        with self._lock:
            return (self._latest_joints is not None or 
                    self._latest_gripper is not None or 
                    self._latest_estop is not None)