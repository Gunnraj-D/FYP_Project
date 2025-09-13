# src/states/context.py
from dataclasses import dataclass
from control.telemetry_store import Telemetry
from control.command_bus import CommandBus
from camera_management.camera_manager import CameraManager
from IO_handling.opc_client import OPCClient
from kinematics.kinematics_solver import InverseKinematicsSolver
from typing import Optional

@dataclass(frozen=True)
class StateContext:
    telemetry: Telemetry
    commands: CommandBus
    camera: CameraManager
    opc: OPCClient
    ik: InverseKinematicsSolver
    hand_tracker: Optional[object] = None  