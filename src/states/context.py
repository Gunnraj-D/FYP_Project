# src/states/context.py
from dataclasses import dataclass
from control.telemetry_store import Telemetry
from control.command_bus import CommandBus
from camera_management.camera_manager import CameraManager
from IO_handling.opc_client import OPCClient
from kinematics.collision_aware_kinematics_solver import CollisionAwareKinematicsSolver
from typing import Optional


@dataclass(frozen=True)
class StateContext:
    telemetry: Telemetry
    commands: CommandBus
    camera: CameraManager
    opc: OPCClient
    ik: CollisionAwareKinematicsSolver
    hand_tracker: Optional[object] = None
    zed_receiver: Optional[object] = None  # ZED skeleton tracking receiver
