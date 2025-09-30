"""
OPC UA client wrapper for robot communication.
Provides clean interface for robot control operations with dedicated background loop.
"""
import asyncio
from asyncua import Client, ua
import logging
import threading
import time
from typing import List, Optional, Dict, Any
from dataclasses import dataclass

from control.command_bus import CommandBus, Command, SetJoints, SetGripper, EmergencyStop
from control.telemetry_store import Telemetry
from config.config import (
    OPC_SERVER_URL, OPC_OBJECTS_NAME, OPC_UPDATE_INTERVAL_SECONDS,
    OPC_POLL_INTERVAL_MS, OPC_COMMAND_BATCH_SIZE, OPC_SKIP_REDUNDANT_WRITES,
    OPC_CONNECTION_TIMEOUT_SECONDS, OPC_RECONNECT_DELAY_SECONDS, OPC_MAX_RECONNECT_ATTEMPTS,
    ROBOT_ID, get_robot_name, get_robot_namespace
)

logger = logging.getLogger(__name__)


@dataclass
class OPCConfig:
    """OPC UA configuration parameters."""
    url: str = OPC_SERVER_URL
    objects_name: str = OPC_OBJECTS_NAME
    robot_id: int = ROBOT_ID
    poll_interval_ms: int = OPC_POLL_INTERVAL_MS
    command_batch_size: int = OPC_COMMAND_BATCH_SIZE
    skip_redundant_writes: bool = OPC_SKIP_REDUNDANT_WRITES
    connection_timeout: float = OPC_CONNECTION_TIMEOUT_SECONDS
    reconnect_delay: float = OPC_RECONNECT_DELAY_SECONDS
    max_reconnect_attempts: int = OPC_MAX_RECONNECT_ATTEMPTS

    @property
    def robot_name(self) -> str:
        """Get the robot name based on robot ID."""
        return get_robot_name(self.robot_id)

    @property
    def robot_namespace(self) -> int:
        """Get the robot namespace based on robot ID."""
        return get_robot_namespace(self.robot_id)


class OPCClient:
    """OPC UA client with dedicated background loop for robot communication."""

    def __init__(self, command_bus: CommandBus, telemetry: Telemetry, config: OPCConfig = None):
        self.command_bus = command_bus
        self.telemetry = telemetry
        self.config = config or OPCConfig()

        # Connection state
        self.client: Optional[Client] = None
        self.running = False
        self.connected = False
        self.comm_thread: Optional[threading.Thread] = None
        self._shutdown_event = threading.Event()

        # Node references
        self.joint_write_nodes: Dict[int, Any] = {}
        self.joint_read_nodes: Dict[int, Any] = {}
        self.control_nodes: Dict[str, Any] = {}

        # State tracking for redundant write detection
        self._last_joint_values: List[float] = [0.0] * 7
        self._last_gripper_status: Optional[str] = None
        self._reconnect_attempts = 0

        # Thread safety
        self._lock = threading.RLock()

    def start(self):
        """Start OPC UA communication in dedicated background thread."""
        with self._lock:
            if self.running:
                logger.warning("OPC UA client already running")
                return

            self.running = True
            self._shutdown_event.clear()
            self.comm_thread = threading.Thread(
                target=self._run_async_loop, name="OPCClient")
            self.comm_thread.daemon = True
            self.comm_thread.start()
            logger.info("OPC UA client started")

    def stop(self):
        """Stop OPC UA communication and cleanup resources."""
        with self._lock:
            if not self.running:
                return

            logger.info("Stopping OPC UA client...")
            self.running = False
            self._shutdown_event.set()

            if self.comm_thread and self.comm_thread.is_alive():
                self.comm_thread.join(timeout=5.0)
                if self.comm_thread.is_alive():
                    logger.warning("OPC client thread did not stop gracefully")

            logger.info("OPC UA client stopped")

    def _run_async_loop(self):
        """Run async event loop in separate thread."""
        try:
            asyncio.run(self._main_loop())
        except Exception as e:
            logger.error(f"OPC client async loop error: {e}")
        finally:
            with self._lock:
                self.connected = False
                self.running = False

    async def _main_loop(self):
        """Main async communication loop with reconnection logic."""
        while self.running and not self._shutdown_event.is_set():
            try:
                async with Client(url=self.config.url) as self.client:
                    self.client.set_session_timeout(
                        self.config.connection_timeout * 1000)
                    await self._initialize_nodes()
                    await self._start_robot_program()

                    with self._lock:
                        self.connected = True
                        self._reconnect_attempts = 0

                    # Update telemetry with connection status
                    self.telemetry.update_robot_status(
                        {'connected': True, 'status_code': 0})

                    await self._communication_loop()

            except Exception as e:
                logger.error(f"OPC UA connection error: {e}")

                with self._lock:
                    self.connected = False
                    self._reconnect_attempts += 1

                # Update telemetry with disconnection status
                self.telemetry.update_robot_status({
                    'connected': False,
                    'status_code': -1,
                    'error_message': str(e)
                })

                if self._reconnect_attempts >= self.config.max_reconnect_attempts:
                    logger.error(
                        f"Max reconnection attempts ({self.config.max_reconnect_attempts}) reached")
                    break

                if self.running and not self._shutdown_event.is_set():
                    logger.info(
                        f"Reconnecting in {self.config.reconnect_delay}s (attempt {self._reconnect_attempts})")
                    await asyncio.sleep(self.config.reconnect_delay)

        # Cleanup on exit
        await self._cleanup()

    async def _initialize_nodes(self):
        """Initialize OPC UA node references."""
        try:
            root = self.client.get_root_node()
            objects = await root.get_child([self.config.objects_name])
            robot = await objects.get_child([self.config.robot_name])

            # Initialize joint write nodes (R{robot_id}c_Joi1 to R{robot_id}c_Joi7)
            for i in range(1, 8):
                node_name = f"R{self.config.robot_id}c_Joi{i}"
                self.joint_write_nodes[i] = await robot.get_child([node_name])

            # Initialize joint read nodes (R{robot_id}d_Joi1 to R{robot_id}d_Joi7)
            for i in range(1, 8):
                node_name = f"R{self.config.robot_id}d_Joi{i}"
                self.joint_read_nodes[i] = await robot.get_child([node_name])

            # Initialize control nodes
            self.control_nodes['start'] = await robot.get_child([f"R{self.config.robot_id}c_Start"])
            self.control_nodes['prog_id'] = await robot.get_child([f"R{self.config.robot_id}c_ProgID"])
            self.control_nodes['status'] = await robot.get_child([f"R{self.config.robot_id}d_Status"])
            self.control_nodes['gripper_control'] = await robot.get_child([f"R{self.config.robot_id}c_GripperAct"])
            self.control_nodes['gripper_current'] = await robot.get_child([f"R{self.config.robot_id}d_GripperAct"])

            logger.info(
                f"OPC UA nodes initialized successfully for robot {self.config.robot_id}")

        except Exception as e:
            logger.error(f"Failed to initialize OPC UA nodes: {e}")
            raise

    async def _start_robot_program(self):
        """Start the robot control program."""
        try:
            # Set program ID (1 for joint control mode)
            program_id = ua.Variant(1, ua.VariantType.Int32)
            await self.control_nodes['prog_id'].write_value(program_id)

            # Start the program
            start = ua.Variant(True, ua.VariantType.Boolean)
            await self.control_nodes['start'].write_value(start)

            logger.info("Robot program started")

        except Exception as e:
            logger.error(f"Failed to start robot program: {e}")

    async def _stop_robot_program(self):
        """Stop the robot control program."""
        try:
            if 'start' in self.control_nodes and 'prog_id' in self.control_nodes:
                # Stop the program
                start = ua.Variant(False, ua.VariantType.Boolean)
                await self.control_nodes['start'].write_value(start)

                # Reset program ID
                program_id = ua.Variant(0, ua.VariantType.Int32)
                await self.control_nodes['prog_id'].write_value(program_id)

                logger.info("Robot program stopped")

        except Exception as e:
            logger.error(f"Failed to stop robot program: {e}")

    async def _communication_loop(self):
        """Main communication loop with telemetry updates and command processing."""
        loop_interval = self.config.poll_interval_ms / 1000.0

        while self.running and not self._shutdown_event.is_set():
            loop_start = time.time()

            try:
                # 1. Read telemetry from robot
                await self._update_telemetry()

                # 2. Process commands from CommandBus
                await self._process_commands()

            except Exception as e:
                logger.error(f"Communication loop error: {e}")

            # Maintain loop timing
            elapsed = time.time() - loop_start
            sleep_time = max(0, loop_interval - elapsed)
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)
            elif elapsed > loop_interval * 1.1:  # Warn if significantly over
                logger.warning(
                    f"OPC loop exceeded target interval by {elapsed - loop_interval:.3f}s")

    async def _update_telemetry(self):
        """Read robot state and update telemetry store."""
        try:
            # Read current joint positions
            current_joints = await self._read_joint_positions()
            if current_joints:
                self.telemetry.update_current_joints(current_joints)

            # Read robot status
            status = await self._read_robot_status()
            if status:
                self.telemetry.update_robot_status(status)

            # Read gripper status
            gripper_status = await self._read_gripper_status()
            if gripper_status:
                self.telemetry.update_current_gripper_status(gripper_status)

        except Exception as e:
            logger.error(f"Failed to update telemetry: {e}")

    async def _process_commands(self):
        """Drain CommandBus and execute commands sequentially."""
        try:
            commands = self.command_bus.recv_all_pending()

            if not commands:
                return  # No commands to process

            logger.debug(
                f"Processing {len(commands)} commands from CommandBus")

            # Process commands sequentially to preserve order
            for command in commands[:self.config.command_batch_size]:
                await self._execute_command(command)

        except Exception as e:
            logger.error(f"Failed to process commands: {e}")

    async def _execute_command(self, command: Command):
        """Execute a single command."""
        try:
            if isinstance(command, SetJoints):
                await self._write_joint_positions(command.joints)
            elif isinstance(command, SetGripper):
                await self._write_gripper_status(command.status)
            elif isinstance(command, EmergencyStop):
                await self._handle_emergency_stop(command.active)
            else:
                logger.warning(f"Unknown command type: {type(command)}")

        except Exception as e:
            logger.error(
                f"Failed to execute command {type(command).__name__}: {e}")

    async def _write_joint_positions(self, joint_positions: List[float]):
        """Write joint positions with redundant write detection."""
        if not self.connected or not self.client:
            return

        # Check for redundant writes
        if self.config.skip_redundant_writes:
            if len(joint_positions) == 7:
                if all(abs(joint_positions[i] - self._last_joint_values[i]) < 1e-6 for i in range(7)):
                    return  # Skip redundant write
                self._last_joint_values = joint_positions.copy()

        try:
            for i in range(1, 8):
                if i <= len(joint_positions):
                    value = ua.Variant(
                        float(joint_positions[i-1]), ua.VariantType.Double)
                    await self.joint_write_nodes[i].write_value(value)

            logger.debug(f"Wrote joint positions: {joint_positions}")

        except Exception as e:
            logger.error(f"Failed to write joint positions: {e}")

    async def _write_gripper_status(self, status: str):
        """Write gripper status with redundant write detection."""
        if not self.connected or not self.client:
            return

        # Check for redundant writes
        if self.config.skip_redundant_writes and self._last_gripper_status == status:
            return  # Skip redundant write

        self._last_gripper_status = status

        try:
            # Convert string status to boolean
            # "open" = False (gripper open), "close" = True (gripper closed)
            gripper_value = status.lower() == "close"

            # Write to gripper control node
            if 'gripper_control' in self.control_nodes:
                value = ua.Variant(gripper_value, ua.VariantType.Boolean)
                await self.control_nodes['gripper_control'].write_value(value)
                logger.debug(
                    f"Wrote gripper control: {gripper_value} (status: {status})")
            else:
                logger.warning("Gripper control node not available")

        except Exception as e:
            logger.error(f"Failed to write gripper status: {e}")

    async def _handle_emergency_stop(self, active: bool):
        """Handle emergency stop command."""
        try:
            if active:
                logger.warning("Emergency stop activated")
                self.telemetry.set_emergency_stop(True)
                # Stop robot program immediately
                await self._stop_robot_program()
            else:
                logger.info("Emergency stop deactivated")
                self.telemetry.set_emergency_stop(False)

        except Exception as e:
            logger.error(f"Failed to handle emergency stop: {e}")

    async def _read_joint_positions(self) -> Optional[List[float]]:
        """Read current joint positions from robot."""
        try:
            current_joints = []
            for i in range(1, 8):
                value = await self.joint_read_nodes[i].read_value()
                current_joints.append(float(value))
            return current_joints

        except Exception as e:
            logger.error(f"Failed to read joint positions: {e}")
            return None

    async def _read_robot_status(self) -> Optional[Dict[str, Any]]:
        """Read robot status information."""
        try:
            status = await self.control_nodes['status'].read_value()
            return {
                'connected': True,
                'status_code': int(status)
            }

        except Exception as e:
            logger.error(f"Failed to read robot status: {e}")
            return {
                'connected': False,
                'status_code': -1,
                'error_message': str(e)
            }

    async def _read_gripper_status(self) -> Optional[str]:
        """Read current gripper status from robot."""
        try:
            if 'gripper_current' in self.control_nodes:
                gripper_value = await self.control_nodes['gripper_current'].read_value()
                # Convert boolean to string: True = "close", False = "open"
                return "close" if bool(gripper_value) else "open"
            else:
                logger.warning("Gripper current node not available")
                return None

        except Exception as e:
            logger.error(f"Failed to read gripper status: {e}")
            return None

    async def _cleanup(self):
        """Cleanup resources on shutdown."""
        try:
            await self._stop_robot_program()
        except Exception as e:
            logger.error(f"Error during cleanup: {e}")

    def is_connected(self) -> bool:
        """Check if OPC UA client is connected and running."""
        with self._lock:
            return self.running and self.connected

    def get_status(self) -> Dict[str, Any]:
        """Get OPC client status information."""
        with self._lock:
            return {
                'running': self.running,
                'connected': self.connected,
                'reconnect_attempts': self._reconnect_attempts,
                'thread_alive': self.comm_thread.is_alive() if self.comm_thread else False
            }
