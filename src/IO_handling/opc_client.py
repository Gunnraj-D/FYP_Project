"""
OPC UA client wrapper for robot communication.
Provides clean interface for robot control operations with dedicated background loop.
"""
import asyncio
from asyncua import Client, ua
import logging
import math
import threading
import time
from typing import List, Optional, Dict, Any
from dataclasses import dataclass

from control.command_bus import CommandBus, Command, SetJoints, SetGripper, EmergencyStop
from control.telemetry_store import Telemetry
from config import (
    OPC_SERVER_URL, OPC_OBJECTS_NAME, OPC_UPDATE_INTERVAL_SECONDS,
    OPC_POLL_INTERVAL_MS, OPC_COMMAND_BATCH_SIZE, OPC_SKIP_REDUNDANT_WRITES,
    OPC_CONNECTION_TIMEOUT_SECONDS, OPC_RECONNECT_DELAY_SECONDS, OPC_MAX_RECONNECT_ATTEMPTS,
    ROBOT_ID, get_robot_name, get_robot_namespace
)

logger = logging.getLogger(__name__)


def radians_to_degrees(radians: float) -> float:
    """Convert radians to degrees."""
    return radians * 180.0 / math.pi


def degrees_to_radians(degrees: float) -> float:
    """Convert degrees to radians."""
    return degrees * math.pi / 180.0


def convert_joints_rad_to_deg(joint_positions: List[float]) -> List[float]:
    """Convert joint positions from radians to degrees."""
    return [radians_to_degrees(angle) for angle in joint_positions]


def convert_joints_deg_to_rad(joint_positions: List[float]) -> List[float]:
    """Convert joint positions from degrees to radians."""
    return [degrees_to_radians(angle) for angle in joint_positions]


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

        # Loop timing history for warning management
        self.loop_timings: List[float] = []
        self.max_timing_history = 100

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

            # Note: Start flag will be set to False in the async cleanup path
            # The _stop_robot_program() is called in _main_loop() when connection ends

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
        logger.info("🔄 Starting OPC client main loop")
        while self.running and not self._shutdown_event.is_set():
            try:
                logger.info(f"🔄 Attempting to connect to {self.config.url}")
                async with Client(url=self.config.url) as self.client:
                    logger.info("✅ OPC client connected to server")
                    await self._initialize_nodes()
                    await self._start_robot_program()

                    with self._lock:
                        self.connected = True
                        self._reconnect_attempts = 0

                    self.telemetry.update_robot_status(
                        {'connected': True, 'status_code': 0})
                    logger.info(
                        "✅ OPC client connected, starting communication loop")

                    await self._communication_loop()

                    # Stop the program before disconnecting
                    logger.info(
                        "Communication loop ended. Stopping robot program...")
                    await self._stop_robot_program()

            except (ConnectionRefusedError, asyncio.TimeoutError) as e:
                logger.warning(f"OPC UA connection failed: {e}")
            except Exception as e:
                logger.error(
                    f"OPC UA main loop error: {e}", exc_info=True)
            finally:
                with self._lock:
                    self.connected = False

                if self.running and not self._shutdown_event.is_set():
                    self._reconnect_attempts += 1
                    if self._reconnect_attempts >= self.config.max_reconnect_attempts:
                        logger.error(
                            f"Max reconnection attempts ({self.config.max_reconnect_attempts}) reached. Stopping client.")
                        self.running = False
                        break

                    wait_time = min(self.config.reconnect_delay *
                                    (2 ** self._reconnect_attempts), 10.0)
                    logger.info(
                        f"Reconnecting in {wait_time:.1f}s (attempt {self._reconnect_attempts})")
                    await asyncio.sleep(wait_time)

        # Cleanup on exit
        logger.info("OPC client main loop finished.")
        await self._cleanup()

    async def _initialize_nodes(self):
        """Initialize OPC UA node references using direct node IDs."""
        try:
            # Access nodes by their node IDs to match server structure
            # Objects folder (ns=0;i=85)
            objects = self.client.get_node("ns=0;i=85")

            # Robot object with dynamic namespace and ID
            robot = self.client.get_node(
                f"ns={self.config.robot_namespace};i={self.config.robot_id}")

            # Initialize joint write nodes (R{robot_id}c_Joi1 to R{robot_id}c_Joi7) with string identifiers
            for i in range(1, 8):
                node_name = f"R{self.config.robot_id}c_Joi{i}"
                node_id = f"ns={self.config.robot_namespace};s={node_name}"
                self.joint_write_nodes[i] = self.client.get_node(node_id)

            # Initialize joint read nodes (R{robot_id}d_Joi1 to R{robot_id}d_Joi7) with string identifiers
            for i in range(1, 8):
                node_name = f"R{self.config.robot_id}d_Joi{i}"
                node_id = f"ns={self.config.robot_namespace};s={node_name}"
                self.joint_read_nodes[i] = self.client.get_node(node_id)

            # Initialize control nodes with string identifiers
            self.control_nodes['start'] = self.client.get_node(
                f"ns={self.config.robot_namespace};s=R{self.config.robot_id}c_Start")
            self.control_nodes['prog_id'] = self.client.get_node(
                f"ns={self.config.robot_namespace};s=R{self.config.robot_id}c_ProgID")
            self.control_nodes['status'] = self.client.get_node(
                f"ns={self.config.robot_namespace};s=R{self.config.robot_id}d_Status")
            self.control_nodes['gripper_control'] = self.client.get_node(
                f"ns={self.config.robot_namespace};s=R{self.config.robot_id}c_GripperAct")
            self.control_nodes['gripper_current'] = self.client.get_node(
                f"ns={self.config.robot_namespace};s=R{self.config.robot_id}d_GripperAct")

            logger.info(
                f"OPC UA nodes initialized successfully for robot {self.config.robot_id}")

        except Exception as e:
            logger.error(f"Failed to initialize OPC UA nodes: {e}")
            raise

    async def _start_robot_program(self):
        """Start the robot control program using batch operations."""
        try:
            # Prepare nodes and values for batch write
            nodes_to_write = [self.control_nodes['prog_id'],
                              self.control_nodes['start']]
            values_to_write = [
                # program ID (1 for joint control mode)
                ua.Variant(1, ua.VariantType.Int32),
                ua.Variant(True, ua.VariantType.Boolean)  # start program
            ]

            # Perform batch write for control commands
            await self.client.write_values(nodes_to_write, values_to_write)

            logger.info("Robot program started")

        except Exception as e:
            logger.error(f"Failed to start robot program: {e}")

    async def _stop_robot_program(self):
        """Stop the robot control program using batch operations."""
        try:
            if self.client and self.connected:
                logger.info("Writing 'stop' to robot program...")

                # Prepare nodes and values for batch write
                nodes_to_write = [self.control_nodes['start'],
                                  self.control_nodes['prog_id']]
                values_to_write = [
                    ua.Variant(False, ua.VariantType.Boolean),  # stop program
                    ua.Variant(0, ua.VariantType.Int32)  # reset program ID
                ]

                # Perform batch write for stop commands
                await self.client.write_values(nodes_to_write, values_to_write)
                logger.info("Robot program stopped successfully.")
            else:
                logger.warning(
                    "Cannot stop robot program, client is not connected.")
        except Exception as e:
            logger.error(
                f"Failed to stop robot program during shutdown: {e}")

    async def _communication_loop(self):
        """Main communication loop with telemetry updates and command processing."""
        loop_interval = self.config.poll_interval_ms / 1000.0
        loop_count = 0

        logger.info("🔄 Starting OPC communication loop")

        while self.running and not self._shutdown_event.is_set():
            loop_start = time.time()
            loop_count += 1

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

            # Track loop timing for warning management
            self.loop_timings.append(elapsed)
            if len(self.loop_timings) > self.max_timing_history:
                self.loop_timings.pop(0)

            # Only warn if average of last 100 loops exceeds threshold
            if len(self.loop_timings) >= self.max_timing_history:
                avg_elapsed = sum(self.loop_timings) / len(self.loop_timings)
                if avg_elapsed > loop_interval * 1.1:  # Warn if average significantly over
                    pass
                    # logger.warning(
                        # f"OPC loop average exceeded target interval by {avg_elapsed - loop_interval:.3f}s "
                        # f"(avg of last {len(self.loop_timings)} loops)")

            if sleep_time > 0:
                await asyncio.sleep(sleep_time)

    async def _update_telemetry(self):
        """Read robot state and update telemetry store using batch operations."""
        try:
            # Prepare all nodes for batch read
            nodes_to_read = []
            node_types = []

            # Add joint read nodes
            for i in range(1, 8):
                nodes_to_read.append(self.joint_read_nodes[i])
                node_types.append('joint')

            # Add robot status node
            nodes_to_read.append(self.control_nodes['status'])
            node_types.append('status')

            # Add gripper current node
            if 'gripper_current' in self.control_nodes:
                nodes_to_read.append(self.control_nodes['gripper_current'])
                node_types.append('gripper')

            # Perform batch read
            values = await self.client.read_values(nodes_to_read)

            # Process results
            joint_values = []
            status_value = None
            gripper_value = None

            for i, (value, node_type) in enumerate(zip(values, node_types)):
                if node_type == 'joint':
                    joint_values.append(float(value))
                elif node_type == 'status':
                    status_value = value
                elif node_type == 'gripper':
                    gripper_value = value

            # Update telemetry with batch results
            if joint_values:
                # Convert joint values from degrees (server) to radians (application)
                joint_values_rad = convert_joints_deg_to_rad(joint_values)
                self.telemetry.update_current_joints(joint_values_rad)

            if status_value is not None:
                self.telemetry.update_robot_status({
                    'connected': True,
                    'status_code': int(status_value)
                })

            if gripper_value is not None:
                gripper_status = "close" if bool(gripper_value) else "open"
                self.telemetry.update_current_gripper_status(gripper_status)

        except Exception as e:
            logger.error(f"Failed to update telemetry: {e}")

    async def _process_commands(self):
        """Drain CommandBus and execute commands sequentially."""
        try:
            commands = self.command_bus.recv_all_pending()

            if not commands:
                return  # No commands to process

            logger.info(
                f"Processing {len(commands)} commands from CommandBus")

            # Process commands sequentially to preserve order
            for command in commands[:self.config.command_batch_size]:
                logger.info(f"Executing command: {type(command).__name__}")
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
        """Write joint positions with redundant write detection using batch operations."""
        if not self.connected or not self.client:
            return

        # Check for redundant writes
        if self.config.skip_redundant_writes:
            if len(joint_positions) == 7:
                if all(abs(joint_positions[i] - self._last_joint_values[i]) < 1e-6 for i in range(7)):
                    return  # Skip redundant write
                self._last_joint_values = joint_positions.copy()

        try:
            # Convert joint positions from radians to degrees for OPC server
            joint_positions_deg = convert_joints_rad_to_deg(joint_positions)

            # Prepare nodes and values for batch write
            nodes_to_write = []
            values_to_write = []

            # Prepare write nodes (R{robot_id}c_Joi1-7) - target positions in degrees
            for i in range(1, 8):
                if i <= len(joint_positions_deg):
                    nodes_to_write.append(self.joint_write_nodes[i])
                    values_to_write.append(ua.Variant(
                        float(joint_positions_deg[i-1]), ua.VariantType.Double))

            # Perform batch write for all joint nodes
            await self.client.write_values(nodes_to_write, values_to_write)

            logger.debug(f"Wrote joint positions: {joint_positions}")

        except Exception as e:
            logger.error(f"Failed to write joint positions: {e}")

    async def _write_gripper_status(self, status: str):
        """Write gripper status with redundant write detection using batch operations."""
        if not self.connected or not self.client:
            logger.warning(
                f"Cannot write gripper status '{status}' - not connected")
            return

        # Check for redundant writes
        if self.config.skip_redundant_writes and self._last_gripper_status == status:
            logger.debug(f"Skipping redundant gripper write: {status}")
            return  # Skip redundant write

        self._last_gripper_status = status

        try:
            # Convert string status to boolean
            # "open" = False (gripper open), "close" = True (gripper closed)
            gripper_value = status.lower() == "close"

            # Prepare nodes and values for batch write
            nodes_to_write = []
            values_to_write = []

            # Add gripper control node
            if 'gripper_control' in self.control_nodes:
                nodes_to_write.append(self.control_nodes['gripper_control'])
                values_to_write.append(ua.Variant(
                    gripper_value, ua.VariantType.Boolean))

                # Perform batch write for gripper nodes
                await self.client.write_values(nodes_to_write, values_to_write)
                logger.info(
                    f"✅ Wrote gripper control: {status} → {gripper_value}")
            else:
                logger.error(
                    "❌ Gripper control node not available in control_nodes!")
                logger.error(
                    f"Available control nodes: {list(self.control_nodes.keys())}")

        except Exception as e:
            logger.error(f"Failed to write gripper status '{status}': {e}")

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
        """Read current joint positions from robot using batch read."""
        try:
            # Prepare nodes for batch read
            nodes_to_read = [self.joint_read_nodes[i] for i in range(1, 8)]

            # Perform batch read
            values = await self.client.read_values(nodes_to_read)

            # Convert to float list (values are in degrees from server)
            current_joints_deg = [float(value) for value in values]

            # Convert from degrees to radians for application
            current_joints_rad = convert_joints_deg_to_rad(current_joints_deg)

            return current_joints_rad

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
            # Only attempt cleanup if we have a valid connection
            if self.connected and self.client and self.control_nodes:
                await self._stop_robot_program()
            else:
                logger.debug(
                    "Skipping cleanup - not connected or nodes not available")
        except Exception as e:
            # Silently ignore cleanup errors during shutdown
            logger.debug(f"Cleanup error (ignored): {e}")
            pass

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
