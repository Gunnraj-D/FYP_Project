"""
Mock OPC UA Server for Kuka LBR iiwa robot simulation.
Replicates the exact node structure expected by the original opc_client.py.
"""
import asyncio
import logging
from asyncua import Server, ua
from asyncua.common.node import Node
import threading
import time
from typing import Dict, List, Optional

logger = logging.getLogger(__name__)


class MockOPCServer:
    """
    Mock OPC UA server that simulates a Kuka LBR iiwa robot.
    Provides the exact node structure expected by the original opc_client.py.
    """

    def __init__(self, url: str = "opc.tcp://127.0.0.1:4840/", robot_id: int = 1):
        self.url = url
        self.robot_id = robot_id
        self.server = Server()
        self.server.set_endpoint(url)

        # Validate robot ID
        if not (1 <= robot_id <= 4):
            raise ValueError(
                f"Robot ID must be between 1 and 4, got {robot_id}")

        # Calculate namespace based on robot ID
        self.namespace = 20 + robot_id  # Robot 1->21, Robot 2->22, etc.

        # Robot state
        self.joint_write_values: List[float] = [
            0.5, -1.0, 0.5, -2.0, 0.5, 1.5, 0.5]  # R1c_Joi1-7 (target positions)
        self.joint_read_values: List[float] = [
            0.5, -1.0, 0.5, -2.0, 0.5, 1.5, 0.5]   # R1d_Joi1-7 (current positions)
        self.start_flag: bool = False
        self.prog_id: int = 0
        self.status: int = 0
        self.gripper_control: bool = False  # R1c_GripperAct (control)
        self.gripper_current: bool = False  # R1d_GripperAct (current state)

        # Node references
        self.joint_write_nodes: Dict[int, Node] = {}
        self.joint_read_nodes: Dict[int, Node] = {}
        self.control_nodes: Dict[str, Node] = {}

        # Server state
        self.running = False
        self.server_thread: Optional[threading.Thread] = None
        self._shutdown_event = threading.Event()

        # Thread safety
        self._lock = threading.RLock()

    async def initialize(self):
        """Initialize the OPC UA server and create the robot node structure."""
        try:
            # Set up server namespace
            await self.server.init()

            # Create the robot object structure
            await self._create_robot_structure()

            logger.info("Mock OPC UA server initialized successfully")

        except Exception as e:
            logger.error(f"Failed to initialize mock OPC UA server: {e}")
            raise

    async def _create_robot_structure(self):
        """Create the robot object structure matching the original client expectations."""
        try:
            # Get the Objects folder (equivalent to "0:Objects")
            objects = self.server.get_objects_node()

            # Create robot object with dynamic namespace and name
            robot_name = f"robot{self.robot_id}"
            # Use robot ID as the object ID for clarity
            robot_obj = await objects.add_object(f"ns={self.namespace};i={self.robot_id}", robot_name)

            # Create joint write nodes (R{robot_id}c_Joi1 to R{robot_id}c_Joi7) - Target positions
            for i in range(1, 8):
                node_name = f"R{self.robot_id}c_Joi{i}"
                # Use string identifier with node name
                node_id = f"ns={self.namespace};s={node_name}"

                # Create variable node for joint target position
                var_node = await robot_obj.add_variable(
                    node_id,
                    node_name,
                    ua.Variant(
                        self.joint_write_values[i-1], ua.VariantType.Double)
                )

                # Set access level to read/write
                await var_node.set_writable()

                self.joint_write_nodes[i] = var_node
                logger.debug(f"Created joint write node: {node_name}")

            # Create joint read nodes (R{robot_id}d_Joi1 to R{robot_id}d_Joi7) - Current positions
            for i in range(1, 8):
                node_name = f"R{self.robot_id}d_Joi{i}"
                # Use string identifier with node name
                node_id = f"ns={self.namespace};s={node_name}"

                # Create variable node for joint current position
                var_node = await robot_obj.add_variable(
                    node_id,
                    node_name,
                    ua.Variant(
                        self.joint_read_values[i-1], ua.VariantType.Double)
                )

                # Set access level to read/write for mock simulation
                await var_node.set_writable()

                self.joint_read_nodes[i] = var_node
                logger.debug(f"Created joint read node: {node_name}")

            # Create control nodes
            # R{robot_id}c_Start - Boolean control flag
            start_node_name = f"R{self.robot_id}c_Start"
            start_node = await robot_obj.add_variable(
                f"ns={self.namespace};s={start_node_name}",
                start_node_name,
                ua.Variant(self.start_flag, ua.VariantType.Boolean)
            )
            # Data type is already set when creating the variable
            await start_node.set_writable()
            self.control_nodes['start'] = start_node

            # R{robot_id}c_ProgID - Program ID (Integer)
            prog_id_node_name = f"R{self.robot_id}c_ProgID"
            prog_id_node = await robot_obj.add_variable(
                f"ns={self.namespace};s={prog_id_node_name}",
                prog_id_node_name,
                ua.Variant(self.prog_id, ua.VariantType.Int32)
            )
            # Data type is already set when creating the variable
            await prog_id_node.set_writable()
            self.control_nodes['prog_id'] = prog_id_node

            # R{robot_id}d_Status - Robot status (Integer)
            status_node_name = f"R{self.robot_id}d_Status"
            status_node = await robot_obj.add_variable(
                f"ns={self.namespace};s={status_node_name}",
                status_node_name,
                ua.Variant(self.status, ua.VariantType.Int32)
            )
            # Data type is already set when creating the variable
            await status_node.set_writable()  # Make writable for mock simulation
            self.control_nodes['status'] = status_node

            # R{robot_id}c_GripperAct - Gripper control (Boolean)
            gripper_control_node_name = f"R{self.robot_id}c_GripperAct"
            gripper_control_node = await robot_obj.add_variable(
                f"ns={self.namespace};s={gripper_control_node_name}",
                gripper_control_node_name,
                ua.Variant(self.gripper_control, ua.VariantType.Boolean)
            )
            await gripper_control_node.set_writable()
            self.control_nodes['gripper_control'] = gripper_control_node

            # R{robot_id}d_GripperAct - Gripper current state (Boolean)
            gripper_current_node_name = f"R{self.robot_id}d_GripperAct"
            gripper_current_node = await robot_obj.add_variable(
                f"ns={self.namespace};s={gripper_current_node_name}",
                gripper_current_node_name,
                ua.Variant(self.gripper_current, ua.VariantType.Boolean)
            )
            await gripper_current_node.set_writable()  # Make writable for mock simulation
            self.control_nodes['gripper_current'] = gripper_current_node

            logger.info(
                f"Robot {self.robot_id} object structure created successfully")
            logger.info(
                f"Created {len(self.joint_write_nodes)} joint write nodes for robot {self.robot_id}")
            logger.info(
                f"Created {len(self.joint_read_nodes)} joint read nodes for robot {self.robot_id}")
            logger.info(
                f"Created {len(self.control_nodes)} control nodes for robot {self.robot_id}")
            logger.info(
                f"Using namespace {self.namespace} for robot {self.robot_id}")

        except Exception as e:
            logger.error(f"Failed to create robot structure: {e}")
            raise

    async def start_server(self):
        """Start the OPC UA server."""
        try:
            await self.server.start()
            self.running = True
            logger.info(f"Mock OPC UA server started at {self.url}")

            # Initialize read nodes with current values
            await self._update_joint_read_nodes()

            # Start the simulation loop
            asyncio.create_task(self._simulation_loop())

        except Exception as e:
            logger.error(f"Failed to start mock OPC UA server: {e}")
            raise

    async def stop_server(self):
        """Stop the OPC UA server."""
        try:
            self.running = False
            self._shutdown_event.set()
            await self.server.stop()
            logger.info("Mock OPC UA server stopped")

        except Exception as e:
            logger.error(f"Error stopping mock OPC UA server: {e}")

    async def _simulation_loop(self):
        """Main simulation loop that updates robot state."""
        logger.info("Starting robot simulation loop")

        while self.running and not self._shutdown_event.is_set():
            try:
                # Update robot status
                await self._update_robot_status()

                # Sleep for simulation update rate (50Hz)
                await asyncio.sleep(0.02)

            except Exception as e:
                logger.error(f"Error in simulation loop: {e}")
                await asyncio.sleep(0.1)  # Wait before retrying

    async def _update_joint_read_nodes(self):
        """Update the joint read nodes with current values."""
        try:
            with self._lock:
                for i in range(1, 8):
                    if i in self.joint_read_nodes:
                        value = ua.Variant(
                            self.joint_read_values[i-1], ua.VariantType.Double)
                        await self.joint_read_nodes[i].write_value(value)

        except Exception as e:
            logger.error(f"Failed to update joint read nodes: {e}")

    async def _update_robot_status(self):
        """Update robot status based on current state."""
        try:
            with self._lock:
                # Set status based on program state
                if self.start_flag and self.prog_id == 1:
                    self.status = 1  # Running
                elif self.start_flag and self.prog_id == 0:
                    self.status = 2  # Stopped
                else:
                    self.status = 0  # Idle

                # Update status node
                if 'status' in self.control_nodes:
                    value = ua.Variant(self.status, ua.VariantType.Int32)
                    await self.control_nodes['status'].write_value(value)

        except Exception as e:
            logger.error(f"Failed to update robot status: {e}")

    async def write_joint_target(self, joint_index: int, value: float):
        """Write a target joint position (simulates command from client)."""
        try:
            if 1 <= joint_index <= 7:
                with self._lock:
                    self.joint_write_values[joint_index - 1] = value
                    # Update the read values to match the write values for instantaneous movement simulation
                    self.joint_read_values[joint_index - 1] = value

                # Update the write node
                if joint_index in self.joint_write_nodes:
                    variant_value = ua.Variant(value, ua.VariantType.Double)
                    await self.joint_write_nodes[joint_index].write_value(variant_value)

                # Update the read node to reflect the new current position
                if joint_index in self.joint_read_nodes:
                    variant_value = ua.Variant(value, ua.VariantType.Double)
                    await self.joint_read_nodes[joint_index].write_value(variant_value)

                logger.info(
                    f"Set joint {joint_index} target to {value} and updated current position")
            else:
                logger.warning(f"Invalid joint index: {joint_index}")

        except Exception as e:
            logger.error(f"Failed to write joint target: {e}")

    async def write_control_flag(self, flag_name: str, value):
        """Write a control flag (start, prog_id, gripper_control)."""
        try:
            with self._lock:
                if flag_name == 'start':
                    self.start_flag = bool(value)
                elif flag_name == 'prog_id':
                    self.prog_id = int(value)
                elif flag_name == 'gripper_control':
                    self.gripper_control = bool(value)
                    # Update current gripper state to match control for instantaneous simulation
                    self.gripper_current = bool(value)
                else:
                    logger.warning(f"Unknown control flag: {flag_name}")
                    return

                # Update the control node
                if flag_name in self.control_nodes:
                    if flag_name in ['start', 'gripper_control']:
                        variant_value = ua.Variant(
                            value, ua.VariantType.Boolean)
                    else:  # prog_id
                        variant_value = ua.Variant(value, ua.VariantType.Int32)

                    await self.control_nodes[flag_name].write_value(variant_value)

                # If gripper control was updated, also update the current gripper state
                if flag_name == 'gripper_control' and 'gripper_current' in self.control_nodes:
                    variant_value = ua.Variant(value, ua.VariantType.Boolean)
                    await self.control_nodes['gripper_current'].write_value(variant_value)

                logger.debug(f"Set {flag_name} to {value}")

        except Exception as e:
            logger.error(f"Failed to write control flag: {e}")

    def get_joint_positions(self) -> List[float]:
        """Get current joint positions (for external access)."""
        with self._lock:
            return self.joint_read_values.copy()

    def get_robot_status(self) -> Dict[str, any]:
        """Get robot status information."""
        with self._lock:
            return {
                'start': self.start_flag,
                'prog_id': self.prog_id,
                'status': self.status,
                'joint_positions': self.joint_read_values.copy(),
                'gripper_control': self.gripper_control,
                'gripper_current': self.gripper_current
            }

    def is_running(self) -> bool:
        """Check if server is running."""
        return self.running


async def run_mock_server(robot_id: int = 1):
    """Run the mock OPC UA server."""
    server = MockOPCServer(robot_id=robot_id)

    try:
        await server.initialize()
        await server.start_server()

        logger.info(
            f"Mock OPC UA server for robot {robot_id} is running. Press Ctrl+C to stop.")

        # Keep server running
        while server.is_running():
            await asyncio.sleep(1)

    except KeyboardInterrupt:
        logger.info("Received shutdown signal")
    except Exception as e:
        logger.error(f"Server error: {e}")
    finally:
        await server.stop_server()


if __name__ == "__main__":
    import sys

    # Configure logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    # Get robot ID from command line argument or use default
    robot_id = 1
    if len(sys.argv) > 1:
        try:
            robot_id = int(sys.argv[1])
            if not (1 <= robot_id <= 4):
                print(
                    f"Error: Robot ID must be between 1 and 4, got {robot_id}")
                sys.exit(1)
        except ValueError:
            print(
                f"Error: Invalid robot ID '{sys.argv[1]}'. Must be an integer between 1 and 4.")
            sys.exit(1)

    print(f"Starting mock OPC UA server for robot {robot_id}...")
    print(f"Robot {robot_id} will use namespace {20 + robot_id}")
    print(
        f"Node names will follow pattern: R{robot_id}d_Status, R{robot_id}c_Joi1, etc.")

    # Run the server
    asyncio.run(run_mock_server(robot_id))
