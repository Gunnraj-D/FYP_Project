"""
OPC UA communication module for KUKA robot control.
Handles bidirectional communication with robot controller.
"""
import asyncio
from asyncua import Client, ua
from shared_state import SharedState
import logging
import threading
from typing import List, Optional

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class RobotCommunication:
    """Handles OPC UA communication with KUKA robot controller."""
    
    def __init__(self, shared_state: SharedState):
        # OPC UA configuration
        self.url = "opc.tcp://172.24.200.1:4840/"
        self.objects_name = "0:Objects"
        self.robot_name = "21:robot1" 
        
        # Shared state reference
        self.shared_state = shared_state
        
        # Control variables
        self.running = False
        self.update_interval = 0.02  # 5ms update rate (50Hz)
        self.client: Optional[Client] = None
        self.comm_thread: Optional[threading.Thread] = None
        
        # Node references (will be initialized on connect)
        self.joint_write_nodes = {}
        self.joint_read_nodes = {}
        self.control_nodes = {}
        
    def start(self):
        """Start OPC UA communication in a separate thread."""
        if self.running:
            logger.warning("OPC UA communication already running")
            return
            
        self.running = True
        self.comm_thread = threading.Thread(target=self._run_async_loop)
        self.comm_thread.daemon = True
        self.comm_thread.start()
        logger.info("OPC UA communication started")
        
    def stop(self):
        """Stop OPC UA communication."""
        if not self.running:
            return
            
        logger.info("Stopping OPC UA communication...")
        self.running = False
        
        if self.comm_thread:
            self.comm_thread.join(timeout=2.0)
            
        logger.info("OPC UA communication stopped")
        
    def _run_async_loop(self):
        """Run the async event loop in a separate thread."""
        asyncio.run(self._main_loop())
        
    async def _main_loop(self):
        """Main async loop for OPC UA communication."""
        try:
            async with Client(url=self.url) as self.client:
                await self._initialize_nodes()
                await self._communication_loop()
        except Exception as e:
            logger.error(f"OPC UA communication error: {e}")
            self.running = False
            
    async def _initialize_nodes(self):
        """Initialize OPC UA node references."""
        try:
            root = self.client.get_root_node()
            objects = await root.get_child([self.objects_name])
            robot = await objects.get_child([self.robot_name])
            
            # Initialize joint write nodes (R1c_Joi1 to R1c_Joi7)
            for i in range(1, 8):
                node_name = f"R1c_Joi{i}"
                self.joint_write_nodes[i] = await robot.get_child([node_name])
                
            # Initialize joint read nodes (R1d_Joi1 to R1d_Joi7)
            for i in range(1, 8):
                node_name = f"R1d_Joi{i}"
                self.joint_read_nodes[i] = await robot.get_child([node_name])
                
            # Initialize control nodes
            self.control_nodes['start'] = await robot.get_child(["R1c_Start"])
            self.control_nodes['prog_id'] = await robot.get_child(["R1c_ProgID"])
            self.control_nodes['status'] = await robot.get_child(["R1d_Status"])
            
            logger.info("OPC UA nodes initialized successfully")
            
            # Start the robot program
            await self._start_robot_program()
            
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
        """Main communication loop for reading and writing robot data."""
        while self.running:
            try:
                # Read current joint positions
                await self._read_joint_positions()
                
                # Write target joint positions
                await self._write_joint_positions()
                
                # Read robot status
                await self._read_robot_status()
                
                # Sleep for update interval
                await asyncio.sleep(self.update_interval)
                
            except Exception as e:
                logger.error(f"Communication loop error: {e}")
                await asyncio.sleep(0.1)  # Brief pause before retry
                
        # Stop robot program when communication ends
        await self._stop_robot_program()
        
    async def _read_joint_positions(self):
        """Read current joint positions from robot."""
        try:
            current_joints = []
            for i in range(1, 8):
                value = await self.joint_read_nodes[i].read_value()
                current_joints.append(float(value))
                
            self.shared_state.update_current_joints(current_joints)
            
        except Exception as e:
            logger.error(f"Failed to read joint positions: {e}")
            
    async def _write_joint_positions(self):
        """Write target joint positions to robot."""
        try:
            target_joints = self.shared_state.get_target_joints()
            
            for i in range(1, 8):
                # Convert to radians if needed (KUKA typically uses radians)
                value = ua.Variant(float(target_joints[i-1]), ua.VariantType.Double)
                await self.joint_write_nodes[i].write_value(value)
                
        except Exception as e:
            logger.error(f"Failed to write joint positions: {e}")
            
    async def _read_robot_status(self):
        """Read robot status information."""
        try:
            status = await self.control_nodes['status'].read_value()
            self.shared_state.update_robot_status({
                'connected': True,
                'status_code': int(status)
            })
            
        except Exception as e:
            logger.error(f"Failed to read robot status: {e}")
            self.shared_state.update_robot_status({
                'connected': False,
                'status_code': -1
            })


# Test function for standalone testing
async def test_opc_connection():
    """Test OPC UA connection to robot."""
    shared_state = SharedState()
    robot_comm = RobotCommunication(shared_state)
    
    # Set some test joint positions
    test_joints = [0.0, 0.5, 0.0, -0.5, 0.0, 0.5, 0.0]
    shared_state.update_target_joints(test_joints)
    
    try:
        async with Client(url=robot_comm.url) as client:
            root = client.get_root_node()
            objects = await root.get_child([robot_comm.objects_name])
            robot = await objects.get_child([robot_comm.robot_name])
            
            # Try to read a joint position
            joint1 = await robot.get_child(["R1d_Joi1"])
            value = await joint1.read_value()
            logger.info(f"Joint 1 position: {value}")
            
            logger.info("OPC UA connection test successful")
            
    except Exception as e:
        logger.error(f"OPC UA connection test failed: {e}")


if __name__ == "__main__":
    # Run connection test
    asyncio.run(test_opc_connection())