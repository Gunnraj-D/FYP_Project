"""
Test script for the mock OPC UA environment.
Demonstrates the mock server and client working together with instantaneous movement simulation.
"""
import asyncio
import logging
import time
import threading
from typing import List

# Import the mock components
from IO_handling.mock_opc_server import MockOPCServer
from IO_handling.mock_opc_client import MockOPCClient, MockOPCConfig
from control.command_bus import CommandBus, SetJoints, SetGripper, EmergencyStop
from control.telemetry_store import Telemetry

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class MockOPCTest:
    """Test class for the mock OPC UA environment."""

    def __init__(self):
        self.server: MockOPCServer = None
        self.client: MockOPCClient = None
        self.command_bus = CommandBus()
        self.telemetry = Telemetry()
        self.server_task = None

    async def setup_mock_environment(self):
        """Set up the mock OPC UA server and client."""
        try:
            # Create and initialize mock server
            self.server = MockOPCServer(url="opc.tcp://127.0.0.1:4840/")
            await self.server.initialize()
            await self.server.start_server()

            # Create mock client with custom config
            config = MockOPCConfig(
                url="opc.tcp://127.0.0.1:4840/",
                poll_interval_ms=100,  # 10Hz for testing
                command_batch_size=5,
                skip_redundant_writes=True
            )

            self.client = MockOPCClient(
                command_bus=self.command_bus,
                telemetry=self.telemetry,
                config=config
            )

            logger.info("Mock OPC UA environment setup complete")

        except Exception as e:
            logger.error(f"Failed to setup mock environment: {e}")
            raise

    async def test_joint_movement(self):
        """Test joint movement with instantaneous simulation."""
        logger.info("=== Testing Joint Movement ===")

        # Test sequence of joint positions
        test_positions = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Home position
            [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],  # Test position 1
            [0.5, 0.4, 0.3, 0.2, 0.1, 0.0, -0.1],  # Test position 2
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Return to home
        ]

        for i, positions in enumerate(test_positions):
            logger.info(f"Moving to position {i+1}: {positions}")

            # Send joint command
            self.command_bus.send(SetJoints(joints=positions))

            # Wait for command to be processed
            await asyncio.sleep(0.2)

            # Check if movement was instantaneous
            current_joints = self.telemetry.get_current_joints()
            logger.info(f"Current joints: {current_joints}")

            # Verify instantaneous movement (current should equal target)
            if current_joints:
                differences = [abs(current_joints[j] - positions[j])
                               for j in range(7)]
                max_diff = max(differences)
                logger.info(f"Max difference: {max_diff:.6f}")

                if max_diff < 1e-6:
                    logger.info("✓ Instantaneous movement confirmed")
                else:
                    logger.warning("⚠ Movement not instantaneous")

            await asyncio.sleep(1.0)  # Wait between movements

    async def test_gripper_control(self):
        """Test gripper control commands."""
        logger.info("=== Testing Gripper Control ===")

        # Test gripper commands
        gripper_commands = ["open", "close", "open"]

        for command in gripper_commands:
            logger.info(f"Sending gripper command: {command}")
            self.command_bus.send(SetGripper(status=command))
            await asyncio.sleep(0.5)

    async def test_emergency_stop(self):
        """Test emergency stop functionality."""
        logger.info("=== Testing Emergency Stop ===")

        # Send some joint commands
        self.command_bus.send(
            SetJoints(joints=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]))
        await asyncio.sleep(0.1)

        # Activate emergency stop
        logger.info("Activating emergency stop...")
        self.command_bus.send(EmergencyStop(active=True))
        await asyncio.sleep(0.5)

        # Check emergency stop status
        emergency_stop = self.telemetry.get_emergency_stop()
        logger.info(f"Emergency stop status: {emergency_stop}")

        # Deactivate emergency stop
        logger.info("Deactivating emergency stop...")
        self.command_bus.send(EmergencyStop(active=False))
        await asyncio.sleep(0.5)

    async def test_telemetry_updates(self):
        """Test telemetry updates from the mock server."""
        logger.info("=== Testing Telemetry Updates ===")

        # Send a joint command
        test_joints = [0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
        self.command_bus.send(SetJoints(joints=test_joints))

        # Wait for updates
        await asyncio.sleep(0.5)

        # Check telemetry
        current_joints = self.telemetry.get_current_joints()
        robot_status = self.telemetry.get_robot_status()

        logger.info(f"Current joints from telemetry: {current_joints}")
        logger.info(f"Robot status from telemetry: {robot_status}")

    async def test_server_status(self):
        """Test server status and joint positions."""
        logger.info("=== Testing Server Status ===")

        # Get server status
        status = self.server.get_robot_status()
        logger.info(f"Server robot status: {status}")

        # Get joint positions from server
        joint_positions = self.server.get_joint_positions()
        logger.info(f"Server joint positions: {joint_positions}")

    async def run_comprehensive_test(self):
        """Run comprehensive test of the mock OPC UA environment."""
        try:
            logger.info("Starting comprehensive mock OPC UA test...")

            # Setup environment
            await self.setup_mock_environment()

            # Start client
            self.client.start()

            # Wait for connection
            await asyncio.sleep(2.0)

            # Run tests
            await self.test_joint_movement()
            await self.test_gripper_control()
            await self.test_emergency_stop()
            await self.test_telemetry_updates()
            await self.test_server_status()

            logger.info("✓ All tests completed successfully!")

        except Exception as e:
            logger.error(f"Test failed: {e}")
        finally:
            # Cleanup
            if self.client:
                self.client.stop()
            if self.server:
                await self.server.stop_server()

    def run_test_in_thread(self):
        """Run the test in a separate thread."""
        def run_async():
            asyncio.run(self.run_comprehensive_test())

        thread = threading.Thread(target=run_async)
        thread.start()
        return thread


async def main():
    """Main test function."""
    test = MockOPCTest()
    await test.run_comprehensive_test()


if __name__ == "__main__":
    asyncio.run(main())
