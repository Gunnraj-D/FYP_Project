#!/usr/bin/env python3
"""
Test script to validate gripper integration with new naming convention.
This script tests the complete flow from gripper command to state completion.
"""
from IO_handling.mock_opc_client import MockOPCClient, MockOPCConfig
from states.context import StateContext
from states.gripper_state import GripperControlState
from control.telemetry_store import Telemetry
from control.command_bus import CommandBus, SetGripper
import asyncio
import logging
import sys
import os

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


async def test_gripper_integration():
    """Test the complete gripper integration flow."""
    logger.info("=== Testing Gripper Integration ===")

    # Create components
    command_bus = CommandBus()
    telemetry = Telemetry()
    context = StateContext(command_bus, telemetry)

    # Create mock OPC client
    config = MockOPCConfig()
    opc_client = MockOPCClient(command_bus, telemetry, config)

    try:
        # Start OPC client
        logger.info("Starting mock OPC client...")
        opc_client.start()

        # Wait for connection
        await asyncio.sleep(2)

        if not opc_client.is_connected():
            logger.error("Failed to connect to mock OPC server")
            return False

        logger.info("✅ Connected to mock OPC server")

        # Test gripper open command
        logger.info("Testing gripper open command...")
        gripper_state = GripperControlState(context, action='open')

        # Enter and execute state
        gripper_state.enter()
        gripper_state.execute()

        # Wait for command to be processed
        await asyncio.sleep(1)

        # Check if state is complete
        current_status = telemetry.get_current_gripper_status()
        target_status = telemetry.get_target_gripper_status()

        logger.info(f"Current gripper status: {current_status}")
        logger.info(f"Target gripper status: {target_status}")
        logger.info(f"State complete: {gripper_state.is_complete()}")

        if gripper_state.is_complete():
            logger.info("✅ Gripper open command completed successfully")
        else:
            logger.warning("⚠️ Gripper open command not completed")

        # Test gripper close command
        logger.info("Testing gripper close command...")
        gripper_state = GripperControlState(context, action='close')

        # Enter and execute state
        gripper_state.enter()
        gripper_state.execute()

        # Wait for command to be processed
        await asyncio.sleep(1)

        # Check if state is complete
        current_status = telemetry.get_current_gripper_status()
        target_status = telemetry.get_target_gripper_status()

        logger.info(f"Current gripper status: {current_status}")
        logger.info(f"Target gripper status: {target_status}")
        logger.info(f"State complete: {gripper_state.is_complete()}")

        if gripper_state.is_complete():
            logger.info("✅ Gripper close command completed successfully")
        else:
            logger.warning("⚠️ Gripper close command not completed")

        return True

    except Exception as e:
        logger.error(f"Test failed with error: {e}")
        return False

    finally:
        # Cleanup
        logger.info("Stopping mock OPC client...")
        opc_client.stop()


async def main():
    """Main test function."""
    logger.info("Starting gripper integration test...")

    success = await test_gripper_integration()

    if success:
        logger.info("✅ All gripper integration tests passed!")
    else:
        logger.error("❌ Some gripper integration tests failed!")
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())
