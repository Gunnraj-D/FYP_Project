"""
Test script for configurable OPC UA client.
Tests both real and mock OPC modes to ensure they work correctly.
"""
from control.telemetry_store import Telemetry
from control.command_bus import CommandBus
from IO_handling.opc_client_factory import OPCClientFactory, OPCConfig
import asyncio
import logging
import sys
import os
import time

# Add the src directory to the path so we can import modules
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class ConfigurableOPCTest:
    """Test class for configurable OPC UA client."""

    def __init__(self):
        self.command_bus = CommandBus()
        self.telemetry = Telemetry()

    def test_opc_mode_validation(self):
        """Test OPC mode validation."""
        logger.info("=== Testing OPC Mode Validation ===")

        # Test valid modes
        assert OPCClientFactory.validate_mode("real") == True
        assert OPCClientFactory.validate_mode("mock") == True
        assert OPCClientFactory.validate_mode(
            "REAL") == True  # Case insensitive
        assert OPCClientFactory.validate_mode("MOCK") == True

        # Test invalid modes
        assert OPCClientFactory.validate_mode("invalid") == False
        assert OPCClientFactory.validate_mode("") == False

        logger.info("✓ Mode validation tests passed")

    def test_config_creation(self):
        """Test configuration creation for different modes."""
        logger.info("=== Testing Configuration Creation ===")

        # Test real mode config
        real_config = OPCClientFactory.create_config_from_mode("real")
        assert real_config.url == "opc.tcp://172.24.200.1:4840/"
        logger.info(f"✓ Real mode config: {real_config.url}")

        # Test mock mode config
        mock_config = OPCClientFactory.create_config_from_mode("mock")
        assert mock_config.url == "opc.tcp://127.0.0.1:4840/"
        logger.info(f"✓ Mock mode config: {mock_config.url}")

        # Test default mode config
        default_config = OPCClientFactory.create_config_from_mode()
        logger.info(f"✓ Default mode config: {default_config.url}")

    def test_mock_client_creation(self):
        """Test mock OPC client creation."""
        logger.info("=== Testing Mock OPC Client Creation ===")

        try:
            config = OPCClientFactory.create_config_from_mode("mock")
            client = OPCClientFactory.create_client(
                self.command_bus, self.telemetry, config, "mock"
            )

            logger.info(f"✓ Mock client created: {type(client).__name__}")
            logger.info(f"✓ Client type: {client.__class__.__name__}")

            # Test client methods exist
            assert hasattr(client, 'start')
            assert hasattr(client, 'stop')
            assert hasattr(client, 'is_connected')
            assert hasattr(client, 'get_status')

            logger.info("✓ Mock client has required methods")

        except Exception as e:
            logger.error(f"❌ Failed to create mock client: {e}")
            raise

    def test_real_client_creation(self):
        """Test real OPC client creation."""
        logger.info("=== Testing Real OPC Client Creation ===")

        try:
            config = OPCClientFactory.create_config_from_mode("real")
            client = OPCClientFactory.create_client(
                self.command_bus, self.telemetry, config, "real"
            )

            logger.info(f"✓ Real client created: {type(client).__name__}")
            logger.info(f"✓ Client type: {client.__class__.__name__}")

            # Test client methods exist
            assert hasattr(client, 'start')
            assert hasattr(client, 'stop')
            assert hasattr(client, 'is_connected')
            assert hasattr(client, 'get_status')

            logger.info("✓ Real client has required methods")

        except Exception as e:
            logger.error(f"❌ Failed to create real client: {e}")
            raise

    def test_convenience_function(self):
        """Test the convenience function for client creation."""
        logger.info("=== Testing Convenience Function ===")

        from IO_handling.opc_client_factory import create_opc_client

        # Test mock client creation
        mock_client = create_opc_client(
            self.command_bus, self.telemetry, mode="mock"
        )
        logger.info(
            f"✓ Convenience function mock client: {type(mock_client).__name__}")

        # Test real client creation
        real_client = create_opc_client(
            self.command_bus, self.telemetry, mode="real"
        )
        logger.info(
            f"✓ Convenience function real client: {type(real_client).__name__}")

    def test_available_modes(self):
        """Test getting available modes."""
        logger.info("=== Testing Available Modes ===")

        modes = OPCClientFactory.get_available_modes()
        logger.info(f"✓ Available modes: {modes}")

        assert "real" in modes
        assert "mock" in modes
        assert len(modes) == 2

    def run_all_tests(self):
        """Run all tests."""
        try:
            logger.info("🧪 Starting Configurable OPC UA Tests...")

            self.test_opc_mode_validation()
            self.test_config_creation()
            self.test_mock_client_creation()
            self.test_real_client_creation()
            self.test_convenience_function()
            self.test_available_modes()

            logger.info("✅ All configurable OPC UA tests passed!")
            return True

        except Exception as e:
            logger.error(f"❌ Test failed: {e}")
            return False


async def test_mock_server_integration():
    """Test integration with mock server."""
    logger.info("=== Testing Mock Server Integration ===")

    try:
        from IO_handling.mock_opc_server import MockOPCServer

        # Start mock server
        server = MockOPCServer(url="opc.tcp://127.0.0.1:4840/")
        await server.initialize()
        await server.start_server()

        logger.info("✓ Mock server started")

        # Create mock client
        command_bus = CommandBus()
        telemetry = Telemetry()

        client = OPCClientFactory.create_client(
            command_bus, telemetry, mode="mock"
        )

        logger.info("✓ Mock client created")

        # Start client
        client.start()
        time.sleep(2)  # Wait for connection

        # Test connection
        if client.is_connected():
            logger.info("✓ Mock client connected to mock server")
        else:
            logger.warning("⚠ Mock client not connected")

        # Test sending commands
        from control.command_bus import SetJoints
        command_bus.send(SetJoints(joints=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]))
        time.sleep(0.5)

        # Check telemetry
        current_joints = telemetry.get_current_joints()
        if current_joints:
            logger.info(f"✓ Joint positions updated: {current_joints}")
        else:
            logger.warning("⚠ No joint positions received")

        # Cleanup
        client.stop()
        await server.stop_server()

        logger.info("✅ Mock server integration test completed")
        return True

    except Exception as e:
        logger.error(f"❌ Mock server integration test failed: {e}")
        return False


def main():
    """Main test function."""
    # Test client creation and configuration
    test = ConfigurableOPCTest()
    success = test.run_all_tests()

    if success:
        logger.info("\n🔄 Testing mock server integration...")
        # Test mock server integration
        integration_success = asyncio.run(test_mock_server_integration())

        if integration_success:
            logger.info("\n🎉 All tests completed successfully!")
            return 0
        else:
            logger.error("\n❌ Integration test failed!")
            return 1
    else:
        logger.error("\n❌ Basic tests failed!")
        return 1


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
