"""
Simple startup script for the mock OPC UA environment.
Starts both the mock server and runs a basic test.
"""
import asyncio
import logging
import threading
import time
from IO_handling.mock_opc_server import MockOPCServer
from examples.mock_opc_test import MockOPCTest

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


async def start_mock_environment():
    """Start the mock OPC UA environment with basic testing."""
    try:
        logger.info("🚀 Starting Mock OPC UA Environment...")

        # Start the mock server
        server = MockOPCServer(url="opc.tcp://127.0.0.1:4840/")
        await server.initialize()
        await server.start_server()

        logger.info("✅ Mock OPC UA Server started successfully!")
        logger.info("📡 Server running at: opc.tcp://127.0.0.1:4840/")

        # Run a quick test
        logger.info("🧪 Running basic functionality test...")
        test = MockOPCTest()
        test.server = server  # Use the already running server

        # Start client
        test.client.start()
        await asyncio.sleep(2.0)  # Wait for connection

        # Quick joint movement test
        logger.info("🦾 Testing joint movement...")
        test.command_bus.send(test.command_bus.SetJoints(
            joints=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]))
        await asyncio.sleep(0.5)

        current_joints = test.telemetry.get_current_joints()
        logger.info(f"📍 Current joint positions: {current_joints}")

        # Test emergency stop
        logger.info("🛑 Testing emergency stop...")
        test.command_bus.send(test.command_bus.EmergencyStop(active=True))
        await asyncio.sleep(0.5)

        emergency_status = test.telemetry.get_emergency_stop()
        logger.info(f"🚨 Emergency stop status: {emergency_status}")

        logger.info("✅ Basic test completed successfully!")
        logger.info("🎯 Mock environment is ready for Unity integration!")
        logger.info("📋 Available nodes for Unity:")
        logger.info("   Joint Current: R1d_Joi1-7 (ns=2;i=2001-2007)")
        logger.info("   Joint Target:  R1c_Joi1-7 (ns=2;i=1001-1007)")
        logger.info("   Control:       R1c_Start, R1c_ProgID, R1d_Status")

        # Keep running
        logger.info("🔄 Environment running... Press Ctrl+C to stop.")
        while server.is_running():
            await asyncio.sleep(1)

    except KeyboardInterrupt:
        logger.info("🛑 Received shutdown signal")
    except Exception as e:
        logger.error(f"❌ Error: {e}")
    finally:
        if 'test' in locals() and test.client:
            test.client.stop()
        if 'server' in locals():
            await server.stop_server()
        logger.info("👋 Mock environment stopped")


if __name__ == "__main__":
    asyncio.run(start_mock_environment())
