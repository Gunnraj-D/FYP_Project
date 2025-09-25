"""
Startup script for debug mode with OPC mode selection.
Demonstrates how to use both real and mock OPC modes.
"""
from IO_handling.mock_opc_server import MockOPCServer
from main_debug import DebugSystemManager
import sys
import os
import asyncio
import logging

# Add the src directory to the path so we can import modules
sys.path.append(os.path.join(os.path.dirname(__file__)))


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


async def start_mock_server():
    """Start the mock OPC UA server."""
    try:
        server = MockOPCServer(url="opc.tcp://127.0.0.1:4840/")
        await server.initialize()
        await server.start_server()
        logger.info("✅ Mock OPC UA server started")
        return server
    except Exception as e:
        logger.error(f"❌ Failed to start mock server: {e}")
        return None


def run_debug_mode(opc_mode: str, start_mock_server_flag: bool = False):
    """Run debug mode with specified OPC mode."""
    logger.info(f"🚀 Starting debug mode with OPC mode: {opc_mode}")

    if opc_mode == "mock" and start_mock_server_flag:
        logger.info("📡 Starting mock server...")
        # Start mock server in background
        import threading
        import asyncio

        def run_server():
            asyncio.run(start_mock_server())

        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()

        # Wait for server to start
        import time
        time.sleep(2)

    # Create and run debug system manager
    manager = DebugSystemManager(opc_mode=opc_mode)

    try:
        # Initialize system
        if not manager.initialize():
            logger.error("Failed to initialize debug system")
            return False

        # Start system
        if not manager.start_system():
            logger.error("Failed to start debug system")
            return False

        # Run interactive debug mode
        manager.interactive_debug_mode()
        return True

    except Exception as e:
        logger.error(f"Debug system error: {e}")
        return False
    finally:
        manager.stop_system()


def main():
    """Main function with OPC mode selection."""
    print("🎮 Configurable Debug Mode for Robot Control System")
    print("=" * 50)

    if len(sys.argv) > 1:
        # Command line argument provided
        opc_mode = sys.argv[1].lower()
        if opc_mode not in ["real", "mock"]:
            print("❌ Invalid OPC mode. Must be 'real' or 'mock'")
            return 1
    else:
        # Interactive mode selection
        print("Select OPC mode:")
        print("1. Real OPC UA client (connects to actual robot)")
        print("2. Mock OPC UA client (simulation mode)")

        while True:
            try:
                choice = input("Enter choice (1 or 2): ").strip()
                if choice == "1":
                    opc_mode = "real"
                    break
                elif choice == "2":
                    opc_mode = "mock"
                    break
                else:
                    print("❌ Invalid choice. Please enter 1 or 2.")
            except KeyboardInterrupt:
                print("\n👋 Goodbye!")
                return 0

    print(f"\n🎯 Selected OPC mode: {opc_mode}")

    if opc_mode == "mock":
        print("📡 Mock mode will simulate robot behavior with instantaneous movement")
        print("🔧 Make sure no other application is using port 4840")

        # Ask if user wants to start mock server automatically
        start_server = input(
            "Start mock server automatically? (y/n): ").strip().lower()
        start_server_flag = start_server in ['y', 'yes']
    else:
        print("🤖 Real mode will connect to actual robot hardware")
        print("⚠️  Make sure robot is properly connected and OPC server is running")
        start_server_flag = False

    print("\n🚀 Starting debug system...")

    try:
        success = run_debug_mode(opc_mode, start_server_flag)
        if success:
            print("\n✅ Debug session completed successfully!")
            return 0
        else:
            print("\n❌ Debug session failed!")
            return 1
    except KeyboardInterrupt:
        print("\n👋 Debug session interrupted by user")
        return 0
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        return 1


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
