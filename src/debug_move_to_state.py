#!/usr/bin/env python3
"""
Debug script to test MoveToState behavior in mock mode.
"""
from states.context import StateContext
from states.move_to_state import MoveToState
from integrated_robot_control_system import IntegratedRobotControlSystem
import logging
import time
import sys
import os
from pathlib import Path

# Add the project root to Python path
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))


# Configure logging
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


def test_move_to_state():
    """Test MoveToState in mock mode."""
    print("🚀 Testing MoveToState in mock mode...")

    # Start mock server first
    print("🚀 Starting mock OPC UA server...")
    import asyncio
    import threading
    from IO_handling.mock_opc_server import MockOPCServer

    mock_server = None
    mock_server_thread = None
    shutdown_event = threading.Event()

    def run_server():
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

            async def start_server():
                nonlocal mock_server
                mock_server = MockOPCServer(url="opc.tcp://127.0.0.1:4841/")
                await mock_server.initialize()
                await mock_server.start_server()
                print("✅ Mock OPC UA server ready")

                # Keep the server running
                try:
                    while mock_server and not shutdown_event.is_set():
                        await asyncio.sleep(0.1)
                except Exception as e:
                    print(f"❌ Server loop error: {e}")
                finally:
                    if mock_server:
                        await mock_server.stop_server()
                        print("✅ Mock OPC UA server stopped")

            loop.run_until_complete(start_server())
            loop.close()
        except Exception as e:
            print(f"❌ Mock server thread error: {e}")

    mock_server_thread = threading.Thread(
        target=run_server, name="MockOPCServer", daemon=True)
    mock_server_thread.start()

    # Wait for server to start
    time.sleep(3)
    print("✅ Mock server ready")

    # Initialize system in mock mode
    system = IntegratedRobotControlSystem(opc_mode="mock")

    # Create context
    context = StateContext(
        telemetry=system.telemetry,
        commands=system.command_bus,
        camera=system.camera_manager,
        opc=system.opc_client,
        ik=system.kinematics_solver
    )

    # Start the system (skip camera for debug)
    print("🔄 Starting system...")
    try:
        system.start()
    except RuntimeError as e:
        if "camera" in str(e).lower():
            print("⚠️ Camera initialization failed, continuing without camera...")
            # Manually start OPC client
            system.opc_client.start()
            time.sleep(2)
        else:
            raise

    # Wait for OPC client to initialize
    time.sleep(2)

    # Check initial joint values
    initial_joints = context.telemetry.get_current_joints()
    print(f"📊 Initial joints: {initial_joints}")

    # Create MoveToState
    target_location = (0.5, 0.0, 0.6)
    move_state = MoveToState(context, target_location=target_location)

    print(f"🎯 Target location: {target_location}")

    # Enter the state
    print("🔄 Entering MoveToState...")
    move_state.enter()

    # Execute the state
    print("🔄 Executing MoveToState...")
    move_state.execute()

    # Check target joint angles
    print(f"🎯 Target joint angles: {move_state.target_joint_angles}")

    # Check if command was sent
    print(f"📤 Commands in queue: {len(context.commands.recv_all_pending())}")

    # Wait a bit for OPC client to process the command
    print("⏳ Waiting for OPC client to process command...")
    time.sleep(1)

    # Check current joints after execution
    current_joints = context.telemetry.get_current_joints()
    print(f"📊 Current joints after execution: {current_joints}")

    # Test completion check
    print("🔄 Testing completion check...")
    is_complete = move_state.is_complete()
    print(f"✅ Is complete: {is_complete}")

    # If not complete, wait and check again
    if not is_complete:
        print("⏳ Waiting for completion...")
        for i in range(10):
            time.sleep(0.5)
            current_joints = context.telemetry.get_current_joints()
            is_complete = move_state.is_complete()
            print(f"📊 Iteration {i+1}: Current joints: {current_joints}")
            print(f"✅ Is complete: {is_complete}")
            if is_complete:
                break

    # Stop the system
    print("🛑 Stopping system...")
    system.stop()

    # Stop mock server
    print("🛑 Stopping mock server...")
    shutdown_event.set()
    if mock_server_thread and mock_server_thread.is_alive():
        mock_server_thread.join(timeout=3.0)
        if mock_server_thread.is_alive():
            print("⚠️ Mock server thread did not stop gracefully")

    print("✅ Test completed")


if __name__ == "__main__":
    test_move_to_state()
