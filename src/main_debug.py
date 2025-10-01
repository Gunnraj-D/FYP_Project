"""
Debug version of the main integrated robot control system.
Allows interactive selection and execution of individual states and task sequencers.
"""
import warnings
from config.config import PRE_PICKUP_POSE, HANDOFF_APPROACH_POSE, ROBOT_ID
from states.placement_task_sequencer import PlacementTaskSequencer, create_placement_sequencer
from states.pickup_task_sequencer import PickupTaskSequencer, create_pickup_sequencer
from states.grasping_state import GraspingState
from states.unified_hand_tracking_state import UnifiedHandTrackingState
from states.gripper_state import GripperControlState
from states.move_to_state import MoveToState
from states.base_state import BaseState
from states.state_machine import StateMachine
from states.context import StateContext
from integrated_robot_control_system import IntegratedRobotControlSystem, SystemMode
import os
import time
import logging
import signal
import sys
import threading
import asyncio
from typing import Optional, Dict, List

# Suppress TensorFlow warnings
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
os.environ['TF_ENABLE_ONEDNN_OPTS'] = '0'


# Configure logging with reduced verbosity for debug mode
logging.basicConfig(
    level=logging.ERROR,  # Only show errors
    format='%(asctime)s - %(levelname)s - %(message)s',
    force=True  # Force reconfiguration to ensure thread logs appear
)

# Set specific loggers to reduce noise
logging.getLogger('asyncua').setLevel(logging.ERROR)
logging.getLogger('opcua').setLevel(logging.ERROR)
logging.getLogger('asyncio').setLevel(logging.ERROR)
logging.getLogger('IO_handling.mock_opc_server').setLevel(logging.INFO)
logging.getLogger('IO_handling.mock_opc_client').setLevel(logging.INFO)
logging.getLogger('camera_management.camera_manager').setLevel(logging.ERROR)
logging.getLogger(
    'hand_detection.hand_detection_module').setLevel(logging.ERROR)
logging.getLogger('object_detection.ggcnn2_module').setLevel(logging.INFO)
logging.getLogger('kinematics.kinematics_solver').setLevel(logging.INFO)
logging.getLogger('states.grasping_state').setLevel(logging.INFO)
logging.getLogger('integrated_robot_control_system').setLevel(logging.ERROR)
logging.getLogger('states.task_orchestrator').setLevel(logging.ERROR)
# Allow MoveToState INFO logs like the IK solver
logging.getLogger('states.move_to_state').setLevel(logging.INFO)
logging.getLogger('control.telemetry_store').setLevel(logging.ERROR)
logging.getLogger('IO_handling.opc_client_factory').setLevel(logging.ERROR)

# Suppress additional noisy loggers
warnings.filterwarnings("ignore", category=UserWarning,
                        module="google.protobuf")
warnings.filterwarnings("ignore", category=FutureWarning)
warnings.filterwarnings("ignore", message=".*TensorFlow.*")
warnings.filterwarnings("ignore", message=".*protobuf.*")
warnings.filterwarnings("ignore", message=".*deprecated.*")
logger = logging.getLogger(__name__)


class DebugSystemManager:
    """
    Debug manager for interactive state and sequencer execution.
    """

    def __init__(self, opc_mode: str = None):
        self.opc_mode = opc_mode
        self.system: Optional[IntegratedRobotControlSystem] = None
        self.context: Optional[StateContext] = None
        self.state_machine: Optional[StateMachine] = None
        self.running = False
        self.current_execution_thread: Optional[threading.Thread] = None
        self.execution_active = False
        self.force_complete = False

        # Mock server management
        self.mock_server = None
        self.mock_server_thread: Optional[threading.Thread] = None
        self._shutdown_event = threading.Event()

    def start_mock_server(self):
        """Start mock server in background thread with proper event loop."""
        if self.opc_mode == "mock" and self.mock_server is None:
            print("🚀 Starting mock OPC UA server...")

            # Create a new event loop for the server thread
            def run_server():
                try:
                    loop = asyncio.new_event_loop()
                    asyncio.set_event_loop(loop)

                    async def start_server():
                        from IO_handling.mock_opc_server import MockOPCServer
                        self.mock_server = MockOPCServer(
                            url="opc.tcp://127.0.0.1:4840/", robot_id=ROBOT_ID)
                        await self.mock_server.initialize()
                        await self.mock_server.start_server()
                        print("✅ Mock OPC UA server ready")

                        # Keep the server running
                        try:
                            while self.mock_server and not self._shutdown_event.is_set():
                                await asyncio.sleep(0.1)
                        except Exception as e:
                            print(f"❌ Server loop error: {e}")
                        finally:
                            if self.mock_server:
                                await self.mock_server.stop_server()
                                print("✅ Mock OPC UA server stopped")

                    loop.run_until_complete(start_server())
                    loop.close()

                except Exception as e:
                    print(f"❌ Mock server thread error: {e}")

            self.mock_server_thread = threading.Thread(
                target=run_server,
                name="MockOPCServer",
                daemon=True
            )
            self.mock_server_thread.start()

            # Wait for server to start and be ready
            time.sleep(3)
            print("✅ Mock server ready")

    async def stop_mock_server(self):
        """Stop the mock OPC UA server."""
        if self.mock_server:
            try:
                self._shutdown_event.set()
                await self.mock_server.stop_server()
                logger.info("✅ Mock OPC UA server stopped")
            except Exception as e:
                print(f"❌ Error stopping mock server: {e}")
            finally:
                self.mock_server = None

    def initialize(self) -> bool:
        """Initialize the robot control system."""
        try:
            logger.info(
                f"🚀 Initializing Debug Robot Control System in '{self.opc_mode}' mode...")
            self.system = IntegratedRobotControlSystem(opc_mode=self.opc_mode)

            # Get the context from the system
            self.context = StateContext(
                telemetry=self.system.telemetry,
                commands=self.system.command_bus,
                camera=self.system.camera_manager,
                opc=self.system.opc_client,
                ik=self.system.kinematics_solver
            )

            # Create a state machine for debug execution
            self.state_machine = StateMachine(
                initial_state=MoveToState(
                    self.context, target_location=(0.5, 0.0, 0.6)),  # Reasonable position in meters
                on_state_completion=self._on_state_completion
            )

            print("✅ Debug system initialized successfully")
            return True
        except Exception as e:
            print(f"❌ Failed to initialize debug system: {e}")
            return False

    def start_system(self):
        """Start the robot control system."""
        if not self.system:
            print("❌ System not initialized")
            return False

        try:
            print("🚀 Starting debug robot control system...")

            # Start mock server if in mock mode
            if self.opc_mode == "mock":
                self.start_mock_server()

            # Initialize camera (optional for debug mode)
            try:
                if not self.system.camera_manager.initialize():
                    print("⚠️ Camera initialization failed - continuing without camera")
            except Exception as e:
                print(
                    f"⚠️ Camera not available")
                return False

            # # Start hand tracker (optional for debug mode)
            # try:
            #     self.system.hand_tracker.start()
            # except Exception as e:
            #     print(
            #         f"⚠️ Hand tracker not available - continuing without hand tracking: {e}")

            # Start OPC UA communication
            if self.opc_mode == "mock":
                # Give mock server a bit more time to be fully ready
                time.sleep(2)
                # Test server connectivity
                import socket
                try:
                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    sock.settimeout(5)
                    result = sock.connect_ex(('127.0.0.1', 4840))
                    sock.close()
                    if result == 0:
                        print("✅ Mock server ready")
                    else:
                        print(f"⚠️ Mock server not ready: {result}")
                except Exception as e:
                    print(f"⚠️ Could not test server connectivity: {e}")

            self.system.opc_client.start()

            # Give OPC client time to initialize joint values
            if self.opc_mode == "mock":
                time.sleep(1)
                print("✅ OPC client initialized")

            # Set initial mode to idle
            self.system.set_mode(SystemMode.IDLE)

            self.running = True
            print("✅ Debug system started successfully")
            return True

        except Exception as e:
            print(f"❌ Failed to start debug system: {e}")
            self.stop_system()
            return False

    def stop_system(self):
        """Stop the robot control system."""
        if self.system:
            print("🛑 Stopping debug robot control system...")
            try:
                self.system.stop()
            except Exception as e:
                print(f"⚠️ Error stopping system: {e}")
            self.running = False

        # Stop mock server if running
        if self.mock_server:
            try:
                self._shutdown_event.set()
                # Wait for server thread to finish
                if self.mock_server_thread and self.mock_server_thread.is_alive():
                    self.mock_server_thread.join(
                        timeout=2.0)  # Reduced timeout
                    if self.mock_server_thread.is_alive():
                        print("⚠️ Mock server thread did not stop gracefully")
                        # Force terminate if needed
                        self.mock_server_thread = None
            except Exception as e:
                print(f"❌ Error stopping mock server: {e}")

        # Force cleanup of any remaining resources
        self.mock_server = None
        self.mock_server_thread = None

        print("✅ Debug system stopped")

    def _on_state_completion(self):
        """Callback when a state completes."""
        logger.info("State completed")

    def get_available_states(self) -> Dict[int, BaseState]:
        """Get dictionary of available states for execution."""
        states = {
            # Reasonable position in workspace
            1: MoveToState(self.context, target_location=(0.5, 0.0, 0.6)),
            # Another reasonable position
            2: MoveToState(self.context, target_location=(0.6, 0.3, 0.5)),
            3: GripperControlState(self.context, action='open'),
            4: GripperControlState(self.context, action='close'),
            5: UnifiedHandTrackingState(self.context),
            6: GraspingState(self.context),
        }
        return states

    def get_available_sequencers(self) -> Dict[int, str]:
        """Get dictionary of available task sequencers."""
        sequencers = {
            1: "Pickup Task Sequencer",
            2: "Placement Task Sequencer",
        }
        return sequencers

    def execute_state(self, state: BaseState):
        """Execute a single state until completion."""
        print(f"\n🚀 Executing: {state.name}")

        # Set up the state machine with the selected state
        # Only exit current state if it exists and is not None
        if self.state_machine.current_state is not None:
            try:
                print(
                    f"🔄 First exiting current state: {self.state_machine.current_state.name}")
                self.state_machine.current_state.exit()
            except Exception as e:
                print(f"⚠️ Warning: Error exiting current state: {e}")

        self.state_machine.current_state = state
        print(f"🔄 Entering new state: {state.name}")
        self.state_machine.current_state.enter()

        self.execution_active = True
        self.force_complete = False

        try:
            # First, execute the state to send commands
            self.state_machine.step()

            # Give more time for OPC client to process commands and update telemetry
            # This is especially important for mock mode where commands are processed asynchronously
            time.sleep(0.5)

            # Then wait for completion
            iteration = 0
            while not state.is_complete() and not self.force_complete:
                self.state_machine.step()
                time.sleep(0.1)  # 10Hz execution rate

                # Debug logging every 50 iterations (5 seconds)
                if iteration % 50 == 0:
                    current_joints = self.context.telemetry.get_current_joints()
                    target_joints = getattr(state, 'target_joint_angles', None)
                    print(
                        f"🔄 Debug iteration {iteration}: Current joints: {current_joints}")
                    print(f"🎯 Target joints: {target_joints}")
                    print(f"✅ Is complete: {state.is_complete()}")

                iteration += 1

                # Safety timeout to prevent infinite hanging
                if iteration > 100:  # 10 seconds timeout
                    print("⚠️ Timeout reached, forcing completion")
                    break

                # Check for user input to force completion
                if self._check_for_force_complete():
                    break

        except Exception as e:
            print(f"❌ Error executing state: {e}")
        finally:
            # This block is GUARANTEED to run, ensuring cleanup.
            print(f"Exiting state: {state.name}")
            state.exit()  # This will call hand_tracker.stop()
            self.execution_active = False
            print(f"✅ Completed: {state.name}")

    def execute_sequencer(self, sequencer_type: str):
        """Execute a task sequencer until completion."""
        print(f"\n🚀 Executing: {sequencer_type}")

        # Create the appropriate sequencer
        if sequencer_type == "Pickup Task Sequencer":
            sequencer = create_pickup_sequencer(
                self.state_machine, self.context)
        elif sequencer_type == "Placement Task Sequencer":
            sequencer = create_placement_sequencer(
                self.state_machine, self.context)
        else:
            print(f"❌ Unknown sequencer type: {sequencer_type}")
            return

        # Set the state machine's completion callback to queue the next task
        self.state_machine.on_state_completion = sequencer.queue_next_task

        # Queue the first task to start the sequence
        sequencer.queue_next_task()

        self.execution_active = True
        self.force_complete = False
        last_logged_step = -1

        try:
            while sequencer.task_queue and not self.force_complete:
                sequencer.step()
                time.sleep(0.1)  # 10Hz execution rate

                # Show progress only when step changes
                progress = sequencer.get_progress()
                current_step = progress['current_step']
                if current_step > 0 and current_step != last_logged_step:
                    logger.info(f"Sequencer progress: {current_step}/{progress['total_steps']} "
                                f"({progress['progress_percent']:.1f}%)")
                    last_logged_step = current_step

                # Check for user input to force completion
                if self._check_for_force_complete():
                    break

        except Exception as e:
            print(f"❌ Error executing sequencer: {e}")
        finally:
            self.execution_active = False
            print(f"✅ Completed: {sequencer_type}")

    def _check_for_force_complete(self) -> bool:
        """Check if user wants to force completion (non-blocking)."""
        # This is a simplified check - in a real implementation,
        # you might want to use a more sophisticated input handling
        return self.force_complete

    def force_complete_execution(self):
        """Force completion of current execution."""
        if self.execution_active:
            self.force_complete = True
            print("⚡ Force completion requested")

    def interactive_debug_mode(self):
        """Run interactive debug mode for manual state/sequencer selection."""
        if not self.system or not self.context:
            print("❌ System not initialized")
            return

        print("\n" + "="*50)
        print("🤖 ROBOT DEBUG MODE")
        print("="*50)
        print(f"OPC Mode: {self.opc_mode or 'default from config'}")
        if self.opc_mode == "mock" and self.mock_server:
            print("✅ Mock OPC UA server running")
        print("\nAvailable commands:")
        print("  states - Show available states")
        print("  sequencers - Show available sequencers")
        print("  run <number> - Execute state by number")
        print("  seq <number> - Execute sequencer by number")
        print("  force - Force completion of current execution")
        print("  status - Show system status")
        print("  quit - Exit program")

        while self.running:
            try:
                command = input("\n🤖 Debug> ").strip().lower()

                if command == "quit":
                    print("🛑 Stopping all executions...")
                    self.force_complete_execution()
                    if self.current_execution_thread and self.current_execution_thread.is_alive():
                        print("⚠️ Waiting for execution thread to stop...")
                        self.current_execution_thread.join(timeout=1.0)
                        if self.current_execution_thread.is_alive():
                            print("⚠️ Execution thread did not stop gracefully")
                        self.current_execution_thread = None
                    self.running = False
                    break
                elif command == "states":
                    self._show_available_states()
                elif command == "sequencers":
                    self._show_available_sequencers()
                elif command.startswith("run "):
                    self._execute_state_by_number(command)
                elif command.startswith("seq "):
                    self._execute_sequencer_by_number(command)
                elif command == "force":
                    self.force_complete_execution()
                elif command == "status":
                    self._show_system_status()
                elif command == "help":
                    self._show_help()
                else:
                    print("❌ Unknown command. Type 'help' for available commands.")

            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"❌ Error: {e}")

        print("👋 Exiting debug mode")

    def _show_help(self):
        """Show help information."""
        print("\n" + "="*50)
        print("🤖 ROBOT DEBUG MODE - HELP")
        print("="*50)
        print("Available commands:")
        print("  states - Show available states")
        print("  sequencers - Show available sequencers")
        print("  run <number> - Execute state by number")
        print("  seq <number> - Execute sequencer by number")
        print("  force - Force completion of current execution")
        print("  status - Show system status")
        print("  help - Show this help")
        print("  quit - Exit debug mode")
        print("="*50)

    def _show_available_states(self):
        """Show available states for execution."""
        states = self.get_available_states()
        print("\n📋 Available States:")
        for num, state in states.items():
            print(f"  {num}. {state.name}")

    def _show_available_sequencers(self):
        """Show available sequencers for execution."""
        sequencers = self.get_available_sequencers()
        print("\n📋 Available Sequencers:")
        for num, name in sequencers.items():
            print(f"  {num}. {name}")

    def _execute_state_by_number(self, command: str):
        """Execute a state by its number."""
        try:
            parts = command.split()
            if len(parts) != 2:
                print("❌ Usage: run <number>")
                return

            state_num = int(parts[1])

            # Create fresh state instance based on state number
            if state_num == 1:
                state = MoveToState(
                    self.context, target_location=(0.5, 0.0, 0.6))
            elif state_num == 2:
                state = MoveToState(
                    self.context, target_location=(0.6, 0.3, 0.5))
            elif state_num == 3:
                state = GripperControlState(self.context, action='open')
            elif state_num == 4:
                state = GripperControlState(self.context, action='close')
            elif state_num == 5:
                state = UnifiedHandTrackingState(self.context)
            elif state_num == 6:
                state = GraspingState(self.context)
            else:
                print(f"❌ Invalid state number: {state_num}")
                return

            # Run state execution in a separate thread to avoid blocking
            if self.execution_active:
                logger.warning(
                    "Another execution is already active. Use 'force' to stop it first.")
                return

            self.current_execution_thread = threading.Thread(
                target=self.execute_state, args=(state,)
            )
            self.current_execution_thread.start()

        except ValueError:
            print("❌ Invalid number format")
        except Exception as e:
            print(f"❌ Error executing state: {e}")

    def _execute_sequencer_by_number(self, command: str):
        """Execute a sequencer by its number."""
        try:
            parts = command.split()
            if len(parts) != 2:
                print("❌ Usage: seq <number>")
                return

            seq_num = int(parts[1])
            sequencers = self.get_available_sequencers()

            if seq_num not in sequencers:
                print(f"❌ Invalid sequencer number: {seq_num}")
                return

            sequencer_name = sequencers[seq_num]

            # Run sequencer execution in a separate thread to avoid blocking
            if self.execution_active:
                logger.warning(
                    "Another execution is already active. Use 'force' to stop it first.")
                return

            self.current_execution_thread = threading.Thread(
                target=self.execute_sequencer, args=(sequencer_name,)
            )
            self.current_execution_thread.start()

        except ValueError:
            print("❌ Invalid number format")
        except Exception as e:
            print(f"❌ Error executing sequencer: {e}")

    def _show_system_status(self):
        """Show current system status."""
        if self.system:
            status = self.system.get_system_status()
            print(f"\n📊 System Status: {status}")
            print(f"🔄 Execution Active: {self.execution_active}")
            print(f"⚡ Force Complete: {self.force_complete}")
        else:
            print("❌ System not initialized")


def signal_handler(signum, frame):
    """Handle interrupt signals."""
    print(f"\n🛑 Received signal {signum}, shutting down...")
    sys.exit(0)


def main():
    """Main entry point for debug mode."""
    # Set up signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Parse command line arguments
    opc_mode = None
    if len(sys.argv) > 1:
        arg_mode = sys.argv[1].lower()
        if arg_mode in ["real", "mock"]:
            opc_mode = arg_mode
            logger.info(f"Using OPC mode: {opc_mode}")
        elif arg_mode in ["-h", "--help"]:
            print("Usage: python main_debug.py [real|mock]")
            print("  real  - Use real OPC UA client (default)")
            print("  mock  - Use mock OPC UA client for simulation")
            print("  -h    - Show this help message")
            return 0
        else:
            print(f"❌ Invalid argument: {arg_mode}")
            print("Usage: python main_debug.py [real|mock]")
            print("  real  - Use real OPC UA client")
            print("  mock  - Use mock OPC UA client for simulation")
            return 1

    # Create debug system manager
    manager = DebugSystemManager(opc_mode=opc_mode)

    try:
        # Initialize system
        if not manager.initialize():
            print("❌ Failed to initialize debug system")
            return 1

        # Start system
        if not manager.start_system():
            print("❌ Failed to start debug system")
            return 1

        # Run interactive debug mode
        manager.interactive_debug_mode()

        return 0

    except Exception as e:
        print(f"❌ Debug system error: {e}")
        return 1
    finally:
        manager.stop_system()


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
