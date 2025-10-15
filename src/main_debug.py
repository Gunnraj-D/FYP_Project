"""
Debug version of the main integrated robot control system.
Allows interactive selection and execution of individual states and task sequencers.
"""
import warnings
from config import (
    PRE_PICKUP_POSE,
    HANDOFF_APPROACH_POSE,
    ROBOT_ID,
    PICKUP_LOCATION,
    set_camera_transform_mode,
    print_camera_transform_info,
    CAMERA_TRANSFORM_MODE,
    print_config_summary
)
from states.placement_task_sequencer import PlacementTaskSequencer, create_placement_sequencer
from states.pickup_task_sequencer import PickupTaskSequencer, create_pickup_sequencer
from states.grasping_state import GraspingState
from states.unified_hand_tracking_state import UnifiedHandTrackingState
from states.gripper_state import GripperControlState
from states.move_to_state import MoveToState
from states.human_aware_move_to_state import HumanAwareMoveToState
from states.human_handoff_approach_state import HumanHandoffApproachState
from states.base_state import BaseState
from states.state_machine import StateMachine
from states.context import StateContext
from integrated_robot_control_system import IntegratedRobotControlSystem, SystemMode
from hand_detection.zed_joint_receiver import ZEDJointReceiver
import os
import time
import logging
import signal
import sys
import threading
import asyncio
from typing import Optional, Dict, List
import argparse

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
logging.getLogger('object_detection.grasp_detector').setLevel(logging.INFO)
logging.getLogger('kinematics.kinematics_solver').setLevel(logging.INFO)
logging.getLogger('states.grasping_state').setLevel(logging.INFO)
logging.getLogger('integrated_robot_control_system').setLevel(logging.ERROR)
logging.getLogger('states.task_orchestrator').setLevel(logging.ERROR)
# Allow MoveToState INFO logs like the IK solver
logging.getLogger('states.move_to_state').setLevel(logging.INFO)
logging.getLogger('control.telemetry_store').setLevel(logging.ERROR)
logging.getLogger('IO_handling.opc_client_factory').setLevel(logging.INFO)
logging.getLogger('IO_handling.opc_client').setLevel(logging.INFO)
# Enable human-aware state logging for debugging
logging.getLogger('states.human_aware_move_to_state').setLevel(logging.INFO)
logging.getLogger('states.human_handoff_approach_state').setLevel(logging.INFO)
logging.getLogger('kinematics.human_aware_path_planner').setLevel(logging.INFO)
logging.getLogger('hand_detection.zed_joint_receiver').setLevel(logging.INFO)
logging.getLogger('object_detection.postprocessing').setLevel(logging.INFO)
logging.getLogger(
    'object_detection.postprocessing.__init__').setLevel(logging.INFO)
logging.getLogger(
    'object_detection.postprocessing.candidate_selection').setLevel(logging.INFO)
logging.getLogger('object_detection.grasp_transforms').setLevel(
    logging.WARNING)

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

    def __init__(self, opc_mode: str = None, no_camera: bool = False,
                 verbose: bool = False, debug_visuals: Optional[bool] = None,
                 zed: Optional[bool] = None, profile: Optional[str] = None):
        self.opc_mode = opc_mode
        self.no_camera = bool(no_camera)
        self.verbose = bool(verbose)
        self.debug_visuals = debug_visuals
        self.profile_name = profile
        self.zed_enabled = True if zed is None else bool(zed)
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

        # ZED skeleton tracking
        self.zed_receiver: Optional[ZEDJointReceiver] = None

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

            # Initialize ZED receiver if enabled
            if self.zed_enabled:
                try:
                    self.zed_receiver = ZEDJointReceiver(
                        host='0.0.0.0',
                        port=5005,
                        smoothing_factor=0.3,
                        tracking_loss_frames=5
                    )
                    self.zed_receiver.start()
                    logger.info("✅ ZED Joint Receiver started")
                except Exception as e:
                    logger.warning(f"⚠️ ZED receiver not started: {e}")
                    self.zed_receiver = None

            # Get the context from the system
            self.context = StateContext(
                telemetry=self.system.telemetry,
                commands=self.system.command_bus,
                camera=self.system.camera_manager,
                opc=self.system.opc_client,
                ik=self.system.kinematics_solver,
                zed_receiver=self.zed_receiver
            )

            # Create a state machine for debug execution
            self.state_machine = StateMachine(
                initial_state=MoveToState(
                    self.context, target_location=(0.3, 0.415, 0.24)),  # Updated position 1 in meters
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

            # Initialize camera (optional)
            if not self.no_camera:
                try:
                    if not self.system.camera_manager.initialize():
                        print(
                            "⚠️ Camera initialization failed - continuing without camera")
                except Exception as e:
                    print(f"⚠️ Camera not available")
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
        # Stop ZED receiver
        if self.zed_receiver:
            try:
                self.zed_receiver.stop()
                print("✅ ZED receiver stopped")
            except Exception as e:
                print(f"⚠️ Error stopping ZED receiver: {e}")
            self.zed_receiver = None

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
        from kinematics.kinematics_solver import get_facing_down_orientation

        states = {
            # Reasonable position in workspace
            1: MoveToState(self.context, target_location=(0.3, 0.415, 0.36)),
            # Updated second position
            2: MoveToState(self.context, target_location=tuple(PICKUP_LOCATION['position'])),
            3: GripperControlState(self.context, action='open'),
            4: GripperControlState(self.context, action='close'),
            5: UnifiedHandTrackingState(self.context),
            6: GraspingState(self.context, object_profile=self.profile_name, enable_visuals=self.debug_visuals),
            # Safe retreat position (far from human interaction zone)
            # Far left, high
            11: MoveToState(self.context, target_location=(-0.4, 0.4, 0.5)),
            7: HumanAwareMoveToState(
                context=self.context,
                target_position=[0.3, 0.415, 0.6],
                target_orientation=get_facing_down_orientation()
            ) if self.zed_receiver else None,
            8: HumanAwareMoveToState(
                context=self.context,
                target_position=list(PICKUP_LOCATION['position']),
                target_orientation=get_facing_down_orientation()
            ) if self.zed_receiver else None,
            9: HumanHandoffApproachState(
                context=self.context,
                approach_offset=[0.0, 0.0, 0.30],  # 30cm above right hand
                hand_joint_name='RIGHT_WRIST'
            ) if self.zed_receiver else None,
            10: HumanHandoffApproachState(
                context=self.context,
                approach_offset=[0.0, 0.0, 0.30],
                hand_joint_name='LEFT_WRIST'  # Left hand variant
            ) if self.zed_receiver else None,
        }
        # Remove None entries (ZED not available)
        return {k: v for k, v in states.items() if v is not None}

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
                # 20 seconds timeout (200 iterations * 0.1s)
                if iteration > 200:
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
        sequence_complete = False

        try:
            # Run until sequence is complete (queue empty AND current state done)
            while not sequence_complete and not self.force_complete:
                sequencer.step()
                time.sleep(0.1)  # 10Hz execution rate

                # Show progress only when step changes
                progress = sequencer.get_progress()
                current_step = progress['current_step']
                if current_step > 0 and current_step != last_logged_step:
                    logger.info(f"Sequencer progress: {current_step}/{progress['total_steps']} "
                                f"({progress['progress_percent']:.1f}%)")
                    last_logged_step = current_step

                # Check if truly complete: queue empty AND current state finished
                if not sequencer.task_queue and self.state_machine.current_state.is_complete():
                    sequence_complete = True
                    logger.info("All sequence states completed")

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
        print(f"Camera Transform Mode: {CAMERA_TRANSFORM_MODE}")
        print("\nAvailable commands:")
        print("  states - Show available states")
        print("  sequencers - Show available sequencers")
        print("  run <number> - Execute state by number")
        print("  seq <number> - Execute sequencer by number")
        print("  config - Show config summary")
        print("  profile <name> - Set grasp object profile")
        print("  camera - Toggle camera transform mode (calibrated/simple)")
        print("  camera info - Show camera transform details")
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
                elif command == "camera":
                    self._toggle_camera_mode()
                elif command == "camera info":
                    print_camera_transform_info()
                elif command == "config":
                    print_config_summary()
                elif command.startswith("profile "):
                    parts = command.split()
                    if len(parts) == 2:
                        self.profile_name = parts[1]
                        print(f"✅ Profile set: {self.profile_name}")
                    else:
                        print("❌ Usage: profile <name>")
                elif command == "debug":
                    self._toggle_grasp_debug()
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

    def _toggle_camera_mode(self):
        """Toggle between calibrated and simple camera transform modes."""
        from config import CAMERA_TRANSFORM_MODE as current_mode
        new_mode = 'simple' if current_mode == 'calibrated' else 'calibrated'

        print(
            f"\n🔄 Switching camera transform mode: {current_mode} -> {new_mode}")

        try:
            set_camera_transform_mode(new_mode)
            print("⚠️ Note: This will affect all future transformations.")
            print("   Already running states will not be affected until restarted.")
            print_camera_transform_info()
        except Exception as e:
            print(f"❌ Failed to switch camera mode: {e}")

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
        print("  camera - Toggle camera transform mode (calibrated/simple)")
        print("  camera info - Show camera transform details")
        print("  debug - Toggle grasp height debugging")
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
            from kinematics.kinematics_solver import get_facing_down_orientation

            if state_num == 1:
                state = MoveToState(
                    self.context, target_location=(0.520, -0.055, 0.25))
            elif state_num == 2:
                state = MoveToState(
                    self.context, target_location=(0.0, -.6, .25))
            elif state_num == 3:
                state = GripperControlState(self.context, action='open')
            elif state_num == 4:
                state = GripperControlState(self.context, action='close')
            elif state_num == 5:
                state = UnifiedHandTrackingState(self.context)
            elif state_num == 6:
                state = GraspingState(
                    self.context, object_profile=self.profile_name, enable_visuals=self.debug_visuals)
            elif state_num == 7:
                if not self.zed_receiver:
                    print("❌ ZED receiver not running. Start Unity with ZED first.")
                    return
                state = HumanAwareMoveToState(
                    context=self.context,
                    target_position=[0.3, 0.415, 0.6],
                    target_orientation=get_facing_down_orientation()
                )
            elif state_num == 8:
                if not self.zed_receiver:
                    print("❌ ZED receiver not running. Start Unity with ZED first.")
                    return
                state = HumanAwareMoveToState(
                    context=self.context,
                    target_position=list(PICKUP_LOCATION['position']),
                    target_orientation=get_facing_down_orientation()
                )
            elif state_num == 9:
                if not self.zed_receiver:
                    print("❌ ZED receiver not running. Start Unity with ZED first.")
                    return
                state = HumanHandoffApproachState(
                    context=self.context,
                    approach_offset=[0.0, 0.0, 0.30],
                    hand_joint_name='RIGHT_WRIST'
                )
            elif state_num == 10:
                if not self.zed_receiver:
                    print("❌ ZED receiver not running. Start Unity with ZED first.")
                    return
                state = HumanHandoffApproachState(
                    context=self.context,
                    approach_offset=[0.0, 0.0, 0.30],
                    hand_joint_name='LEFT_WRIST'
                )
            elif state_num == 11:
                # Safe retreat position
                state = MoveToState(
                    self.context, target_location=(-0.4, 0.4, 0.5))
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

    def _toggle_grasp_debug(self):
        """Toggle grasp height debugging on/off."""
        try:
            from object_detection.grasp_debug import enable_grasp_debug, _grasp_debugger

            # Toggle the debug state
            _grasp_debugger.enabled = not _grasp_debugger.enabled
            enable_grasp_debug(_grasp_debugger.enabled)

            status = "enabled" if _grasp_debugger.enabled else "disabled"
            print(f"🔍 Grasp height debugging {status}")

            if _grasp_debugger.enabled:
                print("   Debug images will be saved to: grasp_debug/")
                print("   Issues will be logged during grasp detection")
            else:
                print("   Debug images and logging disabled")

        except Exception as e:
            print(f"❌ Failed to toggle grasp debug: {e}")

    def _show_system_status(self):
        """Show current system status."""
        try:
            from object_detection.grasp_debug import _grasp_debugger
            debug_status = "enabled" if _grasp_debugger.enabled else "disabled"
        except:
            debug_status = "unknown"

        if self.system:
            status = self.system.get_system_status()
            print(f"\n📊 System Status: {status}")
            print(f"🔄 Execution Active: {self.execution_active}")
            print(f"⚡ Force Complete: {self.force_complete}")
            print(f"🔍 Grasp Debug: {debug_status}")
        else:
            print("❌ System not initialized")


def signal_handler(signum, frame):
    """Handle interrupt signals."""
    print(f"\n🛑 Received signal {signum}, shutting down...")
    sys.exit(0)


def main():
    """Main entry point for debug mode."""
    # Set up signal handlers
    try:
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
    except Exception:
        pass

    # Parse command line arguments
    parser = argparse.ArgumentParser(add_help=True)
    parser.add_argument('--opc-mode', choices=['real', 'mock'], default=None)
    parser.add_argument('--no-camera', action='store_true')
    parser.add_argument('--verbose', action='store_true')
    parser.add_argument('--debug-visuals',
                        dest='debug_visuals', action='store_true')
    parser.add_argument('--no-debug-visuals',
                        dest='debug_visuals', action='store_false')
    parser.set_defaults(debug_visuals=None)
    parser.add_argument('--zed', dest='zed', action='store_true')
    parser.add_argument('--no-zed', dest='zed', action='store_false')
    parser.set_defaults(zed=None)
    parser.add_argument('--profile', type=str, default=None)

    args = parser.parse_args()
    if args.verbose:
        logging.getLogger().setLevel(logging.INFO)

    # Create debug system manager
    manager = DebugSystemManager(
        opc_mode=args.opc_mode,
        no_camera=args.no_camera,
        verbose=args.verbose,
        debug_visuals=args.debug_visuals,
        zed=args.zed,
        profile=args.profile
    )

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
