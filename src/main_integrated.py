"""
Main entry point for the integrated robot control system.
Demonstrates the complete system integration with proper state machine usage.
"""
import time
import logging
import signal
import sys
from typing import Optional

from integrated_robot_control_system import IntegratedRobotControlSystem, SystemMode

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class IntegratedSystemManager:
    """
    Manager class for the integrated robot control system.
    Provides high-level control and monitoring capabilities.
    """

    def __init__(self):
        self.system: Optional[IntegratedRobotControlSystem] = None
        self.running = False

    def initialize(self) -> bool:
        """Initialize the robot control system."""
        try:
            logger.info("Initializing Integrated Robot Control System...")
            self.system = IntegratedRobotControlSystem()
            logger.info("System initialized successfully")
            return True
        except Exception as e:
            logger.error(f"Failed to initialize system: {e}")
            return False

    def start_system(self):
        """Start the robot control system."""
        if not self.system:
            logger.error("System not initialized")
            return False

        try:
            logger.info("Starting robot control system...")

            # Initialize camera
            if not self.system.camera_manager.initialize():
                raise RuntimeError("Failed to initialize camera")

            # Start hand tracker
            self.system.hand_tracker.start()

            # Start OPC UA communication
            self.system.opc_client.start()

            # Set initial mode to tracking
            self.system.set_mode(SystemMode.TRACKING)

            self.running = True
            logger.info("System started successfully")
            return True

        except Exception as e:
            logger.error(f"Failed to start system: {e}")
            self.stop_system()
            return False

    def stop_system(self):
        """Stop the robot control system."""
        if self.system:
            logger.info("Stopping robot control system...")
            self.system.stop()
            self.running = False
            logger.info("System stopped")

    def run_control_loop(self):
        """Run the main control loop."""
        if not self.system or not self.running:
            logger.error("System not running")
            return

        loop_start_time = 0.0
        loop_rate_ms = 20  # 50Hz control loop

        try:
            logger.info("Control loop started. Press Ctrl+C to stop.")

            while self.running:
                loop_start_time = time.time()

                # Update state machine
                self.system.state_machine.step()

                # Update task orchestrator if active
                current_mode = self.system.get_current_mode()
                if current_mode in [SystemMode.PICKUP, SystemMode.PLACEMENT, SystemMode.COMPLETE_TASK]:
                    self.system.task_orchestrator.step()

                # Maintain loop rate
                elapsed_ms = (time.time() - loop_start_time) * 1000
                remaining_ms = loop_rate_ms - elapsed_ms

                if remaining_ms > 0:
                    time.sleep(remaining_ms / 1000)
                else:
                    logger.warning(
                        f"Loop exceeded {loop_rate_ms}ms by {-remaining_ms:.1f}ms")

        except KeyboardInterrupt:
            logger.info("Stopping due to keyboard interrupt...")
        except Exception as e:
            logger.error(f"Error in control loop: {e}")
        finally:
            self.stop_system()

    def demonstrate_system_capabilities(self):
        """Demonstrate various system capabilities."""
        if not self.system:
            logger.error("System not initialized")
            return

        logger.info("=== System Capabilities Demonstration ===")

        # Check system status
        status = self.system.get_system_status()
        logger.info(f"System Status: {status}")

        # Demonstrate mode switching
        logger.info("\n--- Mode Switching Demonstration ---")

        # Switch to tracking mode
        logger.info("Switching to TRACKING mode...")
        self.system.set_mode(SystemMode.TRACKING)
        time.sleep(2)

        # Switch to pickup mode
        logger.info("Switching to PICKUP mode...")
        self.system.set_mode(SystemMode.PICKUP)
        time.sleep(2)

        # Switch to placement mode
        logger.info("Switching to PLACEMENT mode...")
        self.system.set_mode(SystemMode.PLACEMENT)
        time.sleep(2)

        # Switch to complete task mode
        logger.info("Switching to COMPLETE_TASK mode...")
        self.system.set_mode(SystemMode.COMPLETE_TASK)
        time.sleep(2)

        # Return to idle
        logger.info("Returning to IDLE mode...")
        self.system.set_mode(SystemMode.IDLE)

        logger.info("=== Demonstration Complete ===")

    def interactive_mode(self):
        """Run interactive mode for manual control."""
        if not self.system:
            logger.error("System not initialized")
            return

        logger.info("=== Interactive Mode ===")
        logger.info("Available commands:")
        logger.info("  status - Show system status")
        logger.info("  tracking - Switch to tracking mode")
        logger.info("  pickup - Start pickup task")
        logger.info("  placement - Start placement task")
        logger.info("  complete - Start complete task")
        logger.info("  idle - Return to idle")
        logger.info("  emergency - Emergency stop")
        logger.info("  reset - Reset system")
        logger.info("  quit - Exit program")

        while self.running:
            try:
                command = input("\nEnter command: ").strip().lower()

                if command == "quit":
                    break
                elif command == "status":
                    status = self.system.get_system_status()
                    logger.info(f"System Status: {status}")
                elif command == "tracking":
                    self.system.set_mode(SystemMode.TRACKING)
                    logger.info("Switched to tracking mode")
                elif command == "pickup":
                    self.system.set_mode(SystemMode.PICKUP)
                    logger.info("Started pickup task")
                elif command == "placement":
                    self.system.set_mode(SystemMode.PLACEMENT)
                    logger.info("Started placement task")
                elif command == "complete":
                    self.system.set_mode(SystemMode.COMPLETE_TASK)
                    logger.info("Started complete task")
                elif command == "idle":
                    self.system.set_mode(SystemMode.IDLE)
                    logger.info("Returned to idle mode")
                elif command == "emergency":
                    self.system.emergency_stop()
                    logger.info("Emergency stop activated")
                elif command == "reset":
                    self.system.reset_system()
                    logger.info("System reset")
                else:
                    logger.info("Unknown command. Type 'quit' to exit.")

            except KeyboardInterrupt:
                break
            except Exception as e:
                logger.error(f"Error processing command: {e}")

        logger.info("Exiting interactive mode")


def signal_handler(signum, frame):
    """Handle interrupt signals."""
    logger.info(f"Received signal {signum}, shutting down...")
    sys.exit(0)


def main():
    """Main entry point."""
    # Set up signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Create system manager
    manager = IntegratedSystemManager()

    try:
        # Initialize system
        if not manager.initialize():
            logger.error("Failed to initialize system")
            return 1

        # Start system
        if not manager.start_system():
            logger.error("Failed to start system")
            return 1

        # Check command line arguments for mode
        if len(sys.argv) > 1:
            mode = sys.argv[1].lower()

            if mode == "demo":
                # Run demonstration
                manager.demonstrate_system_capabilities()
            elif mode == "interactive":
                # Run interactive mode
                manager.interactive_mode()
            elif mode == "run":
                # Run normal control loop
                manager.run_control_loop()
            else:
                logger.error(f"Unknown mode: {mode}")
                logger.info("Available modes: demo, interactive, run")
                return 1
        else:
            # Default: run normal control loop
            manager.run_control_loop()

        return 0

    except Exception as e:
        logger.error(f"System error: {e}")
        return 1
    finally:
        manager.stop_system()


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
