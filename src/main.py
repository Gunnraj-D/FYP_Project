"""
Example usage of the refactored robot hand tracking system.
Demonstrates the new modular architecture and clean interfaces.
"""
import time
import logging
from robot_control_system import RobotControlSystem

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def main():
    """Example usage of the refactored system."""

    # Create system instance
    system = RobotControlSystem()

    try:
        logger.info("Starting robot hand tracking system...")

        # Start the system
        system.start()

        # Example: Monitor system status
        for i in range(10):
            status = system.get_system_status()
            logger.info(f"System status: {status}")
            time.sleep(2)

        # Example: Change modes
        logger.info("Switching to IDLE mode...")
        system.set_mode("IDLE")
        time.sleep(3)

        logger.info("Switching back to TRACKING mode...")
        system.set_mode("TRACKING")
        time.sleep(3)

        # Example: Check robot connection
        if system.is_robot_connected():
            logger.info("Robot is connected and ready")
        else:
            logger.warning("Robot is not connected")

        # Keep running for a while
        logger.info("System running. Press Ctrl+C to stop...")
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        logger.info("Stopping system...")
    except Exception as e:
        logger.error(f"System error: {e}")
    finally:
        system.stop()
        logger.info("System stopped.")


if __name__ == "__main__":
    main()
