"""
Example usage of the debug mode for interactive state and sequencer execution.
"""
import logging
from main_debug import DebugSystemManager
import sys
import os

# Add the src directory to the path so we can import modules
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def run_debug_example():
    """Example of how to use the debug system programmatically."""

    # Create debug system manager
    manager = DebugSystemManager()

    try:
        # Initialize system
        if not manager.initialize():
            logger.error("Failed to initialize debug system")
            return False

        # Start system
        if not manager.start_system():
            logger.error("Failed to start debug system")
            return False

        logger.info("Debug system ready!")

        # Show available states
        logger.info("Available States:")
        states = manager.get_available_states()
        for num, state in states.items():
            logger.info(f"  {num}. {state.name}")

        # Show available sequencers
        logger.info("\nAvailable Sequencers:")
        sequencers = manager.get_available_sequencers()
        for num, name in sequencers.items():
            logger.info(f"  {num}. {name}")

        # Example: Execute a specific state
        logger.info("\nExample: Executing MoveToState...")
        move_state = states[1]  # MoveToState with target (100, 100, 200)
        manager.execute_state(move_state)

        # Example: Execute a sequencer
        logger.info("\nExample: Executing Pickup Task Sequencer...")
        manager.execute_sequencer("Pickup Task Sequencer")

        return True

    except Exception as e:
        logger.error(f"Debug example error: {e}")
        return False
    finally:
        manager.stop_system()


if __name__ == "__main__":
    success = run_debug_example()
    if success:
        logger.info("Debug example completed successfully!")
    else:
        logger.error("Debug example failed!")
        sys.exit(1)
