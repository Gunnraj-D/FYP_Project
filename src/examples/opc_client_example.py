"""
Example demonstrating OPCClient usage with dedicated background loop.
Shows how to start, monitor, and stop the OPC client.
"""
import time
import logging
from control.command_bus import CommandBus, SetJoints, SetGripper, EmergencyStop
from control.telemetry_store import Telemetry
from IO_handling.opc_client import OPCClient, OPCConfig

# Setup logging
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


def main():
    """Example of OPCClient usage."""

    # Initialize shared components
    telemetry = Telemetry()
    command_bus = CommandBus()

    # Create OPC client with custom configuration
    opc_config = OPCConfig(
        poll_interval_ms=100,  # 10Hz polling
        command_batch_size=5,
        skip_redundant_writes=True,
        connection_timeout=3.0,
        reconnect_delay=1.0,
        max_reconnect_attempts=3
    )

    opc_client = OPCClient(command_bus, telemetry, opc_config)

    try:
        # Start the OPC client background loop
        logger.info("Starting OPC client...")
        opc_client.start()

        # Wait for connection
        logger.info("Waiting for OPC connection...")
        for i in range(10):
            if opc_client.is_connected():
                logger.info("OPC client connected!")
                break
            time.sleep(1)
            logger.info(f"Connection attempt {i+1}/10")
        else:
            logger.error("Failed to connect to OPC server")
            return

        # Demonstrate command sending
        logger.info("Sending test commands...")

        # Send joint position command
        test_joints = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
        command_bus.send(SetJoints(joints=test_joints))
        logger.info(f"Sent joint command: {test_joints}")

        # Send gripper command
        command_bus.send(SetGripper(status="open"))
        logger.info("Sent gripper open command")

        # Monitor system for a few seconds
        logger.info("Monitoring system for 5 seconds...")
        for i in range(5):
            status = opc_client.get_status()
            robot_status = telemetry.get_robot_status()
            current_joints = telemetry.get_current_joints()

            logger.info(f"OPC Status: {status}")
            logger.info(f"Robot Status: {robot_status}")
            logger.info(f"Current Joints: {current_joints}")

            time.sleep(1)

        # Send emergency stop
        logger.info("Sending emergency stop...")
        command_bus.send(EmergencyStop(active=True))
        time.sleep(1)

        # Clear emergency stop
        logger.info("Clearing emergency stop...")
        command_bus.send(EmergencyStop(active=False))
        time.sleep(1)

    except KeyboardInterrupt:
        logger.info("Received keyboard interrupt")
    except Exception as e:
        logger.error(f"Error: {e}")
    finally:
        # Stop the OPC client
        logger.info("Stopping OPC client...")
        opc_client.stop()
        logger.info("OPC client stopped")


if __name__ == "__main__":
    main()
