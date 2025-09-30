"""
OPC UA Client Factory - Configurable client creation for real or mock environments.
"""
import logging
from typing import Optional, Any
from dataclasses import dataclass

from control.command_bus import CommandBus
from control.telemetry_store import Telemetry
from config.config import OPC_MODE, OPC_SERVER_URL, OPC_MOCK_SERVER_URL, ROBOT_ID

logger = logging.getLogger(__name__)


@dataclass
class OPCConfig:
    """Unified OPC UA configuration parameters."""
    url: str
    objects_name: str = "0:Objects"
    robot_id: int = ROBOT_ID
    poll_interval_ms: int = 50
    command_batch_size: int = 10
    skip_redundant_writes: bool = True
    connection_timeout: float = 5.0
    reconnect_delay: float = 2.0
    max_reconnect_attempts: int = 5

    @property
    def robot_name(self) -> str:
        """Get the robot name based on robot ID."""
        return f"{20 + self.robot_id}:robot{self.robot_id}"


class OPCClientFactory:
    """
    Factory for creating OPC UA clients based on configuration.
    Supports both real and mock OPC UA clients.
    """

    @staticmethod
    def create_client(
        command_bus: CommandBus,
        telemetry: Telemetry,
        config: Optional[OPCConfig] = None,
        mode: Optional[str] = None
    ) -> Any:
        """
        Create an OPC UA client based on the specified mode.

        Args:
            command_bus: Command bus for sending commands
            telemetry: Telemetry store for receiving data
            config: OPC configuration (optional)
            mode: OPC mode - "real" or "mock" (optional, uses config default)

        Returns:
            OPC UA client instance (real or mock)
        """
        # Determine mode
        if mode is None:
            mode = OPC_MODE.lower()

        # Create default config if not provided
        if config is None:
            if mode == "mock":
                url = OPC_MOCK_SERVER_URL
            else:
                url = OPC_SERVER_URL

            config = OPCConfig(url=url)

        logger.info(f"Creating OPC UA client in '{mode}' mode")

        if mode == "mock":
            return OPCClientFactory._create_mock_client(command_bus, telemetry, config)
        elif mode == "real":
            return OPCClientFactory._create_real_client(command_bus, telemetry, config)
        else:
            raise ValueError(
                f"Unknown OPC mode: {mode}. Must be 'real' or 'mock'")

    @staticmethod
    def _create_real_client(command_bus: CommandBus, telemetry: Telemetry, config: OPCConfig) -> Any:
        """Create a real OPC UA client."""
        try:
            from IO_handling.opc_client import OPCClient, OPCConfig as RealOPCConfig

            # Convert to real OPC config format
            real_config = RealOPCConfig(
                url=config.url,
                objects_name=config.objects_name,
                robot_id=config.robot_id,
                poll_interval_ms=config.poll_interval_ms,
                command_batch_size=config.command_batch_size,
                skip_redundant_writes=config.skip_redundant_writes,
                connection_timeout=config.connection_timeout,
                reconnect_delay=config.reconnect_delay,
                max_reconnect_attempts=config.max_reconnect_attempts
            )

            logger.info(
                f"Created real OPC UA client connecting to {config.url}")
            return OPCClient(command_bus, telemetry, real_config)

        except ImportError as e:
            logger.error(f"Failed to import real OPC client: {e}")
            raise RuntimeError("Real OPC client not available")

    @staticmethod
    def _create_mock_client(command_bus: CommandBus, telemetry: Telemetry, config: OPCConfig) -> Any:
        """Create a mock OPC UA client."""
        try:
            from IO_handling.mock_opc_client import MockOPCClient, MockOPCConfig

            # Convert to mock OPC config format
            mock_config = MockOPCConfig(
                url=config.url,
                objects_name=config.objects_name,
                robot_id=config.robot_id,
                poll_interval_ms=config.poll_interval_ms,
                command_batch_size=config.command_batch_size,
                skip_redundant_writes=config.skip_redundant_writes,
                connection_timeout=config.connection_timeout,
                reconnect_delay=config.reconnect_delay,
                max_reconnect_attempts=config.max_reconnect_attempts
            )

            logger.info(
                f"Created mock OPC UA client connecting to {config.url}")
            return MockOPCClient(command_bus, telemetry, mock_config)

        except ImportError as e:
            logger.error(f"Failed to import mock OPC client: {e}")
            raise RuntimeError("Mock OPC client not available")

    @staticmethod
    def create_config_from_mode(mode: str = None) -> OPCConfig:
        """
        Create OPC configuration based on mode.

        Args:
            mode: OPC mode - "real" or "mock"

        Returns:
            OPC configuration
        """
        if mode is None:
            mode = OPC_MODE.lower()

        if mode == "mock":
            url = OPC_MOCK_SERVER_URL
        else:
            url = OPC_SERVER_URL

        return OPCConfig(url=url)

    @staticmethod
    def get_available_modes() -> list:
        """Get list of available OPC modes."""
        return ["real", "mock"]

    @staticmethod
    def validate_mode(mode: str) -> bool:
        """Validate if the specified mode is supported."""
        return mode.lower() in OPCClientFactory.get_available_modes()


def create_opc_client(
    command_bus: CommandBus,
    telemetry: Telemetry,
    mode: str = None,
    config: Optional[OPCConfig] = None
) -> Any:
    """
    Convenience function to create an OPC UA client.

    Args:
        command_bus: Command bus for sending commands
        telemetry: Telemetry store for receiving data
        mode: OPC mode - "real" or "mock" (optional)
        config: OPC configuration (optional)

    Returns:
        OPC UA client instance
    """
    return OPCClientFactory.create_client(command_bus, telemetry, config, mode)
