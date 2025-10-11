"""
OPC UA communication configuration.
"""

# ============================================================================
# OPC UA SERVER CONNECTION
# ============================================================================

# OPC UA Server connection
OPC_SERVER_URL = "opc.tcp://172.24.200.1:4840/"
OPC_OBJECTS_NAME = "0:Objects"

# OPC UA communication rate
OPC_UPDATE_INTERVAL_SECONDS = 0.05  # 20Hz update rate

# ============================================================================
# OPC UA CLIENT CONFIGURATION
# ============================================================================

OPC_POLL_INTERVAL_MS = 50  # Poll interval in milliseconds (20Hz)
OPC_COMMAND_BATCH_SIZE = 10  # Maximum commands to process per loop
OPC_SKIP_REDUNDANT_WRITES = True  # Skip writes if values haven't changed
OPC_CONNECTION_TIMEOUT_SECONDS = 5.0  # Connection timeout
OPC_RECONNECT_DELAY_SECONDS = 2.0  # Delay before reconnection attempts
OPC_MAX_RECONNECT_ATTEMPTS = 5  # Maximum reconnection attempts

# ============================================================================
# OPC UA MODE
# ============================================================================

OPC_MODE = "real"  # Options: "real", "mock"
OPC_MOCK_SERVER_URL = "opc.tcp://127.0.0.1:4840/"  # URL for mock server
