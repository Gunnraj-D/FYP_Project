# OPCClient Refactoring Documentation

## Overview

The OPCClient has been refactored to implement a dedicated background loop that handles robot communication with configurable behavior, thread safety, and proper lifecycle management.

## Key Features

### 1. Dedicated Background Loop

- Runs in its own thread with configurable frequency (`OPC_POLL_INTERVAL_MS`)
- Maintains consistent timing with loop interval monitoring
- Handles connection errors gracefully with automatic reconnection

### 2. Telemetry Updates

- Automatically reads robot state every loop iteration
- Updates TelemetryStore with thread-safe operations
- Reports connection status and error messages

### 3. CommandBus Integration

- Drains all available commands from CommandBus each loop
- Processes commands sequentially to preserve order
- Supports configurable batch size (`OPC_COMMAND_BATCH_SIZE`)
- Non-blocking writes with redundant write detection

### 4. Configuration Options

All behavior is configurable via `config.py`:

```python
# OPC UA Client Configuration
OPC_POLL_INTERVAL_MS = 50  # Poll interval in milliseconds (20Hz)
OPC_COMMAND_BATCH_SIZE = 10  # Maximum commands to process per loop
OPC_SKIP_REDUNDANT_WRITES = True  # Skip writes if values haven't changed
OPC_CONNECTION_TIMEOUT_SECONDS = 5.0  # Connection timeout
OPC_RECONNECT_DELAY_SECONDS = 2.0  # Delay before reconnection attempts
OPC_MAX_RECONNECT_ATTEMPTS = 5  # Maximum reconnection attempts
```

### 5. Thread Safety

- Uses RLock for thread-safe operations
- Proper startup/shutdown with thread joining
- Graceful cleanup of OPC UA sessions

### 6. Error Handling

- Connection error recovery with exponential backoff
- Comprehensive logging for debugging
- Status reporting via telemetry

## Usage

### Basic Usage

```python
from control.command_bus import CommandBus
from control.telemetry_store import Telemetry
from IO_handling.opc_client import OPCClient, OPCConfig

# Initialize components
telemetry = Telemetry()
command_bus = CommandBus()
opc_client = OPCClient(command_bus, telemetry)

# Start background loop
opc_client.start()

# Send commands
command_bus.send(SetJoints(joints=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]))
command_bus.send(SetGripper(status="open"))

# Check status
if opc_client.is_connected():
    print("Robot connected")

# Stop when done
opc_client.stop()
```

### Custom Configuration

```python
opc_config = OPCConfig(
    poll_interval_ms=100,  # 10Hz polling
    command_batch_size=5,
    skip_redundant_writes=True,
    connection_timeout=3.0,
    reconnect_delay=1.0,
    max_reconnect_attempts=3
)

opc_client = OPCClient(command_bus, telemetry, opc_config)
```

## Architecture

### Loop Structure

```
Main Loop (every OPC_POLL_INTERVAL_MS):
├── 1. Update Telemetry
│   ├── Read joint positions from robot
│   ├── Read robot status
│   └── Update TelemetryStore (thread-safe)
├── 2. Process Commands
│   ├── Drain CommandBus
│   ├── Execute commands sequentially
│   └── Skip redundant writes (if enabled)
└── 3. Maintain Timing
    ├── Calculate elapsed time
    ├── Sleep for remaining interval
    └── Log timing warnings if needed
```

### Command Processing

Commands are processed in priority order:

1. **EmergencyStop** - Highest priority, clears all other commands
2. **SetJoints** - Joint position commands
3. **SetGripper** - Gripper control commands

### Redundant Write Detection

When `OPC_SKIP_REDUNDANT_WRITES = True`:

- Joint positions: Compares with previous values (tolerance: 1e-6)
- Gripper status: Compares with previous status string
- Reduces unnecessary OPC UA writes

## Integration with RobotControlSystem

The refactored OPCClient integrates seamlessly with the existing RobotControlSystem:

```python
# In RobotControlSystem.__init__()
self.opc_client = OPCClient(self.command_bus, self.telemetry, opc_config)

# Start/stop handled automatically
def start(self):
    self.opc_client.start()

def stop(self):
    self.opc_client.stop()
```

## Status Monitoring

The OPCClient provides status information:

```python
status = opc_client.get_status()
# Returns:
# {
#     'running': bool,
#     'connected': bool,
#     'reconnect_attempts': int,
#     'thread_alive': bool
# }
```

## Error Recovery

- **Connection Loss**: Automatic reconnection with configurable delays
- **Max Reconnect Attempts**: Graceful shutdown after max attempts
- **Command Errors**: Logged but don't stop the loop
- **Telemetry Errors**: Logged, loop continues

## Performance Considerations

- **Loop Frequency**: Higher frequency = more responsive but more CPU usage
- **Batch Size**: Larger batches = fewer OPC writes but higher latency
- **Redundant Writes**: Disable for debugging, enable for performance
- **Connection Timeout**: Balance between responsiveness and stability

## Example

See `src/examples/opc_client_example.py` for a complete working example demonstrating:

- Starting/stopping the OPC client
- Sending various command types
- Monitoring system status
- Handling emergency stops
