# Integrated Robot Control System

This document describes the new integrated robot control system that properly connects all components using the current state machine and task orchestration implementations.

## Overview

The integrated system resolves the key integration gaps identified in the original system:

1. **State Machine Interface Alignment**: Uses the correct `StateMachine` interface that matches the state implementations
2. **Task Orchestration Integration**: Properly integrates the task orchestrator into the main control loop
3. **Configuration Integration**: All states now use configuration parameters consistently
4. **Missing Components**: Implements the missing camera transform module

## Files Created/Modified

### New Files

- `src/integrated_robot_control_system.py` - Main integrated system class
- `src/main_integrated.py` - New main entry point with multiple modes
- `src/camera_management/camera_transform_module.py` - Camera coordinate transformations
- `src/examples/integrated_system_example.py` - Demonstration of integrated system

### Modified Files

- `src/states/generate_pickup_state.py` - Uses configuration parameters consistently
- `src/states/hand_tracking_state.py` - Uses configuration parameters consistently
- `src/config/config.py` - Added missing configuration parameters

## System Architecture

### IntegratedRobotControlSystem Class

The main system class that properly integrates all components:

```python
class IntegratedRobotControlSystem:
    def __init__(self):
        # Initialize all components with proper configuration
        self.telemetry = Telemetry()
        self.command_bus = CommandBus()
        self.camera_manager = CameraManager(camera_config)
        self.opc_client = OPCClient(self.command_bus, self.telemetry, opc_config)
        self.kinematics_solver = InverseKinematicsSolver(...)
        self.hand_tracker = HandTracker(...)
        self.ggcnn2_module = GGcnn2Module(...)

        # Create shared context
        self.context = StateContext(...)

        # Initialize state machine with correct interface
        self.idle_state = IdleState(self.context)
        self.state_machine = StateMachine(self.idle_state)

        # Initialize task orchestrator
        self.task_orchestrator = TaskOrchestrator(self.state_machine, self.context)
```

### System Modes

The system supports multiple operating modes:

- `IDLE`: System ready but not executing tasks
- `TRACKING`: Hand tracking active
- `PICKUP`: Object pickup sequence
- `PLACEMENT`: Object placement sequence
- `COMPLETE_TASK`: Complete pickup→placement workflow
- `ERROR`: Error state with recovery capabilities

### State Machine Integration

The system uses the correct state machine interface:

```python
# Correct interface matching state implementations
self.state_machine = StateMachine(initial_state, on_state_completion)

# State execution
self.state_machine.step()  # Executes current state

# State transitions
self.state_machine.transition(next_state)  # Transitions to next state
```

### Task Orchestration Integration

The task orchestrator is properly integrated into the main control loop:

```python
# Update state machine
self.state_machine.step()

# Update task orchestrator if active
if self.current_mode in [SystemMode.PICKUP, SystemMode.PLACEMENT, SystemMode.COMPLETE_TASK]:
    self.task_orchestrator.step()
```

## Usage

### Running the System

#### Basic Usage

```bash
cd src
python main_integrated.py
```

#### Different Modes

```bash
# Run normal control loop
python main_integrated.py run

# Run demonstration
python main_integrated.py demo

# Run interactive mode
python main_integrated.py interactive
```

### Programmatic Usage

```python
from integrated_robot_control_system import IntegratedRobotControlSystem, SystemMode

# Create system
system = IntegratedRobotControlSystem()

# Initialize camera
system.camera_manager.initialize()

# Start components
system.hand_tracker.start()
system.opc_client.start()

# Set mode
system.set_mode(SystemMode.TRACKING)

# Run control loop
system.run_control_loop()
```

### Interactive Mode Commands

When running in interactive mode, you can use these commands:

- `status` - Show system status
- `tracking` - Switch to tracking mode
- `pickup` - Start pickup task
- `placement` - Start placement task
- `complete` - Start complete task
- `idle` - Return to idle
- `emergency` - Emergency stop
- `reset` - Reset system
- `quit` - Exit program

## Configuration Integration

All states now use configuration parameters consistently:

### GeneratePickupState

```python
# Uses configuration parameters
self.grasp_generation_timeout = GRASP_EXECUTION_CONFIG.get('grasp_generation_timeout', 10.0)
self.frame_processing_interval = GRASP_DETECTION_CONFIG.get('frame_processing_interval', 0.5)
```

### HandTrackingState

```python
# Uses configuration parameters
self.stability_check_interval = HAND_STABILITY_TIME_THRESHOLD / 20.0
if distance < HAND_STABILITY_THRESHOLD:  # Uses config threshold
```

### Configuration Parameters Added

```python
GRASP_DETECTION_CONFIG = {
    # ... existing parameters ...
    'frame_processing_interval': 0.5,  # Process frames every N seconds
}

GRASP_EXECUTION_CONFIG = {
    # ... existing parameters ...
    'grasp_generation_timeout': 10.0,  # Timeout for grasp generation
}
```

## Camera Transform Module

The missing camera transform module is now implemented:

```python
# Transform camera position to robot base frame
base_position = transform_camera_to_base(camera_position, tcp_matrix)

# Transform base position to camera frame
camera_position = transform_base_to_camera(base_position, tcp_matrix)

# Convert pixel to camera coordinates
camera_coords = pixel_to_camera_frame(u, v, depth, intrinsics)
```

## Key Improvements

### 1. Proper State Machine Integration

- Uses correct `StateMachine` interface
- States can be executed by main control loop
- Proper state transitions and completion callbacks

### 2. Task Orchestration Integration

- Task orchestrator integrated into main control loop
- Complete pickup→placement workflow executable
- Proper state completion handling

### 3. Configuration Consistency

- All states use configuration parameters
- No more hardcoded values
- System behavior easily tunable

### 4. Complete Component Integration

- Camera transform module implemented
- All components properly connected
- End-to-end execution path available

### 5. Error Handling and Recovery

- Emergency stop functionality
- System reset capabilities
- Error state management

## Testing

Run the integrated system example to test all functionality:

```bash
cd src/examples
python integrated_system_example.py
```

This will demonstrate:

- System initialization and startup
- Mode switching
- State machine operations
- Task orchestrator integration
- Configuration usage
- Emergency stop and reset

## Next Steps

With the integrated system, you can now:

1. **Test Individual Components**: Each component works independently
2. **Test State Sequences**: Pickup and placement sequences are executable
3. **Test Complete Workflow**: End-to-end object manipulation
4. **Tune Parameters**: All configuration parameters are accessible
5. **Add New States**: Easy to extend with new functionality

The system is now ready for full operational testing and deployment.
