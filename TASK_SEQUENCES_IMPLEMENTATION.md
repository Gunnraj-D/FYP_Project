# Task Sequences Implementation

This document describes the implementation of two reusable task sequences within the existing Finite State Machine (FSM) architecture: **Pickup Task Sequencer** and **Placement Task Sequencer**.

## Overview

The implementation provides a complete object manipulation workflow that allows the robot to:

1. **Pick up objects** from a table using GGCNN2-based grasp detection
2. **Place objects** into a human operator's hand using active hand tracking

## Architecture

### Core Components

1. **Task Orchestrator** (`task_orchestrator.py`) - Coordinates the overall workflow
2. **Pickup Task Sequencer** (`pickup_task_sequencer.py`) - Manages object pickup sequence
3. **Placement Task Sequencer** (`placement_task_sequencer.py`) - Manages object placement sequence
4. **Hand Tracking State** (`hand_tracking_state.py`) - Active hand tracking for placement
5. **Enhanced States** - Modified existing states to support new functionality

### Configuration Updates

New configuration constants added to `config.py`:

```python
# Approximate pose for robot before object detection
PRE_PICKUP_POSE = [400, 0, 300, 0, 0, -90]  # [x, y, z, rx, ry, rz]

# General pose for robot before hand tracking
HANDOFF_APPROACH_POSE = [500, 200, 350, 0, 0, -90]  # [x, y, z, rx, ry, rz]
```

### Telemetry Store Extensions

New telemetry storage capabilities:

- `generated_grasp_pose` - Final grasp pose from GGCNN2
- `generated_approach_pose` - Safe approach pose above grasp
- `pickup_height_offset` - Vertical distance between grasp and approach
- `live_hand_pose` - Real-time hand position
- `calculated_handoff_pose` - Final placement pose

## Task Sequences

### 1. Pickup Task Sequence

**Purpose**: Move to object location, detect grasp, and pick up object.

**Sequence**:

1. `MoveToState(pose=PRE_PICKUP_POSE)` - Move to pre-pickup position
2. `GeneratePickupState()` - Detect object and generate grasp poses
3. `GripperControlState(action='open')` - Open gripper
4. `MoveToState(pose_from_telemetry='generated_approach_pose')` - Move to approach pose
5. `MoveToState(pose_from_telemetry='generated_grasp_pose')` - Move to grasp pose
6. `GripperControlState(action='close')` - Close gripper to grasp
7. `MoveToState(pose_from_telemetry='generated_approach_pose')` - Lift object

**Key Features**:

- Uses GGCNN2 for intelligent grasp detection
- Calculates safe approach and grasp poses
- Stores pickup height offset for subsequent placement
- Thread-safe pose storage in telemetry

### 2. Placement Task Sequence

**Purpose**: Track operator's hand and place object into it.

**Sequence**:

1. `MoveToState(pose=HANDOFF_APPROACH_POSE)` - Move to handoff area
2. `HandTrackingState()` - Track hand and calculate placement pose
3. `MoveToState(pose_from_telemetry='calculated_handoff_pose')` - Move to hand
4. `GripperControlState(action='open')` - Release object

**Key Features**:

- Active hand tracking using MediaPipe
- Real-time placement pose calculation
- Hand stability detection before placement
- Automatic height offset from pickup sequence

## State Modifications

### Enhanced MoveToState

The `MoveToState` now supports two initialization modes:

```python
# Legacy mode (direct coordinates)
MoveToState(context, target_location=(x, y, z))

# Telemetry mode (retrieve from stored poses)
MoveToState(context, pose_from_telemetry='generated_grasp_pose')
```

Supported telemetry keys:

- `'generated_grasp_pose'`
- `'generated_approach_pose'`
- `'calculated_handoff_pose'`

### Enhanced GeneratePickupState

The `GeneratePickupState` now stores comprehensive pose information:

- **Grasp Pose**: Final end-effector pose for gripping
- **Approach Pose**: Safe pose above grasp position
- **Pickup Height Offset**: Vertical distance for placement calculations

### New HandTrackingState

Features:

- Continuous hand detection using MediaPipe
- Real-time pose calculation with height offset
- Hand stability monitoring
- Automatic transition when hand is stable

## Usage Examples

### Basic Usage

```python
from states.task_orchestrator import TaskOrchestrator
from states.context import StateContext
from states.state_machine import StateMachine

# Create context and state machine
context = StateContext(...)
state_machine = StateMachine(initial_state)

# Create orchestrator
orchestrator = TaskOrchestrator(state_machine, context)

# Start complete workflow (pickup + placement)
orchestrator.start_complete_task()

# Run control loop
while not orchestrator.get_status()['is_complete']:
    orchestrator.step()
```

### Individual Sequences

```python
# Pickup only
orchestrator.start_pickup_task()

# Placement only
orchestrator.start_placement_task()
```

### Progress Monitoring

```python
status = orchestrator.get_status()
print(f"Current phase: {status['current_phase']}")
print(f"Progress: {status['pickup_progress']['progress_percent']:.1f}%")
```

## Integration with Existing System

### Command Bus Integration

All robot movements use the existing `CommandBus`:

```python
# Joint commands
SetJoints([j1, j2, j3, j4, j5, j6, j7])

# Gripper commands
SetGripper("open") / SetGripper("close")
```

### Telemetry Integration

Thread-safe data sharing through enhanced `Telemetry` store:

```python
# Store poses
telemetry.set_generated_grasp_pose([x, y, z, rx, ry, rz])
telemetry.set_pickup_height_offset(0.05)  # 50mm

# Retrieve poses
grasp_pose = telemetry.get_generated_grasp_pose()
```

### Camera Integration

Uses existing `CameraManager` for:

- Depth image capture for GGCNN2
- RGB image capture for hand tracking
- 3D coordinate conversion

## Safety Features

1. **Hand Stability Detection**: Ensures hand is stable before placement
2. **Height Offset Preservation**: Maintains safe placement height
3. **Timeout Protection**: Prevents indefinite waiting states
4. **Error Handling**: Graceful error recovery and logging
5. **Emergency Stop Support**: Compatible with existing safety systems

## Configuration Parameters

### Hand Tracking

- `HAND_STABILITY_THRESHOLD`: 20mm movement tolerance
- `HAND_STABILITY_TIME_THRESHOLD`: 2.0 seconds stability requirement

### Grasp Detection

- `approach_height_offset`: 50mm safe approach distance
- `min_quality_threshold`: 0.5 minimum grasp quality

### Task Timeouts

- Hand tracking: 30 seconds maximum
- Grasp generation: 10 seconds maximum

## File Structure

```
src/states/
├── task_orchestrator.py          # Main workflow coordinator
├── pickup_task_sequencer.py      # Pickup sequence management
├── placement_task_sequencer.py   # Placement sequence management
├── hand_tracking_state.py        # Active hand tracking state
├── move_to_state.py              # Enhanced movement state
├── generate_pickup_state.py      # Enhanced grasp generation
└── gripper_state.py              # Existing gripper control

src/examples/
└── task_sequence_example.py      # Usage demonstrations
```

## Testing

Run the example to test the implementation:

```bash
cd src/examples
python task_sequence_example.py
```

This will demonstrate:

- Task orchestrator functionality
- Individual sequencer operations
- Progress monitoring
- Error handling

## Future Enhancements

Potential improvements:

1. **Multi-object Support**: Handle multiple objects in sequence
2. **Dynamic Height Calculation**: Real-time height adjustment
3. **Collision Avoidance**: Path planning integration
4. **Quality Metrics**: Grasp success rate tracking
5. **Operator Feedback**: Visual/audio feedback during handoff

## Troubleshooting

### Common Issues

1. **Hand Not Detected**: Check camera initialization and lighting
2. **Grasp Generation Failed**: Verify GGCNN2 model and depth data
3. **Movement Errors**: Check kinematics solver and joint limits
4. **Telemetry Errors**: Verify thread-safe access patterns

### Debug Logging

Enable detailed logging:

```python
import logging
logging.getLogger('states').setLevel(logging.DEBUG)
```

This implementation provides a robust, reusable foundation for object manipulation tasks while maintaining compatibility with the existing FSM architecture.
