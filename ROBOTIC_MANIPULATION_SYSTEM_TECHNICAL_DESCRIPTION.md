# Robotic Object Manipulation System - Technical Description

## System Overview

This Python application implements a sophisticated robotic object manipulation system designed for a 7-DOF KUKA iiwa robot arm. The system uses a finite state machine (FSM) architecture to orchestrate the complete workflow of picking up objects from a table and placing them into a human operator's hand. The robot is controlled via OPC UA communication, with object detection and grasping guided by a RealSense camera mounted on the robot's end-effector.

### Core Purpose

The system enables autonomous object manipulation through:

- **Object Detection**: GGCNN2-based grasp pose generation from depth images
- **Hand Tracking**: MediaPipe-based human hand detection and tracking
- **Robotic Control**: OPC UA communication for real-time robot control
- **State Management**: FSM-based task sequencing and coordination

## Architecture Overview

The system follows a modular, layered architecture with clear separation of concerns:

```
┌─────────────────────────────────────────────────────────────┐
│                    RobotControlSystem                       │
│                   (Main Coordinator)                        │
└─────────────────────────────────────────────────────────────┘
                              │
        ┌─────────────────────┼─────────────────────┐
        │                     │                     │
┌───────▼────────┐  ┌─────────▼─────────┐  ┌────────▼────────┐
│   StateMachine │  │   TelemetryStore   │  │   CommandBus    │
│   (FSM Core)   │  │  (Shared State)   │  │ (Command Queue) │
└────────────────┘  └───────────────────┘  └─────────────────┘
        │                     │                     │
        │                     │                     │
┌───────▼────────┐  ┌─────────▼─────────┐  ┌────────▼────────┐
│ TaskSequencers │  │   CameraManager    │  │   OPCClient     │
│ (Pickup/Place) │  │  (RealSense)       │  │ (Robot Comm)    │
└────────────────┘  └───────────────────┘  └─────────────────┘
        │                     │                     │
        │                     │                     │
┌───────▼────────┐  ┌─────────▼─────────┐  ┌────────▼────────┐
│   GGCNN2Module │  │ HandDetectionModule│  │KinematicsSolver │
│ (Grasp Detect) │  │  (MediaPipe)       │  │   (ikpy)        │
└────────────────┘  └───────────────────┘  └─────────────────┘
```

## Core Components

### 1. State Machine Architecture (`src/states/`)

**Base State System**:

- `BaseState`: Abstract base class defining state interface (`enter()`, `execute()`, `is_complete()`, `exit()`)
- `StateMachine`: Manages state transitions and execution
- `StateContext`: Immutable context object containing shared resources (telemetry, commands, camera, OPC client, kinematics)

**State Implementations**:

- `GeneratePickupState`: Uses GGCNN2 to detect objects and generate grasp poses
- `MoveToState`: Executes robot motion to target poses or locations
- `GripperControlState`: Controls gripper open/close operations
- `HandTrackingState`: Tracks human hand for object placement

**Task Sequencers**:

- `PickupTaskSequencer`: Orchestrates 7-step pickup sequence (pre-pickup → grasp generation → approach → grasp → lift)
- `PlacementTaskSequencer`: Orchestrates 4-step placement sequence (approach → hand tracking → handoff → release)
- `TaskOrchestrator`: Coordinates between pickup and placement sequences

### 2. Communication Layer (`src/IO_handling/`, `src/control/`)

**OPC UA Client** (`OPCClient`):

- Async communication with KUKA robot via OPC UA protocol
- Dedicated background thread for robot communication
- Automatic reconnection and error handling
- Command batching and redundant write detection
- Real-time telemetry updates (joint positions, robot status)

**Command Bus** (`CommandBus`):

- Thread-safe command queue with priority handling
- Command types: `SetJoints`, `SetGripper`, `EmergencyStop`
- Latest-command-wins semantics for efficiency

**Telemetry Store** (`Telemetry`):

- Thread-safe shared state management with fine-grained locking
- Tracks: robot state, hand position, grasp poses, object heights
- Provides synchronized access across multiple threads

### 3. Perception System (`src/camera_management/`, `src/object_detection/`, `src/hand_detection/`)

**Camera Management** (`CameraManager`):

- RealSense camera initialization and frame capture
- Depth-to-color alignment and coordinate transformation
- Pixel-to-3D conversion using camera intrinsics

**Object Detection** (`GGcnn2Module`):

- GGCNN2 neural network for grasp pose generation
- Depth image preprocessing and inference
- 2D grasp parameters → 3D pose conversion
- Camera-to-base frame transformation
- Inverse kinematics for joint angle computation

**Hand Detection** (`HandTracker`):

- MediaPipe-based hand landmark detection
- Palm centroid and radius calculation
- Hand stability tracking with configurable thresholds
- Real-time 3D hand position estimation

**Table Reference** (`TableReferenceModule`):

- Per-pixel table surface depth modeling
- Exponential moving average updates
- Object height above table computation
- Robust to flat objects and surface variations

### 4. Kinematics (`src/kinematics/`)

**Inverse Kinematics Solver** (`InverseKinematicsSolver`):

- ikpy-based forward and inverse kinematics
- KUKA iiwa14 URDF model integration
- Joint limit enforcement and workspace validation
- TCP pose computation and error checking

### 5. Configuration (`src/config/`)

**Centralized Configuration** (`config.py`):

- System parameters (loop rates, timeouts)
- Robot model paths and kinematic chain configuration
- Camera calibration and transformation parameters
- Grasp detection thresholds and execution parameters
- Safety limits and error recovery settings

## State Machine Workflow

### Complete Object Manipulation Sequence

1. **System Initialization**:

   - Camera initialization and calibration
   - OPC UA connection establishment
   - Robot program startup
   - Hand tracking initialization

2. **Pickup Sequence** (`PickupTaskSequencer`):

   ```
   MoveToState(PRE_PICKUP_POSE)
   → GeneratePickupState() [GGCNN2 inference]
   → GripperControlState('open')
   → MoveToState(approach_pose)
   → MoveToState(grasp_pose)
   → GripperControlState('close')
   → MoveToState(approach_pose) [lift object]
   ```

3. **Placement Sequence** (`PlacementTaskSequencer`):
   ```
   MoveToState(HANDOFF_APPROACH_POSE)
   → HandTrackingState() [track human hand]
   → MoveToState(calculated_handoff_pose)
   → GripperControlState('open') [release object]
   ```

### State Execution Flow

Each state follows the pattern:

1. `enter()`: Initialize state-specific resources
2. `execute()`: Perform state logic (repeated until completion)
3. `is_complete()`: Check completion criteria
4. `exit()`: Cleanup and transition preparation

## Data Flow and Communication

### Telemetry Flow

```
Robot (OPC UA) → OPCClient → TelemetryStore ← StateMachine
Camera → CameraManager → GGCNN2Module/HandTracker → TelemetryStore
```

### Command Flow

```
StateMachine → CommandBus → OPCClient → Robot (OPC UA)
```

### Coordinate Transformations

```
Camera Frame → TCP Frame → Base Frame
[Hand/Grasp positions transformed through kinematic chain]
```

## Current Implementation Status

### ✅ Implemented Components

**Core Infrastructure**:

- Complete FSM architecture with state management
- Thread-safe telemetry and command systems
- OPC UA communication with reconnection logic
- Camera management and coordinate transformations
- Kinematics solver with URDF integration

**Perception Modules**:

- GGCNN2 integration for grasp detection
- MediaPipe hand tracking with stability detection
- Table reference depth modeling
- Real-time depth image processing

**Task Sequencing**:

- Pickup and placement task sequencers
- State orchestration and progress tracking
- Error handling and recovery mechanisms

### ⚠️ Partially Implemented/Stubbed

**Integration Gaps**:

- State machine not fully integrated into main control loop
- Task orchestrator not connected to main system
- Some state transitions may be incomplete
- Error recovery mechanisms need testing

**Missing Components**:

- Gripper control implementation (OPC UA nodes not mapped)
- Hand tracking state implementation
- Complete end-to-end testing pipeline
- Safety monitoring and collision detection

### 🔧 Development Status

**Individual Modules**: Well-developed and functional
**System Integration**: Incomplete - modules exist but not fully connected
**End-to-End Pipeline**: Not yet operational

## Technical Strengths

1. **Modular Architecture**: Clear separation of concerns with well-defined interfaces
2. **Thread Safety**: Comprehensive locking and synchronization mechanisms
3. **Robust Communication**: OPC UA client with automatic reconnection
4. **Advanced Perception**: State-of-the-art grasp detection and hand tracking
5. **Configurable System**: Centralized configuration management
6. **Error Handling**: Comprehensive error recovery and logging

## Development Gaps and Next Steps

### Immediate Priorities

1. **System Integration**:

   - Connect state machine to main control loop
   - Integrate task orchestrator with robot control system
   - Implement missing state transitions

2. **Gripper Control**:

   - Map gripper OPC UA nodes
   - Implement gripper status feedback
   - Add gripper force control

3. **End-to-End Testing**:
   - Create complete workflow tests
   - Validate state machine transitions
   - Test error recovery scenarios

### Medium-term Goals

1. **Safety Systems**: Collision detection and emergency stop integration
2. **Performance Optimization**: Reduce latency in perception pipeline
3. **User Interface**: Real-time system monitoring and control interface
4. **Calibration Tools**: Automated camera-robot calibration procedures

## Conclusion

This robotic manipulation system represents a sophisticated, well-architected solution for autonomous object manipulation. The modular design, comprehensive state management, and advanced perception capabilities provide a solid foundation for robotic handoff operations. While individual components are well-developed, the primary development focus should be on system integration and end-to-end pipeline completion to achieve a fully operational robotic manipulation system.

The system demonstrates strong software engineering practices with clear separation of concerns, robust error handling, and comprehensive configuration management. The FSM-based approach provides excellent maintainability and extensibility for future enhancements.
