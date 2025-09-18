# Codebase Technical Summary: Robot Hand Tracking System

## 1. Overall Purpose & Domain

- **Core Function:** A non-ROS robot control application that enables a robot arm with mounted camera to detect objects, pick them up, and place them into a human operator's hand.
- **Primary Task:** Autonomous object manipulation with human-robot collaboration, specifically detecting graspable objects using computer vision and delivering them to a tracked human hand.
- **Operating Environment:** Structured industrial/laboratory setting with controlled lighting, using a KUKA LBR iiwa robot arm with wrist-mounted RGB-D camera.

## 2. Core Technologies & Libraries

| Category                  | Technology/Library         | Notes                                                       |
| ------------------------- | -------------------------- | ----------------------------------------------------------- |
| Language(s)               | Python 3.11                | Primary control and computer vision language                |
| Communication Protocol(s) | OPC UA                     | Real-time communication with robot controller via Ethernet  |
| Vision & ML Libraries     | OpenCV, PyTorch, MediaPipe | Object detection (GGCNN2), hand tracking, depth processing  |
| Robotics Libraries        | ikpy                       | Inverse kinematics calculations for 7-DOF robot arm         |
| Core Libraries            | asyncio, NumPy, SciPy      | Asynchronous I/O, numerical computation, spatial transforms |
| Development Environment   | Python venv                | Standard Python virtual environment                         |

## 3. High-Level Architecture

- **Architectural Pattern:** Finite State Machine (FSM) with modular component architecture
- **Control Loop:** A main control loop (`robot_control_system.py`) runs at 50Hz, executing a state machine that coordinates between perception, planning, and actuation. Each state performs specific tasks (object detection, hand tracking, grasping) by processing sensor data, calculating actions via kinematics, and sending commands through a decoupled command bus to the robot controller.
- **Key Software Components:**

  - **State Machine:** Manages task sequence and transitions between operational states (`IDLE`, `TRACKING`, `PICKUP`, `PLACE`, `GRASPING`)
  - **Communication Interface:** The `OPCClient` class handles all low-level OPC UA communication with the robot controller in a dedicated background thread
  - **Vision System:** Modular components for camera management (`CameraManager`), object detection (`GGcnn2Module`), and hand detection (`HandTracker`)
  - **Kinematics Solver:** `InverseKinematicsSolver` calculates required joint angles to reach specific end-effector poses using ikpy
  - **Control Bus:** Central message queue (`CommandBus`) for decoupling components and managing command flow
  - **Telemetry Store:** Thread-safe repository (`Telemetry`) for storing and accessing robot's current state (joint angles, hand position, grasp poses)

- **Data & Control Flow:** The `main.py` initializes the `RobotControlSystem` and starts the state machine. The vision system provides object coordinates and hand positions. The active state (e.g., `GeneratePickupState`) uses the kinematics solver to determine target joint angles. A `SetJoints` command is placed on the command bus. The `OPCClient` picks up commands and transmits them to the robot controller via OPC UA. Telemetry is continuously read back to update the system's world model.

## 4. Key Directory & Component Breakdown

| Path                      | Responsibility                                                               | Key Technologies/Files                                          |
| ------------------------- | ---------------------------------------------------------------------------- | --------------------------------------------------------------- |
| `/src/states`             | Defines all states for the FSM and contains state machine execution logic    | `state_machine.py`, `base_state.py`, `generate_pickup_state.py` |
| `/src/IO_handling`        | Handles low-level hardware communication with OPC UA server                  | `opc_client.py`, `asyncua`                                      |
| `/src/kinematics`         | Provides kinematic calculations (forward/inverse kinematics) for 7-DOF robot | `kinematics_solver.py`, `ikpy`                                  |
| `/src/object_detection`   | Implements GGCNN2 model for identifying graspable objects in depth images    | `ggcnn2_module.py`, `ggcnn2.py`                                 |
| `/src/hand_detection`     | Implements computer vision logic to detect and track operator's hand         | `hand_detection_module.py`, `mediapipe`                         |
| `/src/control`            | Core infrastructure for managing commands and robot state (telemetry)        | `command_bus.py`, `telemetry_store.py`                          |
| `/src/camera_management`  | Manages RGB-D camera operations and coordinate transformations               | `camera_manager.py`, `camera_transform_module.py`               |
| `/src/config`             | Centralized configuration parameters for entire application                  | `config.py`                                                     |
| `/resources/ml_models`    | Stores pre-trained machine learning model files                              | `.pt` files (GGCNN2), `.task` files (MediaPipe)                 |
| `/resources/robot_models` | Contains URDF robot model files for kinematics calculations                  | `.urdf` files (KUKA iiwa)                                       |

## 5. State Management & Configuration

- **Real-time State:** The robot's live state (joint positions, status, hand position) is stored in-memory in the `Telemetry` object with fine-grained thread locks, updated by the `OPCClient` at 20Hz
- **Task State:** High-level task progress is managed by the active state within the `StateMachine`, with states like `GeneratePickupState` handling object detection and grasp generation
- **Configuration:** System parameters (IP addresses, timeouts, node names, kinematic chain configuration) are managed in `config/config.py` and loaded at startup

## 6. Hardware & External System Integrations

- **Primary Hardware:** KUKA LBR iiwa 14 R820 robot arm (7-DOF manipulator)
- **Communication Link:** Communicates with robot controller over Ethernet via OPC UA protocol (`opc.tcp://172.24.200.1:4840/`)
- **Sensors:** Wrist-mounted RGB-D camera (Intel RealSense) for object detection and hand tracking
- **Gripper:** Integrated gripper with open/close control via OPC UA nodes

## 7. Runtime & Execution

- **Setup:** `pip install -r src/requirements.txt` (installs OpenCV, PyTorch, asyncua, ikpy, etc.)
- **Entrypoint:** The system is launched by running `python src/main.py` or `python src/robot_control_system.py`
- **Dependencies:** Key dependencies include `asyncua` (OPC UA), `torch` (GGCNN2), `opencv-python` (vision), `ikpy` (kinematics), `mediapipe` (hand tracking)
- **Threading Model:** OPC UA communication runs in a separate background thread (`OPCClient._run_async_loop`) to avoid blocking the main control loop, managed via Python's `threading` and `asyncio`

## 8. Testing & Validation

- **Testing Strategy:** Unit tests for individual modules, particularly the vision components and GGCNN2 integration
- **Test Runner:** Custom test scripts in `/tests/gcnn2_tests/` directory
- **Key Commands:** `python tests/run_tests.py` for running GGCNN2 model tests
