# KUKA Robot Hand Tracking and Control System

A robust, safety-focused robot control system that tracks hand movements using computer vision and translates them into safe robot movements through inverse kinematics.

## 🚀 Recent Improvements

This system has been completely redesigned with a focus on **safety**, **reliability**, and **maintainability**. Here are the key improvements:

### 🔒 Safety Features

- **Joint Limit Validation**: All joint movements are validated against KUKA LBR iiwa 14 R820 specifications
- **Velocity & Acceleration Limits**: Real-time monitoring of joint velocities and accelerations
- **Emergency Stop System**: Automatic emergency stop on safety violations
- **Collision Detection**: Basic collision detection based on movement patterns
- **Communication Health Monitoring**: Automatic detection of communication failures

### 🛡️ Error Handling & Robustness

- **Comprehensive Exception Handling**: All critical operations are wrapped in try-catch blocks
- **Connection Retry Logic**: Automatic retry with exponential backoff for robot connections
- **Data Validation**: All input data is validated before processing
- **Graceful Degradation**: System continues operating safely even when some components fail
- **Automatic Recovery**: System attempts to recover from errors automatically

### 🏗️ Architecture Improvements

- **Modular Design**: Clear separation of concerns between components
- **Thread Safety**: All shared resources are properly protected
- **Configuration Management**: Centralized configuration with validation
- **Logging & Monitoring**: Comprehensive logging with performance metrics
- **Signal Handling**: Graceful shutdown on system signals

### 📊 Performance & Monitoring

- **Performance Metrics**: Real-time monitoring of loop times and detection rates
- **Adaptive Timing**: System adapts to performance constraints
- **Resource Management**: Efficient resource usage and cleanup
- **Debugging Support**: Extensive debugging information and error reporting

## 🏗️ System Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Hand Tracker  │    │  Safety Monitor │    │ Robot Interface │
│                 │    │                 │    │                 │
│ • Camera Setup  │    │ • Joint Limits  │    │ • FRI Protocol  │
│ • Hand Detection│    │ • Velocity Check│    │ • Connection    │
│ • 3D Conversion │    │ • Acceleration  │    │ • Joint Control │
│ • Confidence    │    │ • Collision     │    │ • Emergency Stop│
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Shared State Manager                         │
│                                                                 │
│ • Thread-safe data sharing                                     │
│ • Data validation and freshness checks                         │
│ • Error tracking and system status                             │
└─────────────────────────────────────────────────────────────────┘
         │                       │                       │
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Main Controller│    │ Kinematics      │    │ Camera Transform│
│                 │    │ Solver          │    │ Module          │
│ • Control Loop  │    │ • Forward Kin.  │    │ • Coordinate    │
│ • Safety Checks │    │ • Inverse Kin.  │    │   Transform     │
│ • Error Handling│    │ • Joint Limits  │    │ • Camera Calib. │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## 🚀 Getting Started

### Prerequisites

- Python 3.8+
- KUKA LBR iiwa 14 R820 robot
- RealSense camera (D415/D435)
- FRI (Fast Robot Interface) enabled on robot

### Installation

1. **Clone the repository**

   ```bash
   git clone <repository-url>
   cd FYP_Project
   ```

2. **Create virtual environment**

   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. **Install dependencies**

   ```bash
   pip install -r src/requirements.txt
   ```

4. **Configure the system**

   - Update `src/config.py` with your robot's IP address
   - Ensure URDF file path is correct
   - Verify hand model file path

5. **Run the system**
   ```bash
   cd src
   python main.py
   ```

## ⚙️ Configuration

### Robot Configuration

```python
# Robot connection settings
FRI_IP = "172.24.201.001"  # Your robot's IP address
FRI_PORT = 30200            # FRI port
FRI_CONNECTION_TIMEOUT = 5.0  # Connection timeout in seconds
FRI_RETRY_ATTEMPTS = 3      # Number of connection retry attempts
```

### Safety Thresholds

```python
# Joint limits (radians)
JOINT_LIMITS = {
    'min': [-2.967, -2.094, -2.967, -2.094, -2.967, -2.094, -3.142],
    'max': [2.967, 2.094, 2.967, 2.094, 2.967, 2.094, 3.142]
}

# Safety limits
MAX_JOINT_VELOCITY = 1.0      # rad/s
MAX_JOINT_ACCELERATION = 2.0  # rad/s²
EMERGENCY_STOP_DISTANCE = 0.1 # meters
```

### Hand Tracking Configuration

```python
# Detection parameters
MIN_HAND_CONFIDENCE = 0.5     # Minimum confidence for hand detection
MAX_HAND_DISTANCE = 2.0       # Maximum tracking distance (meters)
MIN_HAND_DISTANCE = 0.1       # Minimum tracking distance (meters)
```

## 🔧 Usage

### Basic Operation

1. **Start the system**: The system automatically initializes all components
2. **Hand tracking**: Position your hand in front of the camera
3. **Robot movement**: The robot will follow your hand movements safely
4. **Emergency stop**: Press Ctrl+C or the system will stop automatically on safety violations

### Safety Features

- **Automatic joint limit checking**: Prevents robot from reaching unsafe positions
- **Velocity monitoring**: Ensures smooth, safe movements
- **Collision detection**: Monitors for potential collisions
- **Communication health**: Detects connection issues automatically

### Monitoring and Debugging

- **Log files**: Check `robot_control.log` for detailed system information
- **Performance metrics**: Real-time monitoring of system performance
- **Safety status**: Continuous monitoring of all safety parameters

## 🧪 Testing

### Unit Tests

```bash
cd src
python -m pytest test_*.py -v
```

### Integration Tests

```bash
# Test hand tracking without robot
python -m pytest test_hand_tracking.py -v

# Test robot interface (requires robot connection)
python -m pytest test_robot_interface.py -v
```

### Safety Tests

```bash
# Test safety monitoring
python -m pytest test_safety_monitor.py -v
```

## 📁 Project Structure

```
src/
├── main.py                    # Main application entry point
├── config.py                  # Configuration and constants
├── safety_monitor.py          # Safety monitoring system
├── hand_detection_module.py   # Hand tracking and detection
├── FRI_Interface.py          # Robot communication interface
├── kinematics_solver.py       # Forward/inverse kinematics
├── shared_state_joints.py     # Thread-safe data sharing
├── camera_transform_module.py # Camera coordinate transforms
├── requirements.txt           # Python dependencies
└── resources/
    └── models/
        └── iiwa14.urdf       # Robot model file
```

## 🚨 Safety Guidelines

### Before Operation

1. **Verify robot workspace**: Ensure no obstacles in robot path
2. **Check emergency stop**: Verify emergency stop button is accessible
3. **Test hand tracking**: Ensure camera can detect hand movements
4. **Verify joint limits**: Check that robot can reach all required positions

### During Operation

1. **Monitor system status**: Watch for safety warnings in logs
2. **Keep clear of robot**: Maintain safe distance from robot workspace
3. **Watch for errors**: Monitor for any error messages or warnings
4. **Be ready to stop**: Know how to trigger emergency stop

### Emergency Procedures

1. **Immediate stop**: Press Ctrl+C or emergency stop button
2. **Check robot**: Verify robot has stopped moving
3. **Review logs**: Check log files for error information
4. **Restart safely**: Only restart after resolving safety issues

## 🔍 Troubleshooting

### Common Issues

#### Connection Problems

- **Robot not responding**: Check IP address and network connectivity
- **Connection timeout**: Verify robot is in FRI mode
- **JVM errors**: Restart Python application

#### Hand Tracking Issues

- **No hand detection**: Check camera connection and lighting
- **Poor accuracy**: Ensure hand is well-lit and clearly visible
- **Tracking lag**: Check system performance and loop timing

#### Safety Violations

- **Joint limit errors**: Check robot workspace and movement range
- **Velocity violations**: Reduce movement speed or increase limits
- **Communication timeouts**: Check network stability

### Debug Mode

Enable debug logging by modifying the logging level in `main.py`:

```python
logging.basicConfig(level=logging.DEBUG)
```

### Performance Monitoring

Monitor system performance through the built-in metrics:

- Loop execution time
- Hand detection rate
- Safety check frequency
- Error rates

## 📈 Performance Optimization

### System Tuning

1. **Loop rate adjustment**: Modify `LOOP_RATE_MS` in config
2. **Camera settings**: Adjust camera resolution and frame rate
3. **Detection parameters**: Tune hand detection confidence thresholds
4. **Safety monitoring**: Adjust safety check intervals

### Resource Management

- **Memory usage**: Monitor memory consumption during operation
- **CPU usage**: Check CPU utilization for bottlenecks
- **Network latency**: Monitor robot communication delays

## 🤝 Contributing

### Development Setup

1. **Fork the repository**
2. **Create feature branch**: `git checkout -b feature/new-feature`
3. **Make changes**: Follow coding standards and add tests
4. **Test thoroughly**: Ensure all tests pass
5. **Submit pull request**: Include detailed description of changes

### Coding Standards

- **Python style**: Follow PEP 8 guidelines
- **Documentation**: Add docstrings to all functions
- **Testing**: Include unit tests for new features
- **Safety**: Prioritize safety in all new code

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## ⚠️ Disclaimer

This software is provided "as is" without warranty. Robot control systems can be dangerous if not used properly. Always follow safety guidelines and test thoroughly before use in production environments.

## 📞 Support

For questions, issues, or contributions:

- Create an issue on GitHub
- Contact the development team
- Check the documentation and troubleshooting guides

---

**⚠️ Safety First**: This system controls industrial robots. Always prioritize safety and follow proper procedures.
