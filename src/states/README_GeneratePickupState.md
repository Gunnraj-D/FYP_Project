# GeneratePickupState Documentation

## Overview

The `GeneratePickupState` is a specialized state that handles the process of generating optimal grasp poses for objects using the GGCNN2 deep learning module. This state captures depth images, runs object detection and grasp synthesis, validates the results, and stores the best grasp pose in telemetry for subsequent pickup operations.

## Purpose

This state serves as a preprocessing step in the robot's pickup workflow:

1. **Object Detection**: Uses GGCNN2 to detect objects in the scene
2. **Grasp Synthesis**: Generates optimal grasp poses for detected objects
3. **Validation**: Ensures grasp poses meet quality and feasibility criteria
4. **Storage**: Saves the best grasp pose in telemetry for pickup execution

## Key Features

### Intelligent Grasp Generation

- Uses GGCNN2 deep learning model for robust object detection
- Generates multiple grasp candidates and selects the best one
- Validates grasp quality against configurable thresholds

### Quality Validation

- **Quality Threshold**: Ensures grasp quality meets minimum requirements
- **Joint Feasibility**: Validates that joint angles are within robot limits
- **Grasp Width**: Checks grasp width against min/max constraints
- **Pose Validity**: Ensures grasp pose is geometrically sound

### Performance Optimization

- **Frame Throttling**: Processes frames at configurable intervals (default: 500ms)
- **Timeout Protection**: Prevents infinite loops with configurable timeout
- **Attempt Limiting**: Limits maximum attempts to prevent resource exhaustion

### Robust Error Handling

- Graceful handling of camera failures
- Recovery from GGCNN2 inference errors
- Comprehensive logging for debugging

## Configuration

The state behavior is controlled by configuration parameters in `config.py`:

```python
# GGCNN2 model configuration
GGCNN2_MODEL_PATH = "path/to/model.pt"

# Grasp detection parameters
GRASP_DETECTION_CONFIG = {
    'min_quality_threshold': 0.5,      # Minimum grasp quality
    'max_grasp_width': 100.0,         # Maximum grasp width (mm)
    'min_grasp_width': 20.0,          # Minimum grasp width (mm)
    'approach_angle': -90.0,          # Approach angle (degrees)
}

# Grasp execution parameters
GRASP_EXECUTION_CONFIG = {
    'retry_attempts': 3,              # Maximum retry attempts
    'grasp_duration': 2.0,             # Grasp execution time
}
```

## State Lifecycle

### 1. Enter Phase

- Initializes GGCNN2 module with model loading
- Resets state variables and timers
- Validates camera and robot connectivity

### 2. Execute Phase

- Captures depth frames from camera
- Runs GGCNN2 inference on depth images
- Validates generated grasp poses
- Stores best results in telemetry

### 3. Exit Phase

- Logs final generation statistics
- Cleans up resources
- Resets state variables

## Usage Example

```python
from states.context import StateContext
from states.generate_pickup_state import GeneratePickupState

# Create state context
context = StateContext(
    telemetry=telemetry,
    commands=command_bus,
    camera=camera_manager,
    opc=opc_client,
    ik=kinematics_solver
)

# Create and run the state
generate_pickup_state = GeneratePickupState(context)
generate_pickup_state.enter()

# Execute until completion
while not generate_pickup_state.is_complete():
    generate_pickup_state.execute()
    time.sleep(0.1)

# Get results
if generate_pickup_state.get_grasp_joint_angles() is not None:
    print("Grasp pose generated successfully!")
    print(f"Joint angles: {generate_pickup_state.get_grasp_joint_angles()}")
    print(f"Quality: {generate_pickup_state.get_grasp_quality():.3f}")

generate_pickup_state.exit()
```

## Integration with State Machine

The `GeneratePickupState` integrates seamlessly with the existing state machine:

```python
# In state machine implementation
from states.states_enum import States
from states.generate_pickup_state import GeneratePickupState

def create_state(state_type: States, context: StateContext):
    if state_type == States.GENERATE_PICKUP:
        return GeneratePickupState(context)
    # ... other states
```

## Telemetry Integration

The state stores generated grasp poses in the telemetry system:

```python
# Access generated grasp pose
pickup_joints = telemetry.get_pickup_pose_joints()
```

The stored joint angles can be used by subsequent states (e.g., `MoveToState`, `GraspingState`) to execute the pickup operation.

## API Reference

### Constructor

```python
GeneratePickupState(context: StateContext)
```

### Main Methods

- `enter()` - Initialize state and GGCNN2 module
- `execute()` - Main execution loop for grasp generation
- `is_complete()` - Check if generation is complete
- `exit()` - Clean up resources and log results

### Result Access Methods

- `get_grasp_quality() -> float` - Get best grasp quality score
- `get_grasp_joint_angles() -> Optional[List[float]]` - Get joint angles
- `get_grasp_pose_base() -> Optional[List[float]]` - Get base frame pose
- `get_generation_stats() -> Dict` - Get generation statistics

## Performance Considerations

### Timing

- **Frame Processing**: Throttled to 500ms intervals to balance responsiveness and CPU usage
- **Timeout**: Default 10-second timeout prevents infinite loops
- **Attempt Limiting**: Maximum 3 attempts to prevent resource exhaustion

### Resource Usage

- **GPU Memory**: GGCNN2 model requires GPU memory for inference
- **CPU Usage**: Frame processing and validation are CPU-intensive
- **Camera Access**: Requires exclusive access to depth camera

### Optimization Tips

1. **Adjust Frame Interval**: Increase for lower CPU usage, decrease for faster response
2. **Quality Threshold**: Lower threshold for more candidates, higher for better quality
3. **Timeout Settings**: Adjust based on expected scene complexity

## Error Handling

The state handles various error conditions gracefully:

### Camera Errors

- Missing depth frames
- Camera initialization failures
- Frame processing errors

### GGCNN2 Errors

- Model loading failures
- Inference errors
- Invalid output processing

### Validation Errors

- Low quality grasps
- Invalid joint angles
- Geometric constraints violations

All errors are logged with appropriate detail levels for debugging.

## Future Enhancements

Potential improvements for future versions:

1. **Multi-Object Support**: Generate grasps for multiple objects simultaneously
2. **Grasp Ranking**: Rank multiple grasp candidates by quality and feasibility
3. **Adaptive Parameters**: Adjust thresholds based on scene characteristics
4. **Performance Metrics**: Add detailed timing and success rate statistics
5. **Visualization**: Add debug visualization for grasp candidates

## Troubleshooting

### Common Issues

**No Valid Grasp Generated**

- Check camera calibration and depth quality
- Verify GGCNN2 model is loaded correctly
- Adjust quality threshold in configuration
- Ensure objects are within camera field of view

**High CPU Usage**

- Increase frame processing interval
- Reduce GGCNN2 inference frequency
- Check for camera frame rate issues

**Timeout Errors**

- Increase timeout duration
- Check for camera connectivity issues
- Verify GGCNN2 model performance

### Debug Logging

Enable debug logging to troubleshoot issues:

```python
import logging
logging.getLogger('states.generate_pickup_state').setLevel(logging.DEBUG)
```

This will provide detailed information about:

- Frame processing times
- Grasp validation results
- GGCNN2 inference details
- State transition information
