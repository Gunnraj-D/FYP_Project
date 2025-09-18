# GG-CNN2 Visual Debugging Feature

This document describes the visual debugging feature implemented for the GG-CNN2 grasp detection system.

## Overview

The debugging feature allows you to visualize the input and output of the GG-CNN2 neural network during grasp detection. When enabled, it displays:

1. **Input Frame**: The processed depth image that is fed into the GG-CNN2 network
2. **Grasp Output**: The detected grasp pose overlaid on the original depth image

## Configuration

### Enabling Debug Mode

To enable visual debugging, set the following flag in `src/config/config.py`:

```python
# Visual debugging mode for GG-CNN2 grasp detection
VISUAL_DEBUG_MODE = True
```

**Default Value**: `False`

**Location**: At the top of `config.py` in the DEBUG CONFIGURATION section

## Implementation Details

### Files Modified

1. **`src/config/config.py`**

   - Added `VISUAL_DEBUG_MODE` boolean flag

2. **`src/object_detection/ggcnn2_module.py`**

   - Added `_visualize_input_frame()` method
   - Added `_visualize_grasp_output()` method
   - Integrated visualization calls into the `infer()` method
   - Added import for `VISUAL_DEBUG_MODE`

3. **`src/states/grasping_state.py`**
   - Added import for `VISUAL_DEBUG_MODE`
   - Added window cleanup in `exit()` method

### Visualization Features

#### Input Frame Visualization

- Displays the processed depth image that goes into the GG-CNN2 network
- Uses color mapping (JET colormap) for better depth visualization
- Shows the title "GG-CNN2 Input Frame"
- Handles NaN values and normalizes depth values for display

#### Grasp Output Visualization

- Overlays the detected grasp pose on the original depth image
- Shows grasp center as a green circle
- Draws grasp rectangle in green showing gripper orientation
- Displays approach direction as a red arrow
- Shows grasp quality, angle, and width as text overlay
- Scales coordinates back to original image resolution

### Visual Elements

- **Green Circle**: Grasp center point
- **Green Rectangle**: Grasp pose (gripper orientation and width)
- **Red Arrow**: Approach direction
- **White Text**: Grasp quality, angle, and width information

## Usage

### Basic Usage

1. Set `VISUAL_DEBUG_MODE = True` in `config.py`
2. Run your grasping system as normal
3. Two visualization windows will appear during grasp detection
4. Windows automatically close when the grasping state exits

### Example Script

A complete example is provided in `src/examples/debug_ggcnn2_example.py`:

```bash
# Enable debug mode first
# Edit config.py: VISUAL_DEBUG_MODE = True

# Run the example
python src/examples/debug_ggcnn2_example.py
```

### Integration with Existing Code

The debugging feature is automatically integrated into the existing grasp detection pipeline. No changes to your main application code are required - just enable the debug flag.

## Technical Details

### Performance Impact

- **When `VISUAL_DEBUG_MODE = False`**: No performance impact (visualization code is skipped)
- **When `VISUAL_DEBUG_MODE = True`**: Minimal overhead for image processing and display

### Dependencies

- OpenCV (`cv2`) for image visualization
- NumPy for image processing
- All dependencies are already part of the existing system

### Window Management

- Windows are created using OpenCV's `cv2.imshow()`
- Non-blocking display with `cv2.waitKey(1)`
- Automatic cleanup when grasping state exits
- Manual cleanup available via `cv2.destroyAllWindows()`

## Troubleshooting

### Common Issues

1. **No windows appear**

   - Check that `VISUAL_DEBUG_MODE = True` in config.py
   - Ensure you're running in an environment that supports GUI (not headless)

2. **Windows don't close**

   - Press any key when prompted
   - Call `cv2.destroyAllWindows()` manually
   - Restart the application

3. **Performance issues**
   - Set `VISUAL_DEBUG_MODE = False` for production use
   - Debug mode is intended for development/testing only

### Debug Information

The system logs debug information when visualizations are displayed:

```
DEBUG - Displayed input frame: GG-CNN2 Input Frame
DEBUG - Displayed grasp output: GG-CNN2 Grasp Output, Quality: 0.856
```

## Best Practices

1. **Development**: Enable debug mode during development and testing
2. **Production**: Always disable debug mode in production systems
3. **Testing**: Use debug mode to verify grasp detection accuracy
4. **Tuning**: Use visualizations to tune grasp detection parameters

## Future Enhancements

Potential improvements to the debugging feature:

1. Save visualization images to disk
2. Record visualization videos
3. Add more detailed grasp information overlays
4. Support for multiple grasp candidates visualization
5. Integration with logging systems for debug data export

## Support

For issues or questions about the debugging feature:

1. Check the logs for error messages
2. Verify configuration settings
3. Test with the provided example script
4. Review the implementation in the modified files
