# Hand-Eye Recalibration Guide

## Current Situation

- **Simple mode**: ✅ Works perfectly (0mm offset)
- **Calibrated mode**: ❌ Has 278mm lateral offset error
- **Default**: Simple mode (recommended)

## When to Recalibrate

You only need to recalibrate if:

1. Camera physical mount changes
2. You need the calibrated transform for some reason
3. Simple mode becomes inaccurate

**For most use cases, simple mode is sufficient!**

## How to Recalibrate (Future Reference)

### Prerequisites

1. Ensure camera is securely mounted (won't move during calibration)
2. Have calibration target ready (ChArUco board or known object)
3. Robot can move to various poses safely

### Run Calibration

```bash
cd src
python calibration/semi_automated_calibration.py
```

### Calibration Tips for Better Results

1. **Use diverse poses** (at least 10-15):

   - Different X, Y, Z positions
   - Different orientations
   - Cover the full workspace
   - Avoid symmetrical poses only

2. **Check each pose**:

   - Marker/object clearly visible in camera
   - Good lighting, no glare
   - Robot fully stopped before capture
   - Camera image is sharp (not motion-blurred)

3. **Verify results**:
   ```bash
   python src/debug_coordinate_transform.py
   ```
   Should show < 10mm offset for good calibration

### Update the Matrix

After successful calibration, update in `src/config/config.py`:

```python
HAND_EYE_MATRIX_CALIBRATED = np.array([
    [... new values from calibration ...],
    [...],
    [...],
    [0.0, 0.0, 0.0, 1.0]
], dtype=np.float32)

# Then switch to calibrated mode
CAMERA_TRANSFORM_MODE = 'calibrated'
```

## Troubleshooting Bad Calibrations

### Symptom: Large lateral offset (like current 278mm)

**Causes:**

- Camera moved between calibration and use
- Calibration target not detected correctly
- Robot pose data mismatched with images

**Solution:**

- Secure camera firmly before starting
- Use fresh calibration session
- Verify each pose before capturing

### Symptom: Small but consistent offset (5-20mm)

**Causes:**

- Camera-to-flange mounting offset not accounted for
- Slight calibration error

**Solution:**

- Add translation offset to matrix manually if consistent
- Or use more calibration poses

### Symptom: Rotation errors (grasps at wrong angle)

**Causes:**

- Calibration target orientation errors
- Not enough rotational diversity in poses

**Solution:**

- Include poses with camera rotated at different angles
- Check calibration target is mounted square

## For Now: Stick with Simple Mode

Since simple mode works perfectly (0mm offset), there's no urgent need to recalibrate. Only recalibrate if:

- Camera mounting changes
- You observe drift in accuracy over time
- You have specific requirements for calibrated mode

The simple mode assumes:

- Camera optical center at TCP
- Camera pointing down (180° X rotation)
- No additional offsets

This is accurate as long as the camera stays in the same physical mount position.


