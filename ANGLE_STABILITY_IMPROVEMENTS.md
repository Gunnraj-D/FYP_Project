# Angle Stability Improvements for Grasp Detection

## Problem Statement

GR-ConvNet (and GGCNN2) can produce inconsistent grasp angles across frames, even when viewing the same static object. This is caused by:

1. **Camera noise** - Depth sensor noise affects predictions
2. **Network uncertainty** - Multiple valid grasps may have similar quality scores
3. **Ambiguous objects** - Symmetric objects have multiple equally good angles
4. **Edge effects** - Quality peaks near object edges can be unstable

## Solution 1: Temporal Filtering ✅ IMPLEMENTED

**Multi-frame averaging stabilizes predictions over time.**

### Configuration (config.py)

```python
GRASP_DETECTION_CONFIG = {
    # Enable temporal filtering
    'temporal_filter_enabled': True,

    # Number of frames to average (3-7 recommended)
    'temporal_window_size': 5,

    # Filter type: 'circular_mean', 'median', or 'ema'
    'temporal_filter_type': 'circular_mean',

    # EMA smoothing factor (0.1-0.5, lower=smoother)
    'temporal_ema_alpha': 0.3,

    # Outlier rejection threshold (degrees, None to disable)
    'temporal_outlier_threshold_deg': 30,
}
```

### How It Works

1. **Circular Mean** (default):

   - Averages last N angle predictions
   - Uses circular statistics (handles wrap-around at ±90°)
   - Best for general use

2. **Median Filter**:

   - Takes median of last N predictions
   - More robust to outliers
   - Good for noisy environments

3. **Exponential Moving Average (EMA)**:

   - Weighted average favoring recent frames
   - Smoothest output, responds to changes gradually
   - Best for real-time continuous grasping

4. **Outlier Rejection**:
   - Discards angles that differ > threshold from recent history
   - Prevents sudden jumps from bad predictions
   - Adjustable threshold (default 30°)
   - **Requires 3+ samples** before activating (prevents first-frame bias)

### Usage

```python
# Already integrated into GGcnn2Module
# Just enable in config and run as normal

# The module will automatically:
# 1. Store angle history in circular buffer
# 2. Apply selected filter type
# 3. Reject outliers if configured
# 4. Return temporally-filtered angle
```

### Tuning Guide

| Scenario       | Window Size | Filter Type   | Outlier Threshold |
| -------------- | ----------- | ------------- | ----------------- |
| Static objects | 5-7         | circular_mean | 30°               |
| Moving objects | 3-4         | ema (α=0.4)   | 45°               |
| Noisy depth    | 5-7         | median        | 25°               |
| Fast response  | 3           | ema (α=0.5)   | None              |

### Debug Logs

When enabled, you'll see:

```
✨ Temporal filtering enabled: circular_mean, window=5
🔄 Temporal filter: raw=45.3° → filtered=42.1° (history size: 5)
📐 Angle: raw=45.3° → filtered=42.1°
```

---

## Solution 2: Increase Spatial Gaussian Blur (Already Applied)

GR-ConvNet already applies Gaussian filtering (σ=2.0) to angle maps **spatially** (within each frame). This smooths out small-scale noise.

**To increase spatial smoothing** (if temporal filtering isn't enough):

```python
# In grasp_detector_module.py, postprocess() method (line ~593)
q_blur = cv2.GaussianBlur(q_np, (7, 7), 3)  # Increase from (5,5), sigma=2
```

⚠️ **Trade-off**: More spatial blur = smoother angles BUT less precise localization.

---

## Solution 3: Top-K Consensus (Alternative Approach)

Instead of picking the single best grasp, collect top-K candidates and find the most stable angle.

### Implementation (Optional)

```python
def _get_consensus_angle(self, top_candidates):
    """Pick angle that appears most frequently in top-K grasps."""
    angles = [c['angle'] for c in top_candidates]

    # Cluster angles (account for 180° gripper symmetry)
    angle_bins = np.linspace(-np.pi/2, np.pi/2, 18)  # 10° bins
    hist, _ = np.histogram(angles, bins=angle_bins)

    # Pick most common bin
    max_bin = np.argmax(hist)
    bin_center = (angle_bins[max_bin] + angle_bins[max_bin + 1]) / 2

    # Average angles in winning bin
    mask = np.abs(angles - bin_center) < np.deg2rad(10)
    consensus = self._circular_angle_mean(np.array(angles)[mask])
    return consensus
```

---

## Solution 4: Hysteresis (Prefer Previous Grasp)

If quality difference is small, stick with previous grasp angle.

### Implementation (Optional)

```python
def _apply_hysteresis(self, new_grasp, prev_grasp, quality_threshold=0.02):
    """Prefer previous grasp if quality is similar."""
    if prev_grasp is None:
        return new_grasp

    quality_diff = new_grasp['quality'] - prev_grasp['quality']

    if abs(quality_diff) < quality_threshold:
        # Qualities are similar, use previous angle
        logger.debug(f"⚙️  Hysteresis: keeping previous angle (ΔQ={quality_diff:.3f})")
        new_grasp['angle'] = prev_grasp['angle']

    return new_grasp
```

---

## Solution 5: Model-Level Improvements (Advanced)

If temporal filtering isn't sufficient, consider:

1. **Fine-tune GR-ConvNet** on your specific objects
2. **Ensemble models** (average predictions from multiple checkpoints)
3. **Depth inpainting** (fill holes before inference)
4. **Multi-scale inference** (test at different resolutions)

---

## Recommended Workflow

### Phase 1: Enable Temporal Filtering

1. Set `temporal_filter_enabled = True`
2. Start with `circular_mean`, window=5
3. Test with static object, multiple frames
4. Check debug logs for angle stability

### Phase 2: Tune Parameters

- If still jittery: **Increase window_size to 7**
- If too slow to respond: **Decrease to 3, try EMA**
- If outliers persist: **Lower outlier_threshold to 20°**
- If symmetric object: **Disable outlier rejection**

### Phase 3: Advanced (If Needed)

- Increase spatial Gaussian blur (σ=3.0)
- Add hysteresis for quality-ambiguous cases
- Implement top-K consensus

---

## Expected Results

### Before Temporal Filtering:

```
Frame 1: Angle = 45.2°
Frame 2: Angle = 51.8°
Frame 3: Angle = 43.1°
Frame 4: Angle = 67.4° ← outlier
Frame 5: Angle = 49.3°

Average deviation: ±8.5°
```

### After Temporal Filtering (circular_mean, window=5):

```
Frame 1: Angle = 45.2° (not enough history)
Frame 2: Angle = 48.5° (avg of 2)
Frame 3: Angle = 46.7° (avg of 3)
Frame 4: Angle = 46.3° (outlier rejected)
Frame 5: Angle = 47.2° (stable avg)

Average deviation: ±1.5° ✅
```

---

## Testing

### Quick Test Script

```bash
# Run visualization tool multiple times and check angle consistency
python visualize_grconvnet_outputs.py

# With temporal filtering, you should see:
# - Angles converge to stable value after 3-5 frames
# - Less variation between consecutive frames
# - Outliers logged and rejected
```

### Verification

1. Place static object in view
2. Run grasp detection 10 times
3. Record predicted angles
4. **Without filtering**: σ > 5°
5. **With filtering**: σ < 2° ✅

---

## Trade-offs

| Approach          | Pros               | Cons                   |
| ----------------- | ------------------ | ---------------------- |
| Temporal (mean)   | Simple, effective  | Requires static scene  |
| Temporal (EMA)    | Smooth, responsive | Slight lag on changes  |
| Temporal (median) | Robust to outliers | Slower convergence     |
| Spatial blur      | Works per-frame    | Reduces precision      |
| Top-K consensus   | No history needed  | More computation       |
| Hysteresis        | Very stable        | May miss better grasps |

---

## Implementation Status

✅ **Temporal filtering**: Fully implemented, configurable  
✅ **Circular angle averaging**: Handles wrap-around correctly  
✅ **Outlier rejection**: Configurable threshold  
✅ **Debug logging**: Real-time angle tracking  
✅ **Config integration**: Easy enable/disable

⚙️ **Spatial blur**: Already applied (σ=2.0), tunable  
⏸️ **Top-K consensus**: Optional, code provided  
⏸️ **Hysteresis**: Optional, code provided

---

## Troubleshooting

### "Angles still jumping around"

- **Increase** `temporal_window_size` to 7-9
- **Switch** to `median` filter (more robust)
- **Lower** `temporal_outlier_threshold_deg` to 20°

### "Grasp angle too slow to update"

- **Decrease** `temporal_window_size` to 3
- **Switch** to `ema` with `alpha=0.5`
- **Disable** outlier rejection

### "Temporal filter rejecting good angles"

- **Increase** `temporal_outlier_threshold_deg` to 45°
- **Disable** outlier rejection (set to `None`)
- Check if object is actually rotating

### "First frame is always rejected as outlier"

- ✅ **Fixed!** Outlier detection now requires 3+ samples before activating
- First 3 frames are always accepted to build initial history
- This prevents first-frame bias where an outlier anchors the filter

### "No improvement from temporal filtering"

- Verify `temporal_filter_enabled = True` in config
- Check debug logs for "Temporal filtering enabled" message
- Ensure you're testing on **static** object (temporal needs stable scene)
- Try increasing spatial Gaussian blur instead

---

## References

- **Circular statistics**: Used for averaging angles with wrap-around
- **EMA**: Exponential moving average for time-series smoothing
- **Median filter**: Robust statistics for outlier rejection
- **GR-ConvNet**: Already applies spatial Gaussian (σ=2.0) to outputs

---

## Next Steps

1. **Test on your robot**: Enable temporal filtering and run grasping state
2. **Monitor logs**: Watch for angle convergence over 5-10 frames
3. **Tune parameters**: Adjust window size based on results
4. **Compare methods**: Try circular_mean vs. median vs. ema
5. **Document best config**: Save optimal parameters for your setup

**Expected outcome**: Consistent grasp angles (±2°) across frames, leading to more reliable robot execution.
