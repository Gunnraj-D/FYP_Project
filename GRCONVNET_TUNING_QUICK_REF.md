# GR-ConvNet Anti-Tip Tuning Quick Reference

## 🎯 Quick Diagnosis

| Symptom                          | Likely Cause                    | Fix                                       |
| -------------------------------- | ------------------------------- | ----------------------------------------- |
| Still selecting tips             | NMS not aggressive enough       | Increase `dilate_size` to 11-13           |
| All grasps rejected              | `min_overlap` too strict        | Lower to 0.2-0.25                         |
| Widths consistently off          | Wrong scaling factor            | Calibrate `*150.0` multiplier (line 446)  |
| Object mask too small            | Background threshold too strict | Lower `bg_percentile` to 70-75            |
| Object mask includes background  | Foreground threshold too loose  | Increase `depth_diff_thresh` to 0.03-0.05 |
| No valid grasps on small objects | Overlap requirement too strict  | Lower `min_overlap` to 0.15-0.2           |

## 🔧 Main Tuning Parameters

### 1. NMS Aggressiveness (line 347)

```python
# DEFAULT (balanced)
top_indices = topk_local_maxima(q_img, k_actual, dilate_size=9, min_thr=0.03)

# AGGRESSIVE (large objects, suppress tips more)
top_indices = topk_local_maxima(q_img, k_actual, dilate_size=13, min_thr=0.05)

# GENTLE (small/thin objects, cluttered scenes)
top_indices = topk_local_maxima(q_img, k_actual, dilate_size=7, min_thr=0.02)
```

### 2. Object Overlap Requirement (line 721)

```python
# DEFAULT
result = process_frame_with_analysis(model, camera, gripper, min_overlap=0.3)

# STRICT (only high-confidence on-object grasps)
result = process_frame_with_analysis(model, camera, gripper, min_overlap=0.5)

# PERMISSIVE (allow edge grasps, small objects)
result = process_frame_with_analysis(model, camera, gripper, min_overlap=0.15)
```

### 3. Foreground Mask Sensitivity (line 437)

```python
# DEFAULT
object_mask = depth_foreground_mask(depth_resized_m, bg_percentile=80, depth_diff_thresh=0.02)

# STRICT FOREGROUND (only closest objects)
object_mask = depth_foreground_mask(depth_resized_m, bg_percentile=85, depth_diff_thresh=0.01)

# PERMISSIVE FOREGROUND (include more depth range)
object_mask = depth_foreground_mask(depth_resized_m, bg_percentile=70, depth_diff_thresh=0.04)
```

### 4. Width Scaling Factor (line 446)

```python
# DEFAULT (model-dependent, may need adjustment)
width_img = (F.relu(width) * 150.0).squeeze().cpu().numpy()

# If widths are OVERESTIMATED
width_img = (F.relu(width) * 100.0).squeeze().cpu().numpy()

# If widths are UNDERESTIMATED
width_img = (F.relu(width) * 200.0).squeeze().cpu().numpy()
```

**Calibration method:**

1. Place object with known width (e.g., 40mm)
2. Note predicted width
3. Adjust multiplier: `new_mult = current_mult × (true_width / predicted_width)`

### 5. Border Distance Penalty (line 395)

```python
# DEFAULT (reject grasps within 20% of border)
if border_dist < 0.2:

# STRICT (reject within 30% of border)
if border_dist < 0.3:

# PERMISSIVE (reject only within 10% of border)
if border_dist < 0.1:
```

## 📊 What to Look For in Output

### Good Grasp Indicators

```
Frame 1: 48.3° Q:0.756 W:32.5mm D:0.487m Ovlp:0.82 ✓ VALID
         ✓ High overlap (>0.6)
         ✓ Reasonable width (15-60mm range)
         ✓ Depth consistent with object body
```

### Tip Grasp Indicators (Should Be Rare Now)

```
Frame 2: 5.2° Q:0.891 W:8.5mm D:0.320m Ovlp:0.15 ✗ Low overlap
         ✗ Very low overlap (<0.3)
         ✗ Narrow width (often <10mm)
         ✗ Depth much closer than other candidates
```

### Warning Signs

- **All grasps have low overlap (<0.4)**: Object mask may be incorrect
- **All widths >80mm**: Scaling factor is too high
- **All widths <10mm**: Scaling factor is too low
- **Depth jumps >0.1m between Alt1 and Alt2**: Object has complex geometry (OK)

## 🚀 Quick Start Recipe

### For Screwdrivers / Long Thin Tools

```python
# Aggressive anti-tip settings
result = process_frame_with_analysis(
    model, camera, gripper,
    use_nms=True,
    min_overlap=0.4  # Strict overlap requirement
)

# In analyze_top_k_grasps(), use:
top_indices = topk_local_maxima(q_img, k_actual, dilate_size=11, min_thr=0.04)
```

### For Small Objects (Batteries, USB Drives)

```python
# Permissive settings for small targets
result = process_frame_with_analysis(
    model, camera, gripper,
    use_nms=True,
    min_overlap=0.2  # Lower requirement for small objects
)

object_mask = depth_foreground_mask(
    depth_resized_m,
    bg_percentile=75,        # More permissive
    depth_diff_thresh=0.03
)
```

### For Cluttered Scenes

```python
# Gentle NMS, moderate overlap
result = process_frame_with_analysis(
    model, camera, gripper,
    use_nms=True,
    min_overlap=0.25
)

top_indices = topk_local_maxima(q_img, k_actual, dilate_size=7, min_thr=0.02)
```

### For Large Flat Objects (Books, Boxes)

```python
# Standard settings work well
result = process_frame_with_analysis(model, camera, gripper)
# No changes needed, defaults are optimal
```

## 🔍 Visual Debugging Checklist

1. **Check grasp rectangles**: Should overlap object body (not just tips)
2. **Check green mask overlay**: Should cover object, not background
3. **Check depth annotations**: Tip grasps should show much smaller depth
4. **Check overlap values**: Valid grasps >0.5, rejected tips <0.3

## 🛠 Emergency Fallbacks

If nothing works:

1. **Disable all new features temporarily:**

   ```python
   result = process_frame_with_analysis(
       model, camera, gripper,
       use_nms=False,
       min_overlap=0.0  # Accept all
   )
   ```

2. **Verify depth units:**

   ```python
   print(f"Median depth: {median_depth_m:.3f}m")  # Should be 0.3-1.0m for typical scenes
   ```

3. **Check preprocessed depth:**

   ```python
   cv2.imshow("Depth", depth_resized_m / depth_resized_m.max())
   cv2.waitKey(1)
   ```

4. **Verify camera FOV:**
   ```python
   # Line 55-56: Adjust if using different camera
   def estimate_pixel_to_mm_ratio(depth_m: float, camera_fov_deg: float = 69.4, ...
   # D435: 69.4°, D415: 65.0°, D455: 90.0°
   ```

## 📈 Performance Targets

After tuning, expect:

- **Valid grasp rate**: >70% of frames
- **Tip grasp rate**: <10% of selections
- **Average overlap (valid)**: >0.6
- **Average overlap (rejected)**: <0.3
- **Width estimate accuracy**: ±5mm for objects 20-80mm

## 🎓 Pro Tips

1. **Start with defaults**, tune only if needed
2. **Tune one parameter at a time**, observe effect
3. **Use screwdriver as test case** (hardest scenario)
4. **Check console output first**, then visualizations
5. **Save successful configurations** for different object types
6. **Consider object scale**: small objects need permissive settings

## 📝 Configuration Template

Save successful configurations for future use:

```python
# CONFIGURATION: Long thin tools (screwdrivers, pens)
CONFIG_THIN_TOOLS = {
    'use_nms': True,
    'min_overlap': 0.4,
    'nms_dilate': 11,
    'nms_threshold': 0.04,
    'bg_percentile': 82,
    'depth_diff_thresh': 0.02,
    'border_threshold': 0.25
}

# CONFIGURATION: Small objects (USB, batteries)
CONFIG_SMALL_OBJECTS = {
    'use_nms': True,
    'min_overlap': 0.2,
    'nms_dilate': 7,
    'nms_threshold': 0.02,
    'bg_percentile': 75,
    'depth_diff_thresh': 0.03,
    'border_threshold': 0.15
}

# CONFIGURATION: Large flat objects (books, boxes)
CONFIG_LARGE_FLAT = {
    'use_nms': True,
    'min_overlap': 0.3,
    'nms_dilate': 9,
    'nms_threshold': 0.03,
    'bg_percentile': 80,
    'depth_diff_thresh': 0.02,
    'border_threshold': 0.20
}
```

## 🔗 Related Files

- Full implementation guide: `GRCONVNET_ANTI_TIP_IMPLEMENTATION.md`
- Main script: `visualize_grconvnet_temporal.py`
