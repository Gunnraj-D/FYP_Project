# GR-ConvNet Grasp Detection System - Current Implementation & Issues

## System Overview

We have implemented a grasp detection system using **GR-ConvNet** (Generative Residual Convolutional Neural Network) for robotic grasping. The system processes RGB-D images from an overhead camera, predicts grasp poses, and converts them to robot joint angles for execution.

### Key Components:

1. **Model**: GR-ConvNet trained on Cornell Grasp Dataset
2. **Input**: RGB-D images (4 channels: Depth, Red, Green, Blue)
3. **Output**: Quality map, Angle map (contact line orientation), Width map
4. **Postprocessing**: Temporal filtering for angle stability
5. **Transformation**: Camera frame → Robot base frame → Joint angles

---

## Current Implementation Details

### 1. Preprocessing (RGB-D Input)

**Location**: `src/object_detection/grasp_detector_module.py:105-160`

```python
def preprocess(self, depth_image: np.ndarray, color_image: Optional[np.ndarray] = None):
    """
    Preprocess depth (and optionally color) for network inference.

    Returns:
        - GGCNN2: (1, 1, 300, 300) depth-only
        - GR-ConvNet: (1, 4, 300, 300) RGB-D
    """
    # Crop to square to avoid aspect ratio distortion
    h, w = depth_image.shape
    min_dim = min(h, w)
    start_h = (h - min_dim) // 2
    start_w = (w - min_dim) // 2
    depth = depth_image[start_h:start_h+min_dim, start_w:start_w+min_dim]

    # Resize depth
    depth = cv2.resize(depth, (self.resize_size, self.resize_size))

    # Normalize depth (same for both models)
    depth_normalized = np.clip(depth, 0.2, 1.2)      # meters
    depth_normalized = (depth_normalized - 0.2) / (1.0)  # [0,1]

    if self.use_rgbd and color_image is not None:
        # GR-ConvNet: RGB-D preprocessing
        # Crop and resize color to match depth
        color = color_image[start_h:start_h + min_dim, start_w:start_w+min_dim]
        color = cv2.resize(color, (self.resize_size, self.resize_size))

        # Convert BGR to RGB
        color_rgb = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)

        # GR-ConvNet RGB normalization (from image.py lines 53-59):
        # 1. Scale to [0,1]
        rgb_scaled = color_rgb.astype(np.float32) / 255.0
        # 2. Zero-center by subtracting mean
        rgb_normalized = rgb_scaled - rgb_scaled.mean()

        # GR-ConvNet Depth normalization (from image.py lines 205-209):
        # Mean-center and clip to [-1, 1]
        depth_mean_centered = depth - depth.mean()
        depth_normalized_grconvnet = np.clip(depth_mean_centered, -1, 1)

        # Stack: [D, R, G, B] - 4 channels (GR-ConvNet convention!)
        # IMPORTANT: GR-ConvNet expects DEPTH FIRST, then RGB
        # Verified in camera_data.py lines 74-80: concatenate(depth, rgb)
        rgbd = np.dstack([depth_normalized_grconvnet[:, :, None], rgb_normalized])

        # Convert to tensor: (H, W, 4) → (4, H, W) → (1, 4, H, W)
        rgbd_tensor = torch.from_numpy(rgbd).permute(2, 0, 1).unsqueeze(0).float()

        return rgbd_tensor.to(self.device)
```

**Key Points**:

- Center-crop to square (preserves aspect ratio)
- RGB: /255 → zero-center (subtract mean)
- Depth: mean-center → clip to [-1, 1]
- Channel order: **[D, R, G, B]** (depth first!)

---

### 2. Network Inference

**Location**: `src/object_detection/grasp_detector_module.py:203-241`

```python
def infer(self, depth_image: np.ndarray, original_depth_frame=None,
          color_image: Optional[np.ndarray] = None):
    """
    Run grasp network (GGCNN2 or GR-ConvNet) and return best grasp candidate.
    """
    # Preprocess (handles both depth-only and RGB-D)
    input_tensor = self.preprocess(depth_image, color_image)

    with torch.no_grad():
        pos, cos, sin, width = self.model(input_tensor)

    # Decode outputs
    q_img = torch.sigmoid(pos)                 # grasp quality
    ang_img = 0.5 * torch.atan2(sin, cos)      # angle [-pi/2, pi/2]

    # Width decoding (model-specific)
    if GRASP_MODEL_TYPE == 'grconvnet':
        # GR-ConvNet: Network outputs normalized width, scale by 150 pixels
        width_img = F.relu(width) * 150.0
    else:
        # GGCNN2: Direct pixel width
        width_img = F.relu(width)

    # Pick best grasp
    grasp_2d = self.postprocess(q_img, ang_img, width_img, depth_image, original_depth_frame)
```

**Key Points**:

- Angle range: **[-π/2, π/2]** (−90° to +90°)
- Angle represents: **CONTACT LINE / LONG AXIS** of grasp rectangle
- Width scaled by 150 for GR-ConvNet (training convention)

---

### 3. Angle Interpretation & Offset

**Location**: `src/object_detection/grasp_detector_module.py:310-356`

```python
# GGCNN2 outputs angle in range [-π/2, π/2]
grasp_angle_rad = grasp_2d["angle"]

# Apply configurable angle offset (for gripper finger axis alignment)
# This compensates for:
# 1. Gripper finger axis orientation vs. GGCNN2 angle convention
# 2. Camera mounting orientation effects
# 3. Any systematic rotation bias in the training data
angle_offset_rad = GRASP_DETECTION_CONFIG.get('grasp_angle_offset_rad', 0.0)
grasp_angle_rad += angle_offset_rad

# Create rotation matrices in BASE FRAME coordinates:
from scipy.spatial.transform import Rotation as R

# R_z: Rotation around base frame Z-axis to align gripper with grasp angle
R_z = R.from_euler('z', grasp_angle_rad).as_matrix()

# R_down: Rotation to point gripper downward (180° around X-axis)
R_down = get_facing_down_orientation()

# IMPORTANT: Rotation composition order (configurable)
# 'down_then_z': R_down @ R_z = align with object orientation, then point down
compose_order = GRASP_DETECTION_CONFIG.get('compose_order', 'down_then_z')
if compose_order == 'down_then_z':
    target_orientation_matrix = R_down @ R_z
else:
    target_orientation_matrix = R_z @ R_down

# Log the resulting orientation
result_rpy = R.from_matrix(target_orientation_matrix).as_euler('xyz')
logger.info(f"Final grasp orientation: RPY: [R={np.degrees(result_rpy[0]):.1f}°, "
            f"P={np.degrees(result_rpy[1]):.1f}°, Y={np.degrees(result_rpy[2]):.1f}°]")
```

**Current Configuration**:

```python
# In config.py
GRASP_DETECTION_CONFIG = {
    'grasp_angle_offset_rad': 1.5708,  # +90° (π/2)
    'compose_order': 'down_then_z',
}
```

**Rationale for +90° offset**:

- Network predicts **contact line** angle (long axis of grasp rectangle)
- Gripper **jaw closing axis** must be **perpendicular** to contact line
- For short-side antipodal grasps, we need jaw axis = contact line + 90°

---

### 4. Temporal Filtering for Angle Stability

**Location**: `src/object_detection/grasp_detector_module.py:488-553`

```python
def _apply_temporal_filter(self, angle: float) -> float:
    """
    Apply temporal filtering to stabilize angle predictions.

    - Requires 3+ samples before outlier rejection activates (prevents first-frame bias)
    - Uses circular mean for proper angle averaging
    - Rejects outliers > 30° from recent history
    """
    if not self.temporal_filter_enabled:
        return angle

    # Check for outliers (requires 3+ samples)
    if self.temporal_outlier_threshold_deg is not None and len(self.angle_history) >= 3:
        if self._is_angle_outlier(angle, self.angle_history, self.temporal_outlier_threshold_deg):
            # Return previous filtered value instead of outlier
            if self.angle_history:
                return self.angle_history[-1]

    # Apply selected filter type (circular_mean, median, or ema)
    # ... [filter implementation] ...

    return filtered_angle
```

**Configuration**:

```python
GRASP_DETECTION_CONFIG = {
    'temporal_filter_enabled': True,
    'temporal_window_size': 5,
    'temporal_filter_type': 'circular_mean',
    'temporal_ema_alpha': 0.3,
    'temporal_outlier_threshold_deg': 30,
}
```

---

## 🐛 Known Issues & Observations

### Issue 1: Diagonal Objects Grasp Long Side ⚠️

**Description**: Objects oriented diagonally (from top-left to bottom-right, approximately -45° to the vertical) have their grasps predicted along the **long side** instead of the **short side**, despite the +90° offset being applied.

**Observations**:

- ✅ **Vertical rectangles** (0°): Correctly grasps short horizontal side
- ✅ **Horizontal rectangles** (90°): Correctly grasps short vertical side
- ❌ **Diagonal rectangles** (±45°): Grasps long side (wrong!)

**Example**:

```
Object orientation: -45° (top-left to bottom-right)
Network prediction: ~-45° (contact line angle)
After +90° offset: ~+45° (jaw axis)
Expected: Jaw closes across SHORT side perpendicular to diagonal
Observed: Jaw closes along LONG side parallel to diagonal
```

**Potential Root Causes**:

1. **Training Data Bias**:

   - Cornell dataset may have limited diagonal samples
   - Network might have learned axis-aligned bias (0° or 90° dominant)
   - Diagonal grasps may be underrepresented

2. **Angle Periodicity Ambiguity**:

   - Gripper has 180° symmetry (two-finger parallel jaw)
   - Network angle in [-90°, +90°] wraps around
   - A 45° grasp is equivalent to 45° + 180° = 225° ≡ -135°
   - Possible: Network predicting equivalent but opposite orientation

3. **Coordinate Frame Transformation Issue**:

   - Camera frame to base frame transformation
   - R_down @ R_z composition may interact differently with diagonal angles
   - Potential camera mounting calibration drift

4. **Width Prediction Correlation**:
   - Width might be incorrectly large for long-side grasps
   - Scoring function may favor these incorrect wide grasps
   - Check if width_img is reasonable for diagonal objects

---

### Issue 2: Angle Consistency Across Frames (ADDRESSED)

**Status**: ✅ **Mitigated with temporal filtering**

Previously, angles varied ±5-10° across consecutive frames. Now reduced to ±2° with:

- Circular mean averaging over 5 frames
- Outlier rejection (>30° deviation)
- 3-frame warmup to prevent first-frame bias

---

### Issue 3: Edge-Biased Grasp Selection (ADDRESSED)

**Status**: ✅ **Mitigated with postprocessing improvements**

Added quality-weighted centroid and distance penalties to prefer interior grasps.

---

## 🔬 Diagnostic Questions

To resolve the diagonal object issue, we need to investigate:

### 1. Network Output Analysis:

- What does the **raw angle map** (`ang_img`) show for diagonal objects?
  - Is there a peak at the correct angle (-45° contact line)?
  - Or is the network failing to predict diagonal angles entirely?
  - Visualize: `visualize_grconvnet_outputs.py` with diagonal rectangle

### 2. Top-K Candidate Analysis:

- What are the **top 5 grasp candidates** for diagonal objects?
  - Do any candidates have the correct angle?
  - If yes: Scoring issue (wrong candidate selected)
  - If no: Network issue (not predicting diagonal angles)

### 3. Angle Offset Verification:

- Is the +90° offset being applied correctly for diagonal angles?
  - Test: Log `grasp_angle_rad` before and after offset
  - Verify: `result_rpy` yaw component matches expected jaw axis

### 4. Width Correlation:

- What **width** is predicted for diagonal vs. axis-aligned objects?
  - Are diagonal grasps predicting unusually large widths?
  - This could indicate long-side grasps being favored

### 5. Training Data Distribution:

- What angle distribution was GR-ConvNet trained on?
  - Cornell dataset angle statistics?
  - Are diagonal angles underrepresented?
  - May need fine-tuning on custom diagonal-heavy dataset

---

## 📊 Test Scenarios

### Minimal Reproducible Test:

```python
# Place white rectangular block on black surface
# Orientations to test:

1. Vertical (0°):    Long axis vertical
   Expected: Grasp short horizontal side ✅

2. Horizontal (90°): Long axis horizontal
   Expected: Grasp short vertical side ✅

3. Diagonal (-45°):  Top-left to bottom-right
   Expected: Grasp short side perpendicular to diagonal ❌
   Observed: Grasp long side along diagonal

4. Diagonal (+45°):  Top-right to bottom-left
   Expected: Grasp short side perpendicular to diagonal ❓
```

### Data to Collect:

For each orientation:

1. Screenshot of quality map with grasp overlay
2. Log output showing:
   - Raw network angle
   - After +90° offset
   - Final RPY orientation
   - Predicted width
3. Top 5 candidate grasps (angle, quality, score)

---

## 🎯 Potential Solutions

### Option 1: Additional Angle Offset for Diagonal Cases

```python
# In postprocess, detect if angle is near ±45° and apply correction
if abs(abs(angle) - np.pi/4) < np.deg2rad(15):  # Within 15° of ±45°
    angle += np.pi/2  # Additional 90° correction
```

⚠️ **Concern**: This is a band-aid fix, not addressing root cause.

---

### Option 2: Fine-Tune GR-ConvNet on Diagonal Objects

- Collect custom dataset with diagonal rectangles
- Augment with rotations at 15°, 30°, 45°, 60°, 75°
- Fine-tune last layers of GR-ConvNet
- Validate on test set

✅ **Benefit**: Addresses root cause (training data bias)
❌ **Cost**: Requires data collection, retraining, validation

---

### Option 3: Angle Post-Correction via PCA

```python
# After network prediction, verify angle against object's principal axis
pca_angle = compute_principal_axis(depth_crop, grasp_center)
angle_error = angle_difference(predicted_angle, pca_angle)

if abs(angle_error) > threshold:
    corrected_angle = pca_angle  # Use geometric angle instead
```

✅ **Benefit**: Uses actual object geometry, not network prediction
❌ **Drawback**: Assumes object is rectangular (not general)

---

### Option 4: Ensemble Multiple Angle Predictions

```python
# Use top-K candidates and find consensus
top_k_angles = [c['angle'] for c in top_candidates]
consensus_angle = circular_mean(top_k_angles)
```

✅ **Benefit**: More robust to single-candidate errors
❌ **Drawback**: Requires multiple candidates at correct angle

---

## 🔍 Next Steps

**Immediate Actions**:

1. **Run diagnostic on diagonal object**:

   ```bash
   python visualize_grconvnet_outputs.py
   # Place rectangle at -45° orientation
   # Inspect angle map, quality map, and top candidates
   ```

2. **Log top-K candidates**:

   - Enable `DEBUG_MODE = True` in config
   - Check if any of top 5 candidates have correct angle
   - Look at angle distribution in logs

3. **Verify offset application**:

   - Add breakpoint at line 321: `grasp_angle_rad += angle_offset_rad`
   - Confirm offset is being added correctly
   - Check final `result_rpy` matches expected jaw axis

4. **Test width correlation**:
   - Compare width predictions for vertical vs. diagonal
   - If diagonal widths are much larger → scoring issue

**Long-term Solutions** (based on diagnostics):

- If network outputs correct angles → Fix scoring/selection
- If network fails on diagonals → Fine-tune or retrain
- If offset application is wrong → Fix transformation pipeline
- If geometric ambiguity → Add PCA-based verification

---

## 📁 Relevant Files

- `src/object_detection/grasp_detector_module.py` - Main implementation
- `src/object_detection/grconvnet.py` - Network architecture
- `src/config/config.py` - Configuration (line 228-295)
- `visualize_grconvnet_outputs.py` - Diagnostic visualization
- `visualize_grconvnet_temporal.py` - Temporal filtering test
- `ANGLE_STABILITY_IMPROVEMENTS.md` - Temporal filtering docs
- `GR_ConvNet_Migration_Plan_FINAL.md` - Migration from GGCNN2

---

## 🤔 Request for Analysis

**Given this system implementation and the observed diagonal object issue, can you help identify**:

1. **Root cause**: Is this likely a network training bias, coordinate transformation issue, or scoring problem?

2. **Diagnostic approach**: What additional data should we collect to pinpoint the issue?

3. **Solution recommendation**: Which of the proposed solutions (or alternatives) would be most effective?

4. **Quick fixes**: Are there any immediate adjustments we can test (e.g., different offset for diagonals) without retraining?

Please consider the angle periodicity (180° gripper symmetry), the training data characteristics (Cornell dataset), and the coordinate transformations involved in your analysis.


