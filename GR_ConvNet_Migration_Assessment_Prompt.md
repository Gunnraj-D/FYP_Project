# GR-ConvNet Migration Assessment Prompt

## Objective

Assess the feasibility and effort required to migrate from GGCNN2 to GR-ConvNet (Generative Residual Convolutional Neural Network) for robotic grasping to improve performance on arbitrarily rotated objects (especially 45° rotations).

---

## Current System Architecture

### Overview

**Robot System**: KUKA LBR iiwa 14 with Robotiq 85 gripper  
**Vision**: Intel RealSense D435 (overhead mounted)  
**Task**: Top-down grasping of objects on flat table  
**Control**: OPC UA communication with real-time telemetry  
**Kinematics**: Collision-aware IK solver with PyBullet

### Grasp Generation Pipeline (Current - GGCNN2)

```
┌─────────────────────────────────────────────────────────────────┐
│  1. CAMERA INPUT                                                │
│     ├─ Color stream: 640×480 @ 30fps (color intrinsics)        │
│     ├─ Depth stream: 848×480 @ 30fps (depth intrinsics)        │
│     └─ Aligned depth: Depth aligned to color frame             │
└─────────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────────┐
│  2. PREPROCESSING (GGcnn2Module.preprocess)                     │
│     ├─ Input: Depth array (meters), shape (H, W)               │
│     ├─ Crop to square: min(H, W)                               │
│     ├─ Resize: 300×300                                          │
│     ├─ Clip: [0.2m, 1.2m] (tabletop range)                     │
│     ├─ Normalize: (depth - 0.2) / 1.0 → [0, 1]                 │
│     └─ Output: Tensor (1, 1, 300, 300)                          │
└─────────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────────┐
│  3. NETWORK INFERENCE (GGCNN2.forward)                          │
│     ├─ Input: depth_tensor (1, 1, 300, 300)                    │
│     ├─ Architecture: U-Net style encoder-decoder               │
│     ├─ Output heads (all 300×300):                             │
│     │   ├─ pos_output: Grasp quality map Q(u,v)                │
│     │   ├─ cos_output: cos(2θ) for angle                       │
│     │   ├─ sin_output: sin(2θ) for angle                       │
│     │   └─ width_output: Grasp width in pixels                 │
│     └─ Post-activation:                                         │
│         ├─ q_img = sigmoid(pos)          [0, 1]                │
│         ├─ ang_img = 0.5*atan2(sin, cos) [-π/2, π/2]           │
│         └─ width_img = ReLU(width)       [0, ∞)                │
└─────────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────────┐
│  4. POSTPROCESSING (GGcnn2Module.postprocess)                   │
│     ├─ Gaussian blur quality map (5×5, σ=2)                    │
│     ├─ Optional angle masking (currently disabled)             │
│     ├─ Local maxima detection (NMS with 5×5 dilation)          │
│     ├─ Top-K candidates selection (K=8)                        │
│     ├─ Scoring function (per candidate):                       │
│     │   score = 2.0*quality - 0.4*dist - 0.9*edge - 0.7*width │
│     │   ├─ quality: Blurred Q value                            │
│     │   ├─ dist: Distance to quality-weighted centroid         │
│     │   ├─ edge: Local depth variance penalty                  │
│     │   └─ width: Gripper compatibility (0.02m - 0.12m)        │
│     ├─ Nearest-valid depth fallback (3×3 search, radius=5)    │
│     └─ Output: Best grasp {center, angle, width, quality}      │
└─────────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────────┐
│  5. 2D → 3D CONVERSION (_grasp_2d_to_3d_pose)                   │
│     ├─ Scale center from 300×300 back to original resolution   │
│     ├─ Sample depth at center (radius=5, with fallback)        │
│     ├─ Pixel to 3D using depth intrinsics:                     │
│     │   (x, y, z) = rs2_deproject_pixel_to_point(...)          │
│     └─ Output: [x, y, z, roll, pitch, yaw] in camera frame     │
└─────────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────────┐
│  6. CAMERA → BASE TRANSFORM (transform_camera_to_base)          │
│     ├─ Use calibrated hand-eye matrix (4×4)                    │
│     ├─ Camera → TCP: camera_to_tcp @ [x, y, z, 1]              │
│     ├─ TCP → Base: tcp_matrix @ [x_tcp, y_tcp, z_tcp, 1]       │
│     └─ Output: position in base frame (meters)                 │
└─────────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────────┐
│  7. ORIENTATION COMPOSITION (base frame)                        │
│     ├─ Apply 90° offset: grasp_angle_rad += 1.5708             │
│     │   (converts contact line to jaw-closing axis)            │
│     ├─ R_z = Rotation.from_euler('z', grasp_angle_rad)         │
│     ├─ R_down = get_facing_down_orientation()  # 180° around X │
│     ├─ Compose: target_orientation = R_down @ R_z              │
│     └─ Output: 3×3 rotation matrix in base frame               │
└─────────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────────┐
│  8. INVERSE KINEMATICS (solve_XYZ)                              │
│     ├─ Z-clamping: Ensure Z ≥ 0 (above table)                  │
│     ├─ Input: position (3D), orientation (3×3), current joints │
│     ├─ Solver: PyBullet IK with collision checking             │
│     └─ Output: 7 joint angles (radians)                        │
└─────────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────────┐
│  9. GRASP EXECUTION (via states & OPC UA)                       │
│     ├─ Store in telemetry: grasp pose, approach pose, height   │
│     ├─ Sequencer executes: Move → Approach → Grasp → Lift     │
│     └─ OPC client sends joint commands to robot                │
└─────────────────────────────────────────────────────────────────┘
```

---

## Current GGCNN2 Implementation Details

### Class Structure

```python
class GGcnn2Module:
    def __init__(self, model_path, telemetry, command_bus, camera_manager, kinematics_solver):
        # Load GGCNN2 model
        # Store dependencies (telemetry, camera, IK solver)
        # Config: resize_size=300, depth_sample_radius=5

    def preprocess(self, depth_image: np.ndarray) -> torch.Tensor:
        # Crop, resize to 300×300, normalize to [0,1]
        # Input: meters, Output: normalized tensor

    def infer(self, depth_image, original_depth_frame) -> Optional[Dict]:
        # Full pipeline: preprocess → network → postprocess → 3D
        # Returns: {grasp_2d, grasp_pose_camera, grasp_pose_base, joint_angles, quality, height}

    def postprocess(self, q_img, ang_img, width_img, depth_image, original_depth_frame, top_k=8):
        # NMS, scoring, best candidate selection
        # Returns: {center, angle, width, quality, depth_m, width_m}

    def process_depth_frame(self, depth_frame):
        # Convenience wrapper for RealSense frames
```

### Network Architecture (GGCNN2)

```python
class GGCNN2(nn.Module):
    # Encoder: 4 conv layers with 2 max pools (downsamples by 4x)
    # Bottleneck: 2 dilated convs (dilation=2,4)
    # Decoder: 2 upsampling + conv layers (back to original size)
    # Heads: 4 separate conv1×1 outputs (pos, cos, sin, width)
```

**Model File**: `epoch_50_cornell_statedict.pt` (PyTorch state dict)  
**Input**: Single-channel depth (1, 1, 300, 300)  
**Device**: CPU (can use CUDA if available)

---

## Key Integration Points

### 1. **Camera Manager Interface**

```python
camera_manager.get_average_depth(depth_frame, (u, v), radius=5) → float (meters)
camera_manager.pixel_to_3d(u, v, depth) → (x, y, z) in meters
camera_manager.depth_intrinsics → RealSense intrinsics object
camera_manager.color_intrinsics → RealSense intrinsics object
```

### 2. **Telemetry Store Interface**

```python
telemetry.get_current_joints() → np.ndarray[7] (radians)
telemetry.set_generated_grasp_pose(pose) → void
telemetry.set_generated_approach_pose(pose) → void
telemetry.update_grasp_height(height) → void
```

### 3. **Kinematics Solver Interface**

```python
kinematics_solver.tcp_from_joints(joints) → (tcp_matrix_4x4, tcp_pose)
kinematics_solver.solve_XYZ(position, current_joints, target_orientation) → joint_angles
get_facing_down_orientation() → 3×3 rotation matrix
```

### 4. **Coordinate Frame Conventions**

- **Camera frame**: X-right, Y-down (in image), Z-forward (depth)
- **TCP frame**: X-?, Y-?, Z-tool axis
- **Base frame**: X-?, Y-?, Z-up (vertical)
- **Hand-eye matrix**: 4×4 transform from camera to TCP
- **All distances**: METERS
- **All angles**: RADIANS
- **Joint angles**: RADIANS (converted to degrees for OPC UA)

---

## Current GGCNN2 Performance

### Strengths ✅

- Fast inference (~150ms on CPU)
- Well-centered grasps (edge bias fixed)
- Works perfectly for axis-aligned objects (0°, 90°)
- Robust depth handling with fallbacks
- Clean integration with robot pipeline

### Limitations ❌

- **Cornell training bias**: Cannot detect diagonal angles (45°, 60°, etc.)
- Angle predictions limited to ±30° range (observed max: 29.5°)
- Falls back to horizontal grasps for diagonal objects
- No rotation invariance built into architecture

### Test Results

| Object Orientation | Network Angle | Grasp Quality | Result                                       |
| ------------------ | ------------- | ------------- | -------------------------------------------- |
| Vertical (0°)      | 2.3°          | 0.684         | ✅ Perfect - short horizontal side           |
| Horizontal (90°)   | 68.3°         | 0.672         | ✅ Perfect - short vertical side             |
| Diagonal (45°)     | 8-28°         | 0.682         | ⚠️ Fallback - horizontal instead of diagonal |

---

## GR-ConvNet Overview (Target Model)

### Key Differences from GGCNN2

**Architecture:**

- Generative model with residual connections
- Better rotation equivariance through design
- Multiple output resolutions
- May have different input size requirements

**Outputs:**

- Quality map Q(u,v)
- Angle map θ(u,v)
- Width map W(u,v)
- (Potentially different encoding: may output θ directly instead of sin/cos)

**Training:**

- Typically trained on Jacquard or Cornell+
- May have better rotation augmentation
- Unknown angle range and convention

---

## Migration Assessment Questions

### 1. Model Architecture Compatibility

**Q1.1**: What is GR-ConvNet's input format?

- Input size: 300×300 like GGCNN2? Or different (e.g., 224×224, 320×320)?
- Channels: Single depth? RGB-D?
- Normalization: Same [0,1] range or different?

**Q1.2**: What are the output formats?

- Number of output heads: 3 (Q, θ, W) or 4 (Q, cos, sin, W)?
- Angle encoding: Direct θ or sin/cos decomposition?
- Angle range: [-π/2, π/2] like GGCNN2 or [-π, π]?
- Width units: Pixels or normalized?

**Q1.3**: Are there pre-trained weights available?

- Trained on what dataset: Cornell, Jacquard, other?
- PyTorch format compatible with our loader?
- What pre/post-processing was used during training?

### 2. Code Modification Scope

**Q2.1**: What changes are needed to `preprocess()`?

- Different resize dimensions?
- Different normalization range?
- Additional preprocessing steps?
- Can we reuse current crop-to-square logic?

**Q2.2**: What changes are needed to network loading/inference?

```python
# Current GGCNN2:
self.model = GGCNN2()
state_dict = torch.load(model_path)
self.model.load_state_dict(state_dict)
pos, cos, sin, width = self.model(depth_tensor)
```

- Can GR-ConvNet follow same pattern?
- Are there additional hyperparameters?
- Does it need special initialization?

**Q2.3**: What changes to output decoding?

```python
# Current GGCNN2:
q_img = torch.sigmoid(pos)
ang_img = 0.5 * torch.atan2(sin, cos)  # [-π/2, π/2]
width_img = F.relu(width)
```

- Does GR-ConvNet use same sigmoid/atan2/relu?
- Different angle decoding formula?
- Different output ranges to handle?

**Q2.4**: Can postprocessing remain unchanged?

- Current: NMS, scoring, depth sampling, width conversion
- Dependencies: Assumes 300×300 output, angle in [-π/2, π/2]
- Would any of this break with GR-ConvNet?

### 3. Integration & Configuration

**Q3.1**: Angle convention compatibility

- Does GR-ConvNet's angle represent contact line or jaw axis?
- Would we still need the 90° offset (`grasp_angle_offset_rad = 1.5708`)?
- Or does GR-ConvNet output differently?

**Q3.2**: Coordinate frame consistency

- Are camera/pixel coordinate conventions the same?
- Would `pixel_to_3d()` and transform chains still work?
- Any differences in angle/orientation interpretation?

**Q3.3**: Performance characteristics

- Inference speed: GGCNN2 is ~150ms on CPU, what about GR-ConvNet?
- GPU requirement: Can it run on CPU or CUDA required?
- Memory footprint: Larger model?

### 4. Expected Improvements

**Q4.1**: Rotation invariance

- Will GR-ConvNet detect 45° angles correctly?
- What's the expected angle prediction range?
- Is there data showing rotation performance?

**Q4.2**: Quality improvements

- Better grasp success rate on diagonal objects?
- Maintained performance on axis-aligned objects?
- Trade-offs in speed/accuracy?

---

## Code Compatibility Analysis Needed

### Current Code That Would Need Review:

**1. Model Definition (`src/object_detection/ggcnn2.py`):**

```python
class GGCNN2(nn.Module):
    def __init__(self, input_channels=1, filter_sizes=[16,16,32,16], ...):
        # Specific architecture

    def forward(self, x):
        # Returns: pos, cos, sin, width
```

**Question**: Can we create a `GRConvNet` class with same interface?

**2. Inference Call (`src/object_detection/ggcnn2_module.py` line 109):**

```python
with torch.no_grad():
    pos, cos, sin, width = self.model(depth_tensor)
```

**Question**: Does GR-ConvNet return same 4 outputs in same order?

**3. Angle Decoding (line 114):**

```python
ang_img = 0.5 * torch.atan2(sin, cos)  # [-π/2, π/2]
```

**Question**: Same for GR-ConvNet or different formula?

**4. Postprocessing (lines 323-631):**

- Entire `postprocess()` method
- Assumes 300×300 output maps
- Scaling logic: `scale_u = w_orig / 300.0`

**Question**: Would output size differ? If so, what needs updating?

---

## Migration Difficulty Estimate Request

Please provide:

### **Effort Level**:

- ⭐ **Easy** (1-2 hours): Drop-in replacement, minimal changes
- ⭐⭐ **Moderate** (4-8 hours): Some refactoring, testing needed
- ⭐⭐⭐ **Hard** (1-2 days): Significant rework, careful testing
- ⭐⭐⭐⭐ **Very Hard** (3-5 days): Major architectural changes

### **Risk Level**:

- 🟢 **Low**: Well-documented, similar architecture
- 🟡 **Medium**: Some unknowns, needs validation
- 🔴 **High**: Major differences, uncertain compatibility

### **Specific Changes Checklist**:

```
□ New model class definition (complexity: ?)
□ Preprocessing modifications (list specific changes)
□ Output decoding changes (list specific changes)
□ Postprocessing adaptations (list specific changes)
□ Config parameter updates (list what changes)
□ New dependencies/packages (list if any)
□ Testing/validation approach (describe)
```

### **Expected Performance Gain**:

- Angle detection at 45°: ? (qualitative or quantitative estimate)
- Inference speed change: ? (faster/slower/same)
- Overall grasp success rate: ? (current: ~67% for mixed angles)

### **Recommended Approach**:

- Should we migrate? Yes/No/Maybe with clear rationale
- If yes: Step-by-step migration plan
- If no: Alternative solutions or stick with GGCNN2

---

## Additional Context

### Current Configuration

```python
GRASP_DETECTION_CONFIG = {
    'min_quality_threshold': 0.10,
    'max_grasp_width': 0.100,  # meters
    'min_grasp_width': 0.020,  # meters
    'approach_height_offset': 0.050,  # meters
    'grasp_angle_offset_rad': 1.5708,  # 90° for jaw axis conversion
    'compose_order': 'down_then_z',
    'depth_sample_radius': 5,
    'topdown_angle_tolerance_rad': None,  # Disabled - free rotation
}

GRASP_EXECUTION_CONFIG = {
    'gripper_min_width_m': 0.020,
    'gripper_max_width_m': 0.120,
    'lift_height': 0.100,
}
```

### Dependencies

```
torch >= 1.9
torchvision
opencv-python (cv2)
numpy
scipy
pyrealsense2
```

### Constraints

- Must run on Windows 10
- Prefer CPU compatibility (CUDA optional)
- Real-time requirement: <500ms inference
- Integration must be minimal disruption to existing states/sequencer

---

## Success Criteria for Migration

A successful GR-ConvNet migration would:

✅ **Maintain or improve** grasp success on axis-aligned objects (0°, 90°)  
✅ **Significantly improve** diagonal object detection (45°, 60°, etc.)  
✅ **Preserve** integration with camera, IK, telemetry, OPC systems  
✅ **Keep** inference time reasonable (<500ms)  
✅ **Require** minimal changes to downstream pipeline (transform, IK, execution)

---

## Request

Based on the architecture described above:

1. **Assess migration difficulty** (Easy/Moderate/Hard/Very Hard)
2. **Identify specific code changes** needed in each component
3. **Estimate time/effort** required
4. **List risks/unknowns** that could complicate migration
5. **Provide recommendation**: Migrate now, defer, or skip?
6. **If recommending migration**: Provide step-by-step implementation plan

Focus particularly on:

- Compatibility with our preprocessing/postprocessing pipeline
- Angle convention differences (and whether 90° offset still needed)
- Output format alignment with our 2D→3D→IK chain
- Practical rotation invariance improvements we'd actually see

---

## Files Available for Review

- `src/object_detection/ggcnn2_module.py` (full implementation - 1194 lines)
- `src/object_detection/ggcnn2.py` (model definition - 89 lines)
- `src/config/config.py` (configuration)

[These can be provided upon request]
