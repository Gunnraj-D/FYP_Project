# GR-ConvNet Migration Implementation Plan

## Executive Summary

**Goal**: Migrate from GGCNN2 to GR-ConvNet to improve diagonal object grasping (45°, 60° rotations)

**Assessment Results**:

- ⭐⭐ **Effort**: Moderate (4-8 hours)
- 🟡 **Risk**: Medium
- ✅ **Feasibility**: High - Compatible architecture, same output format
- 📈 **Expected Gain**: Better rotation invariance, maintained axis-aligned performance

**Key Finding**: GR-ConvNet uses **identical output format** (Q, cos2θ, sin2θ, W) with same decoding, minimizing integration changes.

---

## Phase 1: Preparation & Research (1-2 hours)

### 1.1 Acquire GR-ConvNet Resources

**Tasks**:

- [ ] Find official GR-ConvNet v2 repository/paper
- [ ] Download pre-trained model weights (Cornell or Jacquard)
- [ ] Review model architecture code
- [ ] Verify PyTorch compatibility
- [ ] Check license compatibility for your use case

**Deliverables**:

- GR-ConvNet model weights file (`.pt` or `.pth`)
- Model architecture code (Python file)
- Documentation on input/output format
- Training preprocessing details

**Rollback**: None (just research)

---

### 1.2 Create Side-by-Side Comparison Environment

**Tasks**:

- [ ] Create backup branch: `git checkout -b grconvnet-migration`
- [ ] Copy current GGCNN2 module to `ggcnn2_module_backup.py`
- [ ] Document current performance baseline
- [ ] Prepare test dataset (3+ objects at 0°, 45°, 90°)

**Deliverables**:

- Git branch for migration work
- Backup of working GGCNN2 code
- Test images/objects documented

**Rollback**: `git checkout main` to return to GGCNN2

---

## Phase 2: Model Integration (2-3 hours)

### 2.1 Create GR-ConvNet Model Class

**File**: `src/object_detection/grconvnet.py` (NEW)

**Tasks**:

- [ ] Implement `GRConvNet` class following GGCNN2 interface pattern
- [ ] Ensure `forward()` returns: `pos, cos, sin, width` (same as GGCNN2)
- [ ] Configure input size to 300×300 (or make configurable)
- [ ] Add model initialization with filter sizes, residual blocks
- [ ] Test model loads and runs inference on dummy data

**Code Template**:

```python
class GRConvNet(nn.Module):
    def __init__(self, input_channels=1, filter_sizes=None, dropout=False,
                 dropout_prob=0.0, input_size=300):
        super().__init__()

        if filter_sizes is None:
            filter_sizes = [32, 16, 8, 8, 16, 32]  # GR-ConvNet defaults

        # TODO: Implement architecture based on GR-ConvNet paper
        # Encoder with residual blocks
        # Decoder with upsampling
        # 4 output heads: pos, cos, sin, width

    def forward(self, x):
        # TODO: Forward pass
        # Return: pos_output, cos_output, sin_output, width_output
        # SAME format as GGCNN2
        return pos_output, cos_output, sin_output, width_output
```

**Verification**:

```python
# Test script
model = GRConvNet()
dummy_input = torch.randn(1, 1, 300, 300)
pos, cos, sin, width = model(dummy_input)
assert pos.shape == (1, 1, 300, 300)
assert cos.shape == (1, 1, 300, 300)
# etc.
```

**Rollback**: Delete `grconvnet.py`, keep using `ggcnn2.py`

---

### 2.2 Update Configuration

**File**: `src/config/config.py`

**Tasks**:

- [ ] Add `GRCONVNET_MODEL_PATH` configuration
- [ ] Add `USE_GRCONVNET` flag (for easy switching)
- [ ] Keep all existing GRASP_DETECTION_CONFIG parameters
- [ ] Add GR-ConvNet specific params if needed (dropout, etc.)

**Code Changes**:

```python
# Add after GGCNN2_MODEL_PATH:
GRCONVNET_MODEL_PATH = SRC_DIR / "resources" / "ml_models" / \
    "grconvnet_weights" / "grconvnet_trained.pt"

# Add model selection flag
GRASP_MODEL_TYPE = 'ggcnn2'  # Options: 'ggcnn2', 'grconvnet'

# Optional GR-ConvNet specific config
GRCONVNET_CONFIG = {
    'use_dropout': False,
    'dropout_prob': 0.0,
    'input_size': 300,  # Keep 300 to minimize changes
}
```

**Rollback**: Set `GRASP_MODEL_TYPE = 'ggcnn2'`

---

### 2.3 Modify GGcnn2Module to Support Both Models

**File**: `src/object_detection/ggcnn2_module.py` → Rename to `grasp_detector_module.py`

**Tasks**:

- [ ] Rename class to `GraspDetectorModule` (or keep as `GGcnn2Module` for compatibility)
- [ ] Add model type selection in `__init__`
- [ ] Load GGCNN2 OR GR-ConvNet based on config
- [ ] Keep all other methods unchanged (preprocessing, postprocessing, etc.)

**Code Changes**:

```python
# In __init__:
from config.config import GRASP_MODEL_TYPE, GGCNN2_MODEL_PATH, GRCONVNET_MODEL_PATH
from object_detection.ggcnn2 import GGCNN2
from object_detection.grconvnet import GRConvNet

# Model selection
if GRASP_MODEL_TYPE == 'grconvnet':
    self.model = GRConvNet(input_channels=1, input_size=self.resize_size)
    model_path = GRCONVNET_MODEL_PATH
    logger.info("Using GR-ConvNet model")
else:
    self.model = GGCNN2()
    model_path = GGCNN2_MODEL_PATH
    logger.info("Using GGCNN2 model")

state_dict = torch.load(model_path, map_location=self.device)
self.model.load_state_dict(state_dict)
self.model.to(self.device).eval()
```

**Verification**:

- Set `GRASP_MODEL_TYPE = 'ggcnn2'` → Should work exactly as before
- Set `GRASP_MODEL_TYPE = 'grconvnet'` → Should load GR-ConvNet

**Rollback**: Set `GRASP_MODEL_TYPE = 'ggcnn2'`

---

## Phase 3: Testing & Validation (2-3 hours)

### 3.1 Synthetic Data Testing

**Tasks**:

- [ ] Create simple test with known depth images
- [ ] Verify GR-ConvNet outputs correct shape tensors
- [ ] Verify angle decoding produces expected range [-π/2, π/2]
- [ ] Compare GR-ConvNet vs GGCNN2 on same image

**Test Script**:

```python
# test_grconvnet_integration.py
import torch
import numpy as np
from object_detection.grconvnet import GRConvNet

# Load model
model = GRConvNet()
model.load_state_dict(torch.load('path/to/weights.pt'))
model.eval()

# Test image (synthetic or real)
depth = np.random.rand(300, 300) * 0.5 + 0.3  # 0.3-0.8m
depth_tensor = torch.from_numpy(depth).unsqueeze(0).unsqueeze(0).float()

# Inference
with torch.no_grad():
    pos, cos, sin, width = model(depth_tensor)

# Verify shapes
assert pos.shape == (1, 1, 300, 300), f"Got {pos.shape}"
assert cos.shape == (1, 1, 300, 300)

# Decode angle
ang = 0.5 * torch.atan2(sin, cos)
print(f"Angle range: [{ang.min():.2f}, {ang.max():.2f}] (should be ~[-π/2, π/2])")
```

**Success Criteria**:

- ✅ No shape mismatches
- ✅ Angle range is [-π/2, π/2]
- ✅ No runtime errors

---

### 3.2 Angle Convention Validation

**Tasks**:

- [ ] Test with rectangular object at 0°, 45°, 90°
- [ ] Log predicted angles from GR-ConvNet
- [ ] Verify 90° offset still produces correct jaw axis
- [ ] Visualize grasps to confirm perpendicularity

**Test Procedure**:

```
1. Place rectangle vertical (long axis up-down)
   Expected: Network predicts ~0-10°
   With offset: Jaw axis ~90-100° (grasps short side) ✓

2. Place rectangle at 45°
   Expected: Network predicts ~40-50°
   With offset: Jaw axis ~130-140° (grasps short side) ✓

3. Place rectangle horizontal (long axis left-right)
   Expected: Network predicts ~80-90° or ~-90-80°
   With offset: Jaw axis ~170-180° or ~0-10° (grasps short side) ✓
```

**Adjustments**:

- If angles are inverted: Try `-1.5708` instead of `1.5708`
- If grasps are 180° off: Adjust `compose_order`

---

### 3.3 Full Pipeline Integration Test

**Tasks**:

- [ ] Set `GRASP_MODEL_TYPE = 'grconvnet'` in config
- [ ] Run full grasp detection pipeline
- [ ] Verify 2D→3D conversion works
- [ ] Verify IK solver receives correct orientation
- [ ] Test on real robot (if available) or simulation

**Validation Checkpoints**:

```
✓ Preprocessing: Depth normalized correctly
✓ Inference: 4 output maps generated
✓ Decoding: Quality [0,1], Angle [-π/2, π/2], Width [0, ∞)
✓ Postprocessing: Best grasp selected
✓ 2D→3D: Camera frame pose generated
✓ Transform: Base frame pose calculated
✓ Orientation: Target orientation matrix composed
✓ IK: Joint angles computed
✓ Execution: Robot moves to grasp (if testing on hardware)
```

**Success Criteria**:

- ✅ All axis-aligned grasps still work (0°, 90°)
- ✅ Diagonal grasps significantly improved (45°)
- ✅ No integration errors in pipeline
- ✅ Inference time <500ms

---

## Phase 4: Performance Optimization (1-2 hours)

### 4.1 Benchmark & Compare

**Tasks**:

- [ ] Run side-by-side comparison: GGCNN2 vs GR-ConvNet
- [ ] Measure inference time on your CPU
- [ ] Measure grasp success rate on test set
- [ ] Document angle prediction ranges

**Metrics to Track**:

```
| Metric | GGCNN2 | GR-ConvNet | Improvement |
|--------|--------|------------|-------------|
| Inference time (CPU) | 150ms | ?ms | ? |
| Vertical grasp success | 100% | ?% | ? |
| Horizontal grasp success | 100% | ?% | ? |
| 45° grasp success | ~30% | ?% | ? |
| Angle range (max abs) | 30° | ?° | ? |
```

---

### 4.2 Fine-Tuning

**Tasks**:

- [ ] Adjust `min_quality_threshold` if needed
- [ ] Tune scoring weights if GR-ConvNet has different quality distribution
- [ ] Verify/adjust `grasp_angle_offset_rad` (may still be 1.5708 or different)
- [ ] Test depth inpainting if needed for sparse depth

**Optional Optimizations**:

- GPU acceleration if available
- Batch processing if multiple frames
- Reduce input size to 224×224 for speed (if accuracy acceptable)

---

## Phase 5: Production Deployment (0.5-1 hour)

### 5.1 Final Configuration

**Tasks**:

- [ ] Set `GRASP_MODEL_TYPE = 'grconvnet'` as default
- [ ] Remove or comment out GGCNN2 fallback code
- [ ] Update documentation
- [ ] Commit to git with clear message

**Configuration Lock-in**:

```python
# config.py - Final production settings
GRASP_MODEL_TYPE = 'grconvnet'  # Production model
GRCONVNET_MODEL_PATH = ...      # Production weights

GRASP_DETECTION_CONFIG = {
    'min_quality_threshold': 0.10,  # Validated threshold
    'grasp_angle_offset_rad': 1.5708,  # Validated offset (or -1.5708)
    # ... other validated params
}
```

---

### 5.2 Documentation & Cleanup

**Tasks**:

- [ ] Update README with GR-ConvNet information
- [ ] Document migration changes
- [ ] Remove backup files
- [ ] Archive GGCNN2 weights (keep for reference)

---

## Detailed Implementation Steps

### Step 1: Obtain GR-ConvNet Code & Weights

**Where to find**:

- **Official repo**: Search for "GR-ConvNet v2 GitHub" or "Generative Residual Convolutional Neural Network grasping"
- **Alternative**: `https://github.com/dougsm/` (author of original GGCNN work)
- **Pre-trained weights**: Check repo releases or model zoo

**What to download**:

```
grconvnet/
  ├── model.py              # GR-ConvNet architecture
  ├── cornell_trained.pt    # Cornell weights
  └── jacquard_trained.pt   # Jacquard weights (if available)
```

---

### Step 2: Implement GR-ConvNet Model Class

**Create**: `src/object_detection/grconvnet.py`

**Implementation checklist**:

```python
import torch
import torch.nn as nn
import torch.nn.functional as F

class GRConvNet(nn.Module):
    """
    GR-ConvNet v2: Generative Residual Convolutional Neural Network for grasping.

    Architecture based on: [cite paper]
    Compatible interface with GGCNN2 for drop-in replacement.
    """

    def __init__(self, input_channels=1, filter_sizes=None, dropout=False,
                 dropout_prob=0.0, input_size=300):
        super().__init__()

        # TODO: Copy architecture from GR-ConvNet repo
        # Encoder blocks with residual connections
        # Decoder blocks with upsampling
        # 4 output heads matching GGCNN2 format

    def forward(self, x):
        """
        Forward pass.

        Args:
            x: Input tensor (B, 1, H, W) - depth image

        Returns:
            pos_output: Quality map (B, 1, H, W)
            cos_output: cos(2θ) map (B, 1, H, W)
            sin_output: sin(2θ) map (B, 1, H, W)
            width_output: Width map (B, 1, H, W)
        """
        # TODO: Implement forward pass
        # CRITICAL: Return 4 outputs in SAME order as GGCNN2
        return pos_output, cos_output, sin_output, width_output

    def compute_loss(self, xc, yc):
        """Optional: For training (not needed for inference only)."""
        # TODO: Implement if planning to retrain
        pass
```

**Key Requirements**:

- ✅ Same 4-output format as GGCNN2
- ✅ Input size configurable (default 300)
- ✅ Single-channel depth input
- ✅ Output resolution matches input (300×300)

---

### Step 3: Update Grasp Detector Module

**File**: `src/object_detection/ggcnn2_module.py`

**Option A: Minimal Change (Recommended)**
Keep filename, add model selection:

```python
# Line ~18: Add import
from object_detection.grconvnet import GRConvNet

# Line ~46-56: Modify __init__
# Configuration parameters
self.resize_size = GRCONVNET_CONFIG.get('input_size', 300) if GRASP_MODEL_TYPE == 'grconvnet' else 300
self.depth_sample_radius = int(GRASP_DETECTION_CONFIG.get('depth_sample_radius', 5))

# Model selection
if GRASP_MODEL_TYPE == 'grconvnet':
    self.model = GRConvNet(
        input_channels=1,
        input_size=self.resize_size,
        dropout=GRCONVNET_CONFIG.get('use_dropout', False),
        dropout_prob=GRCONVNET_CONFIG.get('dropout_prob', 0.0)
    )
    model_path = str(GRCONVNET_MODEL_PATH)
    logger.info("🔄 Using GR-ConvNet model for grasp detection")
else:
    self.model = GGCNN2()
    model_path = str(GGCNN2_MODEL_PATH)
    logger.info("Using GGCNN2 model for grasp detection")

state_dict = torch.load(model_path, map_location=self.device)
self.model.load_state_dict(state_dict)
self.model.to(self.device).eval()

logger.info(f"Model loaded from {model_path} on {self.device}")
logger.info(f"Depth sample radius: {self.depth_sample_radius}, resize: {self.resize_size}x{self.resize_size}")
```

**NO changes needed to**:

- `preprocess()` - Works for both models
- `infer()` - Same output format
- `postprocess()` - Same map format
- All downstream methods - Unchanged

**Option B: Separate Module (More Work)**
Create `grconvnet_module.py` as separate class (not recommended - adds maintenance burden)

---

### Step 4: Add Preprocessing Validation

**File**: `src/object_detection/ggcnn2_module.py` (or renamed)

**Optional Enhancement** (if GR-ConvNet training used different normalization):

```python
def preprocess(self, depth_image: np.ndarray) -> torch.Tensor:
    """
    Preprocess depth image for grasp network inference.

    Handles both GGCNN2 and GR-ConvNet preprocessing requirements.
    """
    # Crop to square
    h, w = depth_image.shape
    min_dim = min(h, w)
    start_h = (h - min_dim) // 2
    start_w = (w - min_dim) // 2
    depth = depth_image[start_h:start_h+min_dim, start_w:start_w+min_dim]

    # Resize
    depth = cv2.resize(depth, (self.resize_size, self.resize_size))

    # Model-specific normalization
    if GRASP_MODEL_TYPE == 'grconvnet':
        # TODO: Verify GR-ConvNet training normalization
        # May need depth inpainting for sparse regions
        # depth = inpaint_depth(depth)  # if needed

        # Use same normalization for now
        depth = np.clip(depth, 0.2, 1.2)
        depth = (depth - 0.2) / 1.0
    else:
        # GGCNN2 normalization (current)
        depth = np.clip(depth, 0.2, 1.2)
        depth = (depth - 0.2) / 1.0

    depth_tensor = torch.from_numpy(depth).unsqueeze(0).unsqueeze(0).float()
    return depth_tensor.to(self.device)
```

---

## Phase 6: Angle Convention Verification (CRITICAL)

### 6.1 Validate 90° Offset Requirement

**Test Script**: `test_grconvnet_angle_convention.py`

```python
"""
Verify that GR-ConvNet's angle convention matches GGCNN2.
Test if 90° offset is still needed or if it should be removed/changed.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

# Test cases: 3 rectangular objects
test_cases = [
    {"name": "Vertical", "long_axis_angle": 90, "expected_network_angle": 90, "expected_jaw": 180},
    {"name": "Horizontal", "long_axis_angle": 0, "expected_network_angle": 0, "expected_jaw": 90},
    {"name": "45°", "long_axis_angle": 45, "expected_network_angle": 45, "expected_jaw": 135},
]

for test in test_cases:
    print(f"\nTest: {test['name']} rectangle")
    print(f"  Long axis at: {test['long_axis_angle']}°")
    print(f"  Expected network: ~{test['expected_network_angle']}°")
    print(f"  Expected jaw (with offset): ~{test['expected_jaw']}°")
    print(f"  Run grasp detection and compare!")
```

**Instructions**:

1. Place rectangle at each orientation
2. Run grasp detection with GR-ConvNet
3. Check logs for predicted angle
4. Verify jaw axis = predicted + 90° gives correct grasp
5. If wrong, try offset = 0.0, -1.5708, or other value

---

## Risk Mitigation & Rollback Strategy

### Rollback Points

| Phase           | Rollback Action                   | Time to Rollback |
| --------------- | --------------------------------- | ---------------- |
| After Phase 1   | None needed (just research)       | N/A              |
| After Phase 2.1 | Delete `grconvnet.py`             | 1 minute         |
| After Phase 2.3 | Set `GRASP_MODEL_TYPE = 'ggcnn2'` | 1 minute         |
| After Phase 3   | `git checkout main`               | 1 minute         |
| Production      | Revert config, restart system     | 5 minutes        |

### Risk Mitigation

**Risk 1: Model weights incompatible**

- **Mitigation**: Test loading in Phase 2.1 before integration
- **Fallback**: Stay with GGCNN2, defer migration

**Risk 2: Angle convention mismatch**

- **Mitigation**: Dedicated testing in Phase 6.1
- **Fallback**: Adjust offset or composition order

**Risk 3: Performance degradation on axis-aligned**

- **Mitigation**: Benchmark in Phase 3.3
- **Fallback**: Revert to GGCNN2 if <90% success

**Risk 4: Inference too slow on CPU**

- **Mitigation**: Benchmark in Phase 4.1
- **Fallback**: Reduce input size or stick with GGCNN2

---

## Timeline & Effort Breakdown

| Phase                | Tasks                            | Estimated Time | Can Pause?         |
| -------------------- | -------------------------------- | -------------- | ------------------ |
| 1.1 Research         | Find resources, download weights | 0.5-1 hour     | ✅ Yes             |
| 1.2 Setup            | Branch, backup, baseline         | 0.5 hour       | ✅ Yes             |
| 2.1 Model class      | Implement GRConvNet              | 1-2 hours      | ✅ Yes             |
| 2.2 Config           | Add parameters                   | 0.5 hour       | ✅ Yes             |
| 2.3 Integration      | Modify module init               | 0.5-1 hour     | ✅ Yes             |
| 3.1 Synthetic test   | Verify outputs                   | 0.5 hour       | ✅ Yes             |
| 3.2 Angle validation | Verify convention                | 1 hour         | ✅ Yes             |
| 3.3 Full pipeline    | End-to-end test                  | 1-2 hours      | ⚠️ Should complete |
| 4.1 Benchmark        | Performance testing              | 0.5-1 hour     | ✅ Yes             |
| 4.2 Tuning           | Optimize parameters              | 0.5-1 hour     | ✅ Yes             |
| 5 Deployment         | Final config, docs               | 0.5-1 hour     | ✅ Yes             |

**Total Estimate**: 6-10 hours (spread over 2-3 days with testing breaks)

---

## Success Criteria

### Must Have (Go/No-Go)

- ✅ GR-ConvNet loads and runs inference
- ✅ Output format matches GGCNN2 (4 maps, correct shapes)
- ✅ Angle decoding works ([-π/2, π/2] range)
- ✅ Integration with pipeline (no errors)
- ✅ Axis-aligned performance ≥90% (maintained)

### Should Have (Target Goals)

- ✅ 45° grasp success >70% (up from ~30%)
- ✅ Angle prediction range >45° (up from ~30°)
- ✅ Inference time <300ms on CPU (vs 150ms GGCNN2)
- ✅ No new dependencies beyond PyTorch

### Nice to Have (Stretch Goals)

- ✅ Inference time <150ms (match or beat GGCNN2)
- ✅ 45° grasp success >90%
- ✅ Works well on complex shapes (not just rectangles)

---

## Configuration Changes Required

### File: `src/config/config.py`

```python
# Add after line 199 (GGCNN2_MODEL_PATH):

# Model selection: 'ggcnn2' or 'grconvnet'
GRASP_MODEL_TYPE = 'ggcnn2'  # Switch to 'grconvnet' after migration

# GR-ConvNet model path
GRCONVNET_MODEL_PATH = SRC_DIR / "resources" / "ml_models" / \
    "grconvnet_weights" / "grconvnet_jacquard.pt"

# GR-ConvNet specific configuration
GRCONVNET_CONFIG = {
    'input_size': 300,       # Keep 300 to match current pipeline
    'use_dropout': False,    # Disable for inference
    'dropout_prob': 0.0,
    'use_depth_inpainting': False,  # Enable if depth is sparse
}

# Note: GRASP_DETECTION_CONFIG parameters apply to both models
# The grasp_angle_offset_rad may need re-validation for GR-ConvNet
```

---

## Testing Checklist

### Pre-Migration Baseline (GGCNN2)

```
□ Test vertical rectangle: ___ success rate
□ Test horizontal rectangle: ___ success rate
□ Test 45° rectangle: ___ success rate
□ Inference time: ___ ms
□ Angle range observed: [___, ___]°
```

### Post-Migration Validation (GR-ConvNet)

```
□ Model loads successfully
□ Output shapes match (4 maps, 300×300)
□ Angle decoding works ([-π/2, π/2])
□ Test vertical rectangle: ___ success rate (should be ≥baseline)
□ Test horizontal rectangle: ___ success rate (should be ≥baseline)
□ Test 45° rectangle: ___ success rate (should be >baseline)
□ Inference time: ___ ms (should be <500ms)
□ Angle range observed: [___, ___]° (should be >baseline)
□ 90° offset still correct? Yes/No/Needs adjustment to ___
```

---

## Code Compatibility Verification

### Files That Need Changes

- ✅ `src/object_detection/grconvnet.py` - **NEW** model class
- ✅ `src/object_detection/ggcnn2_module.py` - Model selection logic
- ✅ `src/config/config.py` - Add GR-ConvNet configuration

### Files That Should NOT Need Changes

- ✅ `src/camera_management/camera_manager.py` - No changes
- ✅ `src/camera_management/camera_transform_module.py` - No changes
- ✅ `src/kinematics/kinematics_solver.py` - No changes
- ✅ `src/control/telemetry_store.py` - No changes
- ✅ `src/states/grasping_state.py` - No changes (if using same module)
- ✅ All other states and pipeline components - No changes

**Reason**: GR-ConvNet is a drop-in replacement at the network level; all downstream processing remains identical.

---

## Decision Points

### Decision 1: After Phase 2.1

**Question**: Does GR-ConvNet model load and produce correct output shapes?

- **Yes** → Proceed to Phase 2.3
- **No** → Debug architecture or seek alternative weights

### Decision 2: After Phase 3.2

**Question**: Does GR-ConvNet improve 45° angle detection?

- **Yes** → Proceed to full integration
- **No** → Reassess if migration worth the effort

### Decision 3: After Phase 3.3

**Question**: Does GR-ConvNet maintain axis-aligned performance?

- **Yes** → Proceed to production
- **No** → Tune or rollback to GGCNN2

### Decision 4: After Phase 4.1

**Question**: Is performance acceptable (success rate, speed)?

- **Yes** → Deploy to production
- **No** → Optimize or rollback

---

## Alternative: Phased Rollout

If you want to minimize risk, use **dual-model mode**:

```python
# Run both models, compare results
if GRASP_MODEL_TYPE == 'both':
    ggcnn2_result = run_ggcnn2(depth)
    grconvnet_result = run_grconvnet(depth)

    # Log both for comparison
    logger.info(f"GGCNN2: angle={ggcnn2_result['angle']}, Q={ggcnn2_result['quality']}")
    logger.info(f"GR-ConvNet: angle={grconvnet_result['angle']}, Q={grconvnet_result['quality']}")

    # Use GR-ConvNet by default, fallback to GGCNN2 if low quality
    if grconvnet_result['quality'] > 0.5:
        return grconvnet_result
    else:
        return ggcnn2_result
```

This allows gradual transition with safety net.

---

## Next Steps

1. **Review this plan** with another AI system or colleague
2. **Gather feedback** on phasing, risk mitigation, timeline
3. **Obtain GR-ConvNet resources** (code, weights)
4. **Begin Phase 1** when ready
5. **Test at each phase** with decision points

---

## Open Questions for Review

1. Should we rename `ggcnn2_module.py` to `grasp_detector_module.py` for clarity?
2. Should we implement dual-model comparison mode for safety?
3. What specific test objects/scenarios should be in the validation set?
4. Should we test on simulation first or directly on robot hardware?
5. What is the rollback tolerance (% success rate degradation acceptable)?

---

## Approval Checklist

Before starting implementation:

```
□ Plan reviewed by stakeholder/reviewer
□ GR-ConvNet repo/weights identified and accessible
□ Backup strategy confirmed
□ Test environment prepared
□ Timeline acceptable (6-10 hours over 2-3 days)
□ Success criteria agreed upon
□ Rollback procedure understood
□ Resource allocation confirmed (GPU if needed)
```

---

**Status**: ✅ PLAN READY FOR REVIEW

**Recommendation**: Proceed with migration - High probability of success with moderate effort and manageable risk.
