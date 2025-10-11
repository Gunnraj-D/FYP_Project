# GR-ConvNet Migration Plan - FINAL VERSION

**Status**: ✅ APPROVED - Ready for Implementation  
**Date**: 2025-10-11  
**Estimated Effort**: 6-10 hours over 2-3 days

---

## Executive Summary

**Decision**: **PROCEED** with migration to GR-ConvNet v2

**Rationale**:

- ✅ **Compatible architecture** - Same 4-head output format (Q, cos2θ, sin2θ, W)
- ✅ **Same decoding** - Identical sigmoid/atan2/ReLU pipeline
- ✅ **Drop-in replacement** - Minimal integration changes
- ✅ **Expected improvement** - Better rotation invariance for 45° objects
- ✅ **Manageable risk** - Easy rollback at each phase

**Critical Success Factors** (from review feedback):

1. **Verify angle/jaw convention** - Test +90° offset still correct
2. **Match training normalization** - Avoid distribution shift
3. **Keep 300×300 input** - Preserve current scaling
4. **Dual-model comparison** - Safe validation before cutover

---

## Phase 1: Preparation (1-2 hours)

### Task 1.1: Acquire GR-ConvNet Resources ⭐ CRITICAL

**Actions**:

```bash
# 1. Clone GR-ConvNet repository
git clone https://github.com/[grconvnet-repo-url]
# Or search: "GR-ConvNet v2 GitHub robotic grasping"

# 2. Download pre-trained weights
# Look for: cornell_trained.pt or jacquard_trained.pt
# Preferred: Cornell (matches current GGCNN2 training)

# 3. Review architecture code
# File: model.py or grconvnet.py in repo
```

**Deliverables**:

- [ ] GR-ConvNet repository cloned
- [ ] Pre-trained weights downloaded (Cornell preferred)
- [ ] Architecture code reviewed
- [ ] Training normalization documented (CRITICAL for Step 1.3)

**Decision Point**: If weights unavailable or incompatible license → **ABORT**, stay with GGCNN2

---

### Task 1.2: Create Migration Branch

**Actions**:

```bash
cd C:\Users\Raj\Documents\Year_4\FYP\FYP_Project

# Create migration branch
git checkout -b grconvnet-migration

# Backup current working code
cp src/object_detection/ggcnn2_module.py src/object_detection/ggcnn2_module_backup.py

# Document current baseline
# Run 3 test cases and record results
```

**Pre-Migration Baseline** (fill in):

```
✅ GGCNN2 Baseline (record before starting):
□ Vertical rectangle (0°):   Success: ___%, Angle: ___°
□ Horizontal rectangle (90°): Success: ___%, Angle: ___°
□ Diagonal rectangle (45°):   Success: ___%, Angle: ___°
□ Inference time:             ___ms per frame
□ Angle range observed:       [___, ___]°
```

---

### Task 1.3: Document GR-ConvNet Training Preprocessing ⭐ CRITICAL

**From GR-ConvNet repo/paper, document**:

```
□ Input size used in training: ___ × ___ (likely 224 or 300)
□ Depth normalization formula: ___
□ Depth clipping range: [___m, ___m]
□ Inpainting used?: Yes/No
□ RGB channels used?: Yes/No (we'll use depth-only)
```

**If different from GGCNN2**:

- Update preprocessing to match GR-ConvNet training
- Avoid distribution shift that degrades performance

---

## Phase 2: Implementation (2-3 hours)

### Task 2.1: Implement GR-ConvNet Model Class

**File**: `src/object_detection/grconvnet.py` ← **NEW**

**Actions**:

1. Copy GR-ConvNet architecture from repo
2. Adapt to match GGCNN2 interface
3. Ensure 4-output format: `(pos, cos, sin, width)`
4. Configure for 300×300 input size
5. Test on dummy data

**Implementation**:

```python
import torch
import torch.nn as nn
import torch.nn.functional as F

class GRConvNet(nn.Module):
    """
    GR-ConvNet v2: Generative Residual Convolutional Neural Network.

    Compatible drop-in replacement for GGCNN2.
    Returns same 4-head output format.
    """

    def __init__(self, input_channels=1, filter_sizes=None, dropout=False,
                 dropout_prob=0.0, input_size=300):
        super().__init__()

        if filter_sizes is None:
            # GR-ConvNet v2 defaults (adjust based on repo)
            filter_sizes = [32, 16, 8, 8, 16, 32]

        self.input_size = input_size

        # TODO: Copy encoder/decoder architecture from GR-ConvNet repo
        # Encoder with residual blocks
        # Decoder with upsampling
        # 4 output heads (SAME as GGCNN2)

        self.pos_output = nn.Conv2d(filter_sizes[-1], 1, kernel_size=1)
        self.cos_output = nn.Conv2d(filter_sizes[-1], 1, kernel_size=1)
        self.sin_output = nn.Conv2d(filter_sizes[-1], 1, kernel_size=1)
        self.width_output = nn.Conv2d(filter_sizes[-1], 1, kernel_size=1)

    def forward(self, x):
        """
        Forward pass.

        Args:
            x: (B, 1, H, W) depth tensor

        Returns:
            pos_output: (B, 1, H, W) quality map
            cos_output: (B, 1, H, W) cos(2θ)
            sin_output: (B, 1, H, W) sin(2θ)
            width_output: (B, 1, H, W) width map
        """
        # TODO: Implement forward pass from GR-ConvNet repo

        # CRITICAL: Return in SAME order as GGCNN2
        return pos_output, cos_output, sin_output, width_output
```

**Verification Test**:

```python
# Quick test before integration
model = GRConvNet(input_size=300)
test_input = torch.randn(1, 1, 300, 300)
pos, cos, sin, width = model(test_input)

assert pos.shape == (1, 1, 300, 300), f"Expected (1,1,300,300), got {pos.shape}"
assert cos.shape == (1, 1, 300, 300)
assert sin.shape == (1, 1, 300, 300)
assert width.shape == (1, 1, 300, 300)

print("✅ GRConvNet output shapes correct")
```

**Decision Point**: If shapes don't match → Debug architecture before proceeding

---

### Task 2.2: Update Configuration

**File**: `src/config/config.py`

**Add after line 200** (after GGCNN2_MODEL_PATH):

```python
# ============================================================================
# GR-CONVNET CONFIGURATION
# ============================================================================

# Model selection: 'ggcnn2' or 'grconvnet'
GRASP_MODEL_TYPE = 'ggcnn2'  # ← Start with GGCNN2, switch after validation

# GR-ConvNet model path
GRCONVNET_MODEL_PATH = SRC_DIR / "resources" / "ml_models" / \
    "grconvnet_weights" / "grconvnet_cornell.pt"

# GR-ConvNet specific configuration
GRCONVNET_CONFIG = {
    'input_size': 300,              # Keep 300 to preserve current pipeline
    'use_dropout': False,           # Disable for inference
    'dropout_prob': 0.0,
    'use_depth_inpainting': False,  # Enable if depth sparse (test first)
}

# Note: GRASP_DETECTION_CONFIG applies to BOTH models
# grasp_angle_offset_rad will be re-validated for GR-ConvNet
```

---

### Task 2.3: Rename and Update Grasp Detector Module ⭐ CRITICAL

**File Changes**:

```bash
# Rename for clarity (per reviewer feedback)
git mv src/object_detection/ggcnn2_module.py src/object_detection/grasp_detector_module.py
```

**Update Class** (`grasp_detector_module.py`):

**Line ~1: Update docstring**:

```python
"""
Grasp Detector Module - Wrapper for grasp synthesis networks.

Supports both GGCNN2 and GR-ConvNet models with unified interface.
Model selection via GRASP_MODEL_TYPE config parameter.
"""
```

**Line ~18: Add import**:

```python
from object_detection.ggcnn2 import GGCNN2
from object_detection.grconvnet import GRConvNet  # ← ADD THIS
from config.config import (
    GRASP_DETECTION_CONFIG, GRASP_EXECUTION_CONFIG,
    DEBUG_MODE, DEBUG_CONFIG,
    GRASP_MODEL_TYPE, GRCONVNET_CONFIG,  # ← ADD THESE
    GGCNN2_MODEL_PATH, GRCONVNET_MODEL_PATH  # ← ADD THIS
)
```

**Line ~46-57: Replace **init** model loading**:

```python
# Configuration parameters
if GRASP_MODEL_TYPE == 'grconvnet':
    self.resize_size = GRCONVNET_CONFIG.get('input_size', 300)
else:
    self.resize_size = 300

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

# Load weights
state_dict = torch.load(model_path, map_location=self.device)
self.model.load_state_dict(state_dict)
self.model.to(self.device).eval()

logger.info(f"Model loaded from {model_path} on {self.device}")
logger.info(f"Model type: {GRASP_MODEL_TYPE.upper()}, "
           f"Input size: {self.resize_size}x{self.resize_size}, "
           f"Depth sample radius: {self.depth_sample_radius}")
```

**NO OTHER CHANGES** to grasp_detector_module.py needed!

- ✅ `preprocess()` - Works for both
- ✅ `infer()` - Same output format
- ✅ `postprocess()` - Same maps
- ✅ All other methods - Unchanged

---

### Task 2.4: Update All Import References

**Files that import ggcnn2_module**:

```bash
# Search for imports
grep -r "from.*ggcnn2_module import" src/
grep -r "import.*ggcnn2_module" src/
```

**Update each import**:

```python
# OLD:
from object_detection.ggcnn2_module import GGcnn2Module

# NEW:
from object_detection.grasp_detector_module import GGcnn2Module
```

**Likely files to update**:

- `src/states/grasping_state.py`
- Any test files
- Main integration files

---

## Phase 3: Validation & Testing (3-4 hours)

### Task 3.1: Preprocessing Normalization Match ⭐ CRITICAL

**Goal**: Ensure preprocessing matches GR-ConvNet training to avoid distribution shift

**Actions**:

```python
# In grasp_detector_module.py, update preprocess() if needed:

def preprocess(self, depth_image: np.ndarray) -> torch.Tensor:
    """Preprocess depth for grasp network (GGCNN2 or GR-ConvNet)."""

    # Crop to square (SAME for both)
    h, w = depth_image.shape
    min_dim = min(h, w)
    start_h = (h - min_dim) // 2
    start_w = (w - min_dim) // 2
    depth = depth_image[start_h:start_h+min_dim, start_w:start_w+min_dim]

    # Resize (SAME for both if using 300)
    depth = cv2.resize(depth, (self.resize_size, self.resize_size))

    # Model-specific preprocessing
    if GRASP_MODEL_TYPE == 'grconvnet':
        # TODO: Apply GR-ConvNet training preprocessing from Task 1.3
        # If GR-ConvNet used inpainting:
        if GRCONVNET_CONFIG.get('use_depth_inpainting', False):
            depth = self._inpaint_depth(depth)  # Implement if needed

        # Use SAME normalization as training (verify from Task 1.3)
        # Default assumption: same as GGCNN2 unless documented otherwise
        depth = np.clip(depth, 0.2, 1.2)
        depth = (depth - 0.2) / 1.0
    else:
        # GGCNN2 normalization (current, proven)
        depth = np.clip(depth, 0.2, 1.2)
        depth = (depth - 0.2) / 1.0

    depth_tensor = torch.from_numpy(depth).unsqueeze(0).unsqueeze(0).float()
    return depth_tensor.to(self.device)
```

**Verification**:

- [ ] Normalization range matches training docs
- [ ] No NaN or Inf values in preprocessed tensor
- [ ] Visual check: depth visualization looks reasonable

---

### Task 3.2: Model Loading Test

**Test Script**: `test_grconvnet_loading.py`

```python
"""Test GR-ConvNet model loading and basic inference."""

import torch
import numpy as np
from object_detection.grconvnet import GRConvNet

print("=" * 60)
print("GR-ConvNet Model Loading Test")
print("=" * 60)

# 1. Create model
model = GRConvNet(input_channels=1, input_size=300)
print(f"✓ Model instantiated")

# 2. Load weights
try:
    weights_path = "path/to/grconvnet_cornell.pt"  # Update path
    state_dict = torch.load(weights_path, map_location='cpu')
    model.load_state_dict(state_dict)
    print(f"✓ Weights loaded from {weights_path}")
except Exception as e:
    print(f"✗ Failed to load weights: {e}")
    exit(1)

# 3. Set to eval mode
model.eval()
print(f"✓ Model set to eval mode")

# 4. Test inference with dummy data
dummy_depth = torch.randn(1, 1, 300, 300)
with torch.no_grad():
    pos, cos, sin, width = model(dummy_depth)

# 5. Verify output shapes
assert pos.shape == (1, 1, 300, 300), f"pos shape: {pos.shape}"
assert cos.shape == (1, 1, 300, 300), f"cos shape: {cos.shape}"
assert sin.shape == (1, 1, 300, 300), f"sin shape: {sin.shape}"
assert width.shape == (1, 1, 300, 300), f"width shape: {width.shape}"
print(f"✓ Output shapes correct: (1, 1, 300, 300)")

# 6. Test decoding
q_img = torch.sigmoid(pos)
ang_img = 0.5 * torch.atan2(sin, cos)
width_img = F.relu(width)

print(f"✓ Decoding works:")
print(f"  Quality range: [{q_img.min():.3f}, {q_img.max():.3f}]")
print(f"  Angle range: [{ang_img.min():.3f}, {ang_img.max():.3f}] rad")
print(f"  Angle range: [{np.degrees(ang_img.min()):.1f}°, {np.degrees(ang_img.max()):.1f}°]")
print(f"  Width range: [{width_img.min():.3f}, {width_img.max():.3f}]")

print("\n" + "=" * 60)
print("✅ GR-ConvNet model ready for integration!")
print("=" * 60)
```

**Run**:

```bash
python test_grconvnet_loading.py
```

**Decision Point**: If test fails → Debug before proceeding to integration

---

### Task 3.3: Angle Convention Validation ⭐ CRITICAL

**Goal**: Verify if +90° offset is still correct for GR-ConvNet

**Test Procedure**:

1. **Set** `GRASP_MODEL_TYPE = 'grconvnet'` temporarily
2. **Place** rectangular object at 3 orientations
3. **Record** predicted angles from logs
4. **Verify** jaw axis alignment

**Expected Results**:

| Object Orientation | Long Axis Angle | Expected Network Prediction | With +90° Offset | Correct Grasp?        |
| ------------------ | --------------- | --------------------------- | ---------------- | --------------------- |
| Vertical           | 90° (up-down)   | ~85-90°                     | ~175-180° (≈0°)  | Short horizontal side |
| Horizontal         | 0° (left-right) | ~0-10°                      | ~90-100°         | Short vertical side   |
| 45° Diagonal       | 45°             | ~40-50° ⭐                  | ~130-140°        | Short side at angle   |

**Test Script**: Run your normal grasp detection with DEBUG_MODE=True

**Check logs for**:

```
📐 GGCNN2 predicted angle: XX.X° (contact line / long axis)
🔄 Applying 90.0° offset → Jaw axis: YY.Y°
```

**Validation**:

```
□ Vertical test: Network predicted ___°, Jaw ___°, Grasped short side? ___
□ Horizontal test: Network predicted ___°, Jaw ___°, Grasped short side? ___
□ 45° test: Network predicted ___°, Jaw ___°, Grasped short side? ___
```

**If grasps are WRONG**:

- Try `'grasp_angle_offset_rad': 0.0` (no offset)
- Try `'grasp_angle_offset_rad': -1.5708` (-90°)
- Try `'grasp_angle_offset_rad': 3.1416` (180°)

**Decision Point**: Must get correct grasps on 2/3 orientations to proceed

---

### Task 3.4: Angle Histogram Diagnostic (per reviewer)

**Test Script**: `test_angle_distribution.py`

```python
"""
Angle histogram test to verify GR-ConvNet predicts full angular range.
Per reviewer: Should see angles well beyond 30° for diagonal objects.
"""

import torch
import numpy as np
import matplotlib.pyplot as plt
from object_detection.grasp_detector_module import GGcnn2Module

# Load real depth images at different orientations
test_cases = [
    {"name": "Vertical", "image_path": "test_data/rect_vertical.npy", "expected_peak": "~0-10°"},
    {"name": "Horizontal", "image_path": "test_data/rect_horizontal.npy", "expected_peak": "~80-90°"},
    {"name": "45°", "image_path": "test_data/rect_45deg.npy", "expected_peak": "~40-50°"},
]

# Set to GR-ConvNet
from config import config
config.GRASP_MODEL_TYPE = 'grconvnet'

# Run detection and collect angles
for test in test_cases:
    depth = np.load(test['image_path'])

    # Get angle map from GR-ConvNet
    # (run through detector and extract ang_img before postprocessing)

    # Plot histogram
    plt.figure()
    plt.hist(angles.flatten(), bins=50, range=(-90, 90))
    plt.title(f"{test['name']}: Angle Distribution")
    plt.xlabel("Angle (degrees)")
    plt.ylabel("Frequency")
    plt.axvline(expected_angle, color='r', linestyle='--', label='Expected')
    plt.legend()
    plt.savefig(f"angle_hist_{test['name']}.png")

    print(f"{test['name']}: Range [{min:.1f}°, {max:.1f}°], Peak: {peak:.1f}°")
```

**Success Criteria**:

- ✅ 45° test shows predictions near 40-50° (not just <30° like GGCNN2)
- ✅ Angle range spans full [-90°, 90°] capability
- ✅ Peaks align with expected object orientations

---

### Task 3.5: Full Pipeline Integration Test

**Actions**:

1. Set `GRASP_MODEL_TYPE = 'grconvnet'`
2. Run full grasp detection on test objects
3. Verify end-to-end pipeline works
4. Check all integration points

**Validation Checkpoints**:

```
□ Camera input: Depth frame acquired
□ Preprocessing: Normalized tensor created
□ Inference: 4 output maps generated (300×300)
□ Decoding: Quality, angle, width decoded correctly
□ Postprocessing: Best grasp selected, depth sampled
□ 2D→3D: Camera frame pose calculated
□ Transform: Base frame position computed
□ Orientation: 3×3 rotation matrix composed
□ IK: 7 joint angles computed
□ Execution: Poses stored in telemetry
□ No errors or exceptions in pipeline
```

**Decision Point**: If any step fails → Debug before declaring success

---

## Phase 4: Dual-Model Comparison (1-2 hours)

### Task 4.1: Implement Dual-Model Mode (per reviewer)

**File**: `src/config/config.py`

**Update model type options**:

```python
GRASP_MODEL_TYPE = 'both'  # Options: 'ggcnn2', 'grconvnet', 'both'
```

**File**: `src/object_detection/grasp_detector_module.py`

**Add in infer() method** (before returning result):

```python
# In infer() around line 280, before return:

# Dual-model comparison mode
if GRASP_MODEL_TYPE == 'both':
    # Store GR-ConvNet result
    grconvnet_result = grasp_result.copy()

    # Run GGCNN2 for comparison
    self.model = GGCNN2()
    ggcnn2_state_dict = torch.load(str(GGCNN2_MODEL_PATH), map_location=self.device)
    self.model.load_state_dict(ggcnn2_state_dict)
    self.model.to(self.device).eval()

    # Re-run inference with GGCNN2
    depth_tensor = self.preprocess(depth_image)
    with torch.no_grad():
        pos, cos, sin, width = self.model(depth_tensor)
    q_img = torch.sigmoid(pos)
    ang_img = 0.5 * torch.atan2(sin, cos)
    width_img = F.relu(width)
    grasp_2d_ggcnn2 = self.postprocess(q_img, ang_img, width_img, depth_image, original_depth_frame)

    # Log comparison
    logger.info(f"📊 DUAL-MODEL COMPARISON:")
    logger.info(f"   GGCNN2:     Angle={np.degrees(grasp_2d_ggcnn2['angle']):6.1f}°, Q={grasp_2d_ggcnn2['quality']:.3f}")
    logger.info(f"   GR-ConvNet: Angle={np.degrees(grasp_2d['angle']):6.1f}°, Q={grasp_2d['quality']:.3f}")
    logger.info(f"   Δ Angle={abs(np.degrees(grasp_2d['angle'] - grasp_2d_ggcnn2['angle'])):.1f}°")

    # Use GR-ConvNet result (already computed)
    # Could add fallback logic here if needed
```

**Use this mode for** initial validation, then switch to 'grconvnet' only when confident.

---

### Task 4.2: Benchmark Performance

**Test**: Run on 10+ objects at multiple orientations

**Metrics to Record**:

```
Test Object: Rectangle A
  ┌─────────────────────┬─────────┬────────────┬─────────────┐
  │ Orientation         │ GGCNN2  │ GR-ConvNet │ Winner      │
  ├─────────────────────┼─────────┼────────────┼─────────────┤
  │ 0° (vertical)       │ Angle   │ Angle      │ ✓/✗         │
  │                     │ Quality │ Quality    │             │
  ├─────────────────────┼─────────┼────────────┼─────────────┤
  │ 45° (diagonal)      │ Angle   │ Angle      │ ✓/✗         │
  │                     │ Quality │ Quality    │             │
  ├─────────────────────┼─────────┼────────────┼─────────────┤
  │ 90° (horizontal)    │ Angle   │ Angle      │ ✓/✗         │
  │                     │ Quality │ Quality    │             │
  └─────────────────────┴─────────┴────────────┴─────────────┘

Inference Time:
  GGCNN2:     ___ms per frame
  GR-ConvNet: ___ms per frame

Angle Prediction Range:
  GGCNN2:     [___°, ___°]
  GR-ConvNet: [___°, ___°]  ← Should be wider!
```

**Success Criteria**:

- ✅ GR-ConvNet angle range > 45° on diagonal objects
- ✅ GR-ConvNet maintains or improves quality on 0°/90°
- ✅ Inference time < 500ms (prefer <200ms)

---

### Task 4.3: Visualization Validation (per reviewer)

**Goal**: Draw grasp rectangles from 4-head maps to verify jaw orientation

**Test Script**: Enable DEBUG_MODE and use built-in visualization

**Check**:

```
For each test object:
  1. Look at debug window showing grasp rectangle
  2. Verify rectangle's SHORT dimension is perpendicular to predicted angle
  3. Verify gripper would close across SHORT side
  4. Compare GGCNN2 vs GR-ConvNet visualizations
```

**Document with screenshots if possible**

---

## Phase 5: Production Cutover (0.5-1 hour)

### Task 5.1: Final Decision

**Based on Phase 4 results, decide**:

**GO criteria** (all must pass):

- ✅ GR-ConvNet detects 45° angles (range >40°)
- ✅ Axis-aligned performance maintained (≥90% success)
- ✅ Inference time acceptable (<500ms)
- ✅ Angle offset validated (correct grasps)
- ✅ No integration errors

**NO-GO criteria** (any triggers rollback):

- ❌ 45° detection doesn't improve (still <35° predictions)
- ❌ Axis-aligned performance degrades (<90%)
- ❌ Inference too slow (>500ms consistently)
- ❌ Integration issues can't be resolved
- ❌ Angle convention incompatible

---

### Task 5.2: Production Configuration

**If GO decision**, update `src/config/config.py`:

```python
# Set production model
GRASP_MODEL_TYPE = 'grconvnet'  # ← Production model

# Lock in validated offset
GRASP_DETECTION_CONFIG = {
    # ... existing params ...
    'grasp_angle_offset_rad': 1.5708,  # ← Or 0.0 / -1.5708 if different
    # ... rest unchanged ...
}
```

**Commit changes**:

```bash
git add .
git commit -m "Migrate to GR-ConvNet: Improved rotation invariance for diagonal grasps

- Add GRConvNet model class with GGCNN2-compatible interface
- Rename ggcnn2_module.py → grasp_detector_module.py
- Add model selection via GRASP_MODEL_TYPE config
- Validate angle offset: [record validated value]
- Test results: [summarize improvements]
- Maintains 100% success on 0°/90°, improves 45° from 30% to X%"

git push origin grconvnet-migration
```

---

### Task 5.3: Rollback Plan (if NO-GO)

**Actions**:

```bash
# Revert all changes
git checkout opc_communication_focus  # Or your main branch
git branch -D grconvnet-migration

# Restore backup if needed
cp src/object_detection/ggcnn2_module_backup.py src/object_detection/ggcnn2_module.py
```

**Set config back**:

```python
GRASP_MODEL_TYPE = 'ggcnn2'
```

**Document reasons** for rollback in project notes

---

## Critical Checkpoints Summary

### ⭐ CHECKPOINT 1: After Task 2.1

**Question**: Does GR-ConvNet model load and produce correct tensor shapes?

- **PASS** → Continue to Task 2.3
- **FAIL** → Debug or abort migration

### ⭐ CHECKPOINT 2: After Task 3.3

**Question**: Does +90° offset produce correct grasps on 2/3 orientations?

- **PASS** → Continue to Task 4.1
- **FAIL** → Try different offsets; if none work, abort

### ⭐ CHECKPOINT 3: After Task 4.2

**Question**: Does GR-ConvNet improve 45° angle detection (range >40°)?

- **PASS** → Proceed to production
- **FAIL** → Consider rollback (marginal improvement not worth migration)

### ⭐ CHECKPOINT 4: After Task 4.3

**Question**: Are axis-aligned grasps maintained at ≥90% success?

- **PASS** → Proceed to production
- **FAIL** → Rollback immediately

---

## Implementation Timeline

### Day 1 (3-4 hours)

- ✅ **Morning**: Phase 1 (Preparation) - 1-2 hours
- ✅ **Afternoon**: Phase 2 (Implementation) - 2-3 hours
- 🛑 **STOP**: Checkpoint 1 - Model loads?

### Day 2 (3-4 hours)

- ✅ **Morning**: Phase 3 (Validation) - 2-3 hours
- 🛑 **STOP**: Checkpoint 2 & 3 - Angle convention correct? Range improved?

### Day 3 (1-2 hours)

- ✅ **Morning**: Phase 4 (Comparison) - 1-2 hours
- 🛑 **DECISION**: Checkpoint 4 - GO or NO-GO?
- ✅ **Afternoon**: Phase 5 (Cutover or Rollback) - 0.5-1 hour

**Total**: 6-10 hours over 2-3 days with natural break points

---

## Resource Checklist

**Before starting, ensure you have**:

```
□ GR-ConvNet repository cloned
□ Pre-trained weights downloaded (Cornell recommended)
□ Training normalization documented
□ Test objects available (rectangle at minimum)
□ Camera system operational
□ DEBUG_MODE enabled for validation
□ Backup branch created
□ Baseline performance recorded
□ Time allocated (6-10 hours over 2-3 days)
□ Rollback procedure understood
```

---

## Files Modified Summary

### New Files

- ✅ `src/object_detection/grconvnet.py` - GR-ConvNet model class
- ✅ `test_grconvnet_loading.py` - Model loading test
- ✅ `test_angle_distribution.py` - Angle histogram test

### Modified Files

- ✅ `src/object_detection/ggcnn2_module.py` → `grasp_detector_module.py` (renamed)
- ✅ `src/config/config.py` - Add GR-ConvNet configuration
- ⚠️ Any files importing ggcnn2_module - Update imports

### Unchanged Files (verified)

- ✅ `src/camera_management/camera_manager.py`
- ✅ `src/camera_management/camera_transform_module.py`
- ✅ `src/kinematics/kinematics_solver.py`
- ✅ `src/control/telemetry_store.py`
- ✅ `src/control/command_bus.py`
- ✅ All state files (except import updates)

---

## Success Metrics

### Minimum Acceptable (Must Achieve)

- ✅ 0° grasp success: ≥90%
- ✅ 90° grasp success: ≥90%
- ✅ 45° grasp success: ≥50% (up from ~30%)
- ✅ Inference time: <500ms
- ✅ Angle range: >40° (up from ~30°)

### Target Goals

- 🎯 0° grasp success: ≥95%
- 🎯 90° grasp success: ≥95%
- 🎯 45° grasp success: ≥70%
- 🎯 Inference time: <300ms
- 🎯 Angle range: >60°

### Stretch Goals

- 🌟 45° grasp success: ≥90%
- 🌟 Inference time: <150ms (match GGCNN2)
- 🌟 Works on complex shapes, not just rectangles

---

## Post-Migration Actions

**If successful**:

1. Update main README with GR-ConvNet info
2. Merge migration branch to main
3. Archive GGCNN2 weights (keep for reference)
4. Document angle offset value used
5. Create migration summary document

**If rolled back**:

1. Document why migration failed
2. Keep learnings for future attempt
3. Consider alternatives (ensemble, different model)
4. Stay with optimized GGCNN2 setup

---

## Quick Reference Card

```
╔════════════════════════════════════════════════════════════╗
║           GR-CONVNET MIGRATION QUICK REFERENCE              ║
╠════════════════════════════════════════════════════════════╣
║ Switch to GR-ConvNet:                                       ║
║   config.GRASP_MODEL_TYPE = 'grconvnet'                    ║
║                                                             ║
║ Switch back to GGCNN2:                                      ║
║   config.GRASP_MODEL_TYPE = 'ggcnn2'                       ║
║                                                             ║
║ Dual-model comparison:                                      ║
║   config.GRASP_MODEL_TYPE = 'both'                         ║
║                                                             ║
║ Adjust angle offset if needed:                             ║
║   config.GRASP_DETECTION_CONFIG['grasp_angle_offset_rad']  ║
║   Try: 0.0, 1.5708, -1.5708, 3.1416                        ║
║                                                             ║
║ Emergency rollback:                                         ║
║   1. Set GRASP_MODEL_TYPE = 'ggcnn2'                       ║
║   2. Restart application                                    ║
║   3. Time required: <5 minutes                             ║
╚════════════════════════════════════════════════════════════╝
```

---

## Status: ✅ FINAL PLAN APPROVED

**Next Action**: Begin Phase 1, Task 1.1 (Acquire GR-ConvNet resources)

**Key Takeaways from Review**:

1. ✅ Outputs perfectly compatible - Same 4-head format
2. ✅ Preprocessing alignment critical - Match training normalization
3. ✅ Angle convention must be validated - Test +90° offset
4. ✅ Dual-model mode recommended - Safe comparison
5. ✅ Rename module for clarity - grasp_detector_module.py

**Proceed when ready!** 🚀
