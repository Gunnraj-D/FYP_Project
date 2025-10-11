# Grasp Height Adjustment - 1.5cm Too Low

## 🔴 Issue: Gripper Going Too Low

**Symptom:** Gripper is ~1.5cm (15mm) too low when grasping  
**Cause:** No depth offset being applied to lift grasp position

---

## ✅ Fix Applied

### Added `grasp_depth_offset` Application

**Config setting:**

```python
'grasp_depth_offset': 0.025,  # 25mm offset (raised from 10mm)
```

**Where applied:** `grasping_state.py` when storing grasp pose

```python
# After getting grasp pose from detector:
grasp_pose_base = grasp_result['pose']

# Apply depth offset to raise position
depth_offset = GRASP_DETECTION_CONFIG.get('grasp_depth_offset', 0.0)
grasp_pose_base[2] += depth_offset  # Lift by 25mm

# Store adjusted pose
self.context.telemetry.set_generated_grasp_pose(grasp_pose_base)
```

---

## 🎯 Why This Is Needed

### Several Factors Contribute:

1. **TCP Link Correction**

   - Now using actual TCP (gripper tip) instead of tool0
   - This already accounts for 138mm gripper extension
   - But may have revealed calibration offset

2. **Depth Estimation Accuracy**

   - Camera depth might be slightly off
   - Table surface detection has some variance
   - Small systematic error accumulates

3. **Grasp Point Convention**
   - Detector predicts contact point center
   - May not account for gripper finger thickness
   - Small lift prevents collision

---

## 📊 Adjustment Values

| Setting                  | Value             | Purpose                                       |
| ------------------------ | ----------------- | --------------------------------------------- |
| `grasp_depth_offset`     | **0.025m (25mm)** | Lifts grasp position to prevent going too low |
| `approach_height_offset` | 0.050m (50mm)     | Approach pose above grasp (unchanged)         |
| `pickup_z_offset`        | 0.200m (200mm)    | Sequencer approach height (unchanged)         |

### Total Heights:

```
Detected grasp Z: e.g., 0.080m
  ↓ + grasp_depth_offset (+25mm)
Actual grasp Z: 0.105m
  ↓ + pickup_z_offset (+200mm)
Approach Z: 0.305m
```

---

## 🧪 Testing

**After restarting main_debug.py:**

```bash
run 6  # Test grasping

# Check logs for:
✅ "Applied grasp depth offset: 25.0mm"
✅ Gripper height should now be correct!
```

### If Still Too Low/High:

**Too low by Xmm** → Increase offset:

```python
'grasp_depth_offset': 0.025 + (X/1000),
```

**Too high by Xmm** → Decrease offset:

```python
'grasp_depth_offset': 0.025 - (X/1000),
```

---

## 🎓 Calibration Process

For perfect grasp height:

1. Place object with known height (e.g., 30mm screwdriver)
2. Run grasping, observe gripper height
3. Measure error (too low/high)
4. Adjust `grasp_depth_offset` accordingly
5. Repeat until perfect

**Current offset (25mm) is a good starting point for most objects.**

---

**File Modified:** `src/states/grasping_state.py`, `src/config/grasp_config.py`  
**Status:** ✅ Offset applied  
**Next:** Restart and test
