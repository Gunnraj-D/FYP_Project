# Grasp Angle & TCP Link Fixes

## 🔴 Two Critical Issues Fixed

---

## Issue 1: Wrong TCP Link (Gripper Extension Missing)

### Problem:

IK solver was using **`tool0`** instead of **`tcp`** as end effector.

**Impact:**

- `tool0` → `tcp` = **138mm gripper extension downward**
- When IK targets Z=0.10m:
  - `tool0` positioned at 0.10m ✅
  - `tcp` (gripper tip) at 0.10 - 0.138 = **-0.038m** ❌ **INTO TABLE!**

### Fix:

Changed end effector link priority to find `tcp` first:

```python
# kinematics_solver.py
# Priority: tcp (gripper tip) > ee_link > flange > tool0
if link_name == 'tcp':  # HIGHEST PRIORITY
    self.end_effector_link_index = i
    break
```

### Result:

- ✅ IK now solves for gripper tip position
- ✅ Accounts for 138mm gripper extension
- ✅ Robot won't drive into table

---

## Issue 2: Grasp Angle Offset

### Question:

Does `grasp_angle_offset_rad` affect grasp DETECTION or only final EXECUTION?

### Answer:

**Only affects execution!** The offset is applied AFTER neural network prediction:

```python
# 1. Neural network predicts angle (NO offset involved)
predicted_angle = model.predict(...)

# 2. Offset applied when creating 3D orientation
angle += grasp_angle_offset_rad  # <-- HERE (not during detection)
orientation = compose_grasp_orientation(angle)
```

### User Requirement:

> "If this offset doesn't effect the actual process of determining grasps, then just make it 0."

✅ **Offset does NOT affect detection** → Set to 0.0

### Change Made:

```python
# Before:
'grasp_angle_offset_rad': 1.5708,   # 90° offset (π/2)

# After:
'grasp_angle_offset_rad': 0.0,      # No offset - apply rotation later if needed
```

### If You Need Gripper Rotation:

**Option A:** Apply after IK solve:

```python
# After getting joint_angles from IK
joint_angles[6] += np.pi/2  # Add 90° to A7 (wrist roll)
```

**Option B:** Modify target orientation before IK:

```python
# When calling solve_XYZ
target_orientation = get_facing_down_orientation() @ R_z(90°)
```

**Option C:** Re-enable the offset in config:

```python
'grasp_angle_offset_rad': 1.5708,  # 90° if needed
```

---

## 📊 Summary of Both Fixes

| Issue            | Before        | After          | Impact              |
| ---------------- | ------------- | -------------- | ------------------- |
| **TCP Link**     | tool0 (idx 7) | tcp (idx 9) ⭐ | +138mm compensation |
| **Grasp Offset** | 90° (1.5708)  | 0° (0.0)       | No pre-rotation     |

### Expected Behavior Now:

1. ✅ IK solves for gripper tip (`tcp` link index 9)
2. ✅ Gripper won't go below table (138mm accounted for)
3. ✅ Grasp angle used as-is from detector (no offset)
4. ✅ You can add wrist rotation later if needed

---

## 🧪 Testing Checklist:

```bash
# Run grasping
run 6

# Check logs:
✅ "Found TCP (gripper tip) at link index 9"
✅ Grasp Z position > 0.0m (above table)
✅ No "Z IS NEGATIVE" warnings

# Verify gripper orientation:
# - Should face down (180° roll)
# - Yaw should match detected grasp angle (no 90° added)
```

If gripper orientation is wrong, we'll add the 90° rotation in a different place.

---

**Files Modified:**

- `src/kinematics/kinematics_solver.py` - TCP link priority fix
- `src/config/grasp_config.py` - Angle offset set to 0

**Status:** ✅ Ready to test
