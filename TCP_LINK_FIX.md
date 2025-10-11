# TCP Link Fix - Gripper Extension Issue

## 🔴 CRITICAL BUG FOUND!

### The Problem:

**IK solver was using wrong end effector link!**

- **Current:** Using `tool0` (link index 7)
- **Correct:** Should use `tcp` (link index 9)
- **Difference:** **138mm gripper extension** not accounted for!

### Impact:

When IK solves for target Z = 0.10m:

- IK positions `tool0` at 0.10m ✅
- But actual gripper tip (`tcp`) is at 0.10 - 0.138 = **-0.038m** ❌
- **Robot drives gripper 38mm into the table!**

### Test Results:

```
End Effector Link    | X (m)   | Y (m)   | Z (m)    | Note
---------------------|---------|---------|----------|------------------
tool0 (index 7)      | 0.39281 | 0.05295 |  0.05984 | ✅ Above table
robotiq_base (idx 8) | 0.39281 | 0.05295 |  0.05984 | Same as tool0
tcp (index 9) ⭐      | 0.39275 | 0.05292 | -0.07816 | ❌ 78mm BELOW table!
```

**Gripper extension: 138mm downward from tool0 to tcp**

---

## ✅ Fix Applied

### Changed End Effector Link Selection:

**Priority order:**

1. **`tcp`** ⭐ (gripper tip - HIGHEST PRIORITY)
2. `ee_link` (if tcp not found)
3. `flange` (if neither found)
4. `tool0` (last resort, with warning)

**Code change in `kinematics_solver.py`:**

```python
# OLD (wrong priority):
if link_name in ('tcp', 'tool0', 'ee_link', 'flange'):
    self.end_effector_link_index = i
    break  # Takes first match (tool0 came before tcp!)

# NEW (correct priority):
if link_name == 'tcp':  # MUST find tcp first!
    self.end_effector_link_index = i
    logger.info(f"✅ Found TCP (gripper tip) at link index {i}")
    break
elif not ee_found and link_name in ('ee_link', 'flange'):
    # Secondary options
    ...
```

---

## 📊 Expected Results After Fix:

### Before:

```
Target Z: 0.10m
tool0 positioned at: 0.10m
tcp (actual gripper) at: -0.04m ❌ BELOW TABLE!
```

### After:

```
Target Z: 0.10m
tcp (gripper tip) positioned at: 0.10m ✅
tool0 positioned at: 0.24m (0.10 + 0.138 gripper)
```

---

## ⚠️ Additional Issue: Grasp Angle Offset

### Current Setting:

```python
'grasp_angle_offset_rad': 1.5708,   # 90° offset (π/2) for GGCNN2
```

**Question:** You mentioned "we need to add 90 degrees to our grasp for the robot"

**Options:**

1. **Offset already exists** (1.5708 rad = 90°) - Is this correct or wrong?
2. **Need ADDITIONAL 90°** → Change to `3.1416` (180°)?
3. **Need to REMOVE 90°** → Change to `0.0`?
4. **Different offset needed** → Specify value?

### Where It's Applied:

The offset is added in two places:

1. `grasp_2d_to_3d_pose()` - When converting 2D pixel grasp to 3D camera frame
2. `compose_grasp_orientation()` - When creating rotation matrix

---

## 🧪 Testing

To verify the fix:

```bash
cd src
..\venv\Scripts\python.exe main_debug.py mock

# Run grasping state
run 6

# Check logs for:
✅ Found TCP (gripper tip) at link index 9
✅ Grasp Z should now be ABOVE table
```

---

## ✅ Summary

### Root Cause:

- IK used `tool0` instead of `tcp`
- Missing 138mm gripper extension
- Robot commanded positions were 138mm too high
- Gripper ended up below table

### Fix:

- Changed end effector link priority to find `tcp` first
- Now accounts for full gripper length
- IK solutions will be 138mm higher to compensate

### Next Steps:

1. Test with corrected TCP link
2. Clarify grasp angle offset requirement (90°, 180°, or 0°?)
3. Verify robot no longer goes below table

---

**Date:** October 12, 2025  
**Status:** ✅ Critical fix applied  
**Impact:** HIGH - prevents robot from hitting table
