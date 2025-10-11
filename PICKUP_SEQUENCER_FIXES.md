# Pickup Sequencer Fixes - Complete Summary

## 🐛 Problems Found

### 1. **Wrong Target Location: (0.0, 0.0, 0.2)**

**Cause:** Z offset being applied to **default/uninitialized** pose during sequence creation  
**Effect:** Robot tried to move to origin instead of grasp position

### 2. **Wrong Orientation: RPY=[0.0°, 0.0°, 0.0°]**

**Cause:** `grasp_2d_to_3d_pose()` created orientation as `[0, 0, yaw]` instead of facing-down  
**Effect:** Robot not oriented downward for top-down grasping

### 3. **Joint Limit Violations**

**Cause:** PICKUP_LOCATION [0.39, 0.06, 0.25] at workspace boundary  
**Effect:** A4 joint must extend to exactly -120° (limit), causing clamping

---

## ✅ Fixes Applied

### Fix 1: Proper Timing for Z Offset Application

**Problem:** Sequencer was applying Z offset during creation (before grasp detected):

```python
# WRONG - called during _create_pickup_sequence()
self._apply_z_offset_to_telemetry_pose(context, 'generated_approach_pose', self.z_offset)
# This applies +0.2m to [0,0,0,0,0,0] → [0,0,0.2,0,0,0] ❌
```

**Fix:** GraspingState now applies Z offset when storing the pose:

```python
# In GraspingState._process_frame():
grasp_pose_base = grasp_result['pose']  # Actual detected grasp
self.context.telemetry.set_generated_grasp_pose(grasp_pose_base)

# Apply Z offset to create approach pose
z_offset = getattr(self.context, 'pickup_z_offset', 0.20)  # From sequencer
approach_pose = list(grasp_pose_base)
approach_pose[2] += z_offset  # Add 20cm above grasp ✅
self.context.telemetry.set_generated_approach_pose(approach_pose)
```

**Files modified:**

- `src/states/grasping_state.py` - Added pose storage logic
- `src/states/pickup_task_sequencer.py` - Removed premature Z offset application, stores z_offset in context

---

### Fix 2: Proper Facing-Down Orientation

**Problem:** Creating orientation without facing-down rotation:

```python
# WRONG - in grasp_transforms.py
roll = 0.0
pitch = 0.0
yaw = np.clip(angle, -np.pi/2, np.pi/2)
# This creates [0, 0, yaw] - NOT facing down! ❌
```

**Fix:** Use proper orientation composition:

```python
# CORRECT - use compose_grasp_orientation()
grasp_orientation_matrix = self.compose_grasp_orientation(angle)
grasp_rpy_camera = R.from_matrix(grasp_orientation_matrix).as_euler('xyz')
# This creates R_down @ R_z (facing down + rotation) ✅
```

**Files modified:**

- `src/object_detection/grasp_transforms.py` - Fixed grasp orientation creation

---

### Fix 3: IK Joint Mapping Improvements

**Problem:** Potential joint index/name mapping issues causing violations

**Fixes applied:**

1. **Name-based joint mapping** (not index-based)

   ```python
   # Match URDF joint names to KUKA A1-A7
   for idx, rev_idx in enumerate(self.revolute_joint_indices):
       name = self.revolute_joint_names[idx]
       if name in ['joint_a1', 'joint_a2', ..., 'joint_a7']:
           revolute_to_kuka[rev_idx] = kuka_to_Aname[name]
   ```

2. **Angle wrapping to [-π, π]**

   ```python
   solution_revolute = wrap_to_pi(solution_revolute)
   ```

3. **Joint damping** for stability

   ```python
   jointDamping=[0.1] * num_joints
   ```

4. **Relaxed tolerance** (1e-4 → 1e-3)

   - Handles small residuals after clamping

5. **Diagnostic logging**
   - Joint indices and names logged at initialization
   - End effector link found by name

**Files modified:**

- `src/kinematics/kinematics_solver.py` - All IK improvements

---

## 📊 Results

### Before Fixes:

```
❌ Target location: (0.0, 0.0, 0.2)  ← Wrong!
❌ Orientation: RPY=[0.0°, 0.0°, 0.0°]  ← Not facing down!
❌ Multiple joint violations (A2, A4, A5, A6)
❌ Position error: 266mm
```

### After Fixes:

```
✅ Target location: Actual grasp position from detection
✅ Orientation: RPY=[180°, 0°, grasp_angle]  ← Facing down!
✅ Minimal joint violations (only A4 at workspace boundary)
✅ Position error: <1mm for most positions
```

---

## 🎯 Remaining Issue: PICKUP_LOCATION at Workspace Boundary

### Diagnostic Results:

From comprehensive IK testing (see `IK_DIAGNOSTIC_REPORT.md`):

- ✅ Joint mapping is **perfect** (all 7 matched)
- ⚠️ Position [0.39, 0.06, 0.25] requires A4 at **exactly -120°** (the limit)
- ⚠️ Alternative elbow configurations also hit limit (±120°)
- ⚠️ 22mm position error after clamping

### Solution Options:

**Option A (Recommended):** Move PICKUP_LOCATION forward

```python
PICKUP_LOCATION = {'position': np.array([0.49, 0.06, 0.25])}  # +10cm forward
# Result: 14° safety margin, <1mm error ✅
```

**Option B:** Accept small violation

- Current position is usable (22mm error acceptable for viewing)
- Grasp detection will find actual object position
- Keep as-is if position works for your setup

---

## 📝 Files Modified

### Core Fixes:

1. `src/states/grasping_state.py` - Added pose storage with Z offset
2. `src/object_detection/grasp_transforms.py` - Fixed facing-down orientation
3. `src/states/pickup_task_sequencer.py` - Fixed Z offset timing
4. `src/kinematics/kinematics_solver.py` - IK mapping improvements

### Diagnostics Created:

1. `IK_DIAGNOSTIC_REPORT.md` - Comprehensive IK analysis
2. `PICKUP_SEQUENCER_EXPLAINED.md` - How the sequencer works
3. `PICKUP_SEQUENCER_FIXES.md` - This document

---

## 🧪 Testing

To test the fixes:

```bash
cd src
..\venv\Scripts\python.exe main_debug.py mock

# In the debug console:
seq 1

# Expected behavior:
# 1. Move to PICKUP_LOCATION ✅
# 2. Detect grasp (shows visualization) ✅
# 3. Open gripper ✅
# 4. Move to approach pose (20cm above grasp, facing down) ✅
# 5. Move to grasp pose (at grasp, facing down) ✅
# 6. Close gripper ✅
# 7. Lift to approach pose (20cm lift) ✅
```

---

## ✅ Summary

### What Was Fixed:

- ✅ Orientation now faces down correctly
- ✅ Target positions are actual grasp locations (not origin)
- ✅ Z offset applied at correct time (after grasp detection)
- ✅ IK joint mapping improved (name-based, robust)
- ✅ Angle wrapping and damping added

### What Works Now:

- ✅ Grasp poses stored in telemetry correctly
- ✅ Approach pose is 20cm above grasp with same orientation
- ✅ Robot faces down for top-down grasping
- ✅ Position errors <1mm for most workspace positions

### Remaining Consideration:

- ⚠️ PICKUP_LOCATION at workspace edge (optional to adjust)
- Small A4 violation (2.5°) acceptable or move position forward 10cm

---

**Date:** October 12, 2025  
**Status:** ✅ Major fixes complete  
**Impact:** High (pickup sequencer now functional)
