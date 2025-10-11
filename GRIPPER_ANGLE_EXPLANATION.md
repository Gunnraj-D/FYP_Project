# Gripper Angle Convention - 90° Offset Explained

## 🎯 The Problem

**Visualization shows correct angle, but robot gripper is 90° off!**

This is a classic **gripper convention mismatch**.

---

## 📐 Two Different Conventions

### Convention 1: Jaw Axis (What Detector Predicts)

```
    finger          finger
      |              |
      |    object    |
      |______________|
           ↑
      Jaw axis (parallel to contact line)
```

The neural network predicts the angle of this **jaw axis** (line between contact points).

### Convention 2: Approach Axis (What Robot Might Use)

```
         ↓ Approach direction
         |
    _____|_____
   |     ↓     |
   |  gripper  |
   |___________|
```

Some robots use the **approach axis** (perpendicular to jaws).

**Difference: 90°!**

---

## 🔍 Where Each Is Used

### Grasp Detection & Visualization:

- Uses **jaw axis** convention
- Visualization arrow shows jaw orientation
- This is what you see in the image ✅

### Robot Execution:

- May need **approach axis** or different mounting
- Requires 90° conversion
- Applied by `grasp_angle_offset_rad`

---

## ✅ The Solution

### How It Works:

1. **Grasp detection** (no offset):

   ```python
   predicted_angle = neural_network.predict(...)  # Jaw axis
   # Visualization uses this directly ✅
   ```

2. **Robot execution** (offset applied):
   ```python
   # In compose_grasp_orientation():
   grasp_angle_rad += grasp_angle_offset_rad  # Add 90°
   R_z = R.from_euler('z', grasp_angle_rad).as_matrix()
   robot_orientation = R_down @ R_z  # ✅ Robot uses offset angle
   ```

### The Offset:

```python
'grasp_angle_offset_rad': 1.5708,  # 90° (π/2)
```

**Applied only when creating robot orientation, NOT during detection!**

---

## 🧪 Why This Is Correct

### Test Scenario:

**Detected grasp:**

- Object at 0° (horizontal)
- Visualization shows horizontal arrow ✅

**Without offset (grasp_angle_offset_rad = 0.0):**

- Robot gripper: Jaws horizontal
- **If gripper is 90° off** → jaws are vertical (wrong!) ❌

**With offset (grasp_angle_offset_rad = π/2):**

- Robot gripper: Jaws vertical
- Converted from horizontal (detected) + 90° (offset)
- **Now matches object orientation** ✅

---

## 🎯 Verification

### How To Tell If Offset Is Correct:

1. **Place horizontal object** (e.g., screwdriver handle at 0°)
2. **Visualization should show** horizontal grasp arrow
3. **Robot gripper should orient** to match object

**If gripper is perpendicular to object:**

- Need offset (currently ±90°)

**If gripper matches object:**

- No offset needed (set to 0°)

**If gripper is opposite:**

- Need 180° offset or flip sign

---

## 📝 Current Configuration

```python
# Config restored to:
'grasp_angle_offset_rad': 1.5708,  # 90° offset
```

**Why:** User reported gripper is 90° off from visualization.

**This should fix it!**

---

## 🔧 If Still Wrong After Testing

### If gripper is still 90° off:

Try **-90°** instead:

```python
'grasp_angle_offset_rad': -1.5708,  # -90° offset
```

### If gripper is 180° off:

```python
'grasp_angle_offset_rad': 3.1416,  # 180° offset
```

### If gripper is correct now:

```python
'grasp_angle_offset_rad': 1.5708,  # Keep at 90°
```

---

## ✅ Summary

**Offset Purpose:** Convert between detector convention and gripper convention

**Where Applied:** `compose_grasp_orientation()` - only for robot execution

**Not Applied To:** Visualization (shows raw detected angle)

**Current Value:** 1.5708 rad = 90°

**Test it and let me know if gripper now matches the visualization!**

---

**Date:** October 12, 2025  
**Status:** ✅ Offset restored to 90°  
**Next:** Test if gripper orientation is now correct
