# Pickup Task Sequencer - How It Works

## 🎯 Overview

The Pickup Task Sequencer orchestrates a multi-step pickup operation by creating and queuing a sequence of states.

---

## 📊 The Flow

### 1. Sequence Creation (`_create_pickup_sequence`)

Creates a list of states to execute in order:

```python
states = [
    MoveToState(context, target_location=PICKUP_LOCATION["position"]),  # 1
    GraspingState(context),                                              # 2
    GripperControlState(context, action='open'),                         # 3
    MoveToState(context, pose_from_telemetry='generated_approach_pose'), # 4
    MoveToState(context, pose_from_telemetry='generated_grasp_pose'),    # 5
    GripperControlState(context, action='close'),                        # 6
    MoveToState(context, pose_from_telemetry='generated_approach_pose')  # 7
]
```

### 2. Step-by-Step Execution

**Step 1: Move to Pre-Pickup**

- Direct location: `PICKUP_LOCATION["position"]` (e.g., [0.39, 0.06, 0.25])
- Moves robot to viewing position above the object

**Step 2: Generate Grasp (GraspingState)**

- Captures camera image
- Runs grasp detection (GR-ConvNet)
- **Stores TWO poses in telemetry:**
  - `generated_grasp_pose` - [x, y, z, rx, ry, rz] - actual grasp position
  - `generated_approach_pose` - [x, y, z, rx, ry, rz] - position above grasp

**Step 3: Open Gripper**

- Opens gripper to prepare for grasping

**Step 4: Move to Approach Pose**

- **Uses string key**: `pose_from_telemetry='generated_approach_pose'`
- **Z offset applied**: +0.2m (20cm) above the grasp
- MoveToState looks up the pose from telemetry

**Step 5: Move to Grasp Pose**

- **Uses string key**: `pose_from_telemetry='generated_grasp_pose'`
- Moves to final grasping position

**Step 6: Close Gripper**

- Closes gripper to grasp object

**Step 7: Lift Object**

- Returns to approach pose (20cm above grasp)
- Lifts the object

---

## 🔍 How String Keys Work

### The Mechanism:

1. **Storage** (GraspingState):

```python
# GraspingState generates poses and stores them:
context.telemetry.set_generated_grasp_pose([x, y, z, rx, ry, rz])
context.telemetry.set_generated_approach_pose([x, y, z, rx, ry, rz])
```

2. **Reference** (PickupTaskSequencer):

```python
# Create MoveToState with a string key:
MoveToState(context, pose_from_telemetry='generated_approach_pose')
```

3. **Retrieval** (MoveToState):

```python
# MoveToState._get_pose_from_telemetry() looks up the key:
if self.pose_from_telemetry == 'generated_approach_pose':
    return self.context.telemetry.get_generated_approach_pose()
```

**Result**: MoveToState gets the actual pose [x, y, z, rx, ry, rz] and moves to it!

---

## 📐 Z Offset Mechanism

### Purpose:

Add a safety margin above the grasp position to prevent collisions during approach.

### Implementation:

**In PickupTaskSequencer:**

```python
self.z_offset = 0.2  # 20cm above grasp

# Before creating the MoveToState for approach:
self._apply_z_offset_to_telemetry_pose(context, 'generated_approach_pose', self.z_offset)
```

**What it does:**

1. Gets current approach pose from telemetry
2. Adds 0.2m to the Z coordinate (lifts it up)
3. Stores the modified pose back in telemetry

**Example:**

```python
# Original approach pose from GraspingState:
[0.40, 0.05, 0.15, 0.0, 0.0, -1.57]

# After applying +0.2m Z offset:
[0.40, 0.05, 0.35, 0.0, 0.0, -1.57]  # Z changed from 0.15 to 0.35
       │        ↑
       │        └─ Z increased by 0.2m
       └─ X, Y unchanged
```

---

## 🎯 Why This Design?

### Benefits:

1. **Decoupling**

   - GraspingState doesn't need to know about approach offsets
   - PickupTaskSequencer controls the sequence logic
   - MoveToState is generic and reusable

2. **Flexibility**

   - Can modify poses between generation and execution
   - Easy to adjust offsets without changing GraspingState
   - Multiple states can reference the same pose

3. **Clarity**
   - Each state has a single responsibility
   - Sequence is easy to read and modify
   - Clear data flow through telemetry

---

## 🔧 Current Configuration

### Z Offset:

```python
self.z_offset = 0.2  # 20cm (200mm) above grasp
```

### Applied to:

- **Approach pose** (step 4 and step 7) - 20cm above grasp position
- **Grasp pose** (step 5) - no offset, directly at detected grasp

---

## 🐛 Common Issues & Solutions

### Issue 1: "Telemetry object has no attribute 'get_pose'"

**Cause**: Using wrong method name  
**Fix**: Use specific getters:

- `get_generated_grasp_pose()`
- `get_generated_approach_pose()`
- NOT `get_pose(key)` ❌

### Issue 2: "Target location: (0.0, 0.0, 0.0)"

**Cause**: Pose not set in telemetry before MoveToState executes  
**Fix**: Ensure GraspingState completes before approach/grasp moves

### Issue 3: Robot moves to wrong position

**Cause**: Z offset not applied or applied incorrectly  
**Fix**: Check `_apply_z_offset_to_telemetry_pose` is called before MoveToState

### Issue 4: Joint limit violations

**Cause**: Generated pose is unreachable or outside workspace  
**Fix**:

- Verify grasp is within robot reach
- Check if Z offset makes position unreachable
- Adjust `PICKUP_LOCATION` viewing position

---

## 📝 Quick Reference

### Telemetry Pose Methods:

**Getters:**

- `context.telemetry.get_generated_grasp_pose()` → [x,y,z,rx,ry,rz]
- `context.telemetry.get_generated_approach_pose()` → [x,y,z,rx,ry,rz]

**Setters:**

- `context.telemetry.set_generated_grasp_pose([x,y,z,rx,ry,rz])`
- `context.telemetry.set_generated_approach_pose([x,y,z,rx,ry,rz])`

### MoveToState Usage:

**Direct position:**

```python
MoveToState(context, target_location=(x, y, z))
```

**From telemetry:**

```python
MoveToState(context, pose_from_telemetry='generated_approach_pose')
```

---

## ✅ Summary

The Pickup Task Sequencer is like a **recipe**:

1. **Recipe steps** = List of states
2. **Ingredients** = Poses stored in telemetry
3. **String keys** = References to ingredients
4. **Z offset** = Adjustments to ingredients before use

Each state executes in order, and states can share data through telemetry using string keys!

---

**Date:** October 12, 2025  
**Status:** ✅ Explained & Fixed  
**Z Offset:** 0.2m (20cm) above grasp
