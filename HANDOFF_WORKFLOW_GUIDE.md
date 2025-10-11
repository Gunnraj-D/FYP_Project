# Human Handoff Workflow Guide

## 🎯 Understanding the Different States

You now have **3 types** of movement states, each for different purposes:

---

## State Comparison

| State #  | State Name                  | Target         | Human Aware? | Use Case                                     |
| -------- | --------------------------- | -------------- | ------------ | -------------------------------------------- |
| **1-2**  | `MoveToState`               | Fixed position | ❌ No        | Fast direct movement when workspace is clear |
| **7-8**  | `HumanAwareMoveToState`     | Fixed position | ✅ Yes       | Move to fixed position while avoiding humans |
| **9-10** | `HumanHandoffApproachState` | **Human hand** | ✅ Yes       | **Approach human hand for handoff**          |

---

## 📋 All Available States

```bash
🤖 Debug> states

📋 Available States:
  1. MoveToState → [0.3, 0.415, 0.6] (simple)
  2. MoveToState → PICKUP_LOCATION (simple)
  3. GripperControlState (open)
  4. GripperControlState (close)
  5. UnifiedHandTrackingState
  6. GraspingState
  7. HumanAwareMoveToState → [0.3, 0.415, 0.6] (avoid humans)
  8. HumanAwareMoveToState → PICKUP_LOCATION (avoid humans)
  9. HumanHandoffApproachState → RIGHT hand + 30cm above ⭐ NEW!
  10. HumanHandoffApproachState → LEFT hand + 30cm above ⭐ NEW!
```

---

## 🤝 Human Handoff Workflow

Here's the complete workflow for handing an object to a human:

### **Phase 1: Approach Hand (State 9)**

```python
🤖 Debug> run 9  # HumanHandoffApproachState (RIGHT hand)
```

**What it does**:

1. ✅ Gets RIGHT_WRIST position from ZED skeleton
2. ✅ Calculates target = hand_position + [0, 0, 0.3] (30cm above)
3. ✅ Plans collision-free path avoiding human body
4. ✅ **Dynamically updates** as hand moves
5. ✅ Applies speed scaling near human
6. ✅ Completes when TCP is ~30cm above hand (±5cm threshold)

**Example Output**:

```
Entering HumanHandoffApproachState
Tracking RIGHT_WRIST with offset [0.0, 0.0, 0.3]
Initial hand position: [0.45, 0.23, 0.65]
Initial target (approach) position: [0.45, 0.23, 0.95]
Path planner initialized
Initial plan successful: 48 waypoints, target at [0.45, 0.23, 0.95]

Waypoint 10/48, distance to target: 0.245m, clearance: 0.412m, speed: 100%
Hand moved 0.082m - Replanning to new position
Replan successful: 35 waypoints, target at [0.48, 0.25, 0.98]
Waypoint 30/48, distance to target: 0.034m, clearance: 0.356m, speed: 70%

Handoff approach complete!
  Final hand position: [0.48, 0.25, 0.68]
  Final TCP position: [0.48, 0.25, 0.98]
  Replans: 2
```

### **Phase 2: Fine Tracking (State 5)**

```python
🤖 Debug> run 5  # UnifiedHandTrackingState
```

**What it does**:

1. ✅ Uses MediaPipe/camera for precise hand tracking
2. ✅ Follows hand movements in real-time
3. ✅ Maintains handoff position/orientation
4. ✅ Ready for object transfer

### **Phase 3: Handoff**

```python
# Open gripper to release object
🤖 Debug> run 3  # GripperControlState (open)
```

---

## 🆚 When to Use Each State

### Use `MoveToState` (States 1-2)

- ✅ Workspace is clear (no humans)
- ✅ Moving to known fixed positions
- ✅ Speed is critical
- ✅ Example: Moving to home position, moving between storage locations

### Use `HumanAwareMoveToState` (States 7-8)

- ✅ Human is in workspace
- ✅ Target is a **fixed position** (not the human)
- ✅ Need to avoid human while going somewhere else
- ✅ Example: Human observing while robot moves to grasp location

### Use `HumanHandoffApproachState` (States 9-10) ⭐

- ✅ **Target IS the human hand**
- ✅ Hand position is dynamic (human moves)
- ✅ Need to approach hand safely
- ✅ Example: **Handing object to human**, **receiving object from human**

---

## 🔄 Complete Handoff Sequence Example

### Manual Execution

```bash
# 1. Pick up object (existing workflow)
🤖 Debug> run 6  # GraspingState

# 2. Approach human's right hand (30cm above)
🤖 Debug> run 9  # HumanHandoffApproachState (RIGHT)
# Robot plans path toward hand while avoiding body
# Completes when ~30cm above hand

# 3. Switch to fine hand tracking
🤖 Debug> run 5  # UnifiedHandTrackingState
# Precise tracking, maintains position relative to hand

# 4. Release object
🤖 Debug> run 3  # Open gripper
```

### Future: Automated Sequencer

```python
class HandoffTaskSequencer:
    def __init__(self, hand='right'):
        self.sequence = [
            GraspingState(),                    # Pick up object
            HumanHandoffApproachState(hand),    # Approach hand
            UnifiedHandTrackingState(),         # Fine tracking
            GripperControlState('open'),        # Release
            # Optional: Retreat state
        ]
```

---

## 🎛️ Configuration Options

### Approach Offset (how far above/around hand)

```python
# In main_debug.py state creation
HumanHandoffApproachState(
    context=self.context,
    approach_offset=[0.0, 0.0, 0.30],  # 30cm directly above
    hand_joint_name='RIGHT_WRIST'
)

# Variations:
approach_offset=[0.0, 0.0, 0.30]   # 30cm above (default)
approach_offset=[0.0, 0.0, 0.20]   # 20cm above (closer)
approach_offset=[0.0, 0.10, 0.30]  # 30cm above, 10cm to the side
approach_offset=[0.10, 0.0, 0.25]  # 25cm above, 10cm forward
```

### Position Threshold (when to complete)

```python
HumanHandoffApproachState(
    context=self.context,
    position_threshold=0.05  # Complete when within 5cm (default)
    # or
    position_threshold=0.03  # More precise (3cm)
)
```

### Hand Selection

```python
hand_joint_name='RIGHT_WRIST'  # Track right wrist (default)
hand_joint_name='LEFT_WRIST'   # Track left wrist
hand_joint_name='RIGHT_HAND'   # Track right hand center (alternative)
```

---

## 🔍 Key Differences

### HumanAwareMoveToState (States 7-8)

- **Target**: Fixed position you specify
- **Planning**: Once initially, replans if human moves
- **Use**: "Go to position X while avoiding human"
- **Example**: Move to grasp location with human nearby

### HumanHandoffApproachState (States 9-10) ⭐

- **Target**: **Human hand position** (dynamic!)
- **Planning**: Continuously updates as hand moves
- **Use**: "Follow and approach the human's hand"
- **Example**: Handoff, collaborative tasks

---

## 📊 How Hand Targeting Works

### Step-by-Step

```python
# Every execute() cycle (50 Hz):

1. Get latest ZED frame
   frame_data = zed_receiver.get_latest_frame()

2. Extract hand position
   skeleton = frame_data.skeletons[0]
   hand_pos = skeleton.get_joint_position('RIGHT_WRIST')
   # Returns: [x, y, z] in robot base frame (meters)
   # Example: [0.45, 0.23, 0.68]

3. Calculate approach target
   target = hand_pos + offset
   # With offset [0, 0, 0.3]:
   # target = [0.45, 0.23, 0.68] + [0, 0, 0.3]
   #        = [0.45, 0.23, 0.98]  ← 30cm above hand

4. Plan path to target
   # Plans collision-free trajectory
   # Avoids: head, torso, other arm, shoulders
   # Targets: position above hand

5. Execute trajectory
   # Moves toward hand
   # Replans if hand moves significantly

6. Check if arrived
   distance = ||current_tcp - target||
   if distance < 0.05m:
       complete!  # Ready for UnifiedHandTracking
```

---

## 🎯 Data Flow Diagram

```
ZED Unity
   ↓ (30-60 Hz)
ZEDJointReceiver (background thread)
   ↓ (stores latest frame)
HumanHandoffApproachState.execute() @ 50Hz
   ↓
Get RIGHT_WRIST position → [x, y, z]
   ↓
Calculate target = hand_pos + [0, 0, 0.3]
   ↓
Update target position (if hand moved)
   ↓
Check if replan needed
   ↓
Execute trajectory waypoint
   ↓
Check if within 5cm of target → Complete!
```

---

## 🧪 Testing the Handoff Workflow

### Test 1: Static Hand

```bash
# Have person hold hand still at waist height
🤖 Debug> run 9

Expected:
- Robot plans path toward hand
- Avoids torso/head/other arm
- Stops 30cm above hand
- Completes successfully
```

### Test 2: Moving Hand

```bash
# Have person slowly move hand around
🤖 Debug> run 9

Expected:
- Robot continuously replans as hand moves
- Target position updates
- Eventually reaches hand (when it stops moving)
- Speed slows near body
```

### Test 3: Full Handoff Sequence

```bash
# 1. Robot picks up object
🤖 Debug> run 6

# 2. Robot approaches hand
🤖 Debug> run 9
# Wait for completion (robot 30cm above hand)

# 3. Fine tracking
🤖 Debug> run 5
# Robot tracks small hand movements

# 4. Release object
🤖 Debug> run 3
```

---

## ⚙️ Advanced: Custom Approach Positions

You can create custom states for specific handoff scenarios:

```python
# Approach from the side (easier for human)
HumanHandoffApproachState(
    context=context,
    approach_offset=[0.15, 0.0, 0.20],  # 20cm above, 15cm to the side
    hand_joint_name='RIGHT_WRIST'
)

# Approach from front (better visibility)
HumanHandoffApproachState(
    context=context,
    approach_offset=[0.0, 0.15, 0.20],  # 20cm above, 15cm toward human
    hand_joint_name='RIGHT_WRIST'
)

# Lower approach (for seated person)
HumanHandoffApproachState(
    context=context,
    approach_offset=[0.0, 0.0, 0.15],  # Only 15cm above
    hand_joint_name='RIGHT_WRIST'
)
```

---

## 📝 Summary

### Why Two Human-Aware States?

**States 7-8** (`HumanAwareMoveToState`):

- Fixed targets: Position 1 and Position 2
- Used for A/B testing vs. simple MoveToState
- General collision avoidance

**States 9-10** (`HumanHandoffApproachState`): ⭐ **This is what you want!**

- **Dynamic target**: Tracks RIGHT_WRIST or LEFT_WRIST
- **For handoffs**: Approaches human hand
- **Follows movement**: Replans as hand moves
- **30cm offset**: Safe approach distance

### Complete Handoff Workflow

```
1. GraspingState (pick object)
   ↓
2. HumanHandoffApproachState (approach hand)  ← State 9 or 10
   ↓
3. UnifiedHandTrackingState (fine tracking)    ← State 5
   ↓
4. GripperControlState (release)               ← State 3
```

**State 9 is the key state you asked for** - it moves toward the human's hand while avoiding their body! 🎉

