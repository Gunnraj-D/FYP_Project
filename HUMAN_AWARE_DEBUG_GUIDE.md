# Human-Aware Move State - Debug Guide

## ✅ Integration Complete!

The `HumanAwareMoveToState` is now fully integrated into `main_debug.py` and ready to use!

---

## 🚀 How to Run

### 1. **Start Unity with ZED Tracking**

First, start your Unity application with the ZED body tracking and joint sender:

```
# Unity should be sending skeleton data to 127.0.0.1:5005
```

### 2. **Start main_debug.py**

```bash
cd src
python main_debug.py mock    # For mock OPC UA mode
# or
python main_debug.py real    # For real robot
```

### 3. **Check ZED Connection**

When the system starts, you'll see:

```
✅ ZED Joint Receiver started (waiting for Unity connection)
```

Once Unity connects:

```
# In Unity console (you should see)
Connected to Python TCP server.

# In Python (the receiver logs internally when connected)
```

### 4. **View Available States**

```
🤖 Debug> states

📋 Available States:
  1. MoveToState (simple move to pos 1)
  2. MoveToState (simple move to pos 2)
  3. GripperControlState (open)
  4. GripperControlState (close)
  5. UnifiedHandTrackingState
  6. GraspingState
  7. HumanAwareMoveToState (to pos 1 with collision avoidance)  ← NEW!
  8. HumanAwareMoveToState (to pos 2 with collision avoidance)  ← NEW!
```

### 5. **Run Human-Aware State**

```
🤖 Debug> run 7

🚀 Executing: HumanAwareMoveToState
Entering HumanAwareMoveToState
Target: pos=[0.3, 0.415, 0.6], orientation=specified
Path planner initialized
Human detected - planning collision-free trajectory
Initial plan successful: 50 waypoints, min clearance: 0.412m, planning time: 0.234s
```

---

## 📊 ZED Data Polling Mechanism

### How It Works

```
Unity (ZED SDK)              Python                      State Machine
     |                          |                              |
     |-- Update() @ 30-60Hz     |                              |
     |   (ZED tracking runs)    |                              |
     |                          |                              |
     |-- Send skeleton JSON --> |                              |
     |   via TCP @ ~30-60Hz     |                              |
     |                       [TCP Receiver]                     |
     |                       (background thread)                |
     |                          |                              |
     |                       Store latest                       |
     |                       frame in buffer                    |
     |                       (thread-safe)                      |
     |                          |                              |
     |                          | <-- get_latest_frame() ---- |
     |                          |      @ 50Hz (every execute) |
     |                          | --- return cached frame --> |
     |                          |                              |
```

### Key Points

1. **Unity Send Rate**: ~30-60 Hz (depends on Unity's `Update()` frequency)

   - ZED SDK tracks at 15-30 FPS typically
   - Unity sends every frame via TCP

2. **Python Reception**: Continuous background thread

   - Always listening for TCP data
   - Parses JSON and updates `latest_frame` buffer
   - Thread-safe with lock

3. **State Polling**: 50 Hz (every `execute()` call in state machine)

   - State calls `zed_receiver.get_latest_frame()`
   - **No blocking** - just returns cached frame
   - **No overhead** - simple memory read with lock

4. **Data Freshness**:
   - At 50Hz control loop and 30Hz ZED, typical age: **16-33ms**
   - Still fresh enough for reactive planning
   - Can add latency compensation if needed

### Code Flow

```python
# In ZEDJointReceiver (background thread)
def _handle_client(self):
    while self.running:
        data = self.client_socket.recv(self.buffer_size)  # Blocking recv
        self.message_buffer += data.decode('utf-8')
        self._process_buffer()  # Parse JSON

def _parse_and_store_frame(self, json_string):
    frame_data = self._convert_to_frame_data(raw_data)
    with self.frame_lock:
        self.latest_frame = frame_data  # Update buffer (atomic)
        self.total_frames_received += 1

# In HumanAwareMoveToState (50Hz execute loop)
def execute(self):
    # Check if replanning needed
    if self._should_replan():
        frame_data = self.context.zed_receiver.get_latest_frame()  # Fast!
        # ... replan with latest skeleton data

def _should_replan(self):
    frame_data = self.context.zed_receiver.get_latest_frame()  # Fast!
    if frame_data and frame_data.skeletons:
        skeleton = frame_data.skeletons[0]
        torso_pos = skeleton.get_joint_position('PELVIS')
        # Check if human moved > threshold
```

---

## 🎛️ Usage Example Session

```bash
$ python main_debug.py mock

🚀 Starting mock OPC UA server...
✅ Mock server ready
🚀 Initializing Debug Robot Control System in 'mock' mode...
✅ ZED Joint Receiver started (waiting for Unity connection)
✅ Debug system initialized successfully
🚀 Starting debug robot control system...
✅ OPC client initialized
✅ Debug system started successfully

==================================================
🤖 ROBOT DEBUG MODE
==================================================
OPC Mode: mock
✅ Mock OPC UA server running
Camera Transform Mode: simple

Available commands:
  states - Show available states
  run <number> - Execute state by number
  status - Show system status
  quit - Exit program

🤖 Debug> states

📋 Available States:
  1. MoveToState
  2. MoveToState
  3. GripperControlState
  4. GripperControlState
  5. UnifiedHandTrackingState
  6. GraspingState
  7. HumanAwareMoveToState  ← Human-aware path planning
  8. HumanAwareMoveToState

🤖 Debug> run 7

🚀 Executing: HumanAwareMoveToState
Entering HumanAwareMoveToState
Target: pos=[0.3, 0.415, 0.6], orientation=specified
Path planner initialized
Human detected - planning collision-free trajectory
Initial plan successful: 50 waypoints, min clearance: 0.412m, planning time: 0.234s

Waypoint 0/50, clearance: 0.412m, speed: 100%
Waypoint 10/50, clearance: 0.385m, speed: 85%
Human moved 0.112m (threshold: 0.100m)
Triggering replan due to human motion
Replan successful: 45 waypoints, min clearance: 0.356m, planning time: 0.089s
Waypoint 20/50, clearance: 0.298m, speed: 50%
Waypoint 30/50, clearance: 0.421m, speed: 100%
Waypoint 40/50, clearance: 0.445m, speed: 100%

Motion to target complete!
  Final clearance: 0.445m
  Replans: 1
Exiting HumanAwareMoveToState
Motion statistics:
  Replans: 1
  Total planning time: 0.323s
  Final clearance: 0.445m
✅ Completed: HumanAwareMoveToState

🤖 Debug> quit
```

---

## 🔍 Monitoring ZED Connection

### Check if Unity is Connected

```python
🤖 Debug> status

📊 System Status: IDLE
🔄 Execution Active: False

# Check ZED connection in your code
if manager.zed_receiver and manager.zed_receiver.is_connected():
    stats = manager.zed_receiver.get_stats()
    print(f"ZED connected: {stats}")
    # {'connected': True, 'total_frames': 1234, 'latest_frame_number': 1234}
else:
    print("ZED not connected - waiting for Unity")
```

### Verify Data is Flowing

Add to your state execution:

```python
frame_data = self.context.zed_receiver.get_latest_frame()
if frame_data:
    print(f"Receiving ZED data: Frame {frame_data.frame}, "
          f"{len(frame_data.skeletons)} skeleton(s)")
else:
    print("No ZED data yet")
```

---

## ⚠️ Troubleshooting

### ZED Receiver Not Starting

```
⚠️ ZED receiver not started: [Errno 10048] Only one usage of each socket address...
```

**Solution**: Port 5005 already in use. Kill other Python processes or change port in both Unity and Python.

### Unity Not Connecting

```
✅ ZED Joint Receiver started (waiting for Unity connection)
# But Unity shows: TCP connection failed: Connection refused
```

**Solution**:

1. Check firewall settings (allow port 5005)
2. Verify IP address (use 127.0.0.1 for localhost)
3. Start Python receiver BEFORE Unity connects

### No Human Detected

```
No human detected - planning direct trajectory
```

**Solution**:

1. Make sure person is in ZED camera view
2. Check Unity console for skeleton tracking status
3. Verify ZED SDK is properly initialized in Unity

### State 7/8 Not Available

```
❌ Invalid state number: 7
```

**Solution**: ZED receiver failed to start. Check logs for initialization errors.

---

## 📈 Performance Metrics

### Expected Performance

| Metric                  | Value    | Notes                       |
| ----------------------- | -------- | --------------------------- |
| **ZED Send Rate**       | 30-60 Hz | Unity Update() frequency    |
| **Python Receive Rate** | 30-60 Hz | Matches Unity send rate     |
| **State Poll Rate**     | 50 Hz    | Main control loop frequency |
| **Data Latency**        | 16-33 ms | Time from ZED → State       |
| **Initial Planning**    | < 500 ms | First trajectory generation |
| **Replanning**          | < 100 ms | Dynamic replanning          |

### Actual Data Flow

```
Time (ms)   Unity                  Python Receiver         State Machine
0           Capture skeleton
16          Send JSON -->
17                                 Receive & parse
17                                 Update latest_frame
20                                                         Poll @ 50Hz
20                                                         Use frame from t=17
40                                                         Poll @ 50Hz
                                                           (same frame if no new data)
50          Capture skeleton
66          Send JSON -->
67                                 Receive & parse
67                                 Update latest_frame
80                                                         Poll @ 50Hz
80                                                         Use frame from t=67
```

**Key Insight**: State always gets the most recent frame available, typically 20-30ms old.

---

## 🎯 Advanced: Polling Frequency Tuning

If you want to optimize polling:

### Option 1: Poll Only When Needed (Current)

```python
def execute(self):
    # Only poll when checking for replanning
    if self._should_replan():  # Checks every 0.5s minimum
        frame_data = self.context.zed_receiver.get_latest_frame()
```

**Pros**: Minimal overhead
**Cons**: Might miss rapid movements

### Option 2: Poll Every Execute

```python
def execute(self):
    # Always get latest (50Hz polling)
    frame_data = self.context.zed_receiver.get_latest_frame()
    # ... use it
```

**Pros**: Most reactive
**Cons**: Tiny overhead (just a dict lookup with lock)

### Option 3: Event-Based (Future)

```python
# Register callback with receiver
receiver.set_callback(self.on_skeleton_update)

def on_skeleton_update(self, frame_data):
    # Called automatically when new data arrives
    self.check_if_replan_needed(frame_data)
```

**Pros**: Zero polling overhead, most reactive
**Cons**: More complex threading

**Current implementation uses Option 1 - good balance of performance and simplicity!**

---

## 📝 Summary

### ZED Polling Architecture

✅ **Non-blocking**: `get_latest_frame()` is instant (cached value)
✅ **Thread-safe**: Lock protects shared frame buffer
✅ **Real-time**: Latest data always available
✅ **Low latency**: 16-33ms typical age
✅ **Efficient**: No polling overhead, just memory read

### Integration Status

✅ ZED receiver automatically starts with `main_debug.py`
✅ States 7 & 8 are human-aware versions of states 1 & 2
✅ Graceful fallback if ZED not connected
✅ Clean shutdown on exit

### Next Steps

1. Start Unity with ZED tracking
2. Run `python main_debug.py mock`
3. Execute `run 7` or `run 8`
4. Watch the robot avoid humans in real-time!

---

**You're all set! The system is ready for human-aware path planning!** 🚀

