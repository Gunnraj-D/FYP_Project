"""
Quick diagnostic script to check ZED skeleton data quality.
Run this to see what data is actually being received.
"""
import time
import logging
from hand_detection.zed_joint_receiver import ZEDJointReceiver

logging.basicConfig(level=logging.INFO)

print("=" * 60)
print("ZED Data Quality Test")
print("=" * 60)
print("Make sure Unity with ZED is running and person is visible!")
print("=" * 60)

receiver = ZEDJointReceiver(host='127.0.0.1', port=5005)
receiver.start()

print("\nWaiting for connection...")
time.sleep(2)

if receiver.is_connected():
    print("✅ Unity connected!")
else:
    print("❌ Unity NOT connected - start Unity first!")
    receiver.stop()
    exit(1)

print("\nChecking skeleton data for 10 seconds...")
print("=" * 60)

for i in range(100):  # 10 seconds at 10Hz
    frame_data = receiver.get_latest_frame()

    if frame_data is None:
        print(f"[{i}] ❌ No frame data yet")
        time.sleep(0.1)
        continue

    print(
        f"\n[Frame {frame_data.frame}] Skeletons: {len(frame_data.skeletons)}")

    if len(frame_data.skeletons) == 0:
        print("  ❌ No person detected!")
    else:
        skeleton = frame_data.skeletons[0]
        print(f"  ✅ Person detected (ID: {skeleton.skeleton_id})")
        print(f"  Total joints: {len(skeleton.joints)}")

        # Check critical joints
        critical_joints = ['PELVIS', 'NECK', 'NOSE', 'RIGHT_WRIST', 'LEFT_WRIST',
                           'RIGHT_SHOULDER', 'LEFT_SHOULDER', 'CHEST_SPINE', 'SPINE_2']

        print(f"\n  Critical joints:")
        for joint_name in critical_joints:
            pos = skeleton.get_joint_position(joint_name)
            if pos:
                print(
                    f"    ✅ {joint_name:20s}: [{pos[0]:7.3f}, {pos[1]:7.3f}, {pos[2]:7.3f}]")
            else:
                print(f"    ❌ {joint_name:20s}: NOT FOUND")

        # Check if positions are reasonable for robot workspace
        right_wrist = skeleton.get_joint_position('RIGHT_WRIST')
        if right_wrist:
            x, y, z = right_wrist
            print(f"\n  Right wrist analysis:")
            print(f"    Position: [{x:.3f}, {y:.3f}, {z:.3f}]")

            # Robot workspace is roughly:
            # X: -1.0 to 1.0m, Y: -1.0 to 1.0m, Z: 0.0 to 1.5m
            in_workspace = (
                -1.0 <= x <= 1.0 and
                -1.0 <= y <= 1.0 and
                0.0 <= z <= 1.5
            )

            if in_workspace:
                print(f"    ✅ IN robot workspace")
                target = [x, y, z + 0.3]
                print(
                    f"    Target (30cm above): [{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}]")
            else:
                print(f"    ❌ OUTSIDE robot workspace!")
                print(
                    f"       Expected: X[-1.0, 1.0], Y[-1.0, 1.0], Z[0.0, 1.5]")
                if abs(x) > 2.0 or abs(y) > 2.0 or abs(z) > 3.0:
                    print(f"    🚨 Coordinates look WRONG - may need transform!")

    time.sleep(0.1)

print("\n" + "=" * 60)
print("Test complete!")
stats = receiver.get_stats()
print(f"Stats: {stats}")
print("=" * 60)

receiver.stop()


