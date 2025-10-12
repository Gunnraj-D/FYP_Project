"""
Debug script to understand Y-axis reflection in hand tracking.

This will help determine if the Y axis needs to be negated.
"""
from config import HAND_EYE_MATRIX
import numpy as np
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))


print("=" * 70)
print("Y-AXIS REFLECTION DIAGNOSTIC")
print("=" * 70)

# Extract rotation matrix
rotation = HAND_EYE_MATRIX[:3, :3]
translation = HAND_EYE_MATRIX[:3, 3]

print("\nCamera axes in TCP frame (rows of rotation matrix):")
print(f"  Cam X-axis: {rotation[0, :]}")
print(f"  Cam Y-axis: {rotation[1, :]}")
print(f"  Cam Z-axis: {rotation[2, :]}")

print("\n" + "=" * 70)
print("Y-AXIS MOVEMENT TESTS")
print("=" * 70)

print("\nScenario: Hand moves in camera frame, how does it appear in TCP frame?")

# Test 1: Move hand in +Y direction in camera frame (down in image)
print("\n[Test 1] Hand moves DOWN in camera image (+Y camera):")
print("  Camera movement: [0, +0.05, 0] (5cm down in image)")
cam_movement = np.array([0.0, 0.05, 0.0, 1.0])
tcp_movement = HAND_EYE_MATRIX @ cam_movement
# Remove translation to see just the movement
tcp_movement = tcp_movement[:3] - translation
print(f"  TCP movement:    {tcp_movement}")
print(
    f"  → X: {tcp_movement[0]*1000:+.1f}mm, Y: {tcp_movement[1]*1000:+.1f}mm, Z: {tcp_movement[2]*1000:+.1f}mm")

# Test 2: Move hand in -Y direction in camera frame (up in image)
print("\n[Test 2] Hand moves UP in camera image (-Y camera):")
print("  Camera movement: [0, -0.05, 0] (5cm up in image)")
cam_movement = np.array([0.0, -0.05, 0.0, 1.0])
tcp_movement = HAND_EYE_MATRIX @ cam_movement
tcp_movement = tcp_movement[:3] - translation
print(f"  TCP movement:    {tcp_movement}")
print(
    f"  → X: {tcp_movement[0]*1000:+.1f}mm, Y: {tcp_movement[1]*1000:+.1f}mm, Z: {tcp_movement[2]*1000:+.1f}mm")

# Test 3: Move hand in +X direction in camera frame (right in image)
print("\n[Test 3] Hand moves RIGHT in camera image (+X camera):")
print("  Camera movement: [+0.05, 0, 0] (5cm right in image)")
cam_movement = np.array([0.05, 0.0, 0.0, 1.0])
tcp_movement = HAND_EYE_MATRIX @ cam_movement
tcp_movement = tcp_movement[:3] - translation
print(f"  TCP movement:    {tcp_movement}")
print(
    f"  → X: {tcp_movement[0]*1000:+.1f}mm, Y: {tcp_movement[1]*1000:+.1f}mm, Z: {tcp_movement[2]*1000:+.1f}mm")

print("\n" + "=" * 70)
print("INTERPRETATION")
print("=" * 70)
print("""
If you observe Y-axis reflection, it means:
- Moving hand RIGHT (robot's perspective) causes robot to move LEFT
- Moving hand FORWARD causes robot to move BACKWARD

The camera Y-axis components show: [0.027, -0.996, -0.012]
→ Camera Y ≈ -TCP Y (inverted!)

This means:
- Camera DOWN (+Y) = TCP UP (-Y)
- Camera UP (-Y) = TCP DOWN (+Y)

If you're experiencing reflection, we may need to negate the Y component
of the hand position or the offset calculation.
""")

print("\nExpected TCP frame convention (gripper facing down):")
print("  +X = Forward (along gripper approach)")
print("  +Y = Left (across gripper)")
print("  +Z = Down (gripper opening direction)")

print("\nCamera frame convention (RealSense):")
print("  +X = Right (in image)")
print("  +Y = Down (in image)")
print("  +Z = Forward (depth into scene)")
