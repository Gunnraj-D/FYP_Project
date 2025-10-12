"""
Quick diagnostic to analyze IK violation frequency from logs.
Run: python check_ik_violations.py <log_file>
"""

import sys
import re
from collections import Counter

if len(sys.argv) < 2:
    print("Usage: python check_ik_violations.py <log_file>")
    sys.exit(1)

log_file = sys.argv[1]

violation_pattern = re.compile(
    r"Joint (A\d) .* angle ([-\d.]+) exceeds limits")
clamp_pattern = re.compile(r"Clamped joint (A\d) from ([-\d.]+) to ([-\d.]+)")

violations = Counter()
clamps = []
total_lines = 0

with open(log_file, 'r') as f:
    for line in f:
        total_lines += 1

        # Check for violations
        match = violation_pattern.search(line)
        if match:
            joint = match.group(1)
            angle = float(match.group(2))
            violations[joint] += 1

        # Check for clamps
        match = clamp_pattern.search(line)
        if match:
            joint = match.group(1)
            from_angle = float(match.group(2))
            to_angle = float(match.group(3))
            overshoot = abs(from_angle - to_angle)
            clamps.append((joint, overshoot))

print("=" * 60)
print("IK VIOLATION ANALYSIS")
print("=" * 60)
print(f"Total log lines:     {total_lines}")
print(f"Total violations:    {sum(violations.values())}")
print()

if violations:
    print("Violations by joint:")
    for joint, count in sorted(violations.items()):
        print(f"  {joint}: {count}")
    print()

    print("Overshoot statistics (degrees):")
    overshoots_deg = [abs(o * 57.2958) for _, o in clamps]
    if overshoots_deg:
        print(f"  Min:  {min(overshoots_deg):.2f}°")
        print(f"  Max:  {max(overshoots_deg):.2f}°")
        print(f"  Avg:  {sum(overshoots_deg)/len(overshoots_deg):.2f}°")
        print()

    # Recommendation
    avg_overshoot = sum(overshoots_deg) / \
        len(overshoots_deg) if overshoots_deg else 0

    if avg_overshoot > 5.0:
        print("⚠️  RECOMMENDATION: Implement Phase 2 (Multi-Seed Solving)")
        print("   Average overshoot >5° indicates bad local minima issues")
    elif sum(violations.values()) > total_lines * 0.01:
        print("⚠️  RECOMMENDATION: Consider Phase 2 if violations >1% of operations")
    else:
        print("✅ Phase 1 epsilon-clamping handling violations well")
        print("   Consider increasing grasp_depth_offset if needed")
else:
    print("✅ No violations detected - Phase 1 working perfectly!")

print("=" * 60)
