#!/usr/bin/env python3
"""
Analyze yaw gaps in current calibration poses.
"""
import numpy as np
import json
from pathlib import Path


def analyze_yaw_gaps():
    """Analyze yaw gaps in the current pose dataset."""
    print("🎯 Analyzing Yaw Gaps in Current Poses")
    print("=" * 60)

    # Load the pose analysis data
    analysis_file = "calibration_poses_analysis.json"
    if not Path(analysis_file).exists():
        print(f"❌ Analysis file {analysis_file} not found!")
        print("Run src/extract_all_pose_matrices.py first.")
        return

    with open(analysis_file, 'r') as f:
        pose_data = json.load(f)

    # Extract yaw values
    yaw_values = []
    for pose in pose_data:
        yaw = pose['tcp_euler_xyz'][2]  # Yaw is the third element
        yaw_values.append(yaw)

    yaw_values = np.array(yaw_values)

    print(f"📊 Current Yaw Distribution:")
    print(f"  Total poses: {len(yaw_values)}")
    print(
        f"  Yaw range: {np.min(yaw_values):.1f}° to {np.max(yaw_values):.1f}°")
    print(f"  Yaw values: {np.sort(yaw_values)}")

    # Sort yaw values and find gaps
    yaw_sorted = np.sort(yaw_values)
    print(f"\n📈 Gap Analysis:")

    gaps = []
    for i in range(len(yaw_sorted) - 1):
        current_yaw = yaw_sorted[i]
        next_yaw = yaw_sorted[i+1]
        gap_size = next_yaw - current_yaw

        # Handle wrap-around at ±180°
        if gap_size > 180:
            gap_size = gap_size - 360

        gaps.append({
            'start': current_yaw,
            'end': next_yaw,
            'size': gap_size,
            'midpoint': (current_yaw + next_yaw) / 2
        })

        print(
            f"  Gap {i+1}: {current_yaw:.1f}° → {next_yaw:.1f}° (size: {gap_size:.1f}°)")

    # Find significant gaps (>30°)
    significant_gaps = [gap for gap in gaps if abs(gap['size']) > 30]

    print(f"\n⚠️ Significant Gaps (>30°):")
    if significant_gaps:
        for i, gap in enumerate(significant_gaps):
            print(
                f"  Gap {i+1}: {gap['size']:.1f}° between {gap['start']:.1f}° and {gap['end']:.1f}°")
            print(f"    Suggested yaw: {gap['midpoint']:.1f}°")
    else:
        print("  No significant gaps found!")

    # Suggest specific yaw targets to fill gaps
    print(f"\n🎯 Recommended Yaw Targets:")

    # Target yaw angles to improve diversity
    target_yaws = []

    # Analyze each 30-degree segment
    for start_yaw in range(-180, 180, 30):
        end_yaw = start_yaw + 30
        segment_yaws = yaw_values[(
            yaw_values >= start_yaw) & (yaw_values < end_yaw)]

        if len(segment_yaws) == 0:
            target_yaw = start_yaw + 15  # Middle of empty segment
            target_yaws.append(target_yaw)
            print(
                f"  Empty segment {start_yaw:4.0f}° to {end_yaw:4.0f}°: Target yaw {target_yaw:6.1f}°")
        elif len(segment_yaws) == 1:
            target_yaw = start_yaw + 15  # Add one more in this segment
            target_yaws.append(target_yaw)
            print(
                f"  Sparse segment {start_yaw:4.0f}° to {end_yaw:4.0f}°: Target yaw {target_yaw:6.1f}°")

    # Specific recommendations based on largest gaps
    print(f"\n🔥 Priority Yaw Targets (based on largest gaps):")

    # Sort gaps by size
    gaps_by_size = sorted(gaps, key=lambda x: abs(x['size']), reverse=True)

    for i, gap in enumerate(gaps_by_size[:5]):  # Top 5 gaps
        if abs(gap['size']) > 30:
            target_yaw = gap['midpoint']
            print(
                f"  Priority {i+1}: Yaw {target_yaw:6.1f}° (fills {abs(gap['size']):.1f}° gap)")
            target_yaws.append(target_yaw)

    # Remove duplicates and sort
    target_yaws = sorted(list(set(target_yaws)))

    print(f"\n📋 Complete List of Missing Yaw Angles:")
    for i, yaw in enumerate(target_yaws):
        print(f"  {i+1:2d}. Yaw {yaw:6.1f}°")

    return target_yaws


if __name__ == "__main__":
    analyze_yaw_gaps()
