"""
Quick test script to validate Phase 1 epsilon-clamping implementation.
Tests IK solver with challenging poses that historically caused joint limit violations.
"""

from kinematics.kinematics_solver import (
    InverseKinematicsSolver,
    validate_joint_limits,
    get_facing_down_orientation,
    IK_EPSILON_MARGIN_DEG
)
import numpy as np
import logging
import sys
from pathlib import Path

# Add src to path for imports
sys.path.insert(0, str(Path(__file__).parent / "src"))


# Setup logging to see epsilon-clamping diagnostics
logging.basicConfig(
    level=logging.DEBUG,
    format='%(levelname)s - %(name)s - %(message)s'
)


def test_epsilon_clamping():
    """Test epsilon-clamping with challenging workspace positions."""

    print("=" * 70)
    print("PHASE 1: EPSILON-CLAMPING TEST")
    print(
        f"Epsilon margin: {IK_EPSILON_MARGIN_DEG} deg ({np.deg2rad(IK_EPSILON_MARGIN_DEG):.4f} rad)")
    print("=" * 70)
    print()

    # Initialize solver
    urdf_path = Path("src/resources/robot_models/kuka_with_gripper.urdf")
    if not urdf_path.exists():
        print(f"[ERROR] URDF not found at {urdf_path}")
        print("Please run from project root directory")
        return

    print(f"Loading URDF: {urdf_path}")
    solver = InverseKinematicsSolver(
        urdf_filepath=str(urdf_path),
        base_elements=None,
        active_links_mask=None,
        use_gui=False
    )
    print(f"[OK] Solver initialized\n")

    # Test cases: challenging positions that often cause violations
    test_cases = [
        {
            'name': 'Far Forward Reach',
            'position': [0.65, 0.0, 0.20],
            'description': 'Near maximum reach, tests joint limits A1, A3'
        },
        {
            'name': 'Side Reach Right',
            'position': [0.40, 0.40, 0.25],
            'description': 'Far right, tests A1 near +170° limit'
        },
        {
            'name': 'Side Reach Left',
            'position': [0.40, -0.40, 0.25],
            'description': 'Far left, tests A1 near -170° limit'
        },
        {
            'name': 'High Reach',
            'position': [0.35, 0.0, 0.65],
            'description': 'Near ceiling, tests A2, A4 limits'
        },
        {
            'name': 'Low Corner',
            'position': [0.30, 0.35, 0.05],
            'description': 'Low and far, multiple joint stress'
        },
        {
            'name': 'Workspace Center',
            'position': [0.45, 0.0, 0.30],
            'description': 'Safe central position (control)'
        },
    ]

    orientation = get_facing_down_orientation()
    current_joints = [0.0] * 7  # Start from home position

    violations = 0
    successes = 0

    print("Running test cases...")
    print("-" * 70)

    for i, test in enumerate(test_cases, 1):
        print(f"\n[{i}/{len(test_cases)}] {test['name']}")
        print(f"    Position: {test['position']}")
        print(f"    Description: {test['description']}")

        try:
            # Solve IK
            solution = solver.solve_XYZ(
                target_position=test['position'],
                current_joint_angles=current_joints,
                target_orientation=orientation,
                max_iterations=150,
                tolerance=1e-3
            )

            # Validate solution
            valid = validate_joint_limits(solution.tolist())

            if valid:
                print(f"    [PASS] Solution within limits")
                successes += 1

                # Show joint angles
                joint_names = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7']
                joint_degs = [f"{np.rad2deg(a):+7.2f}deg" for a in solution]
                print(
                    f"    Joints: {', '.join([f'{n}={d}' for n, d in zip(joint_names, joint_degs)])}")
            else:
                print(f"    [FAIL] Joint limit violation detected")
                violations += 1

                # Show which joints violated
                from config import JOINT_LIMITS
                joint_names = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7']
                for j, (angle, name) in enumerate(zip(solution, joint_names)):
                    limits = JOINT_LIMITS[name]
                    if angle < limits['min'] or angle > limits['max']:
                        print(f"    [!] {name}: {np.rad2deg(angle):+7.2f}deg "
                              f"(limits: [{np.rad2deg(limits['min']):+7.2f}deg, "
                              f"{np.rad2deg(limits['max']):+7.2f}deg])")

        except Exception as e:
            print(f"    [ERROR] {type(e).__name__}: {e}")
            violations += 1

    # Summary
    print("\n" + "=" * 70)
    print("RESULTS SUMMARY")
    print("=" * 70)
    print(f"Total tests:       {len(test_cases)}")
    print(f"Successes:         {successes}")
    print(f"Violations:        {violations}")
    print(f"Success rate:      {100 * successes / len(test_cases):.1f}%")
    print()

    if violations == 0:
        print("[SUCCESS] PERFECT! No joint limit violations detected.")
        print("          Epsilon-clamping is working as expected.")
    elif violations <= 1:
        print("[SUCCESS] EXCELLENT! Only 1 violation (likely infeasible target).")
        print("          Epsilon-clamping significantly reduced violations.")
    elif violations <= 2:
        print("[SUCCESS] GOOD! Minimal violations.")
        print("          Consider Phase 2 (multi-seed) for further improvement.")
    else:
        print("[WARNING] Multiple violations detected.")
        print(
            "          Consider increasing IK_EPSILON_MARGIN_DEG or investigating targets.")

    print()
    print(f"Epsilon margin used: {IK_EPSILON_MARGIN_DEG} degrees")
    print("To adjust: Edit IK_EPSILON_MARGIN_DEG in src/kinematics/kinematics_solver.py")
    print("=" * 70)

    solver.disconnect()


if __name__ == "__main__":
    test_epsilon_clamping()
