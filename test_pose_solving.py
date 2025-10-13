"""
Test script to validate solve_pose and solve_pose_iterative with orientation tracking.
Demonstrates that Phase 1 epsilon-clamping applies to full 6-DOF pose solving.
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


# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(levelname)s - %(name)s - %(message)s'
)


def test_pose_solving():
    """Test pose solving with orientation constraints and epsilon-clamping."""

    print("=" * 70)
    print("POSE SOLVING TEST (with epsilon-clamping)")
    print(f"Epsilon margin: {IK_EPSILON_MARGIN_DEG} deg")
    print("=" * 70)
    print()

    # Initialize solver
    urdf_path = Path("src/resources/robot_models/kuka_with_gripper.urdf")
    if not urdf_path.exists():
        print(f"[ERROR] URDF not found at {urdf_path}")
        return

    print(f"Loading URDF: {urdf_path}")
    solver = InverseKinematicsSolver(
        urdf_filepath=str(urdf_path),
        base_elements=None,
        active_links_mask=None,
        use_gui=False
    )
    print(f"[OK] Solver initialized\n")

    # Test cases: Position + Orientation (full 6-DOF pose)
    test_cases = [
        {
            'name': 'Downward Grasp (Standard)',
            'pose': [0.45, 0.0, 0.30, np.pi, 0.0, 0.0],  # Facing down
            'description': 'TCP pointing straight down (typical grasp)'
        },
        {
            'name': 'Tilted Grasp (15° pitch)',
            'pose': [0.40, 0.2, 0.25, np.pi, np.deg2rad(15), 0.0],
            'description': 'TCP tilted 15° forward'
        },
        {
            'name': 'Side Approach (90° roll)',
            'pose': [0.35, -0.3, 0.30, np.pi, 0.0, np.deg2rad(90)],
            'description': 'TCP rotated 90° for side grasp'
        },
        {
            'name': 'High Reach with Tilt',
            'pose': [0.35, 0.0, 0.55, np.pi - np.deg2rad(20), 0.0, 0.0],
            'description': 'High position with 20° backward tilt'
        },
    ]

    current_joints = [0.0] * 7

    print("=" * 70)
    print("TEST 1: Standard solve_pose with orientation verification")
    print("=" * 70)
    print()

    violations = 0
    successes = 0
    high_orient_errors = 0

    for i, test in enumerate(test_cases, 1):
        print(f"[{i}/{len(test_cases)}] {test['name']}")
        print(f"    Target: pos=[{test['pose'][0]:.3f}, {test['pose'][1]:.3f}, {test['pose'][2]:.3f}], "
              f"euler=[{np.rad2deg(test['pose'][3]):.1f}°, {np.rad2deg(test['pose'][4]):.1f}°, "
              f"{np.rad2deg(test['pose'][5]):.1f}°]")
        print(f"    Description: {test['description']}")

        try:
            # Solve with orientation verification enabled
            solution = solver.solve_pose(
                target_pose=test['pose'],
                current_joint_angles=current_joints,
                max_iterations=150,
                tolerance=1e-3,
                verify_orientation=True,  # Enable orientation checking
                orientation_tolerance_deg=2.0
            )

            # Validate joint limits
            valid = validate_joint_limits(solution.tolist())

            if valid:
                print(f"    [PASS] Joint limits satisfied")
                successes += 1
            else:
                print(f"    [FAIL] Joint limit violation")
                violations += 1

        except Exception as e:
            print(f"    [ERROR] {type(e).__name__}: {e}")
            violations += 1

        print()

    print("=" * 70)
    print("TEST 2: Iterative pose refinement (strict accuracy)")
    print("=" * 70)
    print()

    # Test iterative refinement on challenging pose
    challenging_pose = [0.50, 0.25, 0.35,
                        np.pi, np.deg2rad(10), np.deg2rad(30)]

    print(
        f"Challenging pose: pos=[{challenging_pose[0]:.3f}, {challenging_pose[1]:.3f}, {challenging_pose[2]:.3f}]")
    print(f"                  euler=[{np.rad2deg(challenging_pose[3]):.1f}°, "
          f"{np.rad2deg(challenging_pose[4]):.1f}°, {np.rad2deg(challenging_pose[5]):.1f}°]")
    print()

    try:
        print("Running iterative refinement...")
        solution = solver.solve_pose_iterative(
            target_pose=challenging_pose,
            current_joint_angles=[0.0]*7,
            max_outer_iterations=3,
            max_ik_iterations=150,
            position_tolerance=1e-3,
            orientation_tolerance_deg=1.0  # Stricter than standard
        )

        if validate_joint_limits(solution.tolist()):
            print("[PASS] Iterative refinement succeeded with valid joint limits")
            successes += 1
        else:
            print("[FAIL] Iterative refinement violated joint limits")
            violations += 1

    except Exception as e:
        print(f"[ERROR] Iterative refinement failed: {e}")
        violations += 1

    # Summary
    print()
    print("=" * 70)
    print("RESULTS SUMMARY")
    print("=" * 70)
    print(f"Total tests:       {len(test_cases) + 1}")
    print(f"Successes:         {successes}")
    print(f"Violations:        {violations}")
    print(f"Success rate:      {100 * successes / (len(test_cases) + 1):.1f}%")
    print()

    if violations == 0:
        print("[SUCCESS] All pose tests passed with epsilon-clamping!")
        print("          Orientation constraints respected joint limits.")
    else:
        print(f"[WARNING] {violations} test(s) failed.")
        print("          Review targets or increase epsilon margin.")

    print()
    print("Key Points:")
    print("  - solve_pose inherits epsilon-clamping from solve_XYZ")
    print("  - Higher default iterations (150) for orientation convergence")
    print("  - Optional orientation verification available")
    print("  - Iterative refinement for strict accuracy requirements")
    print("=" * 70)

    solver.disconnect()


if __name__ == "__main__":
    test_pose_solving()


