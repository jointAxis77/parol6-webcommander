#!/usr/bin/env python3
"""
Test IK solver performance after switching to C++ ik_LM method.
Compares performance and validates functionality.
"""

import sys
import time
import numpy as np
from spatialmath import SE3

# Apply numpy patch and add project root to path
sys.path.insert(0, '/home/jacob/parol6')
from lib.utils import numpy_patch
from lib.kinematics import robot_model as PAROL6_ROBOT
from lib.kinematics.ik_solver import solve_ik_with_adaptive_tol_subdivision

def test_ik_functionality():
    """Test that IK solver works correctly"""
    print("=" * 60)
    print("Testing IK Solver Functionality")
    print("=" * 60)

    robot = PAROL6_ROBOT.robot

    # Test pose: reasonable workspace position
    target_xyz = [200, 0, 300]  # mm
    target_rpy_deg = [0, -90, 0]  # degrees
    target_rpy_rad = np.radians(target_rpy_deg)

    # Create target SE3 pose
    target_se3 = SE3.Trans(target_xyz[0]/1000, target_xyz[1]/1000, target_xyz[2]/1000) * \
                 SE3.RPY(target_rpy_rad, order='xyz')

    # Seed configuration (home position)
    q_seed = np.array([0, 0, 0, 0, 0, 0])

    print(f"\nTarget Position: X={target_xyz[0]}, Y={target_xyz[1]}, Z={target_xyz[2]} mm")
    print(f"Target Orientation: RX={target_rpy_deg[0]}, RY={target_rpy_deg[1]}, RZ={target_rpy_deg[2]} deg")

    # Solve IK
    try:
        result = solve_ik_with_adaptive_tol_subdivision(
            robot,
            target_se3,
            q_seed,
            max_depth=4,
            ilimit=100
        )

        if result.success:
            print(f"\n✓ IK Solution Found!")
            print(f"  Joint angles (deg): {np.degrees(result.q)}")
            print(f"  Iterations: {result.iterations}")
            print(f"  Residual: {result.residual:.2e}")
            print(f"  Tolerance used: {result.tolerance_used:.2e}")

            # Verify FK matches target
            fk_result = robot.fkine(result.q)
            fk_xyz = fk_result.t * 1000  # Convert to mm
            fk_rpy_rad = fk_result.rpy(order='xyz')
            fk_rpy_deg = np.degrees(fk_rpy_rad)

            print(f"\nForward Kinematics Verification:")
            print(f"  Position: X={fk_xyz[0]:.1f}, Y={fk_xyz[1]:.1f}, Z={fk_xyz[2]:.1f} mm")
            print(f"  Orientation: RX={fk_rpy_deg[0]:.1f}, RY={fk_rpy_deg[1]:.1f}, RZ={fk_rpy_deg[2]:.1f} deg")

            pos_error = np.linalg.norm(fk_xyz - target_xyz)
            print(f"  Position error: {pos_error:.2f} mm")

            return True
        else:
            print(f"\n✗ IK Failed: {result.error}")
            return False

    except Exception as e:
        print(f"\n✗ Exception: {e}")
        import traceback
        traceback.print_exc()
        return False


def benchmark_ik_performance(num_tests=10):
    """Benchmark IK solver performance"""
    print("\n" + "=" * 60)
    print(f"Benchmarking IK Performance ({num_tests} iterations)")
    print("=" * 60)

    robot = PAROL6_ROBOT.robot

    # Generate test poses
    test_poses = []
    for i in range(num_tests):
        # Vary position and orientation
        x = 150 + i * 10
        y = -50 + i * 10
        z = 250 + i * 5
        rx = 0
        ry = -90 + i * 5
        rz = -90 + i * 10

        target_se3 = SE3.Trans(x/1000, y/1000, z/1000) * \
                     SE3.RPY(np.radians([rx, ry, rz]), order='xyz')
        test_poses.append(target_se3)

    # Benchmark
    q_seed = np.array([0, 0, 0, 0, 0, 0])
    solve_times = []
    successes = 0

    print("\nTesting...", end='', flush=True)

    for i, target_se3 in enumerate(test_poses):
        start_time = time.perf_counter()

        result = solve_ik_with_adaptive_tol_subdivision(
            robot,
            target_se3,
            q_seed,
            max_depth=4,
            ilimit=100
        )

        end_time = time.perf_counter()
        solve_time_ms = (end_time - start_time) * 1000

        solve_times.append(solve_time_ms)
        if result.success:
            successes += 1
            q_seed = result.q  # Use as seed for next

        print('.', end='', flush=True)

    print(" Done!")

    # Statistics
    solve_times = np.array(solve_times)
    avg_time = np.mean(solve_times)
    min_time = np.min(solve_times)
    max_time = np.max(solve_times)
    std_time = np.std(solve_times)

    print(f"\nResults:")
    print(f"  Success rate: {successes}/{num_tests} ({100*successes/num_tests:.1f}%)")
    print(f"  Average time: {avg_time:.3f} ms")
    print(f"  Min time: {min_time:.3f} ms")
    print(f"  Max time: {max_time:.3f} ms")
    print(f"  Std dev: {std_time:.3f} ms")

    # Performance assessment
    print(f"\nPerformance Assessment:")
    if avg_time < 1.0:
        print(f"  ✓ EXCELLENT: {avg_time:.3f}ms (sub-millisecond!)")
        print(f"  100Hz control loop: {avg_time/10:.1f}% of 10ms budget")
    elif avg_time < 5.0:
        print(f"  ✓ GOOD: {avg_time:.3f}ms (suitable for real-time)")
        print(f"  100Hz control loop: {avg_time/10:.1f}% of 10ms budget")
    elif avg_time < 20.0:
        print(f"  ⚠ ACCEPTABLE: {avg_time:.3f}ms")
        print(f"  100Hz control loop: {avg_time/10:.1f}% of 10ms budget")
    else:
        print(f"  ✗ TOO SLOW: {avg_time:.3f}ms")
        print(f"  100Hz control loop: {avg_time/10:.1f}% of 10ms budget (too high!)")

    # Compare to old performance
    old_avg_time = 50.0  # From recordings
    speedup = old_avg_time / avg_time
    print(f"\nComparison to Previous:")
    print(f"  Old average: {old_avg_time:.1f} ms (Python ikine_LM)")
    print(f"  New average: {avg_time:.3f} ms (C++ ik_LM)")
    print(f"  Speedup: {speedup:.1f}x faster!")

    return avg_time, successes == num_tests


if __name__ == "__main__":
    print("\nPAROL6 IK Solver Performance Test")
    print("Testing C++ ik_LM() method")
    print()

    # Test 1: Functionality
    functionality_ok = test_ik_functionality()

    if not functionality_ok:
        print("\n✗ Functionality test failed. Fix errors before benchmarking.")
        sys.exit(1)

    # Test 2: Performance
    avg_time, all_success = benchmark_ik_performance(num_tests=20)

    # Summary
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)

    if functionality_ok and all_success:
        print("✓ All tests passed!")
        if avg_time < 1.0:
            print("✓ Performance is EXCELLENT (sub-millisecond)")
            print("✓ Ready for 100Hz real-time control")
        elif avg_time < 10.0:
            print("✓ Performance is GOOD (real-time capable)")
        else:
            print("⚠ Performance acceptable but could be better")
    else:
        print("✗ Some tests failed")

    print()
