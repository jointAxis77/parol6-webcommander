"""
Inverse Kinematics Solver for PAROL6 Robot

This module provides centralized IK solving functionality with:
- Adaptive tolerance based on manipulability (proximity to singularities)
- Recursive subdivision for difficult targets
- Angle unwrapping for continuous motion
- Joint limit checking
- Configuration-dependent reach calculations

Author: Extracted from headless_commander.py
Date: 2025-01-12
"""

import numpy as np
from collections import namedtuple
from spatialmath import SE3
from spatialmath.base import trinterp
from roboticstoolbox import DHRobot
import logging

# Get logger
logger = logging.getLogger(__name__)

# Named tuple for IK results
IKResult = namedtuple('IKResult', 'success q iterations residual tolerance_used violations')

# Global state for tolerance tracking
_prev_tolerance = None

# Global performance monitor (set by commander.py)
_performance_monitor = None

def set_performance_monitor(monitor):
    """Set the global performance monitor for IK timing"""
    global _performance_monitor
    _performance_monitor = monitor


def normalize_angle(angle):
    """
    Normalize angle to [-pi, pi] range to handle angle wrapping.

    Parameters
    ----------
    angle : float
        Angle in radians

    Returns
    -------
    float
        Normalized angle in range [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle


def unwrap_angles(q_solution, q_current):
    """
    Unwrap angles in the solution to be closest to current position.

    This handles the angle wrapping issue where -179° and 181° are close
    but appear far apart. Adjusts solution angles to minimize joint motion.

    Parameters
    ----------
    q_solution : array_like
        Solution joint angles
    q_current : array_like
        Current joint angles

    Returns
    -------
    ndarray
        Unwrapped solution angles
    """
    q_unwrapped = q_solution.copy()

    for i in range(len(q_solution)):
        # Calculate the difference
        diff = q_solution[i] - q_current[i]

        # If the difference is more than pi, we need to unwrap
        if diff > np.pi:
            # Solution is too far in positive direction, subtract 2*pi
            q_unwrapped[i] = q_solution[i] - 2 * np.pi
        elif diff < -np.pi:
            # Solution is too far in negative direction, add 2*pi
            q_unwrapped[i] = q_solution[i] + 2 * np.pi

    return q_unwrapped


def calculate_adaptive_tolerance(robot, q, strict_tol=1e-10, loose_tol=1e-7):
    """
    Calculate adaptive tolerance based on proximity to singularities.

    Near singularities: looser tolerance for easier convergence.
    Away from singularities: stricter tolerance for precise solutions.

    Parameters
    ----------
    robot : DHRobot
        Robot model
    q : array_like
        Joint configuration in radians
    strict_tol : float, optional
        Strict tolerance away from singularities (default: 1e-10)
    loose_tol : float, optional
        Loose tolerance near singularities (default: 1e-7)

    Returns
    -------
    float
        Adaptive tolerance value
    """
    global _prev_tolerance

    q_array = np.array(q, dtype=float)

    # Calculate manipulability measure (closer to 0 = closer to singularity)
    if _performance_monitor:
        _performance_monitor.start_phase('ik_manipulability')
    manip = robot.manipulability(q_array)
    if _performance_monitor:
        _performance_monitor.end_phase('ik_manipulability')

    singularity_threshold = 0.001

    # Normalize singularity proximity to [0, 1] range
    sing_normalized = np.clip(manip / singularity_threshold, 0.0, 1.0)

    # Interpolate between loose and strict tolerance
    adaptive_tol = loose_tol + (strict_tol - loose_tol) * sing_normalized

    # Log tolerance changes (only log significant changes to avoid spam)
    if _prev_tolerance is None or abs(adaptive_tol - _prev_tolerance) / _prev_tolerance > 0.5:
        tol_category = "LOOSE" if adaptive_tol > 1e-7 else "MODERATE" if adaptive_tol > 5e-10 else "STRICT"
        logger.debug(f"[IKSolver] Adaptive tolerance: {adaptive_tol:.2e} ({tol_category}) - "
                    f"Manipulability: {manip:.8f} (threshold: {singularity_threshold})")
        _prev_tolerance = adaptive_tol

    return adaptive_tol


def calculate_configuration_dependent_max_reach(q_seed):
    """
    Calculate maximum reach based on joint configuration, particularly joint 5.

    When joint 5 is at ±90 degrees, the effective reach is reduced by approximately
    0.045m due to the robot's kinematic structure.

    Parameters
    ----------
    q_seed : array_like
        Current joint configuration in radians

    Returns
    -------
    float
        Configuration-dependent maximum reach threshold in meters
    """
    base_max_reach = 0.44  # Base maximum reach from experimentation

    # Get J5 angle
    j5_angle = q_seed[4] if len(q_seed) > 4 else 0.0
    j5_normalized = normalize_angle(j5_angle)

    # Calculate distance from ±90 degrees
    angle_90_deg = np.pi / 2
    angle_neg_90_deg = -np.pi / 2
    dist_from_90 = abs(j5_normalized - angle_90_deg)
    dist_from_neg_90 = abs(j5_normalized - angle_neg_90_deg)
    dist_from_90_deg = min(dist_from_90, dist_from_neg_90)

    # Apply reach reduction within 45-degree range of ±90°
    reduction_range = np.pi / 4  # 45 degrees
    if dist_from_90_deg <= reduction_range:
        # Calculate reduction factor based on proximity to 90°
        proximity_factor = 1.0 - (dist_from_90_deg / reduction_range)
        reach_reduction = 0.045 * proximity_factor
        effective_max_reach = base_max_reach - reach_reduction
        return effective_max_reach
    else:
        return base_max_reach


def solve_ik_with_adaptive_tol_subdivision(
        robot: DHRobot,
        target_pose: SE3,
        current_q,
        current_pose: SE3 = None,
        max_depth: int = 4,
        ilimit: int = 100,
        jogging: bool = False,
        joint_limits_checker=None
):
    """
    Solve inverse kinematics with adaptive tolerance and recursive subdivision.

    Uses adaptive tolerance based on proximity to singularities:
    - Near singularities: looser tolerance for easier convergence
    - Away from singularities: stricter tolerance for precise solutions

    If necessary, recursively subdivides the motion until ikine_LMS converges
    on every segment. Finally checks that solution respects joint limits.

    From experimentation, jogging with lower tolerances often produces q_paths
    that do not differ from current_q, essentially freezing the robot.

    Parameters
    ----------
    robot : DHRobot
        Robot model
    target_pose : SE3
        Target pose to reach
    current_q : array_like
        Current joint configuration in radians
    current_pose : SE3, optional
        Current pose (computed if None)
    max_depth : int, optional
        Maximum subdivision depth (default: 4)
    ilimit : int, optional
        Maximum iterations for IK solver (default: 100)
    jogging : bool, optional
        If True, use strict tolerance for jogging (default: False)
    joint_limits_checker : callable, optional
        Function to check joint limits: checker(current_q, target_q) -> (valid, violations)

    Returns
    -------
    IKResult
        NamedTuple with fields:
        - success: True if solution found
        - q: Final joint configuration (or None if failed)
        - iterations: Total IK iterations used
        - residual: Final residual error
        - tolerance_used: Tolerance value used
        - violations: Joint limit violations (if any)
    """
    if current_pose is None:
        current_pose = robot.fkine(current_q)

    # ── Inner recursive solver ───────────────────────────────────────────
    def _solve(Ta: SE3, Tb: SE3, q_seed, depth, tol):
        """
        Recursive IK solver with subdivision.

        Returns
        -------
        tuple
            (path_list, success_flag, iterations, residual)
        """
        # Calculate current and target reach
        current_reach = np.linalg.norm(Ta.t)
        target_reach = np.linalg.norm(Tb.t)

        # Check if this is an inward movement (recovery)
        is_recovery = target_reach < current_reach

        # Calculate configuration-dependent maximum reach based on joint 5 position
        max_reach_threshold = calculate_configuration_dependent_max_reach(q_seed)

        # Determine damping based on reach and movement direction
        if is_recovery:
            # Recovery mode - always use low damping
            damping = 0.0000001
        else:
            # Check if we're near configuration-dependent max reach
            if target_reach > max_reach_threshold:
                logger.error(f"[IKSolver] Target reach limit exceeded: {target_reach:.3f}m > {max_reach_threshold:.3f}m")
                return [], False, depth, 0
            else:
                damping = 0.0000001  # Normal low damping

        # Attempt IK solution (using C++ ik_LM for performance)
        if _performance_monitor:
            _performance_monitor.start_phase('ik_solve')
        # ik_LM returns: (q, success_flag, iterations, searches, residual)
        q_result, success_flag, iterations, searches, residual = robot.ik_LM(
            Tb, q0=q_seed, ilimit=ilimit, tol=tol, k=damping, method='sugihara'
        )
        if _performance_monitor:
            _performance_monitor.end_phase('ik_solve')

        if success_flag:
            q_good = unwrap_angles(q_result, q_seed)  # Unwrap vs previous configuration
            return [q_good], True, iterations, residual

        # If failed and max depth reached, give up
        if depth >= max_depth:
            return [], False, iterations, residual

        # Split the segment and recurse
        Tc = SE3(trinterp(Ta.A, Tb.A, 0.5))  # Mid-pose (screw interpolation)

        # Solve first half
        left_path, ok_L, it_L, r_L = _solve(Ta, Tc, q_seed, depth + 1, tol)
        if not ok_L:
            return [], False, it_L, r_L

        # Solve second half using end of first half as seed
        q_mid = left_path[-1]
        right_path, ok_R, it_R, r_R = _solve(Tc, Tb, q_mid, depth + 1, tol)

        return (
            left_path + right_path,
            ok_R,
            it_L + it_R,
            r_R
        )

    # ── Kick off with adaptive tolerance ─────────────────────────────────
    if jogging:
        adaptive_tol = 1e-10  # Strict tolerance for jogging
    else:
        adaptive_tol = calculate_adaptive_tolerance(robot, current_q)

    # Solve IK with subdivision
    path, ok, its, resid = _solve(current_pose, target_pose, current_q, 0, adaptive_tol)

    # Check if solution respects joint limits
    target_q = path[-1] if len(path) != 0 else None

    if joint_limits_checker is not None:
        solution_valid, violations = joint_limits_checker(current_q, target_q)
    else:
        # No checker provided, assume valid
        solution_valid = ok
        violations = []

    if ok and solution_valid:
        return IKResult(True, path[-1], its, resid, adaptive_tol, violations)
    else:
        return IKResult(False, None, its, resid, adaptive_tol, violations)


class IKSolver:
    """
    Inverse kinematics solver class for PAROL6 robot.

    Provides a convenient object-oriented interface to the IK solving functions.
    """

    def __init__(self, robot_model, joint_limits_checker=None):
        """
        Initialize IK solver.

        Parameters
        ----------
        robot_model : DHRobot
            Robot kinematic model
        joint_limits_checker : callable, optional
            Function to check joint limits
        """
        self.robot = robot_model
        self.joint_limits_checker = joint_limits_checker
        self._solve_count = 0
        self._success_count = 0

    def solve(self, target_pose, current_q, current_pose=None,
             max_depth=4, ilimit=100, jogging=False):
        """
        Solve inverse kinematics for target pose.

        Parameters
        ----------
        target_pose : SE3
            Target pose to reach
        current_q : array_like
            Current joint configuration
        current_pose : SE3, optional
            Current pose (computed if None)
        max_depth : int, optional
            Maximum subdivision depth (default: 4)
        ilimit : int, optional
            Maximum IK iterations (default: 100)
        jogging : bool, optional
            Use strict tolerance for jogging (default: False)

        Returns
        -------
        IKResult
            Solution result with success flag and joint angles
        """
        self._solve_count += 1

        result = solve_ik_with_adaptive_tol_subdivision(
            self.robot,
            target_pose,
            current_q,
            current_pose=current_pose,
            max_depth=max_depth,
            ilimit=ilimit,
            jogging=jogging,
            joint_limits_checker=self.joint_limits_checker
        )

        if result.success:
            self._success_count += 1

        return result

    @property
    def success_rate(self):
        """Get IK solver success rate."""
        if self._solve_count == 0:
            return 0.0
        return self._success_count / self._solve_count

    def reset_stats(self):
        """Reset solver statistics."""
        self._solve_count = 0
        self._success_count = 0
