"""
Kinematics Module

Contains IK/FK solvers, robot model, and trajectory generation.

Exports:
- ik_solver: Inverse kinematics solving functions
- robot_model: PAROL6 robot kinematic model
- trajectory_math: Trajectory generation (circular, spline, etc.)
"""

# Import submodules so they can be accessed as:
# from lib.kinematics import ik_solver, robot_model, trajectory_math
from . import ik_solver
from . import robot_model
from . import trajectory_math

# Also expose commonly used items directly
from .ik_solver import solve_ik_with_adaptive_tol_subdivision, IKResult
from .robot_model import robot, check_joint_limits

__all__ = [
    'ik_solver',
    'robot_model',
    'trajectory_math',
    'solve_ik_with_adaptive_tol_subdivision',
    'IKResult',
    'robot',
    'check_joint_limits',
]
