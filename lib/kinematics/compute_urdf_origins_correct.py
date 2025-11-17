#!/usr/bin/env python3
"""
Correctly compute URDF joint origins from DH parameters.

For Modified DH, the transform from frame i-1 to frame i is:
  T_i = Rx(alpha_{i-1}) * Tx(a_{i-1}) * Tz(d_i) * Rz(theta_i)

The URDF origin is the FIXED part (before joint rotation):
  origin = Rx(alpha_{i-1}) * Tx(a_{i-1}) * Tz(d_i)

Then the joint rotates about z-axis by theta_i.
"""

import sys
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path.cwd()))
from lib.utils.numpy_patch import *

from spatialmath import SE3
from math import pi

# DH parameters from robot_model.py
a1 = 110.50 / 1000
a2 = 23.42 / 1000
a3 = 180 / 1000
a4 = 43.5 / 1000
a5 = 176.35 / 1000
a6 = 34.0 / 1000
a7 = 0

alpha_DH = [-pi/2, pi, pi/2, -pi/2, pi/2, pi]

# Modified DH parameters: (d, a, alpha)
# Note: We ignore theta and offset here since they're part of the joint rotation
dh_params = [
    (a1, a2, alpha_DH[0]),     # Joint 1: previous joint is world (a_0=0, alpha_0=0)
    (0, a3, alpha_DH[1]),      # Joint 2: previous is J1
    (0, -a4, alpha_DH[2]),     # Joint 3: previous is J2
    (-a5, 0, alpha_DH[3]),     # Joint 4: previous is J3
    (0, 0, alpha_DH[4]),       # Joint 5: previous is J4
    (-a6, -a7, alpha_DH[5]),   # Joint 6: previous is J5
]

print("=" * 80)
print("CORRECT URDF JOINT ORIGINS FROM DH PARAMETERS")
print("=" * 80)
print()

def compute_fixed_transform(d, a_prev, alpha_prev):
    """
    Compute the fixed transform (URDF origin) from DH parameters.

    Modified DH: T = Rx(alpha_prev) * Tx(a_prev) * Tz(d) * Rz(theta)

    Returns the fixed part: Rx(alpha_prev) * Tx(a_prev) * Tz(d)
    """
    # Build transform step by step
    T = SE3()

    # Step 1: Rotate about x by alpha_prev
    if abs(alpha_prev) > 1e-10:
        T = T * SE3.Rx(alpha_prev)

    # Step 2: Translate along x by a_prev
    if abs(a_prev) > 1e-10:
        T = T * SE3([a_prev, 0, 0])

    # Step 3: Translate along z by d
    if abs(d) > 1e-10:
        T = T * SE3([0, 0, d])

    return T

# Joint 1: world to link1
# Previous: world frame (a_0=0, alpha_0=0)
# Current: d_1=a1, a_1=a2, alpha_1=alpha_DH[0]
print("Joint 1: world/base_link -> link1")
T1 = compute_fixed_transform(dh_params[0][0], 0, 0)
xyz = T1.t
rpy = T1.rpy(order='xyz')
print(f"  Origin xyz: [{xyz[0]:.5f}, {xyz[1]:.5f}, {xyz[2]:.5f}]")
print(f"  Origin rpy: [{rpy[0]:.5f}, {rpy[1]:.5f}, {rpy[2]:.5f}]")
print(f"  Origin rpy (deg): [{np.rad2deg(rpy[0]):.3f}, {np.rad2deg(rpy[1]):.3f}, {np.rad2deg(rpy[2]):.3f}]")
print()

# Joint 2: link1 -> link2
# Previous: a_1=a2, alpha_1=alpha_DH[0]
# Current: d_2=0, a_2=a3, alpha_2=alpha_DH[1]
print("Joint 2: link1 -> link2")
T2 = compute_fixed_transform(dh_params[1][0], dh_params[0][1], dh_params[0][2])
xyz = T2.t
rpy = T2.rpy(order='xyz')
print(f"  Origin xyz: [{xyz[0]:.5f}, {xyz[1]:.5f}, {xyz[2]:.5f}]")
print(f"  Origin rpy: [{rpy[0]:.5f}, {rpy[1]:.5f}, {rpy[2]:.5f}]")
print(f"  Origin rpy (deg): [{np.rad2deg(rpy[0]):.3f}, {np.rad2deg(rpy[1]):.3f}, {np.rad2deg(rpy[2]):.3f}]")
print()

# Joint 3: link2 -> link3
print("Joint 3: link2 -> link3")
T3 = compute_fixed_transform(dh_params[2][0], dh_params[1][1], dh_params[1][2])
xyz = T3.t
rpy = T3.rpy(order='xyz')
print(f"  Origin xyz: [{xyz[0]:.5f}, {xyz[1]:.5f}, {xyz[2]:.5f}]")
print(f"  Origin rpy: [{rpy[0]:.5f}, {rpy[1]:.5f}, {rpy[2]:.5f}]")
print(f"  Origin rpy (deg): [{np.rad2deg(rpy[0]):.3f}, {np.rad2deg(rpy[1]):.3f}, {np.rad2deg(rpy[2]):.3f}]")
print()

# Joint 4: link3 -> link4
print("Joint 4: link3 -> link4")
T4 = compute_fixed_transform(dh_params[3][0], dh_params[2][1], dh_params[2][2])
xyz = T4.t
rpy = T4.rpy(order='xyz')
print(f"  Origin xyz: [{xyz[0]:.5f}, {xyz[1]:.5f}, {xyz[2]:.5f}]")
print(f"  Origin rpy: [{rpy[0]:.5f}, {rpy[1]:.5f}, {rpy[2]:.5f}]")
print(f"  Origin rpy (deg): [{np.rad2deg(rpy[0]):.3f}, {np.rad2deg(rpy[1]):.3f}, {np.rad2deg(rpy[2]):.3f}]")
print()

# Joint 5: link4 -> link5
print("Joint 5: link4 -> link5")
T5 = compute_fixed_transform(dh_params[4][0], dh_params[3][1], dh_params[3][2])
xyz = T5.t
rpy = T5.rpy(order='xyz')
print(f"  Origin xyz: [{xyz[0]:.5f}, {xyz[1]:.5f}, {xyz[2]:.5f}]")
print(f"  Origin rpy: [{rpy[0]:.5f}, {rpy[1]:.5f}, {rpy[2]:.5f}]")
print(f"  Origin rpy (deg): [{np.rad2deg(rpy[0]):.3f}, {np.rad2deg(rpy[1]):.3f}, {np.rad2deg(rpy[2]):.3f}]")
print()

# Joint 6: link5 -> link6
print("Joint 6: link5 -> link6")
T6 = compute_fixed_transform(dh_params[5][0], dh_params[4][1], dh_params[4][2])
xyz = T6.t
rpy = T6.rpy(order='xyz')
print(f"  Origin xyz: [{xyz[0]:.5f}, {xyz[1]:.5f}, {xyz[2]:.5f}]")
print(f"  Origin rpy: [{rpy[0]:.5f}, {rpy[1]:.5f}, {rpy[2]:.5f}]")
print(f"  Origin rpy (deg): [{np.rad2deg(rpy[0]):.3f}, {np.rad2deg(rpy[1]):.3f}, {np.rad2deg(rpy[2]):.3f}]")
print()

print("=" * 80)
print("NOTES")
print("=" * 80)
print("- All joint axes are [0, 0, 1] in the child frame")
print("- Joint 6 has a theta offset of pi/2 which affects the zero position")
print("- This offset is NOT part of the origin transform")
print("- It must be handled separately in the URDF or joint angle conversion")
print()
