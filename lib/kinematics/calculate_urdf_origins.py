#!/usr/bin/env python3
"""
Calculate URDF joint origins from DH parameters.

Modified DH convention used by roboticstoolbox:
- Transform from link i-1 to link i:
  1. Rotate alpha_{i-1} about x_{i-1}
  2. Translate a_{i-1} along x_{i-1}
  3. Translate d_i along z_i
  4. Rotate theta_i about z_i (joint variable)

For URDF:
- Joint origin specifies transform from parent link to child link
- Origin xyz is the translation
- Origin rpy is the rotation (roll about x, pitch about y, yaw about z)
"""

import sys
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path.cwd()))
from lib.utils.numpy_patch import *

from math import pi

# DH parameters from robot_model.py
a1 = 110.50 / 1000  # meters
a2 = 23.42 / 1000
a3 = 180 / 1000
a4 = 43.5 / 1000
a5 = 176.35 / 1000
a6 = 34.0 / 1000
a7 = 0

alpha_DH = [-pi/2, pi, pi/2, -pi/2, pi/2, pi]

# Modified DH parameters for each joint
# Format: (d, a, alpha, offset)
dh_params = [
    (a1, a2, alpha_DH[0], 0),           # Joint 1
    (0, a3, alpha_DH[1], 0),            # Joint 2
    (0, -a4, alpha_DH[2], 0),           # Joint 3
    (-a5, 0, alpha_DH[3], 0),           # Joint 4
    (0, 0, alpha_DH[4], 0),             # Joint 5
    (-a6, -a7, alpha_DH[5], pi/2),      # Joint 6
]

print("=" * 80)
print("DH PARAMETERS TO URDF JOINT ORIGINS")
print("=" * 80)
print()

print("DH Parameters (d, a, alpha, offset):")
print("-" * 80)
for i, (d, a, alpha, offset) in enumerate(dh_params):
    print(f"Joint {i+1}: d={d:.5f}, a={a:.5f}, alpha={alpha:.5f}, offset={offset:.5f}")
print()

print("=" * 80)
print("URDF JOINT ORIGINS")
print("=" * 80)
print()

# For modified DH, the joint origin is:
# - Translation: [a_{i-1}, -d_i*sin(alpha_{i-1}), d_i*cos(alpha_{i-1})]
# - Rotation: [alpha_{i-1}, 0, 0] (roll about x-axis)
#
# BUT: We need to account for the parent's alpha rotation
# The origin of joint i in the parent (link i-1) frame is:
# - xyz: [a_{i-1}, 0, 0] rotated by alpha_{i-1}, then translate d_i along z

def dh_to_urdf_origin(d_i, a_prev, alpha_prev):
    """
    Calculate URDF joint origin from DH parameters.

    Parameters
    ----------
    d_i : float
        DH parameter d for current joint (offset along z)
    a_prev : float
        DH parameter a for previous joint (offset along x)
    alpha_prev : float
        DH parameter alpha for previous joint (rotation about x)

    Returns
    -------
    xyz : list
        Translation [x, y, z]
    rpy : list
        Rotation [roll, pitch, yaw]
    """
    # Translation:
    # First translate a_prev along x, then rotate alpha_prev about x, then translate d_i along z
    # After rotating alpha_prev about x: z becomes [0, -sin(alpha), cos(alpha)]
    # So translation is: [a_prev, -d_i*sin(alpha_prev), d_i*cos(alpha_prev)]

    x = a_prev
    y = -d_i * np.sin(alpha_prev)
    z = d_i * np.cos(alpha_prev)

    # Rotation: alpha_prev about x-axis
    roll = alpha_prev
    pitch = 0
    yaw = 0

    return [x, y, z], [roll, pitch, yaw]

# Base joint (world to link1/base_link)
print("Joint: base_joint (world -> base_link)")
print("  Origin xyz: [0.0, 0.0, 0.0]")
print("  Origin rpy: [0.0, 0.0, 0.0]")
print()

# Joint 1 (base_link -> link1)
# Previous: world (a_0=0, alpha_0=0)
# Current: d_1=a1, a_1=a2, alpha_1=alpha_DH[0]
xyz, rpy = dh_to_urdf_origin(dh_params[0][0], 0, 0)
print(f"Joint 1: base_link -> link1")
print(f"  Origin xyz: [{xyz[0]:.5f}, {xyz[1]:.5f}, {xyz[2]:.5f}]")
print(f"  Origin rpy: [{rpy[0]:.5f}, {rpy[1]:.5f}, {rpy[2]:.5f}]")
print()

# Joint 2 (link1 -> link2)
# Previous: a_1=a2, alpha_1=alpha_DH[0]
# Current: d_2=0, a_2=a3, alpha_2=alpha_DH[1]
xyz, rpy = dh_to_urdf_origin(dh_params[1][0], dh_params[0][1], dh_params[0][2])
print(f"Joint 2: link1 -> link2")
print(f"  Origin xyz: [{xyz[0]:.5f}, {xyz[1]:.5f}, {xyz[2]:.5f}]")
print(f"  Origin rpy: [{rpy[0]:.5f}, {rpy[1]:.5f}, {rpy[2]:.5f}]")
print()

# Joint 3 (link2 -> link3)
xyz, rpy = dh_to_urdf_origin(dh_params[2][0], dh_params[1][1], dh_params[1][2])
print(f"Joint 3: link2 -> link3")
print(f"  Origin xyz: [{xyz[0]:.5f}, {xyz[1]:.5f}, {xyz[2]:.5f}]")
print(f"  Origin rpy: [{rpy[0]:.5f}, {rpy[1]:.5f}, {rpy[2]:.5f}]")
print()

# Joint 4 (link3 -> link4)
xyz, rpy = dh_to_urdf_origin(dh_params[3][0], dh_params[2][1], dh_params[2][2])
print(f"Joint 4: link3 -> link4")
print(f"  Origin xyz: [{xyz[0]:.5f}, {xyz[1]:.5f}, {xyz[2]:.5f}]")
print(f"  Origin rpy: [{rpy[0]:.5f}, {rpy[1]:.5f}, {rpy[2]:.5f}]")
print()

# Joint 5 (link4 -> link5)
xyz, rpy = dh_to_urdf_origin(dh_params[4][0], dh_params[3][1], dh_params[3][2])
print(f"Joint 5: link4 -> link5")
print(f"  Origin xyz: [{xyz[0]:.5f}, {xyz[1]:.5f}, {xyz[2]:.5f}]")
print(f"  Origin rpy: [{rpy[0]:.5f}, {rpy[1]:.5f}, {rpy[2]:.5f}]")
print()

# Joint 6 (link5 -> link6)
xyz, rpy = dh_to_urdf_origin(dh_params[5][0], dh_params[4][1], dh_params[4][2])
# Joint 6 has an offset of pi/2, which affects the zero position but not the origin
print(f"Joint 6: link5 -> link6 (has pi/2 offset)")
print(f"  Origin xyz: [{xyz[0]:.5f}, {xyz[1]:.5f}, {xyz[2]:.5f}]")
print(f"  Origin rpy: [{rpy[0]:.5f}, {rpy[1]:.5f}, {rpy[2]:.5f}]")
print()

print("=" * 80)
print("NOTES:")
print("=" * 80)
print("- Joint 6 has a theta offset of pi/2 radians")
print("- This affects the zero position, not the joint origin")
print("- All joint axes in modified DH point along local z-axis: [0, 0, 1]")
print("- Joint limits from robot_model.py should be applied")
print()
