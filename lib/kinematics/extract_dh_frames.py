#!/usr/bin/env python3
"""
Extract link frame transforms directly from DHRobot at zero configuration.
Use these to build a URDF that matches exactly.
"""

import sys
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path.cwd()))
from lib.utils.numpy_patch import *

from spatialmath import SE3
from lib.kinematics.robot_model import robot as dhrobot
from math import pi

print("=" * 80)
print("EXTRACTING DH LINK FRAMES")
print("=" * 80)
print()

# Zero configuration (all joints at 0, but accounting for offsets)
q_zero = np.zeros(6)

print("Computing link transforms at zero configuration:")
print(f"Joint angles: {q_zero}")
print()

# Compute cumulative transforms for each link
T_cumulative = SE3()  # Identity at base
link_transforms = [T_cumulative]  # Base

for i in range(len(dhrobot.links)):
    # Get individual link transform
    T_link = dhrobot.links[i].A(q_zero[i])
    T_cumulative = T_cumulative * T_link
    link_transforms.append(T_cumulative)

    pos = T_cumulative.t
    R = T_cumulative.R
    rpy_rad = T_cumulative.rpy(order='xyz')
    rpy_deg = np.rad2deg(rpy_rad)

    print(f"Link {i+1} frame:")
    print(f"  Position (m): [{pos[0]:.5f}, {pos[1]:.5f}, {pos[2]:.5f}]")
    print(f"  RPY (deg): [{rpy_deg[0]:.3f}, {rpy_deg[1]:.3f}, {rpy_deg[2]:.3f}]")
    print(f"  Rotation matrix:")
    for row in R:
        print(f"    [{row[0]:7.4f}, {row[1]:7.4f}, {row[2]:7.4f}]")
    print()

# Now compute relative transforms (parent to child)
print("=" * 80)
print("RELATIVE TRANSFORMS (for URDF joint origins)")
print("=" * 80)
print()

for i in range(len(dhrobot.links)):
    if i == 0:
        # First joint: base to link1
        T_rel = link_transforms[1]
        parent_name = "base_link"
        child_name = "link1"
    else:
        # Subsequent joints: link(i) to link(i+1)
        T_rel = link_transforms[i].inv() * link_transforms[i+1]
        parent_name = f"link{i}"
        child_name = f"link{i+1}"

    pos = T_rel.t
    rpy_rad = T_rel.rpy(order='xyz')
    rpy_deg = np.rad2deg(rpy_rad)
    R = T_rel.R

    print(f"Joint {i+1}: {parent_name} -> {child_name}")
    print(f"  Origin xyz: [{pos[0]:.5f}, {pos[1]:.5f}, {pos[2]:.5f}]")
    print(f"  Origin rpy: [{rpy_rad[0]:.5f}, {rpy_rad[1]:.5f}, {rpy_rad[2]:.5f}]")
    print(f"  Origin rpy (deg): [{rpy_deg[0]:.3f}, {rpy_deg[1]:.3f}, {rpy_deg[2]:.3f}]")

    # Compute joint axis in child frame
    # The joint rotates about z in the DH frame, but after the origin transform
    # So we need to find what direction z points in the child frame
    # For DH, joint i rotates about z_i, which is the z-axis of the child frame
    # After the origin transform, the joint axis should be [0, 0, 1]
    print(f"  Joint axis (child frame): [0, 0, 1]")
    print()

print("=" * 80)
print("NOTES")
print("=" * 80)
print("- These transforms are extracted at zero configuration")
print("- Joint 6 has a theta offset of pi/2, already accounted for")
print("- All joint axes should be [0, 0, 1] in child frame")
print("- These values can be used directly in URDF joint origins")
print()
