#!/usr/bin/env python3
"""
Debug URDF vs DH FK step by step
"""

import sys
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path.cwd()))
from lib.utils.numpy_patch import *

from spatialmath import SE3
from lib.kinematics.robot_model import robot as dhrobot
import roboticstoolbox as rtb

# Load URDF
urdf_path = Path.cwd() / "frontend" / "public" / "urdf" / "PAROL6_DH.urdf"
urdf_robot = rtb.ERobot.URDF(str(urdf_path))

# Test at zero configuration
q_zero = np.zeros(6)

print("=" * 80)
print("STEP-BY-STEP FK COMPARISON AT ZERO CONFIGURATION")
print("=" * 80)
print()

# DH Robot FK
print("DH ROBOT:")
print("-" * 80)
T_dh_cumulative = SE3()
for i in range(len(dhrobot.links)):
    T_link = dhrobot.links[i].A(q_zero[i])
    T_dh_cumulative = T_dh_cumulative * T_link

    pos = T_dh_cumulative.t
    rpy_rad = T_dh_cumulative.rpy(order='xyz')
    rpy_deg = np.rad2deg(rpy_rad)

    print(f"After Joint {i+1}:")
    print(f"  Pos (mm): X={pos[0]*1000:.3f}, Y={pos[1]*1000:.3f}, Z={pos[2]*1000:.3f}")
    print(f"  RPY (deg): RX={rpy_deg[0]:.3f}, RY={rpy_deg[1]:.3f}, RZ={rpy_deg[2]:.3f}")

print()
print("URDF ROBOT:")
print("-" * 80)

# Check if URDF has ets() method (Elementary Transform Sequence)
if hasattr(urdf_robot, 'ets'):
    print("URDF robot has ETS")
    ets = urdf_robot.ets()
    print(f"ETS: {ets}")
    print()

    # Try to get individual link transforms
    for i in range(1, len(urdf_robot.links)):
        try:
            link = urdf_robot.links[i]
            print(f"Link {i}: {link.name}")
            if hasattr(link, 'parent'):
                print(f"  Parent: {link.parent.name if link.parent else 'None'}")
            if hasattr(link, 'jindex'):
                print(f"  Joint index: {link.jindex}")
        except Exception as e:
            print(f"  Error: {e}")

print()

# Check what the URDF fkine gives
T_urdf = urdf_robot.fkine(q_zero)
pos_urdf = T_urdf.t
rpy_urdf_rad = T_urdf.rpy(order='xyz')
rpy_urdf_deg = np.rad2deg(rpy_urdf_rad)

print("URDF FK at zero:")
print(f"  Pos (mm): X={pos_urdf[0]*1000:.3f}, Y={pos_urdf[1]*1000:.3f}, Z={pos_urdf[2]*1000:.3f}")
print(f"  RPY (deg): RX={rpy_urdf_deg[0]:.3f}, RY={rpy_urdf_deg[1]:.3f}, RZ={rpy_urdf_deg[2]:.3f}")
print()

# Final DH FK
T_dh_final = dhrobot.fkine(q_zero)
pos_dh = T_dh_final.t
rpy_dh_rad = T_dh_final.rpy(order='xyz')
rpy_dh_deg = np.rad2deg(rpy_dh_rad)

print("DH FK at zero:")
print(f"  Pos (mm): X={pos_dh[0]*1000:.3f}, Y={pos_dh[1]*1000:.3f}, Z={pos_dh[2]*1000:.3f}")
print(f"  RPY (deg): RX={rpy_dh_deg[0]:.3f}, RY={rpy_dh_deg[1]:.3f}, RZ={rpy_dh_deg[2]:.3f}")
print()

# Difference
pos_error = np.linalg.norm((pos_dh - pos_urdf) * 1000)
print(f"Position error: {pos_error:.3f} mm")
print()

# Let me also check what the URDF joints look like
print("=" * 80)
print("URDF JOINT INFORMATION")
print("=" * 80)
for i, link in enumerate(urdf_robot.links):
    print(f"Link {i}: {link.name}")
    if hasattr(link, 'geometry'):
        print(f"  Geometry: {link.geometry}")
    if hasattr(link, 'v'):
        print(f"  Visual: {link.v}")

print()

# Check the actual URDF structure
print("=" * 80)
print("CHECKING URDF FILE STRUCTURE")
print("=" * 80)
import xml.etree.ElementTree as ET
tree = ET.parse(str(urdf_path))
root = tree.getroot()

for joint in root.findall('joint'):
    joint_name = joint.get('name')
    joint_type = joint.get('type')
    origin = joint.find('origin')
    axis = joint.find('axis')

    xyz = origin.get('xyz') if origin is not None else "0 0 0"
    rpy = origin.get('rpy') if origin is not None else "0 0 0"
    axis_xyz = axis.get('xyz') if axis is not None else "0 0 1"

    print(f"{joint_name} ({joint_type}):")
    print(f"  Origin xyz: {xyz}")
    print(f"  Origin rpy: {rpy}")
    print(f"  Axis: {axis_xyz}")

print()
