#!/usr/bin/env python3
"""
Test SE3 composition to understand the transform behavior
"""

import sys
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path.cwd()))
from lib.utils.numpy_patch import *

from spatialmath import SE3
from math import pi

print("=" * 80)
print("TESTING SE3 COMPOSITION")
print("=" * 80)
print()

# Test 1: Rx rotation followed by translation
print("Test 1: Rx(-90°) * Tx(0.02342)")
print("-" * 80)

T = SE3()
T = T * SE3.Rx(-pi/2)
print(f"After Rx(-90°):")
print(f"  Position: {T.t}")
print(f"  RPY: {np.rad2deg(T.rpy(order='xyz'))}")
print(f"  Rotation matrix:")
print(T.R)
print()

T = T * SE3([0.02342, 0, 0])
print(f"After * SE3([0.02342, 0, 0]):")
print(f"  Position: {T.t}")
print(f"  RPY: {np.rad2deg(T.rpy(order='xyz'))}")
print(f"  Rotation matrix:")
print(T.R)
print()

# Test 2: Create combined transform directly
print("Test 2: SE3.Rx(-90°) with translation")
print("-" * 80)

# Try creating with Rt
T2 = SE3.Rx(-pi/2) * SE3([0.02342, 0, 0])
print(f"SE3.Rx(-90°) * SE3([0.02342, 0, 0]):")
print(f"  Position: {T2.t}")
print(f"  RPY: {np.rad2deg(T2.rpy(order='xyz'))}")
print()

# Test 3: Translation then rotation
print("Test 3: Tx(0.02342) * Rx(-90°)")
print("-" * 80)

T3 = SE3([0.02342, 0, 0]) * SE3.Rx(-pi/2)
print(f"SE3([0.02342, 0, 0]) * SE3.Rx(-90°):")
print(f"  Position: {T3.t}")
print(f"  RPY: {np.rad2deg(T3.rpy(order='xyz'))}")
print()

# Test 4: Check if RPY is unique
print("Test 4: Check RPY uniqueness")
print("-" * 80)

# Create rotation Rx(-90°)
R1 = SE3.Rx(-pi/2)
print(f"Rx(-90°):")
print(f"  RPY: {np.rad2deg(R1.rpy(order='xyz'))}")

# Create rotation Rz(-90°)
R2 = SE3.Rz(-pi/2)
print(f"Rz(-90°):")
print(f"  RPY: {np.rad2deg(R2.rpy(order='xyz'))}")

# Are they the same?
print(f"Are rotation matrices equal? {np.allclose(R1.R, R2.R)}")
print()

# Test 5: What does URDF origin (0.02342, 0, 0; 0, 0, -90°) produce?
print("Test 5: URDF origin interpretation")
print("-" * 80)

# In URDF, origin is: xyz then rpy
# SE3 with position and orientation
xyz = np.array([0.02342, 0, 0])
rpy = np.array([0, 0, -pi/2])

# Method 1: RPY then translation
T_urdf_1 = SE3.RPY(rpy, order='xyz') * SE3(xyz)
print(f"Method 1: SE3.RPY([0, 0, -90°]) * SE3([0.02342, 0, 0]):")
print(f"  Position: {T_urdf_1.t}")
print(f"  RPY: {np.rad2deg(T_urdf_1.rpy(order='xyz'))}")
print()

# Method 2: Translation then RPY
T_urdf_2 = SE3(xyz) * SE3.RPY(rpy, order='xyz')
print(f"Method 2: SE3([0.02342, 0, 0]) * SE3.RPY([0, 0, -90°]):")
print(f"  Position: {T_urdf_2.t}")
print(f"  RPY: {np.rad2deg(T_urdf_2.rpy(order='xyz'))}")
print()

# Method 3: Combined (this is how URDF should work)
T_urdf_3 = SE3.Rt(SE3.RPY(rpy, order='xyz').R, xyz)
print(f"Method 3: SE3.Rt(R, t) where R=RPY([0,0,-90°]), t=[0.02342,0,0]:")
print(f"  Position: {T_urdf_3.t}")
print(f"  RPY: {np.rad2deg(T_urdf_3.rpy(order='xyz'))}")
print()
