#!/usr/bin/env python3
"""
Test FK on IK solution to verify correctness
"""

import sys
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path.cwd()))
from lib.utils.numpy_patch import *

from lib.kinematics.robot_model import robot as dhrobot

# Target pose from user (Y=20mm)
target_pose = {
    'X': 233.77,
    'Y': 20.00,
    'Z': 334.00,
    'RX': 0.0,
    'RY': 90.0,
    'RZ': -180.0
}

# IK solution from backend
ik_solution_deg = [5.72, -89.76, 180.25, 89.97, -5.72, 180.0]  # Assuming J6=180

print("=" * 80)
print("FK VERIFICATION OF IK SOLUTION")
print("=" * 80)
print()

print("Target Pose (what user wanted):")
print(f"  Position: X={target_pose['X']:.2f}, Y={target_pose['Y']:.2f}, Z={target_pose['Z']:.2f} mm")
print(f"  Orientation: RX={target_pose['RX']:.1f}°, RY={target_pose['RY']:.1f}°, RZ={target_pose['RZ']:.1f}°")
print()

print("IK Solution (joint angles from backend):")
print(f"  {ik_solution_deg}")
print()

# Run FK on solution
ik_solution_rad = np.deg2rad(ik_solution_deg)
T_fk = dhrobot.fkine(ik_solution_rad)

# Extract position and orientation
pos_mm = T_fk.t * 1000.0
rpy_rad = T_fk.rpy(order='xyz')
rpy_deg = np.rad2deg(rpy_rad)

print("FK Result (what robot actually achieves):")
print(f"  Position: X={pos_mm[0]:.2f}, Y={pos_mm[1]:.2f}, Z={pos_mm[2]:.2f} mm")
print(f"  Orientation: RX={rpy_deg[0]:.1f}°, RY={rpy_deg[1]:.1f}°, RZ={rpy_deg[2]:.1f}°")
print()

# Calculate errors
pos_error = np.sqrt(
    (target_pose['X'] - pos_mm[0])**2 +
    (target_pose['Y'] - pos_mm[1])**2 +
    (target_pose['Z'] - pos_mm[2])**2
)

def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

rx_error = normalize_angle(target_pose['RX'] - rpy_deg[0])
ry_error = normalize_angle(target_pose['RY'] - rpy_deg[1])
rz_error = normalize_angle(target_pose['RZ'] - rpy_deg[2])

print("Errors:")
print(f"  Position: {pos_error:.4f} mm")
print(f"  Orientation: ΔRX={rx_error:.3f}°, ΔRY={ry_error:.3f}°, ΔRZ={rz_error:.3f}°")
print()

if pos_error < 1.0 and abs(rx_error) < 1.0 and abs(ry_error) < 1.0 and abs(rz_error) < 1.0:
    print("✓ IK solution is CORRECT - FK matches target!")
else:
    print("✗ IK solution is WRONG - FK does NOT match target!")
    print()
    print("This suggests a coordinate system mismatch or IK solver bug.")
print()
