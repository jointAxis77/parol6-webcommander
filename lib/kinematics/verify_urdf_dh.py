#!/usr/bin/env python3
"""
Verify that PAROL6_DH.urdf produces identical FK results to DHRobot model.

Tests FK equivalence for multiple configurations across the workspace.
"""

import sys
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path.cwd()))
from lib.utils.numpy_patch import *

from spatialmath import SE3
from lib.kinematics.robot_model import robot as dhrobot
import roboticstoolbox as rtb
from math import pi

# Load URDF model
urdf_path = Path.cwd() / "frontend" / "public" / "urdf" / "PAROL6_DH.urdf"
print("=" * 80)
print("LOADING MODELS")
print("=" * 80)
print(f"URDF path: {urdf_path}")
print()

try:
    urdf_robot = rtb.ERobot.URDF(str(urdf_path))
    print(f"✓ Loaded URDF: {urdf_robot.name}")
    print(f"  Number of joints: {urdf_robot.n}")
    print()
except Exception as e:
    print(f"✗ Failed to load URDF: {e}")
    sys.exit(1)

print(f"✓ Loaded DHRobot: {dhrobot.name}")
print(f"  Number of joints: {dhrobot.n}")
print()

# Test configurations (from previous debugging)
test_configs = [
    {
        'name': 'Config 1 (J1=90°)',
        'joints_deg': [90.0, -90.0, 180.0, 0.0, 0.0, 180.0],
        'expected_pos_mm': [0.0, 233.8, 334.0],
        'expected_rpy_deg': [-180.0, 0.0, -90.0]
    },
    {
        'name': 'Config 2 (J1=0°)',
        'joints_deg': [0.0, -90.0, 180.0, 0.0, 0.0, 90.0],
        'expected_pos_mm': [233.8, 0.0, 334.0],
        'expected_rpy_deg': [0.0, 90.0, -180.0]
    },
    {
        'name': 'Home position',
        'joints_deg': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        'expected_pos_mm': None,  # Don't know expected
        'expected_rpy_deg': None
    },
    {
        'name': 'Extended reach',
        'joints_deg': [45.0, -45.0, 90.0, 0.0, 45.0, 90.0],
        'expected_pos_mm': None,
        'expected_rpy_deg': None
    },
]

def normalize_angle(angle):
    """Normalize angle to [-180, 180] degrees"""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

def compare_fk(joints_deg, config_name, expected_pos_mm=None, expected_rpy_deg=None):
    """Compare FK results between DHRobot and URDF"""

    print("=" * 80)
    print(f"{config_name}")
    print("=" * 80)
    print(f"Joint angles (deg): {joints_deg}")
    print()

    # Convert to radians
    joints_rad = np.deg2rad(joints_deg)

    # IMPORTANT: Joint 6 has a pi/2 offset in DHRobot
    # We need to account for this when comparing
    dh_joints = joints_rad.copy()
    # The offset is already in the DH model, so we don't modify the input

    # Compute DHRobot FK
    T_dh = dhrobot.fkine(dh_joints)
    pos_dh_m = T_dh.t
    pos_dh_mm = pos_dh_m * 1000.0
    rpy_dh_rad = T_dh.rpy(order='xyz')
    rpy_dh_deg = np.rad2deg(rpy_dh_rad)

    # Compute URDF FK
    T_urdf = urdf_robot.fkine(joints_rad)
    pos_urdf_m = T_urdf.t
    pos_urdf_mm = pos_urdf_m * 1000.0
    rpy_urdf_rad = T_urdf.rpy(order='xyz')
    rpy_urdf_deg = np.rad2deg(rpy_urdf_rad)

    # Display results
    print("DHRobot FK:")
    print(f"  Position (mm): X={pos_dh_mm[0]:.3f}, Y={pos_dh_mm[1]:.3f}, Z={pos_dh_mm[2]:.3f}")
    print(f"  Orientation (deg): RX={rpy_dh_deg[0]:.3f}, RY={rpy_dh_deg[1]:.3f}, RZ={rpy_dh_deg[2]:.3f}")
    print()

    print("URDF FK:")
    print(f"  Position (mm): X={pos_urdf_mm[0]:.3f}, Y={pos_urdf_mm[1]:.3f}, Z={pos_urdf_mm[2]:.3f}")
    print(f"  Orientation (deg): RX={rpy_urdf_deg[0]:.3f}, RY={rpy_urdf_deg[1]:.3f}, RZ={rpy_urdf_deg[2]:.3f}")
    print()

    # Compare positions
    pos_error = np.linalg.norm(pos_dh_mm - pos_urdf_mm)
    print(f"Position error: {pos_error:.4f} mm")

    # Compare orientations (accounting for angle wrap-around)
    rpy_diff = np.array([
        normalize_angle(rpy_dh_deg[0] - rpy_urdf_deg[0]),
        normalize_angle(rpy_dh_deg[1] - rpy_urdf_deg[1]),
        normalize_angle(rpy_dh_deg[2] - rpy_urdf_deg[2])
    ])
    orientation_error = np.linalg.norm(rpy_diff)
    print(f"Orientation error: {orientation_error:.4f} deg")
    print(f"  ΔRX={rpy_diff[0]:.3f}°, ΔRY={rpy_diff[1]:.3f}°, ΔRZ={rpy_diff[2]:.3f}°")
    print()

    # Check against expected values if provided
    if expected_pos_mm is not None:
        expected_pos = np.array(expected_pos_mm)
        dh_error_from_expected = np.linalg.norm(pos_dh_mm - expected_pos)
        print(f"DHRobot vs Expected: {dh_error_from_expected:.4f} mm")

    if expected_rpy_deg is not None:
        expected_rpy = np.array(expected_rpy_deg)
        rpy_diff_expected = np.array([
            normalize_angle(rpy_dh_deg[0] - expected_rpy[0]),
            normalize_angle(rpy_dh_deg[1] - expected_rpy[1]),
            normalize_angle(rpy_dh_deg[2] - expected_rpy[2])
        ])
        dh_orient_error_from_expected = np.linalg.norm(rpy_diff_expected)
        print(f"DHRobot orientation vs Expected: {dh_orient_error_from_expected:.4f} deg")
        print()

    # Pass/fail criteria
    pos_pass = pos_error < 0.1  # 0.1mm tolerance
    orient_pass = orientation_error < 0.5  # 0.5 degree tolerance

    if pos_pass and orient_pass:
        print("✓ PASS - FK results match within tolerance")
    else:
        print("✗ FAIL - FK results differ beyond tolerance")
        if not pos_pass:
            print(f"  Position error {pos_error:.4f} mm exceeds 0.1mm threshold")
        if not orient_pass:
            print(f"  Orientation error {orientation_error:.4f}° exceeds 0.5° threshold")

    print()

    return pos_pass and orient_pass

# Run tests
print("=" * 80)
print("FK VERIFICATION TEST")
print("=" * 80)
print()

all_passed = True
for config in test_configs:
    passed = compare_fk(
        config['joints_deg'],
        config['name'],
        config['expected_pos_mm'],
        config['expected_rpy_deg']
    )
    all_passed = all_passed and passed

print("=" * 80)
print("SUMMARY")
print("=" * 80)
if all_passed:
    print("✓ ALL TESTS PASSED")
    print("URDF FK matches DHRobot FK within tolerance.")
else:
    print("✗ SOME TESTS FAILED")
    print("URDF FK does not match DHRobot FK.")
print()
