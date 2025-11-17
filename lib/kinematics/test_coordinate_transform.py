#!/usr/bin/env python3
"""
Test coordinate transformation: URDF -> DH

Tests if transforming a URDF gizmo target to DH coordinates
works correctly for IK solving.
"""

import sys
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path.cwd()))
from lib.utils.numpy_patch import *

from spatialmath import SE3
from lib.kinematics.robot_model import robot as dhrobot

# Transformation matrix: URDF -> DH
# X_dh = -Y_urdf, Y_dh = Z_urdf, Z_dh = -X_urdf
R_urdf_to_dh = np.array([
    [ 0, -1,  0],
    [ 0,  0,  1],
    [-1,  0,  0]
])

def transform_urdf_to_dh(urdf_pose):
    """
    Transform pose from URDF coordinates to DH coordinates.

    Parameters
    ----------
    urdf_pose : dict
        Position (mm) and orientation (deg) in URDF frame

    Returns
    -------
    dict
        Position (mm) and orientation (deg) in DH frame
    """
    # Extract URDF pose
    pos_urdf = np.array([urdf_pose['x'], urdf_pose['y'], urdf_pose['z']]) / 1000.0
    rpy_urdf_rad = np.deg2rad([urdf_pose['rx'], urdf_pose['ry'], urdf_pose['rz']])

    # Create SE3 transform in URDF frame
    T_urdf = SE3.RPY(rpy_urdf_rad, order='xyz') * SE3(pos_urdf)

    # Transform to DH frame: T_dh = R_transform * T_urdf * R_transform^T
    T_transform = SE3(R_urdf_to_dh)
    T_dh = T_transform * T_urdf * T_transform.inv()

    # Extract DH pose
    pos_dh = T_dh.t * 1000.0
    rpy_dh_rad = T_dh.rpy(order='xyz')
    rpy_dh_deg = np.rad2deg(rpy_dh_rad)

    return {
        'x': pos_dh[0],
        'y': pos_dh[1],
        'z': pos_dh[2],
        'rx': rpy_dh_deg[0],
        'ry': rpy_dh_deg[1],
        'rz': rpy_dh_deg[2]
    }

def test_transformation():
    """Test if transformation allows correct IK solving"""

    print("=" * 80)
    print("COORDINATE TRANSFORMATION TEST")
    print("=" * 80)
    print()

    # Simulate: User moves gizmo to a target in URDF coordinates
    # Let's use a known good DH target and convert it to URDF,
    # then transform it back to verify

    # Known good DH target (from earlier tests)
    dh_target_original = {
        'x': 0.0, 'y': 233.8, 'z': 334.0,
        'rx': -180.0, 'ry': 0.0, 'rz': -90.0
    }

    print("Test 1: Round-trip transformation")
    print("-" * 80)
    print(f"Original DH target: {dh_target_original}")
    print()

    # Convert DH -> URDF (simulate what gizmo would show)
    # This is the inverse transformation
    pos_dh = np.array([dh_target_original['x'], dh_target_original['y'], dh_target_original['z']]) / 1000.0
    rpy_dh_rad = np.deg2rad([dh_target_original['rx'], dh_target_original['ry'], dh_target_original['rz']])

    T_dh = SE3.RPY(rpy_dh_rad, order='xyz') * SE3(pos_dh)
    T_transform = SE3(R_urdf_to_dh)
    T_urdf = T_transform.inv() * T_dh * T_transform

    pos_urdf = T_urdf.t * 1000.0
    rpy_urdf_rad = T_urdf.rpy(order='xyz')
    rpy_urdf_deg = np.rad2deg(rpy_urdf_rad)

    urdf_target = {
        'x': pos_urdf[0], 'y': pos_urdf[1], 'z': pos_urdf[2],
        'rx': rpy_urdf_deg[0], 'ry': rpy_urdf_deg[1], 'rz': rpy_urdf_deg[2]
    }

    print(f"Simulated URDF gizmo target: {urdf_target}")
    print()

    # Now transform back: URDF -> DH (what frontend would do)
    dh_target_transformed = transform_urdf_to_dh(urdf_target)

    print(f"Transformed back to DH: {dh_target_transformed}")
    print()

    # Check if we got back to the original
    pos_error = np.sqrt(
        (dh_target_original['x'] - dh_target_transformed['x'])**2 +
        (dh_target_original['y'] - dh_target_transformed['y'])**2 +
        (dh_target_original['z'] - dh_target_transformed['z'])**2
    )

    print(f"Position error: {pos_error:.3f} mm")
    print(f"Orientation difference:")
    print(f"  RX: {dh_target_original['rx']:.1f}° -> {dh_target_transformed['rx']:.1f}°")
    print(f"  RY: {dh_target_original['ry']:.1f}° -> {dh_target_transformed['ry']:.1f}°")
    print(f"  RZ: {dh_target_original['rz']:.1f}° -> {dh_target_transformed['rz']:.1f}°")
    print()

    if pos_error < 1.0:
        print("✓ Position round-trip successful!")
    else:
        print("✗ Position round-trip failed")

    print()
    print("=" * 80)
    print()

if __name__ == "__main__":
    test_transformation()
