#!/usr/bin/env python3
"""
Debug orientation differences between DHRobot and URDF
"""

import sys
import numpy as np
from pathlib import Path

# Add lib to path
sys.path.insert(0, str(Path.cwd()))
from lib.utils.numpy_patch import *

from spatialmath import SE3
from lib.kinematics.robot_model import robot as dhrobot

def rpy_to_rotation_matrix(rx, ry, rz, order='xyz'):
    """Convert RPY angles (degrees) to rotation matrix"""
    rx_rad = np.deg2rad(rx)
    ry_rad = np.deg2rad(ry)
    rz_rad = np.deg2rad(rz)

    T = SE3.RPY([rx_rad, ry_rad, rz_rad], order=order)
    return T.R

def compare_orientations(urdf_rpy, dhrobot_rpy, config_name):
    """Compare two orientations"""
    print(f"\n{config_name}:")
    print("=" * 80)

    print(f"URDF RPY:    RX={urdf_rpy[0]:.1f}°, RY={urdf_rpy[1]:.1f}°, RZ={urdf_rpy[2]:.1f}°")
    print(f"DHRobot RPY: RX={dhrobot_rpy[0]:.1f}°, RY={dhrobot_rpy[1]:.1f}°, RZ={dhrobot_rpy[2]:.1f}°")
    print()

    # Convert to rotation matrices
    R_urdf = rpy_to_rotation_matrix(urdf_rpy[0], urdf_rpy[1], urdf_rpy[2])
    R_dhrobot = rpy_to_rotation_matrix(dhrobot_rpy[0], dhrobot_rpy[1], dhrobot_rpy[2])

    print("URDF Rotation Matrix:")
    print(R_urdf)
    print()

    print("DHRobot Rotation Matrix:")
    print(R_dhrobot)
    print()

    # Check if they're equal
    diff = np.abs(R_urdf - R_dhrobot)
    max_diff = np.max(diff)

    print(f"Max difference: {max_diff:.6f}")

    if max_diff < 1e-6:
        print("✓ Orientations are IDENTICAL")
    else:
        print("✗ Orientations are DIFFERENT")

        # Check if one is a transform of the other
        # Compute relative rotation
        R_rel = R_dhrobot.T @ R_urdf
        print()
        print("Relative rotation (DHRobot -> URDF):")
        print(R_rel)

        # Convert relative rotation to RPY
        T_rel = SE3(R_rel)
        try:
            rel_rpy_rad = T_rel.rpy(order='xyz')
            rel_rpy = np.rad2deg(rel_rpy_rad)
            print(f"Relative RPY (deg): {rel_rpy}")
        except Exception as e:
            print(f"Could not convert to RPY: {e}")

    print()

def main():
    print("=" * 80)
    print("ORIENTATION DEBUG: DHRobot vs URDF")
    print("=" * 80)

    # Configuration 1: J1=90°
    config1_joints = [90.0, -90.0, 180.0, 0.0, 0.0, 180.0]
    config1_urdf_rpy = [0.0, 90.0, 0.0]

    joints_rad = np.deg2rad(config1_joints)
    T = dhrobot.fkine(joints_rad)
    config1_dhrobot_rpy = np.rad2deg(T.rpy(order='xyz'))

    compare_orientations(config1_urdf_rpy, config1_dhrobot_rpy, "Config 1 (J1=90°)")

    # Configuration 2: J1=0°
    config2_joints = [0.0, -90.0, 180.0, 0.0, 0.0, 90.0]
    config2_urdf_rpy = [-90.0, 0.0, -180.0]

    joints_rad = np.deg2rad(config2_joints)
    T = dhrobot.fkine(joints_rad)
    config2_dhrobot_rpy = np.rad2deg(T.rpy(order='xyz'))

    compare_orientations(config2_urdf_rpy, config2_dhrobot_rpy, "Config 2 (J1=0°)")

    print("=" * 80)
    print("ANALYSIS")
    print("=" * 80)
    print()
    print("If the relative rotation is consistent across configurations,")
    print("there's a fixed coordinate frame transformation between DHRobot and URDF.")
    print()

if __name__ == "__main__":
    main()
