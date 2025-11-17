#!/usr/bin/env python3
"""
Test if the TCP offset fix makes DHRobot coordinates match URDF
"""

import sys
import numpy as np
from pathlib import Path

# Add lib to path
sys.path.insert(0, str(Path(__file__).parent))

# Apply numpy patch first
from lib.utils.numpy_patch import *

from lib.kinematics.robot_model import robot as dhrobot

def test_tcp_fix():
    """Test FK with the fixed TCP offset"""

    print("=" * 80)
    print("Testing TCP Offset Fix")
    print("=" * 80)
    print()

    # Test "Home" position from config.yaml
    joints_deg = [90, -90, 180, 0, 0, 180]
    joints_rad = np.deg2rad(joints_deg)

    print(f"Joint angles (deg): {joints_deg}")
    print()

    # Compute FK
    T = dhrobot.fkine(joints_rad)

    # Extract position (mm)
    pos_mm = T.t * 1000.0

    # Extract orientation (deg)
    rpy_rad = T.rpy(order='xyz')
    ori_deg = np.rad2deg(rpy_rad)

    print(f"DHRobot FK Result (with tool offset):")
    print(f"  Position: X={pos_mm[0]:.1f}, Y={pos_mm[1]:.1f}, Z={pos_mm[2]:.1f} mm")
    print(f"  Orientation: RX={ori_deg[0]:.1f}, RY={ori_deg[1]:.1f}, RZ={ori_deg[2]:.1f} deg")
    print()

    # Expected URDF coordinates (from previous ERobot test)
    # Home position ERobot result was: X=0.0, Y=-113.1, Z=-65.8 mm
    expected_pos = np.array([0.0, -113.1, -65.8])
    expected_ori = np.array([90.0, 0.0, 0.0])

    print(f"Expected URDF Coordinates:")
    print(f"  Position: X={expected_pos[0]:.1f}, Y={expected_pos[1]:.1f}, Z={expected_pos[2]:.1f} mm")
    print(f"  Orientation: RX={expected_ori[0]:.1f}, RY={expected_ori[1]:.1f}, RZ={expected_ori[2]:.1f} deg")
    print()

    # Calculate error
    pos_err = np.linalg.norm(pos_mm - expected_pos)
    ori_err = np.linalg.norm(ori_deg - expected_ori)

    print(f"Difference from URDF:")
    print(f"  Position error: {pos_err:.1f} mm")
    print(f"  Orientation error: {ori_err:.1f} deg")
    print()

    if pos_err < 1.0 and ori_err < 1.0:
        print("✓ SUCCESS! Coordinates now match URDF (< 1mm, < 1° error)")
    elif pos_err < 10.0 and ori_err < 10.0:
        print("✓ GOOD! Coordinates are close to URDF (< 10mm, < 10° error)")
    else:
        print("✗ Still significant difference from URDF")

    print()
    print("=" * 80)


if __name__ == "__main__":
    test_tcp_fix()
