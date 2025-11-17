#!/usr/bin/env python3
"""
Test DHRobot FK against actual known positions
"""

import sys
import numpy as np
from pathlib import Path

# Add lib to path
sys.path.insert(0, str(Path(__file__).parent))

# Apply numpy patch first
from lib.utils.numpy_patch import *

from lib.kinematics.robot_model import robot as dhrobot

def test_positions():
    """Test FK against actual known positions"""

    print("=" * 80)
    print("Testing DHRobot FK Against Actual Known Positions")
    print("=" * 80)
    print()

    # Test positions from user
    positions = [
        {
            "name": "Position 1",
            "joints": [90.0, -90.0, 180.0, 0.0, 0.0, 180.0],
            "expected": {"x": 42.0, "y": 262.6, "z": 334.0, "rx": 0.0, "ry": -90.0, "rz": -270.0}
        },
        {
            "name": "Position 2",
            "joints": [90.0, -145.0, 107.9, 0.0, 0.0, 180.0],
            "expected": {"x": 42.0, "y": 117.3, "z": 184.9, "rx": -17.1, "ry": -90.0, "rz": -270.0}
        }
    ]

    for pos in positions:
        print(f"{pos['name']}:")
        print(f"  Joints (deg): {pos['joints']}")
        print()

        # Convert to radians and compute FK
        joints_rad = np.deg2rad(pos['joints'])
        T = dhrobot.fkine(joints_rad)

        # Extract position (mm)
        pos_mm = T.t * 1000.0

        # Extract orientation (deg)
        rpy_rad = T.rpy(order='xyz')
        ori_deg = np.rad2deg(rpy_rad)

        # Expected values
        exp = pos['expected']

        print(f"  DHRobot FK:")
        print(f"    Position: X={pos_mm[0]:.1f}, Y={pos_mm[1]:.1f}, Z={pos_mm[2]:.1f} mm")
        print(f"    Orientation: RX={ori_deg[0]:.1f}, RY={ori_deg[1]:.1f}, RZ={ori_deg[2]:.1f} deg")
        print()

        print(f"  Expected (actual system):")
        print(f"    Position: X={exp['x']:.1f}, Y={exp['y']:.1f}, Z={exp['z']:.1f} mm")
        print(f"    Orientation: RX={exp['rx']:.1f}, RY={exp['ry']:.1f}, RZ={exp['rz']:.1f} deg")
        print()

        # Calculate error
        pos_err_x = abs(pos_mm[0] - exp['x'])
        pos_err_y = abs(pos_mm[1] - exp['y'])
        pos_err_z = abs(pos_mm[2] - exp['z'])
        pos_err_total = np.sqrt(pos_err_x**2 + pos_err_y**2 + pos_err_z**2)

        ori_err_rx = abs(ori_deg[0] - exp['rx'])
        ori_err_ry = abs(ori_deg[1] - exp['ry'])
        ori_err_rz = abs(ori_deg[2] - exp['rz'])
        ori_err_total = np.sqrt(ori_err_rx**2 + ori_err_ry**2 + ori_err_rz**2)

        print(f"  Error:")
        print(f"    Position: ΔX={pos_err_x:.1f}mm, ΔY={pos_err_y:.1f}mm, ΔZ={pos_err_z:.1f}mm (total: {pos_err_total:.1f}mm)")
        print(f"    Orientation: ΔRX={ori_err_rx:.1f}°, ΔRY={ori_err_ry:.1f}°, ΔRZ={ori_err_rz:.1f}° (total: {ori_err_total:.1f}°)")
        print()

        if pos_err_total < 10.0 and ori_err_total < 10.0:
            print(f"  ✓ GOOD: Close match!")
        else:
            print(f"  ✗ Significant difference")
        print()
        print("-" * 80)
        print()

    print("=" * 80)


if __name__ == "__main__":
    test_positions()
