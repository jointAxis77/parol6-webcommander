#!/usr/bin/env python3
"""
Debug TCP offset - compare FK with and without tool transform
"""

import sys
import numpy as np
from pathlib import Path

# Add lib to path
sys.path.insert(0, str(Path(__file__).parent))

# Apply numpy patch first
from lib.utils.numpy_patch import *

from spatialmath import SE3
from lib.kinematics.robot_model import robot as dhrobot

def debug_tcp():
    """Debug TCP offset by testing with/without tool"""

    print("=" * 80)
    print("Debugging TCP Offset")
    print("=" * 80)
    print()

    # Test "Home" position
    joints_deg = [90, -90, 180, 0, 0, 180]
    joints_rad = np.deg2rad(joints_deg)

    print(f"Joint angles (deg): {joints_deg}")
    print()

    # Test 1: FK without tool
    print("1. FK WITHOUT tool transform:")
    saved_tool = dhrobot.tool
    dhrobot.tool = SE3()  # Identity (no tool offset)

    T_no_tool = dhrobot.fkine(joints_rad)
    pos_no_tool = T_no_tool.t * 1000.0
    ori_no_tool = np.rad2deg(T_no_tool.rpy(order='xyz'))

    print(f"   Position: X={pos_no_tool[0]:.1f}, Y={pos_no_tool[1]:.1f}, Z={pos_no_tool[2]:.1f} mm")
    print(f"   Orientation: RX={ori_no_tool[0]:.1f}, RY={ori_no_tool[1]:.1f}, RZ={ori_no_tool[2]:.1f} deg")
    print()

    # Test 2: FK with tool
    dhrobot.tool = saved_tool

    print("2. FK WITH tool transform:")
    T_with_tool = dhrobot.fkine(joints_rad)
    pos_with_tool = T_with_tool.t * 1000.0
    ori_with_tool = np.rad2deg(T_with_tool.rpy(order='xyz'))

    print(f"   Position: X={pos_with_tool[0]:.1f}, Y={pos_with_tool[1]:.1f}, Z={pos_with_tool[2]:.1f} mm")
    print(f"   Orientation: RX={ori_with_tool[0]:.1f}, RY={ori_with_tool[1]:.1f}, RZ={ori_with_tool[2]:.1f} deg")
    print()

    # Test 3: Show tool transform
    print("3. Tool transform itself:")
    tool_pos = saved_tool.t * 1000.0
    tool_ori = np.rad2deg(saved_tool.rpy(order='xyz'))
    print(f"   Position: X={tool_pos[0]:.1f}, Y={tool_pos[1]:.1f}, Z={tool_pos[2]:.1f} mm")
    print(f"   Orientation: RX={tool_ori[0]:.1f}, RY={tool_ori[1]:.1f}, RZ={tool_ori[2]:.1f} deg")
    print()

    # Test 4: What config.yaml says
    print("4. Config.yaml TCP offset (what frontend expects):")
    print(f"   Position: X=0.0, Y=-42.0, Z=-62.8 mm")
    print(f"   Orientation: RX=90.0, RY=180.0, RZ=0.0 deg")
    print()

    print("=" * 80)
    print("Analysis:")
    print("=" * 80)
    delta_pos = pos_with_tool - pos_no_tool
    delta_ori = ori_with_tool - ori_no_tool
    print(f"Tool transform changed position by: {delta_pos}")
    print(f"Tool transform changed orientation by: {delta_ori}")
    print()


if __name__ == "__main__":
    debug_tcp()
