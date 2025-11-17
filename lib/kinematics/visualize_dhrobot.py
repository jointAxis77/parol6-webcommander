#!/usr/bin/env python3
"""
Simple DH Robot Visualizer using Swift

Usage:
  python3 visualize_dhrobot.py

Controls:
  - Drag to rotate view
  - Scroll to zoom
  - Displays joint angles and TCP coordinates
"""

import sys
import numpy as np
from pathlib import Path

# Add lib to path
sys.path.insert(0, str(Path(__file__).parent))

# Apply numpy patch first
from lib.utils.numpy_patch import *

from swift import Swift
from lib.kinematics.robot_model import robot as dhrobot

def visualize_positions():
    """Visualize robot at different positions"""

    # Test positions
    positions = [
        {
            "name": "Position 1 (Home)",
            "joints": [90.0, -90.0, 180.0, 0.0, 0.0, 180.0],
            "expected": {"x": 42.0, "y": 262.6, "z": 334.0, "rx": 0.0, "ry": -90.0, "rz": -270.0}
        },
        {
            "name": "Position 2",
            "joints": [90.0, -145.0, 107.9, 0.0, 0.0, 180.0],
            "expected": {"x": 42.0, "y": 117.3, "z": 184.9, "rx": -17.1, "ry": -90.0, "rz": -270.0}
        },
        {
            "name": "All Zeros",
            "joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "expected": None
        }
    ]

    print("=" * 80)
    print("DH Robot Visualizer")
    print("=" * 80)
    print()
    print("Available positions:")
    for i, pos in enumerate(positions):
        print(f"  {i}: {pos['name']}")
    print()

    # Choose position
    try:
        choice = int(input(f"Select position (0-{len(positions)-1}): "))
        if choice < 0 or choice >= len(positions):
            print("Invalid choice, using position 0")
            choice = 0
    except:
        print("Invalid input, using position 0")
        choice = 0

    selected = positions[choice]
    joints_deg = selected['joints']
    joints_rad = np.deg2rad(joints_deg)

    print()
    print(f"Visualizing: {selected['name']}")
    print(f"Joint angles (deg): {joints_deg}")
    print()

    # Compute FK
    T = dhrobot.fkine(joints_rad)
    pos_mm = T.t * 1000.0
    ori_deg = np.rad2deg(T.rpy(order='xyz'))

    print(f"DHRobot FK Result:")
    print(f"  Position: X={pos_mm[0]:.1f}, Y={pos_mm[1]:.1f}, Z={pos_mm[2]:.1f} mm")
    print(f"  Orientation: RX={ori_deg[0]:.1f}, RY={ori_deg[1]:.1f}, RZ={ori_deg[2]:.1f} deg")
    print()

    if selected['expected']:
        exp = selected['expected']
        print(f"Expected (actual system):")
        print(f"  Position: X={exp['x']:.1f}, Y={exp['y']:.1f}, Z={exp['z']:.1f} mm")
        print(f"  Orientation: RX={exp['rx']:.1f}, RY={exp['ry']:.1f}, RZ={exp['rz']:.1f} deg")
        print()

        # Calculate error
        pos_err = np.sqrt((pos_mm[0]-exp['x'])**2 + (pos_mm[1]-exp['y'])**2 + (pos_mm[2]-exp['z'])**2)
        print(f"  Position error: {pos_err:.1f} mm")
        print()

    # Launch Swift visualizer
    print("Launching Swift visualizer...")
    print("Close the visualizer window to exit.")
    print()

    env = Swift()
    env.launch(realtime=True)

    # Add robot to environment
    env.add(dhrobot)

    # Set robot configuration
    dhrobot.q = joints_rad

    # Keep visualizer open
    try:
        while True:
            env.step(0.05)
    except KeyboardInterrupt:
        print("\nClosing visualizer...")
    finally:
        env.close()


if __name__ == "__main__":
    visualize_positions()
