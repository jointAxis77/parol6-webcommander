#!/usr/bin/env python3
"""
Simple DH Robot Plotter using matplotlib

Creates a 3D plot of the robot and saves to PNG.
Works over SSH / headless systems.
"""

import sys
import numpy as np
from pathlib import Path
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Add lib to path
sys.path.insert(0, str(Path(__file__).parent))

# Apply numpy patch first
from lib.utils.numpy_patch import *

from lib.kinematics.robot_model import robot as dhrobot

def plot_robot(joints_rad, title="DH Robot", filename="robot_plot.png"):
    """Plot robot configuration and save to file"""

    # Get link frames for all joints
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Compute FK for each link
    n_joints = len(joints_rad)
    positions = []

    # Base position
    positions.append([0, 0, 0])

    # Get position of each joint
    for i in range(n_joints):
        T = dhrobot.fkine(joints_rad, end=dhrobot.links[i])
        positions.append(T.t)

    # Get TCP position (with tool offset if any)
    T_tcp = dhrobot.fkine(joints_rad)
    positions.append(T_tcp.t)

    positions = np.array(positions)

    # Plot links
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2],
            'o-', linewidth=3, markersize=8, color='blue', label='Links')

    # Plot base
    ax.scatter([0], [0], [0], c='green', s=200, marker='s', label='Base')

    # Plot TCP
    ax.scatter([positions[-1, 0]], [positions[-1, 1]], [positions[-1, 2]],
               c='red', s=200, marker='*', label='TCP')

    # Add joint labels
    for i, pos in enumerate(positions[:-1]):
        ax.text(pos[0], pos[1], pos[2], f'  J{i}', fontsize=8)

    # TCP coordinates
    tcp_pos = T_tcp.t * 1000
    tcp_ori = np.rad2deg(T_tcp.rpy(order='xyz'))

    # Set labels and title
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(title + f'\nTCP: X={tcp_pos[0]:.1f}, Y={tcp_pos[1]:.1f}, Z={tcp_pos[2]:.1f} mm\n' +
                 f'Orient: RX={tcp_ori[0]:.1f}°, RY={tcp_ori[1]:.1f}°, RZ={tcp_ori[2]:.1f}°',
                 fontsize=10)

    # Equal aspect ratio
    max_range = np.array([positions[:, 0].max()-positions[:, 0].min(),
                          positions[:, 1].max()-positions[:, 1].min(),
                          positions[:, 2].max()-positions[:, 2].min()]).max() / 2.0

    mid_x = (positions[:, 0].max()+positions[:, 0].min()) * 0.5
    mid_y = (positions[:, 1].max()+positions[:, 1].min()) * 0.5
    mid_z = (positions[:, 2].max()+positions[:, 2].min()) * 0.5

    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    ax.legend()
    ax.grid(True)

    # Save to file
    plt.savefig(filename, dpi=150, bbox_inches='tight')
    print(f"Saved plot to: {filename}")

    return tcp_pos, tcp_ori


def main():
    """Plot robot at test positions"""

    print("=" * 80)
    print("DH Robot Plotter (matplotlib)")
    print("=" * 80)
    print()

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

    # Plot each position
    for i, pos in enumerate(positions):
        joints_deg = pos['joints']
        joints_rad = np.deg2rad(joints_deg)

        filename = f"robot_plot_pos{i+1}.png"

        print(f"{pos['name']}:")
        print(f"  Joints (deg): {joints_deg}")

        tcp_pos, tcp_ori = plot_robot(joints_rad, pos['name'], filename)

        print(f"  DHRobot FK: X={tcp_pos[0]:.1f}, Y={tcp_pos[1]:.1f}, Z={tcp_pos[2]:.1f} mm")
        print(f"              RX={tcp_ori[0]:.1f}°, RY={tcp_ori[1]:.1f}°, RZ={tcp_ori[2]:.1f}°")

        if pos['expected']:
            exp = pos['expected']
            pos_err = np.sqrt((tcp_pos[0]-exp['x'])**2 + (tcp_pos[1]-exp['y'])**2 + (tcp_pos[2]-exp['z'])**2)
            print(f"  Expected:   X={exp['x']:.1f}, Y={exp['y']:.1f}, Z={exp['z']:.1f} mm")
            print(f"              RX={exp['rx']:.1f}°, RY={exp['ry']:.1f}°, RZ={exp['rz']:.1f}°")
            print(f"  Error: {pos_err:.1f} mm")

        print()

    print("=" * 80)
    print("All plots saved!")
    print("=" * 80)


if __name__ == "__main__":
    main()
