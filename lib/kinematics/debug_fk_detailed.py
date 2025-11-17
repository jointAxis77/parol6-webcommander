#!/usr/bin/env python3
"""
Detailed FK debugging - show all link transforms
"""

import sys
import numpy as np
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Add lib to path
sys.path.insert(0, str(Path(__file__).parent))

# Apply numpy patch first
from lib.utils.numpy_patch import *

from lib.kinematics.robot_model import robot as dhrobot

def debug_fk(joints_deg):
    """Detailed FK analysis"""

    print("=" * 80)
    print("DETAILED FORWARD KINEMATICS DEBUG")
    print("=" * 80)
    print()

    joints_rad = np.deg2rad(joints_deg)

    print(f"Joint angles (degrees): {joints_deg}")
    print(f"Joint angles (radians): {joints_rad}")
    print()

    print("=" * 80)
    print("DH Parameters:")
    print("=" * 80)
    for i, link in enumerate(dhrobot.links):
        print(f"Link {i}: a={link.a:.4f}, d={link.d:.4f}, alpha={link.alpha:.4f}, offset={link.offset:.4f}")
    print()

    print("=" * 80)
    print("Link-by-Link FK (Base to Link i):")
    print("=" * 80)

    positions = [[0, 0, 0]]  # Base

    # Compute cumulative transforms manually
    from spatialmath import SE3
    T_cumulative = SE3()  # Identity at base

    for i in range(len(dhrobot.links)):
        # Get individual link transform for joint i
        T_link = dhrobot.links[i].A(joints_rad[i])
        T_cumulative = T_cumulative * T_link

        pos = T_cumulative.t
        positions.append(pos.tolist())

        rpy = np.rad2deg(T_cumulative.rpy(order='xyz'))

        print(f"Link {i} (J{i+1}):")
        print(f"  Position: X={pos[0]*1000:.1f}, Y={pos[1]*1000:.1f}, Z={pos[2]*1000:.1f} mm")
        print(f"  Orientation: RX={rpy[0]:.1f}°, RY={rpy[1]:.1f}°, RZ={rpy[2]:.1f}°")
        print()

    print("=" * 80)
    print("TCP (End-Effector with tool offset):")
    print("=" * 80)

    T_tcp = dhrobot.fkine(joints_rad)
    tcp_pos = T_tcp.t
    tcp_rpy = np.rad2deg(T_tcp.rpy(order='xyz'))
    positions.append(tcp_pos.tolist())

    print(f"Position: X={tcp_pos[0]*1000:.1f}, Y={tcp_pos[1]*1000:.1f}, Z={tcp_pos[2]*1000:.1f} mm")
    print(f"Orientation: RX={tcp_rpy[0]:.1f}°, RY={tcp_rpy[1]:.1f}°, RZ={tcp_rpy[2]:.1f}°")
    print()

    # Check if tool is set
    has_tool = dhrobot.tool is not None
    print(f"Tool transform set: {has_tool}")
    if has_tool:
        tool_pos = dhrobot.tool.t
        tool_rpy = np.rad2deg(dhrobot.tool.rpy(order='xyz'))
        print(f"Tool offset: X={tool_pos[0]*1000:.1f}, Y={tool_pos[1]*1000:.1f}, Z={tool_pos[2]*1000:.1f} mm")
        print(f"Tool orient: RX={tool_rpy[0]:.1f}°, RY={tool_rpy[1]:.1f}°, RZ={tool_rpy[2]:.1f}°")
    print()

    # Plot
    print("=" * 80)
    print("Creating visualization...")
    print("=" * 80)

    positions = np.array(positions)

    fig = plt.figure(figsize=(16, 12))

    # 3D plot
    ax1 = fig.add_subplot(221, projection='3d')
    ax1.plot(positions[:, 0], positions[:, 1], positions[:, 2],
             'o-', linewidth=2, markersize=10, color='blue')

    # Add labels for each joint
    for i, pos in enumerate(positions):
        if i == 0:
            label = 'Base'
            ax1.scatter([pos[0]], [pos[1]], [pos[2]], c='green', s=200, marker='s')
        elif i == len(positions) - 1:
            label = 'TCP'
            ax1.scatter([pos[0]], [pos[1]], [pos[2]], c='red', s=200, marker='*')
        else:
            label = f'J{i}'

        ax1.text(pos[0], pos[1], pos[2], f'  {label}', fontsize=9, weight='bold')

    ax1.set_xlabel('X (m)', fontsize=10, weight='bold')
    ax1.set_ylabel('Y (m)', fontsize=10, weight='bold')
    ax1.set_zlabel('Z (m)', fontsize=10, weight='bold')
    ax1.set_title('3D View', fontsize=12, weight='bold')
    ax1.grid(True, alpha=0.3)

    # XY view
    ax2 = fig.add_subplot(222)
    ax2.plot(positions[:, 0], positions[:, 1], 'o-', linewidth=2, markersize=8, color='blue')
    ax2.scatter([positions[0, 0]], [positions[0, 1]], c='green', s=200, marker='s', label='Base')
    ax2.scatter([positions[-1, 0]], [positions[-1, 1]], c='red', s=200, marker='*', label='TCP')
    for i, pos in enumerate(positions):
        label = 'B' if i == 0 else ('TCP' if i == len(positions)-1 else f'J{i}')
        ax2.text(pos[0], pos[1], f'  {label}', fontsize=8)
    ax2.set_xlabel('X (m)', fontsize=10, weight='bold')
    ax2.set_ylabel('Y (m)', fontsize=10, weight='bold')
    ax2.set_title('Top View (XY)', fontsize=12, weight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.axis('equal')
    ax2.legend()

    # XZ view
    ax3 = fig.add_subplot(223)
    ax3.plot(positions[:, 0], positions[:, 2], 'o-', linewidth=2, markersize=8, color='blue')
    ax3.scatter([positions[0, 0]], [positions[0, 2]], c='green', s=200, marker='s', label='Base')
    ax3.scatter([positions[-1, 0]], [positions[-1, 2]], c='red', s=200, marker='*', label='TCP')
    for i, pos in enumerate(positions):
        label = 'B' if i == 0 else ('TCP' if i == len(positions)-1 else f'J{i}')
        ax3.text(pos[0], pos[2], f'  {label}', fontsize=8)
    ax3.set_xlabel('X (m)', fontsize=10, weight='bold')
    ax3.set_ylabel('Z (m)', fontsize=10, weight='bold')
    ax3.set_title('Front View (XZ)', fontsize=12, weight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.axis('equal')
    ax3.legend()

    # YZ view
    ax4 = fig.add_subplot(224)
    ax4.plot(positions[:, 1], positions[:, 2], 'o-', linewidth=2, markersize=8, color='blue')
    ax4.scatter([positions[0, 1]], [positions[0, 2]], c='green', s=200, marker='s', label='Base')
    ax4.scatter([positions[-1, 1]], [positions[-1, 2]], c='red', s=200, marker='*', label='TCP')
    for i, pos in enumerate(positions):
        label = 'B' if i == 0 else ('TCP' if i == len(positions)-1 else f'J{i}')
        ax4.text(pos[1], pos[2], f'  {label}', fontsize=8)
    ax4.set_xlabel('Y (m)', fontsize=10, weight='bold')
    ax4.set_ylabel('Z (m)', fontsize=10, weight='bold')
    ax4.set_title('Side View (YZ)', fontsize=12, weight='bold')
    ax4.grid(True, alpha=0.3)
    ax4.axis('equal')
    ax4.legend()

    plt.suptitle(f'DH Robot FK Debug\nJoints: {joints_deg}\nTCP: X={tcp_pos[0]*1000:.1f}, Y={tcp_pos[1]*1000:.1f}, Z={tcp_pos[2]*1000:.1f} mm',
                 fontsize=14, weight='bold')

    plt.tight_layout()

    filename = "fk_debug_detailed.png"
    plt.savefig(filename, dpi=150, bbox_inches='tight')
    print(f"Saved detailed plot to: {filename}")
    print()


if __name__ == "__main__":
    # Test position from user
    joints = [90.0, -90.0, 180.0, 0.0, 0.0, 180.0]
    debug_fk(joints)
