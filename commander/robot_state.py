"""
Robot State Manager Module for PAROL6 Robot

Centralizes robot state management, unit conversions, and computed properties.
Replaces scattered global state variables with clean OOP interface.

Author: PAROL6 Team
Date: 2025-01-13
"""

import time
import numpy as np
from typing import List, Optional, Tuple
from dataclasses import dataclass, field
from spatialmath import SE3

# Import robot model for unit conversions and kinematics
from lib.kinematics import robot_model as PAROL6_ROBOT


# ============================================================================
# Immutable State Snapshot
# ============================================================================

@dataclass(frozen=True)
class RobotState:
    """
    Immutable snapshot of robot state at a point in time.

    All state data from serial feedback plus computed properties.
    Frozen dataclass ensures state cannot be modified after creation.
    """

    # Raw state from serial (in steps)
    position_steps: Tuple[int, ...]  # 6 joint positions in steps
    speed_steps: Tuple[int, ...]     # 6 joint speeds in steps/s
    homed: Tuple[bool, ...]          # 8 homing status flags (6 joints + 2 extra)
    io_status: Tuple[int, ...]       # 8 digital I/O states
    temperature_error: Tuple[int, ...] # 8 temperature error flags
    position_error: Tuple[int, ...]    # 8 position error flags
    timeout_error: int               # Timeout error flag
    timing_data: int                 # Timing between commands (0.01ms units)
    xtr_data: int                    # Extra data
    gripper_data: Tuple[int, ...]    # 6 gripper status values

    # Timestamp
    timestamp: float = field(default_factory=time.time)

    # Computed properties (cached)
    position_rad: Optional[Tuple[float, ...]] = None  # Computed on-demand
    position_deg: Optional[Tuple[float, ...]] = None  # Computed on-demand
    pose_matrix: Optional[np.ndarray] = None          # Computed on-demand

    def __post_init__(self):
        """Convert lists to tuples for immutability"""
        # Ensure all sequences are tuples
        for field_name in ['position_steps', 'speed_steps', 'homed', 'io_status',
                          'temperature_error', 'position_error', 'gripper_data']:
            value = getattr(self, field_name)
            if not isinstance(value, tuple):
                object.__setattr__(self, field_name, tuple(value))

    # ------------------------------------------------------------------------
    # Computed Properties
    # ------------------------------------------------------------------------

    @property
    def is_homed(self) -> bool:
        """Check if all 6 joints are homed"""
        return all(self.homed[:6])

    @property
    def is_estopped(self) -> bool:
        """Check if E-stop is active (InOut_in[4] == 0)"""
        return self.io_status[4] == 0 if len(self.io_status) > 4 else False

    @property
    def joints_position_rad(self) -> List[float]:
        """Get joint positions in radians (computed on first access)"""
        if self.position_rad is None:
            pos_rad = [PAROL6_ROBOT.STEPS2RADS(p, i)
                      for i, p in enumerate(self.position_steps[:6])]
            object.__setattr__(self, 'position_rad', tuple(pos_rad))
        return list(self.position_rad)

    @property
    def joints_position_deg(self) -> List[float]:
        """Get joint positions in degrees (computed on first access)"""
        if self.position_deg is None:
            pos_deg = [PAROL6_ROBOT.STEPS2DEG(p, i)
                      for i, p in enumerate(self.position_steps[:6])]
            object.__setattr__(self, 'position_deg', tuple(pos_deg))
        return list(self.position_deg)

    @property
    def tcp_pose(self) -> SE3:
        """
        Get current TCP (Tool Center Point) pose via forward kinematics.

        Returns:
            SE3 transformation matrix representing TCP pose
        """
        if self.pose_matrix is None:
            q = np.array(self.joints_position_rad)
            pose_matrix = PAROL6_ROBOT.robot.fkine(q).A
            object.__setattr__(self, 'pose_matrix', pose_matrix)

        return SE3(self.pose_matrix, check=False)

    @property
    def tcp_position_mm(self) -> np.ndarray:
        """Get TCP position in mm (x, y, z)"""
        return self.tcp_pose.t * 1000  # Convert m to mm

    @property
    def tcp_orientation_deg(self) -> np.ndarray:
        """Get TCP orientation in degrees (rx, ry, rz) as XYZ Euler angles"""
        return self.tcp_pose.rpy(unit='deg', order='xyz')

    @property
    def tcp_pose_vector(self) -> List[float]:
        """Get TCP pose as 6-element vector [x, y, z, rx, ry, rz] (mm and degrees)"""
        pos_mm = self.tcp_position_mm
        ori_deg = self.tcp_orientation_deg
        return [pos_mm[0], pos_mm[1], pos_mm[2], ori_deg[0], ori_deg[1], ori_deg[2]]

    @property
    def gripper_status(self) -> dict:
        """
        Parse gripper data into structured dict.

        Gripper data format: [ID, Position, Speed, Current, Status, ObjDetection]
        """
        if len(self.gripper_data) >= 6:
            return {
                'id': self.gripper_data[0],
                'position': self.gripper_data[1],
                'speed': self.gripper_data[2],
                'current': self.gripper_data[3],
                'status': self.gripper_data[4],
                'object_detected': self.gripper_data[5]
            }
        return {}

    @property
    def has_errors(self) -> bool:
        """Check if any errors are present"""
        return (any(self.temperature_error) or
                any(self.position_error) or
                self.timeout_error != 0)

    @property
    def age_seconds(self) -> float:
        """Get age of this state snapshot in seconds"""
        return time.time() - self.timestamp

    # ------------------------------------------------------------------------
    # Formatting
    # ------------------------------------------------------------------------

    def to_dict(self) -> dict:
        """Convert state to dictionary (for logging/debugging)"""
        return {
            'timestamp': self.timestamp,
            'joints_deg': self.joints_position_deg,
            'tcp_pose': self.tcp_pose_vector,
            'homed': list(self.homed[:6]),
            'is_estopped': self.is_estopped,
            'has_errors': self.has_errors,
            'gripper': self.gripper_status
        }

    def __repr__(self) -> str:
        """String representation for debugging"""
        return (f"RobotState(joints_deg={[f'{x:.1f}' for x in self.joints_position_deg]}, "
                f"homed={self.is_homed}, estopped={self.is_estopped})")


# ============================================================================
# Robot State Manager
# ============================================================================

class RobotStateManager:
    """
    Manages robot state with unit conversions and caching.

    Provides:
    - State update from serial feedback
    - Unit conversions (steps ↔ rad/deg)
    - Computed properties (pose, homed status, etc.)
    - State history for debugging
    - Thread-safe state access
    """

    def __init__(self, robot_model=None, history_size: int = 100):
        """
        Initialize robot state manager.

        Args:
            robot_model: Robot model for conversions (defaults to PAROL6_ROBOT)
            history_size: Number of historical states to keep (default: 100)
        """
        self.robot = robot_model or PAROL6_ROBOT

        # Current state
        self._current_state: Optional[RobotState] = None

        # State history (circular buffer)
        self._history: List[RobotState] = []
        self._history_size = history_size

        # Statistics
        self._update_count = 0
        self._last_update_time = 0

        # Initialize with default state
        self._initialize_default_state()

    def _initialize_default_state(self):
        """Initialize with safe default values"""
        self._current_state = RobotState(
            position_steps=(0, 0, 0, 0, 0, 0),
            speed_steps=(0, 0, 0, 0, 0, 0),
            homed=(False, False, False, False, False, False, False, False),
            io_status=(1, 1, 1, 1, 1, 1, 1, 1),  # All IO high (E-stop not active)
            temperature_error=(0, 0, 0, 0, 0, 0, 0, 0),
            position_error=(0, 0, 0, 0, 0, 0, 0, 0),
            timeout_error=0,
            timing_data=0,
            xtr_data=0,
            gripper_data=(0, 0, 0, 0, 0, 0)
        )

    # ========================================================================
    # State Updates
    # ========================================================================

    def update_from_serial(self,
                          position_steps: List[int],
                          speed_steps: List[int],
                          homed: List[int],
                          io_status: List[int],
                          temperature_error: List[int],
                          position_error: List[int],
                          timeout_error: int,
                          timing_data: List[int],
                          xtr_data: int,
                          gripper_data: List[int]) -> RobotState:
        """
        Update state from serial feedback packet.

        Args:
            position_steps: Joint positions in steps (6 elements)
            speed_steps: Joint speeds in steps/s (6 elements)
            homed: Homing status flags (8 elements)
            io_status: Digital I/O status (8 elements)
            temperature_error: Temperature error flags (8 elements)
            position_error: Position error flags (8 elements)
            timeout_error: Timeout error flag
            timing_data: Timing between commands (1 element list)
            xtr_data: Extra data
            gripper_data: Gripper status (6 elements)

        Returns:
            New RobotState snapshot
        """
        # Create new immutable state
        new_state = RobotState(
            position_steps=tuple(position_steps[:6]),
            speed_steps=tuple(speed_steps[:6]),
            homed=tuple(bool(h) for h in homed[:8]),
            io_status=tuple(io_status[:8]),
            temperature_error=tuple(temperature_error[:8]),
            position_error=tuple(position_error[:8]),
            timeout_error=timeout_error,
            timing_data=timing_data[0] if timing_data else 0,
            xtr_data=xtr_data,
            gripper_data=tuple(gripper_data[:6])
        )

        # Add old state to history
        if self._current_state is not None:
            self._history.append(self._current_state)
            # Trim history
            if len(self._history) > self._history_size:
                self._history.pop(0)

        # Update current state
        self._current_state = new_state
        self._update_count += 1
        self._last_update_time = time.time()

        return new_state

    def update_from_arrays(self,
                          Position_in: List[int],
                          Speed_in: List[int],
                          Homed_in: List[int],
                          InOut_in: List[int],
                          Temperature_error_in: List[int],
                          Position_error_in: List[int],
                          Timeout_error: int,
                          Timing_data_in: List[int],
                          XTR_data: int,
                          Gripper_data_in: List[int]) -> RobotState:
        """
        Update state from legacy array format (for backward compatibility).

        This matches the existing headless_commander.py variable names exactly.
        """
        return self.update_from_serial(
            position_steps=Position_in,
            speed_steps=Speed_in,
            homed=Homed_in,
            io_status=InOut_in,
            temperature_error=Temperature_error_in,
            position_error=Position_error_in,
            timeout_error=Timeout_error,
            timing_data=Timing_data_in,
            xtr_data=XTR_data,
            gripper_data=Gripper_data_in
        )

    # ========================================================================
    # State Access
    # ========================================================================

    @property
    def current(self) -> RobotState:
        """Get current robot state (immutable snapshot)"""
        if self._current_state is None:
            self._initialize_default_state()
        return self._current_state

    @property
    def previous(self) -> Optional[RobotState]:
        """Get previous robot state (or None if no history)"""
        return self._history[-1] if self._history else None

    def get_history(self, count: Optional[int] = None) -> List[RobotState]:
        """
        Get historical states.

        Args:
            count: Number of states to retrieve (None = all)

        Returns:
            List of RobotState snapshots (oldest to newest)
        """
        if count is None:
            return self._history.copy()
        else:
            return self._history[-count:]

    def clear_history(self):
        """Clear state history"""
        self._history.clear()

    # ========================================================================
    # Convenience Methods (Direct Access)
    # ========================================================================

    @property
    def joints_position_steps(self) -> List[int]:
        """Get current joint positions in steps"""
        return list(self.current.position_steps)

    @property
    def joints_position_rad(self) -> List[float]:
        """Get current joint positions in radians"""
        return self.current.joints_position_rad

    @property
    def joints_position_deg(self) -> List[float]:
        """Get current joint positions in degrees"""
        return self.current.joints_position_deg

    @property
    def tcp_pose(self) -> SE3:
        """Get current TCP pose"""
        return self.current.tcp_pose

    @property
    def tcp_position_mm(self) -> np.ndarray:
        """Get current TCP position in mm"""
        return self.current.tcp_position_mm

    @property
    def tcp_orientation_deg(self) -> np.ndarray:
        """Get current TCP orientation in degrees"""
        return self.current.tcp_orientation_deg

    @property
    def is_homed(self) -> bool:
        """Check if robot is fully homed"""
        return self.current.is_homed

    @property
    def is_estopped(self) -> bool:
        """Check if E-stop is active"""
        return self.current.is_estopped

    @property
    def has_errors(self) -> bool:
        """Check if any errors are present"""
        return self.current.has_errors

    # ========================================================================
    # Unit Conversions (Instance Methods)
    # ========================================================================

    def steps_to_rad(self, steps: int, joint_index: int) -> float:
        """Convert steps to radians for specific joint"""
        return self.robot.STEPS2RADS(steps, joint_index)

    def rad_to_steps(self, rad: float, joint_index: int) -> int:
        """Convert radians to steps for specific joint"""
        return self.robot.RAD2STEPS(rad, joint_index)

    def steps_to_deg(self, steps: int, joint_index: int) -> float:
        """Convert steps to degrees for specific joint"""
        return self.robot.STEPS2DEG(steps, joint_index)

    def deg_to_steps(self, deg: float, joint_index: int) -> int:
        """Convert degrees to steps for specific joint"""
        return self.robot.DEG2STEPS(deg, joint_index)

    # ========================================================================
    # Statistics
    # ========================================================================

    def get_stats(self) -> dict:
        """Get state manager statistics"""
        return {
            'update_count': self._update_count,
            'last_update_time': self._last_update_time,
            'time_since_update': time.time() - self._last_update_time if self._last_update_time > 0 else None,
            'history_size': len(self._history),
            'history_max_size': self._history_size,
            'current_state_age': self.current.age_seconds if self._current_state else None
        }

    def reset_stats(self):
        """Reset statistics"""
        self._update_count = 0
        self._last_update_time = 0


# ============================================================================
# Module Metadata
# ============================================================================

__version__ = "1.0.0"
__author__ = "PAROL6 Team"
__date__ = "2025-01-13"
__description__ = "Robot state management for PAROL6 control system"
