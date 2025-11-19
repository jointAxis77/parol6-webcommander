"""
Command Queue Module for PAROL6 Robot

Provides size-limited command queue with memory protection and statistics.
Prevents memory issues from unlimited command buffering.

Author: PAROL6 Team
Date: 2025-01-13
"""

import logging
from collections import deque
from typing import Optional, Any, List, Callable
from dataclasses import dataclass

from constants import COMMAND_QUEUE_MAX_SIZE


# ============================================================================
# Queue Statistics
# ============================================================================

@dataclass
class QueueStats:
    """Statistics for command queue"""
    total_queued: int = 0
    total_executed: int = 0
    total_dropped: int = 0
    total_cancelled: int = 0
    current_size: int = 0
    max_size: int = 0
    peak_size: int = 0  # Highest size reached


# ============================================================================
# Command Queue Class
# ============================================================================

class CommandQueue:
    """
    Size-limited FIFO command queue with memory protection.

    Features:
    - Maximum size limit to prevent memory issues
    - Separate limits for trajectory commands (memory-intensive)
    - Queue overflow detection and handling
    - Statistics tracking
    - Command filtering capabilities
    - Thread-safe operations (with external locking)
    """

    def __init__(self,
                 logger: logging.Logger,
                 max_size: int = COMMAND_QUEUE_MAX_SIZE,
                 max_trajectory_commands: int = 10):
        """
        Initialize command queue.

        Args:
            logger: Logger instance
            max_size: Maximum total commands in queue
            max_trajectory_commands: Maximum trajectory-heavy commands
                                    (SMOOTH_*, large trajectories)
        """
        self.logger = logger
        self.max_size = max_size
        self.max_trajectory_commands = max_trajectory_commands

        # Queue implementation
        self._queue = deque()

        # Statistics
        self._stats = QueueStats(max_size=max_size)

        # Trajectory command tracking
        self._trajectory_count = 0
        self._trajectory_command_types = {
            'SmoothCircleCommand',
            'SmoothArcCenterCommand',
            'SmoothArcParamCommand',
            'SmoothSplineCommand',
            'SmoothHelixCommand',
            'SmoothBlendCommand',
        }

    # ========================================================================
    # Queue Operations
    # ========================================================================

    def can_add(self, command: Any) -> tuple[bool, str]:
        """
        Check if command can be added to queue.

        Args:
            command: Command object to check

        Returns:
            Tuple of (can_add: bool, reason: str)
            - (True, "") if can add
            - (False, "reason") if cannot add

        Example:
            can_add, reason = queue.can_add(cmd)
            if not can_add:
                logger.warning(f"Cannot queue command: {reason}")
        """
        # Check total size limit
        if len(self._queue) >= self.max_size:
            return False, f"Queue full ({self.max_size} commands)"

        # Check trajectory command limit
        if self._is_trajectory_command(command):
            if self._trajectory_count >= self.max_trajectory_commands:
                return False, f"Too many trajectory commands ({self.max_trajectory_commands} max)"

        return True, ""

    def add(self, command: Any) -> bool:
        """
        Add command to queue (if space available).

        Args:
            command: Command object to add

        Returns:
            True if added successfully, False if queue full

        Example:
            if queue.add(cmd):
                logger.info("Command queued")
            else:
                logger.warning("Queue full, command dropped")
        """
        can_add, reason = self.can_add(command)

        if not can_add:
            self.logger.warning(f"[CommandQueue] Cannot add command: {reason}")
            self._stats.total_dropped += 1
            return False

        # Add to queue
        self._queue.append(command)
        self._stats.total_queued += 1
        self._stats.current_size = len(self._queue)

        # Update peak size
        if self._stats.current_size > self._stats.peak_size:
            self._stats.peak_size = self._stats.current_size

        # Track trajectory commands
        if self._is_trajectory_command(command):
            self._trajectory_count += 1

        self.logger.debug(f"[CommandQueue] Command added (size: {self.size}/{self.max_size})")
        return True

    def pop(self) -> Optional[Any]:
        """
        Remove and return next command from queue (FIFO).

        Returns:
            Command object or None if queue empty

        Example:
            cmd = queue.pop()
            if cmd:
                execute_command(cmd)
        """
        if not self._queue:
            return None

        command = self._queue.popleft()
        self._stats.total_executed += 1
        self._stats.current_size = len(self._queue)

        # Update trajectory count
        if self._is_trajectory_command(command):
            self._trajectory_count = max(0, self._trajectory_count - 1)

        return command

    def peek(self) -> Optional[Any]:
        """
        View next command without removing it.

        Returns:
            Command object or None if queue empty
        """
        return self._queue[0] if self._queue else None

    def clear(self, cancel_callback: Optional[Callable[[Any], None]] = None):
        """
        Clear all commands from queue.

        Args:
            cancel_callback: Optional function to call for each cancelled command
                           Signature: cancel_callback(command)

        Example:
            def on_cancel(cmd):
                send_ack(cmd.id, "CANCELLED", "Queue cleared")

            queue.clear(cancel_callback=on_cancel)
        """
        count = len(self._queue)

        # Call cancel callback for each command
        if cancel_callback:
            for command in self._queue:
                try:
                    cancel_callback(command)
                except Exception as e:
                    self.logger.error(f"[CommandQueue] Cancel callback error: {e}")

        # Clear queue
        self._queue.clear()
        self._trajectory_count = 0
        self._stats.total_cancelled += count
        self._stats.current_size = 0

        if count > 0:
            self.logger.info(f"[CommandQueue] Cleared {count} commands")

    def remove(self, command: Any) -> bool:
        """
        Remove specific command from queue.

        Args:
            command: Command object to remove

        Returns:
            True if removed, False if not found

        Example:
            if queue.remove(cmd):
                logger.info("Command removed from queue")
        """
        try:
            self._queue.remove(command)
            self._stats.total_cancelled += 1
            self._stats.current_size = len(self._queue)

            # Update trajectory count
            if self._is_trajectory_command(command):
                self._trajectory_count = max(0, self._trajectory_count - 1)

            return True
        except ValueError:
            return False

    # ========================================================================
    # Queue Inspection
    # ========================================================================

    @property
    def size(self) -> int:
        """Get current queue size"""
        return len(self._queue)

    @property
    def is_empty(self) -> bool:
        """Check if queue is empty"""
        return len(self._queue) == 0

    @property
    def is_full(self) -> bool:
        """Check if queue is at maximum capacity"""
        return len(self._queue) >= self.max_size

    @property
    def available_slots(self) -> int:
        """Get number of available queue slots"""
        return self.max_size - len(self._queue)

    @property
    def trajectory_count(self) -> int:
        """Get number of trajectory commands in queue"""
        return self._trajectory_count

    def get_all_commands(self) -> List[Any]:
        """
        Get list of all queued commands (without removing them).

        Returns:
            List of command objects (oldest to newest)
        """
        return list(self._queue)

    def get_command_types(self) -> List[str]:
        """
        Get list of command type names in queue.

        Returns:
            List of class names (e.g., ["MoveJointCommand", "HomeCommand"])
        """
        return [type(cmd).__name__ for cmd in self._queue]

    # ========================================================================
    # Helper Methods
    # ========================================================================

    def _is_trajectory_command(self, command: Any) -> bool:
        """
        Check if command is trajectory-based (memory-intensive).

        Args:
            command: Command object

        Returns:
            True if trajectory command, False otherwise
        """
        command_type = type(command).__name__
        return command_type in self._trajectory_command_types

    # ========================================================================
    # Statistics
    # ========================================================================

    def get_stats(self) -> dict:
        """
        Get queue statistics.

        Returns:
            Dictionary with statistics

        Example:
            stats = queue.get_stats()
            print(f"Queue: {stats['current_size']}/{stats['max_size']}")
            print(f"Peak: {stats['peak_size']}")
            print(f"Dropped: {stats['total_dropped']}")
        """
        return {
            'current_size': self._stats.current_size,
            'max_size': self._stats.max_size,
            'peak_size': self._stats.peak_size,
            'available_slots': self.available_slots,
            'trajectory_count': self._trajectory_count,
            'max_trajectory_commands': self.max_trajectory_commands,
            'total_queued': self._stats.total_queued,
            'total_executed': self._stats.total_executed,
            'total_dropped': self._stats.total_dropped,
            'total_cancelled': self._stats.total_cancelled,
            'is_full': self.is_full,
        }

    def reset_stats(self):
        """Reset statistics counters (but not current queue)"""
        self._stats.total_queued = 0
        self._stats.total_executed = 0
        self._stats.total_dropped = 0
        self._stats.total_cancelled = 0
        # Note: peak_size is NOT reset, it's a lifetime metric

    def get_stats_object(self) -> QueueStats:
        """Get QueueStats dataclass (copy)"""
        stats_copy = QueueStats(
            total_queued=self._stats.total_queued,
            total_executed=self._stats.total_executed,
            total_dropped=self._stats.total_dropped,
            total_cancelled=self._stats.total_cancelled,
            current_size=self._stats.current_size,
            max_size=self._stats.max_size,
            peak_size=self._stats.peak_size
        )
        return stats_copy

    # ========================================================================
    # String Representation
    # ========================================================================

    def __repr__(self) -> str:
        """String representation for debugging"""
        return (f"CommandQueue(size={self.size}/{self.max_size}, "
                f"trajectory={self._trajectory_count}/{self.max_trajectory_commands}, "
                f"dropped={self._stats.total_dropped})")

    def __len__(self) -> int:
        """Support len(queue) syntax"""
        return len(self._queue)

    def __bool__(self) -> bool:
        """Support if queue: syntax"""
        return not self.is_empty


# ============================================================================
# Module Metadata
# ============================================================================

__version__ = "1.0.0"
__author__ = "PAROL6 Team"
__date__ = "2025-01-13"
__description__ = "Command queue with size limits for PAROL6 control system"
