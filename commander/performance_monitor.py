"""
Performance Monitoring Module for PAROL6 Robot

Monitors control loop timing, identifies bottlenecks, and tracks system performance.
Critical for maintaining 100Hz control loop requirement.

Author: PAROL6 Team
Date: 2025-01-13
"""

import time
import logging
from collections import deque
from typing import Optional, Dict, Any, List
from dataclasses import dataclass, field
import numpy as np

from constants import (
    CONTROL_LOOP_HZ,
    CONTROL_INTERVAL_S,
    PERF_MONITOR_WINDOW_SIZE,
    PERF_WARNING_THRESHOLD_MS,
    PERF_CRITICAL_THRESHOLD_MS,
)


# ============================================================================
# Data Classes
# ============================================================================

@dataclass
class CycleTimings:
    """Timing breakdown for a single control loop cycle"""
    total_ms: float
    network_ms: float = 0
    command_processing_ms: float = 0
    command_execution_ms: float = 0
    serial_ms: float = 0
    timestamp: float = field(default_factory=time.time)


@dataclass
class PerformanceStats:
    """Aggregate performance statistics"""
    mean_ms: float
    median_ms: float
    std_ms: float
    min_ms: float
    max_ms: float
    p95_ms: float
    p99_ms: float
    over_budget_count: int
    over_budget_percentage: float
    samples: int


# ============================================================================
# Performance Monitor Class
# ============================================================================

class PerformanceMonitor:
    """
    Monitor control loop performance and identify bottlenecks.

    Tracks:
    - Total cycle time
    - Time spent in each phase (network, processing, execution, serial)
    - Statistical analysis (mean, median, percentiles)
    - Budget violations (cycles exceeding 10ms target)
    - Historical data for trending
    """

    def __init__(self,
                 logger: logging.Logger,
                 target_hz: int = CONTROL_LOOP_HZ,
                 window_size: int = PERF_MONITOR_WINDOW_SIZE,
                 warning_threshold_ms: float = PERF_WARNING_THRESHOLD_MS,
                 critical_threshold_ms: float = PERF_CRITICAL_THRESHOLD_MS,
                 debug_mode: bool = None,
                 collect_samples: bool = False):
        """
        Initialize performance monitor.

        Args:
            logger: Logger instance
            target_hz: Target control loop frequency (default: 100Hz)
            window_size: Number of cycles to keep in rolling window (default: 1000)
            warning_threshold_ms: Warning threshold in ms (default: 8ms)
            critical_threshold_ms: Critical threshold in ms (default: 9.5ms)
            debug_mode: Enable DEBUG logging (default: auto-detect from logger level)
            collect_samples: Enable performance sample collection (default: False)
        """
        self.logger = logger
        self.target_hz = target_hz
        self.target_interval_s = 1.0 / target_hz
        self.target_interval_ms = self.target_interval_s * 1000
        self.window_size = window_size
        self.warning_threshold_ms = warning_threshold_ms
        self.critical_threshold_ms = critical_threshold_ms

        # Check if we're in DEBUG mode (for logging only)
        if debug_mode is None:
            self._debug_mode = logger.isEnabledFor(logging.DEBUG)
        else:
            self._debug_mode = debug_mode

        # Sample collection mode (independent of debug logging)
        self._collect_samples = collect_samples

        # Lightweight Hz tracking (always on)
        self._hz_cycle_count = 0
        self._hz_last_calc_time = time.time()
        self._current_hz = 0.0

        # Detailed timing data (enabled by debug_mode OR collect_samples)
        if self._debug_mode or self._collect_samples:
            self._cycle_times = deque(maxlen=window_size)
            self._network_times = deque(maxlen=window_size)
            self._processing_times = deque(maxlen=window_size)
            self._execution_times = deque(maxlen=window_size)
            self._serial_times = deque(maxlen=window_size)
            self._ik_manipulability_times = deque(maxlen=window_size)
            self._ik_solve_times = deque(maxlen=window_size)

            # Current cycle tracking
            self._cycle_start = None
            self._phase_start = None
            self._current_timings = {}

            # Violation tracking
            self._warning_violations = 0
            self._critical_violations = 0
            self._last_violation_time = 0

            # Statistics
            self._total_cycles = 0
        else:
            # Minimal tracking in production mode (neither debug nor collecting samples)
            self._cycle_times = None
            self._network_times = None
            self._processing_times = None
            self._execution_times = None
            self._serial_times = None
            self._ik_manipulability_times = None
            self._ik_solve_times = None
            self._cycle_start = None
            self._phase_start = None
            self._current_timings = {}
            self._warning_violations = 0
            self._critical_violations = 0
            self._last_violation_time = 0
            self._total_cycles = 0

    # ========================================================================
    # Sample Collection Control
    # ========================================================================

    def enable_sample_collection(self):
        """
        Enable performance sample collection.
        Initializes deques if they don't exist yet.
        """
        self._collect_samples = True

        # Initialize deques if they don't exist
        if self._cycle_times is None:
            self._cycle_times = deque(maxlen=self.window_size)
            self._network_times = deque(maxlen=self.window_size)
            self._processing_times = deque(maxlen=self.window_size)
            self._execution_times = deque(maxlen=self.window_size)
            self._serial_times = deque(maxlen=self.window_size)
            self._ik_manipulability_times = deque(maxlen=self.window_size)
            self._ik_solve_times = deque(maxlen=self.window_size)

            # Current cycle tracking
            self._cycle_start = None
            self._phase_start = None
            self._current_timings = {}

            # Violation tracking
            self._warning_violations = 0
            self._critical_violations = 0
            self._last_violation_time = 0

            # Statistics
            self._total_cycles = 0

    def disable_sample_collection(self):
        """
        Disable performance sample collection.
        Note: This does NOT clear existing data, just stops collecting new samples.
        """
        self._collect_samples = False

    # ========================================================================
    # Cycle Timing
    # ========================================================================

    def start_cycle(self):
        """
        Mark the start of a control loop cycle.

        Call this at the very beginning of the main loop.

        Example:
            while True:
                perf.start_cycle()
                # ... loop body ...
                perf.end_cycle()
        """
        if self._debug_mode or self._collect_samples:
            self._cycle_start = time.perf_counter()
            self._current_timings = {}

    def end_cycle(self):
        """
        Mark the end of a control loop cycle and calculate statistics.

        Call this at the very end of the main loop.
        Automatically warns if cycle time exceeds thresholds.
        """
        # Always increment Hz cycle counter
        self._hz_cycle_count += 1

        # Calculate Hz every 1 second
        now = time.time()
        elapsed = now - self._hz_last_calc_time
        if elapsed >= 1.0:
            self._current_hz = self._hz_cycle_count / elapsed
            self._hz_cycle_count = 0
            self._hz_last_calc_time = now

        # Only do detailed tracking when collecting samples OR in debug mode
        if not (self._debug_mode or self._collect_samples):
            return

        if self._cycle_start is None:
            self.logger.warning("[PerfMonitor] end_cycle() called without start_cycle()")
            return

        # Calculate total cycle time
        cycle_time_s = time.perf_counter() - self._cycle_start
        cycle_time_ms = cycle_time_s * 1000

        # Store cycle time
        self._cycle_times.append(cycle_time_ms)

        # Store phase times
        self._network_times.append(self._current_timings.get('network', 0))
        self._processing_times.append(self._current_timings.get('processing', 0))
        self._execution_times.append(self._current_timings.get('execution', 0))
        self._serial_times.append(self._current_timings.get('serial', 0))
        self._ik_manipulability_times.append(self._current_timings.get('ik_manipulability', 0))
        self._ik_solve_times.append(self._current_timings.get('ik_solve', 0))

        # Increment counter
        self._total_cycles += 1

        # Check for violations
        if cycle_time_ms > self.critical_threshold_ms:
            self._critical_violations += 1
            self._last_violation_time = time.time()
            self.logger.warning(f"[PerfMonitor] CRITICAL: Cycle time {cycle_time_ms:.2f}ms "
                              f"exceeds {self.critical_threshold_ms}ms threshold "
                              f"(budget: {self.target_interval_ms:.2f}ms)")

        elif cycle_time_ms > self.warning_threshold_ms:
            self._warning_violations += 1
            self._last_violation_time = time.time()
            self.logger.debug(f"[PerfMonitor] WARNING: Cycle time {cycle_time_ms:.2f}ms "
                            f"exceeds {self.warning_threshold_ms}ms threshold")

        # Reset for next cycle
        self._cycle_start = None
        self._current_timings = {}

    # ========================================================================
    # Hz Tracking
    # ========================================================================

    def get_hz(self) -> float:
        """
        Get current control loop frequency in Hz.

        Returns:
            Current Hz (cycles per second)

        Example:
            hz = perf.get_hz()
            print(f"Running at {hz:.1f}Hz")
        """
        return self._current_hz

    # ========================================================================
    # Phase Timing
    # ========================================================================

    def start_phase(self, phase_name: str):
        """
        Mark the start of a timed phase within the cycle.

        Only active when collecting samples OR in debug mode. No-op otherwise.

        Args:
            phase_name: Phase identifier (e.g., 'network', 'execution', 'serial')

        Example:
            perf.start_phase('network')
            receive_commands()
            perf.end_phase('network')
        """
        if not (self._debug_mode or self._collect_samples):
            return

        self._phase_start = time.perf_counter()
        self._current_phase_name = phase_name

    def end_phase(self, phase_name: str):
        """
        Mark the end of a timed phase.

        Only active when collecting samples OR in debug mode. No-op otherwise.

        Args:
            phase_name: Phase identifier (must match start_phase call)
        """
        if not (self._debug_mode or self._collect_samples):
            return

        if self._phase_start is None:
            return

        if self._current_phase_name != phase_name:
            self.logger.warning(f"[PerfMonitor] Phase mismatch: "
                              f"started '{self._current_phase_name}', ended '{phase_name}'")
            return

        # Calculate phase time
        phase_time_s = time.perf_counter() - self._phase_start
        phase_time_ms = phase_time_s * 1000

        # Store phase time
        self._current_timings[phase_name] = phase_time_ms

        # Reset
        self._phase_start = None
        self._current_phase_name = None

    # ========================================================================
    # Context Manager for Timed Blocks
    # ========================================================================

    class TimedPhase:
        """Context manager for timing a code block"""

        def __init__(self, monitor: 'PerformanceMonitor', phase_name: str):
            self.monitor = monitor
            self.phase_name = phase_name

        def __enter__(self):
            self.monitor.start_phase(self.phase_name)
            return self

        def __exit__(self, exc_type, exc_val, exc_tb):
            self.monitor.end_phase(self.phase_name)
            return False

    def timed_phase(self, phase_name: str) -> 'PerformanceMonitor.TimedPhase':
        """
        Context manager for timing a phase.

        Example:
            with perf.timed_phase('network'):
                receive_commands()

            with perf.timed_phase('execution'):
                execute_command()
        """
        return self.TimedPhase(self, phase_name)

    # ========================================================================
    # Statistics
    # ========================================================================

    def get_stats(self) -> Dict[str, Any]:
        """
        Get performance statistics for current window.

        Returns:
            Dictionary with comprehensive statistics (when collecting samples or in debug mode)
            Otherwise, returns Hz only

        Example:
            stats = perf.get_stats()
            print(f"Mean cycle time: {stats['total']['mean_ms']:.2f}ms")
            print(f"Budget violations: {stats['violations']['percentage']:.1f}%")
        """
        if not (self._debug_mode or self._collect_samples):
            return {
                'hz': self._current_hz,
                'mode': 'production'
            }

        if not self._cycle_times:
            return {'error': 'No data collected yet'}

        # Convert to numpy arrays
        cycle_times = np.array(self._cycle_times)
        network_times = np.array(self._network_times)
        processing_times = np.array(self._processing_times)
        execution_times = np.array(self._execution_times)
        serial_times = np.array(self._serial_times)
        ik_manipulability_times = np.array(self._ik_manipulability_times)
        ik_solve_times = np.array(self._ik_solve_times)

        # Calculate statistics for each phase
        def calc_stats(data: np.ndarray, name: str) -> Dict[str, float]:
            over_budget = np.sum(data > self.target_interval_ms)
            return {
                f'{name}_mean_ms': float(np.mean(data)),
                f'{name}_median_ms': float(np.median(data)),
                f'{name}_std_ms': float(np.std(data)),
                f'{name}_min_ms': float(np.min(data)),
                f'{name}_max_ms': float(np.max(data)),
                f'{name}_p95_ms': float(np.percentile(data, 95)),
                f'{name}_p99_ms': float(np.percentile(data, 99)),
            }

        stats = {}
        stats.update(calc_stats(cycle_times, 'cycle'))
        stats.update(calc_stats(network_times, 'network'))
        stats.update(calc_stats(processing_times, 'processing'))
        stats.update(calc_stats(execution_times, 'execution'))
        stats.update(calc_stats(serial_times, 'serial'))
        stats.update(calc_stats(ik_manipulability_times, 'ik_manipulability'))
        stats.update(calc_stats(ik_solve_times, 'ik_solve'))

        # Violation statistics
        over_budget_count = np.sum(cycle_times > self.target_interval_ms)
        stats['violations'] = {
            'warning_count': self._warning_violations,
            'critical_count': self._critical_violations,
            'over_budget_count': int(over_budget_count),
            'over_budget_percentage': float(over_budget_count / len(cycle_times) * 100),
            'last_violation_time': self._last_violation_time,
        }

        # General info
        stats['info'] = {
            'target_hz': self.target_hz,
            'target_interval_ms': self.target_interval_ms,
            'window_size': len(self._cycle_times),
            'max_window_size': self.window_size,
            'total_cycles': self._total_cycles,
            'warning_threshold_ms': self.warning_threshold_ms,
            'critical_threshold_ms': self.critical_threshold_ms,
        }

        return stats

    def get_summary(self) -> str:
        """
        Get human-readable performance summary.

        Returns:
            Multi-line string with performance summary

        Example:
            print(perf.get_summary())
        """
        if not self._cycle_times:
            return "[PerfMonitor] No data collected yet"

        stats = self.get_stats()

        lines = [
            "[PerfMonitor] Performance Summary",
            "=" * 50,
            f"Target: {self.target_hz}Hz ({self.target_interval_ms:.2f}ms per cycle)",
            f"Samples: {self._total_cycles} total, {len(self._cycle_times)} in window",
            "",
            "Cycle Times:",
            f"  Mean:   {stats['cycle_mean_ms']:.3f}ms",
            f"  Median: {stats['cycle_median_ms']:.3f}ms",
            f"  Std:    {stats['cycle_std_ms']:.3f}ms",
            f"  Min:    {stats['cycle_min_ms']:.3f}ms",
            f"  Max:    {stats['cycle_max_ms']:.3f}ms",
            f"  P95:    {stats['cycle_p95_ms']:.3f}ms",
            f"  P99:    {stats['cycle_p99_ms']:.3f}ms",
            "",
            "Phase Breakdown (Mean):",
            f"  Network:    {stats['network_mean_ms']:.3f}ms",
            f"  Processing: {stats['processing_mean_ms']:.3f}ms",
            f"  Execution:  {stats['execution_mean_ms']:.3f}ms",
            f"  Serial:     {stats['serial_mean_ms']:.3f}ms",
            "",
            "Violations:",
            f"  Warning:  {stats['violations']['warning_count']}",
            f"  Critical: {stats['violations']['critical_count']}",
            f"  Over budget: {stats['violations']['over_budget_count']} "
            f"({stats['violations']['over_budget_percentage']:.1f}%)",
            "=" * 50,
        ]

        return "\n".join(lines)

    def print_summary(self):
        """Print performance summary to logger"""
        self.logger.info(self.get_summary())

    def reset(self):
        """
        Reset all statistics and buffers.

        Note: This clears historical data. Use with caution.
        """
        self._cycle_times.clear()
        self._network_times.clear()
        self._processing_times.clear()
        self._execution_times.clear()
        self._serial_times.clear()
        self._ik_manipulability_times.clear()
        self._ik_solve_times.clear()

        self._warning_violations = 0
        self._critical_violations = 0
        self._total_cycles = 0

        self.logger.info("[PerfMonitor] Statistics reset")

    # ========================================================================
    # Convenience Properties
    # ========================================================================

    @property
    def latest_cycle_time_ms(self) -> Optional[float]:
        """Get most recent cycle time in ms"""
        return self._cycle_times[-1] if self._cycle_times else None

    def get_latest_phase_times(self) -> Optional[Dict[str, float]]:
        """
        Get most recent cycle's phase timing breakdown.

        Returns:
            Dict with phase names and their times in ms, or None if no data
        """
        if not (self._debug_mode or self._collect_samples) or not self._cycle_times:
            return None

        return {
            'cycle': self._cycle_times[-1],
            'network': self._network_times[-1] if self._network_times else 0,
            'processing': self._processing_times[-1] if self._processing_times else 0,
            'execution': self._execution_times[-1] if self._execution_times else 0,
            'serial': self._serial_times[-1] if self._serial_times else 0,
            'ik_manipulability': self._ik_manipulability_times[-1] if self._ik_manipulability_times else 0,
            'ik_solve': self._ik_solve_times[-1] if self._ik_solve_times else 0,
            'hz': self._current_hz
        }

    @property
    def mean_cycle_time_ms(self) -> Optional[float]:
        """Get mean cycle time in ms"""
        return float(np.mean(self._cycle_times)) if self._cycle_times else None

    @property
    def is_meeting_target(self) -> bool:
        """Check if performance is meeting target (mean < target interval)"""
        mean = self.mean_cycle_time_ms
        return mean < self.target_interval_ms if mean else True

    @property
    def violation_rate(self) -> float:
        """Get violation rate (0-1)"""
        if not self._cycle_times:
            return 0.0
        over_budget = sum(1 for t in self._cycle_times if t > self.target_interval_ms)
        return over_budget / len(self._cycle_times)


# ============================================================================
# Module Metadata
# ============================================================================

__version__ = "1.0.0"
__author__ = "PAROL6 Team"
__date__ = "2025-01-13"
__description__ = "Performance monitoring for PAROL6 control loop"
