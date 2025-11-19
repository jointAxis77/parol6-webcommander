"""
Logging Conventions and Helper Utilities for PAROL6 Commander

This module provides standardized logging utilities to ensure consistent
logging patterns across the codebase.

Author: PAROL6 Team
Date: 2025-01-12
"""

import logging
from typing import Optional


# ============================================================================
# Logging Conventions
# ============================================================================
"""
STANDARDIZED LOGGING FORMAT:

All log messages should follow this pattern:
    logger.level(f"[ModuleName] Message with {variables}")

Examples:
    logger.info(f"[HomeCommand] Homing sequence complete")
    logger.debug(f"[JogCommand] Preparing execution: joint={joint}, speed={speed}%")
    logger.warning(f"[MultiJogCommand] Limit reached on joint {i+1}")
    logger.error(f"[CartesianJogCommand] IK failed: {error_message}")

RULES:
1. Use module/class name in square brackets [ClassName] at the start
2. Use f-strings for variable interpolation
3. Use consistent verb tense: present continuous for actions ("Preparing..."),
   past tense for completed actions ("Prepared", "Finished")
4. Include relevant context variables in log messages
5. NEVER use print() - always use logger
6. Log levels:
   - DEBUG: Detailed execution flow, variable values (use sparingly in hot paths)
   - INFO: Important state changes, command start/completion
   - WARNING: Unexpected conditions that don't prevent operation
   - ERROR: Failures that prevent command execution
   - CRITICAL: System-level failures (E-stop, hardware disconnection)

MESSAGE PATTERNS:
- Initialization: "[ClassName] Initializing {operation}: param1={value1}, param2={value2}"
- Preparation: "[ClassName] Preparing execution: context={details}"
- Validation: "[ClassName] Validation failed: reason={reason}"
- Execution: "[ClassName] Executing step {n}/{total}"
- Completion: "[ClassName] Execution finished: elapsed={time:.2f}s"
- Errors: "[ClassName] Error in {operation}: {error_message}"
"""


# ============================================================================
# Module Name Registry
# ============================================================================

# Command class names (for consistency)
class LogModules:
    """Standardized module names for logging"""

    # Main system components
    MAIN = "Main"
    NETWORK = "Network"
    SERIAL = "Serial"
    IK_SOLVER = "IKSolver"

    # Commands
    HOME = "HomeCommand"
    JOG = "JogCommand"
    MULTI_JOG = "MultiJogCommand"
    CARTESIAN_JOG = "CartesianJogCommand"
    MOVE_JOINT = "MoveJointCommand"
    MOVE_POSE = "MovePoseCommand"
    MOVE_CART = "MoveCartCommand"
    GRIPPER = "GripperCommand"
    DELAY = "DelayCommand"

    # Smooth motion commands
    SMOOTH_CIRCLE = "SmoothCircle"
    SMOOTH_ARC = "SmoothArc"
    SMOOTH_SPLINE = "SmoothSpline"
    SMOOTH_HELIX = "SmoothHelix"
    SMOOTH_BLEND = "SmoothBlend"

    # Utilities
    TRAJECTORY = "Trajectory"
    VALIDATION = "Validation"
    PARSER = "CommandParser"


# ============================================================================
# Logging Helper Functions
# ============================================================================

def log_command_init(logger: logging.Logger, module_name: str, operation: str, **kwargs):
    """
    Log command initialization with standardized format.

    Args:
        logger: Logger instance
        module_name: Name of the module/command (use LogModules constants)
        operation: Description of operation being initialized
        **kwargs: Key-value pairs to include in log message

    Example:
        log_command_init(logger, LogModules.JOG, "Jog",
                        joint=1, speed=50, duration=2.0)
        Output: "[JogCommand] Initializing Jog: joint=1, speed=50, duration=2.0"
    """
    params = ", ".join(f"{k}={v}" for k, v in kwargs.items())
    if params:
        logger.info(f"[{module_name}] Initializing {operation}: {params}")
    else:
        logger.info(f"[{module_name}] Initializing {operation}")


def log_command_prepare(logger: logging.Logger, module_name: str, **kwargs):
    """
    Log command preparation with standardized format.

    Args:
        logger: Logger instance
        module_name: Name of the module/command (use LogModules constants)
        **kwargs: Key-value pairs to include in log message

    Example:
        log_command_prepare(logger, LogModules.MOVE_JOINT,
                           trajectory_points=100, duration=2.5)
        Output: "[MoveJointCommand] Preparing execution: trajectory_points=100, duration=2.5"
    """
    params = ", ".join(f"{k}={v}" for k, v in kwargs.items())
    if params:
        logger.debug(f"[{module_name}] Preparing execution: {params}")
    else:
        logger.debug(f"[{module_name}] Preparing execution")


def log_command_complete(logger: logging.Logger, module_name: str, operation: Optional[str] = None, **kwargs):
    """
    Log command completion with standardized format.

    Args:
        logger: Logger instance
        module_name: Name of the module/command (use LogModules constants)
        operation: Optional operation description (defaults to generic "Execution")
        **kwargs: Key-value pairs to include in log message (e.g., elapsed_time)

    Example:
        log_command_complete(logger, LogModules.HOME, "Homing")
        Output: "[HomeCommand] Homing finished"

        log_command_complete(logger, LogModules.MOVE_POSE, elapsed_time=2.34)
        Output: "[MovePoseCommand] Execution finished: elapsed_time=2.34"
    """
    operation = operation or "Execution"
    params = ", ".join(f"{k}={v}" for k, v in kwargs.items())
    if params:
        logger.info(f"[{module_name}] {operation} finished: {params}")
    else:
        logger.info(f"[{module_name}] {operation} finished")


def log_validation_error(logger: logging.Logger, module_name: str, reason: str, **kwargs):
    """
    Log validation failure with standardized format.

    Args:
        logger: Logger instance
        module_name: Name of the module/command (use LogModules constants)
        reason: Description of validation failure
        **kwargs: Additional context to include

    Example:
        log_validation_error(logger, LogModules.JOG,
                           "Target position out of limits",
                           target=180, limit=170)
        Output: "[JogCommand] Validation failed: Target position out of limits (target=180, limit=170)"
    """
    params = ", ".join(f"{k}={v}" for k, v in kwargs.items())
    if params:
        logger.error(f"[{module_name}] Validation failed: {reason} ({params})")
    else:
        logger.error(f"[{module_name}] Validation failed: {reason}")


def log_error(logger: logging.Logger, module_name: str, operation: str, error: str, **kwargs):
    """
    Log error with standardized format.

    Args:
        logger: Logger instance
        module_name: Name of the module/command (use LogModules constants)
        operation: Operation that failed
        error: Error message
        **kwargs: Additional context

    Example:
        log_error(logger, LogModules.IK_SOLVER, "solve",
                 "Failed to converge", iterations=100, residual=0.01)
        Output: "[IKSolver] Error in solve: Failed to converge (iterations=100, residual=0.01)"
    """
    params = ", ".join(f"{k}={v}" for k, v in kwargs.items())
    if params:
        logger.error(f"[{module_name}] Error in {operation}: {error} ({params})")
    else:
        logger.error(f"[{module_name}] Error in {operation}: {error}")


def log_warning(logger: logging.Logger, module_name: str, message: str, **kwargs):
    """
    Log warning with standardized format.

    Args:
        logger: Logger instance
        module_name: Name of the module/command (use LogModules constants)
        message: Warning message
        **kwargs: Additional context

    Example:
        log_warning(logger, LogModules.MULTI_JOG,
                   "Limit reached on joint", joint=2)
        Output: "[MultiJogCommand] Warning: Limit reached on joint (joint=2)"
    """
    params = ", ".join(f"{k}={v}" for k, v in kwargs.items())
    if params:
        logger.warning(f"[{module_name}] Warning: {message} ({params})")
    else:
        logger.warning(f"[{module_name}] Warning: {message}")


def log_debug(logger: logging.Logger, module_name: str, message: str, **kwargs):
    """
    Log debug message with standardized format.

    Args:
        logger: Logger instance
        module_name: Name of the module/command (use LogModules constants)
        message: Debug message
        **kwargs: Additional context

    Example:
        log_debug(logger, LogModules.TRAJECTORY,
                 "Generated trajectory", points=250, duration=2.5)
        Output: "[Trajectory] Debug: Generated trajectory (points=250, duration=2.5)"
    """
    params = ", ".join(f"{k}={v}" for k, v in kwargs.items())
    if params:
        logger.debug(f"[{module_name}] {message} ({params})")
    else:
        logger.debug(f"[{module_name}] {message}")


# ============================================================================
# Context Manager for Timed Operations
# ============================================================================

class LoggedOperation:
    """
    Context manager for logging operation start/end with timing.

    Example:
        with LoggedOperation(logger, LogModules.IK_SOLVER, "IK solve"):
            result = solve_ik(...)

        Output:
        [IKSolver] Starting IK solve...
        [IKSolver] Completed IK solve (elapsed=0.003s)
    """

    def __init__(self, logger: logging.Logger, module_name: str, operation: str, level: str = "info"):
        self.logger = logger
        self.module_name = module_name
        self.operation = operation
        self.level = level
        self.start_time = None

    def __enter__(self):
        import time
        self.start_time = time.perf_counter()
        log_method = getattr(self.logger, self.level)
        log_method(f"[{self.module_name}] Starting {self.operation}...")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        import time
        elapsed = time.perf_counter() - self.start_time

        if exc_type is None:
            log_method = getattr(self.logger, self.level)
            log_method(f"[{self.module_name}] Completed {self.operation} (elapsed={elapsed:.3f}s)")
        else:
            self.logger.error(f"[{self.module_name}] Failed {self.operation} after {elapsed:.3f}s: {exc_val}")

        return False  # Don't suppress exceptions


# ============================================================================
# Module Metadata
# ============================================================================

__version__ = "1.0.0"
__author__ = "PAROL6 Team"
__date__ = "2025-01-12"
__description__ = "Logging conventions and utilities for PAROL6 robot control system"
