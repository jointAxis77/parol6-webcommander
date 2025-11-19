"""
PAROL6 Controller Constants

Central configuration constants for PAROL6 robot control system.
All timing values, thresholds, and magic numbers in one place.

Author: Extracted from headless_commander.py
Date: 2025-01-12
"""

# ============================================================================
# Timing Constants
# ============================================================================

# Control loop timing
CONTROL_LOOP_HZ = 100
CONTROL_INTERVAL_S = 0.01  # 1/CONTROL_LOOP_HZ = 10ms per cycle
CONTROL_INTERVAL_MS = 10   # Milliseconds per cycle

# Network command processing
COMMAND_COOLDOWN_S = 0.1   # Minimum time between processing network commands (100ms)
COMMAND_COOLDOWN_CYCLES = 10  # COMMAND_COOLDOWN_S / CONTROL_INTERVAL_S

# Performance monitoring
PERF_MONITOR_WINDOW_SIZE = 100  # Number of samples for performance statistics
PERF_WARNING_THRESHOLD_MS = 20  # Warn if cycle takes more than 20ms (2x budget)
PERF_CRITICAL_THRESHOLD_MS = 50  # Critical if cycle takes more than 50ms (5x budget)

# ============================================================================
# Timeout Constants (in control loop cycles at 100Hz)
# ============================================================================

# Gripper timeouts
GRIPPER_CALIBRATION_CYCLES = 200  # 2 seconds at 100Hz
GRIPPER_OPERATION_TIMEOUT_CYCLES = 1000  # 10 seconds for gripper operations

# Homing timeouts
HOMING_TIMEOUT_CYCLES = 2000  # 20 seconds for homing sequence
HOMING_START_CMD_CYCLES = 10  # Send home command for 10 cycles (0.1s)

# General command timeouts
DEFAULT_COMMAND_TIMEOUT_CYCLES = 1000  # 10 seconds default timeout

# ============================================================================
# IK Solver Constants
# ============================================================================

# IK solver limits
IK_MAX_ITERATIONS = 100  # Maximum iterations for IK solver
IK_MAX_SUBDIVISION_DEPTH = 4  # Maximum recursion depth for path subdivision

# IK tolerances
IK_STRICT_TOLERANCE = 1e-10  # Strict tolerance away from singularities
IK_LOOSE_TOLERANCE = 1e-7  # Loose tolerance near singularities
IK_JOGGING_TOLERANCE = 1e-10  # Fixed tolerance for jogging mode

# Singularity detection
IK_SINGULARITY_THRESHOLD = 0.001  # Manipulability threshold for singularity detection

# Damping parameters
IK_DEFAULT_DAMPING = 0.0000001  # Default damping for IK solver
IK_RECOVERY_DAMPING = 0.0000001  # Damping for recovery (inward) movements

# ============================================================================
# Robot Physical Constants
# ============================================================================

# Reach limits
BASE_MAX_REACH_M = 0.44  # Base maximum reach from experimentation (meters)
REACH_REDUCTION_AT_J5_90DEG_M = 0.045  # Reach reduction when J5 at ±90° (meters)
REACH_REDUCTION_RANGE_RAD = 0.785398  # π/4 radians = 45 degrees

# Cartesian velocity limits (for jogging and smooth motion)
CARTESIAN_LINEAR_VELOCITY_MIN_M_S = 0.005  # Minimum linear velocity (5mm/s)
CARTESIAN_LINEAR_VELOCITY_MAX_M_S = 0.100  # Maximum linear velocity (100mm/s)
CARTESIAN_LINEAR_VELOCITY_MIN_JOG_M_S = 0.010  # Minimum jog velocity (10mm/s)
CARTESIAN_LINEAR_VELOCITY_MAX_JOG_M_S = 0.050  # Maximum jog velocity (50mm/s)

# Cartesian angular velocity limits (degrees/second)
CARTESIAN_ANGULAR_VELOCITY_MIN_DEG_S = 5.0  # Minimum angular velocity
CARTESIAN_ANGULAR_VELOCITY_MAX_DEG_S = 45.0  # Maximum angular velocity

# ============================================================================
# Trajectory Generation Constants
# ============================================================================

# Trajectory sampling
TRAJECTORY_POINTS_PER_SECOND = 100  # Sample rate for trajectory generation (matches control loop)

# Motion blending
DEFAULT_BLEND_SAMPLES = 50  # Default number of samples for motion blending
MIN_BLEND_TIME_S = 0.1  # Minimum blend time (seconds)
MAX_BLEND_TIME_S = 2.0  # Maximum blend time (seconds)

# Velocity limiting
VELOCITY_LIMIT_SAFETY_MARGIN = 1.2  # 20% safety margin for joint velocity limits
MAX_STEP_DIFF_MULTIPLIER = 1.2  # Maximum step difference multiplier for smooth trajectories

# Smooth motion thresholds
SMOOTH_MOTION_POSITION_THRESHOLD_MM = 2.0  # Position error threshold for smooth motion (2mm)
SMOOTH_MOTION_POSITION_THRESHOLD_RELAXED_MM = 5.0  # Relaxed threshold (5mm)

# ============================================================================
# Networking Constants
# ============================================================================

# UDP communication
UDP_LISTEN_IP = "127.0.0.1"  # Loopback address for local communication
UDP_COMMAND_PORT = 5001  # Port for receiving commands
UDP_ACK_PORT = 5002  # Port for sending acknowledgments

# Network buffer sizes
UDP_RECEIVE_BUFFER_SIZE = 1024  # Bytes
COMMAND_QUEUE_MAX_SIZE = 100  # Maximum number of queued commands
TRAJECTORY_COMMAND_MAX_SIZE = 10  # Maximum number of trajectory-heavy commands in queue

# Command ID format
COMMAND_ID_LENGTH = 8  # Length of command ID string

# ============================================================================
# Serial Communication Constants
# ============================================================================

# Serial port settings
SERIAL_PORT_DEFAULT = "/dev/ttyACM0"  # Default serial port on Linux
SERIAL_BAUD_RATE = 3_000_000  # 3 Mbaud
SERIAL_TIMEOUT_S = 0.0  # Non-blocking mode

# Serial buffer sizes
SERIAL_RX_BUFFER_SIZE = 120  # Maximum receive buffer size
SERIAL_TX_PACKET_LENGTH = 52  # TX packet data length (excluding start/length/end bytes)
SERIAL_RX_PACKET_LENGTH = 56  # RX packet data length (typical)

# ============================================================================
# Gripper Constants
# ============================================================================

# Electric gripper limits
GRIPPER_POSITION_MIN = 0  # Minimum gripper position
GRIPPER_POSITION_MAX = 255  # Maximum gripper position
GRIPPER_SPEED_MIN = 0  # Minimum gripper speed
GRIPPER_SPEED_SPEED_MAX = 255  # Maximum gripper speed
GRIPPER_CURRENT_MIN = 100  # Minimum gripper current (mA)
GRIPPER_CURRENT_MAX = 1000  # Maximum gripper current (mA)

# Gripper position tolerance
GRIPPER_POSITION_TOLERANCE = 5  # Position error tolerance for gripper move completion

# Gripper modes
GRIPPER_MODE_OPERATION = 0  # Normal operation mode
GRIPPER_MODE_CALIBRATE = 1  # Calibration mode
GRIPPER_MODE_CLEAR_ERROR = 2  # Clear error mode

# ============================================================================
# Performance Monitoring Constants
# ============================================================================

# Performance monitoring
PERF_MONITOR_WINDOW_SIZE = 1000  # Number of samples to keep for performance monitoring
PERF_MONITOR_LOG_INTERVAL_CYCLES = 10000  # Log performance stats every N cycles (100 seconds)

# Timing thresholds
CYCLE_TIME_TARGET_MS = 10.0  # Target cycle time (10ms for 100Hz)
CYCLE_TIME_WARNING_MS = 15.0  # Warn if cycle time exceeds this
CYCLE_TIME_CRITICAL_MS = 20.0  # Critical if cycle time exceeds this

# ============================================================================
# Logging Constants
# ============================================================================

# Log levels
LOG_LEVEL_DEFAULT = "INFO"  # Default logging level
LOG_LEVEL_DEBUG = "DEBUG"  # Debug level for detailed output
LOG_LEVEL_INFO = "INFO"  # Info level for normal operation
LOG_LEVEL_WARNING = "WARNING"  # Warning level for potential issues
LOG_LEVEL_ERROR = "ERROR"  # Error level for serious problems

# Log formatting
LOG_FORMAT_TIMESTAMP = "%Y-%m-%d %H:%M:%S"  # Timestamp format for logs
LOG_MODULE_NAME_WIDTH = 20  # Width for module name in log messages

# ============================================================================
# Command Constants
# ============================================================================

# Command types (sent to robot firmware)
CMD_IDLE = 255  # Idle command
CMD_HOME = 100  # Homing command
CMD_ENABLE = 101  # Enable motors
CMD_DISABLE = 102  # Disable motors / E-stop
CMD_JOG = 123  # Jog command (speed control)
CMD_POSITION = 156  # Position command (move to target)

# Command validation
MIN_DURATION_S = 0.001  # Minimum duration for timed commands (1ms)
MAX_DURATION_S = 3600.0  # Maximum duration for timed commands (1 hour)
MIN_SPEED_PERCENTAGE = 0.0  # Minimum speed percentage
MAX_SPEED_PERCENTAGE = 100.0  # Maximum speed percentage

# ============================================================================
# Safety Constants
# ============================================================================

# Joint limits safety
JOINT_LIMIT_APPROACH_THRESHOLD_DEG = 5.0  # Warn when within 5° of joint limit
JOINT_VELOCITY_SAFETY_FACTOR = 0.9  # Use 90% of max velocity for safety

# E-stop
ESTOP_CHECK_INTERVAL_CYCLES = 1  # Check E-stop every cycle
ESTOP_RECOVERY_WAIT_S = 0.5  # Wait time after E-stop cleared before motion

# Collision detection (placeholder for future implementation)
COLLISION_DETECTION_ENABLED = False  # Not yet implemented
COLLISION_FORCE_THRESHOLD_N = 50.0  # Force threshold for collision detection

# ============================================================================
# Validation Constants
# ============================================================================

# Pose validation
POSE_ELEMENT_COUNT = 6  # Number of elements in pose [x, y, z, rx, ry, rz]
JOINT_ANGLE_COUNT = 6  # Number of joint angles

# Parameter validation
PERCENTAGE_MIN = 0.0  # Minimum percentage value
PERCENTAGE_MAX = 100.0  # Maximum percentage value

# ============================================================================
# Utility Functions
# ============================================================================

def cycles_to_seconds(cycles):
    """Convert control loop cycles to seconds."""
    return cycles * CONTROL_INTERVAL_S


def seconds_to_cycles(seconds):
    """Convert seconds to control loop cycles."""
    return int(seconds / CONTROL_INTERVAL_S)


def mm_to_meters(mm):
    """Convert millimeters to meters."""
    return mm / 1000.0


def meters_to_mm(meters):
    """Convert meters to millimeters."""
    return meters * 1000.0


def deg_to_rad(degrees):
    """Convert degrees to radians."""
    import math
    return degrees * math.pi / 180.0


def rad_to_deg(radians):
    """Convert radians to degrees."""
    import math
    return radians * 180.0 / math.pi


# ============================================================================
# Module Metadata
# ============================================================================

__version__ = "1.0.0"
__author__ = "PAROL6 Team"
__date__ = "2025-01-12"
__description__ = "Central constants for PAROL6 robot control system"
