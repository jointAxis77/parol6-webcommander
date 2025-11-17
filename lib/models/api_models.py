"""
Pydantic models for FastAPI server - PAROL6 Robot API
"""

from pydantic import BaseModel, Field, validator
from typing import List, Optional, Literal, Dict, Any
from datetime import datetime


# ============================================================================
# Request Models - Commands sent to the robot
# ============================================================================

class MoveJointsRequest(BaseModel):
    """Request to move robot joints to specific angles"""
    angles: List[float] = Field(
        ..., 
        description="Joint angles in degrees [J1, J2, J3, J4, J5, J6] where J1=Base, J2=Shoulder, J3=Elbow, J4=Wrist pitch, J5=Wrist roll, J6=End effector", 
        min_items=6, 
        max_items=6,
        example=[0, -45, 90, 0, 45, 0]
    )
    duration: Optional[float] = Field(None, description="Duration in seconds", gt=0)
    speed_percentage: Optional[int] = Field(None, description="Speed as percentage (1-100)", ge=1, le=100)
    wait_for_ack: bool = Field(False, description="Wait for command acknowledgment")
    timeout: float = Field(2.0, description="Acknowledgment timeout in seconds", gt=0)
    
    @validator('angles')
    def validate_angles(cls, v):
        # Could add joint limit validation here if needed
        return v
    
    @validator('duration', 'speed_percentage')
    def validate_timing(cls, v, values):
        if 'duration' in values and 'speed_percentage' in values:
            if values['duration'] is not None and values['speed_percentage'] is not None:
                raise ValueError("Provide either duration or speed_percentage, not both")
        return v


class MovePoseRequest(BaseModel):
    """Request to move robot to a specific pose"""
    pose: List[float] = Field(..., description="Target pose [x, y, z, rx, ry, rz] (mm and degrees)", min_items=6, max_items=6)
    duration: Optional[float] = Field(None, description="Duration in seconds", gt=0)
    speed_percentage: Optional[int] = Field(None, description="Speed as percentage (1-100)", ge=1, le=100)
    wait_for_ack: bool = Field(False, description="Wait for command acknowledgment")
    timeout: float = Field(2.0, description="Acknowledgment timeout in seconds", gt=0)


class MoveCartesianRequest(BaseModel):
    """Request to move robot in cartesian space"""
    pose: List[float] = Field(..., description="Target pose [x, y, z, rx, ry, rz] (mm and degrees)", min_items=6, max_items=6)
    duration: Optional[float] = Field(None, description="Duration in seconds", gt=0)
    speed_percentage: Optional[float] = Field(None, description="Speed as percentage (1-100)", ge=1, le=100)
    wait_for_ack: bool = Field(False, description="Wait for command acknowledgment")
    timeout: float = Field(2.0, description="Acknowledgment timeout in seconds", gt=0)


class JogJointRequest(BaseModel):
    """Request to jog a single joint"""
    joint_index: int = Field(..., description="Joint index (0-5)", ge=0, le=5)
    speed_percentage: int = Field(..., description="Speed as percentage (1-100)", ge=1, le=100)
    duration: Optional[float] = Field(None, description="Jog duration in seconds", gt=0)
    distance_deg: Optional[float] = Field(None, description="Jog distance in degrees")
    wait_for_ack: bool = Field(False, description="Wait for command acknowledgment")
    timeout: float = Field(2.0, description="Acknowledgment timeout in seconds", gt=0)


class JogCartesianRequest(BaseModel):
    """Request to jog in cartesian space"""
    frame: Literal['TRF', 'WRF'] = Field(..., description="Reference frame (Tool or World)")
    axis: Literal['X+', 'X-', 'Y+', 'Y-', 'Z+', 'Z-', 'RX+', 'RX-', 'RY+', 'RY-', 'RZ+', 'RZ-'] = Field(..., description="Jog axis and direction")
    speed_percentage: int = Field(..., description="Speed as percentage (1-100)", ge=1, le=100)
    duration: float = Field(..., description="Jog duration in seconds", gt=0)
    wait_for_ack: bool = Field(False, description="Wait for command acknowledgment")
    timeout: float = Field(2.0, description="Acknowledgment timeout in seconds", gt=0)


class ElectricGripperRequest(BaseModel):
    """Request to control electric gripper"""
    action: Literal['move', 'calibrate'] = Field(..., description="Gripper action")
    position: Optional[int] = Field(255, description="Target position (0-255)", ge=0, le=255)
    speed: Optional[int] = Field(150, description="Movement speed", ge=0, le=255)
    current: Optional[int] = Field(500, description="Current limit (100-1000)", ge=100, le=1000)
    wait_for_ack: bool = Field(False, description="Wait for command acknowledgment")
    timeout: float = Field(2.0, description="Acknowledgment timeout in seconds", gt=0)


class PneumaticGripperRequest(BaseModel):
    """Request to control pneumatic gripper"""
    action: Literal['open', 'close'] = Field(..., description="Gripper action")
    port: Literal[1, 2] = Field(..., description="Output port number")
    wait_for_ack: bool = Field(False, description="Wait for command acknowledgment")
    timeout: float = Field(2.0, description="Acknowledgment timeout in seconds", gt=0)


class DelayRequest(BaseModel):
    """Request to add delay"""
    duration: float = Field(..., description="Delay duration in seconds", gt=0)
    wait_for_ack: bool = Field(False, description="Wait for command acknowledgment")
    timeout: float = Field(2.0, description="Acknowledgment timeout in seconds", gt=0)


class IKRequest(BaseModel):
    """Request to solve inverse kinematics"""
    target_pose: List[float] = Field(
        ...,
        description="Target pose [x, y, z, rx, ry, rz] (mm and degrees)",
        min_items=6,
        max_items=6
    )
    target_quaternion: Optional[List[float]] = Field(
        None,
        description="Target orientation as quaternion [w, x, y, z] (optional, preferred over Euler angles)",
        min_items=4,
        max_items=4
    )
    current_joints: List[float] = Field(
        ...,
        description="Current joint angles in degrees [J1, J2, J3, J4, J5, J6] (seed position)",
        min_items=6,
        max_items=6
    )
    axis_mask: Optional[List[int]] = Field(
        None,
        description="Axis mask for selective IK [1,1,1,1,1,1] = full 6-DOF, [1,1,1,0,0,0] = position-only",
        min_items=6,
        max_items=6
    )


# ============================================================================
# Smooth Motion Request Models
# ============================================================================

class SmoothCircleRequest(BaseModel):
    """Request for smooth circular motion"""
    center: List[float] = Field(..., description="Circle center [x, y, z] in mm", min_items=3, max_items=3)
    radius: float = Field(..., description="Circle radius in mm", gt=0)
    plane: Literal['XY', 'XZ', 'YZ'] = Field('XY', description="Circle plane")
    frame: Literal['WRF', 'TRF'] = Field('WRF', description="Reference frame")
    start_pose: Optional[List[float]] = Field(None, description="Start pose [x, y, z, rx, ry, rz]", min_items=6, max_items=6)
    duration: Optional[float] = Field(None, description="Duration in seconds", gt=0)
    speed_percentage: Optional[float] = Field(None, description="Speed as percentage", ge=1, le=100)
    clockwise: bool = Field(False, description="Direction of motion")
    wait_for_ack: bool = Field(False, description="Wait for acknowledgment")
    timeout: float = Field(10.0, description="Timeout in seconds", gt=0)


class SmoothArcRequest(BaseModel):
    """Request for smooth arc motion"""
    end_pose: List[float] = Field(..., description="End pose [x, y, z, rx, ry, rz]", min_items=6, max_items=6)
    center: List[float] = Field(..., description="Arc center [x, y, z] in mm", min_items=3, max_items=3)
    frame: Literal['WRF', 'TRF'] = Field('WRF', description="Reference frame")
    start_pose: Optional[List[float]] = Field(None, description="Start pose", min_items=6, max_items=6)
    duration: Optional[float] = Field(None, description="Duration in seconds", gt=0)
    speed_percentage: Optional[float] = Field(None, description="Speed as percentage", ge=1, le=100)
    clockwise: bool = Field(False, description="Direction of motion")
    wait_for_ack: bool = Field(False, description="Wait for acknowledgment")
    timeout: float = Field(10.0, description="Timeout in seconds", gt=0)


class SmoothSplineRequest(BaseModel):
    """Request for smooth spline motion"""
    waypoints: List[List[float]] = Field(..., description="List of waypoint poses [x, y, z, rx, ry, rz]", min_items=2)
    frame: Literal['WRF', 'TRF'] = Field('WRF', description="Reference frame")
    start_pose: Optional[List[float]] = Field(None, description="Start pose", min_items=6, max_items=6)
    duration: Optional[float] = Field(None, description="Duration in seconds", gt=0)
    speed_percentage: Optional[float] = Field(None, description="Speed as percentage", ge=1, le=100)
    wait_for_ack: bool = Field(False, description="Wait for acknowledgment")
    timeout: float = Field(10.0, description="Timeout in seconds", gt=0)
    
    @validator('waypoints')
    def validate_waypoints(cls, v):
        for wp in v:
            if len(wp) != 6:
                raise ValueError("Each waypoint must have 6 values [x, y, z, rx, ry, rz]")
        return v


class SmoothHelixRequest(BaseModel):
    """Request for smooth helical motion"""
    center: List[float] = Field(..., description="Helix center [x, y, z] in mm", min_items=3, max_items=3)
    radius: float = Field(..., description="Helix radius in mm", gt=0)
    pitch: float = Field(..., description="Vertical distance per revolution in mm", gt=0)
    height: float = Field(..., description="Total height in mm", gt=0)
    frame: Literal['WRF', 'TRF'] = Field('WRF', description="Reference frame")
    start_pose: Optional[List[float]] = Field(None, description="Start pose", min_items=6, max_items=6)
    duration: Optional[float] = Field(None, description="Duration in seconds", gt=0)
    speed_percentage: Optional[float] = Field(None, description="Speed as percentage", ge=1, le=100)
    clockwise: bool = Field(False, description="Direction of motion")
    wait_for_ack: bool = Field(False, description="Wait for acknowledgment")
    timeout: float = Field(10.0, description="Timeout in seconds", gt=0)


class BatchIKRequest(BaseModel):
    """Request to solve IK for multiple Cartesian waypoints"""
    waypoints: List[List[float]] = Field(
        ...,
        description="List of Cartesian poses [x, y, z, rx, ry, rz] in mm and degrees",
        min_items=1
    )
    seed_joints: Optional[List[float]] = Field(
        None,
        description="Initial joint configuration for IK seeding [J1-J6] in degrees",
        min_items=6,
        max_items=6
    )
    use_previous_as_seed: bool = Field(
        True,
        description="Use previous IK solution as seed for next waypoint (faster convergence)"
    )

    @validator('waypoints')
    def validate_waypoints(cls, v):
        for i, waypoint in enumerate(v):
            if len(waypoint) != 6:
                raise ValueError(f"Waypoint {i} must have 6 values [x,y,z,rx,ry,rz], got {len(waypoint)}")
        return v


class ExecuteTrajectoryRequest(BaseModel):
    """Request to execute a pre-computed joint trajectory"""
    trajectory: List[List[float]] = Field(
        ...,
        description="Pre-computed joint trajectory, each waypoint is [J1-J6] in degrees",
        min_items=1
    )
    duration: Optional[float] = Field(
        None,
        description="Expected duration in seconds (for validation)",
        gt=0
    )
    wait_for_ack: bool = Field(False, description="Wait for command acknowledgment")
    timeout: float = Field(30.0, description="Acknowledgment timeout in seconds", gt=0)

    @validator('trajectory')
    def validate_trajectory(cls, v):
        for i, waypoint in enumerate(v):
            if len(waypoint) != 6:
                raise ValueError(f"Waypoint {i} must have 6 joint values, got {len(waypoint)}")
        return v


# ============================================================================
# Response Models - Data returned from the robot
# ============================================================================

class RobotPose(BaseModel):
    """Robot end-effector pose"""
    x: float = Field(..., description="X position in mm")
    y: float = Field(..., description="Y position in mm")
    z: float = Field(..., description="Z position in mm")
    roll: float = Field(..., description="Roll angle in degrees")
    pitch: float = Field(..., description="Pitch angle in degrees")
    yaw: float = Field(..., description="Yaw angle in degrees")
    timestamp: datetime = Field(default_factory=datetime.now, description="Timestamp of reading")


class JointAngles(BaseModel):
    """Robot joint angles"""
    angles: List[float] = Field(
        ..., 
        description="Joint angles in degrees [J1, J2, J3, J4, J5, J6] where J1=Base, J2=Shoulder, J3=Elbow, J4=Wrist pitch, J5=Wrist roll, J6=End effector", 
        min_items=6, 
        max_items=6,
        example=[0, -45, 90, 0, 45, 0]
    )
    timestamp: datetime = Field(default_factory=datetime.now, description="Timestamp of reading")


class JointSpeeds(BaseModel):
    """Robot joint speeds"""
    speeds: List[float] = Field(..., description="Joint speeds in steps/sec", min_items=6, max_items=6)
    timestamp: datetime = Field(default_factory=datetime.now, description="Timestamp of reading")


class IOStatus(BaseModel):
    """Digital I/O status"""
    input_1: bool = Field(..., description="Digital input 1 state")
    input_2: bool = Field(..., description="Digital input 2 state")
    output_1: bool = Field(..., description="Digital output 1 state")
    output_2: bool = Field(..., description="Digital output 2 state")
    estop_pressed: bool = Field(..., description="Emergency stop state (True = pressed)")
    timestamp: datetime = Field(default_factory=datetime.now, description="Timestamp of reading")


class GripperStatus(BaseModel):
    """Electric gripper status"""
    device_id: int = Field(..., description="Gripper device ID")
    position: int = Field(..., description="Current position (0-255)")
    speed: int = Field(..., description="Current speed")
    current: int = Field(..., description="Current draw")
    status_byte: int = Field(..., description="Status byte")
    object_detected: int = Field(..., description="Object detection (0=none, 1=closing, 2=opening)")
    is_calibrated: bool = Field(..., description="Calibration status")
    is_active: bool = Field(..., description="Active status")
    is_moving: bool = Field(..., description="Movement status")
    timestamp: datetime = Field(default_factory=datetime.now, description="Timestamp of reading")


class RobotStatus(BaseModel):
    """Complete robot status"""
    pose: Optional[RobotPose] = None
    joints: Optional[JointAngles] = None
    speeds: Optional[JointSpeeds] = None
    io: Optional[IOStatus] = None
    gripper: Optional[GripperStatus] = None
    is_stopped: Optional[bool] = Field(None, description="Robot stopped status (None if unknown)")
    estop_active: Optional[bool] = Field(None, description="E-stop active status (None if unknown)")
    homed: Optional[List[bool]] = Field(None, description="Homing status for each joint [J1, J2, J3, J4, J5, J6] (None if unknown)")
    commander_hz: Optional[float] = Field(None, description="Commander control loop frequency in Hz (None if unknown)")
    timestamp: datetime = Field(default_factory=datetime.now, description="Timestamp of reading")


class CommandResponse(BaseModel):
    """Response from robot command execution"""
    success: bool = Field(..., description="Command success status")
    command_id: Optional[str] = Field(None, description="Command tracking ID")
    message: str = Field(..., description="Response message")
    status: Optional[str] = Field(None, description="Command status (QUEUED, EXECUTING, COMPLETED, FAILED)")
    details: Optional[str] = Field(None, description="Additional details")


class IKResponse(BaseModel):
    """Response from inverse kinematics solver"""
    success: bool = Field(..., description="IK solution found")
    joints: Optional[List[float]] = Field(None, description="Joint angles in degrees [J1, J2, J3, J4, J5, J6]")
    error: Optional[str] = Field(None, description="Error message if solution failed")
    iterations: Optional[int] = Field(None, description="Number of iterations used")
    residual: Optional[float] = Field(None, description="Solution residual/error")


class BatchIKResponse(BaseModel):
    """Response from batch inverse kinematics solver"""
    success: bool = Field(..., description="All IK solutions found successfully")
    joint_trajectory: Optional[List[List[float]]] = Field(
        None,
        description="List of joint configurations [[J1-J6], [J1-J6], ...] in degrees"
    )
    failed_at: Optional[int] = Field(
        None,
        description="Index of waypoint where IK failed (null if all succeeded)"
    )
    error: Optional[str] = Field(None, description="Error message if solution failed")
    warnings: List[Dict[str, Any]] = Field(
        default_factory=list,
        description="Warnings about velocity/acceleration limits or other issues"
    )
    total_waypoints: int = Field(..., description="Total number of waypoints processed")
    planning_time_s: Optional[float] = Field(None, description="Time spent planning in seconds")


class FKRequest(BaseModel):
    """Request to compute forward kinematics"""
    joints: List[float] = Field(
        ...,
        description="Joint angles in degrees [J1, J2, J3, J4, J5, J6]",
        min_items=6,
        max_items=6
    )


class FKVariation(BaseModel):
    """A single FK variation with specific transformations applied"""
    label: str = Field(..., description="Description of this variation")
    pose: List[float] = Field(..., description="TCP pose [x, y, z, rx, ry, rz] (mm and degrees)")
    quaternion: List[float] = Field(..., description="TCP orientation as quaternion [w, x, y, z]")
    euler_order: str = Field(..., description="Euler angle extraction order (e.g., 'xyz', 'zyx')")
    coord_transform: Optional[str] = Field(None, description="Coordinate transformation applied (e.g., 'Y-up to Z-up')")


class FKResponse(BaseModel):
    """Response from forward kinematics calculation"""
    success: bool = Field(..., description="FK calculation succeeded")
    pose: Optional[List[float]] = Field(None, description="TCP pose [x, y, z, rx, ry, rz] (mm and degrees)")
    quaternion: Optional[List[float]] = Field(None, description="TCP orientation as quaternion [w, x, y, z]")
    variations: Optional[List[FKVariation]] = Field(None, description="Alternative FK calculations with different transforms")
    error: Optional[str] = Field(None, description="Error message if calculation failed")


class CommandAcknowledgment(BaseModel):
    """Command acknowledgment with tracking"""
    command_id: str = Field(..., description="Command tracking ID")
    status: str = Field(..., description="Command status")
    details: Optional[str] = Field(None, description="Status details")
    completed: bool = Field(..., description="Completion status")
    ack_time: Optional[datetime] = Field(None, description="Acknowledgment time")


# ============================================================================
# WebSocket Models
# ============================================================================

class WebSocketMessage(BaseModel):
    """WebSocket message wrapper"""
    type: str = Field(..., description="Message type")
    data: Dict[str, Any] = Field(..., description="Message data")
    timestamp: datetime = Field(default_factory=datetime.now, description="Message timestamp")


class WebSocketSubscription(BaseModel):
    """WebSocket subscription request"""
    subscribe: List[Literal['pose', 'joints', 'speeds', 'io', 'gripper', 'status', 'logs']] = Field(
        ..., description="Data types to subscribe to"
    )
    rate_hz: int = Field(10, description="Update rate in Hz", ge=1, le=50)


class WebSocketError(BaseModel):
    """WebSocket error message"""
    error: str = Field(..., description="Error message")
    code: int = Field(..., description="Error code")
    timestamp: datetime = Field(default_factory=datetime.now, description="Error timestamp")


# ============================================================================
# Logging Models
# ============================================================================

class LogMessage(BaseModel):
    """Log message structure"""
    timestamp: datetime = Field(..., description="Log timestamp")
    level: Literal['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL'] = Field(..., description="Log level")
    source: str = Field(..., description="Logger name/source module")
    message: str = Field(..., description="Log message")
    module: str = Field(..., description="Python module name")
    function: str = Field(..., description="Function name")
    line: int = Field(..., description="Line number")
    details: Optional[Dict[str, Any]] = Field(None, description="Additional log details")


class LogFilter(BaseModel):
    """Log filtering parameters"""
    level: Optional[Literal['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL']] = Field(None, description="Filter by log level")
    source: Optional[str] = Field(None, description="Filter by source/module")
    limit: Optional[int] = Field(None, description="Maximum number of logs to return", ge=1, le=10000)


# ============================================================================
# Performance Recording Models
# ============================================================================

class PerformanceSample(BaseModel):
    """Single cycle performance data"""
    cycle: float = Field(..., description="Total cycle time in ms")
    network: float = Field(..., description="Network phase time in ms")
    processing: float = Field(..., description="Processing phase time in ms")
    execution: float = Field(..., description="Execution phase time in ms")
    serial: float = Field(..., description="Serial communication time in ms")
    ik_manipulability: float = Field(..., description="IK manipulability calculation time in ms")
    ik_solve: float = Field(..., description="IK solver time in ms")


class CycleStats(BaseModel):
    """Cycle statistics"""
    avg_ms: float = Field(..., description="Average cycle time in ms")
    min_ms: float = Field(..., description="Minimum cycle time in ms")
    max_ms: float = Field(..., description="Maximum cycle time in ms")


class PhaseStats(BaseModel):
    """Phase timing statistics"""
    network_ms: float = Field(..., description="Average network time in ms")
    processing_ms: float = Field(..., description="Average processing time in ms")
    execution_ms: float = Field(..., description="Average execution time in ms")
    serial_ms: float = Field(..., description="Average serial time in ms")
    ik_manipulability_ms: float = Field(..., description="Average IK manipulability time in ms")
    ik_solve_ms: float = Field(..., description="Average IK solver time in ms")


class CommandPerformance(BaseModel):
    """Performance data for a single command"""
    command_id: str = Field(..., description="Command tracking ID")
    command_type: str = Field(..., description="Command type (e.g., MoveJointCommand)")
    timestamp: str = Field(..., description="Command execution timestamp (ISO format)")
    duration_s: float = Field(..., description="Command duration in seconds")
    num_cycles: int = Field(..., description="Number of control loop cycles")
    cycle_stats: CycleStats = Field(..., description="Cycle timing statistics")
    phase_stats: PhaseStats = Field(..., description="Phase timing statistics")
    samples: List[PerformanceSample] = Field(..., description="Per-cycle performance samples")


class RecordingMetadata(BaseModel):
    """Recording metadata"""
    name: str = Field(..., description="Recording name")
    timestamp: str = Field(..., description="Recording start timestamp (ISO format)")
    robot_config: Dict[str, Any] = Field(..., description="Robot configuration at recording time")


class PerformanceRecording(BaseModel):
    """Complete performance recording"""
    metadata: RecordingMetadata = Field(..., description="Recording metadata")
    commands: List[CommandPerformance] = Field(..., description="Recorded command performances")


class RecordingListItem(BaseModel):
    """Recording list item with summary info"""
    filename: str = Field(..., description="Recording filename")
    name: str = Field(..., description="Recording name from metadata")
    timestamp: str = Field(..., description="Recording timestamp")
    num_commands: int = Field(..., description="Number of recorded commands")
    total_duration_s: float = Field(..., description="Total duration of all commands in seconds")


class StartRecordingRequest(BaseModel):
    """Request to start performance recording"""
    name: Optional[str] = Field(None, description="Optional recording name (auto-generated if not provided)")


class StopRecordingRequest(BaseModel):
    """Request to stop performance recording (no parameters needed)"""
    pass


# ============================================================================
# Tool Management Models
# ============================================================================

class ToolOffsetPosition(BaseModel):
    """Tool offset position in millimeters"""
    x: float = Field(0, description="X offset in mm")
    y: float = Field(0, description="Y offset in mm")
    z: float = Field(0, description="Z offset in mm")


class ToolOffsetRotation(BaseModel):
    """Tool offset rotation in degrees"""
    rx: float = Field(0, description="Rotation around X in degrees")
    ry: float = Field(0, description="Rotation around Y in degrees")
    rz: float = Field(0, description="Rotation around Z in degrees")


class CreateToolRequest(BaseModel):
    """Request to create a new tool"""
    name: str = Field(..., description="Tool name", min_length=1)
    description: str = Field("", description="Tool description")
    mesh_file: Optional[str] = Field(None, description="STL filename")
    mesh_units: Optional[str] = Field("mm", description="Units of mesh file: 'mm' or 'm'")
    mesh_offset_position: ToolOffsetPosition = Field(default_factory=ToolOffsetPosition, description="Mesh visual offset position in meters")
    mesh_offset_rotation: ToolOffsetRotation = Field(default_factory=ToolOffsetRotation, description="Mesh visual offset rotation")
    tcp_offset_position: ToolOffsetPosition = Field(default_factory=ToolOffsetPosition, description="TCP functional offset position in mm")
    tcp_offset_rotation: ToolOffsetRotation = Field(default_factory=ToolOffsetRotation, description="TCP functional offset rotation")
    stl_data: Optional[str] = Field(None, description="Base64 encoded STL file data")


class UpdateToolRequest(BaseModel):
    """Request to update an existing tool"""
    name: Optional[str] = Field(None, description="Tool name", min_length=1)
    description: Optional[str] = Field(None, description="Tool description")
    mesh_file: Optional[str] = Field(None, description="STL filename")
    mesh_units: Optional[str] = Field(None, description="Units of mesh file: 'mm' or 'm'")
    mesh_offset_position: Optional[ToolOffsetPosition] = Field(None, description="Mesh visual offset position in meters")
    mesh_offset_rotation: Optional[ToolOffsetRotation] = Field(None, description="Mesh visual offset rotation")
    tcp_offset_position: Optional[ToolOffsetPosition] = Field(None, description="TCP functional offset position in mm")
    tcp_offset_rotation: Optional[ToolOffsetRotation] = Field(None, description="TCP functional offset rotation")
    stl_data: Optional[str] = Field(None, description="Base64 encoded STL file data")