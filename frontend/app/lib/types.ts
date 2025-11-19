// Joint name type
export type JointName = 'J1' | 'J2' | 'J3' | 'J4' | 'J5' | 'J6';

// Cartesian axis type
export type CartesianAxis = 'X' | 'Y' | 'Z' | 'RX' | 'RY' | 'RZ';

// Motion mode type
export type MotionMode = 'joint' | 'cartesian';

// Joint angles for 6-DOF robot
export interface JointAngles {
  J1: number;
  J2: number;
  J3: number;
  J4: number;
  J5: number;
  J6: number;
}

// Cartesian pose for end effector
export interface CartesianPose {
  X: number;  // mm
  Y: number;  // mm
  Z: number;  // mm
  RX: number; // degrees
  RY: number; // degrees
  RZ: number; // degrees
}

// IK axis mask - selectively enable/disable axes during IK solving
export interface IkAxisMask {
  X: boolean;   // Solve for X position
  Y: boolean;   // Solve for Y position
  Z: boolean;   // Solve for Z position
  RX: boolean;  // Solve for RX rotation
  RY: boolean;  // Solve for RY rotation
  RZ: boolean;  // Solve for RZ rotation
}

// Hardware feedback types (from WebSocket)

// I/O Status from robot hardware
export interface IOStatus {
  input_1: boolean;
  input_2: boolean;
  output_1: boolean;
  output_2: boolean;
  estop_pressed: boolean;
  timestamp: string;
}

// Gripper status from robot hardware
export interface GripperStatus {
  device_id: number;
  position: number;        // 0-255
  speed: number;
  current: number;
  status_byte: number;
  object_detected: number; // 0=none, 1=closing, 2=opening
  is_calibrated: boolean;
  is_active: boolean;
  is_moving: boolean;
  timestamp: string;
}

// Overall robot status
export interface RobotStatus {
  is_stopped: boolean | null;
  estop_active: boolean | null;
  homed: boolean[] | null;  // Homing status for all 6 joints [J1, J2, J3, J4, J5, J6]
  commander_hz: number | null;  // Commander control loop frequency in Hz
  timestamp: string;
}

// WebSocket connection status
export type ConnectionStatus = 'disconnected' | 'connecting' | 'connected' | 'error';

// Keyframe = single pose snapshot at a specific time
// Stores all 6 joint angles together (robot commands are coordinated, not per-joint)
export interface Keyframe {
  id: string;
  time: number; // seconds
  jointAngles: JointAngles; // All 6 joint angles (absolute truth - always stored)
  motionType?: 'joint' | 'cartesian'; // How to interpolate to next keyframe (default: 'joint')
  cartesianPose?: CartesianPose; // FK-computed cartesian pose (for reference/editing)
  label?: string;
  toolId?: string; // Tool active at this keyframe
  gripperState?: 'open' | 'closed'; // Gripper state at this keyframe (if tool has gripper)
}

// Timeline = collection of keyframes (robot poses over time)
// Each keyframe represents one coordinated robot pose
export interface Timeline {
  name: string;
  mode: MotionMode; // joint or cartesian (display preference only)
  keyframes: Keyframe[]; // Array of pose keyframes
  duration: number; // total duration in seconds
  fps?: number; // playback frame rate (default 60)
}

// Playback state
export interface PlaybackState {
  isPlaying: boolean;
  currentTime: number;
  startTime: number | null;
  loop: boolean;
  executeOnRobot: boolean;  // Whether to send commands to actual robot during playback
  playbackError: string | null;  // Error message if playback fails (e.g., IK failure during cartesian interpolation)
}

// Joint limits (from PAROL6_ROBOT.py)
export interface JointLimit {
  min: number;
  max: number;
}

// Tool configuration (from config.yaml)
export interface Tool {
  id: string;
  name: string;
  description: string;
  mesh_file: string | null;
  mesh_units?: 'mm' | 'm';
  mesh_offset: {
    x: number;
    y: number;
    z: number;
    rx: number;
    ry: number;
    rz: number;
  };
  tcp_offset: {
    x: number;
    y: number;
    z: number;
    rx: number;
    ry: number;
    rz: number;
  };
  gripper_config?: {
    enabled: boolean;
    io_pin: number;
    open_is_high: boolean;
    mesh_file_open: string | null;
    mesh_file_closed: string | null;
  };
}

// Store state
export interface TimelineStore {
  // Data
  timeline: Timeline;
  currentJointAngles: JointAngles;
  currentCartesianPose: CartesianPose;
  playbackState: PlaybackState;
  ikAxisMask: IkAxisMask;

  // Target TCP pose from target robot URDF (commanded position, updated by TargetTCPVisualizer)
  targetTcpPosition: { X: number; Y: number; Z: number; RX: number; RY: number; RZ: number } | null;

  // Actual TCP pose from actual robot URDF (hardware feedback, updated by ActualTCPVisualizer)
  actualTcpPosition: { X: number; Y: number; Z: number; RX: number; RY: number; RZ: number } | null;

  // Hardware feedback from WebSocket
  actualJointAngles: JointAngles | null;
  actualCartesianPose: CartesianPose | null;
  ioStatus: IOStatus | null;
  gripperStatus: GripperStatus | null;
  robotStatus: RobotStatus | null;
  connectionStatus: ConnectionStatus;

  // UI state
  selectedJoint: JointName | null;
  setSelectedJoint: (joint: JointName | null) => void;
  showActualRobot: boolean;
  setShowActualRobot: (show: boolean) => void;
  showTargetRobot: boolean;
  setShowTargetRobot: (show: boolean) => void;
  stepAngle: number; // Step angle in degrees for keyboard and slider step buttons
  setStepAngle: (angle: number) => void;
  jointHomedStatus: Record<JointName, boolean>;
  setJointHomed: (joint: JointName, homed: boolean) => void;

  // Robot following modes (mutually exclusive)
  targetFollowsActual: boolean; // Target robot mirrors actual robot (teaching mode)
  actualFollowsTarget: boolean; // Send commands when target changes (live control mode)
  setTargetFollowsActual: (follows: boolean) => void;
  setActualFollowsTarget: (follows: boolean) => void;

  // Movement parameters (speed/accel from UI controls)
  speed: number; // Speed percentage (0-100)
  accel: number; // Acceleration percentage (0-100)
  setSpeed: (speed: number) => void;
  setAccel: (accel: number) => void;

  // Motion mode actions
  setMotionMode: (mode: MotionMode) => void;

  // Joint keyframe actions
  addKeyframe: (time: number, joint: JointName, value: number) => void;
  removeKeyframe: (id: string) => void;
  updateKeyframe: (id: string, updates: Partial<Keyframe>) => void;
  recordKeyframes: () => void; // Records 6 separate keyframes (one per joint)
  setJointAngle: (joint: keyof JointAngles, angle: number) => void;

  // Cartesian keyframe actions
  addCartesianKeyframe: (time: number, axis: CartesianAxis, value: number) => void;
  removeCartesianKeyframe: (id: string) => void;
  updateCartesianKeyframe: (id: string, updates: Partial<CartesianKeyframe>) => void;
  recordCartesianKeyframes: () => void; // Records 6 separate cartesian keyframes
  setCartesianValue: (axis: CartesianAxis, value: number) => void;

  // Playback actions
  setCurrentTime: (time: number) => void;
  play: (executeOnRobot?: boolean) => void;
  pause: () => void;
  stop: () => void;

  // Timeline management
  loadTimeline: (timeline: Timeline) => void;
  exportTimeline: () => string; // JSON

  // IK axis mask
  setIkAxisMask: (updates: Partial<IkAxisMask>) => void;

  // Hardware feedback actions (from WebSocket)
  setActualJointAngles: (angles: JointAngles | null) => void;
  setActualCartesianPose: (pose: CartesianPose | null) => void;
  setIOStatus: (status: IOStatus | null) => void;
  setGripperStatus: (status: GripperStatus | null) => void;
  setRobotStatus: (status: RobotStatus | null) => void;
  setConnectionStatus: (status: ConnectionStatus) => void;
}
