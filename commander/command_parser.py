"""
Command Parser Module for PAROL6 Robot

Extracts UDP command parsing logic from main controller loop.
Provides clean interface for parsing command strings into command objects.

Author: PAROL6 Team
Date: 2025-01-13
"""

import logging
import numpy as np
from typing import Optional, Tuple, List, Dict, Any
from spatialmath import SE3

# Import robot model for constants and calculations
from lib.kinematics import robot_model as PAROL6_ROBOT

# Import constants
from constants import (
    POSE_ELEMENT_COUNT,
    JOINT_ANGLE_COUNT,
)

# Note: Command classes will be imported dynamically or passed as parameter
# to avoid circular imports


# ============================================================================
# Command Parser Class
# ============================================================================

class CommandParser:
    """
    Parses UDP command strings into command objects.

    Handles all command types including:
    - Motion commands (MOVEJOINT, MOVEPOSE, MOVECART)
    - Jog commands (JOG, MULTIJOG, CARTJOG)
    - Smooth motion (CIRCLE, ARC, SPLINE, HELIX, BLEND)
    - Utility (HOME, DELAY, GRIPPER)

    Returns (command_object, error_message) tuple.
    On success: (cmd_obj, None)
    On failure: (None, "error description")
    """

    def __init__(self, logger: logging.Logger, robot_model=None):
        """
        Initialize command parser.

        Args:
            logger: Logger instance for parsing messages
            robot_model: Robot model for kinematics (defaults to PAROL6_ROBOT)
        """
        self.logger = logger
        self.robot = robot_model or PAROL6_ROBOT

        # Command parser registry (only kept commands after cleanup)
        self._parsers = {
            'HOME': self._parse_home,
            'MOVEJOINT': self._parse_move_joint,
            'EXECUTETRAJECTORY': self._parse_execute_trajectory,
            'PNEUMATICGRIPPER': self._parse_pneumatic_gripper,
            'ELECTRICGRIPPER': self._parse_electric_gripper,
            'DELAY': self._parse_delay,
        }

    def parse(self, message: str, command_classes: Dict[str, Any]) -> Tuple[Optional[Any], Optional[str]]:
        """
        Parse command string into command object.

        Args:
            message: Raw UDP command string (e.g., "MOVEJOINT|0,0,0,0,0,0|2.5|50")
            command_classes: Dictionary mapping command names to classes
                            (passed to avoid circular imports)

        Returns:
            Tuple of (command_object, error_message)
            - On success: (command_obj, None)
            - On failure: (None, "error description")

        Example:
            cmd_classes = {
                'MoveJoint': MoveJointCommand,
                'MovePose': MovePoseCommand,
                # ... etc
            }
            cmd_obj, error = parser.parse("MOVEJOINT|0,0,0,0,0,0|2.5|50", cmd_classes)
        """
        self.command_classes = command_classes  # Store for use in parser methods

        try:
            # Split command into parts
            parts = message.split('|')
            if not parts:
                return None, "Empty command"

            command_name = parts[0].upper()

            # Look up parser method
            parser_method = self._parsers.get(command_name)
            if not parser_method:
                return None, f"Unknown command: {command_name}"

            # Parse command
            return parser_method(parts)

        except Exception as e:
            error_msg = f"Parse error: {str(e)}"
            self.logger.error(f"[CommandParser] {error_msg}")
            return None, error_msg

    # ========================================================================
    # Motion Command Parsers
    # ========================================================================

    def _parse_move_joint(self, parts: List[str]) -> Tuple[Optional[Any], Optional[str]]:
        """Parse MOVEJOINT command: MOVEJOINT|j1,j2,j3,j4,j5,j6|duration|speed"""
        try:
            if len(parts) != 9:
                return None, f"MOVEJOINT expects 9 parts, got {len(parts)}"

            joint_vals = [float(p) for p in parts[1:7]]
            duration = None if parts[7].upper() == 'NONE' else float(parts[7])
            speed = None if parts[8].upper() == 'NONE' else float(parts[8])

            MoveJointCommand = self.command_classes.get('MOVEJOINT')
            if not MoveJointCommand:
                return None, "MoveJointCommand class not provided"

            cmd_obj = MoveJointCommand(
                target_angles=joint_vals,
                duration=duration,
                velocity_percent=speed
            )
            return cmd_obj, None

        except ValueError as e:
            return None, f"MOVEJOINT parameter error: {e}"
        except Exception as e:
            return None, f"MOVEJOINT parse error: {e}"

    def _parse_move_pose(self, parts: List[str]) -> Tuple[Optional[Any], Optional[str]]:
        """Parse MOVEPOSE command: MOVEPOSE|x,y,z,rx,ry,rz|duration|speed"""
        try:
            if len(parts) != 9:
                return None, f"MOVEPOSE expects 9 parts, got {len(parts)}"

            pose_vals = [float(p) for p in parts[1:7]]
            duration = None if parts[7].upper() == 'NONE' else float(parts[7])
            speed = None if parts[8].upper() == 'NONE' else float(parts[8])

            MovePoseCommand = self.command_classes.get('MOVEPOSE')
            if not MovePoseCommand:
                return None, "MovePoseCommand class not provided"

            cmd_obj = MovePoseCommand(
                pose=pose_vals,
                duration=duration,
                velocity_percent=speed
            )
            return cmd_obj, None

        except ValueError as e:
            return None, f"MOVEPOSE parameter error: {e}"
        except Exception as e:
            return None, f"MOVEPOSE parse error: {e}"

    def _parse_move_cart(self, parts: List[str]) -> Tuple[Optional[Any], Optional[str]]:
        """Parse MOVECART command: MOVECART|x,y,z,rx,ry,rz|duration|speed"""
        try:
            if len(parts) != 9:
                return None, f"MOVECART expects 9 parts, got {len(parts)}"

            pose_vals = [float(p) for p in parts[1:7]]
            duration = None if parts[7].upper() == 'NONE' else float(parts[7])
            speed = None if parts[8].upper() == 'NONE' else float(parts[8])

            MoveCartCommand = self.command_classes.get('MOVECART')
            if not MoveCartCommand:
                return None, "MoveCartCommand class not provided"

            cmd_obj = MoveCartCommand(
                pose=pose_vals,
                duration=duration,
                velocity_percent=speed
            )
            return cmd_obj, None

        except ValueError as e:
            return None, f"MOVECART parameter error: {e}"
        except Exception as e:
            return None, f"MOVECART parse error: {e}"

    def _parse_execute_trajectory(self, parts: List[str]) -> Tuple[Optional[Any], Optional[str]]:
        """Parse EXECUTETRAJECTORY command: EXECUTETRAJECTORY|<json_trajectory>|duration"""
        import json

        try:
            if len(parts) != 3:
                return None, f"EXECUTETRAJECTORY expects 3 parts, got {len(parts)}"

            # Parse JSON trajectory
            try:
                trajectory = json.loads(parts[1])
            except json.JSONDecodeError as e:
                return None, f"EXECUTETRAJECTORY invalid JSON: {e}"

            # Validate trajectory structure
            if not isinstance(trajectory, list) or len(trajectory) == 0:
                return None, "EXECUTETRAJECTORY trajectory must be non-empty list"

            for i, waypoint in enumerate(trajectory):
                if not isinstance(waypoint, list) or len(waypoint) != 6:
                    return None, f"EXECUTETRAJECTORY waypoint {i} must have 6 joints"

            # Parse duration (optional)
            duration = None if parts[2].upper() == 'NONE' else float(parts[2])

            ExecuteTrajectoryCommand = self.command_classes.get('EXECUTETRAJECTORY')
            if not ExecuteTrajectoryCommand:
                return None, "ExecuteTrajectoryCommand class not provided"

            cmd_obj = ExecuteTrajectoryCommand(
                trajectory_deg=trajectory,
                duration=duration
            )
            return cmd_obj, None

        except ValueError as e:
            return None, f"EXECUTETRAJECTORY parameter error: {e}"
        except Exception as e:
            return None, f"EXECUTETRAJECTORY parse error: {e}"

    # ========================================================================
    # Jog Command Parsers
    # ========================================================================

    def _parse_jog(self, parts: List[str]) -> Tuple[Optional[Any], Optional[str]]:
        """Parse JOG command: JOG|joint|speed|duration|distance"""
        try:
            if len(parts) != 5:
                return None, f"JOG expects 5 parts, got {len(parts)}"

            joint_idx = int(parts[1])
            speed = float(parts[2])
            duration = None if parts[3].upper() == 'NONE' else float(parts[3])
            distance = None if parts[4].upper() == 'NONE' else float(parts[4])

            JogCommand = self.command_classes.get('JOG')
            if not JogCommand:
                return None, "JogCommand class not provided"

            cmd_obj = JogCommand(
                joint=joint_idx,
                speed_percentage=speed,
                duration=duration,
                distance_deg=distance
            )
            return cmd_obj, None

        except ValueError as e:
            return None, f"JOG parameter error: {e}"
        except Exception as e:
            return None, f"JOG parse error: {e}"

    def _parse_multi_jog(self, parts: List[str]) -> Tuple[Optional[Any], Optional[str]]:
        """Parse MULTIJOG command: MULTIJOG|joints|speeds|duration"""
        try:
            if len(parts) != 4:
                return None, f"MULTIJOG expects 4 parts, got {len(parts)}"

            joint_indices = [int(j) for j in parts[1].split(',')]
            speeds = [float(s) for s in parts[2].split(',')]
            duration = float(parts[3])

            MultiJogCommand = self.command_classes.get('MULTIJOG')
            if not MultiJogCommand:
                return None, "MultiJogCommand class not provided"

            cmd_obj = MultiJogCommand(
                joints=joint_indices,
                speed_percentages=speeds,
                duration=duration
            )
            return cmd_obj, None

        except ValueError as e:
            return None, f"MULTIJOG parameter error: {e}"
        except Exception as e:
            return None, f"MULTIJOG parse error: {e}"

    def _parse_cart_jog(self, parts: List[str]) -> Tuple[Optional[Any], Optional[str]]:
        """Parse CARTJOG command: CARTJOG|frame|axis|speed|duration"""
        try:
            if len(parts) != 5:
                return None, f"CARTJOG expects 5 parts, got {len(parts)}"

            frame = parts[1].upper()
            axis = parts[2]
            speed = float(parts[3])
            duration = float(parts[4])

            CartesianJogCommand = self.command_classes.get('CARTJOG')
            if not CartesianJogCommand:
                return None, "CartesianJogCommand class not provided"

            cmd_obj = CartesianJogCommand(
                frame=frame,
                axis=axis,
                speed_percentage=speed,
                duration=duration
            )
            return cmd_obj, None

        except ValueError as e:
            return None, f"CARTJOG parameter error: {e}"
        except Exception as e:
            return None, f"CARTJOG parse error: {e}"

    # ========================================================================
    # Utility Command Parsers
    # ========================================================================

    def _parse_home(self, parts: List[str]) -> Tuple[Optional[Any], Optional[str]]:
        """Parse HOME command: HOME"""
        try:
            HomeCommand = self.command_classes.get('HOME')
            if not HomeCommand:
                return None, "HomeCommand class not provided"

            cmd_obj = HomeCommand()
            return cmd_obj, None

        except Exception as e:
            return None, f"HOME parse error: {e}"

    def _parse_delay(self, parts: List[str]) -> Tuple[Optional[Any], Optional[str]]:
        """Parse DELAY command: DELAY|duration"""
        try:
            if len(parts) != 2:
                return None, f"DELAY expects 2 parts, got {len(parts)}"

            duration = float(parts[1])

            DelayCommand = self.command_classes.get('DELAY')
            if not DelayCommand:
                return None, "DelayCommand class not provided"

            cmd_obj = DelayCommand(duration=duration)
            return cmd_obj, None

        except ValueError as e:
            return None, f"DELAY parameter error: {e}"
        except Exception as e:
            return None, f"DELAY parse error: {e}"

    def _parse_pneumatic_gripper(self, parts: List[str]) -> Tuple[Optional[Any], Optional[str]]:
        """Parse PNEUMATICGRIPPER command: PNEUMATICGRIPPER|action|port"""
        try:
            if len(parts) != 3:
                return None, f"PNEUMATICGRIPPER expects 3 parts, got {len(parts)}"

            action = parts[1].lower()
            port = int(parts[2])

            GripperCommand = self.command_classes.get('PNEUMATICGRIPPER')
            if not GripperCommand:
                return None, "GripperCommand class not provided"

            cmd_obj = GripperCommand(
                gripper_type='pneumatic',
                action=action,
                output_port=port
            )
            return cmd_obj, None

        except ValueError as e:
            return None, f"PNEUMATICGRIPPER parameter error: {e}"
        except Exception as e:
            return None, f"PNEUMATICGRIPPER parse error: {e}"

    def _parse_electric_gripper(self, parts: List[str]) -> Tuple[Optional[Any], Optional[str]]:
        """Parse ELECTRICGRIPPER command: ELECTRICGRIPPER|action|pos|speed|current"""
        try:
            if len(parts) != 5:
                return None, f"ELECTRICGRIPPER expects 5 parts, got {len(parts)}"

            action_str = parts[1].upper()
            action = None if action_str == 'NONE' or action_str == 'MOVE' else parts[1].lower()
            pos = int(parts[2])
            spd = int(parts[3])
            curr = int(parts[4])

            GripperCommand = self.command_classes.get('ELECTRICGRIPPER')
            if not GripperCommand:
                return None, "GripperCommand class not provided"

            cmd_obj = GripperCommand(
                gripper_type='electric',
                action=action,
                position=pos,
                speed=spd,
                current=curr
            )
            return cmd_obj, None

        except ValueError as e:
            return None, f"ELECTRICGRIPPER parameter error: {e}"
        except Exception as e:
            return None, f"ELECTRICGRIPPER parse error: {e}"

    # ========================================================================
    # Smooth Motion Command Parsers
    # ========================================================================

    def _parse_smooth_motion(self, parts: List[str]) -> Tuple[Optional[Any], Optional[str]]:
        """
        Parse smooth motion commands.

        Handles: SMOOTH_CIRCLE, SMOOTH_ARC_CENTER, SMOOTH_ARC_PARAM,
                SMOOTH_SPLINE, SMOOTH_HELIX, SMOOTH_BLEND

        All commands support:
        - Reference frame selection (WRF or TRF)
        - Optional start position (CURRENT or specified pose)
        - Both DURATION and SPEED timing modes
        """
        command_type = parts[0]

        try:
            if command_type == 'SMOOTH_CIRCLE':
                return self._parse_smooth_circle(parts)
            elif command_type == 'SMOOTH_ARC_CENTER':
                return self._parse_smooth_arc_center(parts)
            elif command_type == 'SMOOTH_ARC_PARAM':
                return self._parse_smooth_arc_param(parts)
            elif command_type == 'SMOOTH_SPLINE':
                return self._parse_smooth_spline(parts)
            elif command_type == 'SMOOTH_HELIX':
                return self._parse_smooth_helix(parts)
            elif command_type == 'SMOOTH_BLEND':
                return self._parse_smooth_blend(parts)
            else:
                return None, f"Unknown smooth motion command: {command_type}"

        except Exception as e:
            self.logger.error(f"[CommandParser] Error parsing {command_type}: {e}")
            self.logger.debug(f"Command parts: {parts}")
            import traceback
            traceback.print_exc()
            return None, f"Smooth motion parse error: {e}"

    # ------------------------------------------------------------------------
    # Helper Methods
    # ------------------------------------------------------------------------

    @staticmethod
    def _parse_start_pose(start_str: str) -> Optional[List[float]]:
        """
        Parse start pose string.

        Returns:
            None for CURRENT/NONE, or list of 6 floats for specified pose
        """
        if start_str == 'CURRENT' or start_str == 'NONE':
            return None
        else:
            try:
                return list(map(float, start_str.split(',')))
            except:
                logging.warning(f"[CommandParser] Invalid start pose format: {start_str}")
                return None

    @staticmethod
    def _calculate_duration_from_speed(trajectory_length: float, speed_percentage: float) -> float:
        """
        Calculate duration based on trajectory length and speed percentage.

        Args:
            trajectory_length: Path length in mm
            speed_percentage: Speed as percentage (0-100)

        Returns:
            Duration in seconds
        """
        # Map speed percentage to mm/s
        min_speed = PAROL6_ROBOT.Cartesian_linear_velocity_min * 1000  # m/s to mm/s
        max_speed = PAROL6_ROBOT.Cartesian_linear_velocity_max * 1000  # m/s to mm/s
        speed_mm_s = np.interp(speed_percentage, [0, 100], [min_speed, max_speed])

        if speed_mm_s > 0:
            return trajectory_length / speed_mm_s
        else:
            return 5.0  # Default fallback

    # ------------------------------------------------------------------------
    # Individual Smooth Motion Parsers
    # ------------------------------------------------------------------------

    def _parse_smooth_circle(self, parts: List[str]) -> Tuple[Optional[Any], Optional[str]]:
        """
        Parse SMOOTH_CIRCLE command.
        Format: SMOOTH_CIRCLE|center_x,center_y,center_z|radius|plane|frame|start_pose|timing_type|timing_value|clockwise
        """
        try:
            center = list(map(float, parts[1].split(',')))
            radius = float(parts[2])
            plane = parts[3]
            frame = parts[4]  # 'WRF' or 'TRF'
            start_pose = self._parse_start_pose(parts[5])
            timing_type = parts[6]  # 'DURATION' or 'SPEED'
            timing_value = float(parts[7])
            clockwise = parts[8] == '1'

            # Calculate duration
            if timing_type == 'DURATION':
                duration = timing_value
            else:  # SPEED
                # Circle circumference
                path_length = 2 * np.pi * radius
                duration = self._calculate_duration_from_speed(path_length, timing_value)

            self.logger.debug(f"[CommandParser] Parsed circle: r={radius}mm, plane={plane}, "
                            f"frame={frame}, {timing_type}={timing_value}, duration={duration:.2f}s")

            SmoothCircleCommand = self.command_classes.get('SMOOTH_CIRCLE')
            if not SmoothCircleCommand:
                return None, "SmoothCircleCommand class not provided"

            cmd_obj = SmoothCircleCommand(center, radius, plane, duration, clockwise, frame, start_pose)
            return cmd_obj, None

        except Exception as e:
            return None, f"SMOOTH_CIRCLE parse error: {e}"

    def _parse_smooth_arc_center(self, parts: List[str]) -> Tuple[Optional[Any], Optional[str]]:
        """
        Parse SMOOTH_ARC_CENTER command.
        Format: SMOOTH_ARC_CENTER|end_pose|center|frame|start_pose|timing_type|timing_value|clockwise
        """
        try:
            end_pose = list(map(float, parts[1].split(',')))
            center = list(map(float, parts[2].split(',')))
            frame = parts[3]  # 'WRF' or 'TRF'
            start_pose = self._parse_start_pose(parts[4])
            timing_type = parts[5]  # 'DURATION' or 'SPEED'
            timing_value = float(parts[6])
            clockwise = parts[7] == '1'

            # Calculate duration
            if timing_type == 'DURATION':
                duration = timing_value
            else:  # SPEED
                # Estimate arc length
                radius_estimate = np.linalg.norm(np.array(center) - np.array(end_pose[:3]))
                estimated_arc_angle = np.pi / 2  # 90 degrees estimate
                arc_length = radius_estimate * estimated_arc_angle
                duration = self._calculate_duration_from_speed(arc_length, timing_value)

            self.logger.debug(f"[CommandParser] Parsed arc (center): frame={frame}, "
                            f"{timing_type}={timing_value}, duration={duration:.2f}s")

            SmoothArcCenterCommand = self.command_classes.get('SMOOTH_ARC_CENTER')
            if not SmoothArcCenterCommand:
                return None, "SmoothArcCenterCommand class not provided"

            cmd_obj = SmoothArcCenterCommand(end_pose, center, duration, clockwise, frame, start_pose)
            return cmd_obj, None

        except Exception as e:
            return None, f"SMOOTH_ARC_CENTER parse error: {e}"

    def _parse_smooth_arc_param(self, parts: List[str]) -> Tuple[Optional[Any], Optional[str]]:
        """
        Parse SMOOTH_ARC_PARAM command.
        Format: SMOOTH_ARC_PARAM|end_pose|radius|angle|frame|start_pose|timing_type|timing_value|clockwise
        """
        try:
            end_pose = list(map(float, parts[1].split(',')))
            radius = float(parts[2])
            arc_angle = float(parts[3])
            frame = parts[4]  # 'WRF' or 'TRF'
            start_pose = self._parse_start_pose(parts[5])
            timing_type = parts[6]  # 'DURATION' or 'SPEED'
            timing_value = float(parts[7])
            clockwise = parts[8] == '1'

            # Calculate duration
            if timing_type == 'DURATION':
                duration = timing_value
            else:  # SPEED
                # Arc length = radius * angle (in radians)
                arc_length = radius * np.deg2rad(arc_angle)
                duration = self._calculate_duration_from_speed(arc_length, timing_value)

            self.logger.debug(f"[CommandParser] Parsed arc (param): r={radius}mm, θ={arc_angle}°, "
                            f"frame={frame}, duration={duration:.2f}s")

            SmoothArcParamCommand = self.command_classes.get('SMOOTH_ARC_PARAM')
            if not SmoothArcParamCommand:
                return None, "SmoothArcParamCommand class not provided"

            cmd_obj = SmoothArcParamCommand(end_pose, radius, arc_angle, duration, clockwise, frame, start_pose)
            return cmd_obj, None

        except Exception as e:
            return None, f"SMOOTH_ARC_PARAM parse error: {e}"

    def _parse_smooth_spline(self, parts: List[str]) -> Tuple[Optional[Any], Optional[str]]:
        """
        Parse SMOOTH_SPLINE command.
        Format: SMOOTH_SPLINE|num_waypoints|frame|start_pose|timing_type|timing_value|waypoint1|waypoint2|...
        """
        try:
            num_waypoints = int(parts[1])
            frame = parts[2]  # 'WRF' or 'TRF'
            start_pose = self._parse_start_pose(parts[3])
            timing_type = parts[4]  # 'DURATION' or 'SPEED'
            timing_value = float(parts[5])

            # Parse waypoints
            waypoints = []
            idx = 6
            for i in range(num_waypoints):
                wp = []
                for j in range(6):  # Each waypoint has 6 values (x,y,z,rx,ry,rz)
                    wp.append(float(parts[idx]))
                    idx += 1
                waypoints.append(wp)

            # Calculate duration
            if timing_type == 'DURATION':
                duration = timing_value
            else:  # SPEED
                # Calculate total path length
                total_dist = 0
                for i in range(1, len(waypoints)):
                    dist = np.linalg.norm(np.array(waypoints[i][:3]) - np.array(waypoints[i-1][:3]))
                    total_dist += dist

                duration = self._calculate_duration_from_speed(total_dist, timing_value)

            self.logger.debug(f"[CommandParser] Parsed spline: {num_waypoints} points, "
                            f"frame={frame}, duration={duration:.2f}s")

            SmoothSplineCommand = self.command_classes.get('SMOOTH_SPLINE')
            if not SmoothSplineCommand:
                return None, "SmoothSplineCommand class not provided"

            cmd_obj = SmoothSplineCommand(waypoints, duration, frame, start_pose)
            return cmd_obj, None

        except Exception as e:
            return None, f"SMOOTH_SPLINE parse error: {e}"

    def _parse_smooth_helix(self, parts: List[str]) -> Tuple[Optional[Any], Optional[str]]:
        """
        Parse SMOOTH_HELIX command.
        Format: SMOOTH_HELIX|center|radius|pitch|height|frame|start_pose|timing_type|timing_value|clockwise
        """
        try:
            center = list(map(float, parts[1].split(',')))
            radius = float(parts[2])
            pitch = float(parts[3])
            height = float(parts[4])
            frame = parts[5]  # 'WRF' or 'TRF'
            start_pose = self._parse_start_pose(parts[6])
            timing_type = parts[7]  # 'DURATION' or 'SPEED'
            timing_value = float(parts[8])
            clockwise = parts[9] == '1'

            # Calculate duration
            if timing_type == 'DURATION':
                duration = timing_value
            else:  # SPEED
                # Calculate helix path length
                num_revolutions = height / pitch if pitch > 0 else 1
                horizontal_length = 2 * np.pi * radius * num_revolutions
                helix_length = np.sqrt(horizontal_length**2 + height**2)
                duration = self._calculate_duration_from_speed(helix_length, timing_value)

            self.logger.debug(f"[CommandParser] Parsed helix: h={height}mm, pitch={pitch}mm, "
                            f"frame={frame}, duration={duration:.2f}s")

            SmoothHelixCommand = self.command_classes.get('SMOOTH_HELIX')
            if not SmoothHelixCommand:
                return None, "SmoothHelixCommand class not provided"

            cmd_obj = SmoothHelixCommand(center, radius, pitch, height, duration, clockwise, frame, start_pose)
            return cmd_obj, None

        except Exception as e:
            return None, f"SMOOTH_HELIX parse error: {e}"

    def _parse_smooth_blend(self, parts: List[str]) -> Tuple[Optional[Any], Optional[str]]:
        """
        Parse SMOOTH_BLEND command.
        Format: SMOOTH_BLEND|num_segments|blend_time|frame|start_pose|timing_type|timing_value|segment1||segment2||...

        Note: This is a complex parser handling multiple segment types.
        """
        try:
            num_segments = int(parts[1])
            blend_time = float(parts[2])
            frame = parts[3]  # 'WRF' or 'TRF'
            start_pose = self._parse_start_pose(parts[4])
            timing_type = parts[5]  # 'DEFAULT', 'DURATION', or 'SPEED'

            # Parse overall timing
            if timing_type == 'DEFAULT':
                overall_duration = None
                overall_speed = None
                segments_start_idx = 6
            else:
                timing_value = float(parts[6])
                if timing_type == 'DURATION':
                    overall_duration = timing_value
                    overall_speed = None
                else:  # SPEED
                    overall_speed = timing_value
                    overall_duration = None
                segments_start_idx = 7

            # Parse segments (separated by ||)
            segments_data = '|'.join(parts[segments_start_idx:])
            segment_strs = segments_data.split('||')

            # Parse segment definitions
            segment_definitions = []
            total_original_duration = 0
            total_estimated_length = 0

            for seg_str in segment_strs:
                if not seg_str:  # Skip empty segments
                    continue

                seg_parts = seg_str.split('|')
                seg_type = seg_parts[0]

                if seg_type == 'LINE':
                    # Format: LINE|end_x,end_y,end_z,end_rx,end_ry,end_rz|duration
                    end = list(map(float, seg_parts[1].split(',')))
                    segment_duration = float(seg_parts[2])
                    total_original_duration += segment_duration

                    # Estimate length
                    estimated_length = 100  # mm, conservative estimate
                    total_estimated_length += estimated_length

                    segment_definitions.append({
                        'type': 'LINE',
                        'end': end,
                        'duration': segment_duration,
                        'original_duration': segment_duration
                    })

                elif seg_type == 'CIRCLE':
                    # Format: CIRCLE|center_x,center_y,center_z|radius|plane|duration|clockwise
                    center = list(map(float, seg_parts[1].split(',')))
                    radius = float(seg_parts[2])
                    plane = seg_parts[3]
                    segment_duration = float(seg_parts[4])
                    total_original_duration += segment_duration
                    clockwise = seg_parts[5] == '1'

                    # Circle circumference
                    estimated_length = 2 * np.pi * radius
                    total_estimated_length += estimated_length

                    segment_definitions.append({
                        'type': 'CIRCLE',
                        'center': center,
                        'radius': radius,
                        'plane': plane,
                        'duration': segment_duration,
                        'clockwise': clockwise,
                        'original_duration': segment_duration
                    })

            # Apply overall timing scaling if specified
            if overall_duration is not None:
                if total_original_duration > 0:
                    scale_factor = overall_duration / total_original_duration
                    for seg in segment_definitions:
                        seg['duration'] = seg['original_duration'] * scale_factor
                self.logger.debug(f"[CommandParser] Scaled blend segments to total duration: {overall_duration:.2f}s")

            elif overall_speed is not None:
                # Calculate duration from speed and estimated path length
                overall_duration = self._calculate_duration_from_speed(total_estimated_length, overall_speed)
                if total_original_duration > 0:
                    scale_factor = overall_duration / total_original_duration
                    for seg in segment_definitions:
                        seg['duration'] = seg['original_duration'] * scale_factor
                self.logger.debug(f"[CommandParser] Calculated blend duration from speed: {overall_duration:.2f}s")
            else:
                self.logger.debug(f"[CommandParser] Using original segment durations (total: {total_original_duration:.2f}s)")

            self.logger.debug(f"[CommandParser] Parsed blend: {num_segments} segments, "
                            f"frame={frame}, blend_time={blend_time}s")

            SmoothBlendCommand = self.command_classes.get('SMOOTH_BLEND')
            if not SmoothBlendCommand:
                return None, "SmoothBlendCommand class not provided"

            cmd_obj = SmoothBlendCommand(segment_definitions, blend_time, frame, start_pose)
            return cmd_obj, None

        except Exception as e:
            return None, f"SMOOTH_BLEND parse error: {e}"


# ============================================================================
# Utility Functions
# ============================================================================

def parse_command_with_id(message: str) -> Tuple[Optional[str], str]:
    """
    Extract command ID from message if present.

    Format: [CMD_ID]COMMAND|params

    Args:
        message: Raw UDP message

    Returns:
        Tuple of (command_id, command_message)
        - If ID present: ("abc123", "MOVEJOINT|...")
        - If no ID: (None, "MOVEJOINT|...")

    Example:
        >>> parse_command_with_id("[abc123]MOVEJOINT|0,0,0,0,0,0|2.5|50")
        ("abc123", "MOVEJOINT|0,0,0,0,0,0|2.5|50")
    """
    if message.startswith('['):
        end_bracket_idx = message.find(']')
        if end_bracket_idx > 0:
            cmd_id = message[1:end_bracket_idx]
            command_message = message[end_bracket_idx+1:]

            # Validate command ID
            parts = command_message.split('|')
            if (parts and
                len(parts[0]) > 0 and
                not parts[0].isupper()):  # Prevents "MOVEPOSE" being treated as an ID
                # Invalid - first part should be uppercase command name
                return None, message

            return cmd_id, command_message

    return None, message


# ============================================================================
# Module Metadata
# ============================================================================

__version__ = "1.0.0"
__author__ = "PAROL6 Team"
__date__ = "2025-01-13"
__description__ = "Command parsing module for PAROL6 robot control system"
