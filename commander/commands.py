"""
Command Classes for PAROL6 Robot

This module contains all command class definitions for the PAROL6 robot control system.
Commands implement a two-phase execution model:
1. __init__(): Initial validation and parameter storage
2. prepare_for_execution(): JIT trajectory generation using live robot state
3. execute_step(): Non-blocking execution called every control loop cycle (0.01s)

Extracted from headless_commander.py as part of Tier 2 refactoring.
"""

import logging
import time
import numpy as np
from spatialmath import SE3
from math import pi
import roboticstoolbox as rp

# Import robot model and motion generators from lib/
from lib.kinematics import robot_model as PAROL6_ROBOT

# Module logger
logger = logging.getLogger(__name__)

# Constants
INTERVAL_S = 0.01  # Control loop interval in seconds (100Hz)


def quintic_scaling(s: float) -> float:
    """
    Calculates a smooth 0-to-1 scaling factor for progress 's'
    using a quintic polynomial, ensuring smooth start/end accelerations.
    """
    return 6 * (s**5) - 15 * (s**4) + 10 * (s**3)

#########################################################################
# Robot Commands
#########################################################################
class HomeCommand:
    """
    A non-blocking command that tells the robot to perform its internal homing sequence.
    This version uses a state machine to allow re-homing even if the robot is already homed.
    """
    def __init__(self):
        self.is_valid = True
        self.is_finished = False
        # State machine: START -> WAIT_FOR_UNHOMED -> WAIT_FOR_HOMED -> FINISHED
        self.state = "START"
        # Counter to send the home command for multiple cycles
        self.start_cmd_counter = 10  # Send command 100 for 10 cycles (0.1s)
        # Safety timeout (20 seconds at 0.01s interval)
        self.timeout_counter = 2000
        logger.info("Initializing Home command...")

    def execute_step(self, Position_in, Homed_in, Speed_out, Command_out, **kwargs):
        """
        Manages the homing command and monitors for completion using a state machine.
        """
        if self.is_finished:
            return True

        # --- State: START ---
        # On the first few executions, continuously send the 'home' (100) command.
        if self.state == "START":
            logger.debug(f"  -> Sending home signal (100)... Countdown: {self.start_cmd_counter}")
            Command_out.value = 100
            self.start_cmd_counter -= 1
            if self.start_cmd_counter <= 0:
                # Once sent for enough cycles, move to the next state
                self.state = "WAITING_FOR_UNHOMED"
            return False

        # --- State: WAITING_FOR_UNHOMED ---
        # The robot's firmware should reset the homed status. We wait to see that happen.
        # During this time, we send 'idle' (255) to let the robot's controller take over.
        if self.state == "WAITING_FOR_UNHOMED":
            Command_out.value = 255
            # Check if at least one joint has started homing (is no longer homed)
            if any(h == 0 for h in Homed_in[:6]):
                logger.debug("  -> Homing sequence initiated by robot.")
                self.state = "WAITING_FOR_HOMED"
            # Check for timeout
            self.timeout_counter -= 1
            if self.timeout_counter <= 0:
                logger.debug("  -> ERROR: Timeout waiting for robot to start homing sequence.")
                self.is_finished = True
            return self.is_finished

        # --- State: WAITING_FOR_HOMED ---
        # Now we wait for all joints to report that they are homed (all flags are 1).
        if self.state == "WAITING_FOR_HOMED":
            Command_out.value = 255
            # Check if all joints have finished homing
            if all(h == 1 for h in Homed_in[:6]):
                logger.info("Homing sequence complete. All joints reported home.")
                self.is_finished = True
                Speed_out[:] = [0] * 6 # Ensure robot is stopped

        return self.is_finished


class MoveJointCommand:
    """
    A non-blocking command to move the robot's joints to a specific configuration.
    It pre-calculates the entire trajectory upon initialization.
    """
    def __init__(self, target_angles, duration=None, velocity_percent=None, accel_percent=50, trajectory_type='poly'):
        self.is_valid = False  # Will be set to True after basic validation
        self.is_finished = False
        self.command_step = 0
        self.trajectory_steps = []

        logger.info(f"Initializing MoveJoint to {target_angles}...")

        # --- MODIFICATION: Store parameters for deferred planning ---
        self.target_angles = target_angles
        self.duration = duration
        self.velocity_percent = velocity_percent
        self.accel_percent = accel_percent
        self.trajectory_type = trajectory_type

        # --- Perform only state-independent validation ---
        target_pos_rad = np.array([np.deg2rad(angle) for angle in self.target_angles])
        for i in range(6):
            min_rad, max_rad = PAROL6_ROBOT.Joint_limits_radian[i]
            if not (min_rad <= target_pos_rad[i] <= max_rad):
                logger.debug(f"  -> VALIDATION FAILED: Target for Joint {i+1} ({self.target_angles[i]} deg) is out of range.")
                return
        
        self.is_valid = True

    def prepare_for_execution(self, current_position_in):
        """Calculates the trajectory just before execution begins."""
        logger.info(f"[DEBUG] Preparing trajectory for MoveJoint: duration={self.duration}, velocity_percent={self.velocity_percent}")

        initial_pos_rad = np.array([PAROL6_ROBOT.STEPS2RADS(p, i) for i, p in enumerate(current_position_in)])
        target_pos_rad = np.array([np.deg2rad(angle) for angle in self.target_angles])

        if self.duration and self.duration > 0:
            logger.info(f"[DEBUG] Using duration path: {self.duration}s")
            if self.velocity_percent is not None:
                logger.debug("  -> INFO: Both duration and velocity were provided. Using duration.")
            command_len = int(self.duration / INTERVAL_S)
            traj_generator = rp.tools.trajectory.jtraj(initial_pos_rad, target_pos_rad, command_len)
            
            for i in range(len(traj_generator.q)):
                pos_step = [int(PAROL6_ROBOT.RAD2STEPS(p, j)) for j, p in enumerate(traj_generator.q[i])]
                self.trajectory_steps.append((pos_step, None))

        elif self.velocity_percent is not None:
            logger.info(f"[DEBUG] Entering velocity_percent path")
            try:
                accel_percent = self.accel_percent if self.accel_percent is not None else 50
                initial_pos_steps = np.array(current_position_in)
                target_pos_steps = np.array([int(PAROL6_ROBOT.RAD2STEPS(rad, i)) for i, rad in enumerate(target_pos_rad)])
                logger.info(f"[DEBUG] initial_pos_steps: {initial_pos_steps}")
                logger.info(f"[DEBUG] target_pos_steps: {target_pos_steps}")
                
                all_joint_times = []
                for i in range(6):
                    path_to_travel = abs(target_pos_steps[i] - initial_pos_steps[i])
                    if path_to_travel == 0:
                        all_joint_times.append(0)
                        continue

                    v_max_joint = np.interp(self.velocity_percent, [0, 100], [PAROL6_ROBOT.Joint_min_speed[i], PAROL6_ROBOT.Joint_max_speed[i]])
                    a_max_rad = np.interp(accel_percent, [0, 100], [PAROL6_ROBOT.Joint_min_acc, PAROL6_ROBOT.Joint_max_acc])
                    a_max_steps = PAROL6_ROBOT.SPEED_RAD2STEP(a_max_rad, i)

                    if v_max_joint <= 0 or a_max_steps <= 0:
                        raise ValueError(f"Invalid speed/acceleration for joint {i+1}. Must be positive.")

                    t_accel = v_max_joint / a_max_steps
                    if path_to_travel < v_max_joint * t_accel:
                        t_accel = np.sqrt(path_to_travel / a_max_steps)
                        joint_time = 2 * t_accel
                    else:
                        joint_time = path_to_travel / v_max_joint + t_accel
                    all_joint_times.append(joint_time)

                total_time = max(all_joint_times)

                if total_time <= 0:
                    self.is_finished = True
                    return

                if total_time < (2 * INTERVAL_S):
                    total_time = 2 * INTERVAL_S

                execution_time = np.arange(0, total_time, INTERVAL_S)
                
                all_q, all_qd = [], []
                for i in range(6):
                    if abs(target_pos_steps[i] - initial_pos_steps[i]) == 0:
                        all_q.append(np.full(len(execution_time), initial_pos_steps[i]))
                        all_qd.append(np.zeros(len(execution_time)))
                    else:
                        joint_traj = rp.trapezoidal(initial_pos_steps[i], target_pos_steps[i], execution_time)
                        all_q.append(joint_traj.q)
                        all_qd.append(joint_traj.qd)

                self.trajectory_steps = list(zip(np.array(all_q).T.astype(int), np.array(all_qd).T.astype(int)))
                logger.info(f"[DEBUG] Trajectory generated successfully: {len(self.trajectory_steps)} steps")
                logger.debug(f"  -> Command is valid (duration calculated from speed: {total_time:.2f}s).")

            except Exception as e:
                logger.error(f"[DEBUG] VALIDATION FAILED: Could not calculate velocity-based trajectory. Error: {e}")
                logger.debug(f"  -> Please check Joint_min/max_speed and Joint_min/max_acc values in PAROL6_ROBOT.py.")
                self.is_valid = False
                return
        
        else:
            logger.debug("  -> Using conservative values for MoveJoint.")
            command_len = 200
            traj_generator = rp.tools.trajectory.jtraj(initial_pos_rad, target_pos_rad, command_len)
            for i in range(len(traj_generator.q)):
                pos_step = [int(PAROL6_ROBOT.RAD2STEPS(p, j)) for j, p in enumerate(traj_generator.q[i])]
                self.trajectory_steps.append((pos_step, None))
        
        if not self.trajectory_steps:
             logger.error(" -> Trajectory calculation resulted in no steps. Command is invalid.")
             self.is_valid = False
        else:
             logger.debug(f" -> Trajectory prepared with {len(self.trajectory_steps)} steps.")

    def execute_step(self, Position_in, Homed_in, Speed_out, Command_out, **kwargs):
        # This method remains unchanged.
        Position_out = kwargs.get('Position_out', Position_in)

        if self.is_finished or not self.is_valid:
            return True

        if self.command_step >= len(self.trajectory_steps):
            logger.info(f"{type(self).__name__} finished.")
            self.is_finished = True
            Position_out[:] = Position_in[:]
            Speed_out[:] = [0] * 6
            Command_out.value = 156
            return True
        else:
            pos_step, _ = self.trajectory_steps[self.command_step]
            Position_out[:] = pos_step
            Speed_out[:] = [0] * 6
            Command_out.value = 156
            self.command_step += 1
            return False

class ExecuteTrajectoryCommand:
    """
    Execute a pre-computed joint trajectory at 100Hz.

    This command is designed for Cartesian straight-line motion where:
    1. Frontend generates Cartesian waypoints dynamically
    2. Backend batch IK solves all waypoints ONCE (offline)
    3. This command plays back the joint trajectory at 100Hz

    Unlike MoveCartCommand (which solves IK every cycle → 16Hz),
    this achieves 100Hz by using pre-computed joint positions.
    """
    def __init__(self, trajectory_deg, duration=None):
        """
        Initialize ExecuteTrajectoryCommand.

        Parameters
        ----------
        trajectory_deg : list of list of float
            Pre-computed joint trajectory, each waypoint is [J1-J6] in degrees
        duration : float, optional
            Expected duration in seconds (for validation)
        """
        self.is_valid = False
        self.is_finished = False
        self.command_step = 0
        self.trajectory_steps = []

        logger.info(f"Initializing ExecuteTrajectory with {len(trajectory_deg)} waypoints...")

        # Store parameters
        self.trajectory_deg = trajectory_deg
        self.duration = duration

        # Validate trajectory
        if not trajectory_deg or len(trajectory_deg) == 0:
            logger.debug("  -> VALIDATION FAILED: Empty trajectory")
            return

        # Validate each waypoint has 6 joints
        for i, waypoint in enumerate(trajectory_deg):
            if len(waypoint) != 6:
                logger.debug(f"  -> VALIDATION FAILED: Waypoint {i} has {len(waypoint)} joints (expected 6)")
                return

        # Validate joint limits
        for i, waypoint in enumerate(trajectory_deg):
            waypoint_rad = np.array([np.deg2rad(angle) for angle in waypoint])
            for j in range(6):
                min_rad, max_rad = PAROL6_ROBOT.Joint_limits_radian[j]
                if not (min_rad <= waypoint_rad[j] <= max_rad):
                    logger.debug(f"  -> VALIDATION FAILED: Waypoint {i} Joint {j+1} ({waypoint[j]:.1f}°) out of range")
                    return

        # Validate duration if provided
        if duration is not None:
            expected_waypoints = int(duration / INTERVAL_S)
            actual_waypoints = len(trajectory_deg)
            if abs(expected_waypoints - actual_waypoints) > 5:  # Allow 5 waypoint tolerance
                logger.warning(f"  -> Duration mismatch: expected {expected_waypoints} waypoints for {duration}s, got {actual_waypoints}")

        self.is_valid = True
        logger.debug(f"  -> Trajectory validated successfully")

    def prepare_for_execution(self, current_position_in):
        """Convert trajectory from degrees to steps just before execution."""
        logger.debug(f"  -> Preparing ExecuteTrajectory with {len(self.trajectory_deg)} waypoints...")

        # Convert each waypoint from degrees to steps
        for waypoint_deg in self.trajectory_deg:
            pos_step = [int(PAROL6_ROBOT.DEG2STEPS(angle, j)) for j, angle in enumerate(waypoint_deg)]
            self.trajectory_steps.append((pos_step, None))

        logger.debug(f"  -> Trajectory prepared with {len(self.trajectory_steps)} steps")

    def execute_step(self, Position_in, Homed_in, Speed_out, Command_out, **kwargs):
        """Execute one step of the trajectory (called at 100Hz)."""
        Position_out = kwargs.get('Position_out', Position_in)

        if self.is_finished or not self.is_valid:
            return True

        if self.command_step >= len(self.trajectory_steps):
            logger.info(f"{type(self).__name__} finished.")
            self.is_finished = True
            Position_out[:] = Position_in[:]
            Speed_out[:] = [0] * 6
            Command_out.value = 156  # Position mode
            return True
        else:
            pos_step, _ = self.trajectory_steps[self.command_step]
            Position_out[:] = pos_step
            Speed_out[:] = [0] * 6
            Command_out.value = 156  # Position mode
            self.command_step += 1
            return False

class GripperCommand:
    """
    A single, unified, non-blocking command to control all gripper functions.
    It internally selects the correct logic (position-based waiting, timed delay,
    or instantaneous) based on the specified action.
    """
    def __init__(self, gripper_type, action=None, position=100, speed=100, current=500, output_port=1):
        """
        Initializes the Gripper command and configures its internal state machine
        based on the requested action.
        """
        self.is_valid = True
        self.is_finished = False
        self.gripper_type = gripper_type.lower()
        self.action = action.lower() if action else 'move'
        self.state = "START"
        self.timeout_counter = 1000 # 10-second safety timeout for all waiting states

        # --- Configure based on Gripper Type and Action ---
        if self.gripper_type == 'electric':
            if self.action == 'move':
                self.target_position = position
                self.speed = speed
                self.current = current
                if not (0 <= position <= 255 and 0 <= speed <= 255 and 100 <= current <= 1000):
                    self.is_valid = False
            elif self.action == 'calibrate':
                self.wait_counter = 200 # 2-second fixed delay for calibration
            else:
                self.is_valid = False # Invalid action

        elif self.gripper_type == 'pneumatic':
            if self.action not in ['open', 'close']:
                self.is_valid = False
            self.state_to_set = 1 if self.action == 'open' else 0
            self.port_index = 2 if output_port == 1 else 3
        else:
            self.is_valid = False

        if not self.is_valid:
            logger.debug(f"  -> VALIDATION FAILED for GripperCommand with action: '{self.action}'")

    def execute_step(self, Gripper_data_out, InOut_out, Gripper_data_in, InOut_in, **kwargs):
        if self.is_finished or not self.is_valid:
            return True

        self.timeout_counter -= 1
        if self.timeout_counter <= 0:
            logger.debug(f"  -> ERROR: Gripper command timed out in state {self.state}.")
            self.is_finished = True
            return True

        # --- Pneumatic Logic (Instantaneous) ---
        if self.gripper_type == 'pneumatic':
            InOut_out[self.port_index] = self.state_to_set
            logger.debug("  -> Pneumatic gripper command sent.")
            self.is_finished = True
            return True

        # --- Electric Gripper Logic ---
        if self.gripper_type == 'electric':
            # On the first run, transition to the correct state for the action
            if self.state == "START":
                if self.action == 'calibrate':
                    self.state = "SEND_CALIBRATE"
                else: # 'move'
                    self.state = "WAIT_FOR_POSITION"
            
            # --- Calibrate Logic (Timed Delay) ---
            if self.state == "SEND_CALIBRATE":
                logger.debug("  -> Sending one-shot calibrate command...")
                Gripper_data_out[4] = 1 # Set mode to calibrate
                self.state = "WAITING_CALIBRATION"
                return False

            if self.state == "WAITING_CALIBRATION":
                self.wait_counter -= 1
                if self.wait_counter <= 0:
                    logger.info(logger.debug("  -> Calibration delay finished."))
                    Gripper_data_out[4] = 0 # Reset to operation mode
                    self.is_finished = True
                    return True
                return False

            # --- Move Logic (Position-Based) ---
            if self.state == "WAIT_FOR_POSITION":
                # Persistently send the move command
                Gripper_data_out[0], Gripper_data_out[1], Gripper_data_out[2] = self.target_position, self.speed, self.current
                Gripper_data_out[4] = 0 # Operation mode
                bitfield = [1, 1, not InOut_in[4], 1, 0, 0, 0, 0]
                fused = PAROL6_ROBOT.fuse_bitfield_2_bytearray(bitfield)
                Gripper_data_out[3] = int(fused.hex(), 16)

                # Check for completion
                current_position = Gripper_data_in[1]
                if abs(current_position - self.target_position) <= 5:
                    logger.debug(f"  -> Gripper move complete.")
                    self.is_finished = True
                    # Set command back to idle
                    bitfield = [1, 0, not InOut_in[4], 1, 0, 0, 0, 0]
                    fused = PAROL6_ROBOT.fuse_bitfield_2_bytearray(bitfield)
                    Gripper_data_out[3] = int(fused.hex(), 16)
                    return True
                return False
        
        return self.is_finished

class DelayCommand:
    """
    A non-blocking command that pauses execution for a specified duration.
    During the delay, it ensures the robot remains idle by sending the
    appropriate commands.
    """
    def __init__(self, duration):
        """
        Initializes and validates the Delay command.

        Args:
            duration (float): The delay time in seconds.
        """
        self.is_valid = False
        self.is_finished = False

        # --- 1. Parameter Validation ---
        if not isinstance(duration, (int, float)) or duration <= 0:
            logger.debug(f"  -> VALIDATION FAILED: Delay duration must be a positive number, but got {duration}.")
            return

        logger.info(f"Initializing Delay for {duration} seconds...")
        
        self.duration = duration
        self.end_time = None  # Will be set in prepare_for_execution
        self.is_valid = True

    def prepare_for_execution(self, current_position_in):
        """Set the end time when the command actually starts."""
        self.end_time = time.time() + self.duration
        logger.debug(f"  -> Delay starting for {self.duration} seconds...")

    def execute_step(self, Position_in, Homed_in, Speed_out, Command_out, **kwargs):
        """
        Checks if the delay duration has passed and keeps the robot idle.
        This method is called on every loop cycle (~0.01s).
        """
        if self.is_finished or not self.is_valid:
            return True

        # --- A. Keep the robot idle during the delay ---
        Command_out.value = 255  # Set command to idle
        Speed_out[:] = [0] * 6   # Set all speeds to zero

        # --- B. Check for completion ---
        if self.end_time and time.time() >= self.end_time:
            logger.info(f"Delay finished after {self.duration} seconds.")
            self.is_finished = True
        
        return self.is_finished
    
