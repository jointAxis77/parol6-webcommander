'''
A full fledged "API" for the PAROL6 robot. To use this, you should pair it with the "robot_api.py" where you can import commands
from said file and use them anywhere within your code. This Python script will handle sending and performing all the commands
to the PAROL6 robot, as well as E-Stop functionality and safety limitations.

To run this program, you must use the "experimental-kinematics" branch of the "PAROL-commander-software" on GitHub
which can be found through this link: https://github.com/PCrnjak/PAROL-commander-software/tree/experimental_kinematics.
You must also save these files into the following folder: "Project Files\PAROL-commander-software\GUI\files".
'''

# * If you press estop robot will stop and you need to enable it by pressing e

# Apply numpy compatibility patch for numpy 2.0+
from lib.utils import numpy_patch

# Mock tkinter for headless environments (oclock imports it but we don't need GUI)
import sys
sys.modules['tkinter'] = type(sys)('tkinter')

from roboticstoolbox import DHRobot, RevoluteDH, ERobot, ELink, ETS, trapezoidal, quintic
import roboticstoolbox as rp
from math import pi, sin, cos
import numpy as np
from oclock import Timer, loop, interactiveloop
import time
import socket
from spatialmath import SE3
import select
import serial
import platform
import os
import re
import logging
import struct
import keyboard
from typing import Optional, Tuple
from spatialmath.base import trinterp
from collections import namedtuple, deque
import json
import datetime
from lib.kinematics import robot_model as PAROL6_ROBOT
from lib.kinematics.trajectory_math import CircularMotion, SplineMotion, MotionBlender
from lib.utils.logging_handler import setup_logging

# ============================================================================
# TIER 1 REFACTORING: Import from centralized modules (now in lib/)
# ============================================================================
# IK Solver
from lib.kinematics.ik_solver import (
    IKResult,
    normalize_angle,
    unwrap_angles,
    calculate_adaptive_tolerance,
    calculate_configuration_dependent_max_reach,
    solve_ik_with_adaptive_tol_subdivision
)

# Serial Protocol
from serial_protocol import (
    split_to_3_bytes as Split_2_3_bytes,
    fuse_3_bytes as Fuse_3_bytes,
    fuse_2_bytes as Fuse_2_bytes,
    split_to_bitfield as Split_2_bitfield,
    fuse_bitfield_to_byte as Fuse_bitfield_2_bytearray,
    pack_command_packet as Pack_data,
    unpack_feedback_packet as Unpack_data,
    START_BYTES as start_bytes,
    END_BYTES as end_bytes,
    START_COND1_BYTE as start_cond1_byte,
    START_COND2_BYTE as start_cond2_byte,
    START_COND3_BYTE as start_cond3_byte,
    END_COND1_BYTE as end_cond1_byte,
    END_COND2_BYTE as end_cond2_byte,
    INT_TO_3_BYTES as int_to_3_bytes
)

# TIER 2: Network and Command Parser modules
from network_handler import NetworkHandler
from command_parser import CommandParser
from command_queue import CommandQueue
from performance_monitor import PerformanceMonitor

# Command classes and utilities
from commands import (
    # Utility functions
    quintic_scaling,
    # Commands (simplified - frontend handles IK/FK)
    HomeCommand,
    MoveJointCommand,
    ExecuteTrajectoryCommand,
    GripperCommand,
    DelayCommand,
)
# ============================================================================

# Set interval
INTERVAL_S = 0.01
prev_time = 0

# Logging will be configured after config.yaml is loaded

# Load configuration from parent directory (project root)
import yaml
from pathlib import Path

# Get project root (parent of commander directory)
PROJECT_ROOT = Path(__file__).parent.parent
CONFIG_PATH = PROJECT_ROOT / "config.yaml"

try:
    with open(CONFIG_PATH, "r") as f:
        config = yaml.safe_load(f)
except FileNotFoundError:
    # Use basic logging for this early warning before setup_logging is called
    logging.basicConfig(level=logging.WARNING)
    logging.warning(f"config.yaml not found at {CONFIG_PATH}, using defaults")
    config = {
        'robot': {'com_port': 'COM6', 'baud_rate': 3000000, 'timeout': 0}
    }

# Set up logging with WebSocket handler (after config is loaded)
logging_config = config.get('logging', {})
setup_logging(logging_config, 'commander')
logger = logging.getLogger('commander')

# Explicitly set logger level to match config
log_level = logging_config.get('commander', {}).get('level', 'INFO')
logger.setLevel(getattr(logging, log_level.upper()))

# Connect to robot via serial port (works on all platforms)
try:
    com_port_str = config['robot']['com_port']
    baud_rate = config['robot'].get('baud_rate', 3000000)
    timeout = config['robot'].get('timeout', 0)

    ser = serial.Serial(port=com_port_str, baudrate=baud_rate, timeout=timeout)
    logger.info(f"Connected to serial port from config: {com_port_str}")
except (KeyError, serial.SerialException) as e:
    logger.warning(f"Could not connect to configured port: {e}")
    logger.warning("Main loop will continue attempting to reconnect every second...")
    ser = None

# in big endian machines, first byte of binary representation of the multibyte data-type is stored first. 
int_to_3_bytes = struct.Struct('>I').pack # BIG endian order

# data for output string (data that is being sent to the robot)
#######################################################################################
#######################################################################################
start_bytes =  [0xff,0xff,0xff] 
start_bytes = bytes(start_bytes)

end_bytes =  [0x01,0x02] 
end_bytes = bytes(end_bytes)


# data for input string (Data that is being sent by the robot)
#######################################################################################
#######################################################################################
input_byte = 0 # Here save incoming bytes from serial

start_cond1_byte = bytes([0xff])
start_cond2_byte = bytes([0xff])
start_cond3_byte = bytes([0xff])

end_cond1_byte = bytes([0x01])
end_cond2_byte = bytes([0x02])

start_cond1 = 0 #Flag if start_cond1_byte is received
start_cond2 = 0 #Flag if start_cond2_byte is received
start_cond3 = 0 #Flag if start_cond3_byte is received

good_start = 0 #Flag if we got all 3 start condition bytes
data_len = 0 #Length of the data after -3 start condition bytes and length byte, so -4 bytes

data_buffer = [None]*255 #Here save all data after data length byte
data_counter = 0 #Data counter for incoming bytes; compared to data length to see if we have correct length
#######################################################################################
#######################################################################################
prev_positions = [0,0,0,0,0,0]
prev_speed = [0,0,0,0,0,0]
robot_pose = [0,0,0,0,0,0] #np.array([0,0,0,0,0,0])
#######################################################################################
#######################################################################################

# --- Wrapper class to make integers mutable when passed to functions ---
class CommandValue:
    def __init__(self, value):
        self.value = value

#######################################################################################
#######################################################################################
Position_out = [1,11,111,1111,11111,10]
Speed_out = [2,21,22,23,24,25]
Command_out = CommandValue(255)
Affected_joint_out = [1,1,1,1,1,1,1,1]
InOut_out = [0,0,0,0,0,0,0,0]
Timeout_out = 0
#Positon,speed,current,command,mode,ID
Gripper_data_out = [1,1,1,1,0,0]
#######################################################################################
#######################################################################################
# Data sent from robot to PC
Position_in = [31,32,33,34,35,36]
Speed_in = [41,42,43,44,45,46]
Homed_in = [0,0,0,0,0,0,0,0]
InOut_in = [1,1,1,1,1,1,1,1]
Temperature_error_in = [1,1,1,1,1,1,1,1]
Position_error_in = [1,1,1,1,1,1,1,1]
Timeout_error = 0
# how much time passed between 2 sent commands (2byte value, last 2 digits are decimal so max value is 655.35ms?)
Timing_data_in = [0]
XTR_data =   0

# --- State variables for program execution ---
Robot_mode = "Dummy"  # Start in an idle state
Program_step = 0      # Which line of the program to run
Command_step = 0      # The current step within a single command
Command_len = 0       # The total steps for the current command
ik_error = 0          # Flag for inverse kinematics errors
error_state = 0       # General error flag
program_running = False # A flag to start and stop the program

# This will be your "program"
command_list = []

#ID,Position,speed,current,status,obj_detection
Gripper_data_in = [1,1,1,1,1,1] 

# Global variable to track previous tolerance for logging changes
_prev_tolerance = None

# ============================================================================
# IK Functions now imported from ik_solver.py (see imports above)
# REFACTORED: Removed ~215 lines of duplicate IK code
# ============================================================================

# ============================================================================
# TIER 2: Initialize NetworkHandler and CommandParser
# ============================================================================
# Get ports from config or use defaults
command_port = config.get('server', {}).get('command_port', 5001)
ack_port = config.get('server', {}).get('ack_port', 5002)

# Initialize network handler for UDP communication
network_handler = NetworkHandler(
    logger=logger,
    listen_ip="127.0.0.1",
    command_port=command_port,
    ack_port=ack_port,
    buffer_max_size=100
)
# Bind UDP sockets
if not network_handler.initialize():
    logger.error("Failed to initialize NetworkHandler!")
    import sys
    sys.exit(1)
logger.info(f'NetworkHandler initialized: listening on port {command_port}, sending ACKs on port {ack_port}')

# Initialize command parser (simplified - frontend handles IK/FK)
command_classes = {
    'HOME': HomeCommand,
    'MOVEJOINT': MoveJointCommand,
    'EXECUTETRAJECTORY': ExecuteTrajectoryCommand,
    'PNEUMATICGRIPPER': GripperCommand,
    'ELECTRICGRIPPER': GripperCommand,
    'DELAY': DelayCommand,
}
command_parser = CommandParser(logger, robot_model=PAROL6_ROBOT)
logger.info(f'CommandParser initialized with {len(command_classes)} command types')
# ============================================================================

def Unpack_data(data_buffer_list, Position_in,Speed_in,Homed_in,InOut_in,Temperature_error_in,Position_error_in,Timeout_error,Timing_data_in,
         XTR_data,Gripper_data_in):

    Joints = []
    Speed = []

    for i in range(0,18, 3):
        variable = data_buffer_list[i:i+3]
        Joints.append(variable)

    for i in range(18,36, 3):
        variable = data_buffer_list[i:i+3]
        Speed.append(variable)


    for i in range(6):
        var =  b'\x00' + b''.join(Joints[i])
        Position_in[i] = Fuse_3_bytes(var)
        var =  b'\x00' + b''.join(Speed[i])
        Speed_in[i] = Fuse_3_bytes(var)

    Homed = data_buffer_list[36]
    IO_var = data_buffer_list[37]
    temp_error = data_buffer_list[38]
    position_error = data_buffer_list[39]
    timing_data = data_buffer_list[40:42]
    Timeout_error_var = data_buffer_list[42]
    xtr2 = data_buffer_list[43]
    device_ID = data_buffer_list[44]
    Gripper_position = data_buffer_list[45:47]
    Gripper_speed = data_buffer_list[47:49]
    Gripper_current = data_buffer_list[49:51]
    Status = data_buffer_list[51]
    # The original object_detection byte at index 52 is ignored as it is not reliable.
    CRC_byte = data_buffer_list[53]
    endy_byte1 = data_buffer_list[54]
    endy_byte2 = data_buffer_list[55]

    # ... (Code for Homed, IO_var, temp_error, etc. remains the same) ...

    temp = Split_2_bitfield(int.from_bytes(Homed,"big"))
    for i in range(8):
        Homed_in[i] = temp[i]

    temp = Split_2_bitfield(int.from_bytes(IO_var,"big"))
    for i in range(8):
        InOut_in[i] = temp[i]

    temp = Split_2_bitfield(int.from_bytes(temp_error,"big"))
    for i in range(8):
        Temperature_error_in[i] = temp[i]

    temp = Split_2_bitfield(int.from_bytes(position_error,"big"))
    for i in range(8):
        Position_error_in[i] = temp[i]

    var = b'\x00' + b'\x00' + b''.join(timing_data)
    Timing_data_in[0] = Fuse_3_bytes(var)
    Timeout_error = int.from_bytes(Timeout_error_var,"big")
    XTR_data = int.from_bytes(xtr2,"big")

    # --- Gripper Data Unpacking ---
    Gripper_data_in[0] = int.from_bytes(device_ID,"big")

    var =  b'\x00'+ b'\x00' + b''.join(Gripper_position)
    Gripper_data_in[1] = Fuse_2_bytes(var)

    var =  b'\x00'+ b'\x00' + b''.join(Gripper_speed)
    Gripper_data_in[2] = Fuse_2_bytes(var)

    var =  b'\x00'+ b'\x00' + b''.join(Gripper_current)
    Gripper_data_in[3] = Fuse_2_bytes(var)

    # --- Start of Corrected Logic ---
    # This section now mirrors the working logic from GUI_PAROL_latest.py
    
    # 1. Store the raw status byte (from index 51)
    status_byte = int.from_bytes(Status,"big")
    Gripper_data_in[4] = status_byte

    # 2. Split the status byte into a list of 8 individual bits
    status_bits = Split_2_bitfield(status_byte)
    
    # 3. Combine the 3rd and 4th bits (at indices 2 and 3) to get the true object detection status
    # This creates a 2-bit number (0-3) which represents the full state.
    object_detection_status = (status_bits[2] << 1) | status_bits[3]
    Gripper_data_in[5] = object_detection_status
    # --- End of Corrected Logic ---

def Pack_data(Position_out,Speed_out,Command_out,Affected_joint_out,InOut_out,Timeout_out,Gripper_data_out):

    # Len is defined by all bytes EXCEPT start bytes and len
    # Start bytes = 3
    len = 52 #1
    Position = [Position_out[0],Position_out[1],Position_out[2],Position_out[3],Position_out[4],Position_out[5]]  #18
    Speed = [Speed_out[0], Speed_out[1], Speed_out[2], Speed_out[3], Speed_out[4], Speed_out[5],] #18
    Command = Command_out#1
    Affected_joint = Affected_joint_out
    InOut = InOut_out #1
    Timeout = Timeout_out #1
    Gripper_data = Gripper_data_out #9
    CRC_byte = 228 #1
    # End bytes = 2


    test_list = []
    #print(test_list)

    #x = bytes(start_bytes)
    test_list.append((start_bytes))
    
    test_list.append(bytes([len]))


    # Position data
    for i in range(6):
        position_split = Split_2_3_bytes(Position[i])
        test_list.append(position_split[1:4])

    # Speed data
    for i in range(6):
        speed_split = Split_2_3_bytes(Speed[i])
        test_list.append(speed_split[1:4])

    # Command data
    test_list.append(bytes([Command]))

    # Affected joint data
    Affected_list = Fuse_bitfield_2_bytearray(Affected_joint[:])
    test_list.append(Affected_list)

    # Inputs outputs data
    InOut_list = Fuse_bitfield_2_bytearray(InOut[:])
    test_list.append(InOut_list)

    # Timeout data
    test_list.append(bytes([Timeout]))

    # Gripper position
    Gripper_position = Split_2_3_bytes(Gripper_data[0])
    test_list.append(Gripper_position[2:4])

    # Gripper speed
    Gripper_speed = Split_2_3_bytes(Gripper_data[1])
    test_list.append(Gripper_speed[2:4])

    # Gripper current
    Gripper_current = Split_2_3_bytes(Gripper_data[2])
    test_list.append(Gripper_current[2:4])  

    # Gripper command
    test_list.append(bytes([Gripper_data[3]]))
    # Gripper mode
    test_list.append(bytes([Gripper_data[4]]))
    
    # ==========================================================
    # === FIX: Make sure calibrate is a one-shot command      ====
    # ==========================================================
    # If the mode was set to calibrate (1) or clear_error (2), reset it
    # back to normal (0) for the next cycle. This prevents an endless loop.
    if Gripper_data_out[4] == 1 or Gripper_data_out[4] == 2:
        Gripper_data_out[4] = 0
    # ==========================================================
    
    # Gripper ID
    test_list.append(bytes([Gripper_data[5]]))
 
    # CRC byte
    test_list.append(bytes([CRC_byte]))

    # END bytes
    test_list.append((end_bytes))
    
    #print(test_list)
    return test_list

def Get_data(Position_in,Speed_in,Homed_in,InOut_in,Temperature_error_in,Position_error_in,Timeout_error,Timing_data_in,
         XTR_data,Gripper_data_in):
    global input_byte 

    global start_cond1_byte 
    global start_cond2_byte 
    global start_cond3_byte 

    global end_cond1_byte 
    global end_cond2_byte 

    global start_cond1 
    global start_cond2 
    global start_cond3 

    global good_start 
    global data_len 

    global data_buffer 
    global data_counter

    while (ser.inWaiting() > 0):
        input_byte = ser.read()

        #UNCOMMENT THIS TO GET ALL DATA FROM THE ROBOT PRINTED
        #print(input_byte) 

        # When data len is received start is good and after that put all data in receive buffer
        # Data len is ALL data after it; that includes input buffer, end bytes and CRC
        if (good_start != 1):
            # All start bytes are good and next byte is data len
            if (start_cond1 == 1 and start_cond2 == 1 and start_cond3 == 1):
                good_start = 1
                data_len = input_byte
                data_len = struct.unpack('B', data_len)[0]
                logger.debug("data len we got from robot packet= ")
                logger.debug(input_byte)
                logger.debug("good start for DATA that we received at PC")
            # Third start byte is good
            if (input_byte == start_cond3_byte and start_cond2 == 1 and start_cond1 == 1):
                start_cond3 = 1
                #print("good cond 3 PC")
            #Third start byte is bad, reset all flags
            elif (start_cond2 == 1 and start_cond1 == 1):
                #print("bad cond 3 PC")
                start_cond1 = 0
                start_cond2 = 0
            # Second start byte is good
            if (input_byte == start_cond2_byte and start_cond1 == 1):
                start_cond2 = 1
                #print("good cond 2 PC ")
            #Second start byte is bad, reset all flags   
            elif (start_cond1 == 1):
                #print("Bad cond 2 PC")
                start_cond1 = 0
            # First start byte is good
            if (input_byte == start_cond1_byte):
                start_cond1 = 1
                #print("good cond 1 PC")
        else:
            # Here data goes after good  start
            data_buffer[data_counter] = input_byte
            if (data_counter == data_len - 1):

                logger.debug("Data len PC")
                logger.debug(data_len)
                logger.debug("End bytes are:")
                logger.debug(data_buffer[data_len -1])
                logger.debug(data_buffer[data_len -2])

                # Here if last 2 bytes are end condition bytes we process the data 
                if (data_buffer[data_len -1] == end_cond2_byte and data_buffer[data_len - 2] == end_cond1_byte):

                    logger.debug("GOOD END CONDITION PC")
                    logger.debug("I UNPACKED RAW DATA RECEIVED FROM THE ROBOT")
                    Unpack_data(data_buffer, Position_in,Speed_in,Homed_in,InOut_in,Temperature_error_in,Position_error_in,Timeout_error,Timing_data_in,
                    XTR_data,Gripper_data_in)
                    logger.debug("DATA UNPACK FINISHED")
                    # ako su dobri izračunaj crc
                    # if crc dobar raspakiraj podatke
                    # ako je dobar paket je dobar i spremi ga u nove variable!
                
                # Print every byte
                #print("podaci u data bufferu su:")
                #for i in range(data_len):
                    #print(data_buffer[i])

                good_start = 0
                start_cond1 = 0
                start_cond3 = 0
                start_cond2 = 0
                data_len = 0
                data_counter = 0
            else:
                data_counter = data_counter + 1

# Split data to 3 bytes 
def Split_2_3_bytes(var_in):
    y = int_to_3_bytes(var_in & 0xFFFFFF) # converts my int value to bytes array
    return y

# Splits byte to bitfield list
def Split_2_bitfield(var_in):
    #return [var_in >> i & 1 for i in range(7,-1,-1)] 
    return [(var_in >> i) & 1 for i in range(7, -1, -1)]

# Fuses 3 bytes to 1 signed int
def Fuse_3_bytes(var_in):
    value = struct.unpack(">I", bytearray(var_in))[0] # converts bytes array to int

    # convert to negative number if it is negative
    if value >= 1<<23:
        value -= 1<<24

    return value

# Fuses 2 bytes to 1 signed int
def Fuse_2_bytes(var_in):
    value = struct.unpack(">I", bytearray(var_in))[0] # converts bytes array to int

    # convert to negative number if it is negative
    if value >= 1<<15:
        value -= 1<<16

    return value

# Fuse bitfield list to byte
def Fuse_bitfield_2_bytearray(var_in):
    number = 0
    for b in var_in:
        number = (2 * number) + b
    return bytes([number])

# Check if there is element 1 in the list. 
# If yes return its index, if no element is 1 return -1
def check_elements(lst):
    for i, element in enumerate(lst):
        if element == 1:
            return i
    return -1  # Return -1 if no element is 1

#########################################################################
#########################################################################

def calculate_duration_from_speed(trajectory_length: float, speed_percentage: float) -> float:
    """
    Calculate duration based on trajectory length and speed percentage.
    
    Args:
        trajectory_length: Total path length in mm
        speed_percentage: Speed as percentage (1-100)
        
    Returns:
        Duration in seconds
    """
    # Map speed percentage to mm/s (adjustable based on robot capabilities)
    # For example: 100% = 100mm/s, 50% = 50mm/s
    speed_mm_s = np.interp(speed_percentage, [0, 100], 
                          [PAROL6_ROBOT.Cartesian_linear_velocity_min * 1000,
                           PAROL6_ROBOT.Cartesian_linear_velocity_max * 1000])
    
    if speed_mm_s > 0:
        return trajectory_length / speed_mm_s
    else:
        return 5.0  # Default fallback
    
def parse_smooth_motion_commands(parts):
    """
    Parse smooth motion commands received via UDP and create appropriate command objects.
    All commands support:
    - Reference frame selection (WRF or TRF)
    - Optional start position (CURRENT or specified pose)
    - Both DURATION and SPEED timing modes
    
    Args:
        parts: List of command parts split by '|'
        
    Returns:
        Command object or None if parsing fails
    """
    command_type = parts[0]
    
    # Helper function for parsing optional start pose
    def parse_start_pose(start_str):
        """Parse start pose - returns None for CURRENT, or list of floats for specified pose."""
        if start_str == 'CURRENT' or start_str == 'NONE':
            return None
        else:
            try:
                return list(map(float, start_str.split(',')))
            except:
                logger.warning(f" Invalid start pose format: {start_str}")
                return None
    
    # Helper function for calculating duration from speed
    def calculate_duration_from_speed(trajectory_length: float, speed_percentage: float) -> float:
        """Calculate duration based on trajectory length and speed percentage."""
        # Map speed percentage to mm/s
        min_speed = PAROL6_ROBOT.Cartesian_linear_velocity_min * 1000  # Convert to mm/s
        max_speed = PAROL6_ROBOT.Cartesian_linear_velocity_max * 1000  # Convert to mm/s
        speed_mm_s = np.interp(speed_percentage, [0, 100], [min_speed, max_speed])
        
        if speed_mm_s > 0:
            return trajectory_length / speed_mm_s
        else:
            return 5.0  # Default fallback
    
    try:
        if command_type == 'SMOOTH_CIRCLE':
            # Format: SMOOTH_CIRCLE|center_x,center_y,center_z|radius|plane|frame|start_pose|timing_type|timing_value|clockwise
            center = list(map(float, parts[1].split(',')))
            radius = float(parts[2])
            plane = parts[3]
            frame = parts[4]  # 'WRF' or 'TRF'
            start_pose = parse_start_pose(parts[5])
            timing_type = parts[6]  # 'DURATION' or 'SPEED'
            timing_value = float(parts[7])
            clockwise = parts[8] == '1'
            
            # Calculate duration
            if timing_type == 'DURATION':
                duration = timing_value
            else:  # SPEED
                # Circle circumference
                path_length = 2 * np.pi * radius
                duration = calculate_duration_from_speed(path_length, timing_value)
            
            logger.debug(f"  -> Parsed circle: r={radius}mm, plane={plane}, frame={frame}, {timing_type}={timing_value}, duration={duration:.2f}s")
            
            # Return command object with frame parameter
            return SmoothCircleCommand(center, radius, plane, duration, clockwise, frame, start_pose)
            
        elif command_type == 'SMOOTH_ARC_CENTER':
            # Format: SMOOTH_ARC_CENTER|end_pose|center|frame|start_pose|timing_type|timing_value|clockwise
            end_pose = list(map(float, parts[1].split(',')))
            center = list(map(float, parts[2].split(',')))
            frame = parts[3]  # 'WRF' or 'TRF'
            start_pose = parse_start_pose(parts[4])
            timing_type = parts[5]  # 'DURATION' or 'SPEED'
            timing_value = float(parts[6])
            clockwise = parts[7] == '1'
            
            # Calculate duration
            if timing_type == 'DURATION':
                duration = timing_value
            else:  # SPEED
                # Estimate arc length (will be more accurate when we have actual positions)
                # Use a conservative estimate based on radius
                radius_estimate = np.linalg.norm(np.array(center) - np.array(end_pose[:3]))
                estimated_arc_angle = np.pi / 2  # 90 degrees estimate
                arc_length = radius_estimate * estimated_arc_angle
                duration = calculate_duration_from_speed(arc_length, timing_value)
            
            logger.debug(f"  -> Parsed arc (center): frame={frame}, {timing_type}={timing_value}, duration={duration:.2f}s")
            
            # Return command with frame
            return SmoothArcCenterCommand(end_pose, center, duration, clockwise, frame, start_pose)
            
        elif command_type == 'SMOOTH_ARC_PARAM':
            # Format: SMOOTH_ARC_PARAM|end_pose|radius|angle|frame|start_pose|timing_type|timing_value|clockwise
            end_pose = list(map(float, parts[1].split(',')))
            radius = float(parts[2])
            arc_angle = float(parts[3])
            frame = parts[4]  # 'WRF' or 'TRF'
            start_pose = parse_start_pose(parts[5])
            timing_type = parts[6]  # 'DURATION' or 'SPEED'
            timing_value = float(parts[7])
            clockwise = parts[8] == '1'
            
            # Calculate duration
            if timing_type == 'DURATION':
                duration = timing_value
            else:  # SPEED
                # Arc length = radius * angle (in radians)
                arc_length = radius * np.deg2rad(arc_angle)
                duration = calculate_duration_from_speed(arc_length, timing_value)
            
            logger.debug(f"  -> Parsed arc (param): r={radius}mm, θ={arc_angle}°, frame={frame}, duration={duration:.2f}s")
            
            # Return command object with frame
            return SmoothArcParamCommand(end_pose, radius, arc_angle, duration, clockwise, frame, start_pose)
            
        elif command_type == 'SMOOTH_SPLINE':
            # Format: SMOOTH_SPLINE|num_waypoints|frame|start_pose|timing_type|timing_value|waypoint1|waypoint2|...
            num_waypoints = int(parts[1])
            frame = parts[2]  # 'WRF' or 'TRF'
            start_pose = parse_start_pose(parts[3])
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
                
                duration = calculate_duration_from_speed(total_dist, timing_value)
            
            logger.debug(f"  -> Parsed spline: {num_waypoints} points, frame={frame}, duration={duration:.2f}s")
            
            # Return command object with frame
            return SmoothSplineCommand(waypoints, duration, frame, start_pose)
            
        elif command_type == 'SMOOTH_HELIX':
            # Format: SMOOTH_HELIX|center|radius|pitch|height|frame|start_pose|timing_type|timing_value|clockwise
            center = list(map(float, parts[1].split(',')))
            radius = float(parts[2])
            pitch = float(parts[3])
            height = float(parts[4])
            frame = parts[5]  # 'WRF' or 'TRF'
            start_pose = parse_start_pose(parts[6])
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
                duration = calculate_duration_from_speed(helix_length, timing_value)
            
            logger.debug(f"  -> Parsed helix: h={height}mm, pitch={pitch}mm, frame={frame}, duration={duration:.2f}s")
            
            # Return command object with frame
            return SmoothHelixCommand(center, radius, pitch, height, duration, clockwise, frame, start_pose)
            
        elif command_type == 'SMOOTH_BLEND':
            # Format: SMOOTH_BLEND|num_segments|blend_time|frame|start_pose|timing_type|timing_value|segment1||segment2||...
            num_segments = int(parts[1])
            blend_time = float(parts[2])
            frame = parts[3]  # 'WRF' or 'TRF'
            start_pose = parse_start_pose(parts[4])
            timing_type = parts[5]  # 'DEFAULT', 'DURATION', or 'SPEED'
            
            # Parse overall timing
            if timing_type == 'DEFAULT':
                # Use individual segment durations as-is
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
                    
                    # Estimate length (will be refined when we have actual start)
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
                        'original_duration': segment_duration,
                        'clockwise': clockwise
                    })
                    
                elif seg_type == 'ARC':
                    # Format: ARC|end_x,end_y,end_z,end_rx,end_ry,end_rz|center_x,center_y,center_z|duration|clockwise
                    end = list(map(float, seg_parts[1].split(',')))
                    center = list(map(float, seg_parts[2].split(',')))
                    segment_duration = float(seg_parts[3])
                    total_original_duration += segment_duration
                    clockwise = seg_parts[4] == '1'
                    
                    # Estimate arc length
                    estimated_radius = 50  # mm
                    estimated_arc_angle = np.pi / 2  # 90 degrees
                    estimated_length = estimated_radius * estimated_arc_angle
                    total_estimated_length += estimated_length
                    
                    segment_definitions.append({
                        'type': 'ARC',
                        'end': end,
                        'center': center,
                        'duration': segment_duration,
                        'original_duration': segment_duration,
                        'clockwise': clockwise
                    })
                    
                elif seg_type == 'SPLINE':
                    # Format: SPLINE|num_points|waypoint1;waypoint2;...|duration
                    num_points = int(seg_parts[1])
                    waypoints = []
                    wp_strs = seg_parts[2].split(';')
                    for wp_str in wp_strs:
                        waypoints.append(list(map(float, wp_str.split(','))))
                    segment_duration = float(seg_parts[3])
                    total_original_duration += segment_duration
                    
                    # Estimate spline length
                    estimated_length = 0
                    for i in range(1, len(waypoints)):
                        estimated_length += np.linalg.norm(
                            np.array(waypoints[i][:3]) - np.array(waypoints[i-1][:3])
                        )
                    total_estimated_length += estimated_length
                    
                    segment_definitions.append({
                        'type': 'SPLINE',
                        'waypoints': waypoints,
                        'duration': segment_duration,
                        'original_duration': segment_duration
                    })
            
            # Adjust segment durations if overall timing is specified
            if overall_duration is not None:
                # Scale all segment durations proportionally
                if total_original_duration > 0:
                    scale_factor = overall_duration / total_original_duration
                    for seg in segment_definitions:
                        seg['duration'] = seg['original_duration'] * scale_factor
                logger.debug(f"  -> Scaled blend segments to total duration: {overall_duration:.2f}s")
                        
            elif overall_speed is not None:
                # Calculate duration from speed and estimated path length
                overall_duration = calculate_duration_from_speed(total_estimated_length, overall_speed)
                if total_original_duration > 0:
                    scale_factor = overall_duration / total_original_duration
                    for seg in segment_definitions:
                        seg['duration'] = seg['original_duration'] * scale_factor
                logger.debug(f"  -> Calculated blend duration from speed: {overall_duration:.2f}s")
            else:
                logger.debug(f"  -> Using original segment durations (total: {total_original_duration:.2f}s)")
            
            logger.debug(f"  -> Parsed blend: {num_segments} segments, frame={frame}, blend_time={blend_time}s")
            
            # Return command with frame
            return SmoothBlendCommand(segment_definitions, blend_time, frame, start_pose)
            
    except Exception as e:
        logger.error(f"Error parsing smooth motion command: {e}")
        logger.debug(f"Command parts: {parts}")
        import traceback
        traceback.print_exc()
        return None
    
    logger.warning(f" Unknown smooth motion command type: {command_type}")
    return None

def transform_command_params_to_wrf(command_type: str, params: dict, frame: str, current_position_in) -> dict:
    """
    Transform command parameters from TRF to WRF.
    Handles position, orientation, and directional vectors correctly.
    """
    if frame == 'WRF':
        return params
    
    # Get current tool pose
    current_q = np.array([PAROL6_ROBOT.STEPS2RADS(p, i) 
                         for i, p in enumerate(current_position_in)])
    tool_pose = PAROL6_ROBOT.robot.fkine(current_q)
    
    transformed = params.copy()
    
    # SMOOTH_CIRCLE - Transform center and plane normal
    if command_type == 'SMOOTH_CIRCLE':
        if 'center' in params:
            center_trf = SE3(params['center'][0]/1000, 
                           params['center'][1]/1000, 
                           params['center'][2]/1000)
            center_wrf = tool_pose * center_trf
            transformed['center'] = (center_wrf.t * 1000).tolist()
        
        if 'plane' in params:
            plane_normals_trf = {
                'XY': [0, 0, 1],   # Tool's Z-axis
                'XZ': [0, 1, 0],   # Tool's Y-axis  
                'YZ': [1, 0, 0]    # Tool's X-axis
            }
            normal_trf = np.array(plane_normals_trf[params['plane']])
            normal_wrf = tool_pose.R @ normal_trf
            transformed['normal_vector'] = normal_wrf.tolist()
            logger.debug(f"  -> TRF circle plane {params['plane']} transformed to WRF")
    
    # SMOOTH_ARC_CENTER - Transform center, end_pose, and implied plane
    elif command_type == 'SMOOTH_ARC_CENTER':
        if 'center' in params:
            center_trf = SE3(params['center'][0]/1000, 
                           params['center'][1]/1000, 
                           params['center'][2]/1000)
            center_wrf = tool_pose * center_trf
            transformed['center'] = (center_wrf.t * 1000).tolist()
        
        if 'end_pose' in params:
            end_trf = SE3(params['end_pose'][0]/1000, 
                         params['end_pose'][1]/1000, 
                         params['end_pose'][2]/1000) * \
                      SE3.RPY(params['end_pose'][3:], unit='deg', order='xyz')
            end_wrf = tool_pose * end_trf
            transformed['end_pose'] = np.concatenate([
                end_wrf.t * 1000,
                end_wrf.rpy(unit='deg', order='xyz')
            ]).tolist()
        
        # Arc plane is determined by start, end, and center points
        # But we should transform any specified plane normal
        if 'plane' in params:
            # Similar to circle plane transformation
            plane_normals_trf = {
                'XY': [0, 0, 1],
                'XZ': [0, 1, 0],
                'YZ': [1, 0, 0]
            }
            normal_trf = np.array(plane_normals_trf[params['plane']])
            normal_wrf = tool_pose.R @ normal_trf
            transformed['normal_vector'] = normal_wrf.tolist()
    
    # SMOOTH_ARC_PARAM - Transform end_pose and arc plane
    elif command_type == 'SMOOTH_ARC_PARAM':
        if 'end_pose' in params:
            end_trf = SE3(params['end_pose'][0]/1000, 
                         params['end_pose'][1]/1000, 
                         params['end_pose'][2]/1000) * \
                      SE3.RPY(params['end_pose'][3:], unit='deg', order='xyz')
            end_wrf = tool_pose * end_trf
            transformed['end_pose'] = np.concatenate([
                end_wrf.t * 1000,
                end_wrf.rpy(unit='deg', order='xyz')
            ]).tolist()
        
        # For parametric arc, the plane is usually XY of the tool
        # Transform the assumed plane normal
        if 'plane' not in params:
            params['plane'] = 'XY'  # Default to XY plane
        
        plane_normals_trf = {
            'XY': [0, 0, 1],
            'XZ': [0, 1, 0],
            'YZ': [1, 0, 0]
        }
        normal_trf = np.array(plane_normals_trf[params.get('plane', 'XY')])
        normal_wrf = tool_pose.R @ normal_trf
        transformed['normal_vector'] = normal_wrf.tolist()
    
    # SMOOTH_HELIX - Transform center and helix axis
    elif command_type == 'SMOOTH_HELIX':
        if 'center' in params:
            center_trf = SE3(params['center'][0]/1000, 
                           params['center'][1]/1000, 
                           params['center'][2]/1000)
            center_wrf = tool_pose * center_trf
            transformed['center'] = (center_wrf.t * 1000).tolist()
        
        # Helix axis - default is Z-axis (vertical in TRF)
        # In TRF, helix rises along tool's Z-axis
        helix_axis_trf = np.array([0, 0, 1])  # Tool's Z-axis
        helix_axis_wrf = tool_pose.R @ helix_axis_trf
        transformed['helix_axis'] = helix_axis_wrf.tolist()
        
        # Also need to transform the "up" direction for proper orientation
        up_vector_trf = np.array([0, 1, 0])  # Tool's Y-axis
        up_vector_wrf = tool_pose.R @ up_vector_trf
        transformed['up_vector'] = up_vector_wrf.tolist()
    
    # SMOOTH_SPLINE - Transform all waypoints
    elif command_type == 'SMOOTH_SPLINE':
        if 'waypoints' in params:
            transformed_waypoints = []
            for wp in params['waypoints']:
                wp_trf = SE3(wp[0]/1000, wp[1]/1000, wp[2]/1000) * \
                         SE3.RPY(wp[3:], unit='deg', order='xyz')
                wp_wrf = tool_pose * wp_trf
                wp_transformed = np.concatenate([
                    wp_wrf.t * 1000,
                    wp_wrf.rpy(unit='deg', order='xyz')
                ]).tolist()
                transformed_waypoints.append(wp_transformed)
            transformed['waypoints'] = transformed_waypoints
    
    # SMOOTH_BLEND - Transform all segments recursively
    elif command_type == 'SMOOTH_BLEND':
        if 'segments' in params:
            transformed_segments = []
            for seg in params['segments']:
                seg_copy = seg.copy()
                seg_type = seg['type']
                
                if seg_type == 'LINE':
                    if 'end' in seg:
                        end_trf = SE3(seg['end'][0]/1000, 
                                    seg['end'][1]/1000, 
                                    seg['end'][2]/1000) * \
                                  SE3.RPY(seg['end'][3:], unit='deg', order='xyz')
                        end_wrf = tool_pose * end_trf
                        seg_copy['end'] = np.concatenate([
                            end_wrf.t * 1000,
                            end_wrf.rpy(unit='deg', order='xyz')
                        ]).tolist()
                
                elif seg_type == 'CIRCLE':
                    if 'center' in seg:
                        center_trf = SE3(seg['center'][0]/1000, 
                                       seg['center'][1]/1000, 
                                       seg['center'][2]/1000)
                        center_wrf = tool_pose * center_trf
                        seg_copy['center'] = (center_wrf.t * 1000).tolist()
                    
                    if 'plane' in seg:
                        plane_normals_trf = {
                            'XY': [0, 0, 1],
                            'XZ': [0, 1, 0],
                            'YZ': [1, 0, 0]
                        }
                        normal_trf = np.array(plane_normals_trf[seg['plane']])
                        normal_wrf = tool_pose.R @ normal_trf
                        seg_copy['normal_vector'] = normal_wrf.tolist()
                
                elif seg_type == 'ARC':
                    if 'center' in seg:
                        center_trf = SE3(seg['center'][0]/1000, 
                                       seg['center'][1]/1000, 
                                       seg['center'][2]/1000)
                        center_wrf = tool_pose * center_trf
                        seg_copy['center'] = (center_wrf.t * 1000).tolist()
                    
                    if 'end' in seg:
                        end_trf = SE3(seg['end'][0]/1000, 
                                    seg['end'][1]/1000, 
                                    seg['end'][2]/1000) * \
                                  SE3.RPY(seg['end'][3:], unit='deg', order='xyz')
                        end_wrf = tool_pose * end_trf
                        seg_copy['end'] = np.concatenate([
                            end_wrf.t * 1000,
                            end_wrf.rpy(unit='deg', order='xyz')
                        ]).tolist()
                
                elif seg_type == 'SPLINE':
                    if 'waypoints' in seg:
                        transformed_waypoints = []
                        for wp in seg['waypoints']:
                            wp_trf = SE3(wp[0]/1000, wp[1]/1000, wp[2]/1000) * \
                                     SE3.RPY(wp[3:], unit='deg', order='xyz')
                            wp_wrf = tool_pose * wp_trf
                            wp_transformed = np.concatenate([
                                wp_wrf.t * 1000,
                                wp_wrf.rpy(unit='deg', order='xyz')
                            ]).tolist()
                            transformed_waypoints.append(wp_transformed)
                        seg_copy['waypoints'] = transformed_waypoints
                
                transformed_segments.append(seg_copy)
            transformed['segments'] = transformed_segments
    
    # Transform start_pose if specified (common to all commands)
    if 'start_pose' in params and params['start_pose'] is not None:
        start_trf = SE3(params['start_pose'][0]/1000, 
                       params['start_pose'][1]/1000, 
                       params['start_pose'][2]/1000) * \
                    SE3.RPY(params['start_pose'][3:], unit='deg', order='xyz')
        start_wrf = tool_pose * start_trf
        transformed['start_pose'] = np.concatenate([
            start_wrf.t * 1000,
            start_wrf.rpy(unit='deg', order='xyz')
        ]).tolist()
    
    return transformed

#########################################################################
# Smooth Motion Commands and Robot Commands End Here
#########################################################################

# ============================================================================
# TIER 2: Legacy acknowledgment functions removed - now handled by NetworkHandler
# NetworkHandler provides: network_handler.send_ack(cmd_id, status, details, addr)
# ============================================================================

# ============================================================================
# TIER 2: Initialize CommandQueue (replaces bare deque)
# ============================================================================
# Create command queue with size limits and overflow protection
command_queue = CommandQueue(
    logger=logger,
    max_size=100,  # Maximum total commands
    max_trajectory_commands=10  # Maximum memory-intensive trajectory commands
)
logger.info(f'CommandQueue initialized (max_size={command_queue.max_size})')

# ============================================================================
# TIER 2: Initialize PerformanceMonitor
# ============================================================================
# Monitor control loop timing to ensure 100Hz operation
performance_monitor = PerformanceMonitor(
    logger=logger,
    target_hz=100,  # 100Hz control loop
    window_size=1000,  # Keep last 1000 cycles (10 seconds)
    debug_mode=(log_level.upper() == 'DEBUG'),  # Enable detailed phase tracking in DEBUG mode
    collect_samples=False  # Enable sample collection when recording is active
)
logger.info(f'PerformanceMonitor initialized (target={performance_monitor.target_hz}Hz, debug_mode={performance_monitor._debug_mode})')

# Set performance monitor for IK solver timing
from lib.kinematics import ik_solver
ik_solver.set_performance_monitor(performance_monitor)

# Map command objects to their IDs and addresses for acknowledgment tracking
command_id_map = {}

# Currently active command ID and address
active_command_id = None
active_command_addr = None

# --------------------------------------------------------------------------
# --- Test 1: Homing and Initial Setup
# --------------------------------------------------------------------------

# 1. Optionally start with Home command (if configured)
if config.get('robot', {}).get('auto_home_on_startup', True):
    logger.info("Auto-home on startup enabled, adding home command to queue")
    command_queue.add(HomeCommand())
else:
    logger.info("Auto-home on startup disabled")

# --- State variable for the currently running command ---
active_command = None
e_stop_active = False

# Use deque for an efficient FIFO queue
incoming_command_buffer = deque()
# Timestamp of the last processed network command
last_command_time = 0
# Cooldown period in seconds to prevent command flooding
COMMAND_COOLDOWN_S = 0.1 # 100ms

# Set interval
timer = Timer(interval=INTERVAL_S, warnings=False, precise=True)

# ============================================================================
# MODIFIED MAIN LOOP WITH ACKNOWLEDGMENTS
# ============================================================================

timer = Timer(interval=INTERVAL_S, warnings=False, precise=True)
prev_time = 0

# Command performance tracking
active_command_start_time = None
active_command_perf_samples = []

# Performance recording state
recording_enabled = False  # When true, each command completion auto-saves to JSON

while timer.elapsed_time < 1100000:
    # ========================================================================
    # TIER 2: Performance monitoring - start cycle timing
    # ========================================================================
    performance_monitor.start_cycle()

    # --- Connection Handling ---
    if ser is None or not ser.is_open:
        logger.warning("Serial port not open. Attempting to reconnect...")
        try:
            ser = serial.Serial(port=com_port_str, baudrate=3000000, timeout=0)
            if ser.is_open:
                logger.info(f"Successfully reconnected to {com_port_str}")
        except serial.SerialException as e:
            ser = None
            time.sleep(1)
            continue  # Skip this cycle if reconnection failed

    # =======================================================================
    # === NETWORK COMMAND RECEPTION WITH ID PARSING ===
    # =======================================================================
    # TIER 2: Network command reception via NetworkHandler
    # =======================================================================
    performance_monitor.start_phase('network')
    try:
        # Receive commands from UDP (returns list of tuples: (raw_msg, cmd_id, parsed_msg, addr))
        received_commands = network_handler.receive_commands()

        for raw_message, cmd_id, message, addr in received_commands:
            parts = message.split('|')
            command_name = parts[0].upper()

            # Handle immediate response commands
            if command_name == 'STOP':
                logger.warning("Received STOP command. Halting all motion and clearing queue.")

                # Cancel active command
                if active_command and active_command_id:
                    network_handler.send_ack(active_command_id, "CANCELLED",
                                      "Stopped by user", addr)
                active_command = None
                active_command_id = None

                # Clear queue with cancel callback to notify about cancelled commands
                def cancel_callback(cmd):
                    if cmd in command_id_map:
                        cmd_id, cmd_addr = command_id_map[cmd]
                        network_handler.send_ack(cmd_id, "CANCELLED", "Queue cleared by STOP", cmd_addr)

                command_queue.clear(cancel_callback=cancel_callback)
                command_id_map.clear()

                # Stop robot
                Command_out.value = 255
                Speed_out[:] = [0] * 6

                # Send acknowledgment for STOP command itself
                if cmd_id:
                    network_handler.send_ack(cmd_id, "COMPLETED", "Emergency stop executed", addr)

            elif command_name == 'CLEAR_ESTOP':
                logger.info("Clearing E-stop flag...")
                Command_out.value = 101  # Re-enable signal
                e_stop_active = False

                # Send acknowledgment for CLEAR_ESTOP command
                if cmd_id:
                    network_handler.send_ack(cmd_id, "COMPLETED", "E-stop cleared", addr)

            elif command_name == 'GET_POSE':
                q_current = np.array([PAROL6_ROBOT.STEPS2RADS(p, i) for i, p in enumerate(Position_in)])
                current_pose_matrix = PAROL6_ROBOT.robot.fkine(q_current).A
                pose_flat = current_pose_matrix.flatten()
                pose_str = ",".join(map(str, pose_flat))
                response_message = f"POSE|{pose_str}"
                network_handler.command_socket.sendto(response_message.encode('utf-8'), addr)

                if cmd_id:
                    network_handler.send_ack(cmd_id, "COMPLETED", "Pose data sent", addr)

            elif command_name == 'GET_ANGLES':
                angles_rad = [PAROL6_ROBOT.STEPS2RADS(p, i) for i, p in enumerate(Position_in)]
                angles_deg = np.rad2deg(angles_rad)
                angles_str = ",".join(map(str, angles_deg))
                response_message = f"ANGLES|{angles_str}"
                network_handler.command_socket.sendto(response_message.encode('utf-8'), addr)

                if cmd_id:
                    network_handler.send_ack(cmd_id, "COMPLETED", "Angles data sent", addr)

            elif command_name == 'GET_IO':
                io_status_str = ",".join(map(str, InOut_in[:5]))
                response_message = f"IO|{io_status_str}"
                network_handler.command_socket.sendto(response_message.encode('utf-8'), addr)

                if cmd_id:
                    network_handler.send_ack(cmd_id, "COMPLETED", "IO data sent", addr)

            elif command_name == 'GET_GRIPPER':
                gripper_status_str = ",".join(map(str, Gripper_data_in))
                response_message = f"GRIPPER|{gripper_status_str}"
                network_handler.command_socket.sendto(response_message.encode('utf-8'), addr)

                if cmd_id:
                    network_handler.send_ack(cmd_id, "COMPLETED", "Gripper data sent", addr)

            elif command_name == 'GET_SPEEDS':
                speeds_str = ",".join(map(str, Speed_in))
                response_message = f"SPEEDS|{speeds_str}"
                network_handler.command_socket.sendto(response_message.encode('utf-8'), addr)

                if cmd_id:
                    network_handler.send_ack(cmd_id, "COMPLETED", "Speed data sent", addr)

            elif command_name == 'GET_ESTOP_STATUS':
                # Return software E-stop flag status (not physical button)
                estop_status = "1" if e_stop_active else "0"
                response_message = f"ESTOP_STATUS|{estop_status}"
                network_handler.command_socket.sendto(response_message.encode('utf-8'), addr)

                if cmd_id:
                    network_handler.send_ack(cmd_id, "COMPLETED", "E-stop status sent", addr)

            elif command_name == 'GET_HOMED':
                # Return homing status for all 6 joints (0=not homed, 1=homed)
                homed_str = ",".join(map(str, Homed_in[:6]))
                response_message = f"HOMED|{homed_str}"
                network_handler.command_socket.sendto(response_message.encode('utf-8'), addr)

                if cmd_id:
                    network_handler.send_ack(cmd_id, "COMPLETED", "Homed status sent", addr)

            elif command_name == 'GET_HZ':
                # Return current control loop frequency
                current_hz = performance_monitor.get_hz()
                response_message = f"HZ|{current_hz:.1f}"
                network_handler.command_socket.sendto(response_message.encode('utf-8'), addr)

                if cmd_id:
                    network_handler.send_ack(cmd_id, "COMPLETED", "Hz data sent", addr)

            else:
                # Queue command for processing (store parsed data to avoid re-parsing)
                incoming_command_buffer.append((cmd_id, message, addr))

    except Exception as e:
        logger.error(f"Network receive error: {e}")
    finally:
        performance_monitor.end_phase('network')

    # =======================================================================
    # === PROCESS COMMANDS FROM BUFFER WITH ACKNOWLEDGMENTS ===
    # =======================================================================
    performance_monitor.start_phase('processing')
    current_time = time.time()
    if incoming_command_buffer and (current_time - last_command_time) > COMMAND_COOLDOWN_S and not e_stop_active:
        cmd_id, message, addr = incoming_command_buffer.popleft()
        last_command_time = current_time

        logger.debug(f"Processing command{' (ID: ' + cmd_id + ')' if cmd_id else ''}: {message[:50]}...")

        parts = message.split('|')
        command_name = parts[0].upper()

        # Variable to track if command was successfully queued
        command_queued = False
        command_obj = None
        error_details = ""

        # ===================================================================
        # Handle recording toggle command
        # ===================================================================
        if command_name == 'SET_RECORDING':
            # Enable or disable automatic recording (1 = on, 0 = off)
            enabled = parts[1] if len(parts) > 1 else '0'
            recording_enabled = (enabled == '1' or enabled.upper() == 'TRUE')

            # Update performance monitor to collect samples when recording
            if recording_enabled:
                performance_monitor.enable_sample_collection()
            else:
                performance_monitor.disable_sample_collection()

            if cmd_id:
                status = "enabled" if recording_enabled else "disabled"
                network_handler.send_ack(cmd_id, "COMPLETED", f"Auto-recording {status}", addr)
            logger.info(f"[Recording] Auto-recording {status}")
            continue

        # ===================================================================
        # TIER 2: Parse command using CommandParser
        # ===================================================================
        command_obj, error_details = command_parser.parse(message, command_classes)

        if command_obj is not None:
            command_queued = True
        else:
            command_queued = False
            if not error_details:
                error_details = f"Unknown or malformed command: {command_name}"
        
        # Handle command queueing and acknowledgments
        if command_queued and command_obj:
            # Check if command is initially valid
            if hasattr(command_obj, 'is_valid') and not command_obj.is_valid:
                if cmd_id:
                    network_handler.send_ack(cmd_id, "INVALID",
                                       "Command failed validation", addr)
            else:
                # Check if queue can accept command
                can_add, reason = command_queue.can_add(command_obj)
                if not can_add:
                    # Queue full or trajectory limit reached
                    if cmd_id:
                        network_handler.send_ack(cmd_id, "REJECTED", reason, addr)
                    logger.warning(f"Command rejected: {reason}")
                else:
                    # Add to queue
                    command_queue.add(command_obj)
                    logger.info(f"[DEBUG] Command added to queue. Queue size: {command_queue.size}, Command type: {type(command_obj).__name__}")
                    if cmd_id:
                        command_id_map[command_obj] = (cmd_id, addr)
                        network_handler.send_ack(cmd_id, "QUEUED",
                                           f"Position {command_queue.size} in queue", addr)
        else:
            # Command was not queued
            if cmd_id:
                network_handler.send_ack(cmd_id, "INVALID", error_details, addr)
            logger.warning(f" {error_details}")
    performance_monitor.end_phase('processing')

    # =======================================================================
    # === MAIN EXECUTION LOGIC WITH ACKNOWLEDGMENTS ===
    # =======================================================================
    try:
        # --- E-Stop Handling ---
        if InOut_in[4] == 0:  # E-Stop pressed
            if not e_stop_active:
                cancelled_command_info = "None"
                if active_command is not None:
                    cancelled_command_info = type(active_command).__name__
                    if active_command_id:
                        network_handler.send_ack(active_command_id, "CANCELLED", 
                                          "E-Stop activated")
                
                # Cancel all queued commands with callback
                def estop_cancel_callback(cmd):
                    if cmd in command_id_map:
                        cmd_id, addr = command_id_map[cmd]
                        network_handler.send_ack(cmd_id, "CANCELLED", "E-Stop activated", addr)

                command_queue.clear(cancel_callback=estop_cancel_callback)

                # Cancel all buffered but unprocessed commands
                for buffered_cmd_id, buffered_message, buffered_addr in incoming_command_buffer:
                    if buffered_cmd_id:
                        network_handler.send_ack(buffered_cmd_id, "CANCELLED", "E-Stop activated - command not processed", buffered_addr)

                logger.error(f"E-STOP TRIGGERED! Active command '{cancelled_command_info}' cancelled.")
                logger.info("Release E-Stop and press 'e' to re-enable.")
                e_stop_active = True

            Command_out.value = 102
            Speed_out[:] = [0] * 6
            Gripper_data_out[3] = 0
            active_command = None
            active_command_id = None
            command_id_map.clear()
            incoming_command_buffer.clear()
            
        elif e_stop_active:
            # Waiting for re-enable
            try:
                if keyboard.is_pressed('e'):
                    logger.info("Re-enabling robot...")
                    Command_out.value = 101
                    e_stop_active = False
                else:
                    Command_out.value = 255
                    Speed_out[:] = [0] * 6
                    Position_out[:] = Position_in[:]
            except (ImportError, Exception):
                # Keyboard library requires root on Linux, so it may not work
                # Just maintain E-stop state until cleared via UDP command
                Command_out.value = 255
                Speed_out[:] = [0] * 6
                Position_out[:] = Position_in[:]
                
        else:
            # --- Normal Command Processing ---

            # Start new command if none active
            logger.debug(f"[DEBUG] Checking for new command: active_command={'None' if active_command is None else type(active_command).__name__}, queue_empty={command_queue.is_empty}, queue_size={command_queue.size}")
            if active_command is None and not command_queue.is_empty:
                logger.info(f"[DEBUG] Popping command from queue (size before pop: {command_queue.size})")
                new_command = command_queue.pop()
                
                # Get command ID and address if tracked
                cmd_info = command_id_map.get(new_command, (None, None))
                new_cmd_id, new_addr = cmd_info
                
                # Initial validation
                if hasattr(new_command, 'is_valid') and not new_command.is_valid:
                    # Command was invalid from the start
                    if new_cmd_id:
                        network_handler.send_ack(new_cmd_id, "INVALID", 
                                        "Initial validation failed", new_addr)
                    if new_command in command_id_map:
                        del command_id_map[new_command]
                    continue  # Skip to next command
                
                # Prepare command
                if hasattr(new_command, 'prepare_for_execution'):
                    try:
                        logger.info(f"[DEBUG] Calling prepare_for_execution for {type(new_command).__name__}")
                        new_command.prepare_for_execution(current_position_in=Position_in)
                        logger.info(f"[DEBUG] prepare_for_execution completed successfully")
                    except Exception as e:
                        logger.error(f"[DEBUG] Command preparation failed: {e}", exc_info=True)
                        if hasattr(new_command, 'is_valid'):
                            new_command.is_valid = False
                        if hasattr(new_command, 'error_message'):
                            new_command.error_message = str(e)
                else:
                    logger.info(f"[DEBUG] Command has no prepare_for_execution method")
                
                # Check if still valid after preparation
                logger.info(f"[DEBUG] After preparation: is_valid={getattr(new_command, 'is_valid', 'no is_valid attribute')}")
                if hasattr(new_command, 'is_valid') and not new_command.is_valid:
                    # Failed during preparation
                    error_msg = "Failed during preparation"
                    if hasattr(new_command, 'error_message'):
                        error_msg = new_command.error_message

                    logger.error(f"[DEBUG] Command rejected after preparation: {error_msg}")

                    if new_cmd_id:
                        network_handler.send_ack(new_cmd_id, "FAILED", error_msg, new_addr)

                    # Clean up
                    if new_command in command_id_map:
                        del command_id_map[new_command]
                else:
                    # Command is valid, make it active
                    logger.info(f"[DEBUG] Making command active: {type(new_command).__name__}")
                    active_command = new_command
                    active_command_id = new_cmd_id

                    # Reset performance tracking for this command
                    active_command_start_time = time.time()
                    active_command_perf_samples = []

                    if new_cmd_id:
                        network_handler.send_ack(new_cmd_id, "EXECUTING",
                                        f"Starting {type(new_command).__name__}", new_addr)
            
            # Execute active command
            if active_command:
                try:
                    is_done = active_command.execute_step(
                        Position_in=Position_in,
                        Homed_in=Homed_in,
                        Speed_out=Speed_out,
                        Command_out=Command_out,
                        Gripper_data_out=Gripper_data_out,
                        InOut_out=InOut_out,
                        InOut_in=InOut_in,
                        Gripper_data_in=Gripper_data_in,
                        Position_out=Position_out  # Add this if needed
                    )
                    
                    if is_done:
                        # Command completed - handle performance data
                        if active_command_start_time and active_command_perf_samples:
                            duration = time.time() - active_command_start_time
                            num_cycles = len(active_command_perf_samples)

                            # Calculate statistics
                            cycle_times = [s['cycle'] for s in active_command_perf_samples]
                            avg_cycle = sum(cycle_times) / len(cycle_times)
                            min_cycle = min(cycle_times)
                            max_cycle = max(cycle_times)

                            # Average phase times
                            avg_network = sum(s['network'] for s in active_command_perf_samples) / num_cycles
                            avg_processing = sum(s['processing'] for s in active_command_perf_samples) / num_cycles
                            avg_execution = sum(s['execution'] for s in active_command_perf_samples) / num_cycles
                            avg_serial = sum(s['serial'] for s in active_command_perf_samples) / num_cycles
                            avg_ik_manipulability = sum(s.get('ik_manipulability', 0) for s in active_command_perf_samples) / num_cycles
                            avg_ik_solve = sum(s.get('ik_solve', 0) for s in active_command_perf_samples) / num_cycles

                            cmd_name = type(active_command).__name__
                            cmd_id = active_command_id if active_command_id else 'N/A'

                            # If recording is enabled, save this command as a separate recording file
                            if recording_enabled:
                                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]  # Include milliseconds
                                filename = f"{timestamp}_{cmd_name}.json"
                                filepath = PROJECT_ROOT / "recordings" / filename

                                # Create recording object for this single command
                                recording_obj = {
                                    "metadata": {
                                        "name": f"{cmd_name}_{timestamp}",
                                        "timestamp": datetime.datetime.now().isoformat(),
                                        "robot_config": {
                                            "com_port": config.get('robot', {}).get('com_port', ''),
                                            "baud_rate": config.get('robot', {}).get('baud_rate', 3000000)
                                        }
                                    },
                                    "commands": [{
                                        "command_id": cmd_id,
                                        "command_type": cmd_name,
                                        "timestamp": datetime.datetime.now().isoformat(),
                                        "duration_s": duration,
                                        "num_cycles": num_cycles,
                                        "cycle_stats": {
                                            "avg_ms": avg_cycle,
                                            "min_ms": min_cycle,
                                            "max_ms": max_cycle
                                        },
                                        "phase_stats": {
                                            "network_ms": avg_network,
                                            "processing_ms": avg_processing,
                                            "execution_ms": avg_execution,
                                            "serial_ms": avg_serial,
                                            "ik_manipulability_ms": avg_ik_manipulability,
                                            "ik_solve_ms": avg_ik_solve
                                        },
                                        "samples": active_command_perf_samples
                                    }]
                                }

                                try:
                                    with open(filepath, 'w') as f:
                                        json.dump(recording_obj, f, indent=2)
                                    logger.info(f"[Recording] Saved {cmd_name}: {filename} ({num_cycles} cycles, {duration:.2f}s)")
                                except Exception as e:
                                    logger.error(f"[Recording] Failed to save {filename}: {e}")

                        if active_command_id:
                            # Check for error state in smooth motion commands
                            if hasattr(active_command, 'error_state') and active_command.error_state:
                                error_msg = getattr(active_command, 'error_message', 'Command failed during execution')
                                network_handler.send_ack(active_command_id, "FAILED", error_msg)
                            else:
                                network_handler.send_ack(active_command_id, "COMPLETED",
                                                f"{type(active_command).__name__} finished successfully")

                        # Clean up
                        if active_command in command_id_map:
                            del command_id_map[active_command]

                        active_command = None
                        active_command_id = None
                        active_command_start_time = None
                        active_command_perf_samples = []
                        
                except Exception as e:
                    # Command execution error
                    logger.error(f"Command execution error: {e}")
                    if active_command_id:
                        network_handler.send_ack(active_command_id, "FAILED", 
                                          f"Execution error: {str(e)}")
                    
                    # Clean up
                    if active_command in command_id_map:
                        del command_id_map[active_command]
                    
                    active_command = None
                    active_command_id = None
                    
            else:
                # No active command - idle
                Command_out.value = 255
                Speed_out[:] = [0] * 6
                Position_out[:] = Position_in[:]

        # --- Communication with Robot ---
        performance_monitor.start_phase('serial')
        s = Pack_data(Position_out, Speed_out, Command_out.value,
                     Affected_joint_out, InOut_out, Timeout_out, Gripper_data_out)
        for chunk in s:
            ser.write(chunk)

        Get_data(Position_in, Speed_in, Homed_in, InOut_in, Temperature_error_in,
                Position_error_in, Timeout_error, Timing_data_in, XTR_data, Gripper_data_in)
        performance_monitor.end_phase('serial')

    except serial.SerialException as e:
        logger.error(f"Serial communication error: {e}")
        
        # Send failure acknowledgments for active command
        if active_command_id:
            network_handler.send_ack(active_command_id, "FAILED", "Serial communication lost")
        
        if ser:
            ser.close()
        ser = None
        active_command = None
        active_command_id = None

    # ========================================================================
    # TIER 2: Performance monitoring - end cycle timing
    # ========================================================================
    performance_monitor.end_cycle()

    # Capture performance data for active command
    if active_command and active_command_start_time:
        latest_times = performance_monitor.get_latest_phase_times()
        if latest_times:
            # In DEBUG mode, we have detailed phase data
            active_command_perf_samples.append(latest_times)
        else:
            logger.debug(f"[PERF] No latest_times available (debug_mode={performance_monitor._debug_mode})")

    timer.checkpt()