"""
PAROL6 Serial Protocol Module

This module handles binary serial communication protocol for PAROL6 robot.
Includes packet encoding/decoding, data packing/unpacking, and protocol constants.

Protocol Structure:
-------------------
TX (PC -> Robot):
- Start bytes: 0xFF 0xFF 0xFF
- Length: 1 byte (52)
- Position data: 18 bytes (6 joints × 3 bytes each)
- Speed data: 18 bytes (6 joints × 3 bytes each)
- Command: 1 byte
- Affected joints: 1 byte (bitfield)
- I/O: 1 byte (bitfield)
- Timeout: 1 byte
- Gripper data: 9 bytes
- CRC: 1 byte
- End bytes: 0x01 0x02

RX (Robot -> PC):
- Start bytes: 0xFF 0xFF 0xFF
- Length: 1 byte
- Position data: 18 bytes
- Speed data: 18 bytes
- Homed status: 1 byte (bitfield)
- I/O status: 1 byte (bitfield)
- Error flags: 2 bytes (bitfields)
- Timing data: 2 bytes
- Timeout: 1 byte
- Device info: 2 bytes
- Gripper data: 8 bytes
- CRC: 1 byte
- End bytes: 0x01 0x02

Author: Extracted from headless_commander.py
Date: 2025-01-12
"""

import struct
import logging

# Get logger
logger = logging.getLogger(__name__)

# ============================================================================
# Protocol Constants
# ============================================================================

# Packet framing bytes
START_BYTES = bytes([0xFF, 0xFF, 0xFF])
END_BYTES = bytes([0x01, 0x02])

# Individual start/end condition bytes (for parsing)
START_COND1_BYTE = bytes([0xFF])
START_COND2_BYTE = bytes([0xFF])
START_COND3_BYTE = bytes([0xFF])
END_COND1_BYTE = bytes([0x01])
END_COND2_BYTE = bytes([0x02])

# Packet lengths
TX_PACKET_LENGTH = 52  # Length byte value for TX packets
RX_DATA_BUFFER_SIZE = 120  # Maximum receive buffer size

# Struct packers/unpackers
INT_TO_3_BYTES = struct.Struct('>I').pack  # Big-endian 4-byte int (use last 3)


# ============================================================================
# Data Conversion Utilities
# ============================================================================

def split_to_3_bytes(value):
    """
    Convert integer to 3-byte representation (big-endian).

    Parameters
    ----------
    value : int
        Integer value to convert (masked to 24 bits)

    Returns
    -------
    bytes
        3-byte representation
    """
    return INT_TO_3_BYTES(value & 0xFFFFFF)[1:4]


def fuse_3_bytes(byte_array):
    """
    Convert 3 bytes to signed integer.

    Parameters
    ----------
    byte_array : bytes
        3-byte or 4-byte array (if 4, first byte ignored)

    Returns
    -------
    int
        Signed integer value
    """
    value = struct.unpack(">I", bytearray(byte_array))[0]

    # Convert to negative number if it is negative (two's complement)
    if value >= 1 << 23:
        value -= 1 << 24

    return value


def fuse_2_bytes(byte_array):
    """
    Convert 2 bytes to signed integer.

    Parameters
    ----------
    byte_array : bytes
        2-byte or 4-byte array (if 4, first 2 bytes ignored)

    Returns
    -------
    int
        Signed integer value
    """
    value = struct.unpack(">I", bytearray(byte_array))[0]

    # Convert to negative number if it is negative (two's complement)
    if value >= 1 << 15:
        value -= 1 << 16

    return value


def split_to_bitfield(byte_value):
    """
    Split byte into 8-bit list.

    Parameters
    ----------
    byte_value : int
        Byte value (0-255)

    Returns
    -------
    list
        List of 8 bits [MSB, ..., LSB]
    """
    return [(byte_value >> i) & 1 for i in range(7, -1, -1)]


def fuse_bitfield_to_byte(bitfield):
    """
    Fuse 8-bit list into byte.

    Parameters
    ----------
    bitfield : list
        List of 8 bits [MSB, ..., LSB]

    Returns
    -------
    bytes
        Single byte
    """
    number = 0
    for b in bitfield:
        number = (2 * number) + b
    return bytes([number])


# ============================================================================
# Packet Packing (PC -> Robot)
# ============================================================================

def pack_command_packet(position_out, speed_out, command_out,
                       affected_joint_out, io_out, timeout_out, gripper_data_out):
    """
    Pack command data into serial packet format for transmission to robot.

    Parameters
    ----------
    position_out : list
        Joint positions in steps [J1, J2, J3, J4, J5, J6]
    speed_out : list
        Joint speeds in steps/second [J1, J2, J3, J4, J5, J6]
    command_out : int
        Command byte (e.g., 123=jog, 156=position, 255=idle)
    affected_joint_out : list
        Bitfield [J1, J2, J3, J4, J5, J6, -, -]
    io_out : list
        I/O bitfield [8 bits]
    timeout_out : int
        Timeout value
    gripper_data_out : list
        Gripper data [position, speed, current, command, mode, id]

    Returns
    -------
    list
        List of bytes objects ready for serial transmission

    Notes
    -----
    Gripper calibrate/clear_error modes are automatically reset after packing
    to prevent endless loops.
    """
    packet = []

    # Start bytes and length
    packet.append(START_BYTES)
    packet.append(bytes([TX_PACKET_LENGTH]))

    # Position data (6 joints × 3 bytes)
    for i in range(6):
        position_split = split_to_3_bytes(position_out[i])
        packet.append(position_split)

    # Speed data (6 joints × 3 bytes)
    for i in range(6):
        speed_split = split_to_3_bytes(speed_out[i])
        packet.append(speed_split)

    # Command byte
    packet.append(bytes([command_out]))

    # Affected joints bitfield
    affected_list = fuse_bitfield_to_byte(affected_joint_out[:])
    packet.append(affected_list)

    # I/O bitfield
    io_list = fuse_bitfield_to_byte(io_out[:])
    packet.append(io_list)

    # Timeout
    packet.append(bytes([timeout_out]))

    # Gripper data
    gripper_position = split_to_3_bytes(gripper_data_out[0])
    packet.append(gripper_position[1:3])  # Use last 2 bytes

    gripper_speed = split_to_3_bytes(gripper_data_out[1])
    packet.append(gripper_speed[1:3])

    gripper_current = split_to_3_bytes(gripper_data_out[2])
    packet.append(gripper_current[1:3])

    packet.append(bytes([gripper_data_out[3]]))  # Gripper command
    packet.append(bytes([gripper_data_out[4]]))  # Gripper mode

    # FIX: Make sure calibrate is a one-shot command
    # If the mode was set to calibrate (1) or clear_error (2), reset it
    # back to normal (0) for the next cycle. This prevents an endless loop.
    if gripper_data_out[4] == 1 or gripper_data_out[4] == 2:
        gripper_data_out[4] = 0

    packet.append(bytes([gripper_data_out[5]]))  # Gripper ID

    # CRC byte (placeholder - actual CRC not implemented)
    packet.append(bytes([228]))

    # End bytes
    packet.append(END_BYTES)

    return packet


# ============================================================================
# Packet Unpacking (Robot -> PC)
# ============================================================================

def unpack_feedback_packet(data_buffer, position_in, speed_in, homed_in, io_in,
                          temperature_error_in, position_error_in, timeout_error,
                          timing_data_in, xtr_data, gripper_data_in):
    """
    Unpack feedback packet from robot into data arrays.

    Parameters
    ----------
    data_buffer : list
        Raw byte buffer received from robot
    position_in : list
        Output: Joint positions in steps [J1-J6, J7, J8]
    speed_in : list
        Output: Joint speeds in steps/second [J1-J6, ...]
    homed_in : list
        Output: Homing status bitfield [J1-J6, ...]
    io_in : list
        Output: I/O status bitfield [8 bits]
    temperature_error_in : list
        Output: Temperature error bitfield [8 bits]
    position_error_in : list
        Output: Position error bitfield [8 bits]
    timeout_error : list
        Output: Timeout error value
    timing_data_in : list
        Output: Timing data
    xtr_data : list
        Output: Extra data
    gripper_data_in : list
        Output: Gripper data [id, position, speed, current, status, object_detected]

    Notes
    -----
    This function modifies the input arrays in-place.
    """
    # Extract joint positions (18 bytes = 6 joints × 3 bytes)
    joints_raw = []
    for i in range(0, 18, 3):
        joints_raw.append(data_buffer[i:i+3])

    # Extract joint speeds (18 bytes)
    speeds_raw = []
    for i in range(18, 36, 3):
        speeds_raw.append(data_buffer[i:i+3])

    # Unpack positions and speeds
    for i in range(6):
        var = b'\x00' + b''.join(joints_raw[i])
        position_in[i] = fuse_3_bytes(var)

        var = b'\x00' + b''.join(speeds_raw[i])
        speed_in[i] = fuse_3_bytes(var)

    # Extract status bytes
    homed_byte = data_buffer[36]
    io_byte = data_buffer[37]
    temp_error_byte = data_buffer[38]
    position_error_byte = data_buffer[39]
    timing_data_bytes = data_buffer[40:42]
    timeout_error_byte = data_buffer[42]
    xtr2_byte = data_buffer[43]
    device_id_byte = data_buffer[44]

    # Extract gripper data
    gripper_position_bytes = data_buffer[45:47]
    gripper_speed_bytes = data_buffer[47:49]
    gripper_current_bytes = data_buffer[49:51]
    status_byte = data_buffer[51]
    # Note: Original object_detection byte at index 52 is ignored (unreliable)
    # CRC at 53, end bytes at 54-55

    # Unpack bitfields
    temp = split_to_bitfield(int.from_bytes(homed_byte, "big"))
    for i in range(8):
        homed_in[i] = temp[i]

    temp = split_to_bitfield(int.from_bytes(io_byte, "big"))
    for i in range(8):
        io_in[i] = temp[i]

    temp = split_to_bitfield(int.from_bytes(temp_error_byte, "big"))
    for i in range(8):
        temperature_error_in[i] = temp[i]

    temp = split_to_bitfield(int.from_bytes(position_error_byte, "big"))
    for i in range(8):
        position_error_in[i] = temp[i]

    # Unpack timing data
    var = b'\x00' + b'\x00' + b''.join(timing_data_bytes)
    timing_data_in[0] = fuse_3_bytes(var)

    # Timeout and extra data
    timeout_error[0] = int.from_bytes(timeout_error_byte, "big")
    xtr_data[0] = int.from_bytes(xtr2_byte, "big")

    # Unpack gripper data
    gripper_data_in[0] = int.from_bytes(device_id_byte, "big")

    var = b'\x00' + b'\x00' + b''.join(gripper_position_bytes)
    gripper_data_in[1] = fuse_2_bytes(var)

    var = b'\x00' + b'\x00' + b''.join(gripper_speed_bytes)
    gripper_data_in[2] = fuse_2_bytes(var)

    var = b'\x00' + b'\x00' + b''.join(gripper_current_bytes)
    gripper_data_in[3] = fuse_2_bytes(var)

    # Store raw status byte
    status_val = int.from_bytes(status_byte, "big")
    gripper_data_in[4] = status_val

    # Extract object detection status from bits 2-3 of status byte
    # This mirrors the working logic from GUI_PAROL_latest.py
    status_bits = split_to_bitfield(status_val)
    object_detection_status = (status_bits[2] << 1) | status_bits[3]
    gripper_data_in[5] = object_detection_status


# ============================================================================
# Serial Reception State Machine
# ============================================================================

class SerialReceiver:
    """
    State machine for receiving serial packets from robot.

    Handles packet framing, start/end byte detection, and data buffering.
    """

    def __init__(self):
        """Initialize serial receiver state machine."""
        self.input_byte = None
        self.start_cond1 = 0
        self.start_cond2 = 0
        self.start_cond3 = 0
        self.good_start = 0
        self.data_len = 0
        self.data_buffer = [bytes([0])] * RX_DATA_BUFFER_SIZE
        self.data_counter = 0

    def reset(self):
        """Reset receiver state."""
        self.good_start = 0
        self.start_cond1 = 0
        self.start_cond2 = 0
        self.start_cond3 = 0
        self.data_len = 0
        self.data_counter = 0

    def process_byte(self, byte_data):
        """
        Process single byte from serial stream.

        Parameters
        ----------
        byte_data : bytes
            Single byte from serial port

        Returns
        -------
        tuple or None
            (data_buffer, data_len) if complete packet received, None otherwise
        """
        self.input_byte = byte_data

        # State: Waiting for start sequence
        if self.good_start != 1:
            # All start bytes received, next is data length
            if self.start_cond1 == 1 and self.start_cond2 == 1 and self.start_cond3 == 1:
                self.good_start = 1
                self.data_len = struct.unpack('B', self.input_byte)[0]
                logger.debug(f"[SerialProtocol] Good start detected, data_len={self.data_len}")
                return None

            # Check third start byte
            if (self.input_byte == START_COND3_BYTE and
                    self.start_cond2 == 1 and self.start_cond1 == 1):
                self.start_cond3 = 1
            # Third byte bad, reset
            elif self.start_cond2 == 1 and self.start_cond1 == 1:
                self.start_cond1 = 0
                self.start_cond2 = 0

            # Check second start byte
            if self.input_byte == START_COND2_BYTE and self.start_cond1 == 1:
                self.start_cond2 = 1
            # Second byte bad, reset
            elif self.start_cond1 == 1:
                self.start_cond1 = 0

            # Check first start byte
            if self.input_byte == START_COND1_BYTE:
                self.start_cond1 = 1

            return None

        # State: Receiving data after good start
        else:
            self.data_buffer[self.data_counter] = self.input_byte

            # Check if we've received all data
            if self.data_counter == self.data_len - 1:
                logger.debug(f"[SerialProtocol] Packet complete: len={self.data_len}")

                # Verify end bytes
                if (self.data_buffer[self.data_len - 1] == END_COND2_BYTE and
                        self.data_buffer[self.data_len - 2] == END_COND1_BYTE):
                    logger.debug("[SerialProtocol] Good end condition")

                    # Return complete packet
                    result = (self.data_buffer.copy(), self.data_len)
                    self.reset()
                    return result
                else:
                    logger.warning("[SerialProtocol] Bad end condition")
                    self.reset()
                    return None
            else:
                self.data_counter += 1
                return None


# ============================================================================
# High-Level Interface
# ============================================================================

def receive_packets(serial_port, position_in, speed_in, homed_in, io_in,
                   temperature_error_in, position_error_in, timeout_error,
                   timing_data_in, xtr_data, gripper_data_in, receiver=None):
    """
    Receive and process all available packets from serial port.

    Parameters
    ----------
    serial_port : serial.Serial
        Open serial port object
    position_in, speed_in, etc. : list
        Output arrays (modified in-place)
    receiver : SerialReceiver, optional
        Existing receiver state machine (created if None)

    Returns
    -------
    SerialReceiver
        Receiver state machine for next call
    """
    if receiver is None:
        receiver = SerialReceiver()

    while serial_port.inWaiting() > 0:
        byte_data = serial_port.read()

        result = receiver.process_byte(byte_data)
        if result is not None:
            data_buffer, data_len = result
            unpack_feedback_packet(
                data_buffer, position_in, speed_in, homed_in, io_in,
                temperature_error_in, position_error_in, timeout_error,
                timing_data_in, xtr_data, gripper_data_in
            )
            logger.debug("[SerialProtocol] Packet unpacked successfully")

    return receiver


# ============================================================================
# Legacy Compatibility Functions
# ============================================================================
# These maintain backward compatibility with existing code

# Legacy function names (for backward compatibility)
Split_2_3_bytes = split_to_3_bytes
Fuse_3_bytes = fuse_3_bytes
Fuse_2_bytes = fuse_2_bytes
Split_2_bitfield = split_to_bitfield
Fuse_bitfield_2_bytearray = fuse_bitfield_to_byte
Pack_data = pack_command_packet
Unpack_data = unpack_feedback_packet
