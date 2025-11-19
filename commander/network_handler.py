"""
Network Handler Module for PAROL6 Robot

Handles UDP communication for robot commands and acknowledgments.
Provides clean interface for network operations separated from control loop.

Author: PAROL6 Team
Date: 2025-01-13
"""

import socket
import select
import logging
import time
from collections import deque
from typing import Optional, Tuple, Dict, Any, List
from dataclasses import dataclass

from constants import (
    UDP_COMMAND_PORT,
    UDP_ACK_PORT,
    COMMAND_COOLDOWN_S,
)


# ============================================================================
# Data Classes
# ============================================================================

@dataclass
class ReceivedCommand:
    """Represents a received command from network"""
    raw_message: str
    command_id: Optional[str]
    parsed_message: str
    sender_address: Tuple[str, int]
    timestamp: float


# ============================================================================
# Network Handler Class
# ============================================================================

class NetworkHandler:
    """
    Handles UDP network communication for robot commands.

    Responsibilities:
    - Receive commands on UDP port (non-blocking)
    - Send acknowledgments on separate port
    - Buffer incoming commands with rate limiting
    - Track command IDs and sender addresses
    - Parse command IDs from messages
    """

    def __init__(self,
                 logger: logging.Logger,
                 listen_ip: str = "127.0.0.1",
                 command_port: int = UDP_COMMAND_PORT,
                 ack_port: int = UDP_ACK_PORT,
                 buffer_max_size: int = 100):
        """
        Initialize network handler.

        Args:
            logger: Logger instance
            listen_ip: IP address to listen on (default: localhost)
            command_port: Port for receiving commands (default: 5001)
            ack_port: Port for sending acknowledgments (default: 5002)
            buffer_max_size: Maximum commands in buffer (default: 100)
        """
        self.logger = logger
        self.listen_ip = listen_ip
        self.command_port = command_port
        self.ack_port = ack_port
        self.buffer_max_size = buffer_max_size

        # Create sockets
        self.command_socket = None
        self.ack_socket = None

        # Command buffer (rate-limited processing)
        self.incoming_buffer = deque(maxlen=buffer_max_size)
        self.last_processed_time = 0

        # Statistics
        self.commands_received = 0
        self.commands_processed = 0
        self.acks_sent = 0
        self.network_errors = 0

    def initialize(self) -> bool:
        """
        Initialize UDP sockets.

        Returns:
            True if successful, False otherwise
        """
        try:
            # Create command receive socket
            self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.command_socket.bind((self.listen_ip, self.command_port))
            # Non-blocking mode handled via select()

            # Create ACK send socket
            self.ack_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            self.logger.info(f"[NetworkHandler] Listening on {self.listen_ip}:{self.command_port}")
            self.logger.info(f"[NetworkHandler] Sending ACKs to port {self.ack_port}")

            return True

        except Exception as e:
            self.logger.error(f"[NetworkHandler] Failed to initialize: {e}")
            return False

    def close(self):
        """Close all sockets"""
        try:
            if self.command_socket:
                self.command_socket.close()
            if self.ack_socket:
                self.ack_socket.close()
            self.logger.info("[NetworkHandler] Sockets closed")
        except Exception as e:
            self.logger.error(f"[NetworkHandler] Error closing sockets: {e}")

    # ========================================================================
    # Command Reception
    # ========================================================================

    def receive_commands(self) -> List[Tuple[str, str, str, Tuple[str, int]]]:
        """
        Receive all pending UDP commands (non-blocking).

        Returns:
            List of tuples: (raw_message, command_id, parsed_message, sender_address)

        Example:
            [("[abc123]MOVEJOINT|0,0,0,0,0,0|2.5|50",
              "abc123",
              "MOVEJOINT|0,0,0,0,0,0|2.5|50",
              ("192.168.1.100", 12345)
            )]
        """
        commands = []

        if not self.command_socket:
            return commands

        try:
            # Use select to check for data (non-blocking)
            while self.command_socket in select.select([self.command_socket], [], [], 0)[0]:
                data, addr = self.command_socket.recvfrom(65535)  # Max UDP packet size for large trajectories
                raw_message = data.decode('utf-8').strip()

                if raw_message:
                    # Parse command ID if present
                    cmd_id, parsed_message = self._parse_command_id(raw_message)

                    commands.append((raw_message, cmd_id, parsed_message, addr))
                    self.commands_received += 1

        except Exception as e:
            self.logger.error(f"[NetworkHandler] Receive error: {e}")
            self.network_errors += 1

        return commands

    def buffer_command(self, raw_message: str, addr: Tuple[str, int]):
        """
        Add command to processing buffer with rate limiting.

        Args:
            raw_message: Raw UDP message
            addr: Sender address (ip, port)
        """
        if len(self.incoming_buffer) >= self.buffer_max_size:
            self.logger.warning(f"[NetworkHandler] Buffer full ({self.buffer_max_size}), dropping command")
            return

        self.incoming_buffer.append((raw_message, addr))

    def get_next_buffered_command(self) -> Optional[Tuple[str, Tuple[str, int]]]:
        """
        Get next command from buffer if cooldown period has elapsed.

        Returns:
            Tuple of (raw_message, sender_address) or None if buffer empty or cooldown active

        Example:
            cmd_data = handler.get_next_buffered_command()
            if cmd_data:
                raw_message, addr = cmd_data
                # Process command
        """
        current_time = time.time()

        # Check cooldown
        if (current_time - self.last_processed_time) < COMMAND_COOLDOWN_S:
            return None

        # Check buffer
        if not self.incoming_buffer:
            return None

        # Get command from buffer
        self.last_processed_time = current_time
        self.commands_processed += 1

        return self.incoming_buffer.popleft()

    def clear_buffer(self):
        """Clear all buffered commands"""
        count = len(self.incoming_buffer)
        self.incoming_buffer.clear()
        if count > 0:
            self.logger.info(f"[NetworkHandler] Cleared {count} buffered commands")

    @property
    def buffer_size(self) -> int:
        """Get current buffer size"""
        return len(self.incoming_buffer)

    # ========================================================================
    # Acknowledgment Transmission
    # ========================================================================

    def send_ack(self,
                 command_id: Optional[str],
                 status: str,
                 details: str = "",
                 addr: Optional[Tuple[str, int]] = None):
        """
        Send acknowledgment to client.

        Args:
            command_id: Command ID to acknowledge (None = no ACK sent)
            status: Status string (QUEUED, EXECUTING, COMPLETED, FAILED, CANCELLED, INVALID)
            details: Optional details/error message
            addr: Sender address (ip, port) or None for broadcast only

        ACK Format: ACK|{command_id}|{status}|{details}

        Status Values:
            - QUEUED: Command added to queue
            - EXECUTING: Command started execution
            - COMPLETED: Command finished successfully
            - FAILED: Command execution failed
            - CANCELLED: Command cancelled (E-stop, STOP command, etc.)
            - INVALID: Command parsing or validation failed

        Example:
            handler.send_ack("abc123", "COMPLETED", "Motion finished")
            # Sends: "ACK|abc123|COMPLETED|Motion finished"
        """
        if not command_id:
            return

        if not self.ack_socket:
            self.logger.warning("[NetworkHandler] ACK socket not initialized")
            return

        ack_message = f"ACK|{command_id}|{status}|{details}"

        # Send to original sender if we have their address
        if addr:
            try:
                self.ack_socket.sendto(ack_message.encode('utf-8'), (addr[0], self.ack_port))
                self.acks_sent += 1
            except Exception as e:
                self.logger.error(f"[NetworkHandler] Failed to send ACK to {addr}: {e}")
                self.network_errors += 1

        # Also broadcast to localhost (for local clients)
        try:
            self.ack_socket.sendto(ack_message.encode('utf-8'), ('127.0.0.1', self.ack_port))
        except:
            pass  # Silent failure for broadcast

    def send_response(self, message: str, addr: Tuple[str, int]):
        """
        Send direct response message to client (for GET commands).

        Args:
            message: Response message (e.g., "POSE|..." or "ANGLES|...")
            addr: Recipient address (ip, port)

        Example:
            handler.send_response("ANGLES|0,0,0,0,0,0", client_addr)
        """
        if not self.command_socket:
            self.logger.warning("[NetworkHandler] Command socket not initialized")
            return

        try:
            self.command_socket.sendto(message.encode('utf-8'), addr)
        except Exception as e:
            self.logger.error(f"[NetworkHandler] Failed to send response to {addr}: {e}")
            self.network_errors += 1

    # ========================================================================
    # Helper Methods
    # ========================================================================

    @staticmethod
    def _parse_command_id(message: str) -> Tuple[Optional[str], str]:
        """
        Extract command ID from message if present.

        Format: [cmd_id]COMMAND|params...
        Alternative legacy format: cmd_id|COMMAND|params (8-char alphanumeric, not all uppercase)

        Args:
            message: Raw UDP message

        Returns:
            Tuple of (command_id or None, command_message)

        Example:
            >>> _parse_command_id("[abc123]MOVEJOINT|0,0,0,0,0,0|2.5|50")
            ("abc123", "MOVEJOINT|0,0,0,0,0,0|2.5|50")

            >>> _parse_command_id("abc12345|MOVEJOINT|0,0,0,0,0,0|2.5|50")
            ("abc12345", "MOVEJOINT|0,0,0,0,0,0|2.5|50")

            >>> _parse_command_id("MOVEJOINT|0,0,0,0,0,0|2.5|50")
            (None, "MOVEJOINT|0,0,0,0,0,0|2.5|50")
        """
        # Handle bracket format: [cmd_id]COMMAND
        if message.startswith('['):
            end_bracket_idx = message.find(']')
            if end_bracket_idx > 0:
                cmd_id = message[1:end_bracket_idx]
                command_message = message[end_bracket_idx+1:]
                return cmd_id, command_message

        # Handle pipe format: cmd_id|COMMAND (legacy)
        # Clean up any logging artifacts first
        if "ID:" in message or "):" in message:
            if "):" in message:
                message = message[message.rindex("):")+2:].strip()
            elif "ID:" in message:
                message = message[message.index("ID:")+3:].strip()
                message = message.lstrip('):').strip()

        parts = message.split('|', 1)

        # Check if first part looks like a valid command ID
        # Command IDs are 8 chars alphanumeric (may contain hyphens)
        # Exclude all-uppercase strings (those are command names)
        if (len(parts) > 1 and
            len(parts[0]) == 8 and
            parts[0].replace('-', '').isalnum() and
            not parts[0].isupper()):
            return parts[0], parts[1]

        return None, message

    # ========================================================================
    # Statistics and Status
    # ========================================================================

    def get_stats(self) -> Dict[str, Any]:
        """
        Get network handler statistics.

        Returns:
            Dictionary with statistics

        Example:
            stats = handler.get_stats()
            print(f"Commands received: {stats['commands_received']}")
        """
        return {
            'commands_received': self.commands_received,
            'commands_processed': self.commands_processed,
            'acks_sent': self.acks_sent,
            'network_errors': self.network_errors,
            'buffer_size': self.buffer_size,
            'buffer_max_size': self.buffer_max_size,
        }

    def reset_stats(self):
        """Reset statistics counters"""
        self.commands_received = 0
        self.commands_processed = 0
        self.acks_sent = 0
        self.network_errors = 0


# ============================================================================
# Command ID Tracking Helper
# ============================================================================

class CommandIDTracker:
    """
    Tracks command IDs and their associated metadata.

    Maps command objects to (command_id, sender_address) for ACK routing.
    """

    def __init__(self):
        self._map: Dict[Any, Tuple[Optional[str], Optional[Tuple[str, int]]]] = {}
        self.active_command_id: Optional[str] = None

    def track(self, command_obj: Any, cmd_id: Optional[str], addr: Optional[Tuple[str, int]]):
        """
        Track a command with its ID and address.

        Args:
            command_obj: Command object
            cmd_id: Command ID (can be None)
            addr: Sender address (can be None)
        """
        if cmd_id or addr:
            self._map[command_obj] = (cmd_id, addr)

    def get(self, command_obj: Any) -> Tuple[Optional[str], Optional[Tuple[str, int]]]:
        """
        Get command ID and address for a command object.

        Args:
            command_obj: Command object

        Returns:
            Tuple of (command_id, sender_address) or (None, None) if not tracked
        """
        return self._map.get(command_obj, (None, None))

    def remove(self, command_obj: Any):
        """Remove command from tracking"""
        if command_obj in self._map:
            del self._map[command_obj]

    def clear(self):
        """Clear all tracked commands"""
        self._map.clear()
        self.active_command_id = None

    def get_all_ids(self) -> List[Tuple[Any, str, Optional[Tuple[str, int]]]]:
        """
        Get all tracked command IDs.

        Returns:
            List of (command_obj, command_id, sender_address)
        """
        return [(cmd_obj, cmd_id, addr)
                for cmd_obj, (cmd_id, addr) in self._map.items()
                if cmd_id]

    @property
    def count(self) -> int:
        """Get number of tracked commands"""
        return len(self._map)


# ============================================================================
# Module Metadata
# ============================================================================

__version__ = "1.0.0"
__author__ = "PAROL6 Team"
__date__ = "2025-01-13"
__description__ = "Network communication handler for PAROL6 robot control system"
