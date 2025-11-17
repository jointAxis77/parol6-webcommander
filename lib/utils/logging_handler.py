"""
Custom logging handler for streaming logs via WebSocket and UDP forwarding
"""

import logging
from collections import deque
from datetime import datetime
from typing import Optional, List, Dict, Any
import json
import socket


class WebSocketLogHandler(logging.Handler):
    """
    Logging handler that stores logs in a circular buffer and 
    allows broadcasting to WebSocket clients
    """
    
    def __init__(self, buffer_size: int = 1000):
        super().__init__()
        self.buffer_size = buffer_size
        self.logs = deque(maxlen=buffer_size)
        self.websocket_manager = None
        
    def set_websocket_manager(self, manager):
        """Set the WebSocket manager for broadcasting logs"""
        self.websocket_manager = manager
        
    def emit(self, record: logging.LogRecord):
        """Called by logging system for each log message"""
        try:
            # Format the log entry
            log_entry = {
                "timestamp": datetime.now().isoformat(),
                "level": record.levelname,
                "source": record.name,
                "message": self.format(record),
                "module": record.module,
                "function": record.funcName,
                "line": record.lineno
            }
            
            # Add any extra fields from the record
            if hasattr(record, 'extra'):
                log_entry['details'] = record.extra
                
            # Store in circular buffer
            self.logs.append(log_entry)
            
            # Broadcast to WebSocket clients if manager is set
            if self.websocket_manager:
                import asyncio
                try:
                    # Create async task to broadcast
                    loop = asyncio.get_event_loop()
                    if loop.is_running():
                        asyncio.create_task(
                            self.websocket_manager.broadcast_json({
                                "type": "log",
                                "data": log_entry
                            }, "logs")
                        )
                except RuntimeError:
                    # No event loop running, skip WebSocket broadcast
                    pass
                    
        except Exception as e:
            # Don't let logging errors crash the application
            # Use sys.stderr instead of logger to avoid potential infinite recursion
            import sys
            sys.stderr.write(f"Error in WebSocketLogHandler: {e}\n")
            
    def get_logs(self, 
                 level: Optional[str] = None,
                 source: Optional[str] = None,
                 limit: Optional[int] = None) -> List[Dict[str, Any]]:
        """Get filtered logs from buffer"""
        logs_list = list(self.logs)
        
        # Filter by level if specified
        if level:
            logs_list = [log for log in logs_list if log['level'] == level.upper()]
            
        # Filter by source if specified
        if source:
            logs_list = [log for log in logs_list if source in log['source']]
            
        # Limit results if specified
        if limit:
            logs_list = logs_list[-limit:]
            
        return logs_list
        
    def clear_logs(self):
        """Clear the log buffer"""
        self.logs.clear()
        
    def export_logs(self, format: str = "json") -> str:
        """Export logs in specified format"""
        logs_list = list(self.logs)
        
        if format == "json":
            return json.dumps(logs_list, indent=2, default=str)
        elif format == "text":
            lines = []
            for log in logs_list:
                line = f"{log['timestamp']} [{log['level']}] {log['source']}: {log['message']}"
                lines.append(line)
            return "\n".join(lines)
        else:
            raise ValueError(f"Unsupported format: {format}")


class UDPLogHandler(logging.Handler):
    """
    Logging handler that forwards logs to a UDP receiver (typically fastapi_server)
    Non-blocking, fire-and-forget design for minimal overhead
    """

    def __init__(self, host: str = '127.0.0.1', port: int = 5003):
        super().__init__()
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Set socket to non-blocking to avoid delays
        self.socket.setblocking(False)

    def emit(self, record: logging.LogRecord):
        """Send log entry via UDP"""
        try:
            # Format the log entry (same structure as WebSocketLogHandler)
            log_entry = {
                "timestamp": datetime.now().isoformat(),
                "level": record.levelname,
                "source": record.name,
                "message": self.format(record),
                "module": record.module,
                "function": record.funcName,
                "line": record.lineno
            }

            # Add any extra fields
            if hasattr(record, 'extra'):
                log_entry['details'] = record.extra

            # Send as JSON over UDP (fire and forget)
            message = json.dumps(log_entry, default=str).encode('utf-8')
            self.socket.sendto(message, (self.host, self.port))

        except Exception:
            # Silently fail - don't let logging errors crash the application
            # Don't even log the error to avoid potential recursion
            pass

    def close(self):
        """Close the UDP socket"""
        try:
            self.socket.close()
        except:
            pass
        super().close()


# Global instance
_websocket_handler = None


def get_websocket_handler(buffer_size: int = 1000) -> WebSocketLogHandler:
    """Get or create the global WebSocket log handler"""
    global _websocket_handler
    if _websocket_handler is None:
        _websocket_handler = WebSocketLogHandler(buffer_size)
    return _websocket_handler


def setup_logging(config: Dict[str, Any], service_name: str = None):
    """
    Configure logging system with WebSocket handler and optional UDP forwarding

    Args:
        config: Logging configuration dict with keys:
            - level: Log level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
            - <service_name>: Per-service config dict with 'level' key (optional)
            - buffer_size: Number of logs to keep in memory
            - stream_to_websocket: Whether to stream logs via WebSocket
            - file_output: Optional file path for file logging
            - log_forward_enabled: Whether to forward logs via UDP
            - log_forward_host: UDP destination host (default: 127.0.0.1)
            - log_forward_port: UDP destination port (default: 5003)
        service_name: Optional service name to read service-specific config (e.g., 'commander', 'api')
    """
    # Get config values - check service-specific config first, then fall back to global
    if service_name and service_name in config:
        level = config[service_name].get('level', 'INFO')
    else:
        level = config.get('level', 'INFO')
    buffer_size = config.get('buffer_size', 1000)
    stream_to_websocket = config.get('stream_to_websocket', True)
    file_output = config.get('file_output')
    log_forward_enabled = config.get('log_forward_enabled', False)
    log_forward_host = config.get('log_forward_host', '127.0.0.1')
    log_forward_port = config.get('log_forward_port', 5003)

    # Set up root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(getattr(logging, level.upper()))

    # Clear existing handlers
    root_logger.handlers.clear()

    # Console handler (still useful for development)
    console_handler = logging.StreamHandler()
    console_handler.setLevel(getattr(logging, level.upper()))
    console_formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    console_handler.setFormatter(console_formatter)
    root_logger.addHandler(console_handler)

    # WebSocket handler
    if stream_to_websocket:
        ws_handler = get_websocket_handler(buffer_size)
        ws_handler.setLevel(getattr(logging, level.upper()))
        ws_handler.setFormatter(console_formatter)
        root_logger.addHandler(ws_handler)

    # UDP forward handler (for headless_commander to send logs to fastapi_server)
    if log_forward_enabled:
        udp_handler = UDPLogHandler(host=log_forward_host, port=log_forward_port)
        udp_handler.setLevel(getattr(logging, level.upper()))
        udp_handler.setFormatter(console_formatter)
        root_logger.addHandler(udp_handler)

    # File handler
    if file_output:
        file_handler = logging.FileHandler(file_output)
        file_handler.setLevel(getattr(logging, level.upper()))
        file_handler.setFormatter(console_formatter)
        root_logger.addHandler(file_handler)

    # Set specific logger levels to reduce noise
    logging.getLogger('asyncio').setLevel(logging.WARNING)
    logging.getLogger('uvicorn.access').setLevel(logging.DEBUG)  # Enable access logs