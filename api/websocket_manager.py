"""
WebSocket connection manager for real-time robot data streaming
"""

from typing import Dict, Set, List, Optional
from fastapi import WebSocket
import json
import asyncio
from datetime import datetime
import logging
import yaml
from pathlib import Path
from lib.models.api_models import WebSocketMessage, WebSocketError, RobotStatus

logger = logging.getLogger(__name__)


class ConnectionManager:
    """Manages WebSocket connections and broadcasts robot state"""

    def __init__(self):
        # Active connections mapped by client ID
        self.active_connections: Dict[str, WebSocket] = {}
        # Client subscriptions (which data types each client wants)
        self.subscriptions: Dict[str, Set[str]] = {}
        # Update rates for each client (Hz)
        self.update_rates: Dict[str, int] = {}
        # Last update time for rate limiting
        self.last_update: Dict[str, datetime] = {}
        # Log level filters per client
        self.log_filters: Dict[str, str] = {}  # client_id -> min log level

        # Load frontend log level from config
        self.frontend_log_level = self._load_frontend_log_level()

    def _load_frontend_log_level(self) -> int:
        """Load frontend log level from config.yaml"""
        try:
            config_path = Path(__file__).parent.parent / "config.yaml"
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)

            level_str = config.get('logging', {}).get('frontend', {}).get('level', 'DEBUG')
            level_map = {
                'DEBUG': logging.DEBUG,
                'INFO': logging.INFO,
                'WARNING': logging.WARNING,
                'ERROR': logging.ERROR,
                'CRITICAL': logging.CRITICAL
            }
            return level_map.get(level_str.upper(), logging.DEBUG)
        except Exception as e:
            logger.warning(f"Failed to load frontend log level from config: {e}, using DEBUG")
            return logging.DEBUG
        
    async def connect(self, websocket: WebSocket, client_id: str):
        """Accept new WebSocket connection"""
        await websocket.accept()
        self.active_connections[client_id] = websocket
        self.subscriptions[client_id] = {'status'}  # Default subscription
        self.update_rates[client_id] = 10  # Default 10Hz
        self.last_update[client_id] = datetime.now()
        self.log_filters[client_id] = 'INFO'  # Default log level
        logger.debug(f"Client {client_id} connected")
        
    def disconnect(self, client_id: str):
        """Remove disconnected client"""
        if client_id in self.active_connections:
            del self.active_connections[client_id]
            del self.subscriptions[client_id]
            del self.update_rates[client_id]
            del self.last_update[client_id]
            del self.log_filters[client_id]
            logger.debug(f"Client {client_id} disconnected")
            
    async def send_personal_message(self, message: str, client_id: str):
        """Send message to specific client"""
        if client_id in self.active_connections:
            websocket = self.active_connections[client_id]
            try:
                await websocket.send_text(message)
            except Exception as e:
                logger.error(f"Error sending to {client_id}: {e}")
                self.disconnect(client_id)
                
    async def send_json_to_client(self, data: dict, client_id: str):
        """Send JSON data to specific client"""
        message = json.dumps(data, default=str)
        await self.send_personal_message(message, client_id)
        
    async def _broadcast_to_clients(self, data: dict, subscription_type: str, client_ids: List[str], timestamp: datetime):
        """Internal method to broadcast to specific clients (rate limiting already checked)"""
        disconnected_clients = []

        for client_id in client_ids:
            # Check if client is subscribed to this data type
            if subscription_type not in self.subscriptions.get(client_id, set()):
                continue

            # Send the data
            websocket = self.active_connections.get(client_id)
            if websocket:
                try:
                    message = json.dumps(data, default=str)
                    await websocket.send_text(message)
                except Exception as e:
                    logger.error(f"Error broadcasting to {client_id}: {e}")
                    disconnected_clients.append(client_id)

        # Clean up disconnected clients
        for client_id in disconnected_clients:
            self.disconnect(client_id)

    async def broadcast_json(self, data: dict, subscription_type: Optional[str] = None):
        """Broadcast JSON data to all subscribed clients"""
        # Get current time for rate limiting
        now = datetime.now()

        # List to track disconnected clients
        disconnected_clients = []

        # Create a snapshot of connections to avoid "dictionary changed size" error
        for client_id, websocket in list(self.active_connections.items()):
            # Check if client is subscribed to this data type
            if subscription_type and subscription_type not in self.subscriptions.get(client_id, set()):
                continue

            # Check rate limiting
            last_update = self.last_update.get(client_id, now)
            rate_hz = self.update_rates.get(client_id, 10)
            min_interval_ms = 1000 / rate_hz

            if (now - last_update).total_seconds() * 1000 < min_interval_ms:
                continue  # Skip this update due to rate limiting

            # Send the data
            try:
                message = json.dumps(data, default=str)
                await websocket.send_text(message)
                self.last_update[client_id] = now
            except Exception as e:
                logger.error(f"Error broadcasting to {client_id}: {e}")
                disconnected_clients.append(client_id)

        # Clean up disconnected clients
        for client_id in disconnected_clients:
            self.disconnect(client_id)
            
    async def broadcast_robot_status(self, status: RobotStatus):
        """Broadcast robot status to subscribed clients"""
        # Get current time for rate limiting
        now = datetime.now()

        # Check rate limiting ONCE per client before broadcasting all messages
        clients_to_update = []
        for client_id in list(self.active_connections.keys()):
            last_update = self.last_update.get(client_id, now)
            rate_hz = self.update_rates.get(client_id, 10)
            min_interval_ms = 1000 / rate_hz

            if (now - last_update).total_seconds() * 1000 >= min_interval_ms:
                clients_to_update.append(client_id)

        # If no clients need updates, skip broadcasting
        if not clients_to_update:
            return

        # Create separate messages for each data type
        if status.pose:
            await self._broadcast_to_clients({
                "type": "pose",
                "data": status.pose.dict(),
                "timestamp": now.isoformat()
            }, "pose", clients_to_update, now)

        if status.joints:
            await self._broadcast_to_clients({
                "type": "joints",
                "data": status.joints.dict(),
                "timestamp": now.isoformat()
            }, "joints", clients_to_update, now)

        if status.speeds:
            await self._broadcast_to_clients({
                "type": "speeds",
                "data": status.speeds.dict(),
                "timestamp": now.isoformat()
            }, "speeds", clients_to_update, now)

        if status.io:
            await self._broadcast_to_clients({
                "type": "io",
                "data": status.io.dict(),
                "timestamp": now.isoformat()
            }, "io", clients_to_update, now)

        if status.gripper:
            await self._broadcast_to_clients({
                "type": "gripper",
                "data": status.gripper.dict(),
                "timestamp": now.isoformat()
            }, "gripper", clients_to_update, now)

        # Always send overall status to status subscribers
        await self._broadcast_to_clients({
            "type": "status",
            "data": status.dict(),
            "timestamp": now.isoformat()
        }, "status", clients_to_update, now)

        # Update last_update timestamp ONCE after all messages sent
        for client_id in clients_to_update:
            self.last_update[client_id] = now
        
    async def handle_client_message(self, client_id: str, message: str):
        """Handle incoming message from client"""
        try:
            data = json.loads(message)

            # Handle subscription updates
            if "subscribe" in data:
                subscriptions = set(data["subscribe"])
                valid_types = {'pose', 'joints', 'speeds', 'io', 'gripper', 'status', 'logs'}
                self.subscriptions[client_id] = subscriptions & valid_types
                logger.debug(f"Client {client_id[:8]} subscribed to: {list(self.subscriptions[client_id])}")

                # Send confirmation
                await self.send_json_to_client({
                    "type": "subscription_update",
                    "subscribed": list(self.subscriptions[client_id]),
                    "timestamp": datetime.now().isoformat()
                }, client_id)
                
            # Handle rate updates
            if "rate_hz" in data:
                rate = int(data["rate_hz"])
                if 1 <= rate <= 50:
                    self.update_rates[client_id] = rate
                    await self.send_json_to_client({
                        "type": "rate_update",
                        "rate_hz": rate,
                        "timestamp": datetime.now().isoformat()
                    }, client_id)
                else:
                    await self.send_json_to_client({
                        "type": "error",
                        "error": "Rate must be between 1 and 50 Hz",
                        "timestamp": datetime.now().isoformat()
                    }, client_id)
                    
            # Handle log level filter updates
            if "log_level" in data:
                log_level = data["log_level"].upper()
                valid_levels = ['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL']
                if log_level in valid_levels:
                    self.log_filters[client_id] = log_level
                    await self.send_json_to_client({
                        "type": "log_level_update",
                        "log_level": log_level,
                        "timestamp": datetime.now().isoformat()
                    }, client_id)
                else:
                    await self.send_json_to_client({
                        "type": "error",
                        "error": f"Invalid log level: {log_level}",
                        "timestamp": datetime.now().isoformat()
                    }, client_id)

            # Handle frontend log messages
            if "type" in data and data["type"] == "frontend_log":
                level = data.get("level", "INFO").upper()
                message = data.get("message", "")
                source = data.get("source", "frontend")
                details = data.get("details")

                # Map log levels to Python logging
                level_map = {
                    'DEBUG': logging.DEBUG,
                    'INFO': logging.INFO,
                    'WARNING': logging.WARNING,
                    'ERROR': logging.ERROR,
                    'CRITICAL': logging.CRITICAL
                }

                # Format message with details if present
                if details:
                    try:
                        details_str = json.dumps(details, indent=2, default=str)
                        full_message = f"{message}\n{details_str}"
                    except:
                        full_message = f"{message} | {details}"
                else:
                    full_message = message

                # Inject frontend log into backend logging system
                log_level = level_map.get(level, logging.INFO)
                frontend_logger = logging.getLogger(f"frontend.{source}")
                # Apply configured frontend log level
                frontend_logger.setLevel(self.frontend_log_level)
                frontend_logger.log(log_level, full_message)

        except json.JSONDecodeError:
            await self.send_json_to_client({
                "type": "error",
                "error": "Invalid JSON message",
                "timestamp": datetime.now().isoformat()
            }, client_id)
        except Exception as e:
            logger.error(f"Error handling message from {client_id}: {e}")
            await self.send_json_to_client({
                "type": "error",
                "error": str(e),
                "timestamp": datetime.now().isoformat()
            }, client_id)
            
    def get_connection_count(self) -> int:
        """Get number of active connections"""
        return len(self.active_connections)
        
    def get_client_info(self) -> List[Dict]:
        """Get information about all connected clients"""
        return [
            {
                "client_id": client_id,
                "subscriptions": list(self.subscriptions.get(client_id, set())),
                "rate_hz": self.update_rates.get(client_id, 10),
                "last_update": self.last_update.get(client_id, datetime.now()).isoformat()
            }
            for client_id in self.active_connections.keys()
        ]