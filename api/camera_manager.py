"""
Camera Manager Module

Handles USB camera detection, streaming, and configuration for PAROL6 robot.
Uses OpenCV for camera access and MJPEG streaming.
"""

import cv2
import os
import glob
import threading
import time
import logging
from typing import Optional, List, Dict, Tuple
import yaml
from pathlib import Path

logger = logging.getLogger(__name__)


class CameraManager:
    """Manages USB camera access and MJPEG streaming."""

    def __init__(self, config_path: str = "config.yaml"):
        self.config_path = Path(config_path)
        self.camera: Optional[cv2.VideoCapture] = None
        self.current_device: Optional[str] = None
        self.is_streaming = False
        self.lock = threading.Lock()
        self.frame_buffer: Optional[bytes] = None
        self.jpeg_quality = 80

        # Load config
        self.load_config()

    def load_config(self):
        """Load camera configuration from config.yaml"""
        try:
            with open(self.config_path, 'r') as f:
                config = yaml.safe_load(f)
                camera_config = config.get('camera', {})
                self.default_device = camera_config.get('default_device', '/dev/video0')
                self.default_width = camera_config.get('resolution', {}).get('width', 640)
                self.default_height = camera_config.get('resolution', {}).get('height', 480)
                self.default_fps = camera_config.get('fps', 30)
                self.jpeg_quality = camera_config.get('jpeg_quality', 80)
                auto_start = camera_config.get('auto_start', False)

                if auto_start and os.path.exists(self.default_device):
                    logger.info(f"Auto-starting camera on {self.default_device}")
                    self.start_camera(self.default_device)
        except Exception as e:
            logger.error(f"Error loading camera config: {e}")
            # Set defaults if config fails
            self.default_device = '/dev/video0'
            self.default_width = 640
            self.default_height = 480
            self.default_fps = 30
            self.jpeg_quality = 80

    def save_config(self, device: str):
        """Save current camera device to config.yaml"""
        try:
            with open(self.config_path, 'r') as f:
                config = yaml.safe_load(f)

            if 'camera' not in config:
                config['camera'] = {}

            config['camera']['default_device'] = device

            with open(self.config_path, 'w') as f:
                yaml.dump(config, f, default_flow_style=False)

            logger.info(f"Saved camera device {device} to config")
        except Exception as e:
            logger.error(f"Error saving camera config: {e}")

    def detect_cameras(self) -> List[Dict[str, str]]:
        """
        Detect available USB cameras on the system.
        Returns list of camera info dicts with 'device' and 'name' keys.
        """
        cameras = []

        # Method 1: Check /dev/video* devices
        video_devices = glob.glob('/dev/video*')

        for device_path in sorted(video_devices):
            # Try to get camera name from v4l2
            name = self._get_camera_name(device_path)
            cameras.append({
                'device': device_path,
                'name': name or os.path.basename(device_path)
            })

        logger.info(f"Detected {len(cameras)} camera(s): {cameras}")
        return cameras

    def _get_camera_name(self, device_path: str) -> Optional[str]:
        """Get camera name using v4l2-ctl if available."""
        try:
            import subprocess
            result = subprocess.run(
                ['v4l2-ctl', '--device', device_path, '--info'],
                capture_output=True,
                text=True,
                timeout=2
            )

            if result.returncode == 0:
                # Parse output to find card name
                for line in result.stdout.split('\n'):
                    if 'Card type' in line:
                        return line.split(':', 1)[1].strip()
        except (FileNotFoundError, subprocess.TimeoutExpired, Exception) as e:
            logger.debug(f"Could not get camera name for {device_path}: {e}")

        return None

    def start_camera(self, device_path: str, width: Optional[int] = None,
                    height: Optional[int] = None, fps: Optional[int] = None) -> bool:
        """
        Start camera capture on specified device.

        Args:
            device_path: Path to video device (e.g., '/dev/video0')
            width: Frame width (default from config)
            height: Frame height (default from config)
            fps: Frames per second (default from config)

        Returns:
            True if camera started successfully, False otherwise
        """
        with self.lock:
            # Stop existing camera if running
            if self.camera is not None:
                self.stop_camera()

            # Check if device exists
            if not os.path.exists(device_path):
                logger.error(f"Camera device {device_path} does not exist")
                return False

            try:
                # Open camera
                self.camera = cv2.VideoCapture(device_path)

                if not self.camera.isOpened():
                    logger.error(f"Failed to open camera {device_path}")
                    self.camera = None
                    return False

                # Set camera properties
                width = width or self.default_width
                height = height or self.default_height
                fps = fps or self.default_fps

                self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                self.camera.set(cv2.CAP_PROP_FPS, fps)

                # Verify settings (camera may not support exact values)
                actual_width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
                actual_fps = int(self.camera.get(cv2.CAP_PROP_FPS))

                logger.info(f"Camera started: {device_path} at {actual_width}x{actual_height} @ {actual_fps}fps")

                self.current_device = device_path
                self.is_streaming = True

                # Capture first frame
                self._update_frame()

                # Save to config
                self.save_config(device_path)

                return True

            except Exception as e:
                logger.error(f"Error starting camera {device_path}: {e}")
                if self.camera:
                    self.camera.release()
                    self.camera = None
                return False

    def stop_camera(self):
        """Stop camera capture and release resources."""
        with self.lock:
            if self.camera is not None:
                self.camera.release()
                self.camera = None
                self.is_streaming = False
                self.current_device = None
                self.frame_buffer = None
                logger.info("Camera stopped")

    def _update_frame(self) -> bool:
        """
        Capture a frame from camera and encode as JPEG.
        Should be called with lock held.

        Returns:
            True if frame captured successfully, False otherwise
        """
        if self.camera is None or not self.camera.isOpened():
            return False

        try:
            ret, frame = self.camera.read()
            if not ret:
                logger.warning("Failed to read frame from camera")
                return False

            # Encode frame as JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            ret, jpeg = cv2.imencode('.jpg', frame, encode_param)

            if not ret:
                logger.warning("Failed to encode frame as JPEG")
                return False

            self.frame_buffer = jpeg.tobytes()
            return True

        except Exception as e:
            logger.error(f"Error capturing frame: {e}")
            return False

    def get_frame(self) -> Optional[bytes]:
        """
        Get latest JPEG frame from camera.

        Returns:
            JPEG image bytes, or None if no frame available
        """
        with self.lock:
            if not self.is_streaming or self.camera is None:
                return None

            # Update frame buffer with fresh capture
            if self._update_frame():
                return self.frame_buffer

            return None

    def get_mjpeg_frame(self) -> bytes:
        """
        Get MJPEG multipart frame for streaming.

        Returns:
            Multipart MJPEG frame bytes with headers
        """
        frame = self.get_frame()

        if frame is None:
            # Return empty frame if camera not available
            return b''

        # MJPEG multipart format
        return (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    def get_status(self) -> Dict:
        """
        Get current camera status.

        Returns:
            Dict with camera status information
        """
        with self.lock:
            if not self.is_streaming or self.camera is None:
                return {
                    'streaming': False,
                    'device': None,
                    'width': None,
                    'height': None,
                    'fps': None
                }

            return {
                'streaming': True,
                'device': self.current_device,
                'width': int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH)),
                'height': int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                'fps': int(self.camera.get(cv2.CAP_PROP_FPS))
            }

    def get_frame_dimensions(self) -> Optional[Tuple[int, int]]:
        """Get current frame width and height."""
        with self.lock:
            if self.camera is None:
                return None
            return (
                int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH)),
                int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
            )


# Global camera manager instance
_camera_manager: Optional[CameraManager] = None


def get_camera_manager() -> CameraManager:
    """Get global camera manager instance (singleton)."""
    global _camera_manager
    if _camera_manager is None:
        _camera_manager = CameraManager(config_path="/home/jacob/parol6/commander/config.yaml")
    return _camera_manager
