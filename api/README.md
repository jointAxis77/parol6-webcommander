# PAROL6 API Server (Refactored)

This directory contains the FastAPI server that bridges the frontend to the refactored commander system.

## Architecture

```
Frontend (port 3000)
  ↓ HTTP REST / WebSocket
API Server (port 3001) ← YOU ARE HERE
  ↓ Imports from /commander for IK/FK
  ↓ UDP (5001/5002) for robot commands
Commander (100Hz control loop)
  ↓ Serial (3Mbaud)
PAROL6 Hardware
```

## Files

- **fastapi_server.py** - Main FastAPI application with REST endpoints and WebSocket
- **websocket_manager.py** - WebSocket connection management
- **camera_manager.py** - USB camera streaming

## Module Dependencies

All files import from `/commander` directory:
- `robot_api.py` - UDP client for sending commands to commander
- `models.py` - Pydantic validation models
- `logging_handler.py` - Centralized logging
- `PAROL6_ROBOT.py` - Robot kinematics for IK/FK endpoints
- `smooth_motion.py` - IK solver
- `numpy_patch.py` - Numpy 2.0 compatibility
- `config.yaml` - Shared configuration

## Running

### Via PM2 (Recommended)

**Start new refactored system:**
```bash
pm2 start parol-commander-new parol-api-new parol-nextjs
```

**Check status:**
```bash
pm2 status
pm2 logs parol-api-new
```

**Stop new system:**
```bash
pm2 stop parol-commander-new parol-api-new
```

### Direct Python (Development)

```bash
cd /home/jacob/parol6/api
python3 fastapi_server.py
```

Make sure commander is running first!

## Switching Between Old/New Systems

**Old system (original backend):**
```bash
pm2 stop parol-commander-new parol-api-new
pm2 start parol-commander parol-api parol-nextjs
```

**New system (refactored):**
```bash
pm2 stop parol-commander parol-api
pm2 start parol-commander-new parol-api-new parol-nextjs
```

## Testing

**Health check:**
```bash
curl http://localhost:3001/health
```

**Test IK endpoint:**
```bash
curl -X POST http://localhost:3001/api/ik \
  -H "Content-Type: application/json" \
  -d '{"target_pose": [200, 0, 300, 0, 0, 0], "current_joints": [0,0,0,0,0,0]}'
```

**Get robot status:**
```bash
curl http://localhost:3001/api/robot/status
```

## Key Features

- **IK/FK Endpoints** - Solved locally without blocking commander's control loop
- **WebSocket Streaming** - Real-time robot state (joints, pose, IO, logs)
- **Camera Streaming** - MJPEG video from USB camera
- **Command Bridge** - Translates REST → UDP commands to commander

## Configuration

Configuration is loaded from `/home/jacob/parol6/commander/config.yaml`.

API server listens on port 3001 (configured in config.yaml).

## Notes

- PYTHONPATH includes `/commander` for module imports
- All 3 files have `sys.path.insert(0, '/home/jacob/parol6/commander')` at the top
- Camera config path explicitly set to `/commander/config.yaml`
- This allows side-by-side testing with the old backend system
