# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a PAROL6 robotic arm control system with a Next.js-based timeline editor frontend and a Python FastAPI/robotics backend. The system features dual motion modes (joint space + cartesian space), browser-based IK, real-time 3D visualization, and WebSocket communication between frontend and backend.

## Architecture

### Three-Process System (PM2 Orchestrated)

The system runs three independent processes managed by PM2:

1. **parol-commander** (`backend/headless_commander.py`):
   - Core robot control loop running at 100Hz (0.01s interval)
   - Direct serial communication with PAROL6 hardware via `/dev/ttyACM0`
   - Listens on UDP port 5001 for commands, sends acknowledgments on port 5002
   - Implements motion planning, IK solving, trajectory generation
   - Safety features: E-stop handling, joint limit enforcement
   - Uses roboticstoolbox-python for kinematics

2. **parol-api** (`backend/fastapi_server.py`):
   - FastAPI server on port 3001 (HTTP/WebSocket bridge)
   - Translates RESTful HTTP requests → UDP commands to headless_commander
   - WebSocket server for streaming robot status (joints, pose, IO, logs)
   - IK solver endpoint using same kinematics as headless_commander
   - CORS-enabled for frontend communication

3. **parol-nextjs** (`frontend/`):
   - Next.js 14 dev server on port 3000
   - Timeline-based motion editor with keyframe recording
   - URDF-based 3D robot visualization (Three.js + React Three Fiber)
   - Browser-based IK solver for offline editing
   - Dual motion modes: joint space (J1-J6) and cartesian space (X,Y,Z,RX,RY,RZ)
   - WebSocket client for real-time robot state

### Communication Flow

```
Frontend (port 3000)
  ↓ HTTP REST / WebSocket
FastAPI Server (port 3001)
  ↓ UDP (port 5001 commands, 5002 acks)
Headless Commander
  ↓ Serial (3Mbaud)
PAROL6 Robot Hardware
```

### Key Backend Files

- `config.yaml`: Central configuration (ports, robot settings, UI defaults)
- `PAROL6_ROBOT.py`: Robot DH parameters and URDF-based kinematics model
- `smooth_motion.py`: Adaptive IK solver, spline motion, circular interpolation
- `robot_api.py`: Zero-overhead command API with optional acknowledgment tracking
- `models.py`: Pydantic models for API validation
- `websocket_manager.py`: WebSocket connection management
- `logging_handler.py`: Centralized logging with WebSocket streaming
- `numpy_patch.py`: Numpy 2.0+ compatibility shim

### Key Frontend Files

- `app/lib/store.ts`: Zustand state management (timeline, joints, playback)
- `app/lib/kinematics.ts`: Browser IK/FK solver (damped least squares)
- `app/lib/api.ts`: Backend API client for IK calls
- `app/components/RobotViewer.tsx`: URDF-based 3D visualization
- `app/components/Timeline.tsx`: Keyframe editor using animation-timeline-js
- `app/components/CartesianSliders.tsx`: Cartesian control + IK UI
- `app/hooks/useRobotWebSocket.ts`: Real-time WebSocket connection

## Common Development Commands

### Running the Full System

```bash
# Start all three processes via PM2 (from root directory)
pm2 start ecosystem.config.js

# View status
pm2 status

# View logs (all processes)
pm2 logs

# View specific process logs
pm2 logs parol-commander
pm2 logs parol-api
pm2 logs parol-nextjs

# Restart all
pm2 restart all

# Stop all
pm2 stop all

# Delete all processes from PM2
pm2 delete all
```

### Running Individual Processes (Development)

```bash
# Backend commander (from backend/)
cd backend
python3 headless_commander.py

# Backend API server (from backend/)
cd backend
python3 fastapi_server.py

# Frontend dev server (from frontend/)
cd frontend
npm run dev
# OR explicitly specify port (see IMPORTANT note below)
node node_modules/next/dist/bin/next dev -p 3000
```

**IMPORTANT**: The frontend MUST run on port 3000 and the API MUST run on port 3001. These ports are specified in `config.yaml` and hardcoded in various places. Do not change ports unless explicitly instructed.

### Python Backend Setup

```bash
cd backend

# Install dependencies (first time)
pip install -r requirements.txt

# Check Python version (requires numpy==1.23.4 compatibility)
python3 --version

# Test serial connection
python3 -c "import serial; print(serial.Serial('/dev/ttyACM0', 3000000))"
```

### Frontend Development

```bash
cd frontend

# Install dependencies (first time)
npm install

# Development with hot reload
npm run dev

# Production build
npm run build
npm start

# Linting
npm run lint

# Type checking
npx tsc --noEmit
```

### Configuration

- Edit `backend/config.yaml` to change:
  - API ports and CORS origins
  - Robot serial port and baud rate
  - WebSocket topics and rates
  - UI defaults (speed, timeline duration, saved positions)
  - Joint colors and TCP offset

## Motion Modes

### Joint Space Mode
- Direct control of 6 joint angles (J1-J6)
- Keyframes store per-joint values
- No IK required
- Faster execution, predictable joint trajectories
- Robot path may be curved in 3D space

### Cartesian Space Mode
- Control TCP position (X,Y,Z) and orientation (RX,RY,RZ)
- Keyframes store per-axis cartesian values
- IK solved during playback for each interpolated point
- Straight-line motion in task space
- May hit singularities or joint limits

## IK Solver Architecture

Two parallel IK implementations ensure consistency:

1. **Backend IK** (`backend/smooth_motion.py`):
   - Used by actual robot controller
   - Adaptive tolerance with subdivision for precision
   - Numerical jacobian-based solver
   - Joint limit aware

2. **Frontend IK** (`frontend/app/lib/kinematics.ts`):
   - Browser-based for offline editing
   - Damped least squares algorithm
   - Configurable axis masking (position-only, full 6-DOF)
   - Real-time visualization

Frontend can optionally call backend IK via `/api/ik` REST endpoint.

## Robot Coordinate System

- **Base frame**: J1 rotation axis (vertical)
- **TCP (Tool Center Point)**: Configurable offset from J6 flange (default: [47, 0, -62] mm)
- **Joint limits**: See `config.yaml` or frontend `constants.ts`
- **DH parameters**: Defined in `backend/PAROL6_ROBOT.py`

## WebSocket Topics

Frontend subscribes to these topics (port 3001):
- `status`: Robot state, connection status, E-stop
- `joints`: Real-time joint angles [J1-J6]
- `pose`: TCP cartesian pose [X,Y,Z,RX,RY,RZ]
- `io`: Digital I/O state
- `gripper`: Gripper status
- `logs`: Backend log messages

## Testing

### Backend Tests
```bash
cd backend
pytest                    # Run all tests
pytest test_kinematics.py # Specific test file
pytest -v                 # Verbose output
```

### API Manual Testing
```bash
# Health check
curl http://localhost:3001/health

# Get robot status
curl http://localhost:3001/api/status

# Move joints (example)
curl -X POST http://localhost:3001/api/move/joints \
  -H "Content-Type: application/json" \
  -d '{"angles": [0, -45, 90, 0, 45, 0], "speed_percentage": 50}'

# Test IK endpoint
curl -X POST http://localhost:3001/api/ik \
  -H "Content-Type: application/json" \
  -d '{"target_pose": [200, 0, 300, 0, 0, 0], "current_joints": [0,0,0,0,0,0]}'
```

## Troubleshooting

### Serial Port Issues
- Check device permissions: `ls -l /dev/ttyACM0`
- Add user to dialout group: `sudo usermod -aG dialout $USER` (requires logout)
- Verify port in config.yaml matches actual device

### Port Conflicts
- Frontend must be on 3000, API must be on 3001
- Check `config.yaml` CORS origins if changing ports
- Kill conflicting processes: `lsof -ti:3000 | xargs kill -9`

### PM2 Issues
- Clear logs: `pm2 flush`
- Reset PM2: `pm2 kill && pm2 start ecosystem.config.js`
- View detailed errors: `pm2 logs --err`

### URDF Not Loading
- Verify `/frontend/public/urdf/PAROL6.urdf` exists
- Check STL meshes in `/frontend/public/urdf/meshes/`
- Browser console shows urdf-loader errors

### WebSocket Connection Failed
- Ensure FastAPI server is running on port 3001
- Check browser console for connection errors
- Verify CORS origins in `config.yaml` include `http://localhost:3000`

## Safety Notes

- E-stop can be triggered by pressing 'e' key when headless_commander has focus
- Auto-homing on startup is configurable in `config.yaml` (`robot.auto_home_on_startup`)
- Joint limits are enforced in both frontend and backend
- Serial timeout is set to 0 (non-blocking) in config
