# PAROL6 Project Primer

## Quick Start
This is a PAROL6 robotic arm control system with 3 processes managed by PM2:
1. **Backend Commander** (`backend/headless_commander.py`) - Robot control loop, serial comms
2. **Backend API** (`backend/fastapi_server.py`) - FastAPI server on port 3001
3. **Frontend** (`frontend/`) - Next.js timeline editor on port 3000

**CRITICAL**: Frontend MUST be on port 3000, API MUST be on port 3001.

## Architecture
```
Frontend (3000) ←→ FastAPI (3001) ←→ Commander (UDP 5001/5002) ←→ Robot (Serial)
```

## Key Locations
- **Config**: `backend/config.yaml` - All ports, settings, robot params
- **Backend**: `backend/` - Python robotics, kinematics, API
- **Frontend**: `frontend/app/` - React components, hooks, state management
- **Process Manager**: `ecosystem.config.js` - PM2 configuration

## Common Commands
```bash
pm2 start ecosystem.config.js  # Start all
pm2 logs                       # View logs
pm2 restart all                # Restart
cd frontend && npm run dev     # Frontend only
cd backend && python3 fastapi_server.py  # API only
```

## Architecture Details
See `CLAUDE.md` for comprehensive documentation on:
- Process communication flow
- Motion modes (joint/cartesian)
- IK solver architecture
- WebSocket topics
- Troubleshooting guides
