# Edge Core

Onboard services for the Jetson Orin Nano (Python 3.13).

## Core Modules

| Module | Purpose |
|--------|---------|
| `main.py` | Entry point, runs FastAPI server and MAVLink service |
| `api.py` | REST API endpoints (status, health, WebSocket) |
| `state.py` | Thread-safe state manager singleton |
| `models.py` | Pydantic models (SystemState, DetectionInfo) |
| `mavlink_interface.py` | MAVLink communication with flight controller |
| `time_manager.py` | NTP/GPS time synchronization |
| `geospatial.py` | GPS calculations, coordinate conversions |
| `ipc.py` | ZeroMQ inter-process communication |
| `logging_service.py` | Mission event logging |

## Quick Start

```bash
# Install dependencies
pip install -r requirements.txt

# Run the server
python -m edge_core.main --host 0.0.0.0 --port 8000
```

## API Endpoints

- `GET /` - Service info
- `GET /status` - Full system state
- `GET /health` - Health check
- `WS /ws/state` - Real-time state WebSocket
