# NOMAD - Networked Operations for MAD

**McGill Aerial Design - AEAC 2026 Competition System**

🚁 Autonomous drone system with:
- **Platform:** Tricopter Tiltrotor
- **Computer:** NVIDIA Jetson Orin Nano  
- **Vision:** ZED 2i Stereo Camera  
- **Flight Controller:** Cube Orange (ArduPilot)  
- **Communication:** 4G/LTE + Tailscale VPN

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    GROUND STATION                               │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │         Mission Planner + NOMAD Plugin (C#)              │  │
│  │  • Health Monitor    • Telemetry     • Settings          │  │
│  └──────────────────────────────────────────────────────────┘  │
│                            ↕ MAVLink + HTTP                     │
│                    ┌────────────────────┐                       │
│                    │   Tailscale VPN    │                       │
│                    │   100.x.x.x/16     │                       │
│                    └────────────────────┘                       │
└─────────────────────────────────────────────────────────────────┘
                              ↕ 4G/LTE
┌─────────────────────────────────────────────────────────────────┐
│                     DRONE                                       │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │              EDGE CORE (Jetson Orin Nano)                │  │
│  │  • FastAPI Server     • MAVLink Interface                │  │
│  │  • State Manager      • Time Sync Service                │  │
│  └──────────────────────────────────────────────────────────┘  │
│                            ↕ MAVLink Router                     │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │         Cube Orange Flight Controller (ArduPilot)        │  │
│  └──────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📁 Repository Structure

```
NOMAD/
├── docs/                   # Documentation
│   ├── architecture.md     # System design
│   └── PRD.md              # Product requirements
│
├── config/                 # Configuration files
│   └── landmarks.json      # GPS waypoints
│
├── edge_core/              # Jetson software (Python 3.13)
│   ├── main.py             # Entry point
│   ├── api.py              # REST API endpoints
│   ├── state.py            # State manager
│   ├── mavlink_interface.py  # Flight controller comms
│   ├── time_manager.py     # Time synchronization
│   ├── geospatial.py       # GPS calculations
│   ├── ipc.py              # ZMQ IPC
│   ├── logging_service.py  # Mission logging
│   └── models.py           # Data models
│
├── tailscale/              # VPN configuration
│   ├── SETUP.md            # Installation guide
│   ├── src/                # Python managers
│   ├── scripts/            # Setup/watchdog scripts
│   └── config/             # Systemd services
│
├── transport/              # MAVLink routing
│   └── mavlink_router/
│       └── main.conf       # Router config
│
├── mission_planner/        # Ground Control Plugin (C#)
│   └── src/
│       ├── NOMADPlugin.cs
│       └── ...
│
├── infra/                  # Deployment configs
│   ├── Dockerfile
│   └── nomad.service
│
└── scripts/                # Dev scripts
    └── run_dev.sh
```

---

## 🚀 Quick Start

### 1. Jetson Setup

```bash
# Clone repo
git clone https://github.com/mcgill-aerial-design/NOMAD.git
cd NOMAD

# Install Python dependencies
pip install -r edge_core/requirements.txt

# Run Edge Core
python -m edge_core.main --host 0.0.0.0 --port 8000
```

### 2. Tailscale VPN Setup

```bash
# On Jetson
cd tailscale/scripts
sudo ./setup.sh --authkey <YOUR_KEY>

# Verify
tailscale status
tailscale ip -4
```

### 3. Configure MAVLink Router

```bash
sudo cp transport/mavlink_router/main.conf /etc/mavlink-router/main.conf
# Edit to set Ground Station Tailscale IP
sudo systemctl restart mavlink-router
```

---

## 📡 API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Service info |
| `/status` | GET | Full system state |
| `/health` | GET | Health check |
| `/ws/state` | WS | Real-time state |

---

## 🔧 Development

```bash
# Run in development mode
python -m edge_core.main --log-level debug
```

---

## 📋 Status

| Component | Status |
|-----------|--------|
| Edge Core API | ✅ Ready |
| MAVLink Interface | ✅ Ready |
| Time Sync | ✅ Ready |
| Tailscale VPN | ✅ Ready |
| Mission Planner Plugin | ⏳ In Progress |
