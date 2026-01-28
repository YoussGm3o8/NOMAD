# NOMAD - Networked Operations for MAD

**McGill Aerial Design - AEAC 2026 Competition System**

Drone system for two distinct competition tasks:

| Task | Environment | Flight Control | Positioning | Jetson Role |
|------|-------------|----------------|-------------|-------------|
| **Task 1** (Outdoor Recon) | Outdoor | RC Pilot (ELRS) | GPS/RTK | Video/Imaging only |
| **Task 2** (Indoor Extinguish) | Indoor | Autonomous (Nav2) | ZED VIO | Full navigation |

---

## Task Overview

### Task 1: Outdoor Reconnaissance
- **Traditional RC pilot control** via ELRS directly to ArduPilot
- Jetson provides video streaming and target imaging only
- GPS/RTK positioning for outdoor operation
- No autonomous navigation

### Task 2: Indoor Fire Extinguishing  
- **Jetson-centric autonomous navigation** (Nav2/Nvblox)
- ArduPilot in GUIDED mode as flight controller only
- ZED 2i Visual-Inertial Odometry for indoor positioning
- YOLO target detection
- **WASD controls** available for human intervention over LTE if needed
- See [JETSON_NAV_ARCHITECTURE.md](docs/JETSON_NAV_ARCHITECTURE.md)

---

## System Architecture

### Task 1: Outdoor (RC Pilot Control)
```
+---------------------------------------------------------------+
|                    GROUND STATION                              |
|  Mission Planner (telemetry display)                          |
|  ELRS TX (RC control)                                          |
+---------------------------------------------------------------+
                     |
              ELRS Radio Link
                     |
+---------------------------------------------------------------+
|                     DRONE                                      |
|  +-- Cube Orange (ArduPilot) <-- ELRS RX                      |
|  |   Standard flight modes (pilot control)                    |
|  |                                                             |
|  +-- Jetson Orin Nano (video only)                            |
|      +-- ZED camera streaming                                 |
|      +-- Tailscale (video/status to ground)                   |
+---------------------------------------------------------------+
```

### Task 2: Indoor (Jetson Autonomous)
```
+---------------------------------------------------------------+
|                    GROUND STATION                              |
|  Mission Planner + NOMAD Plugin                                |
|  +-- Status Display (Health, VIO, Nav)                        |
|  +-- WASD Controls (backup intervention over LTE)             |
|                    |                                           |
|            Tailscale VPN (4G/LTE)                             |
+---------------------------------------------------------------+
                     |
+---------------------------------------------------------------+
|                    JETSON ORIN NANO                            |
|  +-- Edge Core (Python 3.13)                                  |
|  |   +-- NavController (velocity commands to AP)              |
|  |   +-- VIO Pipeline (position feedback to AP)               |
|  |   +-- MavlinkService (GUIDED mode control)                 |
|  |                                                             |
|  +-- Isaac ROS (Docker)                                       |
|      +-- Nav2 (autonomous path planning)                      |
|      +-- Nvblox (3D obstacle mapping)                         |
|      +-- ros_http_bridge (ROS -> Edge Core)                   |
+---------------------------------------------------------------+
                     |
            mavlink-router
                     |
+---------------------------------------------------------------+
|               CUBE ORANGE (ArduPilot)                          |
|  GUIDED mode - flight controller only                          |
|  +-- Attitude control                                          |
|  +-- Motor mixing                                              |
|  +-- Failsafe logic                                            |
+---------------------------------------------------------------+
```

### Task 1 vs Task 2 Differences
| Component | Task 1 (Outdoor) | Task 2 (Indoor) |
|-----------|------------------|-----------------|
| Flight Control | RC Pilot via ELRS | Jetson NavController |
| ArduPilot Mode | Normal (pilot control) | GUIDED (velocity commands) |
| Position Source | GPS/RTK | ZED VIO |
| Isaac ROS | Not used | Active (Nvblox, Nav2) |
| Jetson Role | Video streaming only | Full autonomous navigation |
| WASD Controls | Not used | Backup human intervention |

---

## Repository Structure

```
NOMAD/
├── docs/                   # Documentation
│   ├── architecture.md     # System design
│   └── PRD.md              # Product requirements
│
├── edge_core/              # Jetson software (Task 2 autonomy + Task 1 imaging support)
│   ├── main.py             # Entry point
│   ├── api.py              # REST API endpoints
│   ├── state.py            # State manager
│   ├── mavlink_interface.py  # Flight controller comms
│   ├── time_manager.py     # Time synchronization
│   ├── geospatial.py       # GPS calculations
│   └── models.py           # Data models
│
├── tailscale/              # VPN configuration (Task 2)
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
│       ├── NOMADPlugin.cs         # Plugin entry point
│       ├── NOMADFullPage.cs       # Full-page control interface
│       ├── NOMADControlPanel.cs   # Quick access panel
│       ├── EmbeddedVideoPlayer.cs # Built-in video streaming
│       ├── JetsonTerminalControl.cs # Remote terminal
│       └── ...
│
├── config/                 # Configuration files
│   ├── params/             # ArduPilot parameter files
│   └── env/                # Environment configs
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

### Task 1 Setup (Jetson camera)
```bash
# Ground station + Jetson camera (imaging only)
1. Connect ELRS Gemini TX to computer
2. Open Mission Planner
3. Connect to drone via ELRS
4. Configure RTK/NTRIP for corrections
5. Use NOMAD → Open Full Control Page for capture controls
6. Fly with GPS waypoints
```

### Task 2 Setup (With Jetson)
```bash
# On Jetson
cd NOMAD
pip install -r edge_core/requirements.txt
sudo tailscale/scripts/setup.sh --authkey <KEY>
python -m edge_core.main --host 0.0.0.0 --port 8000

# On Ground Station
1. Connect via Tailscale IP
2. Open Mission Planner with NOMAD plugin
3. Use NOMAD menu → Open Full Control Page
4. Check Jetson health in Health tab
5. Use embedded video or terminal as needed
```

---

## 🎮 Mission Planner Plugin Features

### Full Control Page
- **Dashboard**: System overview, quick actions, connection status
- **Task 1 Tab**: GPS capture controls, waypoint management
- **Task 2 Tab**: VIO controls, WASD nudge, exclusion map
- **Video Tab**: Embedded RTSP streaming (no VLC needed)
- **Terminal Tab**: Remote command execution on Jetson
- **Health Tab**: CPU/GPU temps, memory, disk, network status

### Quick Access
- Right-click FlightData map → NOMAD Full Control
- Menu bar → NOMAD → Open Full Control Page
- Keyboard shortcut support (configurable)

---

## 📡 Communication Links

| Link | Task 1 | Task 2 |
|------|--------|--------|
| **ELRS 2.4GHz** | Primary control | Backup control |
| **ELRS 900MHz** | Extended range | Backup control |
| **4G/LTE** | Not used | Primary data |
| **Tailscale** | Not used | API + Video |

---

## 📋 Status

| Component | Task 1 | Task 2 |
|-----------|--------|--------|
| ArduPilot Integration | ✅ Ready | ✅ Ready |
| ELRS Telemetry | ✅ Ready | ✅ Ready |
| Edge Core API | ✅ Ready | ✅ Ready |
| Tailscale VPN | N/A | ✅ Ready |
| ZED 2i Camera | ✅ Ready | ✅ Ready |
| YOLO Detection | N/A | ⏳ In Progress |
| Mission Planner Plugin | ✅ Ready | ✅ Ready |
| Embedded Video | ✅ Ready | ✅ Ready |
| Remote Terminal | ✅ Ready | ✅ Ready |
| Health Monitoring | ✅ Ready | ✅ Ready |
