# NOMAD System Architecture

## Overview

NOMAD (Networked Operations for MAD) is designed for two distinct competition tasks:

| Task | Configuration | Navigation | Edge Compute |
|------|---------------|------------|--------------|
| **Task 1** (Outdoor Recon) | No Jetson | GPS/RTK | Not required |
| **Task 2** (Indoor Extinguish) | With Jetson | ZED VIO | Full stack |

## System Domains

### Domain A: Transport Layer
- **MAVLink Router**: Fan-out telemetry to multiple endpoints
- **Link Management**: 4G/LTE + ELRS failover
- **Tailscale VPN**: Secure remote access

### Domain B: Edge Core (Task 2 Only)
- **FastAPI Orchestrator**: REST API + state management
- **ZED VIO Pipeline**: Visual-Inertial Odometry at 30Hz
- **Vision Engine**: YOLO detection + target tracking
- **Gimbal Controller**: PID-based visual servoing

### Domain C: Mission Planner Plugin
- **Full Control Page**: Tabbed interface for all operations
- **Embedded Video**: RTSP streaming without external VLC
- **Remote Terminal**: Execute commands on Jetson
- **Health Dashboard**: Real-time system monitoring
- **Task Controls**: Task 1 capture, Task 2 exclusion map

## Process Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    JETSON ORIN NANO                         │
│  ┌────────────────┐  ┌────────────────┐  ┌──────────────┐  │
│  │  edge_core/    │  │  edge_core/    │  │  MediaMTX    │  │
│  │  orchestrator  │  │  vision        │  │  RTSP Server │  │
│  │  (FastAPI)     │  │  (ZED+YOLO)    │  │  (8554)      │  │
│  └───────┬────────┘  └───────┬────────┘  └──────────────┘  │
│          │                   │                              │
│          └───────────────────┼──────────────────────────────│
│                              ▼                              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │                  mavlink-router                       │  │
│  │  UART(FC) ──► UDP(Orch) + UDP(Vision) + UDP(GCS)    │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

## Data Flow

### Task 1 (GPS-based)
```
FC (GPS) ──► ELRS TX ──► ELRS RX ──► Mission Planner
                                          │
                                          ▼
                                    NOMAD Plugin
                                    (Capture control)
```

### Task 2 (VIO-based)
```
ZED Camera ──► VIO Process ──► VISION_POSITION_ESTIMATE ──► FC EKF
    │
    ▼
YOLO Detection ──► Gimbal Commands ──► Targeting
    │
    ▼
RTSP Stream ──► MediaMTX ──► Mission Planner Video Tab
```

## Key Design Principles

1. **Separation of Concerns**: Each module has a single responsibility
2. **Graceful Degradation**: ELRS backup if 4G/Tailscale fails
3. **Task Isolation**: Task 1 and Task 2 configs are completely independent
4. **Real-time Priority**: VIO process pinned to high-priority CPU cores
5. **Watchdog Protection**: Auto-restart hung processes without severing MAVLink

## Directory Structure

```
NOMAD/
├── edge_core/           # Jetson software (Task 2)
│   ├── api.py           # FastAPI endpoints
│   ├── state.py         # State machine
│   ├── mavlink_interface.py
│   └── zed_camera.py    # ZED SDK integration
├── mission_planner/     # C# Plugin
│   └── src/
│       ├── NOMADPlugin.cs
│       ├── NOMADFullPage.cs
│       ├── EmbeddedVideoPlayer.cs
│       └── JetsonTerminalControl.cs
├── transport/           # MAVLink routing
├── config/              # Parameters and profiles
└── tailscale/           # VPN setup
```

Keep each module small: one responsibility per package, tests in `tests/` mirroring package paths.

