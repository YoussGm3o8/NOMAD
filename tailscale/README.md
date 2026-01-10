# NOMAD Tailscale Integration

This folder contains all Tailscale VPN configuration and code for secure 4G/LTE communication between the Jetson Orin Nano (drone) and Ground Station.

## 📂 Folder Structure

```
tailscale/
├── README.md           # This file
├── SETUP.md            # Installation & configuration guide
├── TASK.md             # Development task specification
├── scripts/
│   ├── setup.sh        # Automated Jetson setup
│   └── watchdog.sh     # Connection watchdog service
├── config/
│   └── tailscale-watchdog.service  # Systemd service file
└── src/
    ├── tailscale_manager.py    # Python Tailscale manager
    └── network_monitor.py      # Network/4G monitoring
```

## 🔗 Quick Links

- [Setup Guide](SETUP.md) - How to install and configure Tailscale
- [Development Task](TASK.md) - Task spec for implementing the system

## 🎯 Purpose

Tailscale provides:
- **Secure VPN tunnel** over 4G/LTE for beyond-WiFi-range operation
- **MAVLink telemetry** streaming to Mission Planner
- **HTTP API access** to Jetson Edge Core
- **SSH access** for remote debugging
- **RTSP video streaming** from ZED camera

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                       Tailscale Cloud                        │
│                    (Coordination Server)                     │
└─────────────────────────────────────────────────────────────┘
                           │
           ┌───────────────┴───────────────┐
           │                               │
    ┌──────▼──────┐               ┌───────▼──────┐
    │   Jetson    │               │    Ground    │
    │ Orin Nano   │◄─────────────►│   Station    │
    │ (on drone)  │  Encrypted    │   (Laptop)   │
    │             │  WireGuard    │              │
    │ 4G/LTE USB  │    Tunnel     │  WiFi/LTE    │
    └─────────────┘               └──────────────┘
    100.x.x.x                     100.y.y.y
```

## 📡 Ports & Endpoints

| Service | Port | Protocol | Description |
|---------|------|----------|-------------|
| MAVLink | 14550 | UDP | Telemetry to Mission Planner |
| HTTP API | 8000 | TCP | Edge Core REST API |
| SSH | 22 | TCP | Remote terminal |
| RTSP Primary | 8554 | TCP | ZED left camera stream |
| RTSP Secondary | 8554 | TCP | ZED right camera stream |

## 🚀 Quick Start

```bash
# On Jetson
cd tailscale/scripts
sudo ./setup.sh --authkey <YOUR_KEY>

# Verify
tailscale status
tailscale ip -4
```

## 📋 Status

| Component | Status |
|-----------|--------|
| Setup Documentation | ✅ Complete |
| MAVLink Router Config | ✅ Complete |
| Python Manager | ⏳ Pending |
| Network Monitor | ⏳ Pending |
| Watchdog Script | ⏳ Pending |
| API Endpoints | ⏳ Pending |
| Mission Planner UI | ⏳ Pending |

See [TASK.md](TASK.md) for development specifications.
