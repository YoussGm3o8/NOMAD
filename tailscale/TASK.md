# Task: Tailscale Integration System for NOMAD

**Status:** Ryan Plouffe
**Created:** January 9, 2026

---

## 📋 Task Overview

Implement the Tailscale VPN system for secure 4G/LTE communication between the Jetson Orin Nano (drone) and Ground Station (laptop). This enables remote MAVLink telemetry, API access, SSH, and video streaming when the drone is beyond WiFi range.

---

## 📂 Files in This Folder

| File | Purpose | Status |
|------|---------|--------|
| [README.md](README.md) | Folder overview | ✅ Complete |
| [SETUP.md](SETUP.md) | Installation & configuration guide | ✅ Complete |
| [TASK.md](TASK.md) | This task document | ✅ Complete |
| [src/tailscale_manager.py](src/tailscale_manager.py) | Python Tailscale manager | ✅ Complete |
| [src/network_monitor.py](src/network_monitor.py) | Network/4G monitoring | ✅ Complete |
| [src/__init__.py](src/__init__.py) | Package exports | ✅ Complete |
| [scripts/setup.sh](scripts/setup.sh) | Automated Jetson setup | ✅ Complete |
| [scripts/watchdog.sh](scripts/watchdog.sh) | Connection watchdog | ✅ Complete |
| [config/tailscale-watchdog.service](config/tailscale-watchdog.service) | Systemd service | ✅ Complete |

**Related Files (Outside This Folder):**

| File | Purpose | Status |
|------|---------|--------|
| [../transport/mavlink_router/main.conf](../transport/mavlink_router/main.conf) | MAVLink routing config | ✅ Complete |
| [../mission_planner/src/DualLinkSender.cs](../mission_planner/src/DualLinkSender.cs) | Dual HTTP/MAVLink sender | ✅ Complete |

---

## 🎯 Remaining Work

### 1. Add API Endpoints to `edge_core/api.py`

**Purpose:** Expose Tailscale and network status via REST API.

**New Endpoints Required:**

```python
# GET /network/status
# Returns current network and Tailscale status
{
    "tailscale": {
        "status": "connected",
        "ip": "100.100.10.5",
        "hostname": "nomad-jetson",
        "peer_count": 2,
        "latency_ms": 45.2
    },
    "modem": {
        "connected": true,
        "signal_strength_dbm": -85,
        "signal_quality": "good",
        "carrier": "AT&T",
        "technology": "LTE"
    },
    "internet_reachable": true,
    "gcs_reachable": true
}

# POST /network/reconnect
# Force Tailscale reconnection
{
    "success": true,
    "message": "Reconnection initiated"
}

# GET /network/ping/{host}
# Ping arbitrary host and return latency
{
    "host": "100.100.10.1",
    "latency_ms": 52.3,
    "packets_sent": 3,
    "packets_received": 3
}
```

**Integration Code:**

```python
# In edge_core/main.py - add to imports
import sys
sys.path.insert(0, str(Path(__file__).parent.parent / "tailscale" / "src"))
from tailscale_manager import TailscaleManager, init_tailscale_manager
from network_monitor import NetworkMonitor, init_network_monitor

# In run() function - initialize managers
tailscale_manager = init_tailscale_manager(
    hostname="nomad-jetson",
    on_status_change=lambda info: logger.info(f"Tailscale: {info.status.value}")
)
await tailscale_manager.start()

network_monitor = init_network_monitor(gcs_tailscale_ip="100.100.10.1")
await network_monitor.start()
```

---

### 2. Update Mission Planner Plugin

**File:** `../mission_planner/src/JetsonHealthTab.cs`

**Add:** Network status panel showing:
- Tailscale connection status (green/yellow/red indicator)
- Current Tailscale IP
- 4G/LTE signal strength bar
- Latency to Jetson
- Auto-refresh every 5 seconds

---

## ✅ Completed Implementation

### `src/tailscale_manager.py`

**Features Implemented:**
- `TailscaleManager` class with async monitoring
- Parses `tailscale status --json` output
- Auto-reconnect on disconnection
- Status change callbacks
- Peer tracking
- Module-level singleton: `get_tailscale_manager()`, `init_tailscale_manager()`

**Classes:**
- `TailscaleStatus` - Enum: CONNECTED, CONNECTING, DISCONNECTED, NEEDS_AUTH, NOT_INSTALLED, ERROR
- `TailscalePeer` - Dataclass for peer info
- `TailscaleInfo` - Dataclass with full status info
- `TailscaleManager` - Main monitoring class

---

### `src/network_monitor.py`

**Features Implemented:**
- `NetworkMonitor` class with async monitoring
- 4G/LTE modem status via ModemManager (`mmcli`)
- Signal strength (RSRP dBm) to quality/percentage conversion
- Internet/GCS connectivity checks via ping
- Module-level singleton: `get_network_monitor()`, `init_network_monitor()`

**Classes:**
- `ConnectionType` - Enum: LTE_4G, LTE_5G, WIFI, ETHERNET, UNKNOWN, NONE
- `SignalQuality` - Enum: EXCELLENT, GOOD, FAIR, POOR, NO_SIGNAL
- `ModemStatus` - Dataclass for modem info
- `NetworkStatus` - Dataclass with full network status
- `NetworkMonitor` - Main monitoring class

---

### `scripts/setup.sh`

**Features Implemented:**
- Root check and internet connectivity verification
- Tailscale download and installation
- Service configuration and auto-start
- Interactive or auth-key authentication
- UFW firewall configuration for Tailscale
- Watchdog service installation
- Post-install verification and next steps

**Usage:**
```bash
sudo ./setup.sh                      # Interactive auth
sudo ./setup.sh --authkey <KEY>      # Auth key
sudo ./setup.sh --hostname custom    # Custom hostname
```

---

### `scripts/watchdog.sh`

**Features Implemented:**
- Connection status monitoring loop
- Auto-restart on disconnection
- Multiple retry attempts with delays
- Systemd journal logging
- SIGTERM/SIGINT handling
- Status change detection

**Configuration:**
- `CHECK_INTERVAL=30` seconds
- `MAX_RETRIES=3` attempts
- `RETRY_DELAY=10` seconds

---

### `config/tailscale-watchdog.service`

**Systemd service file** for running watchdog as a background service.

**Installation:**
```bash
sudo cp config/tailscale-watchdog.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable tailscale-watchdog
sudo systemctl start tailscale-watchdog
```

---

## 📝 Testing Checklist

- [ ] TailscaleManager correctly parses `tailscale status --json`
- [ ] Auto-reconnect triggers when disconnected
- [ ] ModemStatus correctly reads signal strength via `mmcli`
- [ ] API endpoints return correct data
- [ ] Watchdog script restarts Tailscale on failure
- [ ] Mission Planner shows network status
- [ ] Works without sudo (where possible)
- [ ] Graceful degradation when Tailscale not installed

---

## 📚 Reference Commands

```bash
# Tailscale CLI
tailscale status              # Text status
tailscale status --json       # JSON status (use this!)
tailscale ip -4               # Get IPv4 address
tailscale ip -6               # Get IPv6 address
tailscale ping <peer>         # Ping another device
tailscale up                  # Connect
tailscale down                # Disconnect
tailscale logout              # Logout completely

# ModemManager (for 4G/LTE)
mmcli -L                      # List modems
mmcli -m 0                    # Modem info
mmcli -m 0 --signal-get       # Signal strength

# Network testing
ping -c 3 8.8.8.8            # Internet connectivity
ping -c 3 <gcs-ip>           # GCS connectivity
```

---

## 🔗 Dependencies

**Python packages:**
```
# None required - uses subprocess to call Tailscale CLI
```

**System packages (on Jetson):**
```bash
sudo apt install tailscale modemmanager
```

---

## 📎 Acceptance Criteria

1. ✅ `TailscaleManager` class monitors connection status
2. ✅ `NetworkMonitor` class reports 4G/LTE signal strength  
3. ⏳ API endpoints expose network status
4. ✅ Watchdog auto-reconnects Tailscale
5. ⏳ Mission Planner shows network health
6. ✅ All code follows project style (Python 3.13, type hints)
7. ✅ Documentation updated

---
