# MAVLink Router Configuration

## Purpose
Split FC UART (921600) to local UDP endpoints and ground VPN.

## Outputs
- `127.0.0.1:14550` - Orchestrator (Edge Core)
- `127.0.0.1:14551` - Vision/VIO Process
- `<GCS_IP>:14550` - Ground Station (via Tailscale VPN)

## Configuration
- **Primary config file:** `main.conf` - This is the single source of truth
- Copy to `/etc/mavlink-router/main.conf` on the Jetson

## Installation
```bash
sudo cp main.conf /etc/mavlink-router/main.conf
sudo systemctl enable mavlink-router
sudo systemctl start mavlink-router
```
