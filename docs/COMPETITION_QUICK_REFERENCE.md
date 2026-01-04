# NOMAD Competition Day Quick Reference

## Pre-Flight Checklist

### Jetson (Drone)
```bash
# Verify services
sudo systemctl status nomad
sudo systemctl status mavlink-router
sudo systemctl status tailscaled

# Check Tailscale connection
tailscale status
tailscale ip -4  # Note this IP

# Test API
curl http://127.0.0.1:8000/health

# Check logs
sudo journalctl -u nomad -n 50
```

### Ground Station (Laptop)
```bash
# Verify Tailscale
tailscale status
ping <jetson-tailscale-ip>

# Test API access
curl http://<jetson-tailscale-ip>:8000/health

# Test video
vlc rtsp://<jetson-tailscale-ip>:8554/primary
```

### Mission Planner
- [ ] Launch Mission Planner
- [ ] Connect to UDP port 14550
- [ ] Verify telemetry received
- [ ] Open NOMAD Control Panel (Tools → NOMAD)
- [ ] Verify Jetson Health Tab shows OK status
- [ ] Keep WASD Control DISABLED until Task 2

---

## During Flight

### Task 1: Locate (Outdoor GPS)

**Objective:** Take snapshot of target, determine GPS coordinates

1. Fly to search area using GPS waypoints
2. Identify target visually
3. Click **[CAP] Capture Snapshot** in NOMAD panel
4. System generates relative text description
5. Mission log saved automatically

**Status Messages to Expect:**
- "Vision: OK"
- "Task 1: Snapshot Captured"
- "Target: <description>"

### Task 2: Extinguish (Indoor VIO)

**Objective:** Find and extinguish fire using vision

1. **Before entering building:** Click **[CLR] Reset Exclusion Map**
2. Switch to VIO mode (ZED positional tracking)
3. **OPTIONAL:** Enable WASD Control for fine adjustments
4. Fly autonomously or use WASD for nudging
5. Vision system detects fire and registers hits
6. System tracks extinguished targets to avoid revisits

**WASD Controls (if enabled):**
- W/S = Forward/Backward
- A/D = Left/Right
- Q/E = Up/Down
- **Speed:** 0.5 m/s (configurable)

**Status Messages to Expect:**
- "Vision: YOLO Running"
- "Target: Fire Detected"
- "Target: Extinguished"

---

## Emergency Procedures

### Lost Telemetry

**Symptoms:** No MAVLink in Mission Planner

1. Check Tailscale: `tailscale status`
2. Check 4G/LTE signal strength
3. Restart mavlink-router: `sudo systemctl restart mavlink-router`
4. **Fallback:** Use RC transmitter (ELRS link)

### High Latency

**Symptoms:** >500ms ping time

1. Accept degraded performance
2. Rely on RC link for control
3. Use WASD sparingly or disable
4. Continue mission - telemetry is for monitoring only

### Vision Process Crash

**Automatic Recovery:** Watchdog restarts after 3 failures (3 seconds)

**Manual Override:**
```bash
ssh nomad@<jetson-ip>
sudo systemctl restart nomad
```

### API Not Responding

**Check:**
```bash
ssh nomad@<jetson-ip>
sudo systemctl status nomad
sudo systemctl restart nomad
```

### WASD Not Working

1. Ensure checkbox is CHECKED in Mission Planner
2. Click on Mission Planner window (give focus)
3. Verify Guided mode is active
4. Check MAVLink connection

---

## Important IP Addresses

```
Jetson Tailscale IP:     100.___.___.___  (Fill in)
Ground Station IP:       100.___.___.___  (Fill in)

Jetson Local Services:
  - API:        http://127.0.0.1:8000
  - RTSP:       rtsp://127.0.0.1:8554/primary
  - MAVLink:    127.0.0.1:14550

Remote Access (via Tailscale):
  - API:        http://<jetson-ip>:8000
  - RTSP:       rtsp://<jetson-ip>:8554/primary
  - SSH:        ssh nomad@<jetson-ip>
  - MAVLink:    UDP to Ground Station IP:14550
```

---

## Key Commands

### Jetson SSH Access
```bash
ssh nomad@<jetson-tailscale-ip>

# View logs
sudo journalctl -u nomad -f

# Restart service
sudo systemctl restart nomad

# Check hardware
jtop  # GPU/CPU/Temp monitor

# Download logs
scp nomad@<jetson-ip>:/home/nomad/NOMAD/data/mission_logs/*.json ./logs/
```

### Mission Planner

**Open NOMAD Panel:**
- Menu: Tools → NOMAD → Open Control Panel
- Shortcut: Ctrl+Shift+N (if configured)

**Enable WASD:**
- Check "Enable WASD Indoor Control"
- Only use in Task 2 (indoor)

**View Health:**
- Jetson Health Tab updates every 2 seconds
- Green = OK, Orange = Warning, Red = Critical

---

## Performance Expectations

### Normal Operation
- CPU: 10-50%
- GPU: 30-70% (with YOLO)
- Temp: 50-75°C
- RAM: <2GB used

### Warning Thresholds
- CPU Temp > 75°C
- GPU Temp > 75°C
- Latency > 200ms

### Critical Thresholds
- CPU Temp > 85°C (YOLO throttling activates)
- GPU Temp > 85°C
- Latency > 500ms (degrade gracefully)

---

## Post-Flight Data Collection

```bash
# Download mission logs
scp -r nomad@<jetson-ip>:/home/nomad/NOMAD/data/mission_logs ./logs_$(date +%Y%m%d_%H%M%S)/

# Download video recordings (if enabled)
scp -r nomad@<jetson-ip>:/mnt/nvme/recordings ./videos_$(date +%Y%m%d_%H%M%S)/

# View logs locally
cat logs/*.json | jq .
```

---

## Contact Information

**Team Members:**
- Name: _______________  Phone: _______________
- Name: _______________  Phone: _______________

**Tailscale Account:**
- Email: _______________________________
- Password: [Keep secure]

**4G/LTE Provider:**
- Carrier: _______________
- APN: _______________
- Support: _______________

---

## Quick Troubleshooting Matrix

| Symptom | Cause | Solution |
|---------|-------|----------|
| No telemetry | Tailscale down | `sudo systemctl restart tailscaled` |
| No video | MediaMTX not running | `sudo systemctl restart mediamtx` |
| Vision not working | Process crashed | Watchdog auto-restarts in 3s |
| High temp | Heavy YOLO load | YOLO throttles automatically |
| WASD not responding | Wrong mode | Switch to Guided mode |
| API error | Service crashed | `sudo systemctl restart nomad` |

---

## Notes Section

Use this space for competition-specific notes:

```
Weather: _______________
Wind: _______________
Cell Signal: _______________
Issues Encountered: _______________
_______________
_______________
_______________
```

---

**Remember:**
- RC link (ELRS) is primary control - always have transmitter ready
- Tailscale/MAVLink is for monitoring and telemetry
- WASD is for fine adjustments only, not primary control
- Vision watchdog provides automatic recovery
- System designed to degrade gracefully under adverse conditions

**Good Luck!**
