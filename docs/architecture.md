# Architecture Outline

## Domains
- Transport (A): tailscale, link bonding/failover via 4G/WiFi + ELRS.
- Edge Core (B): FastAPI orchestrator, watchdog/time-sync, Task 1 recon logic, Task 2 VIO + CV + gimbal control.
- Mission Planner (C): Centralized control plugin with RTSP video viewer, ELRS tunneling, indoor nudge, telemetry injection, task controls.

## Process Separation
- `edge_core/orchestrator`: API + state management; watchdog for vision/VIO.
- `edge_core/vision`: ZED VIO + YOLO + gimbal PID + exclusion map.
- `transport`: mavlink routing; keep FC-facing ports stable.

## Data Flow (high-level)
- FC UART → mavlink-router → Orchestrator + Vision + Mission Planner (UDP/ELRS).
- Vision outputs detections + VIO → Orchestrator decisions → FC setpoints/gimbal commands.
- Mission Planner plugin provides centralized control, receives RTSP video streams, and sends task triggers via HTTP API.

Keep each module small: one responsibility per package, tests in `tests/` mirroring package paths.
