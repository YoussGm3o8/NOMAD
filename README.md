# NOMAD Workspace Skeleton

Clean Architecture-aligned scaffolding for NOMAD (Networked Operations for MAD), targeting Jetson Orin Nano / ZED 2i / Tricopter Tiltrotor. See `PRD.md` for the approved requirements baseline.

## Domains (Clean Architecture)
- Domain A: Transport Layer (MAVLink routing, link bonding)
- Domain B: Edge Core (Onboard brain: Orchestrator, Vision, VIO)
- Domain C: Mission Planner Integration (Centralized control, RTSP video, ELRS fallback)

## Layout
- `docs/` — architecture notes and diagrams.
- `config/` — parameters, environment templates, and profiles.
- `transport/` — mavlink routing for FC-to-Jetson-to-Ground communication.
- `edge_core/` — orchestrator, vision, and shared components for the Jetson.
- `mission_planner/` — Ground Station plugin providing centralized control with embedded video viewer.
- `infra/` — deployment, packaging (MediaMTX for RTSP streaming), and CI/CD hooks.
- `scripts/` — helper utilities for dev/ops.
- `tests/` — validation per domain.

## Control Architecture
All control is centralized through the **Mission Planner plugin**:
- **Task 1 & 2 Controls**: Buttons and status displays in the NOMAD tab
- **RTSP Video Streams**: Open ZED/Gimbal camera feeds via VLC or FFplay
- **Indoor Nudge**: WASD keyboard controls for manual indoor positioning
- **ELRS Fallback**: Direct MAVLink commands when 4G/WiFi is unavailable

## Next
Populate each domain with implementation modules and tests; keep modules small and focused per requirement IDs in `PRD.md`.
