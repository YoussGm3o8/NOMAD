This document outlines the software architecture and requirements for **NOMAD** (**N**etworked **O**perations for **MAD**).

This specification is designed to be included in the **AEAC Phase 1 Design Paper** under the "System Integration" and "Mission Strategy" sections. It adheres to **Clean Architecture** principles, ensuring modularity, testability, and separation of concerns.

***

# Product Requirements Document (PRD)

| **Project Name** | **NOMAD** (Networked Operations for MAD) |
| :--- | :--- |
| **Team** | McGill Aerial Design (MAD) |
| **Competition** | AEAC Student Competition 2026 |
| **Platform** | Jetson Orin Nano / ZED 2i / Tricopter Tiltrotor (Quad Config) |
| **Software Stack** | Python 3.13.9 (Free-Threading), C# .NET, Vue.js |
| **Version** | **2.1.0 (Final Release)** |
| **Status** | **APPROVED FOR IMPLEMENTATION** |

---

## 1. Executive Summary
**NOMAD** is a distributed Command and Control (C2) architecture designed to bridge the "Air Gap" between the pilot and the intelligent edge. It utilizes a hybrid network strategy: a high-bandwidth 4G/LTE link for real-time video/API control, and a low-latency 900MHz ELRS link for safety-critical telemetry and fallback maneuvering.

The system is designed for **Human-Machine Teaming**:
1.  **Task 1 (Recon):** The Pilot flies manually while the Edge AI acts as a "Flight Officer," performing complex geospatial calculations to generate relative text descriptions of targets relative to landmarks.
2.  **Task 2 (Extinguish):** The Drone operates semi-autonomously indoors using Visual Inertial Odometry (VIO), actively hunting pH-sensitive targets while maintaining a stateful exclusion map to prevent redundant spraying.

---

## 2. System Architecture Scope

The system follows **Clean Architecture** principles, divided into three logical domains:

1.  **Domain A: Transport Layer (Connectivity)** – MAVLink routing for telemetry and command distribution.
2.  **Domain B: Edge Core (The Brain)** – The Onboard Orchestrator, Vision Engine, and VIO pipeline running on the Jetson.
3.  **Domain C: Mission Planner Integration (The Control Center)** – A custom plugin providing centralized control, telemetry display, RTSP video streaming, and ELRS fallback control.

---

## 3. Functional Requirements

### 3.1 Domain A: Transport Layer (Connectivity)
*Objective: Route MAVLink messages between the Flight Controller, Jetson, and Ground Station.*

*   **[NOMAD-NET-001] Mavlink Routing**
    *   The system **MUST** implement a `mavlink-router` instance on the Jetson.
    *   **Input:** UART (baud 921600) from the Flight Controller (Cube Orange+).
    *   **Output 1 (Internal):** UDP `127.0.0.1:14550` (For NOMAD Orchestrator).
    *   **Output 2 (Internal):** UDP `127.0.0.1:14551` (For Vision Engine/VIO).
    *   **Output 3 (Ground):** UDP `<Ground_IP>:14550` (For Mission Planner).
*   **[NOMAD-NET-002] Link Bonding & Failover**
    *   The system **SHOULD** prioritize the 4G/WiFi interface for high-bandwidth data (Video/Logs).
    *   The system **MUST** maintain the ELRS UART link as a hard-wired fallback. If 4G is lost, control authority falls back to ELRS immediately.

### 3.2 Domain B: Edge Core (Jetson Orin Nano)
*Objective: The central processing unit running Python 3.13.9 to utilize Free-Threading (No-GIL) for parallel networking and AI inference.*

#### **B.1 Orchestrator & State Management**
*   **[EDGE-ORCH-01] API Interface:** A FastAPI service **MUST** expose endpoints for `/arm`, `/disarm`, `/mode/{flight_mode}`, `/task/{id}/start`, and `/payload/trigger`.
*   **[EDGE-ORCH-02] Watchdog:** The Orchestrator **MUST** monitor the Vision/VIO process. If the Vision process hangs, it **MUST** restart it automatically without severing the Mavlink connection to the ground.
*   **[EDGE-TIME-01] Hybrid Time Sync:** The system **MUST** run an NTP client (Chrony) to sync system time via 4G. If 4G is unavailable, it **MUST** force-sync system time from the GPS Mavlink stream to ensure accurate timestamping for Task 1.

#### **B.2 Task 1 Logic (Manual Recon Assist)**
*   **[T1-LOC-01] Landmark Database:** The system **MUST** load a configuration file (`landmarks.json`) containing the GPS coordinates of competition landmarks (e.g., "Red Car", "Blue Tent").
*   **[T1-CALC-01] Relative Description Generator:**
    *   **Trigger:** Upon pilot command (Web UI or Controller Button).
    *   **Input:** Drone GPS, Compass Heading, Gimbal Pitch, Lidar Distance.
    *   **Logic:** Raycast to find Target GPS $\rightarrow$ Find nearest Landmark $\rightarrow$ Calculate relative vector.
    *   **Output:** Generate text string: *"Target is {X}m {Direction} of {Landmark}"*.
*   **[T1-LOG-01] Evidence Capture:** The system **MUST** save the high-res image and the generated text description to the NVMe drive immediately.

#### **B.3 Task 2 Logic (Indoor Autonomy)**
*   **[T2-NAV-01] VIO Integration (ZED 2i):**
    *   The system **MUST** run a dedicated thread interfacing with the ZED SDK.
    *   It **MUST** transform ZED pose data to NED frame and stream `VISION_POSITION_ESTIMATE` to the Flight Controller at **30Hz**.
    *   **Constraint:** The ZED process must be pinned to high-priority CPU cores.
*   **[T2-CV-01] pH-Target Detection:**
    *   The Vision Engine **MUST** utilize a YOLOv8 model trained to detect **Purple Circles** (Dry) and **Blue/Green Circles** (Wet).
    *   **Firing Solution:** The Water Pump **MUST** trigger only if:
        1.  Target Class is "Purple".
        2.  Target is NOT in the [Exclusion Map].
        3.  Aiming Error < 5 degrees.
        4.  Range is 1.5m - 2.5m.
*   **[T2-MEM-01] The Exclusion Map:**
    *   When a target is sprayed, the system **MUST** log its 3D coordinates ($x, y, z$) in the local VIO frame.
    *   Future detections within **0.5m** of these coordinates **MUST** be ignored to prevent re-spraying.
*   **[T2-AIM-01] Gimbal Visual Servoing:** The system **MUST** run a PID loop on the Gimbal Camera feed to keep the target centered, sending `MOUNT_CONTROL` Mavlink messages to the gimbal controller.

### 3.3 Domain C: Mission Planner Integration (Control Center)
*Objective: Centralized control interface via Mission Planner plugin with embedded video streaming.*

*   **[MP-PLUG-01] ELRS Tunneling:** The C# Plugin **MUST** allow the user to send high-level commands (e.g., "Start Auto-Extinguish", "Switch to VIO Mode") via the ELRS Mavlink stream. This ensures control even if 4G/WiFi fails inside the metal shed.
*   **[MP-PLUG-02] Indoor Nudge:** The plugin **MUST** map keyboard WASD keys to `SET_POSITION_TARGET_LOCAL_NED` velocity commands, allowing the pilot to manually reposition the drone indoors without GPS.
*   **[MP-PLUG-03] Telemetry Injection:** The plugin **MUST** inject Jetson health data (CPU Temp, GPU Load, Vision Confidence) into the Mission Planner Quick View tab.
*   **[MP-PLUG-04] Video Streaming:** The plugin **MUST** display RTSP video streams from the Jetson (ZED camera, gimbal camera) via embedded video player controls.
*   **[MP-PLUG-05] Task 1 Interface:** The plugin **MUST** provide capture button and display results with target text descriptions directly in the UI.
*   **[MP-PLUG-06] Task 2 Interface:** The plugin **MUST** provide exclusion map controls and target hit registration for Task 2 operations.

---

## 4. Hardware & Configuration Requirements

*   **[HW-CFG-01] Configuration Profiles:**
    *   The system **MUST** store separate ArduPilot parameter files for **Task 1 (Winged/Outdoor)** and **Task 2 (Wingless/Indoor)**.
    *   The Orchestrator **MUST** verify the loaded profile matches the selected software mission mode upon boot.
*   **[HW-THRM-01] Thermal Safety:**
    *   If Jetson Core Temp > 85°C, the system **MUST** throttle the YOLO inference rate (skip frames) but **MUST NOT** throttle the ZED VIO thread.
*   **[HW-PWR-01] Brownout Protection:**
    *   If Battery Voltage < Critical Threshold (under load), the Orchestrator **MUST** disable the Vision Engine (GPU) to preserve power for the Flight Controller and motors.

---

## 5. AEAC Compliance Checklist

*   **[COMP-01] Evidence:** The Orchestrator **MUST** automatically zip all mission logs and captured images into a standardized folder structure (`/Mission_Data/Task_X/`) accessible via HTTP download immediately post-flight.
*   **[COMP-02] Safety Failsafe:** If the ZED 2i disconnects or VIO confidence drops below threshold indoors, the system **MUST** automatically switch the drone to `ALT_HOLD` mode (Barometer/IMU only) to prevent a fly-away.

---

## 6. Development Architecture Diagram Description

1.  **Transport:** `Mavlink-Router` splits the traffic between Jetson and Ground Station.
2.  **Edge (Process 1 - Orchestrator):** `FastAPI` handles logic. `Pymavlink` talks to FC. `NTP` syncs time.
3.  **Edge (Process 2 - Vision):** `ZED SDK` handles VIO. `YOLOv8` handles Detection. `PID` handles Gimbal. `RTSP` handles Streaming.
4.  **Ground:** `Mission Planner` provides centralized control with embedded RTSP video viewer, task controls, and telemetry display. Communication via MAVLink (UDP) and ELRS (Serial).