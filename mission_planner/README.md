# NOMAD Mission Planner Plugin

**Centralized Control Interface for NOMAD Operations**

This C# plugin integrates directly with Mission Planner to provide full control of the NOMAD drone system including task execution, RTSP video streaming, and indoor manual control.

## Features

- **Task 1 (Recon)**: Capture snapshot and calculate target position relative to landmarks
- **Task 2 (Extinguish)**: Manage exclusion map and target hit registration
- **RTSP Video Streaming**: View live ZED camera and gimbal feeds via VLC or FFplay
- **Indoor Nudge**: WASD keyboard controls for manual indoor positioning via ELRS
- **Telemetry Display**: Real-time position and connection status
- **Dual-Link Communication**: HTTP API or MAVLink via ELRS for redundancy

## Files

| File | Description |
|------|-------------|
| `src/NOMADPlugin.cs` | Main plugin class (implements `MissionPlanner.Plugin.Plugin`) |
| `src/DualLinkSender.cs` | HTTP and MAVLink communication handler |
| `src/NOMADControlPanel.cs` | Flight Data tab UI control with video streaming |
| `src/NOMADConfig.cs` | Configuration persistence |
| `src/NOMADSettingsForm.cs` | Settings dialog |
| `src/NOMADPlugin.csproj` | Project file |

## Installation

### Prerequisites

1. **Mission Planner** - Download from [ardupilot.org](https://ardupilot.org/planner/docs/mission-planner-installation.html)
2. **Visual Studio 2022** - For building the plugin (Community edition is free)
3. **.NET Framework 4.7.2 or 4.8**
4. **VLC Media Player** (recommended) - For low-latency RTSP video playback

### Building

1. Open `src/NOMADPlugin.csproj` in Visual Studio

2. Restore NuGet packages:
   - Right-click solution → "Restore NuGet Packages"
   - Required: `Newtonsoft.Json`

3. Add Mission Planner references:
   - Right-click "References" → "Add Reference"
   - Browse to your Mission Planner installation folder
   - Add: `MissionPlanner.exe`, `MissionPlanner.Comms.dll`, `MAVLink.dll`

4. Build the solution (Release mode recommended):
   ```
   Build → Build Solution (Ctrl+Shift+B)
   ```

5. Copy the output DLL to Mission Planner plugins folder:
   ```powershell
   Copy-Item "bin\Release\NOMADPlugin.dll" "$env:LOCALAPPDATA\Mission Planner\plugins\"
   ```
   Or: `C:\Program Files (x86)\Mission Planner\plugins\`

## Configuration

Settings stored in:
```
%LOCALAPPDATA%\Mission Planner\plugins\nomad_config.json
```

Example:
```json
{
  "JetsonIP": "192.168.1.100",
  "JetsonPort": 8000,
  "RtspUrlPrimary": "rtsp://192.168.1.100:8554/live",
  "RtspUrlSecondary": "rtsp://192.168.1.100:8554/gimbal",
  "UseELRS": false,
  "HttpTimeoutSeconds": 5,
  "DebugMode": false
}
```

## Usage

### First Launch

1. Start Mission Planner
2. If Debug Mode is enabled, you may see a popup: "NOMAD Plugin v1.0.0 loaded"
3. Go to **Flight Data** tab
4. Use **NOMAD → Open Control Panel** (reliable), and/or look for the **NOMAD** tab if it was injected successfully

### Settings

1. Click **Tools → NOMAD Settings** in the menu
2. Configure Jetson IP, API port, and RTSP URLs
3. Click **Test Connection** to verify connectivity
4. Click **OK** to save

### Task 1: Recon

1. Navigate the drone to view a target
2. Click **📸 Capture Snapshot**
3. Result shows: "Target is Xm Direction of Landmark"

### Task 2: Extinguish

1. Click **🗑️ Reset Exclusion Map** at the start of a new run
2. Manually register hits with X, Y, Z coordinates and **Hit** button

### Video Streaming

1. Select video player: VLC (recommended), FFplay, or System Default
2. Click **🎥 Open Primary** for ZED/Navigation view
3. Click **🎯 Open Secondary** for Gimbal/Targeting view

### Indoor Nudge

1. Check **Enable Indoor Nudge (WASD)**
2. Use W/A/S/D for forward/left/back/right movement
3. Adjust nudge speed (default 0.5 m/s)

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Mission Planner                      │
│  ┌─────────────────────────────────────────────────┐   │
│  │              NOMAD Plugin                        │   │
│  │  ┌─────────┐ ┌─────────┐ ┌────────────────────┐ │   │
│  │  │ Task 1  │ │ Task 2  │ │   Video Streams    │ │   │
│  │  │ Capture │ │  Map    │ │  (VLC/FFplay)      │ │   │
│  │  └────┬────┘ └────┬────┘ └─────────┬──────────┘ │   │
│  │       │           │                │            │   │
│  │       ▼           ▼                ▼            │   │
│  │  ┌──────────────────────────────────────────┐   │   │
│  │  │           DualLinkSender                 │   │   │
│  │  │  HTTP API ──────────┐  MAVLink/ELRS     │   │   │
│  │  └──────────────────────┼───────────────────┘   │   │
│  └──────────────────────────┼──────────────────────┘   │
└──────────────────────────────┼─────────────────────────┘
                               │
         ┌─────────────────────┼─────────────────────┐
         │                     ▼                     │
         │  ┌──────────────────────────────────┐     │
         │  │        Jetson Orin Nano          │     │
         │  │  ┌─────────────────────────────┐ │     │
         │  │  │    NOMAD Edge Core API      │ │     │
         │  │  │    (FastAPI on port 8000)   │ │     │
         │  │  └─────────────────────────────┘ │     │
         │  │  ┌─────────────────────────────┐ │     │
         │  │  │   MediaMTX RTSP Server      │ │     │
         │  │  │   (port 8554: /live /gimbal)│ │     │
         │  │  └─────────────────────────────┘ │     │
         │  └──────────────────────────────────┘     │
         └───────────────────────────────────────────┘
```

## MAVLink Commands (ELRS Mode)

| Command ID | Name | Parameters |
|------------|------|------------|
| 31010 | `CMD_NOMAD_TASK1_CAPTURE` | p1: heading, p2: gimbal, p3: lidar |
| 31011 | `CMD_NOMAD_TASK2_RESET` | (none) |
| 31012 | `CMD_NOMAD_TASK2_HIT` | p1: x, p2: y, p3: z |

## Troubleshooting

- **Plugin Not Loading**: Check DLL is in correct plugins folder
- **Connection Failed**: Verify Jetson IP/port, check firewall
- **Video Not Playing**: Ensure VLC is installed and in PATH
- **Nudge Not Working**: Check ELRS connection, drone must be in GUIDED mode

