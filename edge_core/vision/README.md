# Vision / VIO

Responsibilities:
- ZED VIO to NED at 30 Hz; pin to high-priority cores.
- YOLOv8 detection for purple (dry) vs blue/green (wet) circles.
- Exclusion map storage (local frame) to avoid re-spraying.
- Gimbal PID control via MOUNT_CONTROL.

Organize by pipelines: sensors, detection, control, logging.
