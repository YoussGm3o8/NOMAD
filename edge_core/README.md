# Domain B: Edge Core (Jetson)

Onboard services (Python 3.13, free-threading). Keep processes small: orchestrator vs vision.

- `orchestrator/`: FastAPI interface, watchdog, time sync, task logic coordination.
- `vision/`: ZED VIO, YOLO detection, gimbal servoing, exclusion map.
- `common/`: shared utilities (logging, settings, message schemas).
