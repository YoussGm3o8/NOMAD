"""
Edge Core - Domain B for NOMAD.

Drone-side processing including state management, geospatial calculations,
MAVLink interface, and core services.

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

from .models import (
    SystemState,
    DetectionInfo,
)
from .state import StateManager
from .geospatial import (
    GPSCoordinate,
    NEDOffset,
    DroneState,
    calculate_gps_offset_meters,
    raycast_target_gps,
    haversine_distance,
    offset_gps_by_meters,
    bearing_between_points,
)
from .logging_service import (
    log_mission_event,
    list_mission_logs,
    read_mission_log,
)
from .ipc import (
    IPCMessage,
    ZMQPublisher,
    ZMQSubscriber,
    HeartbeatMonitor,
    DEFAULT_VISION_HEARTBEAT_ENDPOINT,
    DEFAULT_VISION_DATA_ENDPOINT,
)
from .time_manager import (
    TimeSyncService,
    TimeSyncStatus,
    TimeSyncSource,
    get_time_sync_service,
    init_time_sync_service,
)

__all__ = [
    # Models
    "SystemState",
    "StateManager",
    "DetectionInfo",
    # Geospatial
    "GPSCoordinate",
    "NEDOffset",
    "DroneState",
    "calculate_gps_offset_meters",
    "raycast_target_gps",
    "haversine_distance",
    "offset_gps_by_meters",
    "bearing_between_points",
    # Logging
    "log_mission_event",
    "list_mission_logs",
    "read_mission_log",
    # IPC
    "IPCMessage",
    "ZMQPublisher",
    "ZMQSubscriber",
    "HeartbeatMonitor",
    "DEFAULT_VISION_HEARTBEAT_ENDPOINT",
    "DEFAULT_VISION_DATA_ENDPOINT",
    # Time Synchronization
    "TimeSyncService",
    "TimeSyncStatus",
    "TimeSyncSource",
    "get_time_sync_service",
    "init_time_sync_service",
]
