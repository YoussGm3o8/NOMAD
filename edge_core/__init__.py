"""
Edge Core - Domain B for NOMAD.

Drone-side processing including state management, geospatial calculations,
MAVLink interface, and mission-specific logic.

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

from .models import (
    SystemState,
    Task1CaptureRequest,
    Task1CaptureResponse,
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
from .task1 import (
    Landmark,
    LandmarkMatch,
    Task1Result,
    load_landmarks,
    find_nearest_landmark,
    identify_target_relative_to_landmark,
    get_target_description,
)
from .task2 import (
    Point3D,
    ExclusionMap,
    ExclusionMapEntry,
    GimbalPID,
    get_gimbal_pid,
    reset_gimbal_pid,
)
from .visual_servoing import (
    ServoMode,
    BoundingBox,
    TrackingError,
    PIDController,
    PIDConfig,
    TargetTracker,
    TargetTrackerConfig,
    TrackingState,
    Task2Controller,
)
from .logging_service import (
    log_mission_event,
    log_task1_capture,
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
from .vision_process import (
    VisionConfig,
    run_vision_process,
)
from .mocks import (
    is_sim_mode,
    set_sim_mode,
    MockZedCamera,
    MockYOLO,
    MockMavlinkConnection,
    create_camera,
    create_yolo,
    create_mavlink_connection,
)
from .time_manager import (
    TimeSyncService,
    TimeSyncStatus,
    TimeSyncSource,
    get_time_sync_service,
    init_time_sync_service,
)
from .hardware_monitor import (
    HealthMonitor,
    HardwareStatus,
    HardwareThresholds,
    ThrottleLevel,
    get_health_monitor,
    init_health_monitor,
    DEFAULT_THROTTLE_ENDPOINT,
)
from .mission_validator import (
    EKFSourcePosXY,
    EKFSourceVelXY,
    EKFSourceYaw,
    EKFMode,
    EKFStatus,
    EKFValidator,
    ValidationResult,
    MissionValidator,
    check_ekf_source_sync,
)

__all__ = [
    # Models
    "SystemState",
    "StateManager",
    "Task1CaptureRequest",
    "Task1CaptureResponse",
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
    # Task 1
    "Landmark",
    "LandmarkMatch",
    "Task1Result",
    "load_landmarks",
    "find_nearest_landmark",
    "identify_target_relative_to_landmark",
    "get_target_description",
    # Task 2
    "Point3D",
    "ExclusionMap",
    "ExclusionMapEntry",
    "GimbalPID",
    "get_gimbal_pid",
    "reset_gimbal_pid",
    # Visual Servoing
    "ServoMode",
    "BoundingBox",
    "TrackingError",
    "PIDController",
    "PIDConfig",
    "TargetTracker",
    "TargetTrackerConfig",
    "TrackingState",
    "Task2Controller",
    # Logging
    "log_mission_event",
    "log_task1_capture",
    "list_mission_logs",
    "read_mission_log",
    # IPC
    "IPCMessage",
    "ZMQPublisher",
    "ZMQSubscriber",
    "HeartbeatMonitor",
    "DEFAULT_VISION_HEARTBEAT_ENDPOINT",
    "DEFAULT_VISION_DATA_ENDPOINT",
    # Vision Process (for external use)
    "VisionConfig",
    "run_vision_process",
    # Mocks / Simulation
    "is_sim_mode",
    "set_sim_mode",
    "MockZedCamera",
    "MockYOLO",
    "MockMavlinkConnection",
    "create_camera",
    "create_yolo",
    "create_mavlink_connection",
    # Time Synchronization
    "TimeSyncService",
    "TimeSyncStatus",
    "TimeSyncSource",
    "get_time_sync_service",
    "init_time_sync_service",
    # Hardware Monitoring
    "HealthMonitor",
    "HardwareStatus",
    "HardwareThresholds",
    "ThrottleLevel",
    "get_health_monitor",
    "init_health_monitor",
    "DEFAULT_THROTTLE_ENDPOINT",
    # Mission Validator
    "EKFSourcePosXY",
    "EKFSourceVelXY",
    "EKFSourceYaw",
    "EKFMode",
    "EKFStatus",
    "EKFValidator",
    "ValidationResult",
    "MissionValidator",
    "check_ekf_source_sync",
]
