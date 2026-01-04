import asyncio
from typing import TYPE_CHECKING, Optional

from fastapi import FastAPI, WebSocket, HTTPException
from fastapi.encoders import jsonable_encoder
from fastapi.websockets import WebSocketDisconnect
from pydantic import BaseModel

from .models import Task1CaptureRequest, Task1CaptureResponse
from .state import StateManager
from .geospatial import GPSCoordinate, DroneState
from .task1 import identify_target_relative_to_landmark, Landmark
from .logging_service import log_task1_capture

if TYPE_CHECKING:
    pass


# ==================== Task 2 Request/Response Models ====================


class Task2TargetHitRequest(BaseModel):
    """Request model for registering a target hit."""

    x: float
    y: float
    z: float
    label: Optional[str] = ""


class Task2TargetHitResponse(BaseModel):
    """Response model for target hit registration."""

    success: bool
    is_new_target: bool
    total_targets: int
    message: str


class Task2ResetMapResponse(BaseModel):
    """Response model for exclusion map reset."""

    success: bool
    cleared_count: int
    message: str


class Task2CheckTargetRequest(BaseModel):
    """Request model for checking if target is extinguished."""

    x: float
    y: float
    z: float
    threshold: Optional[float] = 0.5


class Task2CheckTargetResponse(BaseModel):
    """Response model for target check."""

    is_extinguished: bool
    nearest_distance: Optional[float] = None
    nearest_label: Optional[str] = None


def create_app(
    state_manager: StateManager,
    landmarks: list[Landmark] | None = None,
) -> FastAPI:
    """
    Create the FastAPI application for Edge Core.

    Args:
        state_manager: StateManager instance for system state
        landmarks: Pre-loaded landmarks for Task 1

    Returns:
        Configured FastAPI application
    """
    app = FastAPI(
        title="NOMAD Edge Core API",
        description="Drone-side API for NOMAD (AEAC 2026) - Mission Planner Integration",
        version="0.1.0",
    )

    # Store landmarks in app state for Task 1
    app.state.landmarks = landmarks or []

    # ==================== Root / Health ====================

    @app.get("/")
    async def root():
        """API root - returns status information."""
        return {
            "service": "NOMAD Edge Core",
            "version": "0.1.0",
            "description": "Connect via Mission Planner plugin for full control",
        }

    # ==================== Status Endpoints ====================

    @app.get("/status")
    async def get_status():
        """Get current system state."""
        return state_manager.get_state()

    @app.get("/health")
    async def health_check():
        """Health check endpoint."""
        state = state_manager.get_state()
        return {
            "status": "healthy" if state.connected else "degraded",
            "connected": state.connected,
            "gps_fix": state.gps_fix,
            "flight_mode": state.flight_mode,
        }

    @app.websocket("/ws/state")
    async def ws_state(websocket: WebSocket):
        """WebSocket endpoint for real-time state updates."""
        await websocket.accept()
        try:
            while True:
                state = state_manager.get_state()
                await websocket.send_json(jsonable_encoder(state))
                await asyncio.sleep(0.1)
        except WebSocketDisconnect:
            return

    @app.post("/api/task/1/capture", response_model=Task1CaptureResponse)
    async def task1_capture(request: Task1CaptureRequest | None = None):
        """
        Task 1 Capture Endpoint.

        Captures the current position, calculates target location relative
        to nearest landmark, logs the event, and returns the result.

        Request body (optional):
            - heading_deg: Override heading (for testing)
            - gimbal_pitch_deg: Override gimbal pitch (for testing)
            - lidar_distance_m: Override LiDAR distance (for testing)

        Returns:
            Task1CaptureResponse with target text and log file path
        """
        # Get current system state
        state = state_manager.get_state()

        # Check for valid GPS
        if not state.has_valid_gps():
            raise HTTPException(
                status_code=400,
                detail="No valid GPS fix available",
            )

        # Get sensor values (use overrides from request if provided)
        request = request or Task1CaptureRequest()
        heading = request.heading_deg if request.heading_deg is not None else state.heading_deg
        gimbal_pitch = request.gimbal_pitch_deg if request.gimbal_pitch_deg is not None else state.gimbal_pitch_deg
        lidar_dist = request.lidar_distance_m if request.lidar_distance_m is not None else state.lidar_distance_m

        # Validate required sensor data
        if heading is None:
            raise HTTPException(
                status_code=400,
                detail="Heading not available. Provide heading_deg in request.",
            )
        if gimbal_pitch is None:
            raise HTTPException(
                status_code=400,
                detail="Gimbal pitch not available. Provide gimbal_pitch_deg in request.",
            )
        if lidar_dist is None:
            raise HTTPException(
                status_code=400,
                detail="LiDAR distance not available. Provide lidar_distance_m in request.",
            )

        # Check landmarks are loaded
        if not app.state.landmarks:
            raise HTTPException(
                status_code=500,
                detail="No landmarks configured. Check landmarks.json.",
            )

        # Build drone state for geospatial calculation
        drone_state = DroneState(
            gps=GPSCoordinate(
                lat=state.gps_lat,  # type: ignore
                lon=state.gps_lon,  # type: ignore
                alt=state.gps_alt or 0.0,
            ),
            heading_deg=heading,
            gimbal_pitch_deg=gimbal_pitch,
            lidar_distance_m=lidar_dist,
        )

        # Calculate target relative to landmark
        result = identify_target_relative_to_landmark(
            drone_state,
            landmarks=app.state.landmarks,
        )

        if result is None:
            raise HTTPException(
                status_code=500,
                detail="Failed to identify target relative to landmark",
            )

        # Prepare drone state data for logging
        drone_state_data = {
            "gps_lat": state.gps_lat,
            "gps_lon": state.gps_lon,
            "gps_alt": state.gps_alt,
            "heading_deg": heading,
            "gimbal_pitch_deg": gimbal_pitch,
            "lidar_distance_m": lidar_dist,
            "flight_mode": state.flight_mode,
            "battery_voltage": state.battery_voltage,
            "timestamp": state.timestamp.isoformat(),
        }

        # Log the capture event
        log_path = log_task1_capture(
            target_text=result.formatted_text,
            target_gps={
                "lat": result.target_gps.lat,
                "lon": result.target_gps.lon,
                "alt": result.target_gps.alt,
            },
            drone_state=drone_state_data,
            landmark_name=result.nearest_landmark.landmark.name,
            offset_meters={
                "north": result.raw_offset.north,
                "east": result.raw_offset.east,
            },
        )

        return Task1CaptureResponse(
            success=True,
            target_text=result.formatted_text,
            file_path=str(log_path),
            target_gps={
                "lat": result.target_gps.lat,
                "lon": result.target_gps.lon,
                "alt": result.target_gps.alt,
            },
            nearest_landmark=result.nearest_landmark.landmark.name,
            offset_meters={
                "north": round(result.raw_offset.north, 2),
                "east": round(result.raw_offset.east, 2),
            },
        )

    @app.get("/api/task/1/landmarks")
    async def get_landmarks():
        """Get configured landmarks for Task 1."""
        return {
            "count": len(app.state.landmarks),
            "landmarks": [
                {
                    "name": lm.name,
                    "lat": lm.lat,
                    "lon": lm.lon,
                    "description": lm.description,
                }
                for lm in app.state.landmarks
            ],
        }

    # ==================== Task 2 Endpoints ====================

    @app.post("/api/task/2/reset_map", response_model=Task2ResetMapResponse)
    async def task2_reset_map():
        """
        Reset the exclusion map (clear all stored targets).

        Clears all extinguished target positions from memory.
        Use this at the start of a new Task 2 run.

        Returns:
            Task2ResetMapResponse with cleared count
        """
        cleared_count = state_manager.clear_exclusion_map()

        return Task2ResetMapResponse(
            success=True,
            cleared_count=cleared_count,
            message=f"Cleared {cleared_count} targets from exclusion map",
        )

    @app.post("/api/task/2/target_hit", response_model=Task2TargetHitResponse)
    async def task2_target_hit(request: Task2TargetHitRequest):
        """
        Register a target hit (mark target as extinguished).

        Adds the target coordinates to the exclusion map so it won't
        be re-targeted. Can be used for manual override or testing.

        Request body:
            - x: X coordinate (meters)
            - y: Y coordinate (meters)
            - z: Z coordinate (meters)
            - label: Optional label for the target

        Returns:
            Task2TargetHitResponse with registration status
        """
        is_new = state_manager.add_extinguished_target(
            x=request.x,
            y=request.y,
            z=request.z,
            label=request.label or "",
        )

        total = state_manager.exclusion_map.count

        if is_new:
            message = f"New target registered at ({request.x:.2f}, {request.y:.2f}, {request.z:.2f})"
        else:
            message = f"Target merged with existing entry (hit count incremented)"

        return Task2TargetHitResponse(
            success=True,
            is_new_target=is_new,
            total_targets=total,
            message=message,
        )

    @app.post("/api/task/2/check_target", response_model=Task2CheckTargetResponse)
    async def task2_check_target(request: Task2CheckTargetRequest):
        """
        Check if a target position is already extinguished.

        Queries the exclusion map to determine if a potential target
        has already been processed.

        Request body:
            - x, y, z: Target coordinates (meters)
            - threshold: Distance threshold (default 0.5m)

        Returns:
            Task2CheckTargetResponse with extinguished status
        """
        is_extinguished = state_manager.is_target_extinguished(
            x=request.x,
            y=request.y,
            z=request.z,
            threshold=request.threshold or 0.5,
        )

        # Find nearest for additional context
        nearest, distance = state_manager.exclusion_map.find_nearest(
            request.x, request.y, request.z
        )

        return Task2CheckTargetResponse(
            is_extinguished=is_extinguished,
            nearest_distance=distance if nearest else None,
            nearest_label=nearest.label if nearest else None,
        )

    @app.get("/api/task/2/exclusion_map")
    async def task2_get_exclusion_map():
        """
        Get the current exclusion map state.

        Returns all stored target positions and metadata.

        Returns:
            Dictionary with exclusion map data
        """
        return state_manager.get_exclusion_map_data()

    return app
