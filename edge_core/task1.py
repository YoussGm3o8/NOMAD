"""
Task 1 (Recon Mission) Logic for NOMAD.

Handles landmark-relative text generation for target identification.
Manual flight with relative text output based on nearest landmark.

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

from .geospatial import (
    GPSCoordinate,
    NEDOffset,
    calculate_gps_offset_meters,
    haversine_distance,
    raycast_target_gps,
    DroneState,
)

if TYPE_CHECKING:
    from .models import SystemState


# Default landmarks config path
DEFAULT_LANDMARKS_PATH = Path(__file__).parent.parent / "config" / "landmarks.json"


@dataclass(frozen=True, slots=True)
class Landmark:
    """A landmark reference point for Task 1."""

    name: str
    lat: float
    lon: float
    description: str = ""

    @property
    def gps(self) -> GPSCoordinate:
        """Get GPS coordinate for this landmark."""
        return GPSCoordinate(lat=self.lat, lon=self.lon)


@dataclass(frozen=True, slots=True)
class LandmarkMatch:
    """Result of finding the nearest landmark to a target."""

    landmark: Landmark
    distance_m: float
    offset: NEDOffset


@dataclass(frozen=True, slots=True)
class Task1Result:
    """Result of Task 1 target identification."""

    target_gps: GPSCoordinate
    nearest_landmark: LandmarkMatch
    formatted_text: str
    raw_offset: NEDOffset


def load_landmarks(config_path: Path | str | None = None) -> list[Landmark]:
    """
    Load landmarks from JSON configuration file.

    Args:
        config_path: Path to landmarks.json file. Uses default if None.

    Returns:
        List of Landmark objects

    Raises:
        FileNotFoundError: If config file doesn't exist
        json.JSONDecodeError: If config file is invalid JSON
    """
    path = Path(config_path) if config_path else DEFAULT_LANDMARKS_PATH

    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    landmarks = []
    for item in data.get("landmarks", []):
        landmarks.append(
            Landmark(
                name=item["name"],
                lat=item["lat"],
                lon=item["lon"],
                description=item.get("description", ""),
            )
        )

    return landmarks


def find_nearest_landmark(
    target: GPSCoordinate,
    landmarks: list[Landmark],
) -> LandmarkMatch | None:
    """
    Find the nearest landmark to a target GPS coordinate.

    Args:
        target: Target GPS coordinate to find nearest landmark for
        landmarks: List of available landmarks

    Returns:
        LandmarkMatch with nearest landmark, distance, and offset.
        None if landmarks list is empty.
    """
    if not landmarks:
        return None

    nearest: LandmarkMatch | None = None
    min_distance = float("inf")

    for landmark in landmarks:
        distance = haversine_distance(target, landmark.gps)
        if distance < min_distance:
            min_distance = distance
            # Calculate offset FROM landmark TO target
            # This gives us "target is X meters North/East of landmark"
            offset = calculate_gps_offset_meters(landmark.gps, target)
            nearest = LandmarkMatch(
                landmark=landmark,
                distance_m=distance,
                offset=offset,
            )

    return nearest


def format_direction_text(offset: NEDOffset, precision: int = 1) -> str:
    """
    Format NED offset as human-readable direction text.

    Args:
        offset: North/East/Down offset in meters
        precision: Decimal places for distance values

    Returns:
        Formatted string like "1.5m North and 0.5m East"

    Examples:
        >>> format_direction_text(NEDOffset(north=1.5, east=0.5))
        '1.5m North and 0.5m East'
        >>> format_direction_text(NEDOffset(north=-2.0, east=-1.0))
        '2.0m South and 1.0m West'
        >>> format_direction_text(NEDOffset(north=3.0, east=0.0))
        '3.0m North'
    """
    parts = []

    # North/South component
    if abs(offset.north) >= 0.1:  # 10cm threshold
        direction = "North" if offset.north >= 0 else "South"
        parts.append(f"{abs(offset.north):.{precision}f}m {direction}")

    # East/West component
    if abs(offset.east) >= 0.1:  # 10cm threshold
        direction = "East" if offset.east >= 0 else "West"
        parts.append(f"{abs(offset.east):.{precision}f}m {direction}")

    if not parts:
        return "at location"

    return " and ".join(parts)


def format_landmark_relative_text(
    match: LandmarkMatch,
    precision: int = 1,
) -> str:
    """
    Format the landmark-relative position as text.

    Args:
        match: LandmarkMatch containing landmark and offset info
        precision: Decimal places for distance values

    Returns:
        Formatted string like "Target is 1.5m North and 0.5m East of Red Car"
    """
    direction_text = format_direction_text(match.offset, precision)
    return f"Target is {direction_text} of {match.landmark.name}"


def identify_target_relative_to_landmark(
    drone_state: DroneState,
    landmarks: list[Landmark] | None = None,
    config_path: Path | str | None = None,
) -> Task1Result | None:
    """
    Main Task 1 function: Identify target position relative to nearest landmark.

    Takes drone state (GPS, heading, gimbal pitch, LiDAR distance), calculates
    target location via raycasting, finds nearest landmark, and returns
    formatted text description.

    Args:
        drone_state: Current drone state with GPS, heading, gimbal pitch, LiDAR
        landmarks: Pre-loaded landmarks list (optional)
        config_path: Path to landmarks.json if landmarks not provided

    Returns:
        Task1Result with target GPS, nearest landmark, and formatted text.
        None if no landmarks available.

    Example:
        >>> state = DroneState(
        ...     gps=GPSCoordinate(lat=45.5, lon=-73.5, alt=50.0),
        ...     heading_deg=0.0,
        ...     gimbal_pitch_deg=-45.0,
        ...     lidar_distance_m=70.71
        ... )
        >>> result = identify_target_relative_to_landmark(state)
        >>> print(result.formatted_text)
        'Target is 1.5m North and 0.5m East of Red Car'
    """
    # Load landmarks if not provided
    if landmarks is None:
        landmarks = load_landmarks(config_path)

    if not landmarks:
        return None

    # Calculate target GPS via raycasting
    target_gps = raycast_target_gps(drone_state)

    # Find nearest landmark
    match = find_nearest_landmark(target_gps, landmarks)
    if match is None:
        return None

    # Format the result text
    formatted_text = format_landmark_relative_text(match)

    return Task1Result(
        target_gps=target_gps,
        nearest_landmark=match,
        formatted_text=formatted_text,
        raw_offset=match.offset,
    )


def process_state_for_task1(
    state: "SystemState",
    heading_deg: float,
    gimbal_pitch_deg: float,
    lidar_distance_m: float,
    landmarks: list[Landmark] | None = None,
) -> Task1Result | None:
    """
    Process SystemState for Task 1 target identification.

    This is a convenience function that extracts GPS from SystemState
    and performs the landmark-relative calculation.

    Args:
        state: Current SystemState (must have gps_lat, gps_lon, gps_alt attributes)
        heading_deg: Drone heading in degrees (0-360)
        gimbal_pitch_deg: Gimbal pitch in degrees (-90 to 0)
        lidar_distance_m: LiDAR distance to target in meters
        landmarks: Pre-loaded landmarks (optional)

    Returns:
        Task1Result or None if processing fails

    Note:
        The SystemState model needs gps_lat, gps_lon, gps_alt fields.
        If not available, use identify_target_relative_to_landmark directly.
    """
    # Extract GPS from state if available
    gps_lat = getattr(state, "gps_lat", None)
    gps_lon = getattr(state, "gps_lon", None)
    gps_alt = getattr(state, "gps_alt", 0.0)

    if gps_lat is None or gps_lon is None:
        return None

    drone_state = DroneState(
        gps=GPSCoordinate(lat=gps_lat, lon=gps_lon, alt=gps_alt),
        heading_deg=heading_deg,
        gimbal_pitch_deg=gimbal_pitch_deg,
        lidar_distance_m=lidar_distance_m,
    )

    return identify_target_relative_to_landmark(drone_state, landmarks)


# Convenience function for direct usage
def get_target_description(
    lat: float,
    lon: float,
    alt: float,
    heading_deg: float,
    gimbal_pitch_deg: float,
    lidar_distance_m: float,
    config_path: Path | str | None = None,
) -> str:
    """
    Get target description relative to nearest landmark.

    Simplified API for getting the formatted text output.

    Args:
        lat: Drone latitude
        lon: Drone longitude
        alt: Drone altitude (meters MSL)
        heading_deg: Drone heading (0-360, 0=North)
        gimbal_pitch_deg: Gimbal pitch (-90 to 0, -90=straight down)
        lidar_distance_m: LiDAR measured distance to target
        config_path: Optional path to landmarks.json

    Returns:
        Formatted description string or error message
    """
    drone_state = DroneState(
        gps=GPSCoordinate(lat=lat, lon=lon, alt=alt),
        heading_deg=heading_deg,
        gimbal_pitch_deg=gimbal_pitch_deg,
        lidar_distance_m=lidar_distance_m,
    )

    result = identify_target_relative_to_landmark(drone_state, config_path=config_path)

    if result is None:
        return "No landmarks available for reference"

    return result.formatted_text
