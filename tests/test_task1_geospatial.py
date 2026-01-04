"""
Tests for Task 1 Geospatial Logic.

Run with: python -m pytest tests/test_task1_geospatial.py -v
Or standalone: python tests/test_task1_geospatial.py
"""

from __future__ import annotations

import math
import sys

try:
    import pytest
    HAS_PYTEST = True
except ImportError:
    HAS_PYTEST = False
    # Create minimal pytest compatibility for standalone execution
    class _PytestCompat:
        @staticmethod
        def main(args):
            print("pytest not installed. Running tests manually...")
            return 1
    pytest = _PytestCompat()  # type: ignore

from edge_core.geospatial import (
    GPSCoordinate,
    NEDOffset,
    DroneState,
    calculate_gps_offset_meters,
    raycast_target_gps,
    haversine_distance,
    offset_gps_by_meters,
    deg_to_rad,
    rad_to_deg,
    bearing_between_points,
)
from edge_core.task1 import (
    Landmark,
    load_landmarks,
    find_nearest_landmark,
    format_direction_text,
    identify_target_relative_to_landmark,
    get_target_description,
)


class TestGeospatialBasics:
    """Test basic geospatial utility functions."""

    def test_deg_to_rad(self):
        assert deg_to_rad(0) == 0
        assert abs(deg_to_rad(180) - math.pi) < 1e-10
        assert abs(deg_to_rad(90) - math.pi / 2) < 1e-10

    def test_rad_to_deg(self):
        assert rad_to_deg(0) == 0
        assert abs(rad_to_deg(math.pi) - 180) < 1e-10
        assert abs(rad_to_deg(math.pi / 2) - 90) < 1e-10


class TestGPSOffset:
    """Test GPS offset calculations."""

    def test_zero_offset(self):
        """Same coordinates should return zero offset."""
        coord = GPSCoordinate(lat=45.5, lon=-73.5)
        offset = calculate_gps_offset_meters(coord, coord)
        assert abs(offset.north) < 0.01
        assert abs(offset.east) < 0.01

    def test_north_offset(self):
        """Moving north should give positive north offset."""
        origin = GPSCoordinate(lat=45.5, lon=-73.5)
        target = GPSCoordinate(lat=45.5001, lon=-73.5)  # ~11m north
        offset = calculate_gps_offset_meters(origin, target)
        assert offset.north > 10  # Should be approximately 11m
        assert abs(offset.east) < 0.1  # No east/west movement

    def test_east_offset(self):
        """Moving east should give positive east offset."""
        origin = GPSCoordinate(lat=45.5, lon=-73.5)
        target = GPSCoordinate(lat=45.5, lon=-73.4999)  # ~7.8m east
        offset = calculate_gps_offset_meters(origin, target)
        assert abs(offset.north) < 0.1  # No north/south movement
        assert offset.east > 7  # Should be approximately 7.8m

    def test_south_west_offset(self):
        """Moving south-west should give negative offsets."""
        origin = GPSCoordinate(lat=45.5, lon=-73.5)
        target = GPSCoordinate(lat=45.4999, lon=-73.5001)
        offset = calculate_gps_offset_meters(origin, target)
        assert offset.north < 0  # South
        assert offset.east < 0  # West


class TestHaversineDistance:
    """Test Haversine distance calculation."""

    def test_same_point_distance(self):
        """Same point should have zero distance."""
        coord = GPSCoordinate(lat=45.5, lon=-73.5)
        assert haversine_distance(coord, coord) == 0

    def test_known_distance(self):
        """Test with known coordinates."""
        # ~11.1m north
        a = GPSCoordinate(lat=45.5, lon=-73.5)
        b = GPSCoordinate(lat=45.5001, lon=-73.5)
        dist = haversine_distance(a, b)
        assert 11.0 < dist < 11.2  # Should be ~11.1m


class TestRaycasting:
    """Test raycast target GPS calculation."""

    def test_straight_down_raycast(self):
        """Gimbal pointing straight down at 50m altitude."""
        drone = DroneState(
            gps=GPSCoordinate(lat=45.5, lon=-73.5, alt=50.0),
            heading_deg=0.0,
            gimbal_pitch_deg=-90.0,  # Straight down
            lidar_distance_m=50.0,
        )
        target = raycast_target_gps(drone)
        # Should be directly below drone
        assert abs(target.lat - 45.5) < 0.00001
        assert abs(target.lon - (-73.5)) < 0.00001

    def test_45_degree_north_raycast(self):
        """Gimbal at 45 degrees looking north."""
        drone = DroneState(
            gps=GPSCoordinate(lat=45.5, lon=-73.5, alt=50.0),
            heading_deg=0.0,  # North
            gimbal_pitch_deg=-45.0,  # 45 degrees down
            lidar_distance_m=70.71,  # sqrt(50^2 + 50^2)
        )
        target = raycast_target_gps(drone)
        # Should be ~50m north of drone
        offset = calculate_gps_offset_meters(drone.gps, target)
        assert 49 < offset.north < 51  # ~50m north
        assert abs(offset.east) < 1  # No east/west offset

    def test_east_heading_raycast(self):
        """Gimbal looking east."""
        drone = DroneState(
            gps=GPSCoordinate(lat=45.5, lon=-73.5, alt=50.0),
            heading_deg=90.0,  # East
            gimbal_pitch_deg=-45.0,
            lidar_distance_m=70.71,
        )
        target = raycast_target_gps(drone)
        offset = calculate_gps_offset_meters(drone.gps, target)
        assert abs(offset.north) < 1  # No north/south offset
        assert 49 < offset.east < 51  # ~50m east


class TestOffsetGPSByMeters:
    """Test GPS coordinate offset calculation."""

    def test_north_offset(self):
        """Adding north offset should increase latitude."""
        origin = GPSCoordinate(lat=45.5, lon=-73.5, alt=100.0)
        offset = NEDOffset(north=100.0, east=0.0, down=0.0)
        target = offset_gps_by_meters(origin, offset)
        assert target.lat > origin.lat
        assert abs(target.lon - origin.lon) < 0.00001

    def test_roundtrip(self):
        """Offset and calculate should be inverse operations."""
        origin = GPSCoordinate(lat=45.5, lon=-73.5)
        offset = NEDOffset(north=50.0, east=30.0)
        target = offset_gps_by_meters(origin, offset)
        calculated_offset = calculate_gps_offset_meters(origin, target)
        assert abs(calculated_offset.north - 50.0) < 0.1
        assert abs(calculated_offset.east - 30.0) < 0.1


class TestLandmarks:
    """Test landmark loading and matching."""

    def test_load_landmarks(self):
        """Test loading landmarks from config."""
        landmarks = load_landmarks()
        assert len(landmarks) > 0
        assert all(isinstance(lm, Landmark) for lm in landmarks)
        assert landmarks[0].name == "Red Car"

    def test_find_nearest_landmark(self):
        """Test finding nearest landmark."""
        landmarks = [
            Landmark(name="A", lat=45.5, lon=-73.5),
            Landmark(name="B", lat=45.5001, lon=-73.5001),
        ]
        # Target closer to A
        target = GPSCoordinate(lat=45.50001, lon=-73.50001)
        match = find_nearest_landmark(target, landmarks)
        assert match is not None
        assert match.landmark.name == "A"

    def test_find_nearest_landmark_empty_list(self):
        """Empty landmarks list should return None."""
        target = GPSCoordinate(lat=45.5, lon=-73.5)
        match = find_nearest_landmark(target, [])
        assert match is None


class TestFormatting:
    """Test text formatting functions."""

    def test_format_north_east(self):
        """Test north-east formatting."""
        offset = NEDOffset(north=1.5, east=0.5)
        text = format_direction_text(offset)
        assert "1.5m North" in text
        assert "0.5m East" in text

    def test_format_south_west(self):
        """Test south-west formatting."""
        offset = NEDOffset(north=-2.0, east=-1.0)
        text = format_direction_text(offset)
        assert "2.0m South" in text
        assert "1.0m West" in text

    def test_format_north_only(self):
        """Test north-only formatting (small east component)."""
        offset = NEDOffset(north=3.0, east=0.05)  # Below threshold
        text = format_direction_text(offset)
        assert "3.0m North" in text
        assert "East" not in text


class TestTask1Integration:
    """Integration tests for Task 1 workflow."""

    def test_identify_target_relative_to_landmark(self):
        """Test full Task 1 workflow."""
        # Drone at known position looking at target near Red Car
        drone = DroneState(
            gps=GPSCoordinate(lat=45.5, lon=-73.5, alt=50.0),
            heading_deg=0.0,
            gimbal_pitch_deg=-90.0,  # Straight down
            lidar_distance_m=50.0,
        )
        result = identify_target_relative_to_landmark(drone)
        assert result is not None
        assert result.target_gps is not None
        assert result.nearest_landmark is not None
        assert "Target is" in result.formatted_text
        assert result.nearest_landmark.landmark.name in result.formatted_text

    def test_get_target_description(self):
        """Test convenience function."""
        text = get_target_description(
            lat=45.5,
            lon=-73.5,
            alt=50.0,
            heading_deg=0.0,
            gimbal_pitch_deg=-90.0,
            lidar_distance_m=50.0,
        )
        assert "Target is" in text


class TestBearing:
    """Test bearing calculations."""

    def test_bearing_north(self):
        """North bearing should be ~0 degrees."""
        origin = GPSCoordinate(lat=45.5, lon=-73.5)
        target = GPSCoordinate(lat=45.6, lon=-73.5)
        bearing = bearing_between_points(origin, target)
        assert abs(bearing - 0) < 1 or abs(bearing - 360) < 1

    def test_bearing_east(self):
        """East bearing should be ~90 degrees."""
        origin = GPSCoordinate(lat=45.5, lon=-73.5)
        target = GPSCoordinate(lat=45.5, lon=-73.4)
        bearing = bearing_between_points(origin, target)
        assert abs(bearing - 90) < 1

    def test_bearing_south(self):
        """South bearing should be ~180 degrees."""
        origin = GPSCoordinate(lat=45.5, lon=-73.5)
        target = GPSCoordinate(lat=45.4, lon=-73.5)
        bearing = bearing_between_points(origin, target)
        assert abs(bearing - 180) < 1


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
