"""
Task 2 (Extinguish Mission) Logic for NOMAD.

Handles stateful target memory (Exclusion Map) and gimbal PID control
for indoor autonomous fire extinguishing operations.

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import NamedTuple


class Point3D(NamedTuple):
    """3D coordinate point."""

    x: float
    y: float
    z: float

    def distance_to(self, other: "Point3D") -> float:
        """Calculate Euclidean distance to another point."""
        return math.sqrt(
            (self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2
        )


@dataclass
class ExclusionMapEntry:
    """Entry in the exclusion map with metadata."""

    point: Point3D
    timestamp: float = field(default_factory=time.time)
    hit_count: int = 1
    label: str = ""


class ExclusionMap:
    """
    Tracks extinguished/hit targets to avoid re-targeting.

    Stores 3D coordinates of targets that have been processed,
    allowing new targets to be checked against known positions.
    """

    def __init__(self) -> None:
        """Initialize empty exclusion map."""
        self._entries: list[ExclusionMapEntry] = []

    @property
    def count(self) -> int:
        """Number of tracked targets."""
        return len(self._entries)

    @property
    def entries(self) -> list[ExclusionMapEntry]:
        """Get all entries (read-only view)."""
        return list(self._entries)

    def add_target(
        self,
        x: float,
        y: float,
        z: float,
        label: str = "",
        merge_threshold: float = 0.5,
    ) -> bool:
        """
        Add a target to the exclusion map.

        If a target is within merge_threshold of an existing entry,
        increments the hit count instead of adding a new entry.

        Args:
            x: X coordinate
            y: Y coordinate
            z: Z coordinate
            label: Optional label for the target
            merge_threshold: Distance threshold for merging (meters)

        Returns:
            True if new entry added, False if merged with existing
        """
        point = Point3D(x, y, z)

        # Check for existing nearby target
        for entry in self._entries:
            if entry.point.distance_to(point) < merge_threshold:
                entry.hit_count += 1
                entry.timestamp = time.time()
                return False

        # Add new entry
        self._entries.append(
            ExclusionMapEntry(
                point=point,
                label=label,
            )
        )
        return True

    def is_target_extinguished(
        self,
        x: float,
        y: float,
        z: float,
        threshold: float = 0.5,
    ) -> bool:
        """
        Check if a target position is already in the exclusion map.

        Args:
            x: X coordinate to check
            y: Y coordinate to check
            z: Z coordinate to check
            threshold: Distance threshold in meters (default 0.5m = 50cm)

        Returns:
            True if target is within threshold of any stored point

        Example:
            >>> emap = ExclusionMap()
            >>> emap.add_target(1.0, 2.0, 3.0)
            True
            >>> emap.is_target_extinguished(1.1, 2.0, 3.0, threshold=0.5)
            True
            >>> emap.is_target_extinguished(5.0, 5.0, 5.0, threshold=0.5)
            False
        """
        point = Point3D(x, y, z)

        for entry in self._entries:
            if entry.point.distance_to(point) <= threshold:
                return True

        return False

    def find_nearest(
        self,
        x: float,
        y: float,
        z: float,
    ) -> tuple[ExclusionMapEntry | None, float]:
        """
        Find the nearest entry to a given point.

        Args:
            x: X coordinate
            y: Y coordinate
            z: Z coordinate

        Returns:
            Tuple of (nearest entry, distance). (None, inf) if map is empty.
        """
        if not self._entries:
            return None, float("inf")

        point = Point3D(x, y, z)
        nearest: ExclusionMapEntry | None = None
        min_dist = float("inf")

        for entry in self._entries:
            dist = entry.point.distance_to(point)
            if dist < min_dist:
                min_dist = dist
                nearest = entry

        return nearest, min_dist

    def clear(self) -> int:
        """
        Clear all entries from the exclusion map.

        Returns:
            Number of entries that were cleared
        """
        count = len(self._entries)
        self._entries.clear()
        return count

    def remove_by_index(self, index: int) -> ExclusionMapEntry | None:
        """
        Remove an entry by index.

        Args:
            index: Index of entry to remove

        Returns:
            Removed entry or None if index invalid
        """
        if 0 <= index < len(self._entries):
            return self._entries.pop(index)
        return None

    def to_dict(self) -> dict:
        """Serialize to dictionary."""
        return {
            "count": self.count,
            "entries": [
                {
                    "x": e.point.x,
                    "y": e.point.y,
                    "z": e.point.z,
                    "timestamp": e.timestamp,
                    "hit_count": e.hit_count,
                    "label": e.label,
                }
                for e in self._entries
            ],
        }

    @classmethod
    def from_dict(cls, data: dict) -> "ExclusionMap":
        """Deserialize from dictionary."""
        emap = cls()
        for entry in data.get("entries", []):
            emap._entries.append(
                ExclusionMapEntry(
                    point=Point3D(entry["x"], entry["y"], entry["z"]),
                    timestamp=entry.get("timestamp", time.time()),
                    hit_count=entry.get("hit_count", 1),
                    label=entry.get("label", ""),
                )
            )
        return emap


@dataclass
class PIDState:
    """Internal state for PID controller."""

    last_error: float = 0.0
    integral: float = 0.0
    last_time: float = field(default_factory=time.time)


class GimbalPID:
    """
    PID controller for gimbal tracking.

    Controls pitch and yaw to track targets in the camera frame.
    Uses separate PID controllers for each axis.
    """

    def __init__(
        self,
        kp: float = 0.5,
        ki: float = 0.1,
        kd: float = 0.05,
        max_velocity: float = 30.0,
        integral_limit: float = 50.0,
    ) -> None:
        """
        Initialize the gimbal PID controller.

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            max_velocity: Maximum velocity output (degrees/sec)
            integral_limit: Anti-windup limit for integral term
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_velocity = max_velocity
        self.integral_limit = integral_limit

        # Separate state for pitch and yaw axes
        self._pitch_state = PIDState()
        self._yaw_state = PIDState()

    def update(
        self,
        error_x: float,
        error_y: float,
    ) -> tuple[float, float]:
        """
        Calculate pitch and yaw velocity corrections.

        Args:
            error_x: Horizontal error (pixels or normalized, positive = right)
            error_y: Vertical error (pixels or normalized, positive = down)

        Returns:
            Tuple of (pitch_velocity, yaw_velocity) in degrees/sec
            - pitch_velocity: Positive = tilt up, Negative = tilt down
            - yaw_velocity: Positive = pan right, Negative = pan left

        Example:
            >>> pid = GimbalPID(kp=0.5, ki=0.1, kd=0.05)
            >>> pitch_vel, yaw_vel = pid.update(error_x=10.0, error_y=-5.0)
            >>> print(f"Pitch: {pitch_vel:.2f} deg/s, Yaw: {yaw_vel:.2f} deg/s")
        """
        current_time = time.time()

        # Calculate pitch correction (vertical error -> pitch)
        # Negative error_y (target above center) -> positive pitch (tilt up)
        pitch_velocity = self._compute_pid(
            error=-error_y,  # Invert: positive error_y -> negative pitch
            state=self._pitch_state,
            current_time=current_time,
        )

        # Calculate yaw correction (horizontal error -> yaw)
        # Positive error_x (target right of center) -> positive yaw (pan right)
        yaw_velocity = self._compute_pid(
            error=error_x,
            state=self._yaw_state,
            current_time=current_time,
        )

        return pitch_velocity, yaw_velocity

    def _compute_pid(
        self,
        error: float,
        state: PIDState,
        current_time: float,
    ) -> float:
        """
        Compute PID output for a single axis.

        Args:
            error: Current error value
            state: PID state for this axis
            current_time: Current timestamp

        Returns:
            Velocity correction (clamped to max_velocity)
        """
        dt = current_time - state.last_time
        if dt <= 0:
            dt = 0.01  # Minimum dt to avoid division by zero

        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup
        state.integral += error * dt
        state.integral = max(-self.integral_limit, min(self.integral_limit, state.integral))
        i_term = self.ki * state.integral

        # Derivative term (on error, not on setpoint)
        derivative = (error - state.last_error) / dt
        d_term = self.kd * derivative

        # Update state
        state.last_error = error
        state.last_time = current_time

        # Calculate output
        output = p_term + i_term + d_term

        # Clamp to maximum velocity
        return max(-self.max_velocity, min(self.max_velocity, output))

    def reset(self) -> None:
        """Reset PID controller state."""
        self._pitch_state = PIDState()
        self._yaw_state = PIDState()

    def set_gains(
        self,
        kp: float | None = None,
        ki: float | None = None,
        kd: float | None = None,
    ) -> None:
        """
        Update PID gains.

        Args:
            kp: New proportional gain (or None to keep current)
            ki: New integral gain (or None to keep current)
            kd: New derivative gain (or None to keep current)
        """
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd


# Global gimbal PID instance (can be configured at runtime)
_gimbal_pid: GimbalPID | None = None


def get_gimbal_pid() -> GimbalPID:
    """Get or create the global gimbal PID controller."""
    global _gimbal_pid
    if _gimbal_pid is None:
        _gimbal_pid = GimbalPID()
    return _gimbal_pid


def reset_gimbal_pid(
    kp: float = 0.5,
    ki: float = 0.1,
    kd: float = 0.05,
) -> GimbalPID:
    """Reset and reconfigure the global gimbal PID controller."""
    global _gimbal_pid
    _gimbal_pid = GimbalPID(kp=kp, ki=ki, kd=kd)
    return _gimbal_pid
