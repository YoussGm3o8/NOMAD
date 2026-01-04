from __future__ import annotations

import threading
from datetime import datetime, timezone
from typing import Any, TYPE_CHECKING

from .models import SystemState

if TYPE_CHECKING:
    from .task2 import ExclusionMap


class StateManager:
    """
    Thread-safe singleton state manager for NOMAD Edge Core.

    Manages:
    - SystemState: Telemetry and sensor data (immutable snapshots)
    - ExclusionMap: Task 2 target memory (mutable, thread-safe access)
    """

    _instance: "StateManager | None" = None
    _instance_lock = threading.Lock()

    def __init__(self) -> None:
        self._state = SystemState.default()
        self._lock = threading.RLock()

        # Task 2: Exclusion map for extinguished targets
        # Lazy import to avoid circular dependency
        from .task2 import ExclusionMap

        self._exclusion_map = ExclusionMap()
        self._exclusion_map_lock = threading.Lock()

    @classmethod
    def instance(cls) -> "StateManager":
        if cls._instance is None:
            with cls._instance_lock:
                if cls._instance is None:
                    cls._instance = cls()
        return cls._instance

    @classmethod
    def reset_instance(cls) -> None:
        """Reset the singleton instance (for testing)."""
        with cls._instance_lock:
            cls._instance = None

    def get_state(self) -> SystemState:
        with self._lock:
            return self._state

    def update_state(self, **fields: Any) -> SystemState:
        with self._lock:
            data = self._state.model_dump()
            data.update(fields)
            if "timestamp" not in fields:
                data["timestamp"] = datetime.now(timezone.utc)
            self._state = SystemState(**data)
            return self._state

    # ==================== Exclusion Map Methods ====================

    @property
    def exclusion_map(self) -> "ExclusionMap":
        """Get the exclusion map instance."""
        return self._exclusion_map

    def is_target_extinguished(
        self,
        x: float,
        y: float,
        z: float,
        threshold: float = 0.5,
    ) -> bool:
        """
        Thread-safe check if target is already extinguished.

        Args:
            x, y, z: Target coordinates
            threshold: Distance threshold in meters

        Returns:
            True if target is within threshold of any stored point
        """
        with self._exclusion_map_lock:
            return self._exclusion_map.is_target_extinguished(x, y, z, threshold)

    def add_extinguished_target(
        self,
        x: float,
        y: float,
        z: float,
        label: str = "",
    ) -> bool:
        """
        Thread-safe add target to exclusion map.

        Args:
            x, y, z: Target coordinates
            label: Optional label

        Returns:
            True if new entry added, False if merged
        """
        with self._exclusion_map_lock:
            return self._exclusion_map.add_target(x, y, z, label)

    def clear_exclusion_map(self) -> int:
        """
        Thread-safe clear of exclusion map.

        Returns:
            Number of entries cleared
        """
        with self._exclusion_map_lock:
            return self._exclusion_map.clear()

    def get_exclusion_map_data(self) -> dict:
        """
        Thread-safe serialization of exclusion map.

        Returns:
            Dictionary representation of exclusion map
        """
        with self._exclusion_map_lock:
            return self._exclusion_map.to_dict()
