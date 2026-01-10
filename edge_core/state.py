"""
NOMAD Edge Core - State Manager

Thread-safe singleton state manager for system telemetry and status.

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

from __future__ import annotations

import threading
from datetime import datetime, timezone
from typing import Any

from .models import SystemState


class StateManager:
    """
    Thread-safe singleton state manager for NOMAD Edge Core.

    Manages:
    - SystemState: Telemetry and sensor data (immutable snapshots)
    """

    _instance: "StateManager | None" = None
    _instance_lock = threading.Lock()

    def __init__(self) -> None:
        self._state = SystemState.default()
        self._lock = threading.RLock()

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
