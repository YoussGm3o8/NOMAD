"""
Inter-Process Communication (IPC) module for NOMAD.

Provides ZeroMQ-based communication between Orchestrator and Vision processes.
Uses PUB/SUB pattern for decoupled, reliable messaging.

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

from __future__ import annotations

import json
import logging
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Callable

import zmq

logger = logging.getLogger(__name__)


# Default IPC endpoints
DEFAULT_VISION_HEARTBEAT_ENDPOINT = "tcp://127.0.0.1:5555"
DEFAULT_VISION_DATA_ENDPOINT = "tcp://127.0.0.1:5556"


@dataclass(frozen=True, slots=True)
class IPCMessage:
    """Standard IPC message format."""

    msg_type: str
    timestamp: str
    data: dict[str, Any] = field(default_factory=dict)

    def to_json(self) -> str:
        """Serialize to JSON string."""
        return json.dumps({
            "msg_type": self.msg_type,
            "timestamp": self.timestamp,
            "data": self.data,
        })

    @classmethod
    def from_json(cls, json_str: str) -> "IPCMessage":
        """Deserialize from JSON string."""
        d = json.loads(json_str)
        return cls(
            msg_type=d["msg_type"],
            timestamp=d["timestamp"],
            data=d.get("data", {}),
        )

    @classmethod
    def heartbeat(cls, process_name: str = "vision") -> "IPCMessage":
        """Create a heartbeat message."""
        return cls(
            msg_type="HEARTBEAT",
            timestamp=datetime.now(timezone.utc).isoformat(),
            data={"process": process_name},
        )


class ZMQPublisher:
    """
    ZeroMQ Publisher for sending messages.

    Used by Vision process to broadcast heartbeats and detection results.
    """

    def __init__(
        self,
        endpoint: str = DEFAULT_VISION_HEARTBEAT_ENDPOINT,
        topic: str = "",
    ) -> None:
        """
        Initialize the publisher.

        Args:
            endpoint: ZMQ endpoint to bind to
            topic: Topic prefix for messages (empty = all)
        """
        self.endpoint = endpoint
        self.topic = topic
        self._context: Any = None  # zmq.Context
        self._socket: Any = None  # zmq.Socket

    def start(self) -> None:
        """Start the publisher and bind to endpoint."""
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.PUB)
        self._socket.bind(self.endpoint)
        # Allow time for subscribers to connect
        time.sleep(0.1)
        logger.info(f"ZMQ Publisher bound to {self.endpoint}")

    def stop(self) -> None:
        """Stop the publisher and cleanup."""
        if self._socket:
            self._socket.close()
            self._socket = None
        if self._context:
            self._context.term()
            self._context = None
        logger.info("ZMQ Publisher stopped")

    def send(self, message: IPCMessage) -> None:
        """
        Send a message.

        Args:
            message: IPCMessage to send
        """
        if not self._socket:
            raise RuntimeError("Publisher not started")

        payload = f"{self.topic}{message.to_json()}"
        self._socket.send_string(payload)

    def send_heartbeat(self, process_name: str = "vision") -> None:
        """Send a heartbeat message."""
        self.send(IPCMessage.heartbeat(process_name))

    def __enter__(self) -> "ZMQPublisher":
        self.start()
        return self

    def __exit__(self, *args) -> None:
        self.stop()


class ZMQSubscriber:
    """
    ZeroMQ Subscriber for receiving messages.

    Used by Orchestrator to receive heartbeats and data from Vision process.
    """

    def __init__(
        self,
        endpoint: str = DEFAULT_VISION_HEARTBEAT_ENDPOINT,
        topic: str = "",
        timeout_ms: int = 1000,
    ) -> None:
        """
        Initialize the subscriber.

        Args:
            endpoint: ZMQ endpoint to connect to
            topic: Topic filter (empty = all)
            timeout_ms: Receive timeout in milliseconds
        """
        self.endpoint = endpoint
        self.topic = topic
        self.timeout_ms = timeout_ms
        self._context: Any = None  # zmq.Context
        self._socket: Any = None  # zmq.Socket

    def start(self) -> None:
        """Start the subscriber and connect to endpoint."""
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.SUB)
        self._socket.connect(self.endpoint)
        self._socket.setsockopt_string(zmq.SUBSCRIBE, self.topic)
        self._socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        logger.info(f"ZMQ Subscriber connected to {self.endpoint}")

    def stop(self) -> None:
        """Stop the subscriber and cleanup."""
        if self._socket:
            self._socket.close()
            self._socket = None
        if self._context:
            self._context.term()
            self._context = None
        logger.info("ZMQ Subscriber stopped")

    def receive(self) -> IPCMessage | None:
        """
        Receive a message (blocking with timeout).

        Returns:
            IPCMessage if received, None on timeout
        """
        if not self._socket:
            raise RuntimeError("Subscriber not started")

        try:
            payload = self._socket.recv_string()
            # Remove topic prefix if present
            json_str = payload[len(self.topic):] if self.topic else payload
            return IPCMessage.from_json(json_str)
        except zmq.Again:
            # Timeout
            return None
        except json.JSONDecodeError as e:
            logger.error(f"Failed to decode message: {e}")
            return None

    def __enter__(self) -> "ZMQSubscriber":
        self.start()
        return self

    def __exit__(self, *args) -> None:
        self.stop()


class HeartbeatMonitor:
    """
    Monitors heartbeats from a process and triggers callback on timeout.

    Runs in a background thread, subscribing to ZMQ heartbeats.
    """

    def __init__(
        self,
        endpoint: str = DEFAULT_VISION_HEARTBEAT_ENDPOINT,
        timeout_seconds: float = 5.0,
        on_timeout: Callable[[], None] | None = None,
        on_heartbeat: Callable[[IPCMessage], None] | None = None,
    ) -> None:
        """
        Initialize the heartbeat monitor.

        Args:
            endpoint: ZMQ endpoint to subscribe to
            timeout_seconds: Time without heartbeat before triggering callback
            on_timeout: Callback when heartbeat timeout occurs
            on_heartbeat: Callback when heartbeat received
        """
        self.endpoint = endpoint
        self.timeout_seconds = timeout_seconds
        self.on_timeout = on_timeout
        self.on_heartbeat = on_heartbeat

        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._last_heartbeat: float = 0.0
        self._lock = threading.Lock()

    @property
    def last_heartbeat_age(self) -> float:
        """Seconds since last heartbeat."""
        with self._lock:
            if self._last_heartbeat == 0:
                return float("inf")
            return time.time() - self._last_heartbeat

    @property
    def is_alive(self) -> bool:
        """Check if heartbeats are being received within timeout."""
        return self.last_heartbeat_age < self.timeout_seconds

    def start(self) -> None:
        """Start the heartbeat monitor thread."""
        if self._thread and self._thread.is_alive():
            return

        self._stop_event.clear()
        self._last_heartbeat = time.time()  # Give initial grace period
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        logger.info(f"Heartbeat monitor started (timeout: {self.timeout_seconds}s)")

    def stop(self) -> None:
        """Stop the heartbeat monitor thread."""
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        logger.info("Heartbeat monitor stopped")

    def _run(self) -> None:
        """Main monitoring loop."""
        subscriber = ZMQSubscriber(
            endpoint=self.endpoint,
            timeout_ms=1000,  # Check every second
        )

        try:
            subscriber.start()

            while not self._stop_event.is_set():
                msg = subscriber.receive()

                if msg and msg.msg_type == "HEARTBEAT":
                    with self._lock:
                        self._last_heartbeat = time.time()
                    if self.on_heartbeat:
                        self.on_heartbeat(msg)
                    logger.debug(f"Heartbeat received: {msg.data}")

                # Check for timeout
                if self.last_heartbeat_age > self.timeout_seconds:
                    logger.warning(
                        f"Heartbeat timeout! Last: {self.last_heartbeat_age:.1f}s ago"
                    )
                    if self.on_timeout:
                        self.on_timeout()
                    # Reset to avoid repeated callbacks
                    with self._lock:
                        self._last_heartbeat = time.time()

        finally:
            subscriber.stop()
