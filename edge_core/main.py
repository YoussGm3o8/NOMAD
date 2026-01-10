"""
NOMAD Edge Core - Main Entry Point.

Initializes and runs the drone-side services including:
- MAVLink interface for flight controller communication
- FastAPI server for REST/WebSocket API
- Task 1 landmark configuration
- Vision process with watchdog monitoring
- Detection subscriber for vision results

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

from __future__ import annotations

import atexit
import logging
import multiprocessing
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Any

import uvicorn

from .api import create_app
from .mavlink_interface import MavlinkService
from .state import StateManager
from .task1 import load_landmarks, Landmark
from .task2 import ExclusionMap
from .ipc import (
    HeartbeatMonitor,
    ZMQSubscriber,
    ZMQPublisher,
    IPCMessage,
    DEFAULT_VISION_HEARTBEAT_ENDPOINT,
)
from .vision_process import run_vision_process
from .models import DetectionInfo
from .visual_servoing import (
    Task2Controller,
    TargetTracker,
    TargetTrackerConfig,
    ServoMode,
)
from .time_manager import TimeSyncService, TimeSyncStatus

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("edge_core")

# Default detection endpoint (must match vision_process)
DEFAULT_DETECTION_ENDPOINT = "tcp://127.0.0.1:5556"

# Target detection endpoint for Task 2 (pH targets)
DEFAULT_TARGET_ENDPOINT = "tcp://127.0.0.1:5557"


def load_and_validate_landmarks(config_path: Path | str | None = None) -> list[Landmark]:
    """
    Load landmarks from configuration and validate.

    Args:
        config_path: Path to landmarks.json. Uses default if None.

    Returns:
        List of validated Landmark objects

    Raises:
        SystemExit: If landmarks cannot be loaded or are invalid
    """
    try:
        landmarks = load_landmarks(config_path)

        if not landmarks:
            logger.error("No landmarks found in configuration")
            sys.exit(1)

        logger.info(f"Loaded {len(landmarks)} landmarks:")
        for lm in landmarks:
            logger.info(f"  - {lm.name} ({lm.lat:.6f}, {lm.lon:.6f})")

        # Validate coordinates are reasonable
        for lm in landmarks:
            if not (-90 <= lm.lat <= 90):
                logger.error(f"Invalid latitude for {lm.name}: {lm.lat}")
                sys.exit(1)
            if not (-180 <= lm.lon <= 180):
                logger.error(f"Invalid longitude for {lm.name}: {lm.lon}")
                sys.exit(1)

        return landmarks

    except FileNotFoundError as e:
        logger.error(f"Landmarks configuration not found: {e}")
        sys.exit(1)
    except Exception as e:
        logger.error(f"Failed to load landmarks: {e}")
        sys.exit(1)


class VisionProcessManager:
    """
    Manages the Vision process with watchdog monitoring.

    Spawns vision process, monitors heartbeats, and restarts on failure.
    """

    def __init__(
        self,
        heartbeat_endpoint: str = DEFAULT_VISION_HEARTBEAT_ENDPOINT,
        watchdog_timeout: float = 5.0,
        max_restarts: int = 10,
    ) -> None:
        """
        Initialize the vision process manager.

        Args:
            heartbeat_endpoint: ZMQ endpoint for heartbeats
            watchdog_timeout: Seconds without heartbeat before restart
            max_restarts: Maximum restart attempts before giving up
        """
        self.heartbeat_endpoint = heartbeat_endpoint
        self.watchdog_timeout = watchdog_timeout
        self.max_restarts = max_restarts

        self._process: Any = None  # multiprocessing.Process (SpawnProcess on Windows)
        self._heartbeat_monitor: HeartbeatMonitor | None = None
        self._restart_count = 0
        self._lock = threading.Lock()
        self._running = False

    @property
    def is_running(self) -> bool:
        """Check if vision process is running."""
        with self._lock:
            return self._process is not None and self._process.is_alive()

    @property
    def restart_count(self) -> int:
        """Get current restart count."""
        with self._lock:
            return self._restart_count

    def start(self) -> None:
        """Start the vision process and watchdog monitor."""
        if self._running:
            return

        self._running = True
        self._spawn_vision_process()
        self._start_heartbeat_monitor()

        logger.info("Vision process manager started")

    def stop(self) -> None:
        """Stop the vision process and watchdog."""
        self._running = False

        # Stop heartbeat monitor
        if self._heartbeat_monitor:
            self._heartbeat_monitor.stop()
            self._heartbeat_monitor = None

        # Terminate vision process
        self._terminate_vision_process()

        logger.info("Vision process manager stopped")

    def _spawn_vision_process(self) -> None:
        """Spawn a new vision process."""
        with self._lock:
            if self._process and self._process.is_alive():
                logger.warning("Vision process already running")
                return

            logger.info(f"Spawning vision process (attempt #{self._restart_count + 1})")

            # Use spawn method for better isolation (especially on Windows)
            ctx = multiprocessing.get_context("spawn")
            self._process = ctx.Process(
                target=run_vision_process,
                args=(self.heartbeat_endpoint,),
                daemon=True,
                name="vision_process",
            )
            self._process.start()

            logger.info(f"Vision process started (PID: {self._process.pid})")

    def _terminate_vision_process(self) -> None:
        """Terminate the current vision process."""
        with self._lock:
            if not self._process:
                return

            if self._process.is_alive():
                logger.info(f"Terminating vision process (PID: {self._process.pid})")
                self._process.terminate()

                # Wait for graceful shutdown
                self._process.join(timeout=2.0)

                # Force kill if still alive
                if self._process.is_alive():
                    logger.warning("Vision process didn't terminate, killing...")
                    self._process.kill()
                    self._process.join(timeout=1.0)

            self._process = None

    def _on_heartbeat_timeout(self) -> None:
        """Handle heartbeat timeout - restart vision process."""
        if not self._running:
            return

        with self._lock:
            self._restart_count += 1

            if self._restart_count > self.max_restarts:
                logger.error(
                    f"Vision process exceeded max restarts ({self.max_restarts}). "
                    "Manual intervention required."
                )
                return

        logger.warning(
            f"Vision process heartbeat timeout! "
            f"Restarting... (attempt {self._restart_count}/{self.max_restarts})"
        )

        self._terminate_vision_process()

        # Brief delay before restart
        time.sleep(0.5)

        self._spawn_vision_process()

    def _on_heartbeat_received(self, msg: Any) -> None:
        """Handle heartbeat received."""
        # Reset restart count on successful heartbeat
        with self._lock:
            if self._restart_count > 0:
                logger.info("Vision process recovered, resetting restart counter")
                self._restart_count = 0

    def _start_heartbeat_monitor(self) -> None:
        """Start the heartbeat monitor."""
        self._heartbeat_monitor = HeartbeatMonitor(
            endpoint=self.heartbeat_endpoint,
            timeout_seconds=self.watchdog_timeout,
            on_timeout=self._on_heartbeat_timeout,
            on_heartbeat=self._on_heartbeat_received,
        )
        self._heartbeat_monitor.start()


class DetectionSubscriber:
    """
    Subscribes to detection messages from the vision process.

    Updates SystemState with current detection information.
    """

    def __init__(
        self,
        state_manager: StateManager,
        endpoint: str = DEFAULT_DETECTION_ENDPOINT,
        detection_timeout: float = 1.0,
    ) -> None:
        """
        Initialize detection subscriber.

        Args:
            state_manager: StateManager to update with detections
            endpoint: ZMQ endpoint to subscribe to
            detection_timeout: Seconds after which detection is considered stale
        """
        self._state_manager = state_manager
        self._endpoint = endpoint
        self._detection_timeout = detection_timeout
        self._subscriber: ZMQSubscriber | None = None
        self._running = False
        self._thread: threading.Thread | None = None
        self._last_detection_time = 0.0

    def start(self) -> None:
        """Start the detection subscriber thread."""
        if self._running:
            return

        self._running = True
        self._subscriber = ZMQSubscriber(
            endpoint=self._endpoint,
            topic="detection",
        )
        self._subscriber.start()

        self._thread = threading.Thread(
            target=self._subscribe_loop,
            daemon=True,
            name="detection_subscriber",
        )
        self._thread.start()

        logger.info(f"Detection subscriber started on {self._endpoint}")

    def stop(self) -> None:
        """Stop the detection subscriber."""
        self._running = False

        if self._subscriber:
            self._subscriber.stop()
            self._subscriber = None

        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

        logger.info("Detection subscriber stopped")

    def _subscribe_loop(self) -> None:
        """Main subscription loop - runs in background thread."""
        while self._running and self._subscriber:
            try:
                message = self._subscriber.receive()

                if message is not None:
                    self._handle_detection(message)

                # Check for stale detections
                self._check_detection_timeout()

            except Exception as e:
                logger.error(f"Detection subscriber error: {e}")
                time.sleep(0.1)

    def _handle_detection(self, message: IPCMessage) -> None:
        """Handle incoming detection message."""
        try:
            data = message.data
            if not data:
                return

            # Create DetectionInfo from message
            detection = DetectionInfo.from_dict(data)

            # Update state with detection
            self._state_manager.update_state(
                target_visible=True,
                current_detection=detection,
            )

            self._last_detection_time = time.time()

            # Log significant detections
            if detection.confidence >= 0.7:
                logger.debug(
                    f"Detection: {detection.class_name} "
                    f"({detection.confidence:.2f})"
                )

        except Exception as e:
            logger.error(f"Failed to handle detection: {e}")

    def _check_detection_timeout(self) -> None:
        """Clear detection if no recent detections."""
        if self._last_detection_time == 0.0:
            return

        elapsed = time.time() - self._last_detection_time
        if elapsed > self._detection_timeout:
            # Clear target_visible flag
            current_state = self._state_manager.get_state()
            if current_state.target_visible:
                self._state_manager.update_state(
                    target_visible=False,
                    current_detection=None,
                )
                logger.debug("Detection timeout - target no longer visible")

            self._last_detection_time = 0.0


class Task2Subscriber:
    """
    Subscribes to TARGET_DETECTED messages for Task 2 visual servoing.
    
    Processes pH target detections and forwards to Task2Controller
    for autonomous tracking and payload dispensing.
    """
    
    def __init__(
        self,
        task2_controller: Task2Controller,
        endpoint: str = DEFAULT_TARGET_ENDPOINT,
    ) -> None:
        """
        Initialize Task 2 subscriber.
        
        Args:
            task2_controller: Controller for visual servoing
            endpoint: ZMQ endpoint to subscribe to
        """
        self._controller = task2_controller
        self._endpoint = endpoint
        self._subscriber: ZMQSubscriber | None = None
        self._running = False
        self._thread: threading.Thread | None = None
        self._control_thread: threading.Thread | None = None
    
    def start(self) -> None:
        """Start the Task 2 subscriber threads."""
        if self._running:
            return
        
        self._running = True
        
        # Start ZMQ subscriber
        self._subscriber = ZMQSubscriber(
            endpoint=self._endpoint,
            topic="target",  # Subscribe to TARGET_DETECTED topic
        )
        self._subscriber.start()
        
        # Start message processing thread
        self._thread = threading.Thread(
            target=self._subscribe_loop,
            daemon=True,
            name="task2_subscriber",
        )
        self._thread.start()
        
        # Start control loop thread (runs at higher frequency)
        self._control_thread = threading.Thread(
            target=self._control_loop,
            daemon=True,
            name="task2_control",
        )
        self._control_thread.start()
        
        logger.info(f"Task 2 subscriber started on {self._endpoint}")
    
    def stop(self) -> None:
        """Stop the Task 2 subscriber."""
        self._running = False
        
        if self._subscriber:
            self._subscriber.stop()
            self._subscriber = None
        
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        
        if self._control_thread:
            self._control_thread.join(timeout=2.0)
            self._control_thread = None
        
        # Deactivate controller (stops commands)
        self._controller.deactivate()
        
        logger.info("Task 2 subscriber stopped")
    
    def activate(self) -> None:
        """Activate Task 2 autonomous mode."""
        self._controller.activate()
        logger.info("Task 2 autonomous mode activated")
    
    def deactivate(self) -> None:
        """Deactivate Task 2 autonomous mode."""
        self._controller.deactivate()
        logger.info("Task 2 autonomous mode deactivated")
    
    @property
    def is_active(self) -> bool:
        """Check if Task 2 is active."""
        return self._controller.is_active
    
    def _subscribe_loop(self) -> None:
        """Main subscription loop - processes target detections."""
        while self._running and self._subscriber:
            try:
                message = self._subscriber.receive()
                
                if message is not None:
                    self._handle_target_detection(message)
                
            except Exception as e:
                logger.error(f"Task 2 subscriber error: {e}")
                time.sleep(0.1)
    
    def _handle_target_detection(self, message: IPCMessage) -> None:
        """Handle incoming TARGET_DETECTED message."""
        try:
            data = message.data
            if not data:
                return
            
            # Extract 3D position if available (from VIO/depth)
            target_position = None
            if "position" in data:
                pos = data["position"]
                target_position = (
                    pos.get("x", 0.0),
                    pos.get("y", 0.0),
                    pos.get("z", 0.0),
                )
            
            # Process detection through controller
            self._controller.process_detection(
                detection_data=data,
                target_position=target_position,
            )
            
            # Log tracking state changes
            state = self._controller.tracker.state
            if state.is_fine_aligned:
                logger.info(
                    f"Target ALIGNED: {data.get('class_name', 'unknown')} "
                    f"(conf={data.get('confidence', 0):.2f})"
                )
            elif state.is_aligned:
                error_str = f"{state.error.magnitude:.3f}" if state.error is not None else "N/A"
                logger.debug(
                    f"Target tracking: {data.get('class_name', 'unknown')} "
                    f"(error={error_str})"
                )
            
        except Exception as e:
            logger.error(f"Failed to handle target detection: {e}")
    
    def _control_loop(self) -> None:
        """High-frequency control loop for smooth tracking."""
        control_rate = 50  # Hz
        control_period = 1.0 / control_rate
        
        while self._running:
            try:
                start_time = time.time()
                
                # Update controller (handles timeouts, dispense timing, etc.)
                self._controller.update()
                
                # Sleep for remaining time in period
                elapsed = time.time() - start_time
                sleep_time = control_period - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
            except Exception as e:
                logger.error(f"Task 2 control loop error: {e}")
                time.sleep(0.1)


# Initialize services
state_manager = StateManager.instance()
mavlink_service = MavlinkService(state_manager)
vision_manager: VisionProcessManager | None = None
detection_subscriber: DetectionSubscriber | None = None

# Time synchronization service
time_sync_service: TimeSyncService | None = None

# Task 2 components
exclusion_map: ExclusionMap = ExclusionMap()
task2_controller: Task2Controller | None = None
task2_subscriber: Task2Subscriber | None = None

# Load landmarks on module import (for startup validation)
_landmarks: list[Landmark] = []


def get_app():
    """Get or create the FastAPI application."""
    global _landmarks
    if not _landmarks:
        _landmarks = load_and_validate_landmarks()
    return create_app(state_manager, landmarks=_landmarks)


# Create app instance for uvicorn
app = get_app()


def cleanup() -> None:
    """Cleanup on shutdown."""
    global vision_manager, detection_subscriber, task2_subscriber, time_sync_service
    logger.info("Shutting down Edge Core...")

    # Stop Task 2 components first
    if task2_subscriber:
        task2_subscriber.stop()
        logger.info("Task 2 subscriber stopped")

    if detection_subscriber:
        detection_subscriber.stop()

    if vision_manager:
        vision_manager.stop()

    # Stop time sync service
    if time_sync_service:
        time_sync_service.stop()
        logger.info("Time sync service stopped")

    mavlink_service.stop()
    logger.info("Cleanup complete")


# Register cleanup
atexit.register(cleanup)


def run(
    host: str = "0.0.0.0",
    port: int = 8000,
    log_level: str = "info",
    enable_vision: bool = True,
    enable_task2: bool = True,
    watchdog_timeout: float = 5.0,
    servo_mode: str = "gimbal",
) -> None:
    """
    Run the Edge Core server.

    Args:
        host: Host address to bind to
        port: Port number
        log_level: Logging level
        enable_vision: Whether to start vision process
        enable_task2: Whether to enable Task 2 autonomous control
        watchdog_timeout: Seconds before watchdog restarts vision
        servo_mode: Visual servoing mode ("gimbal" or "drone_body")
    """
    global vision_manager, detection_subscriber, task2_controller, task2_subscriber, time_sync_service

    logger.info("=" * 50)
    logger.info("NOMAD Edge Core Starting")
    logger.info("=" * 50)
    logger.info(f"Host: {host}:{port}")
    logger.info(f"Landmarks: {len(_landmarks)} configured")
    logger.info(f"Vision: {'enabled' if enable_vision else 'disabled'}")
    logger.info(f"Task 2: {'enabled' if enable_task2 else 'disabled'}")
    if enable_vision:
        logger.info(f"Watchdog timeout: {watchdog_timeout}s")
    if enable_task2:
        logger.info(f"Servo mode: {servo_mode}")
    logger.info("=" * 50)

    # Initialize time synchronization service
    def on_time_sync_change(status: TimeSyncStatus) -> None:
        """Callback when time sync status changes."""
        if status.synced:
            logger.info(f"Time synchronized via {status.source.name}")
        else:
            logger.warning(f"Time synchronization lost (offset: {status.offset_seconds:.3f}s)")

    time_sync_service = TimeSyncService(
        state_manager=state_manager,
        on_sync_change=on_time_sync_change,
    )
    time_sync_service.start()
    
    # Log initial sync status
    sync_status = time_sync_service.status
    if sync_status.synced:
        logger.info(f"Time sync: {sync_status.source.name}")
    else:
        logger.warning("Time not synchronized - will use GPS time when available")

    # Start MAVLink service
    mavlink_service.set_time_sync_service(time_sync_service)
    mavlink_service.start()
    logger.info("MAVLink service started")

    # Start health status broadcast (every 2 seconds)
    mavlink_service.start_health_broadcast(interval=2.0)
    logger.info("Health status broadcast started (2s interval)")

    # Start vision process with watchdog
    if enable_vision:
        vision_manager = VisionProcessManager(
            watchdog_timeout=watchdog_timeout,
        )
        vision_manager.start()
        logger.info("Vision process manager started")

        # Start detection subscriber
        detection_subscriber = DetectionSubscriber(
            state_manager=state_manager,
            endpoint=DEFAULT_DETECTION_ENDPOINT,
        )
        detection_subscriber.start()
        logger.info("Detection subscriber started")

    # Initialize Task 2 components
    if enable_task2:
        # Configure tracker based on servo mode
        tracker_config = TargetTrackerConfig(
            mode=ServoMode.GIMBAL if servo_mode == "gimbal" else ServoMode.DRONE_BODY,
        )
        
        # Create Task 2 controller
        task2_controller = Task2Controller(
            mavlink_service=mavlink_service,
            exclusion_map=exclusion_map,
            tracker_config=tracker_config,
        )
        
        # Create and start Task 2 subscriber
        task2_subscriber = Task2Subscriber(
            task2_controller=task2_controller,
            endpoint=DEFAULT_TARGET_ENDPOINT,
        )
        task2_subscriber.start()
        logger.info("Task 2 controller initialized (inactive by default)")
        logger.info("  - Use API to activate: POST /api/task2/activate")

    # Handle shutdown signals
    def signal_handler(signum: int, frame: Any) -> None:
        logger.info(f"Received signal {signum}")
        cleanup()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Run FastAPI server
    try:
        uvicorn.run(app, host=host, port=port, log_level=log_level)
    finally:
        cleanup()


def main() -> None:
    """
    CLI entry point with argument parsing.
    
    Supports --sim flag for simulation mode.
    """
    import argparse
    import os
    
    parser = argparse.ArgumentParser(
        description="NOMAD Edge Core - Drone-side processing system",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    
    # Server arguments
    parser.add_argument(
        "--host",
        type=str,
        default="0.0.0.0",
        help="Host address to bind to",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8000,
        help="Port number for REST API",
    )
    parser.add_argument(
        "--log-level",
        type=str,
        default="info",
        choices=["debug", "info", "warning", "error"],
        help="Logging level",
    )
    
    # Vision arguments
    parser.add_argument(
        "--no-vision",
        action="store_true",
        help="Disable vision process",
    )
    parser.add_argument(
        "--watchdog-timeout",
        type=float,
        default=5.0,
        help="Vision watchdog timeout in seconds",
    )
    
    # Task 2 arguments
    parser.add_argument(
        "--no-task2",
        action="store_true",
        help="Disable Task 2 autonomous control",
    )
    parser.add_argument(
        "--servo-mode",
        type=str,
        default="gimbal",
        choices=["gimbal", "drone_body"],
        help="Visual servoing mode",
    )
    
    # Simulation mode
    parser.add_argument(
        "--sim",
        action="store_true",
        help="Enable simulation mode (mock hardware for development)",
    )
    
    args = parser.parse_args()
    
    # Set simulation mode environment variable before spawning subprocesses
    if args.sim:
        os.environ["NOMAD_SIM_MODE"] = "true"
        logger.info("=" * 50)
        logger.info("  SIMULATION MODE ENABLED")
        logger.info("  Using mock hardware for development testing")
        logger.info("=" * 50)
    
    # Run the server
    run(
        host=args.host,
        port=args.port,
        log_level=args.log_level,
        enable_vision=not args.no_vision,
        enable_task2=not args.no_task2,
        watchdog_timeout=args.watchdog_timeout,
        servo_mode=args.servo_mode,
    )


if __name__ == "__main__":
    main()

