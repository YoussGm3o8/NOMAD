"""
Hardware Abstraction Layer (HAL) Mocks for NOMAD.

Provides simulated hardware for development and testing without
actual ZED camera, YOLO model, or Jetson hardware.

Enables full system testing on laptop/desktop environments.

Target: Python 3.13 | Development/Testing
"""

from __future__ import annotations

import logging
import math
import os
import random
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)


# ============================================================
# Environment Variable for Simulation Mode
# ============================================================

def is_sim_mode() -> bool:
    """Check if simulation mode is enabled via environment variable."""
    return os.environ.get("NOMAD_SIM_MODE", "").lower() in ("true", "1", "yes")


def set_sim_mode(enabled: bool = True) -> None:
    """Set simulation mode via environment variable."""
    os.environ["NOMAD_SIM_MODE"] = "true" if enabled else "false"


# ============================================================
# Mock ZED Camera
# ============================================================

@dataclass
class MockPose:
    """Simulated 6DOF pose data."""
    timestamp_ns: int
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    confidence: float
    tracking_state: str = "OK"
    
    @property
    def valid(self) -> bool:
        """Check if pose is valid."""
        return self.tracking_state == "OK"
    
    def to_dict(self) -> dict:
        """Convert to dictionary."""
        return {
            "timestamp_ns": self.timestamp_ns,
            "timestamp_us": self.timestamp_ns // 1000,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "roll": self.roll,
            "pitch": self.pitch,
            "yaw": self.yaw,
            "confidence": self.confidence,
            "tracking_state": self.tracking_state,
        }


class MockZedCamera:
    """
    Simulates a ZED 2i camera for development testing.
    
    Generates:
    - Synthetic pose data (drone flying in a circle pattern)
    - Blank or static test images
    - Simulated VIO tracking state
    
    Usage:
        camera = MockZedCamera()
        if camera.open():
            while True:
                if camera.grab():
                    pose = camera.get_pose_ned()
                    image = camera.get_image()
            camera.close()
    """
    
    def __init__(
        self,
        resolution: str = "HD720",
        fps: int = 30,
        enable_tracking: bool = True,
        circle_radius: float = 5.0,
        circle_period: float = 30.0,
        altitude: float = -2.0,
        image_path: str | Path | None = None,
    ) -> None:
        """
        Initialize mock ZED camera.
        
        Args:
            resolution: Simulated resolution (VGA, HD720, HD1080)
            fps: Simulated frame rate
            enable_tracking: Enable simulated VIO
            circle_radius: Radius of circular flight path (meters)
            circle_period: Time to complete one circle (seconds)
            altitude: Fixed altitude in NED (negative = up)
            image_path: Optional path to static test image
        """
        self.resolution = resolution
        self.fps = fps
        self.enable_tracking = enable_tracking
        self.circle_radius = circle_radius
        self.circle_period = circle_period
        self.altitude = altitude
        self.image_path = Path(image_path) if image_path else None
        
        # Derived parameters
        self._resolution_map = {
            "VGA": (672, 376),
            "HD720": (1280, 720),
            "HD1080": (1920, 1080),
            "HD2K": (2208, 1242),
        }
        self._width, self._height = self._resolution_map.get(resolution, (1280, 720))
        
        # State
        self._is_open = False
        self._tracking_enabled = False
        self._start_time: float = 0.0
        self._frame_count = 0
        self._last_grab_time: float = 0.0
        self._static_image: Any = None
        
        # Add some noise to make it realistic
        self._noise_position = 0.02  # meters
        self._noise_angle = 0.01  # radians
    
    @property
    def is_open(self) -> bool:
        """Check if camera is open."""
        return self._is_open
    
    @property
    def is_tracking(self) -> bool:
        """Check if tracking is enabled."""
        return self._tracking_enabled
    
    def open(self) -> bool:
        """
        Open the mock camera.
        
        Returns:
            True (always succeeds in simulation)
        """
        logger.info(f"[SIM] MockZedCamera opening: {self.resolution}@{self.fps}fps")
        
        self._is_open = True
        self._start_time = time.time()
        self._frame_count = 0
        
        if self.enable_tracking:
            self._tracking_enabled = True
            logger.info("[SIM] VIO tracking enabled (simulated)")
        
        # Load static image if provided
        if self.image_path and self.image_path.exists():
            try:
                import cv2
                self._static_image = cv2.imread(str(self.image_path))
                logger.info(f"[SIM] Loaded static test image: {self.image_path}")
            except Exception as e:
                logger.warning(f"[SIM] Failed to load test image: {e}")
        
        logger.info("[SIM] MockZedCamera opened successfully")
        return True
    
    def close(self) -> None:
        """Close the mock camera."""
        self._is_open = False
        self._tracking_enabled = False
        logger.info(f"[SIM] MockZedCamera closed. Frames: {self._frame_count}")
    
    def grab(self) -> bool:
        """
        Simulate frame grab (rate-limited to target FPS).
        
        Returns:
            True if frame available
        """
        if not self._is_open:
            return False
        
        # Rate limiting based on FPS
        current_time = time.time()
        min_interval = 1.0 / self.fps
        
        if current_time - self._last_grab_time < min_interval:
            # Sleep to maintain frame rate
            time.sleep(min_interval - (current_time - self._last_grab_time))
        
        self._last_grab_time = time.time()
        self._frame_count += 1
        return True
    
    def get_image(self) -> Any:
        """
        Get simulated camera image (BGRA format).
        
        Returns:
            numpy array (H, W, 4) in BGRA format
        """
        if not self._is_open:
            return None
        
        # Return static image if loaded
        if self._static_image is not None:
            # Convert BGR to BGRA
            import cv2
            bgra = cv2.cvtColor(self._static_image, cv2.COLOR_BGR2BGRA)
            # Resize to target resolution
            return cv2.resize(bgra, (self._width, self._height))
        
        # Generate synthetic test image
        # Create a gradient background with some visual markers
        image = np.zeros((self._height, self._width, 4), dtype=np.uint8)
        
        # Gradient background (gray)
        for y in range(self._height):
            value = int(50 + (y / self._height) * 100)
            image[y, :, 0] = value  # B
            image[y, :, 1] = value  # G
            image[y, :, 2] = value  # R
            image[y, :, 3] = 255    # A
        
        # Add crosshair at center
        cx, cy = self._width // 2, self._height // 2
        # Vertical line
        image[cy-50:cy+50, cx-1:cx+2, :3] = [0, 255, 0]
        # Horizontal line
        image[cy-1:cy+2, cx-50:cx+50, :3] = [0, 255, 0]
        
        # Add frame counter text (simple pixel pattern)
        # Just a visual indicator, not actual text rendering
        frame_indicator = self._frame_count % 100
        x_pos = 10 + frame_indicator
        image[10:15, x_pos:x_pos+5, :3] = [255, 255, 255]
        
        return image
    
    def get_pose_ned(self) -> MockPose | None:
        """
        Get simulated VIO pose in NED frame.
        
        Simulates a drone flying in a circle pattern.
        
        Returns:
            MockPose in NED coordinates, or None if tracking disabled
        """
        if not self._is_open or not self._tracking_enabled:
            return None
        
        # Calculate elapsed time
        elapsed = time.time() - self._start_time
        
        # Circular flight pattern
        angle = (elapsed / self.circle_period) * 2 * math.pi
        
        # Position (NED: North, East, Down)
        x = self.circle_radius * math.cos(angle)  # North
        y = self.circle_radius * math.sin(angle)  # East
        z = self.altitude  # Down (negative = above ground)
        
        # Add noise
        x += random.gauss(0, self._noise_position)
        y += random.gauss(0, self._noise_position)
        z += random.gauss(0, self._noise_position * 0.5)
        
        # Attitude (drone heading tangent to circle)
        yaw = angle + math.pi / 2  # Heading tangent to path
        roll = 0.0 + random.gauss(0, self._noise_angle)
        pitch = 0.0 + random.gauss(0, self._noise_angle)
        
        # Normalize yaw to [-pi, pi]
        while yaw > math.pi:
            yaw -= 2 * math.pi
        while yaw < -math.pi:
            yaw += 2 * math.pi
        
        # Timestamp
        timestamp_ns = int(time.time() * 1e9)
        
        # Confidence (simulate occasional tracking issues)
        confidence = 90.0 + random.gauss(0, 5)
        confidence = max(0, min(100, confidence))
        
        # Tracking state
        if confidence < 50:
            tracking_state = "SEARCHING"
        else:
            tracking_state = "OK"
        
        return MockPose(
            timestamp_ns=timestamp_ns,
            x=x,
            y=y,
            z=z,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            confidence=confidence,
            tracking_state=tracking_state,
        )
    
    def get_camera_info(self) -> dict:
        """Get simulated camera info."""
        return {
            "serial_number": "SIM-12345",
            "firmware_version": "1.0.0-sim",
            "camera_model": "MockZED",
            "resolution": self.resolution,
            "fps": self.fps,
        }
    
    def reset_tracking(self) -> bool:
        """Reset tracking to current position as origin."""
        self._start_time = time.time()
        logger.info("[SIM] Tracking reset")
        return True


# ============================================================
# Mock YOLO Detector
# ============================================================

@dataclass
class MockBoundingBox:
    """Simulated bounding box (normalized 0-1)."""
    x1: float
    y1: float
    x2: float
    y2: float
    
    @property
    def center_x(self) -> float:
        return (self.x1 + self.x2) / 2
    
    @property
    def center_y(self) -> float:
        return (self.y1 + self.y2) / 2
    
    @property
    def width(self) -> float:
        return self.x2 - self.x1
    
    @property
    def height(self) -> float:
        return self.y2 - self.y1


@dataclass
class MockDetection:
    """Simulated detection result."""
    class_id: int
    class_name: str
    confidence: float
    bbox: MockBoundingBox
    timestamp: float
    
    def to_dict(self) -> dict:
        """Convert to dictionary for JSON serialization."""
        return {
            "class_id": self.class_id,
            "class_name": self.class_name,
            "confidence": self.confidence,
            "bbox": {
                "x1": self.bbox.x1,
                "y1": self.bbox.y1,
                "x2": self.bbox.x2,
                "y2": self.bbox.y2,
            },
            "timestamp": self.timestamp,
        }


class MockYOLO:
    """
    Simulates YOLOv8 object detection for development testing.
    
    Generates random detections at configurable intervals.
    
    Usage:
        model = MockYOLO()
        detections = model.detect(frame)
    """
    
    # Default class names (COCO subset relevant to NOMAD)
    DEFAULT_CLASSES = {
        0: "person",
        1: "fire",
        2: "smoke", 
        3: "target",
        4: "marker",
        15: "cat",
        16: "dog",
        56: "chair",
    }
    
    def __init__(
        self,
        model_path: str = "mock_yolov8.pt",
        detection_probability: float = 0.3,
        detection_interval: float = 2.0,
        confidence_range: tuple[float, float] = (0.6, 0.95),
        target_classes: list[str] | None = None,
        class_names: dict[int, str] | None = None,
    ) -> None:
        """
        Initialize mock YOLO detector.
        
        Args:
            model_path: Fake model path (for logging)
            detection_probability: Probability of detection per frame
            detection_interval: Minimum seconds between detections
            confidence_range: (min, max) confidence for detections
            target_classes: Classes to detect (None = all)
            class_names: Custom class name mapping
        """
        self.model_path = model_path
        self.detection_probability = detection_probability
        self.detection_interval = detection_interval
        self.confidence_range = confidence_range
        self.target_classes = target_classes
        self.names = class_names or self.DEFAULT_CLASSES.copy()
        
        # State
        self._last_detection_time: float = 0.0
        self._detection_count = 0
        
        logger.info(f"[SIM] MockYOLO initialized: {model_path}")
        logger.info(f"[SIM] Detection probability: {detection_probability}")
        logger.info(f"[SIM] Classes: {list(self.names.values())}")
    
    def __call__(
        self,
        frame: Any,
        imgsz: int = 640,
        conf: float = 0.5,
        verbose: bool = False,
    ) -> list[Any]:
        """
        Run simulated detection on frame.
        
        Args:
            frame: Input image (numpy array)
            imgsz: Inference size (ignored in mock)
            conf: Confidence threshold
            verbose: Verbose output (ignored)
        
        Returns:
            List of mock result objects
        """
        current_time = time.time()
        
        # Check if enough time has passed since last detection
        if current_time - self._last_detection_time < self.detection_interval:
            return [MockResult([])]
        
        # Random chance of detection
        if random.random() > self.detection_probability:
            return [MockResult([])]
        
        # Generate random detection
        self._last_detection_time = current_time
        self._detection_count += 1
        
        # Pick random class
        if self.target_classes:
            # Filter to target classes
            valid_ids = [
                cid for cid, name in self.names.items()
                if name in self.target_classes
            ]
            if not valid_ids:
                valid_ids = list(self.names.keys())
        else:
            valid_ids = list(self.names.keys())
        
        class_id = random.choice(valid_ids)
        class_name = self.names.get(class_id, f"class_{class_id}")
        
        # Random confidence
        confidence = random.uniform(*self.confidence_range)
        if confidence < conf:
            return [MockResult([])]
        
        # Random bounding box (biased toward center)
        cx = random.gauss(0.5, 0.15)
        cy = random.gauss(0.5, 0.15)
        w = random.uniform(0.1, 0.3)
        h = random.uniform(0.1, 0.3)
        
        # Clamp to valid range
        cx = max(w/2, min(1-w/2, cx))
        cy = max(h/2, min(1-h/2, cy))
        
        x1 = cx - w/2
        y1 = cy - h/2
        x2 = cx + w/2
        y2 = cy + h/2
        
        # Get frame dimensions
        if frame is not None and hasattr(frame, 'shape'):
            h_px, w_px = frame.shape[:2]
        else:
            h_px, w_px = 720, 1280
        
        # Convert to pixel coordinates for mock boxes
        box = MockBoxes(
            xyxy=np.array([[x1 * w_px, y1 * h_px, x2 * w_px, y2 * h_px]]),
            conf=np.array([confidence]),
            cls=np.array([class_id]),
        )
        
        logger.info(
            f"[SIM] Detection #{self._detection_count}: "
            f"{class_name} ({confidence:.2f}) at ({cx:.2f}, {cy:.2f})"
        )
        
        return [MockResult([box])]


class MockBoxes:
    """Mock detection boxes (mimics ultralytics structure)."""
    
    def __init__(self, xyxy: np.ndarray, conf: np.ndarray, cls: np.ndarray):
        self._xyxy = xyxy
        self._conf = conf
        self._cls = cls
    
    @property
    def xyxy(self):
        return [MockTensor(row) for row in self._xyxy]
    
    @property
    def conf(self):
        return [MockTensor(c) for c in self._conf]
    
    @property
    def cls(self):
        return [MockTensor(c) for c in self._cls]
    
    def __len__(self):
        return len(self._xyxy)


class MockTensor:
    """Mock tensor that mimics PyTorch tensor behavior."""
    
    def __init__(self, data):
        self._data = np.array(data)
    
    def cpu(self):
        return self
    
    def numpy(self):
        return self._data


class MockResult:
    """Mock YOLO result object."""
    
    def __init__(self, boxes_list: list):
        self._boxes = boxes_list[0] if boxes_list else None
    
    @property
    def boxes(self):
        return self._boxes


# ============================================================
# Mock MAVLink Connection
# ============================================================

class MockMavlinkConnection:
    """
    Simulates a MAVLink connection for development testing.
    
    Generates synthetic telemetry without actual flight controller.
    """
    
    def __init__(self):
        self.target_system = 1
        self.target_component = 1
        self._is_armed = False
        self._mode = "STABILIZE"
        self._battery_voltage = 16.8
        self._last_message_time = time.time()
        
        # Create mock mav object
        self.mav = MockMav()
        
        logger.info("[SIM] MockMavlinkConnection initialized")
    
    def recv_match(self, type: list[str], blocking: bool = True, timeout: float = 0.2) -> Any:
        """Simulate receiving MAVLink messages."""
        current_time = time.time()
        
        # Simulate message arrival delay
        if blocking:
            time.sleep(min(timeout, 0.1))
        
        # Randomly select message type to return
        msg_type = random.choice(type)
        
        if msg_type == "HEARTBEAT":
            return MockMessage(
                type="HEARTBEAT",
                custom_mode=0,
                base_mode=128 if self._is_armed else 0,
            )
        elif msg_type == "SYS_STATUS":
            # Simulate battery drain
            self._battery_voltage = max(14.0, self._battery_voltage - 0.0001)
            return MockMessage(
                type="SYS_STATUS",
                voltage_battery=int(self._battery_voltage * 1000),
            )
        elif msg_type == "GLOBAL_POSITION_INT":
            # Simulate GPS position (McGill coordinates)
            return MockMessage(
                type="GLOBAL_POSITION_INT",
                lat=int(45.5048 * 1e7),
                lon=int(-73.5772 * 1e7),
                alt=100000,  # 100m in mm
            )
        elif msg_type == "ATTITUDE":
            return MockMessage(
                type="ATTITUDE",
                roll=random.gauss(0, 0.05),
                pitch=random.gauss(0, 0.05),
                yaw=random.uniform(-math.pi, math.pi),
            )
        
        return None
    
    def close(self):
        """Close mock connection."""
        logger.info("[SIM] MockMavlinkConnection closed")


class MockMav:
    """Mock MAV message sender."""
    
    def command_long_send(self, *args, **kwargs):
        """Log command but don't actually send."""
        logger.debug(f"[SIM] MAV command_long_send: {args[:3]}")
    
    def set_position_target_local_ned_send(self, *args, **kwargs):
        """Log velocity command."""
        logger.debug(f"[SIM] MAV velocity command: vx={args[7]:.2f} vy={args[8]:.2f} vz={args[9]:.2f}")
    
    def vision_position_estimate_send(self, *args, **kwargs):
        """Log VIO position."""
        pass  # Silent - too frequent
    
    def vision_speed_estimate_send(self, *args, **kwargs):
        """Log VIO speed."""
        pass  # Silent - too frequent


class MockMessage:
    """Mock MAVLink message."""
    
    def __init__(self, type: str, **kwargs):
        self._type = type
        for key, value in kwargs.items():
            setattr(self, key, value)
    
    def get_type(self) -> str:
        return self._type


# ============================================================
# Factory Functions
# ============================================================

def create_camera(
    resolution: str = "HD720",
    fps: int = 30,
    enable_tracking: bool = True,
    **kwargs,
) -> MockZedCamera | Any:
    """
    Factory function to create camera (mock or real based on SIM_MODE).
    
    Args:
        resolution: Camera resolution
        fps: Frame rate
        enable_tracking: Enable VIO
        **kwargs: Additional arguments
    
    Returns:
        MockZedCamera if SIM_MODE, else real ZedCamera
    """
    if is_sim_mode():
        logger.info("[SIM] Creating MockZedCamera")
        return MockZedCamera(
            resolution=resolution,
            fps=fps,
            enable_tracking=enable_tracking,
            **kwargs,
        )
    else:
        # Import real camera
        from .zed_interface import ZedCamera
        return ZedCamera(
            resolution=resolution,
            fps=fps,
            enable_tracking=enable_tracking,
        )


def create_yolo(
    model_path: str = "yolov8n.pt",
    **kwargs,
) -> MockYOLO | Any:
    """
    Factory function to create YOLO detector (mock or real based on SIM_MODE).
    
    Args:
        model_path: Path to YOLO model
        **kwargs: Additional arguments
    
    Returns:
        MockYOLO if SIM_MODE, else real YOLO
    """
    if is_sim_mode():
        logger.info("[SIM] Creating MockYOLO")
        return MockYOLO(model_path=model_path, **kwargs)
    else:
        # Import real YOLO
        from ultralytics import YOLO  # type: ignore[import]
        return YOLO(model_path)


def create_mavlink_connection(endpoint: str) -> MockMavlinkConnection | Any:
    """
    Factory function to create MAVLink connection (mock or real based on SIM_MODE).
    
    Args:
        endpoint: MAVLink endpoint string
    
    Returns:
        MockMavlinkConnection if SIM_MODE, else real connection
    """
    if is_sim_mode():
        logger.info("[SIM] Creating MockMavlinkConnection")
        return MockMavlinkConnection()
    else:
        from pymavlink import mavutil
        return mavutil.mavlink_connection(endpoint, autoreconnect=True)
