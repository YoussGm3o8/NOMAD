"""
Vision Process for NOMAD.

Standalone process that handles computer vision tasks:
- YOLOv8 object detection
- Target tracking and confidence filtering
- ZMQ publishing of detection results

Runs in isolation with heartbeat broadcasting for watchdog monitoring.
This module is designed to be spawned via multiprocessing.

Supports simulation mode via NOMAD_SIM_MODE environment variable.

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

from __future__ import annotations

import json
import logging
import math
import os
import signal
import sys
import threading
import time
from dataclasses import dataclass, asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

# Configure logging for vision process
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] vision: %(message)s",
)
logger = logging.getLogger("vision")

# Lazy imports to avoid loading heavy libraries in main process
# Type hints use Any since these are loaded dynamically
cv2: Any = None
YOLO: Any = None


def _lazy_import_vision_libs() -> None:
    """Lazy import OpenCV and Ultralytics to avoid loading in main process."""
    global cv2, YOLO
    if cv2 is None:
        import cv2 as _cv2  # type: ignore[import]
        cv2 = _cv2
    if YOLO is None:
        # Check simulation mode before importing heavy YOLO
        from .mocks import is_sim_mode
        if not is_sim_mode():
            from ultralytics import YOLO as _YOLO  # type: ignore[import]
            YOLO = _YOLO


# Import IPC
from .ipc import (
    ZMQPublisher,
    ZMQSubscriber,
    IPCMessage,
    DEFAULT_VISION_HEARTBEAT_ENDPOINT,
)

# Import ZED interface (lazy import in process)
from .zed_interface import ZedCamera, NEDPose, create_vio_message, is_zed_available

# Default VIO endpoint
DEFAULT_VIO_ENDPOINT = "tcp://127.0.0.1:5557"

# Default throttle endpoint (receives from main process)
DEFAULT_THROTTLE_ENDPOINT = "tcp://127.0.0.1:5560"


# ============================================================
# Detection Data Structures
# ============================================================

@dataclass
class BoundingBox:
    """Bounding box coordinates (normalized 0-1)."""
    x1: float  # Left
    y1: float  # Top
    x2: float  # Right
    y2: float  # Bottom

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
class Detection:
    """Object detection result."""
    class_id: int
    class_name: str
    confidence: float
    bbox: BoundingBox
    timestamp: float

    def to_dict(self) -> dict:
        """Convert to dictionary for JSON serialization."""
        return {
            "class_id": self.class_id,
            "class_name": self.class_name,
            "confidence": self.confidence,
            "bbox": asdict(self.bbox),
            "timestamp": self.timestamp,
        }


# ============================================================
# Vision Configuration
# ============================================================

@dataclass
class VisionConfig:
    """Configuration for vision process."""
    # Model
    model_path: str = "yolov8n.pt"

    # Video source (0 = webcam, "zed" = ZED camera, or file path)
    video_source: str | int = 0

    # Detection
    confidence_threshold: float = 0.5
    target_classes: list[str] | None = None  # None = all classes

    # Performance
    inference_size: int = 640
    skip_frames: int = 0  # Process every Nth frame (0 = no skip)

    # IPC
    heartbeat_endpoint: str = DEFAULT_VISION_HEARTBEAT_ENDPOINT
    detection_endpoint: str = "tcp://127.0.0.1:5556"
    vio_endpoint: str = DEFAULT_VIO_ENDPOINT
    heartbeat_interval: float = 1.0

    # ZED camera settings (used when video_source="zed")
    zed_resolution: str = "HD720"
    zed_fps: int = 30
    enable_vio: bool = True  # Enable VIO publishing

    # Simulation mode
    sim_mode: bool = False  # Use mock hardware

    # RTSP streaming output
    rtsp_enabled: bool = False  # Enable RTSP output
    rtsp_url: str = "rtsp://127.0.0.1:8554/live"  # MediaMTX endpoint
    rtsp_framerate: int = 30  # Output framerate
    rtsp_bitrate: int = 4000  # kbps
    rtsp_width: int = 1280  # Output width (0 = use source)
    rtsp_height: int = 720  # Output height (0 = use source)

    @classmethod
    def from_env(cls) -> "VisionConfig":
        """Load configuration from environment variables."""
        return cls(
            model_path=os.environ.get("VISION_MODEL_PATH", "yolov8n.pt"),
            video_source=os.environ.get("VISION_VIDEO_SOURCE", "zed"),
            confidence_threshold=float(os.environ.get("VISION_CONFIDENCE", "0.5")),
            inference_size=int(os.environ.get("VISION_INFERENCE_SIZE", "640")),
            skip_frames=int(os.environ.get("VISION_SKIP_FRAMES", "0")),
            heartbeat_endpoint=os.environ.get(
                "VISION_HEARTBEAT_ENDPOINT",
                DEFAULT_VISION_HEARTBEAT_ENDPOINT
            ),
            detection_endpoint=os.environ.get(
                "VISION_DETECTION_ENDPOINT",
                "tcp://127.0.0.1:5556"
            ),
            vio_endpoint=os.environ.get(
                "VISION_VIO_ENDPOINT",
                DEFAULT_VIO_ENDPOINT
            ),
            zed_resolution=os.environ.get("ZED_RESOLUTION", "HD720"),
            zed_fps=int(os.environ.get("ZED_FPS", "30")),
            enable_vio=os.environ.get("ENABLE_VIO", "true").lower() == "true",
            sim_mode=os.environ.get("NOMAD_SIM_MODE", "").lower() in ("true", "1", "yes"),
            # RTSP streaming configuration
            rtsp_enabled=os.environ.get("RTSP_ENABLED", "").lower() in ("true", "1", "yes"),
            rtsp_url=os.environ.get("RTSP_OUTPUT_URL", "rtsp://127.0.0.1:8554/live"),
            rtsp_framerate=int(os.environ.get("RTSP_FRAMERATE", "30")),
            rtsp_bitrate=int(os.environ.get("RTSP_BITRATE", "4000")),
            rtsp_width=int(os.environ.get("RTSP_WIDTH", "1280")),
            rtsp_height=int(os.environ.get("RTSP_HEIGHT", "720")),
        )


# ============================================================
# RTSP Streamer (GStreamer-based output to MediaMTX)
# ============================================================

class RTSPStreamer:
    """
    RTSP video streamer using GStreamer pipeline.
    
    Outputs processed frames (with detection overlays) to MediaMTX
    server for WebRTC delivery to web clients.
    """
    
    def __init__(
        self,
        rtsp_url: str = "rtsp://127.0.0.1:8554/live",
        width: int = 1280,
        height: int = 720,
        framerate: int = 30,
        bitrate: int = 4000,  # kbps
    ) -> None:
        """
        Initialize the RTSP streamer.
        
        Args:
            rtsp_url: RTSP endpoint (MediaMTX)
            width: Output frame width
            height: Output frame height
            framerate: Output framerate
            bitrate: Video bitrate in kbps
        """
        self._rtsp_url = rtsp_url
        self._width = width
        self._height = height
        self._framerate = framerate
        self._bitrate = bitrate
        self._writer: Any = None  # cv2.VideoWriter
        self._initialized = False
        self._frame_count = 0
        self._error_count = 0
        self._max_errors = 10
    
    def _build_gstreamer_pipeline(self) -> str:
        """Build GStreamer pipeline string for RTSP output."""
        # Parse RTSP URL to extract host and path
        # Format: rtsp://host:port/path
        import re
        match = re.match(r"rtsp://([^:/]+):?(\d+)?(/.*)?", self._rtsp_url)
        if not match:
            raise ValueError(f"Invalid RTSP URL: {self._rtsp_url}")
        
        host = match.group(1)
        port = match.group(2) or "8554"
        path = match.group(3) or "/live"
        
        # GStreamer pipeline for RTSP output via rtspclientsink
        # Alternative: use appsrc ! x264enc ! rtspclientsink
        pipeline = (
            f"appsrc ! "
            f"videoconvert ! "
            f"video/x-raw,format=I420 ! "
            # Use hardware encoder on Jetson (nvv4l2h264enc) or software (x264enc)
            f"x264enc tune=zerolatency bitrate={self._bitrate} speed-preset=ultrafast ! "
            f"video/x-h264,profile=baseline ! "
            f"h264parse ! "
            f"rtspclientsink location={self._rtsp_url} protocols=tcp"
        )
        
        # Alternative pipeline for FFmpeg-based output (more compatible)
        # This uses FFmpeg's RTSP muxer which is more widely supported
        pipeline_ffmpeg = (
            f"appsrc ! "
            f"videoconvert ! "
            f"video/x-raw,format=I420 ! "
            f"x264enc tune=zerolatency bitrate={self._bitrate} speed-preset=ultrafast ! "
            f"flvmux ! "
            f"rtmpsink location=rtmp://{host}:{port}{path}"
        )
        
        # Use simpler pipeline that works with MediaMTX's RTSP publisher
        simple_pipeline = (
            f"appsrc ! "
            f"videoconvert ! "
            f"x264enc tune=zerolatency speed-preset=ultrafast bitrate={self._bitrate} ! "
            f"rtspclientsink location={self._rtsp_url} protocols=tcp latency=0"
        )
        
        return simple_pipeline
    
    def initialize(self, frame_width: int | None = None, frame_height: int | None = None) -> bool:
        """
        Initialize the video writer with GStreamer backend.
        
        Args:
            frame_width: Source frame width (optional, uses config if not provided)
            frame_height: Source frame height (optional, uses config if not provided)
            
        Returns:
            True if initialization successful
        """
        if self._initialized:
            return True
        
        _lazy_import_vision_libs()
        if cv2 is None:
            logger.error("OpenCV not available for RTSP streaming")
            return False
        
        # Use provided dimensions or config
        width = frame_width or self._width
        height = frame_height or self._height
        
        try:
            # Build GStreamer pipeline
            gst_pipeline = self._build_gstreamer_pipeline()
            logger.info(f"RTSP GStreamer pipeline: {gst_pipeline}")
            
            # Create VideoWriter with GStreamer backend
            fourcc = cv2.VideoWriter_fourcc(*'X264')
            self._writer = cv2.VideoWriter(
                gst_pipeline,
                cv2.CAP_GSTREAMER,
                fourcc,
                float(self._framerate),
                (width, height),
                True,  # isColor
            )
            
            if not self._writer.isOpened():
                # Fallback: Try FFmpeg-based RTSP output
                logger.warning("GStreamer pipeline failed, trying FFmpeg fallback...")
                ffmpeg_pipeline = (
                    f"appsrc ! videoconvert ! "
                    f"x264enc tune=zerolatency ! "
                    f"flvmux streamable=true ! "
                    f"rtmpsink location={self._rtsp_url.replace('rtsp://', 'rtmp://')}"
                )
                self._writer = cv2.VideoWriter(
                    ffmpeg_pipeline,
                    cv2.CAP_FFMPEG,
                    fourcc,
                    float(self._framerate),
                    (width, height),
                    True,
                )
            
            if self._writer.isOpened():
                self._initialized = True
                logger.info(f"RTSP streamer initialized: {self._rtsp_url} ({width}x{height}@{self._framerate}fps)")
                return True
            else:
                logger.error("Failed to open RTSP video writer")
                return False
                
        except Exception as e:
            logger.error(f"RTSP streamer initialization failed: {e}")
            return False
    
    def write_frame(self, frame: Any) -> bool:
        """
        Write a frame to the RTSP stream.
        
        Args:
            frame: OpenCV BGR frame (numpy array)
            
        Returns:
            True if write successful
        """
        if not self._initialized or self._writer is None:
            return False
        
        try:
            # Resize if needed
            if frame.shape[1] != self._width or frame.shape[0] != self._height:
                frame = cv2.resize(frame, (self._width, self._height))
            
            self._writer.write(frame)
            self._frame_count += 1
            self._error_count = 0  # Reset error count on success
            return True
            
        except Exception as e:
            self._error_count += 1
            if self._error_count <= 3:  # Only log first few errors
                logger.error(f"RTSP write error: {e}")
            
            # Try to reconnect if too many errors
            if self._error_count >= self._max_errors:
                logger.warning("Too many RTSP errors, attempting reconnect...")
                self.release()
                self.initialize()
            
            return False
    
    def release(self) -> None:
        """Release the video writer."""
        if self._writer is not None:
            try:
                self._writer.release()
            except Exception as e:
                logger.warning(f"Error releasing RTSP writer: {e}")
            self._writer = None
        self._initialized = False
        logger.info(f"RTSP streamer released. Total frames: {self._frame_count}")
    
    @property
    def is_initialized(self) -> bool:
        """Check if streamer is initialized."""
        return self._initialized
    
    @property
    def frame_count(self) -> int:
        """Get number of frames written."""
        return self._frame_count


# ============================================================
# Throttle Subscriber (receives from main process)
# ============================================================

class ThrottleState:
    """Thread-safe throttle state management."""
    
    def __init__(self) -> None:
        self._throttled = False
        self._level = "NONE"
        self._reason = ""
        self._lock = threading.Lock()
    
    @property
    def is_throttled(self) -> bool:
        with self._lock:
            return self._throttled
    
    @property
    def level(self) -> str:
        with self._lock:
            return self._level
    
    def update(self, throttled: bool, level: str = "NONE", reason: str = "") -> None:
        with self._lock:
            old_throttled = self._throttled
            self._throttled = throttled
            self._level = level
            self._reason = reason
            
            if throttled and not old_throttled:
                logger.warning(f"Throttle activated: {level} - {reason}")
            elif not throttled and old_throttled:
                logger.info("Throttle deactivated - resuming normal operation")


class ThrottleSubscriber:
    """Subscribes to throttle messages from main process."""
    
    def __init__(
        self,
        throttle_state: ThrottleState,
        endpoint: str = DEFAULT_THROTTLE_ENDPOINT,
    ) -> None:
        self._state = throttle_state
        self._endpoint = endpoint
        self._running = False
        self._thread: threading.Thread | None = None
        self._subscriber: ZMQSubscriber | None = None
    
    def _poll_loop(self) -> None:
        """Background thread that polls for throttle messages."""
        while self._running:
            if self._subscriber:
                msg = self._subscriber.receive()
                if msg:
                    if msg.msg_type == "THROTTLE_DOWN":
                        data = msg.data or {}
                        self._state.update(
                            throttled=True,
                            level=data.get("level", "THROTTLE"),
                            reason=data.get("reason", ""),
                        )
                    elif msg.msg_type == "THROTTLE_UP":
                        self._state.update(throttled=False)
    
    def start(self) -> None:
        """Start listening for throttle messages."""
        if self._running:
            return
        
        self._running = True
        
        self._subscriber = ZMQSubscriber(
            endpoint=self._endpoint,
            timeout_ms=100,  # Short timeout for responsive polling
        )
        self._subscriber.start()
        
        # Start background polling thread
        self._thread = threading.Thread(
            target=self._poll_loop,
            daemon=True,
            name="throttle_subscriber",
        )
        self._thread.start()
        logger.info(f"Throttle subscriber started: {self._endpoint}")
    
    def stop(self) -> None:
        """Stop the subscriber."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._subscriber:
            self._subscriber.stop()
            self._subscriber = None


# ============================================================
# Vision Engine
# ============================================================

class VisionEngine:
    """
    YOLOv8-based object detection engine with ZED camera support.

    Handles model loading, video capture (ZED or generic), VIO, and inference.
    Supports simulation mode for development without hardware.
    """

    def __init__(self, config: VisionConfig):
        self.config = config
        self.model = None
        self.cap = None
        self.zed_camera: Any = None  # ZedCamera or MockZedCamera
        self.frame_count = 0
        self._class_names: dict[int, str] = {}
        self._use_zed = False
        self._sim_mode = config.sim_mode
        self._current_frame: Any = None  # Store current frame for RTSP streaming

    def initialize(self) -> bool:
        """
        Initialize the vision engine.

        Returns:
            True if initialization successful, False otherwise
        """
        _lazy_import_vision_libs()

        # Log simulation mode
        if self._sim_mode:
            logger.info("=" * 40)
            logger.info("  SIMULATION MODE ACTIVE")
            logger.info("  Using mock hardware for development")
            logger.info("=" * 40)

        # Load YOLOv8 model (mock or real)
        try:
            logger.info(f"Loading YOLOv8 model: {self.config.model_path}")
            
            if self._sim_mode:
                from .mocks import MockYOLO
                self.model = MockYOLO(
                    model_path=self.config.model_path,
                    target_classes=self.config.target_classes,
                )
                self._class_names = self.model.names
            else:
                self.model = YOLO(self.config.model_path)
                # Cache class names
                if hasattr(self.model, "names"):
                    self._class_names = self.model.names
                    logger.info(f"Model loaded with {len(self._class_names)} classes")
                else:
                    logger.warning("Model has no class names attribute")

        except Exception as e:
            logger.error(f"Failed to load model: {e}")
            return False

        # Initialize video capture (ZED or generic)
        source = self.config.video_source
        
        # Check if using ZED camera
        if isinstance(source, str) and source.lower() == "zed":
            return self._initialize_zed()
        else:
            return self._initialize_generic_capture(source)

    def _initialize_zed(self) -> bool:
        """Initialize ZED 2i camera (or mock in simulation mode)."""
        if self._sim_mode:
            # Use mock camera in simulation mode
            from .mocks import MockZedCamera
            logger.info(f"[SIM] Opening MockZedCamera: {self.config.zed_resolution}@{self.config.zed_fps}fps")
            self.zed_camera = MockZedCamera(
                resolution=self.config.zed_resolution,
                fps=self.config.zed_fps,
                enable_tracking=self.config.enable_vio,
            )
            
            if not self.zed_camera.open():
                logger.error("[SIM] Failed to open MockZedCamera")
                return False
            
            self._use_zed = True
            logger.info("[SIM] MockZedCamera ready")
            return True
        
        # Real ZED camera
        if not is_zed_available():
            logger.error("ZED SDK not available, falling back to webcam")
            return self._initialize_generic_capture(0)
        
        try:
            logger.info(f"Opening ZED camera: {self.config.zed_resolution}@{self.config.zed_fps}fps")
            self.zed_camera = ZedCamera(
                resolution=self.config.zed_resolution,
                fps=self.config.zed_fps,
                enable_tracking=self.config.enable_vio,
            )
            
            if not self.zed_camera.open():
                logger.error("Failed to open ZED camera")
                return False
            
            self._use_zed = True
            
            # Log camera info
            info = self.zed_camera.get_camera_info()
            if info:
                logger.info(f"ZED camera ready: SN={info.get('serial_number')}")
            
            return True
            
        except Exception as e:
            logger.error(f"ZED initialization error: {e}")
            return False
            return False

    def _initialize_generic_capture(self, source) -> bool:
        """Initialize generic OpenCV video capture."""
        try:
            # Convert "0" string to int for webcam
            if isinstance(source, str) and source.isdigit():
                source = int(source)

            logger.info(f"Opening video source: {source}")
            self.cap = cv2.VideoCapture(source)

            if not self.cap.isOpened():
                logger.error(f"Failed to open video source: {source}")
                return False

            # Get video properties
            width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fps = self.cap.get(cv2.CAP_PROP_FPS)
            logger.info(f"Video source: {width}x{height} @ {fps:.1f} FPS")
            
            self._use_zed = False
            return True

        except Exception as e:
            logger.error(f"Failed to initialize video capture: {e}")
            return False

    def grab_frame(self) -> bool:
        """
        Grab a new frame from the video source.
        
        Returns:
            True if frame available
        """
        if self._use_zed and self.zed_camera:
            return self.zed_camera.grab()
        elif self.cap:
            ret, _ = self.cap.read()
            return ret
        return False

    def get_frame(self) -> Any | None:
        """Get the current frame as numpy array."""
        if self._use_zed and self.zed_camera:
            # ZED returns BGRA, convert to BGR for YOLO
            image = self.zed_camera.get_image()
            if image is not None:
                return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
            return None
        elif self.cap:
            ret, frame = self.cap.read()
            return frame if ret else None
        return None

    def get_current_frame(self) -> Any | None:
        """
        Get the most recently processed frame (for RTSP streaming).
        
        Returns:
            OpenCV BGR frame or None
        """
        return self._current_frame

    def get_pose(self) -> NEDPose | Any | None:
        """
        Get the current VIO pose in NED frame.
        
        Returns:
            NEDPose (or MockPose in sim mode) if VIO is active, None otherwise
        """
        if self._use_zed and self.zed_camera and self.config.enable_vio:
            if self._sim_mode:
                # MockZedCamera returns MockPose directly in NED
                return self.zed_camera.get_pose_ned()
            else:
                return self.zed_camera.get_pose_ned()
        return None

    def process_frame(self) -> list[Detection]:
        """
        Capture and process a single frame.

        Returns:
            List of Detection objects found in the frame
        """
        if self.model is None:
            return []

        # Get frame based on source type
        if self._use_zed and self.zed_camera:
            if not self.zed_camera.grab():
                return []
            frame = self.zed_camera.get_image()
            if frame is not None:
                # Convert BGRA to BGR
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        else:
            if self.cap is None:
                return []
            ret, frame = self.cap.read()
            if not ret or frame is None:
                return []

        # Store current frame for RTSP streaming
        self._current_frame = frame

        self.frame_count += 1

        # Skip frames if configured
        if self.config.skip_frames > 0:
            if self.frame_count % (self.config.skip_frames + 1) != 0:
                return []

        # Run inference
        try:
            results = self.model(
                frame,
                imgsz=self.config.inference_size,
                conf=self.config.confidence_threshold,
                verbose=False,
            )
        except Exception as e:
            logger.error(f"Inference error: {e}")
            return []

        # Parse detections
        detections: list[Detection] = []
        timestamp = time.time()

        for result in results:
            if result.boxes is None:
                continue

            boxes = result.boxes
            for i in range(len(boxes)):
                # Get detection data
                xyxy = boxes.xyxy[i].cpu().numpy()
                conf = float(boxes.conf[i].cpu().numpy())
                cls_id = int(boxes.cls[i].cpu().numpy())

                # Get class name
                class_name = self._class_names.get(cls_id, f"class_{cls_id}")

                # Filter by target classes if configured
                if self.config.target_classes:
                    if class_name not in self.config.target_classes:
                        continue

                # Normalize bounding box to 0-1
                h, w = frame.shape[:2]
                bbox = BoundingBox(
                    x1=float(xyxy[0]) / w,
                    y1=float(xyxy[1]) / h,
                    x2=float(xyxy[2]) / w,
                    y2=float(xyxy[3]) / h,
                )

                detection = Detection(
                    class_id=cls_id,
                    class_name=class_name,
                    confidence=conf,
                    bbox=bbox,
                    timestamp=timestamp,
                )
                detections.append(detection)

        return detections

    def release(self):
        """Release video capture resources."""
        if self.zed_camera is not None:
            self.zed_camera.close()
            self.zed_camera = None
        if self.cap is not None:
            self.cap.release()
            self.cap = None


# ============================================================
# Global State
# ============================================================

_shutdown_requested = False


def signal_handler(signum: int, frame: Any) -> None:
    """Handle shutdown signals gracefully."""
    global _shutdown_requested
    logger.info(f"Received signal {signum}, shutting down...")
    _shutdown_requested = True


# ============================================================
# Main Process Entry
# ============================================================

def vision_process_main(
    config: VisionConfig | None = None,
) -> None:
    """
    Main entry point for the vision process.

    Runs an infinite loop that:
    1. Initializes YOLOv8 model and video capture (ZED or generic)
    2. Processes frames and detects objects
    3. Publishes detections via ZMQ
    4. Publishes VIO pose data (when using ZED)
    5. Sends heartbeats for watchdog monitoring

    Args:
        config: Vision configuration (defaults to env-based config)
    """
    global _shutdown_requested

    # Setup signal handlers
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)

    # Load config
    if config is None:
        config = VisionConfig.from_env()

    logger.info("=" * 50)
    if config.sim_mode:
        logger.info("Vision Process Starting [SIMULATION MODE]")
    else:
        logger.info("Vision Process Starting (YOLOv8 + ZED VIO)")
    logger.info(f"Model: {config.model_path}")
    logger.info(f"Video source: {config.video_source}")
    logger.info(f"Confidence threshold: {config.confidence_threshold}")
    logger.info(f"Heartbeat endpoint: {config.heartbeat_endpoint}")
    logger.info(f"Detection endpoint: {config.detection_endpoint}")
    logger.info(f"VIO endpoint: {config.vio_endpoint}")
    logger.info(f"VIO enabled: {config.enable_vio}")
    logger.info(f"Simulation mode: {config.sim_mode}")
    logger.info(f"Throttle endpoint: {DEFAULT_THROTTLE_ENDPOINT}")
    logger.info(f"RTSP streaming: {config.rtsp_enabled}")
    if config.rtsp_enabled:
        logger.info(f"RTSP URL: {config.rtsp_url}")
        logger.info(f"RTSP framerate: {config.rtsp_framerate}")
    logger.info("=" * 50)

    # Initialize throttle state and subscriber
    throttle_state = ThrottleState()
    throttle_sub = ThrottleSubscriber(throttle_state, DEFAULT_THROTTLE_ENDPOINT)

    # Initialize publishers
    heartbeat_pub = ZMQPublisher(endpoint=config.heartbeat_endpoint)
    detection_pub = ZMQPublisher(endpoint=config.detection_endpoint)
    vio_pub = ZMQPublisher(endpoint=config.vio_endpoint)

    # Initialize RTSP streamer (optional)
    rtsp_streamer: RTSPStreamer | None = None
    if config.rtsp_enabled:
        rtsp_streamer = RTSPStreamer(
            rtsp_url=config.rtsp_url,
            width=config.rtsp_width,
            height=config.rtsp_height,
            framerate=config.rtsp_framerate,
            bitrate=config.rtsp_bitrate,
        )

    # Initialize vision engine
    engine = VisionEngine(config)
    detection_count = 0  # Initialize before try for finally block access
    vio_count = 0
    rtsp_frame_count = 0  # Track RTSP frames sent
    frame_counter = 0  # For throttle skip logic
    throttle_skip_every = 2  # Skip every Nth frame when throttled
    throttle_sleep = 0.1  # Sleep duration when throttled (10Hz cap)

    try:
        # Start publishers
        heartbeat_pub.start()
        detection_pub.start()
        vio_pub.start()

        # Start throttle subscriber
        throttle_sub.start()

        # Initialize vision (load model, open video)
        if not engine.initialize():
            logger.error("Vision engine initialization failed")
            sys.exit(1)

        # Initialize RTSP streamer after engine is ready (to get frame dimensions)
        if rtsp_streamer is not None:
            if not rtsp_streamer.initialize(config.rtsp_width, config.rtsp_height):
                logger.warning("RTSP streamer initialization failed - streaming disabled")
                rtsp_streamer = None
            else:
                logger.info("RTSP streamer ready")

        logger.info("Vision process initialized successfully")

        last_heartbeat = 0.0
        last_vio_log = 0.0
        last_throttle_log = 0.0
        last_rtsp_log = 0.0  # For RTSP status logging

        while not _shutdown_requested:
            current_time = time.time()
            frame_counter += 1

            # Send heartbeat periodically
            if current_time - last_heartbeat >= config.heartbeat_interval:
                heartbeat_pub.send_heartbeat("vision")
                last_heartbeat = current_time

            # Check throttle state
            if throttle_state.is_throttled:
                # Log throttle status periodically
                if current_time - last_throttle_log >= 5.0:
                    logger.warning(f"Throttled: {throttle_state.level} - skipping frames to cool down")
                    last_throttle_log = current_time
                
                # Option 1: Skip every 2nd frame
                if frame_counter % throttle_skip_every == 0:
                    time.sleep(throttle_sleep / 2)  # Brief sleep to reduce load
                    continue
                
                # Option 2: Rate limit with sleep (10Hz cap)
                time.sleep(throttle_sleep)

            # Process frame and get detections
            detections = engine.process_frame()

            # Get and publish VIO pose (from ZED tracking)
            if config.enable_vio:
                pose = engine.get_pose()
                # Check validity - MockPose has .valid, NEDPose has .tracking_state
                pose_valid = False
                if pose is not None:
                    if hasattr(pose, 'valid'):
                        pose_valid = getattr(pose, 'valid', False)
                    elif hasattr(pose, 'tracking_state'):
                        # NEDPose uses tracking_state enum
                        tracking_state = getattr(pose, 'tracking_state', None)
                        if tracking_state is not None:
                            pose_valid = str(tracking_state.value) == "OK"
                
                if pose is not None and pose_valid:
                    vio_count += 1
                    vio_message = IPCMessage(
                        msg_type="vio_pose",
                        timestamp=datetime.now(timezone.utc).isoformat(),
                        data=pose.to_dict(),
                    )
                    vio_pub.send(vio_message)

                    # Log VIO periodically (every 5 seconds)
                    if current_time - last_vio_log >= 5.0:
                        logger.info(
                            f"VIO: x={pose.x:.3f} y={pose.y:.3f} z={pose.z:.3f} "
                            f"yaw={math.degrees(pose.yaw):.1f}°"
                        )
                        last_vio_log = current_time

            # Publish detections
            if detections:
                detection_count += len(detections)

                # Publish each detection
                for det in detections:
                    message = IPCMessage(
                        msg_type="detection",
                        timestamp=datetime.now(timezone.utc).isoformat(),
                        data=det.to_dict(),
                    )
                    detection_pub.send(message)

                # Log high-confidence detections
                best = max(detections, key=lambda d: d.confidence)
                if best.confidence >= 0.7:
                    logger.info(
                        f"Detection: {best.class_name} "
                        f"({best.confidence:.2f}) at "
                        f"({best.bbox.center_x:.2f}, {best.bbox.center_y:.2f})"
                    )

            # Write frame to RTSP stream (with detection overlays)
            if rtsp_streamer is not None:
                frame = engine.get_current_frame()
                if frame is not None:
                    # Draw detection overlays on frame
                    display_frame = frame.copy()
                    for det in detections:
                        h, w = display_frame.shape[:2]
                        # Convert normalized bbox to pixel coordinates
                        x1 = int(det.bbox.x1 * w)
                        y1 = int(det.bbox.y1 * h)
                        x2 = int(det.bbox.x2 * w)
                        y2 = int(det.bbox.y2 * h)
                        
                        # Draw bounding box
                        color = (0, 255, 0) if det.confidence >= 0.7 else (0, 255, 255)
                        cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 2)
                        
                        # Draw label
                        label = f"{det.class_name}: {det.confidence:.2f}"
                        cv2.putText(
                            display_frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2
                        )
                    
                    # Write frame to RTSP
                    if rtsp_streamer.write_frame(display_frame):
                        rtsp_frame_count += 1
                        
                        # Log RTSP stats periodically
                        if current_time - last_rtsp_log >= 10.0:
                            logger.info(f"RTSP streaming: {rtsp_frame_count} frames sent")
                            last_rtsp_log = current_time

            # Small sleep to prevent CPU spinning when no frames
            time.sleep(0.001)

    except Exception as e:
        logger.error(f"Vision process error: {e}")
        sys.exit(1)

    finally:
        # Cleanup
        throttle_sub.stop()
        if rtsp_streamer is not None:
            rtsp_streamer.release()
        engine.release()
        heartbeat_pub.stop()
        detection_pub.stop()
        vio_pub.stop()
        logger.info(
            f"Vision process terminated. "
            f"Detections: {detection_count}, VIO updates: {vio_count}, "
            f"RTSP frames: {rtsp_frame_count}"
        )


def run_vision_process(
    heartbeat_endpoint: str = DEFAULT_VISION_HEARTBEAT_ENDPOINT,
) -> None:
    """
    Wrapper function for multiprocessing.

    This is the target function passed to multiprocessing.Process.

    Args:
        heartbeat_endpoint: ZMQ endpoint for heartbeats
    """
    try:
        config = VisionConfig.from_env()
        config.heartbeat_endpoint = heartbeat_endpoint
        vision_process_main(config=config)
    except KeyboardInterrupt:
        logger.info("Vision process interrupted")
    except Exception as e:
        logger.error(f"Vision process crashed: {e}")
        sys.exit(1)


if __name__ == "__main__":
    # Allow running directly for testing
    run_vision_process()

