"""
Isaac ROS Bridge for NOMAD Edge Core.

This module provides integration with NVIDIA Isaac ROS for:
- ZED VSLAM (Visual SLAM) via isaac_ros_visual_slam
- YOLO object detection via isaac_ros_yolov8
- Automatic exclusion map management

Target: Python 3.13 | NVIDIA Jetson Orin Nano with Isaac ROS
"""

from __future__ import annotations

import logging
import math
import os
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional

logger = logging.getLogger(__name__)

# Import ROS2 - required (no fallback)
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection2DArray


@dataclass
class VIOState:
    """Visual Inertial Odometry state from Isaac ROS."""
    timestamp: float  # Unix timestamp
    x: float  # North (NED) in meters
    y: float  # East (NED) in meters
    z: float  # Down (NED) in meters
    roll: float  # Radians
    pitch: float  # Radians
    yaw: float  # Radians
    vx: float = 0.0  # Velocity North
    vy: float = 0.0  # Velocity East
    vz: float = 0.0  # Velocity Down
    confidence: float = 1.0  # Tracking confidence 0-1
    valid: bool = True


@dataclass
class DetectedTarget:
    """Detected target from Isaac ROS YOLO."""
    timestamp: float
    class_name: str
    class_id: int
    confidence: float
    bbox_x: int  # Top-left x
    bbox_y: int  # Top-left y
    bbox_w: int  # Width
    bbox_h: int  # Height
    world_x: Optional[float] = None  # 3D position X
    world_y: Optional[float] = None  # 3D position Y
    world_z: Optional[float] = None  # 3D position Z


@dataclass
class ExclusionEntry:
    """Entry in the exclusion map for engaged targets."""
    id: str
    x: float  # NED North
    y: float  # NED East
    z: float  # NED Down
    timestamp: float
    hit_count: int = 1
    radius: float = 0.5  # Exclusion radius in meters


class NOMADRosNode(Node):
    """ROS2 node for NOMAD Isaac ROS integration."""
    
    def __init__(
        self,
        vio_topic: str,
        detection_topic: str,
        vio_callback: Callable,
        detection_callback: Callable,
    ):
        super().__init__("nomad_bridge")
        
        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        
        # VIO subscription
        self.create_subscription(
            Odometry,
            vio_topic,
            vio_callback,
            sensor_qos,
        )
        
        # YOLO detection subscription
        self.create_subscription(
            Detection2DArray,
            detection_topic,
            detection_callback,
            sensor_qos,
        )
        
        self.get_logger().info(f"Subscribed to VIO: {vio_topic}")
        self.get_logger().info(f"Subscribed to detections: {detection_topic}")


class IsaacROSBridge:
    """
    Bridge between NOMAD Edge Core and Isaac ROS perception stack.
    
    Subscribes to:
    - VIO/VSLAM pose from isaac_ros_visual_slam
    - YOLO detections from isaac_ros_yolov8
    
    Manages:
    - Automatic exclusion map for engaged targets
    """
    
    def __init__(
        self,
        vio_topic: str = "/visual_slam/tracking/odometry",
        detection_topic: str = "/yolov8/detections",
        exclusion_radius: float = 0.5,
    ):
        """
        Initialize the Isaac ROS bridge.
        
        Args:
            vio_topic: Topic for VIO/VSLAM odometry
            detection_topic: Topic for YOLO detections
            exclusion_radius: Default radius for exclusion zones (meters)
        """
        self._vio_topic = vio_topic
        self._detection_topic = detection_topic
        self._exclusion_radius = exclusion_radius
        
        # State
        self._running = False
        self._lock = threading.Lock()
        self._vio_state: Optional[VIOState] = None
        self._detections: list[DetectedTarget] = []
        self._exclusion_map: dict[str, ExclusionEntry] = {}
        
        # ROS2 node
        self._node: Optional[NOMADRosNode] = None
        self._spin_thread: Optional[threading.Thread] = None
        
        # Callbacks for external listeners
        self._vio_callbacks: list[Callable[[VIOState], None]] = []
        self._detection_callbacks: list[Callable[[list[DetectedTarget]], None]] = []
        
        # Stats
        self._vio_count = 0
        self._detection_count = 0
        
        logger.info("IsaacROSBridge initialized")
    
    @property
    def vio_state(self) -> Optional[VIOState]:
        """Get latest VIO state."""
        with self._lock:
            return self._vio_state
    
    @property
    def detections(self) -> list[DetectedTarget]:
        """Get latest detections (filtered by exclusion map)."""
        with self._lock:
            return self._filter_detections(list(self._detections))
    
    @property
    def raw_detections(self) -> list[DetectedTarget]:
        """Get raw detections without exclusion filtering."""
        with self._lock:
            return list(self._detections)
    
    @property
    def exclusion_map(self) -> dict[str, ExclusionEntry]:
        """Get the exclusion map."""
        with self._lock:
            return dict(self._exclusion_map)
    
    def start(self) -> None:
        """Start the ROS2 bridge."""
        if self._running:
            return
        
        self._running = True
        
        # Initialize ROS2
        rclpy.init()
        
        # Create node
        self._node = NOMADRosNode(
            vio_topic=self._vio_topic,
            detection_topic=self._detection_topic,
            vio_callback=self._handle_vio,
            detection_callback=self._handle_detections,
        )
        
        # Start spin thread
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()
        
        logger.info("IsaacROSBridge started")
    
    def stop(self) -> None:
        """Stop the ROS2 bridge."""
        self._running = False
        
        if self._node:
            self._node.destroy_node()
        
        try:
            rclpy.shutdown()
        except Exception:
            pass
        
        if self._spin_thread and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
        
        logger.info("IsaacROSBridge stopped")
    
    def _spin_loop(self) -> None:
        """ROS2 spin loop."""
        while self._running and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.1)
    
    def _handle_vio(self, msg: Odometry) -> None:
        """Handle VIO odometry from Isaac ROS visual_slam."""
        try:
            pose = msg.pose.pose
            twist = msg.twist.twist
            
            # Quaternion to Euler
            q = pose.orientation
            roll, pitch, yaw = self._quat_to_euler(q.x, q.y, q.z, q.w)
            
            # Convert from ZED camera frame (X-right, Y-down, Z-forward)
            # to NED frame (X-north, Y-east, Z-down)
            vio = VIOState(
                timestamp=time.time(),
                x=pose.position.z,   # Forward -> North
                y=pose.position.x,   # Right -> East
                z=-pose.position.y,  # Up -> Down
                roll=roll,
                pitch=pitch,
                yaw=yaw,
                vx=twist.linear.z,
                vy=twist.linear.x,
                vz=-twist.linear.y,
                confidence=1.0,
                valid=True,
            )
            
            with self._lock:
                self._vio_state = vio
                self._vio_count += 1
            
            # Notify callbacks
            for cb in self._vio_callbacks:
                try:
                    cb(vio)
                except Exception as e:
                    logger.error(f"VIO callback error: {e}")
                    
        except Exception as e:
            logger.error(f"VIO processing error: {e}")
    
    def _handle_detections(self, msg: Detection2DArray) -> None:
        """Handle YOLO detections from Isaac ROS."""
        try:
            detections = []
            
            for det in msg.detections:
                if not det.results:
                    continue
                
                result = det.results[0]
                bbox = det.bbox
                
                target = DetectedTarget(
                    timestamp=time.time(),
                    class_name=result.hypothesis.class_id,
                    class_id=int(result.hypothesis.class_id) if result.hypothesis.class_id.isdigit() else 0,
                    confidence=result.hypothesis.score,
                    bbox_x=int(bbox.center.position.x - bbox.size_x / 2),
                    bbox_y=int(bbox.center.position.y - bbox.size_y / 2),
                    bbox_w=int(bbox.size_x),
                    bbox_h=int(bbox.size_y),
                    # 3D position added if available from depth
                    world_x=None,
                    world_y=None,
                    world_z=None,
                )
                detections.append(target)
            
            with self._lock:
                self._detections = detections
                self._detection_count += 1
            
            # Notify callbacks with filtered detections
            filtered = self._filter_detections(detections)
            for cb in self._detection_callbacks:
                try:
                    cb(filtered)
                except Exception as e:
                    logger.error(f"Detection callback error: {e}")
                    
        except Exception as e:
            logger.error(f"Detection processing error: {e}")
    
    def _filter_detections(
        self, detections: list[DetectedTarget]
    ) -> list[DetectedTarget]:
        """Filter detections by exclusion map."""
        filtered = []
        for det in detections:
            if det.world_x is None:
                # No 3D position, include it
                filtered.append(det)
                continue
            
            excluded, _ = self.is_excluded(det.world_x, det.world_y, det.world_z)
            if not excluded:
                filtered.append(det)
        
        return filtered
    
    def _quat_to_euler(
        self, x: float, y: float, z: float, w: float
    ) -> tuple[float, float, float]:
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    # ========================================================================
    # Exclusion Map Management
    # ========================================================================
    
    def add_to_exclusion_map(
        self,
        x: float,
        y: float,
        z: float,
        radius: Optional[float] = None,
        target_id: Optional[str] = None,
    ) -> str:
        """
        Add a position to the exclusion map.
        
        Args:
            x, y, z: Position in NED frame
            radius: Exclusion radius (uses default if None)
            target_id: Optional ID (auto-generated if None)
        
        Returns:
            The ID of the exclusion entry
        """
        with self._lock:
            if target_id is None:
                target_id = f"target_{len(self._exclusion_map):04d}"
            
            if radius is None:
                radius = self._exclusion_radius
            
            entry = ExclusionEntry(
                id=target_id,
                x=x,
                y=y,
                z=z,
                timestamp=time.time(),
                radius=radius,
            )
            self._exclusion_map[target_id] = entry
            logger.info(f"Added exclusion: {target_id} at ({x:.2f}, {y:.2f}, {z:.2f})")
            return target_id
    
    def clear_exclusion_map(self) -> int:
        """Clear all exclusion entries. Returns count cleared."""
        with self._lock:
            count = len(self._exclusion_map)
            self._exclusion_map.clear()
            logger.info(f"Cleared {count} exclusion entries")
            return count
    
    def is_excluded(
        self, x: float, y: float, z: float
    ) -> tuple[bool, Optional[str]]:
        """Check if position is excluded. Returns (is_excluded, entry_id)."""
        with self._lock:
            for entry_id, entry in self._exclusion_map.items():
                dist = math.sqrt(
                    (x - entry.x) ** 2 +
                    (y - entry.y) ** 2 +
                    (z - entry.z) ** 2
                )
                if dist <= entry.radius:
                    return True, entry_id
            return False, None
    
    # ========================================================================
    # Callbacks
    # ========================================================================
    
    def register_vio_callback(self, callback: Callable[[VIOState], None]) -> None:
        """Register callback for VIO updates."""
        self._vio_callbacks.append(callback)
    
    def register_detection_callback(
        self, callback: Callable[[list[DetectedTarget]], None]
    ) -> None:
        """Register callback for detection updates."""
        self._detection_callbacks.append(callback)
    
    # ========================================================================
    # Status
    # ========================================================================
    
    def get_status(self) -> dict:
        """Get bridge status."""
        with self._lock:
            return {
                "running": self._running,
                "vio_valid": self._vio_state is not None and self._vio_state.valid,
                "vio_confidence": self._vio_state.confidence if self._vio_state else 0,
                "vio_count": self._vio_count,
                "detection_count": self._detection_count,
                "exclusion_map_size": len(self._exclusion_map),
                "last_vio_time": self._vio_state.timestamp if self._vio_state else None,
            }


# ============================================================================
# Singleton Instance
# ============================================================================

_instance: Optional[IsaacROSBridge] = None


def get_isaac_bridge() -> Optional[IsaacROSBridge]:
    """Get the Isaac ROS bridge singleton."""
    return _instance


def init_isaac_bridge(
    vio_topic: Optional[str] = None,
    detection_topic: Optional[str] = None,
    exclusion_radius: Optional[float] = None,
) -> IsaacROSBridge:
    """
    Initialize the Isaac ROS bridge singleton.
    
    Args:
        vio_topic: VIO topic (default from env or /visual_slam/tracking/odometry)
        detection_topic: Detection topic (default from env or /yolov8/detections)
        exclusion_radius: Default exclusion radius (default from env or 0.5m)
    """
    global _instance
    
    if _instance is not None:
        logger.warning("Isaac bridge already initialized, returning existing instance")
        return _instance
    
    # Get from environment if not specified
    if vio_topic is None:
        vio_topic = os.environ.get(
            "ISAAC_VIO_TOPIC", "/visual_slam/tracking/odometry"
        )
    if detection_topic is None:
        detection_topic = os.environ.get(
            "ISAAC_DETECTION_TOPIC", "/yolov8/detections"
        )
    if exclusion_radius is None:
        exclusion_radius = float(os.environ.get("EXCLUSION_RADIUS", "0.5"))
    
    _instance = IsaacROSBridge(
        vio_topic=vio_topic,
        detection_topic=detection_topic,
        exclusion_radius=exclusion_radius,
    )
    
    return _instance
