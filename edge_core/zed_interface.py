"""
ZED 2i Camera Interface for NOMAD.

Wraps the Stereolabs ZED SDK (pyzed.sl) for:
- Stereo camera capture
- Visual Inertial Odometry (VIO) / Positional Tracking
- Coordinate frame conversion (ZED → ArduPilot NED)

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum
from typing import Any, Tuple

logger = logging.getLogger(__name__)

# Lazy import of ZED SDK to allow module import on systems without ZED
sl: Any = None


def _lazy_import_zed() -> bool:
    """Lazy import ZED SDK. Returns True if successful."""
    global sl
    if sl is None:
        try:
            import pyzed.sl as _sl  # type: ignore[import]
            sl = _sl
            return True
        except ImportError:
            logger.warning("ZED SDK (pyzed.sl) not available")
            return False
    return True


class ZedTrackingState(Enum):
    """ZED Positional Tracking State."""
    OK = "OK"
    SEARCHING = "SEARCHING"
    OFF = "OFF"
    FPS_TOO_LOW = "FPS_TOO_LOW"


@dataclass(frozen=True, slots=True)
class ZedPose:
    """
    6DOF Pose from ZED Positional Tracking.
    
    Position is in meters relative to the initial pose.
    Rotation is in radians.
    """
    timestamp_ns: int  # Nanoseconds since epoch
    x: float  # Forward (ZED: positive X)
    y: float  # Up (ZED: positive Y)
    z: float  # Right (ZED: positive Z)
    roll: float  # Rotation around X axis
    pitch: float  # Rotation around Y axis
    yaw: float  # Rotation around Z axis
    confidence: float  # 0-100 tracking confidence
    tracking_state: ZedTrackingState

    def to_ned(self) -> "NEDPose":
        """
        Convert ZED coordinate frame to ArduPilot NED frame.
        
        ZED Frame (Left-handed, Y-up):
            X = Forward
            Y = Up
            Z = Right
        
        ArduPilot NED Frame (Right-handed, Z-down):
            X = North (Forward)
            Y = East (Right)
            Z = Down
        
        Conversion:
            NED_X = ZED_X (forward)
            NED_Y = ZED_Z (right)
            NED_Z = -ZED_Y (down = -up)
        
        Rotation conversion:
            NED_roll = ZED_roll
            NED_pitch = ZED_pitch
            NED_yaw = -ZED_yaw (sign flip for handedness)
        """
        return NEDPose(
            timestamp_ns=self.timestamp_ns,
            x=self.x,       # North = Forward
            y=self.z,       # East = Right
            z=-self.y,      # Down = -Up
            roll=self.roll,
            pitch=self.pitch,
            yaw=-self.yaw,  # Sign flip for coordinate system handedness
            confidence=self.confidence,
            tracking_state=self.tracking_state,
        )

    def to_dict(self) -> dict:
        """Convert to dictionary for serialization."""
        return {
            "timestamp_ns": self.timestamp_ns,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "roll": self.roll,
            "pitch": self.pitch,
            "yaw": self.yaw,
            "confidence": self.confidence,
            "tracking_state": self.tracking_state.value,
        }


@dataclass(frozen=True, slots=True)
class NEDPose:
    """
    6DOF Pose in ArduPilot NED Frame.
    
    Position:
        x = North (forward, meters)
        y = East (right, meters)
        z = Down (meters)
    
    Rotation (radians):
        roll = rotation around X (North) axis
        pitch = rotation around Y (East) axis
        yaw = rotation around Z (Down) axis, clockwise from North
    """
    timestamp_ns: int
    x: float  # North (forward)
    y: float  # East (right)
    z: float  # Down
    roll: float
    pitch: float
    yaw: float
    confidence: float
    tracking_state: ZedTrackingState

    @property
    def timestamp_us(self) -> int:
        """Timestamp in microseconds (for MAVLink)."""
        return self.timestamp_ns // 1000

    def to_dict(self) -> dict:
        """Convert to dictionary for serialization."""
        return {
            "timestamp_ns": self.timestamp_ns,
            "timestamp_us": self.timestamp_us,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "roll": self.roll,
            "pitch": self.pitch,
            "yaw": self.yaw,
            "confidence": self.confidence,
            "tracking_state": self.tracking_state.value,
        }


def quaternion_to_euler(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    """
    Convert quaternion to Euler angles (roll, pitch, yaw).
    
    Args:
        qx, qy, qz, qw: Quaternion components
    
    Returns:
        Tuple of (roll, pitch, yaw) in radians
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        # Use 90 degrees if out of range
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class ZedCamera:
    """
    ZED 2i Camera wrapper for NOMAD.
    
    Provides:
    - Camera initialization and configuration
    - Positional Tracking (VIO)
    - Frame capture for object detection
    - Coordinate frame conversion to NED
    
    Usage:
        camera = ZedCamera()
        if camera.open():
            while True:
                if camera.grab():
                    pose = camera.get_pose_ned()  # NED frame pose
                    image = camera.get_image()    # BGRA numpy array
            camera.close()
    """

    def __init__(
        self,
        resolution: str = "HD720",
        fps: int = 30,
        enable_tracking: bool = True,
        depth_mode: str = "PERFORMANCE",
        coordinate_units: str = "METER",
    ) -> None:
        """
        Initialize ZED camera configuration.
        
        Args:
            resolution: Camera resolution (VGA, HD720, HD1080, HD2K)
            fps: Frame rate (15, 30, 60, 100 depending on resolution)
            enable_tracking: Enable positional tracking (VIO)
            depth_mode: Depth sensing mode (PERFORMANCE, QUALITY, ULTRA, NEURAL)
            coordinate_units: Unit system (METER, CENTIMETER, MILLIMETER)
        """
        self.resolution = resolution
        self.fps = fps
        self.enable_tracking = enable_tracking
        self.depth_mode = depth_mode
        self.coordinate_units = coordinate_units
        
        self._zed: Any = None
        self._pose: Any = None
        self._runtime_params: Any = None
        self._image: Any = None
        self._is_open = False
        self._tracking_enabled = False

    @property
    def is_open(self) -> bool:
        """Check if camera is open."""
        return self._is_open

    @property
    def is_tracking(self) -> bool:
        """Check if positional tracking is enabled."""
        return self._tracking_enabled

    def open(self) -> bool:
        """
        Open the ZED camera and initialize positional tracking.
        
        Returns:
            True if successful, False otherwise
        """
        if not _lazy_import_zed():
            logger.error("ZED SDK not available")
            return False

        try:
            # Create camera instance
            self._zed = sl.Camera()
            
            # Configure init parameters
            init_params = sl.InitParameters()
            init_params.camera_resolution = getattr(
                sl.RESOLUTION, self.resolution, sl.RESOLUTION.HD720
            )
            init_params.camera_fps = self.fps
            init_params.depth_mode = getattr(
                sl.DEPTH_MODE, self.depth_mode, sl.DEPTH_MODE.PERFORMANCE
            )
            init_params.coordinate_units = getattr(
                sl.UNIT, self.coordinate_units, sl.UNIT.METER
            )
            init_params.coordinate_system = sl.COORDINATE_SYSTEM.LEFT_HANDED_Y_UP
            
            # Open camera
            status = self._zed.open(init_params)
            if status != sl.ERROR_CODE.SUCCESS:
                logger.error(f"Failed to open ZED camera: {status}")
                self._zed = None
                return False
            
            self._is_open = True
            logger.info(f"ZED camera opened: {self.resolution}@{self.fps}fps")
            
            # Initialize runtime parameters
            self._runtime_params = sl.RuntimeParameters()
            
            # Initialize pose holder
            self._pose = sl.Pose()
            
            # Initialize image mat
            self._image = sl.Mat()
            
            # Enable positional tracking if requested
            if self.enable_tracking:
                self._enable_positional_tracking()
            
            return True
            
        except Exception as e:
            logger.error(f"ZED camera open error: {e}")
            self._zed = None
            self._is_open = False
            return False

    def _enable_positional_tracking(self) -> bool:
        """Enable ZED positional tracking (VIO)."""
        if not self._is_open or not self._zed:
            return False
        
        try:
            tracking_params = sl.PositionalTrackingParameters()
            tracking_params.enable_area_memory = True
            tracking_params.enable_imu_fusion = True  # Use IMU for better tracking
            
            status = self._zed.enable_positional_tracking(tracking_params)
            if status != sl.ERROR_CODE.SUCCESS:
                logger.error(f"Failed to enable tracking: {status}")
                return False
            
            self._tracking_enabled = True
            logger.info("ZED positional tracking enabled (VIO)")
            return True
            
        except Exception as e:
            logger.error(f"Enable tracking error: {e}")
            return False

    def close(self) -> None:
        """Close the ZED camera and release resources."""
        if self._zed:
            try:
                if self._tracking_enabled:
                    self._zed.disable_positional_tracking()
                self._zed.close()
            except Exception as e:
                logger.error(f"ZED close error: {e}")
            finally:
                self._zed = None
                self._is_open = False
                self._tracking_enabled = False
                logger.info("ZED camera closed")

    def grab(self) -> bool:
        """
        Grab a new frame from the camera.
        
        Returns:
            True if frame grabbed successfully
        """
        if not self._is_open or not self._zed:
            return False
        
        try:
            status = self._zed.grab(self._runtime_params)
            return status == sl.ERROR_CODE.SUCCESS
        except Exception as e:
            logger.error(f"ZED grab error: {e}")
            return False

    def get_pose(self) -> ZedPose | None:
        """
        Get the current pose in ZED coordinate frame.
        
        Returns:
            ZedPose if tracking is active, None otherwise
        """
        if not self._tracking_enabled or not self._zed:
            return None
        
        try:
            tracking_state = self._zed.get_position(
                self._pose,
                sl.REFERENCE_FRAME.WORLD
            )
            
            # Get translation
            translation = self._pose.get_translation(sl.Translation())
            tx = translation.get()[0]
            ty = translation.get()[1]
            tz = translation.get()[2]
            
            # Get rotation as quaternion and convert to Euler
            orientation = self._pose.get_orientation(sl.Orientation())
            qx = orientation.get()[0]
            qy = orientation.get()[1]
            qz = orientation.get()[2]
            qw = orientation.get()[3]
            roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)
            
            # Get timestamp (nanoseconds since camera start)
            timestamp_ns = self._pose.timestamp.get_nanoseconds()
            
            # Map tracking state
            state_map = {
                sl.POSITIONAL_TRACKING_STATE.OK: ZedTrackingState.OK,
                sl.POSITIONAL_TRACKING_STATE.SEARCHING: ZedTrackingState.SEARCHING,
                sl.POSITIONAL_TRACKING_STATE.OFF: ZedTrackingState.OFF,
                sl.POSITIONAL_TRACKING_STATE.FPS_TOO_LOW: ZedTrackingState.FPS_TOO_LOW,
            }
            state = state_map.get(tracking_state, ZedTrackingState.OFF)
            
            # Confidence is 0-100
            confidence = self._pose.pose_confidence
            
            return ZedPose(
                timestamp_ns=timestamp_ns,
                x=tx,
                y=ty,
                z=tz,
                roll=roll,
                pitch=pitch,
                yaw=yaw,
                confidence=confidence,
                tracking_state=state,
            )
            
        except Exception as e:
            logger.error(f"Get pose error: {e}")
            return None

    def get_pose_ned(self) -> NEDPose | None:
        """
        Get the current pose converted to ArduPilot NED frame.
        
        Returns:
            NEDPose if tracking is active, None otherwise
        """
        pose = self.get_pose()
        if pose is None:
            return None
        return pose.to_ned()

    def get_image(self) -> Any | None:
        """
        Get the current left camera image.
        
        Returns:
            Numpy array (BGRA) if available, None otherwise
        """
        if not self._is_open or not self._zed:
            return None
        
        try:
            self._zed.retrieve_image(self._image, sl.VIEW.LEFT)
            return self._image.get_data()
        except Exception as e:
            logger.error(f"Get image error: {e}")
            return None

    def get_depth(self) -> Any | None:
        """
        Get the current depth map.
        
        Returns:
            Numpy array (float32, meters) if available, None otherwise
        """
        if not self._is_open or not self._zed:
            return None
        
        try:
            depth_mat = sl.Mat()
            self._zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
            return depth_mat.get_data()
        except Exception as e:
            logger.error(f"Get depth error: {e}")
            return None

    def reset_tracking(self) -> bool:
        """
        Reset positional tracking to current position as origin.
        
        Returns:
            True if successful
        """
        if not self._tracking_enabled or not self._zed:
            return False
        
        try:
            # Create identity transform (current position as new origin)
            transform = sl.Transform()
            transform.set_identity()
            status = self._zed.reset_positional_tracking(transform)
            return status == sl.ERROR_CODE.SUCCESS
        except Exception as e:
            logger.error(f"Reset tracking error: {e}")
            return False

    def get_camera_info(self) -> dict | None:
        """Get camera information including serial number and calibration."""
        if not self._is_open or not self._zed:
            return None
        
        try:
            info = self._zed.get_camera_information()
            return {
                "serial_number": info.serial_number,
                "model": str(info.camera_model),
                "firmware_version": info.camera_configuration.firmware_version,
                "resolution": {
                    "width": info.camera_configuration.resolution.width,
                    "height": info.camera_configuration.resolution.height,
                },
                "fps": info.camera_configuration.fps,
            }
        except Exception as e:
            logger.error(f"Get camera info error: {e}")
            return None

    def __enter__(self) -> "ZedCamera":
        """Context manager entry."""
        self.open()
        return self

    def __exit__(self, *args) -> None:
        """Context manager exit."""
        self.close()


# ============== Utility Functions ==============

def create_vio_message(pose: NEDPose) -> dict:
    """
    Create a VIO_UPDATE message dictionary for IPC.
    
    Args:
        pose: NEDPose from ZED tracking
    
    Returns:
        Dictionary suitable for IPCMessage
    """
    return {
        "timestamp_us": pose.timestamp_us,
        "x": pose.x,
        "y": pose.y,
        "z": pose.z,
        "roll": pose.roll,
        "pitch": pose.pitch,
        "yaw": pose.yaw,
        "confidence": pose.confidence,
        "tracking_state": pose.tracking_state.value,
    }


def is_zed_available() -> bool:
    """Check if ZED SDK is available on this system."""
    return _lazy_import_zed()
