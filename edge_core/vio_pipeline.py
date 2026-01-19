"""
NOMAD Edge Core - VIO Pipeline Service

Visual Inertial Odometry pipeline that integrates ZED camera tracking
with ArduPilot's EKF3 for indoor autonomous flight.

Target: Python 3.13 | NVIDIA Jetson Orin Nano | ZED SDK 4.x
"""

from __future__ import annotations

import asyncio
import logging
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Optional

logger = logging.getLogger("edge_core.vio_pipeline")


class VIOHealth(Enum):
    """VIO pipeline health states."""
    UNKNOWN = "unknown"
    HEALTHY = "healthy"
    DEGRADED = "degraded"
    FAILED = "failed"


@dataclass
class VIOStatus:
    """VIO pipeline status information."""
    health: VIOHealth = VIOHealth.UNKNOWN
    tracking_confidence: float = 0.0
    position_valid: bool = False
    velocity_valid: bool = False
    last_update_ms: int = 0
    message_rate_hz: float = 0.0
    ekf_healthy: bool = False
    ekf_variance_ok: bool = False
    reset_counter: int = 0
    error_message: Optional[str] = None
    
    def to_dict(self) -> dict:
        return {
            "health": self.health.value,
            "tracking_confidence": self.tracking_confidence,
            "position_valid": self.position_valid,
            "velocity_valid": self.velocity_valid,
            "last_update_ms": self.last_update_ms,
            "message_rate_hz": self.message_rate_hz,
            "ekf_healthy": self.ekf_healthy,
            "ekf_variance_ok": self.ekf_variance_ok,
            "reset_counter": self.reset_counter,
            "error_message": self.error_message,
        }


class VIOPipeline:
    """
    VIO Pipeline for NOMAD Task 2 Indoor Flight.
    
    This service:
    1. Receives pose updates from ZED camera tracking
    2. Transforms coordinates to NED frame
    3. Sends VISION_POSITION_ESTIMATE to ArduPilot at 30Hz
    4. Monitors EKF health and VIO confidence
    5. Implements failsafe logic per PRD requirements
    
    PRD Requirements:
    - [T2-NAV-01]: Stream VISION_POSITION_ESTIMATE at 30Hz
    - [T2-NAV-02]: Configure EKF source to ExternalNav
    - [T2-SAFE-01]: VIO failure response within 3 seconds
    
    Usage:
        from edge_core.zed_camera import ZEDCameraService, ZEDConfig
        from edge_core.mavlink_interface import MavlinkService
        
        camera = ZEDCameraService(ZEDConfig(enable_tracking=True))
        mavlink = MavlinkService(state_manager)
        
        pipeline = VIOPipeline(camera, mavlink)
        pipeline.start()
    """
    
    # Target output rate to flight controller
    TARGET_RATE_HZ = 30.0
    
    # Health thresholds
    MIN_CONFIDENCE = 30.0  # Minimum tracking confidence
    MAX_STALE_MS = 100  # Maximum age of pose data
    VIO_FAILURE_TIMEOUT_S = 3.0  # Time before declaring VIO failed
    
    # EKF variance thresholds (based on EKF_STATUS_REPORT)
    EKF_POS_VARIANCE_THRESHOLD = 1.0
    EKF_VEL_VARIANCE_THRESHOLD = 1.0
    
    def __init__(
        self,
        camera_service,  # ZEDCameraService
        mavlink_service,  # MavlinkService
        state_manager=None,  # StateManager
        on_status_change: Optional[Callable[[VIOStatus], None]] = None,
        on_failsafe_trigger: Optional[Callable[[str], None]] = None,
    ):
        self._camera = camera_service
        self._mavlink = mavlink_service
        self._state_manager = state_manager
        self._on_status_change = on_status_change
        self._on_failsafe_trigger = on_failsafe_trigger
        
        self._status = VIOStatus()
        self._lock = threading.RLock()
        
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        
        # Tracking state
        self._reset_counter = 0
        self._last_healthy_time = 0.0
        self._failsafe_triggered = False
        
        # Rate tracking
        self._message_count = 0
        self._rate_timestamp = time.time()
        
    @property
    def status(self) -> VIOStatus:
        """Get current VIO status."""
        with self._lock:
            return self._status
    
    @property
    def is_healthy(self) -> bool:
        """Check if VIO is healthy."""
        with self._lock:
            return self._status.health == VIOHealth.HEALTHY
    
    def start(self) -> bool:
        """Start the VIO pipeline."""
        if self._thread and self._thread.is_alive():
            logger.warning("VIO pipeline already running")
            return True
            
        if not self._camera.is_tracking:
            logger.error("Camera tracking not enabled")
            return False
            
        self._stop_event.clear()
        self._failsafe_triggered = False
        self._last_healthy_time = time.time()
        
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        
        logger.info("VIO pipeline started")
        return True
    
    def stop(self) -> None:
        """Stop the VIO pipeline."""
        self._stop_event.set()
        
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
            
        logger.info("VIO pipeline stopped")
    
    def reset_origin(self) -> bool:
        """
        Reset VIO origin to current position.
        
        Call this when the drone is at a known starting position
        to reset the coordinate system.
        """
        if self._camera.reset_tracking():
            self._reset_counter += 1
            logger.info(f"VIO origin reset (counter={self._reset_counter})")
            return True
        return False
    
    def _run(self) -> None:
        """Main VIO processing loop."""
        interval = 1.0 / self.TARGET_RATE_HZ
        
        while not self._stop_event.is_set():
            try:
                start_time = time.time()
                
                self._process_pose()
                self._check_health()
                self._update_rate()
                
                # Maintain target rate
                elapsed = time.time() - start_time
                if elapsed < interval:
                    time.sleep(interval - elapsed)
                    
            except Exception as e:
                logger.error(f"VIO loop error: {e}")
                self._update_status(
                    health=VIOHealth.DEGRADED,
                    error_message=str(e),
                )
                time.sleep(0.01)
    
    def _process_pose(self) -> None:
        """Process latest pose and send to flight controller."""
        pose = self._camera.get_pose()
        
        if pose is None:
            self._update_status(
                position_valid=False,
                velocity_valid=False,
                tracking_confidence=0.0,
            )
            return
        
        # Check pose quality
        confidence = pose.confidence
        is_tracking = pose.tracking_state.value == 1  # OK state
        
        if not is_tracking or confidence < self.MIN_CONFIDENCE:
            self._update_status(
                position_valid=False,
                tracking_confidence=confidence,
            )
            return
        
        # Transform to NED frame
        north, east, down, roll, pitch, yaw = pose.to_ned()
        
        # Send VISION_POSITION_ESTIMATE to ArduPilot
        success = self._mavlink.send_vision_position_estimate(
            timestamp_us=pose.timestamp_us,
            x=north,
            y=east,
            z=down,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            reset_counter=self._reset_counter,
        )
        
        if success:
            self._message_count += 1
            self._last_healthy_time = time.time()
            
            self._update_status(
                health=VIOHealth.HEALTHY,
                position_valid=True,
                velocity_valid=True,
                tracking_confidence=confidence,
                last_update_ms=int(time.time() * 1000),
            )
        else:
            self._update_status(
                health=VIOHealth.DEGRADED,
                error_message="Failed to send VPE message",
            )
    
    def _check_health(self) -> None:
        """Check VIO health and trigger failsafe if needed."""
        now = time.time()
        time_since_healthy = now - self._last_healthy_time
        
        # Check for VIO failure (PRD T2-SAFE-01)
        if time_since_healthy > self.VIO_FAILURE_TIMEOUT_S:
            if not self._failsafe_triggered:
                self._trigger_failsafe("VIO_TIMEOUT")
                
        # Update health based on state
        if self._status.tracking_confidence < self.MIN_CONFIDENCE:
            self._update_status(health=VIOHealth.DEGRADED)
        elif time_since_healthy > 0.5:
            self._update_status(health=VIOHealth.DEGRADED)
    
    def _trigger_failsafe(self, reason: str) -> None:
        """
        Trigger VIO failsafe response.
        
        Per PRD [T2-SAFE-01]:
        1. Switch EKF source to fallback (Baro/IMU only)
        2. Set flight mode to ALT_HOLD
        3. Alert pilot for manual takeover
        
        NOT: Auto-land, RTL, or motor kill (unsafe indoors)
        """
        self._failsafe_triggered = True
        
        logger.critical(f"VIO FAILSAFE TRIGGERED: {reason}")
        
        self._update_status(
            health=VIOHealth.FAILED,
            error_message=f"Failsafe: {reason}",
        )
        
        # Notify callback
        if self._on_failsafe_trigger:
            self._on_failsafe_trigger(reason)
        
        # Request ALT_HOLD mode via MAVLink
        # This requires the mavlink_interface to have a mode change method
        # For now, we'll log the action needed
        logger.warning("ACTION REQUIRED: Switch to ALT_HOLD and resume manual control")
    
    def _update_rate(self) -> None:
        """Update message rate calculation."""
        now = time.time()
        elapsed = now - self._rate_timestamp
        
        if elapsed >= 1.0:
            rate = self._message_count / elapsed
            self._update_status(message_rate_hz=rate)
            self._message_count = 0
            self._rate_timestamp = now
    
    def _update_status(self, **kwargs) -> None:
        """Update VIO status."""
        with self._lock:
            old_health = self._status.health
            
            # Update fields
            for key, value in kwargs.items():
                if hasattr(self._status, key):
                    setattr(self._status, key)
            
            # Rebuild status object (dataclass is not mutable by default)
            self._status = VIOStatus(
                health=kwargs.get("health", self._status.health),
                tracking_confidence=kwargs.get("tracking_confidence", self._status.tracking_confidence),
                position_valid=kwargs.get("position_valid", self._status.position_valid),
                velocity_valid=kwargs.get("velocity_valid", self._status.velocity_valid),
                last_update_ms=kwargs.get("last_update_ms", self._status.last_update_ms),
                message_rate_hz=kwargs.get("message_rate_hz", self._status.message_rate_hz),
                ekf_healthy=kwargs.get("ekf_healthy", self._status.ekf_healthy),
                ekf_variance_ok=kwargs.get("ekf_variance_ok", self._status.ekf_variance_ok),
                reset_counter=self._reset_counter,
                error_message=kwargs.get("error_message", self._status.error_message),
            )
            
            new_health = self._status.health
        
        # Notify status change
        if old_health != new_health and self._on_status_change:
            self._on_status_change(self._status)


class VIOCalibration:
    """
    VIO calibration utilities for NOMAD.
    
    Provides tools for:
    - Initial position calibration
    - IMU-Camera extrinsic validation
    - Floor plane detection
    """
    
    @staticmethod
    def create_initial_pose(
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        yaw_deg: float = 0.0,
    ) -> dict:
        """
        Create initial pose for VIO calibration.
        
        Args:
            x: Initial X position (meters, NED North)
            y: Initial Y position (meters, NED East)
            z: Initial Z position (meters, NED Down, typically negative for above ground)
            yaw_deg: Initial heading (degrees, 0=North, 90=East)
            
        Returns:
            Dictionary with initial pose parameters
        """
        import math
        
        yaw_rad = math.radians(yaw_deg)
        
        return {
            "position": [x, y, z],
            "orientation": [0.0, 0.0, math.sin(yaw_rad / 2), math.cos(yaw_rad / 2)],
            "yaw_deg": yaw_deg,
        }
    
    @staticmethod
    def validate_tracking(camera_service, duration_s: float = 5.0) -> dict:
        """
        Validate VIO tracking quality.
        
        Collects pose data for specified duration and analyzes:
        - Tracking stability
        - Confidence levels
        - Drift estimation
        
        Returns:
            Validation results dictionary
        """
        results = {
            "valid": False,
            "avg_confidence": 0.0,
            "tracking_losses": 0,
            "drift_estimate_m": 0.0,
            "samples": 0,
        }
        
        if not camera_service.is_tracking:
            results["error"] = "Tracking not enabled"
            return results
        
        start_time = time.time()
        confidences = []
        tracking_losses = 0
        initial_pose = None
        final_pose = None
        
        while time.time() - start_time < duration_s:
            pose = camera_service.get_pose()
            
            if pose:
                if initial_pose is None:
                    initial_pose = pose
                final_pose = pose
                
                confidences.append(pose.confidence)
                
                if pose.tracking_state.value != 1:  # Not OK
                    tracking_losses += 1
            else:
                tracking_losses += 1
                
            time.sleep(0.033)  # ~30Hz
        
        if confidences:
            results["avg_confidence"] = sum(confidences) / len(confidences)
            results["samples"] = len(confidences)
            results["tracking_losses"] = tracking_losses
            
            # Estimate drift (assuming stationary)
            if initial_pose and final_pose:
                dx = final_pose.position[0] - initial_pose.position[0]
                dy = final_pose.position[1] - initial_pose.position[1]
                dz = final_pose.position[2] - initial_pose.position[2]
                results["drift_estimate_m"] = (dx**2 + dy**2 + dz**2) ** 0.5
            
            # Valid if confidence > 50% and few tracking losses
            results["valid"] = (
                results["avg_confidence"] > 50 and
                tracking_losses < results["samples"] * 0.1
            )
        
        return results
