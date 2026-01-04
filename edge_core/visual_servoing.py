"""
Visual Servoing Controller for NOMAD Task 2.

Implements target tracking using PID control to convert
bounding box position errors into velocity/gimbal commands.

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import NamedTuple, TYPE_CHECKING

if TYPE_CHECKING:
    from .mavlink_interface import MavlinkService
    from .task2 import ExclusionMap

logger = logging.getLogger(__name__)


class ServoMode(Enum):
    """Visual servoing control mode."""
    
    DRONE_BODY = auto()  # Control via drone velocity (yaw rate + Z velocity)
    GIMBAL = auto()       # Control via gimbal angles (pitch + yaw)


class BoundingBox(NamedTuple):
    """Bounding box in pixel or normalized coordinates."""
    
    x1: float  # Left edge
    y1: float  # Top edge
    x2: float  # Right edge
    y2: float  # Bottom edge
    
    @property
    def center_x(self) -> float:
        """Center X coordinate."""
        return (self.x1 + self.x2) / 2
    
    @property
    def center_y(self) -> float:
        """Center Y coordinate."""
        return (self.y1 + self.y2) / 2
    
    @property
    def width(self) -> float:
        """Bounding box width."""
        return self.x2 - self.x1
    
    @property
    def height(self) -> float:
        """Bounding box height."""
        return self.y2 - self.y1
    
    @property
    def area(self) -> float:
        """Bounding box area."""
        return self.width * self.height
    
    @classmethod
    def from_normalized(
        cls,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
    ) -> "BoundingBox":
        """Create from normalized (0-1) coordinates."""
        return cls(x1, y1, x2, y2)
    
    @classmethod
    def from_pixels(
        cls,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        image_width: int,
        image_height: int,
    ) -> "BoundingBox":
        """Create from pixel coordinates, normalizing to 0-1."""
        return cls(
            x1 / image_width,
            y1 / image_height,
            x2 / image_width,
            y2 / image_height,
        )


@dataclass
class TrackingError:
    """
    Normalized tracking error from image center.
    
    Values range from -1.0 to 1.0:
    - error_x: Negative = target left, Positive = target right
    - error_y: Negative = target above, Positive = target below
    """
    
    error_x: float  # Horizontal error (-1 to 1)
    error_y: float  # Vertical error (-1 to 1)
    magnitude: float = 0.0  # Total error magnitude
    
    def __post_init__(self):
        """Calculate magnitude if not provided."""
        if self.magnitude == 0.0:
            self.magnitude = (self.error_x ** 2 + self.error_y ** 2) ** 0.5
    
    def is_aligned(self, threshold: float = 0.1) -> bool:
        """Check if error is within alignment threshold."""
        return self.magnitude < threshold


@dataclass
class PIDState:
    """Internal state for a single-axis PID controller."""
    
    last_error: float = 0.0
    integral: float = 0.0
    last_time: float = field(default_factory=time.time)
    last_output: float = 0.0


@dataclass
class PIDConfig:
    """PID controller configuration."""
    
    kp: float = 1.0           # Proportional gain
    ki: float = 0.0           # Integral gain
    kd: float = 0.1           # Derivative gain
    output_limit: float = 1.0  # Max output magnitude
    integral_limit: float = 0.5  # Anti-windup limit
    deadband: float = 0.02    # Error deadband (ignore small errors)


class PIDController:
    """
    Single-axis PID controller with anti-windup and filtering.
    
    Used for converting tracking error to control output.
    """
    
    def __init__(self, config: PIDConfig | None = None) -> None:
        """
        Initialize PID controller.
        
        Args:
            config: PID configuration (uses defaults if None)
        """
        self.config = config or PIDConfig()
        self._state = PIDState()
    
    def update(self, error: float, dt: float | None = None) -> float:
        """
        Calculate PID output for given error.
        
        Args:
            error: Current error value
            dt: Time delta (auto-calculated if None)
        
        Returns:
            Control output (clamped to output_limit)
        """
        current_time = time.time()
        
        if dt is None:
            dt = current_time - self._state.last_time
            if dt <= 0:
                dt = 0.01
        
        # Apply deadband
        if abs(error) < self.config.deadband:
            error = 0.0
        
        # Proportional term
        p_term = self.config.kp * error
        
        # Integral term with anti-windup
        self._state.integral += error * dt
        self._state.integral = max(
            -self.config.integral_limit,
            min(self.config.integral_limit, self._state.integral)
        )
        i_term = self.config.ki * self._state.integral
        
        # Derivative term (on error)
        derivative = (error - self._state.last_error) / dt if dt > 0 else 0.0
        d_term = self.config.kd * derivative
        
        # Update state
        self._state.last_error = error
        self._state.last_time = current_time
        
        # Calculate and clamp output
        output = p_term + i_term + d_term
        output = max(-self.config.output_limit, min(self.config.output_limit, output))
        
        self._state.last_output = output
        return output
    
    def reset(self) -> None:
        """Reset controller state."""
        self._state = PIDState()
    
    @property
    def last_output(self) -> float:
        """Get last computed output."""
        return self._state.last_output


@dataclass
class TargetTrackerConfig:
    """Configuration for the target tracker."""
    
    # Control mode
    mode: ServoMode = ServoMode.GIMBAL
    
    # PID configurations
    horizontal_pid: PIDConfig = field(default_factory=lambda: PIDConfig(
        kp=0.8, ki=0.1, kd=0.15, output_limit=1.0
    ))
    vertical_pid: PIDConfig = field(default_factory=lambda: PIDConfig(
        kp=0.6, ki=0.08, kd=0.12, output_limit=1.0
    ))
    
    # Alignment thresholds
    alignment_threshold: float = 0.08  # Error magnitude for "aligned"
    fine_alignment_threshold: float = 0.03  # Tighter threshold for payload
    
    # Velocity scaling (for DRONE_BODY mode)
    max_yaw_rate: float = 30.0      # deg/s
    max_z_velocity: float = 0.5     # m/s (vertical)
    
    # Gimbal scaling (for GIMBAL mode)
    max_gimbal_pitch_rate: float = 45.0  # deg/s
    max_gimbal_yaw_rate: float = 60.0    # deg/s
    
    # Gimbal limits
    gimbal_pitch_min: float = -90.0  # Looking straight down
    gimbal_pitch_max: float = 0.0    # Looking at horizon
    gimbal_yaw_min: float = -180.0
    gimbal_yaw_max: float = 180.0
    
    # Tracking parameters
    tracking_timeout: float = 2.0    # Seconds without update before lost
    min_confidence: float = 0.5      # Minimum detection confidence
    min_bbox_area: float = 0.001     # Minimum bounding box area (normalized)


@dataclass
class TrackingState:
    """Current state of target tracking."""
    
    is_tracking: bool = False
    is_aligned: bool = False
    is_fine_aligned: bool = False
    
    # Current target info
    target_bbox: BoundingBox | None = None
    target_confidence: float = 0.0
    target_class: str = ""
    
    # Current error
    error: TrackingError | None = None
    
    # Control outputs (normalized -1 to 1)
    horizontal_output: float = 0.0
    vertical_output: float = 0.0
    
    # Timing
    last_update: float = 0.0
    tracking_duration: float = 0.0
    
    # Statistics
    alignment_time: float = 0.0  # Time spent aligned


class TargetTracker:
    """
    Visual servoing target tracker.
    
    Accepts bounding box detections and calculates control outputs
    to center the target in the camera frame.
    
    Example usage:
        tracker = TargetTracker()
        
        # Update with new detection
        tracker.update_target(
            bbox=BoundingBox.from_normalized(0.3, 0.4, 0.5, 0.6),
            confidence=0.85,
            class_name="fire"
        )
        
        # Get control commands
        if tracker.state.is_tracking:
            # For gimbal mode
            pitch_cmd, yaw_cmd = tracker.get_gimbal_command()
            mavlink.send_gimbal_rate_command(pitch_cmd, yaw_cmd)
            
            # Check alignment for payload
            if tracker.state.is_fine_aligned:
                mavlink.trigger_payload(2000)  # Fire!
    """
    
    def __init__(
        self,
        config: TargetTrackerConfig | None = None,
    ) -> None:
        """
        Initialize target tracker.
        
        Args:
            config: Tracker configuration (uses defaults if None)
        """
        self.config = config or TargetTrackerConfig()
        self._state = TrackingState()
        
        # Create PID controllers
        self._horizontal_pid = PIDController(self.config.horizontal_pid)
        self._vertical_pid = PIDController(self.config.vertical_pid)
        
        # Track start time for duration
        self._tracking_start: float = 0.0
        self._aligned_start: float = 0.0
        
        # Current gimbal angles (for position-based control)
        self._gimbal_pitch: float = -45.0  # Default looking down
        self._gimbal_yaw: float = 0.0
    
    @property
    def state(self) -> TrackingState:
        """Get current tracking state (read-only copy)."""
        return self._state
    
    @property
    def is_tracking(self) -> bool:
        """Check if currently tracking a target."""
        return self._state.is_tracking
    
    @property
    def is_aligned(self) -> bool:
        """Check if target is aligned (coarse)."""
        return self._state.is_aligned
    
    @property
    def is_fine_aligned(self) -> bool:
        """Check if target is fine-aligned (ready for payload)."""
        return self._state.is_fine_aligned
    
    def update_target(
        self,
        bbox: BoundingBox,
        confidence: float = 1.0,
        class_name: str = "",
    ) -> TrackingState:
        """
        Update tracker with new target detection.
        
        Args:
            bbox: Target bounding box (normalized 0-1 coordinates)
            confidence: Detection confidence (0-1)
            class_name: Detected class name
        
        Returns:
            Updated tracking state
        """
        current_time = time.time()
        
        # Validate detection
        if confidence < self.config.min_confidence:
            logger.debug(f"Detection confidence {confidence:.2f} below threshold")
            return self._state
        
        if bbox.area < self.config.min_bbox_area:
            logger.debug(f"Detection area {bbox.area:.4f} below threshold")
            return self._state
        
        # Calculate error from center
        # Image center is at (0.5, 0.5) in normalized coordinates
        error_x = bbox.center_x - 0.5  # Positive = target right of center
        error_y = bbox.center_y - 0.5  # Positive = target below center
        
        # Scale to -1 to 1 range (center ± 0.5 -> ± 1.0)
        error_x_normalized = error_x * 2.0
        error_y_normalized = error_y * 2.0
        
        error = TrackingError(
            error_x=error_x_normalized,
            error_y=error_y_normalized,
        )
        
        # Update PID controllers
        horizontal_output = self._horizontal_pid.update(error_x_normalized)
        vertical_output = self._vertical_pid.update(error_y_normalized)
        
        # Check alignment
        is_aligned = error.is_aligned(self.config.alignment_threshold)
        is_fine_aligned = error.is_aligned(self.config.fine_alignment_threshold)
        
        # Track alignment duration
        if is_aligned and not self._state.is_aligned:
            self._aligned_start = current_time
        
        alignment_time = 0.0
        if is_aligned and self._aligned_start > 0:
            alignment_time = current_time - self._aligned_start
        
        # Track tracking duration
        if not self._state.is_tracking:
            self._tracking_start = current_time
        
        tracking_duration = current_time - self._tracking_start
        
        # Update state
        self._state = TrackingState(
            is_tracking=True,
            is_aligned=is_aligned,
            is_fine_aligned=is_fine_aligned,
            target_bbox=bbox,
            target_confidence=confidence,
            target_class=class_name,
            error=error,
            horizontal_output=horizontal_output,
            vertical_output=vertical_output,
            last_update=current_time,
            tracking_duration=tracking_duration,
            alignment_time=alignment_time,
        )
        
        return self._state
    
    def update_from_detection(
        self,
        detection_data: dict,
    ) -> TrackingState:
        """
        Update tracker from detection message dictionary.
        
        Args:
            detection_data: Detection dict with bbox, confidence, class_name
        
        Returns:
            Updated tracking state
        """
        bbox_data = detection_data.get("bbox", {})
        
        bbox = BoundingBox.from_normalized(
            x1=bbox_data.get("x1", 0.0),
            y1=bbox_data.get("y1", 0.0),
            x2=bbox_data.get("x2", 0.0),
            y2=bbox_data.get("y2", 0.0),
        )
        
        return self.update_target(
            bbox=bbox,
            confidence=detection_data.get("confidence", 0.0),
            class_name=detection_data.get("class_name", ""),
        )
    
    def check_timeout(self) -> bool:
        """
        Check if tracking has timed out.
        
        Returns:
            True if tracking was lost due to timeout
        """
        if not self._state.is_tracking:
            return False
        
        elapsed = time.time() - self._state.last_update
        if elapsed > self.config.tracking_timeout:
            self.reset()
            logger.info("Tracking lost - timeout")
            return True
        
        return False
    
    def reset(self) -> None:
        """Reset tracker state and PID controllers."""
        self._state = TrackingState()
        self._horizontal_pid.reset()
        self._vertical_pid.reset()
        self._tracking_start = 0.0
        self._aligned_start = 0.0
        logger.debug("Tracker reset")
    
    def get_velocity_command(self) -> tuple[float, float, float, float]:
        """
        Get drone body velocity command (for DRONE_BODY mode).
        
        Returns:
            Tuple of (vx, vy, vz, yaw_rate):
            - vx: Forward velocity (m/s) - typically 0 for hovering
            - vy: Right velocity (m/s) - typically 0 for hovering
            - vz: Down velocity (m/s) - positive = descend
            - yaw_rate: Yaw rate (rad/s) - positive = clockwise
        
        Note:
            In visual servoing, we typically:
            - Use yaw_rate to track horizontal error (pan left/right)
            - Use vz to track vertical error (climb/descend)
            - Keep vx, vy at 0 to hover in place
        """
        if not self._state.is_tracking:
            return (0.0, 0.0, 0.0, 0.0)
        
        # Convert normalized output to actual rates
        # Horizontal output -> yaw rate (positive output = target right -> yaw right)
        yaw_rate_deg = self._state.horizontal_output * self.config.max_yaw_rate
        yaw_rate_rad = yaw_rate_deg * 3.14159 / 180.0
        
        # Vertical output -> Z velocity (positive output = target below -> descend)
        vz = self._state.vertical_output * self.config.max_z_velocity
        
        return (0.0, 0.0, vz, yaw_rate_rad)
    
    def get_gimbal_rate_command(self) -> tuple[float, float]:
        """
        Get gimbal rate command (for GIMBAL mode with rate control).
        
        Returns:
            Tuple of (pitch_rate, yaw_rate) in deg/s:
            - pitch_rate: Positive = tilt up
            - yaw_rate: Positive = pan right
        """
        if not self._state.is_tracking:
            return (0.0, 0.0)
        
        # Vertical output -> pitch rate
        # Target below center (positive error) -> tilt down (negative pitch rate)
        pitch_rate = -self._state.vertical_output * self.config.max_gimbal_pitch_rate
        
        # Horizontal output -> yaw rate
        # Target right of center (positive error) -> pan right (positive yaw rate)
        yaw_rate = self._state.horizontal_output * self.config.max_gimbal_yaw_rate
        
        return (pitch_rate, yaw_rate)
    
    def get_gimbal_angle_command(self, dt: float = 0.02) -> tuple[float, float]:
        """
        Get gimbal angle command (for GIMBAL mode with position control).
        
        Integrates rate commands to produce target angles.
        
        Args:
            dt: Time delta since last call (seconds)
        
        Returns:
            Tuple of (pitch_angle, yaw_angle) in degrees
        """
        pitch_rate, yaw_rate = self.get_gimbal_rate_command()
        
        # Integrate rates
        self._gimbal_pitch += pitch_rate * dt
        self._gimbal_yaw += yaw_rate * dt
        
        # Clamp to limits
        self._gimbal_pitch = max(
            self.config.gimbal_pitch_min,
            min(self.config.gimbal_pitch_max, self._gimbal_pitch)
        )
        self._gimbal_yaw = max(
            self.config.gimbal_yaw_min,
            min(self.config.gimbal_yaw_max, self._gimbal_yaw)
        )
        
        return (self._gimbal_pitch, self._gimbal_yaw)
    
    def set_gimbal_angles(self, pitch: float, yaw: float) -> None:
        """
        Set current gimbal angles (for synchronization with actual gimbal).
        
        Args:
            pitch: Current gimbal pitch in degrees
            yaw: Current gimbal yaw in degrees
        """
        self._gimbal_pitch = pitch
        self._gimbal_yaw = yaw
    
    def set_mode(self, mode: ServoMode) -> None:
        """
        Change servoing mode.
        
        Args:
            mode: New servo mode (DRONE_BODY or GIMBAL)
        """
        if self.config.mode != mode:
            self.config.mode = mode
            self.reset()
            logger.info(f"Servo mode changed to {mode.name}")


class Task2Controller:
    """
    High-level controller for Task 2 autonomous operation.
    
    Integrates target tracking, exclusion map, and payload control.
    
    Example:
        controller = Task2Controller(mavlink, exclusion_map)
        
        # In detection callback:
        controller.process_detection(detection_data, target_position_3d)
    """
    
    def __init__(
        self,
        mavlink_service: "MavlinkService",
        exclusion_map: "ExclusionMap",
        tracker_config: TargetTrackerConfig | None = None,
        payload_pwm_on: int = 2000,
        payload_pwm_off: int = 1000,
        payload_duration: float = 2.0,
        servo_channel: int = 9,
    ) -> None:
        """
        Initialize Task 2 controller.
        
        Args:
            mavlink_service: MAVLink interface for sending commands
            exclusion_map: Map of extinguished targets
            tracker_config: Target tracker configuration
            payload_pwm_on: PWM value to activate payload
            payload_pwm_off: PWM value to deactivate payload
            payload_duration: How long to activate payload (seconds)
            servo_channel: Servo channel for payload
        """
        self.mavlink = mavlink_service
        self.exclusion_map = exclusion_map
        self.tracker = TargetTracker(tracker_config)
        
        # Payload settings
        self.payload_pwm_on = payload_pwm_on
        self.payload_pwm_off = payload_pwm_off
        self.payload_duration = payload_duration
        self.servo_channel = servo_channel
        
        # State
        self._is_active = False
        self._is_dispensing = False
        self._dispense_start: float = 0.0
        self._current_target_position: tuple[float, float, float] | None = None
        
        # Control loop timing
        self._last_control_time: float = 0.0
        self._control_interval: float = 0.02  # 50 Hz
    
    @property
    def is_active(self) -> bool:
        """Check if controller is active."""
        return self._is_active
    
    @property
    def is_dispensing(self) -> bool:
        """Check if currently dispensing payload."""
        return self._is_dispensing
    
    def activate(self) -> None:
        """Activate Task 2 control loop."""
        self._is_active = True
        self.tracker.reset()
        logger.info("Task 2 controller activated")
    
    def deactivate(self) -> None:
        """Deactivate Task 2 control loop."""
        self._is_active = False
        self.tracker.reset()
        self._stop_payload()
        self.mavlink.stop_velocity()
        logger.info("Task 2 controller deactivated")
    
    def process_detection(
        self,
        detection_data: dict,
        target_position: tuple[float, float, float] | None = None,
    ) -> bool:
        """
        Process a detection and execute control loop.
        
        Args:
            detection_data: Detection message dictionary
            target_position: 3D position (x, y, z) for exclusion checking
        
        Returns:
            True if control command was sent
        """
        if not self._is_active:
            return False
        
        # Check exclusion map
        if target_position is not None:
            x, y, z = target_position
            if self.exclusion_map.is_target_extinguished(x, y, z):
                logger.debug("Target in exclusion map, ignoring")
                return False
            self._current_target_position = target_position
        
        # Update tracker
        self.tracker.update_from_detection(detection_data)
        
        # Execute control loop
        return self._execute_control()
    
    def update(self) -> bool:
        """
        Periodic update for control loop.
        
        Call this at regular intervals to maintain tracking.
        
        Returns:
            True if control command was sent
        """
        if not self._is_active:
            return False
        
        # Check tracking timeout
        self.tracker.check_timeout()
        
        # Handle ongoing dispensing
        if self._is_dispensing:
            self._check_dispense_complete()
        
        # Execute control at configured rate
        current_time = time.time()
        if current_time - self._last_control_time < self._control_interval:
            return False
        
        self._last_control_time = current_time
        return self._execute_control()
    
    def _execute_control(self) -> bool:
        """Execute control based on current tracking state."""
        if not self.tracker.is_tracking:
            return False
        
        state = self.tracker.state
        
        # Check alignment for payload trigger
        if state.is_fine_aligned and not self._is_dispensing:
            logger.info(
                f"Target aligned! Triggering payload "
                f"(alignment time: {state.alignment_time:.2f}s)"
            )
            self._trigger_payload()
            return True
        
        # Send control commands based on mode
        if self.tracker.config.mode == ServoMode.DRONE_BODY:
            vx, vy, vz, yaw_rate = self.tracker.get_velocity_command()
            self.mavlink.send_velocity_command(vx, vy, vz, yaw_rate)
        else:  # GIMBAL mode
            pitch_rate, yaw_rate = self.tracker.get_gimbal_rate_command()
            self.mavlink.send_gimbal_rate_command(pitch_rate, yaw_rate)
        
        return True
    
    def _trigger_payload(self) -> None:
        """Trigger payload release."""
        if self._is_dispensing:
            return
        
        self._is_dispensing = True
        self._dispense_start = time.time()
        
        # Activate servo
        self.mavlink.trigger_payload(
            self.payload_pwm_on,
            self.servo_channel,
        )
        
        logger.info(f"Payload activated (PWM={self.payload_pwm_on})")
    
    def _stop_payload(self) -> None:
        """Stop payload release."""
        if not self._is_dispensing:
            return
        
        self._is_dispensing = False
        
        # Deactivate servo
        self.mavlink.trigger_payload(
            self.payload_pwm_off,
            self.servo_channel,
        )
        
        logger.info(f"Payload deactivated (PWM={self.payload_pwm_off})")
    
    def _check_dispense_complete(self) -> None:
        """Check if dispensing duration is complete."""
        if not self._is_dispensing:
            return
        
        elapsed = time.time() - self._dispense_start
        if elapsed >= self.payload_duration:
            self._stop_payload()
            
            # Add to exclusion map
            if self._current_target_position is not None:
                x, y, z = self._current_target_position
                self.exclusion_map.add_target(x, y, z, label="extinguished")
                logger.info(f"Target added to exclusion map at ({x:.2f}, {y:.2f}, {z:.2f})")
                self._current_target_position = None
            
            # Reset tracker for next target
            self.tracker.reset()
