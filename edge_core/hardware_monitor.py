"""
Hardware Health Monitor for NOMAD.

Monitors system health metrics on NVIDIA Jetson Orin Nano:
- CPU/GPU Temperature (via jtop)
- GPU Load (via jtop)
- Power Draw (via jtop)
- Disk Space (via psutil)
- Memory Usage (via psutil)

Provides throttling recommendations when thresholds are exceeded.

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

from __future__ import annotations

import logging
import os
import platform
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import Enum, auto
from typing import Any, Callable, TYPE_CHECKING

if TYPE_CHECKING:
    from .state import StateManager
    from .ipc import ZMQPublisher

logger = logging.getLogger(__name__)

# Try to import psutil (cross-platform)
try:
    import psutil
    PSUTIL_AVAILABLE = True
except ImportError:
    logger.warning("psutil not available - disk/memory monitoring disabled")
    PSUTIL_AVAILABLE = False

# Try to import jtop (Jetson only)
JTOP_AVAILABLE = False
jtop: Any = None

def _try_import_jtop() -> bool:
    """Try to import jtop module."""
    global JTOP_AVAILABLE, jtop
    if jtop is not None:
        return JTOP_AVAILABLE
    try:
        from jtop import jtop as _jtop
        jtop = _jtop
        JTOP_AVAILABLE = True
        return True
    except ImportError:
        logger.debug("jtop not available - Jetson monitoring disabled")
        return False


class ThrottleLevel(Enum):
    """System throttle level."""
    
    NONE = auto()       # Normal operation
    WARNING = auto()    # Approaching limits
    THROTTLE = auto()   # Active throttling
    CRITICAL = auto()   # Emergency throttling


@dataclass
class HardwareThresholds:
    """Configurable hardware thresholds."""
    
    # Temperature thresholds (Celsius)
    cpu_temp_warning: float = 75.0
    cpu_temp_critical: float = 85.0
    gpu_temp_warning: float = 75.0
    gpu_temp_critical: float = 85.0
    
    # Power thresholds (Watts)
    power_warning: float = 12.0  # Orin Nano is 7-15W
    power_critical: float = 14.0
    
    # Battery voltage thresholds (for external power monitoring)
    voltage_warning: float = 14.5  # 4S LiPo
    voltage_critical: float = 14.0
    
    # Disk space thresholds (GB)
    disk_warning_gb: float = 2.0
    disk_critical_gb: float = 0.5
    
    # Memory thresholds (percentage)
    memory_warning_pct: float = 85.0
    memory_critical_pct: float = 95.0


@dataclass
class HardwareStatus:
    """Current hardware status snapshot."""
    
    timestamp: datetime = field(default_factory=lambda: datetime.now(timezone.utc))
    
    # Temperatures
    cpu_temp_c: float | None = None
    gpu_temp_c: float | None = None
    
    # GPU
    gpu_load_pct: float | None = None
    
    # Power
    power_draw_w: float | None = None
    
    # Storage
    disk_free_gb: float | None = None
    disk_total_gb: float | None = None
    
    # Memory
    memory_used_pct: float | None = None
    memory_total_gb: float | None = None
    
    # Throttle state
    throttle_level: ThrottleLevel = ThrottleLevel.NONE
    throttle_reason: str = ""
    
    # Jetson specific
    jetson_model: str | None = None
    jetson_power_mode: str | None = None
    
    def to_dict(self) -> dict:
        """Convert to dictionary."""
        return {
            "timestamp": self.timestamp.isoformat(),
            "cpu_temp_c": self.cpu_temp_c,
            "gpu_temp_c": self.gpu_temp_c,
            "gpu_load_pct": self.gpu_load_pct,
            "power_draw_w": self.power_draw_w,
            "disk_free_gb": self.disk_free_gb,
            "disk_total_gb": self.disk_total_gb,
            "memory_used_pct": self.memory_used_pct,
            "memory_total_gb": self.memory_total_gb,
            "throttle_level": self.throttle_level.name,
            "throttle_reason": self.throttle_reason,
            "jetson_model": self.jetson_model,
            "jetson_power_mode": self.jetson_power_mode,
        }


class HealthMonitor:
    """
    Hardware health monitoring service.
    
    Monitors CPU/GPU temperature, power draw, disk space, and memory.
    Provides throttling recommendations and publishes control messages.
    
    Usage:
        monitor = HealthMonitor(state_manager, throttle_publisher)
        monitor.start()
        
        # Get current status
        status = monitor.status
        
        # Check if throttled
        if monitor.should_throttle:
            print(f"Throttling due to: {monitor.status.throttle_reason}")
    """
    
    # Default monitoring interval
    DEFAULT_INTERVAL = 2.0  # seconds
    
    # Evidence storage path on Jetson
    EVIDENCE_PATH = "/mnt/nvme"
    
    def __init__(
        self,
        state_manager: "StateManager | None" = None,
        throttle_publisher: "ZMQPublisher | None" = None,
        thresholds: HardwareThresholds | None = None,
        monitor_interval: float = DEFAULT_INTERVAL,
        evidence_path: str | None = None,
        on_throttle_change: Callable[[ThrottleLevel, str], None] | None = None,
    ) -> None:
        """
        Initialize health monitor.
        
        Args:
            state_manager: StateManager for updating hardware metrics
            throttle_publisher: ZMQ publisher for throttle messages
            thresholds: Custom threshold configuration
            monitor_interval: Monitoring interval in seconds
            evidence_path: Path to monitor for disk space
            on_throttle_change: Callback when throttle level changes
        """
        self._state_manager = state_manager
        self._throttle_pub = throttle_publisher
        self._thresholds = thresholds or HardwareThresholds()
        self._interval = monitor_interval
        self._evidence_path = evidence_path or self.EVIDENCE_PATH
        self._on_throttle_change = on_throttle_change
        
        # Current status
        self._status = HardwareStatus()
        self._lock = threading.RLock()
        
        # Previous throttle level (for change detection)
        self._prev_throttle_level = ThrottleLevel.NONE
        
        # Background monitoring
        self._running = False
        self._monitor_thread: threading.Thread | None = None
        
        # Platform detection
        self._is_jetson = self._detect_jetson()
        self._jtop_context: Any = None
    
    @staticmethod
    def _detect_jetson() -> bool:
        """Detect if running on NVIDIA Jetson."""
        try:
            with open("/etc/nv_tegra_release", "r") as f:
                return "NVIDIA" in f.read()
        except FileNotFoundError:
            return False
    
    @property
    def status(self) -> HardwareStatus:
        """Get current hardware status."""
        with self._lock:
            return self._status
    
    @property
    def should_throttle(self) -> bool:
        """Check if system should be throttled."""
        with self._lock:
            return self._status.throttle_level in (ThrottleLevel.THROTTLE, ThrottleLevel.CRITICAL)
    
    @property
    def is_critical(self) -> bool:
        """Check if system is in critical state."""
        with self._lock:
            return self._status.throttle_level == ThrottleLevel.CRITICAL
    
    def start(self) -> None:
        """Start the health monitor."""
        if self._running:
            return
        
        self._running = True
        
        # Initialize jtop if available
        if self._is_jetson and _try_import_jtop():
            logger.info("Jetson detected - jtop monitoring enabled")
        else:
            logger.info("Generic platform - using psutil only")
        
        # Do initial reading
        self._update_status()
        
        # Start background monitor
        self._monitor_thread = threading.Thread(
            target=self._monitor_loop,
            daemon=True,
            name="health_monitor",
        )
        self._monitor_thread.start()
        
        logger.info(f"HealthMonitor started (interval={self._interval}s)")
    
    def stop(self) -> None:
        """Stop the health monitor."""
        self._running = False
        
        if self._monitor_thread:
            self._monitor_thread.join(timeout=2.0)
            self._monitor_thread = None
        
        logger.info("HealthMonitor stopped")
    
    def _monitor_loop(self) -> None:
        """Background monitoring loop."""
        while self._running:
            try:
                self._update_status()
                self._check_thresholds()
                self._update_state_manager()
                
            except Exception as e:
                logger.error(f"Health monitor error: {e}")
            
            time.sleep(self._interval)
    
    def _update_status(self) -> None:
        """Update hardware status."""
        status = HardwareStatus()
        
        # Read Jetson stats via jtop
        if self._is_jetson and JTOP_AVAILABLE:
            self._read_jtop_stats(status)
        
        # Read generic stats via psutil
        if PSUTIL_AVAILABLE:
            self._read_psutil_stats(status)
        
        # Fallback CPU temp for non-Jetson Linux
        if status.cpu_temp_c is None and platform.system() == "Linux":
            status.cpu_temp_c = self._read_linux_cpu_temp()
        
        with self._lock:
            self._status = status
    
    def _read_jtop_stats(self, status: HardwareStatus) -> None:
        """Read stats from jtop (Jetson only)."""
        try:
            # jtop requires context manager but we want continuous monitoring
            # Use a single read approach
            with jtop() as jetson:
                # Wait for jtop to initialize
                if not jetson.ok():
                    return
                
                # Get stats
                stats = jetson.stats
                
                # Temperature
                temps = jetson.temperature
                if temps:
                    # CPU temperature (average of cores or main sensor)
                    if 'CPU' in temps:
                        status.cpu_temp_c = temps['CPU']
                    elif 'cpu' in temps:
                        status.cpu_temp_c = temps['cpu']
                    
                    # GPU temperature
                    if 'GPU' in temps:
                        status.gpu_temp_c = temps['GPU']
                    elif 'gpu' in temps:
                        status.gpu_temp_c = temps['gpu']
                
                # GPU load
                gpu = jetson.gpu
                if gpu:
                    # GPU is a dict with 'status' containing load
                    if isinstance(gpu, dict) and 'status' in gpu:
                        gpu_status = gpu['status']
                        if isinstance(gpu_status, dict) and 'load' in gpu_status:
                            status.gpu_load_pct = gpu_status['load']
                    # Try alternative structure
                    elif isinstance(gpu, dict) and 'load' in gpu:
                        status.gpu_load_pct = gpu['load']
                
                # Power draw
                power = jetson.power
                if power:
                    # Total power consumption
                    if isinstance(power, dict):
                        total = power.get('tot', power.get('total', {}))
                        if isinstance(total, dict):
                            status.power_draw_w = total.get('power', total.get('avg'))
                        elif isinstance(total, (int, float)):
                            status.power_draw_w = total / 1000.0  # mW to W
                
                # Jetson info
                status.jetson_model = jetson.board.get('platform', {}).get('Machine', 'Unknown')
                
                # Power mode
                nvpmodel = jetson.nvpmodel
                if nvpmodel:
                    status.jetson_power_mode = str(nvpmodel)
                
        except Exception as e:
            logger.debug(f"jtop read error: {e}")
    
    def _read_psutil_stats(self, status: HardwareStatus) -> None:
        """Read stats from psutil (cross-platform)."""
        try:
            # Memory
            mem = psutil.virtual_memory()
            status.memory_used_pct = mem.percent
            status.memory_total_gb = mem.total / (1024 ** 3)
            
            # Disk space
            disk_path = self._evidence_path
            if not os.path.exists(disk_path):
                # Fallback to root
                disk_path = "/"
            
            try:
                disk = psutil.disk_usage(disk_path)
                status.disk_free_gb = disk.free / (1024 ** 3)
                status.disk_total_gb = disk.total / (1024 ** 3)
            except OSError:
                # Path might not exist, use root
                disk = psutil.disk_usage("/")
                status.disk_free_gb = disk.free / (1024 ** 3)
                status.disk_total_gb = disk.total / (1024 ** 3)
            
            # CPU temperature (psutil on some platforms)
            if status.cpu_temp_c is None:
                temps = psutil.sensors_temperatures()
                if temps:
                    # Find CPU temperature
                    for name, entries in temps.items():
                        if 'cpu' in name.lower() or 'core' in name.lower():
                            if entries:
                                status.cpu_temp_c = entries[0].current
                                break
                    
                    # Try coretemp or k10temp
                    if status.cpu_temp_c is None:
                        for sensor_name in ['coretemp', 'k10temp', 'cpu_thermal']:
                            if sensor_name in temps and temps[sensor_name]:
                                status.cpu_temp_c = temps[sensor_name][0].current
                                break
            
        except Exception as e:
            logger.debug(f"psutil read error: {e}")
    
    @staticmethod
    def _read_linux_cpu_temp() -> float | None:
        """Read CPU temperature from Linux thermal zone."""
        try:
            # Try thermal_zone0 (common on embedded systems)
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                temp_millic = int(f.read().strip())
                return temp_millic / 1000.0
        except (FileNotFoundError, ValueError, PermissionError):
            return None
    
    def _check_thresholds(self) -> None:
        """Check thresholds and update throttle level."""
        status = self._status
        t = self._thresholds
        
        throttle_level = ThrottleLevel.NONE
        reasons: list[str] = []
        
        # CPU Temperature
        if status.cpu_temp_c is not None:
            if status.cpu_temp_c >= t.cpu_temp_critical:
                throttle_level = max(throttle_level, ThrottleLevel.CRITICAL, key=lambda x: x.value)
                reasons.append(f"CPU temp {status.cpu_temp_c:.1f}°C >= {t.cpu_temp_critical}°C")
            elif status.cpu_temp_c >= t.cpu_temp_warning:
                throttle_level = max(throttle_level, ThrottleLevel.WARNING, key=lambda x: x.value)
        
        # GPU Temperature
        if status.gpu_temp_c is not None:
            if status.gpu_temp_c >= t.gpu_temp_critical:
                throttle_level = max(throttle_level, ThrottleLevel.CRITICAL, key=lambda x: x.value)
                reasons.append(f"GPU temp {status.gpu_temp_c:.1f}°C >= {t.gpu_temp_critical}°C")
            elif status.gpu_temp_c >= t.gpu_temp_warning:
                throttle_level = max(throttle_level, ThrottleLevel.WARNING, key=lambda x: x.value)
        
        # Power Draw
        if status.power_draw_w is not None:
            if status.power_draw_w >= t.power_critical:
                throttle_level = max(throttle_level, ThrottleLevel.THROTTLE, key=lambda x: x.value)
                reasons.append(f"Power {status.power_draw_w:.1f}W >= {t.power_critical}W")
            elif status.power_draw_w >= t.power_warning:
                throttle_level = max(throttle_level, ThrottleLevel.WARNING, key=lambda x: x.value)
        
        # Disk Space
        if status.disk_free_gb is not None:
            if status.disk_free_gb <= t.disk_critical_gb:
                throttle_level = max(throttle_level, ThrottleLevel.WARNING, key=lambda x: x.value)
                reasons.append(f"Disk {status.disk_free_gb:.2f}GB <= {t.disk_critical_gb}GB")
            elif status.disk_free_gb <= t.disk_warning_gb:
                throttle_level = max(throttle_level, ThrottleLevel.WARNING, key=lambda x: x.value)
        
        # Memory
        if status.memory_used_pct is not None:
            if status.memory_used_pct >= t.memory_critical_pct:
                throttle_level = max(throttle_level, ThrottleLevel.THROTTLE, key=lambda x: x.value)
                reasons.append(f"Memory {status.memory_used_pct:.1f}% >= {t.memory_critical_pct}%")
            elif status.memory_used_pct >= t.memory_warning_pct:
                throttle_level = max(throttle_level, ThrottleLevel.WARNING, key=lambda x: x.value)
        
        # Set WARNING to THROTTLE if there are critical conditions
        if throttle_level == ThrottleLevel.CRITICAL:
            throttle_level = ThrottleLevel.THROTTLE  # Upgrade to at least THROTTLE
        
        # Update status
        with self._lock:
            self._status.throttle_level = throttle_level
            self._status.throttle_reason = "; ".join(reasons) if reasons else ""
        
        # Handle throttle level change
        if throttle_level != self._prev_throttle_level:
            self._handle_throttle_change(throttle_level, self._status.throttle_reason)
            self._prev_throttle_level = throttle_level
    
    def _handle_throttle_change(self, level: ThrottleLevel, reason: str) -> None:
        """Handle throttle level change."""
        # Log the change
        if level == ThrottleLevel.CRITICAL:
            logger.critical(f"CRITICAL Hardware Warning: {reason}")
        elif level == ThrottleLevel.THROTTLE:
            logger.warning(f"Hardware throttling activated: {reason}")
        elif level == ThrottleLevel.WARNING:
            logger.warning(f"Hardware warning: {reason}")
        elif level == ThrottleLevel.NONE and self._prev_throttle_level != ThrottleLevel.NONE:
            logger.info("Hardware conditions normalized - throttling deactivated")
        
        # Send throttle message via ZMQ
        if self._throttle_pub and level in (ThrottleLevel.THROTTLE, ThrottleLevel.CRITICAL):
            self._send_throttle_message(level)
        elif self._throttle_pub and level == ThrottleLevel.NONE and self._prev_throttle_level != ThrottleLevel.NONE:
            self._send_throttle_message(ThrottleLevel.NONE)
        
        # Callback
        if self._on_throttle_change:
            self._on_throttle_change(level, reason)
    
    def _send_throttle_message(self, level: ThrottleLevel) -> None:
        """Send throttle control message via ZMQ."""
        if not self._throttle_pub:
            return
        
        from .ipc import IPCMessage
        
        if level in (ThrottleLevel.THROTTLE, ThrottleLevel.CRITICAL):
            msg = IPCMessage(
                msg_type="THROTTLE_DOWN",
                timestamp=datetime.now(timezone.utc).isoformat(),
                data={
                    "level": level.name,
                    "reason": self._status.throttle_reason,
                    "cpu_temp": self._status.cpu_temp_c,
                    "gpu_temp": self._status.gpu_temp_c,
                    "power_w": self._status.power_draw_w,
                },
            )
        else:
            msg = IPCMessage(
                msg_type="THROTTLE_UP",
                timestamp=datetime.now(timezone.utc).isoformat(),
                data={"level": "NONE"},
            )
        
        self._throttle_pub.send(msg)
        logger.debug(f"Sent throttle message: {msg.msg_type}")
    
    def _update_state_manager(self) -> None:
        """Update state manager with hardware metrics."""
        if not self._state_manager:
            return
        
        status = self._status
        
        self._state_manager.update_state(
            cpu_temp_c=status.cpu_temp_c,
            gpu_temp_c=status.gpu_temp_c,
            gpu_load_pct=status.gpu_load_pct,
            power_draw_w=status.power_draw_w,
            disk_free_gb=status.disk_free_gb,
            memory_used_pct=status.memory_used_pct,
            throttled=self.should_throttle,
        )
    
    def check_battery_voltage(self, voltage: float) -> None:
        """
        Check external battery voltage against thresholds.
        
        Called from MAVLink service when voltage is updated.
        
        Args:
            voltage: Battery voltage in volts
        """
        t = self._thresholds
        
        if voltage <= t.voltage_critical:
            with self._lock:
                if self._status.throttle_level != ThrottleLevel.CRITICAL:
                    self._status.throttle_level = ThrottleLevel.CRITICAL
                    self._status.throttle_reason = f"Battery voltage {voltage:.2f}V critical"
                    self._handle_throttle_change(ThrottleLevel.CRITICAL, self._status.throttle_reason)
        elif voltage <= t.voltage_warning:
            with self._lock:
                if self._status.throttle_level == ThrottleLevel.NONE:
                    self._status.throttle_level = ThrottleLevel.WARNING


# Default ZMQ endpoint for throttle messages
DEFAULT_THROTTLE_ENDPOINT = "tcp://127.0.0.1:5560"


# ============================================================
# Vision Process Watchdog
# ============================================================

class VisionProcessWatchdog:
    """
    Watchdog for monitoring Vision Process health.
    
    Pings the Vision Process ZMQ heartbeat endpoint every 1 second.
    If 3 consecutive pings fail, triggers a restart.
    
    Usage:
        watchdog = VisionProcessWatchdog(vision_process_handle)
        watchdog.start()
    """
    
    PING_INTERVAL = 1.0  # seconds
    FAILURE_THRESHOLD = 3  # consecutive failures before restart
    HEARTBEAT_ENDPOINT = "tcp://127.0.0.1:5556"  # Vision heartbeat endpoint
    
    def __init__(
        self,
        restart_callback: Callable[[], None],
        heartbeat_endpoint: str | None = None,
    ) -> None:
        """
        Initialize vision process watchdog.
        
        Args:
            restart_callback: Function to call when restart is needed
            heartbeat_endpoint: Custom heartbeat endpoint (optional)
        """
        self._restart_callback = restart_callback
        self._endpoint = heartbeat_endpoint or self.HEARTBEAT_ENDPOINT
        
        self._running = False
        self._watchdog_thread: threading.Thread | None = None
        self._failure_count = 0
        self._lock = threading.Lock()
        
        logger.info(f"VisionProcessWatchdog initialized (endpoint={self._endpoint})")
    
    def start(self) -> None:
        """Start the watchdog."""
        if self._running:
            return
        
        self._running = True
        self._failure_count = 0
        
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop,
            daemon=True,
            name="vision_watchdog",
        )
        self._watchdog_thread.start()
        
        logger.info("VisionProcessWatchdog started")
    
    def stop(self) -> None:
        """Stop the watchdog."""
        self._running = False
        
        if self._watchdog_thread:
            self._watchdog_thread.join(timeout=2.0)
            self._watchdog_thread = None
        
        logger.info("VisionProcessWatchdog stopped")
    
    def _watchdog_loop(self) -> None:
        """Watchdog monitoring loop."""
        # Import here to avoid circular imports
        from .ipc import ZMQSubscriber, IPCMessage
        
        subscriber = None
        
        try:
            # Connect to vision heartbeat endpoint
            subscriber = ZMQSubscriber(endpoint=self._endpoint, timeout_ms=500)
            logger.info(f"Watchdog connected to {self._endpoint}")
            
            while self._running:
                try:
                    # Try to receive heartbeat message
                    msg = subscriber.receive()
                    
                    if msg and msg.msg_type == "heartbeat":
                        # Heartbeat received - reset failure count
                        with self._lock:
                            if self._failure_count > 0:
                                logger.info("Vision process heartbeat restored")
                            self._failure_count = 0
                    else:
                        # No valid heartbeat - increment failure count
                        self._handle_failure()
                
                except TimeoutError:
                    # No message received within timeout
                    self._handle_failure()
                
                except Exception as e:
                    logger.error(f"Watchdog ping error: {e}")
                    self._handle_failure()
                
                time.sleep(self.PING_INTERVAL)
        
        finally:
            if subscriber:
                subscriber.close()
    
    def _handle_failure(self) -> None:
        """Handle a ping failure."""
        with self._lock:
            self._failure_count += 1
            
            logger.warning(
                f"Vision process heartbeat failure "
                f"({self._failure_count}/{self.FAILURE_THRESHOLD})"
            )
            
            if self._failure_count >= self.FAILURE_THRESHOLD:
                logger.error(
                    f"Vision process failed {self.FAILURE_THRESHOLD} consecutive "
                    "heartbeat checks - triggering restart"
                )
                self._trigger_restart()
    
    def _trigger_restart(self) -> None:
        """Trigger vision process restart."""
        try:
            self._restart_callback()
            # Reset failure count after triggering restart
            self._failure_count = 0
            logger.info("Vision process restart triggered")
        except Exception as e:
            logger.error(f"Failed to restart vision process: {e}")


# Global instance
_health_monitor: HealthMonitor | None = None


def get_health_monitor() -> HealthMonitor | None:
    """Get the global HealthMonitor instance."""
    return _health_monitor


def init_health_monitor(
    state_manager: "StateManager",
    throttle_publisher: "ZMQPublisher | None" = None,
) -> HealthMonitor:
    """Initialize the global HealthMonitor."""
    global _health_monitor
    _health_monitor = HealthMonitor(
        state_manager=state_manager,
        throttle_publisher=throttle_publisher,
    )
    return _health_monitor
