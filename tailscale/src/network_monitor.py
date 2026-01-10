"""
NOMAD Network Monitor

Monitors 4G/LTE modem and overall network health for the Jetson Orin Nano.
Provides connectivity metrics for the Edge Core API.

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

from __future__ import annotations

import asyncio
import logging
import re
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from typing import Any

logger = logging.getLogger(__name__)


class ConnectionType(Enum):
    """Network connection type."""

    LTE_4G = "4g_lte"
    LTE_5G = "5g"
    WIFI = "wifi"
    ETHERNET = "ethernet"
    UNKNOWN = "unknown"
    NONE = "none"


class SignalQuality(Enum):
    """Signal quality classification."""

    EXCELLENT = "excellent"  # RSRP >= -80 dBm
    GOOD = "good"  # RSRP >= -90 dBm
    FAIR = "fair"  # RSRP >= -100 dBm
    POOR = "poor"  # RSRP >= -110 dBm
    NO_SIGNAL = "no_signal"  # RSRP < -110 dBm or no connection


@dataclass
class ModemStatus:
    """4G/LTE modem status."""

    connected: bool = False
    signal_strength_dbm: int | None = None  # RSRP: -140 to -44 dBm
    signal_quality: SignalQuality = SignalQuality.NO_SIGNAL
    signal_percent: int = 0  # 0-100%
    carrier: str | None = None  # "AT&T", "Verizon", etc.
    technology: str | None = None  # "LTE", "5G NR", "HSPA+"
    ip_address: str | None = None
    interface: str | None = None  # "wwan0", "usb0"
    imei: str | None = None
    model: str | None = None

    def to_dict(self) -> dict[str, Any]:
        """Convert to dictionary for API response."""
        return {
            "connected": self.connected,
            "signal_strength_dbm": self.signal_strength_dbm,
            "signal_quality": self.signal_quality.value,
            "signal_percent": self.signal_percent,
            "carrier": self.carrier,
            "technology": self.technology,
            "ip_address": self.ip_address,
            "interface": self.interface,
        }


@dataclass
class NetworkStatus:
    """Overall network status."""

    connection_type: ConnectionType = ConnectionType.NONE
    internet_reachable: bool = False
    modem: ModemStatus | None = None
    tailscale_reachable: bool = False
    latency_to_internet_ms: float | None = None
    latency_to_gcs_ms: float | None = None
    gcs_ip: str | None = None
    last_check: datetime = field(default_factory=datetime.now)

    def to_dict(self) -> dict[str, Any]:
        """Convert to dictionary for API response."""
        return {
            "connection_type": self.connection_type.value,
            "internet_reachable": self.internet_reachable,
            "modem": self.modem.to_dict() if self.modem else None,
            "tailscale_reachable": self.tailscale_reachable,
            "latency_to_internet_ms": self.latency_to_internet_ms,
            "latency_to_gcs_ms": self.latency_to_gcs_ms,
            "gcs_ip": self.gcs_ip,
            "last_check": self.last_check.isoformat(),
        }


def _rsrp_to_quality(rsrp: int | None) -> SignalQuality:
    """Convert RSRP (dBm) to signal quality classification."""
    if rsrp is None:
        return SignalQuality.NO_SIGNAL
    if rsrp >= -80:
        return SignalQuality.EXCELLENT
    if rsrp >= -90:
        return SignalQuality.GOOD
    if rsrp >= -100:
        return SignalQuality.FAIR
    if rsrp >= -110:
        return SignalQuality.POOR
    return SignalQuality.NO_SIGNAL


def _rsrp_to_percent(rsrp: int | None) -> int:
    """Convert RSRP (dBm) to percentage (0-100)."""
    if rsrp is None:
        return 0
    # RSRP range: -140 dBm (worst) to -44 dBm (best)
    # Map to 0-100%
    percent = int(((rsrp + 140) / 96) * 100)
    return max(0, min(100, percent))


class NetworkMonitor:
    """
    Monitors network connectivity for NOMAD.

    Features:
    - Monitor 4G/LTE modem signal strength
    - Check internet connectivity
    - Measure latency to Ground Station
    - Provide metrics for API

    Usage:
        monitor = NetworkMonitor(gcs_tailscale_ip="100.100.10.1")
        await monitor.start()
        status = monitor.status
        await monitor.stop()
    """

    def __init__(
        self,
        gcs_tailscale_ip: str | None = None,
        check_interval: float = 30.0,
        internet_check_host: str = "8.8.8.8",
    ):
        """
        Initialize network monitor.

        Args:
            gcs_tailscale_ip: Tailscale IP of Ground Control Station
            check_interval: Seconds between status checks
            internet_check_host: Host to ping for internet connectivity
        """
        self._gcs_ip = gcs_tailscale_ip
        self._check_interval = check_interval
        self._internet_host = internet_check_host

        self._status = NetworkStatus()
        self._running = False
        self._task: asyncio.Task[None] | None = None

    @property
    def status(self) -> NetworkStatus:
        """Get current network status."""
        return self._status

    @property
    def gcs_ip(self) -> str | None:
        """Get configured GCS IP."""
        return self._gcs_ip

    @gcs_ip.setter
    def gcs_ip(self, value: str | None) -> None:
        """Set GCS IP address."""
        self._gcs_ip = value

    async def start(self) -> None:
        """Start network monitoring."""
        if self._running:
            logger.warning("NetworkMonitor already running")
            return

        self._running = True
        logger.info(f"NetworkMonitor starting (interval={self._check_interval}s)")

        # Do initial check
        await self.check_connectivity()

        # Start monitoring task
        self._task = asyncio.create_task(self._monitor_loop())

    async def stop(self) -> None:
        """Stop monitoring."""
        if not self._running:
            return

        self._running = False

        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None

        logger.info("NetworkMonitor stopped")

    async def check_connectivity(self) -> NetworkStatus:
        """Perform full connectivity check and update status."""
        status = NetworkStatus(last_check=datetime.now())

        # Check modem status
        status.modem = await self._check_modem_status()

        # Determine connection type
        status.connection_type = await self._determine_connection_type()

        # Check internet connectivity
        internet_latency = await self._ping_host(self._internet_host)
        status.internet_reachable = internet_latency is not None
        status.latency_to_internet_ms = internet_latency

        # Check GCS connectivity
        if self._gcs_ip:
            status.gcs_ip = self._gcs_ip
            gcs_latency = await self._ping_host(self._gcs_ip)
            status.tailscale_reachable = gcs_latency is not None
            status.latency_to_gcs_ms = gcs_latency

        self._status = status
        return status

    async def ping(self, host: str, count: int = 3) -> dict[str, Any]:
        """
        Ping arbitrary host and return results.

        Args:
            host: IP or hostname to ping
            count: Number of ping packets

        Returns:
            Dict with latency_ms, packets_sent, packets_received
        """
        latency = await self._ping_host(host, count)

        return {
            "host": host,
            "latency_ms": latency,
            "packets_sent": count,
            "packets_received": count if latency else 0,
            "reachable": latency is not None,
        }

    async def _monitor_loop(self) -> None:
        """Background monitoring loop."""
        while self._running:
            try:
                await self.check_connectivity()
                await asyncio.sleep(self._check_interval)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Monitor loop error: {e}")
                await asyncio.sleep(self._check_interval)

    async def _check_modem_status(self) -> ModemStatus | None:
        """
        Query 4G/LTE modem status via ModemManager (mmcli).

        Returns:
            ModemStatus or None if modem not available
        """
        try:
            # List modems
            exit_code, stdout, stderr = await self._run_command(["mmcli", "-L"])

            if exit_code != 0 or "No modems" in stdout:
                logger.debug("No modem found via mmcli")
                return None

            # Parse modem index (e.g., "/org/freedesktop/ModemManager1/Modem/0")
            modem_match = re.search(r"/Modem/(\d+)", stdout)
            if not modem_match:
                return None

            modem_idx = modem_match.group(1)

            # Get modem info
            exit_code, stdout, stderr = await self._run_command(
                ["mmcli", "-m", modem_idx]
            )

            if exit_code != 0:
                return None

            status = ModemStatus()

            # Parse modem output
            # State
            if "state: 'connected'" in stdout.lower():
                status.connected = True
            elif "state:" in stdout:
                status.connected = False

            # Model
            model_match = re.search(r"model:\s*(.+)", stdout, re.IGNORECASE)
            if model_match:
                status.model = model_match.group(1).strip()

            # Access technology
            tech_match = re.search(
                r"access tech:\s*(.+)", stdout, re.IGNORECASE
            )
            if tech_match:
                status.technology = tech_match.group(1).strip()

            # Operator/carrier
            operator_match = re.search(
                r"operator name:\s*(.+)", stdout, re.IGNORECASE
            )
            if operator_match:
                status.carrier = operator_match.group(1).strip()

            # Get signal info
            exit_code, stdout, stderr = await self._run_command(
                ["mmcli", "-m", modem_idx, "--signal-get"]
            )

            if exit_code == 0:
                # Parse RSRP (LTE reference signal)
                rsrp_match = re.search(r"rsrp:\s*([-\d.]+)\s*dBm", stdout)
                if rsrp_match:
                    status.signal_strength_dbm = int(float(rsrp_match.group(1)))
                    status.signal_quality = _rsrp_to_quality(
                        status.signal_strength_dbm
                    )
                    status.signal_percent = _rsrp_to_percent(
                        status.signal_strength_dbm
                    )

            return status

        except FileNotFoundError:
            logger.debug("mmcli not found - ModemManager not installed")
            return None
        except Exception as e:
            logger.error(f"Modem status check error: {e}")
            return None

    async def _determine_connection_type(self) -> ConnectionType:
        """Determine primary network connection type."""
        try:
            # Check for wwan/mobile interface
            exit_code, stdout, _ = await self._run_command(
                ["ip", "link", "show"]
            )

            if exit_code == 0:
                if "wwan" in stdout or "usb0" in stdout:
                    # Check if 5G or LTE
                    if self._status.modem and self._status.modem.technology:
                        if "5g" in self._status.modem.technology.lower():
                            return ConnectionType.LTE_5G
                    return ConnectionType.LTE_4G

                if "wlan" in stdout:
                    return ConnectionType.WIFI

                if "eth" in stdout:
                    return ConnectionType.ETHERNET

            return ConnectionType.UNKNOWN

        except Exception:
            return ConnectionType.UNKNOWN

    async def _ping_host(
        self, host: str, count: int = 3, timeout: float = 5.0
    ) -> float | None:
        """
        Ping host and return average latency in ms.

        Args:
            host: IP or hostname to ping
            count: Number of ping packets
            timeout: Total timeout in seconds

        Returns:
            Average latency in ms, or None if unreachable
        """
        try:
            exit_code, stdout, stderr = await self._run_command(
                ["ping", "-c", str(count), "-W", "2", host],
                timeout=timeout,
            )

            if exit_code != 0:
                return None

            # Parse average latency from ping output
            # Example: "rtt min/avg/max/mdev = 10.5/15.2/20.1/3.2 ms"
            match = re.search(r"rtt.*=\s*[\d.]+/([\d.]+)/", stdout)
            if match:
                return float(match.group(1))

            # Alternative format: "time=XX ms"
            times = re.findall(r"time=([\d.]+)\s*ms", stdout)
            if times:
                return sum(float(t) for t in times) / len(times)

            return None

        except Exception as e:
            logger.debug(f"Ping error for {host}: {e}")
            return None

    async def _run_command(
        self, cmd: list[str], timeout: float = 10.0
    ) -> tuple[int, str, str]:
        """Run shell command and return (exit_code, stdout, stderr)."""
        try:
            proc = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )

            stdout, stderr = await asyncio.wait_for(
                proc.communicate(), timeout=timeout
            )

            return (
                proc.returncode or 0,
                stdout.decode("utf-8", errors="replace"),
                stderr.decode("utf-8", errors="replace"),
            )

        except asyncio.TimeoutError:
            return (1, "", "Command timed out")
        except FileNotFoundError:
            return (127, "", f"Command not found: {cmd[0]}")
        except Exception as e:
            return (1, "", str(e))


# ============================================================
# Module-level singleton access
# ============================================================

_monitor: NetworkMonitor | None = None


def get_network_monitor() -> NetworkMonitor | None:
    """Get the global NetworkMonitor instance."""
    return _monitor


def init_network_monitor(
    gcs_tailscale_ip: str | None = None,
    check_interval: float = 30.0,
) -> NetworkMonitor:
    """
    Initialize the global NetworkMonitor instance.

    Args:
        gcs_tailscale_ip: Tailscale IP of Ground Control Station
        check_interval: Seconds between status checks

    Returns:
        The initialized NetworkMonitor instance
    """
    global _monitor
    _monitor = NetworkMonitor(
        gcs_tailscale_ip=gcs_tailscale_ip,
        check_interval=check_interval,
    )
    return _monitor
