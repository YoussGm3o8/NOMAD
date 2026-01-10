"""
NOMAD Tailscale Integration Package

Provides Tailscale VPN management and network monitoring for secure
4G/LTE communication between Jetson and Ground Station.
"""

from .tailscale_manager import (
    TailscaleManager,
    TailscaleStatus,
    TailscaleInfo,
    TailscalePeer,
    get_tailscale_manager,
    init_tailscale_manager,
)
from .network_monitor import (
    NetworkMonitor,
    NetworkStatus,
    ModemStatus,
    ConnectionType,
    SignalQuality,
    get_network_monitor,
    init_network_monitor,
)

__all__ = [
    # Tailscale Manager
    "TailscaleManager",
    "TailscaleStatus",
    "TailscaleInfo",
    "TailscalePeer",
    "get_tailscale_manager",
    "init_tailscale_manager",
    # Network Monitor
    "NetworkMonitor",
    "NetworkStatus",
    "ModemStatus",
    "ConnectionType",
    "SignalQuality",
    "get_network_monitor",
    "init_network_monitor",
]
