#!/usr/bin/env python3
"""
NOMAD Edge Core - Python GStreamer RTSP Server

Provides a robust RTSP server for ZED camera streaming using GstRtspServer.
Supports multiple simultaneous viewers (Mission Planner, VLC, phone, etc.).

Target: Python 3.10+ | NVIDIA Jetson Orin Nano | GStreamer 1.x
Stream URL: rtsp://<JETSON_IP>:8554/zed

Usage:
    python3 -m edge_core.rtsp_server
    # or
    ./rtsp_server.py
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import os

# GObject Introspection imports
try:
    import gi
    gi.require_version('Gst', '1.0')
    gi.require_version('GstRtspServer', '1.0')
    from gi.repository import Gst, GstRtspServer, GLib
except ImportError as e:
    print(f"ERROR: GStreamer Python bindings not available: {e}")
    print("Install with: sudo apt install python3-gi gstreamer1.0-rtsp gir1.2-gst-rtsp-server-1.0")
    sys.exit(1)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)
logger = logging.getLogger("edge_core.rtsp_server")


def build_software_pipeline(device: str, bitrate: int) -> str:
    """Build software encoding pipeline string."""
    return (
        f"( v4l2src device={device} ! "
        f"video/x-raw,width=2560,height=720,framerate=30/1 ! "
        f"videocrop left=0 right=1280 ! "
        f"videoconvert ! "
        f"x264enc tune=zerolatency bitrate={bitrate} speed-preset=ultrafast key-int-max=15 ! "
        f"h264parse ! rtph264pay name=pay0 pt=96 config-interval=1 )"
    )


def build_nvidia_pipeline(device: str, bitrate: int) -> str:
    """Build NVIDIA hardware encoding pipeline string."""
    return (
        f"( v4l2src device={device} ! "
        f"video/x-raw,width=2560,height=720,framerate=30/1 ! "
        f"videocrop left=0 right=1280 ! "
        f"nvvidconv ! video/x-raw(memory:NVMM) ! "
        f"nvv4l2h264enc bitrate={bitrate * 1000} preset-level=1 iframeinterval=15 insert-sps-pps=true ! "
        f"h264parse ! rtph264pay name=pay0 pt=96 config-interval=1 )"
    )


class ZEDRtspServer:
    """
    GStreamer RTSP Server for ZED Camera.
    
    Creates an RTSP endpoint at rtsp://<ip>:8554/zed that:
    - Captures from ZED camera (/dev/video0)
    - Crops to left eye only (for stereo camera)
    - Encodes with H.264 zerolatency for minimum lag
    - Supports multiple simultaneous viewers
    """
    
    DEFAULT_PORT = 8554
    DEFAULT_MOUNT = "/zed"
    DEFAULT_DEVICE = "/dev/video0"
    
    def __init__(
        self,
        port: int = DEFAULT_PORT,
        mount_point: str = DEFAULT_MOUNT,
        device: str = DEFAULT_DEVICE,
        bitrate: int = 4000,
    ):
        self.port = port
        self.mount_point = mount_point
        self.device = device
        self.bitrate = bitrate
        
        self._server = None
        self._factory = None
        self._loop = None
        self._running = False
        
    def start(self) -> None:
        """Start the RTSP server."""
        # Initialize GStreamer
        Gst.init(None)
        
        # Create RTSP server
        self._server = GstRtspServer.RTSPServer.new()
        self._server.set_service(str(self.port))
        
        # Create media factory
        self._factory = GstRtspServer.RTSPMediaFactory.new()
        
        # Try NVIDIA pipeline first
        pipeline_str = None
        try:
            nvidia_pipeline = build_nvidia_pipeline(self.device, self.bitrate)
            # Test parse
            test = Gst.parse_launch(nvidia_pipeline.strip("() "))
            if test:
                pipeline_str = nvidia_pipeline
                logger.info("Using NVIDIA hardware encoding")
                del test
        except Exception as e:
            logger.warning(f"NVIDIA encoding unavailable: {e}")
        
        # Fall back to software encoding
        if not pipeline_str:
            pipeline_str = build_software_pipeline(self.device, self.bitrate)
            logger.info("Using software encoding (x264)")
        
        logger.debug(f"Pipeline: {pipeline_str}")
        
        # Set the launch string
        self._factory.set_launch(pipeline_str)
        self._factory.set_shared(True)
        
        # Get mount points and add factory
        mounts = self._server.get_mount_points()
        mounts.add_factory(self.mount_point, self._factory)
        
        # Attach server to default main context
        self._server.attach(None)
        
        self._running = True
        logger.info(f"RTSP server started at rtsp://0.0.0.0:{self.port}{self.mount_point}")
        logger.info(f"Device: {self.device} | Bitrate: {self.bitrate} kbps")
        
    def run_loop(self) -> None:
        """Run the GLib main loop (blocking)."""
        self._loop = GLib.MainLoop.new(None, False)
        
        # Handle shutdown signals
        def signal_handler(signum, frame):
            logger.info(f"Received signal {signum}, shutting down...")
            self.stop()
            
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        try:
            logger.info("RTSP server running. Press Ctrl+C to stop.")
            self._loop.run()
        except KeyboardInterrupt:
            self.stop()
            
    def stop(self) -> None:
        """Stop the RTSP server."""
        self._running = False
        if self._loop:
            self._loop.quit()
        logger.info("RTSP server stopped")


def get_local_ip() -> str:
    """Get the local IP address for display purposes."""
    import socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "localhost"


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="NOMAD ZED Camera RTSP Server",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("--port", "-p", type=int, default=ZEDRtspServer.DEFAULT_PORT, help="RTSP server port")
    parser.add_argument("--device", "-d", default=ZEDRtspServer.DEFAULT_DEVICE, help="V4L2 device path")
    parser.add_argument("--bitrate", "-b", type=int, default=4000, help="H.264 bitrate (kbps)")
    parser.add_argument("--debug", action="store_true", help="Enable debug logging")
    
    args = parser.parse_args()
    
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
        os.environ["GST_DEBUG"] = "3"
    
    # Display banner
    local_ip = get_local_ip()
    print()
    print("=" * 50)
    print("  NOMAD ZED Camera RTSP Server")
    print("  AEAC 2026 - McGill Aerial Design")
    print("=" * 50)
    print(f"  Stream URL: rtsp://{local_ip}:{args.port}/zed")
    print(f"  Device:     {args.device}")
    print(f"  Bitrate:    {args.bitrate} kbps")
    print("=" * 50)
    print()
    
    # Create and start server
    server = ZEDRtspServer(
        port=args.port,
        device=args.device,
        bitrate=args.bitrate,
    )
    
    try:
        server.start()
        server.run_loop()
    except Exception as e:
        logger.error(f"Server error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
