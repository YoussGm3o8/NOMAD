"""
Video Stream Manager - Dynamic Multi-Stream Support

Manages multiple concurrent ros_video_bridge.py streams for dynamic RTSP streaming.
Supports starting/stopping individual streams with automatic port allocation.
Runs on Jetson Edge Core.
"""

import logging
import subprocess
import threading
import time
import os
from typing import List, Dict, Optional
from dataclasses import dataclass, asdict

logger = logging.getLogger("edge_core.video_manager")

# Default ROS topics to try for auto-start (in order of preference)
DEFAULT_ZED_TOPICS = [
    "/zed/zed_node/rgb/image_rect_color",
    "/zed/zed_node/left/image_rect_color", 
    "/zed/zed_node/stereo/image_rect_color",
]


@dataclass
class StreamInfo:
    """Information about an active video stream."""
    stream_name: str
    topic: str
    tcp_port: int
    width: int
    height: int
    fps: int
    rtsp_url: str
    bridge_pid: Optional[int] = None
    encoder_pid: Optional[int] = None
    started_at: float = 0.0
    
    def to_dict(self) -> dict:
        return asdict(self)


class VideoStreamManager:
    """
    Manages dynamic multi-stream video pipeline.
    
    Each stream consists of:
    1. ROS video bridge (inside container) - subscribes to ROS topic, sends to TCP
    2. FFmpeg encoder (on host) - reads from TCP, encodes, pushes to MediaMTX
    """
    
    def __init__(self, container_name="nomad_isaac_ros_32"):
        self.container_name = container_name
        self.streams: Dict[str, StreamInfo] = {}
        self.lock = threading.RLock()
        self.next_port = 9999  # Start port allocation from 9999
        self.mediamtx_host = "localhost"
        self.mediamtx_port = 8554
        self._auto_started = False
        self._container_ready = False
        self._use_nvenc = True  # Default to NVENC hardware encoding
        
        logger.info(f"VideoStreamManager initialized for container: {container_name}")
        logger.info(f"Using {'NVENC hardware' if self._use_nvenc else 'libx264 software'} encoding")

    def set_use_nvenc(self, use_nvenc: bool) -> None:
        """Set whether to use NVENC hardware encoding (default: True)."""
        self._use_nvenc = use_nvenc
        logger.info(f"Encoding mode: {'NVENC hardware' if use_nvenc else 'libx264 software'}")

    def set_container_name(self, name: str) -> None:
        """Update the container name (useful when container is discovered dynamically)."""
        self.container_name = name
        logger.info(f"VideoStreamManager container updated to: {name}")

    def is_container_running(self) -> bool:
        """Check if the Isaac ROS container is running."""
        try:
            result = subprocess.run(
                ["docker", "ps", "--filter", f"name={self.container_name}", "--format", "{{.Names}}"],
                capture_output=True,
                text=True,
                timeout=5
            )
            running = self.container_name in result.stdout
            self._container_ready = running
            return running
        except Exception as e:
            logger.error(f"Error checking container status: {e}")
            return False

    def wait_for_container(self, timeout: int = 60) -> bool:
        """Wait for the container to be running."""
        logger.info(f"Waiting for container '{self.container_name}' (max {timeout}s)...")
        start = time.time()
        while time.time() - start < timeout:
            if self.is_container_running():
                logger.info(f"Container ready after {int(time.time() - start)}s")
                return True
            time.sleep(2)
        logger.warning(f"Container not ready after {timeout}s")
        return False

    def auto_start_default_stream(self) -> Optional[Dict]:
        """
        Auto-start a default video stream when edge_core initializes.
        
        Tries to find an available ZED topic and starts streaming it as 'zed'.
        This is called automatically after Isaac ROS container is ready.
        
        Returns:
            Stream info dict if successful, None otherwise.
        """
        if self._auto_started:
            logger.info("Auto-start already completed, skipping")
            return None
        
        if not self.is_container_running():
            logger.warning("Cannot auto-start: container not running")
            return None
        
        # Find available image topics
        topics = self.list_topics()
        if not topics:
            logger.warning("No image topics available for auto-start")
            return None
        
        # Try preferred topics first, then any available
        topic_to_use = None
        for preferred in DEFAULT_ZED_TOPICS:
            if preferred in topics:
                topic_to_use = preferred
                break
        
        if not topic_to_use:
            # Use first available image topic
            topic_to_use = topics[0]
        
        logger.info(f"Auto-starting default stream with topic: {topic_to_use}")
        
        try:
            stream = self.start_stream(
                stream_name="zed",
                topic=topic_to_use,
                width=1280,
                height=720,
                fps=30
            )
            self._auto_started = True
            logger.info(f"Auto-start successful: {stream['rtsp_url']}")
            return stream
        except Exception as e:
            logger.error(f"Auto-start failed: {e}")
            return None

    def list_topics(self) -> List[str]:
        """List available image topics from ROS 2."""
        try:
            cmd = [
                "docker", "exec", self.container_name,
                "bash", "-c",
                "source /opt/ros/humble/setup.bash && ros2 topic list -t 2>/dev/null"
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            
            if result.returncode != 0:
                logger.error(f"Failed to list topics: {result.stderr}")
                return []
                
            topics = []
            for line in result.stdout.splitlines():
                parts = line.split()
                if len(parts) >= 2:
                    topic, type_ = parts[0], parts[1].strip('[]')
                    # Filter for image topics
                    if "sensor_msgs/msg/Image" in type_ or "CompressedImage" in type_:
                        topics.append(topic)
                        
            return sorted(topics)
            
        except Exception as e:
            logger.error(f"Error listing topics: {e}")
            return []

    def list_streams(self) -> List[Dict]:
        """List all active streams."""
        with self.lock:
            return [stream.to_dict() for stream in self.streams.values()]

    def get_stream(self, stream_name: str) -> Optional[Dict]:
        """Get info about a specific stream."""
        with self.lock:
            stream = self.streams.get(stream_name)
            return stream.to_dict() if stream else None

    def start_stream(
        self,
        stream_name: str,
        topic: str,
        width: int = 1280,
        height: int = 720,
        fps: int = 30
    ) -> Dict:
        """
        Start a new video stream.
        
        Args:
            stream_name: Name for the RTSP stream (e.g., 'zed_left')
            topic: ROS image topic to subscribe to
            width: Output video width
            height: Output video height
            fps: Output video FPS
            
        Returns:
            Stream info dict with RTSP URL
        """
        with self.lock:
            # Check if stream already exists
            if stream_name in self.streams:
                raise ValueError(f"Stream '{stream_name}' already exists. Stop it first.")
            
            # Allocate TCP port
            tcp_port = self._allocate_port()
            rtsp_url = f"rtsp://{self.mediamtx_host}:{self.mediamtx_port}/{stream_name}"
            
            logger.info(f"Starting stream '{stream_name}': {topic} -> {rtsp_url}")
            
            if self._use_nvenc:
                # Use NVENC hardware encoding (single process in container)
                bridge_pid = self._start_nvenc_bridge(stream_name, topic, rtsp_url, width, height, fps)
                encoder_pid = None  # NVENC bridge handles encoding internally
            else:
                # Use legacy TCP + FFmpeg libx264 approach
                bridge_pid = self._start_bridge(stream_name, topic, tcp_port, width, height, fps)
                time.sleep(2)
                encoder_pid = self._start_encoder(stream_name, tcp_port, rtsp_url, width, height, fps)
            
            # Track stream
            stream = StreamInfo(
                stream_name=stream_name,
                topic=topic,
                tcp_port=tcp_port,
                width=width,
                height=height,
                fps=fps,
                rtsp_url=rtsp_url,
                bridge_pid=bridge_pid,
                encoder_pid=encoder_pid,
                started_at=time.time()
            )
            self.streams[stream_name] = stream
            
            logger.info(f"Stream '{stream_name}' started successfully")
            return stream.to_dict()

    def stop_stream(self, stream_name: str) -> bool:
        """
        Stop a specific stream.
        
        Args:
            stream_name: Name of the stream to stop
            
        Returns:
            True if stopped, False if not found
        """
        with self.lock:
            stream = self.streams.get(stream_name)
            if not stream:
                logger.warning(f"Stream '{stream_name}' not found")
                return False
            
            logger.info(f"Stopping stream '{stream_name}'")
            
            # Stop FFmpeg encoder first (consumer) - use pkill to find by TCP port
            if stream.encoder_pid and stream.encoder_pid != -1:
                try:
                    subprocess.run(["pkill", "-f", f"tcp://127.0.0.1:{stream.tcp_port}"], timeout=2, capture_output=True)
                    time.sleep(0.5)
                    subprocess.run(["pkill", "-9", "-f", f"tcp://127.0.0.1:{stream.tcp_port}"], timeout=2, capture_output=True)
                except:
                    pass
            
            # Stop bridge inside container by matching the stream name in process args
            # Since we can't reliably get PID, we kill by matching the topic pattern
            try:
                # Kill any bridge process that is streaming to this stream name
                subprocess.run([
                    "docker", "exec", self.container_name,
                    "pkill", "-f", f"--stream '{stream_name}'"
                ], timeout=5, capture_output=True)
                time.sleep(0.5)
                subprocess.run([
                    "docker", "exec", self.container_name,
                    "pkill", "-9", "-f", f"--stream '{stream_name}'"
                ], timeout=5, capture_output=True)
            except Exception as e:
                logger.warning(f"Error killing bridge: {e}")
            
            # Remove from tracking
            del self.streams[stream_name]
            logger.info(f"Stream '{stream_name}' stopped")
            return True

    def stop_all_streams(self) -> int:
        """Stop all active streams. Returns count stopped."""
        with self.lock:
            stream_names = list(self.streams.keys())
            count = 0
            for name in stream_names:
                if self.stop_stream(name):
                    count += 1
            return count

    def _allocate_port(self) -> int:
        """Allocate next available TCP port."""
        port = self.next_port
        self.next_port += 1
        return port

    def _start_nvenc_bridge(
        self,
        stream_name: str,
        topic: str,
        rtsp_url: str,
        width: int,
        height: int,
        fps: int
    ) -> Optional[int]:
        """Start NVENC video bridge inside container (hardware encoding). Returns PID.
        
        This approach uses GStreamer with nvv4l2h264enc for hardware H.264 encoding
        directly in the container. Lower CPU/memory usage than TCP+FFmpeg libx264.
        """
        try:
            # Copy NVENC bridge script to container /tmp
            bridge_script = os.path.join(os.path.dirname(__file__), 'ros_video_bridge_nvenc.py')
            if os.path.exists(bridge_script):
                subprocess.run(
                    ["docker", "cp", bridge_script, f"{self.container_name}:/tmp/ros_video_bridge_nvenc.py"],
                    capture_output=True,
                    timeout=5
                )
            else:
                logger.warning(f"NVENC bridge script not found: {bridge_script}, falling back to TCP+FFmpeg")
                # Fallback to legacy bridge
                tcp_port = self._allocate_port()
                pid = self._start_bridge(stream_name, topic, tcp_port, width, height, fps)
                time.sleep(2)
                self._start_encoder(stream_name, tcp_port, rtsp_url, width, height, fps)
                return pid
            
            # Start NVENC bridge in background using docker exec -d
            cmd = [
                "docker", "exec", "-d", self.container_name,
                "bash", "-c",
                f"source /opt/ros/humble/setup.bash && "
                f"source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null ; "
                f"python3 /tmp/ros_video_bridge_nvenc.py "
                f"--topic '{topic}' "
                f"--stream '{stream_name}' "
                f"--host '{self.mediamtx_host}' "
                f"--port {self.mediamtx_port} "
                f"--width {width} "
                f"--height {height} "
                f"--fps {fps} "
                f"--bitrate 4000 "
                f"> /tmp/video_bridge_{stream_name}.log 2>&1"
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                logger.info(f"NVENC bridge started for '{stream_name}' (hardware encoding)")
                return 1  # Dummy PID
            else:
                logger.error(f"Failed to start NVENC bridge: {result.stderr}")
                return None
                
        except Exception as e:
            logger.error(f"Error starting NVENC bridge: {e}")
            return None

    def _start_bridge(
        self,
        stream_name: str,
        topic: str,
        tcp_port: int,
        width: int,
        height: int,
        fps: int
    ) -> Optional[int]:
        """Start ROS video bridge inside container. Returns PID."""
        try:
            # Copy bridge script to container /tmp
            bridge_script = os.path.join(os.path.dirname(__file__), 'ros_video_bridge.py')
            if os.path.exists(bridge_script):
                subprocess.run(
                    ["docker", "cp", bridge_script, f"{self.container_name}:/tmp/ros_video_bridge.py"],
                    capture_output=True,
                    timeout=5
                )
            else:
                logger.error(f"Bridge script not found: {bridge_script}")
                return None
            
            # Start bridge in background using docker exec -d
            # Note: PID tracking isn't reliable with 'docker exec -d', but the process will run
            cmd = [
                "docker", "exec", "-d", self.container_name,
                "bash", "-c",
                f"source /opt/ros/humble/setup.bash && "
                f"source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null ; "
                f"python3 /tmp/ros_video_bridge.py "
                f"--topic '{topic}' "
                f"--stream '{stream_name}' "
                f"--tcp-port {tcp_port} "
                f"--width {width} "
                f"--height {height} "
                f"--fps {fps} "
                f"> /tmp/video_bridge_{stream_name}.log 2>&1"
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                logger.info(f"Bridge started for '{stream_name}' on TCP port {tcp_port}")
                # Return a dummy PID since we can't reliably get it with 'docker exec -d'
                return 1
            else:
                logger.error(f"Failed to start bridge: {result.stderr}")
                return None
                
        except Exception as e:
            logger.error(f"Error starting bridge: {e}")
            return None

    def _start_encoder(
        self,
        stream_name: str,
        tcp_port: int,
        rtsp_url: str,
        width: int,
        height: int,
        fps: int
    ) -> Optional[int]:
        """Start FFmpeg encoder on host. Returns PID."""
        try:
            # Quality settings optimized for low latency
            cmd = [
                "nohup", "ffmpeg",
                "-f", "rawvideo",
                "-pix_fmt", "bgr24",
                "-s", f"{width}x{height}",
                "-r", str(fps),
                "-i", f"tcp://127.0.0.1:{tcp_port}",
                "-c:v", "libx264",
                "-preset", "ultrafast",
                "-tune", "zerolatency",
                "-crf", "18",
                "-g", "30",
                "-keyint_min", "15",
                "-bf", "0",
                "-maxrate", "4000k",
                "-bufsize", "2000k",
                "-flags", "+cgop",
                "-f", "rtsp",
                "-rtsp_transport", "tcp",
                rtsp_url
            ]
            
            log_file = f"/tmp/nomad_video/ffmpeg_{stream_name}.log"
            os.makedirs("/tmp/nomad_video", exist_ok=True)
            
            with open(log_file, "w") as log:
                process = subprocess.Popen(
                    cmd,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    preexec_fn=os.setpgrp  # Create new process group
                )
                
            logger.info(f"FFmpeg encoder started for '{stream_name}' (PID: {process.pid})")
            return process.pid
            
        except Exception as e:
            logger.error(f"Error starting encoder: {e}")
            return None


# Global instance
_video_manager: Optional[VideoStreamManager] = None


def get_video_manager() -> VideoStreamManager:
    """Get the global video manager instance."""
    global _video_manager
    if _video_manager is None:
        # Default to nomad_isaac_ros_32 (the common container name)
        _video_manager = VideoStreamManager(container_name="nomad_isaac_ros_32")
    return _video_manager


def init_video_manager(container_name: str = "nomad_isaac_ros_32", auto_start: bool = True) -> VideoStreamManager:
    """
    Initialize the global video manager with specific settings.
    
    Args:
        container_name: Name of the Isaac ROS Docker container
        auto_start: Whether to auto-start a default stream when container is ready
        
    Returns:
        The initialized VideoStreamManager instance
    """
    global _video_manager
    _video_manager = VideoStreamManager(container_name=container_name)
    
    # Clean up any legacy video processes from old approach
    _cleanup_legacy_video_processes(container_name)
    
    if auto_start:
        # Start auto-start in background thread to not block startup
        def _delayed_auto_start():
            # Wait for container to be ready
            if _video_manager.wait_for_container(timeout=90):
                # Wait a bit more for ZED to publish topics
                logger.info("Container ready, waiting for ZED topics...")
                time.sleep(10)
                _video_manager.auto_start_default_stream()
            else:
                logger.warning("Skipping video auto-start: container not available")
        
        thread = threading.Thread(target=_delayed_auto_start, daemon=True)
        thread.start()
        logger.info("Video auto-start scheduled in background")
    
    return _video_manager


def _cleanup_legacy_video_processes(container_name: str) -> None:
    """
    Clean up any legacy video bridge or FFmpeg processes from old static approach.
    
    This ensures the new dynamic VideoStreamManager has a clean slate.
    """
    logger.info("Cleaning up any legacy video processes...")
    
    try:
        # Kill any old FFmpeg RTSP processes on host
        subprocess.run(
            ["pkill", "-f", "ffmpeg.*rtsp://localhost:8554"],
            capture_output=True,
            timeout=5
        )
    except Exception as e:
        logger.debug(f"FFmpeg cleanup: {e}")
    
    try:
        # Kill any old video bridge processes in container
        subprocess.run(
            ["docker", "exec", container_name, "pkill", "-f", "ros_video_bridge"],
            capture_output=True,
            timeout=5
        )
    except Exception as e:
        logger.debug(f"Container video bridge cleanup: {e}")
    
    logger.info("Legacy video process cleanup complete")
