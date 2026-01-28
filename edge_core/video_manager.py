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
    
    def __init__(self, container_name="nomad_isaac_ros"):
        self.container_name = container_name
        self.streams: Dict[str, StreamInfo] = {}
        self.lock = threading.RLock()
        self.next_port = 9999  # Start port allocation from 9999
        self.mediamtx_host = "localhost"
        self.mediamtx_port = 8554
        
        logger.info(f"VideoStreamManager initialized for container: {container_name}")

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
            
            # Start ROS video bridge inside container
            bridge_pid = self._start_bridge(stream_name, topic, tcp_port, width, height, fps)
            
            # Wait for bridge to initialize
            time.sleep(2)
            
            # Start FFmpeg encoder on host
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
            
            # Stop FFmpeg encoder first (consumer)
            if stream.encoder_pid:
                try:
                    subprocess.run(["kill", "-TERM", str(stream.encoder_pid)], timeout=2)
                    time.sleep(1)
                    subprocess.run(["kill", "-KILL", str(stream.encoder_pid)], timeout=1)
                except:
                    pass
            
            # Stop bridge inside container (producer)
            if stream.bridge_pid:
                try:
                    subprocess.run([
                        "docker", "exec", self.container_name,
                        "kill", "-TERM", str(stream.bridge_pid)
                    ], timeout=2)
                    time.sleep(1)
                    subprocess.run([
                        "docker", "exec", self.container_name,
                        "kill", "-KILL", str(stream.bridge_pid)
                    ], timeout=1)
                except:
                    pass
            
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
            # Copy bridge script to container /tmp if not already there
            bridge_script = os.path.join(os.path.dirname(__file__), '../edge_core/ros_video_bridge.py')
            if os.path.exists(bridge_script):
                subprocess.run(
                    ["docker", "cp", bridge_script, f"{self.container_name}:/tmp/ros_video_bridge.py"],
                    capture_output=True,
                    timeout=5
                )
            
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
_video_manager = VideoStreamManager()


def get_video_manager() -> VideoStreamManager:
    """Get the global video manager instance."""
    return _video_manager
