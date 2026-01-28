#!/usr/bin/env python3
"""
ROS-to-RTSP Video Bridge with NVIDIA Hardware Encoding (NVENC)

Subscribes to ROS 2 image topics and streams via GStreamer NVENC to MediaMTX RTSP server.
Uses NVIDIA hardware encoder for low CPU/memory usage compared to libx264.

Runs inside the Isaac ROS container on Jetson Orin Nano.

SAFETY CRITICAL: This code runs on a flying drone. Memory safety and reliability
are paramount. All buffer operations MUST be validated before access.

Usage:
    python3 ros_video_bridge_nvenc.py --topic /zed/zed_node/rgb/image_rect_color --stream live

Target: Python 3.10+ | ROS 2 Humble | NVIDIA Jetson Orin Nano | GStreamer 1.x
"""

import argparse
import logging
import signal
import sys
import threading
import time
from typing import Optional

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image
    import cv2
    import numpy as np
except ImportError as e:
    print(f"ERROR: ROS 2 dependencies not available: {e}")
    print("This script must run inside a ROS 2 environment")
    sys.exit(1)

# GStreamer imports
try:
    import gi
    gi.require_version('Gst', '1.0')
    from gi.repository import Gst, GLib
    Gst.init(None)
    GSTREAMER_AVAILABLE = True
except ImportError:
    GSTREAMER_AVAILABLE = False
    print("WARNING: GStreamer Python bindings not available")

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)
logger = logging.getLogger("ros_video_bridge_nvenc")

# SAFETY: Maximum allowed frame size to prevent memory exhaustion
MAX_FRAME_WIDTH = 4096
MAX_FRAME_HEIGHT = 2160
MAX_FRAME_BYTES = MAX_FRAME_WIDTH * MAX_FRAME_HEIGHT * 4  # RGBA max


class NvencVideoBridge(Node):
    """ROS 2 node that subscribes to image topics and streams via GStreamer NVENC.
    
    Benefits over ros_video_bridge.py + FFmpeg libx264:
    - Uses NVIDIA hardware encoder (NVENC) instead of software x264
    - ~5-10% CPU usage vs ~60-90% for libx264
    - Lower memory usage (GPU memory vs RAM)
    - Lower latency (~5ms vs ~30ms)
    
    SAFETY: This class handles raw video data. All buffer operations
    must be validated to prevent memory access violations.
    """

    def __init__(
        self,
        topic: str,
        rtsp_url: str,
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
        bitrate: int = 4000,  # kbps
    ):
        super().__init__('ros_video_bridge_nvenc')
        
        self.topic = topic
        self.rtsp_url = rtsp_url
        self.width = width
        self.height = height
        self.fps = fps
        self.bitrate = bitrate
        
        self.pipeline: Optional[Gst.Pipeline] = None
        self.appsrc: Optional[Gst.Element] = None
        self.frame_count = 0
        self.error_count = 0
        self.last_frame_time = time.time()
        self.lock = threading.RLock()
        self._shutdown_requested = False
        self._pipeline_running = False
        
        # Pre-calculate expected sizes
        self._expected_frame_size = width * height * 3  # BGR
        
        # Initialize GStreamer pipeline
        if GSTREAMER_AVAILABLE:
            self._init_gstreamer_pipeline()
        else:
            logger.error("GStreamer not available - cannot start NVENC encoder")
            return
        
        # SAFETY: Use QoS compatible with ZED (RELIABLE publisher)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,  # Only keep latest frame
            durability=DurabilityPolicy.VOLATILE,
        )
        
        # Subscribe to image topic
        self.subscription = self.create_subscription(
            Image,
            topic,
            self._image_callback,
            sensor_qos
        )
        
        # Status timer
        self.status_timer = self.create_timer(5.0, self._status_callback)
        
        logger.info(f"Subscribed to {topic}, streaming via NVENC to {rtsp_url}")

    def _init_gstreamer_pipeline(self):
        """Initialize the GStreamer NVENC pipeline.
        
        Pipeline: appsrc -> videoconvert -> nvvidconv -> nvv4l2h264enc -> rtph264pay -> udpsink
        """
        # Build NVENC pipeline
        # Note: Using RTSP via rtspclientsink or direct to MediaMTX path
        pipeline_str = (
            f"appsrc name=src format=time is-live=true block=true do-timestamp=true "
            f"caps=video/x-raw,format=BGR,width={self.width},height={self.height},framerate={self.fps}/1 ! "
            f"queue max-size-buffers=2 leaky=downstream ! "
            f"videoconvert ! video/x-raw,format=BGRx ! "
            f"nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! "
            f"nvv4l2h264enc bitrate={self.bitrate * 1000} preset-level=1 "
            f"iframeinterval={self.fps} insert-sps-pps=true control-rate=1 ! "
            f"h264parse config-interval=1 ! "
            f"rtspclientsink location={self.rtsp_url} protocols=tcp latency=0"
        )
        
        logger.info(f"Creating NVENC pipeline: {pipeline_str[:100]}...")
        
        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
            self.appsrc = self.pipeline.get_by_name('src')
            
            if not self.appsrc:
                logger.error("Failed to get appsrc element from pipeline")
                return
            
            # Connect to bus messages
            bus = self.pipeline.get_bus()
            bus.add_signal_watch()
            bus.connect('message', self._on_bus_message)
            
            # Start pipeline
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                logger.error("Failed to start GStreamer pipeline")
                return
            
            self._pipeline_running = True
            logger.info("NVENC GStreamer pipeline started successfully")
            
        except Exception as e:
            logger.error(f"Failed to create GStreamer pipeline: {e}")
            self.pipeline = None

    def _on_bus_message(self, bus, message):
        """Handle GStreamer bus messages."""
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            logger.error(f"GStreamer error: {err.message}")
            logger.debug(f"Debug info: {debug}")
            self._pipeline_running = False
        elif t == Gst.MessageType.WARNING:
            err, debug = message.parse_warning()
            logger.warning(f"GStreamer warning: {err.message}")
        elif t == Gst.MessageType.EOS:
            logger.info("End of stream")
            self._pipeline_running = False
        elif t == Gst.MessageType.STATE_CHANGED:
            if message.src == self.pipeline:
                old, new, pending = message.parse_state_changed()
                logger.debug(f"Pipeline state: {old.value_nick} -> {new.value_nick}")

    def _image_callback(self, msg: Image):
        """Process incoming ROS image and push to GStreamer pipeline.
        
        SAFETY CRITICAL: All buffer operations must validate sizes.
        """
        if self._shutdown_requested or not self._pipeline_running:
            return
        
        if not self.appsrc:
            return
        
        try:
            # SAFETY: Validate message
            if msg is None or msg.data is None:
                logger.warning("Received null message")
                self.error_count += 1
                return
            
            encoding = msg.encoding
            height = msg.height
            width = msg.width
            
            # SAFETY: Validate dimensions
            if width <= 0 or height <= 0:
                logger.warning(f"Invalid dimensions: {width}x{height}")
                self.error_count += 1
                return
            
            if width > MAX_FRAME_WIDTH or height > MAX_FRAME_HEIGHT:
                logger.warning(f"Frame too large: {width}x{height}")
                self.error_count += 1
                return
            
            # Convert to BGR
            cv_image = self._convert_to_bgr(msg.data, encoding, width, height)
            if cv_image is None:
                self.error_count += 1
                return
            
            # Resize if needed
            if cv_image.shape[1] != self.width or cv_image.shape[0] != self.height:
                cv_image = cv2.resize(cv_image, (self.width, self.height), interpolation=cv2.INTER_LINEAR)
            
            # SAFETY: Validate output size
            if cv_image.nbytes != self._expected_frame_size:
                logger.warning(f"Unexpected frame size: {cv_image.nbytes} vs {self._expected_frame_size}")
                self.error_count += 1
                return
            
            # Push to GStreamer
            self._push_frame(cv_image)
            
        except MemoryError as e:
            logger.error(f"MEMORY ERROR processing image: {e}")
            self.error_count += 1
        except Exception as e:
            logger.error(f"Error processing image: {e}")
            self.error_count += 1

    def _convert_to_bgr(self, data: bytes, encoding: str, width: int, height: int) -> Optional[np.ndarray]:
        """Convert raw image data to BGR format.
        
        SAFETY: All buffer operations wrapped in try-except.
        """
        try:
            if encoding in ('bgr8', 'rgb8', 'bgra8', 'rgba8'):
                channels = 4 if 'a' in encoding else 3
                img_array = np.frombuffer(data, dtype=np.uint8).copy()
                
                expected_elements = height * width * channels
                if img_array.size != expected_elements:
                    return None
                
                img_array = img_array.reshape(height, width, channels)
                
                if encoding == 'rgb8':
                    return cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)
                elif encoding == 'rgba8':
                    return cv2.cvtColor(img_array, cv2.COLOR_RGBA2BGR)
                elif encoding == 'bgra8':
                    return cv2.cvtColor(img_array, cv2.COLOR_BGRA2BGR)
                else:  # bgr8
                    return img_array
                    
            elif encoding == '32FC1':
                # Depth image - convert to visualization
                depth_array = np.frombuffer(data, dtype=np.float32).copy()
                if depth_array.size != height * width:
                    return None
                depth_array = depth_array.reshape(height, width)
                
                valid_depth = np.isfinite(depth_array)
                if np.any(valid_depth):
                    valid_values = depth_array[valid_depth]
                    min_d, max_d = np.min(valid_values), np.max(valid_values)
                    if max_d > min_d:
                        normalized = np.clip((depth_array - min_d) / (max_d - min_d), 0, 1)
                    else:
                        normalized = np.zeros_like(depth_array)
                    normalized = np.where(valid_depth, normalized, 0)
                    depth_uint8 = (normalized * 255).astype(np.uint8)
                    return cv2.applyColorMap(depth_uint8, cv2.COLORMAP_JET)
                else:
                    return np.zeros((height, width, 3), dtype=np.uint8)
            else:
                logger.warning(f"Unsupported encoding: {encoding}")
                return None
        except Exception as e:
            logger.error(f"Conversion error for {encoding}: {e}")
            return None

    def _push_frame(self, frame: np.ndarray):
        """Push a frame to the GStreamer pipeline.
        
        SAFETY: Handles pipeline errors gracefully.
        """
        if not self._pipeline_running or not self.appsrc:
            return
        
        try:
            # Create GStreamer buffer from numpy array
            data = frame.tobytes()
            buf = Gst.Buffer.new_allocate(None, len(data), None)
            buf.fill(0, data)
            
            # Set timestamp
            pts = self.frame_count * (Gst.SECOND // self.fps)
            buf.pts = pts
            buf.duration = Gst.SECOND // self.fps
            
            # Push to appsrc
            ret = self.appsrc.emit('push-buffer', buf)
            if ret != Gst.FlowReturn.OK:
                if ret == Gst.FlowReturn.FLUSHING:
                    logger.debug("Pipeline flushing, dropping frame")
                else:
                    logger.warning(f"Failed to push buffer: {ret}")
                return
            
            self.frame_count += 1
            self.last_frame_time = time.time()
            
        except Exception as e:
            logger.error(f"Error pushing frame: {e}")
            self.error_count += 1

    def _status_callback(self):
        """Log status periodically."""
        elapsed = time.time() - self.last_frame_time
        if elapsed > 2.0:
            logger.warning(f"No frames received for {elapsed:.1f}s")
        else:
            status = f"Frames: {self.frame_count}"
            if self.error_count > 0:
                status += f", Errors: {self.error_count}"
            logger.info(status)

    def shutdown(self):
        """Cleanup resources."""
        logger.info("Shutting down NVENC Video Bridge...")
        
        self._shutdown_requested = True
        
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None
        
        logger.info(f"Shutdown complete. Frames: {self.frame_count}, Errors: {self.error_count}")


def main():
    parser = argparse.ArgumentParser(description='ROS-to-RTSP Video Bridge with NVENC')
    parser.add_argument('--topic', default='/zed/zed_node/rgb/image_rect_color',
                        help='ROS image topic to subscribe to')
    parser.add_argument('--stream', default='live',
                        help='MediaMTX stream name')
    parser.add_argument('--host', default='localhost',
                        help='MediaMTX host')
    parser.add_argument('--port', default=8554, type=int,
                        help='MediaMTX RTSP port')
    parser.add_argument('--width', default=1280, type=int,
                        help='Output video width')
    parser.add_argument('--height', default=720, type=int,
                        help='Output video height')
    parser.add_argument('--fps', default=30, type=int,
                        help='Output video FPS')
    parser.add_argument('--bitrate', default=4000, type=int,
                        help='Video bitrate in kbps')
    
    args = parser.parse_args()
    
    rtsp_url = f"rtsp://{args.host}:{args.port}/{args.stream}"
    
    logger.info("=" * 60)
    logger.info("ROS-to-RTSP Video Bridge (NVENC Hardware Encoding)")
    logger.info("=" * 60)
    logger.info(f"  Topic: {args.topic}")
    logger.info(f"  Stream: {rtsp_url}")
    logger.info(f"  Resolution: {args.width}x{args.height}@{args.fps}fps")
    logger.info(f"  Bitrate: {args.bitrate} kbps")
    logger.info(f"  Encoder: NVIDIA NVENC (nvv4l2h264enc)")
    logger.info("=" * 60)
    
    if not GSTREAMER_AVAILABLE:
        logger.error("GStreamer not available. Install with: apt install python3-gi gstreamer1.0-plugins-bad")
        sys.exit(1)
    
    rclpy.init()
    
    node = NvencVideoBridge(
        topic=args.topic,
        rtsp_url=rtsp_url,
        width=args.width,
        height=args.height,
        fps=args.fps,
        bitrate=args.bitrate,
    )
    
    def signal_handler(sig, frame):
        logger.info("Received shutdown signal")
        node.shutdown()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
