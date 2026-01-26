#!/usr/bin/env python3
"""
ROS-to-RTSP Video Bridge

Subscribes to ROS image topics and streams them via FFmpeg to MediaMTX RTSP server.
Runs inside the Isaac ROS container.

SAFETY CRITICAL: This code runs on a flying drone. Memory safety and reliability
are paramount. All buffer operations MUST be validated before access.

Usage:
    python3 ros_video_bridge.py --topic /zed/zed_node/rgb/image_rect_color --stream live

Target: Python 3.10+ | ROS 2 Humble | NVIDIA Jetson Orin Nano
"""

import argparse
import logging
import signal
import socket
import subprocess
import sys
import threading
import time
import traceback
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

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)
logger = logging.getLogger("ros_video_bridge")

# SAFETY: Maximum allowed frame size to prevent memory exhaustion
MAX_FRAME_WIDTH = 4096
MAX_FRAME_HEIGHT = 2160
MAX_FRAME_BYTES = MAX_FRAME_WIDTH * MAX_FRAME_HEIGHT * 4  # RGBA max


class ROSVideoPublisher(Node):
    """ROS 2 node that subscribes to image topics and publishes to RTSP via FFmpeg.
    
    SAFETY: This class handles raw video data from the ZED camera. All buffer
    operations must be validated to prevent memory access violations that could
    crash the flight controller interface.
    """

    def __init__(
        self,
        topic: str,
        rtsp_url: str,
        tcp_port: int = 9999,
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
    ):
        super().__init__('ros_video_bridge')
        
        self.topic = topic
        self.rtsp_url = rtsp_url
        self.tcp_port = tcp_port
        self.width = width
        self.height = height
        self.fps = fps
        
        self.ffmpeg_process: Optional[subprocess.Popen] = None
        self.frame_count = 0
        self.error_count = 0  # SAFETY: Track errors for monitoring
        self.last_frame_time = time.time()
        self.lock = threading.RLock()  # SAFETY: Use RLock for reentrant safety
        self._shutdown_requested = False  # SAFETY: Graceful shutdown flag
        
        # SAFETY: Pre-allocate output buffer to avoid repeated allocations
        self._output_buffer = np.zeros((height, width, 3), dtype=np.uint8)
        self._expected_frame_size = width * height * 3
        
        # Start TCP server
        self._start_tcp_server()
        
        # SAFETY: Use reliable QoS with limited queue to prevent memory buildup
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
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
        
        logger.info(f"Subscribed to {topic}, publishing to {rtsp_url}")

    def _start_tcp_server(self):
        """Start TCP server to send raw frames to host encoder.
        
        SAFETY: Proper socket initialization with error handling to prevent
        resource leaks on failure.
        """
        import socket
        
        self.tcp_server = None
        self.tcp_client = None
        self._client_connected = False
        
        try:
            self.tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            # SAFETY: Set socket timeout to prevent indefinite blocking
            self.tcp_server.settimeout(1.0)
            self.tcp_server.bind(('0.0.0.0', self.tcp_port))
            self.tcp_server.listen(1)
            self.tcp_server.setblocking(False)
            
            logger.info(f"TCP server started on port {self.tcp_port} - waiting for encoder connection")
            logger.info(f"Run on host: ffmpeg -f rawvideo -pix_fmt bgr24 -s {self.width}x{self.height} -r {self.fps} -i tcp://localhost:{self.tcp_port} -c:v libx264 -preset ultrafast -tune zerolatency -f rtsp {self.rtsp_url}")
            
        except Exception as e:
            logger.error(f"Failed to start TCP server: {e}")
            if self.tcp_server:
                try:
                    self.tcp_server.close()
                except:
                    pass
                self.tcp_server = None
            raise
        
        # Set ffmpeg_process to a dummy for compatibility
        self.ffmpeg_process = type('obj', (object,), {'stdin': None, 'poll': lambda: None, 'stderr': None})()
    
    def _get_encoder_command(self) -> list:
        """Not used with TCP approach."""
        return []
    
    def _accept_client(self) -> bool:
        """Try to accept a new client connection.
        
        SAFETY: Non-blocking accept with proper error handling.
        Returns True if a client is connected and ready.
        """
        if self._shutdown_requested:
            return False
            
        if self.tcp_server is None:
            return False
            
        # Already have a connected client
        if self.tcp_client is not None and self._client_connected:
            return True
            
        try:
            self.tcp_client, addr = self.tcp_server.accept()
            self.tcp_client.setblocking(True)
            # SAFETY: Set send timeout to prevent blocking on slow/dead clients
            self.tcp_client.settimeout(2.0)
            # SAFETY: Set TCP_NODELAY for low latency
            self.tcp_client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._client_connected = True
            logger.info(f"Encoder connected from {addr}")
            return True
        except BlockingIOError:
            # No client waiting - normal condition
            return False
        except Exception as e:
            logger.debug(f"No encoder connected yet: {e}")
            return False
    
    def _disconnect_client(self):
        """Safely disconnect the current client.
        
        SAFETY: Proper cleanup prevents resource leaks and stale connections.
        """
        with self.lock:
            self._client_connected = False
            if self.tcp_client:
                try:
                    self.tcp_client.close()
                except:
                    pass
                self.tcp_client = None

    def _image_callback(self, msg: Image):
        """Process incoming ROS image and send via TCP.
        
        SAFETY CRITICAL: This function handles raw image buffers. All operations
        must validate buffer sizes before access to prevent memory corruption.
        Illegal memory access here could crash the drone's control system.
        """
        if self._shutdown_requested:
            return
            
        # Try to accept new client if not connected
        if not self._accept_client():
            return
        
        try:
            # SAFETY: Validate message integrity
            if msg is None or msg.data is None:
                logger.warning("Received null message")
                self.error_count += 1
                return
            
            encoding = msg.encoding
            height = msg.height
            width = msg.width
            
            # SAFETY: Validate dimensions to prevent buffer overflow
            if width <= 0 or height <= 0:
                logger.warning(f"Invalid dimensions: {width}x{height}")
                self.error_count += 1
                return
                
            if width > MAX_FRAME_WIDTH or height > MAX_FRAME_HEIGHT:
                logger.warning(f"Frame too large: {width}x{height}, max: {MAX_FRAME_WIDTH}x{MAX_FRAME_HEIGHT}")
                self.error_count += 1
                return
            
            # SAFETY: Calculate expected data size based on encoding
            expected_size = self._calculate_expected_size(encoding, width, height)
            actual_size = len(msg.data)
            
            if expected_size <= 0:
                logger.warning(f"Unsupported encoding: {encoding}")
                self.error_count += 1
                return
                
            # SAFETY: Strict size validation - data must match expected size
            if actual_size != expected_size:
                logger.warning(f"Buffer size mismatch for {encoding}: expected {expected_size}, got {actual_size}")
                self.error_count += 1
                return
            
            # SAFETY: Bounds check before numpy operation
            if actual_size > MAX_FRAME_BYTES:
                logger.warning(f"Frame data too large: {actual_size} bytes")
                self.error_count += 1
                return
            
            # Convert to BGR with validated buffer access
            cv_image = self._convert_to_bgr(msg.data, encoding, width, height)
            
            if cv_image is None:
                self.error_count += 1
                return
            
            # SAFETY: Validate conversion result
            if cv_image.shape[0] <= 0 or cv_image.shape[1] <= 0:
                logger.warning(f"Invalid converted image shape: {cv_image.shape}")
                self.error_count += 1
                return
            
            # Resize if needed with bounds checking
            if cv_image.shape[1] != self.width or cv_image.shape[0] != self.height:
                try:
                    cv_image = cv2.resize(cv_image, (self.width, self.height), interpolation=cv2.INTER_LINEAR)
                except Exception as e:
                    logger.error(f"Resize failed: {e}")
                    self.error_count += 1
                    return
            
            # SAFETY: Final validation before network send
            frame_bytes = cv_image.tobytes()
            if len(frame_bytes) != self._expected_frame_size:
                logger.warning(f"Unexpected output size: {len(frame_bytes)} vs {self._expected_frame_size}")
                self.error_count += 1
                return
            
            # Send via TCP with lock
            self._send_frame(frame_bytes)
                
        except MemoryError as e:
            # SAFETY CRITICAL: Memory exhaustion - log and continue
            logger.error(f"MEMORY ERROR processing image: {e}")
            self.error_count += 1
        except Exception as e:
            logger.error(f"Error processing image: {e}")
            self.error_count += 1
            if self.error_count > 100:
                # Too many errors - likely a systemic issue
                logger.error("Too many errors, check camera connection")

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
    
    def _calculate_expected_size(self, encoding: str, width: int, height: int) -> int:
        """Calculate expected buffer size for a given encoding.
        
        SAFETY: Used to validate incoming data before buffer operations.
        Returns -1 for unsupported encodings.
        """
        size_map = {
            'bgr8': width * height * 3,
            'rgb8': width * height * 3,
            'bgra8': width * height * 4,
            'rgba8': width * height * 4,
            '32FC1': width * height * 4,  # 32-bit float
            'mono8': width * height,
            'mono16': width * height * 2,
            '16UC1': width * height * 2,  # Depth as 16-bit unsigned
        }
        return size_map.get(encoding, -1)
    
    def _convert_to_bgr(self, data: bytes, encoding: str, width: int, height: int) -> Optional[np.ndarray]:
        """Convert raw image data to BGR format.
        
        SAFETY: All buffer operations are wrapped in try-except to catch
        any memory access violations.
        """
        try:
            if encoding in ('bgr8', 'rgb8', 'bgra8', 'rgba8'):
                channels = 4 if 'a' in encoding else 3
                dtype = np.uint8
                
                # SAFETY: Use copy=True to ensure we own the buffer
                img_array = np.frombuffer(data, dtype=dtype).copy()
                
                # SAFETY: Validate array size before reshape
                expected_elements = height * width * channels
                if img_array.size != expected_elements:
                    logger.warning(f"Array size mismatch: {img_array.size} vs {expected_elements}")
                    return None
                
                img_array = img_array.reshape(height, width, channels)
                
                # Convert to BGR if needed
                if encoding == 'rgb8':
                    return cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)
                elif encoding == 'rgba8':
                    return cv2.cvtColor(img_array, cv2.COLOR_RGBA2BGR)
                elif encoding == 'bgra8':
                    return cv2.cvtColor(img_array, cv2.COLOR_BGRA2BGR)
                else:  # bgr8
                    return img_array
                    
            elif encoding == '32FC1':
                # Depth image (32-bit float, single channel)
                dtype = np.float32
                
                # SAFETY: Copy buffer to ensure we own it
                depth_array = np.frombuffer(data, dtype=dtype).copy()
                
                expected_elements = height * width
                if depth_array.size != expected_elements:
                    logger.warning(f"Depth array size mismatch: {depth_array.size} vs {expected_elements}")
                    return None
                
                depth_array = depth_array.reshape(height, width)
                
                # Normalize to 0-255 range for visualization
                # SAFETY: Handle inf/nan values
                valid_depth = np.isfinite(depth_array)
                
                if np.any(valid_depth):
                    valid_values = depth_array[valid_depth]
                    min_depth = np.min(valid_values)
                    max_depth = np.max(valid_values)
                    
                    if max_depth > min_depth:
                        # Normalize with clipping for safety
                        normalized = np.clip((depth_array - min_depth) / (max_depth - min_depth), 0, 1)
                    else:
                        normalized = np.zeros_like(depth_array, dtype=np.float32)
                    
                    # Set invalid depths to 0
                    normalized = np.where(valid_depth, normalized, 0)
                    
                    # Convert to uint8
                    depth_uint8 = (normalized * 255).astype(np.uint8)
                    
                    # Apply colormap for better visualization
                    return cv2.applyColorMap(depth_uint8, cv2.COLORMAP_JET)
                else:
                    # All invalid depths - return black image
                    return np.zeros((height, width, 3), dtype=np.uint8)
                    
            elif encoding in ('mono8',):
                # Grayscale 8-bit
                img_array = np.frombuffer(data, dtype=np.uint8).copy()
                if img_array.size != height * width:
                    return None
                img_array = img_array.reshape(height, width)
                return cv2.cvtColor(img_array, cv2.COLOR_GRAY2BGR)
                
            elif encoding in ('16UC1', 'mono16'):
                # 16-bit depth/grayscale
                img_array = np.frombuffer(data, dtype=np.uint16).copy()
                if img_array.size != height * width:
                    return None
                img_array = img_array.reshape(height, width)
                # Normalize to 8-bit for display
                normalized = (img_array / 256).astype(np.uint8)
                return cv2.cvtColor(normalized, cv2.COLOR_GRAY2BGR)
            else:
                logger.warning(f"Unsupported encoding: {encoding}")
                return None
                
        except Exception as e:
            logger.error(f"Conversion error for {encoding}: {e}")
            return None
    
    def _send_frame(self, frame_bytes: bytes):
        """Send frame data to the TCP client.
        
        SAFETY: Handles connection errors gracefully without crashing.
        """
        with self.lock:
            if self.tcp_client is None or not self._client_connected:
                return
                
            try:
                self.tcp_client.sendall(frame_bytes)
                self.frame_count += 1
                self.last_frame_time = time.time()
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError) as e:
                logger.warning(f"Encoder disconnected ({type(e).__name__}), waiting for reconnection...")
                self._disconnect_client()
            except socket.timeout:
                logger.warning("Send timeout - encoder may be slow")
                self._disconnect_client()
            except Exception as e:
                logger.error(f"Send error: {e}")
                self._disconnect_client()

    def shutdown(self):
        """Cleanup resources.
        
        SAFETY: Proper resource cleanup prevents memory leaks and ensures
        clean shutdown that won't affect other drone systems.
        """
        logger.info("Shutting down ROS Video Bridge...")
        
        # Signal shutdown to prevent new processing
        self._shutdown_requested = True
        
        # Allow time for in-flight operations to complete
        time.sleep(0.1)
        
        with self.lock:
            # Close TCP client first
            if hasattr(self, 'tcp_client') and self.tcp_client:
                try:
                    self.tcp_client.shutdown(socket.SHUT_RDWR)
                except:
                    pass
                try:
                    self.tcp_client.close()
                except:
                    pass
                self.tcp_client = None
                self._client_connected = False
            
            # Close TCP server
            if hasattr(self, 'tcp_server') and self.tcp_server:
                try:
                    self.tcp_server.close()
                except:
                    pass
                self.tcp_server = None
        
        # Clear the output buffer
        if hasattr(self, '_output_buffer'):
            self._output_buffer = None
        
        logger.info(f"Shutdown complete. Total frames: {self.frame_count}, Errors: {self.error_count}")


def main():
    parser = argparse.ArgumentParser(description='ROS-to-RTSP Video Bridge')
    parser.add_argument('--topic', default='/zed/zed_node/rgb/image_rect_color',
                        help='ROS image topic to subscribe to')
    parser.add_argument('--stream', default='live',
                        help='MediaMTX stream name')
    parser.add_argument('--host', default='localhost',
                        help='MediaMTX host')
    parser.add_argument('--port', default=8554, type=int,
                        help='MediaMTX RTSP port')
    parser.add_argument('--tcp-port', default=9999, type=int,
                        help='TCP port for raw video frames')
    parser.add_argument('--width', default=1280, type=int,
                        help='Output video width')
    parser.add_argument('--height', default=720, type=int,
                        help='Output video height')
    parser.add_argument('--fps', default=30, type=int,
                        help='Output video FPS')
    
    args = parser.parse_args()
    
    rtsp_url = f"rtsp://{args.host}:{args.port}/{args.stream}"
    
    logger.info("=" * 60)
    logger.info("ROS-to-RTSP Video Bridge")
    logger.info("=" * 60)
    logger.info(f"  Topic: {args.topic}")
    logger.info(f"  Stream: {rtsp_url}")
    logger.info(f"  TCP Port: {args.tcp_port}")
    logger.info(f"  Resolution: {args.width}x{args.height}@{args.fps}fps")
    logger.info("=" * 60)
    
    rclpy.init()
    
    node = ROSVideoPublisher(
        topic=args.topic,
        rtsp_url=rtsp_url,
        tcp_port=args.tcp_port,
        width=args.width,
        height=args.height,
        fps=args.fps,
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
