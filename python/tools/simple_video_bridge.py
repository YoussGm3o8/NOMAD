# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""
Simple Video Bridge — ROS2 image topic to RTSP via GStreamer.

Runs inside the Isaac ROS Docker container on the Jetson. Subscribes to a ROS2
image topic, encodes frames with x264enc (software, zerolatency tuning), and
streams to MediaMTX via RTSP. Also provides an HTTP API for topic switching,
status, and overlays.

Architecture:
  ROS2 Image Topic -> GStreamer x264enc -> RTSP -> MediaMTX -> Viewers

HTTP API (default port 9200):
  GET  /health       — liveness probe
  GET  /status       — streaming stats (fps, frame count, errors)
  POST /switch       — change source image topic
  GET  /topics       — list available sensor_msgs/Image topics
  POST /restart      — restart the GStreamer pipeline
  POST /overlay/enable|disable — toggle detection bounding-box overlay
  GET  /overlay/status — current overlay state

Usage:
  python3 simple_video_bridge.py \
    --source-topic /nomad/camera/image \
    --width 640 --height 360 --fps 15 --bitrate 800 \
    --http-port 9200
"""

from __future__ import annotations

import argparse
import logging
import os
import threading
import time

from python.tools.video_bridge_server import BridgeHTTPHandler, BridgeHTTPServer, subprocess

__all__ = ["BridgeHTTPHandler", "BridgeHTTPServer", "VideoBridge", "main", "subprocess"]


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("simple_video_bridge")


def _get_encoding_channels(encoding: str) -> int | None:
    return {
        "rgb8": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
        "mono8": 1,
    }.get(encoding)


def _unpack_image_buffer(buf, width: int, height: int, row_bytes: int, channels: int):
    expected = height * row_bytes
    if row_bytes < width * channels or buf.size < expected:
        return None
    raw = buf[:expected].reshape((height, row_bytes))
    return raw[:, : width * channels].reshape((height, width, channels))


def _convert_packed_to_rgb(packed, encoding: str):
    import numpy as np

    if encoding == "rgb8":
        return packed
    if encoding == "bgr8":
        return packed[:, :, ::-1]
    if encoding == "rgba8":
        return packed[:, :, :3]
    if encoding == "bgra8":
        return packed[:, :, [2, 1, 0]]
    return np.repeat(packed, 3, axis=2)


def _resize_rgb_frame(rgb, src_w: int, src_h: int, target_w: int, target_h: int):
    import numpy as np

    y_idx = np.linspace(0, src_h - 1, target_h).astype(np.intp)
    x_idx = np.linspace(0, src_w - 1, target_w).astype(np.intp)
    return rgb[y_idx][:, x_idx]


class VideoBridge:
    """Manages the GStreamer pipeline from ROS2 topic to RTSP."""

    def __init__(
        self,
        source_topic: str,
        width: int,
        height: int,
        fps: int,
        bitrate: int,
        rtsp_url: str,
        flip_method: str = "identity",
        rtsp_path: str = "stream",
    ):
        self._source_topic = source_topic
        self._width = width
        self._height = height
        self._fps = fps
        self._bitrate = bitrate
        self._rtsp_url = rtsp_url
        self._flip_method = flip_method
        self._rtsp_path = rtsp_path
        self._pipeline = None
        self._appsrc = None
        self._gst = None
        self._ros_node = None
        self._ros_subscription = None
        self._ros_executor = None
        self._ros_thread: threading.Thread | None = None
        self._ros_started = False
        self._lock = threading.Lock()
        self._running = False
        self._frame_count = 0
        self._error_count = 0
        self._dropped_count = 0
        self._fps_value = 0.0
        self._start_time = 0.0
        self._last_frame_time = 0.0
        self._overlay_enabled = False
        self._overlay_detection_count = 0

    @property
    def source_topic(self) -> str:
        return self._source_topic

    @property
    def running(self) -> bool:
        return self._running

    def start(self) -> bool:
        with self._lock:
            if self._running:
                return True
            return self._start_pipeline()

    def stop(self) -> None:
        with self._lock:
            self._stop_pipeline()

    def switch_topic(self, topic: str) -> bool:
        with self._lock:
            if topic == self._source_topic:
                return True
            self._stop_pipeline()
            self._source_topic = topic
            return self._start_pipeline()

    def restart(self) -> bool:
        with self._lock:
            self._stop_pipeline()
            return self._start_pipeline()

    def get_status(self) -> dict:
        return {
            "streaming": self._running,
            "source_topic": self._source_topic,
            "fps": self._fps_value,
            "frame_count": self._frame_count,
            "error_count": self._error_count,
            "dropped_count": self._dropped_count,
            "last_frame_age_s": time.time() - self._last_frame_time if self._last_frame_time > 0 else -1,
            "width": self._width,
            "height": self._height,
            "bitrate_kbps": self._bitrate,
        }

    def get_health(self) -> dict:
        return {
            "healthy": self._running,
            "pipeline_playing": self._running,
            "source_topic": self._source_topic,
        }

    def set_overlay(self, enabled: bool) -> None:
        self._overlay_enabled = enabled

    def get_overlay_status(self) -> dict:
        return {
            "enabled": self._overlay_enabled,
            "detection_count": self._overlay_detection_count,
        }

    def _build_gst_pipeline(self) -> str:
        return (
            "appsrc name=ros_source is-live=true block=false format=time do-timestamp=true "
            f"caps=video/x-raw,format=RGB,width={self._width},height={self._height},framerate={self._fps}/1 ! "
            "queue max-size-buffers=2 leaky=downstream ! "
            "videoconvert ! videoscale ! "
            f"video/x-raw,width={self._width},height={self._height},framerate={self._fps}/1 ! "
            f"videoflip method={self._flip_method} ! "
            f"x264enc tune=zerolatency bitrate={self._bitrate} speed-preset=ultrafast "
            f"key-int-max={self._fps * 2} ! "
            "video/x-h264,profile=baseline ! "
            f"rtspclientsink location={self._rtsp_url} protocols=tcp latency=0"
        )

    def _reset_stats(self) -> None:
        self._running = True
        self._frame_count = 0
        self._dropped_count = 0
        self._fps_value = 0.0
        self._start_time = time.time()
        self._last_frame_time = 0.0

    def _start_pipeline(self) -> bool:
        try:
            import gi

            gi.require_version("Gst", "1.0")
            from gi.repository import Gst

            Gst.init(None)
            self._gst = Gst
            self._pipeline = Gst.parse_launch(self._build_gst_pipeline())
            self._appsrc = self._pipeline.get_by_name("ros_source")
            self._pipeline.set_state(Gst.State.PLAYING)
            self._reset_stats()
            logger.info(f"GStreamer appsrc pipeline started: {self._source_topic} -> {self._rtsp_url}")

            if not self._start_ros_subscription():
                self._stop_pipeline()
                return False
            threading.Thread(target=self._monitor_pipeline, daemon=True).start()
            return True
        except Exception as e:
            logger.error(f"Failed to start GStreamer pipeline: {e}")
            self._error_count += 1
            return False

    def _start_ros_subscription(self) -> bool:
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import Image

            if not rclpy.ok():
                rclpy.init(args=None)
                self._ros_started = True

            self._ros_node = rclpy.create_node("nomad_simple_video_bridge")
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            )
            self._ros_subscription = self._ros_node.create_subscription(Image, self._source_topic, self._on_image, qos)
            # Own the executor so _stop_ros_subscription() can break the spin loop
            # before the node is destroyed; rclpy.spin() would otherwise keep
            # spinning a destroyed node on a topic switch.
            self._ros_executor = SingleThreadedExecutor()
            self._ros_executor.add_node(self._ros_node)
            self._ros_thread = threading.Thread(target=self._ros_executor.spin, daemon=True)
            self._ros_thread.start()
            logger.info(f"Subscribed to ROS image topic: {self._source_topic}")
            return True
        except Exception as e:
            logger.error(f"Failed to start ROS image subscription: {e}")
            self._error_count += 1
            return False

    def _stop_pipeline(self) -> None:
        self._stop_ros_subscription()
        if self._pipeline is not None:
            try:
                self._pipeline.set_state(self._gst.State.NULL)
            except Exception:
                pass
        self._pipeline = None
        self._appsrc = None
        self._running = False
        logger.info("GStreamer pipeline stopped")

    def _stop_ros_executor(self) -> None:
        if self._ros_executor is not None:
            try:
                self._ros_executor.shutdown()
            except Exception:
                pass
        ros_thread = self._ros_thread
        self._ros_thread = None
        if ros_thread is not None and ros_thread.is_alive():
            ros_thread.join(timeout=2.0)

    def _destroy_ros_entities(self) -> None:
        try:
            if self._ros_executor is not None and self._ros_node is not None:
                self._ros_executor.remove_node(self._ros_node)
        except Exception:
            pass
        self._ros_executor = None
        try:
            if self._ros_node is not None and self._ros_subscription is not None:
                self._ros_node.destroy_subscription(self._ros_subscription)
        except Exception:
            pass
        self._ros_subscription = None
        try:
            if self._ros_node is not None:
                self._ros_node.destroy_node()
        except Exception:
            pass
        self._ros_node = None

    def _stop_ros_subscription(self) -> None:
        self._stop_ros_executor()
        self._destroy_ros_entities()
        if self._ros_started:
            try:
                import rclpy

                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass
            self._ros_started = False

    def _on_image(self, msg) -> None:
        # Snapshot the pipeline handles: _stop_pipeline() may null them out from
        # another thread between this check and the push-buffer below.
        appsrc = self._appsrc
        gst = self._gst
        if not self._running or appsrc is None or gst is None:
            return
        try:
            frame = self._image_to_rgb(msg)
            if frame is None:
                return
            payload = frame.tobytes()
            buffer = gst.Buffer.new_allocate(None, len(payload), None)
            buffer.fill(0, payload)
            buffer.duration = gst.SECOND // max(self._fps, 1)
            ret = appsrc.emit("push-buffer", buffer)
            if ret != gst.FlowReturn.OK:
                self._dropped_count += 1
                return
            self._frame_count += 1
            self._last_frame_time = time.time()
            elapsed = self._last_frame_time - self._start_time
            if elapsed > 0:
                self._fps_value = self._frame_count / elapsed
        except Exception as e:
            self._error_count += 1
            logger.debug(f"Failed to push image frame: {e}")

    def _image_to_rgb(self, msg):

        import numpy as np

        encoding = (msg.encoding or "").lower()
        channels = _get_encoding_channels(encoding)
        if channels is None:
            self._error_count += 1
            logger.warning(f"Unsupported image encoding: {msg.encoding}")
            return None

        row_bytes, width, height = int(msg.step), int(msg.width), int(msg.height)
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        packed = _unpack_image_buffer(buf, width, height, row_bytes, channels)
        if packed is None:
            self._error_count += 1
            logger.warning(
                "Image buffer mismatch (encoding=%s, step=%d, w=%d, h=%d, bytes=%d); dropping frame",
                encoding,
                row_bytes,
                width,
                height,
                buf.size,
            )
            return None

        rgb = _convert_packed_to_rgb(packed, encoding)
        if width != self._width or height != self._height:
            rgb = _resize_rgb_frame(rgb, width, height, self._width, self._height)
        return np.ascontiguousarray(rgb)

    def _monitor_pipeline(self) -> None:
        if self._pipeline is None:
            return
        bus = self._pipeline.get_bus()
        while self._running:
            msg = bus.timed_pop_filtered(
                500_000_000,
                self._gst.MessageType.ERROR | self._gst.MessageType.EOS,
            )
            if msg is None:
                continue
            if msg.type == self._gst.MessageType.ERROR:
                err, debug = msg.parse_error()
                self._running = False
                self._error_count += 1
                logger.warning(f"GStreamer pipeline error: {err}; {debug}")
                break
            if msg.type == self._gst.MessageType.EOS:
                self._running = False
                logger.warning("GStreamer pipeline reached EOS")
                break


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="NOMAD Simple Video Bridge")
    parser.add_argument("--source-topic", required=True, help="ROS2 image topic")
    parser.add_argument("--width", type=int, default=640, help="Output width")
    parser.add_argument("--height", type=int, default=360, help="Output height")
    parser.add_argument("--fps", type=int, default=15, help="Output frame rate")
    parser.add_argument("--bitrate", type=int, default=800, help="H.264 bitrate (kbps)")
    parser.add_argument("--http-port", type=int, default=9200, help="HTTP API port")
    parser.add_argument("--rtsp-path", default="stream", help="RTSP stream path")
    parser.add_argument(
        "--flip-method",
        default=os.environ.get("NOMAD_VIDEO_FLIP_METHOD", "identity"),
        help="GStreamer videoflip method, e.g. identity, rotate-180, horizontal-flip.",
    )
    parser.add_argument(
        "--rtsp-url",
        default=os.environ.get("NOMAD_VIDEO_RTSP_PUBLISH_URL"),
        help="Full RTSP publish URL. Defaults to rtsp://localhost:8554/<rtsp-path>.",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    rtsp_url = args.rtsp_url or f"rtsp://localhost:8554/{args.rtsp_path}"
    bridge = VideoBridge(
        source_topic=args.source_topic,
        width=args.width,
        height=args.height,
        fps=args.fps,
        bitrate=args.bitrate,
        rtsp_url=rtsp_url,
        flip_method=args.flip_method,
        rtsp_path=args.rtsp_path,
    )
    if not bridge.start():
        logger.error("Failed to start video bridge pipeline")
        return

    server = BridgeHTTPServer("0.0.0.0", args.http_port, bridge)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    logger.info(f"HTTP API listening on port {args.http_port}")
    try:
        while bridge.running:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.stop()
        server.shutdown()


if __name__ == "__main__":
    main()
