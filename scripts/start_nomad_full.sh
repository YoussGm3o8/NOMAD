#!/bin/bash
# ============================================================
# NOMAD Full System Startup Script
# ============================================================
# Starts Edge Core API and ZED Video Stream via MediaMTX
# 
# Stream URL: rtsp://<JETSON_IP>:8554/zed
# Multiple viewers supported (Mission Planner, VLC, phone, etc.)
# ============================================================

set -e

# Configuration
JETSON_IP=100.75.218.89
API_PORT=8000
RTSP_PORT=8554
LOG_DIR=/home/mad/nomad_logs
NOMAD_DIR=/home/mad/NOMAD

mkdir -p $LOG_DIR

echo "=========================================="
echo "  NOMAD System Startup (MediaMTX RTSP)"
echo "  AEAC 2026 - McGill Aerial Design"
echo "=========================================="
echo "Jetson IP:  $JETSON_IP"
echo "API Port:   $API_PORT"
echo "RTSP Port:  $RTSP_PORT"
echo ""

# Stop any existing services
echo "[1/5] Stopping existing services..."
pkill -9 -f "edge_core.main" 2>/dev/null || true
pkill -9 -f "gst-launch" 2>/dev/null || true
sleep 2

# Check if camera is still busy
echo "[2/5] Checking camera availability..."
if lsof /dev/video0 2>/dev/null | grep -v "^COMMAND"; then
    echo "    WARNING: Camera still in use"
    fuser -k /dev/video0 2>/dev/null || true
    sleep 2
fi

# Check MediaMTX is running
echo "[3/5] Checking MediaMTX RTSP server..."
if pgrep -f mediamtx > /dev/null; then
    echo "    OK: MediaMTX running"
else
    echo "    FAIL: MediaMTX not running! Start with: sudo systemctl start mediamtx"
    exit 1
fi

# Start Edge Core API
echo "[4/5] Starting Edge Core API..."
cd $NOMAD_DIR
export PATH=/home/mad/.local/bin:$PATH
export NOMAD_DEBUG=true
nohup python3 -m edge_core.main > $LOG_DIR/edge_core.log 2>&1 &
EDGE_PID=$!
sleep 2

# Verify Edge Core is running
if curl -s http://localhost:$API_PORT/health > /dev/null; then
    echo "    OK: Edge Core running (PID: $EDGE_PID)"
else
    echo "    FAIL: Edge Core failed to start!"
    cat $LOG_DIR/edge_core.log
    exit 1
fi

# Start ZED Video Stream -> MediaMTX
echo "[5/5] Starting ZED Video Stream (2560x720 Stereo)..."
# Force 2560x720 side-by-side stereo mode
# This provides proper aspect ratio and full resolution

# Ultra-low latency H.264 pipeline for full stereo frame:
# - Full stereo: 2560x720 @ 30fps (side-by-side L+R cameras)
# - x264enc: zerolatency tune, ultrafast preset, no B-frames
# - Higher bitrate (6000) for wider frame quality
# - key-int-max=30: Frequent keyframes for fast recovery
gst-launch-1.0 -q \
  v4l2src device=/dev/video0 do-timestamp=true ! \
  "video/x-raw,width=2560,height=720,framerate=30/1,format=YUY2" ! \
  videoconvert ! \
  "video/x-raw,format=I420" ! \
  x264enc tune=zerolatency bitrate=6000 speed-preset=ultrafast \
    sliced-threads=true key-int-max=30 bframes=0 \
    rc-lookahead=0 sync-lookahead=0 ! \
  h264parse ! \
  rtspclientsink location=rtsp://localhost:$RTSP_PORT/zed \
    latency=0 protocols=tcp > $LOG_DIR/video.log 2>&1 &
VIDEO_PID=$!

sleep 3

if ps -p $VIDEO_PID > /dev/null 2>&1; then
    echo "    OK: Video stream running (PID: $VIDEO_PID) - 2560x720 stereo"
else
    echo "    FAIL: Primary method failed, trying without format specification..."
    cat $LOG_DIR/video.log
    
    # Method 2: Try without specifying format but keep resolution
    gst-launch-1.0 -q \
      v4l2src device=/dev/video0 do-timestamp=true ! \
      "video/x-raw,width=2560,height=720,framerate=30/1" ! \
      videoconvert ! \
      "video/x-raw,format=I420" ! \
      x264enc tune=zerolatency bitrate=6000 speed-preset=ultrafast \
        sliced-threads=true key-int-max=30 bframes=0 ! \
      h264parse ! \
      rtspclientsink location=rtsp://localhost:$RTSP_PORT/zed \
        latency=0 protocols=tcp > $LOG_DIR/video.log 2>&1 &
    VIDEO_PID=$!
    sleep 3
    
    if ps -p $VIDEO_PID > /dev/null 2>&1; then
        echo "    OK: Video stream (method 2) running (PID: $VIDEO_PID)"
    else
        echo "    FAIL: All methods failed! Check camera"
        cat $LOG_DIR/video.log
    fi
fi

echo ""
echo "=========================================="
echo "  NOMAD System Running"
echo "=========================================="
echo "API:    http://$JETSON_IP:$API_PORT"
echo "Video:  rtsp://$JETSON_IP:$RTSP_PORT/zed (2560x720 stereo)"
echo "Logs:   $LOG_DIR/"
echo ""
echo "Stereo stream: 2560x720 side-by-side (Left + Right)"
echo "Mission Planner crops to left camera (1280x720) in HUD"
echo "MediaMTX supports multiple viewers simultaneously!"
echo ""
echo "Press Ctrl+C to stop all services"
echo ""

# Trap signals to cleanup on exit
cleanup() {
    echo ""
    echo "Shutting down NOMAD services..."
    kill $EDGE_PID 2>/dev/null || true
    kill $VIDEO_PID 2>/dev/null || true
    echo "Goodbye!"
}
trap cleanup EXIT INT TERM

# Wait for services to exit
wait
