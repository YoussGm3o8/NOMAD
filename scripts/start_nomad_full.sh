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
echo "[5/5] Starting ZED Video Stream (Auto-Detect Resolution)..."
# Auto-detect camera capabilities and adapt accordingly
# Many ZED2i cameras output 1344x376 @ 15fps in certain modes

# First, detect actual camera resolution
CAMERA_INFO=$(v4l2-ctl --device=/dev/video0 --list-formats-ext 2>/dev/null | grep -A1 "YUYV" | tail -1 || echo "")
echo "    Detected camera format: $CAMERA_INFO"

# Try Method 1: GStreamer with auto-negotiation (most reliable)
echo "    Attempting GStreamer with auto-negotiation..."
gst-launch-1.0 -q \
  v4l2src device=/dev/video0 do-timestamp=true ! \
  "video/x-raw,format=YUY2" ! \
  videoscale ! \
  "video/x-raw,width=1280,height=720" ! \
  videoconvert ! \
  "video/x-raw,format=I420" ! \
  x264enc tune=zerolatency bitrate=4000 speed-preset=ultrafast \
    sliced-threads=true key-int-max=30 bframes=0 \
    rc-lookahead=0 sync-lookahead=0 ! \
  h264parse ! \
  rtspclientsink location=rtsp://localhost:$RTSP_PORT/zed \
    latency=0 protocols=tcp > $LOG_DIR/video.log 2>&1 &
VIDEO_PID=$!

sleep 3

if ps -p $VIDEO_PID > /dev/null 2>&1; then
    echo "    OK: Video stream running (PID: $VIDEO_PID)"
else
    echo "    FAIL: GStreamer failed, trying ffmpeg..."
    cat $LOG_DIR/video.log
    
    # Method 2: Use ffmpeg with auto format detection
    if command -v ffmpeg &> /dev/null; then
        # Let ffmpeg auto-detect resolution and scale to 1280x720
        ffmpeg -f v4l2 -input_format yuyv422 -i /dev/video0 \
            -vf "scale=1280:720:force_original_aspect_ratio=decrease,pad=1280:720:(ow-iw)/2:(oh-ih)/2" \
            -c:v libx264 -preset ultrafast -tune zerolatency -b:v 3000k \
            -g 30 -bf 0 -max_delay 0 \
            -f rtsp -rtsp_transport tcp rtsp://localhost:$RTSP_PORT/zed > $LOG_DIR/video.log 2>&1 &
        VIDEO_PID=$!
        sleep 3
        
        if ps -p $VIDEO_PID > /dev/null 2>&1; then
            echo "    OK: Video stream (ffmpeg) running (PID: $VIDEO_PID)"
        else
            echo "    FAIL: ffmpeg also failed!"
            echo "    Trying minimal passthrough mode..."
            cat $LOG_DIR/video.log
            
            # Method 3: Minimal passthrough - just grab and encode whatever we get
            gst-launch-1.0 -q \
              v4l2src device=/dev/video0 ! \
              videoconvert ! \
              x264enc tune=zerolatency bitrate=3000 speed-preset=ultrafast ! \
              h264parse ! \
              rtspclientsink location=rtsp://localhost:$RTSP_PORT/zed \
                protocols=tcp > $LOG_DIR/video.log 2>&1 &
            VIDEO_PID=$!
            sleep 2
            
            if ps -p $VIDEO_PID > /dev/null 2>&1; then
                echo "    OK: Video stream (minimal) running (PID: $VIDEO_PID)"
            else
                echo "    FAIL: All methods failed! Check camera connection"
                cat $LOG_DIR/video.log
            fi
        fi
    fi
fi

echo ""
echo "=========================================="
echo "  NOMAD System Running"
echo "=========================================="
echo "API:    http://$JETSON_IP:$API_PORT"
echo "Video:  rtsp://$JETSON_IP:$RTSP_PORT/zed"
echo "Logs:   $LOG_DIR/"
echo ""
echo "MediaMTX supports multiple viewers simultaneously!"
echo "Use in: Mission Planner, VLC, or any RTSP client"
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
