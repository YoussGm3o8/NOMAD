#!/bin/bash
# NOMAD Full System Startup Script
# Starts Edge Core API, RTSP Server, and ZED Video Stream

set -e

GCS_IP=100.76.127.17
API_PORT=8000
RTSP_PORT=8554
VIDEO_PORT=5600  # Fallback UDP port
LOG_DIR=/home/mad/nomad_logs
USE_RTSP=true  # Set to false to use direct UDP instead

mkdir -p $LOG_DIR

echo "=========================================="
echo "  NOMAD System Startup"
echo "=========================================="
echo "Ground Station: $GCS_IP"
echo "API Port: $API_PORT"
echo "RTSP Port: $RTSP_PORT"
echo ""

# Stop any existing services
echo "[1/4] Stopping existing services..."
pkill -9 -f "edge_core.main" 2>/dev/null || true
pkill -9 -f "gst-launch" 2>/dev/null || true
# Wait for processes to fully terminate and release camera
sleep 2

# Check if camera is still busy
echo "[2/4] Checking camera availability..."
if lsof /dev/video0 2>/dev/null | grep -v "^COMMAND"; then
    echo "    WARNING: Camera still in use by another process:"
    lsof /dev/video0
    echo "    Attempting to free camera..."
    fuser -k /dev/video0 2>/dev/null || true
    sleep 2
fi

# Start Edge Core API
echo "[3/4] Starting Edge Core API..."
cd /home/mad/NOMAD
export PATH=/home/mad/.local/bin:$PATH
export NOMAD_DEBUG=true  # Enable debug mode for terminal commands
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

# Start ZED Video Stream
echo "[4/5] Starting ZED Video Stream..."

if [ "$USE_RTSP" = true ]; then
    # Check if MediaMTX is running
    if ! pgrep -x mediamtx > /dev/null; then
        echo "    Starting MediaMTX RTSP server..."
        if [ -f /opt/mediamtx/mediamtx ]; then
            /opt/mediamtx/mediamtx /etc/mediamtx/mediamtx.yml > $LOG_DIR/mediamtx.log 2>&1 &
            RTSP_PID=$!
            sleep 2
        else
            echo "    WARNING: MediaMTX not installed. Falling back to UDP."
            echo "    Run: ./scripts/install_mediamtx.sh to enable RTSP"
            USE_RTSP=false
        fi
    else
        echo "    MediaMTX already running"
        RTSP_PID=""
    fi
fi

if [ "$USE_RTSP" = true ]; then
    # RTSP mode - allows multiple viewers
    RTSP_URL="rtsp://localhost:$RTSP_PORT/zed"
    gst-launch-1.0 -q \
      v4l2src device=/dev/video0 num-buffers=-1 ! \
      "video/x-raw,width=2560,height=720,framerate=30/1" ! \
      videocrop left=0 right=1280 ! \
      videoconvert ! \
      x264enc tune=zerolatency bitrate=4000 speed-preset=ultrafast sliced-threads=true key-int-max=15 bframes=0 ! \
      "video/x-h264,profile=baseline" ! \
      rtspclientsink location=$RTSP_URL protocols=tcp latency=0 > $LOG_DIR/video.log 2>&1 &
    VIDEO_PID=$!
    STREAM_URL="rtsp://100.75.218.89:$RTSP_PORT/zed"
else
    # UDP mode (fallback) - only ONE viewer at a time
    gst-launch-1.0 -q \
      v4l2src device=/dev/video0 num-buffers=-1 ! \
      "video/x-raw,width=2560,height=720,framerate=30/1" ! \
      videocrop left=0 right=1280 ! \
      videoconvert ! \
      x264enc tune=zerolatency bitrate=4000 speed-preset=ultrafast sliced-threads=true key-int-max=15 bframes=0 ! \
      "video/x-h264,profile=baseline,stream-format=byte-stream" ! \
      rtph264pay config-interval=1 pt=96 mtu=1400 ! \
      udpsink host=$GCS_IP port=$VIDEO_PORT sync=false > $LOG_DIR/video.log 2>&1 &
    VIDEO_PID=$!
    STREAM_URL="udp://$GCS_IP:$VIDEO_PORT (single viewer only)"
fi

sleep 1

if ps -p $VIDEO_PID > /dev/null 2>&1; then
    echo "    OK: Video stream running (PID: $VIDEO_PID)"
else
    echo "    FAIL: Video stream failed to start!"
    cat $LOG_DIR/video.log
fi

echo ""
echo "=========================================="
echo "  NOMAD System Running"
echo "=========================================="
echo "API:    http://100.75.218.89:$API_PORT"
echo "Video:  $STREAM_URL"
echo "Logs:   $LOG_DIR/"
echo ""
if [ "$USE_RTSP" = true ]; then
    echo "Multiple clients can connect to the same RTSP stream!"
fi
echo "Press Ctrl+C to stop all services"
echo ""

# Trap signals to cleanup on exit
cleanup() {
    echo ""
    echo "Shutting down NOMAD services..."
    kill $EDGE_PID 2>/dev/null || true
    kill $VIDEO_PID 2>/dev/null || true
    [ -n "$RTSP_PID" ] && kill $RTSP_PID 2>/dev/null || true
# Wait for services to exit
wait
