#!/bin/bash
# =============================================================================
# NOMAD Multi-Stream Video Manager
# =============================================================================
# Starts multiple video bridges for different ZED camera topics
# Each bridge publishes to a different MediaMTX stream
#
# Usage: ./start_multi_video.sh [start|stop|status]
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NOMAD_DIR="$(dirname "$SCRIPT_DIR")"
CONFIG_FILE="$NOMAD_DIR/config/video_streams.json"
LOG_DIR="/tmp/nomad_video"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_ok() { echo -e "${GREEN}[OK]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_fail() { echo -e "${RED}[FAIL]${NC} $1"; }

mkdir -p "$LOG_DIR"

# =============================================================================
# Start Video Bridges
# =============================================================================

start_bridges() {
    log_info "Starting ROS video bridges..."
    
    # Start each video bridge from config
    # Stereo Stream (side-by-side)
    log_info "Starting stereo stream bridge..."
    docker exec -d nomad_isaac_ros bash -c '
        source /opt/ros/humble/setup.bash && \
        source /workspaces/isaac_ros-dev/install/setup.bash && \
        python3 /tmp/ros_video_bridge.py \
            --topic /zed/zed_node/stereo/image_rect_color \
            --stream zed_stereo \
            --tcp-port 9999 \
            --host localhost \
            --port 8554 \
            --width 2560 \
            --height 720 \
            --fps 30 \
            > /tmp/video_bridge_stereo.log 2>&1
    '
    
    # Left camera
    log_info "Starting left camera stream bridge..."
    docker exec -d nomad_isaac_ros bash -c '
        source /opt/ros/humble/setup.bash && \
        source /workspaces/isaac_ros-dev/install/setup.bash && \
        python3 /tmp/ros_video_bridge.py \
            --topic /zed/zed_node/left/image_rect_color \
            --stream zed_left \
            --tcp-port 10000 \
            --host localhost \
            --port 8554 \
            --width 1280 \
            --height 720 \
            --fps 30 \
            > /tmp/video_bridge_left.log 2>&1
    '
    
    # Right camera
    log_info "Starting right camera stream bridge..."
    docker exec -d nomad_isaac_ros bash -c '
        source /opt/ros/humble/setup.bash && \
        source /workspaces/isaac_ros-dev/install/setup.bash && \
        python3 /tmp/ros_video_bridge.py \
            --topic /zed/zed_node/right/image_rect_color \
            --stream zed_right \
            --tcp-port 10001 \
            --host localhost \
            --port 8554 \
            --width 1280 \
            --height 720 \
            --fps 30 \
            > /tmp/video_bridge_right.log 2>&1
    '
    
    # Depth map
    log_info "Starting depth stream bridge..."
    docker exec -d nomad_isaac_ros bash -c '
        source /opt/ros/humble/setup.bash && \
        source /workspaces/isaac_ros-dev/install/setup.bash && \
        python3 /tmp/ros_video_bridge.py \
            --topic /zed/zed_node/depth/depth_registered \
            --stream zed_depth \
            --tcp-port 10002 \
            --host localhost \
            --port 8554 \
            --width 1280 \
            --height 720 \
            --fps 30 \
            > /tmp/video_bridge_depth.log 2>&1
    '
    
    sleep 5
    log_ok "Video bridges started"
}

# =============================================================================
# Start FFmpeg Encoders
# =============================================================================

start_encoders() {
    log_info "Starting FFmpeg encoders with quality optimizations..."
    
    # VIDEO QUALITY SETTINGS (optimized to prevent gray frames and artifacts):
    # - CRF 18: Higher quality (lower = better, 18 is visually lossless)
    # - profile:v baseline: Better compatibility and faster decode
    # - g=30: Keyframe every 1 second (30fps) - helps recover from stream switches
    # - keyint_min=15: Minimum keyframe interval
    # - bf=0: No B-frames for lower latency
    # - tune zerolatency: Optimized for real-time streaming
    # - maxrate/bufsize: Rate control to prevent bandwidth spikes
    
    # Stereo encoder (2560x720 side-by-side)
    nohup ffmpeg -f rawvideo -pix_fmt bgr24 -s 2560x720 -r 30 \
        -i tcp://127.0.0.1:9999 \
        -c:v libx264 -preset superfast -tune zerolatency \
        -profile:v baseline -level 4.0 \
        -crf 18 -g 30 -keyint_min 15 -bf 0 \
        -maxrate 8000k -bufsize 4000k \
        -flags +cgop \
        -f rtsp -rtsp_transport tcp \
        rtsp://localhost:8554/zed_stereo \
        > "$LOG_DIR/ffmpeg_stereo.log" 2>&1 &
    
    # Left camera encoder (optimized for primary navigation feed)
    nohup ffmpeg -f rawvideo -pix_fmt bgr24 -s 1280x720 -r 30 \
        -i tcp://127.0.0.1:10000 \
        -c:v libx264 -preset superfast -tune zerolatency \
        -profile:v baseline -level 4.0 \
        -crf 18 -g 30 -keyint_min 15 -bf 0 \
        -maxrate 4000k -bufsize 2000k \
        -flags +cgop \
        -f rtsp -rtsp_transport tcp \
        rtsp://localhost:8554/zed_left \
        > "$LOG_DIR/ffmpeg_left.log" 2>&1 &
    
    # Right camera encoder
    nohup ffmpeg -f rawvideo -pix_fmt bgr24 -s 1280x720 -r 30 \
        -i tcp://127.0.0.1:10001 \
        -c:v libx264 -preset superfast -tune zerolatency \
        -profile:v baseline -level 4.0 \
        -crf 18 -g 30 -keyint_min 15 -bf 0 \
        -maxrate 4000k -bufsize 2000k \
        -flags +cgop \
        -f rtsp -rtsp_transport tcp \
        rtsp://localhost:8554/zed_right \
        > "$LOG_DIR/ffmpeg_right.log" 2>&1 &
    
    # Depth encoder (can use slightly lower quality as it's colormap visualization)
    nohup ffmpeg -f rawvideo -pix_fmt bgr24 -s 1280x720 -r 30 \
        -i tcp://127.0.0.1:10002 \
        -c:v libx264 -preset superfast -tune zerolatency \
        -profile:v baseline -level 4.0 \
        -crf 20 -g 30 -keyint_min 15 -bf 0 \
        -maxrate 3000k -bufsize 1500k \
        -flags +cgop \
        -f rtsp -rtsp_transport tcp \
        rtsp://localhost:8554/zed_depth \
        > "$LOG_DIR/ffmpeg_depth.log" 2>&1 &
    
    sleep 5
    log_ok "FFmpeg encoders started with quality optimizations"
}

# =============================================================================
# Stop All Streams
# =============================================================================

stop_all() {
    log_info "Stopping video bridges and encoders gracefully..."
    
    # SAFETY: Stop FFmpeg encoders first (consumers) with SIGTERM for graceful shutdown
    # This prevents broken pipe errors and allows proper stream finalization
    log_info "Sending SIGTERM to FFmpeg encoders..."
    pkill -SIGTERM -f "ffmpeg.*tcp://127.0.0.1:999" 2>/dev/null || true
    pkill -SIGTERM -f "ffmpeg.*tcp://127.0.0.1:1000" 2>/dev/null || true
    
    # Wait for graceful shutdown (max 5 seconds)
    for i in {1..10}; do
        if ! pgrep -f "ffmpeg.*tcp://127.0.0.1" > /dev/null 2>&1; then
            log_ok "FFmpeg encoders stopped gracefully"
            break
        fi
        sleep 0.5
    done
    
    # Force kill any remaining FFmpeg processes
    pkill -SIGKILL -f "ffmpeg.*tcp://127.0.0.1:999" 2>/dev/null || true
    pkill -SIGKILL -f "ffmpeg.*tcp://127.0.0.1:1000" 2>/dev/null || true
    
    # Now stop video bridges (producers) with SIGTERM
    log_info "Sending SIGTERM to video bridges..."
    docker exec nomad_isaac_ros bash -c "pkill -SIGTERM -f 'ros_video_bridge.py'" 2>/dev/null || true
    
    # Wait for graceful shutdown (max 5 seconds)
    for i in {1..10}; do
        if ! docker exec nomad_isaac_ros pgrep -f "ros_video_bridge.py" > /dev/null 2>&1; then
            log_ok "Video bridges stopped gracefully"
            break
        fi
        sleep 0.5
    done
    
    # Force kill any remaining bridge processes
    docker exec nomad_isaac_ros bash -c "pkill -SIGKILL -f 'ros_video_bridge.py'" 2>/dev/null || true
    
    log_ok "All video streams stopped"
}

# =============================================================================
# Show Status
# =============================================================================

show_status() {
    echo ""
    echo "=========================================="
    echo "  NOMAD Video Streams Status"
    echo "=========================================="
    
    # Check MediaMTX streams
    if curl -s http://localhost:9997/v3/paths/list > /dev/null 2>&1; then
        echo ""
        echo "Available Streams:"
        curl -s http://localhost:9997/v3/paths/list | python3 -c "
import sys, json
try:
    data = json.load(sys.stdin)
    for item in data.get('items', []):
        name = item.get('name', '')
        ready = item.get('ready', False)
        tracks = item.get('tracks', [])
        status = '[LIVE]' if ready else '[--]'
        codec = tracks[0] if tracks else 'N/A'
        print(f'{status} {name:20} {codec}')
except Exception as e:
    print(f'Error: {e}')
" 2>/dev/null || echo "Failed to get stream status"
    else
        log_warn "MediaMTX API not accessible"
    fi
    
    echo ""
    echo "FFmpeg Encoders:"
    pgrep -af "ffmpeg.*tcp://127.0.0.1" | head -5 || echo "No encoders running"
    
    echo ""
    echo "=========================================="
}

# =============================================================================
# Main
# =============================================================================

case "${1:-start}" in
    start)
        log_info "Starting multi-stream video system..."
        start_bridges
        start_encoders
        show_status
        log_ok "Multi-stream video started"
        ;;
    stop)
        stop_all
        ;;
    status)
        show_status
        ;;
    restart)
        stop_all
        sleep 3
        start_bridges
        start_encoders
        show_status
        ;;
    *)
        echo "Usage: $0 {start|stop|status|restart}"
        exit 1
        ;;
esac
