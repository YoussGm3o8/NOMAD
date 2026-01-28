#!/bin/bash
# =============================================================================
# NOMAD Multi-Video Stream Launcher
# =============================================================================
# Starts multiple video streams from ZED camera topics to MediaMTX
# Each stream is accessible via RTSP at rtsp://jetson-ip:8554/<stream-name>
# =============================================================================

set -e

CONTAINER_NAME="nomad_isaac_ros"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Define streams: name|topic|tcp_port|width|height|fps
# Main streams for NOMAD
STREAMS=(
    "zed|/zed/zed_node/rgb/image_rect_color|9999|640|360|30"
    "stereo|/zed/zed_node/stereo/image_rect_color|9998|1280|360|30"
    "left|/zed/zed_node/left/image_rect_color|9997|640|360|30"
    "right|/zed/zed_node/right/image_rect_color|9996|640|360|30"
    "gray|/zed/zed_node/left_gray/image_rect_gray|9995|640|360|30"
)

# Optional: Depth requires special handling (16-bit to 8-bit conversion)
# "depth|/zed/zed_node/depth/depth_registered|9994|640|360|15"

start_stream() {
    local name=$1
    local topic=$2
    local tcp_port=$3
    local width=$4
    local height=$5
    local fps=$6
    
    log_info "Starting stream: $name ($topic -> rtsp://localhost:8554/$name)"
    
    # Kill any existing processes for this stream
    pkill -f "ros_video_bridge.py.*$topic" 2>/dev/null || true
    pkill -f "ffmpeg.*tcp://localhost:$tcp_port" 2>/dev/null || true
    sleep 1
    
    # Start video bridge inside container
    docker exec -d "$CONTAINER_NAME" bash -c "
        source /opt/ros/humble/setup.bash
        source /workspaces/isaac_ros-dev/install/setup.bash
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        python3 /tmp/ros_video_bridge.py \
            --topic $topic \
            --stream $name \
            --host localhost \
            --port 8554 \
            --width $width \
            --height $height \
            --fps $fps \
            --tcp-port $tcp_port \
            > /tmp/video_bridge_$name.log 2>&1
    "
    
    # Wait for TCP server to start
    sleep 2
    
    # Determine if this is a grayscale stream
    local pix_fmt_in="bgr24"
    local pix_fmt_out="yuv420p"
    if [[ "$name" == "gray" ]] || [[ "$topic" == *"gray"* ]]; then
        pix_fmt_in="gray"
    fi
    
    # Start FFmpeg encoder on host
    nohup ffmpeg -f rawvideo -pix_fmt $pix_fmt_in -s ${width}x${height} -r $fps \
        -i tcp://localhost:$tcp_port \
        -c:v libx264 -pix_fmt $pix_fmt_out -preset ultrafast -tune zerolatency -b:v 2M -g $fps \
        -f rtsp -rtsp_transport tcp rtsp://localhost:8554/$name \
        > /tmp/ffmpeg_$name.log 2>&1 &
    
    echo $! > /tmp/ffmpeg_$name.pid
    log_info "Stream $name started (PID: $(cat /tmp/ffmpeg_$name.pid))"
}

stop_stream() {
    local name=$1
    log_info "Stopping stream: $name"
    
    # Kill FFmpeg encoder
    if [ -f /tmp/ffmpeg_$name.pid ]; then
        kill $(cat /tmp/ffmpeg_$name.pid) 2>/dev/null || true
        rm -f /tmp/ffmpeg_$name.pid
    fi
    pkill -f "ffmpeg.*rtsp.*$name" 2>/dev/null || true
    
    # Kill video bridge in container
    docker exec "$CONTAINER_NAME" pkill -f "ros_video_bridge.py.*--stream $name" 2>/dev/null || true
}

stop_all() {
    log_info "Stopping all video streams..."
    for stream_def in "${STREAMS[@]}"; do
        IFS='|' read -r name topic port width height fps <<< "$stream_def"
        stop_stream "$name"
    done
    log_info "All streams stopped"
}

start_all() {
    log_info "Starting all video streams..."
    
    # Copy video bridge script to container
    docker cp "$SCRIPT_DIR/../edge_core/ros_video_bridge.py" "$CONTAINER_NAME:/tmp/ros_video_bridge.py"
    
    for stream_def in "${STREAMS[@]}"; do
        IFS='|' read -r name topic port width height fps <<< "$stream_def"
        start_stream "$name" "$topic" "$port" "$width" "$height" "$fps"
        sleep 1
    done
    
    log_info "All streams started"
    show_streams
}

show_streams() {
    echo ""
    echo "=== Available Video Streams ==="
    echo ""
    curl -s http://localhost:9997/v3/paths/list 2>/dev/null | python3 -c "
import sys, json
try:
    data = json.load(sys.stdin)
    for item in data.get('items', []):
        name = item['name']
        ready = item['ready']
        tracks = item.get('tracks', [])
        status = 'LIVE' if ready else '--'
        track_info = ', '.join(tracks) if tracks else 'none'
        print(f'  [{status}] rtsp://100.75.218.89:8554/{name} ({track_info})')
except:
    print('  Error reading stream status')
"
    echo ""
}

status() {
    echo "=== Video Bridge Status ==="
    for stream_def in "${STREAMS[@]}"; do
        IFS='|' read -r name topic port width height fps <<< "$stream_def"
        
        # Check FFmpeg process
        if pgrep -f "ffmpeg.*rtsp.*$name" > /dev/null; then
            echo -e "${GREEN}[RUNNING]${NC} $name (FFmpeg)"
        else
            echo -e "${RED}[STOPPED]${NC} $name (FFmpeg)"
        fi
    done
    
    echo ""
    show_streams
}

# Main entry point
case "${1:-start}" in
    start)
        start_all
        ;;
    stop)
        stop_all
        ;;
    restart)
        stop_all
        sleep 2
        start_all
        ;;
    status)
        status
        ;;
    *)
        echo "Usage: $0 {start|stop|restart|status}"
        echo ""
        echo "Starts multiple ZED camera streams via MediaMTX:"
        for stream_def in "${STREAMS[@]}"; do
            IFS='|' read -r name topic port width height fps <<< "$stream_def"
            echo "  $name: $topic"
        done
        exit 1
        ;;
esac
