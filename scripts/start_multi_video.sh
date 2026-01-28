#!/bin/bash
# =============================================================================
# NOMAD Dynamic Multi-Stream Video Manager
# =============================================================================
# Uses the VideoStreamManager API to dynamically start/stop video streams.
# No more hardcoded processes - everything is managed via the Edge Core API.
#
# Usage: ./start_multi_video.sh [start|stop|status|add|remove]
#
# Examples:
#   ./start_multi_video.sh start              # Start default streams
#   ./start_multi_video.sh add my_stream /topic  # Add custom stream
#   ./start_multi_video.sh remove zed_left    # Remove specific stream
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NOMAD_DIR="$(dirname "$SCRIPT_DIR")"
API_URL="http://localhost:8000"

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

# =============================================================================
# API Calls
# =============================================================================

list_streams() {
    curl -s "${API_URL}/api/video/streams" | python3 -m json.tool 2>/dev/null
}

start_stream() {
    local name=$1
    local topic=$2
    local width=${3:-1280}
    local height=${4:-720}
    local fps=${5:-30}
    
    log_info "Starting stream: $name ($topic)"
    
    curl -s -X POST "${API_URL}/api/video/streams" \
        -H "Content-Type: application/json" \
        -d "{
            \"stream_name\": \"$name\",
            \"topic\": \"$topic\",
            \"width\": $width,
            \"height\": $height,
            \"fps\": $fps
        }" | python3 -m json.tool 2>/dev/null
}

stop_stream() {
    local name=$1
    log_info "Stopping stream: $name"
    curl -s -X DELETE "${API_URL}/api/video/streams/$name" | python3 -m json.tool 2>/dev/null
}

stop_all() {
    log_info "Stopping all streams..."
    curl -s -X DELETE "${API_URL}/api/video/streams" | python3 -m json.tool 2>/dev/null
}

# =============================================================================
# Default Stream Configurations
# =============================================================================

start_default_streams() {
    log_info "Starting default ZED camera streams..."
    
    # Stereo stream (2560x720 side-by-side)
    start_stream "zed_stereo" "/zed/zed_node/stereo/image_rect_color" 2560 720 30
    sleep 2
    
    # Left camera (primary navigation feed)
    start_stream "zed_left" "/zed/zed_node/left/image_rect_color" 1280 720 30
    sleep 2
    
    # Right camera
    start_stream "zed_right" "/zed/zed_node/right/image_rect_color" 1280 720 30
    sleep 2
    
    # Depth visualization
    start_stream "zed_depth" "/zed/zed_node/depth/depth_registered" 1280 720 30
    sleep 2
    
    log_ok "Default streams started"
}

# =============================================================================
# Status Display
# =============================================================================

show_status() {
    echo ""
    echo "=========================================="
    echo "  NOMAD Video Streams (Dynamic API)"
    echo "=========================================="
    
    # Check if Edge Core is running
    if ! curl -s "${API_URL}/health" > /dev/null 2>&1; then
        log_fail "Edge Core API not accessible at ${API_URL}"
        return 1
    fi
    
    echo ""
    echo "Active Streams:"
    
    streams=$(curl -s "${API_URL}/api/video/streams" 2>/dev/null)
    
    if [ $? -eq 0 ]; then
        echo "$streams" | python3 -c "
import sys, json
try:
    data = json.load(sys.stdin)
    streams = data.get('streams', [])
    if not streams:
        print('  No active streams')
    else:
        for s in streams:
            name = s['stream_name']
            topic = s['topic']
            url = s['rtsp_url']
            res = f\"{s['width']}x{s['height']}@{s['fps']}fps\"
            print(f\"  • {name:15} {topic:40} {res} -> {url}\")
except Exception as e:
    print(f'  Error: {e}')
" || log_fail "Failed to parse streams"
    else
        log_fail "Failed to fetch streams"
    fi
    
    echo ""
    echo "Available Topics:"
    topics=$(curl -s "${API_URL}/api/video/topics" 2>/dev/null)
    
    if [ $? -eq 0 ]; then
        echo "$topics" | python3 -c "
import sys, json
try:
    data = json.load(sys.stdin)
    topics = data.get('topics', [])
    if not topics:
        print('  No ROS topics available (is ZED running?)')
    else:
        for t in topics[:10]:  # Show first 10
            print(f'  • {t}')
        if len(topics) > 10:
            print(f'  ... and {len(topics) - 10} more')
except Exception as e:
    print(f'  Error: {e}')
" || log_fail "Failed to parse topics"
    fi
    
    echo ""
    echo "=========================================="
}

# =============================================================================
# Main
# =============================================================================

case "${1:-status}" in
    start)
        start_default_streams
        sleep 3
        show_status
        ;;
    stop)
        stop_all
        log_ok "All streams stopped"
        ;;
    status)
        show_status
        ;;
    add)
        if [ -z "$2" ] || [ -z "$3" ]; then
            log_fail "Usage: $0 add <stream_name> <topic> [width] [height] [fps]"
            exit 1
        fi
        start_stream "$2" "$3" "${4:-1280}" "${5:-720}" "${6:-30}"
        ;;
    remove)
        if [ -z "$2" ]; then
            log_fail "Usage: $0 remove <stream_name>"
            exit 1
        fi
        stop_stream "$2"
        ;;
    restart)
        stop_all
        sleep 2
        start_default_streams
        sleep 3
        show_status
        ;;
    *)
        echo "Usage: $0 {start|stop|status|add|remove|restart}"
        echo ""
        echo "Commands:"
        echo "  start          - Start default ZED camera streams"
        echo "  stop           - Stop all active streams"
        echo "  status         - Show active streams and available topics"
        echo "  add <name> <topic> [w] [h] [fps] - Add a custom stream"
        echo "  remove <name>  - Remove a specific stream"
        echo "  restart        - Stop all and restart defaults"
        echo ""
        echo "Examples:"
        echo "  $0 status"
        echo "  $0 add my_cam /camera/rgb/image_raw"
        echo "  $0 remove zed_depth"
        exit 1
        ;;
esac
