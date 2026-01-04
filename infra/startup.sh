#!/bin/bash
# ============================================================
# NOMAD System Startup Script
# ============================================================
# Target: NVIDIA Jetson Orin Nano
# Competition: AEAC 2026
#
# This script starts all NOMAD system components:
# 1. MAVLink Router (connects to Cube Orange)
# 2. Edge Core Orchestrator (main application)
#
# Usage:
#   ./startup.sh [--no-mavlink] [--no-venv]
#
# Options:
#   --no-mavlink  Skip MAVLink Router startup
#   --no-venv     Skip virtual environment activation
#   --debug       Enable debug logging
# ============================================================

set -e

# ============================================================
# Configuration
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
MAVLINK_CONF="$PROJECT_ROOT/transport/mavlink_router/mavlink-router.conf"
VENV_PATH="$PROJECT_ROOT/.venv"
LOG_DIR="$PROJECT_ROOT/logs"
PID_FILE="$PROJECT_ROOT/.nomad.pid"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Process tracking
MAVLINK_PID=""
ORCHESTRATOR_PID=""

# ============================================================
# Argument Parsing
# ============================================================

SKIP_MAVLINK=false
SKIP_VENV=false
DEBUG=false

for arg in "$@"; do
    case $arg in
        --no-mavlink)
            SKIP_MAVLINK=true
            shift
            ;;
        --no-venv)
            SKIP_VENV=true
            shift
            ;;
        --debug)
            DEBUG=true
            shift
            ;;
        --help)
            echo "Usage: $0 [--no-mavlink] [--no-venv] [--debug]"
            echo ""
            echo "Options:"
            echo "  --no-mavlink  Skip MAVLink Router startup"
            echo "  --no-venv     Skip virtual environment activation"
            echo "  --debug       Enable debug logging"
            exit 0
            ;;
    esac
done

# ============================================================
# Utility Functions
# ============================================================

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[OK]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

check_command() {
    if command -v "$1" &> /dev/null; then
        return 0
    else
        return 1
    fi
}

# ============================================================
# Cleanup Handler
# ============================================================

cleanup() {
    log_info "Shutting down NOMAD system..."
    
    # Kill Orchestrator
    if [ -n "$ORCHESTRATOR_PID" ] && kill -0 "$ORCHESTRATOR_PID" 2>/dev/null; then
        log_info "Stopping Orchestrator (PID: $ORCHESTRATOR_PID)..."
        kill -TERM "$ORCHESTRATOR_PID" 2>/dev/null || true
        wait "$ORCHESTRATOR_PID" 2>/dev/null || true
    fi
    
    # Kill MAVLink Router
    if [ -n "$MAVLINK_PID" ] && kill -0 "$MAVLINK_PID" 2>/dev/null; then
        log_info "Stopping MAVLink Router (PID: $MAVLINK_PID)..."
        kill -TERM "$MAVLINK_PID" 2>/dev/null || true
        wait "$MAVLINK_PID" 2>/dev/null || true
    fi
    
    # Remove PID file
    rm -f "$PID_FILE"
    
    log_success "NOMAD system stopped"
    exit 0
}

# Register cleanup handler
trap cleanup SIGINT SIGTERM EXIT

# ============================================================
# Main Startup Sequence
# ============================================================

echo ""
echo "============================================================"
echo "  NOMAD System Startup"
echo "  AEAC 2026 - McGill Aerial Design"
echo "============================================================"
echo ""

# Create log directory
mkdir -p "$LOG_DIR"

# ------------------------------------------------------------
# Step 1: Check MAVLink Router
# ------------------------------------------------------------

if [ "$SKIP_MAVLINK" = false ]; then
    log_info "Checking MAVLink Router..."
    
    if check_command mavlink-routerd; then
        MAVLINK_VERSION=$(mavlink-routerd --version 2>&1 | head -n1 || echo "unknown")
        log_success "MAVLink Router found: $MAVLINK_VERSION"
    else
        log_warning "mavlink-routerd not found!"
        log_warning "Install with: sudo apt install mavlink-router"
        log_warning "Or build from: https://github.com/mavlink-router/mavlink-router"
        log_warning ""
        log_warning "Continuing without MAVLink Router..."
        SKIP_MAVLINK=true
    fi
fi

# ------------------------------------------------------------
# Step 2: Start MAVLink Router
# ------------------------------------------------------------

if [ "$SKIP_MAVLINK" = false ]; then
    if [ -f "$MAVLINK_CONF" ]; then
        log_info "Starting MAVLink Router..."
        
        # Start in background, redirect output to log file
        mavlink-routerd -c "$MAVLINK_CONF" >> "$LOG_DIR/mavlink-router.log" 2>&1 &
        MAVLINK_PID=$!
        
        # Wait briefly and check if it started
        sleep 1
        if kill -0 "$MAVLINK_PID" 2>/dev/null; then
            log_success "MAVLink Router started (PID: $MAVLINK_PID)"
        else
            log_error "MAVLink Router failed to start. Check $LOG_DIR/mavlink-router.log"
            MAVLINK_PID=""
        fi
    else
        log_warning "MAVLink config not found: $MAVLINK_CONF"
        log_warning "Skipping MAVLink Router..."
    fi
fi

# ------------------------------------------------------------
# Step 3: Activate Virtual Environment
# ------------------------------------------------------------

if [ "$SKIP_VENV" = false ]; then
    if [ -f "$VENV_PATH/bin/activate" ]; then
        log_info "Activating virtual environment..."
        source "$VENV_PATH/bin/activate"
        log_success "Virtual environment activated: $VIRTUAL_ENV"
    else
        log_warning "Virtual environment not found: $VENV_PATH"
        log_warning "Create with: python3 -m venv .venv && source .venv/bin/activate && pip install -r edge_core/requirements.txt"
        log_warning "Continuing with system Python..."
    fi
fi

# Check Python
PYTHON_VERSION=$(python3 --version 2>&1 || echo "not found")
log_info "Python: $PYTHON_VERSION"

# ------------------------------------------------------------
# Step 4: Start Edge Core Orchestrator
# ------------------------------------------------------------

log_info "Starting Edge Core Orchestrator..."

cd "$PROJECT_ROOT"

# Build command
ORCHESTRATOR_CMD="python3 -m edge_core.main"
if [ "$DEBUG" = true ]; then
    export NOMAD_DEBUG=1
fi

# Start Orchestrator
$ORCHESTRATOR_CMD &
ORCHESTRATOR_PID=$!

# Wait briefly and check if it started
sleep 2
if kill -0 "$ORCHESTRATOR_PID" 2>/dev/null; then
    log_success "Orchestrator started (PID: $ORCHESTRATOR_PID)"
else
    log_error "Orchestrator failed to start"
    exit 1
fi

# Save PIDs
echo "MAVLINK_PID=$MAVLINK_PID" > "$PID_FILE"
echo "ORCHESTRATOR_PID=$ORCHESTRATOR_PID" >> "$PID_FILE"

# ------------------------------------------------------------
# Step 5: Display Status
# ------------------------------------------------------------

echo ""
echo "============================================================"
echo "  NOMAD System Running"
echo "============================================================"
echo ""
log_success "Dashboard: http://localhost:8000/"
log_success "API Docs:  http://localhost:8000/docs"
if [ -n "$MAVLINK_PID" ]; then
    log_success "MAVLink:   127.0.0.1:14550 (Orchestrator)"
    log_success "           127.0.0.1:14551 (Vision)"
fi
echo ""
log_info "Press Ctrl+C to stop all services"
echo ""

# ------------------------------------------------------------
# Step 6: Wait for Exit
# ------------------------------------------------------------

# Wait for Orchestrator process (main process)
wait "$ORCHESTRATOR_PID"
