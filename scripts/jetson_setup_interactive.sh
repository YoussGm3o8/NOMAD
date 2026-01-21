#!/bin/bash
# ============================================================
# NOMAD Jetson Setup Script (Interactive)
# ============================================================
# Run this directly on the Jetson Orin Nano
#
# Usage (from Windows - copy and paste):
#   ssh mad@100.75.218.89
#   # Then paste these commands line by line:
# ============================================================

# Configuration (YOUR TAILSCALE IPs)
GCS_IP="100.76.127.17"           # Your Windows Ground Station
JETSON_IP="100.75.218.89"        # Jetson Tailscale IP

echo "=========================================="
echo "  NOMAD Jetson Setup"
echo "=========================================="
echo "Ground Station IP: $GCS_IP"
echo "Jetson IP: $JETSON_IP"
echo ""

# 1. Update system
echo "[1/6] Updating system packages..."
sudo apt-get update

# 2. Install mavlink-router
echo "[2/6] Installing mavlink-router..."
sudo apt-get install -y mavlink-router

# 3. Clone NOMAD
echo "[3/6] Setting up NOMAD repository..."
if [ ! -d "$HOME/NOMAD" ]; then
    git clone https://github.com/YoussGm3o8/NOMAD.git "$HOME/NOMAD"
else
    cd "$HOME/NOMAD" && git pull
fi

# 4. Configure MAVLink Router
echo "[4/6] Configuring MAVLink Router..."
sudo mkdir -p /etc/mavlink-router
sudo tee /etc/mavlink-router/main.conf > /dev/null << EOF
[General]
TcpServerPort=5760
ReportStats=true
MavlinkDialect=ardupilotmega

[UartEndpoint alpha]
Device=/dev/ttyTHS1
Baud=921600

[UdpEndpoint orchestrator]
Mode=Normal
Address=127.0.0.1
Port=14550

[UdpEndpoint groundstation]
Mode=Normal
Address=${GCS_IP}
Port=14550
EOF

sudo systemctl enable mavlink-router
sudo systemctl restart mavlink-router
echo "MAVLink Router configured to send to $GCS_IP:14550"

# 5. Create .env file
echo "[5/6] Creating environment file..."
cat > "$HOME/NOMAD/.env" << EOF
# NOMAD Edge Core Environment
NOMAD_HOST=0.0.0.0
NOMAD_PORT=8000
TAILSCALE_IP=${JETSON_IP}
GCS_IP=${GCS_IP}
GCS_PORT=14550
MAVLINK_UART_DEV=/dev/ttyTHS1
MAVLINK_UART_BAUD=921600
NOMAD_ENABLE_VISION=true
NOMAD_ENABLE_ISAAC_ROS=false
NOMAD_DEBUG=false
EOF

# 6. Install Python dependencies
echo "[6/6] Installing Python dependencies..."
cd "$HOME/NOMAD"
pip3 install --user -r edge_core/requirements.txt 2>/dev/null || echo "Some deps may need manual install"

echo ""
echo "=========================================="
echo "  Setup Complete!"
echo "=========================================="
echo ""
echo "To start NOMAD Edge Core:"
echo "  cd ~/NOMAD"
echo "  python3 -m edge_core.main"
echo ""
echo "To test from your Windows machine:"
echo "  curl http://${JETSON_IP}:8000/health"
