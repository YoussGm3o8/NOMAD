#!/bin/bash
# Install MediaMTX RTSP Server on Jetson Orin Nano
# This enables multiple clients to view the same video stream

set -e

MEDIAMTX_VERSION="1.10.0"
ARCH="arm64v8"  # For Jetson (aarch64)
INSTALL_DIR="/opt/mediamtx"
CONFIG_DIR="/etc/mediamtx"

echo "=========================================="
echo "  Installing MediaMTX RTSP Server"
echo "=========================================="

# Create directories
sudo mkdir -p $INSTALL_DIR
sudo mkdir -p $CONFIG_DIR

# Download MediaMTX
cd /tmp
DOWNLOAD_URL="https://github.com/bluenviron/mediamtx/releases/download/v${MEDIAMTX_VERSION}/mediamtx_v${MEDIAMTX_VERSION}_linux_${ARCH}.tar.gz"
echo "Downloading from: $DOWNLOAD_URL"
wget -q "$DOWNLOAD_URL" -O mediamtx.tar.gz

# Extract
tar -xzf mediamtx.tar.gz
sudo mv mediamtx $INSTALL_DIR/
rm mediamtx.tar.gz mediamtx.yml 2>/dev/null || true

# Copy config
if [ -f ~/NOMAD/infra/mediamtx.yml ]; then
    sudo cp ~/NOMAD/infra/mediamtx.yml $CONFIG_DIR/mediamtx.yml
    echo "Copied NOMAD mediamtx.yml config"
else
    # Create default config
    sudo tee $CONFIG_DIR/mediamtx.yml > /dev/null << 'EOF'
logLevel: info
api: yes
apiAddress: :9997

# RTSP server for multiple clients
rtsp: yes
rtspAddress: :8554
protocols: [tcp, udp]

# Disable unused protocols
rtmp: no
hls: no
webrtc: no
srt: no

paths:
  # ZED camera stream
  zed:
    source: publisher
    sourceOnDemand: no
  
  # Fallback for any stream
  all:
    source: publisher
EOF
    echo "Created default config"
fi

# Create systemd service
sudo tee /etc/systemd/system/mediamtx.service > /dev/null << 'EOF'
[Unit]
Description=MediaMTX RTSP Server
After=network.target

[Service]
Type=simple
ExecStart=/opt/mediamtx/mediamtx /etc/mediamtx/mediamtx.yml
Restart=always
RestartSec=5
User=root

[Install]
WantedBy=multi-user.target
EOF

# Enable and start service
sudo systemctl daemon-reload
sudo systemctl enable mediamtx
sudo systemctl start mediamtx

echo ""
echo "=========================================="
echo "  MediaMTX Installed Successfully"
echo "=========================================="
echo ""
echo "RTSP Server: rtsp://$(hostname -I | awk '{print $1}'):8554/zed"
echo ""
echo "To publish a stream:"
echo "  gst-launch-1.0 ... ! rtspclientsink location=rtsp://localhost:8554/zed"
echo ""
echo "To view (VLC):"
echo "  vlc rtsp://100.75.218.89:8554/zed"
echo ""
echo "Multiple clients can connect to the same RTSP URL!"
echo ""
