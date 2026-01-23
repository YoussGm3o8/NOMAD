# ============================================================
# NOMAD Edge Core Dockerfile
# 
# Base image: Python 3.13-slim for Jetson Orin Nano
# Includes: OpenCV, ZED SDK (runtime), MAVLink, FastAPI
#
# Build: docker build -t nomad-edge-core -f infra/Dockerfile .
# ============================================================

# Build stage for dependencies
FROM python:3.13-slim AS builder

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc \
    g++ \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

# Create virtual environment
RUN python -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# Copy and install Python dependencies
COPY edge_core/requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir -r /tmp/requirements.txt


# ============================================================
# Runtime stage
# ============================================================
FROM python:3.13-slim

# Labels
LABEL maintainer="McGill Aerial Design <mad@mcgill.ca>"
LABEL description="NOMAD Edge Core - Autonomous drone control system"
LABEL version="1.0.0"

# Environment variables
ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PYTHONFAULTHANDLER=1 \
    PATH="/opt/venv/bin:$PATH" \
    # NOMAD specific
    NOMAD_CONFIG_PATH=/app/config \
    NOMAD_DATA_PATH=/mnt/nvme \
    NOMAD_LOG_LEVEL=INFO

# Install runtime system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    # OpenCV dependencies
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender1 \
    libgomp1 \
    # GStreamer for RTSP streaming
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-rtsp \
    libgstreamer1.0-0 \
    libgstreamer-plugins-base1.0-0 \
    # FFmpeg for video encoding
    ffmpeg \
    # Networking tools
    iproute2 \
    iputils-ping \
    net-tools \
    curl \
    # Process management
    procps \
    # USB/Serial access
    udev \
    && rm -rf /var/lib/apt/lists/*

# Create app user (non-root)
RUN groupadd --gid 1000 nomad && \
    useradd --uid 1000 --gid nomad --shell /bin/bash --create-home nomad

# Create directories
RUN mkdir -p /app /mnt/nvme/evidence /mnt/nvme/logs && \
    chown -R nomad:nomad /app /mnt/nvme

# Copy virtual environment from builder
COPY --from=builder /opt/venv /opt/venv

# Set working directory
WORKDIR /app

# Copy application code
COPY --chown=nomad:nomad edge_core/ /app/edge_core/
COPY --chown=nomad:nomad config/ /app/config/
# Note: web_client is optional - uncomment when frontend is implemented
# COPY --chown=nomad:nomad web_client/ /app/web_client/

# Create __init__.py for package if not exists
RUN touch /app/__init__.py

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD curl -f http://localhost:8000/health || exit 1

# Expose ports
# 8000 - FastAPI REST API
# 5555-5560 - ZMQ endpoints
EXPOSE 8000 5555 5556 5557 5558 5559 5560

# Default command
# Note: Running as root is required for /dev access (MAVLink, ZED)
# In production, use --privileged or specific device mounts
USER root
CMD ["python", "-m", "edge_core.main"]


# ============================================================
# Jetson-specific build (alternative)
# ============================================================
# For Jetson Orin Nano with ZED SDK, use:
# FROM dustynv/l4t-pytorch:r35.4.1-py3 AS jetson-base
# 
# This includes:
# - JetPack 5.1.2 / L4T R35.4.1
# - CUDA 11.4
# - PyTorch 2.0
# - OpenCV with CUDA
# 
# Build with: docker build --target jetson -f infra/Dockerfile .
