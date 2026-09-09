#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# =============================================================================
# NOMAD Development Runner (Linux/macOS)
# =============================================================================
# Convenience script to build the NOMAD C++ core for development.
#
# Usage:
#   ./scripts/dev/run_dev.sh           # Build the C++ core
#   pixi run dev-up                    # Start the optional ArduPilot SITL stack
# =============================================================================

set -e

if [ "$#" -ne 0 ]; then
    echo "run_dev.sh no longer accepts server arguments; use pixi run dev-up for SITL." >&2
    exit 2
fi

# Get script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=============================================${NC}"
echo -e "${BLUE}  NOMAD C++ Core - Development Build${NC}"
echo -e "${BLUE}=============================================${NC}"
echo ""

# Change to project directory
cd "$PROJECT_ROOT"

echo ""
echo -e "${BLUE}Building NOMAD C++ core...${NC}"
echo ""

pixi run build-core
echo -e "${GREEN}Core build complete. Start ArduPilot SITL with: pixi run dev-up${NC}"
