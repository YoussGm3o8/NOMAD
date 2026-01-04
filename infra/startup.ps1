<# 
.SYNOPSIS
    NOMAD System Startup Script (Windows)
    
.DESCRIPTION
    Starts all NOMAD system components for development on Windows.
    Note: MAVLink Router is typically Linux-only (Jetson deployment).
    
.PARAMETER NoVenv
    Skip virtual environment activation
    
.PARAMETER Debug
    Enable debug logging
    
.PARAMETER NoVision
    Skip Vision process startup
    
.EXAMPLE
    .\startup.ps1
    .\startup.ps1 -Debug
    .\startup.ps1 -NoVenv
#>

param(
    [switch]$NoVenv,
    [switch]$Debug,
    [switch]$NoVision
)

# ============================================================
# Configuration
# ============================================================

$ErrorActionPreference = "Stop"
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectRoot = Split-Path -Parent $ScriptDir
$VenvPath = Join-Path $ProjectRoot ".venv"
$LogDir = Join-Path $ProjectRoot "logs"

# ============================================================
# Functions
# ============================================================

function Write-Info { param($msg) Write-Host "[INFO] $msg" -ForegroundColor Cyan }
function Write-Success { param($msg) Write-Host "[OK] $msg" -ForegroundColor Green }
function Write-Warning { param($msg) Write-Host "[WARN] $msg" -ForegroundColor Yellow }
function Write-Error { param($msg) Write-Host "[ERROR] $msg" -ForegroundColor Red }

# ============================================================
# Main
# ============================================================

Write-Host ""
Write-Host "============================================================" -ForegroundColor Blue
Write-Host "  NOMAD System Startup (Windows Development)" -ForegroundColor Blue
Write-Host "  AEAC 2026 - McGill Aerial Design" -ForegroundColor Blue
Write-Host "============================================================" -ForegroundColor Blue
Write-Host ""

# Create log directory
if (-not (Test-Path $LogDir)) {
    New-Item -ItemType Directory -Path $LogDir | Out-Null
}

# ------------------------------------------------------------
# Step 1: Check for MAVLink Router (Linux only)
# ------------------------------------------------------------

Write-Warning "MAVLink Router is Linux-only (for Jetson deployment)"
Write-Warning "For Windows development, use SITL or direct serial connection"
Write-Host ""

# ------------------------------------------------------------
# Step 2: Activate Virtual Environment
# ------------------------------------------------------------

if (-not $NoVenv) {
    $VenvActivate = Join-Path $VenvPath "Scripts\Activate.ps1"
    
    if (Test-Path $VenvActivate) {
        Write-Info "Activating virtual environment..."
        & $VenvActivate
        Write-Success "Virtual environment activated"
    }
    else {
        Write-Warning "Virtual environment not found: $VenvPath"
        Write-Warning "Create with: python -m venv .venv; .\.venv\Scripts\Activate.ps1; pip install -r edge_core\requirements.txt"
    }
}

# Check Python
$pythonVersion = & python --version 2>&1
Write-Info "Python: $pythonVersion"

# ------------------------------------------------------------
# Step 3: Start Edge Core Orchestrator
# ------------------------------------------------------------

Write-Info "Starting Edge Core Orchestrator..."

Set-Location $ProjectRoot

# Set environment variables
if ($Debug) {
    $env:NOMAD_DEBUG = "1"
}

# Determine vision flag
$visionFlag = if ($NoVision) { "--no-vision" } else { "" }

try {
    # Start the orchestrator
    Write-Host ""
    Write-Host "============================================================" -ForegroundColor Blue
    Write-Host "  NOMAD System Running" -ForegroundColor Blue
    Write-Host "============================================================" -ForegroundColor Blue
    Write-Host ""
    Write-Success "Dashboard: http://localhost:8000/"
    Write-Success "API Docs:  http://localhost:8000/docs"
    Write-Host ""
    Write-Info "Press Ctrl+C to stop"
    Write-Host ""
    
    # Run the main module
    python -m edge_core.main
}
catch {
    Write-Error "Failed to start Orchestrator: $_"
    exit 1
}
finally {
    Write-Info "NOMAD system stopped"
}
