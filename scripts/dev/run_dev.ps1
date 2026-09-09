# =============================================================================
# NOMAD Development Runner (Windows PowerShell)
# =============================================================================
# Convenience script to build the NOMAD C++ core for development.
#
# Usage:
#   .\scripts\dev\run_dev.ps1             # Build the C++ core
#   pixi run dev-up                         # Start the optional ArduPilot SITL stack
# =============================================================================

if ($args.Count -ne 0) {
    Write-Error "run_dev.ps1 no longer accepts server arguments; use pixi run dev-up for SITL."
    exit 2
}

# Get script directory and project root
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectRoot = Split-Path -Parent (Split-Path -Parent $ScriptDir)

Write-Host "=============================================" -ForegroundColor Blue
Write-Host "  NOMAD C++ Core - Development Build" -ForegroundColor Blue
Write-Host "=============================================" -ForegroundColor Blue
Write-Host ""

# Change to project directory
Push-Location $ProjectRoot

try {
    Write-Host ""
    Write-Host "Building NOMAD C++ core..." -ForegroundColor Blue
    & pixi run build-core
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
    Write-Host "Core build complete. Start ArduPilot SITL with: pixi run dev-up" -ForegroundColor Green

} finally {
    Pop-Location
}
