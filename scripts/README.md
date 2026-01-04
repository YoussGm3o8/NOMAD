# Scripts

Utility scripts for development and operations.

## Development Scripts

### `run_dev.ps1` (Windows)
Run NOMAD Edge Core in simulation mode for local development without hardware.

```powershell
# Default - runs on port 8000
.\scripts\run_dev.ps1

# Custom port
.\scripts\run_dev.ps1 -Port 8080

# Disable vision for API-only testing
.\scripts\run_dev.ps1 -NoVision
```

### `run_dev.sh` (Linux/macOS)
Same as above for Unix systems.

```bash
./scripts/run_dev.sh
./scripts/run_dev.sh --port 8080
./scripts/run_dev.sh --no-vision
```

## Environment
- Sets `NOMAD_SIM_MODE=true` to enable mock hardware
- API available at `http://localhost:8000`
- API docs at `http://localhost:8000/docs`
