# NOMAD Codebase Analysis Report

**Generated:** 2026-01-23  
**Author:** Automated Analysis

---

## Executive Summary

This document details the consistency and integrity analysis of the NOMAD codebase. The analysis identified several categories of issues that need attention.

---

## Issues Found

### 1. Missing Referenced Files/Directories

| File/Directory | Referenced In | Status |
|----------------|---------------|--------|
| `web_client/` | `infra/Dockerfile` (line 99) | ❌ **Missing** - Directory does not exist |
| `config/landmarks.json` | `config/env/jetson.env.example` | ❌ **Missing** - File does not exist |
| `config/env/jetson.env` | `scripts/setup_jetson.sh` (line 130) | ⚠️ Only `.example` exists |

### 2. Inconsistent User Paths

Multiple home directories are referenced across the codebase:

| Path | Files Using It |
|------|----------------|
| `/home/nomad/NOMAD` | `scripts/setup_jetson.sh`, `config/env/jetson.env.example`, `infra/nomad.service`, `tailscale/SETUP.md` |
| `/home/mad/NOMAD` | `scripts/start_nomad_full.sh`, `docs/JETSON_DEPLOYMENT.md` |
| `/home/ubuntu/NOMAD` | `scripts/setup_jetson_remote.py` |

**Recommendation:** Standardize on a single user (`mad` appears to be the actual deployed user).

### 3. CLI Argument Mismatches

The `edge_core/main.py` CLI parser does NOT support the following arguments that are used in scripts:

| Unsupported Argument | Used In |
|---------------------|---------|
| `--sim` | `scripts/run_dev.sh`, `scripts/run_dev.ps1`, `start_server.py` |
| `--no-vision` | `scripts/run_dev.ps1`, `start_server.py` |
| `--no-task2` | `scripts/run_dev.ps1` |
| `--servo-mode` | `scripts/run_dev.ps1` |

**Current `main.py` supported arguments:**
- `--host`
- `--port`
- `--log-level`

### 4. Security Issues

| Issue | Location | Severity |
|-------|----------|----------|
| Hardcoded password `skibidi123` | `scripts/setup_jetson_remote.py`, `docs/JETSON_DEPLOYMENT.md` | 🔴 **HIGH** |
| Invalid documentation URL | `tailscale/config/tailscale-watchdog.service` (`github.com/bob`) | ⚠️ Low |

### 5. Duplicate Configuration Files

MAVLink router has two configuration files:
- `transport/mavlink_router/main.conf`
- `transport/mavlink_router/mavlink-router.conf`

Both files are largely similar but have minor differences. One should be removed or clearly documented as primary.

### 6. Windows-Specific Hardcoded Path

The `start_server.py` file contains a hardcoded Windows path:
```python
os.chdir(r'c:\Users\Youssef\Documents\Code\MAD\NOMAD')
```

This makes the file non-portable.

---

## Files Not Used (Potentially Orphaned)

| File | Analysis |
|------|----------|
| `zed_stream.sdp` | SDP file for video streaming - used for VLC configuration, valid |
| `start_server.py` | Development helper, but uses non-existent CLI args |
| `config/profiles/task2_indoor.params` | Placeholder file with only TODO comment |

---

## Dependency Analysis

### Requirements Files

1. **`edge_core/requirements.txt`** - Generic requirements with version ranges
2. **`edge_core/requirements-jetson.txt`** - Pinned versions for Jetson deployment

Both files are consistent in purpose but use different versioning strategies. This is intentional (development vs. production).

### Missing pyproject.toml

No `pyproject.toml` was found. Modern Python packaging recommends using this format.

---

## Recommended Actions

### High Priority

1. **Add missing CLI arguments to `edge_core/main.py`:**
   - `--sim` for simulation mode
   - `--no-vision` to disable vision process
   
2. **Create `config/landmarks.json`** with sample landmark data

3. **Standardize user paths** to `/home/mad/NOMAD` (matching actual deployment)

4. **Remove hardcoded password** from documentation and scripts

### Medium Priority

5. **Create `web_client/` directory** or remove reference from Dockerfile

6. **Fix `start_server.py`** to be cross-platform

7. **Consolidate MAVLink router configs** - keep one as primary

8. **Fix documentation URL** in `tailscale-watchdog.service`

### Low Priority

9. **Populate `config/profiles/task2_indoor.params`** with actual parameters

10. **Add `pyproject.toml`** for modern Python packaging

---

## File Usage Summary

### Active Files (Verified Used)

- `edge_core/*.py` - Core Python modules
- `mission_planner/src/*.cs` - Mission Planner plugin
- `config/params/*.param` - ArduPilot parameter files
- `transport/mavlink_router/main.conf` - MAVLink router config
- `infra/mediamtx.yml` - Media server config
- `infra/nomad.service` - Systemd service
- `tailscale/` - Tailscale VPN integration
- `scripts/` - Deployment and development scripts

### Documentation Files

- `README.md` - Main repository documentation
- `PRD.md` - Product Requirements Document
- `docs/JETSON_DEPLOYMENT.md` - Deployment guide

### Test Files

- `tests/test_isaac_bridge.py` - Isaac ROS bridge tests

---

## Conclusion

The codebase is generally well-organized but has several consistency issues primarily around:
1. Path inconsistencies across deployment environments
2. CLI argument mismatches between scripts and actual implementation
3. Missing referenced files

These issues don't affect core functionality but should be addressed for maintainability.
