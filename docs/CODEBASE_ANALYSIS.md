# NOMAD Codebase Analysis Report

**Generated:** 2026-01-23  
**Last Updated:** 2026-01-23 (Issues Fixed)
**Author:** Automated Analysis

---

## Executive Summary

This document details the consistency and integrity analysis of the NOMAD codebase. Most issues have been resolved.

---

## Issues Fixed ✅

### 1. Missing Referenced Files/Directories - RESOLVED

| File/Directory | Status |
|----------------|--------|
| `web_client/` | ✅ **Fixed** - Dockerfile reference already commented out with note |
| `config/landmarks.json` | ✅ **Fixed** - File created with sample competition waypoints |
| `config/env/jetson.env` | ✅ **Fixed** - File exists and populated |

### 2. Inconsistent User Paths - RESOLVED

All paths standardized to `/home/mad/NOMAD`:

| File | Status |
|------|--------|
| `scripts/setup_jetson.sh` | ✅ Updated |
| `config/env/jetson.env` | ✅ Updated |
| `config/env/jetson.env.example` | ✅ Updated |
| `infra/nomad.service` | ✅ Updated |
| `scripts/setup_jetson_remote.py` | ✅ Updated |

### 3. CLI Argument Mismatches - ALREADY RESOLVED

The `edge_core/main.py` CLI parser DOES support all required arguments:
- `--host` ✅
- `--port` ✅
- `--log-level` ✅
- `--sim` ✅
- `--no-vision` ✅
- `--no-task2` ✅
- `--servo-mode` ✅

### 4. Security Issues - RESOLVED

| Issue | Status |
|-------|--------|
| Hardcoded password `skibidi123` | ✅ **Fixed** - Removed, using environment variables |
| Invalid documentation URL | ✅ **Fixed** - Updated to correct GitHub URL |

### 5. Duplicate Configuration Files - RESOLVED

MAVLink router consolidated:
- ✅ `transport/mavlink_router/main.conf` - **Primary config (kept)**
- ✅ `transport/mavlink_router/mavlink-router.conf` - **Deleted**
- ✅ All references updated to use `main.conf`

### 6. Windows-Specific Hardcoded Path - ALREADY RESOLVED

The `start_server.py` file now uses cross-platform path detection:
```python
project_root = Path(__file__).parent.absolute()
os.chdir(project_root)
```

---

## Files Not Used (Potentially Orphaned)

| File | Analysis |
|------|----------|
| `zed_stream.sdp` | SDP file for video streaming - used for VLC configuration, valid |
| `start_server.py` | Development helper - working correctly |
| `config/profiles/task2_indoor.params` | ✅ Fully populated with VIO/indoor parameters |

---

## Dependency Analysis

### Requirements Files

1. **`edge_core/requirements.txt`** - Generic requirements with version ranges
2. **`edge_core/requirements-jetson.txt`** - Pinned versions for Jetson deployment

Both files are consistent in purpose but use different versioning strategies. This is intentional (development vs. production).

### Missing pyproject.toml

No `pyproject.toml` was found. Consider adding for modern Python packaging (low priority).

---

## All Actions Completed ✅

### High Priority - DONE
1. ✅ CLI arguments already supported in `edge_core/main.py`
2. ✅ Created `config/landmarks.json` with sample competition waypoints
3. ✅ Standardized all paths to `/home/mad/NOMAD`
4. ✅ Removed hardcoded password, using environment variables

### Medium Priority - DONE
5. ✅ Dockerfile web_client reference already handled (commented out)
6. ✅ `start_server.py` already cross-platform
7. ✅ MAVLink router configs consolidated to `main.conf`
8. ✅ Fixed documentation URL in `tailscale-watchdog.service`

### Low Priority - DONE
9. ✅ `config/profiles/task2_indoor.params` already fully populated
10. ⏳ `pyproject.toml` - Deferred (not critical)

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

The codebase is well-organized and all identified issues have been resolved:
- ✅ Path standardization complete (`/home/mad/NOMAD`)
- ✅ Security issues fixed (passwords moved to environment variables)
- ✅ Configuration consolidated (single `main.conf` for MAVLink router)
- ✅ Missing files created (`landmarks.json`, `jetson.env`)
- ✅ Documentation updated
