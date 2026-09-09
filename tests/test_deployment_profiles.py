# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Deterministic tests for NOMAD deployment profiles.

Validates the three supported profiles:
- onboard_companion
- groundstation_gpu
- groundstation_minimal

Verifies profile separation of concerns:
- compute placement
- companion presence
- perception/GPU capability
- video source
- VIO availability
- core placement and transport
- direct MAVLink connection
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from scripts.profile import PROFILES, PROFILES_DIR, _parse_env, sync_mission_planner


def test_supported_profiles_exist() -> None:
    expected_profiles = {"onboard_companion", "groundstation_gpu", "groundstation_minimal"}
    for name in expected_profiles:
        profile_file = PROFILES_DIR / f"{name}.env"
        assert profile_file.exists(), f"Missing profile file: {profile_file}"
        assert name in PROFILES, f"Profile {name} not registered in PROFILES dictionary"


def test_onboard_companion_profile_separation() -> None:
    env = _parse_env(PROFILES_DIR / "onboard_companion.env")
    assert env.get("NOMAD_PROFILE") == "onboard_companion"
    assert env.get("NOMAD_COMPUTE_PLACEMENT") == "onboard"
    assert env.get("NOMAD_HAS_COMPANION") == "true"
    assert env.get("NOMAD_HAS_PERCEPTION") == "true"
    assert env.get("NOMAD_VIO_SOURCE_REQUIRED") == "true"
    assert "rtsp://" in env.get("NOMAD_VIDEO_RTSP_URL", "")
    assert env.get("NOMAD_MAVLINK_ENDPOINT") == "127.0.0.1:14550"
    assert env.get("NOMAD_API_KEY")


def test_groundstation_gpu_profile_separation() -> None:
    env = _parse_env(PROFILES_DIR / "groundstation_gpu.env")
    assert env.get("NOMAD_PROFILE") == "groundstation_gpu"
    assert env.get("NOMAD_COMPUTE_PLACEMENT") == "groundstation"
    assert env.get("NOMAD_HAS_COMPANION") == "false"
    assert env.get("NOMAD_HAS_PERCEPTION") == "true"
    assert env.get("NOMAD_VIO_SOURCE_REQUIRED") == "true"
    assert "127.0.0.1" in env.get("NOMAD_VIDEO_RTSP_URL", "")
    assert env.get("NOMAD_CORE_SITL_PORT")
    assert env.get("NOMAD_API_KEY")


def test_groundstation_minimal_profile_separation() -> None:
    env = _parse_env(PROFILES_DIR / "groundstation_minimal.env")
    assert env.get("NOMAD_PROFILE") == "groundstation_minimal"
    assert env.get("NOMAD_COMPUTE_PLACEMENT") == "groundstation"
    assert env.get("NOMAD_HAS_COMPANION") == "false"
    assert env.get("NOMAD_HAS_PERCEPTION") == "false"
    assert env.get("NOMAD_VIO_SOURCE_REQUIRED") == "false"
    assert env.get("NOMAD_VIDEO_RTSP_URL", "") == ""
    # Direct MAVLink / C++ core transport remains active
    assert env.get("NOMAD_CORE_SITL_PORT")
    assert env.get("NOMAD_API_KEY")
    assert env.get("NOMAD_AUTOSTART_ROS_VEHICLE") == "false"
    assert env.get("NOMAD_AUTOSTART_VIDEO_BRIDGE") == "false"


def test_sync_mission_planner_preserves_and_updates_profiles(tmp_path: Path, monkeypatch) -> None:
    cfg_file = tmp_path / "nomad_config.json"
    initial_config = {
        "CustomUserSetting": "preserved_value",
        "ActiveProfile": "initial",
    }
    cfg_file.write_text(json.dumps(initial_config), encoding="utf-8")
    monkeypatch.setenv("NOMAD_MP_CONFIG", str(cfg_file))

    # Sync onboard_companion
    onboard_env = _parse_env(PROFILES_DIR / "onboard_companion.env")
    sync_mission_planner("onboard_companion", onboard_env)
    synced = json.loads(cfg_file.read_text(encoding="utf-8"))
    assert synced["CustomUserSetting"] == "preserved_value"
    assert synced["ActiveProfile"] == "onboard_companion"
    assert synced["CoreApiKey"] == onboard_env["NOMAD_API_KEY"]
    assert synced["CoreMavlinkEndpoint"] == onboard_env["NOMAD_MAVLINK_ENDPOINT"]
    assert synced["VideoUrl"] == onboard_env["NOMAD_VIDEO_RTSP_URL"]

    # Sync groundstation_minimal
    minimal_env = _parse_env(PROFILES_DIR / "groundstation_minimal.env")
    sync_mission_planner("groundstation_minimal", minimal_env)
    synced_min = json.loads(cfg_file.read_text(encoding="utf-8"))
    assert synced_min["ActiveProfile"] == "groundstation_minimal"
    assert synced_min["VideoUrl"] == ""
    assert synced_min["CoreApiKey"] == minimal_env["NOMAD_API_KEY"]
