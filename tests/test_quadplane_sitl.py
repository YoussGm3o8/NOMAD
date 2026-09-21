# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Structural and falsification checks for the QuadPlane observation harness."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts" / "dev"))

import core_sitl_quadplane_observe as quadplane  # noqa: E402


def test_quadplane_profile_pins_firmware_tooling_frame_and_identity() -> None:
    yaml = pytest.importorskip("yaml")
    compose = yaml.safe_load((ROOT / "docker" / "docker-compose.quadplane.yml").read_text(encoding="utf-8"))
    service = compose["services"]["quadplane_sitl"]
    dockerfile = (ROOT / service["build"]["dockerfile"]).read_text(encoding="utf-8")
    entrypoint = (ROOT / "docker" / "sitl-quadplane-entrypoint.sh").read_text(encoding="utf-8")

    build_args = service["build"]["args"]
    revision = build_args["ARDUPILOT_REVISION"]
    assert revision == quadplane.EXPECTED_ARDUPILOT_REVISION
    assert f"ARG ARDUPILOT_REVISION={revision}" in dockerfile
    assert build_args["MAVPROXY_VERSION"] == "1.8.74"
    assert build_args["PYMAVLINK_VERSION"] == "2.4.49"
    assert "ARG MAVPROXY_VERSION=1.8.74" in dockerfile
    assert "ARG PYMAVLINK_VERSION=2.4.49" in dockerfile
    assert '"MAVProxy==${MAVPROXY_VERSION}"' in dockerfile
    assert '"pymavlink==${PYMAVLINK_VERSION}"' in dockerfile
    assert "./waf plane" in dockerfile
    assert "--vehicle ArduPlane" in entrypoint
    assert "--frame quadplane-tilttri" in entrypoint
    assert quadplane.EXPECTED_IDENTITY == {
        "autopilot_type": "3",
        "vehicle_type": "1",
        "aircraft_class": "QuadPlane",
    }


def test_quadplane_profile_supplies_identity_and_telemetry_parameters() -> None:
    profile = (ROOT / "docker" / "quadplane-tilttri.parm").read_text(encoding="utf-8")
    for parameter in (
        "Q_ENABLE 1",
        "Q_FRAME_CLASS 7",
        "Q_TILT_ENABLE 1",
        "Q_TILT_MASK 3",
        "SR0_POSITION 5",
        "SR0_EXT_STAT 2",
        "SR0_EXTRA1 5",
        "SR0_EXTRA2 2",
    ):
        assert parameter in profile


def test_quadplane_observer_rejects_stale_position() -> None:
    fields = {
        "position": "1,2",
        "altitude_m": "3",
        "gps_fix": "3",
        "attitude": "0,0,0",
        "position_age_ms": str(quadplane.MAX_SAMPLE_AGE_MS + 1),
        "gps_age_ms": "1",
        "attitude_age_ms": "1",
    }
    with pytest.raises(quadplane.ScenarioError, match="stale position_age_ms"):
        quadplane.require_fresh_telemetry(fields)


def test_quadplane_observer_rejects_wrong_identity() -> None:
    with pytest.raises(quadplane.ScenarioError, match="unexpected QuadPlane status"):
        quadplane.require_status(
            {"autopilot_type": "0", "vehicle_type": "21", "aircraft_class": "Unknown"},
            quadplane.EXPECTED_IDENTITY,
        )


def test_quadplane_observer_does_not_use_production_mode_admission() -> None:
    source = (ROOT / "scripts" / "dev" / "core_sitl_quadplane_observe.py").read_text(encoding="utf-8")
    assert 'run_cli(binary, port, "mode"' not in source
    assert "connection.mav.set_mode_send" in source
