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
import core_sitl_quadplane_route as route  # noqa: E402
import core_sitl_quadplane_transition as transition  # noqa: E402
import core_sitl_quadplane_vtol_takeoff as vtol_takeoff  # noqa: E402


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
    assert service["environment"]["SITL_UDP_OBSERVER_ADDRESS"].endswith(":14581")
    assert "SITL_UDP_OBSERVER_ADDRESS" in entrypoint
    assert quadplane.EXPECTED_IDENTITY == {
        "autopilot_type": "3",
        "vehicle_type": "1",
        "aircraft_class": "QuadPlane",
    }


def test_quadplane_profile_supplies_identity_and_telemetry_parameters() -> None:
    profile = (ROOT / "docker" / "quadplane-tilttri.parm").read_text(encoding="utf-8")
    for parameter in (
        "Q_ENABLE 2",
        "Q_FRAME_CLASS 7",
        "Q_TILT_ENABLE 1",
        "Q_TILT_MASK 3",
        "Q_ASSIST_SPEED 6",
        "Q_TRANSITION_MS 5000",
        "Q_TRANS_FAIL 0",
        "Q_TRANS_FAIL_ACT 0",
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


def test_quadplane_takeoff_harness_requires_authoritative_climb_state() -> None:
    source = (ROOT / "scripts" / "dev" / "core_sitl_quadplane_vtol_takeoff.py").read_text(encoding="utf-8")
    assert 'run_cli(binary, port, "vtol-takeoff"' in source
    assert "require_climb_status" in source
    assert "baseline_altitude_m" in source
    assert "COMPLETION_TOLERANCE_M" in source
    assert "* 0.8" not in source
    assert "relative_altitude_m" in source
    assert vtol_takeoff.MINIMUM_VERIFIED_ALTITUDE_M == 4.5


def test_quadplane_takeoff_harness_verifies_delta_target_with_strict_tolerance() -> None:
    fields = {
        **quadplane.EXPECTED_IDENTITY,
        "connected": "true",
        "heartbeat_fresh": "true",
        "armed": "true",
        "mode": "15",
        "position": "1,2",
        "altitude_m": "7",
        "relative_altitude_m": "6.6",
        "gps_fix": "3",
        "attitude": "0,0,0",
        "position_age_ms": "1",
        "gps_age_ms": "1",
        "attitude_age_ms": "1",
    }
    vtol_takeoff.require_climb_status(fields, 7.0)

    fields["relative_altitude_m"] = "6.4"
    with pytest.raises(quadplane.ScenarioError, match="below 6.5 m"):
        vtol_takeoff.require_climb_status(fields, 7.0)


def test_quadplane_transition_harness_requires_authoritative_fixed_wing_state() -> None:
    source = (ROOT / "scripts" / "dev" / "core_sitl_quadplane_transition.py").read_text(encoding="utf-8")
    assert "run_cli(" in source
    assert '"transition-to-fixed-wing"' in source
    assert "MAV_CMD_DO_VTOL_TRANSITION" in source
    assert "MAV_VTOL_STATE_FW" in source
    assert "vtol_state_age_ms" in source
    assert "EXTENDED_SYS_STATE" in source
    assert "request_observed_mode(port, MODE_AUTO)" in source
    assert "transition_to_fixed_wing" in source
    assert "fixed_wing" in source


def test_quadplane_transition_state_sequence_is_collapsed_without_fabrication() -> None:
    states = [
        (0.0, transition.VTOL_STATE_MC),
        (0.2, transition.VTOL_STATE_MC),
        (0.4, transition.VTOL_STATE_TRANSITION_TO_FW),
        (0.6, transition.VTOL_STATE_TRANSITION_TO_FW),
        (0.8, transition.VTOL_STATE_FW),
    ]
    assert transition.observed_state_names(states) == ["multicopter", "transition_to_fixed_wing", "fixed_wing"]


def test_quadplane_route_harness_drives_nomad_and_observes_aircraft_positions() -> None:
    source = (ROOT / "scripts" / "dev" / "core_sitl_quadplane_route.py").read_text(encoding="utf-8")
    assert "prepare_transition(" in source
    assert "complete_transition(" in source
    assert '"fixed-wing-route"' in source
    assert '"GLOBAL_POSITION_INT"' in source
    assert "guided_times" in source
    assert "verify_observed_route" in source
    assert "observed_progress=" in source
    assert "MAV_VTOL_STATE_FW" in source or "VTOL_STATE_FW" in source


def test_quadplane_route_harness_builds_two_forward_waypoints_and_verifies_order() -> None:
    fields = {"position": "45.5,-73.5", "relative_altitude_m": "9", "attitude": "0,0,0"}
    waypoints = route.route_waypoints(fields)
    assert len(waypoints) == 2
    assert 215.0 <= route.distance_m((45.5, -73.5), waypoints[0][:2]) <= 225.0
    assert 155.0 <= route.distance_m(waypoints[0][:2], waypoints[1][:2]) <= 165.0

    observer = route.RouteObserver(14581)
    observer.positions = [
        (1.0, waypoints[0][0], waypoints[0][1], 9.0),
        (2.0, waypoints[1][0], waypoints[1][1], 9.0),
    ]
    progress, distances = route.verify_observed_route(observer, (45.5, -73.5), waypoints)
    assert progress == [1, 2]
    assert distances == [0.0, 0.0]

    observer.positions = [(2.0, waypoints[1][0], waypoints[1][1], 9.0)]
    with pytest.raises(quadplane.ScenarioError, match="did not see waypoint 1"):
        route.verify_observed_route(observer, (45.5, -73.5), waypoints)
