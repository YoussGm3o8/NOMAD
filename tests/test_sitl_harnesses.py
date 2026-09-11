# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Unit tests for the C++ SITL safety harness helpers."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest
from pymavlink import mavutil

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts" / "dev"
sys.path.insert(0, str(SCRIPTS))

import core_sitl_command_flow as command_flow  # noqa: E402
import core_sitl_containment as containment  # noqa: E402
import core_sitl_zero_delivery as zero_delivery  # noqa: E402


def test_run_cli_rejection_requires_nonzero_result_and_expected_message(monkeypatch) -> None:
    completed = subprocess.CompletedProcess([], 1, "", "error: outside the geofence")
    monkeypatch.setattr(command_flow.subprocess, "run", lambda *args, **kwargs: completed)

    output = command_flow.run_cli_rejection(Path("nomad"), "14570", "outside the geofence", "goto", "1", "2", "3")

    assert "outside the geofence" in output


def test_run_cli_rejection_rejects_a_successful_command(monkeypatch) -> None:
    completed = subprocess.CompletedProcess([], 0, "goto verified", "")
    monkeypatch.setattr(command_flow.subprocess, "run", lambda *args, **kwargs: completed)

    with pytest.raises(command_flow.ScenarioError, match="expected command to fail"):
        command_flow.run_cli_rejection(Path("nomad"), "14570", "outside the geofence", "goto", "1", "2", "3")


@pytest.mark.parametrize("port, expected", [("1", (1, 2, 3)), ("65533", (65533, 65534, 65535))])
def test_zero_delivery_relay_ports_allow_only_safe_boundaries(port: str, expected: tuple[int, int, int]) -> None:
    assert zero_delivery.get_relay_ports(port) == expected


@pytest.mark.parametrize("port", ["0", "65534", "65535", "not-a-port"])
def test_zero_delivery_relay_ports_fail_closed(port: str) -> None:
    with pytest.raises(ValueError):
        zero_delivery.get_relay_ports(port)


def make_velocity_packet(vx: float, vy: float, vz: float, yaw_rate: float) -> bytes:
    encoder = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
    message = encoder.set_position_target_local_ned_encode(
        0,
        1,
        1,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0,
        0.0,
        0.0,
        0.0,
        vx,
        vy,
        vz,
        0.0,
        0.0,
        0.0,
        0.0,
        yaw_rate,
    )
    return message.pack(encoder)


def test_zero_delivery_observer_decodes_nonzero_and_zero_datagrams() -> None:
    observer = zero_delivery.SetpointObserver(0)
    try:
        observer._record_datagram(make_velocity_packet(0.3, 0.0, 0.0, 0.0))
        observer._record_datagram(make_velocity_packet(0.0, 0.0, 0.0, 0.0))
        points = observer.setpoints_in(1.0)
    finally:
        observer.close()

    assert len(points) == 2
    assert zero_delivery.is_nonzero(points[0])
    assert zero_delivery.is_zero(points[1])


def test_containment_converts_latitude_and_longitude_to_local_metres() -> None:
    north, east = containment.get_local_displacement((45.0, -73.0), (45.0001, -72.9999))

    assert north == pytest.approx(11.054)
    assert east == pytest.approx(7.871, rel=0.01)


def test_containment_fence_corners_have_equal_local_extent() -> None:
    home = (45.0, -73.0)
    corners = [
        tuple(float(part) for part in vertex.split(",")) for vertex in containment.fence_env_around(home).split(";")
    ]

    displacements = [containment.get_local_displacement(home, corner) for corner in corners]

    assert len(displacements) == 4
    for north, east in displacements:
        assert abs(north) == pytest.approx(containment.FENCE_HALF_EXTENT_M, rel=0.001)
        assert abs(east) == pytest.approx(containment.FENCE_HALF_EXTENT_M, rel=0.001)


def test_containment_cleanup_attempts_every_safe_action_after_failures(monkeypatch) -> None:
    attempted: list[str] = []

    def fail_command(binary, port, action, **kwargs):
        attempted.append(action)
        raise containment.ScenarioError(f"{action} failed")

    def fail_status(binary, port, expected, timeout):
        attempted.append("status")
        raise containment.ScenarioError("status failed")

    monkeypatch.setattr(containment, "run_cli", fail_command)
    monkeypatch.setattr(containment, "wait_for_status", fail_status)

    errors = containment.cleanup_containment(Path("nomad"), "14570")

    assert attempted == ["rtl", "land", "status", "disarm"]
    assert len(errors) == 4
