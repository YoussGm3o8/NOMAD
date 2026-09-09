# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Telemetry and fail-closed VIO integration tests for ``nomad_ros``."""

from __future__ import annotations

import threading
import time

import pytest

from ros_integration_support import _publish_cmd_vel, _publish_vio


def test_node_publishes_telemetry(ros_session) -> None:
    """The node derives typed telemetry from the responder's MAVLink frames."""
    if not ros_session["connected"]:
        pytest.fail("node never connected to the MAVLink responder")
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline and ros_session["state"].fix_messages < 2:
        time.sleep(0.2)
    assert ros_session["state"].fix_messages >= 2, (
        f"expected >= 2 /nomad/fix messages, got {ros_session['state'].fix_messages}"
    )


def test_vio_gate_blocks_velocity_without_feed(ros_session) -> None:
    """No VIO feed -> the node refuses the command and nothing reaches the vehicle."""
    if not ros_session["connected"]:
        pytest.fail("node never connected to the MAVLink responder")
    before = len(ros_session["state"].setpoints)
    _publish_cmd_vel(ros_session, 1.5, vx=1.0)
    assert len(ros_session["state"].setpoints) == before, (
        "a velocity setpoint reached the vehicle with no VIO feed (gate failed open)"
    )
    assert any("without a fresh VIO feed" in line for line in ros_session["log_lines"]), (
        "node did not log the VIO-gate refusal"
    )


def test_vio_source_gate_blocks_mismatch(ros_session) -> None:
    """A source other than the configured VIO source cannot open the velocity gate."""
    if not ros_session["connected"]:
        pytest.fail("node never connected to the MAVLink responder")
    before = len(ros_session["state"].setpoints)
    vio_thread = threading.Thread(
        target=_publish_vio,
        args=(ros_session, 1.5),
        kwargs={"source": "other_vio"},
        daemon=True,
    )
    vio_thread.start()
    try:
        time.sleep(0.5)
        _publish_cmd_vel(ros_session, 1.0, vx=1.0)
    finally:
        vio_thread.join(timeout=3.0)

    assert len(ros_session["state"].setpoints) == before
    assert any("VIO source mismatch" in line for line in ros_session["log_lines"])


def test_velocity_command_reaches_vehicle(ros_session) -> None:
    """Healthy VIO + cmd_vel -> a forward setpoint reaches the vehicle."""
    if not ros_session["connected"]:
        pytest.fail("node never connected to the MAVLink responder")
    before = len(ros_session["state"].setpoints)
    vio_thread = threading.Thread(target=_publish_vio, args=(ros_session, 4.0), daemon=True)
    vio_thread.start()
    try:
        time.sleep(0.5)  # let a fresh VIO sample land before the first command
        _publish_cmd_vel(ros_session, 3.0, vx=1.0)
    finally:
        vio_thread.join(timeout=5.0)

    new_setpoints = ros_session["state"].setpoints[before:]
    if not new_setpoints:
        pytest.fail("no SET_POSITION_TARGET_LOCAL_NED reached the vehicle\n" + "\n".join(ros_session["log_lines"]))
    assert any(abs(vx - 1.0) < 0.15 for vx in new_setpoints), (
        f"expected a forward setpoint vx ~= 1.0 m/s, got {new_setpoints}"
    )
