# SPDX-License-Identifier: Apache-2.0
"""Verify read-only bench tooling without a serial device."""

from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

PATH = Path(__file__).resolve().parents[1] / "scripts/hardware/observe_rc.py"
SPEC = importlib.util.spec_from_file_location("observe_rc", PATH)
observer = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(observer)


class Port:
    def __init__(self):
        self.frames = []

    def write(self, frame):
        self.frames.append(frame)


def test_snapshot_transmits_only_read_requests(monkeypatch):
    port = Port()
    firmware = observer.mavlink.MAVLink_autopilot_version_message(
        0, 67568127, 0, 0, 0, [0] * 8, [0] * 8, [0] * 8, 0, 0, 0
    )
    parameter = observer.mavlink.MAVLink_param_value_message(b"RCMAP_ROLL", 1, 9, 1, 0)
    monkeypatch.setattr(observer, "selected_messages", lambda *_: iter([parameter, firmware]))
    result = observer.snapshot(port, None, (1, 1), 5)
    parser = observer.mavlink.MAVLink(None)
    messages = [message for frame in port.frames for message in parser.parse_buffer(frame)]
    assert [message.get_type() for message in messages] == ["PARAM_REQUEST_LIST", "COMMAND_LONG"]
    assert messages[1].command == observer.mavlink.MAV_CMD_REQUEST_MESSAGE
    assert messages[1].param1 == 148
    assert result["complete"]
    assert result["count"] == 1


def test_armed_heartbeat_aborts():
    message = SimpleNamespace(get_type=lambda: "HEARTBEAT", base_mode=128)
    with pytest.raises(observer.ObservationError, match="armed"):
        observer.validate_disarmed(message)


def test_summary_does_not_certify_physical_receiver():
    result = observer.summarize_rc([(1.0, [1000] * 18, 0, 254), (2.0, [2000] * 18, 0, 254)])
    assert result["observed_rate_hz"] == 1
    assert result["channels"]["1"] == {"min": 1000, "max": 2000, "last": 2000, "distinct_values": [1000, 2000]}
    assert result["chancount"] == [0]
    assert result["native_input_proven"] is False


def test_inconsistent_snapshot_rejected():
    values, indices = {}, {}
    first = SimpleNamespace(param_count=2, param_index=0, param_id="FIRST", param_value=1, param_type=9)
    count = observer.record_parameter(first, values, indices, None)
    second = SimpleNamespace(param_count=2, param_index=0, param_id="OTHER", param_value=1, param_type=9)
    with pytest.raises(observer.ObservationError, match="index/name"):
        observer.record_parameter(second, values, indices, count)


def test_evidence_cannot_be_overwritten(tmp_path):
    path = tmp_path / "capture.json"
    digest = observer.save_record(path, {"samples": 0})
    assert len(digest) == 64
    with pytest.raises(FileExistsError):
        observer.save_record(path, {"samples": 1})


def test_capture_sends_nothing(monkeypatch):
    port = Port()
    ticks = iter(index / 10 for index in range(30))
    monkeypatch.setattr(observer, "time", SimpleNamespace(monotonic=lambda: next(ticks)))
    monkeypatch.setattr(observer, "selected_messages", lambda *_: iter([]))
    result = observer.capture(port, None, (1, 1), 1, "no-input")
    assert port.frames == []
    assert result["samples"] == 0
    assert result["native_input_proven"] is False


def test_capture_aborts_on_heartbeat_loss(monkeypatch):
    port = Port()
    ticks = iter(range(20))
    monkeypatch.setattr(observer, "time", SimpleNamespace(monotonic=lambda: next(ticks)))
    monkeypatch.setattr(observer, "selected_messages", lambda *_: iter([]))
    with pytest.raises(observer.ObservationError, match="heartbeat stale"):
        observer.capture(port, None, (1, 1), 8, "lost-link")
    assert port.frames == []
