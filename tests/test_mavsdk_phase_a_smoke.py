# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Contract tests for the MAVSDK Phase A smoke runner."""

from __future__ import annotations

import subprocess
from pathlib import Path

import pytest

from scripts.dev import mavsdk_phase_a_smoke as smoke


@pytest.mark.parametrize("value", ["", "0", "256", "one"])
def test_system_id_rejects_invalid_values(value: str, monkeypatch) -> None:
    monkeypatch.setenv("NOMAD_MAVSDK_SYSTEM_ID", value)
    with pytest.raises(ValueError):
        smoke.get_system_id()


def test_run_smoke_passes_expected_identity_and_accepts_complete_status(monkeypatch, tmp_path: Path) -> None:
    binary = tmp_path / "smoke"
    binary.touch()
    completed = subprocess.CompletedProcess(
        [],
        0,
        "connected=true system=7 samples=3\nposition=45,-73 battery_v=12 gps_fix=3 flight_mode=1\n",
        "",
    )
    calls: list[list[str]] = []

    def fake_run(command: list[str], **_kwargs):
        calls.append(command)
        return completed

    monkeypatch.setattr(subprocess, "run", fake_run)
    assert smoke.run_smoke(binary, "status", "udpin:0.0.0.0:14570", "7") == 0
    assert calls == [[str(binary), "status", "udpin:0.0.0.0:14570", "7"]]


def test_run_smoke_rejects_incomplete_success_output(monkeypatch, tmp_path: Path) -> None:
    binary = tmp_path / "smoke"
    binary.touch()
    result = subprocess.CompletedProcess([], 0, "connected=true system=1\n", "")
    monkeypatch.setattr(subprocess, "run", lambda *_args, **_kwargs: result)
    assert smoke.run_smoke(binary, "status", "udpin:0.0.0.0:14570", "1") == 2


@pytest.mark.parametrize("error", [subprocess.TimeoutExpired("smoke", 20), OSError("loader failed")])
def test_run_smoke_converts_process_failures_to_evidence_error(error: Exception, monkeypatch, tmp_path: Path) -> None:
    binary = tmp_path / "smoke"
    binary.touch()

    def fail(*_args, **_kwargs):
        raise error

    monkeypatch.setattr(subprocess, "run", fail)
    assert smoke.run_smoke(binary, "connect", "udpin:0.0.0.0:14570", "1") == 2
