# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Structural checks for the hosted MAVSDK Phase A gate."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_phase_a_job_builds_checks_provenance_and_runs_peer_cases() -> None:
    workflow = (ROOT / ".github" / "workflows" / "test.yml").read_text(encoding="utf-8")
    phase_a_job = workflow.split("  mavsdk-phase-a:", maxsplit=1)[1]
    assert "os: [ubuntu-latest, windows-latest]" in phase_a_job
    assert "pixi run build-core-mavsdk" in phase_a_job
    assert "pixi run check-mavsdk-phase-a" in phase_a_job
    assert "pixi run test-mavsdk-phase-a" in phase_a_job


def test_ros_workflow_does_not_claim_runtime_qualification() -> None:
    workflow = (ROOT / ".github" / "workflows" / "ros-sim.yml").read_text(encoding="utf-8")
    assert "This is compile evidence only" in workflow
