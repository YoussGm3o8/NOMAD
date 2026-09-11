# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Structural checks for the hosted MAVSDK Phase A gate."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_phase_a_job_builds_checks_provenance_and_runs_peer_cases() -> None:
    workflow = (ROOT / ".github" / "workflows" / "test.yml").read_text(encoding="utf-8")
    phase_a_job = workflow.split("  mavsdk-phase-a:", maxsplit=1)[1]
    assert "os: [ubuntu-latest, windows-latest]" in phase_a_job
    assert "mavsdk_phase_a_metrics.py --build" in phase_a_job
    assert "pixi run check-mavsdk-phase-a" in phase_a_job
    assert "pixi run test-mavsdk-phase-a" in phase_a_job


def test_phase_a_job_retains_resource_evidence() -> None:
    workflow = (ROOT / ".github" / "workflows" / "test.yml").read_text(encoding="utf-8")
    phase_a_job = workflow.split("  mavsdk-phase-a:", maxsplit=1)[1]
    assert "mavsdk_phase_a_metrics.py --build" in phase_a_job
    assert "--output" in phase_a_job
    assert "actions/upload-artifact@v6" in phase_a_job
    assert "if-no-files-found: error" in phase_a_job


def test_ros_workflow_does_not_claim_runtime_qualification() -> None:
    workflow = (ROOT / ".github" / "workflows" / "ros-sim.yml").read_text(encoding="utf-8")
    assert "This is compile evidence only" in workflow


def test_sitl_workflow_retains_per_process_runtime_metrics() -> None:
    workflow = (ROOT / ".github" / "workflows" / "sitl.yml").read_text(encoding="utf-8")
    assert workflow.count("mavsdk-phase-a-runtime.txt") == 4
    assert workflow.count("name: mavsdk-phase-a-runtime") == 2
    assert workflow.count("if-no-files-found: error") == 2
