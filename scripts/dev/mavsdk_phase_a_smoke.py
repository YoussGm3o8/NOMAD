# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Run the optional MAVSDK Phase A connect/status smoke against SITL."""

from __future__ import annotations

import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def get_sitl_port() -> str:
    port = os.environ.get("NOMAD_CORE_SITL_PORT", "14570")
    if not port.isdecimal() or not 1 <= int(port) <= 65535:
        raise ValueError("NOMAD_CORE_SITL_PORT must be a UDP port from 1 to 65535")
    return port


def get_system_id() -> str:
    system_id = os.environ.get("NOMAD_MAVSDK_SYSTEM_ID", "1")
    if not system_id.isdecimal() or not 1 <= int(system_id) <= 255:
        raise ValueError("NOMAD_MAVSDK_SYSTEM_ID must be from 1 to 255")
    return system_id


def find_binary() -> Path | None:
    names = ("nomad_mavsdk_phase_a_smoke.exe", "nomad_mavsdk_phase_a_smoke")
    build_dir = ROOT / "build" / "mavsdk-phase-a"
    for directory in (build_dir, build_dir / "Debug", build_dir / "Release"):
        for name in names:
            candidate = directory / name
            if candidate.is_file():
                return candidate
    return None


def has_required_output(command: str, output: str, system_id: str) -> bool:
    required = ["connected=true", f"system={system_id}"]
    if command == "status":
        required.extend(("samples=", "position=", "battery_v=", "gps_fix=", "flight_mode="))
    return all(field in output for field in required)


def child_peak_rss_bytes() -> int | None:
    """Return peak RSS for completed child processes when the platform supports it."""
    try:
        import resource
    except ImportError:
        return None

    peak = int(resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss)
    if sys.platform == "darwin":
        return peak
    return peak * 1024


def report_runtime_metric(command: str, elapsed_seconds: float) -> None:
    fields = [
        f"command={command}",
        f"elapsed_seconds={elapsed_seconds:.3f}",
    ]
    peak_rss = child_peak_rss_bytes()
    if peak_rss is not None:
        fields.append(f"peak_child_rss_bytes={peak_rss}")
    print(f"mavsdk_phase_a_runtime_metric {' '.join(fields)}", flush=True)


def run_smoke(binary: Path, command: str, endpoint: str, system_id: str) -> int:
    print(f"Running MAVSDK {command} smoke against {endpoint}", flush=True)
    started = time.perf_counter()
    try:
        result = subprocess.run(
            [str(binary), command, endpoint, system_id],
            check=False,
            capture_output=True,
            text=True,
            timeout=20,
        )
    except subprocess.TimeoutExpired:
        report_runtime_metric(command, time.perf_counter() - started)
        print(f"error: MAVSDK {command} smoke timed out", file=sys.stderr)
        return 2
    except OSError as error:
        report_runtime_metric(command, time.perf_counter() - started)
        print(f"error: could not run MAVSDK smoke: {error}", file=sys.stderr)
        return 2

    report_runtime_metric(command, time.perf_counter() - started)
    if result.stdout:
        print(result.stdout, end="")
    if result.stderr:
        print(result.stderr, end="", file=sys.stderr)
    if result.returncode == 0 and not has_required_output(command, result.stdout, system_id):
        print(f"error: MAVSDK {command} output did not satisfy the evidence contract", file=sys.stderr)
        return 2
    return result.returncode


def main() -> int:
    try:
        port = get_sitl_port()
        system_id = get_system_id()
    except ValueError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    binary = find_binary()
    if binary is None:
        print(
            "error: MAVSDK Phase A smoke binary not found; run `pixi run build-core-mavsdk` first",
            file=sys.stderr,
        )
        return 2

    endpoint = f"udpin:0.0.0.0:{port}"
    for command in ("connect", "status"):
        result = run_smoke(binary, command, endpoint, system_id)
        if result != 0:
            return result
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
