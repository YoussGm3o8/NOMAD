# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Run the optional MAVSDK Phase A connect/status smoke against SITL."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def get_sitl_port() -> str:
    port = os.environ.get("NOMAD_CORE_SITL_PORT", "14570")
    if not port.isdecimal() or not 1 <= int(port) <= 65535:
        raise ValueError("NOMAD_CORE_SITL_PORT must be a UDP port from 1 to 65535")
    return port


def find_binary() -> Path | None:
    names = ("nomad_mavsdk_phase_a_smoke.exe", "nomad_mavsdk_phase_a_smoke")
    build_dir = ROOT / "build" / "mavsdk-phase-a"
    for directory in (build_dir, build_dir / "Debug", build_dir / "Release"):
        for name in names:
            candidate = directory / name
            if candidate.is_file():
                return candidate
    return None


def run_smoke(binary: Path, command: str, endpoint: str) -> int:
    print(f"Running MAVSDK {command} smoke against {endpoint}", flush=True)
    try:
        result = subprocess.run([str(binary), command, endpoint], check=False, timeout=20)
    except subprocess.TimeoutExpired:
        print(f"error: MAVSDK {command} smoke timed out", file=sys.stderr)
        return 2
    return result.returncode


def main() -> int:
    try:
        port = get_sitl_port()
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
        result = run_smoke(binary, command, endpoint)
        if result != 0:
            return result
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
