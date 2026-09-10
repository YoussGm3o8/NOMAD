# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Collect repeatable build-footprint metrics for MAVSDK Phase A."""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_BUILD_DIR = ROOT / "build" / "mavsdk-phase-a"


def directory_size(path: Path) -> int:
    """Return the total byte size of regular files below *path*."""
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


def find_smoke_binary(build_dir: Path = DEFAULT_BUILD_DIR) -> Path | None:
    """Find the single- or multi-config Phase A smoke executable."""
    names = ("nomad_mavsdk_phase_a_smoke.exe", "nomad_mavsdk_phase_a_smoke")
    for directory in (build_dir, build_dir / "Release", build_dir / "Debug"):
        for name in names:
            candidate = directory / name
            if candidate.is_file():
                return candidate
    return None


def collect_build_metrics(build_dir: Path = DEFAULT_BUILD_DIR) -> dict[str, int | str]:
    """Collect byte-count evidence from a completed Phase A build."""
    if not build_dir.is_dir():
        raise FileNotFoundError(f"MAVSDK Phase A build directory not found: {build_dir}")

    binary = find_smoke_binary(build_dir)
    if binary is None:
        raise FileNotFoundError("MAVSDK Phase A smoke executable was not found")

    install_lib = build_dir / "mavsdk" / "third_party" / "install" / "lib"
    archives = []
    if install_lib.is_dir():
        archives = [
            item
            for item in install_lib.rglob("*")
            if item.is_file() and item.suffix.lower() in {".a", ".lib"}
        ]

    return {
        "build_tree_bytes": directory_size(build_dir),
        "smoke_executable_bytes": binary.stat().st_size,
        "selected_static_archives": len(archives),
        "selected_static_archives_bytes": sum(item.stat().st_size for item in archives),
    }


def write_github_summary(metrics: dict[str, int | str], summary_path: Path) -> None:
    """Append the build metrics to the current GitHub Actions job summary."""
    rows = [
        "### MAVSDK Phase A build metrics",
        "",
        "| Metric | Value |",
        "|---|---:|",
    ]
    rows.extend(f"| `{key}` | {value} |" for key, value in metrics.items())
    rows.append("")
    with summary_path.open("a", encoding="utf-8") as stream:
        stream.write("\n".join(rows))


def main() -> int:
    try:
        metrics = collect_build_metrics()
    except (FileNotFoundError, OSError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    print(f"mavsdk_phase_a_build_metrics={json.dumps(metrics, sort_keys=True)}")
    summary = os.environ.get("GITHUB_STEP_SUMMARY")
    if summary:
        try:
            write_github_summary(metrics, Path(summary))
        except OSError as error:
            print(f"warning: could not write GitHub step summary: {error}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
