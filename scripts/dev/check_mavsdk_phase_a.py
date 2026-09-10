# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Verify the pinned MAVSDK Phase A source and dependency inventory."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

EXPECTED_REVISIONS = {
    "third_party/MAVSDK": "34b417d45c2c33ce0414bc1bc61b54010d055224",
    "third_party/ardupilot-mavlink": "288b907c384a892c8519bfe271682424b1e1a3a0",
}
EXPECTED_DEPENDENCIES = {
    "cpp/third_party/CMakeLists.txt": 'MAVLINK_HASH "d6a7eeaf43319ce6da19a1973ca40180a4210643"',
    "cpp/third_party/asio/CMakeLists.txt": 'ASIO_GIT_TAG "asio-1-30-2"',
    "cpp/third_party/fmt/CMakeLists.txt": "GIT_TAG 12.1.0",
    "cpp/third_party/libevents/CMakeLists.txt": "GIT_TAG 840a88ea226d4eb0fd4c391ce860317422756435",
    "cpp/third_party/libmavlike/CMakeLists.txt": "GIT_TAG 90498b14262137ae10b633705810e81bdb85de9c",
    "cpp/third_party/liblzma/CMakeLists.txt": "xz-5.4.5.tar.gz",
    "cpp/third_party/nlohmann_json/CMakeLists.txt": "v3.12.0.tar.gz",
    "cpp/third_party/picosha2/CMakeLists.txt": "GIT_TAG cmake-install-support",
    "cpp/third_party/tinyxml2/CMakeLists.txt": "GIT_TAG 11.0.0",
}
NOTICE_COMPONENTS = (
    "MAVSDK",
    "Asio",
    "fmt",
    "libevents",
    "libmavlike",
    "MAVLink",
    "nlohmann JSON",
    "PicoSHA2",
    "tinyxml2",
    "liblzma",
)


def git_revision(path: str) -> str:
    result = subprocess.run(
        ["git", "rev-parse", f"HEAD:{path}"],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()


def check_expected_text(path: Path, expected: str) -> None:
    if expected not in path.read_text(encoding="utf-8"):
        raise RuntimeError(f"{path.relative_to(ROOT)} no longer contains expected provenance: {expected}")


def verify_provenance() -> None:
    for path, expected in EXPECTED_REVISIONS.items():
        actual = git_revision(path)
        if actual != expected:
            raise RuntimeError(f"{path} revision changed: expected {expected}, observed {actual}")

    mavsdk_root = ROOT / "third_party" / "MAVSDK"
    for relative, expected in EXPECTED_DEPENDENCIES.items():
        check_expected_text(mavsdk_root / relative, expected)

    proto = subprocess.run(
        ["git", "rev-parse", "HEAD:proto"],
        cwd=mavsdk_root,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    if proto != "1fd0bc7a05c21336227b1eab266b8b610401cf38":
        raise RuntimeError(f"MAVSDK proto revision changed: {proto}")

    notice = (ROOT / "NOTICE").read_text(encoding="utf-8")
    missing = [component for component in NOTICE_COMPONENTS if component not in notice]
    if missing:
        raise RuntimeError(f"NOTICE is missing MAVSDK dependencies: {', '.join(missing)}")


def main() -> int:
    try:
        verify_provenance()
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    print("MAVSDK Phase A provenance matches the reviewed inventory")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
