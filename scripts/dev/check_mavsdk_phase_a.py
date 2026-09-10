# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Verify the pinned MAVSDK Phase A source and dependency inventory."""

from __future__ import annotations

import hashlib
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

EXPECTED_REVISIONS = {
    "third_party/MAVSDK": "9884f109533f564bc6250e5471e6301d3a62f4a7",
    "third_party/ardupilot-mavlink": "288b907c384a892c8519bfe271682424b1e1a3a0",
}
EXPECTED_DEPENDENCIES = {
    "cpp/third_party/CMakeLists.txt": 'MAVLINK_HASH "d6a7eeaf43319ce6da19a1973ca40180a4210643"',
    "cpp/third_party/asio/CMakeLists.txt": 'ASIO_GIT_TAG "asio-1-30-2"',
    "cpp/third_party/fmt/CMakeLists.txt": "GIT_TAG 12.1.0",
    "cpp/third_party/libevents/CMakeLists.txt": "GIT_TAG 840a88ea226d4eb0fd4c391ce860317422756435",
    "cpp/third_party/libmavlike/CMakeLists.txt": "GIT_TAG 90498b14262137ae10b633705810e81bdb85de9c",
    "cpp/third_party/liblzma/CMakeLists.txt": (
        "URL_HASH SHA256=135c90b934aee8fbc0d467de87a05cb70d627da36abe518c357a873709e5b7d6"
    ),
    "cpp/third_party/mavlink/mavlink.patch": '"PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}"',
    "cpp/third_party/nlohmann_json/CMakeLists.txt": (
        "URL_HASH SHA256=4b92eb0c06d10683f7447ce9406cb97cd4b453be18d7279320f7b2f025c10187"
    ),
    "cpp/third_party/picosha2/CMakeLists.txt": "GIT_TAG 1bf940d8a03bb752604fbb366d47b97b50b9e6ce",
    "cpp/third_party/tinyxml2/CMakeLists.txt": "GIT_TAG 11.0.0",
}
EXPECTED_ARCHIVE_OPTIONS = {
    "cpp/third_party/liblzma/CMakeLists.txt": "DOWNLOAD_EXTRACT_TIMESTAMP TRUE",
    "cpp/third_party/nlohmann_json/CMakeLists.txt": "DOWNLOAD_EXTRACT_TIMESTAMP TRUE",
}
FORBIDDEN_DEPENDENCY_TEXT = {
    "cpp/third_party/liblzma/CMakeLists.txt": ("URL_MD5",),
    "cpp/third_party/mavlink/mavlink.patch": ("-m pip install", "pip-dependencies"),
    "cpp/third_party/picosha2/CMakeLists.txt": ("GIT_TAG cmake-install-support",),
}
EXPECTED_LICENSE_HASHES = {
    "Asio-Boost-1.0.txt": "beb8e42e9d6b4284e03304d05a81a0755200a965fc8d0a5e0aea1e84cf805d6e",
    "fmt-MIT.txt": "25b5db04aa6070c12ac518b91cbd0a59725c050ecb091456e6ebc3d3eeb369cf",
    "libevents-BSD-3-Clause.txt": "6f35a7c5c0d7851c6c220a09b31c1cd9f97c587b8ae6c5a46a5fce662489d30b",
    "libmavlike-BSD-3-Clause.txt": "70e26bc0c922e89a6e54d18083df24c756855b70349a0719d54f722dcc87665b",
    "MAVLink-generator-and-output.txt": "299f94fd2ac2d859ac828ae0d4b6bdea02fc7d0dae1bf8c0658589550ddad487",
    "MAVSDK-BSD-3-Clause.txt": "0ccdceb5a7215d4ab3e638040e1effdafc1197a91bc214f80d67f1c412c4aae2",
    "nlohmann-json-MIT.txt": "46a65cffd1ea955132d95a8dd921640714a8d6b537d2e4e482d31145ae95b603",
    "PicoSHA2-MIT.txt": "3e4ca187c6ffc8b0ed48f84fb06b9a44eb6fd2c15af856806fc51d44911b4496",
    "tinyxml2-Zlib.txt": "035a1f2fd2f6dba120f3c25cd2d04f9987f32405c847c0cb4859fe33e6148f45",
    "XZ-COPYING.txt": "72d7ef9c98be319fd34ce88b45203b36d5936f9c49e82bf3198ffee5e0c7d87e",
    "XZ-GPL-2.0.txt": "8177f97513213526df2cf6184d8ff986c675afb514d4e68a404010521b880643",
    "XZ-GPL-3.0.txt": "3972dc9744f6499f0f9b2dbf76696f2ae7ad8af9b23dde66d6af86c9dfb36986",
    "XZ-LGPL-2.1.txt": "dc626520dcd53a22f727af3ee42c770e56c97a64fe3adb063799d8ab032fe551",
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


def check_forbidden_text(path: Path, forbidden: tuple[str, ...]) -> None:
    contents = path.read_text(encoding="utf-8")
    found = [value for value in forbidden if value in contents]
    if found:
        values = ", ".join(found)
        raise RuntimeError(f"{path.relative_to(ROOT)} contains mutable or weak provenance: {values}")


def normalized_text_sha256(path: Path) -> str:
    """Hash text using the reviewed CRLF representation on every platform."""
    text = path.read_text(encoding="utf-8")
    canonical = text.replace("\r\n", "\n").replace("\r", "\n").replace("\n", "\r\n")
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def verify_license_bundle() -> None:
    license_root = ROOT / "licenses" / "mavsdk-phase-a"
    for name, expected in EXPECTED_LICENSE_HASHES.items():
        path = license_root / name
        if not path.is_file():
            raise RuntimeError(f"missing MAVSDK redistribution license: {path.relative_to(ROOT)}")
        actual = normalized_text_sha256(path)
        if actual != expected:
            raise RuntimeError(f"MAVSDK redistribution license changed: {name}")


def verify_provenance() -> None:
    for path, expected in EXPECTED_REVISIONS.items():
        actual = git_revision(path)
        if actual != expected:
            raise RuntimeError(f"{path} revision changed: expected {expected}, observed {actual}")

    mavsdk_root = ROOT / "third_party" / "MAVSDK"
    for relative, expected in EXPECTED_DEPENDENCIES.items():
        check_expected_text(mavsdk_root / relative, expected)
    for relative, expected in EXPECTED_ARCHIVE_OPTIONS.items():
        check_expected_text(mavsdk_root / relative, expected)
    for relative, forbidden in FORBIDDEN_DEPENDENCY_TEXT.items():
        check_forbidden_text(mavsdk_root / relative, forbidden)

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
    if "licenses/mavsdk-phase-a" not in notice:
        raise RuntimeError("NOTICE does not identify the MAVSDK redistribution license bundle")
    verify_license_bundle()


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
