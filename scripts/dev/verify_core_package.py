#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Verify the contents and offline CLI smoke of a staged NOMAD core package."""

from __future__ import annotations

import argparse
import subprocess
import sys
import tarfile
import tempfile
import zipfile
from pathlib import Path, PurePosixPath

REQUIRED_FILES = (
    Path("include/nomad/vehicle/vehicle.hpp"),
    Path("share/nomad/LICENSE"),
    Path("share/nomad/NOTICE"),
    Path("share/nomad/config/README.md"),
    Path("share/nomad/config/nomad.env.example"),
)


def find_install_root(base: Path) -> Path:
    """Find the install tree whether an archive adds a top-level directory."""
    if (base / "bin").is_dir():
        return base
    candidates = [item for item in base.iterdir() if item.is_dir() and (item / "bin").is_dir()]
    if len(candidates) != 1:
        raise ValueError(f"expected one package root under {base}, found {len(candidates)}")
    return candidates[0]


def validate_install_root(root: Path) -> list[str]:
    """Return content errors without executing anything."""
    errors = [f"missing {path}" for path in REQUIRED_FILES if not (root / path).is_file()]
    binary = next((root / "bin" / name for name in ("nomad", "nomad.exe") if (root / "bin" / name).is_file()), None)
    if binary is None:
        errors.append("missing bin/nomad or bin/nomad.exe")
    license_dir = root / "share/nomad/licenses/mavsdk-phase-a"
    if not license_dir.is_dir() or not any(license_dir.glob("*.txt")):
        errors.append("missing MAVSDK dependency license bundle")
    if (root / "include/mavsdk").exists():
        errors.append("package contains MAVSDK development headers")
    if (root / "lib").exists():
        errors.append("package contains development libraries")
    if (root / "share/nomad/config/nomad.env").exists():
        errors.append("package contains live config/nomad.env")
    return errors


def verify_cli(root: Path) -> list[str]:
    """Run only the no-network usage path of the installed executable."""
    binary = next(root / "bin" / name for name in ("nomad", "nomad.exe") if (root / "bin" / name).is_file())
    result = subprocess.run([str(binary.resolve())], cwd=root, capture_output=True, text=True, timeout=10, check=False)
    errors = []
    if result.returncode != 1:
        errors.append(f"usage invocation returned {result.returncode}, expected 1")
    if "Usage: nomad <" not in result.stdout:
        errors.append("usage invocation did not print the NOMAD command list")
    return errors


def safe_archive_member(name: str) -> bool:
    path = PurePosixPath(name)
    return not path.is_absolute() and ".." not in path.parts


def extract_archive(archive: Path, destination: Path) -> Path:
    """Extract a CPack ZIP/TGZ while rejecting path traversal and links."""
    destination.mkdir(parents=True, exist_ok=True)
    if archive.suffix.lower() == ".zip":
        with zipfile.ZipFile(archive) as package:
            members = package.infolist()
            if any(not safe_archive_member(member.filename) for member in members):
                raise ValueError("archive contains an unsafe ZIP path")
            package.extractall(destination)
    elif archive.name.endswith((".tar.gz", ".tgz")):
        with tarfile.open(archive) as package:
            members = package.getmembers()
            if any(not safe_archive_member(member.name) or member.issym() or member.islnk() for member in members):
                raise ValueError("archive contains an unsafe TAR member")
            for member in members:
                package.extract(member, destination, filter="data")
    else:
        raise ValueError("package must be an install directory, ZIP, TGZ or tar.gz archive")
    return find_install_root(destination)


def package_inputs(path: Path) -> list[Path]:
    """Resolve an install tree, archive, or CPack output directory."""
    if path.is_file() or (path.is_dir() and (path / "bin").is_dir()):
        return [path]
    archives = sorted(
        item
        for item in path.iterdir()
        if item.is_file() and (item.suffix.lower() == ".zip" or item.name.endswith((".tar.gz", ".tgz")))
    )
    if not archives:
        raise ValueError(f"no install tree or CPack archive found at {path}")
    return archives


def verify_package(path: Path) -> list[str]:
    """Verify an install directory or extracted CPack archive."""
    with tempfile.TemporaryDirectory(prefix="nomad-core-package-") as temporary:
        errors = []
        for index, package in enumerate(package_inputs(path.resolve())):
            root = package if package.is_dir() else extract_archive(package, Path(temporary) / str(index))
            package_errors = validate_install_root(root)
            if not package_errors:
                package_errors.extend(verify_cli(root))
            errors.extend(f"{package.name}: {error}" for error in package_errors)
        return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("package", type=Path, help="cmake --install prefix or CPack ZIP/TGZ archive")
    arguments = parser.parse_args()
    try:
        errors = verify_package(arguments.package)
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        print(f"core package verification failed: {error}", file=sys.stderr)
        return 1
    if errors:
        for error in errors:
            print(f"core package verification failed: {error}", file=sys.stderr)
        return 1
    print(f"core package verified: {arguments.package}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
