# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tests for the MAVSDK Phase A provenance gate."""

from pathlib import Path

from scripts.dev import check_mavsdk_phase_a


def test_reviewed_mavsdk_provenance_is_current() -> None:
    check_mavsdk_phase_a.verify_provenance()


def test_phase_a_archives_use_strong_hashes() -> None:
    mavsdk_root = check_mavsdk_phase_a.ROOT / "third_party" / "MAVSDK"
    liblzma = (mavsdk_root / "cpp/third_party/liblzma/CMakeLists.txt").read_text(encoding="utf-8")
    json = (mavsdk_root / "cpp/third_party/nlohmann_json/CMakeLists.txt").read_text(encoding="utf-8")

    assert "URL_HASH SHA256=" in liblzma
    assert "URL_MD5" not in liblzma
    assert "DOWNLOAD_EXTRACT_TIMESTAMP TRUE" in liblzma
    assert "URL_HASH SHA256=" in json
    assert "DOWNLOAD_EXTRACT_TIMESTAMP TRUE" in json


def test_picosha2_uses_immutable_revision() -> None:
    source = (check_mavsdk_phase_a.ROOT / "third_party/MAVSDK/cpp/third_party/picosha2/CMakeLists.txt").read_text(
        encoding="utf-8"
    )

    assert "GIT_TAG 1bf940d8a03bb752604fbb366d47b97b50b9e6ce" in source
    assert "GIT_TAG cmake-install-support" not in source


def test_mavlink_generator_avoids_build_time_package_resolution() -> None:
    patch_path = check_mavsdk_phase_a.ROOT / "third_party/MAVSDK/cpp/third_party/mavlink/mavlink.patch"
    additions = check_mavsdk_phase_a.patch_added_text(patch_path)

    assert '"PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}"' in additions
    assert "-m pip install" not in additions
    assert "pip-dependencies" not in additions


def test_license_hash_ignores_checkout_text_conventions(tmp_path: Path) -> None:
    license_path = tmp_path / "license.txt"
    variants = (
        b"first line\nsecond line",
        b"first line\nsecond line\n",
        b"first line\r\nsecond line",
        b"first line\r\nsecond line\r\n",
    )
    hashes: set[frozenset[str]] = set()
    for contents in variants:
        license_path.write_bytes(contents)
        hashes.add(frozenset(check_mavsdk_phase_a.text_sha256_variants(license_path)))

    assert len(hashes) == 1


def test_license_hash_still_detects_content_changes(tmp_path: Path) -> None:
    license_path = tmp_path / "license.txt"
    license_path.write_text("first line\nsecond line\n", encoding="utf-8")
    expected = check_mavsdk_phase_a.text_sha256_variants(license_path)

    license_path.write_text("first line\nchanged line\n", encoding="utf-8")

    assert expected.isdisjoint(check_mavsdk_phase_a.text_sha256_variants(license_path))


def test_redistribution_license_bundle_is_complete() -> None:
    check_mavsdk_phase_a.verify_license_bundle()
