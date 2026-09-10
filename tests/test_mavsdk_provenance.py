# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tests for the MAVSDK Phase A provenance gate."""

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


def test_redistribution_license_bundle_is_complete() -> None:
    check_mavsdk_phase_a.verify_license_bundle()
