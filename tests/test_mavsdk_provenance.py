# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tests for the MAVSDK Phase A provenance gate."""

from scripts.dev import check_mavsdk_phase_a


def test_reviewed_mavsdk_provenance_is_current() -> None:
    check_mavsdk_phase_a.verify_provenance()
