# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tests for the source-size gate's baseline freshness and line-width checks.

Both baselines are hand-maintained, so an entry that stops pointing at an
oversized target silently widens the gate. These tests pin that a deleted file,
a shrunken file, or a renamed function is reported, and that the column check
fires for C/C++ without touching Python (ruff already owns Python).
"""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

from scripts.dev import line_report

CPP_SOURCE = "src/big.cpp"
PYTHON_SOURCE = "scripts/tool.py"
FILE_BASELINE = "config/file_size_baseline.txt"
FUNCTION_BASELINE = "config/function_size_baseline.txt"
LINE_BASELINE = "config/line_length_baseline.txt"


def _args(**overrides) -> SimpleNamespace:
    values = {
        "max_file_lines": 500,
        "max_function_lines": 40,
        "max_line_length": 120,
        "baseline_file": Path(FILE_BASELINE),
        "baseline_function_file": Path(FUNCTION_BASELINE),
        "baseline_line_file": Path(LINE_BASELINE),
        "fail_over_file_limit": False,
        "fail_over_function_limit": False,
        "fail_line_length": False,
        "fail_stale_baseline": False,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def _write(root: Path, relative: str, content: str) -> None:
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def _function_source(name: str, body_lines: int) -> str:
    body = (f"    step_{index} = {index}" for index in range(body_lines))
    return "\n".join([f"def {name}():", *body, ""])


def test_fresh_baseline_entries_pass(tmp_path: Path):
    _write(tmp_path, CPP_SOURCE, "int main() {}\n" * 501)
    _write(tmp_path, PYTHON_SOURCE, _function_source("big", 40))
    _write(tmp_path, FILE_BASELINE, f"{CPP_SOURCE}\n")
    _write(tmp_path, FUNCTION_BASELINE, f"{PYTHON_SOURCE}:big\n")

    assert line_report.find_stale_baseline_entries(_args(), tmp_path) == []


def test_stale_baseline_entries_are_reported(tmp_path: Path):
    _write(tmp_path, PYTHON_SOURCE, _function_source("tiny", 1))
    _write(tmp_path, FILE_BASELINE, f"{PYTHON_SOURCE}\nsrc/missing.cpp\n")
    _write(tmp_path, FUNCTION_BASELINE, f"{PYTHON_SOURCE}:tiny\n{PYTHON_SOURCE}:ghost\n")

    messages = line_report.find_stale_baseline_entries(_args(), tmp_path)

    assert f"{PYTHON_SOURCE}: at or under the 500-line limit" in messages
    assert "src/missing.cpp: no such file" in messages
    assert f"{PYTHON_SOURCE}:tiny: at or under the 40-line limit" in messages
    assert f"{PYTHON_SOURCE}:ghost: no such function" in messages


def test_baseline_entry_without_function_name_is_stale(tmp_path: Path):
    _write(tmp_path, FUNCTION_BASELINE, f"{PYTHON_SOURCE}\n")

    messages = line_report.find_stale_baseline_entries(_args(), tmp_path)

    assert f"{PYTHON_SOURCE}: expected a path:function entry" in messages


def test_scan_file_collects_every_over_length_line(tmp_path: Path):
    _write(tmp_path, "src/lines.cpp", "short\n" + "a" * 121 + "\n" + "b" * 130 + "\n")

    size, long_lines = line_report.scan_file(tmp_path / "src/lines.cpp", tmp_path, 120)

    assert size.lines == 3
    assert [(item.number, item.length) for item in long_lines] == [(2, 121), (3, 130)]


def test_line_length_gate_fails_on_cpp(tmp_path: Path):
    report = line_report.Report([], [line_report.LongLine(Path("src/long.cpp"), 3, 121, "x")], [])

    assert line_report.enforce_report(report, _args(fail_line_length=True), tmp_path) == 1


def test_line_length_gate_ignores_python(tmp_path: Path):
    report = line_report.Report([], [line_report.LongLine(Path("scripts/long.py"), 3, 121, "x")], [])

    assert line_report.enforce_report(report, _args(fail_line_length=True), tmp_path) == 0


def test_line_length_gate_tolerates_baselined_files_only(tmp_path: Path):
    _write(tmp_path, LINE_BASELINE, f"{CPP_SOURCE}\n")
    report = line_report.Report(
        [],
        [
            line_report.LongLine(Path(CPP_SOURCE), 3, 121, "x"),
            line_report.LongLine(Path("src/other.cpp"), 7, 130, "y"),
        ],
        [],
    )

    assert line_report.enforce_report(report, _args(fail_line_length=True), tmp_path) == 1


def test_line_length_gate_passes_with_only_baselined_files(tmp_path: Path):
    _write(tmp_path, LINE_BASELINE, f"{CPP_SOURCE}\n")
    report = line_report.Report([], [line_report.LongLine(Path(CPP_SOURCE), 3, 121, "x")], [])

    assert line_report.enforce_report(report, _args(fail_line_length=True), tmp_path) == 0


def test_line_length_baseline_entry_needs_a_long_line(tmp_path: Path):
    # The entry outlives its reason once the file is deleted or its lines are
    # wrapped, and the gate must not keep tolerating either case.
    _write(tmp_path, "src/wrapped.cpp", "short\n")
    _write(tmp_path, LINE_BASELINE, "src/wrapped.cpp\nsrc/deleted.cpp\n")

    messages = line_report.find_stale_baseline_entries(_args(), tmp_path)

    assert "src/wrapped.cpp: no line over 120 characters left to tolerate" in messages
    assert "src/deleted.cpp: no such file" in messages
