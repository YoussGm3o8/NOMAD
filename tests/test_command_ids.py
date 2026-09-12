# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Pin the MAV_CMD ids the vehicle layer puts on the wire to the pinned dialect.

``src/vehicle/vehicle.cpp`` and ``src/vehicle/output.cpp`` hand-type the MAV_CMD
identifiers they send. An id that no autopilot handler matches fails silently
rather than loudly: the command is reported as sent while the vehicle does
nothing. C23 was exactly that — motor test sent 139, which is not a MAV_CMD
entry at all, while ``MAV_CMD_DO_MOTOR_TEST`` is 209.

The dialect read here is the pinned ``third_party/ardupilot-mavlink``
submodule's own definition, the same source ``scripts/dev/generate_mavlink.py``
generates the codec from, so an id that drifts from the targeted firmware fails
the gate.
"""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
DIALECT_COMMANDS_XML = REPO_ROOT / "third_party" / "ardupilot-mavlink" / "message_definitions" / "v1.0" / "common.xml"

# Every hand-typed MAV_CMD id the vehicle layer sends, and the dialect entry it
# must equal. test_sources_declare_only_pinned_command_ids fails when a source
# gains a constant that is missing here, so new commands cannot skip this pin.
PINNED_COMMAND_IDS = {
    "src/vehicle/vehicle.cpp": {
        "kArmDisarmCommand": "MAV_CMD_COMPONENT_ARM_DISARM",
        "kSetModeCommand": "MAV_CMD_DO_SET_MODE",
        "kTakeoffCommand": "MAV_CMD_NAV_TAKEOFF",
        "kLandCommand": "MAV_CMD_NAV_LAND",
        "kReturnToLaunchCommand": "MAV_CMD_NAV_RETURN_TO_LAUNCH",
        "kRepositionCommand": "MAV_CMD_DO_REPOSITION",
        "kSetRelayCommand": "MAV_CMD_DO_SET_RELAY",
    },
    "src/vehicle/output.cpp": {
        "kSetServoCommand": "MAV_CMD_DO_SET_SERVO",
        "kSetRelayCommand": "MAV_CMD_DO_SET_RELAY",
        "kMotorTestCommand": "MAV_CMD_DO_MOTOR_TEST",
        "kMountConfigureCommand": "MAV_CMD_DO_MOUNT_CONFIGURE",
        "kUserCommand": "MAV_CMD_USER_1",
    },
}

_COMMAND_ID_CONSTANT = re.compile(r"constexpr\s+std::uint16_t\s+(k\w+)\s*=\s*(\d+)\s*;")


def _dialect_command_ids() -> dict[str, int]:
    """Return every MAV_CMD name -> value from the pinned dialect definition."""
    root = ET.parse(DIALECT_COMMANDS_XML).getroot()
    for enum in root.iter("enum"):
        if enum.get("name") == "MAV_CMD":
            return {entry.get("name"): int(entry.get("value")) for entry in enum.findall("entry")}
    raise AssertionError(f"no MAV_CMD enum in {DIALECT_COMMANDS_XML}")


def _declared_command_ids(source: str) -> dict[str, int]:
    """Return constant name -> declared value for one source file."""
    text = (REPO_ROOT / source).read_text(encoding="utf-8")
    return {name: int(value) for name, value in _COMMAND_ID_CONSTANT.findall(text)}


def _pinned_cases() -> list[tuple[str, str, str]]:
    return [
        (source, constant, command)
        for source, constants in PINNED_COMMAND_IDS.items()
        for constant, command in constants.items()
    ]


DIALECT_COMMAND_IDS = _dialect_command_ids()


def test_pinned_names_are_dialect_commands():
    # Guards the table itself: a typo in a pinned name must not pass quietly.
    missing = sorted({command for _, _, command in _pinned_cases() if command not in DIALECT_COMMAND_IDS})
    assert not missing, f"pinned names are not MAV_CMD entries in the dialect: {missing}"


@pytest.mark.parametrize(("source", "constant", "command"), _pinned_cases())
def test_command_id_matches_the_dialect(source: str, constant: str, command: str) -> None:
    declared = _declared_command_ids(source)
    assert constant in declared, f"{constant} is not declared in {source}"
    assert declared[constant] == DIALECT_COMMAND_IDS[command], (
        f"{source}:{constant} is {declared[constant]}, but {command} is {DIALECT_COMMAND_IDS[command]}"
    )


@pytest.mark.parametrize("source", sorted(PINNED_COMMAND_IDS))
def test_sources_declare_only_pinned_command_ids(source: str) -> None:
    declared = set(_declared_command_ids(source))
    pinned = set(PINNED_COMMAND_IDS[source])
    unpinned = sorted(declared - pinned)
    assert not unpinned, (
        f"{source} declares unpinned command ids {unpinned}; "
        "add each one to PINNED_COMMAND_IDS with the MAV_CMD entry it must equal"
    )
    assert not pinned - declared, f"{source} no longer declares {sorted(pinned - declared)}"
