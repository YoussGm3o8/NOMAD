# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Offline tests for the serial -> virtual Xbox bridge frame logic.

The bridge is hardware-only at runtime (pyserial + vgamepad on Windows), so the
fake ``serial``/``vgamepad`` modules are injected before import. That keeps the
frame decoding and gamepad application testable without a port or a driver.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

JOYSTICK_PATH = Path(__file__).resolve().parent.parent / "scripts" / "hardware" / "joystick.py"


class FakeSerialException(Exception):
    pass


class FakeGamepad:
    """Record every gamepad call in order instead of driving ViGEmBus."""

    def __init__(self):
        self.calls = []

    def update(self):
        self.calls.append(("update",))

    def left_joystick(self, x_value, y_value):
        self.calls.append(("left_joystick", x_value, y_value))

    def right_joystick(self, x_value, y_value):
        self.calls.append(("right_joystick", x_value, y_value))

    def left_trigger(self, value):
        self.calls.append(("left_trigger", value))

    def right_trigger(self, value):
        self.calls.append(("right_trigger", value))

    def press_button(self, button):
        self.calls.append(("press", button))

    def release_button(self, button):
        self.calls.append(("release", button))


class FakeButton:
    XUSB_GAMEPAD_A = "A"
    XUSB_GAMEPAD_B = "B"
    XUSB_GAMEPAD_X = "X"
    XUSB_GAMEPAD_Y = "Y"
    XUSB_GAMEPAD_LEFT_SHOULDER = "LS"
    XUSB_GAMEPAD_RIGHT_SHOULDER = "RS"
    XUSB_GAMEPAD_BACK = "BACK"
    XUSB_GAMEPAD_DPAD_UP = "UP"
    XUSB_GAMEPAD_DPAD_DOWN = "DOWN"
    XUSB_GAMEPAD_DPAD_LEFT = "LEFT"
    XUSB_GAMEPAD_DPAD_RIGHT = "RIGHT"


def load_bridge(monkeypatch):
    """Import the bridge with fake hardware modules so import never calls sys.exit."""
    monkeypatch.setitem(sys.modules, "serial", SimpleNamespace(SerialException=FakeSerialException))
    monkeypatch.setitem(sys.modules, "vgamepad", SimpleNamespace(VX360Gamepad=FakeGamepad, XUSB_BUTTON=FakeButton))
    spec = importlib.util.spec_from_file_location("nomad_joystick_bridge", JOYSTICK_PATH)
    module = importlib.util.module_from_spec(spec)
    # Register before exec: dataclass resolves annotations through sys.modules.
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    return module


def test_parse_frame_scales_sticks_and_triggers(monkeypatch):
    bridge = load_bridge(monkeypatch)

    frame = bridge.parse_frame("0,1023,1023,512,1023,0,1,2,0,3")

    assert frame == bridge.Frame(
        roll=-32768,
        pitch=32767,
        yaw=255,
        throttle=127,
        gimbal_x=32767,
        gimbal_y=-32768,
        sw1=1,
        sw2=2,
        sw3=0,
        mode=3,
        kill=0,
    )


def test_parse_frame_rejects_short_and_unparsable_lines(monkeypatch):
    bridge = load_bridge(monkeypatch)

    assert bridge.parse_frame("1,2,3") is None
    assert bridge.parse_frame("0,0,0,0,0,0,x,0,0,0") is None


def test_parse_frame_defaults_a_malformed_kill_column(monkeypatch):
    bridge = load_bridge(monkeypatch)

    frame = bridge.parse_frame("0,0,0,0,0,0,0,0,0,0,notanint")
    assert frame is not None
    assert frame.kill == 0

    assert bridge.parse_frame("0,0,0,0,0,0,0,0,0,0,1").kill == 1


def test_apply_frame_drives_axes_buttons_and_dpad(monkeypatch):
    bridge = load_bridge(monkeypatch)
    gamepad = FakeGamepad()
    frame = bridge.Frame(
        roll=-32768,
        pitch=32767,
        yaw=255,
        throttle=127,
        gimbal_x=32767,
        gimbal_y=-32768,
        sw1=1,
        sw2=2,
        sw3=0,
        mode=1,
        kill=1,
    )

    bridge.apply_frame(gamepad, frame)

    assert ("left_joystick", -32768, 32767) in gamepad.calls
    assert ("right_joystick", 32767, -32768) in gamepad.calls
    assert ("left_trigger", 127) in gamepad.calls
    assert ("right_trigger", 255) in gamepad.calls
    assert ("press", "A") in gamepad.calls
    assert ("press", "Y") in gamepad.calls
    assert ("press", "BACK") in gamepad.calls
    assert ("press", "DOWN") in gamepad.calls
    assert gamepad.calls[-1] == ("update",)


def test_neutral_mode_releases_every_dpad_direction(monkeypatch):
    bridge = load_bridge(monkeypatch)
    gamepad = FakeGamepad()
    frame = bridge.Frame(
        roll=0, pitch=0, yaw=0, throttle=0, gimbal_x=0, gimbal_y=0, sw1=0, sw2=0, sw3=0, mode=0, kill=0
    )

    bridge.apply_frame(gamepad, frame)

    for direction in ("UP", "DOWN", "LEFT", "RIGHT"):
        assert ("release", direction) in gamepad.calls
        assert ("press", direction) not in gamepad.calls


def test_read_line_skips_empty_reads(monkeypatch):
    bridge = load_bridge(monkeypatch)
    reads = iter([b"", b"\n", b"1,2,3,4,5,6,0,0,0,0\n"])
    reader = SimpleNamespace(readline=lambda: next(reads, b""))

    assert bridge.read_line(reader) is None
    assert bridge.read_line(reader) is None
    assert bridge.read_line(reader) == "1,2,3,4,5,6,0,0,0,0"
