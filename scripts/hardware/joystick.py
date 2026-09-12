# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""
NOMAD Serial → Virtual Xbox 360 Bridge
======================================

Reads CSV telemetry from a microcontroller (RadioMaster passthrough,
gimbal stick MCU, etc.) over a serial port and emits a virtual Xbox 360
controller via vgamepad/ViGEmBus. Once running, Windows + Mission
Planner see it as a regular DirectInput device, so the NOMAD plugin's
NomadJoystickService can pick it up alongside any other gamepad.

Run directly:
    python jotystick.py --port COM10 --baud 115200

Or let the NOMADPlugin auto-launch it (see "Serial Bridge" section in
NOMAD Settings → Joystick).

Wire format (CSV, one frame per newline-terminated line):
    roll,pitch,yaw,throttle,gimbal_x,gimbal_y,sw1,sw2,sw3,mode[,kill]

Sticks and triggers are 0..1023 (10-bit ADC); switches are 0/1/2 (three-
position); mode is 0..3 (DPAD); kill is 0/1 (optional 11th column).
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from dataclasses import dataclass

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: python -m pip install pyserial", file=sys.stderr)
    sys.exit(2)

try:
    import vgamepad as vg
except ImportError:
    print("ERROR: vgamepad not installed. Run: python -m pip install vgamepad", file=sys.stderr)
    print(
        "       (vgamepad also needs the ViGEmBus driver: https://github.com/ViGEm/ViGEmBus/releases)", file=sys.stderr
    )
    sys.exit(2)


PRINT_EVERY = 0.5  # seconds between echoed frames when --verbose is off


@dataclass(frozen=True)
class Frame:
    """One decoded wire frame, already scaled for the virtual gamepad."""

    roll: int
    pitch: int
    yaw: int
    throttle: int
    gimbal_x: int
    gimbal_y: int
    sw1: int
    sw2: int
    sw3: int
    mode: int
    kill: int


# =========================
# SCALE FUNCTIONS
# =========================


def scale_stick(v) -> int:
    """10-bit (0..1023) → signed 16-bit (-32768..32767)."""
    try:
        v = max(0, min(1023, int(v)))
        return int((v / 1023) * 65535 - 32768)
    except (ValueError, TypeError):
        return 0


def scale_trigger(v) -> int:
    """10-bit (0..1023) → unsigned 8-bit (0..255)."""
    try:
        v = max(0, min(1023, int(v)))
        return int((v / 1023) * 255)
    except (ValueError, TypeError):
        return 0


def parse_frame(line: str) -> Frame | None:
    """Decode one CSV line into a Frame, or None when it is unusable."""
    parts = line.split(",")[:11]  # truncate any extras safely
    if len(parts) < 10:
        return None

    try:
        sw1 = int(parts[6])
        sw2 = int(parts[7])
        sw3 = int(parts[8])
        mode = int(parts[9])
    except ValueError:
        return None

    kill = 0
    if len(parts) > 10:
        try:
            kill = int(parts[10])
        except ValueError:
            kill = 0

    return Frame(
        roll=scale_stick(parts[0]),
        pitch=scale_stick(parts[1]),
        yaw=scale_trigger(parts[2]),
        throttle=scale_trigger(parts[3]),
        gimbal_x=scale_stick(parts[4]),
        gimbal_y=scale_stick(parts[5]),
        sw1=sw1,
        sw2=sw2,
        sw3=sw3,
        mode=mode,
        kill=kill,
    )


# =========================
# GAMEPAD OUTPUT
# =========================


def create_gamepad():
    """Create and neutralise the virtual gamepad before the first frame."""
    gamepad = vg.VX360Gamepad()
    # Initial neutral state so consumers don't see a stale frame from a
    # previous run (vgamepad keeps state until process exit, but a fresh
    # update() makes our intent explicit).
    gamepad.update()
    return gamepad


def set_button(gamepad, pressed: bool, button) -> None:
    if pressed:
        gamepad.press_button(button)
    else:
        gamepad.release_button(button)


def apply_mode(gamepad, mode: int) -> None:
    """Drive the DPAD from the three-position mode switch (0 = neutral)."""
    for button in (
        vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_UP,
        vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_DOWN,
        vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_LEFT,
        vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_RIGHT,
    ):
        gamepad.release_button(button)
    if mode == 1:
        gamepad.press_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_DOWN)
    elif mode == 2:
        gamepad.press_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_LEFT)
    elif mode == 3:
        gamepad.press_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_RIGHT)


def apply_frame(gamepad, frame: Frame) -> None:
    """Translate one Frame into gamepad axes, buttons, and DPAD state."""
    # Left stick = vehicle roll/pitch (flight)
    # Right stick = gimbal pan/tilt (NomadJoystickService reads Rx/Ry)
    # Left trigger = throttle, Right trigger = yaw
    # Yaw lives on a slider so it doesn't fight the gimbal axes; vgamepad
    # doesn't expose sliders directly, so yaw → right_trigger keeps it
    # reachable through DirectInput as an axis.
    gamepad.left_joystick(x_value=frame.roll, y_value=frame.pitch)
    gamepad.right_joystick(x_value=frame.gimbal_x, y_value=frame.gimbal_y)
    gamepad.left_trigger(value=frame.throttle)
    gamepad.right_trigger(value=frame.yaw)

    set_button(gamepad, frame.sw1 == 1, vg.XUSB_BUTTON.XUSB_GAMEPAD_A)
    set_button(gamepad, frame.sw1 == 2, vg.XUSB_BUTTON.XUSB_GAMEPAD_B)
    set_button(gamepad, frame.sw2 == 1, vg.XUSB_BUTTON.XUSB_GAMEPAD_X)
    set_button(gamepad, frame.sw2 == 2, vg.XUSB_BUTTON.XUSB_GAMEPAD_Y)
    set_button(gamepad, frame.sw3 == 1, vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_SHOULDER)
    set_button(gamepad, frame.sw3 == 2, vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_SHOULDER)
    set_button(gamepad, frame.kill == 1, vg.XUSB_BUTTON.XUSB_GAMEPAD_BACK)

    apply_mode(gamepad, frame.mode)
    gamepad.update()


# =========================
# SERIAL PORT
# =========================


def open_serial(port: str, baud: int):
    """Open the serial port, retrying so the plugin can launch before the radio."""
    while True:
        try:
            return serial.Serial(port, baud, timeout=0.05)
        except serial.SerialException as e:
            print(f"NOMAD bridge: serial open failed ({e}); retrying in 2s…", flush=True)
            time.sleep(2)


def prime_serial(ser) -> None:
    """Discard in-flight bytes once the port is up and the MCU has settled."""
    time.sleep(2)
    try:
        ser.reset_input_buffer()
    except Exception:
        pass


def close_serial(ser) -> None:
    try:
        ser.close()
    except Exception:
        pass


def reopen_serial(ser, port: str, baud: int):
    """Replace a failed port, waiting until the radio comes back."""
    close_serial(ser)
    while True:
        try:
            new_ser = serial.Serial(port, baud, timeout=0.05)
            time.sleep(1)
            new_ser.reset_input_buffer()
            print("NOMAD bridge: serial reopened.", flush=True)
            return new_ser
        except serial.SerialException:
            time.sleep(2)


def read_line(ser) -> str | None:
    """Return the next non-empty decoded frame, or None when there is none."""
    raw = ser.readline()
    if not raw:
        return None
    line = raw.decode(errors="ignore").strip()
    return line or None


def echo_line(line: str, verbose: bool, last_print: float) -> float:
    """Print a frame (always in verbose mode, else rate-limited) and return the new stamp."""
    if verbose:
        print(line, flush=True)
        return last_print
    now = time.monotonic()
    if now - last_print >= PRINT_EVERY:
        print(line, flush=True)
        return now
    return last_print


# =========================
# MAIN LOOP
# =========================


def run(port: str, baud: int, verbose: bool) -> int:
    gamepad = create_gamepad()
    print(f"NOMAD bridge: opening {port} @ {baud}…", flush=True)

    ser = open_serial(port, baud)
    prime_serial(ser)
    print("NOMAD bridge: bridge active. Ctrl+C to quit.", flush=True)

    last_print = 0.0
    while True:
        try:
            line = read_line(ser)
            if line is None:
                continue

            last_print = echo_line(line, verbose, last_print)
            frame = parse_frame(line)
            if frame is None:
                continue

            apply_frame(gamepad, frame)
            time.sleep(0.005)

        except serial.SerialException as e:
            print(f"NOMAD bridge: serial error ({e}); reopening…", flush=True)
            ser = reopen_serial(ser, port, baud)

        except Exception as e:
            print(f"NOMAD bridge: error: {e}", flush=True)
            time.sleep(0.05)


def main() -> int:
    ap = argparse.ArgumentParser(description="NOMAD serial → virtual Xbox 360 bridge.")
    ap.add_argument("--port", default="COM7", help="Serial port (e.g. COM10, /dev/ttyUSB0)")
    ap.add_argument("--baud", type=int, default=115200, help="Baud rate (default 115200)")
    ap.add_argument("--verbose", action="store_true", help="Print every serial frame")
    args = ap.parse_args()

    # Make Ctrl+C clean (no Python traceback dump).
    def _bye(*_):
        print("NOMAD bridge: shutting down.", flush=True)
        sys.exit(0)

    signal.signal(signal.SIGINT, _bye)
    try:
        signal.signal(signal.SIGTERM, _bye)
    except (AttributeError, ValueError):
        pass  # SIGTERM not supported on Windows for non-console sessions

    return run(args.port, args.baud, args.verbose)


if __name__ == "__main__":
    sys.exit(main())
