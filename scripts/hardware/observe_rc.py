# SPDX-License-Identifier: Apache-2.0
"""Disarmed MAVLink read requests and receive-only, one-control RC captures."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import time
from pathlib import Path

import serial
from pymavlink.dialects.v20 import ardupilotmega as mavlink


class ObservationError(RuntimeError):
    pass


def get_code_identity() -> dict:
    root = Path(__file__).resolve().parents[2]
    revision = subprocess.run(["git", "rev-parse", "HEAD"], cwd=root, capture_output=True, text=True, check=True)
    status = subprocess.run(["git", "status", "--porcelain"], cwd=root, capture_output=True, text=True, check=True)
    return {
        "code_sha": revision.stdout.strip(),
        "source_dirty": bool(status.stdout.strip()),
        "observer_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    }


def open_port(name: str, baud: int):
    port = serial.Serial(port=None, baudrate=baud, timeout=0.1)
    port.dtr = False
    port.rts = False
    port.port = name
    port.open()
    return port


def receive(port, parser):
    return parser.parse_buffer(port.read(max(1, port.in_waiting))) or []


def discover(port, parser, timeout: float) -> tuple[int, int]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for message in receive(port, parser):
            if message.get_type() != "HEARTBEAT" or message.autopilot != mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
                continue
            validate_disarmed(message)
            return message.get_srcSystem(), message.get_srcComponent()
    raise ObservationError("No disarmed ArduPilot heartbeat; no read requests sent")


def validate_disarmed(message) -> None:
    if message.get_type() == "HEARTBEAT" and message.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        raise ObservationError("ABORT: aircraft reports armed")


def selected_messages(port, parser, target):
    messages = receive(port, parser)
    for message in messages:
        if (message.get_srcSystem(), message.get_srcComponent()) == target:
            validate_disarmed(message)
            yield message


def summarize_rc(samples: list[tuple[float, list[int], int, int]]) -> dict:
    if not samples:
        return {"samples": 0, "native_input_proven": False}
    duration = samples[-1][0] - samples[0][0]
    channels = {}
    for index in range(18):
        values = [row[1][index] for row in samples]
        distinct = sorted(set(values))
        channels[str(index + 1)] = {
            "min": min(values),
            "max": max(values),
            "last": values[-1],
            "distinct_values": distinct if len(distinct) <= 12 else None,
        }
    return {
        "samples": len(samples),
        "observed_rate_hz": (len(samples) - 1) / duration if duration > 0 else None,
        "chancount": sorted({row[2] for row in samples}),
        "rssi": sorted({row[3] for row in samples}),
        "channels": channels,
        "native_input_proven": False,
    }


def capture(port, parser, target, duration: float, label: str) -> dict:
    samples = []
    health = []
    deadline = time.monotonic() + duration
    heartbeat_at = time.monotonic()
    while time.monotonic() < deadline:
        if time.monotonic() - heartbeat_at > 3:
            raise ObservationError("ABORT: ownship heartbeat stale")
        for message in selected_messages(port, parser, target):
            now = time.monotonic()
            if message.get_type() == "HEARTBEAT":
                heartbeat_at = now
            elif message.get_type() == "RC_CHANNELS":
                values = [getattr(message, f"chan{channel}_raw") for channel in range(1, 19)]
                samples.append((now, values, message.chancount, message.rssi))
            elif message.get_type() == "SYS_STATUS":
                bit = mavlink.MAV_SYS_STATUS_SENSOR_RC_RECEIVER
                health.append(bool(message.onboard_control_sensors_health & bit))
    return {"label": label, "receiver_health_observed": sorted(set(health)), **summarize_rc(samples)}


def firmware_record(message) -> dict:
    version = message.flight_sw_version
    return {
        "version": f"{version >> 24}.{(version >> 16) & 255}.{(version >> 8) & 255}",
        "release_type": version & 255,
        "custom_version_bytes": list(message.flight_custom_version),
        "capabilities": message.capabilities,
    }


def record_parameter(message, values, indices, expected):
    if expected is not None and expected != message.param_count:
        raise ObservationError("Parameter count changed during snapshot")
    name = message.param_id
    if not 0 <= message.param_index < message.param_count:
        return expected
    previous = indices.get(message.param_index)
    if previous is not None and previous != name:
        raise ObservationError("Parameter index/name changed during snapshot")
    indices[message.param_index] = name
    values[name] = {"value": message.param_value, "type": message.param_type}
    return message.param_count


def snapshot(port, parser, target, timeout: float) -> dict:
    # These are the only outbound messages: read-list/read-index and version request.
    sender = mavlink.MAVLink(port, srcSystem=254, srcComponent=190)
    sender.param_request_list_send(*target)
    sender.command_long_send(*target, mavlink.MAV_CMD_REQUEST_MESSAGE, 0, 148, 0, 0, 0, 0, 0, 0)
    values, indices, firmware = {}, {}, None
    expected = None
    deadline = time.monotonic() + timeout
    heartbeat_at = time.monotonic()
    retry_at = time.monotonic() + timeout / 2
    while time.monotonic() < deadline:
        if time.monotonic() - heartbeat_at > 3:
            raise ObservationError("ABORT: ownship heartbeat stale")
        for message in selected_messages(port, parser, target):
            kind = message.get_type()
            if kind == "HEARTBEAT":
                heartbeat_at = time.monotonic()
            elif kind == "PARAM_VALUE":
                expected = record_parameter(message, values, indices, expected)
            elif kind == "AUTOPILOT_VERSION":
                firmware = firmware_record(message)
        if expected is not None and len(indices) == expected and len(values) == expected and firmware is not None:
            return {"complete": True, "count": expected, "firmware": firmware, "parameters": values}
        if expected is not None and time.monotonic() >= retry_at:
            missing = [index for index in range(expected) if index not in indices]
            for index in missing[:10]:
                sender.param_request_read_send(*target, b"", index)
            retry_at = time.monotonic() + 2
    raise ObservationError(f"Incomplete snapshot: {len(indices)}/{expected}; no qualification allowed")


def save_record(path: Path, record: dict) -> str:
    data = (json.dumps(record, indent=2, sort_keys=True) + "\n").encode()
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as output:
        output.write(data)
    return hashlib.sha256(data).hexdigest()


def parse_arguments():
    arguments = argparse.ArgumentParser(description=__doc__)
    arguments.add_argument("--port", required=True)
    arguments.add_argument("--baud", type=int, default=460800)
    arguments.add_argument("--duration", type=float, default=30)
    arguments.add_argument("--label", default="baseline")
    arguments.add_argument("--snapshot", action="store_true", help="Send read requests; never writes parameters")
    arguments.add_argument("--output", type=Path, required=True, help="New local evidence file, never overwritten")
    args = arguments.parse_args()
    if not 1 <= args.duration <= 180 or args.output.exists():
        arguments.error("Duration must be 1..180 seconds and output must not exist")
    return args


def main() -> int:
    args = parse_arguments()
    try:
        identity = get_code_identity()
        with open_port(args.port, args.baud) as port:
            parser = mavlink.MAVLink(None)
            parser.robust_parsing = True
            target = discover(port, parser, 8)
            record = (
                snapshot(port, parser, target, args.duration)
                if args.snapshot
                else capture(port, parser, target, args.duration, args.label)
            )
        record["source_ids"] = list(target)
        record.update(identity)
        record["disarmed_observed"] = True
        record["read_request_source_ids"] = [254, 190] if args.snapshot else None
        digest = save_record(args.output, record)
        print(
            json.dumps(
                {
                    "sha256": digest,
                    "record": record
                    if not args.snapshot
                    else {key: value for key, value in record.items() if key != "parameters"},
                },
                indent=2,
            )
        )
    except (ObservationError, serial.SerialException, OSError, subprocess.CalledProcessError) as error:
        print(f"ABORT: {error}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
