"""``arm101 calibrate-motor`` — identify a single connected motor and catalog it.

Before an SO-101 is assembled, each Feetech servo is connected to the computer
**one at a time** and identified.  This verb detects the single connected
STS3215 (auto-skipping busy or non-motor serial ports so it never grabs an
unrelated device such as a Reachy daemon), shows the operator the motor's full
read-only register snapshot, then records three operator-supplied spec fields —
**Servo Model**, **Gear Ratio**, and **Corresponding Joint** — keyed by a motor
label (``F1``..``F6`` follower, ``L1``..``L6`` leader) into the motor catalog
(:mod:`arm101.hardware.motor_catalog`).

It is **read-only on the motor**: it pings and reads registers, never enabling
torque, commanding motion, or writing EEPROM.  The human is gated at every input
(label + the three fields); in ``--auto`` mode the CLI also gates on connecting
each motor in turn.

Two modes
---------
* **manual** (default): register the one motor currently connected.  The label
  is taken from the optional positional argument or prompted for.
* **automatic** (``--auto``): walk ``F1``..``F6`` then ``L1``..``L6``, prompting
  the operator to connect each motor before registering it.

Bus injection seam
------------------
:func:`_open_bus` is monkeypatched in tests to inject a
:class:`~arm101.hardware.bus.FakeBus` without physical hardware.
"""

from __future__ import annotations

import argparse
import os
import sys

from arm101.cli._errors import EXIT_ENV_ERROR, EXIT_USER_ERROR, CliError
from arm101.cli._output import emit_diagnostic, emit_result
from arm101.hardware import ports
from arm101.hardware.bus import BAUD_INDEX_TO_BPS, FeetechBus, MotorBus
from arm101.hardware.motor_catalog import MotorEntry, catalog_path, save_entry

#: STS3215 model number — used to confirm a responding device is really a servo.
_STS3215_MODEL = 777

#: Automatic-mode walk order: follower F1..F6 then leader L1..L6.
_AUTO_LABELS = [f"F{i}" for i in range(1, 7)] + [f"L{i}" for i in range(1, 7)]


# ---------------------------------------------------------------------------
# Bus injection seam (monkeypatched in tests)
# ---------------------------------------------------------------------------


def _open_bus(port: str) -> MotorBus:
    """Open and return a :class:`FeetechBus` for *port* (tests patch this)."""
    bus = FeetechBus(port)
    bus.open()
    return bus


# ---------------------------------------------------------------------------
# Stdin prompting
# ---------------------------------------------------------------------------


def _prompt(message: str, *, default: str | None = None, required: bool = False) -> str:
    """Write *message* to stderr, read one line from stdin, return the answer.

    EOF (no input) raises ``CliError(EXIT_ENV_ERROR)`` — this command is
    interactive and must not hang or silently proceed.  A blank answer falls
    back to *default*; if *required* and still blank, raises a user error.
    """
    suffix = f" [{default}]" if default else ""
    emit_diagnostic(f"{message}{suffix}: ")
    line = sys.stdin.readline()
    if line == "":  # EOF
        raise CliError(
            code=EXIT_ENV_ERROR,
            message="No input available; calibrate-motor is interactive.",
            remediation="Run it in a terminal and answer the prompts (or pipe answers via stdin).",
        )
    answer = line.strip()
    if not answer and default is not None:
        return default
    if not answer and required:
        raise CliError(
            code=EXIT_USER_ERROR,
            message=f"{message} is required.",
            remediation="Re-run and provide a value.",
        )
    return answer


# ---------------------------------------------------------------------------
# Port + motor detection
# ---------------------------------------------------------------------------


def _candidate_ports() -> list[str]:
    """Real (symlink-resolved, de-duplicated) candidate serial ports."""
    canon: list[str] = []
    seen: set[str] = set()
    for p in ports.enumerate_ports():  # raises CliError on macOS/Windows
        real = os.path.realpath(p)
        if real not in seen:
            seen.add(real)
            canon.append(real)
    return canon


def _model_of(bus: MotorBus, motor: int) -> int | None:
    """Return *motor*'s model number, or None if it cannot be read."""
    try:
        return int(bus.read_info(motor).get("model", -1))
    except CliError:
        return None


def _detect_one_motor(args: argparse.Namespace) -> tuple[MotorBus, str, int]:
    """Return an open bus, its port, and the single STS3215 motor ID found.

    Ports that cannot be opened (busy — e.g. another robot's daemon) are
    skipped, so an unrelated device never blocks detection.  Exactly one motor
    on exactly one port is required; anything else is a clear CliError.
    """
    target = getattr(args, "port", None)
    ports_to_try = [os.path.realpath(target)] if target else _candidate_ports()

    matches: list[tuple[MotorBus, str, list[int]]] = []
    for port in ports_to_try:
        try:
            bus = _open_bus(port)
        except CliError:
            continue  # busy / unopenable — skip (ignores other devices)
        sts_ids = [i for i in bus.scan() if _model_of(bus, i) == _STS3215_MODEL]
        if sts_ids:
            matches.append((bus, port, sts_ids))
        else:
            bus.close()

    if not matches:
        raise CliError(
            code=EXIT_ENV_ERROR,
            message="No STS3215 servo detected on any serial port.",
            remediation=(
                "Connect the motor and its external power (STS3215 needs V+, not USB "
                "alone), then retry. Use --port to target a specific device."
            ),
        )
    if len(matches) > 1:
        found_ports = [p for _, p, _ in matches]
        for bus, _, _ in matches:
            bus.close()
        raise CliError(
            code=EXIT_USER_ERROR,
            message=f"Motors found on multiple ports: {found_ports}.",
            remediation="Connect one motor at a time, or pass --port to choose.",
        )

    bus, port, ids = matches[0]
    if len(ids) != 1:
        bus.close()
        raise CliError(
            code=EXIT_USER_ERROR,
            message=f"Expected exactly one motor on {port}, found ids {ids}.",
            remediation="Connect a single motor at a time for per-motor registration.",
        )
    return bus, port, ids[0]


# ---------------------------------------------------------------------------
# Presentation
# ---------------------------------------------------------------------------


def _show_info(info: dict[str, int], port: str) -> None:
    """Emit the full read-only register snapshot to stderr for operator review."""
    pos = info["present_position"]
    verified = (
        "Feetech STS3215 (model 777) ✓"
        if info["model"] == _STS3215_MODEL
        else f"UNKNOWN servo (model {info['model']}) ✗"
    )
    baud_idx = info["baud_index"]
    baud_bps = BAUD_INDEX_TO_BPS.get(baud_idx)
    baud_str = (
        f"{baud_bps:,} bps (index {baud_idx})"
        if baud_bps is not None
        else f"unknown (index {baud_idx})"
    )
    lines = [
        "Detected motor (read-only — no torque, motion, or EEPROM writes):",
        f"  port             : {port}",
        f"  verified         : {verified}",
        f"  id               : {info['id']}",
        f"  model            : {info['model']}",
        f"  firmware         : {info['firmware_major']}.{info['firmware_minor']}",
        f"  baudrate         : {baud_str}",
        f"  present position : {pos} (~{pos * 360.0 / 4096.0:.1f} deg)",
        f"  angle limits     : {info['min_angle']}..{info['max_angle']}",
        f"  torque enable    : {'ON' if info['torque_enable'] else 'OFF'}",
        f"  voltage          : {info['present_voltage'] / 10.0:.1f} V",
        f"  temperature      : {info['present_temperature']} C",
        f"  load / speed     : {info['present_load']} / {info['present_speed']}",
    ]
    emit_diagnostic("\n".join(lines))


def _entry_payload(entry: MotorEntry, info: dict[str, int]) -> dict:
    return {
        "label": entry.label,
        "servo_model": entry.servo_model,
        "gear_ratio": entry.gear_ratio,
        "joint": entry.joint,
        "port": entry.port,
        "recorded": entry.recorded,
        "detected": {
            "id": info["id"],
            "model": info["model"],
            "firmware": f"{info['firmware_major']}.{info['firmware_minor']}",
            "voltage_v": round(info["present_voltage"] / 10.0, 1),
            "temperature_c": info["present_temperature"],
            "present_position": info["present_position"],
        },
    }


# ---------------------------------------------------------------------------
# Registration of one motor
# ---------------------------------------------------------------------------


def _register_one(args: argparse.Namespace, label: str | None) -> dict:
    """Detect, display, gate on operator input, and catalog one motor."""
    bus, port, motor_id = _detect_one_motor(args)
    try:
        info = bus.read_info(motor_id)
    finally:
        bus.close()

    _show_info(info, port)

    if not label:
        label = _prompt("Which motor is this? (e.g. F1, L2)", required=True)

    detected_model = f"STS3215 (model {info['model']})"
    servo_model = _prompt("Servo Model", default=detected_model)
    gear_ratio = _prompt("Gear Ratio (e.g. 1:191)", required=True)
    joint = _prompt("Corresponding Joint (e.g. shoulder_pan)", required=True)

    entry = save_entry(
        MotorEntry(
            label=label,
            servo_model=servo_model,
            gear_ratio=gear_ratio,
            joint=joint,
            detected_id=int(info["id"]),
            detected_model=int(info["model"]),
            port=port,
        )
    )
    return _entry_payload(entry, info)


# ---------------------------------------------------------------------------
# Command handler
# ---------------------------------------------------------------------------


def cmd_calibrate_motor(args: argparse.Namespace) -> None:
    """Manual (one motor) or automatic (``--auto``, F1..F6 then L1..L6) registration."""
    json_mode = bool(getattr(args, "json", False))
    results: list[dict] = []

    if getattr(args, "auto", False):
        for label in _AUTO_LABELS:
            answer = _prompt(
                f"Connect the {label} motor ONLY, then press Enter (or 's' to skip, 'q' to finish)"
            )
            if answer.lower() == "q":
                break
            if answer.lower() == "s":
                emit_diagnostic(f"Skipped {label}.")
                continue
            results.append(_register_one(args, label))
    else:
        results.append(_register_one(args, getattr(args, "label", None)))

    _emit_results(results, json_mode=json_mode)


def _emit_results(results: list[dict], *, json_mode: bool) -> None:
    path = str(catalog_path())
    if json_mode:
        emit_result({"motors": results, "catalog": path}, json_mode=True)
        return
    lines = []
    for r in results:
        lines.append(
            f"Registered {r['label']}: {r['servo_model']} | gear {r['gear_ratio']} "
            f"| joint {r['joint']} (motor id {r['detected']['id']})"
        )
    if not lines:
        lines.append("No motors registered.")
    lines.append(f"Catalog: {path}")
    emit_result("\n".join(lines), json_mode=False)


# ---------------------------------------------------------------------------
# Registration
# ---------------------------------------------------------------------------


def register(sub: "argparse._SubParsersAction") -> None:
    """Register the ``calibrate-motor`` subcommand on *sub*."""
    p = sub.add_parser(
        "calibrate-motor",
        help="Identify a single connected motor (read-only) and record its model/gear/joint.",
    )
    p.add_argument(
        "label",
        nargs="?",
        help="Motor label, e.g. F1 or L2 (manual mode). Omit to be prompted.",
    )
    p.add_argument(
        "--auto",
        action="store_true",
        help="Automatic mode: walk F1..F6 then L1..L6, prompting to connect each.",
    )
    p.add_argument(
        "--port",
        default=None,
        help="Serial port of the motor (default: auto-detect, skipping busy/non-motor ports).",
    )
    p.add_argument("--json", action="store_true", help="Emit structured JSON.")
    p.set_defaults(func=cmd_calibrate_motor)
