"""``arm101 setup-motors`` — one-motor-at-a-time EEPROM id/baudrate assignment.

Mirrors the lerobot ``setup-motors`` workflow: walks the arm joints from gripper
(id 6) down to shoulder_pan (id 1), prompting the operator to connect each motor
alone before writing its EEPROM id and baudrate.

Safety invariants
-----------------
* The TTY check runs **before** the bus is opened and before any prompt — a
  non-interactive invocation is rejected immediately (exit 2).
* Each EEPROM write is gated on the operator pressing Enter.  The readline()
  call blocks synchronously; no write ever precedes its prompt.
* On success the result summary goes to stdout; all operator prompts go to
  stderr.  Both text and ``--json`` honour this split.

Bus injection seam
------------------
``_open_bus(args)`` is a module-level factory the test suite monkeypatches to
return a :class:`~arm101.hardware.bus.FakeBus` without touching hardware.
"""

from __future__ import annotations

import argparse
import sys

from arm101.cli._errors import EXIT_ENV_ERROR, CliError
from arm101.cli._output import emit_diagnostic, emit_result
from arm101.hardware.bus import FeetechBus, MotorBus

# ---------------------------------------------------------------------------
# Motor walk order: gripper (6) → shoulder_pan (1)
# ---------------------------------------------------------------------------

_MOTOR_ORDER: list[tuple[int, str]] = [
    (6, "gripper"),
    (5, "wrist_roll"),
    (4, "wrist_flex"),
    (3, "elbow_flex"),
    (2, "shoulder_lift"),
    (1, "shoulder_pan"),
]

_DEFAULT_PORT = "/dev/ttyACM0"
_DEFAULT_BAUDRATE = 1_000_000


# ---------------------------------------------------------------------------
# Bus factory (monkeypatched in tests)
# ---------------------------------------------------------------------------


def _open_bus(args: argparse.Namespace) -> MotorBus:
    """Open a :class:`~arm101.hardware.bus.FeetechBus` for *args.port*.

    Tests replace this function with a lambda that returns a
    :class:`~arm101.hardware.bus.FakeBus` so no hardware is required.

    Raises
    ------
    CliError(EXIT_ENV_ERROR)
        If ``scservo_sdk`` is absent or the port cannot be opened.
    """
    port = getattr(args, "port", None) or _DEFAULT_PORT
    bus = FeetechBus(port, baudrate=_DEFAULT_BAUDRATE)
    bus.open()  # may raise CliError(EXIT_ENV_ERROR)
    return bus


# ---------------------------------------------------------------------------
# Handler
# ---------------------------------------------------------------------------


def cmd_setup_motors(args: argparse.Namespace) -> None:
    """Walk motors 6→1, prompt per motor, write EEPROM id/baudrate after Enter."""
    json_mode = bool(getattr(args, "json", False))

    # Safety check: must be running interactively.
    if not sys.stdin.isatty():
        raise CliError(
            code=EXIT_ENV_ERROR,
            message="setup-motors requires an interactive terminal (stdin is not a TTY).",
            remediation=(
                "This verb is inherently interactive — connect a terminal "
                "and run without pipes or redirects."
            ),
        )

    bus = _open_bus(args)

    assigned: list[dict[str, object]] = []

    try:
        for motor_id, joint_name in _MOTOR_ORDER:
            emit_diagnostic(
                f"connect the {joint_name} motor (id {motor_id}) only, then press Enter"
            )
            line = sys.stdin.readline()
            if line == "":
                # stdin closed / EOF before all motors were processed
                raise CliError(
                    code=EXIT_ENV_ERROR,
                    message=(
                        f"stdin closed unexpectedly before motor {motor_id} "
                        f"({joint_name}) was confirmed."
                    ),
                    remediation=(
                        "Provide an interactive terminal so each motor can be "
                        "confirmed with Enter before its EEPROM is written."
                    ),
                )
            bus.write_id_baudrate(motor=motor_id, new_id=motor_id, baudrate=_DEFAULT_BAUDRATE)
            assigned.append(
                {
                    "joint": joint_name,
                    "motor": motor_id,
                    "new_id": motor_id,
                    "baudrate": _DEFAULT_BAUDRATE,
                }
            )
    finally:
        bus.close()

    # Emit summary to stdout.
    if json_mode:
        emit_result({"assigned": assigned}, json_mode=True)
    else:
        lines = ["Motors assigned:"]
        for entry in assigned:
            lines.append(
                f"  {entry['joint']} (motor {entry['motor']}): "
                f"id={entry['new_id']}, baudrate={entry['baudrate']}"
            )
        emit_result("\n".join(lines), json_mode=False)


# ---------------------------------------------------------------------------
# Registration
# ---------------------------------------------------------------------------


def register(sub: "argparse._SubParsersAction[argparse.ArgumentParser]") -> None:
    """Register ``setup-motors`` on *sub*."""
    p = sub.add_parser(
        "setup-motors",
        help=(
            "Assign EEPROM id and baudrate to each motor one at a time "
            "(gripper=6 down to shoulder_pan=1)."
        ),
    )
    p.add_argument(
        "--port",
        default=_DEFAULT_PORT,
        help=f"Serial port for the motor bus (default: {_DEFAULT_PORT}).",
    )
    p.add_argument("--json", action="store_true", help="Emit structured JSON.")
    p.set_defaults(func=cmd_setup_motors)
