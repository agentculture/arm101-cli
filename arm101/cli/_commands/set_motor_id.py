"""``arm101 set-motor-id`` — assign a new EEPROM ID to the single connected STS3215.

This is the SO-101 pre-assembly motor-ID assignment step.  Before the arm is
assembled, each Feetech servo is connected **one at a time** and given its
joint's EEPROM ID (1–253).  This verb detects the single connected STS3215,
shows a read-only register snapshot, prompts for (or accepts via positional
argument) the target ID, then gates on an explicit ``yes`` confirmation before
writing.

**This is a persistent EEPROM write.**  The operator must confirm by typing
``yes`` at the confirmation prompt.  In non-interactive environments (EOF on
stdin) the confirmation prompt raises ``CliError(EXIT_ENV_ERROR)`` — this is
intentional so a non-interactive run can never silently write EEPROM.

Bus injection seam
------------------
:func:`calibrate_motor._open_bus` and :func:`calibrate_motor._candidate_ports`
are monkeypatched in tests to inject a
:class:`~arm101.hardware.bus.FakeBus` without physical hardware.
Detection is provided entirely by :func:`calibrate_motor._detect_one_motor`;
this module adds only the ID-write flow on top.
"""

from __future__ import annotations

import argparse
import sys

from arm101.cli._commands.calibrate_motor import (  # noqa: F401 (seam imports)
    _candidate_ports,
    _detect_one_motor,
    _open_bus,
    _prompt,
    _show_info,
)
from arm101.cli._errors import EXIT_ENV_ERROR, EXIT_USER_ERROR, CliError
from arm101.cli._output import emit_diagnostic, emit_result


def _require_tty() -> None:
    """Refuse to run unless stdin is an interactive terminal.

    A persistent EEPROM write must never be driven by piped/redirected input: a
    non-TTY stdin could otherwise feed ``yes`` and satisfy the confirmation gate
    non-interactively (a CI run or ``echo yes | …``).  Checked before the bus is
    opened so a non-interactive invocation touches no hardware.
    """
    if not sys.stdin.isatty():
        raise CliError(
            code=EXIT_ENV_ERROR,
            message="set-motor-id requires an interactive terminal (stdin is not a TTY).",
            remediation=(
                "This verb performs a gated EEPROM write — run it without pipes "
                "or redirects so the confirmation is answered by a human."
            ),
        )


# ---------------------------------------------------------------------------
# Confirmation gate
# ---------------------------------------------------------------------------


def _confirm(question: str) -> bool:
    """Return ``True`` iff the operator types exactly ``yes`` at the prompt.

    :func:`_prompt` raises ``CliError(EXIT_ENV_ERROR)`` on EOF so a
    non-interactive run can never silently confirm a destructive EEPROM write.
    """
    answer = _prompt(question)
    return answer.strip().lower() == "yes"


# ---------------------------------------------------------------------------
# Command handler
# ---------------------------------------------------------------------------


def cmd_set_motor_id(args: argparse.Namespace) -> None:
    """Assign a new EEPROM ID to the single connected STS3215 servo (gated)."""
    json_mode = bool(getattr(args, "json", False))
    baudrate: int = getattr(args, "baudrate", 1_000_000)

    _require_tty()

    bus, port, current_id = _detect_one_motor(args)
    try:
        info = bus.read_info(current_id)
        _show_info(info, port)

        # Resolve target ID: positional arg (arrives as str) or interactive prompt.
        raw_new_id = getattr(args, "new_id", None)
        if raw_new_id is not None:
            raw_str = str(raw_new_id)
        else:
            raw_str = _prompt("New motor ID (1-253)", required=True)

        try:
            new_id = int(raw_str)
        except (ValueError, TypeError):
            raise CliError(
                code=EXIT_USER_ERROR,
                message=f"Invalid motor ID {raw_str!r}: must be an integer.",
                remediation="Provide an integer between 1 and 253.",
            )
        if not (1 <= new_id <= 253):
            raise CliError(
                code=EXIT_USER_ERROR,
                message=(
                    f"Motor ID {new_id} is out of range (1–253); "
                    "254 is the broadcast ID and must not be used."
                ),
                remediation="Choose an ID between 1 and 253 inclusive.",
            )

        # Gate: persistent EEPROM write requires explicit operator confirmation.
        emit_diagnostic(
            f"⚠ This WRITES EEPROM (persistent) on the motor at {port}: "
            f"ID {current_id} → {new_id}, baud {baudrate}. "
            "Connect ONE motor only."
        )
        if not _confirm("Type 'yes' to confirm EEPROM write"):
            if json_mode:
                emit_result(
                    {
                        "aborted": True,
                        "port": port,
                        "from_id": current_id,
                        "to_id": new_id,
                        "baudrate": baudrate,
                    },
                    json_mode=True,
                )
            else:
                emit_result("Aborted; no EEPROM write.", json_mode=False)
            return

        bus.write_id_baudrate(motor=current_id, new_id=new_id, baudrate=baudrate)

        if json_mode:
            emit_result(
                {
                    "port": port,
                    "from_id": current_id,
                    "to_id": new_id,
                    "baudrate": baudrate,
                },
                json_mode=True,
            )
        else:
            emit_result(
                f"Set motor ID {current_id} → {new_id} on {port} "
                f"(EEPROM written, baud {baudrate}).",
                json_mode=False,
            )
    finally:
        bus.close()


# ---------------------------------------------------------------------------
# Registration
# ---------------------------------------------------------------------------


def register(sub: "argparse._SubParsersAction") -> None:
    """Register the ``set-motor-id`` subcommand on *sub*."""
    p = sub.add_parser(
        "set-motor-id",
        help=(
            "Assign a new EEPROM ID to the single connected STS3215 servo "
            "(gated persistent write)."
        ),
    )
    p.add_argument(
        "new_id",
        nargs="?",
        help="Target EEPROM id 1-253; omit to be prompted.",
    )
    p.add_argument(
        "--port",
        default=None,
        help="Serial port of the motor (default: auto-detect, skipping busy/non-motor ports).",
    )
    p.add_argument(
        "--baudrate",
        type=int,
        default=1_000_000,
        help="Baud rate to programme into the motor's EEPROM (default: 1000000).",
    )
    p.add_argument("--json", action="store_true", help="Emit structured JSON.")
    p.set_defaults(func=cmd_set_motor_id)
