"""``arm101 set-motor-id`` — assign a new EEPROM ID to the single connected STS3215.

This is the SO-101 pre-assembly motor-ID assignment step.  Before the arm is
assembled, each Feetech servo is connected **one at a time** and given its
joint's EEPROM ID (1–253).  This verb detects the single connected STS3215,
shows a read-only register snapshot, prompts for (or accepts via positional
argument) the target ID, then gates on an explicit ``yes`` confirmation before
writing.

**This is a persistent EEPROM write.**  The operator must confirm by typing
``yes`` at the confirmation prompt.  In non-interactive environments the
consent helper resolves to ``dry_run`` (plan-only) or ``agent`` (with ``--apply``).

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
from arm101.cli._consent import build_audit_record, resolve_consent, resolve_operator, write_audit
from arm101.cli._errors import EXIT_ENV_ERROR, EXIT_USER_ERROR, CliError
from arm101.cli._output import emit_diagnostic, emit_result

# ---------------------------------------------------------------------------
# Command handler
# ---------------------------------------------------------------------------


def _resolve_target_id(args: argparse.Namespace) -> int:
    """Resolve the target id from the positional arg or an interactive prompt.

    Raises CliError(EXIT_USER_ERROR) when no id is given in non-interactive mode,
    when the value is not an integer, or when it falls outside the 1-253 range.
    """
    raw = getattr(args, "new_id", None)
    if raw is None:
        if sys.stdin.isatty():
            raw_str = _prompt("New motor ID (1-253)", required=True)
        else:
            raise CliError(
                code=EXIT_USER_ERROR,
                message="new_id is required in non-interactive mode",
                remediation="Pass the target id, e.g. arm101 set-motor-id 6 --apply",
            )
    else:
        raw_str = str(raw)

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
    return new_id


def _emit_dry_run(port, current_id, new_id, baudrate, info, *, json_mode: bool) -> None:
    """Emit the read-only dry-run plan for a set-motor-id write (zero writes)."""
    if json_mode:
        emit_result(
            {
                "plan": {
                    "verb": "set-motor-id",
                    "port": port,
                    "from_id": current_id,
                    "to_id": new_id,
                    "baudrate": baudrate,
                    "motor_snapshot": {
                        "id": info["id"],
                        "model": info["model"],
                        "present_position": info["present_position"],
                        "torque_enable": info["torque_enable"],
                    },
                },
                "apply_command": f"arm101 set-motor-id {new_id} --apply",
            },
            json_mode=True,
        )
        return
    lines = [
        "## Dry-run plan: set-motor-id",
        "",
        f"- **port**     : {port}",
        f"- **from_id**  : {current_id}",
        f"- **to_id**    : {new_id}",
        f"- **baudrate** : {baudrate}",
        "",
        "### Motor snapshot",
        "",
        f"- id               : {info['id']}",
        f"- model            : {info['model']}",
        f"- present_position : {info['present_position']}",
        f"- torque_enable    : {info['torque_enable']}",
        "",
        "### Next step",
        "",
        f"Re-run to apply: arm101 set-motor-id {new_id} --apply",
    ]
    emit_result("\n".join(lines), json_mode=False)


def _confirm_interactive(port, current_id, new_id, baudrate, *, json_mode: bool) -> bool:
    """Prompt the human; return True to proceed, False (and emit an abort) otherwise."""
    emit_diagnostic(
        f"⚠ This WRITES EEPROM (persistent) on the motor at {port}: "
        f"ID {current_id} -> {new_id}, baud {baudrate}. "
        "Connect ONE motor only."
    )
    ans = _prompt("Type 'yes' to confirm EEPROM write")
    if ans.strip().lower() == "yes":
        return True
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
    return False


def _audit(port, operator, mode, action, outcome, error=None) -> None:
    """Append a set-motor-id audit record (never raises)."""
    write_audit(
        build_audit_record(
            verb="set-motor-id",
            port=port,
            operator=operator,
            consent_mode=mode,
            action=action,
            outcome=outcome,
            error=error,
        )
    )


def cmd_set_motor_id(args: argparse.Namespace) -> None:
    """Assign a new EEPROM ID to the single connected STS3215 servo (gated)."""
    json_mode = bool(getattr(args, "json", False))
    baudrate: int = getattr(args, "baudrate", 1_000_000)

    bus, port, current_id = _detect_one_motor(args)
    try:
        info = bus.read_info(current_id)
        _show_info(info, port)

        new_id = _resolve_target_id(args)
        action = {
            "kind": "eeprom_id_write",
            "from_id": current_id,
            "to_id": new_id,
            "baudrate": baudrate,
        }

        mode = resolve_consent(args, verb="set-motor-id", require_plan_hash=False)

        if mode == "dry_run":
            _emit_dry_run(port, current_id, new_id, baudrate, info, json_mode=json_mode)
            return
        if mode == "interactive":
            if not _confirm_interactive(port, current_id, new_id, baudrate, json_mode=json_mode):
                return

        # mode == 'agent' OR interactive-confirmed: proceed with the write
        operator = resolve_operator()
        _audit(port, operator, mode, action, "pending")
        try:
            bus.write_id_baudrate(motor=current_id, new_id=new_id, baudrate=baudrate)
        except Exception as e:  # noqa: BLE001
            # Audit the failed write, then re-raise.
            _audit(port, operator, mode, action, "failed", error=str(e))
            raise

        # Read-back verification: the motor now answers at new_id. If the
        # EEPROM write didn't actually persist (e.g. the Lock register was
        # never opened — see PR #21), the motor reverts to its old id and
        # this catches it immediately instead of silently shipping a dead write.
        read_back_id = bus.read_info(new_id)["id"]
        if read_back_id != new_id:
            error_message = (
                f"motor id did not persist (read back {read_back_id}, expected {new_id}) "
                "— the EEPROM write may not have stuck"
            )
            _audit(port, operator, mode, action, "failed", error=error_message)
            raise CliError(
                code=EXIT_ENV_ERROR,
                message=error_message,
                remediation=(
                    "Check the motor's EEPROM Lock register; ensure only one motor "
                    "is connected and power is stable, then retry."
                ),
            )

        _audit(port, operator, mode, action, "success")

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
                f"Set motor ID {current_id} -> {new_id} on {port} "
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
    p.add_argument(
        "--apply",
        action="store_true",
        default=False,
        help="Execute the EEPROM write (non-TTY agent mode; ignored under a TTY).",
    )
    p.add_argument("--json", action="store_true", help="Emit structured JSON.")
    p.set_defaults(func=cmd_set_motor_id)
