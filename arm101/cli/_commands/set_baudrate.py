"""``arm101 set-baudrate`` — change the EEPROM baud rate of the single connected STS3215.

This sets the baud-rate register (addr 6) in the motor's EEPROM without
touching the servo ID (addr 5).  Useful when the communication baud rate needs
to be changed independently of the ID-assignment step.

**This is a persistent EEPROM write.**  The operator must confirm by typing
``yes`` at the confirmation prompt.  In non-interactive environments the
consent helper resolves to ``dry_run`` (plan-only) or ``agent`` (with
``--apply``).

On tested STS3215 firmware (3.10) the baud change takes effect **immediately**:
right after the write the motor answers only at the new baud, so the after-card
opens a fresh bus at the new baud to confirm it.  (Some firmware may instead
defer to the next power-up; the after-read degrades gracefully if so.)  One
consequence: the normal CLI always opens at 1 000 000, so after moving a motor
off 1 000 000 you must talk to it at its new baud to reach it again.

Bus injection seam
------------------
:func:`calibrate_motor._open_bus` and :func:`calibrate_motor._candidate_ports`
are monkeypatched in tests to inject a
:class:`~arm101.hardware.bus.FakeBus` without physical hardware.
:func:`_open_bus_after` is also monkeypatched in tests to inject an after-read
bus without opening a real serial port.
Detection is provided entirely by :func:`calibrate_motor._detect_one_motor`;
this module adds only the baud-write flow on top.
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
from arm101.cli._errors import EXIT_USER_ERROR, CliError
from arm101.cli._output import emit_diagnostic, emit_result
from arm101.hardware.bus import BAUD_INDEX_TO_BPS, BAUD_MAP, FeetechBus, MotorBus

# ---------------------------------------------------------------------------
# After-read bus factory (monkeypatched in tests)
# ---------------------------------------------------------------------------


def _open_bus_after(port: str, baud: int) -> MotorBus:
    """Open a :class:`~arm101.hardware.bus.FeetechBus` at *baud* for the after-card.

    On tested STS3215 firmware (3.10) a
    :meth:`~arm101.hardware.bus.MotorBus.write_baudrate` call takes effect
    immediately, so the motor answers only at the new baud; the after-card
    therefore opens a fresh bus at *baud* to confirm the register was written
    correctly.  (Older firmware may defer the change to the next power-up — the
    after-read then fails and degrades to a diagnostic rather than aborting.)

    Tests replace this with a lambda that returns a
    :class:`~arm101.hardware.bus.FakeBus`.

    Raises
    ------
    CliError(EXIT_ENV_ERROR)
        If ``scservo_sdk`` is absent or the port cannot be opened.
    """
    bus = FeetechBus(port, baudrate=baud)
    bus.open()
    return bus


# ---------------------------------------------------------------------------
# Command handler helpers
# ---------------------------------------------------------------------------


def _ensure_supported(target: int) -> None:
    """Raise CliError(EXIT_USER_ERROR) if *target* is not in :data:`BAUD_MAP`."""
    if target not in BAUD_MAP:
        raise CliError(
            code=EXIT_USER_ERROR,
            message=f"Unsupported baudrate {target}. Valid values: {sorted(BAUD_MAP)}.",
            remediation=f"Choose one of: {sorted(BAUD_MAP)}.",
        )


def _validate_baud_early(args: argparse.Namespace) -> None:
    """Reject invalid baud input *before* any serial bus is opened.

    The two cases knowable from ``args`` alone — a provided-but-unsupported
    positional baud, and an omitted baud in a non-interactive (non-TTY) run —
    are failed fast here so a bad value never triggers a port scan or EEPROM
    read.  This is the behaviour the CHANGELOG documents ("invalid value →
    ``EXIT_USER_ERROR`` before any bus is opened").  The interactive TTY prompt
    for an omitted baud is deferred to :func:`_resolve_target_baud`, which runs
    after the BEFORE card.
    """
    raw = getattr(args, "baud", None)
    if raw is None:
        if not sys.stdin.isatty():
            raise CliError(
                code=EXIT_USER_ERROR,
                message="baud is required in non-interactive mode",
                remediation=("Pass the target baudrate, e.g. arm101 set-baudrate 500000 --apply"),
            )
        return  # TTY + omitted: prompt later, after the BEFORE card
    _ensure_supported(raw)


def _resolve_target_baud(args: argparse.Namespace) -> int:
    """Resolve the target baud rate from the positional arg or an interactive prompt.

    Raises CliError(EXIT_USER_ERROR) when no baud is given in non-interactive
    mode, when the value is not an integer, or when it is not in
    :data:`~arm101.hardware.bus.BAUD_MAP`.

    The positional and non-TTY cases are already screened by
    :func:`_validate_baud_early` before the bus is opened; the supported-rate
    re-check here also covers the value typed at the interactive prompt.
    """
    raw = getattr(args, "baud", None)
    if raw is None:
        if sys.stdin.isatty():
            valid_str = ", ".join(str(b) for b in sorted(BAUD_MAP))
            raw_str = _prompt(f"Target baudrate ({valid_str})", required=True)
            try:
                target = int(raw_str)
            except (ValueError, TypeError):
                raise CliError(
                    code=EXIT_USER_ERROR,
                    message=f"Invalid baudrate {raw_str!r}: must be an integer.",
                    remediation=f"Choose from: {sorted(BAUD_MAP)}.",
                )
        else:
            raise CliError(
                code=EXIT_USER_ERROR,
                message="baud is required in non-interactive mode",
                remediation=("Pass the target baudrate, e.g. arm101 set-baudrate 500000 --apply"),
            )
    else:
        target = raw  # already int (type=int on the parser arg)

    _ensure_supported(target)
    return target


def _emit_dry_run(port, motor_id, target_baud, info, *, json_mode: bool) -> None:
    """Emit the read-only dry-run plan for a set-baudrate write (zero writes)."""
    current_baud_idx = info.get("baud_index", 0)
    current_baud = BAUD_INDEX_TO_BPS.get(current_baud_idx, "unknown")
    if json_mode:
        emit_result(
            {
                "plan": {
                    "verb": "set-baudrate",
                    "port": port,
                    "motor": motor_id,
                    "from_baudrate": current_baud,
                    "to_baudrate": target_baud,
                    "motor_snapshot": {
                        "id": info["id"],
                        "model": info["model"],
                        "present_position": info["present_position"],
                        "torque_enable": info["torque_enable"],
                    },
                },
                "apply_command": f"arm101 set-baudrate {target_baud} --apply",
            },
            json_mode=True,
        )
        return
    lines = [
        "## Dry-run plan: set-baudrate",
        "",
        f"- **port**          : {port}",
        f"- **motor**         : {motor_id}",
        f"- **from_baudrate** : {current_baud}",
        f"- **to_baudrate**   : {target_baud}",
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
        f"Re-run to apply: arm101 set-baudrate {target_baud} --apply",
    ]
    emit_result("\n".join(lines), json_mode=False)


def _confirm_interactive(port, motor_id, target_baud, *, json_mode: bool) -> bool:
    """Prompt the human; return True to proceed, False (and emit an abort) otherwise."""
    emit_diagnostic(
        f"⚠ This WRITES EEPROM (persistent) on motor {motor_id} at {port}: "
        f"baud -> {target_baud}. "
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
                "motor": motor_id,
                "baudrate": target_baud,
            },
            json_mode=True,
        )
    else:
        emit_result("Aborted; no EEPROM write.", json_mode=False)
    return False


def _audit(port, operator, mode, action, outcome, error=None) -> None:
    """Append a set-baudrate audit record (never raises)."""
    write_audit(
        build_audit_record(
            verb="set-baudrate",
            port=port,
            operator=operator,
            consent_mode=mode,
            action=action,
            outcome=outcome,
            error=error,
        )
    )


def cmd_set_baudrate(args: argparse.Namespace) -> None:
    """Change the EEPROM baud rate of the single connected STS3215 servo (gated)."""
    json_mode = bool(getattr(args, "json", False))

    # Fail fast on invalid baud input before any serial port is opened/scanned.
    _validate_baud_early(args)

    bus, port, current_id = _detect_one_motor(args)
    try:
        info = bus.read_info(current_id)
        _show_info(info, port)

        target_baud = _resolve_target_baud(args)
        action = {
            "kind": "eeprom_baud_write",
            "motor": current_id,
            "baudrate": target_baud,
        }

        mode = resolve_consent(args, verb="set-baudrate", require_plan_hash=False)

        if mode == "dry_run":
            _emit_dry_run(port, current_id, target_baud, info, json_mode=json_mode)
            return
        if mode == "interactive":
            if not _confirm_interactive(port, current_id, target_baud, json_mode=json_mode):
                return

        # mode == 'agent' OR interactive-confirmed: proceed with the write
        operator = resolve_operator()
        _audit(port, operator, mode, action, "pending")
        try:
            bus.write_baudrate(motor=current_id, baudrate=target_baud)
        except Exception as e:  # noqa: BLE001
            _audit(port, operator, mode, action, "failed", error=str(e))
            raise
        _audit(port, operator, mode, action, "success")

        # AFTER card — open a fresh bus at the new baud to confirm the write
        emit_diagnostic(f"\n-- AFTER write (motor {current_id} baud -> {target_baud}) --")
        try:
            after_bus = _open_bus_after(port, target_baud)
            try:
                after_info = after_bus.read_info(current_id)
                _show_info(after_info, port)
            finally:
                after_bus.close()
        except Exception:  # noqa: BLE001
            emit_diagnostic(
                "Write succeeded but after-read failed (motor may need power-cycle "
                "to apply baud change)."
            )

        if json_mode:
            emit_result(
                {
                    "port": port,
                    "motor": current_id,
                    "baudrate": target_baud,
                },
                json_mode=True,
            )
        else:
            emit_result(
                f"Set baudrate of motor {current_id} to {target_baud} on {port} "
                "(EEPROM written).",
                json_mode=False,
            )
    finally:
        bus.close()


# ---------------------------------------------------------------------------
# Registration
# ---------------------------------------------------------------------------


def register(sub: "argparse._SubParsersAction") -> None:
    """Register the ``set-baudrate`` subcommand on *sub*."""
    p = sub.add_parser(
        "set-baudrate",
        help=(
            "Change the EEPROM baud rate of the single connected STS3215 servo "
            "(gated persistent write; id unchanged)."
        ),
    )
    p.add_argument(
        "baud",
        nargs="?",
        type=int,
        help=(f"Target baud rate; one of {sorted(BAUD_MAP)}. " "Omit to be prompted."),
    )
    p.add_argument(
        "--port",
        default=None,
        help="Serial port of the motor (default: auto-detect, skipping busy/non-motor ports).",
    )
    p.add_argument(
        "--apply",
        action="store_true",
        default=False,
        help="Execute the EEPROM write (non-TTY agent mode; ignored under a TTY).",
    )
    p.add_argument("--json", action="store_true", help="Emit structured JSON.")
    p.set_defaults(func=cmd_set_baudrate)
