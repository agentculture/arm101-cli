"""``arm101 setup-motors`` — one-motor-at-a-time EEPROM id/baudrate assignment.

Mirrors the lerobot ``setup-motors`` workflow: walks the arm joints from gripper
(id 6) down to shoulder_pan (id 1), prompting the operator to connect each motor
alone before writing its EEPROM id and baudrate.

Three consent modes
-------------------
1. **interactive** (TTY): per-motor diagnostic prompt, Enter gate, then EEPROM
   write.  Preserves the original behaviour exactly.
2. **dry_run** (non-TTY, no ``--apply``): emits the full 6→1 assignment table
   (joint / from_id / new_id / baudrate) in both text and ``--json``.  Opens no
   bus; performs ZERO writes.
3. **agent** (non-TTY + ``--apply``): drives the 6→1 walk headless without
   blocking on stdin.  Before each write emits a "connect the <joint> motor now"
   guidance line, then writes the motor.  The physical connect/disconnect is the
   operator's responsibility (human / USB hub / future agent USB-swap
   capability), never the CLI's.

Safety invariants
-----------------
* Each EEPROM write is gated on the operator pressing Enter (interactive mode)
  or the ``--apply`` flag (agent mode).  No write ever precedes its consent.
* On success the result summary goes to stdout; all operator prompts go to
  stderr.  Both text and ``--json`` honour this split.
* Every write is audited (pending → success/failed) via the consent-core audit
  helpers, carrying ``consent_mode`` and ``operator``.

Bus injection seam
------------------
``_open_bus(args)`` is a module-level factory the test suite monkeypatches to
return a :class:`~arm101.hardware.bus.FakeBus` without touching hardware.
"""

from __future__ import annotations

import argparse
import sys

from arm101.cli._consent import (
    build_audit_record,
    resolve_consent,
    resolve_operator,
    write_audit,
)
from arm101.cli._errors import EXIT_ENV_ERROR, EXIT_USER_ERROR, CliError
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

#: Factory/default Feetech servo ID. Fresh STS3215 motors all ship at this ID,
#: so each connected motor is *addressed* here and *reassigned* to its target
#: ID. Override with ``--current-id`` when a motor is already at another ID.
_FACTORY_DEFAULT_ID = 1


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
# Audit helper
# ---------------------------------------------------------------------------


def _audit_write(
    port: str,
    operator: str,
    mode: str,
    motor_id: int,
    joint_name: str,
    current_id: int,
    outcome: str,
    error: str | None = None,
) -> None:
    """Append a setup-motors audit record (never raises)."""
    action = {
        "kind": "eeprom_id_write",
        "from_id": current_id,
        "to_id": motor_id,
        "baudrate": _DEFAULT_BAUDRATE,
        "joint": joint_name,
    }
    write_audit(
        build_audit_record(
            verb="setup-motors",
            port=port,
            operator=operator,
            consent_mode=mode,
            action=action,
            outcome=outcome,
            error=error,
        )
    )


# ---------------------------------------------------------------------------
# Dry-run emitter
# ---------------------------------------------------------------------------


def _emit_dry_run(current_id: int, *, json_mode: bool) -> None:
    """Emit the full 6→1 assignment plan (zero writes)."""
    plan: list[dict[str, object]] = [
        {
            "joint": joint_name,
            "from_id": current_id,
            "new_id": motor_id,
            "baudrate": _DEFAULT_BAUDRATE,
        }
        for motor_id, joint_name in _MOTOR_ORDER
    ]

    if json_mode:
        emit_result({"plan": plan}, json_mode=True)
        return

    lines = [
        "## Dry-run plan: setup-motors",
        "",
        "Motor assignment table (6→1):",
        "",
        "| joint | from_id | new_id | baudrate |",
        "|-------|---------|--------|----------|",
    ]
    for entry in plan:
        lines.append(
            f"| {entry['joint']} | {entry['from_id']} | {entry['new_id']} | {entry['baudrate']} |"
        )
    lines.append("")
    lines.append("To execute, connect each motor one at a time and re-run with --apply.")
    emit_result("\n".join(lines), json_mode=False)


# ---------------------------------------------------------------------------
# Motor walk helper
# ---------------------------------------------------------------------------


def _run_walk(
    bus: MotorBus,
    *,
    mode: str,
    current_id: int,
    port: str,
    operator: str,
) -> list[dict[str, object]]:
    """Walk _MOTOR_ORDER, gating/guiding per mode, auditing each write.

    Returns the assigned list (one entry per motor written).  Raises
    ``CliError(EXIT_ENV_ERROR)`` on EOF mid-walk (interactive mode only);
    any bus exception propagates after recording a ``failed`` audit entry.
    """
    assigned: list[dict[str, object]] = []
    for motor_id, joint_name in _MOTOR_ORDER:
        if mode == "interactive":
            emit_diagnostic(
                f"connect the {joint_name} motor ONLY (currently at id "
                f"{current_id}), then press Enter — it will be reassigned "
                f"to id {motor_id}"
            )
            line = sys.stdin.readline()
            if line == "":
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
        else:
            # agent mode: emit connect guidance, no readline
            emit_diagnostic(f"connect the {joint_name} motor now (id {current_id} → {motor_id})")

        # Audit: pending before write
        _audit_write(port, operator, mode, motor_id, joint_name, current_id, "pending")
        try:
            bus.write_id_baudrate(
                motor=current_id,
                new_id=motor_id,
                baudrate=_DEFAULT_BAUDRATE,
            )
        except Exception as e:  # noqa: BLE001
            _audit_write(
                port,
                operator,
                mode,
                motor_id,
                joint_name,
                current_id,
                "failed",
                error=str(e),
            )
            raise
        _audit_write(port, operator, mode, motor_id, joint_name, current_id, "success")

        assigned.append(
            {
                "joint": joint_name,
                "from_id": current_id,
                "new_id": motor_id,
                "baudrate": _DEFAULT_BAUDRATE,
            }
        )
    return assigned


# ---------------------------------------------------------------------------
# Handler
# ---------------------------------------------------------------------------


def cmd_setup_motors(args: argparse.Namespace) -> None:
    """Walk motors 6→1, prompt per motor, write EEPROM id/baudrate after Enter."""
    json_mode = bool(getattr(args, "json", False))
    port = getattr(args, "port", None) or _DEFAULT_PORT

    # Resolve --current-id with explicit None-defaulting (never `or`, which would
    # silently rewrite a falsy 0 to the factory id and target the wrong motor).
    # Reject anything outside 1–253 (254 is the broadcast id). Validated before
    # mode dispatch so every mode (dry_run / interactive / agent) is guarded.
    raw_current_id = getattr(args, "current_id", None)
    if raw_current_id is None:
        current_id = _FACTORY_DEFAULT_ID
    else:
        try:
            current_id = int(raw_current_id)
        except (ValueError, TypeError):
            raise CliError(
                code=EXIT_USER_ERROR,
                message=f"Invalid --current-id {raw_current_id!r}: must be an integer.",
                remediation="Provide an integer between 1 and 253.",
            )
        if not (1 <= current_id <= 253):
            raise CliError(
                code=EXIT_USER_ERROR,
                message=(
                    f"--current-id {current_id} is out of range (1–253); "
                    "254 is the broadcast id and must not be used."
                ),
                remediation="Choose an id between 1 and 253 inclusive.",
            )

    mode = resolve_consent(args, verb="setup-motors", require_plan_hash=False)

    # --- dry_run: emit plan, zero writes ---
    if mode == "dry_run":
        _emit_dry_run(current_id, json_mode=json_mode)
        return

    # --- interactive / agent: open bus and walk ---
    bus = _open_bus(args)
    operator = resolve_operator()

    try:
        assigned = _run_walk(bus, mode=mode, current_id=current_id, port=port, operator=operator)
    finally:
        bus.close()

    # Emit summary to stdout.
    if json_mode:
        emit_result({"assigned": assigned}, json_mode=True)
    else:
        lines = ["Motors assigned:"]
        for entry in assigned:
            lines.append(
                f"  {entry['joint']}: id {entry['from_id']} -> {entry['new_id']}, "
                f"baudrate={entry['baudrate']}"
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
    p.add_argument(
        "--current-id",
        type=int,
        default=_FACTORY_DEFAULT_ID,
        help=(
            "ID each connected motor currently answers at, used to address it before "
            f"reassigning (default: {_FACTORY_DEFAULT_ID}, the factory default)."
        ),
    )
    p.add_argument(
        "--apply",
        action="store_true",
        default=False,
        help="Execute the EEPROM writes (non-TTY agent mode; ignored under a TTY).",
    )
    p.add_argument("--json", action="store_true", help="Emit structured JSON.")
    p.set_defaults(func=cmd_setup_motors)
