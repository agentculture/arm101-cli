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

Per-motor port re-detection
---------------------------
Rather than opening one bus and reusing it for all 6 motors, each motor in the
walk calls :func:`calibrate_motor._detect_one_motor` fresh.  This handles the
common case where unplugging one motor and plugging in the next changes the
``/dev/ttyACM*`` enumeration — the stale file descriptor from the old port would
return an I/O error on the next write, so re-detection is the correct fix.

Bus injection seam
------------------
Detection (and therefore the motor bus) is injected via
:func:`calibrate_motor._open_bus` and :func:`calibrate_motor._candidate_ports`,
which the test suite monkeypatches to return a
:class:`~arm101.hardware.bus.FakeBus` without touching hardware.  Tests that
previously patched ``setup_motors._open_bus`` should instead patch these two
seams on the ``calibrate_motor`` module.

The after-read bus (used when ``--baudrate`` differs from the communication
baudrate 1 000 000) is injected via :func:`_open_bus_after`, a separate
module-level factory the test suite may also patch.
"""

from __future__ import annotations

import argparse
import sys

from arm101.cli._commands.calibrate_motor import (  # noqa: F401 (seam imports)
    _candidate_ports,
    _detect_one_motor,
    _open_bus,
    _show_info,
)
from arm101.cli._consent import (
    build_audit_record,
    resolve_consent,
    resolve_operator,
    write_audit,
)
from arm101.cli._errors import EXIT_ENV_ERROR, EXIT_USER_ERROR, CliError
from arm101.cli._output import emit_diagnostic, emit_result
from arm101.hardware.bus import BAUD_MAP, FeetechBus, MotorBus

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

_DEFAULT_BAUDRATE = 1_000_000

#: Factory/default Feetech servo ID. Fresh STS3215 motors all ship at this ID.
_FACTORY_DEFAULT_ID = 1


# ---------------------------------------------------------------------------
# After-read bus factory (monkeypatched in tests)
# ---------------------------------------------------------------------------


def _open_bus_after(port: str, baud: int) -> MotorBus:
    """Open a :class:`~arm101.hardware.bus.FeetechBus` at *baud* for the after-card.

    Used when the EEPROM baud written differs from the communication baudrate
    (1 000 000), because the motor's new EEPROM baud takes effect on the next
    power-up — for the current session the motor still responds at 1 000 000 —
    but for the after-read we open at *baud* to be explicit and consistent.

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
    *,
    baudrate: int = _DEFAULT_BAUDRATE,
) -> None:
    """Append a setup-motors audit record (never raises)."""
    action = {
        "kind": "eeprom_id_write",
        "from_id": current_id,
        "to_id": motor_id,
        "baudrate": baudrate,
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


def _emit_dry_run(current_id: int, *, baudrate: int, json_mode: bool) -> None:
    """Emit the full 6→1 assignment plan (zero writes)."""
    plan: list[dict[str, object]] = [
        {
            "joint": joint_name,
            "from_id": current_id,
            "new_id": motor_id,
            "baudrate": baudrate,
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
# Motor walk helper (per-motor re-detection)
# ---------------------------------------------------------------------------


def _run_walk(
    args: argparse.Namespace,
    *,
    mode: str,
    asserted_current_id: int | None,
    baudrate: int,
    operator: str,
) -> list[dict[str, object]]:
    """Walk _MOTOR_ORDER, re-detecting the bus per motor.

    Each iteration calls :func:`calibrate_motor._detect_one_motor` fresh, so
    USB re-enumeration between motors (``/dev/ttyACM*`` path changes) is
    handled transparently.

    Parameters
    ----------
    args:
        Parsed CLI args; ``args.port`` (``None`` = auto-detect) is forwarded to
        ``_detect_one_motor``.
    mode:
        Consent mode: ``"interactive"`` or ``"agent"``.
    asserted_current_id:
        If not ``None``, the detected motor id must equal this value or a
        ``CliError(EXIT_USER_ERROR)`` is raised before any write.
    baudrate:
        EEPROM baud rate to write (bps).
    operator:
        Resolved operator string for audit records.

    Returns
    -------
    list[dict]
        One entry per motor written: ``{joint, from_id, new_id, baudrate}``.
    """
    assigned: list[dict[str, object]] = []

    for motor_id, joint_name in _MOTOR_ORDER:
        # 1. Operator guidance / gating
        if mode == "interactive":
            emit_diagnostic(
                f"connect the {joint_name} motor ONLY (currently at id "
                f"{asserted_current_id if asserted_current_id is not None else _FACTORY_DEFAULT_ID}"
                f"), then press Enter — it will be reassigned to id {motor_id}"
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
            emit_diagnostic(
                f"connect the {joint_name} motor now "
                f"(id {asserted_current_id if asserted_current_id is not None else '?'} "
                f"→ {motor_id})"
            )

        # 2. Re-detect the motor (fresh bus per motor)
        bus, port, detected_id = _detect_one_motor(args)

        try:
            # 3. Validate --current-id assertion if provided
            if asserted_current_id is not None and detected_id != asserted_current_id:
                raise CliError(
                    code=EXIT_USER_ERROR,
                    message=(
                        f"Expected motor at id {asserted_current_id} "
                        f"but detected id {detected_id}."
                    ),
                    remediation=(
                        "Ensure the correct motor is connected, or omit "
                        "--current-id to use the auto-detected id."
                    ),
                )

            # 4. BEFORE card — read registers and show snapshot
            before_info = bus.read_info(detected_id)
            emit_diagnostic(f"\n-- BEFORE write (motor {detected_id} → {motor_id}) --")
            _show_info(before_info, port)

            # 5. Audit: pending before write
            _audit_write(
                port,
                operator,
                mode,
                motor_id,
                joint_name,
                detected_id,
                "pending",
                baudrate=baudrate,
            )

            # 6. EEPROM write
            try:
                bus.write_id_baudrate(
                    motor=detected_id,
                    new_id=motor_id,
                    baudrate=baudrate,
                )
            except Exception as e:  # noqa: BLE001
                _audit_write(
                    port,
                    operator,
                    mode,
                    motor_id,
                    joint_name,
                    detected_id,
                    "failed",
                    error=str(e),
                    baudrate=baudrate,
                )
                raise

            # 7. AFTER card — re-read at the new id (same bus if baud unchanged)
            emit_diagnostic(f"\n-- AFTER write (motor now at id {motor_id}) --")
            try:
                if baudrate == _DEFAULT_BAUDRATE:
                    # Motor still communicates at 1M baud (EEPROM change takes
                    # effect on next power-up); re-read on the same open bus.
                    after_info = bus.read_info(motor_id)
                else:
                    # Motor now responds at the new baud; open a fresh bus.
                    after_bus = _open_bus_after(port, baudrate)
                    try:
                        after_info = after_bus.read_info(motor_id)
                    finally:
                        after_bus.close()
                _show_info(after_info, port)
            except Exception:  # noqa: BLE001
                emit_diagnostic(
                    "Write succeeded but after-read failed (motor may need power-cycle "
                    "to apply baud change); continuing series."
                )

            # 8. Audit: success
            _audit_write(
                port,
                operator,
                mode,
                motor_id,
                joint_name,
                detected_id,
                "success",
                baudrate=baudrate,
            )

        finally:
            bus.close()

        assigned.append(
            {
                "joint": joint_name,
                "from_id": detected_id,
                "new_id": motor_id,
                "baudrate": baudrate,
            }
        )

    return assigned


# ---------------------------------------------------------------------------
# Handler
# ---------------------------------------------------------------------------


def cmd_setup_motors(args: argparse.Namespace) -> None:
    """Walk motors 6→1, re-detecting per motor, writing EEPROM id/baudrate."""
    json_mode = bool(getattr(args, "json", False))

    # --baudrate: validate against the Feetech baud map.
    baudrate = getattr(args, "baudrate", _DEFAULT_BAUDRATE)
    if baudrate not in BAUD_MAP:
        valid = sorted(BAUD_MAP)
        raise CliError(
            code=EXIT_USER_ERROR,
            message=f"Unsupported --baudrate {baudrate}. Valid values: {valid}.",
            remediation=f"Choose one of: {valid}.",
        )

    # --current-id: now an optional safety assertion (auto-detected per motor).
    # None = no assertion; explicit value = detected id must match.
    raw_current_id = getattr(args, "current_id", None)
    asserted_current_id: int | None = None
    if raw_current_id is not None:
        try:
            asserted_current_id = int(raw_current_id)
        except (ValueError, TypeError):
            raise CliError(
                code=EXIT_USER_ERROR,
                message=f"Invalid --current-id {raw_current_id!r}: must be an integer.",
                remediation="Provide an integer between 1 and 253.",
            )
        if not (1 <= asserted_current_id <= 253):
            raise CliError(
                code=EXIT_USER_ERROR,
                message=(
                    f"--current-id {asserted_current_id} is out of range (1–253); "
                    "254 is the broadcast id and must not be used."
                ),
                remediation="Choose an id between 1 and 253 inclusive.",
            )

    mode = resolve_consent(args, verb="setup-motors", require_plan_hash=False)

    # --- dry_run: emit plan, zero writes ---
    if mode == "dry_run":
        display_id = asserted_current_id if asserted_current_id is not None else _FACTORY_DEFAULT_ID
        _emit_dry_run(display_id, baudrate=baudrate, json_mode=json_mode)
        return

    # --- interactive / agent: per-motor detect + write ---
    operator = resolve_operator()
    assigned = _run_walk(
        args,
        mode=mode,
        asserted_current_id=asserted_current_id,
        baudrate=baudrate,
        operator=operator,
    )

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
        default=None,
        help=(
            "Serial port of the motor (default: auto-detect per motor, "
            "handles USB re-enumeration between motors). Pass a fixed path "
            "to skip auto-detection."
        ),
    )
    p.add_argument(
        "--baudrate",
        type=int,
        default=_DEFAULT_BAUDRATE,
        help=(
            f"EEPROM baud rate to programme (default: {_DEFAULT_BAUDRATE}). "
            f"Valid values: {sorted(BAUD_MAP)}."
        ),
    )
    p.add_argument(
        "--current-id",
        type=int,
        default=None,
        help=(
            "Safety assertion: if given, the auto-detected motor id must equal "
            "this value or the walk is aborted with an error. "
            "Omit to accept any detected id (the factory default is 1)."
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
