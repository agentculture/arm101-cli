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
import contextlib
import sys

from arm101.cli._commands.calibrate_motor import _detect_one_motor, _show_info
from arm101.cli._consent import (
    build_audit_record,
    resolve_consent,
    resolve_operator,
    write_audit,
)
from arm101.cli._errors import EXIT_ENV_ERROR, EXIT_USER_ERROR, CliError
from arm101.cli._output import emit_diagnostic, emit_result
from arm101.hardware import arm_spec
from arm101.hardware.bus import BAUD_MAP, FeetechBus, MotorBus
from arm101.hardware.safety import ReleaseReport, torque_guard

# ---------------------------------------------------------------------------
# Motor walk order: gripper (6) → shoulder_pan (1)
# Source: arm_spec — ids and baud are role-invariant (both roles share ids 1..6 and baud 1_000_000)
# ---------------------------------------------------------------------------

_DEFAULT_BAUDRATE: int = arm_spec.DEFAULT_BAUDRATE

_MOTOR_ORDER: list[tuple[int, str]] = sorted(
    [(spec.id, joint) for joint, spec in arm_spec.ARM_SPEC["follower"].items()],
    reverse=True,
)

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


def _gate_operator(
    mode: str,
    joint_name: str,
    motor_id: int,
    asserted_current_id: int | None,
) -> None:
    """Emit per-motor connect guidance and, in interactive mode, gate on Enter.

    When ``--current-id`` is omitted (``asserted_current_id is None``) the
    detected id is unknown until detection runs, so the guidance does **not**
    claim a specific "currently at id N" — it would otherwise mislead the
    operator under the auto-detect semantics.

    Raises
    ------
    CliError(EXIT_ENV_ERROR)
        Interactive mode only, if stdin closes (EOF) before Enter is pressed.
    """
    if mode == "interactive":
        here = (
            f" (currently at id {asserted_current_id})" if asserted_current_id is not None else ""
        )
        emit_diagnostic(
            f"connect the {joint_name} motor ONLY{here}, then press Enter "
            f"— it will be reassigned to id {motor_id}"
        )
        if sys.stdin.readline() == "":
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
        shown = asserted_current_id if asserted_current_id is not None else "?"
        emit_diagnostic(f"connect the {joint_name} motor now (id {shown} → {motor_id})")


def _after_read(bus: MotorBus, port: str, motor_id: int, baudrate: int) -> dict[str, int] | None:
    """AFTER card: re-read the motor at its new id; return the snapshot or ``None``.

    If the EEPROM baud was unchanged the motor still talks at the communication
    baud, so the same open *bus* is reused; otherwise a fresh bus is opened at
    the new baud.  A read FAILURE (e.g. the motor needs a power-cycle to apply
    a baud change) degrades to a diagnostic and returns ``None`` so the 6→1
    walk continues — the write call itself already succeeded, only the
    after-read could not reach the motor at its new baud.

    A *successful* read is returned as-is; the caller is responsible for
    verifying ``after_info["id"] == motor_id`` (read-back-after-write
    persistence check) and treating a mismatch as a hard failure — this
    function does not raise on mismatch itself.
    """
    emit_diagnostic(f"\n-- AFTER write (motor now at id {motor_id}) --")
    try:
        if baudrate == _DEFAULT_BAUDRATE:
            # Motor still communicates at 1M baud (EEPROM change takes effect on
            # next power-up); re-read on the same open bus.
            after_info = bus.read_info(motor_id)
        else:
            # Motor now responds at the new baud; open a fresh bus.
            after_bus = _open_bus_after(port, baudrate)
            try:
                after_info = after_bus.read_info(motor_id)
            finally:
                after_bus.close()
        _show_info(after_info, port)
        return after_info
    except Exception:  # noqa: BLE001
        emit_diagnostic(
            "Write succeeded but after-read failed (motor may need power-cycle "
            "to apply baud change); continuing series."
        )
        return None


def _announce_release(report: ReleaseReport) -> None:
    """Tell the operator, on stderr, that the torque guard de-energised the motor.

    A release only fires while an exception is unwinding, so the walk never
    reaches its summary — without this line the de-energising would be silent,
    and (worse) a motor the release could NOT reach, and which may therefore
    still be hot, would never be named. See :func:`~arm101.hardware.safety.torque_guard`.
    """
    if report.attempted:
        emit_diagnostic(report.describe())


def _process_one_motor(
    args: argparse.Namespace,
    motor_id: int,
    joint_name: str,
    *,
    mode: str,
    asserted_current_id: int | None,
    baudrate: int,
    operator: str,
) -> dict[str, object]:
    """Detect, validate, write, and after-read a single motor.

    Returns the assignment entry ``{joint, from_id, new_id, baudrate}``.  Raises
    ``CliError(EXIT_USER_ERROR)`` if a ``--current-id`` assertion fails, and
    re-raises any write failure (after auditing it).

    Torque ownership (issue #33)
    ----------------------------
    The whole per-motor block runs under a
    :func:`~arm101.hardware.safety.torque_guard`, so any abnormal exit — an
    EEPROM write that faults, a bus that vanishes, a ``Ctrl-C`` between motors —
    leaves the servo LIMP rather than energised and unattended. This walk does
    not itself enable torque, so on a healthy bench the guard is a no-op; it
    matters because the motor on the bench need not be cold to begin with. A
    servo left holding by an earlier ``arm flex``, or latched in overload from a
    previous session, is still hot when ``setup`` picks it up, and this verb had
    no path that would ever have relaxed it. The invariant is the point: a
    powered motor at process exit must be a state somebody CHOSE.

    The ownership hand-off at the id write is deliberate. Writing EEPROM addr 5
    moves the servo to a new bus address, so the id the guard claimed at
    detection stops answering the instant that write lands. The guard therefore
    disowns the old id and claims the new one in the same breath (see
    :meth:`~arm101.hardware.safety.TorqueGuard.disown`) — otherwise the release
    sweep would address a servo that no longer exists, fail, and report the motor
    as possibly-still-energised when in fact it is limp and merely renamed.

    The same hand-off also has to happen on the FAILURE path, one layer up:
    ``bus.write_id_baudrate`` can itself raise *after* the id write has
    already landed (its own EEPROM re-lock, addressed to the new id, runs
    outside the write's inner try/except — see that method's docstring). The
    ``except`` clause around the call therefore makes a best-effort, read-only
    probe (``bus.scan``) for whether the new id actually answers before
    re-raising, and moves the guard's claim only on positive evidence. See the
    inline comment at that ``except`` clause for the full reasoning.
    """
    # Re-detect the motor (fresh bus per motor).
    bus, port, detected_id = _detect_one_motor(args)
    try:
        # Guard nested INSIDE the bus try/finally: the release must run while
        # the bus is still open, or it writes to a closed port and frees nothing.
        with torque_guard(bus, (detected_id,), on_release=_announce_release) as guard:
            # Validate the optional --current-id assertion before any write.
            if asserted_current_id is not None and detected_id != asserted_current_id:
                raise CliError(
                    code=EXIT_USER_ERROR,
                    message=(
                        f"Expected motor at id {asserted_current_id} but detected id {detected_id}."
                    ),
                    remediation=(
                        "Ensure the correct motor is connected, or omit "
                        "--current-id to use the auto-detected id."
                    ),
                )

            # BEFORE card — read registers and show snapshot.
            before_info = bus.read_info(detected_id)
            emit_diagnostic(f"\n-- BEFORE write (motor {detected_id} → {motor_id}) --")
            _show_info(before_info, port)

            # Audit pending → write → audit success/failed.
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
            try:
                bus.write_id_baudrate(motor=detected_id, new_id=motor_id, baudrate=baudrate)
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
                # write_id_baudrate can raise AFTER the id write has already
                # LANDED. FeetechBus.write_id_baudrate (arm101/hardware/bus.py)
                # writes Baud_Rate then ID (addr 5) inside its own try/except;
                # once the ID write itself succeeds it falls through to the
                # SUCCESS path, which restores EEPROM write-protection with
                # `self._set_lock(relock_target, True)` from the `else:`
                # branch — OUTSIDE that inner try/except. If THAT relock call
                # raises, write_id_baudrate raises right along with it, even
                # though the servo has already moved to `motor_id`.
                #
                # If the guard is left owning `detected_id` in that case, the
                # abnormal-exit release sweep will address a bus id nothing
                # answers to any more: the release write fails, and the
                # operator is told a motor "did not respond and may still be
                # energised" when in truth it is limp and merely renamed. A
                # false alarm on a safety report is corrosive — see
                # TorqueGuard.disown's docstring — so before re-raising we
                # make a BEST-EFFORT check for whether the address actually
                # moved, and only then move the guard's claim with it.
                #
                # `scan([motor_id])` is a read-only ping sweep, not a register
                # read: it returns `[motor_id]` when something answers there
                # and `[]` when nothing does, with no exception on the "not
                # there" case — the natural primitive for "does this id exist
                # on the bus right now", unlike `read_info`, which assumes a
                # servo is present and raises when it is not.
                #
                # Wrapped in contextlib.suppress (never a bare
                # try/except/pass — bandit B110 fails CI lint on that) purely
                # so a probe FAILURE (the bus itself is unreachable, e.g. the
                # very fault that failed the write) cannot raise a SECOND
                # exception and replace the one already being handled. The
                # `raise` below is unconditional: it always re-raises the
                # ORIGINAL exception `e` untouched, whatever the probe did or
                # did not find.
                #
                # Ownership only moves on POSITIVE evidence — never both ids
                # "just in case". Owning an id nothing answers to is exactly
                # the false-alarm failure mode this exists to prevent;
                # claiming it unconditionally would just relocate the risk
                # instead of removing it.
                with contextlib.suppress(Exception):
                    if motor_id in bus.scan([motor_id]):
                        guard.own(motor_id)
                        guard.disown(detected_id)
                raise

            # The servo now answers at motor_id; detected_id is a dead address.
            # Move the guard's claim with it — see this function's docstring.
            guard.own(motor_id)
            guard.disown(detected_id)

            after_info = _after_read(bus, port, motor_id, baudrate)

            # Read-back verification: when the after-read succeeded, the reported
            # id must equal the id we just wrote. If it didn't persist (e.g. the
            # EEPROM Lock register was never opened — see PR #21), fail loudly
            # instead of letting the walk continue on a motor that silently
            # reverted to its old id.
            if after_info is not None and after_info["id"] != motor_id:
                error_message = (
                    f"motor id did not persist (read back {after_info['id']}, "
                    f"expected {motor_id}) — the EEPROM write may not have stuck"
                )
                _audit_write(
                    port,
                    operator,
                    mode,
                    motor_id,
                    joint_name,
                    detected_id,
                    "failed",
                    error=error_message,
                    baudrate=baudrate,
                )
                raise CliError(
                    code=EXIT_ENV_ERROR,
                    message=error_message,
                    remediation=(
                        "Check the motor's EEPROM Lock register; ensure only one motor "
                        "is connected and power is stable, then retry."
                    ),
                )

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

    return {
        "joint": joint_name,
        "from_id": detected_id,
        "new_id": motor_id,
        "baudrate": baudrate,
        "port": port,
        "detected_model": before_info["model"],
    }


def _run_walk(
    args: argparse.Namespace,
    *,
    mode: str,
    asserted_current_id: int | None,
    baudrate: int,
    operator: str,
    on_motor_assigned=None,
) -> list[dict[str, object]]:
    """Walk _MOTOR_ORDER, re-detecting the bus per motor.

    Each iteration gates the operator (:func:`_gate_operator`) then processes
    the motor (:func:`_process_one_motor`), which calls
    :func:`calibrate_motor._detect_one_motor` fresh — so USB re-enumeration
    between motors (``/dev/ttyACM*`` path changes) is handled transparently.

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
    on_motor_assigned:
        Optional callable ``(motor_id: int, joint_name: str, entry: dict) -> None``
        invoked after each successful motor write.  The *entry* dict carries
        ``joint``, ``from_id``, ``new_id``, ``baudrate``, ``port``, and
        ``detected_model``.  Defaults to ``None`` (no-op).

    Returns
    -------
    list[dict]
        One entry per motor written: ``{joint, from_id, new_id, baudrate,
        port, detected_model}``.
    """
    assigned: list[dict[str, object]] = []
    for motor_id, joint_name in _MOTOR_ORDER:
        _gate_operator(mode, joint_name, motor_id, asserted_current_id)
        entry = _process_one_motor(
            args,
            motor_id,
            joint_name,
            mode=mode,
            asserted_current_id=asserted_current_id,
            baudrate=baudrate,
            operator=operator,
        )
        assigned.append(entry)
        if on_motor_assigned is not None:
            on_motor_assigned(motor_id, joint_name, entry)
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
