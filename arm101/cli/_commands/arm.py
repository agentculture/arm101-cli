"""``arm101 arm`` — arm-level noun group (overview + read + flex + explore + rezero + setup).

Verbs
-----
``arm overview``
    Read-only snapshot of the arm surface: known roles, joints, and the
    per-role id / baud / servo_model / gear_ratio map from arm_spec.  Accepts
    an ignored positional ``target`` and always exits 0 on any path (rubric:
    descriptive verbs must not hard-fail on a bad path).  Supports ``--json``.

``arm read``
    Read every joint's live register state (position/load/speed/voltage/
    temperature/torque, plus the signed encoder ``offset``) via
    :func:`~arm101.hardware.arm_read.read_arm`.
    Read-only: it opens a bus and reads, but commands no motion and writes no
    register — so it carries NO consent gate.  Retry-tolerant: a joint whose
    reads keep failing is marked ``failed``/``partial`` while the rest still
    read.  Supports ``--role``, ``--port``, ``--json``.

    ``offset`` is the servo's ``Ofs``/``Homing_Offset`` (EEPROM addr 31), shown
    signed.  It exists here so a human can INSPECT the encoder re-zero that
    issue #35 needs for ``elbow_flex`` without writing anything; the write
    primitive lives on the bus (``MotorBus.write_offset``) and is not exposed
    as a verb by this task.

``arm flex``
    Gated motion: move one joint to ``--to <tick>``, or sweep every joint with
    ``--demo``.  ``--gentle`` uses the load-watch back-off-then-hold primitive
    (:func:`~arm101.hardware.gentle.gentle_move`) with an optional
    ``--threshold``; a plain move uses
    :func:`~arm101.hardware.motion.compliant_move`; ``--demo`` is inherently
    gentle (:func:`~arm101.hardware.demo.demo_sweep`).  Gated by the same
    three-mode consent as ``arm setup`` (dry_run / interactive / agent
    ``--apply`` — see :mod:`arm101.cli._consent`): dry-run plans the move(s)
    with zero motion and zero bus writes, interactive confirms at a prompt,
    and non-TTY ``--apply`` proceeds.

``arm explore``
    Gated motion: flood-fill and map the arm's reachable joint-space via
    :func:`~arm101.explore.engine.explore`, whose sole motion path is the
    overload-safe ``gentle_move``.  Writes two artifacts per run — an
    append-only JSONL event log (the resumable source of truth) and a
    derived, compact reachability map (per-joint ranges plus blocked
    combinations, queryable offline via
    :func:`~arm101.explore.reachmap.is_reachable`) — under ``--map`` (default
    ``./arm-explore-<role>.map.json``; resumes from an existing file).  When a
    joint is blocked, a bounded multi-joint escape search perturbs other
    joints to find combination-unblocks rather than stopping at the first
    single-joint contact.  Gated by the same three-mode consent as
    ``arm flex`` (dry_run / interactive / agent ``--apply``).  v1 produces and
    stores the map and lets it be queried; consuming it to gate ``arm flex``
    targets is a documented follow-up, not part of this verb.

``arm rezero <joint>``
    Gated **EEPROM write, and no motion at all**: shift the servo's encoder zero
    (``Ofs``/``Homing_Offset``, addr 31) so the 4095->0 seam falls in the arc the
    joint physically cannot reach — the fix for issue #35, and the only joint it
    applies to is ``elbow_flex``.  ``wrist_roll`` is REFUSED with the reason (a
    re-zero relocates a seam, it cannot evict one from a joint that turns all the
    way round; it has a soft limit instead), and so are the four joints that never
    wrap.  Gated by the same three-mode consent as ``arm flex`` (dry_run /
    interactive / agent ``--apply``); the dry-run touches no bus at all and prints
    the exact register writes.

    ``--verify`` runs the **seam-eviction proof** instead of the write: torque
    off, the human hand-moves the joint through its whole travel, and the verb
    polls ``present_position`` and asserts there is **no discontinuity anywhere**.
    Reading the offset back proves only that it was APPLIED; only the sweep proves
    the seam MOVED.  A discontinuity under a written offset is a STOP condition —
    the verb fails loudly (exit 2) because the re-zero then achieves nothing.  See
    :mod:`arm101.hardware.rezero`.

``arm setup <role>``
    Drive the existing setup-motors gated three-mode-consent walk (dry_run /
    interactive / agent — see :mod:`arm101.cli._consent`) for the given role
    (follower|leader).  All ids, baud, servo_model, and gear_ratio come from
    :mod:`arm101.hardware.arm_spec` — zero numbers typed by the operator.
    After each motor write the catalog entry is saved via
    :func:`~arm101.hardware.motor_catalog.save_entry` with the role-correct
    label (``F{id}`` / ``L{id}``).  Dry-run mode writes nothing to the catalog.

Torque ownership — every gated motion verb releases on an abnormal exit (#33)
----------------------------------------------------------------------------
``flex``, ``flex --demo``, and ``explore`` each wrap their whole run in a
:func:`~arm101.hardware.safety.torque_guard` owning the motors they may
energise. (``setup`` does too, one motor at a time, in
:func:`arm101.cli._commands.setup_motors._process_one_motor` — that is where its
per-motor bus is opened.) ``read`` does not: it energises nothing.

``rezero`` is guarded too, and it is the one verb that guards a motor it never
energises. That is not over-claiming for its own sake: it is a verb whose whole
job is to leave the joint LIMP — it de-energises before the EEPROM write (a
servo must not be *holding* while its frame of reference changes underneath it)
and again before the ``--verify`` sweep (a human is about to move the joint by
hand). A crash between the torque-off and the re-lock, or an operator's Ctrl-C
mid-sweep on a joint some earlier verb left hot, must both end with the motor
released, and the guard is what makes that true without rezero having to
re-implement it. The release is a no-op on an already-limp motor, so the guard
costs nothing on every path where nothing went wrong.

This exists because an ``arm explore`` run died on an unhandled
``serial.SerialException`` — a second process had opened the port — and left
**all six motors energised**, holding the arm up against gravity at ~50 C with
nobody watching. Nothing in these verbs owned torque as a resource: their
``finally`` closed the bus, and closing a bus does not de-energise a servo. Any
unhandled exception, bus fault, or ``Ctrl-C`` walked away from a powered arm.

The contract is **hold on success, release on abnormal**:

* A clean exit performs **zero** release writes. A successful move's deliberate
  stop-and-hold is preserved byte-for-byte — a gripper that has closed on an
  object must not drop it the instant the command returns.
* Any exception propagating out (including ``KeyboardInterrupt``) de-energises
  every owned motor, announces it on stderr, and lets the original exception
  through untouched.

Net effect: **a powered arm at process exit is always a deliberate state, never
an accident.** Note ``explore``'s engine also limps each joint BETWEEN probes
(:func:`arm101.explore.engine._release_joint`) to keep the bus healthy — that is
a different layer, and correct; the guard is the net under the whole run.

Bus injection seam
------------------
``read``/``flex`` resolve the serial port and open the bus through
:func:`calibrate_motor._open_bus` / :func:`calibrate_motor._candidate_ports`,
imported here as module-level names so tests can monkeypatch ``arm._open_bus``
/ ``arm._candidate_ports`` to inject a :class:`~arm101.hardware.bus.FakeBus`
without physical hardware.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Callable

from arm101.cli._commands import setup_motors as _setup_motors
from arm101.cli._commands.calibrate_motor import (  # noqa: F401 (bus/port seam)
    _candidate_ports,
    _open_bus,
    _prompt,
)
from arm101.cli._consent import resolve_consent, resolve_operator
from arm101.cli._errors import EXIT_ENV_ERROR, EXIT_USER_ERROR, CliError
from arm101.cli._output import emit_diagnostic, emit_result
from arm101.explore import engine
from arm101.explore.budget import DEFAULT_MAX_MOVES, Budget
from arm101.explore.types import GridSpec, JointConfig
from arm101.hardware import arm_spec, rezero
from arm101.hardware.arm_read import JointReading, is_complete, read_arm
from arm101.hardware.bus import OverloadError, encode_offset
from arm101.hardware.demo import demo_sweep
from arm101.hardware.gentle import gentle_move
from arm101.hardware.motion import compliant_move
from arm101.hardware.motor_catalog import MotorEntry, save_entry
from arm101.hardware.safety import ReleaseReport, torque_guard

#: Default gentle contact-load threshold for ``arm flex`` when ``--threshold``
#: is not supplied. (``arm explore`` no longer uses this constant — it
#: resolves a threshold PER JOINT via
#: :func:`arm101.hardware.arm_spec.resolve_contact_thresholds`, falling back
#: to :data:`arm101.hardware.arm_spec.DEFAULT_CONTACT_THRESHOLDS` rather than
#: one shared number.)
_DEFAULT_THRESHOLD = 250

#: Default per-joint grid bucket size (encoder ticks) for ``arm explore`` when
#: ``--resolution`` is not supplied. Coarse on purpose: the grid resolution is a
#: hardware-tuned open question (plan risk r2) — a large bucket keeps a first
#: real run bounded, and the shared Budget caps it regardless.
_DEFAULT_RESOLUTION = 512

#: Help text for the shared ``--json`` flag on every ``arm`` parser.
_JSON_HELP = "Emit structured JSON."

#: Placeholder shown for the port in a dry-run plan, where no bus is opened and
#: so no port has been resolved yet (hoisted; every gated motion verb shows it).
_PORT_UNRESOLVED = "(auto-detect at apply)"

#: Consent verb label for ``arm flex`` (hoisted to avoid duplicating the literal).
_FLEX_VERB = "arm flex"

#: Consent verb label for ``arm explore`` (hoisted to avoid duplicating the literal).
_EXPLORE_VERB = "arm explore"

#: Consent verb label for ``arm rezero`` (hoisted to avoid duplicating the literal).
_REZERO_VERB = "arm rezero"

#: Help text for the shared ``--role`` flag on read/flex/explore parsers
#: (hoisted to avoid duplicating the literal). ``setup``'s role help differs
#: intentionally (a required positional, no default clause).
_ROLE_HELP = "Arm role: follower or leader (default: follower)."

#: Help text for the shared ``--port`` flag on read/flex/explore parsers
#: (hoisted to avoid duplicating the literal). ``setup``'s port help differs
#: intentionally (it re-detects per motor across EEPROM writes).
_PORT_HELP = "Serial port (default: auto-detect the first candidate port)."

# ---------------------------------------------------------------------------
# arm overview
# ---------------------------------------------------------------------------


def cmd_arm_overview(args: argparse.Namespace) -> None:
    """Emit a read-only snapshot of the arm noun surface.

    Always exits 0 — descriptive verbs must not hard-fail on a bad path.
    """
    json_mode = bool(getattr(args, "json", False))

    roles_data: dict[str, object] = {}
    for role in arm_spec.roles():
        motors = arm_spec.role_motors(role)
        roles_data[role] = {
            joint: {
                "id": spec.id,
                "baud": spec.baud,
                "servo_model": spec.servo_model,
                "gear_ratio": spec.gear_ratio,
            }
            for joint, spec in motors.items()
        }

    payload: dict[str, object] = {
        "noun": "arm",
        "verbs": ["overview", "read", "flex", "explore", "rezero", "setup"],
        "roles": arm_spec.roles(),
        "motor_map": roles_data,
    }

    if json_mode:
        emit_result(payload, json_mode=True)
        return

    lines = [
        "## arm — arm-level operations",
        "",
        "Verbs: overview, read, flex, explore, rezero, setup",
        "",
        "Roles: " + ", ".join(arm_spec.roles()),
        "",
    ]
    for role in arm_spec.roles():
        lines.append(f"### {role}")
        lines.append("")
        lines.append("| joint | id | baud | servo_model | gear_ratio |")
        lines.append("|-------|-----|------|-------------|------------|")
        for joint, spec in arm_spec.role_motors(role).items():
            lines.append(
                f"| {joint} | {spec.id} | {spec.baud}"
                f" | {spec.servo_model} | {spec.gear_ratio} |"
            )
        lines.append("")
    emit_result("\n".join(lines), json_mode=False)


def _no_verb(args: argparse.Namespace) -> None:
    """Default handler: ``arm101 arm`` with no sub-verb prints the overview."""
    return cmd_arm_overview(args)


# ---------------------------------------------------------------------------
# Shared port/bus helpers (read + flex)
# ---------------------------------------------------------------------------


def _resolve_port(port_arg: "str | None") -> str:
    """Return *port_arg* if given, else the first auto-detected candidate port.

    Raises :class:`CliError(EXIT_ENV_ERROR)` if no port is given and none can be
    auto-detected — there is nothing to talk to.
    """
    if port_arg:
        return port_arg
    candidates = _candidate_ports()
    if candidates:
        return candidates[0]
    raise CliError(
        code=EXIT_ENV_ERROR,
        message="no serial port found",
        remediation="Connect the arm and retry, or name the port with --port /dev/ttyACMx.",
    )


# ---------------------------------------------------------------------------
# Torque ownership (every gated motion verb) — see arm101.hardware.safety
# ---------------------------------------------------------------------------


def _release_announcer(json_mode: bool) -> "Callable[[ReleaseReport], None]":
    """Build the ``on_release`` hook that TELLS the operator the arm was safed.

    A release only ever fires while an exception is unwinding, so the verb never
    reaches its ``emit_result``: without this hook the de-energising would be
    completely silent, and the human would be left staring at a
    ``SerialException`` with no idea whether the arm they cannot see is still
    holding itself up. Worse, the one outcome that genuinely needs a human —
    a motor the release could NOT reach, and which may therefore still be hot —
    would never be spoken aloud. :meth:`ReleaseReport.describe` says both.

    Goes to **stderr** (:func:`~arm101.cli._output.emit_diagnostic`), like every
    other diagnostic: stdout stays reserved for results, and the failure that
    triggered the release is on its way to stderr too. Under ``--json`` the same
    split holds and the line is emitted as a JSON object (from
    :meth:`ReleaseReport.as_dict`) rather than prose, so an agent parsing stderr
    gets a structured record and not a sentence wedged between JSON documents.

    A report with nothing attempted is NOT announced: the guard owning no motors
    means the run failed before anything could be energised, and "released no
    motors" is noise that would train the operator to skim past the line.
    """

    def announce(report: ReleaseReport) -> None:
        if not report.attempted:
            return
        if json_mode:
            emit_diagnostic(json.dumps({"torque_release": report.as_dict()}, ensure_ascii=False))
        else:
            emit_diagnostic(report.describe())

    return announce


def _role_motor_ids(role: str) -> "tuple[int, ...]":
    """Every motor id on *role*'s arm, in :data:`arm_spec.JOINTS` order."""
    ids = arm_spec.joint_ids(role)
    return tuple(ids[joint] for joint in arm_spec.JOINTS)


# ---------------------------------------------------------------------------
# arm read (read-only — no motion gate)
# ---------------------------------------------------------------------------


def _fmt_cell(value: "int | None") -> str:
    """Render a register value for the text table; ``None`` becomes ``-``."""
    return "-" if value is None else str(value)


def _emit_read(
    role: str,
    port: str,
    readings: "list[JointReading]",
    *,
    json_mode: bool,
) -> None:
    """Emit the whole-arm read snapshot as a text table or structured JSON."""
    complete = is_complete(readings)

    if json_mode:
        emit_result(
            {
                "role": role,
                "port": port,
                "complete": complete,
                "joints": [
                    {
                        "joint": r.joint,
                        "id": r.motor_id,
                        "health": r.health,
                        "overloaded": r.overloaded,
                        "position": r.position,
                        "load": r.load,
                        "speed": r.speed,
                        "voltage": r.voltage,
                        "temperature": r.temperature,
                        "torque": r.torque,
                        # Signed encoder offset (Ofs/Homing_Offset, EEPROM addr
                        # 31) — read-only. Issue #35 re-zeros elbow_flex by
                        # writing this; seeing it must never require writing it.
                        "offset": r.offset,
                    }
                    for r in readings
                ],
            },
            json_mode=True,
        )
        return

    lines = [
        f"## arm read ({role}) — {port}",
        "",
        "| joint | id | health | position | load | speed | voltage | temperature | torque"
        " | offset |",
        "|-------|----|--------|----------|------|-------|---------|-------------|--------"
        "|--------|",
    ]
    for r in readings:
        mark = " [OVERLOAD]" if r.overloaded else ""
        lines.append(
            f"| {r.joint} | {r.motor_id} | {r.health} | {_fmt_cell(r.position)}"
            f" | {_fmt_cell(r.load)} | {_fmt_cell(r.speed)} | {_fmt_cell(r.voltage)}"
            f" | {_fmt_cell(r.temperature)} | {_fmt_cell(r.torque)}"
            f" | {_fmt_cell(r.offset)} |{mark}"
        )
    lines.append("")

    failed = [r.joint for r in readings if r.health == "failed"]
    partial = [r.joint for r in readings if r.health == "partial"]
    overloaded = [r.joint for r in readings if r.overloaded]
    summary = f"Snapshot {'complete' if complete else 'incomplete'}: {len(readings)} joints"
    if failed:
        summary += f"; failed: {', '.join(failed)}"
    if partial:
        summary += f"; partial: {', '.join(partial)}"
    if overloaded:
        summary += f"; overloaded: {', '.join(overloaded)}"
    lines.append(summary)
    emit_result("\n".join(lines), json_mode=False)


def cmd_arm_read(args: argparse.Namespace) -> None:
    """Read every joint's live register state for *role* (read-only, no motion)."""
    role: str = args.role
    json_mode = bool(getattr(args, "json", False))

    port = _resolve_port(getattr(args, "port", None))
    bus = _open_bus(port)
    try:
        readings = read_arm(bus, arm_spec.joint_ids(role))
    finally:
        bus.close()

    _emit_read(role, port, readings, json_mode=json_mode)


# ---------------------------------------------------------------------------
# arm flex (gated motion)
# ---------------------------------------------------------------------------


def _validate_flex(joint: "str | None", target: "int | None", demo: bool) -> None:
    """Validate the flex argument combination; raise CliError(EXIT_USER_ERROR).

    Exactly one of ``{joint + --to}`` or ``{--demo}`` is required.
    """
    has_joint = joint is not None
    if demo and (has_joint or target is not None):
        raise CliError(
            code=EXIT_USER_ERROR,
            message="pass either a joint with --to, or --demo — not both",
            remediation="Run 'arm flex <joint> --to <tick>' OR 'arm flex --demo'.",
        )
    if not demo and not has_joint:
        raise CliError(
            code=EXIT_USER_ERROR,
            message="specify a joint with --to, or --demo",
            remediation="Run 'arm flex <joint> --to <tick>' OR 'arm flex --demo'.",
        )
    if has_joint:
        if joint not in arm_spec.JOINTS:
            raise CliError(
                code=EXIT_USER_ERROR,
                message=f"unknown joint {joint!r}",
                remediation=f"Valid joints: {', '.join(arm_spec.JOINTS)}.",
            )
        if target is None:
            raise CliError(
                code=EXIT_USER_ERROR,
                message="--to is required when a joint is given",
                remediation="Pass a target tick, e.g. 'arm flex shoulder_pan --to 2048'.",
            )


def _emit_flex_plan(
    role: str,
    joint: "str | None",
    target: "int | None",
    demo: bool,
    gentle: bool,
    threshold: int,
    *,
    port: "str | None",
    json_mode: bool,
) -> None:
    """Emit the dry-run plan for a flex move — zero motion, zero bus access."""
    port_display = port or _PORT_UNRESOLVED
    if demo:
        plan: dict[str, object] = {
            "verb": _FLEX_VERB,
            "role": role,
            "mode": "demo",
            "threshold": threshold,
            "port": port_display,
            "joints": list(arm_spec.JOINTS),
        }
    else:
        plan = {
            "verb": _FLEX_VERB,
            "role": role,
            "mode": "gentle" if gentle else "compliant",
            "joint": joint,
            "target": target,
            "threshold": threshold if gentle else None,
            "port": port_display,
            "note": "target is clamped to the joint's calibrated [min, max] at apply time",
        }

    if json_mode:
        emit_result({"plan": plan}, json_mode=True)
        return

    lines = ["## Dry-run plan: arm flex", ""]
    for key, value in plan.items():
        lines.append(f"- {key}: {value}")
    lines.append("")
    lines.append("No motion commanded (dry-run). Re-run non-interactively with --apply to execute.")
    emit_result("\n".join(lines), json_mode=False)


def _confirm_flex(
    role: str,
    joint: "str | None",
    target: "int | None",
    demo: bool,
    gentle: bool,
    *,
    json_mode: bool,
) -> bool:
    """Prompt the human; return True to proceed, False (and emit an abort) otherwise."""
    if demo:
        desc = f"a demo sweep of every {role} joint through a safe sub-range"
    else:
        kind = "gentle " if gentle else ""
        desc = f"a {kind}move of {role} {joint} to tick {target}"
    emit_diagnostic(f"⚠ This COMMANDS MOTION on the {role} arm: {desc}.")
    ans = _prompt("Type 'yes' to confirm motion")
    if ans.strip().lower() == "yes":
        return True
    if json_mode:
        emit_result({"aborted": True, "role": role}, json_mode=True)
    else:
        emit_result("Aborted; no motion commanded.", json_mode=False)
    return False


def _resolve_joint_bounds(joint: str, info: "dict[str, int]") -> "tuple[int, int]":
    """Turn one joint's ``read_info`` snapshot into the bounds a move may use.

    The single place in this module where a servo's EEPROM angle limits become
    move bounds — deliberately, so the soft limit cannot be forgotten at one
    call site and honoured at another. It intersects the EEPROM range with the
    joint's :data:`~arm101.hardware.arm_spec.SOFT_LIMITS` entry (see
    :func:`~arm101.hardware.arm_spec.resolve_bounds`): on this arm the EEPROM
    is the untouched factory ``0-4095`` on every joint, so for ``wrist_roll``
    — whose travel wraps the encoder seam — the EEPROM alone would happily
    permit a move into the dead arc and across the seam.

    The soft limit is read-side ONLY: this reads the servo's registers, it
    never writes the resolved range back into EEPROM.

    Raises
    ------
    CliError(EXIT_ENV_ERROR)
        If the servo's configured range and the joint's soft limit have no
        overlap at all (the servo is configured to live entirely inside the
        dead arc). That is a hardware/configuration contradiction, not a bad
        argument from the user — hence an ENV error, raised before any motion.
    """
    try:
        return arm_spec.resolve_bounds(joint, int(info["min_angle"]), int(info["max_angle"]))
    except ValueError as exc:
        raise CliError(
            code=EXIT_ENV_ERROR,
            message=str(exc),
            remediation=(
                f"Check {joint}'s min_angle/max_angle with 'arm101 arm read --json'; they "
                "contradict the joint's software travel limit, so no move is possible."
            ),
        ) from exc


def _execute_single(
    bus: object,
    role: str,
    joint: str,
    target: int,
    gentle: bool,
    threshold: int,
) -> dict[str, object]:
    """Run a single-joint move (gentle or compliant) and return its result dict."""
    motor_id = arm_spec.joint_ids(role)[joint]
    info = bus.read_info(motor_id)  # type: ignore[attr-defined]
    min_angle, max_angle = _resolve_joint_bounds(joint, info)
    if gentle:
        return gentle_move(
            bus,  # type: ignore[arg-type]
            motor_id,
            target,
            min_angle=min_angle,
            max_angle=max_angle,
            threshold=threshold,
            allow_motion=True,
        )
    return compliant_move(
        bus,  # type: ignore[arg-type]
        motor_id,
        target,
        min_angle=min_angle,
        max_angle=max_angle,
        allow_motion=True,
    )


def _emit_flex_move(
    role: str,
    port: str,
    joint: str,
    gentle: bool,
    move: dict[str, object],
    *,
    json_mode: bool,
) -> None:
    """Emit a single-joint flex result (text or JSON)."""
    if json_mode:
        emit_result(
            {"role": role, "port": port, "joint": joint, "gentle": gentle, "move": move},
            json_mode=True,
        )
        return
    kind = "gentle" if gentle else "compliant"
    lines = [f"## arm flex {joint} ({role}) — {kind} move on {port}", ""]
    for key, value in move.items():
        lines.append(f"- {key}: {value}")
    emit_result("\n".join(lines), json_mode=False)


def _emit_flex_demo(
    role: str,
    port: str,
    report: dict[str, object],
    *,
    json_mode: bool,
) -> None:
    """Emit a demo-sweep flex result (text or JSON)."""
    if json_mode:
        emit_result({"role": role, "port": port, "demo": report}, json_mode=True)
        return
    lines = [f"## arm flex --demo ({role}) — safe-exploration sweep on {port}", ""]
    visited: dict[str, dict[str, object]] = report["joints"]  # type: ignore[assignment]
    for joint_name, jr in visited.items():
        if jr["overloaded"]:
            mark = " [OVERLOAD]"
        elif jr["contacted"]:
            mark = " [CONTACT]"
        else:
            mark = ""
        lines.append(
            f"- {joint_name} (id {jr['motor']}): start={jr['start_position']}"
            f" attempted={jr['targets_attempted']} final={jr['final_position']}{mark}"
        )
    lines.append("")
    if report["aborted_on_overload"]:
        lines.append(f"Sweep aborted on overload at joint: {report['overloaded_joint']}.")
    elif report["aborted_on_contact"]:
        lines.append(f"Sweep aborted on contact at joint: {report['aborted_joint']}.")
    else:
        lines.append("Sweep completed with no contact or overload.")
    emit_result("\n".join(lines), json_mode=False)


def cmd_arm_flex(args: argparse.Namespace) -> None:
    """Command a bounded joint move (``--to``) or a demo sweep (``--demo``), gated."""
    role: str = args.role
    json_mode = bool(getattr(args, "json", False))
    joint: "str | None" = getattr(args, "joint", None)
    target: "int | None" = getattr(args, "to", None)
    demo = bool(getattr(args, "demo", False))
    gentle = bool(getattr(args, "gentle", False))
    # Explicit None check, NOT `or`: `--threshold 0` is a valid (falsy) override
    # and must not be silently replaced with the default.
    raw_threshold = getattr(args, "threshold", None)
    threshold: int = _DEFAULT_THRESHOLD if raw_threshold is None else int(raw_threshold)

    _validate_flex(joint, target, demo)

    mode = resolve_consent(args, verb=_FLEX_VERB, require_plan_hash=False)

    # --- dry_run: plan only, zero motion, zero bus access ---
    if mode == "dry_run":
        _emit_flex_plan(
            role,
            joint,
            target,
            demo,
            gentle,
            threshold,
            port=getattr(args, "port", None),
            json_mode=json_mode,
        )
        return

    # --- interactive: confirm at a prompt before any bus is opened ---
    if mode == "interactive":
        if not _confirm_flex(role, joint, target, demo, gentle, json_mode=json_mode):
            return

    # --- agent OR interactive-confirmed: open the bus and move ---
    port = _resolve_port(getattr(args, "port", None))

    # Motors this invocation MAY energise, claimed BEFORE the first bus write.
    # --demo sweeps every joint, so it owns all six even though a sweep that
    # dies on joint 3 never reached joints 4-6: the guard cannot know where the
    # run will stop, over-claiming costs nothing (releasing a limp motor is a
    # no-op), and under-claiming is the entire bug — issue #33 walked away from
    # six energised motors precisely because nothing owned them. A single-joint
    # move only ever energises its own joint, so it owns exactly that one.
    owned = _role_motor_ids(role) if demo else (arm_spec.joint_ids(role)[joint],)

    bus = _open_bus(port)
    try:
        # Nested INSIDE the bus try/finally so the guard's release runs while
        # the bus is still open — a release after bus.close() would write to a
        # closed port and de-energise nothing.
        with torque_guard(bus, owned, on_release=_release_announcer(json_mode)):
            if demo:
                report = demo_sweep(
                    bus,
                    arm_spec.joint_ids(role),
                    allow_motion=True,
                    threshold=threshold,
                )
                _emit_flex_demo(role, port, report, json_mode=json_mode)
            else:
                # joint/target are not-None here (guaranteed by _validate_flex).
                move = _execute_single(bus, role, joint, target, gentle, threshold)
                _emit_flex_move(role, port, joint, gentle, move, json_mode=json_mode)
    finally:
        bus.close()


# ---------------------------------------------------------------------------
# arm explore (gated motion — flood-fill reachability mapping)
# ---------------------------------------------------------------------------


def _explore_paths(map_arg: "str | None", role: str) -> "tuple[Path, Path]":
    """Resolve the ``(map_path, log_path)`` pair for an ``arm explore`` run.

    The map path is ``--map`` if given, else the per-role default
    ``./arm-explore-<role>.map.json``.  The JSONL event log is a sibling with
    the same base name and a ``.events.jsonl`` suffix (the engine resumes from
    this log, and derives the compact map from it).
    """
    if map_arg:
        map_path = Path(map_arg)
    else:
        map_path = Path(f"./arm-explore-{role}.map.json")
    name = map_path.name
    base = name
    for suffix in (".map.json", ".json"):
        if name.endswith(suffix):
            base = name[: -len(suffix)]
            break
    log_path = map_path.with_name(base + ".events.jsonl")
    return map_path, log_path


def _build_grid_spec(bus: object, role: str, resolution: int) -> GridSpec:
    """Read the live arm state and build the exploration :class:`GridSpec`.

    Each joint's live position seeds the grid origin (home), each joint's
    calibrated ``[min_angle, max_angle]`` — intersected with its software soft
    limit via :func:`_resolve_joint_bounds` — seeds the per-joint bounds, and
    *resolution* is the uniform per-joint bucket size.  Reads flow through
    ``bus.read_info`` — a per-joint read failure propagates as a
    :class:`CliError` (never a traceback), matching ``arm read``/``arm flex``.

    Soft-limiting the GRID is what soft-limits the whole exploration run: the
    engine takes every move bound it ever uses from ``GridSpec.bounds`` (both
    the flood-fill's neighbour moves and the multi-joint escape probes read
    ``spec.bounds[joint]``), so a bound that never crosses the encoder seam
    here cannot be crossed anywhere downstream.  The origin is then clamped
    into those same bounds — which matters concretely: the t9 hardware run
    found ``wrist_roll`` parked at raw tick 4, sitting ON the seam, and the
    flood-fill must start from a cell the joint is actually permitted to be in.
    """
    ids = arm_spec.joint_ids(role)
    origin_ticks: "list[int]" = []
    bounds: "list[tuple[int, int]]" = []
    for joint in arm_spec.JOINTS:
        info = bus.read_info(ids[joint])  # type: ignore[attr-defined]
        bound_min, bound_max = _resolve_joint_bounds(joint, info)
        position = max(bound_min, min(bound_max, int(info["present_position"])))
        origin_ticks.append(position)
        bounds.append((bound_min, bound_max))
    origin = JointConfig.from_ticks(origin_ticks)
    bucket_size = tuple(resolution for _ in arm_spec.JOINTS)
    return GridSpec(bucket_size=bucket_size, origin=origin, bounds=tuple(bounds))


def _make_temperature_provider(bus: object, role: str):
    """Return a zero-arg provider of live per-joint temperatures (deg C).

    Injected into :func:`arm101.explore.engine.explore` so the Budget thermal
    guard is live against real hardware.  A flaky read that raises a
    :class:`CliError` (including an ``OverloadError``) is swallowed by the
    engine's ``_read_temperatures`` — a temperature blip never breaks a run.
    """
    motor_ids = [arm_spec.joint_ids(role)[joint] for joint in arm_spec.JOINTS]

    def _read_temps() -> "list[int]":
        return [int(bus.read_info(mid)["present_temperature"]) for mid in motor_ids]  # type: ignore[attr-defined]  # noqa: E501

    return _read_temps


def _parse_threshold_joint_flags(raw: "list[str] | None") -> "dict[str, int]":
    """Parse repeated ``--threshold-joint NAME=VAL`` flags into a dict.

    Each entry is split on the first ``=``; the name is validated against
    :data:`arm_spec.JOINTS` and the value must parse as an int. Raises
    :class:`CliError(EXIT_USER_ERROR)` on any malformed entry, unknown joint,
    or non-integer value — this is user input, caught before any bus is
    opened.
    """
    result: "dict[str, int]" = {}
    if not raw:
        return result
    for entry in raw:
        name, sep, raw_value = entry.partition("=")
        name = name.strip()
        raw_value = raw_value.strip()
        if not sep:
            raise CliError(
                code=EXIT_USER_ERROR,
                message=f"invalid --threshold-joint {entry!r}: expected NAME=VALUE",
                remediation="Pass e.g. --threshold-joint shoulder_lift=350.",
            )
        if name not in arm_spec.JOINTS:
            raise CliError(
                code=EXIT_USER_ERROR,
                message=f"unknown joint {name!r} in --threshold-joint {entry!r}",
                remediation=f"Valid joints: {', '.join(arm_spec.JOINTS)}.",
            )
        try:
            value = int(raw_value)
        except ValueError as exc:
            raise CliError(
                code=EXIT_USER_ERROR,
                message=f"invalid threshold value {raw_value!r} in --threshold-joint {entry!r}",
                remediation="Pass an integer, e.g. --threshold-joint shoulder_lift=350.",
            ) from exc
        result[name] = value
    return result


_THRESHOLD_FILE_LINE_HELP = (
    'Each line must be a JSON object: {"joint": "<name>", "threshold": <int>}.'
)


def _parse_threshold_file_line(line: str, line_no: int, path: str) -> "tuple[str, int]":
    """Parse+validate one JSONL threshold line into a ``(joint, threshold)`` pair.

    Raises :class:`CliError(EXIT_USER_ERROR)` naming *line_no* on malformed
    JSON, a missing ``joint``/``threshold`` key, an unknown joint name, or a
    non-int threshold (``bool`` excluded — it is an ``int`` subclass). Split
    out of :func:`_parse_threshold_file` so the per-line validation branches
    don't inflate that function's cognitive complexity.
    """
    try:
        obj = json.loads(line)
    except json.JSONDecodeError as exc:
        raise CliError(
            code=EXIT_USER_ERROR,
            message=f"--threshold-file {path}: malformed JSON on line {line_no}: {exc}",
            remediation=_THRESHOLD_FILE_LINE_HELP,
        ) from exc
    if not isinstance(obj, dict) or "joint" not in obj or "threshold" not in obj:
        raise CliError(
            code=EXIT_USER_ERROR,
            message=f"--threshold-file {path}: line {line_no} missing 'joint'/'threshold'",
            remediation=_THRESHOLD_FILE_LINE_HELP,
        )
    joint = obj["joint"]
    if joint not in arm_spec.JOINTS:
        raise CliError(
            code=EXIT_USER_ERROR,
            message=f"--threshold-file {path}: line {line_no} names unknown joint {joint!r}",
            remediation=f"Valid joints: {', '.join(arm_spec.JOINTS)}.",
        )
    value = obj["threshold"]
    # bool is an int subclass in Python — exclude it explicitly so
    # {"threshold": true} is rejected rather than silently coerced to 1.
    if not isinstance(value, int) or isinstance(value, bool):
        raise CliError(
            code=EXIT_USER_ERROR,
            message=(
                f"--threshold-file {path}: line {line_no} threshold must be "
                f"an int, got {value!r}"
            ),
            remediation=_THRESHOLD_FILE_LINE_HELP,
        )
    return joint, value


def _parse_threshold_file(path: "str | None") -> "dict[str, int]":
    """Parse a JSONL ``--threshold-file`` into a ``{joint: threshold}`` dict.

    Each non-blank line must be a JSON object ``{"joint": "<name>",
    "threshold": <int>}``. A missing path is a user-input error
    (``EXIT_USER_ERROR``); an existing-but-unreadable file is an environment
    error (``EXIT_ENV_ERROR``); a malformed line, unknown joint name, or
    non-int threshold is a user-input error naming the offending line number.
    """
    if not path:
        return {}
    file_path = Path(path)
    if not file_path.exists():
        raise CliError(
            code=EXIT_USER_ERROR,
            message=f"--threshold-file not found: {path}",
            remediation=(
                "Pass a path to an existing JSONL threshold file, or omit --threshold-file."
            ),
        )
    try:
        text = file_path.read_text(encoding="utf-8")
    except OSError as exc:
        raise CliError(
            code=EXIT_ENV_ERROR,
            message=f"failed to read --threshold-file {path}: {exc}",
            remediation="Check file permissions and try again.",
        ) from exc

    result: "dict[str, int]" = {}
    for line_no, raw_line in enumerate(text.splitlines(), start=1):
        line = raw_line.strip()
        if not line:
            continue
        joint, value = _parse_threshold_file_line(line, line_no, path)
        result[joint] = value
    return result


def _resolve_explore_thresholds(args: argparse.Namespace) -> "dict[str, int]":
    """Resolve the per-joint contact-threshold map for an ``arm explore`` run.

    Reads/parses ``--threshold`` (blanket), ``--threshold-joint`` (repeatable
    per-joint), and ``--threshold-file`` (JSONL) off *args*, then resolves
    them via :func:`arm_spec.resolve_contact_thresholds` (precedence:
    per-joint flag > blanket flag > file > built-in default). Any
    :class:`ValueError` the resolver raises (an unknown joint slipping
    through) is translated into a :class:`CliError`.
    """
    # Explicit None check, NOT `or`: an explicit blanket override (e.g.
    # ``--threshold 0``) must broadcast to every joint that --threshold-joint
    # doesn't already cover; ``None`` (the flag simply absent) must NOT
    # collapse every joint to a fixed number — each joint instead falls
    # through to --threshold-file / its built-in per-joint default.
    raw_threshold = getattr(args, "threshold", None)
    blanket: "int | None" = None if raw_threshold is None else int(raw_threshold)

    per_joint = _parse_threshold_joint_flags(getattr(args, "threshold_joint", None))
    from_file = _parse_threshold_file(getattr(args, "threshold_file", None))

    try:
        resolved = arm_spec.resolve_contact_thresholds(
            blanket=blanket, per_joint=per_joint, from_file=from_file
        )
    except ValueError as exc:
        raise CliError(
            code=EXIT_USER_ERROR,
            message=str(exc),
            remediation=f"Valid joints: {', '.join(arm_spec.JOINTS)}.",
        ) from exc

    return dict(zip(arm_spec.JOINTS, resolved))


def _emit_explore_plan(
    role: str,
    map_path: Path,
    log_path: Path,
    thresholds: "dict[str, int]",
    resolution: int,
    max_moves: "int | None",
    *,
    port: "str | None",
    json_mode: bool,
) -> None:
    """Emit the dry-run plan for an explore run — zero motion, zero bus access."""
    plan: "dict[str, object]" = {
        "verb": _EXPLORE_VERB,
        "role": role,
        "port": port or _PORT_UNRESOLVED,
        "map_path": str(map_path),
        "log_path": str(log_path),
        "thresholds": thresholds,
        "resolution": resolution,
        "max_moves": DEFAULT_MAX_MOVES if max_moves is None else int(max_moves),
        "note": (
            "COMMANDS MOTION: flood-fills the reachable joint-space via the "
            "overload-safe gentle_move, outward from the live home pose read at "
            "apply time."
        ),
    }

    if json_mode:
        emit_result({"plan": plan}, json_mode=True)
        return

    lines = ["## Dry-run plan: arm explore", ""]
    for key, value in plan.items():
        lines.append(f"- {key}: {value}")
    lines.append("")
    lines.append("No motion commanded (dry-run). Re-run non-interactively with --apply to execute.")
    emit_result("\n".join(lines), json_mode=False)


def _confirm_explore(role: str, *, json_mode: bool) -> bool:
    """Prompt the human before an explore run; return True to proceed."""
    emit_diagnostic(
        f"⚠ This COMMANDS MOTION on the {role} arm: a flood-fill exploration of "
        "reachable joint-space (many gentle moves)."
    )
    ans = _prompt("Type 'yes' to confirm motion")
    if ans.strip().lower() == "yes":
        return True
    if json_mode:
        emit_result({"aborted": True, "role": role}, json_mode=True)
    else:
        emit_result("Aborted; no motion commanded.", json_mode=False)
    return False


def _emit_explore_result(
    role: str,
    port: str,
    result: "engine.ExploreResult",
    *,
    json_mode: bool,
) -> None:
    """Render an :class:`~arm101.explore.engine.ExploreResult` (text or JSON)."""
    if json_mode:
        emit_result(
            {
                "verb": _EXPLORE_VERB,
                "role": role,
                "port": port,
                "cells_visited": result.cells_visited,
                "moves": result.moves,
                "reachable": result.reachable,
                "contacts": result.contacts,
                "escapes_attempted": result.escapes_attempted,
                "escapes_succeeded": result.escapes_succeeded,
                "budget_bounded": result.budget_bounded,
                "errors": result.errors,
                "map_path": result.map_path,
                "log_path": result.log_path,
            },
            json_mode=True,
        )
        return

    lines = [
        f"## arm explore ({role}) — {port}",
        "",
        f"- cells visited: {result.cells_visited}",
        f"- moves: {result.moves}",
        f"- reachable: {result.reachable}",
        f"- contacts: {result.contacts}",
        f"- escapes: {result.escapes_succeeded}/{result.escapes_attempted} succeeded",
        f"- budget-bounded: {result.budget_bounded}",
        f"- skipped (comm errors): {result.errors}",
        "",
        f"Map written to: {result.map_path}",
        f"Event log written to: {result.log_path}",
    ]
    emit_result("\n".join(lines), json_mode=False)


def cmd_arm_explore(args: argparse.Namespace) -> None:
    """Flood-fill and map the reachable joint-space for *role* — gated motion.

    Drives :func:`arm101.explore.engine.explore` (whose sole motion path is the
    overload-safe ``gentle_move``), writing both a JSONL event log and a compact
    reachability map, resumable across runs.  Gated by the same three-mode
    consent as ``arm flex`` (dry_run / interactive / agent ``--apply``).
    """
    role: str = args.role
    json_mode = bool(getattr(args, "json", False))
    thresholds_by_joint = _resolve_explore_thresholds(args)
    raw_resolution = getattr(args, "resolution", None)
    resolution: int = _DEFAULT_RESOLUTION if raw_resolution is None else int(raw_resolution)
    if resolution <= 0:
        # A zero/negative bucket size divides by zero in the grid math — reject
        # it as user input up front, before opening the bus or prompting.
        raise CliError(
            code=EXIT_USER_ERROR,
            message=f"--resolution must be a positive number of ticks (got {resolution}).",
            remediation="Pass a positive --resolution (e.g. 256 or 512), or omit it.",
        )
    raw_max_moves = getattr(args, "max_moves", None)

    map_path, log_path = _explore_paths(getattr(args, "map", None), role)

    mode = resolve_consent(args, verb=_EXPLORE_VERB, require_plan_hash=False)

    # --- dry_run: plan only, zero motion, zero bus access ---
    if mode == "dry_run":
        _emit_explore_plan(
            role,
            map_path,
            log_path,
            thresholds_by_joint,
            resolution,
            raw_max_moves,
            port=getattr(args, "port", None),
            json_mode=json_mode,
        )
        return

    # --- interactive: confirm at a prompt before any bus is opened ---
    if mode == "interactive" and not _confirm_explore(role, json_mode=json_mode):
        return

    # --- agent OR interactive-confirmed: open the bus and explore ---
    port = _resolve_port(getattr(args, "port", None))
    bus = _open_bus(port)
    try:
        # The guard starts owning NOTHING on purpose. _build_grid_spec only
        # READS registers (positions and calibrated bounds) — it energises no
        # motor — so a bus fault there has nothing to release, and claiming the
        # arm up front would make the guard announce "torque released on motors
        # 1-6" for six motors that were never hot. A safety report that cries
        # wolf is worse than none.
        with torque_guard(bus, on_release=_release_announcer(json_mode)) as guard:
            spec = _build_grid_spec(bus, role, resolution)
            budget = Budget() if raw_max_moves is None else Budget(max_moves=int(raw_max_moves))

            # From this line on, motion is possible — so claim the WHOLE arm.
            # explore's joints go hot progressively (the flood-fill energises
            # one joint per probe and limps it again afterwards; the escape
            # search HOLDS several joints perturbed while it probes another),
            # but the engine offers no per-move callback, so the CLI cannot
            # observe which joints are live at the instant a fault strikes. It
            # does not need to: the release is per-motor independent and a
            # release write to an already-limp motor is a harmless no-op, so
            # owning all six is both correct and free — whereas owning only the
            # joint we *think* is moving would strand the ones the escape search
            # is holding. That is the exact shape of the incident: the crash
            # left ALL SIX energised, not one.
            guard.own(*_role_motor_ids(role))

            result = engine.explore(
                bus,
                spec,
                log_path=log_path,
                map_path=map_path,
                thresholds=tuple(thresholds_by_joint[joint] for joint in arm_spec.JOINTS),
                budget=budget,
                temperatures=_make_temperature_provider(bus, role),
            )
    finally:
        bus.close()

    _emit_explore_result(role, port, result, json_mode=json_mode)


# ---------------------------------------------------------------------------
# arm rezero (gated EEPROM write — and NOT a move)
# ---------------------------------------------------------------------------


def _rezero_write_sequence(offset: int) -> "list[str]":
    """The exact wire writes ``rezero`` will perform, as the dry-run prints them.

    Rendered from the same :func:`~arm101.hardware.bus.encode_offset` the write
    itself uses, so the plan cannot claim one wire value and the write send
    another. Note what is NOT in the list: no goal position, at any point. See
    :mod:`arm101.hardware.rezero` for why commanding motion here would rotate
    ``elbow_flex`` the long way round, through its whole travel, into a wall.
    """
    wire = encode_offset(offset)
    return [
        "write1ByteTxRx(addr=40, value=0)  # Torque_Enable OFF — the servo must not be "
        "holding while its own frame of reference moves",
        "write1ByteTxRx(addr=55, value=0)  # Lock OPEN — without this the write reads back "
        "fine and REVERTS on the next power-cycle (PR #21)",
        f"write2ByteTxRx(addr=31, value={wire})  # Ofs/Homing_Offset = {offset:+d} "
        "(sign-magnitude on bit 11), and NOTHING else — not min/max angle limits",
        "write1ByteTxRx(addr=55, value=1)  # Lock CLOSED — restore write-protection",
        "(no goal position is ever written — this verb commands NO motion)",
    ]


def _emit_rezero_plan(
    role: str,
    joint: str,
    motor: int,
    offset: int,
    arc: "arm_spec.UnreachableArc",
    verify: bool,
    duration: float,
    *,
    port: "str | None",
    json_mode: bool,
) -> None:
    """Emit the dry-run plan — **zero bus access**, not merely zero writes.

    Deliberately offline, exactly like ``arm flex``'s and ``arm explore``'s
    dry-runs: everything a plan can honestly say about a re-zero is already
    known without a servo (the offset is derived from the arc table, the wire
    sequence from the offset), and everything it CANNOT say offline — what the
    joint currently reads, what offset it already holds, whether it is somewhere
    it should not be — is a live fact that is checked at apply time, where it can
    actually be acted on. A plan that opened a port would fail on a laptop and
    still not know any more than this one does. Use ``arm read`` to see the live
    registers, including the ``offset`` column.
    """
    plan: "dict[str, object]" = {
        "verb": _REZERO_VERB,
        "role": role,
        "joint": joint,
        "motor": motor,
        "port": port or _PORT_UNRESOLVED,
        "mode": "verify" if verify else "write",
    }
    if verify:
        plan.update(
            {
                "duration_s": duration,
                "note": (
                    "COMMANDS NO MOTION, and DE-ENERGISES the joint: torque goes off and "
                    "STAYS off while YOU hand-move it through its entire travel. An arm "
                    "holding a pose will sag when its torque is released — support it. The "
                    "sweep polls present_position and asserts there is no discontinuity "
                    "anywhere; a discontinuity means the re-zero did not evict the seam."
                ),
                "writes": ["write1ByteTxRx(addr=40, value=0)    # Torque_Enable OFF"],
            }
        )
    else:
        plan.update(
            {
                "current_offset": "(read at apply)",
                "target_offset": offset,
                "wire_value": encode_offset(offset),
                "register": f"addr {31} (Ofs/Homing_Offset, EEPROM, 2 bytes)",
                "unreachable_arc": [arc.low, arc.high],
                "seam_moves_to_raw_tick": arc.midpoint,
                "expected_travel_ticks": arc.travel_ticks,
                "note": (
                    "COMMANDS NO MOTION. This is a persistent EEPROM write: it shifts the "
                    "joint's encoder zero so the 4095->0 seam falls inside the arc the "
                    "joint cannot reach. The joint is NOT moved — not before, not during, "
                    "not after."
                ),
                "writes": _rezero_write_sequence(offset),
            }
        )

    if json_mode:
        emit_result({"plan": plan}, json_mode=True)
        return

    lines = [f"## Dry-run plan: {_REZERO_VERB} {joint}", ""]
    for key, value in plan.items():
        if key == "writes":
            continue
        lines.append(f"- {key}: {value}")
    lines += ["", "### Register writes", ""]
    lines += [f"    {line}" for line in plan["writes"]]  # type: ignore[union-attr]
    lines += [
        "",
        "No motion commanded, and no bus opened (dry-run). Re-run non-interactively "
        "with --apply to execute.",
    ]
    emit_result("\n".join(lines), json_mode=False)


def _confirm_rezero(joint: str, motor: int, offset: int, verify: bool, *, json_mode: bool) -> bool:
    """Prompt the human; return True to proceed, False (and emit an abort) otherwise."""
    if verify:
        emit_diagnostic(
            f"⚠ This DE-ENERGISES {joint} (motor {motor}) and LEAVES IT LIMP: torque goes "
            "off and stays off while you hand-move the joint through its entire travel. "
            "If the arm is holding a pose, SUPPORT IT — it will sag. No motion is "
            "commanded."
        )
    else:
        emit_diagnostic(
            f"⚠ This writes {joint}'s (motor {motor}) encoder offset to EEPROM: "
            f"Ofs = {offset:+d} at addr 31. PERSISTENT — it changes every position the "
            "servo reports, for good. Torque is disabled first and left off. No motion is "
            "commanded."
        )
    ans = _prompt("Type 'yes' to confirm")
    if ans.strip().lower() == "yes":
        return True
    if json_mode:
        emit_result({"aborted": True, "joint": joint}, json_mode=True)
    else:
        emit_result("Aborted; nothing written, no motion commanded.", json_mode=False)
    return False


def _emit_rezero_write(
    role: str,
    port: str,
    plan: "rezero.RezeroPlan",
    read_back: int,
    shift: "dict[str, object]",
    *,
    json_mode: bool,
) -> None:
    """Emit the result of a successful offset write, and tell the operator what is left.

    Two things remain undone at this point, and neither is optional:

    * The write is proven **applied** (it read back) but not proven
      **persistent** — PR #21 exists because id/baud writes read back correctly
      and silently reverted on the next power-cycle. Only a power-cycle proves it.
    * The offset is proven applied but the seam is not proven **moved**. Only
      ``--verify`` proves that.

    So the result does not end on a success line; it ends on the next two steps.
    """
    next_steps = [
        "1. POWER-CYCLE the servo — cut and restore BUS POWER (not just the USB/serial "
        "link). An EEPROM write can read back correctly and still revert on the next "
        "power-up if the Lock register was mishandled; that is PR #21, and it is the only "
        "way to know.",
        f"2. Re-read the offset: 'arm101 arm read --json' — {plan.joint}'s 'offset' must "
        f"still be {plan.target_offset}. If it reverted to 0, the write did not persist.",
        f"3. PROVE THE SEAM MOVED: 'arm101 arm rezero {plan.joint} --verify'. The read-back "
        "above proves only that the offset was APPLIED. It does NOT prove the seam "
        "RELOCATED — only a torque-off sweep of the joint's whole travel can.",
    ]

    if json_mode:
        emit_result(
            {
                "verb": _REZERO_VERB,
                "role": role,
                "port": port,
                "plan": plan.as_dict(),
                "read_back_offset": read_back,
                "applied": read_back == plan.target_offset,
                "shift": shift,
                "persistence_proven": False,
                "seam_eviction_proven": False,
                "next_steps": next_steps,
            },
            json_mode=True,
        )
        return

    lines = [
        f"## arm rezero {plan.joint} ({role}) — encoder offset written on {port}",
        "",
        f"- motor            : {plan.motor}",
        f"- offset before    : {plan.current_offset}",
        f"- offset written   : {plan.target_offset:+d} (wire value "
        f"{encode_offset(plan.target_offset)}, EEPROM addr 31)",
        f"- offset read back : {read_back}  <- the write LANDED",
        f"- raw position     : {plan.raw_position} (unchanged — no motion was commanded)",
        f"- reported before  : {plan.reported_position}",
        f"- reported after   : {shift['observed_position']}"
        f"  (predicted {shift['predicted_position']}, delta {shift['delta']})",
        "",
    ]

    if not shift["in_range"]:
        lines += [
            "*** WARNING — the servo now reports a position OUTSIDE [0, 4095]. That is a "
            "value the position register cannot hold, which means the corrected position "
            "is an UNWRAPPED signed subtraction: the seam has NOT moved and this re-zero "
            "achieves nothing. Run --verify to confirm, then stop and re-decide. ***",
            "",
        ]
    elif shift["unchanged"]:
        lines += [
            "*** WARNING — the reported position did not change. The offset register took "
            "the value (it read back), but the servo is not applying it to what it "
            "reports. Run --verify to confirm, then stop and re-decide. ***",
            "",
        ]
    elif not shift["as_predicted"]:
        lines += [
            f"*** WARNING — the reported position moved, but not to the predicted "
            f"{shift['predicted_position']} (delta {shift['delta']} ticks). The joint is "
            "limp, so a few ticks of gravity/backlash are expected; this is more than "
            "that. Run --verify before trusting the frame. ***",
            "",
        ]

    lines += ["### What is NOT yet proven", ""] + next_steps
    emit_result("\n".join(lines), json_mode=False)


def _sweep_progress(json_mode: bool, every: int = 10) -> "Callable[[int, int], None]":
    """Build the ``on_sample`` hook that shows the operator the joint moving.

    A human hand-moving a limp joint for 30 seconds with no feedback has no way
    to tell "I am driving the joint and the tool is watching" apart from "the
    tool wedged and I am wobbling a dead arm". Both look identical from where
    they are standing, and the second one silently produces a useless sweep.

    Goes to **stderr** (:func:`~arm101.cli._output.emit_diagnostic`), like every
    other diagnostic — stdout is reserved for the one result document, and under
    ``--json`` that matters: a progress line interleaved into stdout would wedge
    a partial JSON object between the reader and the report. Throttled to every
    *every*-th sample (~2 lines/second at the default poll interval), because a
    line per 50 ms poll is not feedback, it is a waterfall.
    """

    def announce(index: int, position: int) -> None:
        if index % every:
            return
        if json_mode:
            emit_diagnostic(json.dumps({"sample": index, "position": position}))
        else:
            emit_diagnostic(f"  sample {index:>4}   position {position:>6}")

    return announce


def _run_rezero_verify(
    bus: object,
    role: str,
    port: str,
    joint: str,
    motor: int,
    duration: float,
    *,
    json_mode: bool,
) -> None:
    """Run the seam-eviction sweep and emit its report — raising on the STOP condition.

    The report goes to **stdout on every path**, including the failure path, and
    is emitted BEFORE the :class:`CliError` is raised. That ordering is the
    point: the numbers are why the operator ran the command, and they are exactly
    as valuable when the answer is "the fix does not work" as when it is "the fix
    works". Failing without showing them would leave a human standing at an arm
    with a non-zero exit code and nothing to take back to the decision.

    Raises
    ------
    CliError(EXIT_ENV_ERROR)
        If the sweep found a discontinuity **while the seam-evicting offset was
        in force** — i.e. the servo does not reduce the corrected position modulo
        4096, the seam never moved, and the whole re-zero approach to issue #35
        is dead. This is a stop-and-return-to-the-user condition, not a retryable
        error, and it exits non-zero so that no script can mistake it for
        success.
    """
    emit_diagnostic(
        f"Torque is now OFF on {joint} (motor {motor}) and will STAY off.\n"
        f"Hand-move the joint SLOWLY through its ENTIRE travel — from one hard stop all "
        f"the way to the other — for the next {duration:.0f} seconds. Do not hurry: "
        f"hurrying is how a seam crossing hides between two samples."
    )

    report = rezero.sweep(
        bus,  # type: ignore[arg-type]
        motor,
        joint,
        samples=rezero.samples_for(duration),
        on_sample=_sweep_progress(json_mode),
    )

    if json_mode:
        emit_result(
            {"verb": _REZERO_VERB, "role": role, "port": port, "sweep": report.as_dict()},
            json_mode=True,
        )
    else:
        emit_result(
            f"## arm rezero {joint} --verify ({role}) — seam-eviction sweep on {port}\n\n"
            + report.describe(),
            json_mode=False,
        )

    if report.failed:
        raise CliError(
            code=EXIT_ENV_ERROR,
            message=(
                f"SEAM NOT EVICTED: {joint} carries the seam-evicting offset "
                f"({report.offset_in_force}) and its reported position still jumps "
                f"{report.largest_jump} ticks mid-travel. The servo does NOT reduce the "
                "corrected position modulo 4096 — the offset merely relabels positions and "
                "the discontinuity stays pinned to the physical angle where the magnet "
                "rolls over. The re-zero does not fix issue #35."
            ),
            remediation=(
                "STOP — do not build on this. The one undocumented assumption behind the "
                "re-zero (docs/spikes/sts3215-offset-register.md, section 4) has resolved "
                "against us, and the fix needs a new approach, not a retry. Take the sweep "
                "report above back to the user for a re-decision. The remaining options are "
                "a software-only soft limit (as wrist_roll uses) or unwrapping the encoder "
                "in software; neither is this verb's to choose."
            ),
        )


def _run_rezero_write(
    bus: object,
    role: str,
    port: str,
    joint: str,
    motor: int,
    *,
    json_mode: bool,
) -> None:
    """Plan, write, and read back the encoder offset. Commands NO motion.

    ``plan_rezero`` looks READ-ONLY — ``read_offset`` then ``read_position``,
    no torque write, no EEPROM — and on most servos that would mean it is also
    overload-*proof*. It is not, here. ``FeetechBus._read_register`` raises
    through ``_status_error`` whenever the returned status byte reports a
    non-zero error, and ``_status_error`` hands back an ``OverloadError``
    specifically when the overload bit (0x20) is set — a property of the
    STATUS BYTE that comes back with the reply, not of which register or
    which direction (read vs. write) the packet asked for. A motor latched in
    overload therefore fails a read exactly as it fails a write, and
    ``plan_rezero`` would raise before this verb ever reached
    ``apply_rezero`` — the only place that calls ``bus.clear_overload``.

    That is not a corner case worth shrugging off: ``elbow_flex``'s
    unreachable arc (the whole reason this joint is re-zeroable) was measured
    by driving the joint into a wall, which is precisely how a Feetech servo
    latches an overload. An operator who has just finished that measurement —
    exactly the order ``docs/hardware-rezero-procedure.md`` describes — would
    hit this every single time, on the one joint the verb exists to fix.

    Raises
    ------
    CliError(EXIT_ENV_ERROR)
        If the offset read back from EEPROM is not the one written — the write
        did not take, and every position the servo reports from here is in a
        frame nobody chose.
    """
    try:
        plan = rezero.plan_rezero(bus, motor, joint)  # type: ignore[arg-type]
    except OverloadError:
        # Recover exactly once, and ONLY here — inside the except, never ahead
        # of the `try`. `clear_overload` is `enable_torque(motor, False)`
        # under the hood: it de-energises the joint as its side effect of
        # clearing the latch. Calling it unconditionally, before every plan,
        # would silently drop torque on the common/no-op path too — including
        # the case where `plan_rezero` finds the offset `already_applied` and
        # nothing is ever written — de-energising a joint that was holding
        # its pose just fine for no reason connected to anything that went
        # wrong. The overload branch is the only place this verb has actual
        # evidence the joint is latched, so it is the only place allowed to
        # pay that de-energising cost.
        emit_diagnostic(
            f"{joint} (motor {motor}) was latched in an overload fault while reading "
            "its live state for the re-zero plan. Clearing the latch now: torque is "
            "OFF and the joint is LIMP as a direct result. This is the expected "
            "recovery — not a malfunction — for a joint that was just driven into "
            "its unreachable arc, which is how that arc was measured in the first "
            "place; it is not a surprise this verb should spring on an operator who "
            "did not ask for it."
        )
        bus.clear_overload(motor)  # type: ignore[attr-defined]
        # One retry, no loop: if the servo is still latched after a torque
        # release, the fault is not the transient kind `clear_overload` is
        # documented to clear, and spinning on it would just hang against a
        # servo that is never going to answer differently.
        plan = rezero.plan_rezero(bus, motor, joint)  # type: ignore[arg-type]

    if plan.already_applied:
        _emit_rezero_noop(role, port, plan, json_mode=json_mode)
        return

    read_back = rezero.apply_rezero(bus, motor, plan.target_offset)  # type: ignore[arg-type]
    if read_back != plan.target_offset:
        raise CliError(
            code=EXIT_ENV_ERROR,
            message=(
                f"The encoder offset did NOT take: wrote {plan.target_offset} to "
                f"{joint} (motor {motor}, EEPROM addr 31), read back {read_back}."
            ),
            remediation=(
                "The servo accepted the write but is not holding the value. Check the "
                "EEPROM Lock register (addr 55) is not being re-closed by another process, "
                "and that motor "
                f"{motor} is the servo you think it is ('arm101 arm read'). The Lock dance "
                "is exactly what PR #21 was written to fix."
            ),
        )

    # Free, and already decisive in two of its outcomes: does the servo's OWN
    # report move the way a modular correction would move it?
    observed = bus.read_position(motor)  # type: ignore[attr-defined]
    shift = rezero.describe_shift(plan, observed)
    _emit_rezero_write(role, port, plan, read_back, shift, json_mode=json_mode)


def _emit_rezero_noop(
    role: str,
    port: str,
    plan: "rezero.RezeroPlan",
    *,
    json_mode: bool,
) -> None:
    """Report that the servo already holds the target offset — and write nothing.

    Idempotence matters here more than it usually does: the procedure this verb
    belongs to tells the operator to power-cycle the arm and come back, so a
    second run against an already-re-zeroed joint is the *expected* path, not a
    mistake. Re-writing the same value would be harmless on the wire and
    corrosive in the log — it would make "the offset was written" ambiguous about
    which run wrote it.
    """
    if json_mode:
        emit_result(
            {
                "verb": _REZERO_VERB,
                "role": role,
                "port": port,
                "plan": plan.as_dict(),
                "written": False,
                "reason": "already-applied",
                "seam_eviction_proven": False,
            },
            json_mode=True,
        )
        return
    emit_result(
        "\n".join(
            [
                f"## arm rezero {plan.joint} ({role}) — already re-zeroed on {port}",
                "",
                f"- motor           : {plan.motor}",
                f"- offset in force : {plan.current_offset} (this joint's computed offset)",
                f"- reported now    : {plan.reported_position} (raw {plan.raw_position})",
                "",
                "Nothing written — the servo already holds this offset.",
                "",
                "That the offset is APPLIED does not mean the seam MOVED. If you have not "
                f"proven it yet, run: arm101 arm rezero {plan.joint} --verify",
            ]
        ),
        json_mode=False,
    )


def cmd_arm_rezero(args: argparse.Namespace) -> None:
    """Shift a joint's encoder zero so the seam falls where the joint cannot reach.

    The gated EEPROM write for issue #35, plus (``--verify``) the sweep that
    proves it actually worked. Commands **no motion on any path** — see
    :mod:`arm101.hardware.rezero` for the bootstrap problem that forbids it.
    """
    role: str = args.role
    json_mode = bool(getattr(args, "json", False))
    joint: str = args.joint
    verify = bool(getattr(args, "verify", False))
    raw_duration = getattr(args, "duration", None)
    duration: float = rezero.DEFAULT_SWEEP_DURATION if raw_duration is None else float(raw_duration)

    # Eligibility FIRST — before consent, before a port, before a bus. "Why can't
    # I re-zero wrist_roll?" is a question about the arm's geometry, and it is
    # answerable (and answered, at length) with no hardware attached.
    offset, arc = rezero.require_rezeroable(joint)
    motor = arm_spec.joint_ids(role)[joint]

    # Validate the sweep length before prompting for consent, so a bad --duration
    # is a user error caught up front rather than after the operator has already
    # said yes.
    if verify:
        rezero.samples_for(duration)

    mode = resolve_consent(args, verb=_REZERO_VERB, require_plan_hash=False)

    # --- dry_run: plan only, zero writes, zero bus access ---
    if mode == "dry_run":
        _emit_rezero_plan(
            role,
            joint,
            motor,
            offset,
            arc,
            verify,
            duration,
            port=getattr(args, "port", None),
            json_mode=json_mode,
        )
        return

    # --- interactive: confirm at a prompt before any bus is opened ---
    if mode == "interactive" and not _confirm_rezero(
        joint, motor, offset, verify, json_mode=json_mode
    ):
        return

    port = _resolve_port(getattr(args, "port", None))
    bus = _open_bus(port)
    try:
        # The guard owns the one joint this verb touches. rezero never ENERGISES
        # it — both paths de-energise and leave it limp — so the guard is here to
        # catch the inverse hazard: a crash between the torque-off and the
        # EEPROM re-lock, or a Ctrl-C mid-sweep on a joint an earlier verb left
        # hot. Releasing an already-limp motor is a no-op, so this costs nothing
        # on the paths where nothing went wrong.
        with torque_guard(bus, (motor,), on_release=_release_announcer(json_mode)):
            if verify:
                _run_rezero_verify(bus, role, port, joint, motor, duration, json_mode=json_mode)
            else:
                _run_rezero_write(bus, role, port, joint, motor, json_mode=json_mode)
    finally:
        bus.close()


# ---------------------------------------------------------------------------
# arm setup <role>
# ---------------------------------------------------------------------------


def _emit_dry_run_plan(role: str, json_mode: bool) -> None:
    """Build and emit the dry-run plan for *role* (no writes, no catalog entries)."""
    plan = []
    prefix = "F" if role == "follower" else "L"
    for joint in arm_spec.JOINTS:
        spec = arm_spec.motor_spec(role, joint)
        plan.append(
            {
                "label": f"{prefix}{spec.id}",
                "joint": joint,
                "new_id": spec.id,
                "baudrate": spec.baud,
                "servo_model": spec.servo_model,
                "gear_ratio": spec.gear_ratio,
            }
        )
    if json_mode:
        emit_result({"role": role, "plan": plan}, json_mode=True)
    else:
        lines = [
            f"## Dry-run plan: arm setup {role}",
            "",
            f"Motor assignment table for {role} arm:",
            "",
            "| label | joint | new_id | baudrate | servo_model | gear_ratio |",
            "|-------|-------|--------|----------|-------------|------------|",
        ]
        for entry in plan:
            lines.append(
                f"| {entry['label']} | {entry['joint']} | {entry['new_id']}"
                f" | {entry['baudrate']} | {entry['servo_model']} | {entry['gear_ratio']} |"
            )
        lines.append("")
        lines.append("To execute, re-run with --apply.")
        emit_result("\n".join(lines), json_mode=False)


def cmd_arm_setup(args: argparse.Namespace) -> None:
    """Set up all 6 motors for *role*, cataloging each as they are written."""
    role: str = args.role
    json_mode = bool(getattr(args, "json", False))

    # Resolve consent via the same three-mode mechanism as setup-motors
    # (no new consent code path).
    mode = resolve_consent(args, verb=f"arm setup {role}", require_plan_hash=False)

    # --- dry_run: emit plan only, zero writes, zero catalog entries ---
    if mode == "dry_run":
        _emit_dry_run_plan(role, json_mode)
        return

    # --- interactive / agent: drive the walk + save catalog entries ---
    operator = resolve_operator()
    prefix = "F" if role == "follower" else "L"
    catalog_entries: list[MotorEntry] = []

    def _on_motor_assigned(
        motor_id: int,
        joint_name: str,
        entry: dict[str, object],
    ) -> None:
        """Save a catalog entry right after each motor is written."""
        spec = arm_spec.motor_spec(role, joint_name)
        label = f"{prefix}{motor_id}"
        catalog_entry = MotorEntry(
            label=label,
            servo_model=spec.servo_model,
            gear_ratio=spec.gear_ratio,
            joint=joint_name,
            detected_id=motor_id,
            detected_model=int(entry["detected_model"]),  # type: ignore[arg-type]
            port=str(entry["port"]),
        )
        save_entry(catalog_entry)
        catalog_entries.append(catalog_entry)

    assigned = _setup_motors._run_walk(
        args,
        mode=mode,
        asserted_current_id=None,
        baudrate=arm_spec.DEFAULT_BAUDRATE,
        operator=operator,
        on_motor_assigned=_on_motor_assigned,
    )

    if json_mode:
        emit_result(
            {
                "role": role,
                "assigned": [
                    {
                        "label": e.label,
                        "joint": e.joint,
                        "servo_model": e.servo_model,
                        "gear_ratio": e.gear_ratio,
                        "detected_id": e.detected_id,
                        "detected_model": e.detected_model,
                        "port": e.port,
                    }
                    for e in catalog_entries
                ],
            },
            json_mode=True,
        )
    else:
        lines = [f"Arm {role} setup complete:"]
        for e in catalog_entries:
            lines.append(
                f"  {e.label}: {e.joint}, id={e.detected_id}, {e.servo_model}, {e.gear_ratio}"
            )
        emit_result("\n".join(lines), json_mode=False)

    # Suppress linter warning: assigned is populated to mirror setup-motors
    # behavior; it drives the walk and returns one entry per written motor.
    _ = assigned


# ---------------------------------------------------------------------------
# Registration
# ---------------------------------------------------------------------------


def register(sub: "argparse._SubParsersAction[argparse.ArgumentParser]") -> None:
    """Register the ``arm`` noun group on *sub*."""
    p = sub.add_parser(
        "arm",
        help="Arm-level operations (see 'arm101 arm overview').",
    )
    p.add_argument("--json", action="store_true", help=_JSON_HELP)
    p.set_defaults(func=_no_verb, json=False)

    # ``p`` is a _CliArgumentParser; propagate it so ``arm <verb>`` parse
    # errors route through the structured error contract rather than argparse's
    # default stderr/exit 2.
    noun_sub = p.add_subparsers(dest="arm_command", parser_class=type(p))

    # overview — descriptive, always exits 0, ignores positional target
    ov = noun_sub.add_parser("overview", help="Describe the arm noun surface (roles, joints).")
    ov.add_argument(
        "target",
        nargs="?",
        help="Ignored positional target (overview always exits 0).",
    )
    ov.add_argument("--json", action="store_true", help=_JSON_HELP)
    ov.set_defaults(func=cmd_arm_overview)

    # read — read-only whole-arm snapshot (no motion gate)
    rd = noun_sub.add_parser(
        "read",
        help="Read every joint's live register state (read-only; commands no motion).",
    )
    rd.add_argument(
        "--role",
        choices=arm_spec.roles(),
        default="follower",
        help=_ROLE_HELP,
    )
    rd.add_argument(
        "--port",
        default=None,
        help=_PORT_HELP,
    )
    rd.add_argument("--json", action="store_true", help=_JSON_HELP)
    rd.set_defaults(func=cmd_arm_read)

    # flex — gated motion verb
    fx = noun_sub.add_parser(
        "flex",
        help=(
            "Command a bounded joint move (--to) or a demo sweep (--demo); "
            "gated motion (use --apply in non-TTY agent mode)."
        ),
    )
    fx.add_argument(
        "joint",
        nargs="?",
        help="Joint to move (one of the 6 SO-101 joints). Omit when using --demo.",
    )
    fx.add_argument(
        "--to",
        type=int,
        default=None,
        help="Target encoder tick for the single-joint move (required with a joint).",
    )
    fx.add_argument(
        "--demo",
        action="store_true",
        default=False,
        help="Sweep every joint through a safe sub-range (inherently gentle).",
    )
    fx.add_argument(
        "--gentle",
        action="store_true",
        default=False,
        help="Use the load-watch back-off-then-hold primitive for the move.",
    )
    fx.add_argument(
        "--threshold",
        type=int,
        default=None,
        help="Gentle contact-load threshold override (default 250).",
    )
    fx.add_argument(
        "--role",
        choices=arm_spec.roles(),
        default="follower",
        help=_ROLE_HELP,
    )
    fx.add_argument(
        "--port",
        default=None,
        help=_PORT_HELP,
    )
    fx.add_argument(
        "--apply",
        action="store_true",
        default=False,
        help="Execute the motion (non-TTY agent mode; ignored under a TTY).",
    )
    fx.add_argument("--json", action="store_true", help=_JSON_HELP)
    fx.set_defaults(func=cmd_arm_flex)

    # explore — gated motion verb (flood-fill reachability mapping)
    ex = noun_sub.add_parser(
        "explore",
        help=(
            "Flood-fill and map the arm's reachable joint-space via the "
            "overload-safe gentle move; writes a JSONL event log + compact map "
            "(resumable); gated motion (use --apply in non-TTY agent mode)."
        ),
    )
    ex.add_argument(
        "--role",
        choices=arm_spec.roles(),
        default="follower",
        help=_ROLE_HELP,
    )
    ex.add_argument(
        "--port",
        default=None,
        help=_PORT_HELP,
    )
    ex.add_argument(
        "--map",
        default=None,
        help=(
            "Reachability-map file path — resume input if it exists AND the "
            "written output (default: ./arm-explore-<role>.map.json). The JSONL "
            "event log is a sibling with a .events.jsonl suffix."
        ),
    )
    ex.add_argument(
        "--threshold",
        type=int,
        default=None,
        help=(
            "Blanket contact-load threshold applied to EVERY joint, overriding "
            "--threshold-file and the per-joint defaults (default: per-joint, "
            "hardware-tuned — see 'arm101-cli explain arm explore')."
        ),
    )
    ex.add_argument(
        "--threshold-joint",
        action="append",
        default=None,
        metavar="JOINT=LOAD",
        help=(
            "Override one joint's contact threshold, e.g. "
            "--threshold-joint shoulder_lift=350 (repeatable). Overrides "
            "--threshold-file and the per-joint default."
        ),
    )
    ex.add_argument(
        "--threshold-file",
        default=None,
        metavar="PATH",
        help=(
            "JSONL file of per-joint contact thresholds "
            '({"joint": name, "threshold": N} per line). CLI flags override '
            "file entries; file overrides built-in defaults."
        ),
    )
    ex.add_argument(
        "--max-moves",
        type=int,
        default=None,
        help=(
            "Budget cap on total moves/probes before the run stops "
            f"(default {DEFAULT_MAX_MOVES}; hardware-tuned open question)."
        ),
    )
    ex.add_argument(
        "--resolution",
        type=int,
        default=None,
        help=(
            "Per-joint grid bucket size in encoder ticks "
            f"(default {_DEFAULT_RESOLUTION}; hardware-tuned open question)."
        ),
    )
    ex.add_argument(
        "--apply",
        action="store_true",
        default=False,
        help="Execute the exploration (non-TTY agent mode; ignored under a TTY).",
    )
    ex.add_argument("--json", action="store_true", help=_JSON_HELP)
    ex.set_defaults(func=cmd_arm_explore, json=False)

    # rezero — gated EEPROM write (and NOT a move); --verify is the seam-eviction proof
    rz = noun_sub.add_parser(
        "rezero",
        help=(
            "Shift a joint's encoder zero (EEPROM addr 31) so the 4095->0 seam falls in "
            "the arc it cannot reach — the issue-#35 fix for elbow_flex; commands NO "
            "motion. --verify proves the seam actually moved via a torque-off, "
            "hand-driven sweep. Gated (use --apply in non-TTY agent mode)."
        ),
    )
    rz.add_argument(
        "joint",
        help=(
            "Joint to re-zero. Only elbow_flex wraps inside its travel; every other "
            "joint is refused with the reason (ask, and it will tell you)."
        ),
    )
    rz.add_argument(
        "--verify",
        action="store_true",
        default=False,
        help=(
            "Do not write. De-energise the joint and poll its position while YOU "
            "hand-move it through its whole travel, asserting there is no discontinuity "
            "anywhere — the only proof the seam actually moved. Leaves the joint limp."
        ),
    )
    rz.add_argument(
        "--duration",
        type=float,
        default=None,
        help=(
            "Seconds to sweep for with --verify "
            f"(default {rezero.DEFAULT_SWEEP_DURATION:.0f}); ignored without it."
        ),
    )
    rz.add_argument(
        "--role",
        choices=arm_spec.roles(),
        default="follower",
        help=_ROLE_HELP,
    )
    rz.add_argument(
        "--port",
        default=None,
        help=_PORT_HELP,
    )
    rz.add_argument(
        "--apply",
        action="store_true",
        default=False,
        help="Execute (non-TTY agent mode; ignored under a TTY).",
    )
    rz.add_argument("--json", action="store_true", help=_JSON_HELP)
    rz.set_defaults(func=cmd_arm_rezero, json=False)

    # setup — gated action verb
    sp = noun_sub.add_parser(
        "setup",
        help=(
            "Set up all 6 motors for an arm role, assigning EEPROM ids "
            "and cataloging each motor (use --apply to execute)."
        ),
    )
    sp.add_argument(
        "role",
        choices=arm_spec.roles(),
        help="Arm role: follower or leader.",
    )
    sp.add_argument(
        "--port",
        default=None,
        help="Serial port (default: auto-detect per motor, handles USB re-enumeration).",
    )
    sp.add_argument(
        "--apply",
        action="store_true",
        default=False,
        help="Execute the EEPROM writes (non-TTY agent mode; ignored under a TTY).",
    )
    sp.add_argument("--json", action="store_true", help=_JSON_HELP)
    sp.set_defaults(func=cmd_arm_setup)
