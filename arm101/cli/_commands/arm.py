"""``arm101 arm`` — arm noun group (overview, read, flex, explore, profile, limits, rezero, setup).

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

``arm profile <joint>``
    Gated motion: find the highest Goal_Speed at which the arm can still DETECT a
    contact, via :func:`~arm101.hardware.profile.profile_joint`.  Ramps the speed
    upward and, at every candidate, drives the joint into a real obstacle
    (``--contact-to``, a tick it genuinely cannot reach) and requires the shipped
    ``gentle_move`` stall rule to fire.  A speed the servo merely *survives* is a
    FAILURE, not a pass — free motion at a speed proves nothing about contact
    detection at that speed.  Records the joint's highest safe speed, its measured
    ticks/second, and its motion-onset latency; the arm's motion constants
    (``gentle``'s ``_DEFAULT_SPEED = 150``, ``_MIN_TICKS_PER_SECOND = 120``) were
    hand-fitted in one bench session and this is what replaces the guess.  Gated by
    the same three-mode consent as ``arm flex``.

``arm limits [<joint>...]``
    Gated motion: **measure** each joint's true travel and change nothing.  Per joint
    it opens a :class:`~arm101.hardware.rolling_frame.RollingFrame` (which keeps the
    encoder seam half a turn ahead of the creep), drives
    :func:`~arm101.hardware.probe.probe_end` to BOTH ends under contact detection, and
    classifies the result (:func:`~arm101.hardware.classify.classify_observations`:
    BOUNDED / CONTINUOUS / UNDETERMINED).  Each END carries its own verdict — WALL,
    TORQUE_LIMITED, EDGE or TIMEOUT — because a joint routinely has a solid wall one
    way and a torque-limited stall the other, and only WALL vouches for a limit.

    **MEASURE-ONLY.**  The frame restores the borrowed encoder offset on every exit
    path, so the servo is left exactly as it was found.  There is deliberately no
    ``--commit``: keeping a re-zero is a separate, explicitly gated act, and a verb
    that silently re-calibrated five joints because somebody asked it to *look* at
    them is not one anybody should run.

    It also reports, per joint, the delta between the measured span and the
    EEPROM-derived span ``arm explore`` builds its grid from today — the number that
    settles whether that grid is being fed artifacts (issue #34).  The report is
    written to be able to say it is NOT, which is the only way the finding means
    anything.

``arm rezero <joint>``
    Gated **EEPROM write, and no motion at all**: shift the servo's encoder zero
    (``Ofs``/``Homing_Offset``, addr 31) so the 4095->0 seam falls in the arc the
    joint physically cannot reach — the fix for issue #35, and ``elbow_flex`` is the
    only joint whose arc has been MEASURED.  ``wrist_roll`` is REFUSED as *impossible*
    (a re-zero relocates a seam, it cannot evict one from a joint that turns all the
    way round; it has a soft limit instead).  The other four are refused because their
    arc is *unknown* — **not** because a re-zero is unnecessary: issue #43 withdrew
    that claim (see :data:`arm101.hardware.arm_spec.REZERO_UNKNOWN_HEADLINE`, which
    every operator-facing surface renders rather than restates).  ``arm limits`` is
    what turns an unknown arc into a measured one.  Gated by the same three-mode
    consent as ``arm flex`` (dry_run / interactive / agent ``--apply``); the dry-run
    touches no bus at all and prints the exact register writes.

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
``flex``, ``flex --demo``, ``explore``, ``profile`` and ``limits`` each wrap their
whole run in a :func:`~arm101.hardware.safety.torque_guard` owning the motors they
may energise. (``setup`` does too, one motor at a time, in
:func:`arm101.cli._commands.setup_motors._process_one_motor` — that is where its
per-motor bus is opened.) ``read`` does not: it energises nothing.

``limits`` claims each motor progressively — the instant that joint's frame is about
to open — and **never disowns one**. A joint whose frame has closed can still be
holding (``gentle_move``'s stop-and-hold is its contract), so a bus that dies while
joint 5 is being probed must still release joints 1-4. Nothing in the measuring path
would ever go back to them; only the guard does.

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
from arm101.hardware.classify import (
    SeamRemedy,
    TravelClassification,
    TravelKind,
    classify_observations,
)
from arm101.hardware.demo import demo_sweep
from arm101.hardware.gentle import CONTACT_LOAD_CEILING as _CONTACT_LOAD_CEILING
from arm101.hardware.gentle import gentle_move
from arm101.hardware.journal import CalibrationJournal, commit, require_clean, restore_dirty

# MATERIAL_SPAN_DELTA_TICKS is imported, not re-declared: how far two spans of one
# joint must sit apart before the difference is REAL is a fact about how repeatable this
# hardware is, and it lives with the record of measured travel. `explain arm limits`
# renders the same constant, so the verb and its documentation cannot disagree about
# what "materially different" means.
from arm101.hardware.limits import MATERIAL_SPAN_DELTA_TICKS  # noqa: F401 (re-exported)
from arm101.hardware.limits import ENCODER_TICKS, TravelEnd
from arm101.hardware.motion import compliant_move
from arm101.hardware.motor_catalog import MotorEntry, save_entry
from arm101.hardware.probe import DEFAULT_CREEP_TICKS, ProbeOutcome, probe_end, wall_compliance
from arm101.hardware.profile import (
    DEFAULT_SPEED_MAX,
    DEFAULT_SPEED_START,
    DEFAULT_SPEED_STEP,
    JointSpeedProfile,
    SpeedTrial,
    profile_joint,
    speed_ladder,
)
from arm101.hardware.rolling_frame import RollingFrame
from arm101.hardware.safety import ReleaseReport, torque_guard
from arm101.hardware.soft_limit_store import (
    MeasuredSoftLimit,
    default_soft_limit_path,
    load_soft_limits,
    record_soft_limit,
)

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

#: Consent verb label for ``arm flex`` (hoisted to avoid duplicating the literal).
_FLEX_VERB = "arm flex"

#: Consent verb label for ``arm explore`` (hoisted to avoid duplicating the literal).
_EXPLORE_VERB = "arm explore"

#: Consent verb label for ``arm profile`` (hoisted to avoid duplicating the literal).
_PROFILE_VERB = "arm profile"

#: Consent verb label for ``arm rezero`` (hoisted to avoid duplicating the literal).
_REZERO_VERB = "arm rezero"

#: Consent verb label for ``arm limits`` (hoisted to avoid duplicating the literal).
_LIMITS_VERB = "arm limits"

#: Shared dry-run footer for the gated motion verbs (hoisted; identical text).
_DRY_RUN_FOOTER = "No motion commanded (dry-run). Re-run non-interactively with --apply to execute."

#: Placeholder shown for the port in a dry-run plan, where no bus is opened and
#: so no port has been resolved yet (hoisted; every gated motion verb shows it).
_PORT_UNRESOLVED = "(auto-detect at apply)"

#: The interactive confirmation prompt shared by every gated motion verb. The
#: exact wording is load-bearing — ``_confirm_*`` compares the answer against
#: "yes", so the prompt must ask for precisely that word.
_CONFIRM_MOTION_PROMPT = "Type 'yes' to confirm motion"

#: What every gated motion verb says when the operator declines at the prompt.
_ABORTED_NO_MOTION = "Aborted; no motion commanded."

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
        "verbs": [
            "overview",
            "read",
            "flex",
            "explore",
            "profile",
            "limits",
            "rezero",
            "setup",
        ],
        "roles": arm_spec.roles(),
        "motor_map": roles_data,
    }

    if json_mode:
        emit_result(payload, json_mode=True)
        return

    lines = [
        "## arm — arm-level operations",
        "",
        "Verbs: overview, read, flex, explore, profile, limits, rezero, setup",
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
    lines.append(_DRY_RUN_FOOTER)
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
    ans = _prompt(_CONFIRM_MOTION_PROMPT)
    if ans.strip().lower() == "yes":
        return True
    if json_mode:
        emit_result({"aborted": True, "role": role}, json_mode=True)
    else:
        emit_result(_ABORTED_NO_MOTION, json_mode=False)
    return False


def _soft_limits(args: argparse.Namespace) -> "dict[str, arm_spec.SoftLimit]":
    """The soft-limit table in force for this run: shipped defaults + MEASURED overrides.

    **The seam where a measured soft limit stops being a file and starts being a fence.**

    ``arm limits --commit`` cannot write a soft limit into ``arm_spec.SOFT_LIMITS`` — a
    CLI does not rewrite its own source — so it appends it to
    :mod:`arm101.hardware.soft_limit_store` (default ``~/.arm101/soft-limits.jsonl``,
    relocatable via ``$ARM101_SOFT_LIMITS`` or ``--soft-limit-file``). This function
    reads it back and merges it over the shipped table
    (:func:`~arm101.hardware.arm_spec.resolve_soft_limits`), and the result goes to
    :func:`_resolve_joint_bounds` — the single place a servo's EEPROM limits become move
    bounds. So the fence binds on ``arm flex``, on ``arm explore``'s grid, and on the
    demo sweep, without any of them knowing the store exists.

    **Loaded unconditionally, not behind a flag.** A contact threshold you forget to pass
    leaves you with a sane default; a soft limit you forget to pass leaves you driving a
    joint across the encoder seam it exists to fence off. This repo has already shipped
    an inert soft limit once — the ``wrist_roll`` entry meant nothing until every mover
    was routed through ``resolve_bounds`` — and a store nobody loads by default would be
    the same bug wearing a flag.

    An absent store is the common case (every fresh checkout) and costs one ``exists()``.

    Raises
    ------
    CliError
        If the store is unreadable, names an unknown joint, holds a range that is not a
        valid RAW soft limit, or contradicts the shipped tables (a joint cannot have both
        a soft limit and a re-zero arc: those are the two mutually exclusive answers to a
        wrapping joint). Raised BEFORE the arm is touched — a fence nobody can read is
        not a fence, and a run that shrugged and carried on would be moving the joint
        under a constraint the operator believes is in force.
    """
    path = getattr(args, "soft_limit_file", None)
    try:
        return arm_spec.resolve_soft_limits(from_file=load_soft_limits(path))
    except ValueError as exc:
        raise CliError(
            code=EXIT_USER_ERROR,
            message=str(exc),
            remediation=(
                f"The measured soft-limit store is at {default_soft_limit_path()} (override "
                "with --soft-limit-file or $ARM101_SOFT_LIMITS). It is append-only and the "
                "last record for a joint wins, so a bad entry is corrected by appending a "
                "good one — or by deleting the line."
            ),
        ) from exc


def _resolve_joint_bounds(
    joint: str,
    info: "dict[str, int]",
    soft_limits: "dict[str, arm_spec.SoftLimit] | None" = None,
) -> "tuple[int, int]":
    """Turn one joint's ``read_info`` snapshot into the bounds a move may use.

    The single place in this module where a servo's EEPROM angle limits become
    move bounds — deliberately, so the soft limit cannot be forgotten at one
    call site and honoured at another. It intersects the EEPROM range with the
    joint's soft limit (see :func:`~arm101.hardware.arm_spec.resolve_bounds`):
    on this arm the EEPROM is the untouched factory ``0-4095`` on every joint,
    so for ``wrist_roll`` — whose travel wraps the encoder seam — the EEPROM
    alone would happily permit a move into the dead arc and across the seam.

    *soft_limits* is the resolved table from :func:`_soft_limits`: the shipped
    :data:`~arm101.hardware.arm_spec.SOFT_LIMITS` merged with whatever
    ``arm limits --commit`` has MEASURED. ``None`` falls back to the shipped table alone
    — correct for a caller with no run behind it, and never a silent widening, because a
    measured limit can only ever narrow.

    **This is a frame crossing, which is why the offset goes with it.** The soft
    limit is stored in RAW ticks (physical angles, immune to a re-zero); the EEPROM
    limits and the bounds this returns are REPORTED ticks (what a goal write and a
    position read speak). ``read_info`` hands us both the limits and the servo's
    live ``homing_offset``, so the conversion happens here, once, with the real
    number — never with an assumed 0, which no servo has ever held.

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
        return arm_spec.resolve_bounds(
            joint,
            int(info["min_angle"]),
            int(info["max_angle"]),
            int(info["homing_offset"]),
            limits=soft_limits,
        )
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
    soft_limits: "dict[str, arm_spec.SoftLimit] | None" = None,
) -> dict[str, object]:
    """Run a single-joint move (gentle or compliant) and return its result dict."""
    motor_id = arm_spec.joint_ids(role)[joint]
    info = bus.read_info(motor_id)  # type: ignore[attr-defined]
    min_angle, max_angle = _resolve_joint_bounds(joint, info, soft_limits)
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

    # The fence, resolved BEFORE the port is opened: a soft limit this run cannot read is
    # a soft limit this run must not move without. Shipped table + whatever
    # `arm limits --commit` measured (see _soft_limits).
    soft_limits = _soft_limits(args)

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
                    soft_limits=soft_limits,
                )
                _emit_flex_demo(role, port, report, json_mode=json_mode)
            else:
                # joint/target are not-None here (guaranteed by _validate_flex).
                move = _execute_single(bus, role, joint, target, gentle, threshold, soft_limits)
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


def _build_grid_spec(
    bus: object,
    role: str,
    resolution: int,
    soft_limits: "dict[str, arm_spec.SoftLimit] | None" = None,
) -> GridSpec:
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
        bound_min, bound_max = _resolve_joint_bounds(joint, info, soft_limits)
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


def _resolve_contact_thresholds(args: argparse.Namespace) -> "dict[str, int]":
    """Resolve the per-joint contact-threshold map for a contact-detecting run.

    Shared by ``arm explore`` and ``arm limits`` — one threshold resolution, not two.
    The two verbs drive the same ``gentle_move`` contact detection against the same
    joints, so a threshold that is right for one is right for the other, and letting
    them resolve it separately would be inviting exactly the drift
    :func:`arm_spec.resolve_contact_thresholds` exists to prevent.

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
    lines.append(_DRY_RUN_FOOTER)
    emit_result("\n".join(lines), json_mode=False)


def _confirm_explore(role: str, *, json_mode: bool) -> bool:
    """Prompt the human before an explore run; return True to proceed."""
    emit_diagnostic(
        f"⚠ This COMMANDS MOTION on the {role} arm: a flood-fill exploration of "
        "reachable joint-space (many gentle moves)."
    )
    ans = _prompt(_CONFIRM_MOTION_PROMPT)
    if ans.strip().lower() == "yes":
        return True
    if json_mode:
        emit_result({"aborted": True, "role": role}, json_mode=True)
    else:
        emit_result(_ABORTED_NO_MOTION, json_mode=False)
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
    thresholds_by_joint = _resolve_contact_thresholds(args)
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
            spec = _build_grid_spec(bus, role, resolution, _soft_limits(args))
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
# arm profile (gated motion — find the highest speed that still DETECTS contact)
# ---------------------------------------------------------------------------


def _validate_profile(joint: "str | None", contact_to: "int | None") -> None:
    """Validate the profile argument combination; raise CliError(EXIT_USER_ERROR).

    Both are structurally required by the parser, so this catches a caller that
    built the namespace by hand (tests, an embedding agent) as well as an unknown
    joint name — before any bus is opened.
    """
    if joint not in arm_spec.JOINTS:
        raise CliError(
            code=EXIT_USER_ERROR,
            message=f"unknown joint {joint!r}",
            remediation=f"Valid joints: {', '.join(arm_spec.JOINTS)}.",
        )
    if contact_to is None:
        raise CliError(
            code=EXIT_USER_ERROR,
            message="--contact-to is required: profiling needs a real contact to detect",
            remediation=(
                "Pass a tick the joint genuinely CANNOT reach (its end-stop, or a fixture "
                "in its path), e.g. 'arm profile shoulder_pan --contact-to 3500'."
            ),
        )


def _resolve_ladder(args: argparse.Namespace) -> "tuple[int, ...]":
    """Build the candidate speed ladder from the ``--speed-*`` flags.

    Explicit ``None`` checks, NOT ``or``: this repo's idiom, and here it is load
    bearing twice over — a flag that is merely absent must fall through to the
    module default, while an explicitly-passed value must be honoured even when it
    is one :func:`~arm101.hardware.profile.speed_ladder` will go on to reject
    (which is the right place for that rejection to happen, with the right message).
    """
    raw_start = getattr(args, "speed_start", None)
    raw_step = getattr(args, "speed_step", None)
    raw_max = getattr(args, "speed_max", None)
    return speed_ladder(
        DEFAULT_SPEED_START if raw_start is None else int(raw_start),
        DEFAULT_SPEED_STEP if raw_step is None else int(raw_step),
        DEFAULT_SPEED_MAX if raw_max is None else int(raw_max),
    )


def _profile_threshold(args: argparse.Namespace, joint: str) -> int:
    """The joint's contact-load threshold: ``--threshold`` if given, else its default.

    Falls back to the SAME per-joint numbers ``arm explore`` uses
    (:data:`arm_spec.DEFAULT_CONTACT_THRESHOLDS`), because the speed this verb
    certifies is only meaningful for the threshold it was certified against: a
    detector that fires at speed S with a threshold of 250 says nothing about the
    same joint at 400.
    """
    raw = getattr(args, "threshold", None)
    if raw is None:
        return arm_spec.DEFAULT_CONTACT_THRESHOLDS[joint]

    threshold = int(raw)
    # A threshold at or above the torque cap can NEVER fire, so the run would be
    # a guaranteed lie. present_load SATURATES at the servo's Torque_Limit
    # (proven on hardware: cap 300 pins load at exactly 300, cap 600 pins it at
    # 600), and gentle_move caps Torque_Limit to _CONTACT_TORQUE_LIMIT (500) for
    # the duration of every move. Contact requires load > threshold. So at
    # threshold >= 500 the inequality is unsatisfiable no matter how hard the arm
    # pushes — every probe would come back "no contact", and this verb would
    # report the FIRST rung as a void run ("nothing there to detect") while the
    # joint was pressed hard against a very real obstacle. Refuse before the bus
    # is even opened; a silent impossibility is far worse than a loud refusal.
    if not 0 < threshold < _CONTACT_LOAD_CEILING:
        raise CliError(
            code=EXIT_USER_ERROR,
            message=(
                f"--threshold {threshold} is outside the detectable band "
                f"(0, {_CONTACT_LOAD_CEILING})."
            ),
            remediation=(
                "present_load saturates at the servo's Torque_Limit, which gentle_move caps "
                f"to {_CONTACT_LOAD_CEILING} during a move, and contact requires load > "
                f"threshold — so a threshold >= {_CONTACT_LOAD_CEILING} can never fire. Pass a "
                f"value above {joint}'s free-motion peak load and below {_CONTACT_LOAD_CEILING} "
                f"(its default is {arm_spec.DEFAULT_CONTACT_THRESHOLDS[joint]})."
            ),
        )
    return threshold


def _emit_profile_plan(
    role: str,
    joint: str,
    contact_to: int,
    threshold: int,
    ladder: "tuple[int, ...]",
    *,
    port: "str | None",
    json_mode: bool,
) -> None:
    """Emit the dry-run plan for a profile run — zero motion, zero bus access."""
    plan: "dict[str, object]" = {
        "verb": _PROFILE_VERB,
        "role": role,
        "joint": joint,
        "motor": arm_spec.joint_ids(role)[joint],
        "port": port or _PORT_UNRESOLVED,
        "contact_to": contact_to,
        "threshold": threshold,
        "ladder": list(ladder),
        "note": (
            "COMMANDS MOTION: drives the joint INTO the contact at --contact-to, once per "
            "candidate speed, low to high, and accepts a speed ONLY if the gentle-move stall "
            "rule still detects that contact. Stops at the first speed it does not."
        ),
    }

    if json_mode:
        emit_result({"plan": plan}, json_mode=True)
        return

    lines = ["## Dry-run plan: arm profile", ""]
    for key, value in plan.items():
        lines.append(f"- {key}: {value}")
    lines.append("")
    lines.append(_DRY_RUN_FOOTER)
    emit_result("\n".join(lines), json_mode=False)


def _confirm_profile(role: str, joint: str, contact_to: int, *, json_mode: bool) -> bool:
    """Prompt the human before a profile run; return True to proceed.

    Spells out the part an operator must actually agree to: this verb deliberately
    drives the joint into something, repeatedly, at rising speeds, until it finds a
    speed at which the arm can no longer tell it has hit anything.
    """
    emit_diagnostic(
        f"⚠ This COMMANDS MOTION on the {role} arm: it drives {joint} INTO the contact at "
        f"tick {contact_to}, once per candidate speed, at RISING speeds — deliberately "
        "ramping until contact detection fails or the servo overloads."
    )
    emit_diagnostic(
        "Confirm the joint is physically blocked at (or before) that tick, and that the "
        "path to it is clear."
    )
    ans = _prompt(_CONFIRM_MOTION_PROMPT)
    if ans.strip().lower() == "yes":
        return True
    if json_mode:
        emit_result({"aborted": True, "role": role, "joint": joint}, json_mode=True)
    else:
        emit_result(_ABORTED_NO_MOTION, json_mode=False)
    return False


def _profile_progress(json_mode: bool) -> "Callable[[SpeedTrial], None]":
    """Build the per-trial hook that narrates a long ramp on stderr as it runs.

    A full ladder is many contacts and can take minutes; going silent and then
    printing everything at the end would leave an operator watching an arm bang
    into a wall with no idea which rung it is on. Goes to **stderr**
    (:func:`~arm101.cli._output.emit_diagnostic`) like every other diagnostic, so
    stdout stays reserved for the one result — and under ``--json`` each line is a
    JSON object rather than prose, so an agent tailing stderr gets structured
    records, not sentences wedged between JSON documents.
    """

    def announce(trial: SpeedTrial) -> None:
        if json_mode:
            emit_diagnostic(json.dumps({"trial": trial.as_dict()}, ensure_ascii=False))
            return
        verdict = "accepted" if trial.accepted else f"REJECTED ({trial.reason})"
        rate = "-" if trial.ticks_per_second is None else f"{trial.ticks_per_second:.0f} ticks/s"
        emit_diagnostic(f"speed {trial.speed}: {verdict} — {rate}, peak load {trial.peak_load}")

    return announce


def _fmt_seconds(value: "float | None") -> str:
    """Render a measured duration for the text report; ``None`` becomes ``-``."""
    return "-" if value is None else f"{value * 1000:.0f} ms"


def _fmt_rate(value: "float | None") -> str:
    """Render a measured travel rate for the text report; ``None`` becomes ``-``."""
    return "-" if value is None else f"{value:.0f}"


def _emit_profile_result(
    role: str,
    port: str,
    prof: JointSpeedProfile,
    *,
    json_mode: bool,
) -> None:
    """Render a :class:`~arm101.hardware.profile.JointSpeedProfile` (text or JSON).

    Both forms carry the SAME conclusions — the safe speed, the measurements taken
    at it, the ceiling and why it is the ceiling, and every trial's verdict — so an
    agent reading ``--json`` and a human reading the table learn exactly the same
    thing, including the uncomfortable parts.
    """
    if json_mode:
        emit_result(
            {"verb": _PROFILE_VERB, "role": role, "port": port, **prof.as_dict()},
            json_mode=True,
        )
        return

    lines = [
        f"## arm profile {prof.joint} ({role}) — {port}",
        "",
        "| speed | verdict | reason | ticks/s | onset | peak load | contact |",
        "|-------|---------|--------|---------|-------|-----------|---------|",
    ]
    for trial in prof.trials:
        verdict = "accept" if trial.accepted else "REJECT"
        contact = "-" if trial.contact_position is None else str(trial.contact_position)
        lines.append(
            f"| {trial.speed} | {verdict} | {trial.reason}"
            f" | {_fmt_rate(trial.ticks_per_second)} | {_fmt_seconds(trial.motion_onset_seconds)}"
            f" | {trial.peak_load} | {contact} |"
        )
    lines.append("")

    if prof.certified:
        lines.append(f"Highest speed at which CONTACT IS STILL DETECTED: {prof.safe_speed}")
        lines.append(f"  measured travel rate  : {_fmt_rate(prof.ticks_per_second)} ticks/second")
        lines.append(f"  motion-onset latency  : {_fmt_seconds(prof.motion_onset_seconds)}")
        lines.append(f"  contact threshold     : {prof.threshold}")
    else:
        lines.append(
            "NO SAFE SPEED FOUND: contact detection failed at the very first candidate "
            f"({prof.ladder[0]}). This joint has no certified speed — and none is guessed."
        )
    lines.append("")
    if prof.ceiling_speed is None:
        lines.append(
            f"No ceiling found: every candidate up to {prof.ladder[-1]} still detected the "
            "contact, so the true ceiling is ABOVE this ladder. Re-run with a higher "
            "--speed-max to find it."
        )
    else:
        lines.append(f"Ceiling: {prof.ceiling_speed} ({prof.ceiling_reason}).")

    emit_result("\n".join(lines), json_mode=False)


def cmd_arm_profile(args: argparse.Namespace) -> None:
    """Find the highest speed at which *joint* can still DETECT a contact — gated motion.

    Drives :func:`arm101.hardware.profile.profile_joint`, whose sole motion path is
    the overload-safe ``gentle_move``. Gated by the same three-mode consent as
    ``arm flex``/``arm explore`` (dry_run / interactive / agent ``--apply``), and
    wrapped in a :func:`~arm101.hardware.safety.torque_guard` owning the one motor
    it energises — an abnormal exit here would otherwise walk away from a joint
    pressed into the very obstacle it was just driven at.
    """
    role: str = args.role
    json_mode = bool(getattr(args, "json", False))
    joint: "str | None" = getattr(args, "joint", None)
    contact_to: "int | None" = getattr(args, "contact_to", None)

    _validate_profile(joint, contact_to)
    threshold = _profile_threshold(args, joint)  # type: ignore[arg-type]
    ladder = _resolve_ladder(args)

    mode = resolve_consent(args, verb=_PROFILE_VERB, require_plan_hash=False)

    # --- dry_run: plan only, zero motion, zero bus access ---
    if mode == "dry_run":
        _emit_profile_plan(
            role,
            joint,  # type: ignore[arg-type]
            contact_to,  # type: ignore[arg-type]
            threshold,
            ladder,
            port=getattr(args, "port", None),
            json_mode=json_mode,
        )
        return

    # --- interactive: confirm at a prompt before any bus is opened ---
    if mode == "interactive" and not _confirm_profile(
        role, joint, contact_to, json_mode=json_mode  # type: ignore[arg-type]
    ):
        return

    # --- agent OR interactive-confirmed: open the bus and ramp ---
    port = _resolve_port(getattr(args, "port", None))
    motor_id = arm_spec.joint_ids(role)[joint]  # type: ignore[index]

    bus = _open_bus(port)
    try:
        # Exactly ONE motor is ever energised by this verb, and it is claimed before
        # the first bus write. Nested INSIDE the bus try/finally so the guard's
        # release runs while the bus is still open — a release after bus.close()
        # would write to a closed port and de-energise nothing.
        with torque_guard(bus, (motor_id,), on_release=_release_announcer(json_mode)):
            info = bus.read_info(motor_id)
            prof = profile_joint(
                bus,
                motor_id,
                joint=joint,  # type: ignore[arg-type]
                contact_target=int(contact_to),  # type: ignore[arg-type]
                min_angle=int(info["min_angle"]),
                max_angle=int(info["max_angle"]),
                threshold=threshold,
                ladder=ladder,
                allow_motion=True,
                progress=_profile_progress(json_mode),
            )
    finally:
        bus.close()

    _emit_profile_result(role, port, prof, json_mode=json_mode)


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
                "unreachable_arc_frame": "raw encoder ticks (NOT the ticks a servo reports)",
                "seam_moves_to_raw_tick": arc.midpoint,
                "expected_travel_ticks": arc.travel_ticks,
                "note": (
                    "COMMANDS NO MOTION. This is a persistent EEPROM write: it shifts the "
                    "joint's encoder zero so the 4095->0 seam falls inside the arc the "
                    "joint cannot reach. The joint is NOT moved — not before, not during, "
                    "not after. The offset the servo already holds is READ at apply time "
                    "and converted (raw = reported + offset, mod 4096); a factory servo "
                    f"holds {arm_spec.FACTORY_ENCODER_OFFSET}, not 0. If the offset it "
                    "already holds ALREADY puts the seam inside the unreachable arc, the "
                    "seam is evicted, this verb writes NOTHING, and it says so."
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
        f"still be {plan.target_offset}. If it reverted to "
        f"{arm_spec.FACTORY_ENCODER_OFFSET} (the factory default) or to whatever it held "
        f"before ({plan.current_offset}), the write did not persist.",
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
        f"- offset before    : {plan.current_offset}"
        f"  (seam was at raw tick {plan.current_seam_tick} — inside this joint's travel)",
        f"- offset written   : {plan.target_offset:+d} (wire value "
        f"{encode_offset(plan.target_offset)}, EEPROM addr 31)",
        f"- offset read back : {read_back}  <- the write LANDED",
        f"- seam now at      : raw tick {arm_spec.seam_tick(plan.target_offset)}"
        " — inside the arc the joint cannot reach",
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
        _, arc = rezero.require_rezeroable(joint)
        _emit_rezero_noop(role, port, plan, arc, json_mode=json_mode)
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
    arc: "arm_spec.UnreachableArc",
    *,
    json_mode: bool,
) -> None:
    """Report that the seam is ALREADY out of the joint's travel — and write nothing.

    Idempotence matters here more than it usually does: the procedure this verb
    belongs to tells the operator to power-cycle the arm and come back, so a
    second run against an already-re-zeroed joint is the *expected* path, not a
    mistake. Re-writing would be harmless on the wire and corrosive in the log —
    it would make "the offset was written" ambiguous about which run wrote the
    calibration actually in force.

    The condition is **the seam is evicted**, not "the register holds the number
    we would have written". Those come apart on the arm this was built for: the
    follower carries ``1073`` from the first, frame-confused re-zero, its seam
    sits at raw 1073 — deep inside the unreachable ``(207, 2107)`` — and a hand
    sweep proved its travel continuous across all 2196 ticks. It is FIXED. A verb
    that insisted on the arc's midpoint would burn an EEPROM write to slide a
    seam from one tick the joint can never reach to another tick the joint can
    never reach, and the operator would have nothing to show for it but a
    finite-write part with one fewer write left.
    """
    if json_mode:
        emit_result(
            {
                "verb": _REZERO_VERB,
                "role": role,
                "port": port,
                "plan": plan.as_dict(),
                "unreachable_arc": [arc.low, arc.high],
                "fresh_rezero_would_write": arc.offset,
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
                f"- offset in force : {plan.current_offset}",
                f"- seam sits at    : raw tick {plan.current_seam_tick} — strictly inside "
                f"({arc.low}, {arc.high}), the arc this joint physically cannot reach",
                f"- reported now    : {plan.reported_position} (raw {plan.raw_position})",
                "",
                "Nothing written — the seam is ALREADY out of this joint's travel, which is "
                "the entire goal. A fresh re-zero would write "
                f"{arc.offset} (the arc's midpoint, for maximum margin), but any offset "
                "inside the arc does the job equally well: the joint cannot tell the "
                "difference, and an EEPROM has a finite number of writes.",
                "",
                "That the seam is evicted IN THE TABLE'S ARITHMETIC does not mean it MOVED "
                "on this servo. If you have not proven it yet, run: "
                f"arm101 arm rezero {plan.joint} --verify",
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
# arm limits (gated motion — MEASURE the travel; change nothing)
# ---------------------------------------------------------------------------


def _limits_joints(raw: "list[str] | None") -> "tuple[str, ...]":
    """Normalise the joint selection: every joint by default, in :data:`arm_spec.JOINTS` order.

    A joint named twice is measured once — the second probe would start from the first
    one's wall, and "measure it again from where it stopped" is not a second sample of
    anything.

    Raises
    ------
    CliError(EXIT_USER_ERROR)
        On an unknown joint name. Checked before any bus is opened.
    """
    if not raw:
        return tuple(arm_spec.JOINTS)
    unknown = [joint for joint in raw if joint not in arm_spec.JOINTS]
    if unknown:
        raise CliError(
            code=EXIT_USER_ERROR,
            message=f"unknown joint(s): {', '.join(unknown)}",
            remediation=f"Valid joints: {', '.join(arm_spec.JOINTS)}.",
        )
    chosen = set(raw)
    return tuple(joint for joint in arm_spec.JOINTS if joint in chosen)


def _measure_joint(
    bus: object,
    journal: CalibrationJournal,
    *,
    joint: str,
    motor: int,
    threshold: int,
    step: int,
    max_travel: int,
    compliance: int,
    pose: "str | None",
) -> "tuple[dict[str, ProbeOutcome], TravelClassification]":
    """Probe BOTH ends of ONE joint inside ONE rolling frame, then classify what was found.

    **MEASURE-ONLY, and the frame is what makes that true.** The frame borrows the servo's
    encoder offset to keep the seam half a turn ahead of the creep, and puts the original
    back on the way out — on the clean path and on the exception path alike. Nothing here
    calls :meth:`~arm101.hardware.rolling_frame.RollingFrame.commit`, and that is the whole
    of the measure-only contract: committing a re-zero is a separate, explicitly gated act.

    **One frame, two ends.** That is one calibration transaction rather than two (half the
    EEPROM writes), and — the reason it is not merely tidy — it keeps both ends' raw
    displacements inside one lap of each other, which is the precondition
    :func:`~arm101.hardware.limits.merge_joint_travel` needs to compare them at all.

    **LOW first, then HIGH from wherever LOW stopped.** So the HIGH probe's displacement
    spans the joint's whole travel, wall to wall, in one unbroken accumulation. It also
    means a joint that turns out to be CONTINUOUS is discovered by the FIRST probe: it
    sweeps a full turn, the probe stops at the cap, and there is nothing left to learn —
    so the second end is not driven at all. That is not an optimisation dressed up as a
    rule; :func:`~arm101.hardware.classify.classify_observations` says it outright ("for a
    continuous joint it will never HAVE a second end to offer"), and driving one anyway
    would cost the operator a full extra turn of a joint whose answer is already in.
    """
    outcomes: "dict[str, ProbeOutcome]" = {}
    with RollingFrame(bus, journal, joint=joint, motor=motor) as frame:  # type: ignore[arg-type]
        for end in (TravelEnd.LOW, TravelEnd.HIGH):
            outcome = probe_end(
                bus,  # type: ignore[arg-type]
                frame,
                end=end,
                threshold=threshold,
                step=step,
                max_travel=max_travel,
                compliance=compliance,
                allow_motion=True,
                pose=pose,
            )
            outcomes[end.value] = outcome
            if abs(outcome.observation.displacement) >= ENCODER_TICKS:
                break  # a full turn settles it. See the docstring.
    # The frame restored the original offset on its way out. The servo is exactly as
    # this function found it.
    classification = classify_observations([o.observation for o in outcomes.values()])
    return outcomes, classification


def _joint_bounds_diff(
    joint: str,
    classification: TravelClassification,
    eeprom_bounds: "tuple[int, int]",
) -> "dict[str, object]":
    """One joint's answer to: **does the arm agree with the bounds ``arm explore`` uses?**

    ``arm explore`` builds its grid from ``GridSpec.bounds``, which come from the servo's
    EEPROM ``min_angle``/``max_angle`` intersected with the joint's soft limit
    (:func:`_resolve_joint_bounds` — the same function, called here, so the two cannot
    disagree about what "the bounds explore uses" means). On this arm those registers hold
    the untouched factory ``0-4095``: the EEPROM knows nothing about the joint's real
    travel.

    ``span_delta_ticks`` is ``measured - eeprom``, signed, and the sign is the finding:

    * **negative** — the EEPROM claims travel the arm does not have, so the grid enqueues
      cells the joint can never reach. That is issue #34's artifact, measured.
    * **positive** — the arm reaches further than its own configured limits permit, so
      moves are being clamped short of the joint's real travel.

    ``vouched`` is ``False`` unless a WALL was found at **both** ends. An unvouched span is
    a LOWER BOUND — the true span can only be wider — so a delta computed from one can only
    move UP. It is flagged rather than dropped: a lower bound is still evidence, and
    dropping it would quietly narrow the population the verdict is taken over.
    """
    eeprom_min, eeprom_max = eeprom_bounds
    eeprom_span = eeprom_max - eeprom_min
    measured_span = classification.swept_ticks
    delta = measured_span - eeprom_span
    return {
        "joint": joint,
        "eeprom_reported_bounds": [eeprom_min, eeprom_max],
        "eeprom_span_ticks": eeprom_span,
        "measured_span_ticks": measured_span,
        "span_delta_ticks": delta,
        "material": abs(delta) > MATERIAL_SPAN_DELTA_TICKS,
        "vouched": classification.kind is TravelKind.BOUNDED,
    }


def _bounds_diff(diffs: "list[dict[str, object]]") -> "dict[str, object]":
    """Fold the per-joint diffs into the ONE verdict this report exists to be able to lose.

    The premise behind measuring at all is that ``arm explore``'s grid is fed artifacts by
    the EEPROM bounds — that a joint whose registers claim 4095 ticks of travel and whose
    arm has 1800 makes the grid enqueue thousands of cells nobody can reach. Issue #34 is
    blocked on this measurement *because of* that premise.

    **A premise is not a finding.** If no joint's measured span differs materially from the
    EEPROM-derived one, then the bounds were fine all along, the grid was NOT being fed
    artifacts, and the rationale for the block is FALSE — and this report has to say so, in
    the same breath and the same font it would use to say the opposite. A report that could
    only ever confirm the reason it was commissioned is not a measurement; it is a press
    release. So the "no material difference" branch is written out in full, names the issue,
    and states the consequence for it, rather than leaving a reader to infer the absence of
    a finding from an empty list.
    """
    material = [d for d in diffs if d["material"]]
    material_joints = [str(d["joint"]) for d in material]
    unvouched = [str(d["joint"]) for d in diffs if not d["vouched"]]

    lower_bound_note = ""
    if unvouched:
        lower_bound_note = (
            f" Note: {', '.join(unvouched)} did NOT find a wall at both ends, so their spans "
            "are LOWER BOUNDS — the true travel can only be wider, and their deltas can only "
            "move up."
        )

    if not diffs:
        verdict = (
            "No joint was measured, so this run says nothing about the bounds arm explore "
            "builds its grid from — in either direction. It is not evidence that they are "
            "fine, and it is not evidence that they are not."
        )
    elif material:
        widest = max(
            material,
            key=lambda d: abs(int(d["span_delta_ticks"])),  # type: ignore[arg-type]
        )
        verb = "differs" if len(material) == 1 else "differ"
        verdict = (
            f"{', '.join(material_joints)} {verb} from the EEPROM-derived span by more than "
            f"{MATERIAL_SPAN_DELTA_TICKS} ticks (widest: {widest['joint']}, "
            f"{int(widest['span_delta_ticks']):+d} ticks). `arm explore` builds its grid from "
            "exactly those bounds (GridSpec.bounds, via arm_spec.resolve_bounds over the "
            "servo's EEPROM min_angle/max_angle), so on those joints its grid does not "
            "describe the arm: a NEGATIVE delta is travel the EEPROM claims and the joint does "
            "not have, and every cell in it is one the flood-fill will enqueue and the joint "
            "can never reach. That is issue #34's artifact, measured rather than assumed."
            + lower_bound_note
        )
    else:
        verdict = (
            f"NO joint measured here differs from its EEPROM-derived span by more than "
            f"{MATERIAL_SPAN_DELTA_TICKS} ticks. Stated plainly, and against this work's own "
            "interest: on this evidence `arm explore`'s grid was NOT being fed artifacts by "
            "its bounds, and the rationale for blocking issue #34 on this measurement DOES NOT "
            "HOLD — whatever is wrong with the grid, these bounds are not it, and #34 should "
            "not stay blocked on them." + lower_bound_note
        )

    return {
        "material_threshold_ticks": MATERIAL_SPAN_DELTA_TICKS,
        "any_material": bool(material),
        "material_joints": material_joints,
        "unvouched_joints": unvouched,
        "joints": diffs,
        "verdict": verdict,
    }


# ---------------------------------------------------------------------------
# arm limits --commit — keeping what was measured
#
# The measurement is done and the servo is back exactly as it was found. This is the
# separate, explicitly gated act that keeps a remedy instead of merely reporting one,
# and there are exactly two remedies because there are exactly two instruments:
#
#   BOUNDED (with a usable arc)  -> RE-ZERO. An EEPROM write (Ofs, addr 31) that moves
#                                   the seam into the arc the joint cannot reach.
#                                   VERIFIED BY A SWEEP, and by nothing else.
#   CONTINUOUS / arc-too-narrow  -> SOFT LIMIT. A software-only dead arc containing the
#                                   seam, which no mover is allowed to drive into.
#   travel misses the seam       -> nothing to do. The axis is already linear.
#   UNDETERMINED                 -> nothing to do, and nothing to guess. Measure again.
#
# What is NEVER written, on any of those paths: the servo's Min/Max_Position_Limit
# registers (addrs 9/11). Those CLAMP every goal in firmware and are EEPROM, so they
# would outlive the pose that produced them — a servo re-installed on another arm would
# still carry a fence measured on this one. tests/test_eeprom_limit_write_guard.py pins
# the whole package's write surface shut against them; a measured range lives in
# arm_spec and in the soft-limit store, and nowhere else.
# ---------------------------------------------------------------------------

#: What a joint's commit did. Not booleans: "nothing was committed" has four different
#: meanings here and an operator standing at an arm needs to know which one they got.
_COMMIT_REZERO = "rezero"
_COMMIT_SOFT_LIMIT = "soft_limit"
_COMMIT_NOTHING_NEEDED = "none_needed"
_COMMIT_REFUSED_UNDETERMINED = "refused_undetermined"
_COMMIT_FAILED_SEAM_NOT_EVICTED = "failed_seam_not_evicted"
_COMMIT_FAILED_UNPROVEN = "failed_unproven"


def _sweep_instructions(joint: str, motor: int, travel: int, duration: float) -> str:
    """What the human has to actually DO, and why nothing else will do instead."""
    return (
        f"\n{joint} (motor {motor}) is now LIMP and will stay that way.\n\n"
        f"HAND-MOVE IT SLOWLY THROUGH ITS ENTIRE TRAVEL — one hard stop all the way to "
        f"the other — for the next {duration:.0f} seconds. Its travel is about {travel} "
        f"ticks; the sweep must cover at least {rezero.MIN_COVERAGE:.0%} of that or its "
        "answer means nothing and this commit will be REFUSED.\n\n"
        "This is not ceremony. The offset read back correctly, which proves it was "
        "APPLIED — it proves NOTHING about whether the seam MOVED. Only a torque-off "
        "sweep of the whole travel can, because your hand is the only actuator here that "
        "does not need a linear tick axis to work, and a linear tick axis is exactly what "
        "is in doubt.\n\n"
        "If the arm is holding a pose, SUPPORT IT — it will sag."
    )


def _commit_rezero(
    bus: object,
    journal: CalibrationJournal,
    *,
    joint: str,
    motor: int,
    classification: TravelClassification,
    duration: float,
    json_mode: bool,
) -> "dict[str, object]":
    """Write the re-zero, then PROVE THE SEAM MOVED — and un-write it if it did not.

    **The sweep is the arbiter. The read-back is not.**

    An offset that reads back exactly right proves the register took the value. It says
    nothing whatever about whether the discontinuity moved with it, and there are at
    least two live ways for it not to have:

    * the firmware does a plain signed subtraction rather than reducing the corrected
      position modulo 4096, so the offset merely RELABELS positions and the seam stays
      pinned to the physical angle where the magnet rolls over
      (``docs/spikes/sts3215-offset-register.md`` §4 — settled on one arm, one firmware
      revision, which is not the same as settled);
    * **the arc was wrong.** The measurement found the joint's walls with the arm, and an
      arm can be stopped by the table, by a cable, or by a pose — anywhere short of the
      joint's real stop. Every tick of that error makes the "unreachable" arc wider than
      it truly is, and the seam gets parked somewhere the joint CAN go. This is not
      hypothetical: the first ``elbow_flex`` re-zero used an arc edge taken from a hand
      sweep somebody had stopped short of, and the joint came to rest eleven ticks past
      it. A hand reaches where the arm stopped; that is exactly why the hand is the
      instrument.

    So: journal, write, read back, and then hand the joint to a human. A sweep that finds
    a discontinuity, or that did not cover the travel
    (:attr:`~arm101.hardware.rezero.SweepReport.conclusive` — the ≥80% coverage rule that
    stopped three EMPTY sweeps of ``elbow_flex`` from being declared a pass), is a
    **FAILURE**: the original offset goes back, the journal is closed, and the operator
    is told the re-zero did not work.

    Only :data:`~arm101.hardware.rezero.VERDICT_SEAM_EVICTED` commits. "Did not fail" is
    not the same claim as "proved the fix works", and a verb that conflated them would
    persist an unverified calibration into EEPROM.

    Crash-safe by construction: the offset is journalled (durably, ``fsync``ed) before it
    reaches the wire, and only a passing sweep calls
    :func:`~arm101.hardware.journal.commit`. A Ctrl-C, a power cut or a yanked cable
    anywhere in between leaves the entry DIRTY, and the next run's ``require_clean`` puts
    the original back. **An unverified re-zero cannot survive** — which is correct: "the
    process died before it could check" is not evidence that it would have passed.
    """
    target = arm_spec.rezero_offset(joint, measured=classification)
    arc = arm_spec.rezero_arc(joint, measured=classification)
    assert target is not None and arc is not None  # nosec B101 - remedy REZERO implies both
    before = bus.read_offset(motor)  # type: ignore[attr-defined]

    read_back = rezero.commit_rezero(bus, journal, joint=joint, motor=motor, offset=target)
    if read_back != target:
        # The write did not take. The journal entry stays DIRTY on purpose: its
        # original_offset is the only record of the truth, and the next run's
        # require_clean will try again.
        raise CliError(
            code=EXIT_ENV_ERROR,
            message=(
                f"The encoder offset did NOT take: wrote {target} to {joint} (motor {motor}, "
                f"EEPROM addr 31), read back {read_back}. Nothing has been committed."
            ),
            remediation=(
                "The servo accepted the write but is not holding the value. Check the EEPROM "
                "Lock register (addr 55) is not being re-closed by another process — that dance "
                "is exactly what PR #21 was written to fix — and that motor "
                f"{motor} is the servo you think it is ('arm101 arm read'). The calibration "
                "journal still names this joint's original offset and the next run will restore "
                "it automatically."
            ),
        )

    emit_diagnostic(_sweep_instructions(joint, motor, arc.travel_ticks, duration))
    report = rezero.sweep(
        bus,  # type: ignore[arg-type]
        motor,
        joint,
        samples=rezero.samples_for(duration),
        on_sample=_sweep_progress(json_mode),
        measured=classification,
    )

    if report.verdict != rezero.VERDICT_SEAM_EVICTED:
        # NOT COMMITTED. Put the joint back exactly as it was found — the offset is in
        # EEPROM and the journal names the original, so this is a plain restore, and it
        # closes the transaction.
        restore = restore_dirty(bus, journal)  # type: ignore[arg-type]
        failure = (
            _COMMIT_FAILED_SEAM_NOT_EVICTED
            if report.failed
            else _COMMIT_FAILED_UNPROVEN  # inconclusive: a short sweep proves nothing
        )
        return {
            "committed": False,
            "action": failure,
            "offset_before": before,
            "offset_written": target,
            "offset_read_back": read_back,
            "applied": True,  # the register took it — and that is not the question
            "sweep": report.as_dict(),
            "restored": restore.complete,
            "restore": restore.to_dict(),
            "reason": report.describe(),
        }

    commit(journal, motor=motor)
    return {
        "committed": True,
        "action": _COMMIT_REZERO,
        "offset_before": before,
        "offset_written": target,
        "offset_read_back": read_back,
        "applied": True,
        "seam_now_at_raw_tick": arm_spec.seam_tick(target),
        "unreachable_arc": [arc.low, arc.high],
        "sweep": report.as_dict(),
        "persistence_proven": False,
        "reason": report.describe(),
    }


def _commit_soft_limit(
    bus: object,
    *,
    joint: str,
    motor: int,
    classification: TravelClassification,
    pose: "str | None",
    path: "str | None",
) -> "dict[str, object]":
    """Derive and STORE the soft limit — the software-only remedy. Writes NO servo register.

    Where a measured soft limit actually lands, and what reads it
    ------------------------------------------------------------
    A re-zero is an EEPROM write, so "commit" is obvious. A soft limit is software only,
    and :data:`arm101.hardware.arm_spec.SOFT_LIMITS` is a **checked-in source table** — a
    CLI does not rewrite its own source. So the measured limit is written to
    :mod:`arm101.hardware.soft_limit_store` (default ``~/.arm101/soft-limits.jsonl``),
    which :func:`_soft_limits` loads on **every** run of every mover and merges over the
    shipped table via :func:`~arm101.hardware.arm_spec.resolve_soft_limits`. From there it
    reaches :func:`~arm101.hardware.arm_spec.resolve_bounds`, which is the one function
    ``arm flex``, ``arm explore``'s grid and the demo sweep all take their move bounds
    from. **That is what reads it at runtime**, and it is why the store is loaded by
    default rather than behind a flag: a fence that binds only when you remember to ask
    for it is not a fence.

    The commit also PRINTS the ``arm_spec`` table entry a human would check in
    (:meth:`~arm101.hardware.soft_limit_store.MeasuredSoftLimit.table_entry`). The store
    makes the limit true for this operator's arm today; the checked-in table is how it
    stops being local knowledge. Both, deliberately — neither alone is enough.

    NEVER an EEPROM angle-limit write
    ---------------------------------
    The obvious-looking alternative is to write the servo's ``Min/Max_Position_Limit``
    (addrs 9/11), which really would clamp every goal, in firmware, for free. It is
    forbidden, and ``tests/test_eeprom_limit_write_guard.py`` pins the entire package's
    wire surface shut against it. Those registers are EEPROM: the fence would outlive the
    pose that produced it and travel with the servo onto another arm. A measured range is
    a claim about *this arm in this pose*, and it belongs in software where it can be
    corrected by re-measuring.

    Verified how?
    -------------
    Not by a sweep — there is nothing for a sweep to prove. A sweep answers "did the seam
    MOVE?", and a soft limit does not move the seam; it fences it. The claim a soft limit
    makes is geometric and it is checked as such, before anything is written: the dead arc
    must contain the RAW seam (:func:`~arm101.hardware.arm_spec.dead_arc_contains_seam`)
    **and** the reported seam of the servo's live offset
    (:func:`~arm101.hardware.arm_spec.dead_arc_contains_reported_seam`), each with
    :data:`~arm101.hardware.arm_spec.SEAM_CLEARANCE_TICKS` to spare. That is the whole of
    what it promises, and it is enforced by
    :func:`~arm101.hardware.arm_spec.soft_limit_for_offset`, which derives it rather than
    accepting one.
    """
    # The offset the joint will actually be holding from here — read from the servo, not
    # assumed. `arm limits` restored it, so this is the joint's own calibration, and the
    # dead arc has to contain THAT servo's reported seam (at raw == Ofs), not a factory
    # one somebody hoped for.
    offset = bus.read_offset(motor)  # type: ignore[attr-defined]

    try:
        limit = arm_spec.soft_limit_for_offset(offset)
    except ValueError as exc:
        raise CliError(
            code=EXIT_ENV_ERROR,
            message=f"No soft limit can be derived for {joint} (motor {motor}): {exc}",
            remediation=(
                "The joint's dead arc has to contain both the raw seam and the seam its own "
                f"offset ({offset}) puts in the reported frame, with "
                f"{arm_spec.SEAM_CLEARANCE_TICKS} ticks of clearance. An offset that puts the "
                "reported seam near mid-scale leaves no usable band. Inspect it with "
                "'arm101 arm read --json'."
            ),
        ) from exc

    # RESOLVE THE PROSPECTIVE TABLE BEFORE WRITING IT. A record that would make the NEXT
    # run's `_soft_limits()` raise is a record that would break every motion verb on this
    # machine until a human found and deleted a line of JSON — and it would do it *after*
    # this run reported success. The one way to get here is a genuine contradiction between
    # the shipped tables and the arm (a joint with a re-zero ARC whose fresh measurement
    # says its arc is too narrow to hold the seam), and that is the user's to settle, not
    # this verb's to paper over.
    try:
        arm_spec.resolve_soft_limits(from_file={**load_soft_limits(path), joint: limit})
    except ValueError as exc:
        raise CliError(
            code=EXIT_USER_ERROR,
            message=(
                f"Refusing to store a soft limit for {joint}: it would contradict the shipped "
                f"tables, and nothing has been written. {exc}"
            ),
            remediation=(
                f"{joint} has a MEASURED unreachable arc in arm_spec.REZERO_ARCS *and* this run "
                "measured its travel as needing a soft limit instead. Those are the two "
                "mutually exclusive answers to a wrapping joint, and only a human can say which "
                "is right: the arc in the table was measured on some arm, and this measurement "
                f"was taken on yours. Re-measure with the workspace clear ('arm101 arm limits "
                f"{joint} --apply') and compare the span — if the arm is being stopped short of "
                "its real walls, the arc is fine and the MEASUREMENT is wrong; if the span is "
                "genuinely that wide, the table's arc is stale and should be removed."
            ),
        ) from exc

    measured = MeasuredSoftLimit(
        joint=joint,
        limit=limit,
        offset=offset,
        kind=classification.kind.value,
        swept_ticks=classification.swept_ticks,
        reason=classification.reason,
        pose=pose,
    )
    written_to = record_soft_limit(measured, path)

    return {
        "committed": True,
        "action": _COMMIT_SOFT_LIMIT,
        "soft_limit": {
            "min_tick": limit.min_tick,
            "max_tick": limit.max_tick,
            "frame": "raw",
            "dead_arc_ticks": limit.dead_arc_ticks,
            "derived_for_offset": offset,
            "seam_at_raw_tick": arm_spec.seam_tick(offset),
        },
        "stored_at": str(written_to),
        "arm_spec_entry": measured.table_entry(),
        "registers_written": [],  # software only. NOT addr 9/11, not ever.
        "reason": classification.reason,
    }


def _commit_joint(
    bus: object,
    journal: CalibrationJournal,
    *,
    joint: str,
    motor: int,
    classification: TravelClassification,
    pose: "str | None",
    duration: float,
    path: "str | None",
    json_mode: bool,
) -> "dict[str, object]":
    """Route ONE joint's measurement to its remedy — or to no remedy, and say which.

    The routing is :attr:`~arm101.hardware.classify.TravelClassification.remedy`'s, not
    this function's: which instrument applies is a property of the *joint's travel*, and
    the classifier already derived it from the measurement without knowing the joint's
    name. Re-deriving it here would be a second opinion nobody asked for, and a second
    place for the two to disagree.
    """
    remedy = classification.remedy

    if remedy is SeamRemedy.REZERO:
        return _commit_rezero(
            bus,
            journal,
            joint=joint,
            motor=motor,
            classification=classification,
            duration=duration,
            json_mode=json_mode,
        )

    if remedy is SeamRemedy.SOFT_LIMIT:
        return _commit_soft_limit(
            bus,
            joint=joint,
            motor=motor,
            classification=classification,
            pose=pose,
            path=path,
        )

    if remedy is SeamRemedy.NONE_NEEDED:
        return {
            "committed": False,
            "action": _COMMIT_NOTHING_NEEDED,
            "reason": (
                f"{joint} needs no remedy: its travel does not cross the encoder seam, so its "
                "reported position is already monotonic with joint angle. There is nothing to "
                "evict and nothing to fence off. Committing something here would be inventing "
                "a problem to solve."
            ),
        }

    # UNKNOWN — the travel is UNDETERMINED. This is the answer that must NOT be rounded.
    return {
        "committed": False,
        "action": _COMMIT_REFUSED_UNDETERMINED,
        "reason": (
            f"REFUSED — nothing committed for {joint}. {classification.reason}\n\n"
            "Neither instrument can be chosen on this evidence. A re-zero needs an unreachable "
            "arc, and an arc cannot be sited without a WALL vouching for BOTH ends; a soft limit "
            "needs to know where the joint actually goes. Picking one anyway would burn an "
            "EEPROM write, or fence off real travel, on a measurement that does not support "
            f"either. MEASURE AGAIN: 'arm101 arm limits {joint} --apply' — and if an end keeps "
            "coming back TORQUE_LIMITED rather than WALL, the joint is stalling under its own "
            "load before it reaches its stop, not finding one."
        ),
    }


def _commit_summary(measurements: "list[dict[str, object]]") -> "dict[str, object]":
    """Fold the per-joint commits into one answer — including the one that is a FAILURE.

    ``failed`` is the field that matters, and it is the reason this summary exists at all:
    a run that wrote an offset and could not prove the seam moved must not be able to look
    like a run that quietly had nothing to do. Both have an empty ``committed`` list.
    """
    commits = [m["commit"] for m in measurements if m.get("commit")]  # type: ignore[union-attr]
    committed = [c for c in commits if c["committed"]]  # type: ignore[index]
    failed = [
        c
        for c in commits
        if c["action"]  # type: ignore[index]
        in (_COMMIT_FAILED_SEAM_NOT_EVICTED, _COMMIT_FAILED_UNPROVEN)
    ]
    return {
        "attempted": len(commits),
        "committed": len(committed),
        "failed": len(failed),
        "rezeroed": [
            m["joint"]
            for m in measurements
            if (m.get("commit") or {}).get("action") == _COMMIT_REZERO  # type: ignore[union-attr]
        ],
        "soft_limited": [
            m["joint"]
            for m in measurements
            if (m.get("commit") or {}).get("action")  # type: ignore[union-attr]
            == _COMMIT_SOFT_LIMIT
        ],
        "refused": [
            m["joint"]
            for m in measurements
            if (m.get("commit") or {}).get("action")  # type: ignore[union-attr]
            == _COMMIT_REFUSED_UNDETERMINED
        ],
    }


def _limits_payload(
    role: str,
    port: str,
    pose: "str | None",
    measurements: "list[dict[str, object]]",
    *,
    commit_mode: bool = False,
) -> "dict[str, object]":
    """The whole report: per-joint bounds and verdicts, plus the bounds diff. Nothing else.

    In particular: no cells, no reachability score, no map. Those are ``arm explore``'s
    (and issue #34's), and a measurement verb that quietly grew them would have become the
    very thing this one is a prerequisite for.

    Under ``--commit`` each joint additionally carries a ``commit`` block — what was (or
    was NOT) kept, and why — and the report gains a ``commits`` summary. The measurement
    keys are unchanged either way: a commit run is a measure run that then acted, and its
    evidence is worth exactly as much.
    """
    diffs = [dict(m["bounds"]) for m in measurements]  # type: ignore[arg-type]
    payload: "dict[str, object]" = {
        "verb": _LIMITS_VERB,
        "role": role,
        "port": port,
        "pose": pose,
        "committing": commit_mode,
        "joints": measurements,
        "bounds_diff": _bounds_diff(diffs),
    }
    if commit_mode:
        payload["commits"] = _commit_summary(measurements)
    return payload


def _emit_limits_plan(
    role: str,
    joints: "tuple[str, ...]",
    thresholds: "dict[str, int]",
    step: int,
    max_travel: int,
    compliance: int,
    pose: "str | None",
    *,
    port: "str | None",
    json_mode: bool,
    commit_mode: bool = False,
    duration: float = 0.0,
) -> None:
    """Emit the dry-run plan for a limits run — zero motion, zero bus access.

    Under ``--commit`` the plan cannot name the offsets it would write, and does not
    pretend to: **which remedy each joint gets is derived from a measurement that has not
    been taken yet.** A plan that guessed would be guessing from the shipped table — the
    very table this verb exists to correct. What it CAN state, and does, is the decision
    procedure, the fact that a re-zero is a persistent EEPROM write, and that a human hand
    will be required for each sweep.
    """
    plan: "dict[str, object]" = {
        "verb": _LIMITS_VERB,
        "role": role,
        "port": port or _PORT_UNRESOLVED,
        "joints": list(joints),
        "thresholds": {joint: thresholds[joint] for joint in joints},
        "step": step,
        "max_travel": max_travel,
        "compliance": compliance,
        "pose": pose,
        "commit": commit_mode,
    }

    if not commit_mode:
        plan["note"] = (
            "COMMANDS MOTION: each joint is creeped to BOTH ends of its travel under "
            "contact detection, through a rolling frame that keeps the encoder seam out "
            "of its way. MEASURE-ONLY — the borrowed encoder offset is restored and the "
            "servo is left exactly as it was found. Nothing is kept. Add --commit to keep "
            "the remedy the measurement points to."
        )
    else:
        plan["sweep_duration_s"] = duration
        plan["decision"] = {
            "bounded": (
                "RE-ZERO — a persistent EEPROM write (Ofs/Homing_Offset, addr 31) moving the "
                "seam into the arc the joint cannot reach. THEN VERIFIED BY A HAND SWEEP: the "
                "offset reading back proves it was APPLIED, not that the seam MOVED. A sweep "
                "that finds a discontinuity, or that covers less than "
                f"{rezero.MIN_COVERAGE:.0%} of the travel, RESTORES the original offset and "
                "fails."
            ),
            "continuous": (
                "SOFT LIMIT — software only, no servo register written. A dead arc containing "
                "the seam is derived and appended to the measured soft-limit store "
                f"({default_soft_limit_path()}), which every mover reads through "
                "arm_spec.resolve_bounds. The arm_spec table entry to check in is printed too."
            ),
            "undetermined": (
                "NOTHING. Neither instrument is supported by the evidence. Measure again."
            ),
            "never": (
                "The servo's Min/Max_Position_Limit registers (addrs 9/11) are NEVER written, "
                "on any path. They clamp goals in firmware and are EEPROM — a fence written "
                "there would outlive the pose that produced it and travel with the servo."
            ),
        }
        plan["note"] = (
            "COMMANDS MOTION **and WRITES EEPROM**. Each joint is measured exactly as it is "
            "without --commit; then the remedy its travel points to is KEPT. A re-zero is a "
            "PERSISTENT change to every position that servo will ever report. Each re-zeroed "
            f"joint then needs YOU to hand-move it through its whole travel for {duration:.0f}s "
            "— that sweep is the only thing that can prove the seam actually moved, and a "
            "commit without it is refused."
        )

    if json_mode:
        emit_result({"plan": plan}, json_mode=True)
        return

    lines = ["## Dry-run plan: arm limits", ""]
    for key, value in plan.items():
        lines.append(f"- {key}: {value}")
    lines.append("")
    lines.append(_DRY_RUN_FOOTER)
    emit_result("\n".join(lines), json_mode=False)


def _confirm_limits(
    role: str,
    joints: "tuple[str, ...]",
    *,
    json_mode: bool,
    commit_mode: bool = False,
) -> bool:
    """Prompt the human before a limits run; return True to proceed.

    Two different warnings, because they are two different acts. A measure run moves the
    arm and puts everything back. A ``--commit`` run moves the arm and then **changes what
    the servo will report forever** — and the operator has to be told that in those words,
    before the prompt, not discover it in the report afterwards.
    """
    emit_diagnostic(
        f"⚠ This COMMANDS MOTION on the {role} arm: {', '.join(joints)} will each be driven "
        "to BOTH ends of their travel until they stop. Clear the workspace — a joint that "
        "finds the table instead of its own end-stop will record the table."
    )
    if commit_mode:
        emit_diagnostic(
            "⚠⚠ --commit: this also KEEPS what it finds.\n"
            "   - A joint with real walls gets a PERSISTENT EEPROM WRITE (encoder offset, addr "
            "31): every position it reports afterwards is in a new frame, for good.\n"
            "   - You will then be asked to HAND-MOVE that joint through its entire travel. "
            "That sweep is the only proof the seam moved; without it the write is UNDONE.\n"
            "   - A joint that turns all the way round gets a SOFTWARE soft limit instead — no "
            "servo register is written, and it costs it some travel.\n"
            "   - The servo's angle-limit registers (addrs 9/11) are never written, on any path."
        )
    ans = _prompt(_CONFIRM_MOTION_PROMPT)
    if ans.strip().lower() == "yes":
        return True
    if json_mode:
        emit_result({"aborted": True, "role": role}, json_mode=True)
    else:
        emit_result(_ABORTED_NO_MOTION, json_mode=False)
    return False


def _fmt_delta(value: int) -> str:
    return f"{value:+d}"


def _emit_limits_result(payload: "dict[str, object]", *, json_mode: bool) -> None:
    """Render the measurement (text or JSON).

    The JSON is the artifact that matters. ``loaded_run_ticks`` — the distance a joint
    travelled while already pushing past its contact threshold — is what separates a WALL
    from an arm that merely ran out of torque, and **its cutoff is currently derived from a
    simulation, not from the arm**. The whole point of the first hardware session is to
    retune that cutoff from real data, so every end's ``loaded_run_ticks``, ``free_run``,
    ``peak_load``, verdict and reason are in the payload verbatim. An operator who had to
    re-instrument the run to get them would have wasted it.
    """
    if json_mode:
        emit_result(payload, json_mode=True)
        return

    diff = payload["bounds_diff"]  # type: ignore[index]
    lines = [
        f"## arm limits ({payload['role']}) — {payload['port']}",
        "",
    ]
    if payload["pose"] is not None:
        # Named, not decorative: every limit below was found with the OTHER joints in
        # this pose, and an obstacle in it narrows the travel. The record is evidence
        # about a pose, never about the joint alone.
        lines.append(f"Pose: {payload['pose']}")
        lines.append("")
    lines += [
        "| joint | kind | low | high | measured span | EEPROM span | delta | remedy |",
        "|-------|------|-----|------|---------------|-------------|-------|--------|",
    ]
    for entry in payload["joints"]:  # type: ignore[union-attr]
        ends = entry["ends"]
        bounds = entry["bounds"]
        low = ends.get("low", {}).get("verdict", "-")
        high = ends.get("high", {}).get("verdict", "-")
        lines.append(
            f"| {entry['joint']} | {entry['kind']} | {low} | {high} "
            f"| {bounds['measured_span_ticks']} | {bounds['eeprom_span_ticks']} "
            f"| {_fmt_delta(int(bounds['span_delta_ticks']))} | {entry['remedy']} |"
        )

    lines.append("")
    lines.append("### What each joint's travel means")
    lines.append("")
    for entry in payload["joints"]:  # type: ignore[union-attr]
        lines.append(f"- **{entry['joint']}** — {entry['reason']}")
        for end, evidence in entry["ends"].items():
            lines.append(
                f"  - {end}: {evidence['verdict']} "
                f"(loaded run {evidence['loaded_run_ticks']} ticks against a "
                f"{evidence['compliance']}-tick cutoff; free run "
                f"{evidence['free_run_ticks']}; peak load {evidence['peak_load']}) "
                f"— {evidence['reason']}"
            )

    lines.append("")
    lines.append("### Bounds diff — what `arm explore` believes, and what is true")
    lines.append("")
    lines.append(str(diff["verdict"]))  # type: ignore[index]
    lines.append("")

    if not payload.get("committing"):
        lines.append(
            "The servo was left exactly as it was found: every borrowed encoder offset has "
            "been restored. Nothing was kept — add --commit to keep the remedy each "
            "measurement points to."
        )
        emit_result("\n".join(lines), json_mode=False)
        return

    lines += _commit_lines(payload)
    emit_result("\n".join(lines), json_mode=False)


def _commit_lines(payload: "dict[str, object]") -> "list[str]":
    """The ``--commit`` half of the text report: what was kept, what was refused, what FAILED.

    A failure gets the loudest voice in the room. An operator who scrolls past a
    ``seam-not-evicted`` verdict because it was rendered as one more row in a table has
    been failed by the report, not by the arm.
    """
    summary = payload["commits"]  # type: ignore[index]
    lines = ["### What was COMMITTED", ""]
    lines.append(
        f"- attempted : {summary['attempted']}"  # type: ignore[index]
        f"   committed: {summary['committed']}"  # type: ignore[index]
        f"   FAILED: {summary['failed']}"  # type: ignore[index]
    )
    lines.append("")

    table_entries: "list[str]" = []
    for entry in payload["joints"]:  # type: ignore[union-attr]
        result = entry.get("commit")
        if not result:
            continue
        joint = entry["joint"]

        if result["action"] == _COMMIT_REZERO:
            lines += [
                f"- **{joint}** — RE-ZEROED. Encoder offset {result['offset_before']} -> "
                f"{result['offset_written']} (EEPROM addr 31, read back "
                f"{result['offset_read_back']}); the seam now sits at raw tick "
                f"{result['seam_now_at_raw_tick']}, inside the arc "
                f"{result['unreachable_arc']} the joint cannot reach. "
                "**PROVEN BY SWEEP** — not by the read-back.",
                "  - POWER-CYCLE the servo and re-read the offset ('arm101 arm read --json'): "
                "an EEPROM write can read back correctly and still revert (PR #21). Persistence "
                "is the one thing still unproven.",
            ]
        elif result["action"] == _COMMIT_SOFT_LIMIT:
            limit = result["soft_limit"]  # type: ignore[index]
            lines += [
                f"- **{joint}** — SOFT-LIMITED (software only; no servo register written). "
                f"Permitted RAW travel [{limit['min_tick']}, {limit['max_tick']}]; "
                f"{limit['dead_arc_ticks']} ticks fenced off around the seam at raw "
                f"{limit['seam_at_raw_tick']} (the joint's own offset is "
                f"{limit['derived_for_offset']}).",
                f"  - Stored at {result['stored_at']} — every mover reads it from there, via "
                "arm_spec.resolve_bounds. It is in force on the next `arm flex` / `arm explore`.",
            ]
            table_entries.append(str(result["arm_spec_entry"]))
        elif result["action"] == _COMMIT_NOTHING_NEEDED:
            lines.append(f"- **{joint}** — nothing to commit. {result['reason']}")
        elif result["action"] == _COMMIT_REFUSED_UNDETERMINED:
            lines.append(f"- **{joint}** — REFUSED. {result['reason']}")
        else:
            lines += [
                "",
                f"*** {joint}: THE RE-ZERO FAILED, AND NOTHING WAS COMMITTED. ***",
                "",
                f"The offset was written and read back correctly ({result['offset_read_back']}) "
                "— which proves it was APPLIED, and proves nothing at all about whether the "
                "seam MOVED. The sweep says it did not:",
                "",
                str(result["reason"]),
                "",
                f"The original offset ({result['offset_before']}) has been RESTORED "
                f"(verified: {result['restored']}). The joint is exactly as it was found.",
                "",
            ]

    if table_entries:
        lines += [
            "",
            "### To make a measured soft limit everyone's, not just this arm's",
            "",
            "The store above is read at runtime and is enough for THIS machine. Paste these "
            "into `SOFT_LIMITS` in arm101/hardware/arm_spec.py to ship them — a measurement "
            "that stays in one operator's home directory will be made again by the next "
            "person:",
            "",
        ]
        lines += table_entries

    return lines


def cmd_arm_limits(args: argparse.Namespace) -> None:
    """Measure each joint's TRUE travel — and, with ``--commit``, KEEP the remedy it points to.

    The verb the probe, the rolling frame and the classifier were built for. Per joint: roll
    the encoder seam out of the way, creep to both ends under contact detection, and rule on
    what stopped it (WALL / TORQUE_LIMITED / EDGE / TIMEOUT — per END, because a joint
    routinely has a solid wall one way and a torque-limited stall the other). Then classify
    the travel (BOUNDED / CONTINUOUS / UNDETERMINED) and diff the measured span against the
    EEPROM-derived span ``arm explore`` builds its grid from today.

    Without ``--commit`` it is MEASURE-ONLY, exactly as it was: the borrowed offset is
    restored and the servo is left as it was found.

    ``--commit`` — the separate, explicitly gated act
    ------------------------------------------------
    The measurement is the same. What changes is that its **remedy** is then kept, and which
    remedy that is comes from the joint's travel, not from anybody's preference:

    * **BOUNDED**, with an arc that can take the seam -> **RE-ZERO**: a persistent EEPROM
      write (``Ofs``, addr 31) moving the seam into the arc the joint cannot reach —
      **and then PROVEN BY A HAND SWEEP.** The offset reading back proves it was APPLIED.
      It proves nothing about whether the seam MOVED. A sweep that finds a discontinuity —
      or that covers less than :data:`~arm101.hardware.rezero.MIN_COVERAGE` of the travel,
      which is the rule that stopped three EMPTY sweeps of ``elbow_flex`` from being
      declared a pass — RESTORES the original offset and reports a FAILURE.
    * **CONTINUOUS** (or an arc too narrow to hold the seam clear of both walls) -> **SOFT
      LIMIT**: a software-only dead arc containing the seam, appended to the measured
      soft-limit store and read back by every mover through ``arm_spec.resolve_bounds``.
      No servo register is written.
    * **UNDETERMINED** -> **nothing at all.** Neither instrument is supported by the
      evidence, and choosing one anyway would be inventing a measurement. Measure again.

    The servo's ``Min/Max_Position_Limit`` registers (addrs 9/11) are **never** written, on
    any path — ``tests/test_eeprom_limit_write_guard.py`` pins the package's whole wire
    surface shut against them.

    Contracts, all of them load-bearing:

    * **``require_clean`` first.** Before this verb touches the arm, a calibration a crashed
      run left behind is put back. Layering a fresh temporary offset on top of one nobody
      restored is how the ORIGINAL offset stops being knowable.
    * **Every commit is a TRANSACTION.** The offset is journalled (durably) before it
      reaches the wire, and only a PASSING sweep closes the journal as ``committed``. A
      crash, a Ctrl-C or a yanked cable anywhere in between leaves the entry dirty, and the
      next run's ``require_clean`` restores the original. An unverified re-zero cannot
      survive — and should not.
    * **The whole run is inside a torque guard.** It owns each motor from the moment that
      motor can first go hot, and never disowns it — so a bus that dies while joint 5 is
      being probed still releases joints 1-4, whose frames closed minutes ago and whose
      servos may well still be holding.

    What it deliberately does NOT do: enqueue cells, score reachability, or emit a map.
    Those are ``arm explore``'s, and issue #34's.

    Raises
    ------
    CliError(EXIT_ENV_ERROR)
        If a committed re-zero could not be PROVEN — the sweep found a discontinuity under
        an offset that read back correctly, or did not cover enough travel to have met one.
        The offset is restored first and the full sweep report is emitted to stdout BEFORE
        the error, because the numbers are why the operator ran the command and they are
        worth exactly as much when the answer is "it did not work".
    """
    role: str = args.role
    json_mode = bool(getattr(args, "json", False))
    joints = _limits_joints(getattr(args, "joint", None))
    thresholds = _resolve_contact_thresholds(args)
    pose: "str | None" = getattr(args, "pose", None)
    commit_mode = bool(getattr(args, "commit", False))
    soft_limit_file: "str | None" = getattr(args, "soft_limit_file", None)

    raw_step = getattr(args, "step", None)
    step = DEFAULT_CREEP_TICKS if raw_step is None else int(raw_step)
    raw_max_travel = getattr(args, "max_travel", None)
    max_travel = ENCODER_TICKS if raw_max_travel is None else int(raw_max_travel)
    raw_compliance = getattr(args, "compliance", None)
    compliance = wall_compliance() if raw_compliance is None else int(raw_compliance)
    raw_duration = getattr(args, "sweep_duration", None)
    duration = rezero.DEFAULT_SWEEP_DURATION if raw_duration is None else float(raw_duration)

    # Validate the sweep length BEFORE consent, so a --sweep-duration too short to collect
    # two samples is a user error caught up front rather than after the operator has said
    # yes and the arm has spent ten minutes creeping.
    if commit_mode:
        rezero.samples_for(duration)

    mode = resolve_consent(args, verb=_LIMITS_VERB, require_plan_hash=False)

    # --- dry_run: plan only, zero motion, zero bus access ---
    if mode == "dry_run":
        _emit_limits_plan(
            role,
            joints,
            thresholds,
            step,
            max_travel,
            compliance,
            pose,
            port=getattr(args, "port", None),
            json_mode=json_mode,
            commit_mode=commit_mode,
            duration=duration,
        )
        return

    # --- interactive: confirm at a prompt before any bus is opened ---
    if mode == "interactive" and not _confirm_limits(
        role, joints, json_mode=json_mode, commit_mode=commit_mode
    ):
        return

    # --- agent OR interactive-confirmed: open the bus and measure ---
    port = _resolve_port(getattr(args, "port", None))
    ids = arm_spec.joint_ids(role)

    # The fence already in force, resolved before the port is opened. `limits` reads it for
    # the same reason `flex` does — the bounds it diffs against are the bounds `explore`
    # would actually use — and a store it cannot parse stops the run here, before the arm
    # moves under a constraint the operator believes is in force.
    soft_limits = _soft_limits(args)

    bus = _open_bus(port)
    measurements: "list[dict[str, object]]" = []
    stop: "CliError | None" = None
    try:
        # The guard starts owning NOTHING, exactly as ``explore``'s does: everything
        # before the first frame opens is a READ (require_clean writes only if a previous
        # run left a joint dirty, and an offset write de-energises rather than energises).
        # A fault there has no hot motor to release, and claiming the arm up front would
        # make the guard announce a release for six motors that were never energised — a
        # safety report that cries wolf teaches a human to ignore the one line that must
        # never be ignored.
        with torque_guard(bus, on_release=_release_announcer(json_mode)) as guard:
            journal = CalibrationJournal()

            # BEFORE anything else touches the arm. A crashed run may have left a joint
            # holding a temporary offset, in which case every tick it reports is in a
            # frame nobody chose and no measurement taken now could be trusted.
            require_clean(bus, journal)

            # Read what ``arm explore`` would believe, while the servos still hold their
            # OWN calibration — a bound read through a rolling frame's borrowed offset is
            # a number in a frame explore never sees.
            eeprom_bounds = {
                joint: _resolve_joint_bounds(
                    joint,
                    bus.read_info(ids[joint]),  # type: ignore[attr-defined]
                    soft_limits,
                )
                for joint in joints
            }

            for joint in joints:
                motor = ids[joint]
                # From here this motor CAN go hot. Own it before it does, and never
                # disown it: a joint whose frame has closed may still be holding.
                guard.own(motor)
                outcomes, classification = _measure_joint(
                    bus,
                    journal,
                    joint=joint,
                    motor=motor,
                    threshold=thresholds[joint],
                    step=step,
                    max_travel=max_travel,
                    compliance=compliance,
                    pose=pose,
                )
                record: "dict[str, object]" = {
                    **classification.to_dict(),
                    "motor": motor,
                    "ends": {
                        end: {**outcome.as_dict(), "verdict": outcome.verdict.value}
                        for end, outcome in outcomes.items()
                    },
                    "bounds": _joint_bounds_diff(joint, classification, eeprom_bounds[joint]),
                }

                if commit_mode:
                    # The frame has restored the borrowed offset; the servo is exactly as
                    # it was found. NOW keep the remedy — a separate act, on a joint whose
                    # calibration is its own again.
                    result = _commit_joint(
                        bus,
                        journal,
                        joint=joint,
                        motor=motor,
                        classification=classification,
                        pose=pose,
                        duration=duration,
                        path=soft_limit_file,
                        json_mode=json_mode,
                    )
                    record["commit"] = result
                    if result["action"] in (
                        _COMMIT_FAILED_SEAM_NOT_EVICTED,
                        _COMMIT_FAILED_UNPROVEN,
                    ):
                        # STOP. Not "try the next joint": a re-zero that could not be
                        # proven is a claim about the servo's firmware and about the arc
                        # we measured it from, and both of those are the ground every
                        # remaining joint's commit would stand on. The report is still
                        # emitted in full (below) — the numbers are the whole point — and
                        # THEN this is raised.
                        measurements.append(record)
                        stop = _seam_stop(joint, motor, result)
                        break

                measurements.append(record)
    finally:
        bus.close()

    _emit_limits_result(
        _limits_payload(role, port, pose, measurements, commit_mode=commit_mode),
        json_mode=json_mode,
    )
    if stop is not None:
        raise stop


def _seam_stop(joint: str, motor: int, result: "dict[str, object]") -> CliError:
    """The STOP condition: an offset that landed, and a seam that did not move.

    Two ways to get here, and they are NOT the same finding — so they do not get the same
    message:

    * ``seam-not-evicted`` — the sweep covered the travel and SAW a discontinuity while the
      seam-evicting offset was in force. Either the servo does not reduce its corrected
      position modulo 4096 (the offset merely relabels positions and the seam stays pinned
      to the physical angle where the magnet rolls over), or the arc we derived the offset
      from is wrong — the arm found a wall that was the table, or a pose, or a cable, and
      the "unreachable" arc contains ticks the joint can plainly reach. Both are decisions
      for the user, not retries for the tool.
    * ``inconclusive`` — the sweep was CLEAN but too short to mean anything. Almost always
      the joint was not actually hand-moved: an unattended ``--apply --commit`` in agent
      mode collects six hundred samples of a stationary joint, sees no seam (of course it
      does not), and must NOT be allowed to call that a pass. This is the ≥80% coverage
      rule doing exactly the job it was written for.
    """
    sweep: "dict[str, object]" = result["sweep"]  # type: ignore[assignment]
    largest_jump = sweep["largest_jump"]
    span, expected = sweep["span"], sweep["expected_travel"]
    if result["action"] == _COMMIT_FAILED_SEAM_NOT_EVICTED:
        return CliError(
            code=EXIT_ENV_ERROR,
            message=(
                f"SEAM NOT EVICTED — nothing was committed for {joint} (motor {motor}). The "
                f"offset {result['offset_written']} was written and read back correctly, and "
                f"the joint's reported position STILL jumps {largest_jump} ticks "
                "mid-travel. Reading the offset back proved it was APPLIED; the sweep proves "
                "the seam did NOT move. The original offset has been restored."
            ),
            remediation=(
                "STOP — do not build on this. Two things can produce it and they need "
                "different answers: (1) the servo does not reduce its corrected position "
                "modulo 4096, so no offset can ever evict its seam and a soft limit is the "
                "only instrument left (docs/spikes/sts3215-offset-register.md, section 4); "
                "or (2) the ARC IS WRONG — the arm's wall was an obstacle, not the joint's "
                "stop, so the 'unreachable' arc contains ticks the joint can reach and the "
                "seam was parked in one of them. Re-measure with the workspace clear "
                f"('arm101 arm limits {joint} --apply') and compare the span; if it is wider "
                "this time, the arc was the problem. Take the sweep report above back to the "
                "user for a re-decision either way."
            ),
        )
    return CliError(
        code=EXIT_ENV_ERROR,
        message=(
            f"UNPROVEN — nothing was committed for {joint} (motor {motor}). The sweep found no "
            f"discontinuity, but it only covered {span} of the joint's ~{expected} ticks of "
            f"travel ({rezero.MIN_COVERAGE:.0%} is required), so it proves NOTHING: of course "
            "it saw no seam — it never went near where the seam would be. The original offset "
            "has been restored."
        ),
        remediation=(
            "A clean sweep that did not cover the travel is INCONCLUSIVE, never a pass. This "
            "is the rule that stopped three EMPTY sweeps of elbow_flex from being declared a "
            "success, and it is not negotiable.\n\n"
            "Almost certainly the joint was never hand-moved. --commit REQUIRES A HUMAN at the "
            "arm: the seam-eviction proof is a torque-off sweep through the joint's whole "
            "travel, and a human hand is the only actuator that does not need a linear tick "
            "axis to work — which is precisely what is in doubt. Re-run and move the joint "
            "from one hard stop ALL THE WAY to the other, or raise --sweep-duration if there "
            "was not enough time."
        ),
    )


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
    fx.add_argument(
        "--soft-limit-file",
        default=None,
        metavar="PATH",
        help=(
            "The measured soft-limit store to honour (default "
            f"{default_soft_limit_path()}; also settable via $ARM101_SOFT_LIMITS). Loaded "
            "whether or not you pass this — 'arm limits --commit' writes measured soft "
            "limits there, and every mover reads them back through arm_spec.resolve_bounds. "
            "A fence that only binds when you remember a flag is not a fence."
        ),
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
    ex.add_argument(
        "--soft-limit-file",
        default=None,
        metavar="PATH",
        help=(
            "The measured soft-limit store to honour (default "
            f"{default_soft_limit_path()}; also settable via $ARM101_SOFT_LIMITS). Loaded "
            "whether or not you pass this — 'arm limits --commit' writes measured soft "
            "limits there, and every mover reads them back through arm_spec.resolve_bounds. "
            "A fence that only binds when you remember a flag is not a fence."
        ),
    )
    ex.add_argument("--json", action="store_true", help=_JSON_HELP)
    ex.set_defaults(func=cmd_arm_explore, json=False)

    # profile — gated motion verb (highest speed that still DETECTS contact)
    pr = noun_sub.add_parser(
        "profile",
        help=(
            "Find the highest speed at which contact detection STILL WORKS for one "
            "joint: ramps the speed and certifies each candidate against a REAL "
            "contact (--contact-to); gated motion (use --apply in non-TTY agent mode)."
        ),
    )
    pr.add_argument(
        "joint",
        help="Joint to profile (one of the 6 SO-101 joints).",
    )
    pr.add_argument(
        "--contact-to",
        type=int,
        required=True,
        metavar="TICK",
        help=(
            "REQUIRED. A tick the joint genuinely CANNOT reach — its mechanical "
            "end-stop, or a fixture clamped in its path. Every candidate speed is "
            "certified by driving INTO this contact and requiring the stall rule to "
            "detect it; a reachable target proves nothing and voids the run."
        ),
    )
    pr.add_argument(
        "--threshold",
        type=int,
        default=None,
        help=(
            "Contact-load threshold for this joint (default: its hardware-tuned "
            "per-joint value, the same one 'arm explore' uses). Must be < 500 — "
            "present_load saturates at gentle_move's Torque_Limit cap."
        ),
    )
    pr.add_argument(
        "--speed-start",
        type=int,
        default=None,
        help=(
            f"First candidate speed (default {DEFAULT_SPEED_START}: gentle_move's own "
            "default, and the only speed contact detection has ever been proven at)."
        ),
    )
    pr.add_argument(
        "--speed-step",
        type=int,
        default=None,
        help=f"Step between candidate speeds (default {DEFAULT_SPEED_STEP}).",
    )
    pr.add_argument(
        "--speed-max",
        type=int,
        default=None,
        help=(
            f"Highest candidate speed to try (default {DEFAULT_SPEED_MAX}, which "
            "brackets the speed 400 at which a one-shot overload was measured)."
        ),
    )
    pr.add_argument(
        "--role",
        choices=arm_spec.roles(),
        default="follower",
        help=_ROLE_HELP,
    )
    pr.add_argument(
        "--port",
        default=None,
        help=_PORT_HELP,
    )
    pr.add_argument(
        "--apply",
        action="store_true",
        default=False,
        help="Execute the profiling run (non-TTY agent mode; ignored under a TTY).",
    )
    pr.add_argument("--json", action="store_true", help=_JSON_HELP)
    pr.set_defaults(func=cmd_arm_profile, json=False)

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
        # RENDERED from arm_spec, not re-typed. This help string used to claim "Only
        # elbow_flex wraps inside its travel" — a claim hardware withdrew (issue #43)
        # while this copy of it went on being printed. Prose that RESTATES a table drifts
        # from it; prose that RENDERS it cannot.
        help="Joint to re-zero. " + arm_spec.REZERO_UNKNOWN_HEADLINE,
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

    # limits — gated motion verb (MEASURE the travel; change nothing)
    lm = noun_sub.add_parser(
        "limits",
        help=(
            "Measure each joint's TRUE travel: roll the encoder seam out of the way, "
            "creep to BOTH ends under contact detection, and classify what stopped it "
            "(WALL / TORQUE_LIMITED / EDGE / TIMEOUT, per end). MEASURE-ONLY by default — "
            "the borrowed encoder offset is restored and the servo is left exactly as it "
            "was found. Add --commit to KEEP the remedy the measurement points to: a "
            "sweep-verified encoder re-zero for a joint with real walls, a software soft "
            "limit for one that turns all the way round, and nothing at all for one whose "
            "travel is UNDETERMINED. Reports the delta against the EEPROM-derived bounds "
            "'arm explore' uses today. Gated motion (use --apply in non-TTY agent mode)."
        ),
    )
    lm.add_argument(
        "joint",
        nargs="*",
        help="Joints to measure (default: every joint, in hardware order).",
    )
    lm.add_argument(
        "--threshold",
        type=int,
        default=None,
        help=(
            "Blanket contact-load threshold applied to EVERY joint, overriding "
            "--threshold-file and the per-joint defaults (default: per-joint, "
            "hardware-tuned — the same values 'arm explore' uses)."
        ),
    )
    lm.add_argument(
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
    lm.add_argument(
        "--threshold-file",
        default=None,
        metavar="PATH",
        help=(
            "JSONL file of per-joint contact thresholds "
            '({"joint": name, "threshold": N} per line). CLI flags override '
            "file entries; file overrides built-in defaults."
        ),
    )
    lm.add_argument(
        "--step",
        type=int,
        default=None,
        metavar="TICKS",
        help=(
            f"Ticks per creep step — the length of ONE gentle move (default "
            f"{DEFAULT_CREEP_TICKS}). Smaller steps look around more often and pay the "
            "servo's motion-onset dead window more often."
        ),
    )
    lm.add_argument(
        "--max-travel",
        type=int,
        default=None,
        metavar="TICKS",
        help=(
            f"Travel budget per END, in ticks (default {ENCODER_TICKS}, a full turn — "
            "past which the joint is CONTINUOUS and there is nothing left to learn)."
        ),
    )
    lm.add_argument(
        "--compliance",
        type=int,
        default=None,
        metavar="TICKS",
        help=(
            "The widest LOADED approach a WALL may show (default "
            f"{wall_compliance()}, twice gentle_move's measured contact-relief "
            "distance). Push past the contact threshold for longer than this and the "
            "joint was carrying a load, not meeting one: the verdict becomes "
            "TORQUE_LIMITED. THIS CUTOFF IS CURRENTLY DERIVED FROM A SIMULATION — the "
            "first hardware run should retune it from the loaded_run_ticks the --json "
            "payload reports per end. Raising it is the one change here that can "
            "manufacture a WALL that is not there."
        ),
    )
    lm.add_argument(
        "--pose",
        default=None,
        metavar="LABEL",
        help=(
            "Opaque label recorded on every observation: which pose the OTHER joints "
            "were in. A limit found in one pose is environmental — it may be an "
            "obstacle, not the joint's own stop — so an observation is only ever "
            "evidence ABOUT a pose."
        ),
    )
    lm.add_argument(
        "--commit",
        action="store_true",
        default=False,
        help=(
            "KEEP the remedy each measurement points to, instead of restoring everything. "
            "BOUNDED joint -> a PERSISTENT EEPROM encoder re-zero (Ofs, addr 31), which is "
            "then PROVEN by a torque-off hand sweep: the offset reading back proves it was "
            "applied, NOT that the seam moved, so a sweep that finds a discontinuity — or "
            "that you did not actually perform — restores the original and FAILS. "
            "CONTINUOUS joint -> a software-only soft limit, appended to the measured "
            "soft-limit store and read by every mover thereafter (no servo register is "
            "written). UNDETERMINED joint -> nothing. The servo's angle-limit registers "
            "(addrs 9/11) are NEVER written. Requires a human at the arm."
        ),
    )
    lm.add_argument(
        "--sweep-duration",
        type=float,
        default=None,
        metavar="SECONDS",
        help=(
            f"Seconds to hand-sweep each re-zeroed joint under --commit (default "
            f"{rezero.DEFAULT_SWEEP_DURATION:.0f}). The sweep must cover at least "
            f"{rezero.MIN_COVERAGE:.0%} of the joint's travel or the commit is refused — a "
            "short clean sweep proves nothing, because of course it saw no seam."
        ),
    )
    lm.add_argument(
        "--soft-limit-file",
        default=None,
        metavar="PATH",
        help=(
            "The measured soft-limit store to read, and to append to under --commit "
            f"(default {default_soft_limit_path()}; also settable via $ARM101_SOFT_LIMITS). "
            "It is loaded on EVERY run of every motion verb, not just this one: a fence that "
            "only binds when you remember a flag is not a fence."
        ),
    )
    lm.add_argument(
        "--role",
        choices=arm_spec.roles(),
        default="follower",
        help=_ROLE_HELP,
    )
    lm.add_argument(
        "--port",
        default=None,
        help=_PORT_HELP,
    )
    lm.add_argument(
        "--apply",
        action="store_true",
        default=False,
        help="Execute the measurement (non-TTY agent mode; ignored under a TTY).",
    )
    lm.add_argument("--json", action="store_true", help=_JSON_HELP)
    lm.set_defaults(func=cmd_arm_limits, json=False)

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
