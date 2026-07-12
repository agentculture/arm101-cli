"""``arm101 arm`` — arm noun group (overview + read + flex + explore + profile + setup).

Verbs
-----
``arm overview``
    Read-only snapshot of the arm surface: known roles, joints, and the
    per-role id / baud / servo_model / gear_ratio map from arm_spec.  Accepts
    an ignored positional ``target`` and always exits 0 on any path (rubric:
    descriptive verbs must not hard-fail on a bad path).  Supports ``--json``.

``arm read``
    Read every joint's live register state (position/load/speed/voltage/
    temperature/torque) via :func:`~arm101.hardware.arm_read.read_arm`.
    Read-only: it opens a bus and reads, but commands no motion and writes no
    register — so it carries NO consent gate.  Retry-tolerant: a joint whose
    reads keep failing is marked ``failed``/``partial`` while the rest still
    read.  Supports ``--role``, ``--port``, ``--json``.

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
``flex``, ``flex --demo``, ``explore``, and ``profile`` each wrap their whole run
in a :func:`~arm101.hardware.safety.torque_guard` owning the motors they may
energise. (``setup`` does too, one motor at a time, in
:func:`arm101.cli._commands.setup_motors._process_one_motor` — that is where its
per-motor bus is opened.) ``read`` does not: it energises nothing.

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
from arm101.hardware import arm_spec
from arm101.hardware.arm_read import JointReading, is_complete, read_arm
from arm101.hardware.demo import demo_sweep
from arm101.hardware.gentle import gentle_move
from arm101.hardware.motion import compliant_move
from arm101.hardware.motor_catalog import MotorEntry, save_entry
from arm101.hardware.profile import (
    DEFAULT_SPEED_MAX,
    DEFAULT_SPEED_START,
    DEFAULT_SPEED_STEP,
    JointSpeedProfile,
    SpeedTrial,
    profile_joint,
    speed_ladder,
)
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

#: Consent verb label for ``arm flex`` (hoisted to avoid duplicating the literal).
_FLEX_VERB = "arm flex"

#: Consent verb label for ``arm explore`` (hoisted to avoid duplicating the literal).
_EXPLORE_VERB = "arm explore"

#: Consent verb label for ``arm profile`` (hoisted to avoid duplicating the literal).
_PROFILE_VERB = "arm profile"

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
        "verbs": ["overview", "read", "flex", "explore", "profile", "setup"],
        "roles": arm_spec.roles(),
        "motor_map": roles_data,
    }

    if json_mode:
        emit_result(payload, json_mode=True)
        return

    lines = [
        "## arm — arm-level operations",
        "",
        "Verbs: overview, read, flex, explore, profile, setup",
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
        "| joint | id | health | position | load | speed | voltage | temperature | torque |",
        "|-------|----|--------|----------|------|-------|---------|-------------|--------|",
    ]
    for r in readings:
        mark = " [OVERLOAD]" if r.overloaded else ""
        lines.append(
            f"| {r.joint} | {r.motor_id} | {r.health} | {_fmt_cell(r.position)}"
            f" | {_fmt_cell(r.load)} | {_fmt_cell(r.speed)} | {_fmt_cell(r.voltage)}"
            f" | {_fmt_cell(r.temperature)} | {_fmt_cell(r.torque)} |{mark}"
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
    min_angle = info["min_angle"]
    max_angle = info["max_angle"]
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
    calibrated ``[min_angle, max_angle]`` seeds the per-joint bounds, and
    *resolution* is the uniform per-joint bucket size.  Reads flow through
    ``bus.read_info`` — a per-joint read failure propagates as a
    :class:`CliError` (never a traceback), matching ``arm read``/``arm flex``.
    """
    ids = arm_spec.joint_ids(role)
    origin_ticks: "list[int]" = []
    bounds: "list[tuple[int, int]]" = []
    for joint in arm_spec.JOINTS:
        info = bus.read_info(ids[joint])  # type: ignore[attr-defined]
        bound_min = int(info["min_angle"])
        bound_max = int(info["max_angle"])
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
    return arm_spec.DEFAULT_CONTACT_THRESHOLDS[joint] if raw is None else int(raw)


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
