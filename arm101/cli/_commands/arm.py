"""``arm101 arm`` — arm-level noun group (overview + read + flex + explore + setup <role>).

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

``arm setup <role>``
    Drive the existing setup-motors gated three-mode-consent walk (dry_run /
    interactive / agent — see :mod:`arm101.cli._consent`) for the given role
    (follower|leader).  All ids, baud, servo_model, and gear_ratio come from
    :mod:`arm101.hardware.arm_spec` — zero numbers typed by the operator.
    After each motor write the catalog entry is saved via
    :func:`~arm101.hardware.motor_catalog.save_entry` with the role-correct
    label (``F{id}`` / ``L{id}``).  Dry-run mode writes nothing to the catalog.

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
from pathlib import Path

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

#: Default gentle contact-load threshold when ``--threshold`` is not supplied.
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
        "verbs": ["overview", "read", "flex", "explore", "setup"],
        "roles": arm_spec.roles(),
        "motor_map": roles_data,
    }

    if json_mode:
        emit_result(payload, json_mode=True)
        return

    lines = [
        "## arm — arm-level operations",
        "",
        "Verbs: overview, read, flex, explore, setup",
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
    port_display = port or "(auto-detect at apply)"
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
    bus = _open_bus(port)
    try:
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


def _emit_explore_plan(
    role: str,
    map_path: Path,
    log_path: Path,
    threshold: int,
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
        "port": port or "(auto-detect at apply)",
        "map_path": str(map_path),
        "log_path": str(log_path),
        "threshold": threshold,
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
    # Explicit None checks, NOT `or`: an explicit falsy override (e.g.
    # ``--threshold 0``) must not silently collapse back to the default.
    raw_threshold = getattr(args, "threshold", None)
    threshold: int = _DEFAULT_THRESHOLD if raw_threshold is None else int(raw_threshold)
    raw_resolution = getattr(args, "resolution", None)
    resolution: int = _DEFAULT_RESOLUTION if raw_resolution is None else int(raw_resolution)
    raw_max_moves = getattr(args, "max_moves", None)

    map_path, log_path = _explore_paths(getattr(args, "map", None), role)

    mode = resolve_consent(args, verb=_EXPLORE_VERB, require_plan_hash=False)

    # --- dry_run: plan only, zero motion, zero bus access ---
    if mode == "dry_run":
        _emit_explore_plan(
            role,
            map_path,
            log_path,
            threshold,
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
        spec = _build_grid_spec(bus, role, resolution)
        budget = Budget() if raw_max_moves is None else Budget(max_moves=int(raw_max_moves))
        result = engine.explore(
            bus,
            spec,
            log_path=log_path,
            map_path=map_path,
            threshold=threshold,
            budget=budget,
            temperatures=_make_temperature_provider(bus, role),
        )
    finally:
        bus.close()

    _emit_explore_result(role, port, result, json_mode=json_mode)


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
        help="Arm role: follower or leader (default: follower).",
    )
    rd.add_argument(
        "--port",
        default=None,
        help="Serial port (default: auto-detect the first candidate port).",
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
        help="Arm role: follower or leader (default: follower).",
    )
    fx.add_argument(
        "--port",
        default=None,
        help="Serial port (default: auto-detect the first candidate port).",
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
        help="Arm role: follower or leader (default: follower).",
    )
    ex.add_argument(
        "--port",
        default=None,
        help="Serial port (default: auto-detect the first candidate port).",
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
        help="Contact-load threshold handed to each gentle move (default 250).",
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
