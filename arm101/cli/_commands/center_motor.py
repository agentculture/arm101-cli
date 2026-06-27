"""``arm101 center-motor`` — drive a single connected STS3215 servo to home position.

Before mounting a horn on a Feetech STS3215 servo, the motor must be parked at
its centre (default encoder tick 2048, mid-range of the 12-bit ``[0, 4095]``
scale, ≈180°) so the horn aligns with the known zero point.  This verb detects
the single connected servo (reusing the calibrate-motor seam so busy or
non-motor ports are never grabbed), shows the operator a read-only snapshot,
then **gates hard** — a typed ``yes`` is required before any bus write occurs.

On confirmation the sequence is:

1. Enable torque (Torque_Enable register 40).
2. Command the goal position (Goal_Position register 42).
3. Relax torque (Torque_Enable → 0) — unless ``--keep-torque`` is given.

This is **commanded motion**.  Running it without a clear workspace or without
the motor secured is a hardware risk.  The gate guarantees that a non-interactive
invocation (e.g. a CI run with no stdin) can never silently move the motor.

Bus injection seam
------------------
:func:`~arm101.cli._commands.calibrate_motor._open_bus` and
:func:`~arm101.cli._commands.calibrate_motor._candidate_ports` are
monkeypatched on ``calibrate_motor`` in tests.  Because
:func:`~arm101.cli._commands.calibrate_motor._detect_one_motor` (imported
here) resolves those names in its own module scope, patching ``calibrate_motor``
is sufficient — no re-export from this module is needed.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone

from arm101.cli._commands.calibrate_motor import (
    _detect_one_motor,
    _prompt,
    _show_info,
)
from arm101.cli._consent import (
    build_audit_record,
    build_plan,
    emit_plan_stdout,
    resolve_consent,
    resolve_operator,
    verify_plan_hash,
    write_audit,
    write_plan_file,
)
from arm101.cli._output import emit_diagnostic, emit_result

# ---------------------------------------------------------------------------
# Command handler
# ---------------------------------------------------------------------------


def _emit_dry_run(port, info, action, *, json_mode: bool) -> None:
    """Build the plan, write the plan file, and emit the dry-run summary."""
    operator = resolve_operator()
    created_at = datetime.now(timezone.utc).isoformat()
    plan = build_plan("center-motor", port, info, action, operator=operator, created_at=created_at)
    plan_path = write_plan_file(plan)
    emit_plan_stdout(plan, plan_path, json_mode=json_mode)


def _confirm_interactive(
    motor_id: int, port: str, target: int, deg: float, *, json_mode: bool
) -> bool:
    """Prompt the human; return True to proceed, False (and emit an abort) otherwise."""
    emit_diagnostic(
        f"⚠ This will ENABLE TORQUE and MOVE motor {motor_id} on {port}"
        f" to position {target} (~{deg:.0f} deg). Clear the workspace."
    )
    ans = _prompt("Type 'yes' to proceed, anything else to abort")
    if ans.strip().lower() == "yes":
        return True
    if json_mode:
        emit_result(
            {
                "aborted": True,
                "motor": motor_id,
                "port": port,
                "position": target,
                "moved": False,
            },
            json_mode=True,
        )
    else:
        emit_result("Aborted; motor not moved.", json_mode=False)
    return False


def _perform_motion(bus, motor_id: int, target: int, *, keep_torque: bool) -> bool:
    """Enable torque, command the goal position, then relax (unless keep_torque).

    Returns whether torque was relaxed.  Raises on any bus failure; if the move
    fails, that primary error is preserved even when the relax in the finally
    also fails.
    """
    bus.enable_torque(motor_id, True)
    move_failed = False
    try:
        bus.write_goal_position(motor_id, target)
    except Exception:  # noqa: BLE001
        # Re-raise after the finally relaxes torque; the flag lets a relax
        # failure below avoid masking this primary move error.
        move_failed = True
        raise
    finally:
        # Always relax torque after enabling it (unless asked to hold), even if
        # the goal-position write raised.
        if not keep_torque:
            try:
                bus.enable_torque(motor_id, False)
            except Exception:  # noqa: BLE001
                # A relax failure must not mask a move failure: if the move
                # already failed that primary error wins; otherwise surface this
                # one (torque may still be engaged — the operator must know).
                if not move_failed:
                    raise
    return not keep_torque


def _emit_motion_result(
    motor_id: int, port: str, target: int, torque_relaxed: bool, *, json_mode: bool
) -> None:
    """Emit the success summary for a completed move."""
    if json_mode:
        emit_result(
            {
                "motor": motor_id,
                "port": port,
                "position": target,
                "torque_relaxed": torque_relaxed,
            },
            json_mode=True,
        )
    else:
        torque_state = "relaxed" if torque_relaxed else "still enabled"
        emit_result(
            f"Centered motor {motor_id} to {target} on {port} (torque {torque_state}).",
            json_mode=False,
        )


def _audit(port, operator, mode, action, outcome, plan_hash, error=None) -> None:
    """Append a center-motor audit record (never raises)."""
    write_audit(
        build_audit_record(
            verb="center-motor",
            port=port,
            operator=operator,
            consent_mode=mode,
            action=action,
            outcome=outcome,
            plan_hash=plan_hash,
            error=error,
        )
    )


def cmd_center_motor(args: argparse.Namespace) -> None:
    """Park the single connected STS3215 at the home position for horn mounting."""
    json_mode = bool(getattr(args, "json", False))
    target = int(getattr(args, "position", 2048))
    keep_torque = bool(getattr(args, "keep_torque", False))

    bus, port, motor_id = _detect_one_motor(args)
    try:
        info = bus.read_info(motor_id)
        _show_info(info, port)

        deg = target * 360.0 / 4096.0
        action = {
            "kind": "goal_position_write",
            "motor_id": motor_id,
            "target_position": target,
            "target_degrees": round(deg, 1),
            "keep_torque": keep_torque,
            "workspace_warning": (
                "ENABLE TORQUE and MOVE the motor. Clear the workspace before proceeding."
            ),
        }

        mode = resolve_consent(args, verb="center-motor", require_plan_hash=True)

        if mode == "dry_run":
            _emit_dry_run(port, info, action, json_mode=json_mode)
            return
        if mode == "interactive":
            if not _confirm_interactive(motor_id, port, target, deg, json_mode=json_mode):
                return
        elif mode == "agent":
            verify_plan_hash(
                getattr(args, "plan_hash", None),
                verb="center-motor",
                port=port,
                action=action,
                info=info,
            )

        # interactive-confirmed OR agent-verified: perform the motion.
        operator = resolve_operator()
        plan_hash = getattr(args, "plan_hash", None)
        _audit(port, operator, mode, action, "pending", plan_hash)
        try:
            torque_relaxed = _perform_motion(bus, motor_id, target, keep_torque=keep_torque)
        except Exception as exc:  # noqa: BLE001
            # Audit any motion-step failure, then re-raise.
            _audit(port, operator, mode, action, "failed", plan_hash, error=str(exc))
            raise
        _audit(port, operator, mode, action, "success", plan_hash)
        _emit_motion_result(motor_id, port, target, torque_relaxed, json_mode=json_mode)
    finally:
        bus.close()


# ---------------------------------------------------------------------------
# Registration
# ---------------------------------------------------------------------------


def register(sub: "argparse._SubParsersAction") -> None:
    """Register the ``center-motor`` subcommand on *sub*."""
    p = sub.add_parser(
        "center-motor",
        help=(
            "Drive a single connected motor to the home position (default 2048) "
            "for horn mounting. Gated: requires typed confirmation before moving."
        ),
    )
    p.add_argument(
        "--port",
        default=None,
        help="Serial port of the motor (default: auto-detect, skipping busy/non-motor ports).",
    )
    p.add_argument(
        "--position",
        type=int,
        default=2048,
        help="Target encoder tick [0–4095] (default: 2048 = mid-range / home).",
    )
    p.add_argument(
        "--keep-torque",
        action="store_true",
        default=False,
        help="Leave torque enabled after centering (default: relax after move).",
    )
    p.add_argument(
        "--apply",
        action="store_true",
        default=False,
        help="Execute the move (non-TTY agent mode; requires --plan-hash; ignored under a TTY).",
    )
    p.add_argument(
        "--plan-hash",
        default=None,
        metavar="HASH",
        help="sha256 plan hash from the plan file generated by a prior dry-run.",
    )
    p.add_argument("--json", action="store_true", help="Emit structured JSON.")
    p.set_defaults(func=cmd_center_motor)
