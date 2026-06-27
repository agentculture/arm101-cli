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
            operator = resolve_operator()
            created_at = datetime.now(timezone.utc).isoformat()
            plan = build_plan(
                "center-motor",
                port,
                info,
                action,
                operator=operator,
                created_at=created_at,
            )
            plan_path = write_plan_file(plan)
            emit_plan_stdout(plan, plan_path, json_mode=json_mode)
            return

        if mode == "interactive":
            emit_diagnostic(
                f"⚠ This will ENABLE TORQUE and MOVE motor {motor_id} on {port}"
                f" to position {target} (~{deg:.0f} deg). Clear the workspace."
            )
            ans = _prompt("Type 'yes' to proceed, anything else to abort")
            if ans.strip().lower() != "yes":
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
                return

        if mode == "agent":
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
        write_audit(
            build_audit_record(
                verb="center-motor",
                port=port,
                operator=operator,
                consent_mode=mode,
                action=action,
                outcome="pending",
                plan_hash=plan_hash,
            )
        )
        try:
            bus.enable_torque(motor_id, True)
            try:
                bus.write_goal_position(motor_id, target)
            finally:
                # Always relax torque after enabling it (unless asked to hold),
                # even if the goal-position write raised.
                if not keep_torque:
                    bus.enable_torque(motor_id, False)
        except Exception as exc:  # noqa: BLE001 - audit any motion-step failure, then re-raise
            write_audit(
                build_audit_record(
                    verb="center-motor",
                    port=port,
                    operator=operator,
                    consent_mode=mode,
                    action=action,
                    outcome="failed",
                    plan_hash=plan_hash,
                    error=str(exc),
                )
            )
            raise
        torque_relaxed = not keep_torque
        write_audit(
            build_audit_record(
                verb="center-motor",
                port=port,
                operator=operator,
                consent_mode=mode,
                action=action,
                outcome="success",
                plan_hash=plan_hash,
            )
        )

        torque_state = "relaxed" if torque_relaxed else "still enabled"
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
            emit_result(
                f"Centered motor {motor_id} to {target} on {port} (torque {torque_state}).",
                json_mode=False,
            )
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
