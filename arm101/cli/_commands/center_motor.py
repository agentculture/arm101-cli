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

from arm101.cli._commands.calibrate_motor import (
    _detect_one_motor,
    _prompt,
    _show_info,
)
from arm101.cli._output import emit_diagnostic, emit_result

# ---------------------------------------------------------------------------
# Gate helper
# ---------------------------------------------------------------------------


def _confirm(question: str) -> bool:
    """Ask *question* via :func:`_prompt` and return True iff the answer is ``yes``.

    A non-interactive stdin (EOF) causes :func:`_prompt` to raise
    ``CliError(EXIT_ENV_ERROR)`` — the motor is never moved silently.
    """
    answer = _prompt(question)
    return answer.strip().lower() == "yes"


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
        emit_diagnostic(
            f"⚠ This will ENABLE TORQUE and MOVE motor {motor_id} on {port}"
            f" to position {target} (~{deg:.0f}°). Clear the workspace."
        )

        confirmed = _confirm("Type 'yes' to proceed, anything else to abort")
        if not confirmed:
            emit_result("Aborted; motor not moved.", json_mode=False)
            return

        bus.enable_torque(motor_id, True)
        bus.write_goal_position(motor_id, target)
        torque_relaxed = not keep_torque
        if torque_relaxed:
            bus.enable_torque(motor_id, False)

    finally:
        bus.close()

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
    p.add_argument("--json", action="store_true", help="Emit structured JSON.")
    p.set_defaults(func=cmd_center_motor)
