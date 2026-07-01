"""Bounded, flag-gated, compliant per-joint move primitive.

Pure helpers on top of :mod:`arm101.hardware.bus` — zero third-party
dependencies. Deliberately decoupled from calibration/spec concerns: callers
(the CLI layer) are responsible for sourcing ``min_angle``/``max_angle`` (e.g.
from ``bus.read_info(motor)``) and pass them in explicitly. This module never
imports ``arm_spec`` or reads calibration files.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from arm101.cli._errors import EXIT_USER_ERROR, CliError

if TYPE_CHECKING:
    from arm101.hardware.bus import MotorBus

#: Gentle default acceleration (STS3215 Acceleration register units, [0, 254]).
#: Low values ramp the motor up to speed slowly instead of snapping to it —
#: chosen from prior hardware sessions where ~15-20 felt gentle on an SO-101
#: joint without being sluggish.
_DEFAULT_ACCELERATION = 20

#: Gentle default goal speed (STS3215 Goal/Running Speed register units,
#: [0, 4095]). Moderate rather than minimal so a move still completes in a
#: reasonable time; chosen from prior hardware sessions where ~300-500 felt
#: gentle without being sluggish.
_DEFAULT_SPEED = 400

_REMEDIATION_ALLOW_MOTION_FLAG = (
    "Pass allow_motion=True to confirm the move should actually execute on the bus."
)


def clamp_goal(target: int, lo: int, hi: int) -> tuple[int, bool]:
    """Clamp *target* into ``[lo, hi]``.

    Parameters
    ----------
    target:
        Requested value (encoder ticks).
    lo:
        Lower bound (inclusive).
    hi:
        Upper bound (inclusive).

    Returns
    -------
    tuple[int, bool]
        ``(clamped, was_clamped)`` — *clamped* is always within ``[lo, hi]``;
        *was_clamped* is ``True`` iff *target* fell outside that range.

    Raises
    ------
    CliError(EXIT_USER_ERROR)
        If ``lo > hi`` — an inverted range is treated as a programming error
        rather than silently swapped, since a caller-supplied inverted range
        almost always indicates a bug upstream (e.g. min/max read backwards).
    """
    if lo > hi:
        raise CliError(
            code=EXIT_USER_ERROR,
            message=f"Invalid clamp range: lo={lo} is greater than hi={hi}.",
            remediation="Pass lo <= hi (check min_angle/max_angle were not swapped).",
        )

    if target < lo:
        return lo, True
    if target > hi:
        return hi, True
    return target, False


def compliant_move(
    bus: "MotorBus",
    motor: int,
    target: int,
    *,
    min_angle: int,
    max_angle: int,
    acceleration: int = _DEFAULT_ACCELERATION,
    speed: int = _DEFAULT_SPEED,
    allow_motion: bool = False,
) -> dict[str, object]:
    """Move *motor* to *target*, gently and within bounds, but only if asked.

    This is the single gated entry point for commanding motion: by default
    (``allow_motion=False``) it raises and performs **no bus writes at all** —
    every caller (CLI verb, agent, or test) must explicitly opt in to motion.

    When ``allow_motion=True``, the requested *target* is clamped to
    ``[min_angle, max_angle]`` (see :func:`clamp_goal`) and then the motor is
    set up to move there compliantly, in this exact order:

    1. ``bus.write_acceleration(motor, acceleration)`` — gentle ramp-up.
    2. ``bus.write_goal_speed(motor, speed)`` — moderate travel speed.
    3. ``bus.enable_torque(motor, True)`` — torque must be on to move.
    4. ``bus.write_goal_position(motor, clamped_target)`` — commands the move.

    Parameters
    ----------
    bus:
        An open :class:`~arm101.hardware.bus.MotorBus` (real or fake).
    motor:
        Motor ID (1-indexed, matching the Feetech servo ID).
    target:
        Requested goal position (encoder ticks); may be outside
        ``[min_angle, max_angle]``, in which case it is clamped.
    min_angle:
        Lower bound for this joint (encoder ticks). Callers typically source
        this from ``bus.read_info(motor)["min_angle"]`` — this module does
        not read it itself, to stay decoupled from calibration/spec data.
    max_angle:
        Upper bound for this joint (encoder ticks). See *min_angle*.
    acceleration:
        STS3215 Acceleration register value, ``[0, 254]``. Defaults to a
        gentle :data:`_DEFAULT_ACCELERATION`.
    speed:
        STS3215 Goal/Running Speed register value, ``[0, 4095]``. Defaults to
        a gentle :data:`_DEFAULT_SPEED`.
    allow_motion:
        Must be ``True`` for any bus write to happen at all. This is the
        flag gate: callers (CLI verbs) must surface an explicit
        ``--allow-motion``-style flag rather than defaulting motion to on.

    Returns
    -------
    dict[str, object]
        ``{"motor": int, "requested_target": int, "clamped_target": int,
        "was_clamped": bool, "acceleration": int, "speed": int}``

    Raises
    ------
    CliError(EXIT_USER_ERROR)
        If ``allow_motion`` is not ``True`` — no writes are issued.
    CliError
        Propagated from the underlying ``bus`` writes (e.g.
        ``CliError(EXIT_ENV_ERROR)`` on a comms failure).
    """
    if allow_motion is not True:
        raise CliError(
            code=EXIT_USER_ERROR,
            message="motion requires an explicit flag",
            remediation=_REMEDIATION_ALLOW_MOTION_FLAG,
        )

    clamped_target, was_clamped = clamp_goal(target, min_angle, max_angle)

    bus.write_acceleration(motor, acceleration)
    bus.write_goal_speed(motor, speed)
    bus.enable_torque(motor, True)
    bus.write_goal_position(motor, clamped_target)

    return {
        "motor": motor,
        "requested_target": target,
        "clamped_target": clamped_target,
        "was_clamped": was_clamped,
        "acceleration": acceleration,
        "speed": speed,
    }
