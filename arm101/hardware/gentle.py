"""Load-watch back-off-then-hold compliant move primitive.

Pure helper on top of :mod:`arm101.hardware.bus` and
:mod:`arm101.hardware.motion` — zero third-party dependencies. Where
:func:`arm101.hardware.motion.compliant_move` commands a single gentle move
and trusts the caller to know it is safe, :func:`gentle_move` is for the
"is something in the way?" case: a gripper closing on an unknown object, a
joint sweeping toward a limit it has not been calibrated against, etc. It
steps the goal position in small increments and watches ``present_load``
after every step; if the load spikes past a threshold it treats that as
contact, stops advancing, retreats a bounded number of ticks off the contact
point, and **holds there with torque still enabled** — never a limp release
(no final ``enable_torque(False)``) and never a hard freeze exactly at the
point of contact (which would keep pressing).

Deliberately decoupled from calibration/spec concerns, same as ``motion``:
callers are responsible for sourcing ``min_angle``/``max_angle`` and pass
them in explicitly. This module never imports ``arm_spec`` or reads
calibration files.

On top of the load-watch contact detection above, :func:`gentle_move` layers
two more overload-safety measures around the whole move: it caps the servo's
own RAM ``Torque_Limit`` for the duration of the move (see
:data:`_CONTACT_TORQUE_LIMIT`), and it catches a mid-move ``OverloadError`` —
the servo's *own* overload latch tripping, as distinct from this module's
``present_load``-threshold contact check — recovering gracefully instead of
letting the exception propagate.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from arm101.cli._errors import EXIT_USER_ERROR, CliError
from arm101.hardware.bus import OverloadError, load_magnitude
from arm101.hardware.motion import clamp_goal

if TYPE_CHECKING:
    from arm101.hardware.bus import MotorBus

#: Default present_load threshold (STS3215 Present_Load register units)
#: above which a step is treated as "contact" rather than free motion.
#: Prior hardware sessions on the SO-101 gripper measured free-motion load
#: (gear friction alone, nothing being gripped) in the ~140-208 range, so the
#: default sits comfortably above that band to avoid false-positive contact
#: on ordinary friction while still catching a real obstruction promptly.
#: This is PER-JOINT tunable — pass ``threshold=`` to override for a joint
#: whose free-motion load profile differs (e.g. a heavier limb under gravity
#: load needs a higher floor than the gripper).
_DEFAULT_LOAD_THRESHOLD = 250

#: Default step size (encoder ticks) per incremental goal-position write.
#: Small enough that a contact is caught within roughly one step of the true
#: contact point (bounding overshoot), large enough that a multi-thousand-
#: tick sweep does not take an excessive number of bus round-trips.
_DEFAULT_STEP_TICKS = 25

#: Default back-off distance (encoder ticks) retreated off the contact point
#: once load exceeds the threshold. Bounded deliberately: large enough to
#: meaningfully relieve pressure on the joint/gripper, small enough that the
#: hold position stays close to the actual obstruction rather than yielding
#: the whole approach. Chosen from the same prior hardware sessions as
#: :data:`_DEFAULT_LOAD_THRESHOLD` — a 30-70 tick retreat reliably dropped
#: load back under threshold on the SO-101 gripper joint.
_DEFAULT_BACKOFF_TICKS = 50

#: Gentle default acceleration (STS3215 Acceleration register units,
#: [0, 254]) — mirrors :mod:`arm101.hardware.motion`'s gentle default.
_DEFAULT_ACCELERATION = 20

#: Gentle default goal speed (STS3215 Goal/Running Speed register units,
#: [0, 4095]). Deliberately LOWER than
#: :mod:`arm101.hardware.motion`'s gentle default (400): this is the
#: "is something in the way?" primitive, so a slower approach gives the
#: load-watch loop more/finer samples before a contact can build to a
#: damaging load, and keeps a false-positive-free stop cheap to recover from.
_DEFAULT_SPEED = 150

#: RAM Torque_Limit (STS3215 addr 48, [0, 1000]) applied for the duration of
#: a gentle_move, restored to the motor's pre-move value afterwards (see the
#: read/write/finally dance in :func:`gentle_move`). This makes the SERVO'S
#: OWN overload protection trip at a lower load than its factory rating —
#: a second, hardware-enforced backstop underneath this module's
#: ``present_load``-threshold contact check, in case a step's load spike
#: is missed or the poll lags the actual mechanical load.
#: The exact value is TUNABLE and PARKED AS SPEC RISK v1 — 500 (50% of
#: rated torque) is a conservative first cut for a lightweight gripper
#: joint; a heavier limb joint under gravity load may need a different cap
#: (or none at all). Revisit once real hardware sessions characterise it.
_CONTACT_TORQUE_LIMIT = 500

_REMEDIATION_ALLOW_MOTION_FLAG = (
    "Pass allow_motion=True to confirm the move should actually execute on the bus."
)


def gentle_move(
    bus: "MotorBus",
    motor: int,
    target: int,
    *,
    min_angle: int,
    max_angle: int,
    threshold: int = _DEFAULT_LOAD_THRESHOLD,
    step: int = _DEFAULT_STEP_TICKS,
    backoff: int = _DEFAULT_BACKOFF_TICKS,
    acceleration: int = _DEFAULT_ACCELERATION,
    speed: int = _DEFAULT_SPEED,
    allow_motion: bool = False,
) -> dict[str, object]:
    """Step *motor* toward *target*, watching load, and stop-and-hold on contact.

    This is the single gated entry point for a load-watched move: by default
    (``allow_motion=False``) it raises and performs **no bus writes at all** —
    every caller (CLI verb, agent, or test) must explicitly opt in to motion,
    matching :func:`arm101.hardware.motion.compliant_move`'s contract.

    When ``allow_motion=True``:

    1. The requested *target* is clamped to ``[min_angle, max_angle]`` (see
       :func:`arm101.hardware.motion.clamp_goal`).
    2. The motor's current RAM ``Torque_Limit`` is read and then capped to
       :data:`_CONTACT_TORQUE_LIMIT` for the duration of the move — a
       hardware-enforced backstop underneath the ``present_load`` check
       below. This is restored to its pre-move value in a ``finally``, so it
       is undone whether the move finishes cleanly, contacts, or overloads.
    3. Compliant setup happens once: ``bus.write_acceleration(motor,
       acceleration)``, ``bus.write_goal_speed(motor, speed)``,
       ``bus.enable_torque(motor, True)``.
    4. The start position is read (``bus.read_info(motor)["present_position"]``)
       and the goal is advanced from there toward the clamped target in
       ``step``-tick increments, never overshooting the clamped target or the
       ``[min_angle, max_angle]`` bounds.
    5. After **every** ``write_goal_position`` call, ``present_load`` is read
       back. If it exceeds *threshold*, stepping stops immediately: the
       current position is "contact", and the goal is written once more to a
       retreat position *backoff* ticks back along the direction of travel
       (clamped to bounds) — torque stays enabled, so the joint holds there
       rather than going limp or freezing exactly at the contact point.
    6. If the clamped target is reached with no contact, the motor simply
       holds there (torque already on from step 3; no extra write needed).
    7. If any bus call from step 3 onward raises
       :class:`~arm101.hardware.bus.OverloadError` — the servo's OWN
       overload latch tripping (status error bit 5), distinct from the
       ``present_load``-threshold check in step 5 — stepping stops
       immediately, ``bus.clear_overload(motor)`` is called to release
       torque and clear the latch, and the function RETURNS its result dict
       (``overloaded=True``) instead of raising. The Torque_Limit restore in
       step 2 still happens.

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
        Lower bound for this joint (encoder ticks). See
        :func:`arm101.hardware.motion.compliant_move` — this module stays
        decoupled from calibration/spec data and does not read it itself.
    max_angle:
        Upper bound for this joint (encoder ticks). See *min_angle*.
    threshold:
        ``present_load`` value above which a step is treated as contact.
        Defaults to :data:`_DEFAULT_LOAD_THRESHOLD`; tune per joint.
    step:
        Encoder ticks advanced per incremental goal-position write. Defaults
        to :data:`_DEFAULT_STEP_TICKS`.
    backoff:
        Encoder ticks retreated off the contact point once triggered.
        Defaults to :data:`_DEFAULT_BACKOFF_TICKS`.
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
        ``{"motor", "requested_target", "clamped_target", "was_clamped",
        "start_position", "threshold", "step", "backoff_ticks",
        "acceleration", "speed", "contacted", "contact_position",
        "contact_load", "retreat_position", "final_position",
        "overloaded"}``. When ``contacted`` is ``False``,
        ``contact_position``/``contact_load``/``retreat_position`` are
        ``None`` and ``final_position`` equals ``clamped_target``.
        ``overloaded`` is ``False`` on the happy path (contact or not); it
        is ``True`` only when a mid-move ``OverloadError`` was caught and
        recovered from (see step 7 above), in which case every other key is
        filled best-effort from whatever the move observed before the
        overload — ``start_position``/``contact_*`` may still be ``None`` if
        the overload struck before that observation was made.

    Raises
    ------
    CliError(EXIT_USER_ERROR)
        If ``allow_motion`` is not ``True`` — no writes are issued.
    CliError
        Propagated from the underlying ``bus`` writes (e.g.
        ``CliError(EXIT_ENV_ERROR)`` on a comms failure), or from
        :func:`~arm101.hardware.motion.clamp_goal` if ``min_angle >
        max_angle``. A mid-move :class:`~arm101.hardware.bus.OverloadError`
        specifically is NOT raised — it is caught and reported via the
        ``overloaded`` result key instead (see step 7 above).
    """
    if allow_motion is not True:
        raise CliError(
            code=EXIT_USER_ERROR,
            message="motion requires an explicit flag",
            remediation=_REMEDIATION_ALLOW_MOTION_FLAG,
        )

    # `step` drives the progress loop; a non-positive step never advances
    # `current` toward the target and would spin forever while writing to the
    # bus. `backoff` is a retreat distance, so it must be non-negative.
    if step <= 0:
        raise CliError(
            code=EXIT_USER_ERROR,
            message=f"step must be a positive number of ticks, got {step}",
            remediation="Pass step > 0 (e.g. the default 25) so the move can make progress.",
        )
    if backoff < 0:
        raise CliError(
            code=EXIT_USER_ERROR,
            message=f"backoff must be a non-negative number of ticks, got {backoff}",
            remediation="Pass backoff >= 0 (e.g. the default 50).",
        )

    clamped_target, was_clamped = clamp_goal(target, min_angle, max_angle)

    # Cap the servo's own Torque_Limit for the duration of the move (see
    # _CONTACT_TORQUE_LIMIT) — a hardware-enforced backstop underneath the
    # present_load-threshold check below. Read the pre-move value FIRST so
    # the `finally` can restore it regardless of how the move ends (clean,
    # contacted, or overloaded).
    original_torque_limit = bus.read_torque_limit(motor)
    bus.write_torque_limit(motor, _CONTACT_TORQUE_LIMIT)

    start_position: int | None = None
    current: int | None = None
    contacted = False
    contact_position: int | None = None
    contact_load: int | None = None
    retreat_position: int | None = None
    overloaded = False

    try:
        bus.write_acceleration(motor, acceleration)
        bus.write_goal_speed(motor, speed)
        bus.enable_torque(motor, True)

        start_position = bus.read_info(motor)["present_position"]
        current = start_position

        if clamped_target > start_position:
            direction = 1
        elif clamped_target < start_position:
            direction = -1
        else:
            direction = 0

        while direction != 0 and current != clamped_target:
            if direction > 0:
                next_position = min(current + step, clamped_target)
            else:
                next_position = max(current - step, clamped_target)
            next_position, _ = clamp_goal(next_position, min_angle, max_angle)

            bus.write_goal_position(motor, next_position)
            current = next_position
            # STS3215 present_load carries the load DIRECTION in bit 10 (0x400):
            # compare the magnitude, or a load pointing the "negative" way reads
            # as >=1024 and trips a spurious contact on the very first step.
            present_load = load_magnitude(bus.read_info(motor)["present_load"])

            if present_load > threshold:
                contacted = True
                contact_position = next_position
                contact_load = present_load
                retreat_position, _ = clamp_goal(
                    contact_position - direction * backoff, min_angle, max_angle
                )
                bus.write_goal_position(motor, retreat_position)
                current = retreat_position
                break
    except OverloadError:
        # The servo's OWN overload latch tripped — distinct from the
        # present_load-threshold contact check above. Stop advancing,
        # recover (release torque, clearing the latch), and report this via
        # the result dict rather than letting the exception propagate.
        overloaded = True
        bus.clear_overload(motor)
    finally:
        # Restore the pre-move Torque_Limit no matter how the move ended.
        # clear_overload() (above) disarms any latched fault first, so this
        # write lands normally even on the overload path.
        bus.write_torque_limit(motor, original_torque_limit)

    if overloaded:
        final_position = current
    else:
        final_position = retreat_position if contacted else clamped_target

    return {
        "motor": motor,
        "requested_target": target,
        "clamped_target": clamped_target,
        "was_clamped": was_clamped,
        "start_position": start_position,
        "threshold": threshold,
        "step": step,
        "backoff_ticks": backoff,
        "acceleration": acceleration,
        "speed": speed,
        "contacted": contacted,
        "contact_position": contact_position,
        "contact_load": contact_load,
        "retreat_position": retreat_position,
        "final_position": final_position,
        "overloaded": overloaded,
    }
