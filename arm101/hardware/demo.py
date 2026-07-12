"""Scripted safe-exploration sweep — layers on the gentle compliant primitive.

Pure helper on top of :mod:`arm101.hardware.motion` and
:mod:`arm101.hardware.gentle` — zero third-party dependencies. Where
:func:`arm101.hardware.gentle.gentle_move` drives a SINGLE joint to a SINGLE
target compliantly, :func:`demo_sweep` is a small choreography on top: sweep
EVERY joint in a caller-supplied set through a conservative, safe sub-range
of its own calibrated ``[min_angle, max_angle]``, using ``gentle_move`` for
every individual move so contact detection / back-off-and-hold still apply
per step. It exists to let an operator or agent run a scripted "does the arm
move freely?" demo without ever risking a joint slamming into its mechanical
limit or pressing through an obstruction.

Deliberately decoupled from calibration/spec concerns, same as ``motion`` and
``gentle``: callers are responsible for sourcing the joint-name -> motor-id
mapping (e.g. from ``arm_spec``) and pass it in explicitly. This module never
imports ``arm_spec`` or reads calibration files, and it never imports any CLI
module — it is a hardware-layer primitive only.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from arm101.cli._errors import EXIT_USER_ERROR, CliError
from arm101.hardware.bus import OverloadError
from arm101.hardware.gentle import _DEFAULT_LOAD_THRESHOLD, gentle_move
from arm101.hardware.motion import clamp_goal

if TYPE_CHECKING:
    from arm101.hardware.bus import MotorBus

#: Default fraction of a joint's full calibrated range
#: (``max_angle - min_angle``) that the demo sweep explores, centred on the
#: joint's CURRENT position. Deliberately conservative (within the 0.3-0.5
#: band): large enough that the sweep is a visible, meaningful "the arm
#: moves" demonstration, small enough that — even centred near a limit — the
#: clamped sub-range sweep stays comfortably clear of slamming a joint
#: straight into its mechanical end-stop. ``clamp_goal`` is still applied to
#: every computed target as a hard backstop regardless of this choice.
_DEFAULT_FRACTION = 0.4

_REMEDIATION_ALLOW_MOTION_FLAG = (
    "Pass allow_motion=True to confirm the demo sweep should actually execute on the bus."
)


def _sweep_targets(
    bus: "MotorBus",
    motor: int,
    planned_targets: "list[int]",
    *,
    min_angle: int,
    max_angle: int,
    threshold: int,
    start_position: int,
    gentle_kwargs: "dict[str, object]",
) -> "tuple[list[int], list[dict[str, object]], bool, bool, int]":
    """Drive one joint through *planned_targets* gently, stopping on overload/contact.

    Runs :func:`gentle_move` for each target in order, appending to the
    attempt log; the first move that reports ``overloaded`` (checked first, so
    an overload always wins the stop reason) or ``contacted`` stops the loop —
    the remaining target is never attempted. Factored out of
    :func:`demo_sweep` so that entry point stays under the cognitive-complexity
    budget; the semantics are identical to the inline loop it replaces.

    Returns ``(targets_attempted, moves, contacted, overloaded, final_position)``.
    """
    targets_attempted: list[int] = []
    moves: list[dict[str, object]] = []
    contacted = False
    overloaded = False
    final_position = start_position

    for target in planned_targets:
        move_result = gentle_move(
            bus,
            motor,
            target,
            min_angle=min_angle,
            max_angle=max_angle,
            threshold=threshold,
            allow_motion=True,
            **gentle_kwargs,
        )
        targets_attempted.append(target)
        moves.append(move_result)
        # Keep the last position we actually MEASURED. gentle_move reports None
        # only when it never managed to read the joint at all (an overload latched
        # before its first read), and this report's contract says int — so hold on
        # to the last real value rather than clobbering it with a None.
        moved_to = move_result["final_position"]
        if moved_to is not None:
            final_position = moved_to
        if move_result["overloaded"]:
            # The servo's OWN overload latch tripped mid-move; gentle_move
            # already caught it, recovered (cleared the latch), and returned
            # normally. Checked BEFORE the contact check below so an overload
            # always wins the abort reason.
            overloaded = True
            break
        if move_result["contacted"]:
            contacted = True
            break

    return targets_attempted, moves, contacted, overloaded, final_position


def demo_sweep(
    bus: "MotorBus",
    joints: "dict[str, int]",
    *,
    fraction: float = _DEFAULT_FRACTION,
    allow_motion: bool = False,
    threshold: int = _DEFAULT_LOAD_THRESHOLD,
    **gentle_kwargs: object,
) -> dict[str, object]:
    """Sweep every joint in *joints* through a safe sub-range, gently.

    This is the single gated entry point for the scripted demo: by default
    (``allow_motion=False``) it raises and performs **no bus calls at all** —
    not even a read — matching
    :func:`arm101.hardware.gentle.gentle_move`'s and
    :func:`arm101.hardware.motion.compliant_move`'s contract. Every caller
    (CLI verb, agent, or test) must explicitly opt in to motion.

    When ``allow_motion=True``, joints are swept **in the order given by
    ``joints.items()``** — i.e. dict insertion order, NOT sorted by motor id.
    This hands sweep order to the caller (e.g. base-to-tip, or whatever is
    physically sensible for the arm), rather than this module silently
    re-ordering it.

    For each joint, in that order:

    1. ``info = bus.read_info(motor)`` supplies ``min_angle``, ``max_angle``,
       and the joint's current ``present_position``.
    2. A safe sub-range is computed CENTRED ON THE CURRENT POSITION (not the
       calibrated midpoint, so the sweep explores from wherever the joint
       actually is): ``half_span = fraction * (max_angle - min_angle) / 2``,
       ``low = present_position - half_span``,
       ``high = present_position + half_span``. Both are rounded to the
       nearest tick and then passed through
       :func:`arm101.hardware.motion.clamp_goal` against
       ``[min_angle, max_angle]`` — the hard backstop that guarantees a
       target NEVER exceeds the joint's calibrated bounds, regardless of
       *fraction* or how close ``present_position`` sits to a limit.
    3. ``gentle_move`` drives the joint to ``low``, then to ``high``, in that
       order, with ``allow_motion=True`` forced internally (the joint-level
       gate was already satisfied by the call into this function) and
       *threshold* plus any ``gentle_kwargs`` (e.g. ``step``, ``backoff``,
       ``acceleration``, ``speed``) forwarded unchanged.
    4. If either ``gentle_move`` call reports ``overloaded=True`` — the
       servo's OWN overload latch tripped mid-move; ``gentle_move`` already
       caught the underlying
       :class:`~arm101.hardware.bus.OverloadError`, called
       ``bus.clear_overload``, and returned normally instead of raising (see
       :func:`arm101.hardware.gentle.gentle_move`) — the sweep for THIS joint
       stops immediately (the second target, if any, is never attempted) and
       the WHOLE multi-joint sweep aborts cleanly: no joint later in
       ``joints`` is touched at all — no ``bus.read_info`` and no writes.
       Checked BEFORE the contact check below, so an overload always wins the
       abort reason for that call.
    5. Otherwise, if either ``gentle_move`` call reports ``contacted=True``,
       the sweep for THIS joint stops immediately (the second target, if
       any, is never attempted) and the WHOLE multi-joint sweep aborts
       cleanly: no joint later in ``joints`` is touched at all — no
       ``bus.read_info`` and no writes. Contact is treated as an expected,
       safe outcome: this never raises, it is reported.

    Both overload and contact are treated as expected, safe outcomes: this
    function never raises on either, it reports them.

    Parameters
    ----------
    bus:
        An open :class:`~arm101.hardware.bus.MotorBus` (real or fake).
    joints:
        Mapping of joint-name -> motor-id (1-indexed Feetech servo ID).
        Decoupled from ``arm_spec`` on purpose — the caller supplies it.
        Swept in ``joints.items()`` order (see above).
    fraction:
        Fraction of each joint's full ``[min_angle, max_angle]`` range to
        explore, centred on its current position. Defaults to
        :data:`_DEFAULT_FRACTION`.
    allow_motion:
        Must be ``True`` for any bus call to happen at all. This is the
        flag gate: callers (CLI verbs) must surface an explicit
        ``--allow-motion``-style flag rather than defaulting motion to on.
    threshold:
        ``present_load`` value above which a ``gentle_move`` step is treated
        as contact. Forwarded to every ``gentle_move`` call. Defaults to
        :data:`arm101.hardware.gentle._DEFAULT_LOAD_THRESHOLD`.
    **gentle_kwargs:
        Additional keyword arguments forwarded to every ``gentle_move`` call
        (e.g. ``step``, ``backoff``, ``acceleration``, ``speed``). Must NOT
        include ``min_angle``, ``max_angle``, ``threshold``, or
        ``allow_motion`` — those are computed/managed by this function.

    Returns
    -------
    dict[str, object]
        ``{"fraction", "threshold", "joints", "aborted_on_contact",
        "aborted_joint", "aborted_on_overload", "overloaded_joint"}``.

        ``joints`` is a dict keyed by joint-name, present ONLY for joints
        that were actually visited (a joint after the one that contacted or
        overloaded is absent, never touched). Each value is::

            {"motor": int, "min_angle": int, "max_angle": int,
             "start_position": int, "planned_targets": [int, int],
             "targets_attempted": list[int], "moves": list[dict],
             "contacted": bool, "overloaded": bool, "final_position": int}

        ``moves`` is the list of raw :func:`gentle_move` result dicts, one
        per attempted target, in attempt order.

        ``aborted_on_contact`` is ``True`` iff any joint contacted (and the
        sweep stopped there); ``aborted_joint`` is that joint's name, or
        ``None`` if the full sweep completed without contact.

        ``aborted_on_overload`` is ``True`` iff any joint's ``gentle_move``
        call reported ``overloaded=True`` (and the sweep stopped there);
        ``overloaded_joint`` is that joint's name, or ``None`` if the full
        sweep completed without an overload. Checked before the contact
        outcome for each call (see step 4 above), so a call that somehow
        reports both never also sets ``aborted_on_contact``/``aborted_joint``.

    Raises
    ------
    CliError(EXIT_USER_ERROR)
        If ``allow_motion`` is not ``True`` — no bus calls are issued.
    CliError
        Propagated from the underlying ``gentle_move``/``bus`` calls (e.g.
        ``CliError(EXIT_ENV_ERROR)`` on a comms failure).
    """
    if allow_motion is not True:
        raise CliError(
            code=EXIT_USER_ERROR,
            message="motion requires an explicit flag",
            remediation=_REMEDIATION_ALLOW_MOTION_FLAG,
        )

    joint_reports: dict[str, dict[str, object]] = {}
    aborted_on_contact = False
    aborted_joint: str | None = None
    aborted_on_overload = False
    overloaded_joint: str | None = None

    for joint_name, motor in joints.items():
        try:
            info = bus.read_info(motor)
        except OverloadError:
            # The joint is ALREADY latched in overload before we could read it
            # (e.g. a prior op left it faulted). Treat it exactly like a
            # mid-move overload: recover (clear the latch), record a minimal
            # report, mark the abort, and stop the sweep cleanly — never raise.
            bus.clear_overload(motor)
            joint_reports[joint_name] = {"motor": motor, "overloaded": True}
            aborted_on_overload = True
            overloaded_joint = joint_name
            break
        min_angle = info["min_angle"]
        max_angle = info["max_angle"]
        start_position = info["present_position"]

        half_span = fraction * (max_angle - min_angle) / 2
        low, _ = clamp_goal(round(start_position - half_span), min_angle, max_angle)
        high, _ = clamp_goal(round(start_position + half_span), min_angle, max_angle)
        planned_targets = [low, high]

        targets_attempted, moves, contacted, overloaded, final_position = _sweep_targets(
            bus,
            motor,
            planned_targets,
            min_angle=min_angle,
            max_angle=max_angle,
            threshold=threshold,
            start_position=start_position,
            gentle_kwargs=gentle_kwargs,
        )

        joint_reports[joint_name] = {
            "motor": motor,
            "min_angle": min_angle,
            "max_angle": max_angle,
            "start_position": start_position,
            "planned_targets": planned_targets,
            "targets_attempted": targets_attempted,
            "moves": moves,
            "contacted": contacted,
            "overloaded": overloaded,
            "final_position": final_position,
        }

        if overloaded:
            aborted_on_overload = True
            overloaded_joint = joint_name
            break
        if contacted:
            aborted_on_contact = True
            aborted_joint = joint_name
            break

    return {
        "fraction": fraction,
        "threshold": threshold,
        "joints": joint_reports,
        "aborted_on_contact": aborted_on_contact,
        "aborted_joint": aborted_joint,
        "aborted_on_overload": aborted_on_overload,
        "overloaded_joint": overloaded_joint,
    }
