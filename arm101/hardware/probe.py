"""arm101.hardware.probe — drive ONE joint outward until it learns something.

The probe creeps a joint away from wherever it currently is, one short
``gentle_move`` at a time, through a :class:`~arm101.hardware.rolling_frame.RollingFrame`
that keeps the encoder seam half a turn ahead of it — and it stops the moment it can
say *why* it stopped. What comes back is one
:class:`~arm101.hardware.limits.EndObservation`: the evidence for one END of one
joint's travel, in one pose.

THE PROBLEM: a wall and a weak arm look IDENTICAL at the moment of the stop
==========================================================================

``present_load`` **saturates at ``Torque_Limit``**, and ``gentle_move`` caps that at
:data:`~arm101.hardware.gentle.CONTACT_LOAD_CEILING` (500) for the duration of every
move. So when a joint stops:

* a joint pressed against a **mechanical limit** reads: load 500, not advancing; and
* a joint that has simply **run out of torque** reads: load 500, not advancing.

Same load. Same stall. Same ``contacted=True`` out of ``gentle_move``, which cannot
tell them apart and does not try to. ``shoulder_lift`` carries the whole arm, and a
torque-limited stall recorded as a mechanical limit would write a permanent lie into
``arm_spec`` — a wall in a place the arm can, in another pose, walk straight through.
``test_the_stop_ITSELF_says_nothing_the_two_cases_agree_on_every_bit_of_it`` asserts
this indistinguishability rather than describing it.

THE DISCRIMINATOR: the APPROACH, not the stop
=============================================

What differs is not where the joint ended up but **how it got there**, and that whole
history is available: ``gentle_move`` hands every ``(present_position, present_load)``
sample of the approach to its :data:`~arm101.hardware.gentle.TravelObserver` seam — the
same stream its own :class:`~arm101.hardware.gentle._StallDetector` is fed, so what is
ruled on here is what the shipped detector actually saw, not a re-implementation of it.

Two distances are read out of that stream, both in ticks, both walking BACKWARDS from
the stop (see :meth:`_Approach.runs`):

``loaded_run``
    How far the joint travelled, immediately before it stopped, while pushing **past
    its own contact threshold**. The width of the loaded zone in front of the stop.

``free_run``
    How far it travelled, immediately before *that*, at or **below** the threshold —
    i.e. moving freely. The evidence that it was ever moving freely at all.

And the physics separates them by an order of magnitude:

* **A WALL is a POSITION constraint.** The joint moves freely right up to it, then the
  load spikes over the small give in the gears and links and it stops. ``gentle``
  measured that give from the other side: *a 30-70 tick retreat reliably drops the
  load back under threshold* (:data:`~arm101.hardware.gentle._DEFAULT_BACKOFF_TICKS`).
  So a real contact's loaded zone is **tens of ticks**, with free cruising behind it.
* **TORQUE-EXHAUSTION is a TORQUE constraint.** The required torque rises with the
  angle (the moment arm growing), crosses the joint's contact threshold, and keeps
  rising until it crosses the cap. The joint is therefore **already pushing past its
  threshold while it is still advancing**, and it stays there for the whole of that
  arc — which for a gravity-loaded joint whose threshold sits at 250-280 against a
  500 ceiling is **hundreds of ticks** (the load must climb ~45% of the cap, and near
  the worst angle it climbs slowly indeed). It does not stop dead: it *creeps*, load
  pinned near saturation, which is exactly why the stall rule needs 8 consecutive
  samples to call it.

Hence the rule, and it has two halves because a wall has to show BOTH of them::

    WALL  iff  loaded_run <= compliance  AND  free_run >= free_run_needed
    everything else -> TORQUE_LIMITED

:func:`wall_compliance` sets the cutoff at **twice** ``gentle``'s ``backoff`` — the top
of the measured 30-70 tick give, doubled for noise, and still well below a gravity
climb's loaded arc. It is a *hardware* number, derived from ``gentle`` so the two cannot
drift, and it is a parameter so a bench session can retune it without a code change.

HOW MUCH MARGIN THAT ACTUALLY LEAVES — and it is thinner than it looks
=====================================================================

Be precise about this, because the whole verdict rests on it. Let *slope* be how fast a
joint's load climbs, in load units per tick.

* **A real gravity joint's slope is bounded.** It stalls where its gravity torque meets
  the 500 cap, so at that angle ``dtau/dtheta = sqrt(tau_horizontal**2 - 500**2)``. And
  ``tau_horizontal`` cannot exceed the load register's 1000-unit full scale for a joint
  that can hold itself out horizontally at all — which this arm demonstrably can. With
  652 ticks to the radian that caps the slope at ``sqrt(1000**2 - 500**2) / 652``
  = **1.33 units/tick**, and a loaded run of ``(500 - threshold) / slope`` >= ~165 ticks.
* **The cutoff bites at a slope of about 1.8** (measured against the model in
  ``tests/test_probe.py``: 1.6 still reads TORQUE_LIMITED, 2.0 reads WALL).

So the margin is roughly **1.35x** — real, but THIN, and it narrows as a joint's contact
threshold climbs toward the ceiling, because ``500 - threshold`` is the numerator of the
loaded run. ``wrist_roll``'s threshold of 400 halves it, and gets away with that only
because a roll axis carries no gravity torque to begin with.

``test_a_CRUSHING_load_can_still_fool_the_probe_and_this_is_exactly_where`` pins the
cliff rather than hiding it: a joint loaded far past anything this arm can be is recorded
as WALL, and it is not one. **The first hardware session must record ``loaded_run_ticks``
per joint per end and set *compliance* from that data, rather than from this reasoning.**
Until then the knob exists, and tightening it costs nothing but an under-claim.

THE TIE-BREAK, WHICH IS NOT NEGOTIABLE
======================================

**When the evidence does not clearly support a WALL, the verdict is TORQUE_LIMITED.**
Never the reverse. A false TORQUE_LIMITED records a *lower bound* — the arm under-claims
its reach, and another pose can still widen it
(:func:`~arm101.hardware.limits.merge_end_observations`).
A false WALL records a limit that is not there, permanently, and no number of later poses
can dislodge it. So every gap in the evidence falls the same way:

* the joint was **already jammed** when the probe started (no ``free_run``) — it may be
  resting on its own end-stop, or it may be too weak to lift itself out of this pose;
  nobody has seen it move, so nobody may call it a wall;
* the servo's **own overload latch** (error=32) fired — the hardware gave up before the
  software rule saw anything;
* the wall is so **compliant** the loaded zone looks like a torque climb.

Each of those is a real wall some of the time, and the probe declines to say so. That
cost is deliberate, it is tested
(``test_a_wall_so_SOFT_the_probe_cannot_tell_it_from_a_weak_arm_is_not_called_a_wall``),
and it is the right way round to be wrong.

WHAT THE PROBE DOES *NOT* DO
============================

The decisive experiment is to **raise the torque cap and see if the joint moves**: a
wall does not budge, an exhausted arm does. The probe does not do it. Every overload-
safety measure in this package (PR #24, the error=32 work) exists to keep the cap LOW,
and deliberately driving a joint harder into an unknown obstruction to disambiguate it
is exactly the incident those measures were written to prevent. The approach profile is
an *inference*; that would be a *measurement*. It is named here so that nobody has to
rediscover why it was not taken.

Zero third-party imports, and **no joint table**: like ``gentle``, ``motion`` and
``profile``, this module is handed a threshold and a joint name and branches on neither.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Dict, List, Optional, Tuple

from arm101.cli._errors import EXIT_USER_ERROR, CliError

# The privates below are imported rather than re-typed on purpose: they are the SAME
# numbers `gentle_move` will be driven with, and a probe reasoning about a contact's
# compliant zone from its own copy of `backoff` would be reasoning about a move that
# is not the one it made. (Same argument as `classify`'s import of `_ARC_MARGIN_TICKS`.)
from arm101.hardware.gentle import (
    _DEFAULT_ACCELERATION,
    _DEFAULT_BACKOFF_TICKS,
    _DEFAULT_SPEED,
    _MIN_TICKS_PER_SECOND,
    CONTACT_LOAD_CEILING,
    DEFAULT_LOAD_WATCH,
    LoadWatch,
    gentle_move,
)
from arm101.hardware.limits import (
    ENCODER_TICKS,
    EndObservation,
    LimitVerdict,
    TravelEnd,
    signed_delta,
)
from arm101.hardware.rolling_frame import MAX_HEADROOM, RollingFrame
from arm101.hardware.ticks import TICK_MAX, TICK_MIN, raw_from_reported

if TYPE_CHECKING:  # pragma: no cover - typing only; never imported at runtime
    from arm101.hardware.bus import MotorBus

__all__ = [
    "DEFAULT_CREEP_TICKS",
    "ProbeOutcome",
    "free_run_needed",
    "probe_end",
    "wall_compliance",
]


# ---------------------------------------------------------------------------
# The knobs, and where each of them comes from
# ---------------------------------------------------------------------------

#: Ticks the probe drives per creep step — the length of ONE ``gentle_move``.
#:
#: A compromise, and both ends of it are real. Every move pays the servo's ~95-127 ms
#: motion-onset dead window, so a probe made of tiny moves spends most of its life
#: waiting for a shaft that has not started turning: at the measured 150-500 ticks/s a
#: 300-tick move takes 0.6-2.0 s, which puts onset at 5-20% overhead instead of 50%.
#: Going much LARGER buys little (onset is already amortised) and costs the probe its
#: chance to look around: the frame is only re-centred, and the displacement only
#: folded in, BETWEEN moves. It also sits comfortably under
#: :data:`~arm101.hardware.rolling_frame.MAX_HEADROOM`, so no step ever needs a frame
#: that could not exist.
DEFAULT_CREEP_TICKS: int = 300

#: Multiplier on ``gentle``'s ``backoff`` that gives :func:`wall_compliance`.
#:
#: ``backoff``'s own docstring records the measurement: a **30-70 tick** retreat
#: reliably drops a contact's load back under threshold on this arm. That band IS the
#: give in the gearbox and links — the arc over which a contact goes from free to
#: saturated — and the default ``backoff`` (50) sits in the middle of it. Doubling
#: reaches past the top of the band (70) with room for sampling noise, while staying an
#: order of magnitude below the hundreds of ticks a gravity-loaded joint spends above
#: its threshold on the way to running out. The factor is the one *judgement* in this
#: module; it is a parameter of :func:`probe_end` precisely so a hardware session can
#: correct it without touching code.
_COMPLIANCE_FACTOR: int = 2

#: How many creep steps' worth of moves the probe will make before it gives up on a
#: joint that is not making progress. A BACKSTOP, not a budget: a probe that is
#: actually travelling needs about ``max_travel / step`` moves, so a factor of four
#: plus slack is never reached by a working arm. It exists for the pathological bus —
#: one that reports the joint arriving at goals a shaft that never turns cannot have
#: reached — which would otherwise be commanded forever. (The same role, and the same
#: reasoning, as ``gentle``'s ``_MAX_POLLS_PER_MOVE``.)
_MOVE_BUDGET_FACTOR: int = 4
_MOVE_BUDGET_SLACK: int = 8

_REMEDIATION_ALLOW_MOTION = (
    "Pass allow_motion=True to confirm the probe should actually drive the joint."
)


def wall_compliance(backoff: int = _DEFAULT_BACKOFF_TICKS) -> int:
    """The widest LOADED APPROACH a mechanical limit is allowed to show, in ticks.

    Push past your own contact threshold for longer than this and you were not meeting
    an obstacle — you were carrying a load. Derived from *backoff*, which is the
    distance ``gentle_move`` measured it takes to RELIEVE a contact on this arm, so the
    two numbers describe one physical property (the give in the joint) from its two
    sides and cannot drift apart.
    """
    return _COMPLIANCE_FACTOR * int(backoff)


def free_run_needed(watch: LoadWatch = DEFAULT_LOAD_WATCH) -> int:
    """Ticks of FREE travel a probe must have seen before it may call anything a wall.

    Derived, and deliberately symmetric with the rule it guards: the slowest joint ever
    measured on this arm cruises at :data:`~arm101.hardware.gentle._MIN_TICKS_PER_SECOND`,
    so this is the distance such a joint covers in ``stall_samples`` polls — *the same
    amount of evidence the stall rule itself demands before it will call a joint
    stopped*. A probe must see a joint move freely for at least as long as it must see
    it stand still.

    Without it, a joint that was jammed from the very first tick — resting on its own
    end-stop, or simply too weak to lift itself out of the pose it is in — would sail
    through the ``loaded_run`` test (it never travelled while loaded, because it never
    travelled at all) and be recorded as a wall on the strength of having done nothing.
    """
    return int(_MIN_TICKS_PER_SECOND * watch.poll_interval * watch.stall_samples)


# ---------------------------------------------------------------------------
# The approach — the load profile the verdict is actually taken from
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class _Sample:
    """One poll: how far the joint moved since the last one, and how hard it was pushing.

    ``advance`` is a delta WITHIN one move. It is never taken across a move boundary,
    because a re-centre between two moves changes what a reported tick means and the
    difference of two reported ticks either side of it is not a distance.
    """

    advance: int
    load: int


class _Approach:
    """Every sample of every move of one probe, and the two distances the verdict needs.

    Fed straight off ``gentle_move``'s :data:`~arm101.hardware.gentle.TravelObserver`
    seam, so this holds exactly the stream the shipped
    :class:`~arm101.hardware.gentle._StallDetector` was handed — the approach the arm
    really made, not one reconstructed from the ticks it was commanded to.
    """

    def __init__(self, *, threshold: int, watch: LoadWatch) -> None:
        self._threshold = threshold
        self._stall_eps = watch.stall_eps
        self._previous: Optional[int] = None
        self.samples: List[_Sample] = []
        self.peak_load: int = 0

    def begin_move(self, start: int) -> None:
        """Re-anchor the delta at the start of a move. See :class:`_Sample`."""
        self._previous = start

    def observe(self, position: int, load: int) -> None:
        """The observer callback. Never raises — it is a measurement seam, not a control one."""
        previous = position if self._previous is None else self._previous
        self.samples.append(_Sample(advance=abs(position - previous), load=load))
        self.peak_load = max(self.peak_load, load)
        self._previous = position

    @property
    def last_load(self) -> Optional[int]:
        """The load at the stop, if anything was ever sampled."""
        return self.samples[-1].load if self.samples else None

    # -- the two readings ----------------------------------------------------

    def _cruising_free(self, sample: _Sample) -> bool:
        """The joint was MOVING, and it was moving easily."""
        return sample.advance >= self._stall_eps and sample.load <= self._threshold

    def _cruising_loaded(self, sample: _Sample) -> bool:
        """The joint was MOVING, and it was pushing past its own contact threshold."""
        return sample.advance >= self._stall_eps and sample.load > self._threshold

    def runs(self) -> Tuple[int, int]:
        """``(loaded_run, free_run)`` in ticks, walking backwards from the stop.

        Only a sample that shows the joint **moving** ends a run. A sample that did not
        advance says nothing about the load the joint carries while it travels — it may
        be inside the servo's ~95-127 ms motion-onset dead window, or limp for the
        instant a re-centre de-energises it — so it contributes its (zero) distance and
        the walk carries on through it. That is what lets a loaded run be measured
        across a move boundary, which is where a gravity-loaded joint's climb usually
        lies.
        """
        loaded = 0
        index = len(self.samples) - 1
        while index >= 0 and not self._cruising_free(self.samples[index]):
            loaded += self.samples[index].advance
            index -= 1

        free = 0
        while index >= 0 and not self._cruising_loaded(self.samples[index]):
            free += self.samples[index].advance
            index -= 1
        return loaded, free


# ---------------------------------------------------------------------------
# What a probe found
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ProbeOutcome:
    """One probe: the :class:`~arm101.hardware.limits.EndObservation`, and its evidence.

    The observation is the durable record — it is what
    :func:`~arm101.hardware.limits.merge_end_observations` and
    :func:`~arm101.hardware.classify.classify_travel` consume. Everything else here is
    the *reasoning*, kept so that a report can explain a verdict to the human who has
    to trust it, and so that a bench session can see how close the discriminator came
    to its cutoffs before it decided.

    Attributes
    ----------
    observation:
        What was found, and the only thing downstream of here.
    reason:
        Why, in prose an operator can act on.
    loaded_run_ticks / free_run_ticks:
        The two distances the verdict turned on (:meth:`_Approach.runs`).
    compliance:
        The cutoff ``loaded_run_ticks`` was tested against (:func:`wall_compliance`).
    peak_load:
        Highest load magnitude seen anywhere in the approach. Saturated (== the
        ``Torque_Limit`` cap) for a wall AND for an exhausted arm — recorded because it
        is the number a reader will reach for first, and it is the number that cannot
        settle the question.
    contacted / overloaded / arrived:
        Straight from the last ``gentle_move``: what the move itself reported.
    moves / recentres / samples:
        What it cost. ``recentres`` is EEPROM writes.
    """

    observation: EndObservation
    reason: str
    loaded_run_ticks: int
    free_run_ticks: int
    compliance: int
    peak_load: int
    contacted: bool
    overloaded: bool
    arrived: bool
    moves: int
    recentres: int
    samples: int

    @property
    def verdict(self) -> LimitVerdict:
        return self.observation.verdict

    def as_dict(self) -> Dict[str, object]:
        """A plain-JSON-serializable view, for a verb's ``--json`` payload."""
        return {
            "observation": self.observation.to_dict(),
            "reason": self.reason,
            "loaded_run_ticks": self.loaded_run_ticks,
            "free_run_ticks": self.free_run_ticks,
            "compliance": self.compliance,
            "peak_load": self.peak_load,
            "contacted": self.contacted,
            "overloaded": self.overloaded,
            "arrived": self.arrived,
            "moves": self.moves,
            "recentres": self.recentres,
            "samples": self.samples,
        }


# ---------------------------------------------------------------------------
# The rule
# ---------------------------------------------------------------------------


def _rule_on_a_stop(
    joint: str,
    *,
    loaded: int,
    free: int,
    compliance: int,
    needed: int,
    threshold: int,
) -> Tuple[LimitVerdict, str]:
    """Wall, or out of torque? The load PROFILE decides — never the load at the stop.

    Both halves of the WALL rule are necessary, and each closes a different hole:

    * ``loaded > compliance`` — the joint pushed past its own contact threshold for
      longer than a contact's measured give. It was carrying a load, not meeting one.
    * ``free < needed`` — nobody ever saw this joint move freely, so the "sharp
      transition" a wall is recognised by was not observed at all. A joint that was
      jammed before the probe began is the case, and it is genuinely ambiguous.

    Anything ambiguous is TORQUE_LIMITED. See the module docstring's tie-break.
    """
    if loaded > compliance:
        return LimitVerdict.TORQUE_LIMITED, (
            f"{joint} stopped at a saturated load — but it was ALREADY pushing past its "
            f"contact threshold ({threshold}) for the last {loaded} ticks it travelled, "
            f"far more than the {compliance} ticks of give a real contact on this arm was "
            f"measured to have. A joint that works that hard while it is still moving is "
            f"carrying a load, not meeting an obstacle: it ran out of torque. This is a "
            f"LOWER BOUND on {joint}'s travel, not a limit — and no number of poses will "
            f"promote it, because the arm's own weakness is in every pose."
        )
    if free < needed:
        return LimitVerdict.TORQUE_LIMITED, (
            f"{joint} stopped at a saturated load, and the loaded run in front of the stop "
            f"({loaded} ticks) is short enough for a wall — but the joint was only ever seen "
            f"to travel {free} ticks freely beforehand, short of the {needed} needed to show "
            f"it was moving freely at all. It may be resting on its own end-stop; it may be "
            f"too weak to lift itself out of this pose. Nobody has seen it move, so nobody "
            f"may call this a wall. Recorded as a LOWER BOUND."
        )
    return LimitVerdict.WALL, (
        f"{joint} travelled {free} ticks freely (load at or under its {threshold} contact "
        f"threshold), then went from free to a saturated, stalled load within {loaded} ticks "
        f"— inside the {compliance} ticks of give a real contact on this arm has. A sharp "
        f"transition off a free approach is a mechanical limit: the joint is WALLED here, in "
        f"this pose."
    )


# ---------------------------------------------------------------------------
# Argument validation — every one of these would produce a measurement of nothing
# ---------------------------------------------------------------------------


def _refuse(message: str, remediation: str) -> CliError:
    return CliError(code=EXIT_USER_ERROR, message=message, remediation=remediation)


def _require_probe_args(
    *,
    allow_motion: bool,
    threshold: int,
    step: int,
    backoff: int,
    compliance: int,
    max_travel: int,
    watch: LoadWatch,
) -> None:
    """Raise :class:`CliError` on any argument that could not measure anything.

    Checked BEFORE the joint is even read, so a caller that has not consented to motion
    performs literally zero bus operations — the same contract as ``gentle_move`` and
    ``profile_joint``.
    """
    if allow_motion is not True:
        raise _refuse("motion requires an explicit flag", _REMEDIATION_ALLOW_MOTION)

    if not 0 < threshold < CONTACT_LOAD_CEILING:
        raise _refuse(
            f"contact threshold {threshold} can never fire: present_load SATURATES at "
            f"gentle_move's Torque_Limit cap ({CONTACT_LOAD_CEILING}), and contact needs "
            "load > threshold. The joint would press into a wall while the probe reported "
            "free air.",
            f"Pass a threshold in (0, {CONTACT_LOAD_CEILING}) — e.g. this joint's entry in "
            "arm_spec.DEFAULT_CONTACT_THRESHOLDS.",
        )

    # A step no bigger than the arrival tolerance is a target the joint "arrives" at
    # without moving: the probe would creep forever, at zero ticks per move.
    if not watch.arrival_tolerance < step <= MAX_HEADROOM:
        raise _refuse(
            f"a creep step of {step} ticks cannot probe anything: it must be more than the "
            f"{watch.arrival_tolerance}-tick arrival tolerance (or the joint counts as having "
            f"arrived without moving) and at most {MAX_HEADROOM} (which is all a centred frame "
            "can promise in its worst direction).",
            f"Pass a step in ({watch.arrival_tolerance}, {MAX_HEADROOM}] — e.g. the default "
            f"{DEFAULT_CREEP_TICKS}.",
        )

    if backoff < 0:
        raise _refuse(
            f"backoff must be a non-negative number of ticks, got {backoff}",
            f"Pass backoff >= 0 (e.g. the default {_DEFAULT_BACKOFF_TICKS}).",
        )

    if compliance < 0:
        raise _refuse(
            f"compliance must be a non-negative number of ticks, got {compliance}",
            "Pass compliance >= 0 — the widest LOADED approach a wall may show. The default "
            f"is {wall_compliance(_DEFAULT_BACKOFF_TICKS)}, twice gentle_move's measured "
            "contact-relief distance.",
        )

    if not 1 <= max_travel <= ENCODER_TICKS:
        raise _refuse(
            f"max_travel {max_travel} is not a travel budget an observation could hold: it "
            f"must lie in [1, {ENCODER_TICKS}]. A joint that turns a full circle without "
            "finding a wall is CONTINUOUS, and there is nothing past that to learn.",
            f"Pass a max_travel in [1, {ENCODER_TICKS}] (the default is a full turn).",
        )


# ---------------------------------------------------------------------------
# Reading one move's outcome
# ---------------------------------------------------------------------------


def _arrived(result: Dict[str, object], watch: LoadWatch) -> bool:
    """Did the joint MEASURABLY get to the tick it was sent to?"""
    final = result["final_position"]
    if final is None:
        return False
    target = int(result["clamped_target"])  # type: ignore[arg-type]
    return abs(int(final) - target) <= watch.arrival_tolerance


def _contact_displacement(
    result: Dict[str, object],
    *,
    frame: RollingFrame,
    before_raw: int,
    before_displacement: int,
) -> Optional[int]:
    """The frame-wide displacement at the moment of CONTACT — not at the retreat.

    ``gentle_move`` backs the joint ``backoff`` ticks off whatever it hit and holds it
    there, so by the time it returns, the joint is no longer at the wall. Syncing the
    frame afterwards would therefore record every measured wall ``backoff`` ticks short
    of where it is — and a travel measured NARROWER than it is makes the unreachable arc
    WIDER than it is, which is how a re-zero comes to park the seam on a tick the joint
    can actually reach. So the contact point is reconstructed instead, exactly:
    ``contact_position`` is a REPORTED tick the servo was READ at, and the frame does
    not move during a move, so ``frame.offset`` is the offset it was reported in.

    ``None`` when the move somehow contacted without recording where.
    """
    contact = result.get("contact_position")
    if contact is None:
        return None
    contact_raw = raw_from_reported(int(contact), frame.offset)  # type: ignore[arg-type]
    # The move travelled at most one creep step (<= MAX_HEADROOM < HALF_TURN), so the
    # short way round the circle is the way it actually went.
    return before_displacement + signed_delta(contact_raw, before_raw)


def _fit(displacement: int, end: TravelEnd) -> int:
    """Fit a measured displacement into what an :class:`EndObservation` can hold.

    Two clamps, and neither is cosmetic. A probe whose joint ended up BEHIND where it
    started (gravity dragged it while it was limp for a re-centre, and it never got that
    back) reports **0** — "it got nowhere", which is the truth to the resolution the
    record can express, and an honest zero beats a displacement whose sign contradicts
    the end it claims to probe. And a full turn is the cap: past it there is nothing left
    to learn, and the observation would no longer be within one lap of any reference it
    is compared against.
    """
    outward = max(0, min(displacement * end.sign, ENCODER_TICKS))
    return outward * end.sign


# ---------------------------------------------------------------------------
# The probe
# ---------------------------------------------------------------------------


def probe_end(  # noqa: C901 - the loop IS the probe; splitting it would hide the sequence
    bus: "MotorBus",
    frame: RollingFrame,
    *,
    end: TravelEnd,
    threshold: int,
    step: int = DEFAULT_CREEP_TICKS,
    backoff: int = _DEFAULT_BACKOFF_TICKS,
    speed: int = _DEFAULT_SPEED,
    acceleration: int = _DEFAULT_ACCELERATION,
    max_travel: int = ENCODER_TICKS,
    compliance: Optional[int] = None,
    watch: LoadWatch = DEFAULT_LOAD_WATCH,
    allow_motion: bool = False,
    pose: Optional[str] = None,
) -> ProbeOutcome:
    """Creep *frame*'s joint toward one END of its travel until it learns why it stopped.

    The joint starts **wherever it currently is** — the probe reads it, it does not
    assume it — and each creep step's target is asked of *frame* fresh, immediately
    before the move that uses it (:meth:`~arm101.hardware.rolling_frame.RollingFrame.goal`,
    which syncs and re-centres first). That is not a stylistic choice: a ``Goal_Position``
    is a REPORTED tick, and a re-centre changes which physical angle a reported tick
    names, so a target computed before one and used after it commands an angle nobody
    asked for. This bug fired live on the arm; ``test_a_target_computed_BEFORE_a_re_centre
    _names_a_DIFFERENT_ANGLE_after_it`` makes it concrete and
    ``test_every_target_the_probe_commands_is_measured_from_where_the_joint_IS`` audits
    every goal the probe writes against it.

    The loop stops on exactly one of four measured conditions, and each is a verdict:

    ============================= ====================================================
    the servo's overload latched  :attr:`~arm101.hardware.limits.LimitVerdict.TORQUE_LIMITED`
    ``gentle_move`` called contact WALL or TORQUE_LIMITED — see :func:`_rule_on_a_stop`
    the move did not arrive        :attr:`~arm101.hardware.limits.LimitVerdict.TIMEOUT`
    the travel budget ran out      :attr:`~arm101.hardware.limits.LimitVerdict.EDGE`
    ============================= ====================================================

    The frame is **not** owned here: it is opened, and closed, by the caller. One joint's
    two ends belong in ONE frame — that is one calibration transaction (two EEPROM
    writes, not four) and, more importantly, it keeps both ends' displacements inside one
    lap of one another, which is what
    :func:`~arm101.hardware.limits.merge_joint_travel` needs to compare them.

    Parameters
    ----------
    bus:
        The open :class:`~arm101.hardware.bus.MotorBus` *frame* was built on. Passed
        explicitly, as everywhere else in this package (``gentle_move(bus, ...)``,
        ``shift_offset(bus, journal, ...)``); it must be the same bus.
    frame:
        An OPEN :class:`~arm101.hardware.rolling_frame.RollingFrame`. It names the joint
        and the motor, it keeps the seam half a turn ahead of the creep, and it is what
        accumulates the RAW displacement this observation is made of.
    end:
        Which end of the travel to drive toward
        (:class:`~arm101.hardware.limits.TravelEnd`). ``HIGH`` creeps up the raw scale,
        ``LOW`` down.
    threshold:
        The joint's contact-load threshold — ``arm_spec.DEFAULT_CONTACT_THRESHOLDS``'s
        entry for it. Must sit below :data:`~arm101.hardware.gentle.CONTACT_LOAD_CEILING`,
        or contact can never fire at all. It is also the line the approach profile is read
        against: this module invents no second one.
    step:
        Ticks per creep step. See :data:`DEFAULT_CREEP_TICKS`.
    backoff:
        Ticks ``gentle_move`` retreats off a contact. Also the scale from which
        :func:`wall_compliance` derives the default *compliance*.
    speed / acceleration:
        Passed through to ``gentle_move``. The defaults are the only ones this arm has
        ever been proven to detect contact at (see :mod:`arm101.hardware.profile`).
    max_travel:
        The probe's travel budget, in ticks. Defaults to a full turn — past which there
        is nothing to learn (the joint is CONTINUOUS) and past which an
        :class:`~arm101.hardware.limits.EndObservation` cannot hold the answer.
    compliance:
        The widest LOADED approach a wall may show. ``None`` (the default) takes
        :func:`wall_compliance` of *backoff*. Lower it to demand a more rigid stop before
        vouching for a limit; raise it only with hardware evidence, because raising it is
        the one change here that can manufacture a WALL that is not there.
    watch:
        The poll/stall rule ``gentle_move`` runs under. The default carries the
        hardware-measured values.
    allow_motion:
        Must be ``True`` for any bus write to happen at all.
    pose:
        Opaque label recorded on the observation — which pose the other joints were in.
        An observation is only ever evidence *about a pose*; see
        :mod:`arm101.hardware.limits`.

    Returns
    -------
    ProbeOutcome
        The :class:`~arm101.hardware.limits.EndObservation` and the evidence behind it.

    Raises
    ------
    CliError(EXIT_USER_ERROR)
        If ``allow_motion`` is not ``True`` (no writes are issued), or any argument could
        not measure anything. See :func:`_require_probe_args`.
    CliError
        Propagated from the bus and from the frame — in particular
        ``CliError(EXIT_ENV_ERROR)`` when the joint cannot be held still long enough to
        re-centre it. A mid-move ``OverloadError`` is NOT propagated: ``gentle_move``
        catches it, and it becomes a TORQUE_LIMITED verdict.
    """
    end = TravelEnd(end)
    compliance = wall_compliance(backoff) if compliance is None else int(compliance)
    _require_probe_args(
        allow_motion=allow_motion,
        threshold=threshold,
        step=step,
        backoff=backoff,
        compliance=compliance,
        max_travel=max_travel,
        watch=watch,
    )

    direction = end.sign
    needed = free_run_needed(watch)
    approach = _Approach(threshold=threshold, watch=watch)

    # Where the joint IS — read, never assumed. A frame may already have been creeped
    # through by a probe of the OTHER end, so its own origin is not necessarily this
    # probe's; and even a fresh frame's joint may have sagged while it was limp for the
    # opening EEPROM write.
    frame.sync()
    origin_raw = frame.raw
    origin_displacement = frame.displacement
    origin_recentres = frame.recentres
    budget = _MOVE_BUDGET_FACTOR * (max_travel // step + 1) + _MOVE_BUDGET_SLACK

    verdict: Optional[LimitVerdict] = None
    reason = ""
    load: Optional[int] = None
    displacement = 0
    moves = 0
    contacted = False
    overloaded = False
    arrived = False

    while verdict is None:
        travelled = abs(frame.displacement - origin_displacement)
        room = max_travel - travelled
        if room <= watch.arrival_tolerance or moves >= budget:
            verdict, reason = _ran_out(frame.joint, travelled, room, moves, budget, watch)
            load = approach.last_load
            break

        # THE target, computed HERE and used immediately: goal() syncs the frame and
        # re-centres it if it can no longer promise the move, so the tick it returns
        # names the angle it means IN THE FRAME THAT IS ABOUT TO BE COMMANDED.
        target = frame.goal(direction, min(step, room))
        before_raw, before_displacement = frame.raw, frame.displacement
        approach.begin_move(frame.reported)

        result = gentle_move(
            bus,
            frame.motor,
            target,
            # The frame guarantees the target is inside the reported scale, so these
            # bounds never clamp. They are the servo's own, not a calibration: this
            # module reads no calibration, like everything else on top of `gentle`.
            min_angle=TICK_MIN,
            max_angle=TICK_MAX,
            threshold=threshold,
            # NOT `step=`: gentle_move's own `step` is the goal INCREMENT it advances
            # inside a move (25 ticks — small enough that contact is caught within about
            # one of them). The probe's `step` is the length of the whole move. They are
            # different quantities and conflating them would blunt the contact detection
            # this entire module rests on.
            backoff=backoff,
            acceleration=acceleration,
            speed=speed,
            allow_motion=True,
            watch=watch,
            observer=approach.observe,
        )
        moves += 1

        # Fold in whatever the joint ACTUALLY did — including any post-contact retreat —
        # so the frame's books are straight for the next move, and for the restore.
        frame.sync()
        displacement = frame.displacement - origin_displacement

        overloaded = bool(result["overloaded"])
        contacted = bool(result["contacted"])
        arrived = _arrived(result, watch)

        if overloaded:
            verdict, reason = LimitVerdict.TORQUE_LIMITED, _overloaded_reason(frame.joint)
            load = approach.peak_load or None
        elif contacted:
            at_contact = _contact_displacement(
                result,
                frame=frame,
                before_raw=before_raw,
                before_displacement=before_displacement,
            )
            if at_contact is not None:
                displacement = at_contact - origin_displacement
            loaded_run, free_run = approach.runs()
            verdict, reason = _rule_on_a_stop(
                frame.joint,
                loaded=loaded_run,
                free=free_run,
                compliance=compliance,
                needed=needed,
                threshold=threshold,
            )
            load = result["contact_load"]  # type: ignore[assignment]
        elif not arrived:
            verdict, reason = LimitVerdict.TIMEOUT, _timeout_reason(frame.joint, threshold)
            load = approach.last_load

    loaded_run, free_run = approach.runs()
    observation = EndObservation(
        joint=frame.joint,
        end=end,
        verdict=verdict,
        origin_raw=origin_raw,
        displacement=_fit(displacement, end),
        load=load,
        pose=pose,
    )
    return ProbeOutcome(
        observation=observation,
        reason=reason,
        loaded_run_ticks=loaded_run,
        free_run_ticks=free_run,
        compliance=compliance,
        peak_load=approach.peak_load,
        contacted=contacted,
        overloaded=overloaded,
        arrived=arrived,
        moves=moves,
        recentres=frame.recentres - origin_recentres,
        samples=len(approach.samples),
    )


# ---------------------------------------------------------------------------
# The three verdicts that are not a ruling on a contact
# ---------------------------------------------------------------------------


def _ran_out(
    joint: str,
    travelled: int,
    room: int,
    moves: int,
    budget: int,
    watch: LoadWatch,
) -> Tuple[LimitVerdict, str]:
    """The probe stopped without the joint stopping it. EDGE — or a joint that is not there."""
    if moves >= budget:
        return LimitVerdict.TIMEOUT, (
            f"{joint} was commanded {moves} moves and travelled {travelled} ticks — it is not "
            "following its goals. A joint that reports arriving at ticks its shaft never "
            "reached is a bus or a servo fault, not a measurement; the probe stopped rather "
            "than commanding it forever. Nothing was learned about this end."
        )
    return LimitVerdict.EDGE, (
        f"{joint} travelled {travelled} ticks without ever loading up or stalling — the probe "
        f"ran out of room to look ({room} ticks left, under the {watch.arrival_tolerance}-tick "
        "arrival tolerance, so it could not even be told to move). Nothing stopped this joint, "
        "so nothing bounds it HERE: the record is a LOWER BOUND. (A joint that gets a full turn "
        "without a wall is CONTINUOUS — every angle is reachable — and that is the classifier's "
        "call to make from this, not the probe's.)"
    )


def _overloaded_reason(joint: str) -> str:
    return (
        f"{joint} tripped the servo's OWN overload latch (status error bit 5, error=32): the "
        "hardware cut torque before the software stall rule could see anything, so whatever "
        "stopped the joint, nobody measured it. It may have been a wall; it may have been the "
        "arm driving harder than it can sustain. A LOWER BOUND, never a limit."
    )


def _timeout_reason(joint: str, threshold: int) -> str:
    return (
        f"{joint} neither arrived at its target nor ever loaded past its contact threshold "
        f"({threshold}) — it stopped advancing in what the arm reports as free air. That is not "
        "a limit, it is a joint that did not do what it was told: a slipped gear, a servo that "
        "is not following, or a bus that is not being heard. Nothing was learned about this end."
    )
