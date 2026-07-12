"""Speed profiling — the highest speed at which CONTACT DETECTION still works.

Pure helper over :mod:`arm101.hardware.gentle` — zero third-party dependencies,
and no knowledge of calibration, roles, or the CLI: callers source the joint's
bounds and threshold and pass them in, exactly as ``gentle_move`` requires.

Why this module exists
----------------------
``arm explore`` mapped **2 cells in 25 minutes** and left the arm at ~50 C. Probe
cost is the bottleneck, and probe cost is dominated by TRAVEL TIME. Yet every
motion constant in :mod:`arm101.hardware.gentle` was hand-fitted in a single
bench session: ``_DEFAULT_SPEED = 150``, ``_MIN_TICKS_PER_SECOND = 120``, and a
timeout sized from them. At speed 150 a 500-tick move measures ~930 ms on
``wrist_roll`` but ~3300 ms on the shoulders — a 3.5x spread nobody has ever
explained, against a ceiling nobody has ever measured. This module measures it.

**The whole point, and the thing that makes this hard: speed and contact
detection are COUPLED.**

The stall rule (:class:`arm101.hardware.gentle._StallDetector`) calls CONTACT
when ``present_load`` exceeds a threshold *while the joint has stopped
advancing*. Both halves are needed — a joint merely ACCELERATING through open air
peaks at a load of 300 on ``wrist_roll``, above its own threshold, so the load
gate alone would call contact on every move. The stall gate is what separates
"blocked" from "accelerating", and it needs a *moving* joint to visibly ADVANCE
between samples: at the 25 ms poll interval the slowest joints travel ~150
ticks/s, covering ~4 ticks per sample, comfortably over the 2-tick ``stall_eps``.
Drive faster on the same poll interval and that discrimination erodes — the joint
pressed into a compliant contact keeps creeping *harder*, reads as "still
moving", and the stall counter never accumulates; or the servo's own overload
latch (error=32, one-shot at speed 400 or on a rigid stop) trips first and cuts
torque before the software rule has seen its 8 consecutive stalled samples.

So a speed the servo merely SURVIVES is not a speed this module will accept:

    **A speed at which the arm moves but contact can no longer be detected is a
    FAILURE of that speed, not a pass. Free motion at a speed proves NOTHING.**

Every candidate speed is therefore certified against a REAL CONTACT, by driving
the joint into an obstacle it genuinely cannot pass and requiring the shipped
:func:`~arm101.hardware.gentle.gentle_move` to come back with
``contacted=True``. Not a copy of the detector — the detector itself, through the
:data:`~arm101.hardware.gentle.TravelObserver` seam, so what is certified is the
code that will actually drive the arm.

The ladder
----------
Candidates run LOW to HIGH from a speed with hardware evidence behind it
(:data:`DEFAULT_SPEED_START` = 150, ``gentle_move``'s default and the only speed
the arm has ever been proven to detect contact at), and the ramp **stops at the
first rejection**. It does not "keep looking" past a failure: speed → detection
is expected to be monotone (more speed can only erode the margin, never restore
it), and probing above a speed already known to miss contacts would mean
deliberately slamming the arm into an obstacle at a speed where the software
cannot tell it has hit anything. The last ACCEPTED speed is the answer; the first
rejected one is the ceiling, recorded with the reason it failed.

Verdicts
--------
Each trial ends in exactly one of four verdicts (see :data:`REASON_OVERLOAD` and
friends). Only :data:`REASON_CONTACT_DETECTED` is a pass.

Hardware facts these rules are built on (all MEASURED on the follower arm)
-------------------------------------------------------------------------
* ``present_load`` **saturates at ``Torque_Limit``** — limit 300 pins load at
  300, limit 600 pins it at 600. ``gentle_move`` caps ``Torque_Limit`` to 500 for
  the duration of a move, so any threshold >= 500 can NEVER fire, at any speed.
* Free-motion peak load per joint — the floor of each joint's usable threshold
  band: ``shoulder_pan`` 88, ``shoulder_lift`` 92, ``elbow_flex`` 148,
  ``wrist_flex`` 96, ``wrist_roll`` **300**, ``gripper`` 76.
* The STS3215 throws a dynamic overload (status error bit 5, ``error=32``) if
  driven too hard — one-shot at speed 400, or on a rigid stop — and recovery is a
  torque-disable. This ramp WILL provoke it; that is a legitimate way to find the
  ceiling, and it is handled, not avoided (see :data:`REASON_OVERLOAD`).
* The servo does not begin moving for ~95-127 ms after a goal write, and takes
  ~1.0-1.2 s to reverse off a contact.
"""

from __future__ import annotations

import contextlib
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, Sequence

from arm101.cli._errors import EXIT_USER_ERROR, CliError
from arm101.hardware.gentle import DEFAULT_LOAD_WATCH, LoadWatch, gentle_move
from arm101.hardware.motion import clamp_goal

if TYPE_CHECKING:
    from arm101.hardware.bus import MotorBus

# ---------------------------------------------------------------------------
# Verdicts — the four ways a candidate speed can end
# ---------------------------------------------------------------------------

#: PASS. The joint drove into the obstacle and ``gentle_move`` returned
#: ``contacted=True``: load crossed the threshold AND the joint stopped
#: advancing, for :data:`~arm101.hardware.gentle._DEFAULT_STALL_SAMPLES`
#: consecutive samples. The stall rule works at this speed — the only evidence
#: this module accepts.
REASON_CONTACT_DETECTED = "contact_detected"

#: REJECT. **The crux failure.** The approach loaded past the joint's contact
#: threshold — so the joint demonstrably MET something — yet the move ended
#: without the stall rule ever firing. At this speed the detector can no longer
#: tell "blocked" from "accelerating", which is exactly the discrimination it
#: exists to make. Two physical readings, both disqualifying and both caught by
#: the same test:
#:
#: * the joint drove so hard into a compliant contact that it kept creeping past
#:   ``stall_eps`` every sample and never read as stopped (it may even have
#:   bulldozed through to the commanded target under saturated load); or
#: * free-motion load at this speed has risen above the joint's threshold, so the
#:   load gate no longer separates a loaded joint from a fast one and the usable
#:   band has collapsed.
#:
#: Either way the arm can now press into an obstacle without the software
#: noticing. That is a failure OF THE SPEED.
REASON_CONTACT_MISSED = "contact_missed"

#: REJECT. The servo's OWN overload latch (error=32) tripped mid-probe — the
#: hardware protection beat the software stall rule to the punch and cut torque
#: before 8 consecutive stalled samples could accumulate (~200 ms). Contact was
#: therefore never *detected*; it was survived. ``gentle_move`` has already
#: recovered (``clear_overload`` → torque released, latch cleared) and reported
#: ``overloaded=True`` rather than raising, so this is a clean, expected ceiling —
#: not an error.
REASON_OVERLOAD = "overload"

#: REJECT (and, on the FIRST candidate, a hard :class:`CliError`). The move ended
#: without the joint ever loading past its contact threshold: it sailed to the
#: contact target through free air, or timed out short of it under no load. There
#: was nothing to detect, so nothing was proven — and a speed "validated" on free
#: motion alone is not validated at all. On the first candidate this means the
#: probe geometry is wrong (``--contact-to`` names a tick the joint can actually
#: reach) and the whole run is void; later it means the obstacle moved or the arm
#: went through it, and the run stops with whatever it had already earned.
REASON_NO_CONTACT = "no_contact"

# ---------------------------------------------------------------------------
# Ladder defaults
# ---------------------------------------------------------------------------

#: First candidate. Deliberately ``gentle_move``'s own ``_DEFAULT_SPEED`` — the
#: ONE speed at which contact detection has ever been observed to work on this
#: hardware. Starting anywhere higher would mean the first probe is already an
#: unvalidated speed, and a failure there would leave the run with no safe speed
#: to fall back to (and no way to retreat the joint at a trusted speed).
DEFAULT_SPEED_START = 150

#: Step between candidates. A compromise: fine enough that the answer is useful
#: (a ceiling reported as "somewhere in 150-400" would not be), coarse enough
#: that a full ramp is a handful of probes rather than dozens of contacts against
#: a joint that heats up.
DEFAULT_SPEED_STEP = 50

#: Last candidate the default ladder will try. Sits ABOVE the speed at which a
#: one-shot overload was measured (400), so a default run brackets the known
#: hardware ceiling rather than stopping politely short of it — the ramp is
#: SUPPOSED to find the wall, and it is designed to survive doing so.
DEFAULT_SPEED_MAX = 600

#: Widest Goal_Speed the STS3215 register accepts (addr 46, 2 bytes).
MAX_SPEED = 4095

#: Gentle acceleration for every probe (STS3215 Acceleration register, [0, 254]).
#: Held CONSTANT across the ladder on purpose: the experiment varies exactly one
#: variable — Goal_Speed — and mixing in a second would make the ceiling
#: uninterpretable. It matches ``gentle_move``'s own default so a probe
#: accelerates the way a real gentle move does.
DEFAULT_ACCELERATION = 20


# ---------------------------------------------------------------------------
# Sample trace — what the shipped detector saw, timed from the outside
# ---------------------------------------------------------------------------


class _Trace:
    """Records the samples of one approach, and the clock, as they happen.

    Plugged into ``gentle_move`` as its :data:`~arm101.hardware.gentle.
    TravelObserver`, so every value here comes off the SAME stream the real
    :class:`~arm101.hardware.gentle._StallDetector` is fed. Nothing is inferred
    from the commanded ticks — that assumption is precisely the bug ``gentle.py``
    was rewritten to kill.

    The clock is injected (rather than read straight from :func:`time.monotonic`)
    so a test against a :class:`~arm101.hardware.bus.FakeBus` — which advances the
    simulated shaft per *read*, not per second — can supply a clock that ticks one
    poll interval per sample and get exactly the timings real hardware would
    produce.

    Timings are reported ONLY when the trace was actually PACED (see :attr:`paced`).
    A fake bus on the real monotonic clock lands all of its samples inside a few
    microseconds, and a rate computed from that would be a property of the test
    harness — some millions of ticks per second — dressed up as a property of the
    arm. This module exists because the motion constants were guessed once and
    never checked; the last thing it may do is hand back a fabricated measurement.
    Unmeasurable is reported as ``None``.

    Attributes
    ----------
    samples:
        ``(t, position, load_magnitude)`` per approach sample, in order.
    peak_load:
        Highest load magnitude seen anywhere in the approach. This is what
        separates :data:`REASON_CONTACT_MISSED` (the joint met resistance and the
        rule missed it) from :data:`REASON_NO_CONTACT` (there was nothing there).
        Unlike the timings, this is meaningful on any bus — it is read off the
        servo, not off the clock.
    """

    #: A trace is PACED when its elapsed time is at least this fraction of what
    #: its sample count implies (``len(samples) * poll_interval``). Real hardware
    #: always clears it: :func:`arm101.hardware.gentle._sample` sleeps one poll
    #: interval before every read, so elapsed >= sample_count * interval, with
    #: room to spare. A bus that does not advance by wall-clock never comes close.
    #: The margin is generous on purpose — this is a sanity gate against a
    #: nonsense measurement, not a precision instrument.
    _PACED_FRACTION = 0.5

    def __init__(
        self,
        start: int,
        *,
        onset_ticks: int,
        clock: "Callable[[], float]",
        poll_interval: float = DEFAULT_LOAD_WATCH.poll_interval,
    ) -> None:
        self._start = start
        self._onset_ticks = onset_ticks
        self._clock = clock
        self._poll_interval = poll_interval
        #: Stamped BEFORE the move is commanded, so the onset window includes the
        #: goal write itself — which is the latency a caller actually pays.
        self.t0 = clock()
        self.samples: list[tuple[float, int, int]] = []
        self.peak_load = 0
        self._onset_raw: "float | None" = None

    def observe(self, position: int, load: int) -> None:
        """The :data:`~arm101.hardware.gentle.TravelObserver` callback. Never raises."""
        now = self._clock()
        self.samples.append((now, position, load))
        self.peak_load = max(self.peak_load, load)
        if self._onset_raw is None and abs(position - self._start) >= self._onset_ticks:
            self._onset_raw = now - self.t0

    @property
    def paced(self) -> bool:
        """Did wall-clock time actually pass between samples? See :data:`_PACED_FRACTION`."""
        if not self.samples:
            return False
        elapsed = self.samples[-1][0] - self.t0
        return elapsed >= len(self.samples) * self._poll_interval * self._PACED_FRACTION

    @property
    def onset_seconds(self) -> "float | None":
        """Seconds from the goal write until the joint had measurably moved ``onset_ticks``.

        The servo's motion-onset dead window — ~95-127 ms on hardware, and a cost
        every probe pays. ``None`` if the joint never moved that far, or if the
        trace was not :attr:`paced`.
        """
        return self._onset_raw if self.paced else None

    @property
    def distance_ticks(self) -> "int | None":
        """Ticks travelled over the whole approach, MEASURED, or ``None`` if unsampled.

        The last observed sample is always the end of the approach: the observer
        is only called from the travel loop, and that loop returns immediately
        after the sample that ends it (contact, arrival, or timeout). The retreat
        that follows a contact is not observed — see
        :data:`~arm101.hardware.gentle.TravelObserver`.
        """
        if not self.samples:
            return None
        return abs(self.samples[-1][1] - self._start)

    @property
    def travel_seconds(self) -> "float | None":
        """Wall-clock seconds from the goal write to the end of the approach.

        ``None`` when the trace was not :attr:`paced` — see the class docstring.
        """
        if not self.samples or not self.paced:
            return None
        return self.samples[-1][0] - self.t0

    @property
    def ticks_per_second(self) -> "float | None":
        """Measured travel rate over the approach, INCLUDING the motion-onset window.

        Onset is deliberately included: this number exists to predict probe COST,
        and a probe pays the dead window on every single move. It is directly
        comparable with the bench figures the module docstring quotes (~930 ms for
        a 500-tick move on ``wrist_roll``), which were whole-move measurements too.

        ``None`` when the trace carries no usable time (see :attr:`paced`) — never
        a fabricated number.
        """
        distance = self.distance_ticks
        seconds = self.travel_seconds
        if distance is None or not seconds or seconds <= 0:
            return None
        return distance / seconds


# ---------------------------------------------------------------------------
# Results
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class SpeedTrial:
    """One candidate speed, probed against a real contact, and its verdict.

    Attributes
    ----------
    speed:
        The Goal_Speed under test.
    accepted:
        ``True`` **only** when the shipped stall rule detected the contact at this
        speed (:data:`REASON_CONTACT_DETECTED`). Nothing else counts.
    reason:
        One of the four ``REASON_*`` verdicts.
    contacted / overloaded:
        Straight from ``gentle_move``'s result — what the move itself reported.
    arrived:
        The joint ended within the arrival tolerance of the commanded contact
        target, i.e. it reached a tick it was supposed to be unable to reach.
        Recorded, but never decisive on its own: a joint can arrive because
        nothing was there (:data:`REASON_NO_CONTACT`) *or* because it bulldozed
        through a compliant obstacle under load without the rule firing
        (:data:`REASON_CONTACT_MISSED`). ``peak_load`` is what tells those apart.
    peak_load:
        Highest load magnitude reached during the approach.
    ticks_per_second / motion_onset_seconds:
        The measurements this whole verb exists to produce. ``None`` when the
        clock could not resolve them.
    """

    speed: int
    accepted: bool
    reason: str
    contacted: bool
    overloaded: bool
    arrived: bool
    peak_load: int
    samples: int
    start_position: "int | None"
    contact_position: "int | None"
    contact_load: "int | None"
    final_position: "int | None"
    distance_ticks: "int | None"
    travel_seconds: "float | None"
    ticks_per_second: "float | None"
    motion_onset_seconds: "float | None"

    def as_dict(self) -> dict[str, object]:
        """JSON-serialisable form, for a verb's ``--json`` payload."""
        return {
            "speed": self.speed,
            "accepted": self.accepted,
            "reason": self.reason,
            "contacted": self.contacted,
            "overloaded": self.overloaded,
            "arrived": self.arrived,
            "peak_load": self.peak_load,
            "samples": self.samples,
            "start_position": self.start_position,
            "contact_position": self.contact_position,
            "contact_load": self.contact_load,
            "final_position": self.final_position,
            "distance_ticks": self.distance_ticks,
            "travel_seconds": self.travel_seconds,
            "ticks_per_second": self.ticks_per_second,
            "motion_onset_seconds": self.motion_onset_seconds,
        }


@dataclass(frozen=True)
class JointSpeedProfile:
    """What one joint's ramp established. The deliverable.

    Attributes
    ----------
    safe_speed:
        The highest speed at which a REAL contact was still detected — the answer.
        ``None`` when even the first candidate failed: the profile does NOT fall
        back to a guess, because a guessed speed is exactly the thing this verb
        was written to replace.
    ticks_per_second / motion_onset_seconds:
        Measured AT ``safe_speed`` (the speed a caller would actually use), not at
        the ceiling and not averaged across the ladder. ``None`` when
        ``safe_speed`` is.
    ceiling_speed / ceiling_reason:
        The first REJECTED candidate and why (a ``REASON_*``). ``None`` when the
        whole ladder passed — in which case the true ceiling is above the ladder,
        and ``safe_speed`` is a floor on it, not the wall itself.
    trials:
        Every candidate, in order, verdicts included. The audit trail: a reader can
        see exactly what the arm did at each speed rather than trusting a summary.
    """

    joint: str
    motor: int
    home: int
    contact_target: int
    threshold: int
    ladder: tuple[int, ...]
    safe_speed: "int | None"
    ticks_per_second: "float | None"
    motion_onset_seconds: "float | None"
    ceiling_speed: "int | None"
    ceiling_reason: "str | None"
    trials: tuple[SpeedTrial, ...]

    @property
    def certified(self) -> bool:
        """``True`` iff at least one speed was proven to still detect contact."""
        return self.safe_speed is not None

    def as_dict(self) -> dict[str, object]:
        """JSON-serialisable form, for a verb's ``--json`` payload."""
        return {
            "joint": self.joint,
            "motor": self.motor,
            "home": self.home,
            "contact_target": self.contact_target,
            "threshold": self.threshold,
            "ladder": list(self.ladder),
            "certified": self.certified,
            "safe_speed": self.safe_speed,
            "ticks_per_second": self.ticks_per_second,
            "motion_onset_seconds": self.motion_onset_seconds,
            "ceiling_speed": self.ceiling_speed,
            "ceiling_reason": self.ceiling_reason,
            "trials": [t.as_dict() for t in self.trials],
        }


#: Progress hook, called with each :class:`SpeedTrial` the moment its verdict is
#: known — so a CLI verb can narrate a long ramp on stderr while it runs, instead
#: of going silent for minutes and then printing everything at once.
ProgressHook = Callable[[SpeedTrial], None]


# ---------------------------------------------------------------------------
# The ladder
# ---------------------------------------------------------------------------


def speed_ladder(
    start: int = DEFAULT_SPEED_START,
    step: int = DEFAULT_SPEED_STEP,
    stop: int = DEFAULT_SPEED_MAX,
) -> tuple[int, ...]:
    """Build the ascending candidate ladder ``start, start+step, ... <= stop``.

    Always includes *start*, even when ``start > stop`` would otherwise make the
    ladder empty — an empty ladder would mean "profile nothing" and return a
    profile that certifies nothing while looking like a successful run. The one
    thing this module must never do is come back with an answer it did not earn.

    Raises
    ------
    CliError(EXIT_USER_ERROR)
        If any value is out of the STS3215's ``[1, 4095]`` Goal_Speed range, or
        *step* is not positive (a non-positive step never terminates).
    """
    for name, value in (("--speed-start", start), ("--speed-max", stop)):
        if not 1 <= value <= MAX_SPEED:
            raise CliError(
                code=EXIT_USER_ERROR,
                message=f"{name} must be in [1, {MAX_SPEED}] (STS3215 Goal_Speed), got {value}",
                remediation=f"Pass a {name} between 1 and {MAX_SPEED}.",
            )
    if step <= 0:
        raise CliError(
            code=EXIT_USER_ERROR,
            message=f"--speed-step must be a positive number of speed units, got {step}",
            remediation=f"Pass a positive --speed-step (e.g. the default {DEFAULT_SPEED_STEP}).",
        )

    ladder = []
    speed = start
    while speed <= stop:
        ladder.append(speed)
        speed += step
    return tuple(ladder) if ladder else (start,)


# ---------------------------------------------------------------------------
# One probe
# ---------------------------------------------------------------------------


def _classify(
    result: dict[str, object],
    trace: _Trace,
    *,
    threshold: int,
) -> tuple[bool, str]:
    """Turn one ``gentle_move`` outcome into ``(accepted, reason)``.

    The order of these branches is the safety policy, and it is deliberate:

    1. An **overload** is checked first, because a latched servo never got the
       chance to be detected — whatever else the move reports about a joint whose
       torque was cut mid-press is downstream of the hardware giving up.
    2. **contacted=True** is the ONLY acceptance. Not "the joint stopped", not
       "the load was high", not "it survived" — the shipped rule fired.
    3. A peak load past the joint's own threshold with no contact called is the
       crux failure: the joint met something and the rule missed it.
    4. Anything else means the probe never met resistance at all, so it proved
       nothing about detection either way.
    """
    if result["overloaded"]:
        return False, REASON_OVERLOAD
    if result["contacted"]:
        return True, REASON_CONTACT_DETECTED
    if trace.peak_load > threshold:
        return False, REASON_CONTACT_MISSED
    return False, REASON_NO_CONTACT


def _probe(
    bus: "MotorBus",
    motor: int,
    *,
    speed: int,
    contact_target: int,
    min_angle: int,
    max_angle: int,
    threshold: int,
    acceleration: int,
    watch: LoadWatch,
    clock: "Callable[[], float]",
) -> SpeedTrial:
    """Drive *motor* into the obstacle at *speed* and rule on what happened.

    ONE ``gentle_move`` — the real one, not a re-implementation — with a
    :class:`_Trace` wired into its observer seam. That single move yields
    everything a trial needs: the travel rate and the motion-onset latency (from
    the trace) and the contact verdict (from the move's own result), all measured
    on the same approach, at the same speed, by the same code that will drive the
    arm in anger. There is no second, "free-motion-only" leg, because free motion
    at a speed proves nothing about detection at that speed — that is the whole
    thesis of this module.
    """
    start = bus.read_info(motor)["present_position"]
    trace = _Trace(
        start,
        onset_ticks=watch.onset_ticks,
        clock=clock,
        poll_interval=watch.poll_interval,
    )

    result = gentle_move(
        bus,
        motor,
        contact_target,
        min_angle=min_angle,
        max_angle=max_angle,
        threshold=threshold,
        acceleration=acceleration,
        speed=speed,
        allow_motion=True,
        watch=watch,
        observer=trace.observe,
    )

    accepted, reason = _classify(result, trace, threshold=threshold)

    final = result["final_position"]
    clamped = result["clamped_target"]
    arrived = final is not None and abs(int(final) - int(clamped)) <= watch.arrival_tolerance

    return SpeedTrial(
        speed=speed,
        accepted=accepted,
        reason=reason,
        contacted=bool(result["contacted"]),
        overloaded=bool(result["overloaded"]),
        arrived=arrived,
        peak_load=trace.peak_load,
        samples=len(trace.samples),
        start_position=result["start_position"],  # type: ignore[arg-type]
        contact_position=result["contact_position"],  # type: ignore[arg-type]
        contact_load=result["contact_load"],  # type: ignore[arg-type]
        final_position=final,  # type: ignore[arg-type]
        distance_ticks=trace.distance_ticks,
        travel_seconds=trace.travel_seconds,
        ticks_per_second=trace.ticks_per_second,
        motion_onset_seconds=trace.onset_seconds,
    )


# ---------------------------------------------------------------------------
# Between and after probes — the joint must never be left pressed into anything
# ---------------------------------------------------------------------------


def _return_home(
    bus: "MotorBus",
    motor: int,
    *,
    home: int,
    speed: int,
    min_angle: int,
    max_angle: int,
    threshold: int,
    acceleration: int,
    watch: LoadWatch,
) -> None:
    """Retreat the joint to *home* at a TRUSTED *speed*, so the next probe has a run-up.

    Two jobs, and the second is the important one:

    * every candidate must start from the same geometry, or the ladder is
      comparing probes with different run-ups and its numbers mean nothing; and
    * the previous probe may have left the joint **pressed into the obstacle** —
      that is precisely what a :data:`REASON_CONTACT_MISSED` trial IS — so backing
      it off is the safing step, not a convenience.

    *speed* is never the candidate under test: it is the last speed already PROVEN
    to detect contact (or the ladder's first rung, which carries hardware
    evidence). Retreating at an uncertified speed would mean the one move whose
    job is to make the arm safe is itself made at a speed the arm may not be able
    to stop at.
    """
    gentle_move(
        bus,
        motor,
        home,
        min_angle=min_angle,
        max_angle=max_angle,
        threshold=threshold,
        acceleration=acceleration,
        speed=speed,
        allow_motion=True,
        watch=watch,
    )


def _release(bus: "MotorBus", motor: int) -> None:
    """De-energise the profiled joint. Best-effort; never raises.

    ``gentle_move``'s contract is stop-and-HOLD, which is right for a gripper that
    has closed on something and wrong for this: a profiling run ends with the
    joint parked a few ticks off an obstacle it spent the last several minutes
    slamming into, and holding it there — energised, warm, against a wall, with
    the operator's attention already elsewhere — is the exact shape of the
    incident that produced :mod:`arm101.hardware.safety`. The operator asked for a
    MEASUREMENT, not a pose, so the joint goes limp when the measurement is done.

    ``clear_overload`` and not ``enable_torque(motor, False)``: identical on the
    wire (Torque_Enable = 0, addr 40), but overload-tolerant — and a joint that
    has just been driven into a wall at rising speeds is the single likeliest
    motor on the arm to be sitting latched.
    """
    with contextlib.suppress(CliError):
        bus.clear_overload(motor)


# ---------------------------------------------------------------------------
# The ramp
# ---------------------------------------------------------------------------


def _no_obstacle_error(joint: str, contact_target: int) -> CliError:
    """The hard stop when the very first probe never met anything.

    Loud, and a failure rather than a shrug, because the alternative is worse: a
    run that quietly reports a "safe speed" it certified against thin air. Free
    motion at a speed proves nothing about contact detection at that speed, and a
    profile that pretended otherwise would hand the next task a number with no
    evidence under it — while looking exactly like a number that had.
    """
    return CliError(
        code=EXIT_USER_ERROR,
        message=(
            f"{joint}: the probe reached tick {contact_target} without ever loading past "
            "its contact threshold — there is nothing there to detect, so no speed was "
            "validated. A speed proven on free motion alone is NOT proven."
        ),
        remediation=(
            "Point --contact-to at a tick the joint genuinely CANNOT reach — its "
            "mechanical end-stop, or a fixture you have clamped in its path — so that "
            "every candidate speed is certified against a real contact."
        ),
    )


def _require_motion_gate(allow_motion: bool) -> None:
    """Raise :class:`CliError` unless motion was explicitly opted into.

    Checked FIRST, before the joint's position is even read, so a caller that has
    not consented performs literally zero bus operations — the same contract as
    :func:`~arm101.hardware.gentle.gentle_move` and
    :func:`~arm101.hardware.motion.compliant_move`. The ladder needs no equivalent
    guard: an empty one falls back to :func:`speed_ladder`, which is itself
    incapable of returning empty.
    """
    if allow_motion is not True:
        raise CliError(
            code=EXIT_USER_ERROR,
            message="motion requires an explicit flag",
            remediation=(
                "Pass allow_motion=True to confirm the profiling run should actually "
                "drive the arm into a contact."
            ),
        )


def _reset_speed(safe: "SpeedTrial | None", ladder: "Sequence[int]") -> int:
    """The speed a retreat is allowed to use: the last CERTIFIED one, else rung 1."""
    return safe.speed if safe is not None else ladder[0]


def profile_joint(
    bus: "MotorBus",
    motor: int,
    *,
    joint: str,
    contact_target: int,
    min_angle: int,
    max_angle: int,
    threshold: int,
    ladder: "Sequence[int]" = (),
    acceleration: int = DEFAULT_ACCELERATION,
    allow_motion: bool = False,
    watch: LoadWatch = DEFAULT_LOAD_WATCH,
    clock: "Callable[[], float]" = time.monotonic,
    progress: "ProgressHook | None" = None,
) -> JointSpeedProfile:
    """Ramp *motor*'s Goal_Speed and return the highest speed that still DETECTS contact.

    The gated entry point: by default (``allow_motion=False``) it raises and
    performs **no bus writes at all**, matching
    :func:`~arm101.hardware.gentle.gentle_move` and
    :func:`~arm101.hardware.motion.compliant_move`. A CLI verb must surface an
    explicit consent gate rather than defaulting motion to on.

    For each rung of *ladder*, low to high:

    1. Retreat to the joint's home pose (skipped on the first rung, where it is
       already there) at the last CERTIFIED speed — never at the untested
       candidate. See :func:`_return_home`.
    2. Drive the joint at the candidate speed toward *contact_target*, a tick it
       must be physically UNABLE to reach, via the real ``gentle_move`` with a
       :class:`_Trace` on its observer seam.
    3. Rule on the outcome (:func:`_classify`). **Accept only if ``gentle_move``
       reported ``contacted=True``** — i.e. the shipped stall rule fired on a real
       obstacle at that speed. A speed the servo merely tolerated, or one at which
       the joint pressed into the obstacle without the rule noticing, is a
       FAILURE of that speed.
    4. On the first rejection, STOP. The answer is the last accepted rung; the
       rejected one is the ceiling, recorded with its reason.

    However the run ends — success, rejection, or a bus fault mid-ramp — a
    ``finally`` retreats the joint to home and then DE-ENERGISES it
    (:func:`_release`), because a profiling run's last act must never be to leave a
    joint holding itself against the wall it was just driven into. Both halves are
    best-effort: on a bus that has already failed, a ``CliError`` raised out of the
    cleanup would mask the real failure, and
    :class:`~arm101.hardware.safety.TorqueGuard` is the backstop above this anyway.

    Parameters
    ----------
    bus:
        An open :class:`~arm101.hardware.bus.MotorBus` (real or fake).
    motor:
        Motor ID (1-indexed, matching the Feetech servo ID).
    joint:
        Joint name, recorded on the result. Never used to look anything up — this
        module stays decoupled from ``arm_spec``, like ``gentle``/``motion``.
    contact_target:
        A tick the joint genuinely CANNOT reach: its mechanical end-stop, or a
        fixture in its path. Clamped to ``[min_angle, max_angle]`` like any goal.
        This is the single most important argument, and the module's whole
        validity rests on it: a reachable target certifies nothing (see
        :data:`REASON_NO_CONTACT`).
    min_angle / max_angle:
        The joint's bounds, sourced by the caller (this module reads no
        calibration).
    threshold:
        The joint's contact-load threshold — the same number ``arm explore`` uses
        (``arm_spec.DEFAULT_CONTACT_THRESHOLDS``). Must sit below 500, or the
        contact can never fire at ANY speed: ``present_load`` saturates at
        ``gentle_move``'s ``Torque_Limit`` cap.
    ladder:
        Ascending candidate speeds; defaults to :func:`speed_ladder`'s defaults.
    acceleration:
        Held constant across the ramp — see :data:`DEFAULT_ACCELERATION`.
    allow_motion:
        Must be ``True`` for any bus write to happen at all.
    watch:
        The poll/stall rule under test. The default carries the hardware-measured
        values; overriding it changes *what is being certified*, so a caller that
        tunes it is profiling a different detector.
    clock:
        Injected time source (see :class:`_Trace`).
    progress:
        Optional per-trial hook (see :data:`ProgressHook`).

    Returns
    -------
    JointSpeedProfile
        ``safe_speed`` (the answer, or ``None`` if nothing could be certified),
        the ``ticks_per_second`` and ``motion_onset_seconds`` measured AT that
        speed, the ``ceiling_speed``/``ceiling_reason``, and every trial.

    Raises
    ------
    CliError(EXIT_USER_ERROR)
        If ``allow_motion`` is not ``True`` (no writes are issued); if
        *contact_target* clamps onto the joint's current position (there would be
        no travel, hence no probe); or if the FIRST probe found no obstacle at all
        — see :func:`_no_obstacle_error`. The joint is safed before that last one
        is raised.
    CliError
        Propagated from the bus (e.g. a comms failure). A mid-probe
        ``OverloadError`` is NOT propagated — ``gentle_move`` catches it and it
        becomes a :data:`REASON_OVERLOAD` ceiling.
    """
    _require_motion_gate(allow_motion)
    rungs = tuple(ladder) if ladder else speed_ladder()

    home = int(bus.read_info(motor)["present_position"])
    clamped_target, _ = clamp_goal(contact_target, min_angle, max_angle)
    if clamped_target == home:
        raise CliError(
            code=EXIT_USER_ERROR,
            message=(
                f"{joint}: --contact-to clamps to tick {clamped_target}, which is where the "
                "joint already is — the probe would command no travel at all."
            ),
            remediation=(
                "Move the joint away from the obstacle first, or pick a --contact-to on "
                "the far side of it."
            ),
        )

    trials: list[SpeedTrial] = []
    safe: "SpeedTrial | None" = None
    ceiling_speed: "int | None" = None
    ceiling_reason: "str | None" = None
    # Held rather than raised on the spot: the joint is pressed somewhere it should
    # not be, and safing it comes before reporting anything to anyone.
    void_run: "CliError | None" = None

    def retreat() -> None:
        """Back the joint off to home at the last CERTIFIED speed. See _return_home."""
        _return_home(
            bus,
            motor,
            home=home,
            speed=_reset_speed(safe, rungs),
            min_angle=min_angle,
            max_angle=max_angle,
            threshold=threshold,
            acceleration=acceleration,
            watch=watch,
        )

    try:
        for index, speed in enumerate(rungs):
            if index:
                retreat()  # rung 1 needs no run-up: the joint is already home

            trial = _probe(
                bus,
                motor,
                speed=speed,
                contact_target=contact_target,
                min_angle=min_angle,
                max_angle=max_angle,
                threshold=threshold,
                acceleration=acceleration,
                watch=watch,
                clock=clock,
            )
            trials.append(trial)
            if progress is not None:
                progress(trial)

            if trial.accepted:
                safe = trial
                continue

            ceiling_speed = trial.speed
            ceiling_reason = trial.reason
            if trial.reason == REASON_NO_CONTACT and safe is None:
                void_run = _no_obstacle_error(joint, clamped_target)
            break
    finally:
        # Best-effort on a bus that may have just died: a CliError raised here
        # would MASK the failure the operator actually needs to see.
        with contextlib.suppress(CliError):
            retreat()
        _release(bus, motor)

    if void_run is not None:
        raise void_run

    return JointSpeedProfile(
        joint=joint,
        motor=motor,
        home=home,
        contact_target=clamped_target,
        threshold=threshold,
        ladder=rungs,
        safe_speed=safe.speed if safe else None,
        ticks_per_second=safe.ticks_per_second if safe else None,
        motion_onset_seconds=safe.motion_onset_seconds if safe else None,
        ceiling_speed=ceiling_speed,
        ceiling_reason=ceiling_reason,
        trials=tuple(trials),
    )
