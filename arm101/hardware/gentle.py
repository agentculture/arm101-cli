"""Load-watch back-off-then-hold compliant move primitive.

Pure helper on top of :mod:`arm101.hardware.bus` and
:mod:`arm101.hardware.motion` — zero third-party dependencies. Where
:func:`arm101.hardware.motion.compliant_move` commands a single gentle move
and trusts the caller to know it is safe, :func:`gentle_move` is for the
"is something in the way?" case: a gripper closing on an unknown object, a
joint sweeping toward a limit it has not been calibrated against, etc. It
steps the goal position in small increments and POLLS ``present_position`` and
``present_load`` throughout the travel; if the load climbs past a threshold
*while the joint has stopped advancing* it treats that as contact, stops
advancing, retreats a bounded number of ticks off the contact point, and
**holds there with torque still enabled** — never a limp release (no final
``enable_torque(False)``) and never a hard freeze exactly at the point of
contact (which would keep pressing).

The polling is the whole point, and it is what the original implementation got
wrong: it read load ~1 ms after each goal write — entirely inside the servo's
~95-127 ms dead window — and terminated when its goal writes ran out rather
than when the arm arrived, so it could (and on hardware did) return in 71 ms
claiming a joint had travelled 400 ticks that it had not yet begun to move.
Every value this module reports about where the arm IS is now read back off the
servo. See :class:`LoadWatch` and :class:`_StallDetector`.

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

import contextlib
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable

from arm101.cli._errors import EXIT_USER_ERROR, CliError
from arm101.hardware.bus import FakeBus, OverloadError, load_magnitude
from arm101.hardware.motion import clamp_goal

if TYPE_CHECKING:
    from arm101.hardware.bus import MotorBus

#: Callback handed every ``(present_position, load_magnitude)`` sample the
#: TRAVEL loop takes — the measurement seam, and the only way to observe a move
#: from the outside without re-implementing it.
#:
#: It exists for :mod:`arm101.hardware.profile`, whose whole job is to certify
#: that the contact rule below **still fires at a given Goal_Speed**. A profiler
#: that ran its own copy of the poll loop would certify the copy, not the code
#: that actually drives the arm — so it drives ``gentle_move`` itself and watches
#: the samples the real :class:`_StallDetector` is fed. From that stream a caller
#: can recover what the result dict cannot express: the motion-onset latency (the
#: ~95-127 ms dead window before the servo responds), the travel rate over the
#: free part of the approach, and the peak load reached on the way in.
#:
#: Only the APPROACH is observed. The post-contact retreat (:func:`_settle_at`)
#: is deliberately not, because a rate computed across it would be a rate for a
#: move that spent ~1.0-1.2 s reversing off an obstacle — an honest number for
#: nothing anyone asked about.
#:
#: The observer MUST NOT raise: it is called from inside a loop that is holding a
#: live, energised joint against an obstacle, and an exception there aborts the
#: move mid-press. It is a measurement seam, not a control seam — it cannot stop,
#: steer, or fail a move, by construction.
TravelObserver = Callable[[int, int], None]

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

#: Public name for the same number, because it is not merely an internal knob —
#: it is a CEILING ON WHAT ANY CONTACT THRESHOLD CAN MEAN, and callers that let a
#: human choose a threshold have to know it.
#:
#: ``present_load`` SATURATES at the servo's ``Torque_Limit`` (measured: a cap of
#: 300 pins load at exactly 300, a cap of 600 pins it at 600), and contact
#: requires ``load > threshold``. Since every move here runs under the cap above,
#: a threshold >= this value can NEVER fire, however hard the arm pushes — the
#: joint would press into a wall while the software reported free air. Any verb
#: exposing a ``--threshold`` must reject values outside ``(0, CONTACT_LOAD_CEILING)``
#: rather than accept an impossibility.
CONTACT_LOAD_CEILING = _CONTACT_TORQUE_LIMIT

# ---------------------------------------------------------------------------
# Poll/stall constants — MEASURED on the follower arm, 2026-07-12. These are
# not guesses; each one has a specific failure mode behind it.
# ---------------------------------------------------------------------------

#: Seconds between position/load samples during a move. The stall rule compares
#: consecutive samples, so the interval must be long enough for a *moving* joint
#: to visibly advance: the slowest joints travel ~150 ticks/s, so at 25 ms they
#: cover ~4 ticks — comfortably over :data:`_DEFAULT_STALL_EPS`. Poll much
#: faster (e.g. at raw bus speed, ~2 ms) and a genuinely moving joint advances
#: <1 tick per sample, reads as "not advancing", and trips a phantom contact.
_DEFAULT_POLL_INTERVAL = 0.025

#: Ticks the joint must be within, AND settled at, to count as arrived. The
#: servo parks a few ticks off its goal, so demanding exactness would spin.
_DEFAULT_ARRIVAL_TOLERANCE = 12

#: Minimum per-sample advance that still counts as "moving".
_DEFAULT_STALL_EPS = 2

#: Consecutive non-advancing samples (~200 ms at the default interval) before a
#: loaded joint is called stalled. Contact is GRADUAL, not instant — the bench
#: recording shows a joint creeping 3022 -> 3001 with its load already past
#: threshold — so a single non-advancing sample is not enough.
_DEFAULT_STALL_SAMPLES = 8

#: The joint must move this far before the stall check ARMS. The servo does not
#: begin moving for ~95-127 ms after a goal write; a stall detector that is live
#: during that dead window reports a phantom contact on EVERY move. Arming on
#: measured motion (rather than a fixed timer) also absorbs the ~1.0-1.2 s the
#: servo takes to reverse off an obstacle.
_DEFAULT_ONSET_TICKS = 6

#: Worst-case observed travel rate (the shoulder joints: ~500 ticks in ~3.3 s).
#: Used to size a move's timeout from its distance.
_MIN_TICKS_PER_SECOND = 120

#: Floor under any computed timeout, covering onset latency plus settle.
_TIMEOUT_FLOOR_SECONDS = 6.0

#: Hard backstop on samples per move, independent of the wall clock. A joint
#: that never advances and never loads up — a dead or disconnected motor, or a
#: simulated bus that does not model travel — would otherwise be watched until
#: the timeout expires. On real hardware the wall-clock deadline always fires
#: first (a full-range move needs ~1400 samples at the default interval), so
#: this only bounds the pathological case.
_MAX_POLLS_PER_MOVE = 4000

_REMEDIATION_ALLOW_MOTION_FLAG = (
    "Pass allow_motion=True to confirm the move should actually execute on the bus."
)


#: The servo's ``Torque_Limit`` register (addr 48) only accepts ``[0, 1000]``.
#: A read OUTSIDE that band is not a torque limit — it is a corrupt packet that
#: happened to report success.
_TORQUE_LIMIT_MAX = 1000


def _sane_torque_limit(value: int) -> "int | None":
    """Return *value* if it could be a real ``Torque_Limit``, else ``None``.

    A read can SUCCEED and still be garbage. Hit on hardware: ``read_torque_limit``
    returned **2048** on a healthy motor whose register actually held 500. The
    value sailed through (the bus layer retries FAILED reads; it cannot know a
    successful one is nonsense), was stashed as the pre-move value, and then the
    ``finally`` tried to write it back — where it was rejected as out of range,
    raising a ``CliError`` OUT OF THE CLEANUP and masking whatever the move had
    actually been doing.

    So: validate at the point of READ, not at the point of write. If the value
    cannot be a torque limit we do not know the pre-move one, and we say so by
    returning ``None`` — the ``finally`` then leaves the conservative
    :data:`_CONTACT_TORQUE_LIMIT` cap in place rather than restoring a fiction.
    A joint left slightly under-torqued is a nuisance; a cleanup that raises is a
    lie about why the move failed.

    A plausible-looking wrong value is more dangerous than an error, because
    nothing downstream can tell it from a real reading. This is the same class of
    fault as a ``read_position`` returning 0 immediately after an EEPROM write.
    """
    if 0 <= value <= _TORQUE_LIMIT_MAX:
        return value
    return None


def _require_positive(value: float, name: str, example: str) -> None:
    """Raise :class:`CliError` unless *value* is strictly positive."""
    if value <= 0:
        raise CliError(
            code=EXIT_USER_ERROR,
            message=f"{name} must be greater than zero, got {value}",
            remediation=f"Pass a positive {name} ({example}).",
        )


def _require_non_negative(value: float, name: str, example: str) -> None:
    """Raise :class:`CliError` unless *value* is zero or positive."""
    if value < 0:
        raise CliError(
            code=EXIT_USER_ERROR,
            message=f"{name} must not be negative, got {value}",
            remediation=f"Pass a {name} >= 0 ({example}).",
        )


@dataclass(frozen=True)
class LoadWatch:
    """How the travel loop *watches* the joint, as opposed to how it commands it.

    One cohesive knob-set, grouped so :func:`gentle_move` keeps a readable
    signature: these six all describe the poll/stall rule, and a caller tuning
    one almost always cares about its neighbours. Every default is MEASURED on
    the follower arm (see the module constants) — none is a guess. The defaults
    are good; overriding is for a bench probe or a test that needs the loop to
    give up quickly.

    Attributes
    ----------
    poll_interval:
        Seconds between position/load samples. See
        :data:`_DEFAULT_POLL_INTERVAL` — sampling *faster* than the joint moves
        makes a moving joint read as stalled.
    timeout:
        Wall-clock budget for the move. ``None`` (the default) sizes it from the
        travel distance via :func:`_travel_timeout`.
    arrival_tolerance:
        Ticks the joint must be within, and settled at, to count as arrived.
    stall_eps:
        Minimum per-sample advance that still counts as "moving".
    stall_samples:
        Consecutive non-advancing samples before a loaded joint is called stalled.
    onset_ticks:
        Ticks the joint must move before the stall check arms.
    """

    poll_interval: float = _DEFAULT_POLL_INTERVAL
    timeout: float | None = None
    arrival_tolerance: int = _DEFAULT_ARRIVAL_TOLERANCE
    stall_eps: int = _DEFAULT_STALL_EPS
    stall_samples: int = _DEFAULT_STALL_SAMPLES
    onset_ticks: int = _DEFAULT_ONSET_TICKS

    def __post_init__(self) -> None:
        """Reject a watch that would disable the very detection it configures.

        Validated at CONSTRUCTION so an invalid watch cannot exist, rather than
        in :func:`gentle_move` where a bad value would already be on its way to
        the arm. Two of these are safety-critical, not merely hygienic:

        * ``stall_eps <= 0`` makes ``advanced < stall_eps`` unsatisfiable, so the
          joint can NEVER be seen as stalled and contact detection is silently
          switched off — the arm would push until the Torque_Limit cap.
        * ``stall_samples < 1`` removes the stall gate entirely, so a load spike
          from mere acceleration reads as contact — the false-positive the gate
          exists to prevent.

        ``poll_interval <= 0`` is the original bug in miniature: sampling faster
        than the joint moves makes a moving joint look stationary.
        """
        _require_positive(self.poll_interval, "poll_interval", "e.g. the default 0.025 seconds")
        _require_positive(self.stall_eps, "stall_eps", "e.g. the default 2 ticks")
        _require_positive(self.stall_samples, "stall_samples", "e.g. the default 8 samples")
        _require_non_negative(self.arrival_tolerance, "arrival_tolerance", "e.g. the default 12")
        _require_non_negative(self.onset_ticks, "onset_ticks", "e.g. the default 6 ticks")
        if self.timeout is not None:
            _require_positive(self.timeout, "timeout", "e.g. 6.0 seconds, or None to size it")


#: The measured defaults, shared by every caller that does not tune the watch.
DEFAULT_LOAD_WATCH = LoadWatch()


@dataclass
class _TravelOutcome:
    """What the travel loop observed. ``position`` is always a READ-BACK tick."""

    position: int
    contacted: bool = False
    contact_position: int | None = None
    contact_load: int | None = None
    retreat_position: int | None = None


def _travel_timeout(distance: int) -> float:
    """Wall-clock budget for a *distance*-tick move, from the measured worst rate."""
    return max(_TIMEOUT_FLOOR_SECONDS, 2.0 + 2.0 * distance / _MIN_TICKS_PER_SECOND)


class _StallDetector:
    """Tracks whether a joint has stopped advancing *while under load*.

    Deliberately two-phase. The servo does not begin moving for ~95-127 ms after
    a goal write, so a detector that is live during that dead window reports a
    phantom contact on EVERY move (this really happened to a profiling run). It
    therefore ARMS only once the joint has measurably moved — or is already
    pushing harder than *threshold*, which covers the joint that was jammed
    before the move began and so never reaches onset at all.

    Arming on measured motion rather than a fixed timer also absorbs the
    ~1.0-1.2 s the servo takes to reverse off an obstacle.
    """

    def __init__(self, start: int, *, threshold: int, watch: LoadWatch) -> None:
        self._start = start
        self._previous = start
        self._threshold = threshold
        self._watch = watch
        self._moving = False
        #: Consecutive armed samples on which the joint failed to advance.
        self.stalled = 0
        #: Ticks advanced on the most recent sample.
        self.advanced = 0

    def update(self, position: int, load: int) -> None:
        """Fold one (position, load) sample into the moving/stalled state."""
        self.advanced = abs(position - self._previous)
        if not self._moving and abs(position - self._start) >= self._watch.onset_ticks:
            self._moving = True  # the servo has finally responded
        armed = self._moving or load > self._threshold
        stuck = armed and self.advanced < self._watch.stall_eps
        self.stalled = self.stalled + 1 if stuck else 0
        self._previous = position

    def is_contact(self, load: int) -> bool:
        """CONTACT = pushing hard AND no longer advancing.

        The load gate alone is not enough: a joint merely ACCELERATING through
        free air peaks at 300 on wrist_roll, above its own threshold. The stall
        gate is what tells "blocked" apart from "accelerating".
        """
        return load > self._threshold and self.stalled >= self._watch.stall_samples

    def has_arrived(self, position: int, target: int) -> bool:
        """ARRIVAL = measured, and settled.

        Both halves matter: within tolerance but still coasting is not yet arrived.
        """
        within = abs(position - target) <= self._watch.arrival_tolerance
        return within and self.advanced < self._watch.stall_eps


def _needs_pacing(bus: "MotorBus") -> bool:
    """Does this bus need real time to pass between samples?

    A physical servo does: it advances by wall-clock, so the loop must wait
    between reads or it samples the same tick repeatedly. A simulated bus
    advances *per read*, so pacing it would only make the suite sleep.
    """
    return not isinstance(bus, FakeBus)


def _require_gentle_args(allow_motion: bool, step: int, backoff: int) -> None:
    """Validate ``gentle_move``'s motion gate and step/backoff bounds.

    Raises :class:`CliError` (``EXIT_USER_ERROR``) on any violation; returns
    ``None`` when every argument is acceptable. Factored out of
    :func:`gentle_move` so the entry point stays under the cognitive-complexity
    budget — the raises are identical to the inline guards they replace.
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


def _step_direction(start: int, target: int) -> int:
    """Return ``+1``/``-1``/``0`` for the direction from *start* toward *target*."""
    if target > start:
        return 1
    if target < start:
        return -1
    return 0


def _pace_for(bus: "MotorBus", watch: LoadWatch) -> float:
    """Seconds to wait between samples on *bus* — zero for a bus that needs no wall clock."""
    return watch.poll_interval if _needs_pacing(bus) else 0.0


def _advance_goal(
    bus: "MotorBus",
    motor: int,
    *,
    goal_written: int | None,
    start: int,
    direction: int,
    step: int,
    clamped_target: int,
    min_angle: int,
    max_angle: int,
) -> int:
    """Write the next *step*-tick goal (if it changed) and return it.

    The goal is stepped from the PREVIOUS GOAL, not tethered to the measured
    position. Tethering was tried and is wrong: it pins the servo's position
    error at *step*, which caps its torque, and a gravity-loaded joint
    (shoulder_lift, elbow_flex) then cannot break away at all — it stalls in
    OPEN SPACE and reads exactly like a contact. Measured: tethered at 25 ticks
    every joint stalls at a load of ~208, and shoulder_lift's usable band
    collapses to (188, 208).

    Pressing force does not need the tether to be bounded: it is already
    bounded, by the :data:`_CONTACT_TORQUE_LIMIT` cap held for the duration of
    the move. That is the hardware-proven safety, and it is what ``present_load``
    saturates against on a real contact.

    Once the goal reaches the clamped target it stops changing, and no
    redundant write is issued.
    """
    goal = (goal_written if goal_written is not None else start) + direction * step
    goal = min(goal, clamped_target) if direction > 0 else max(goal, clamped_target)
    goal, _ = clamp_goal(goal, min_angle, max_angle)
    if goal != goal_written:
        bus.write_goal_position(motor, goal)
    return goal


def _sample(bus: "MotorBus", motor: int, pace: float) -> tuple[int, int]:
    """Wait one poll interval, then read back ``(present_position, load_magnitude)``.

    The wait is the whole point: the pre-fix loop read load ~1 ms after the goal
    write, entirely inside the servo's ~95-127 ms dead window, so it never once
    saw the load the move actually produced.

    ``present_load`` carries the load DIRECTION in bit 10 (0x400), so the
    magnitude is what gets compared — a load pointing the "negative" way reads
    as >=1024 and would trip a spurious contact.
    """
    if pace:
        time.sleep(pace)
    info = bus.read_info(motor)
    return info["present_position"], load_magnitude(info["present_load"])


def _settle_at(
    bus: "MotorBus",
    motor: int,
    goal: int,
    *,
    pace: float,
    tolerance: int,
    stall_eps: int,
) -> int:
    """Poll until the joint has actually settled at *goal*; return where it IS.

    Used for the post-contact retreat. The goal has already been written — this
    only *watches*, issuing no further writes, so the retreat stays the last
    thing commanded. It has to wait: coming off an obstacle the servo takes
    ~1.0-1.2 s just to start reversing, so returning immediately would report
    the joint still pressed against the thing it was supposed to back away from.
    """
    previous = bus.read_info(motor)["present_position"]
    deadline = time.monotonic() + _travel_timeout(abs(goal - previous))
    polls = 0

    while polls < _MAX_POLLS_PER_MOVE and time.monotonic() <= deadline:
        polls += 1
        if pace:
            time.sleep(pace)
        position = bus.read_info(motor)["present_position"]
        settled = abs(position - previous) < stall_eps
        previous = position
        if abs(position - goal) <= tolerance and settled:
            return position
    return previous


def _retreat_from(
    bus: "MotorBus",
    motor: int,
    *,
    position: int,
    load: int,
    direction: int,
    backoff: int,
    min_angle: int,
    max_angle: int,
    watch: LoadWatch,
    pace: float,
) -> _TravelOutcome:
    """Back *backoff* ticks off a contact at *position* and hold there.

    Torque stays enabled, so the joint HOLDS off the obstruction rather than
    going limp (which would drop whatever it is holding) or freezing exactly at
    the contact point (which would keep pressing).
    """
    retreat_position, _ = clamp_goal(position - direction * backoff, min_angle, max_angle)
    bus.write_goal_position(motor, retreat_position)
    settled = _settle_at(
        bus,
        motor,
        retreat_position,
        pace=pace,
        tolerance=watch.arrival_tolerance,
        stall_eps=watch.stall_eps,
    )
    return _TravelOutcome(
        position=settled,
        contacted=True,
        contact_position=position,
        contact_load=load,
        retreat_position=retreat_position,
    )


def _travel(
    bus: "MotorBus",
    motor: int,
    *,
    start: int,
    clamped_target: int,
    direction: int,
    min_angle: int,
    max_angle: int,
    threshold: int,
    step: int,
    backoff: int,
    watch: LoadWatch,
    observer: "TravelObserver | None" = None,
) -> _TravelOutcome:
    """Step the goal toward *clamped_target*, watching the joint, until it stops.

    The loop ends on exactly one of three MEASURED conditions — contact, arrival,
    or timeout — and every one of them reports a position read back off the servo.
    The pre-fix loop instead ended when its goal-writes ran out and *asserted* the
    target had been reached, which is how a move that never happened could claim
    it had arrived.

    *observer*, when given, is handed each sample the instant it is read and
    BEFORE the detector folds it in — so an observer sees exactly the stream the
    contact rule sees, no more and no less. See :data:`TravelObserver`.
    """
    pace = _pace_for(bus, watch)
    budget = (
        watch.timeout if watch.timeout is not None else _travel_timeout(abs(clamped_target - start))
    )
    deadline = time.monotonic() + budget

    detector = _StallDetector(start, threshold=threshold, watch=watch)
    goal_written: int | None = None
    position = start
    polls = 0

    while polls < _MAX_POLLS_PER_MOVE:
        polls += 1
        goal_written = _advance_goal(
            bus,
            motor,
            goal_written=goal_written,
            start=start,
            direction=direction,
            step=step,
            clamped_target=clamped_target,
            min_angle=min_angle,
            max_angle=max_angle,
        )
        position, load = _sample(bus, motor, pace)
        if observer is not None:
            observer(position, load)
        detector.update(position, load)

        if detector.is_contact(load):
            return _retreat_from(
                bus,
                motor,
                position=position,
                load=load,
                direction=direction,
                backoff=backoff,
                min_angle=min_angle,
                max_angle=max_angle,
                watch=watch,
                pace=pace,
            )
        if detector.has_arrived(position, clamped_target):
            return _TravelOutcome(position=position)
        if time.monotonic() > deadline:
            return _TravelOutcome(position=position)

    return _TravelOutcome(position=position)


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
    watch: LoadWatch = DEFAULT_LOAD_WATCH,
    observer: "TravelObserver | None" = None,
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
    5. Throughout the travel the loop POLLS ``present_position`` and
       ``present_load`` (every ``watch.poll_interval`` seconds) and ends on one
       of exactly three **measured** conditions — never on "the goal writes ran
       out", which is what the pre-fix loop did:

       * **contact** — ``present_load`` exceeds *threshold* AND the joint has
         stopped advancing (the stall rule; see :class:`_StallDetector`). Load
         alone is not sufficient: a joint accelerating through free air can peak
         above its own threshold. On contact the goal is written once more to a
         retreat position *backoff* ticks back along the direction of travel
         (clamped to bounds), and the loop waits for the joint to actually get
         there — torque stays enabled, so it holds off the obstruction rather
         than going limp or freezing at the point of contact, still pressing.
       * **arrival** — the joint is measured within ``watch.arrival_tolerance``
         of the clamped target and has settled there. The motor simply holds
         (torque is already on from step 3; no extra write needed).
       * **timeout** — the wall-clock budget expires (see :class:`LoadWatch`).

    6. ``final_position`` is whatever was last READ BACK off the servo, on every
       path. It is never the commanded tick.
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
    watch:
        How the travel loop watches the joint — poll interval, timeout, arrival
        tolerance, and the stall rule. See :class:`LoadWatch`; the default
        :data:`DEFAULT_LOAD_WATCH` carries the hardware-measured values and is
        what every caller should use unless it is a bench probe or a test that
        needs the loop to give up quickly.
    observer:
        Optional measurement seam (see :data:`TravelObserver`): called with each
        ``(present_position, load_magnitude)`` sample of the APPROACH, in order,
        as it is read. Purely passive — it cannot stop, steer, or fail a move,
        and passing ``None`` (the default) leaves this function byte-for-byte the
        move it was without it. :mod:`arm101.hardware.profile` uses it to time
        the motion onset and the travel rate of the very move whose contact
        detection it is certifying.

    Returns
    -------
    dict[str, object]
        ``{"motor", "requested_target", "clamped_target", "was_clamped",
        "start_position", "threshold", "step", "backoff_ticks",
        "acceleration", "speed", "contacted", "contact_position",
        "contact_load", "retreat_position", "final_position",
        "overloaded"}``. When ``contacted`` is ``False``,
        ``contact_position``/``contact_load``/``retreat_position`` are
        ``None``. ``final_position`` is always a value READ BACK off the servo —
        where the joint actually IS, which on the no-contact path is *near*
        ``clamped_target`` (within ``watch.arrival_tolerance``) but is not
        assumed to equal it. The pre-fix code reported ``clamped_target`` here
        unconditionally, which is how a move that never happened could claim it
        had arrived.
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
    _require_gentle_args(allow_motion, step, backoff)

    clamped_target, was_clamped = clamp_goal(target, min_angle, max_angle)

    start_position: int | None = None
    current: int | None = None
    contacted = False
    contact_position: int | None = None
    contact_load: int | None = None
    retreat_position: int | None = None
    overloaded = False
    # None until successfully read; guards the finally-restore so an overload
    # raised DURING the cap read/write can't reference an unset value.
    original_torque_limit: int | None = None

    try:
        # Cap the servo's own Torque_Limit for the duration of the move (see
        # _CONTACT_TORQUE_LIMIT) — a hardware backstop underneath the
        # present_load-threshold check below. INSIDE the try so a pre-latched
        # overload on the read/write itself is caught and reported
        # (overloaded=True), not propagated. Read the pre-move value first so
        # the `finally` can restore it however the move ends.
        original_torque_limit = _sane_torque_limit(bus.read_torque_limit(motor))
        bus.write_torque_limit(motor, _CONTACT_TORQUE_LIMIT)

        bus.write_acceleration(motor, acceleration)
        bus.write_goal_speed(motor, speed)
        bus.enable_torque(motor, True)

        start_position = bus.read_info(motor)["present_position"]
        current = start_position

        direction = _step_direction(start_position, clamped_target)

        if direction != 0:
            outcome = _travel(
                bus,
                motor,
                start=start_position,
                clamped_target=clamped_target,
                direction=direction,
                min_angle=min_angle,
                max_angle=max_angle,
                threshold=threshold,
                step=step,
                backoff=backoff,
                watch=watch,
                observer=observer,
            )
            current = outcome.position  # MEASURED — never the commanded tick
            contacted = outcome.contacted
            contact_position = outcome.contact_position
            contact_load = outcome.contact_load
            retreat_position = outcome.retreat_position
    except OverloadError:
        # The servo's OWN overload latch tripped — distinct from the
        # present_load-threshold contact check above. Stop advancing,
        # recover (release torque, clearing the latch), and report this via
        # the result dict rather than letting the exception propagate.
        overloaded = True
        bus.clear_overload(motor)
        # An ALREADY-latched motor raises on the very first bus call, i.e.
        # before `current` was ever read — which would leave this reporting a
        # `final_position` of None for a joint that is sitting at a perfectly
        # readable position. The latch is cleared now, so ask the servo where it
        # is. Best-effort by design: if the bus is still unhappy the value stays
        # None, because the one thing this module must never do is INVENT a
        # position it did not measure — that was the whole bug.
        if current is None:
            with contextlib.suppress(OverloadError, CliError):
                current = bus.read_info(motor)["present_position"]
            if start_position is None:
                start_position = current
    finally:
        # Restore the pre-move Torque_Limit if it was captured. On the overload
        # path clear_overload() already cleared the latch so this lands
        # normally; keep it best-effort so a lingering fault can't turn a
        # reported overload back into a raised exception.
        # NEVER raise from here. A cleanup that throws replaces the failure the
        # operator actually needs to see — the same trap already closed in
        # safety._release_motor and profile._release. It bit here for real: a
        # corrupt read of 2048 made this write raise CliError out of the finally.
        # `original_torque_limit` is None when the read was garbage (see
        # _sane_torque_limit), and then the conservative cap simply stays in
        # place, which is the safe direction to fail in.
        if original_torque_limit is not None:
            try:
                bus.write_torque_limit(motor, original_torque_limit)
            except SystemExit:
                raise
            except BaseException:  # noqa: B036 - cleanup must outlive any bus failure
                pass

    # Where the joint IS, read off the servo — never where it was told to go.
    # The pre-fix code reported `clamped_target` here on the no-contact path,
    # which is how a move that never happened could claim it had arrived.
    final_position = current

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
