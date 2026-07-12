"""Tests for :mod:`arm101.hardware.profile` — speed profiling.

The thesis under test, stated once so every assertion below can be read against
it:

    **A speed is only safe if CONTACT DETECTION STILL WORKS AT IT.** A speed the
    servo tolerates, but at which the stall rule can no longer tell "blocked" from
    "accelerating", is a FAILURE of that speed. Validating a candidate on free
    motion alone does not count.

So the load-bearing tests here are not "does it move faster" but:

* :func:`test_speed_where_contact_is_missed_is_rejected` — the single most
  important test in the module. At a speed where the joint drives into a real
  obstacle and the stall rule DOES NOT FIRE, the speed is rejected and the last
  good speed is what gets reported.
* :func:`test_first_probe_against_a_reachable_target_is_a_void_run` — the same
  thesis from the other side: a probe that never met resistance certifies
  nothing, and the run is void rather than quietly "successful".

Everything is driven against :class:`tests._fakes.ServoModelBus`, which models
the servo honestly (travel takes polls; load saturates at ``Torque_Limit``;
obstacles are compliant and the joint creeps into them), plus two small doubles
below that make the servo's behaviour depend on the COMMANDED GOAL_SPEED — which
is the entire coupling this module exists to measure.
"""

from __future__ import annotations

import contextlib

import pytest

from arm101.cli._errors import EXIT_USER_ERROR, CliError
from arm101.hardware import profile as profile_mod
from arm101.hardware.bus import OverloadError
from arm101.hardware.gentle import DEFAULT_LOAD_WATCH
from arm101.hardware.profile import (
    DEFAULT_SPEED_START,
    MAX_SPEED,
    REASON_CONTACT_DETECTED,
    REASON_CONTACT_MISSED,
    REASON_NO_CONTACT,
    REASON_OVERLOAD,
    JointSpeedProfile,
    SpeedTrial,
    _Trace,
    profile_joint,
    speed_ladder,
)

from ._fakes import ServoModelBus

# ---------------------------------------------------------------------------
# Geometry shared by the ramp tests
#
# home 2000 -> obstacle 2100 -> contact target 2500. A 500-tick commanded move,
# matching the bench figure the module is calibrated against ("a 500-tick move
# takes ~930 ms on wrist_roll but ~3300 ms on the shoulders").
# ---------------------------------------------------------------------------

MOTOR = 1
HOME = 2000
OBSTACLE = 2100
CONTACT_TARGET = 2500
THRESHOLD = 250  # shoulder_pan's hardware-tuned default; sits under the 500 cap


class _SpeedCoupledArm(ServoModelBus):
    """A servo whose TRAVEL RATE and CONTACT BEHAVIOUR both follow ``Goal_Speed``.

    This is the coupling :mod:`arm101.hardware.profile` exists to find, modelled
    honestly enough to be worth testing against:

    * **Travel rate scales with speed.** ``ticks_per_poll`` is derived from the
      commanded Goal_Speed, calibrated so the ladder's first rung (150, the only
      speed with hardware evidence behind it) reproduces the measured ~4 ticks per
      25 ms sample.
    * **Above ``detection_ceiling``, contact stops being detectable.** The joint
      now arrives at the obstacle with enough drive to keep COMPRESSING it instead
      of stopping dead against it: the contact goes soft (``obstacle_stiffness``
      collapses), so the servo's 500-unit torque budget buys it hundreds of ticks
      of penetration rather than ~25. Load is saturated the whole way in — the
      joint is unmistakably pushing against something — but it never stops
      advancing, so the stall rule's counter is reset on every single sample and
      ``is_contact`` can never fire. The joint bulldozes through to the commanded
      target under load, and the software never notices it hit anything.

    That is precisely "the stall rule can no longer tell blocked from
    accelerating", and it is what a candidate speed must be rejected for.
    """

    #: Ticks per 25 ms poll, per unit of Goal_Speed. 150 -> ~4 ticks/poll, the
    #: measured rate for the slowest joints at gentle_move's default speed.
    TICKS_PER_POLL_PER_SPEED = 4 / 150

    def __init__(self, *args, detection_ceiling: int = 250, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._detection_ceiling = detection_ceiling

    def write_goal_speed(self, motor: int, value: int) -> None:
        super().write_goal_speed(motor, value)
        self.ticks_per_poll = max(1, round(value * self.TICKS_PER_POLL_PER_SPEED))
        # Stiffness 20 -> the joint stalls ~25 ticks into the obstacle at the 500
        # Torque_Limit cap (a hard, detectable stop). Stiffness 1 -> it penetrates
        # ~500 ticks, creeping the whole way, and never reads as stopped.
        self.obstacle_stiffness = 20 if value <= self._detection_ceiling else 1


class _OverloadAtSpeedArm(ServoModelBus):
    """A servo that LATCHES an overload on contact once driven above a speed.

    The other half of the measured hardware behaviour: the STS3215 throws a
    dynamic overload (status error bit 5, ``error=32``) one-shot at speed 400 or on
    a rigid stop. When the impact is hard enough, the servo's own protection cuts
    torque *before* the software stall rule has accumulated its 8 consecutive
    stalled samples (~200 ms) — the hardware beats the software to the punch, and
    the contact is survived rather than detected.

    Modelled where it actually happens: the moment the joint is pressed into the
    obstacle while the commanded speed is over the limit, every register read
    raises. ``clear_overload`` (which ``gentle_move`` calls to recover) is exempt,
    exactly as on the real bus.
    """

    def __init__(self, *args, overload_above: int = 250, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._overload_above = overload_above
        self._commanded_speed = 0

    def write_goal_speed(self, motor: int, value: int) -> None:
        super().write_goal_speed(motor, value)
        self._commanded_speed = value
        self.ticks_per_poll = max(1, round(value * _SpeedCoupledArm.TICKS_PER_POLL_PER_SPEED))

    def read_info(self, motor: int) -> dict:
        snapshot = super().read_info(motor)
        pressing = self._penetration(motor, snapshot["present_position"]) > 0
        if pressing and self._commanded_speed > self._overload_above:
            raise OverloadError(
                motor=motor,
                error_byte=32,
                message=(
                    f"simulated dynamic overload on motor {motor} at "
                    f"speed {self._commanded_speed}"
                ),
            )
        return snapshot


class _FakeClock:
    """A clock that advances one poll interval per read — simulated time.

    :class:`ServoModelBus` advances the shaft once per ``read_info``, so one poll
    IS one poll interval of time. The real ``time.monotonic`` against a fake bus
    would report ~0 s elapsed and make every rate unmeasurable; this makes the
    timings come out exactly as they would on hardware moving at the modelled rate.
    """

    def __init__(self, tick: float = DEFAULT_LOAD_WATCH.poll_interval) -> None:
        self._tick = tick
        self.now = 0.0

    def __call__(self) -> float:
        value = self.now
        self.now += self._tick
        return value


def _arm(cls=_SpeedCoupledArm, **kwargs) -> ServoModelBus:
    """An opened fake arm with the shared geometry: joint at HOME, obstacle at OBSTACLE."""
    bus = cls(positions={MOTOR: HOME}, **kwargs)
    bus.open()
    bus.place_obstacle(MOTOR, OBSTACLE)
    return bus


def _run(bus, ladder, **kwargs) -> JointSpeedProfile:
    """Profile the shared geometry over *ladder*."""
    return profile_joint(
        bus,
        MOTOR,
        joint="shoulder_pan",
        contact_target=CONTACT_TARGET,
        min_angle=0,
        max_angle=4095,
        threshold=THRESHOLD,
        ladder=ladder,
        allow_motion=True,
        clock=_FakeClock(),
        **kwargs,
    )


# ===========================================================================
# The ladder
# ===========================================================================


def test_speed_ladder_defaults_start_at_the_only_proven_speed() -> None:
    ladder = speed_ladder()
    assert ladder[0] == DEFAULT_SPEED_START == 150
    assert ladder == tuple(range(150, 601, 50))
    # It must bracket the speed at which a one-shot overload was measured (400).
    assert max(ladder) > 400


def test_speed_ladder_is_ascending_and_bounded() -> None:
    assert speed_ladder(100, 100, 350) == (100, 200, 300)


def test_speed_ladder_never_returns_empty() -> None:
    """An empty ladder would certify nothing while looking like a successful run."""
    assert speed_ladder(400, 50, 100) == (400,)


@pytest.mark.parametrize("start,step,stop", [(0, 50, 600), (150, 0, 600), (150, 50, MAX_SPEED + 1)])
def test_speed_ladder_rejects_out_of_range_values(start, step, stop) -> None:
    with pytest.raises(CliError) as exc:
        speed_ladder(start, step, stop)
    assert exc.value.code == EXIT_USER_ERROR


# ===========================================================================
# The measurement seam (_Trace) — pure, exact
# ===========================================================================


def test_trace_measures_onset_rate_and_peak_load() -> None:
    clock = _FakeClock(tick=0.025)
    trace = _Trace(2000, onset_ticks=6, clock=clock)  # t0 = 0.000

    trace.observe(2000, 10)  # t=0.025 — dead window: the servo has not moved yet
    trace.observe(2002, 20)  # t=0.050 — still under onset_ticks
    trace.observe(2010, 30)  # t=0.075 — 10 ticks moved: ONSET
    trace.observe(2050, 480)  # t=0.100 — 50 ticks from start, peak load

    assert trace.onset_seconds == pytest.approx(0.075)
    assert trace.distance_ticks == 50
    assert trace.travel_seconds == pytest.approx(0.100)
    assert trace.ticks_per_second == pytest.approx(500.0)
    assert trace.peak_load == 480
    assert len(trace.samples) == 4


def test_trace_reports_an_unmeasurable_rate_as_none_not_a_fabricated_number() -> None:
    """A trace whose samples were never PACED carries no timing — say so.

    A fake bus advances per read, not per second, so on the real monotonic clock
    every sample of a move lands within microseconds of the last. Dividing ticks by
    that yields millions of ticks/second: a property of the test harness wearing a
    measurement's clothes. This module exists because motion constants were guessed
    once and never checked; it may not hand back a fabricated one.
    """
    ticks = iter([0.0, 0.000_01, 0.000_02])  # a whole "move" inside 20 microseconds
    trace = _Trace(2000, onset_ticks=6, clock=lambda: next(ticks))
    trace.observe(2050, 40)
    trace.observe(2100, 40)

    assert trace.paced is False
    assert trace.travel_seconds is None
    assert trace.ticks_per_second is None
    assert trace.onset_seconds is None
    # ...but the LOAD is still real: it is read off the servo, not off the clock.
    assert trace.peak_load == 40
    assert trace.distance_ticks == 100


def test_trace_with_no_samples_measures_nothing() -> None:
    trace = _Trace(2000, onset_ticks=6, clock=_FakeClock())
    assert trace.distance_ticks is None
    assert trace.ticks_per_second is None
    assert trace.onset_seconds is None


def test_gentle_move_observer_sees_exactly_the_detector_stream() -> None:
    """The seam profile.py relies on: every approach sample, in order, as read."""
    from arm101.hardware.gentle import gentle_move

    bus = _arm()
    seen: list[tuple[int, int]] = []

    result = gentle_move(
        bus,
        MOTOR,
        CONTACT_TARGET,
        min_angle=0,
        max_angle=4095,
        threshold=THRESHOLD,
        speed=150,
        allow_motion=True,
        observer=lambda position, load: seen.append((position, load)),
    )

    assert seen, "the observer must be called for every sample of the approach"
    polled = list(zip(bus.polled_positions(MOTOR), bus.polled_loads(MOTOR)))
    # Every observed sample is a real bus poll, in order — nothing invented. Poll 0
    # is gentle_move's own start-position read, which happens before the travel
    # loop and so is (correctly) not part of the approach.
    assert seen == polled[1 : 1 + len(seen)]
    # The approach ends at the contact; the retreat that follows is NOT observed,
    # so the trailing settle polls are in `polled` but not in `seen`.
    assert result["contacted"] is True
    assert seen[-1][0] == result["contact_position"]
    assert len(seen) < len(polled), "the retreat's settle polls must not be observed"


# ===========================================================================
# The happy path — a speed at which contact IS still detected
# ===========================================================================


def test_speed_where_contact_is_detected_is_accepted() -> None:
    bus = _arm(detection_ceiling=1000)  # detection holds across the whole ladder
    prof = _run(bus, ladder=(150, 200, 250))

    assert prof.certified is True
    assert prof.safe_speed == 250, "the highest rung that still detected contact"
    assert prof.ceiling_speed is None, "nothing was rejected — the wall is above the ladder"
    assert [t.reason for t in prof.trials] == [REASON_CONTACT_DETECTED] * 3
    assert all(t.contacted for t in prof.trials)


def test_the_profile_measures_rate_and_onset_at_the_safe_speed() -> None:
    bus = _arm(detection_ceiling=1000)
    prof = _run(bus, ladder=(150, 300))

    assert prof.safe_speed == 300
    # Measured, not assumed: the reported numbers are the SAFE speed's, not the
    # ladder's first rung's, and not an average.
    fastest = prof.trials[-1]
    assert prof.ticks_per_second == fastest.ticks_per_second
    assert prof.motion_onset_seconds == fastest.motion_onset_seconds
    assert prof.ticks_per_second is not None and prof.ticks_per_second > 0
    assert prof.motion_onset_seconds is not None and prof.motion_onset_seconds > 0
    # The arm really does travel faster when told to, and the profile sees it.
    assert prof.trials[1].ticks_per_second > prof.trials[0].ticks_per_second


def test_a_detected_contact_reports_where_and_how_hard() -> None:
    bus = _arm(detection_ceiling=1000)
    prof = _run(bus, ladder=(150,))

    trial = prof.trials[0]
    assert trial.contacted is True
    assert trial.arrived is False, "it must NOT have reached the target — something stopped it"
    assert trial.contact_position is not None
    assert OBSTACLE <= trial.contact_position < CONTACT_TARGET
    # present_load saturates at gentle_move's Torque_Limit cap of 500.
    assert trial.contact_load == 500
    assert trial.peak_load == 500


# ===========================================================================
# THE CRUX — a speed the servo survives but at which contact is NOT detected
# ===========================================================================


def test_speed_where_contact_is_missed_is_rejected() -> None:
    """THE most important test here.

    At 300 the joint drives into a REAL obstacle, loads past its contact threshold
    all the way in — it is unmistakably pushing against something — and the stall
    rule never fires, because it never stops advancing. The servo survives it
    perfectly happily. That is a FAILURE of the speed, not a pass: the profile must
    reject 300 and report 150, the last speed at which the detector actually
    worked.
    """
    bus = _arm(detection_ceiling=250)
    prof = _run(bus, ladder=(150, 300))

    good, bad = prof.trials

    # 150: the detector still works.
    assert good.speed == 150
    assert good.accepted is True
    assert good.reason == REASON_CONTACT_DETECTED

    # 300: the arm moved, and moved FASTER — a "does it survive?" test would pass it.
    assert bad.speed == 300
    assert bad.overloaded is False, "the servo tolerated this speed perfectly well"
    assert bad.peak_load > THRESHOLD, "the joint demonstrably met the obstacle"
    # ...and yet:
    assert bad.contacted is False, "the stall rule never fired"
    assert bad.accepted is False
    assert bad.reason == REASON_CONTACT_MISSED

    # The verdict that matters.
    assert prof.safe_speed == 150, "the last speed at which contact was still DETECTED"
    assert prof.ceiling_speed == 300
    assert prof.ceiling_reason == REASON_CONTACT_MISSED


def test_the_ramp_stops_at_the_first_rejection() -> None:
    """No probing above a speed already known to miss contacts."""
    bus = _arm(detection_ceiling=250)
    prof = _run(bus, ladder=(150, 300, 450, 600))

    assert [t.speed for t in prof.trials] == [150, 300], "450 and 600 were never attempted"
    assert prof.safe_speed == 150


def test_free_motion_alone_never_certifies_a_speed() -> None:
    """The thesis, asserted directly: an arrival with no load proves nothing."""
    bus = _arm(detection_ceiling=250)
    prof = _run(bus, ladder=(150, 300))

    rejected = prof.trials[1]
    # It ARRIVED at the commanded target — a free-motion check would call that a
    # clean, successful, fast move. The profile calls it a failure.
    assert rejected.arrived is True
    assert rejected.accepted is False


# ===========================================================================
# The overload ceiling
# ===========================================================================


def test_overload_during_the_ramp_is_a_recorded_ceiling_not_a_crash() -> None:
    """The servo's own latch beats the stall rule: survived, not detected."""
    bus = _arm(cls=_OverloadAtSpeedArm, overload_above=250)
    prof = _run(bus, ladder=(150, 300))

    good, bad = prof.trials
    assert good.reason == REASON_CONTACT_DETECTED

    assert bad.speed == 300
    assert bad.overloaded is True
    assert bad.contacted is False, "the hardware cut torque before the rule could fire"
    assert bad.accepted is False
    assert bad.reason == REASON_OVERLOAD

    assert prof.safe_speed == 150
    assert prof.ceiling_speed == 300
    assert prof.ceiling_reason == REASON_OVERLOAD


def test_overload_leaves_the_joint_de_energised() -> None:
    """An overload must not walk away from a hot motor pressed into a wall."""
    bus = _arm(cls=_OverloadAtSpeedArm, overload_above=250)
    _run(bus, ladder=(150, 300))

    torque_for_motor = [w for w in bus.torque_writes if w["motor"] == MOTOR]
    assert torque_for_motor[-1]["on"] is False, "the run must end with the joint limp"


def test_an_overload_on_the_very_first_rung_certifies_nothing() -> None:
    """No safe speed exists — and the profile does NOT fall back to a guess."""
    bus = _arm(cls=_OverloadAtSpeedArm, overload_above=100)
    prof = _run(bus, ladder=(150, 300))

    assert prof.certified is False
    assert prof.safe_speed is None
    assert prof.ticks_per_second is None
    assert prof.motion_onset_seconds is None
    assert prof.ceiling_speed == 150
    assert prof.ceiling_reason == REASON_OVERLOAD


# ===========================================================================
# A probe that never met anything certifies nothing
# ===========================================================================


def test_first_probe_against_a_reachable_target_is_a_void_run() -> None:
    """The thesis from the other side: no obstacle => no evidence => hard error.

    The joint sails to its target through free air. A verb that shrugged and
    reported "safe speed: 150" here would be handing the next task a number with
    nothing under it, while looking exactly like a number that had evidence.
    """
    bus = _SpeedCoupledArm(positions={MOTOR: HOME}, detection_ceiling=1000)
    bus.open()  # NO obstacle placed

    with pytest.raises(CliError) as exc:
        _run(bus, ladder=(150, 300))

    assert exc.value.code == EXIT_USER_ERROR
    assert "nothing there to detect" in exc.value.message
    assert "--contact-to" in exc.value.remediation


def test_a_void_run_still_safes_the_joint() -> None:
    """The joint is limped BEFORE the error is raised — safety precedes reporting."""
    bus = _SpeedCoupledArm(positions={MOTOR: HOME}, detection_ceiling=1000)
    bus.open()

    with pytest.raises(CliError):
        _run(bus, ladder=(150,))

    torque_for_motor = [w for w in bus.torque_writes if w["motor"] == MOTOR]
    assert torque_for_motor[-1]["on"] is False


# ===========================================================================
# Between probes: the joint is returned home, and never at an untested speed
# ===========================================================================


def test_each_probe_gets_the_same_run_up_from_home() -> None:
    bus = _arm(detection_ceiling=1000)
    prof = _run(bus, ladder=(150, 200, 250))

    # Every probe started where the last one began: the ladder compares like with
    # like, rather than measuring three probes with three different run-ups.
    starts = [t.start_position for t in prof.trials]
    assert all(abs(s - HOME) <= DEFAULT_LOAD_WATCH.arrival_tolerance for s in starts)


def test_retreats_never_use_an_uncertified_speed() -> None:
    """The move whose job is to make the arm safe is made at a PROVEN speed."""
    bus = _arm(detection_ceiling=250)
    _run(bus, ladder=(150, 300))

    speeds = [w["value"] for w in bus.speed_writes if w["motor"] == MOTOR]
    # 300 was written exactly once — for its own probe. Every other write (the
    # run-up retreat before it, and the safing retreat after it was rejected) is
    # at 150, the last speed proven to still detect contact.
    assert speeds.count(300) == 1
    assert all(s in (150, 300) for s in speeds)
    assert speeds[-1] == 150, "the final, safing retreat runs at the certified speed"


def test_the_run_ends_with_the_joint_home_and_limp() -> None:
    bus = _arm(detection_ceiling=1000)
    _run(bus, ladder=(150,))

    assert abs(bus.true_position(MOTOR) - HOME) <= DEFAULT_LOAD_WATCH.arrival_tolerance
    torque_for_motor = [w for w in bus.torque_writes if w["motor"] == MOTOR]
    assert torque_for_motor[-1]["on"] is False


def test_a_bus_fault_mid_ramp_still_safes_the_joint_and_propagates_the_real_error() -> None:
    """Cleanup must never mask the failure the operator has to see."""

    class _DyingArm(_SpeedCoupledArm):
        def __init__(self, *args, **kwargs) -> None:
            super().__init__(*args, **kwargs)
            self.dead = False

        def read_info(self, motor: int) -> dict:
            if self.dead:
                raise CliError(code=2, message="simulated bus fault", remediation="reconnect")
            return super().read_info(motor)

    bus = _arm(cls=_DyingArm, detection_ceiling=1000)
    released: list[int] = []
    original_clear = bus.clear_overload
    bus.clear_overload = lambda motor: released.append(motor)  # type: ignore[assignment]

    def kill_after_first(trial: SpeedTrial) -> None:
        bus.dead = True

    with pytest.raises(CliError) as exc:
        _run(bus, ladder=(150, 300), progress=kill_after_first)

    assert exc.value.message == "simulated bus fault", "the REAL error, not a cleanup error"
    assert released == [MOTOR], "the joint was still released on the way out"
    bus.clear_overload = original_clear  # type: ignore[assignment]


# ===========================================================================
# The motion gate
# ===========================================================================


def test_profile_joint_without_allow_motion_writes_nothing() -> None:
    bus = _arm()

    with pytest.raises(CliError) as exc:
        profile_joint(
            bus,
            MOTOR,
            joint="shoulder_pan",
            contact_target=CONTACT_TARGET,
            min_angle=0,
            max_angle=4095,
            threshold=THRESHOLD,
            ladder=(150,),
            allow_motion=False,
        )

    assert exc.value.code == EXIT_USER_ERROR
    assert bus.register_writes == [], "the motion gate must precede every bus write"


def test_a_contact_target_the_joint_is_already_at_is_rejected() -> None:
    bus = _arm()
    with pytest.raises(CliError) as exc:
        profile_joint(
            bus,
            MOTOR,
            joint="shoulder_pan",
            contact_target=HOME,
            min_angle=0,
            max_angle=4095,
            threshold=THRESHOLD,
            ladder=(150,),
            allow_motion=True,
        )
    assert exc.value.code == EXIT_USER_ERROR
    assert "no travel" in exc.value.message


# ===========================================================================
# Reporting shape
# ===========================================================================


def test_profile_as_dict_carries_every_conclusion() -> None:
    bus = _arm(detection_ceiling=250)
    payload = _run(bus, ladder=(150, 300)).as_dict()

    assert payload["certified"] is True
    assert payload["safe_speed"] == 150
    assert payload["ceiling_speed"] == 300
    assert payload["ceiling_reason"] == REASON_CONTACT_MISSED
    assert payload["threshold"] == THRESHOLD
    assert payload["ladder"] == [150, 300]
    assert [t["reason"] for t in payload["trials"]] == [
        REASON_CONTACT_DETECTED,
        REASON_CONTACT_MISSED,
    ]


def test_every_reason_constant_is_distinct() -> None:
    reasons = {
        REASON_CONTACT_DETECTED,
        REASON_CONTACT_MISSED,
        REASON_OVERLOAD,
        REASON_NO_CONTACT,
    }
    assert len(reasons) == 4


def test_the_module_default_acceleration_matches_a_real_gentle_move() -> None:
    """The ramp varies exactly ONE variable — speed. A second would make it unreadable."""
    from arm101.hardware import gentle

    assert profile_mod.DEFAULT_ACCELERATION == gentle._DEFAULT_ACCELERATION
    assert profile_mod.DEFAULT_SPEED_START == gentle._DEFAULT_SPEED


# ===========================================================================
# Cleanup must never mask the real failure
# ===========================================================================


class _DeadPortArm(_SpeedCoupledArm):
    """A bus that dies mid-ramp with a NON-CliError, and keeps dying during cleanup.

    This models the failure that actually happens: pyserial's ``SerialException``
    comes out of the SDK **unwrapped**, so it is not a :class:`CliError` at all.
    A cleanup path that suppresses only ``CliError`` would sail straight past the
    one exception most likely to be in flight — and worse, the cleanup's OWN
    failure would then replace the hardware fault the operator needs to see.
    """

    def __init__(self, *args, die_after: int, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._die_after = die_after
        self._writes_seen = 0

    def write_goal_position(self, motor: int, position: int) -> None:
        self._writes_seen += 1
        if self._writes_seen > self._die_after:
            raise OSError("device disconnected")  # NOT a CliError — like pyserial
        super().write_goal_position(motor, position)


def test_a_dead_port_mid_ramp_propagates_the_real_fault_not_the_cleanup_s() -> None:
    """The operator must see the dead port, not a secondary error from the retreat.

    The retreat in ``profile_joint``'s ``finally`` runs against the very bus that
    just died, so it will fail too. That second failure must be swallowed: it is
    noise, and the ``OSError`` is the news.
    """
    bus = _arm(_DeadPortArm, die_after=1)

    with pytest.raises(OSError) as exc:
        _run(bus, (150, 200))

    assert "device disconnected" in str(exc.value)


def test_the_joint_is_still_de_energised_when_the_bus_dies_mid_ramp() -> None:
    """A cleanup that gives up on its first failure would walk away from a hot arm.

    This is the whole lesson of #33, applied one layer down: the release must
    survive the failure of the thing it is cleaning up after.
    """
    bus = _arm(_DeadPortArm, die_after=1)

    with contextlib.suppress(OSError):
        _run(bus, (150, 200))

    assert bus.torque_writes, "the guard must have attempted a release"
    assert bus.torque_writes[-1]["on"] is False, "the joint must be left limp"
