"""t6 — the probe that drives ONE joint outward until it learns something.

The four verdicts, each produced by its own hardware model
==========================================================
A verdict no test can generate is one the code cannot really tell from its
neighbours, so each of :class:`LimitVerdict`'s four members is driven out of the
probe by a *physically distinct* servo:

* :class:`_WalledServo` — free travel, then an immovable stop. **WALL.**
* :class:`_GravityServo` — nothing in front of it; its load simply climbs with how
  far it is driven, and it runs out of torque. **TORQUE_LIMITED.**
* :class:`_LatchingServo` — the servo's OWN overload latch fires. **TORQUE_LIMITED.**
* a plain :class:`RollingServoBus` — free all the way round. **EDGE.**
* :class:`_SeizingServo` — stops advancing without ever loading up. **TIMEOUT.**

Every bus here is a :class:`~tests._rolling_servo.RollingServoBus`, never
``ServoModelBus``: the latter converts goals to raw at write time and clamps
linearly in raw, so it drives a shaft near the seam *the long way round* and can
prove nothing about a probe whose whole job is to cross one.

Numbers come from ``gentle``/``arm_spec``/``ticks``, never from a copy of them.
"""

from __future__ import annotations

import pytest

from arm101.cli._errors import EXIT_USER_ERROR, CliError
from arm101.hardware import gentle, probe
from arm101.hardware.arm_spec import DEFAULT_CONTACT_THRESHOLDS, FACTORY_ENCODER_OFFSET
from arm101.hardware.bus import OverloadError
from arm101.hardware.journal import CalibrationJournal
from arm101.hardware.limits import (
    ENCODER_TICKS,
    HALF_TURN,
    LimitVerdict,
    TravelEnd,
    signed_delta,
)
from arm101.hardware.probe import (
    DEFAULT_CREEP_TICKS,
    ProbeOutcome,
    _contact_displacement,
    free_run_needed,
    probe_end,
    suggested_threshold,
    wall_compliance,
)
from arm101.hardware.rolling_frame import MAX_HEADROOM, RollingFrame
from arm101.hardware.ticks import raw_from_reported
from tests._rolling_servo import FREE_TRAVEL_LOAD, RollingServoBus

JOINT = "elbow_flex"
MOTOR = 3

#: The joint's own contact threshold, from the single source of truth.
THRESHOLD = DEFAULT_CONTACT_THRESHOLDS[JOINT]

#: Where ``present_load`` saturates. A real wall and an exhausted arm BOTH read
#: exactly this at the moment of the stop — which is the whole reason the verdict
#: cannot be taken from the load at the stop.
CEILING = gentle.CONTACT_LOAD_CEILING

WATCH = gentle.DEFAULT_LOAD_WATCH

#: ``gentle``'s MEASURED contact-relief distance — the retreat that reliably drops a
#: contact's load back under threshold on this arm, and therefore the scale of a real
#: contact's compliant zone. Imported, not copied: it is the number the probe's own
#: cutoff is derived from, and a test that typed 50 here would still pass if both
#: drifted.
BACKOFF = gentle._DEFAULT_BACKOFF_TICKS


# ---------------------------------------------------------------------------
# The servos. Each one is a different PHYSICAL story about why a joint stopped.
# ---------------------------------------------------------------------------


class _Recording(RollingServoBus):
    """A rolling servo that also keeps the books a probe can be audited against.

    ``travel_log`` is the shaft's odometer at every poll, and ``goal_log`` records
    every ``Goal_Position`` write together with **the offset in force at the moment
    it was written** — which is the only way to ask the question AC1 is about: did
    the tick the probe commanded name the angle it meant, *in the frame it was
    written in*?
    """

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.travel_log: list[int] = []
        self.goal_log: list[dict[str, int]] = []

    def read_info(self, motor: int) -> dict:
        info = super().read_info(motor)
        self.travel_log.append(self.net_travel(motor))
        return info

    def write_goal_position(self, motor: int, position: int) -> None:
        offset = self._offsets.get(motor, 0)
        self.goal_log.append(
            {
                "reported": position,
                "offset": offset,
                "raw": raw_from_reported(position, offset),
                "true_raw": self.true_raw(motor),
            }
        )
        super().write_goal_position(motor, position)

    # -- what the shaft is being TOLD to do, right now ----------------------

    def _commanded(self, motor: int) -> int:
        """``+1``/``-1``/``0``: which way the standing goal is pulling the shaft."""
        goal = self.reported_goal(motor)
        if goal is None or not self.torque_on(motor):
            return 0
        error = goal - self._reported_position(motor, self.true_raw(motor))
        if error > 0:
            return 1
        return -1 if error < 0 else 0


class _WalledServo(_Recording):
    """A shaft that CANNOT pass a point. The signature of a real mechanical limit.

    Free motion at :data:`~tests._rolling_servo.FREE_TRAVEL_LOAD` right up to the
    stop, then an abrupt spike to the saturation ceiling with the shaft immobile.

    *compliance* models the give in the gearbox and links — the ticks a joint keeps
    creeping INTO an obstacle under rising load before it truly stops. ``gentle``
    measured that zone on the SO-101 from the other side: a 30-70 tick retreat
    reliably relieves a contact, which is what ``_DEFAULT_BACKOFF_TICKS`` is. A
    ``compliance`` of 0 is a perfectly rigid stop; a large one is a wall so soft the
    probe can no longer tell it from an arm running out of torque, and the test that
    uses it says so.

    *wall_load* is the load the joint develops pressing on the obstacle. It defaults to
    the saturation :data:`CEILING`, which is what a joint with torque to spare reads
    against a solid stop. Set it BELOW the joint's contact threshold to model a joint too
    weak to push its own threshold — ``wrist_roll``, whose walls press at only 272-288
    against a threshold that was set to 400. Such a wall is real, and INVISIBLE: the
    contact rule needs ``load > threshold`` and the load never gets there.
    """

    def __init__(
        self,
        *args,
        wall_travel: int,
        compliance: int = 0,
        wall_load: int = CEILING,
        **kwargs,
    ) -> None:
        super().__init__(*args, **kwargs)
        self.wall_travel = int(wall_travel)
        self.compliance = int(compliance)
        self.wall_load = int(wall_load)
        self.wall_direction = 1 if self.wall_travel >= 0 else -1

    def _remaining(self, motor: int) -> int:
        """Ticks of free travel left before the shaft meets the obstacle."""
        return (self.wall_travel - self.net_travel(motor)) * self.wall_direction

    def _pressed_load(self, remaining: int) -> int:
        """Load at *remaining* ticks from the wall: free, then ramping into the wall load."""
        if remaining <= 0:
            return self.wall_load
        if self.compliance <= 0 or remaining >= self.compliance:
            return self.travel_load
        into = (self.compliance - remaining) / self.compliance
        return int(self.travel_load + into * (self.wall_load - self.travel_load))

    def _advance(self, motor: int) -> "tuple[int, int]":
        if self._remaining(motor) <= 0 and self._commanded(motor) == self.wall_direction:
            # Pressed into it. The shaft does not move and the load tops out — and at the
            # default saturating wall_load that is EXACTLY what an exhausted joint reads
            # like. Hence t6.
            return self.true_raw(motor), self.wall_load

        raw, load = super()._advance(motor)
        overshoot = -self._remaining(motor)
        if overshoot > 0:  # the step would have gone THROUGH the obstacle
            raw = (raw - overshoot * self.wall_direction) % ENCODER_TICKS
            self._positions[motor] = raw
            self._net_travel[motor] = self.wall_travel
        if not load:
            return raw, load
        return raw, self._pressed_load(self._remaining(motor))


class _GravityServo(_Recording):
    """A joint that must LIFT something, and eventually cannot. Nothing is in its way.

    Its load climbs with how far out it has been driven (the moment arm growing), so
    it is **already working hard while it is still advancing** — and it slows as the
    load approaches the ceiling, until it simply stops. ``shoulder_lift`` carrying
    the whole arm. The stop is indistinguishable from a wall at the moment it
    happens: same saturated load, same joint not advancing.
    """

    def __init__(self, *args, load_per_tick: float, lift: int = 1, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.load_per_tick = float(load_per_tick)
        self.lift = int(lift)

    def _effort(self, motor: int) -> int:
        """The torque this pose demands — a function of ANGLE, not of any obstacle."""
        out = max(0, self.net_travel(motor) * self.lift)
        return min(CEILING, int(self.travel_load + self.load_per_tick * out))

    def _advance(self, motor: int) -> "tuple[int, int]":
        effort = self._effort(motor)
        if effort >= CEILING and self._commanded(motor) == self.lift:
            return self.true_raw(motor), CEILING  # out of torque

        before = self.net_travel(motor)
        full = self.ticks_per_poll
        try:
            # The harder it is working, the slower it goes. A torque-limited joint
            # does not stop dead — it creeps, which is precisely why it takes the
            # stall rule 8 consecutive samples to call it.
            self.ticks_per_poll = max(1, int(full * (1 - effort / CEILING)))
            raw, load = super()._advance(motor)
        finally:
            self.ticks_per_poll = full
        if not load:
            return raw, load
        # ``present_load`` SATURATES at ``Torque_Limit``: the servo is a POSITION
        # controller, so once it can no longer keep up with its goal the error winds up
        # and it commands everything it has. Which is why the load AT THE STOP is the
        # same number a wall produces, and why the verdict cannot be read off it.
        moved = abs(self.net_travel(motor) - before)
        return raw, (CEILING if moved < WATCH.stall_eps else effort)


class _LatchingServo(_Recording):
    """A servo whose OWN overload latch (error=32) trips once it is driven far enough.

    The hardware giving up before the software rule can say anything. One-shot, as
    measured on the follower: ``clear_overload`` releases it and it does not re-trip.
    """

    def __init__(self, *args, latch_after: int, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.latch_after = int(latch_after)
        self.tripped = False

    def read_info(self, motor: int) -> dict:
        if not self.tripped and abs(self.net_travel(motor)) >= self.latch_after:
            self.tripped = True
            raise OverloadError(motor=motor, error_byte=32, message="simulated dynamic overload")
        return super().read_info(motor)


class _AlreadyLatched(_Recording):
    """A servo that is latched in overload BEFORE the probe ever reads it.

    The likeliest servo on the arm to be in this state is the one a previous probe just
    drove into a wall. Every ``read_info`` raises, so ``gentle_move`` never gets a
    position at all and reports ``final_position=None`` — the one path on which the
    probe has nothing whatsoever to go on.
    """

    def read_info(self, motor: int) -> dict:
        raise OverloadError(motor=motor, error_byte=32, message="latched before the probe began")


class _SeizingServo(_Recording):
    """A joint that stops advancing and NEVER loads up. A slipped gear; a dead motor.

    No contact (the load gate never fires) and no arrival. The probe learns nothing,
    and the record has to say so.
    """

    def __init__(self, *args, seizes_after: int, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.seizes_after = int(seizes_after)

    def _advance(self, motor: int) -> "tuple[int, int]":
        if abs(self.net_travel(motor)) >= self.seizes_after:
            return self.true_raw(motor), self.travel_load  # stuck, and nothing to detect
        return super()._advance(motor)


class _PhantomArrival(_Recording):
    """A servo that swears it is AT its goal while its shaft never turns.

    Pathological, and the reason the probe carries a move budget: every move "arrives",
    so nothing ever stops the creep, and no tick of travel is ever made — a probe
    without a budget would command this joint until the heat death.
    """

    def _advance(self, motor: int) -> "tuple[int, int]":
        return self.true_raw(motor), 0  # the shaft does not turn. Ever.

    def read_info(self, motor: int) -> dict:
        info = super().read_info(motor)
        goal = self.reported_goal(motor)
        if goal is not None:
            info["present_position"] = goal  # ...and the servo reports it got there
        return info


# ---------------------------------------------------------------------------
# Harness
# ---------------------------------------------------------------------------


def make_journal(tmp_path) -> CalibrationJournal:
    return CalibrationJournal(tmp_path / "calibration-journal.jsonl")


def opened(bus: RollingServoBus) -> RollingServoBus:
    bus.open()
    return bus


def run(
    bus: RollingServoBus,
    tmp_path,
    *,
    end: TravelEnd = TravelEnd.HIGH,
    **kwargs,
) -> "tuple[ProbeOutcome, RollingFrame]":
    """Open a frame, probe one end through it, and hand back both."""
    frame = RollingFrame(bus, make_journal(tmp_path), joint=JOINT, motor=MOTOR)
    with frame:
        outcome = probe_end(
            bus,
            frame,
            end=end,
            threshold=THRESHOLD,
            allow_motion=True,
            **kwargs,
        )
    return outcome, frame


def servo(cls, *, raw: int = 2048, **kwargs):
    """Build one of the servos above, sitting at *raw*, holding the factory offset."""
    return opened(
        cls(
            positions={MOTOR: raw},
            offsets={MOTOR: FACTORY_ENCODER_OFFSET},
            ids=[MOTOR],
            **kwargs,
        )
    )


# ===========================================================================
# AC2 — the four verdicts. Each one produced by a servo that could produce no
# other, so no verdict is merely a name the code never reaches.
# ===========================================================================


def test_a_wall_is_a_WALL(tmp_path):
    """Free travel, then an immovable stop at a saturated load. The one verdict that vouches."""
    bus = servo(_WalledServo, wall_travel=900)
    outcome, _frame = run(bus, tmp_path)

    assert outcome.observation.verdict is LimitVerdict.WALL
    assert outcome.observation.is_wall
    assert outcome.contacted
    # The joint got to the obstacle and no further — measured, not commanded.
    assert outcome.observation.displacement == pytest.approx(900, abs=WATCH.arrival_tolerance)
    # The load at the stop is SATURATED. It is the same number an exhausted joint
    # reports, which is why the verdict cannot be read off it.
    assert outcome.observation.load == CEILING


def test_an_arm_that_simply_RUNS_OUT_OF_TORQUE_is_never_called_a_wall(tmp_path):
    """THE crux. Nothing is in front of this joint. It stops at a saturated load anyway.

    Read the stop alone and it is indistinguishable from :func:`test_a_wall_is_a_WALL`:
    the same ceiling load, the same joint not advancing, the same ``contacted=True``
    from ``gentle_move``. Only the APPROACH tells them apart — this one was already
    pushing past its own contact threshold while it was still moving, for hundreds of
    ticks, because it was carrying a load rather than meeting an obstacle.
    """
    bus = servo(_GravityServo, load_per_tick=0.5)
    outcome, _frame = run(bus, tmp_path)

    assert outcome.contacted  # gentle_move called contact — it cannot tell the difference
    assert outcome.observation.load == CEILING  # ...at exactly the same saturated load
    assert outcome.observation.verdict is LimitVerdict.TORQUE_LIMITED
    assert not outcome.observation.is_wall

    # The evidence, and it is what separates the two cases:
    assert outcome.loaded_run_ticks > outcome.compliance
    assert outcome.loaded_run_ticks > wall_compliance(BACKOFF)


def test_the_stop_ITSELF_says_nothing_the_two_cases_agree_on_every_bit_of_it(tmp_path):
    """The negative result this task is built on, asserted rather than asserted-about.

    A wall and an exhausted arm produce the SAME contact, the SAME saturated load and
    the SAME stalled joint. Anything that ruled on the stop alone would have to give
    them the same verdict — and one of those two answers would be a permanent lie in
    ``arm_spec``.
    """
    wall, _ = run(servo(_WalledServo, wall_travel=900), tmp_path)
    weak, _ = run(servo(_GravityServo, load_per_tick=0.5), tmp_path)

    assert wall.contacted == weak.contacted is True
    assert wall.observation.load == weak.observation.load == CEILING
    assert wall.peak_load == weak.peak_load == CEILING
    # ...and yet:
    assert wall.observation.verdict is not weak.observation.verdict


def test_the_servos_OWN_overload_latch_is_a_lower_bound_not_a_wall(tmp_path):
    """error=32. The hardware gave up before the software rule saw anything.

    Whatever stopped the joint, nobody measured it. TORQUE_LIMITED — never WALL.
    """
    bus = servo(_LatchingServo, latch_after=400)
    outcome, _frame = run(bus, tmp_path)

    assert bus.tripped
    assert outcome.overloaded
    assert not outcome.contacted
    assert outcome.observation.verdict is LimitVerdict.TORQUE_LIMITED


def test_a_servo_ALREADY_LATCHED_when_the_probe_starts_is_a_lower_bound_too(tmp_path):
    """The probe cannot even read this joint, so it certainly cannot vouch for a limit.

    A servo latched in overload answers every read with the fault bit still set — and
    the single likeliest motor on the arm to be in that state is the one a previous
    probe just drove into a wall. ``gentle_move`` recovers (``clear_overload``) and
    reports ``final_position=None``; the probe has nothing to go on, and says so.
    """
    bus = servo(_AlreadyLatched)
    outcome, _frame = run(bus, tmp_path)

    assert outcome.overloaded
    assert not outcome.arrived  # there was no position to arrive AT
    assert outcome.verdict is LimitVerdict.TORQUE_LIMITED
    assert outcome.observation.displacement == 0
    assert outcome.samples == 0  # not one sample of an approach was ever taken


def test_a_joint_that_goes_ALL_THE_WAY_ROUND_runs_out_of_road_EDGE(tmp_path):
    """A full turn, no wall, no stall. There is nothing further to learn — and no limit."""
    bus = servo(_Recording)  # a free-spinning joint: nothing is in its way, ever
    outcome, frame = run(bus, tmp_path)

    assert outcome.observation.verdict is LimitVerdict.EDGE
    assert not outcome.contacted
    assert not outcome.overloaded
    # It really did go round: past the raw seam, and past more than one frame.
    assert outcome.observation.displacement >= ENCODER_TICKS - WATCH.arrival_tolerance
    assert outcome.observation.displacement <= ENCODER_TICKS
    assert frame.recentres >= 1
    assert bus.net_travel(MOTOR) >= ENCODER_TICKS - WATCH.arrival_tolerance


def test_a_shorter_travel_budget_also_ends_at_the_EDGE(tmp_path):
    """EDGE is 'ran out of ROOM to look' — whether the room was the circle or a budget."""
    bus = servo(_Recording)
    outcome, _frame = run(bus, tmp_path, max_travel=800)

    assert outcome.observation.verdict is LimitVerdict.EDGE
    assert outcome.observation.displacement >= 800 - WATCH.arrival_tolerance
    assert outcome.observation.displacement <= 800 + DEFAULT_CREEP_TICKS


def test_a_joint_that_seizes_without_loading_up_is_a_TIMEOUT(tmp_path):
    """It never arrived and it never met anything. Learned nothing — and it says so."""
    bus = servo(_SeizingServo, seizes_after=120)
    outcome, _frame = run(
        bus,
        tmp_path,
        watch=gentle.LoadWatch(timeout=0.25),
    )

    assert outcome.observation.verdict is LimitVerdict.TIMEOUT
    assert not outcome.contacted
    assert not outcome.overloaded
    assert not outcome.arrived
    # The load never got anywhere near contact — there was nothing there to detect.
    assert outcome.peak_load <= THRESHOLD


def test_a_wall_too_weak_to_push_its_own_threshold_is_UNFIRABLE_not_free_air(tmp_path):
    """THE wrist_roll BUG, as a servo. A real wall the instrument cannot hear.

    The joint drives into a solid stop and presses on it at load 200 — hard, and far
    above the 60 it carried while travelling. But its threshold is 280, and
    ``is_contact`` needs ``load > threshold``, so no contact can EVER be called. It
    presses on that wall for the entire budget while the software reports free air.

    This is not a hypothetical. ``wrist_roll`` shipped with a threshold of 400 against
    walls that press at 272 and 288, and was catalogued for a whole session as a joint
    that "turns freely all the way round, no wall anywhere" — a claim written into
    ``arm_spec`` as PROVEN. The probe must refuse to draw any conclusion here, and must
    say WHICH number blinded it.
    """
    bus = servo(_WalledServo, wall_travel=900, wall_load=THRESHOLD - 80)
    outcome, _frame = run(bus, tmp_path, watch=gentle.LoadWatch(timeout=0.25))

    assert outcome.observation.verdict is LimitVerdict.UNFIRABLE_THRESHOLD
    assert not outcome.contacted  # the load gate never opened...
    assert outcome.peak_load <= THRESHOLD  # ...because it could not
    # It was PRESSING — that is what separates this from a joint that merely seized.
    assert outcome.peak_load > FREE_TRAVEL_LOAD
    # And the report hands the operator the number to fix, not a wild goose chase.
    assert str(outcome.peak_load) in outcome.reason
    assert str(suggested_threshold(outcome.peak_load)) in outcome.reason
    assert "COULD NOT FIRE" in outcome.reason

    # The verdict does NOT vouch for a wall — even though there really is one there.
    # An unheard wall is still unmeasured, and an under-claim is the safe error.
    assert not outcome.observation.verdict.vouches_for_a_wall


def test_every_verdict_is_reachable_and_no_two_share_a_servo(tmp_path):
    """The five are genuinely distinguishable, not five names for one code path."""
    verdicts = {
        run(servo(_WalledServo, wall_travel=900), tmp_path)[0].observation.verdict,
        run(servo(_GravityServo, load_per_tick=0.5), tmp_path)[0].observation.verdict,
        run(servo(_Recording), tmp_path)[0].observation.verdict,
        run(
            servo(_SeizingServo, seizes_after=120),
            tmp_path,
            watch=gentle.LoadWatch(timeout=0.25),
        )[0].observation.verdict,
        run(
            servo(_WalledServo, wall_travel=900, wall_load=THRESHOLD - 80),
            tmp_path,
            watch=gentle.LoadWatch(timeout=0.25),
        )[0].observation.verdict,
    }
    assert verdicts == {
        LimitVerdict.WALL,
        LimitVerdict.TORQUE_LIMITED,
        LimitVerdict.EDGE,
        LimitVerdict.TIMEOUT,
        LimitVerdict.UNFIRABLE_THRESHOLD,
    }


# ===========================================================================
# The tie-break: when the evidence does not clearly support a WALL, the answer
# is TORQUE_LIMITED. Never the other way.
# ===========================================================================


def test_a_joint_already_jammed_when_the_probe_STARTS_is_not_a_wall(tmp_path):
    """It never moved. So nobody has seen it move freely, and nobody may call this a wall.

    Physically this is ambiguous — the joint may be resting on its own end-stop, or it
    may be too weak to lift itself out of the pose it is in — and an ambiguous stop is
    a LOWER BOUND. The under-claim is the safe error: the arm reports less reach than
    it has, rather than a limit that is not there.
    """
    bus = servo(_WalledServo, wall_travel=0)  # the obstacle is right here
    outcome, _frame = run(bus, tmp_path)

    assert outcome.contacted
    assert outcome.observation.verdict is LimitVerdict.TORQUE_LIMITED
    assert outcome.free_run_ticks < free_run_needed(WATCH)
    assert outcome.observation.displacement == 0  # it got precisely nowhere


def test_a_wall_so_SOFT_the_probe_cannot_tell_it_from_a_weak_arm_is_not_called_a_wall(tmp_path):
    """The known cost of the discriminator, pinned rather than papered over.

    This IS a wall — the shaft physically cannot pass it. But the probe spent 600
    ticks pushing into it above its contact threshold before it stopped, which is what
    an arm running out of torque looks like and is far beyond the give a real contact
    was measured to have (``gentle``'s 30-70 tick relief). So the probe declines to
    vouch for it. Under-claiming a real wall is the price of never inventing one.
    """
    bus = servo(_WalledServo, wall_travel=1200, compliance=600)
    outcome, _frame = run(bus, tmp_path)

    assert outcome.contacted
    assert outcome.observation.verdict is LimitVerdict.TORQUE_LIMITED
    assert outcome.loaded_run_ticks > outcome.compliance


def test_a_wall_whose_give_matches_a_REAL_contact_is_still_a_wall(tmp_path):
    """...and the cutoff is not so tight that a real, slightly compliant wall fails it.

    ``gentle``'s ``backoff`` is the MEASURED distance that relieves a contact on this
    arm, so it is the scale of a real contact's compliant zone. A wall whose give is
    that size must still read as a wall, or the WALL verdict would be unreachable on
    hardware and the classifier would return UNDETERMINED forever.
    """
    bus = servo(_WalledServo, wall_travel=900, compliance=BACKOFF)
    outcome, _frame = run(bus, tmp_path)

    assert outcome.observation.verdict is LimitVerdict.WALL
    assert outcome.loaded_run_ticks <= outcome.compliance


def test_the_compliance_cutoff_is_derived_from_gentles_measured_backoff(tmp_path):
    """It is a hardware number, not a preference — and it moves when ``backoff`` moves."""
    assert wall_compliance(BACKOFF) == 2 * BACKOFF

    bus = servo(_WalledServo, wall_travel=900, compliance=BACKOFF)
    outcome, _frame = run(bus, tmp_path, compliance=0)  # a probe that demands a RIGID stop

    # The same wall, ruled on by a stricter probe: it no longer vouches.
    assert outcome.observation.verdict is LimitVerdict.TORQUE_LIMITED
    assert outcome.compliance == 0


@pytest.mark.parametrize("give", [0, 30, 50, 70])
def test_every_wall_in_the_MEASURED_give_band_still_reads_as_a_WALL(tmp_path, give):
    """The whole 30-70 tick band ``gentle`` measured a contact's relief distance over.

    If the cutoff refused any of these, the WALL verdict would be unreachable on real
    hardware and ``classify`` would answer UNDETERMINED forever — which is safe, useless,
    and an open invitation for the next person to raise the knob without evidence.
    """
    outcome, _frame = run(servo(_WalledServo, wall_travel=1200, compliance=give), tmp_path)

    assert outcome.observation.verdict is LimitVerdict.WALL
    assert outcome.loaded_run_ticks <= outcome.compliance


@pytest.mark.parametrize("slope", [0.2, 0.5, 0.8, 1.0, 1.3])
def test_no_gravity_climb_the_ARM_CAN_ACTUALLY_HAVE_is_ever_called_a_wall(tmp_path, slope):
    """The other side of the same cutoff, over every load ramp this arm can physically show.

    *slope* is how fast the joint's load climbs, in load units per tick. Gravity bounds it:
    the joint stalls where its gravity torque meets the 500 cap, so at that angle
    ``dtau/dtheta = sqrt(tau_horizontal**2 - 500**2)`` — and ``tau_horizontal`` cannot
    exceed the load register's 1000-unit full scale for a joint that can hold itself out
    horizontally at all, which this arm demonstrably can. With 4096 ticks to the turn
    (652 per radian) that caps the slope at ``sqrt(1000**2 - 500**2) / 652 = 1.33``
    units/tick. Every value here is inside that, and none of them is a wall.
    """
    outcome, _frame = run(servo(_GravityServo, load_per_tick=slope), tmp_path)

    assert outcome.observation.verdict is LimitVerdict.TORQUE_LIMITED
    assert outcome.loaded_run_ticks > outcome.compliance


def test_a_CRUSHING_load_can_still_fool_the_probe_and_this_is_exactly_where(tmp_path):
    """THE KNOWN HOLE. Pinned, not hidden — because the next person needs to know the cliff.

    A load ramp steep enough stalls the joint so soon, and so close to where the load
    crossed its threshold, that the loaded run is as short as a real contact's. The probe
    then vouches for a wall that is not there.

    **The cliff sits at a slope of about 1.8 units/tick** (1.6 still reads torque-limited;
    2.0 reads wall). The physics above caps a real gravity joint at **1.33**, so the margin
    is about **1.35x** — real, but THIN, and it narrows further as a joint's contact
    threshold rises toward the 500 ceiling (the loaded band ``500 - threshold`` is the
    numerator of the loaded run). ``wrist_roll``'s threshold of 400 halves it — and gets
    away with it only because a roll axis carries no gravity torque to begin with.

    So: the first hardware session must record ``loaded_run_ticks`` per joint per end, and
    set ``compliance`` from that data rather than from this reasoning. Until then the knob
    is there, and tightening it costs nothing but an under-claim.
    """
    crushing = servo(_GravityServo, load_per_tick=3.0)
    fooled, _frame = run(crushing, tmp_path)

    assert fooled.observation.verdict is LimitVerdict.WALL  # ...and there is NO wall.
    assert fooled.loaded_run_ticks <= fooled.compliance

    # The mitigation, and it is a parameter rather than a code change: demand a stop as
    # abrupt as a rigid contact's, and the same joint is honestly recorded as a lower bound.
    honest, _frame = run(servo(_GravityServo, load_per_tick=3.0), tmp_path, compliance=BACKOFF // 2)
    assert honest.observation.verdict is LimitVerdict.TORQUE_LIMITED


# ===========================================================================
# AC1 — the target is always computed in the CURRENT frame. Never a stale one.
# ===========================================================================


def test_a_target_computed_BEFORE_a_re_centre_names_a_DIFFERENT_ANGLE_after_it(tmp_path):
    """The counterfactual, so the test below has teeth.

    A ``Goal_Position`` is a REPORTED tick. Move the frame under it and the very same
    number names a different physical angle — here, one **1500 ticks past** the joint
    instead of the 300 that were meant. Commanded, it would drive the joint straight
    through whatever the probe had just found and hold it there under full torque.
    """
    bus = servo(_Recording)
    step = DEFAULT_CREEP_TICKS

    with RollingFrame(bus, make_journal(tmp_path), joint=JOINT, motor=MOTOR) as frame:
        # Creep out until the joint sits well away from the centre of its frame.
        probe_end(
            bus,
            frame,
            end=TravelEnd.HIGH,
            threshold=THRESHOLD,
            max_travel=1500,
            allow_motion=True,
        )
        frame.sync()

        target = frame.reported + step
        meant = signed_delta(raw_from_reported(target, frame.offset), frame.raw)
        assert meant == step  # in the CURRENT frame it names exactly the tick we meant

        frame.recentre()  # the frame rolls; the joint does not move a tick

        now_means = signed_delta(raw_from_reported(target, frame.offset), frame.raw)
        assert now_means != meant
        assert abs(now_means) > step  # the same number, a wildly different angle


def test_every_target_the_probe_commands_is_measured_from_where_the_joint_IS(tmp_path):
    """AC1. Audited over EVERY goal write, in the frame each one was written in.

    A probe that cached a target across a re-centre would issue a goal naming a tick
    far outside the step it asked for — the failure the test above makes concrete, and
    the one that fired live on the arm. So the audit is exactly that: no commanded goal
    ever names a raw tick further from the joint than the probe could possibly have
    meant.
    """
    step = DEFAULT_CREEP_TICKS
    # 2500 ticks: further than a single frame can command (MAX_HEADROOM), so the probe
    # MUST re-centre mid-approach to reach the wall at all.
    assert 2500 > MAX_HEADROOM
    bus = servo(_WalledServo, wall_travel=2500)
    outcome, frame = run(bus, tmp_path, step=step)

    assert frame.recentres >= 1, "the probe must have rolled the frame to get this far"
    assert outcome.observation.verdict is LimitVerdict.WALL

    reach = max(step, BACKOFF) + WATCH.arrival_tolerance
    for write in bus.goal_log:
        lead = abs(signed_delta(write["raw"], write["true_raw"]))
        assert lead <= reach, write


def test_the_probe_re_centres_and_KEEPS_GOING_it_is_not_bounded_by_the_frame(tmp_path):
    """The rolling frame's promise, kept by its first real consumer."""
    bus = servo(_WalledServo, wall_travel=3400, raw=4000)
    outcome, frame = run(bus, tmp_path)

    assert outcome.observation.verdict is LimitVerdict.WALL
    assert frame.recentres >= 1
    assert outcome.recentres == frame.recentres
    assert outcome.observation.displacement == pytest.approx(3400, abs=WATCH.arrival_tolerance)
    # Every goal ever written sat inside the servo's reported scale. No move was ever
    # asked to cross the seam.
    assert all(0 <= write["reported"] <= 4095 for write in bus.goal_log)


# ===========================================================================
# AC3 — RAW displacement. The measurement survives the frame moving under it.
# ===========================================================================


def test_displacement_is_RAW_and_agrees_with_the_shafts_own_odometer(tmp_path):
    """Not a tick difference, and not a reported one: an accumulated RAW displacement.

    The shaft starts at raw 4000 and is driven 900 ticks up, so it rolls straight
    through the raw 4095 -> 0 seam. Subtract raw ticks and that reads as a 3196-tick
    RETREAT. The displacement says what happened.
    """
    bus = servo(_WalledServo, wall_travel=900, raw=4000)
    outcome, _frame = run(bus, tmp_path)
    observation = outcome.observation

    assert observation.verdict is LimitVerdict.WALL
    assert observation.origin_raw == 4000
    assert observation.displacement > 0
    assert observation.raw_tick < observation.origin_raw  # the raw tick WRAPPED
    # ...and the displacement is the truth, checked against the simulation's own
    # odometer — a number that never passed through the seam or through an offset.
    assert observation.displacement == pytest.approx(
        bus.net_travel(MOTOR) + BACKOFF, abs=WATCH.arrival_tolerance
    )
    # The wall is named as a RAW tick, and it is where the shaft actually met it.
    assert observation.raw_tick == (4000 + 900) % ENCODER_TICKS


def test_the_displacement_names_the_CONTACT_point_not_the_retreat(tmp_path):
    """A wall is where the joint TOUCHED, not where ``gentle_move`` parked it afterwards.

    Recording the retreat would shave ``backoff`` ticks off every measured wall — and
    a travel measured NARROWER than it is makes the unreachable arc WIDER than it is,
    which is how a re-zero comes to park the seam on a tick the joint can actually
    reach. The under-claim that is safe at a torque-limited end is exactly the wrong
    error here.
    """
    bus = servo(_WalledServo, wall_travel=900)
    outcome, frame = run(bus, tmp_path)

    # The shaft ended up backed OFF the wall...
    assert bus.net_travel(MOTOR) == pytest.approx(900 - BACKOFF, abs=WATCH.arrival_tolerance)
    # ...and the record still names the wall.
    assert outcome.observation.displacement == pytest.approx(900, abs=WATCH.arrival_tolerance)
    assert outcome.observation.displacement > bus.net_travel(MOTOR)
    assert frame.closed  # and the frame put the servo's calibration back


def test_a_LOW_end_probe_reports_a_NEGATIVE_displacement(tmp_path):
    """The sign contract ``EndObservation`` enforces: a LOW end travels DOWN."""
    bus = servo(_WalledServo, wall_travel=-900)
    outcome, _frame = run(bus, tmp_path, end=TravelEnd.LOW)

    assert outcome.observation.end is TravelEnd.LOW
    assert outcome.observation.verdict is LimitVerdict.WALL
    assert outcome.observation.displacement < 0
    assert outcome.observation.displacement == pytest.approx(-900, abs=WATCH.arrival_tolerance)


def test_the_displacement_never_exceeds_what_an_observation_can_HOLD(tmp_path):
    """One full turn is the cap: past it there is nothing left to learn, and the record
    would no longer be within a lap of any reference it is compared against."""
    bus = servo(_Recording)
    outcome, _frame = run(bus, tmp_path)

    assert abs(outcome.observation.displacement) <= ENCODER_TICKS
    assert abs(outcome.observation.displacement) <= HALF_TURN * 2


def test_the_second_probe_measures_from_the_joint_but_ANCHORS_on_the_frame(tmp_path):
    """Both are required, and conflating them is what broke elbow_flex on hardware.

    A frame is shared by BOTH ends of a joint: one shift, one restore, one transaction.
    So the second probe starts where the first one STOPPED — at its wall, minus the
    back-off — and it must MEASURE its own travel from there.

    But it must not ANCHOR there. The merge compares the two ends against one reference,
    and if each end names its own origin, the merge has to work out how those origins
    relate — which it can only do by taking the short way round the circle. That is a
    GUESS, and this test's predecessor said so out loud: it noted the merge "can only
    reconcile [them] while they are less than half a turn apart", and then shipped the
    per-probe origin anyway.

    On 2026-07-13, on the arm: elbow_flex's travel is 2297 ticks. Half a turn is 2048.
    The low probe went from raw 76 DOWN to its wall at 2058 — travelling -2114, the long
    way, through the seam. The high probe then started at raw 2106. The short way from 76
    to 2106 is +2030; the true path was -2066. Thirty-six ticks apart, so `min` picked the
    wrong sign, and the span came out 6393 instead of 2297 — over by exactly one turn. A
    joint with two measured walls was classified CONTINUOUS.

    The frame knows the path exactly. `origin_offset` is where the probe says so, and
    nothing downstream ever has to guess again.
    """
    bus = servo(_WalledServo, wall_travel=700)
    frame = RollingFrame(bus, make_journal(tmp_path), joint=JOINT, motor=MOTOR)

    with frame:
        first = probe_end(bus, frame, end=TravelEnd.HIGH, threshold=THRESHOLD, allow_motion=True)
        # The joint is now ~650 ticks up. A second probe, DOWN, must measure from HERE.
        second = probe_end(
            bus,
            frame,
            end=TravelEnd.LOW,
            threshold=THRESHOLD,
            max_travel=300,
            allow_motion=True,
        )

    # ANCHOR: both ends name the SAME origin — the frame's — so the merge needs no guess.
    assert first.observation.origin_raw == second.observation.origin_raw == 2048

    # MEASURE: the first probe began at the anchor; the second began where the first
    # left off, and says so exactly rather than leaving it to be inferred.
    assert first.observation.origin_offset == 0
    assert second.observation.origin_offset == pytest.approx(
        700 - BACKOFF, abs=WATCH.arrival_tolerance
    )
    assert second.observation.displacement < 0

    # And the two are now commensurable BY CONSTRUCTION: each end's extent from the
    # shared anchor is just origin_offset + displacement — no circle arithmetic at all.
    assert first.observation.extent_from(2048) == first.observation.displacement
    assert second.observation.extent_from(2048) == (
        second.observation.origin_offset + second.observation.displacement
    )

    assert bus.offset_writes  # one transaction, however many probes ran inside it


# ===========================================================================
# The gate, the arguments, and the pathological bus
# ===========================================================================


def test_the_probe_writes_NOTHING_without_an_explicit_motion_flag(tmp_path):
    bus = servo(_Recording)
    frame = RollingFrame(bus, make_journal(tmp_path), joint=JOINT, motor=MOTOR)

    with frame:
        before = len(bus.register_writes)
        with pytest.raises(CliError) as excinfo:
            probe_end(bus, frame, end=TravelEnd.HIGH, threshold=THRESHOLD)
        assert excinfo.value.code == EXIT_USER_ERROR
        assert len(bus.register_writes) == before  # not one packet


@pytest.mark.parametrize(
    "kwargs",
    [
        {"threshold": 0},
        {"threshold": CEILING},  # can NEVER fire: present_load saturates at the cap
        {"threshold": CEILING + 1},
        {"step": 0},
        {"step": WATCH.arrival_tolerance},  # a step the joint "arrives" at without moving
        {"step": MAX_HEADROOM + 1},  # no frame, however placed, could promise it
        {"backoff": -1},
        {"compliance": -1},
        {"max_travel": 0},
        {"max_travel": ENCODER_TICKS + 1},  # an observation could not hold the answer
    ],
)
def test_the_probe_refuses_arguments_that_could_not_measure_anything(tmp_path, kwargs):
    bus = servo(_Recording)
    frame = RollingFrame(bus, make_journal(tmp_path), joint=JOINT, motor=MOTOR)
    call = {"end": TravelEnd.HIGH, "threshold": THRESHOLD, "allow_motion": True, **kwargs}

    with frame:
        with pytest.raises(CliError) as excinfo:
            probe_end(bus, frame, **call)
        assert excinfo.value.code == EXIT_USER_ERROR
        assert excinfo.value.remediation


def test_a_joint_that_is_not_THERE_does_not_loop_forever(tmp_path):
    """A bus that reports arrival while the shaft never turns. Bounded, and honest about it."""
    bus = servo(_PhantomArrival)
    outcome, _frame = run(bus, tmp_path)

    assert outcome.observation.verdict is LimitVerdict.TIMEOUT
    assert outcome.observation.displacement == 0
    assert bus.net_travel(MOTOR) == 0
    assert outcome.moves < 100  # it gave up rather than writing goals until the heat death


def test_the_outcome_serializes_to_plain_json(tmp_path):
    bus = servo(_WalledServo, wall_travel=900)
    outcome, _frame = run(bus, tmp_path, pose="t6")
    payload = outcome.as_dict()

    assert payload["observation"]["verdict"] == LimitVerdict.WALL.value
    assert payload["observation"]["pose"] == "t6"
    assert payload["reason"]
    assert set(payload) >= {
        "observation",
        "reason",
        "moves",
        "recentres",
        "contacted",
        "overloaded",
        "arrived",
        "peak_load",
        "loaded_run_ticks",
        "free_run_ticks",
        "compliance",
        "samples",
    }


def test_the_reason_explains_the_verdict_in_words_an_operator_can_act_on(tmp_path):
    wall, _ = run(servo(_WalledServo, wall_travel=900), tmp_path)
    weak, _ = run(servo(_GravityServo, load_per_tick=0.5), tmp_path)

    assert "wall" in wall.reason.lower()
    assert "torque" in weak.reason.lower()
    assert str(weak.loaded_run_ticks) in weak.reason


# ===========================================================================
# The sample stream itself — the probe watches the SHIPPED detector, not a copy
# ===========================================================================


def test_the_probe_reads_the_very_stream_gentle_moves_own_rule_is_fed(tmp_path):
    """No re-implementation of the poll loop, and no inference from commanded ticks.

    The approach profile comes off ``gentle_move``'s observer seam, so what the verdict
    is computed from is exactly what the shipped ``_StallDetector`` saw. A probe that
    ran its own copy of the loop would be ruling on the copy.
    """
    bus = servo(_WalledServo, wall_travel=900)
    outcome, _frame = run(bus, tmp_path)

    # Every poll of every approach is accounted for. (The post-contact retreat is not
    # observed — gentle_move does not offer it, deliberately.)
    assert outcome.samples > 0
    assert outcome.samples <= len(bus.poll_log)
    assert outcome.peak_load == CEILING
    assert outcome.free_run_ticks >= free_run_needed(WATCH)


def test_a_contact_that_did_not_record_WHERE_falls_back_rather_than_inventing_a_tick(tmp_path):
    """The guard behind the contact-point reconstruction. It never fabricates a position.

    ``gentle_move`` cannot currently report a contact without a ``contact_position``, and
    if it ever could, the probe must reach for the frame's own synced displacement rather
    than make a tick up — inventing a position it did not measure is the exact bug
    ``gentle`` was rewritten to kill.
    """
    bus = servo(_Recording)
    with RollingFrame(bus, make_journal(tmp_path), joint=JOINT, motor=MOTOR) as frame:
        assert (
            _contact_displacement(
                {"contact_position": None},
                frame=frame,
                before_raw=frame.raw,
                before_displacement=frame.displacement,
            )
            is None
        )


def test_free_run_is_the_travel_the_joint_made_BELOW_its_own_contact_threshold(tmp_path):
    """And it is the joint's OWN threshold — the line ``gentle`` already draws between
    free motion and contact. The probe invents no second one."""
    bus = servo(_WalledServo, wall_travel=900)
    outcome, _frame = run(bus, tmp_path)

    assert FREE_TRAVEL_LOAD == bus.travel_load < THRESHOLD < CEILING
    # It cruised nearly the whole 900 ticks in free air, and the loaded zone in front of
    # the stop is no wider than the give ``gentle`` measured a real contact to have.
    assert outcome.free_run_ticks == pytest.approx(900, abs=DEFAULT_CREEP_TICKS)
    assert outcome.loaded_run_ticks <= BACKOFF
    assert outcome.free_run_ticks + outcome.loaded_run_ticks == pytest.approx(
        900, abs=DEFAULT_CREEP_TICKS
    )


def test_the_pressing_excess_sits_above_the_MEASURED_noise_floor() -> None:
    """The constant that separates "pressing on a wall" from "just stopped", pinned to hardware.

    It was first written as 25 — reasoned, not measured, from the ~212 excess wrist_roll
    develops driving into its real walls, on the argument that anything well under that was
    safe. Hardware disagreed on the first probe that exercised it: wrist_roll stalled against
    NOTHING at a peak of 92 over a cruising load of ~60. An excess of 32 — which cleared 25
    and produced a confident UNFIRABLE_THRESHOLD verdict for a wall that was not there.

    So the constant is bounded from BELOW by noise, not just from above by the weakest wall,
    and both bounds are now measurements:

        false stall, nothing there            32     <- noise floor (measured)
        real wall, gripper's weak end       ~208     <- weakest real wall on this arm
        real wall, wrist_roll               ~212

    A future tuner who is tempted to shave this number back down toward the wall load has to
    get past this test, which is the point of it.
    """
    MEASURED_NOISE_FLOOR = 32
    WEAKEST_REAL_WALL_EXCESS = 208

    assert probe._PRESSING_EXCESS_LOAD > MEASURED_NOISE_FLOOR
    assert probe._PRESSING_EXCESS_LOAD < WEAKEST_REAL_WALL_EXCESS
    # ...and not marginally so, at either end: a stall that clears it must be a real push.
    assert probe._PRESSING_EXCESS_LOAD >= 2 * MEASURED_NOISE_FLOOR
    assert 2 * probe._PRESSING_EXCESS_LOAD <= WEAKEST_REAL_WALL_EXCESS
