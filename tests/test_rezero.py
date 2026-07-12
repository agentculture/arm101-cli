"""Tests for the encoder re-zero (issue #35) — ``arm_spec``'s arc table and
:mod:`arm101.hardware.rezero`'s plan / write / sweep.

Everything here runs against :class:`~arm101.hardware.bus.FakeBus`, and the most
important thing about that fake is that it models **both** readings of the one
undocumented firmware behaviour the whole re-zero rests on
(``docs/spikes/sts3215-offset-register.md`` §4)::

    offset_wraps=True    Present = (raw - Ofs) mod 4096   seam RELOCATES -> fix works
    offset_wraps=False   Present =  raw - Ofs  (signed)   seam STAYS     -> fix does NOTHING

So every claim this module makes about the sweep is asserted **twice**, once in
each world. Under ``offset_wraps=False`` the verification must FAIL, loudly and
by design: that failure is the feature — it is what stands between the operator
and a re-zero that silently did nothing.

The other invariant pinned here is the one the hardware cannot forgive: this verb
**commands no motion, on any path.** ``elbow_flex`` rests at raw ~126, PAST its
wrap, so a linear goal would rotate it the long way round through its whole
travel and into a wall. Several tests assert the write surface directly — no
goal-position register write, ever, and torque only ever going OFF.

The THIRD thing pinned here, added 2026-07-12 after the arc was measured on
hardware for the first time, is the frame::

    RAW      the magnet on the shaft. The arc, the walls, the seam live here.
    REPORTED what the servo says: (raw - Ofs) mod 4096. Every live read is this.

They are equal only at ``Ofs == 0``, and **no servo ships that way** — the factory
default is 85 on all six joints. The original arc ``(126, 2020)`` was measured in
the REPORTED frame at ``Ofs = 85`` and used as if it were RAW, so the target came
out a factory-offset away from where it was meant to be. It landed inside the true
arc regardless, by luck. Tests below pin the conversion on every path.

The FOURTH thing pinned here, and the reason every tick below is *derived*: **the
arc will be re-measured, and re-measuring it must cost ONE table edit.**
-------------------------------------------------------------------------------
The first cut of the raw arc was declared AT the extremes of a single hand sweep
— and it FALSE-REFUSED within minutes. The joint came to rest at raw 218, eleven
ticks past an edge taken from a sweep the operator had simply stopped short of,
and ``arm rezero`` correctly reported that the joint "cannot be where it says it
is". A hand-found wall is not a crisp number (206..218, depending on how hard a
human pushes), and at least one of ``elbow_flex``'s two walls may be the TABLE
rather than the joint's own mechanical stop. So the arc now carries a deliberate
margin — and it will move again, the next time somebody sweeps the joint.

That makes hard-coding its ticks *here* a bug in its own right: a table that is
expected to be re-measured must not have 52 copies of itself in the test suite.
So every tick below comes out of :data:`arm101.hardware.arm_spec.REZERO_ARCS`,
and the numbers that stay literal stay literal on purpose — a *historical fact*
about a real servo (the factory 85, our follower's 1073), or the encoder's own
geometry (4096). Nothing the arc can move is written down twice.
"""

from __future__ import annotations

import pytest

from arm101.cli._errors import EXIT_ENV_ERROR, EXIT_USER_ERROR, CliError
from arm101.hardware import arm_spec, rezero
from arm101.hardware.bus import (
    ADDR_HOMING_OFFSET,
    ADDR_LOCK,
    ADDR_TORQUE_ENABLE,
    ENCODER_RESOLUTION,
    OFFSET_MAX_MAGNITUDE,
    FakeBus,
    OverloadError,
)

#: ``elbow_flex`` on the follower — the only re-zeroable joint on this arm.
ELBOW = "elbow_flex"
ELBOW_MOTOR = 3

# --- everything from here to FACTORY_OFFSET is DERIVED from the arc table ---

#: The joint's RAW unreachable arc, straight from ``arm_spec`` — the single
#: source of every tick in this module. Re-measure the walls on hardware, edit
#: ``REZERO_ARCS``, and this suite follows without a line of it changing.
ARC = arm_spec.rezero_arc(ELBOW)
ARC_LOW = ARC.low
ARC_HIGH = ARC.high

#: What a FRESH re-zero writes: the arc's midpoint. **Derived, never typed** —
#: which is the same discipline ``rezero_offset`` itself follows, and for the
#: same reason (a typed target and a corrected arc drift apart silently).
EXPECTED_OFFSET = arm_spec.rezero_offset(ELBOW)

#: ``elbow_flex``'s travel: the whole circle minus the arc it cannot reach.
EXPECTED_TRAVEL = ARC.travel_ticks

#: The raw walls a human hand actually reached, and the inset applied to EACH of
#: them before the arc was declared. The arc *is* these walls, pulled in by the
#: margin — and the margin is what stops a harder push from contradicting the
#: table (which is exactly what the un-inset first cut did).
LOW_WALL = arm_spec._LOW_WALL_OBSERVED
HIGH_WALL = arm_spec._HIGH_WALL_OBSERVED
ARC_MARGIN = arm_spec._ARC_MARGIN_TICKS

#: A raw tick the joint is supposed to be physically incapable of holding. Dead
#: centre of the arc — which is also, and not by coincidence, where the seam goes:
#: the same tick is "the safest place for the seam" and "the most impossible place
#: for the shaft", because those are the same claim.
IMPOSSIBLE_RAW = ARC.midpoint

#: An offset nobody would ever compute — and whose seam lands strictly inside the
#: arc anyway, so the joint carrying it is already fixed. Derived (rather than a
#: nice round literal like 500) so that it stays inside a RE-MEASURED arc: a
#: literal here is a literal that eventually falls outside one.
ODD_EVICTING_OFFSET = (ARC_LOW + EXPECTED_OFFSET) // 2

# --- literals, and each one is a FACT rather than a derived quantity ---

#: Where ``elbow_flex`` physically rests: raw ~126, PAST its wrap. A fact about
#: the arm on the bench, not a number the arc can move.
REST_RAW = 126

#: What a factory-fresh STS3215 actually holds. **Not 0.** Measured uniform across
#: all six joints of the follower — this is the state of every un-touched SO-101,
#: and the state the verb used to REFUSE outright.
FACTORY_OFFSET = 85

#: What OUR follower holds right now, written by the first (frame-confused)
#: re-zero. Its seam sits at raw 1073, strictly inside the arc — so the seam IS
#: evicted and the arm IS fixed, even though 1073 is not the midpoint. Re-zeroing
#: it again must be a NO-OP. A historical fact about one servo: it stays literal.
OUR_ARM_OFFSET = 1073

#: STS3215 Goal_Position. Must NEVER appear in this verb's write surface.
ADDR_GOAL_POSITION = 42

#: Raw ticks a (simulated) human hand advances the joint between two polls.
SWEEP_STEP = 25

#: Polls needed to hand-walk the WHOLE travel at :data:`SWEEP_STEP` ticks a poll.
#: Sized from the arc, so a re-measured (wider or narrower) travel re-sizes the
#: sweep instead of quietly leaving it short of the coverage a verdict needs.
FULL_SWEEP_SAMPLES = EXPECTED_TRAVEL // SWEEP_STEP + 1

#: The RAW tick such a sweep finishes on, having started at the arc's high wall
#: and wound up over the 4095->0 roll-over.
FULL_SWEEP_END_RAW = (ARC_HIGH + SWEEP_STEP * (FULL_SWEEP_SAMPLES - 1)) % ENCODER_RESOLUTION


def reported_at(raw: int, offset: int = EXPECTED_OFFSET) -> int:
    """What a servo holding *offset* REPORTS when its shaft sits at RAW *raw*.

    ``Present = (Actual - Ofs) mod 4096`` — the servo's own correction, and the
    inverse of :func:`rezero.raw_from_reported`. Every "and then the servo will
    say X" number in this module goes through here instead of being written down,
    because X moves whenever the arc moves (the offset is the arc's midpoint), and
    a written-down X is a test that breaks on a re-measurement rather than
    following it.
    """
    return (raw - offset) % ENCODER_RESOLUTION


def _full_sweep(offset: int = EXPECTED_OFFSET, step: int = SWEEP_STEP) -> "list[int]":
    """The REPORTED positions a hand-walk of the joint's WHOLE travel produces.

    Walks the RAW travel from the near wall (the arc's high endpoint) up over the
    4095->0 roll-over to the far wall (its low endpoint), reporting each raw tick
    through the correction a servo holding *offset* would apply. That is the
    physical sweep the procedure asks an operator for, in list form — and it is
    sized from the arc, so re-measuring the arc lengthens or shortens it rather
    than invalidating every test that uses it.
    """
    return [
        reported_at((ARC_HIGH + n * step) % ENCODER_RESOLUTION, offset)
        for n in range(FULL_SWEEP_SAMPLES)
    ]


# ---------------------------------------------------------------------------
# Test doubles
# ---------------------------------------------------------------------------


class HandMovedBus(FakeBus):
    """A :class:`FakeBus` whose shaft advances a little on every position read.

    Models the one actuator this procedure actually uses: **a human hand.** Each
    :meth:`read_position` call advances the *raw* (physical) encoder count by
    ``ticks_per_read``, wrapping modulo 4096 exactly as the magnet does, and then
    reports it through :meth:`FakeBus._reported_position` — which is where the
    written offset, and the ``offset_wraps`` question, actually bite.

    That layering is the whole point: the simulated shaft turns in raw ticks and
    knows nothing about offsets, so what the sweep SEES is produced by the same
    correction the servo would apply. A fake that moved the *reported* position
    directly would be assuming the answer to the question under test.

    Parameters
    ----------
    start_raw:
        RAW tick the shaft begins at. Defaults to the arc's HIGH endpoint — the
        near end of the joint's travel, and the wall a human is told to start
        from. Derived, so a re-measured arc starts the hand in the right place.
    ticks_per_read:
        How far the hand advances the joint between two polls. Positive winds
        toward the raw seam (``arc.high -> 4095 -> 0 -> arc.low``), which is the
        direction that crosses it.
    """

    def __init__(
        self,
        *args,
        start_raw: int = ARC_HIGH,
        ticks_per_read: int = SWEEP_STEP,
        motor: int = ELBOW_MOTOR,
        **kwargs,
    ) -> None:
        kwargs.setdefault("positions", {motor: start_raw})
        super().__init__(*args, **kwargs)
        self._hand_motor = motor
        self._ticks_per_read = ticks_per_read

    def read_position(self, motor: int) -> int:
        reported = super().read_position(motor)
        if motor == self._hand_motor:
            raw = self._positions.get(motor, 0)
            self._positions[motor] = (raw + self._ticks_per_read) % ENCODER_RESOLUTION
        return reported


def _rezeroed_bus(**kwargs) -> HandMovedBus:
    """A hand-moved bus whose elbow already carries the seam-evicting offset."""
    bus = HandMovedBus(offsets={ELBOW_MOTOR: EXPECTED_OFFSET}, **kwargs)
    bus.open()
    return bus


# ---------------------------------------------------------------------------
# arm_spec — the arc table and the offset derived from it
# ---------------------------------------------------------------------------


def test_the_arc_is_the_one_MEASURED_on_hardware_in_RAW_ticks():
    """The bug, pinned. The arc is RAW ticks, from a real sweep — walls, inset.

    The table shipped with ``(126, 2020)``, which were REPORTED ticks read off a
    servo already holding the factory offset of 85, used as if they were raw. The
    2026-07-12 hand sweep (torque off, seam already evicted, so the encoder could
    finally be walked across its own wrap) measured the travel as reported
    1034..3230 at ``Ofs = 1073`` — i.e. raw 2107 to raw 207, over the wrap. So the
    arc it cannot reach is the complement of that, and it is in the RAW frame.
    """
    arc = arm_spec.rezero_arc(ELBOW)
    # The arc IS the observed walls, inset by the margin — not a pair of typed ticks.
    assert (arc.low, arc.high) == (LOW_WALL + ARC_MARGIN, HIGH_WALL - ARC_MARGIN)
    # And the raw travel really is what that sweep saw, converted back out of the
    # frame it ran in. 1034/3230 (reported, at Ofs = 1073) and the raw 2107/207 they
    # convert to are HISTORICAL FACTS about one run on one arm — so they stay literal.
    assert rezero.raw_from_reported(1034, OUR_ARM_OFFSET) == 2107  # the near wall
    assert rezero.raw_from_reported(3230, OUR_ARM_OFFSET) == 207  # the far wall, at last
    # ...and the arc does not CLAIM either of them. It may never claim a tick the
    # joint has actually been seen at, however the walls are next re-measured.
    assert not arc.contains(2107)
    assert not arc.contains(207)


def test_the_arc_is_INSET_from_the_walls_so_a_HARDER_PUSH_cannot_false_refuse_the_joint():
    """The lesson of the first cut, pinned so it cannot be re-learned the hard way.

    An arc declared AT the extremes of one hand sweep false-refused within minutes:
    the joint came to rest at raw 218, eleven ticks past an edge taken from a sweep
    the operator had stopped short of, and ``plan_rezero`` correctly reported that
    the joint "cannot be where it says it is". A wall moved 206..218 depending on
    how hard a human pushed — and at least one of these walls may be the TABLE, not
    the joint's own stop, which would make the true travel WIDER still.

    So the declared arc must be a STRICT SUBSET of the unreachable region: inset on
    both sides, never claiming a tick the joint has actually been seen at. Shrinking
    is safe in both directions that matter — it cannot false-refuse a legal position,
    and it cannot claim a tick the joint can really reach. The cost is nothing: the
    arc has only to CONTAIN the seam, and one tick would do.
    """
    arc = arm_spec.rezero_arc(ELBOW)

    assert ARC_MARGIN > 0  # there IS an inset, on both sides
    assert arc.low == LOW_WALL + ARC_MARGIN
    assert arc.high == HIGH_WALL - ARC_MARGIN

    # No tick the joint was ever SEEN at may be inside the arc — that is the
    # false-refusal, and it is what the margin exists to make impossible.
    assert not arc.contains(LOW_WALL)
    assert not arc.contains(HIGH_WALL)
    # ...nor may any tick between a wall and the arc: a harder push lands here.
    assert not arc.contains(arc.low)  # the endpoints are reachable by definition
    assert not arc.contains(arc.high)
    assert not any(arc.contains(t) for t in range(LOW_WALL, arc.low + 1))
    assert not any(arc.contains(t) for t in range(arc.high, HIGH_WALL + 1))

    # And it is still an arc: shrinking it did not cost it the seam.
    assert arc.contains(arc.midpoint)
    assert arc.evicts(arm_spec.rezero_offset(ELBOW))


def test_elbow_flex_offset_is_the_midpoint_of_the_measured_arc():
    """A fresh re-zero writes the arc's midpoint — whatever the arc currently is."""
    assert arm_spec.rezero_offset(ELBOW) == EXPECTED_OFFSET == (ARC_LOW + ARC_HIGH) // 2


def test_offset_is_derived_from_the_arc_not_typed():
    """Correct the arc and the offset follows — the two cannot drift apart.

    Not hypothetical, and not a one-off: on 2026-07-12 the arc was corrected TWICE
    — once into the right frame, once again to inset a margin after the tight
    version false-refused — and each time one tuple changed and the target moved
    with it, without a line of ``rezero_offset`` being touched. Had the offset been
    a typed constant, either correction would have silently left it stale, and a
    stale target is a seam written into a joint's live travel.

    This test is the production-code half of that discipline. The module header is
    the test-code half: the ticks in this file are derived for exactly the same
    reason, because the arc is going to move again.
    """
    arc = arm_spec.rezero_arc(ELBOW)
    assert arc is not None
    assert arm_spec.rezero_offset(ELBOW) == arc.midpoint == arc.offset


def test_the_seam_lands_strictly_inside_the_unreachable_arc():
    """The whole point: the seam must go where the joint physically cannot follow."""
    arc = arm_spec.rezero_arc(ELBOW)
    offset = arm_spec.rezero_offset(ELBOW)
    assert arc.contains(offset)
    assert arc.evicts(offset)
    # ...dead centre, to the tick the integer division allows...
    assert offset - arc.low == arc.width // 2
    assert arc.high - offset == arc.width - arc.width // 2
    # ...which is real clearance on both sides, not one tick: further from either
    # wall than the whole inset the arc was already given.
    assert min(offset - arc.low, arc.high - offset) > ARC_MARGIN


def test_offset_fits_the_registers_sign_magnitude_range():
    """±2047 is the register's whole world; the target sits comfortably inside it."""
    assert abs(arm_spec.rezero_offset(ELBOW)) <= arm_spec.MAX_ENCODER_OFFSET


def test_arc_arithmetic_reconstructs_the_measured_travel():
    """Travel = the whole circle minus the arc — the yardstick every sweep is judged on."""
    arc = arm_spec.rezero_arc(ELBOW)
    assert arc.width == ARC_HIGH - ARC_LOW
    assert arc.travel_ticks == ENCODER_RESOLUTION - arc.width == EXPECTED_TRAVEL
    # The joint can reach strictly MORE than the walls that were actually found,
    # because the arc was inset — never less. An over-claimed arc is a false refusal.
    assert arc.travel_ticks > ENCODER_RESOLUTION - (HIGH_WALL - LOW_WALL)


def test_arc_endpoints_are_reachable_and_the_interior_is_not():
    """``contains`` is the OPEN interval — the endpoints are ticks the joint may hold."""
    arc = arm_spec.rezero_arc(ELBOW)
    assert not arc.contains(ARC_LOW)  # the far edge of the arc, inset from the wall
    assert not arc.contains(ARC_HIGH)  # the near edge
    assert arc.contains(ARC_LOW + 1)
    assert arc.contains(ARC_HIGH - 1)
    assert not arc.contains(REST_RAW)  # its rest position: raw, past the wrap, reachable
    assert not arc.contains(3000)  # in the travel, on the far side of the seam


# --- the frame: RAW ticks vs REPORTED ticks --------------------------------


def test_the_factory_offset_is_85_and_it_is_NOT_zero():
    """The measurement that broke the old reasoning open.

    Every source (and the spike) assumed a factory servo holds 0. All six joints
    of the follower held **85**, straight out of the box — uniform, so a vendor
    default rather than a per-servo calibration. It means a "factory" servo's
    reported ticks were never raw ticks, and the arc measured on one was never a
    raw arc.
    """
    assert arm_spec.FACTORY_ENCODER_OFFSET == FACTORY_OFFSET != 0


def test_a_FACTORY_servos_seam_sits_INSIDE_elbow_flexs_travel():
    """Which is to say: the factory default IS issue #35, and 85 is where it lives.

    ``elbow_flex``'s travel includes the raw band below the arc. A factory servo
    puts its seam at raw 85 — right in it. Nothing is evicted; the joint wraps
    mid-travel; that is the bug.
    """
    arc = arm_spec.rezero_arc(ELBOW)
    assert arm_spec.seam_tick(FACTORY_OFFSET) == FACTORY_OFFSET
    assert not arc.contains(FACTORY_OFFSET)
    assert not arc.evicts(FACTORY_OFFSET)


def test_seam_tick_reduces_a_SIGNED_offset_modulo_4096():
    """The register is signed; the encoder is a circle. −1096 means raw 3000.

    Comparing the signed number straight against a raw arc would place the seam a
    whole turn from where it physically is.
    """
    assert arm_spec.seam_tick(0) == 0
    assert arm_spec.seam_tick(FACTORY_OFFSET) == 85
    assert arm_spec.seam_tick(EXPECTED_OFFSET) == EXPECTED_OFFSET
    assert arm_spec.seam_tick(-1096) == 3000
    assert arm_spec.seam_tick(-1) == 4095


def test_seam_tick_and_offset_for_seam_at_are_inverses():
    """The round-trip the whole re-zero rests on, pinned across the register's range."""
    ticks = (0, 1, FACTORY_OFFSET, ARC_LOW, OUR_ARM_OFFSET, EXPECTED_OFFSET, 2047, 2049, 3000, 4095)
    for tick in ticks:
        assert arm_spec.seam_tick(arm_spec._offset_for_seam_at(tick)) == tick


def test_evicts_is_about_WHERE_THE_SEAM_IS_not_which_number_the_register_holds():
    """The second half of the fix, and the one that saves our arm an EEPROM write.

    Every tick strictly inside the arc — well over a thousand of them — evicts
    ``elbow_flex``'s seam. The midpoint is merely the roomiest. An arm holding ANY
    of them is fixed, and ours holds 1073.
    """
    arc = arm_spec.rezero_arc(ELBOW)

    assert arc.evicts(OUR_ARM_OFFSET)  # our follower: 1073, from the first re-zero
    assert arc.evicts(EXPECTED_OFFSET)  # the canonical midpoint
    assert arc.evicts(ODD_EVICTING_OFFSET)  # a number nobody computed — still evicted
    assert arc.evicts(ARC_LOW + 1)  # one tick inside: ugly, but evicted
    assert arc.evicts(ARC_HIGH - 1)

    assert not arc.evicts(0)  # seam at raw 0 — below the arc, in the travel
    assert not arc.evicts(FACTORY_OFFSET)  # seam at raw 85 — likewise
    assert not arc.evicts(ARC_LOW)  # the arc's own edges are REACHABLE
    assert not arc.evicts(ARC_HIGH)
    assert not arc.evicts(-1096)  # seam at raw 3000 — deep in the far travel


@pytest.mark.parametrize("joint", ["shoulder_pan", "shoulder_lift", "wrist_flex", "gripper"])
def test_non_wrapping_joints_are_refused_as_UNNECESSARY(joint):
    """Four joints do not wrap at all: there is no seam to evict, and they say so."""
    assert arm_spec.rezero_offset(joint) is None
    refusal = arm_spec.rezero_refusal(joint)
    assert "does not need a re-zero" in refusal
    assert joint in refusal


def test_wrist_roll_is_refused_as_IMPOSSIBLE_and_says_why():
    """The distinction that matters: wrist_roll CAN'T be re-zeroed, not "needn't be".

    A re-zero relocates a seam; it can never evict one from a joint whose travel
    covers the whole circle, because eviction needs an arc the joint cannot
    reach and such a joint has none. Collapsing this into the "you don't need
    one" message would teach the operator something false about their arm — and
    would make a permanent, provable impossibility read like an unimplemented
    feature.
    """
    assert arm_spec.rezero_offset("wrist_roll") is None
    refusal = arm_spec.rezero_refusal("wrist_roll")
    assert "RELOCATES" in refusal
    assert "EVICT" in refusal
    assert "SOFT LIMIT" in refusal
    assert "does not need a re-zero" not in refusal
    # And the soft limit it defers to is real and in force.
    assert arm_spec.soft_limit("wrist_roll") is not None


def test_elbow_flex_is_the_only_re_zeroable_joint():
    rezeroable = [j for j in arm_spec.JOINTS if arm_spec.rezero_offset(j) is not None]
    assert rezeroable == [ELBOW]


def test_every_joint_gets_either_an_offset_or_a_reason_never_neither():
    """No joint may answer "no" without saying why. Silence is the failure mode."""
    for joint in arm_spec.JOINTS:
        offset = arm_spec.rezero_offset(joint)
        refusal = arm_spec.rezero_refusal(joint)
        assert (offset is None) != (refusal is None)


def test_unknown_joint_raises_valueerror_not_a_silent_none():
    for fn in (arm_spec.rezero_arc, arm_spec.rezero_offset, arm_spec.rezero_refusal):
        with pytest.raises(ValueError, match="Unknown joint"):
            fn("elbow")  # nearly right, and therefore worth failing loudly


# --- the import-time table guards -----------------------------------------


def test_an_arc_with_no_interior_is_rejected_at_import_time():
    """A one-tick arc has nowhere to PUT the seam — that joint needs a soft limit."""
    with pytest.raises(ValueError, match="nowhere to evict the seam"):
        arm_spec._require_evictable_seam({"x": arm_spec.UnreachableArc(low=100, high=101)})


def test_an_arc_whose_midpoint_is_raw_2048_is_rejected_at_import_time():
    """Raw 2048 is the ONE seam placement sign-magnitude-on-bit-11 cannot express."""
    with pytest.raises(ValueError, match=r"outside the register's"):
        arm_spec._require_evictable_seam({"x": arm_spec.UnreachableArc(low=1, high=4095)})


def test_a_high_arc_uses_the_NEGATIVE_congruent_offset():
    """Seam at raw 3000 is unrepresentable as +3000, but is fine as −1096.

    Modulo 4096 the register reaches every residue but 2048, and this is how: a
    tick above 2047 is expressed as its negative congruent. Getting this wrong
    would reject a perfectly good future arc.
    """
    assert arm_spec._offset_for_seam_at(3000) == 3000 - 4096 == -1096
    assert abs(-1096) <= arm_spec.MAX_ENCODER_OFFSET
    arm_spec._require_evictable_seam({"x": arm_spec.UnreachableArc(low=2900, high=3100)})


def test_malformed_arcs_are_rejected_by_the_dataclass():
    for low, high in ((2107, 207), (100, 100), (-1, 500), (500, 4096)):
        with pytest.raises(ValueError, match="Invalid unreachable arc"):
            arm_spec.UnreachableArc(low=low, high=high)


# --- cross-module single-source pins ---------------------------------------


def test_arm_spec_encoder_constants_agree_with_the_bus():
    """``arm_spec`` restates two bus constants because it must not IMPORT the bus.

    A table of physical facts has no business depending on a serial port
    (``test_arm_spec_module_never_imports_the_bus`` enforces that). The price is
    two restated numbers — and this test is what keeps that price honest, so
    they cannot drift.
    """
    assert arm_spec.ENCODER_TICKS == ENCODER_RESOLUTION
    assert arm_spec.MAX_ENCODER_OFFSET == OFFSET_MAX_MAGNITUDE


# ---------------------------------------------------------------------------
# require_rezeroable — answers with no hardware attached
# ---------------------------------------------------------------------------


def test_require_rezeroable_returns_the_offset_and_the_arc():
    offset, arc = rezero.require_rezeroable(ELBOW)
    assert offset == EXPECTED_OFFSET
    assert (arc.low, arc.high) == (ARC_LOW, ARC_HIGH)


def test_require_rezeroable_refuses_wrist_roll_with_the_full_reason():
    """And it does so as a CliError with a remediation — never a bare ValueError."""
    with pytest.raises(CliError) as exc:
        rezero.require_rezeroable("wrist_roll")
    assert exc.value.code == EXIT_USER_ERROR
    assert "RELOCATES" in exc.value.message
    assert "SOFT LIMIT" in exc.value.message
    assert "elbow_flex" in exc.value.remediation


def test_require_rezeroable_rejects_an_unknown_joint_as_a_user_error():
    with pytest.raises(CliError) as exc:
        rezero.require_rezeroable("nonsense")
    assert exc.value.code == EXIT_USER_ERROR


# ---------------------------------------------------------------------------
# plan_rezero — reads only, and refuses rather than guesses
# ---------------------------------------------------------------------------


def test_plan_reads_the_live_state_and_writes_nothing():
    bus = FakeBus(positions={ELBOW_MOTOR: REST_RAW})  # raw 126; a servo somebody zeroed
    bus.open()

    plan = rezero.plan_rezero(bus, ELBOW_MOTOR, ELBOW)

    assert plan.current_offset == 0
    assert plan.current_seam_tick == 0  # seam at raw 0 — inside the travel
    assert plan.target_offset == EXPECTED_OFFSET
    assert plan.reported_position == REST_RAW
    assert plan.raw_position == REST_RAW  # at offset 0, and ONLY at 0, reported IS raw
    assert plan.already_applied is False
    # Planning is a read. Not one register was touched.
    assert bus.register_writes == []
    assert bus.offset_writes == []
    assert bus.torque_writes == []


def test_a_FACTORY_FRESH_servo_at_offset_85_IS_PLANNED_not_refused():
    """THE case this fix exists for — and the one the old guard blocked outright.

    Every un-touched SO-101 holds ``Ofs = 85`` on every joint. The old
    ``plan_rezero`` refused anything that was "neither the factory 0 nor this
    joint's computed 1073", so on real, factory hardware the verb did not work
    **at all**. It could only ever have run on a servo somebody had already
    hand-zeroed.

    Now it reads the offset, converts out of it, and plans. The shaft is at raw
    126, so a servo holding 85 reports 41 — and the plan must find raw 126 behind
    that, not 41.
    """
    bus = FakeBus(positions={ELBOW_MOTOR: REST_RAW}, offsets={ELBOW_MOTOR: FACTORY_OFFSET})
    bus.open()

    plan = rezero.plan_rezero(bus, ELBOW_MOTOR, ELBOW)

    assert plan.current_offset == FACTORY_OFFSET
    assert plan.current_seam_tick == FACTORY_OFFSET  # the factory seam, INSIDE the travel
    assert plan.reported_position == REST_RAW - FACTORY_OFFSET == 41  # a REPORTED tick
    assert plan.raw_position == REST_RAW  # ...and the raw tick behind it
    assert plan.already_applied is False  # 85 does not evict: there is work to do
    assert plan.target_offset == EXPECTED_OFFSET
    assert bus.register_writes == []


def test_plan_predicts_what_the_servo_will_report_after_the_write():
    """From rest, the target offset must make the servo report ``(rest − target) mod 4096``.

    If this number is wrong, everything downstream of it — the shift probe, the
    operator's sanity check — is decoration.
    """
    bus = FakeBus(positions={ELBOW_MOTOR: REST_RAW})
    bus.open()
    plan = rezero.plan_rezero(bus, ELBOW_MOTOR, ELBOW)
    assert plan.predicted_position == reported_at(REST_RAW)


@pytest.mark.parametrize("raw", [ARC_HIGH, 4000, 4095, 0, REST_RAW, ARC_LOW])
def test_the_whole_travel_becomes_one_contiguous_increasing_interval(raw):
    """The deliverable of issue #35, row by row.

    Walk the raw travel (``arc.high -> 4095 -> |raw seam| -> 0 -> arc.low``) and
    the reported values come out with no discontinuity: the reachable set collapses
    to ONE interval — from ``reported(arc.high)`` to ``reported(arc.low)`` — which a
    ``(min, max)`` pair can honestly describe. Every tick of the travel lands inside
    it, which is what "contiguous" means and what a wrapping encoder denies.
    """
    bus = FakeBus(positions={ELBOW_MOTOR: raw})
    bus.open()

    predicted = rezero.plan_rezero(bus, ELBOW_MOTOR, ELBOW).predicted_position

    assert predicted == reported_at(raw)
    assert reported_at(ARC_HIGH) <= predicted <= reported_at(ARC_LOW)


def test_the_raw_4095_to_0_ROLL_OVER_becomes_a_ONE_TICK_step():
    """The seam, gone — stated as the single fact the whole re-zero is for.

    The two raw ticks either side of the encoder's roll-over are adjacent angles of
    one joint, and after the re-zero the servo reports them as adjacent numbers.
    Before it, they reported ~4095 apart. That step, and nothing else, IS issue #35.
    """

    def predicted_at(raw: int) -> int:
        bus = FakeBus(positions={ELBOW_MOTOR: raw})
        bus.open()
        return rezero.plan_rezero(bus, ELBOW_MOTOR, ELBOW).predicted_position

    assert predicted_at(0) - predicted_at(4095) == 1


def test_OUR_ARM_holding_1073_is_a_NO_OP_not_a_rewrite():
    """The arm on the bench. It holds 1073 and it is DONE — do not touch it.

    1073 came out of the first, frame-confused re-zero. It is not the arc's
    midpoint and it does not have to be: its seam sits at raw 1073, strictly
    inside the unreachable arc, and a torque-off hand sweep proved the travel
    continuous across all 2196 ticks. The seam is out of the joint's travel.
    That IS the goal, and it is met.

    A verb that insisted on its own midpoint would burn an EEPROM write on a
    finite-write part to slide a seam from one unreachable tick to another
    unreachable tick, and the joint could not tell the difference.
    """
    bus = FakeBus(positions={ELBOW_MOTOR: REST_RAW}, offsets={ELBOW_MOTOR: OUR_ARM_OFFSET})
    bus.open()

    plan = rezero.plan_rezero(bus, ELBOW_MOTOR, ELBOW)

    assert plan.already_applied is True  # <- the whole test
    assert plan.current_offset == OUR_ARM_OFFSET
    assert plan.current_seam_tick == OUR_ARM_OFFSET
    assert plan.target_offset == OUR_ARM_OFFSET  # NOT the midpoint: nothing is written
    assert plan.target_offset != EXPECTED_OFFSET  # ...and they really are different
    assert plan.reported_position == reported_at(REST_RAW, OUR_ARM_OFFSET)
    assert plan.raw_position == REST_RAW  # ...and we can still find the shaft
    assert plan.predicted_position == plan.reported_position  # nothing will change
    assert bus.register_writes == []
    assert bus.offset_writes == []


def test_plan_is_idempotent_on_a_joint_holding_the_canonical_midpoint_too():
    """The procedure sends the operator away to power-cycle and come back.

    A second run against an already-re-zeroed joint is the EXPECTED path, not a
    mistake, and it must be recognised rather than re-written — whether the offset
    in force is the midpoint or any other evicting one.
    """
    bus = FakeBus(positions={ELBOW_MOTOR: REST_RAW}, offsets={ELBOW_MOTOR: EXPECTED_OFFSET})
    bus.open()

    plan = rezero.plan_rezero(bus, ELBOW_MOTOR, ELBOW)

    assert plan.already_applied is True
    assert plan.current_offset == EXPECTED_OFFSET
    assert plan.reported_position == reported_at(REST_RAW)  # already reporting corrected
    assert plan.raw_position == REST_RAW  # ...and we can still find the shaft
    assert bus.register_writes == []


def test_ANY_offset_that_already_evicts_the_seam_is_a_no_op():
    """An offset nobody computed — and its seam is already out of the travel.

    The old guard refused this outright ("neither the factory 0 nor this joint's
    computed 1073"). But this seam is strictly inside the arc: the joint cannot
    reach it, cannot cross it, and is therefore already linear. There is nothing
    to do, and "I don't recognise this number" was never a reason to say otherwise.
    """
    bus = FakeBus(positions={ELBOW_MOTOR: REST_RAW}, offsets={ELBOW_MOTOR: ODD_EVICTING_OFFSET})
    bus.open()

    plan = rezero.plan_rezero(bus, ELBOW_MOTOR, ELBOW)

    assert plan.already_applied is True
    assert plan.current_seam_tick == ODD_EVICTING_OFFSET
    assert ODD_EVICTING_OFFSET != EXPECTED_OFFSET  # ...and it is nobody's target
    assert bus.register_writes == []


def test_an_offset_whose_seam_is_IN_the_travel_is_re_zeroed_FROM_ITS_OWN_FRAME():
    """A servo holding −1096 has its seam at raw 3000 — deep in the far travel.

    Not evicted, so there IS work to do. And its reports are shifted by −1096, so
    the plan must convert out of that frame (not the factory's, not the target's)
    to find the shaft. Exercises the negative-offset path, which is where a
    two's-complement or a sign-blind modulo would come apart.
    """
    bus = FakeBus(positions={ELBOW_MOTOR: REST_RAW}, offsets={ELBOW_MOTOR: -1096})
    bus.open()

    plan = rezero.plan_rezero(bus, ELBOW_MOTOR, ELBOW)

    assert plan.current_seam_tick == 3000  # −1096 mod 4096 — a tick, on the circle
    assert plan.already_applied is False
    assert plan.reported_position == reported_at(REST_RAW, -1096) == 1222  # (126 + 1096)
    assert plan.raw_position == REST_RAW  # ...converted back out of ITS frame
    assert plan.target_offset == EXPECTED_OFFSET


def test_the_raw_conversion_WRAPS_modulo_4096():
    """``reported + offset`` genuinely runs past 4096, and folds. It must.

    A joint reporting 4000 on a servo holding +200 is physically at raw 104 —
    there is no tick 4200. Without the modulo the plan would place the shaft
    outside the encoder entirely and then compare that nonsense against the arc.
    """
    assert rezero.raw_from_reported(4000, 200) == 104
    assert rezero.raw_from_reported(4095, 1) == 0
    assert rezero.raw_from_reported(0, -1) == 4095  # and it folds the other way too

    bus = FakeBus(positions={ELBOW_MOTOR: 104}, offsets={ELBOW_MOTOR: 200})
    bus.open()

    plan = rezero.plan_rezero(bus, ELBOW_MOTOR, ELBOW)

    assert plan.reported_position == 4000  # (104 − 200) mod 4096 — near the top
    assert plan.raw_position == 104  # ...and back down, over the wrap
    assert plan.already_applied is False  # seam at raw 200 is BELOW the arc: in the travel
    assert plan.target_offset == EXPECTED_OFFSET


def test_plan_refuses_a_joint_that_reports_a_position_it_cannot_physically_hold():
    """A raw position inside the unreachable arc means the arc does not fit this arm.

    Either the servo is not the joint we think it is, or the travel has changed.
    Either way the offset about to be written comes from a table that does not
    describe the hardware — and writing it would put the seam somewhere the joint
    CAN go, making issue #35 worse, persistently, in EEPROM.

    This is the guard that survives, and it is the one that would have caught the
    frame bug had the numbers been less lucky: it is the only check that compares
    a live reading against the arc, so it is the only one that can notice they are
    in different frames.

    It is also the guard that FALSE-REFUSED when the arc was declared at the raw
    extremes of a single sweep — which is why the arc now carries a margin, and why
    the impossible position below is derived from the arc rather than typed.
    """
    bus = FakeBus(positions={ELBOW_MOTOR: IMPOSSIBLE_RAW})  # dead centre of the arc
    bus.open()

    with pytest.raises(CliError) as exc:
        rezero.plan_rezero(bus, ELBOW_MOTOR, ELBOW)

    assert exc.value.code == EXIT_ENV_ERROR
    assert "INSIDE the arc" in exc.value.message
    assert "Refusing to write" in exc.value.message
    assert "raw = reported + offset" in exc.value.remediation  # it names the frame
    assert bus.register_writes == []


def test_the_impossible_position_guard_judges_the_RAW_tick_not_the_reported_one():
    """The same guard, in a frame: a servo at Ofs=85 reports 85 ticks BELOW its shaft.

    A servo whose shaft is genuinely in the unreachable arc must be caught
    whatever offset it happens to be holding, because the arc is a fact about the
    shaft. Judging the reported tick instead would let a servo 85 ticks deep into
    an arc it cannot reach look perfectly healthy.
    """
    bus = FakeBus(positions={ELBOW_MOTOR: IMPOSSIBLE_RAW}, offsets={ELBOW_MOTOR: FACTORY_OFFSET})
    bus.open()

    with pytest.raises(CliError) as exc:
        rezero.plan_rezero(bus, ELBOW_MOTOR, ELBOW)

    reported = reported_at(IMPOSSIBLE_RAW, FACTORY_OFFSET)
    assert exc.value.code == EXIT_ENV_ERROR
    assert f"raw encoder position {IMPOSSIBLE_RAW}" in exc.value.message
    assert f"it reports {reported} while holding an offset of {FACTORY_OFFSET}" in exc.value.message
    # The two really are different numbers — the guard is judging the right one.
    assert reported != IMPOSSIBLE_RAW


def test_raw_from_reported_inverts_the_servos_own_correction():
    """``Actual = (Present + Ofs) mod 4096``, on every path — not just exotic ones."""
    assert rezero.raw_from_reported(REST_RAW, 0) == REST_RAW  # identity ONLY at offset 0
    assert rezero.raw_from_reported(41, FACTORY_OFFSET) == REST_RAW  # the factory frame
    assert rezero.raw_from_reported(3149, OUR_ARM_OFFSET) == REST_RAW  # our arm's frame
    # ...and the target frame, whose reported tick moves whenever the arc does.
    assert rezero.raw_from_reported(reported_at(REST_RAW), EXPECTED_OFFSET) == REST_RAW


# ---------------------------------------------------------------------------
# apply_rezero — an EEPROM write, and NOT a move
# ---------------------------------------------------------------------------


def test_apply_writes_the_offset_and_reads_it_back():
    bus = FakeBus(positions={ELBOW_MOTOR: REST_RAW})
    bus.open()

    read_back = rezero.apply_rezero(bus, ELBOW_MOTOR, EXPECTED_OFFSET)

    assert read_back == EXPECTED_OFFSET
    assert bus.offset_writes == [{"motor": ELBOW_MOTOR, "offset": EXPECTED_OFFSET}]


def test_apply_COMMANDS_NO_MOTION():
    """The load-bearing safety property of this entire task.

    ``elbow_flex`` rests at raw ~126, PAST its wrap. A linear goal — any linear
    goal — would rotate it the long way round, through its whole travel, into a
    wall. So the write surface must contain no goal position at all, and this
    test pins that at the register level rather than trusting the code to
    continue not doing it.
    """
    bus = FakeBus(positions={ELBOW_MOTOR: REST_RAW})
    bus.open()

    rezero.apply_rezero(bus, ELBOW_MOTOR, EXPECTED_OFFSET)

    assert bus.position_writes == []
    assert all(w["addr"] != ADDR_GOAL_POSITION for w in bus.register_writes)


def test_apply_never_ENERGISES_the_joint():
    """Torque only ever goes OFF. A servo must not be holding while its frame moves."""
    bus = FakeBus(positions={ELBOW_MOTOR: REST_RAW})
    bus.open()

    rezero.apply_rezero(bus, ELBOW_MOTOR, EXPECTED_OFFSET)

    assert bus.torque_writes  # it definitely touched torque...
    assert all(w["on"] is False for w in bus.torque_writes)  # ...only ever downward
    torque_values = [w["value"] for w in bus.register_writes if w["addr"] == ADDR_TORQUE_ENABLE]
    assert torque_values == [0, 0]  # clear_overload, then write_offset's own torque-off


def test_apply_performs_the_full_EEPROM_lock_dance():
    """Unlock -> write addr 31 -> re-lock. Skip it and the write REVERTS (PR #21)."""
    bus = FakeBus(positions={ELBOW_MOTOR: REST_RAW})
    bus.open()

    rezero.apply_rezero(bus, ELBOW_MOTOR, EXPECTED_OFFSET)

    eeprom = [
        (w["addr"], w["value"])
        for w in bus.register_writes
        if w["addr"] in (ADDR_LOCK, ADDR_HOMING_OFFSET)
    ]
    assert eeprom == [
        (ADDR_LOCK, 0),  # unlock
        (ADDR_HOMING_OFFSET, EXPECTED_OFFSET),  # the offset, sign-magnitude on the wire
        (ADDR_LOCK, 1),  # re-lock
    ]


def test_apply_writes_addr_31_and_NOTHING_else_in_eeprom():
    """LeRobot's write_calibration also writes min/max angle limits (addr 9/11).

    Copying it wholesale would CLAMP the servo's goals to a narrower window —
    shrinking the very reachable set this re-zero exists to recover. The write
    surface is pinned to an allow-list so widening it is a deliberate act.
    """
    bus = FakeBus(positions={ELBOW_MOTOR: REST_RAW})
    bus.open()

    rezero.apply_rezero(bus, ELBOW_MOTOR, EXPECTED_OFFSET)

    touched = {w["addr"] for w in bus.register_writes}
    assert touched == {ADDR_TORQUE_ENABLE, ADDR_LOCK, ADDR_HOMING_OFFSET}


def test_apply_clears_a_latched_overload_before_the_write():
    """A joint that just fought a wall is EXACTLY the joint you are asked to re-zero.

    ``write_offset``'s first act is a plain ``enable_torque(False)``, which a
    latched servo answers with the overload bit still set — so without the
    ``clear_overload`` first, the write would raise before it ever opened the
    EEPROM. That is not hypothetical: driving into a wall is how ``elbow_flex``'s
    arc was measured.
    """
    bus = FakeBus(positions={ELBOW_MOTOR: REST_RAW}).fail_with_overload_on_op(1)
    bus.open()

    read_back = rezero.apply_rezero(bus, ELBOW_MOTOR, EXPECTED_OFFSET)

    assert read_back == EXPECTED_OFFSET  # it recovered and wrote


def test_a_latched_motor_defeats_a_write_that_SKIPS_clear_overload():
    """The negative control for the test above — proving the guard is load-bearing."""
    bus = FakeBus(positions={ELBOW_MOTOR: REST_RAW}).fail_with_overload_on_op(1)
    bus.open()

    with pytest.raises(OverloadError):
        bus.write_offset(ELBOW_MOTOR, EXPECTED_OFFSET)  # the primitive, unguarded


def test_an_unrepresentable_offset_is_rejected_before_ANY_wire_traffic():
    """A rejected offset must not leave the joint limp as a side effect of failing."""
    bus = FakeBus(positions={ELBOW_MOTOR: REST_RAW})
    bus.open()

    with pytest.raises(CliError) as exc:
        rezero.apply_rezero(bus, ELBOW_MOTOR, 2048)

    assert exc.value.code == EXIT_USER_ERROR
    # clear_overload ran (that is a torque-OFF, which is safe); the EEPROM never opened.
    assert bus.offset_writes == []
    assert all(w["addr"] != ADDR_HOMING_OFFSET for w in bus.register_writes)
    assert all(w["addr"] != ADDR_LOCK for w in bus.register_writes)


# --- describe_shift: the free, early probe of the open question ------------


def test_shift_matches_the_prediction_when_the_offset_wraps():
    bus = FakeBus(positions={ELBOW_MOTOR: REST_RAW})
    bus.open()
    plan = rezero.plan_rezero(bus, ELBOW_MOTOR, ELBOW)
    rezero.apply_rezero(bus, ELBOW_MOTOR, plan.target_offset)

    shift = rezero.describe_shift(plan, bus.read_position(ELBOW_MOTOR))

    assert shift["observed_position"] == reported_at(REST_RAW)
    assert shift["as_predicted"] is True
    assert shift["in_range"] is True
    assert shift["unchanged"] is False


def test_shift_matches_the_prediction_from_the_FACTORY_frame_too():
    """The probe has to survive the frame conversion, or it warns on every real arm.

    A factory servo at ``Ofs = 85`` reports 41 from rest. After the write it must
    report ``(126 − target) mod 4096`` — the same PLACE as a servo that started from
    any other offset, because the SHAFT did not move; only the frame did. A probe
    that predicted from the reported 41 instead of the raw 126 would be off by
    exactly the factory offset and would cry wolf on every single fresh arm.
    """
    bus = FakeBus(positions={ELBOW_MOTOR: REST_RAW}, offsets={ELBOW_MOTOR: FACTORY_OFFSET})
    bus.open()
    plan = rezero.plan_rezero(bus, ELBOW_MOTOR, ELBOW)
    rezero.apply_rezero(bus, ELBOW_MOTOR, plan.target_offset)

    shift = rezero.describe_shift(plan, bus.read_position(ELBOW_MOTOR))

    assert shift["predicted_position"] == reported_at(REST_RAW)
    assert shift["observed_position"] == reported_at(REST_RAW)
    assert shift["as_predicted"] is True
    assert shift["unchanged"] is False


def test_shift_catches_the_signed_reading_IMMEDIATELY():
    """Under ``offset_wraps=False`` the servo reports a NEGATIVE tick from rest.

    ``rest − target`` is below zero, and a position register cannot hold a negative
    number — so this alone already proves the corrected position is an unwrapped
    signed subtraction and the seam never moved. The sweep is still the proof; this
    is the free warning that arrives 30 seconds earlier.
    """
    bus = FakeBus(positions={ELBOW_MOTOR: REST_RAW}, offset_wraps=False)
    bus.open()
    plan = rezero.plan_rezero(bus, ELBOW_MOTOR, ELBOW)
    rezero.apply_rezero(bus, ELBOW_MOTOR, plan.target_offset)

    shift = rezero.describe_shift(plan, bus.read_position(ELBOW_MOTOR))

    assert shift["observed_position"] == REST_RAW - EXPECTED_OFFSET < 0  # impossible
    assert shift["in_range"] is False
    assert shift["as_predicted"] is False


# ---------------------------------------------------------------------------
# analyse_sweep — pure judgement, no bus, no clock, no servo
# ---------------------------------------------------------------------------


def _analyse(positions, offset_in_force=EXPECTED_OFFSET):
    return rezero.analyse_sweep(
        positions,
        joint=ELBOW,
        motor=ELBOW_MOTOR,
        offset_in_force=offset_in_force,
        arc=arm_spec.rezero_arc(ELBOW),
    )


def test_a_clean_full_sweep_is_a_PASS():
    """Wall to wall, monotonic, no jump: the seam is gone. Issue #35 is fixed."""
    report = _analyse(_full_sweep())

    assert report.continuous is True
    assert report.conclusive is True
    assert report.monotonic is True
    assert report.seam_evicted is True
    assert report.verdict == rezero.VERDICT_SEAM_EVICTED
    assert report.failed is False
    assert report.largest_jump == SWEEP_STEP  # the hand's own step, and nothing bigger
    # The extremes are the two walls, seen through the re-zeroed frame — and the
    # near wall reports LOWER than the far one, which is the seam being gone.
    assert report.minimum == reported_at(ARC_HIGH)
    assert report.maximum == reported_at(FULL_SWEEP_END_RAW)
    assert report.expected_travel == EXPECTED_TRAVEL  # derived from the arc, not passed


def test_a_sweep_of_OUR_ARM_at_1073_is_a_PASS_not_an_INCONCLUSIVE():
    """The report must not deny the measurement in front of it.

    Our follower holds 1073, which is not the arc's midpoint and does not need to
    be: its seam sits at raw 1073, strictly inside the arc. Judging ``rezeroed`` by
    ``offset == expected_offset`` would call this arm un-re-zeroed and downgrade the
    very sweep that PROVED the fix works to "inconclusive" — on the grounds that a
    register held the wrong integer.
    """
    report = _analyse(_full_sweep(OUR_ARM_OFFSET), offset_in_force=OUR_ARM_OFFSET)

    assert report.seam_tick == OUR_ARM_OFFSET
    assert report.rezeroed is True  # <- the whole test
    assert report.continuous is True
    assert report.conclusive is True
    assert report.verdict == rezero.VERDICT_SEAM_EVICTED
    # It still reports the canonical target, it just does not JUDGE by it.
    assert report.expected_offset == EXPECTED_OFFSET
    assert report.offset_in_force != report.expected_offset  # ...and they differ


def test_a_4095_to_0_wrap_under_a_written_offset_is_the_STOP_condition():
    """The seam is still in the travel while the seam-evicting offset is in force.

    That can only mean the servo does NOT reduce the corrected position modulo
    4096 — so the offset merely relabels positions, and the re-zero achieves
    nothing at all. This is the verdict the entire wave exists to be able to
    reach.
    """
    report = _analyse([4000, 4050, 4090, 5, 50, 100])

    assert report.continuous is False
    assert report.failed is True
    assert report.verdict == rezero.VERDICT_SEAM_NOT_EVICTED
    assert report.seam_evicted is False
    assert report.largest_jump == 4085
    assert report.discontinuities == ((2, 4090, 5),)
    assert "STOP" in report.describe()
    assert "does NOT reduce" in report.describe()


def test_the_MASKED_signed_jump_is_caught_too_and_it_is_why_the_threshold_is_not_2048():
    """The subtlest way the caveat can bite, and the one a lazy threshold misses.

    If the firmware does a plain signed subtraction AND ``read_position``'s
    ``& 0x0FFF`` folds the negative result back into range, the report falls from
    ``4095 − H`` to ``H`` at the seam: a jump of ``4095 − 2H``, which for the target
    this arc yields is comfortably UNDER the tempting 2048 threshold. A 2048
    threshold would call this a pass and tell the operator the fix works.

    Note the jump SHRINKS as the seam approaches raw 2048 — so it is a function of
    the arc, and pinning it as a literal would be pinning the wrong thing.
    """
    top = 4095 - EXPECTED_OFFSET  # what the servo reports just BEFORE the seam...
    jump = top - EXPECTED_OFFSET  # ...and how far it then falls
    assert jump < 2048  # the trap, stated: a 2048 threshold would sail past this

    report = _analyse([top - 38, top - 8, top, EXPECTED_OFFSET, EXPECTED_OFFSET + 43])

    assert report.largest_jump == jump
    assert rezero.DISCONTINUITY_TICKS <= jump  # ...and OUR threshold catches it
    assert report.continuous is False
    assert report.failed is True


def test_an_impossible_negative_reading_fails_INDEPENDENTLY_of_the_jump():
    """A position register cannot hold a negative value. Seeing one is proof enough.

    Belt and braces on purpose: the jump test and the range test would each catch
    the unwrapped-signed firmware alone, and neither is asked to carry it by
    itself.
    """
    report = _analyse([-1031, -900, -850, -800])

    assert report.out_of_range == (-1031, -900, -850, -800)
    assert report.continuous is False  # despite every delta being a tame 47 ticks
    assert report.largest_jump < rezero.DISCONTINUITY_TICKS
    assert report.failed is True
    assert "IMPOSSIBLE READS" in report.describe()


def test_an_unre_zeroed_discontinuous_sweep_is_a_BASELINE_not_a_failure():
    """The bug, photographed. Exactly what a factory elbow_flex should look like."""
    report = _analyse([4000, 4090, 5, 100], offset_in_force=0)

    assert report.rezeroed is False
    assert report.continuous is False
    assert report.verdict == rezero.VERDICT_SEAM_PRESENT_BASELINE
    assert report.failed is False  # informative, expected, exit 0
    assert "BASELINE" in report.describe()
    assert "That jump IS issue #35" in report.describe()


def test_a_short_clean_sweep_is_INCONCLUSIVE_never_a_pass():
    """The most dangerous outcome available, and the one it must never claim.

    A sweep that moved the joint 190 ticks of its ~2400 and saw no seam has proved
    NOTHING — of course it saw no seam; it never went near where the seam would
    be. Reporting that as a pass would close the open question with a lie.
    """
    report = _analyse(list(range(1000, 1200, 10)))

    assert report.span < EXPECTED_TRAVEL * rezero.MIN_COVERAGE  # nowhere near enough
    assert report.continuous is True
    assert report.rezeroed is True
    assert report.conclusive is False
    assert report.seam_evicted is False  # <- the whole point
    assert report.verdict == rezero.VERDICT_INCONCLUSIVE
    assert report.failed is False
    assert "proves nothing" in report.describe()
    assert "one hard stop ALL THE WAY to the other" in report.describe()


def test_a_clean_sweep_of_a_joint_whose_seam_is_STILL_IN_ITS_TRAVEL_is_INCONCLUSIVE():
    """The seam was never evicted, so nothing about the eviction was tested.

    Note the judgement is on WHERE THE SEAM IS, not on which number the register
    held: a factory servo at ``Ofs = 85`` has its seam at raw 85, inside the
    joint's travel, and is exactly as un-re-zeroed as one at 0.
    """
    report = _analyse(_full_sweep(), offset_in_force=FACTORY_OFFSET)

    assert report.seam_tick == FACTORY_OFFSET  # below the arc: in the reachable band
    assert report.rezeroed is False
    assert report.continuous is True
    assert report.conclusive is True  # it covered the travel...
    assert report.seam_evicted is False  # ...but there was no eviction to prove
    assert report.verdict == rezero.VERDICT_INCONCLUSIVE
    assert "was NOT re-zeroed" in report.describe()
    assert f"seam at raw tick {FACTORY_OFFSET}" in report.describe()


def test_a_DISCONTINUOUS_sweep_is_conclusive_however_short_it_was():
    """Seeing the seam is proof it is there. No extra travel can un-see it."""
    report = _analyse([4090, 5])
    assert report.span > 0
    assert report.conclusive is True
    assert report.failed is True


def test_a_hand_that_backs_up_is_reported_but_never_FAILS_the_sweep():
    """A human wobbles. That makes the sweep non-monotonic and changes nothing else.

    Monotonicity is DESCRIPTIVE. The verdict turns on continuity, which a hand
    cannot fake in either direction — and a verb that failed an honest sweep
    because the operator's grip slipped would teach them to distrust it.
    """
    forward = _full_sweep()
    slip = len(forward) // 2  # halfway along, the grip goes...
    backed_up = forward[slip - 8 : slip][::-1]  # ...eight samples the wrong way...
    report = _analyse(forward[:slip] + backed_up + forward[slip - 8 :])  # ...then on

    assert report.monotonic is False  # it went both ways...
    assert report.continuous is True  # ...but never jumped
    assert report.conclusive is True  # ...and it still covered the travel
    assert report.seam_evicted is True  # ...so the proof still stands
    assert report.verdict == rezero.VERDICT_SEAM_EVICTED


def test_encoder_jitter_does_not_count_as_a_reversal():
    """A few ticks of noise is not the hand changing direction."""
    report = _analyse([1000, 1005, 1002, 1030, 1028, 1060])
    assert report.monotonic is True


def test_a_single_sample_cannot_be_judged_and_says_so():
    """Continuity is a claim about the change BETWEEN samples. One sample has none.

    Returning a cheerful "no discontinuities found" for a one-sample sweep would
    be the exact false pass this module exists to prevent.
    """
    with pytest.raises(CliError) as exc:
        _analyse([2048])
    assert exc.value.code == EXIT_ENV_ERROR
    assert "too few to judge" in exc.value.message


def test_a_passing_report_states_the_travel_it_measured_IN_BOTH_FRAMES():
    """This is the measurement that corrects the arc — so it must be usable as one.

    A sweep runs in the REPORTED frame (that is all a servo can speak), and the arc
    is RAW. A report that handed back only the reported endpoints would be handing
    the next person exactly the numbers that caused this bug: ticks in one frame,
    destined for a table in the other. So it converts, and it says which is which.
    """
    report = _analyse(_full_sweep())
    text = report.describe()

    assert "Travel measured" in text
    assert f"{report.span} ticks" in text
    assert "RAW" in text
    # The sweep ran from the arc's high wall, over the wrap, to its low one — and
    # the report hands those RAW walls back, not the reported ticks it saw them as.
    assert f"RAW {ARC_HIGH} .. {FULL_SWEEP_END_RAW}" in text
    assert "mod 4096" in text  # and it tells you how to convert


def test_report_json_payload_is_complete_and_serialisable():
    import json

    report = _analyse([4000, 4090, 5, 100])
    payload = report.as_dict()

    json.dumps(payload)  # must not raise
    assert payload["verdict"] == rezero.VERDICT_SEAM_NOT_EVICTED
    assert payload["failed"] is True
    assert payload["discontinuities"] == [{"index": 1, "before": 4090, "after": 5}]
    assert payload["discontinuity_threshold"] == rezero.DISCONTINUITY_TICKS
    # The frame is in the payload, not just the prose.
    assert payload["seam_tick"] == EXPECTED_OFFSET
    assert payload["unreachable_arc"] == [ARC_LOW, ARC_HIGH]
    assert payload["expected_travel"] == EXPECTED_TRAVEL


# ---------------------------------------------------------------------------
# sweep — driven against BOTH readings of the undocumented firmware behaviour
# ---------------------------------------------------------------------------


def test_sweep_PROVES_the_seam_moved_when_the_offset_wraps():
    """The world we hope we live in: the hand walks the travel, the report is smooth.

    The raw shaft crosses 4095->0 during this sweep — it MUST, that is the whole
    travel — and the reported position does not so much as blink.
    """
    bus = _rezeroed_bus(offset_wraps=True)

    report = rezero.sweep(bus, ELBOW_MOTOR, ELBOW, samples=FULL_SWEEP_SAMPLES)

    assert report.rezeroed is True
    assert report.continuous is True
    assert report.conclusive is True
    assert report.seam_evicted is True
    assert report.verdict == rezero.VERDICT_SEAM_EVICTED
    assert report.largest_jump == SWEEP_STEP  # the hand's own step, and nothing bigger
    assert report.minimum == reported_at(ARC_HIGH)  # the near wall, in the corrected frame
    assert max(report.samples) >= reported_at(REST_RAW)  # ...all the way past rest


def test_sweep_FAILS_LOUDLY_when_the_offset_does_not_wrap():
    """The world the caveat warns about — and the reason this verb exists.

    ``offset_wraps=False`` is the pessimistic reading of the undocumented
    firmware behaviour: the offset only RELABELS positions and the discontinuity
    stays pinned to the physical angle where the magnet rolls over. The re-zero
    achieves nothing, and the operator MUST be told so rather than shipping on
    top of it. Under this reading the verification must fail — that failure is
    the feature.
    """
    bus = _rezeroed_bus(offset_wraps=False)

    report = rezero.sweep(bus, ELBOW_MOTOR, ELBOW, samples=FULL_SWEEP_SAMPLES)

    assert report.rezeroed is True
    assert report.continuous is False  # <- the seam is STILL in the travel
    assert report.seam_evicted is False
    assert report.failed is True
    assert report.verdict == rezero.VERDICT_SEAM_NOT_EVICTED
    assert report.out_of_range  # it reported positions no register can hold
    assert "STOP" in report.describe()


def test_sweep_of_a_FACTORY_joint_shows_the_bug_itself():
    """Run it BEFORE the write and you photograph issue #35: a ~4000-tick jump."""
    bus = HandMovedBus()  # offset 0 — a factory servo
    bus.open()

    report = rezero.sweep(bus, ELBOW_MOTOR, ELBOW, samples=FULL_SWEEP_SAMPLES)

    assert report.rezeroed is False
    assert report.continuous is False
    assert report.largest_jump > 4000  # the raw 4095 -> 0 seam, in the open
    assert report.verdict == rezero.VERDICT_SEAM_PRESENT_BASELINE
    assert report.failed is False  # a baseline is not a failure


def test_sweep_DE_ENERGISES_the_joint_and_never_re_energises_it():
    """The human's hand is on this joint. It must be limp, and it must STAY limp."""
    bus = _rezeroed_bus()

    rezero.sweep(bus, ELBOW_MOTOR, ELBOW, samples=10)

    assert bus.torque_writes  # torque was definitely addressed
    assert all(w["on"] is False for w in bus.torque_writes)  # only ever downward
    assert bus.position_writes == []  # and nothing was ever commanded


def test_sweep_clears_a_latched_overload_before_polling():
    """A joint that just stalled against a wall would otherwise refuse to go limp."""
    bus = _rezeroed_bus()
    bus.fail_with_overload_on_op(1)

    report = rezero.sweep(bus, ELBOW_MOTOR, ELBOW, samples=10)

    assert len(report.samples) == 10  # it recovered and swept


def test_sweep_reports_the_offset_ACTUALLY_in_force_not_the_one_we_hoped_for():
    """The report must say which frame its samples are in, not assume the good one."""
    bus = HandMovedBus(offsets={ELBOW_MOTOR: 0})
    bus.open()

    report = rezero.sweep(bus, ELBOW_MOTOR, ELBOW, samples=10)

    assert report.offset_in_force == 0
    assert report.expected_offset == EXPECTED_OFFSET
    assert report.rezeroed is False


def test_sweep_of_OUR_ARM_at_1073_PROVES_the_seam_moved():
    """End to end, on the offset the real follower is actually carrying.

    1073 is not the midpoint and never will be. Its seam is at raw 1073, inside the
    arc, so the joint cannot cross it — and the sweep says so, in full, as
    ``seam-evicted``. This is the run that was done on hardware on 2026-07-12
    (``monotonic: True, discontinuities: 0`` over 2196 ticks); if the code cannot
    return a PASS for it, the code is disagreeing with the arm.
    """
    bus = HandMovedBus(offsets={ELBOW_MOTOR: OUR_ARM_OFFSET})
    bus.open()

    report = rezero.sweep(bus, ELBOW_MOTOR, ELBOW, samples=FULL_SWEEP_SAMPLES)

    assert report.offset_in_force == OUR_ARM_OFFSET
    assert report.seam_tick == OUR_ARM_OFFSET
    assert report.rezeroed is True
    assert report.continuous is True
    assert report.seam_evicted is True
    assert report.verdict == rezero.VERDICT_SEAM_EVICTED
    # The near wall, as OUR arm's frame reports it — not as the midpoint's would.
    assert report.minimum == reported_at(ARC_HIGH, OUR_ARM_OFFSET)


def test_sweep_invokes_the_on_sample_hook_for_every_poll():
    """The operator needs to SEE the position moving, or they cannot tell they are
    driving the joint from merely holding a dead arm."""
    bus = _rezeroed_bus()
    seen: list[tuple[int, int]] = []

    report = rezero.sweep(
        bus, ELBOW_MOTOR, ELBOW, samples=12, on_sample=lambda i, p: seen.append((i, p))
    )

    assert [i for i, _ in seen] == list(range(12))
    assert [p for _, p in seen] == list(report.samples)


def test_sweep_rejects_a_sample_count_that_cannot_have_a_delta():
    bus = _rezeroed_bus()
    with pytest.raises(CliError) as exc:
        rezero.sweep(bus, ELBOW_MOTOR, ELBOW, samples=1)
    assert exc.value.code == EXIT_USER_ERROR


def test_sweep_refuses_a_joint_that_cannot_be_re_zeroed():
    """Sweeping wrist_roll would measure a seam no offset could ever have moved."""
    bus = _rezeroed_bus()
    with pytest.raises(CliError) as exc:
        rezero.sweep(bus, 5, "wrist_roll", samples=10)
    assert exc.value.code == EXIT_USER_ERROR
    assert "RELOCATES" in exc.value.message


def test_sweep_does_not_sleep_against_a_fake_bus():
    """A simulated shaft advances per READ, not per second — pacing it would only
    make the suite sleep. Same seam as ``gentle_move``'s ``_needs_pacing``."""
    bus = _rezeroed_bus()
    assert rezero._needs_pacing(bus) is False


# --- samples_for -----------------------------------------------------------


def test_samples_for_converts_a_duration_into_polls():
    assert rezero.samples_for(30.0, 0.05) == 600
    assert rezero.samples_for(1.0, 0.05) == 20


def test_samples_for_rejects_a_duration_too_short_to_have_a_delta():
    with pytest.raises(CliError) as exc:
        rezero.samples_for(0.05, 0.05)
    assert exc.value.code == EXIT_USER_ERROR
    assert "too few to detect a discontinuity" in exc.value.message
