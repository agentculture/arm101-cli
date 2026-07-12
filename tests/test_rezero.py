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
**commands no motion, on any path.** ``elbow_flex`` currently rests at raw ~126,
PAST its wrap, so a linear goal would rotate it the long way round through its
whole travel and into a wall. Several tests assert the write surface directly —
no goal-position register write, ever, and torque only ever going OFF.
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

#: The offset the spike derives (arc (126, 2020), midpoint 1073).
EXPECTED_OFFSET = 1073

#: STS3215 Goal_Position. Must NEVER appear in this verb's write surface.
ADDR_GOAL_POSITION = 42


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
        Raw tick the shaft begins at. Defaults to ``elbow_flex``'s hard wall
        (2020), the end a human is told to start from.
    ticks_per_read:
        How far the hand advances the joint between two polls. Positive winds
        toward the seam (2020 -> 4095 -> 0 -> 126), which is the direction that
        crosses it.
    """

    def __init__(
        self,
        *args,
        start_raw: int = 2020,
        ticks_per_read: int = 25,
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


def test_elbow_flex_offset_matches_the_spike_arithmetic():
    """The offset is 1073 — the midpoint of the measured unreachable arc (126, 2020)."""
    assert arm_spec.rezero_offset(ELBOW) == EXPECTED_OFFSET


def test_offset_is_derived_from_the_arc_not_typed():
    """Correct the arc and the offset follows — the two cannot drift apart.

    The far wall has never been measured (nothing could see across the seam), so
    the arc WILL be corrected once ``--verify`` measures it. If the offset were a
    typed constant, that correction would silently leave it stale.
    """
    arc = arm_spec.rezero_arc(ELBOW)
    assert arc is not None
    assert arm_spec.rezero_offset(ELBOW) == arc.midpoint


def test_the_seam_lands_strictly_inside_the_unreachable_arc():
    """The whole point: the seam must go where the joint physically cannot follow."""
    arc = arm_spec.rezero_arc(ELBOW)
    offset = arm_spec.rezero_offset(ELBOW)
    assert arc.contains(offset)
    # ...with real clearance on both sides, not by one tick.
    assert offset - arc.low == 947
    assert arc.high - offset == 947


def test_offset_fits_the_registers_sign_magnitude_range():
    """±2047 is the register's whole world; 1073 sits comfortably inside it."""
    assert abs(arm_spec.rezero_offset(ELBOW)) <= arm_spec.MAX_ENCODER_OFFSET


def test_arc_arithmetic_reconstructs_the_measured_travel():
    """Travel = 4096 − arc width = 2202 ticks, exactly as the spike derives it."""
    arc = arm_spec.rezero_arc(ELBOW)
    assert arc.width == 1894
    assert arc.travel_ticks == 2202


def test_arc_endpoints_are_reachable_and_the_interior_is_not():
    """``contains`` is the OPEN interval — the endpoints are the joint's own walls."""
    arc = arm_spec.rezero_arc(ELBOW)
    assert not arc.contains(126)  # its rest position, past the wrap
    assert not arc.contains(2020)  # its measured hard wall
    assert arc.contains(127)
    assert arc.contains(2019)
    assert not arc.contains(3000)  # in the travel, on the far side of the seam


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
    for low, high in ((2020, 126), (100, 100), (-1, 500), (500, 4096)):
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
    assert (arc.low, arc.high) == (126, 2020)


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
    bus = FakeBus(positions={ELBOW_MOTOR: 126})
    bus.open()

    plan = rezero.plan_rezero(bus, ELBOW_MOTOR, ELBOW)

    assert plan.current_offset == 0
    assert plan.target_offset == EXPECTED_OFFSET
    assert plan.reported_position == 126
    assert plan.raw_position == 126  # factory offset: reported IS raw, no assumption
    assert plan.already_applied is False
    # Planning is a read. Not one register was touched.
    assert bus.register_writes == []
    assert bus.offset_writes == []
    assert bus.torque_writes == []


def test_plan_predicts_the_position_the_spike_predicts():
    """From rest at raw 126, an offset of 1073 must make the servo report 3149.

    Spike §3 step 7: reported = (126 − 1073) mod 4096 = 3149. If this number is
    wrong, everything downstream of it is decoration.
    """
    bus = FakeBus(positions={ELBOW_MOTOR: 126})
    bus.open()
    plan = rezero.plan_rezero(bus, ELBOW_MOTOR, ELBOW)
    assert plan.predicted_position == 3149


@pytest.mark.parametrize(
    ("raw", "predicted"),
    [(2020, 947), (4060, 2987), (4095, 3022), (0, 3023), (126, 3149)],
)
def test_the_whole_travel_becomes_one_contiguous_increasing_interval(raw, predicted):
    """Spike §3's table, row by row: 947 -> 2987 -> 3022 -> 3023 -> 3149.

    Strictly increasing, no discontinuity — the reachable set collapses to the
    single interval [947, 3149], which a (min, max) pair can honestly describe.
    That is the entire deliverable of issue #35.
    """
    bus = FakeBus(positions={ELBOW_MOTOR: raw})
    bus.open()
    assert rezero.plan_rezero(bus, ELBOW_MOTOR, ELBOW).predicted_position == predicted


def test_plan_is_idempotent_on_an_already_re_zeroed_joint():
    """The procedure sends the operator away to power-cycle and come back.

    A second run against an already-re-zeroed joint is therefore the EXPECTED
    path, not a mistake, and it must be recognised rather than re-written.
    """
    bus = FakeBus(positions={ELBOW_MOTOR: 126}, offsets={ELBOW_MOTOR: EXPECTED_OFFSET})
    bus.open()

    plan = rezero.plan_rezero(bus, ELBOW_MOTOR, ELBOW)

    assert plan.already_applied is True
    assert plan.current_offset == EXPECTED_OFFSET
    assert plan.reported_position == 3149  # the servo is already reporting corrected
    assert plan.raw_position == 126  # ...and we can still find the shaft
    assert bus.register_writes == []


def test_plan_refuses_a_servo_holding_an_UNRECOGNISED_offset():
    """We cannot honestly convert its reports to raw ticks, so we do not pretend to.

    Writing a new offset on top of an unknown one would bury the problem in
    EEPROM instead of surfacing it.
    """
    bus = FakeBus(positions={ELBOW_MOTOR: 126}, offsets={ELBOW_MOTOR: 500})
    bus.open()

    with pytest.raises(CliError) as exc:
        rezero.plan_rezero(bus, ELBOW_MOTOR, ELBOW)

    assert exc.value.code == EXIT_ENV_ERROR
    assert "already holds an encoder offset of 500" in exc.value.message
    assert bus.register_writes == []  # refused BEFORE touching anything


def test_plan_refuses_a_joint_that_reports_a_position_it_cannot_physically_hold():
    """A raw position inside the unreachable arc means the arc does not fit this arm.

    Either the servo is not the joint we think it is, or the travel has changed.
    Either way the offset about to be written comes from a table that does not
    describe the hardware — and writing it would put the seam somewhere the joint
    CAN go, making issue #35 worse, persistently, in EEPROM.
    """
    bus = FakeBus(positions={ELBOW_MOTOR: 1500})  # dead centre of (126, 2020)
    bus.open()

    with pytest.raises(CliError) as exc:
        rezero.plan_rezero(bus, ELBOW_MOTOR, ELBOW)

    assert exc.value.code == EXIT_ENV_ERROR
    assert "INSIDE the arc" in exc.value.message
    assert "Refusing to write" in exc.value.message
    assert bus.register_writes == []


def test_raw_from_reported_is_the_identity_at_the_factory_offset():
    """The case the whole procedure is designed around carries NO assumption."""
    assert rezero.raw_from_reported(126, 0) == 126
    assert rezero.raw_from_reported(3149, EXPECTED_OFFSET) == 126  # and it inverts


# ---------------------------------------------------------------------------
# apply_rezero — an EEPROM write, and NOT a move
# ---------------------------------------------------------------------------


def test_apply_writes_the_offset_and_reads_it_back():
    bus = FakeBus(positions={ELBOW_MOTOR: 126})
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
    bus = FakeBus(positions={ELBOW_MOTOR: 126})
    bus.open()

    rezero.apply_rezero(bus, ELBOW_MOTOR, EXPECTED_OFFSET)

    assert bus.position_writes == []
    assert all(w["addr"] != ADDR_GOAL_POSITION for w in bus.register_writes)


def test_apply_never_ENERGISES_the_joint():
    """Torque only ever goes OFF. A servo must not be holding while its frame moves."""
    bus = FakeBus(positions={ELBOW_MOTOR: 126})
    bus.open()

    rezero.apply_rezero(bus, ELBOW_MOTOR, EXPECTED_OFFSET)

    assert bus.torque_writes  # it definitely touched torque...
    assert all(w["on"] is False for w in bus.torque_writes)  # ...only ever downward
    torque_values = [w["value"] for w in bus.register_writes if w["addr"] == ADDR_TORQUE_ENABLE]
    assert torque_values == [0, 0]  # clear_overload, then write_offset's own torque-off


def test_apply_performs_the_full_EEPROM_lock_dance():
    """Unlock -> write addr 31 -> re-lock. Skip it and the write REVERTS (PR #21)."""
    bus = FakeBus(positions={ELBOW_MOTOR: 126})
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
    bus = FakeBus(positions={ELBOW_MOTOR: 126})
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
    bus = FakeBus(positions={ELBOW_MOTOR: 126}).fail_with_overload_on_op(1)
    bus.open()

    read_back = rezero.apply_rezero(bus, ELBOW_MOTOR, EXPECTED_OFFSET)

    assert read_back == EXPECTED_OFFSET  # it recovered and wrote


def test_a_latched_motor_defeats_a_write_that_SKIPS_clear_overload():
    """The negative control for the test above — proving the guard is load-bearing."""
    bus = FakeBus(positions={ELBOW_MOTOR: 126}).fail_with_overload_on_op(1)
    bus.open()

    with pytest.raises(OverloadError):
        bus.write_offset(ELBOW_MOTOR, EXPECTED_OFFSET)  # the primitive, unguarded


def test_an_unrepresentable_offset_is_rejected_before_ANY_wire_traffic():
    """A rejected offset must not leave the joint limp as a side effect of failing."""
    bus = FakeBus(positions={ELBOW_MOTOR: 126})
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
    bus = FakeBus(positions={ELBOW_MOTOR: 126})
    bus.open()
    plan = rezero.plan_rezero(bus, ELBOW_MOTOR, ELBOW)
    rezero.apply_rezero(bus, ELBOW_MOTOR, plan.target_offset)

    shift = rezero.describe_shift(plan, bus.read_position(ELBOW_MOTOR))

    assert shift["observed_position"] == 3149
    assert shift["as_predicted"] is True
    assert shift["in_range"] is True
    assert shift["unchanged"] is False


def test_shift_catches_the_signed_reading_IMMEDIATELY():
    """Under ``offset_wraps=False`` the servo reports −947 from rest — impossible.

    A position register cannot hold a negative number, so this alone already
    proves the corrected position is an unwrapped signed subtraction and the
    seam never moved. The sweep is still the proof; this is the free warning
    that arrives 30 seconds earlier.
    """
    bus = FakeBus(positions={ELBOW_MOTOR: 126}, offset_wraps=False)
    bus.open()
    plan = rezero.plan_rezero(bus, ELBOW_MOTOR, ELBOW)
    rezero.apply_rezero(bus, ELBOW_MOTOR, plan.target_offset)

    shift = rezero.describe_shift(plan, bus.read_position(ELBOW_MOTOR))

    assert shift["observed_position"] == 126 - EXPECTED_OFFSET == -947
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
        expected_offset=EXPECTED_OFFSET,
        expected_travel=2202,
    )


def test_a_clean_full_sweep_is_a_PASS():
    """947 -> 3149, monotonic, no jump: the seam is gone. Issue #35 is fixed."""
    report = _analyse(list(range(947, 3150, 25)))

    assert report.continuous is True
    assert report.conclusive is True
    assert report.monotonic is True
    assert report.seam_evicted is True
    assert report.verdict == rezero.VERDICT_SEAM_EVICTED
    assert report.failed is False
    assert report.largest_jump == 25
    assert (report.minimum, report.maximum) == (947, 3147)


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
    ``& 0x0FFF`` folds the negative result back into range, the discontinuity at
    the seam is only ~1949 ticks — comfortably UNDER the tempting 2048 threshold.
    A 2048 threshold would call this a pass and tell the operator the fix works.
    """
    jump = 3022 - 1073
    assert jump == 1949 < 2048  # the trap, stated
    report = _analyse([2900, 2960, 3022, 1073, 1140, 1200])

    assert report.largest_jump == 1949
    assert rezero.DISCONTINUITY_TICKS <= 1949
    assert report.continuous is False
    assert report.failed is True


def test_an_impossible_negative_reading_fails_INDEPENDENTLY_of_the_jump():
    """A position register cannot hold a negative value. Seeing one is proof enough.

    Belt and braces on purpose: the jump test and the range test would each catch
    the unwrapped-signed firmware alone, and neither is asked to carry it by
    itself.
    """
    report = _analyse([-947, -900, -850, -800])

    assert report.out_of_range == (-947, -900, -850, -800)
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

    A sweep that moved the joint 200 of its 2202 ticks and saw no seam has proved
    NOTHING — of course it saw no seam; it never went near where the seam would
    be. Reporting that as a pass would close the open question with a lie.
    """
    report = _analyse(list(range(1000, 1200, 10)))

    assert report.continuous is True
    assert report.rezeroed is True
    assert report.conclusive is False
    assert report.seam_evicted is False  # <- the whole point
    assert report.verdict == rezero.VERDICT_INCONCLUSIVE
    assert report.failed is False
    assert "proves nothing" in report.describe()
    assert "one hard stop ALL THE WAY to the other" in report.describe()


def test_a_clean_sweep_of_a_joint_that_was_never_re_zeroed_is_INCONCLUSIVE():
    """No offset was in force, so nothing about the offset was tested."""
    report = _analyse(list(range(947, 3150, 25)), offset_in_force=0)

    assert report.continuous is True
    assert report.conclusive is True  # it covered the travel...
    assert report.seam_evicted is False  # ...but there was no re-zero to prove
    assert report.verdict == rezero.VERDICT_INCONCLUSIVE
    assert "was NOT re-zeroed" in report.describe()


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
    positions = (
        list(range(947, 2000, 25)) + list(range(2000, 1800, -25)) + list(range(1800, 3150, 25))
    )
    report = _analyse(positions)

    assert report.monotonic is False  # it went both ways...
    assert report.continuous is True  # ...but never jumped
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


def test_the_report_measures_the_far_wall_for_the_first_time():
    """Free, and genuinely new: nothing could see across the seam before.

    The arc table was built on a LOWER BOUND for travel (2202 ticks) because
    ``arm explore`` could not drive past the wrap. A passing sweep measures the
    real number, and says so — which is what lets the arc, and the offset derived
    from it, be corrected.
    """
    report = _analyse(list(range(947, 3300, 25)))  # a longer travel than we assumed
    text = report.describe()
    assert "Far wall measured for the first time" in text
    assert f"{report.span} ticks" in text


def test_report_json_payload_is_complete_and_serialisable():
    import json

    report = _analyse([4000, 4090, 5, 100])
    payload = report.as_dict()

    json.dumps(payload)  # must not raise
    assert payload["verdict"] == rezero.VERDICT_SEAM_NOT_EVICTED
    assert payload["failed"] is True
    assert payload["discontinuities"] == [{"index": 1, "before": 4090, "after": 5}]
    assert payload["discontinuity_threshold"] == rezero.DISCONTINUITY_TICKS


# ---------------------------------------------------------------------------
# sweep — driven against BOTH readings of the undocumented firmware behaviour
# ---------------------------------------------------------------------------


def test_sweep_PROVES_the_seam_moved_when_the_offset_wraps():
    """The world we hope we live in: the hand walks the travel, the report is smooth.

    The raw shaft crosses 4095->0 during this sweep — it MUST, that is the whole
    travel — and the reported position does not so much as blink.
    """
    bus = _rezeroed_bus(offset_wraps=True)

    report = rezero.sweep(bus, ELBOW_MOTOR, ELBOW, samples=90)

    assert report.rezeroed is True
    assert report.continuous is True
    assert report.conclusive is True
    assert report.seam_evicted is True
    assert report.verdict == rezero.VERDICT_SEAM_EVICTED
    assert report.largest_jump == 25  # the hand's own step, and nothing bigger
    assert report.minimum == 947  # the wall, in the corrected frame
    assert max(report.samples) >= 3100  # ...all the way past rest


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

    report = rezero.sweep(bus, ELBOW_MOTOR, ELBOW, samples=90)

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

    report = rezero.sweep(bus, ELBOW_MOTOR, ELBOW, samples=90)

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
