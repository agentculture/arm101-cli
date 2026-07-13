"""The ROLLING FRAME — roll the seam ahead of the joint so travel is unbounded by it.

The three claims under test, in the order they have to be true:

1. **Centring.** From *any* raw position, one temporary offset puts the joint at the
   reported half-turn — the seam then sits half a turn away, the furthest it can
   possibly be, with ~2048 clear commandable ticks on each side. Including from the
   one raw position where the register cannot express that (raw 0, whose offset
   would have to be the unrepresentable residue 2048).
2. **Rolling.** When a creep nears the reported bound the frame RE-CENTRES, and the
   creep carries on. Total travel is therefore bounded by nothing at all — certainly
   not by the 4096-tick scale a servo can report in.
3. **Journalled.** Every offset write is durable on disk *before* it goes on the
   wire, so a process killed mid-roll leaves an arm somebody can put back.

Every expectation below is derived from :mod:`arm101.hardware.ticks`,
:mod:`arm101.hardware.arm_spec` and :mod:`arm101.hardware.gentle`, never transcribed
from them — a test that copies the constant it is checking cannot fail when the
constant is wrong.
"""

from __future__ import annotations

import pytest

from arm101.cli._errors import EXIT_ENV_ERROR, EXIT_USER_ERROR, CliError
from arm101.hardware import gentle
from arm101.hardware.arm_spec import FACTORY_ENCODER_OFFSET
from arm101.hardware.bus import (
    ADDR_HOMING_OFFSET,
    ADDR_LOCK,
    ADDR_TORQUE_ENABLE,
    FakeBus,
    OverloadError,
    decode_offset,
    encode_offset,
)
from arm101.hardware.journal import (
    DISPOSITION_COMMITTED,
    DISPOSITION_RESTORED,
    CalibrationJournal,
    require_clean,
)
from arm101.hardware.limits import HALF_TURN, EndObservation, LimitVerdict, TravelEnd, signed_delta
from arm101.hardware.rolling_frame import (
    CENTRE_TICK,
    DEFAULT_RECENTRE_MARGIN,
    FALLBACK_CENTRE_TICK,
    MAX_HEADROOM,
    RollingFrame,
    centring_offset,
    headroom_at,
)
from arm101.hardware.ticks import (
    ENCODER_TICKS,
    MAX_ENCODER_OFFSET,
    TICK_MAX,
    TICK_MIN,
    reported_from_raw,
    seam_tick,
)
from tests._rolling_servo import RollingServoBus

JOINT = "elbow_flex"
MOTOR = 3


def _goal_position_addr() -> int:
    """``Goal_Position``'s address, DERIVED from the bus rather than typed.

    A test that hard-codes 42 and a bus that writes 43 agree about nothing.
    """
    bus = FakeBus(positions={1: 2048})
    bus.open()
    bus.write_goal_position(1, 2048)
    return bus.register_writes[-1]["addr"]


ADDR_GOAL_POSITION = _goal_position_addr()


def make_journal(tmp_path, name: str = "calibration-journal.jsonl") -> CalibrationJournal:
    return CalibrationJournal(tmp_path / name)


def open_bus(raw: int = 2048, offset: int = FACTORY_ENCODER_OFFSET, **kwargs) -> RollingServoBus:
    """A rolling servo whose shaft sits at *raw* and which holds *offset*."""
    bus = RollingServoBus(positions={MOTOR: raw}, offsets={MOTOR: offset}, ids=[MOTOR], **kwargs)
    bus.open()
    return bus


def creep(frame: RollingFrame, bus: RollingServoBus, *, direction: int, ticks: int) -> None:
    """Drive *frame*'s joint *ticks* further out, in chunks, exactly as t6's probe will.

    The one rule the rolling frame imposes on a mover: **re-centre BETWEEN moves,
    never during one.** A ``gentle_move`` whose reported frame shifted under it
    mid-flight would check its arrival against a target that no longer means what it
    did when it was computed. So a creep is a sequence of short moves, each one
    asking the frame for a goal that is guaranteed to be inside it.
    """
    chunk = 300
    travelled = 0
    while travelled < ticks:
        step = min(chunk, ticks - travelled)
        target = frame.goal(direction, step)
        gentle.gentle_move(
            bus,
            frame.motor,
            target,
            min_angle=TICK_MIN,
            max_angle=TICK_MAX,
            allow_motion=True,
        )
        frame.sync()
        travelled += step


# ---------------------------------------------------------------------------
# 1. Centring — the arithmetic, enumerated over the WHOLE encoder
# ---------------------------------------------------------------------------


def test_centring_puts_every_raw_position_at_the_reported_centre():
    """AC1, exhaustively: all 4096 raw positions, no exceptions and no near-misses."""
    for raw in range(ENCODER_TICKS):
        offset, centre = centring_offset(raw)
        assert reported_from_raw(raw, offset) == centre, raw


def test_every_centring_offset_is_a_value_the_register_can_actually_HOLD():
    """The trap. An offset the servo silently rejects would probe a frame nobody is in."""
    for raw in range(ENCODER_TICKS):
        offset, _centre = centring_offset(raw)
        assert abs(offset) <= MAX_ENCODER_OFFSET, raw
        # ...and it round-trips through the servo's own sign-magnitude codec.
        assert decode_offset(encode_offset(offset)) == offset, raw


def test_exactly_one_raw_position_cannot_be_centred_at_the_half_turn():
    """Raw 0 — and ONLY raw 0. Enumerated, not assumed.

    Centring raw ``r`` at reported ``C`` needs ``Ofs = (r - C) mod 4096``. The
    register is sign-magnitude on bit 11, so residue 2048 is the single value it
    cannot hold (neither ``+2048`` nor ``-2048`` fits in an 11-bit magnitude). That
    residue is required at exactly one raw tick out of 4096 — and with ``C`` at the
    half-turn, that tick is 0.
    """
    unrepresentable_residue = MAX_ENCODER_OFFSET + 1
    expected = {(CENTRE_TICK + unrepresentable_residue) % ENCODER_TICKS}

    fell_back = {raw for raw in range(ENCODER_TICKS) if centring_offset(raw)[1] != CENTRE_TICK}

    assert fell_back == expected == {0}


def test_the_fallback_centre_costs_no_headroom_at_all():
    """Centring raw 0 one tick SHORT of the half-turn is free — it is the mirror image.

    ``C = 2048`` leaves ``4095 - 2048 = 2047`` ticks above and ``2048`` below;
    ``C = 2047`` leaves 2048 above and 2047 below. The *worst* direction is 2047
    ticks either way, so the one tick is a relabelling, not a loss. Which is also why
    the fallback is 2047 and not 2049 — whose worst direction is 2046, a tick worse.
    """

    def worst(centre: int) -> int:
        return min(headroom_at(centre, +1), headroom_at(centre, -1))

    assert worst(FALLBACK_CENTRE_TICK) == worst(CENTRE_TICK) == MAX_HEADROOM
    assert worst(CENTRE_TICK + 1) < MAX_HEADROOM


def test_every_centred_frame_puts_the_seam_HALF_A_TURN_from_the_joint():
    """The escape, stated as the invariant it is: nothing can be further away.

    Exactly half a turn for 4095 of the 4096 raw ticks; one tick short of it for raw
    0, whose centre is one tick short of the half-turn. That single tick is the ENTIRE
    price of the unrepresentable residue — and it is paid in seam distance, not in
    commandable headroom, which the next test shows is unchanged.
    """
    for raw in range(ENCODER_TICKS):
        offset, centre = centring_offset(raw)
        distance = abs(signed_delta(seam_tick(offset), raw))
        expected = HALF_TURN if centre == CENTRE_TICK else HALF_TURN - 1
        assert distance == expected, raw
        assert distance >= HALF_TURN - 1, raw


def test_no_centred_frame_ever_leaves_the_seam_where_the_joint_IS():
    for raw in range(ENCODER_TICKS):
        offset, _centre = centring_offset(raw)
        assert seam_tick(offset) != raw, raw


def test_a_centred_frame_promises_the_same_clear_travel_in_both_directions():
    """~2048 ticks each way — the point of centring, checked at every raw tick."""
    for raw in range(ENCODER_TICKS):
        _offset, centre = centring_offset(raw)
        assert headroom_at(centre, +1) >= MAX_HEADROOM, raw
        assert headroom_at(centre, -1) >= MAX_HEADROOM, raw


def test_centring_rejects_anything_that_is_not_a_raw_tick():
    for bad in (-1, ENCODER_TICKS, TICK_MAX + 1):
        with pytest.raises(ValueError):
            centring_offset(bad)


# ---------------------------------------------------------------------------
# Opening a frame against a bus
# ---------------------------------------------------------------------------


def test_open_centres_the_joint_at_the_reported_half_turn(tmp_path):
    bus = open_bus(raw=3900)  # near the top of the raw scale, holding the factory Ofs
    with RollingFrame(bus, make_journal(tmp_path), joint=JOINT, motor=MOTOR) as frame:
        assert bus.read_position(MOTOR) == CENTRE_TICK
        assert frame.reported == CENTRE_TICK
        assert frame.origin_raw == 3900
        assert frame.displacement == 0
        # The shaft did not move: an EEPROM write is not a motion command.
        assert bus.true_raw(MOTOR) == 3900


def test_open_centres_a_joint_sitting_on_the_ONE_impossible_raw_tick(tmp_path):
    """Raw 0 — the position whose centring offset the register cannot express.

    Handled deliberately (one tick short of the half-turn), never by crashing, and
    never by letting ``write_offset`` reject the value while the probe carries on
    believing it is in a frame it is not in.
    """
    bus = open_bus(raw=0, offset=0)
    with RollingFrame(bus, make_journal(tmp_path), joint=JOINT, motor=MOTOR) as frame:
        assert frame.centre == FALLBACK_CENTRE_TICK
        assert bus.read_position(MOTOR) == FALLBACK_CENTRE_TICK
        assert frame.headroom(+1) >= MAX_HEADROOM
        assert frame.headroom(-1) >= MAX_HEADROOM


def test_open_works_from_every_raw_position_the_encoder_can_hold(tmp_path):
    """No raw tick is a special case at the BUS boundary either.

    Strides of 7 — coprime with 4096, so the walk visits the whole circle, seam,
    fallback tick and all.
    """
    for raw in range(0, ENCODER_TICKS, 7):
        bus = open_bus(raw=raw)
        journal = make_journal(tmp_path, f"j{raw}.jsonl")
        with RollingFrame(bus, journal, joint=JOINT, motor=MOTOR) as frame:
            assert frame.raw == raw
            assert abs(signed_delta(frame.seam_raw, raw)) >= HALF_TURN - 1
            assert frame.reported in (CENTRE_TICK, FALLBACK_CENTRE_TICK)
            assert frame.headroom(+1) >= MAX_HEADROOM
            assert frame.headroom(-1) >= MAX_HEADROOM


def test_open_leaves_a_STANDING_GOAL_at_the_joints_own_position(tmp_path):
    """No stale goal survives the frame change — or the next torque-on is a lurch.

    ``Goal_Position`` is a REPORTED tick. Move the frame under it and the very same
    number names a different physical angle; the servo notices the instant torque
    comes back, and bolts for it.
    """
    bus = open_bus(raw=1000)
    bus.write_goal_position(MOTOR, 3800)  # a goal from the OLD frame, still latched

    with RollingFrame(bus, make_journal(tmp_path), joint=JOINT, motor=MOTOR) as frame:
        assert bus.reported_goal(MOTOR) == frame.reported

        # Prove it: energise and poll. A joint whose standing goal names where it
        # already is does not move.
        bus.enable_torque(MOTOR, True)
        for _ in range(20):
            bus.read_info(MOTOR)
        assert bus.true_raw(MOTOR) == 1000
        assert bus.net_travel(MOTOR) == 0


def test_a_stale_goal_really_WOULD_lurch(tmp_path):
    """The mutation check: this fake CAN see the bug the hold-in-place goal prevents.

    Same setup, except the frame's hold-in-place goal is overwritten with the stale
    one. The joint bolts. A fake that could not show this could not vouch for the
    test above that says it does not happen.
    """
    bus = open_bus(raw=1000)
    with RollingFrame(bus, make_journal(tmp_path), joint=JOINT, motor=MOTOR):
        bus.write_goal_position(MOTOR, 3800)  # put the stale goal back
        bus.enable_torque(MOTOR, True)
        for _ in range(20):
            bus.read_info(MOTOR)
        assert bus.true_raw(MOTOR) != 1000
        assert abs(bus.net_travel(MOTOR)) > 100


def test_open_touches_only_the_registers_it_is_allowed_to(tmp_path):
    """Offset (31), torque (40), lock (55), goal (42). NEVER 9 or 11 — those clamp goals."""
    bus = open_bus(raw=500)
    with RollingFrame(bus, make_journal(tmp_path), joint=JOINT, motor=MOTOR) as frame:
        frame.recentre()
    touched = {write["addr"] for write in bus.register_writes}
    assert touched <= {ADDR_HOMING_OFFSET, ADDR_TORQUE_ENABLE, ADDR_LOCK, ADDR_GOAL_POSITION}
    assert 9 not in touched and 11 not in touched


def test_open_refuses_a_motor_whose_calibration_is_ALREADY_in_flight(tmp_path):
    """``require_clean`` first, always: a frame stacked on an unresolved one loses the original."""
    journal = make_journal(tmp_path)
    journal.begin(joint=JOINT, motor=MOTOR, original_offset=999)

    bus = open_bus(raw=500)
    with pytest.raises(CliError) as excinfo:
        RollingFrame(bus, journal, joint=JOINT, motor=MOTOR).open()

    assert excinfo.value.code == EXIT_ENV_ERROR
    assert "require_clean" in excinfo.value.remediation
    assert not bus.offset_writes  # nothing was written on top of it


# ---------------------------------------------------------------------------
# 3. Journalled first — AC3
# ---------------------------------------------------------------------------


class _OffsetWriteFails(RollingServoBus):
    """A bus that takes every write except the one that matters, from the *n*-th on."""

    def __init__(self, *args, fail_after: int = 0, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.fail_after = fail_after

    def write_offset(self, motor: int, offset: int) -> None:
        if len(self.offset_writes) >= self.fail_after:
            raise CliError(code=EXIT_ENV_ERROR, message="wire fault", remediation="check the cable")
        super().write_offset(motor, offset)


def test_the_original_offset_is_on_disk_BEFORE_the_first_wire_write(tmp_path):
    """AC3. The journal is written first, or it protects nothing at all."""
    bus = _OffsetWriteFails(positions={MOTOR: 700}, offsets={MOTOR: FACTORY_ENCODER_OFFSET})
    bus.open()
    journal = make_journal(tmp_path)

    with pytest.raises(CliError):
        RollingFrame(bus, journal, joint=JOINT, motor=MOTOR).open()

    entry = journal.dirty_entry_for(MOTOR)
    assert entry is not None
    assert entry.original_offset == FACTORY_ENCODER_OFFSET  # THE number nothing else records

    # ...and the temporary offset it was ABOUT to write is named too, so a recovery
    # knows exactly what may be sitting in the servo. (The second record is the
    # in-process rollback the frame then attempted — journalled before ITS write too,
    # which is why it is there even though the bus refused it.)
    shifted, _centre = centring_offset(700)  # 700 is where the SHAFT is: a raw tick
    assert entry.temporary_offsets[0] == shifted
    assert entry.temporary_offsets[-1] == FACTORY_ENCODER_OFFSET
    assert not bus.offset_writes  # ...and not one of them ever reached the servo


def test_a_RE_CENTRE_that_dies_on_the_wire_is_already_on_disk_too(tmp_path):
    """Not just the opening shift — the rolling ones, where a crash is likeliest."""
    bus = _OffsetWriteFails(
        positions={MOTOR: 2048}, offsets={MOTOR: FACTORY_ENCODER_OFFSET}, fail_after=1
    )
    bus.open()
    journal = make_journal(tmp_path)

    frame = RollingFrame(bus, journal, joint=JOINT, motor=MOTOR)
    frame.open()  # first shift lands
    with pytest.raises(CliError):
        frame.recentre()  # second one does not

    entry = journal.dirty_entry_for(MOTOR)
    assert entry is not None
    assert len(entry.temporary_offsets) == 2  # BOTH are named on disk
    assert len(bus.offset_writes) == 1  # only one ever reached the servo


def test_every_re_centre_of_a_real_creep_is_journalled_in_order(tmp_path):
    bus = open_bus(raw=2048)
    journal = make_journal(tmp_path)

    with RollingFrame(bus, journal, joint=JOINT, motor=MOTOR) as frame:
        creep(frame, bus, direction=+1, ticks=5000)

        entry = journal.dirty_entry_for(MOTOR)
        assert entry is not None
        assert len(entry.temporary_offsets) == 1 + frame.recentres
        assert [write["offset"] for write in bus.offset_writes] == list(entry.temporary_offsets)
        assert len(set(entry.temporary_offsets)) == len(entry.temporary_offsets)  # all distinct


def test_a_crash_mid_roll_is_recoverable_from_the_JOURNAL_ALONE(tmp_path):
    """The point of the whole transaction: a fresh process can put the arm back."""
    bus = open_bus(raw=3000)
    journal = make_journal(tmp_path)

    frame = RollingFrame(bus, journal, joint=JOINT, motor=MOTOR)
    frame.open()
    frame.recentre()
    # ...and here the process dies. No restore, no `finally`, nothing.

    assert bus.read_offset(MOTOR) != FACTORY_ENCODER_OFFSET  # the arm IS mis-calibrated
    assert journal.is_dirty()

    require_clean(bus, CalibrationJournal(journal.path))  # a fresh journal, off disk alone

    assert bus.read_offset(MOTOR) == FACTORY_ENCODER_OFFSET
    assert not CalibrationJournal(journal.path).is_dirty()


# ---------------------------------------------------------------------------
# 2. Rolling — AC2, the central claim
# ---------------------------------------------------------------------------


def test_a_creep_that_reaches_the_reported_bound_RE_CENTRES_AND_CONTINUES(tmp_path):
    """AC2, and the whole feature in one test.

    Six thousand ticks — one and a half turns, and half again as far as the servo's
    entire reported scale. Without the roll the joint runs out of commandable frame
    at ~2048 ticks and stops while still physically free: issue #43, exactly.
    """
    bus = open_bus(raw=2048)
    distance = 6000

    with RollingFrame(bus, make_journal(tmp_path), joint=JOINT, motor=MOTOR) as frame:
        creep(frame, bus, direction=+1, ticks=distance)

        assert frame.recentres >= 2, "a 6000-tick creep cannot fit in one 4096-tick frame"
        # The frame's accounting agrees with the shaft's own odometer EXACTLY — across
        # the raw seam it rolled through and the offsets it rolled through.
        assert frame.displacement == bus.net_travel(MOTOR)
        assert frame.displacement == pytest.approx(
            distance, abs=gentle.DEFAULT_LOAD_WATCH.arrival_tolerance
        )
        assert frame.travelled > ENCODER_TICKS  # unbounded by the frame: past a full turn
        assert frame.full_turn

        # Every goal ever commanded sat inside the reported scale: no move was ever
        # asked to cross the seam.
        commanded = [write["position"] for write in bus.position_writes]
        assert all(TICK_MIN <= position <= TICK_MAX for position in commanded)


def test_without_the_roll_that_same_creep_would_have_run_out_of_FRAME(tmp_path):
    """The counterfactual — proof the re-centre was NECESSARY, not merely performed."""
    bus = open_bus(raw=2048)
    with RollingFrame(bus, make_journal(tmp_path), joint=JOINT, motor=MOTOR) as frame:
        opening_offset = frame.offset
        opening_reported = frame.reported
        creep(frame, bus, direction=+1, ticks=3000)

        # Seen through the frame the run STARTED in, this joint — which has just
        # travelled 3000 ticks OUTWARD — reports a position BELOW where it began.
        # That is the seam wearing a limit's clothes, and it is the bug.
        unrolled = reported_from_raw(frame.raw, opening_offset)
        assert unrolled < opening_reported

        # And the goal it would have needed next, in that original frame, is a tick
        # the servo's goal register simply cannot hold.
        needed = opening_reported + frame.displacement
        assert needed > TICK_MAX
        with pytest.raises(CliError) as excinfo:
            bus.write_goal_position(MOTOR, needed)
        assert excinfo.value.code == EXIT_USER_ERROR


def test_a_creep_the_other_way_reports_a_NEGATIVE_displacement(tmp_path):
    """The sign contract ``EndObservation`` enforces: a LOW end travels down."""
    bus = open_bus(raw=2048)
    with RollingFrame(bus, make_journal(tmp_path), joint=JOINT, motor=MOTOR) as frame:
        creep(frame, bus, direction=-1, ticks=5000)

        assert frame.displacement < 0
        assert frame.displacement == bus.net_travel(MOTOR)
        assert frame.recentres >= 2

        # Which is exactly what a LOW-end observation needs — and it is accepted.
        observation = EndObservation(
            joint=JOINT,
            end=TravelEnd.LOW,
            verdict=LimitVerdict.EDGE,
            origin_raw=frame.origin_raw,
            displacement=max(frame.displacement, -ENCODER_TICKS),
            pose="t5",
        )
        assert observation.end.sign * observation.displacement >= 0


def test_displacement_survives_the_RAW_seam(tmp_path):
    """Raw 4000, plus 200 ticks, reads as raw 104. That is not a 3896-tick retreat."""
    bus = open_bus(raw=4000)
    with RollingFrame(bus, make_journal(tmp_path), joint=JOINT, motor=MOTOR) as frame:
        creep(frame, bus, direction=+1, ticks=600)

        assert frame.raw < 1000  # it rolled straight through the raw 4095 -> 0 seam
        assert frame.recentres == 0  # ...and did not need a re-centre to do it
        assert frame.displacement > 0
        assert frame.displacement == bus.net_travel(MOTOR)


def test_the_joint_does_not_MOVE_when_the_frame_re_centres(tmp_path):
    """Writing EEPROM shifts the frame of reference. It does not turn the shaft."""
    bus = open_bus(raw=1234)
    with RollingFrame(bus, make_journal(tmp_path), joint=JOINT, motor=MOTOR) as frame:
        before_raw, before_displacement = frame.raw, frame.displacement
        frame.recentre()

        assert bus.true_raw(MOTOR) == before_raw
        assert frame.raw == before_raw
        assert frame.displacement == before_displacement
        assert frame.reported in (CENTRE_TICK, FALLBACK_CENTRE_TICK)


def test_a_re_centre_leaves_no_stale_goal_behind_either(tmp_path):
    bus = open_bus(raw=2048)
    with RollingFrame(bus, make_journal(tmp_path), joint=JOINT, motor=MOTOR) as frame:
        creep(frame, bus, direction=+1, ticks=1800)
        frame.recentre()

        assert bus.reported_goal(MOTOR) == frame.reported
        settled = bus.true_raw(MOTOR)
        bus.enable_torque(MOTOR, True)
        for _ in range(20):
            bus.read_info(MOTOR)
        assert bus.true_raw(MOTOR) == settled


# ---------------------------------------------------------------------------
# The re-centre trigger
# ---------------------------------------------------------------------------


def test_the_frame_recentres_only_when_it_can_no_longer_promise_the_move(tmp_path):
    """Not every step — an EEPROM cell has a finite number of writes in it."""
    bus = open_bus(raw=2048)
    with RollingFrame(bus, make_journal(tmp_path), joint=JOINT, motor=MOTOR) as frame:
        creep(frame, bus, direction=+1, ticks=ENCODER_TICKS)
        # A full turn of travel through a frame that clears ~2048 ticks each side:
        # two or three writes. Not eighty.
        assert 1 <= frame.recentres <= 3


def test_the_margin_leaves_room_for_everything_gentle_move_can_do_in_one_go():
    """The trigger threshold is a claim about the MOVER, so it is checked against the mover."""
    assert DEFAULT_RECENTRE_MARGIN > gentle._DEFAULT_STEP_TICKS + gentle._DEFAULT_BACKOFF_TICKS
    assert DEFAULT_RECENTRE_MARGIN > gentle.DEFAULT_LOAD_WATCH.arrival_tolerance
    # ...and it is a small fraction of the frame, so it costs a handful of EEPROM
    # writes over a whole probe, not one per step.
    assert DEFAULT_RECENTRE_MARGIN < MAX_HEADROOM // 4


def test_goal_never_returns_a_tick_outside_the_reported_scale(tmp_path):
    """Whatever it is asked for, the goal it hands back is one the servo can hold."""
    for direction in (+1, -1):
        for step in (1, 25, 300, MAX_HEADROOM):
            bus = open_bus(raw=2048)
            journal = make_journal(tmp_path, f"j{direction}-{step}.jsonl")
            with RollingFrame(bus, journal, joint=JOINT, motor=MOTOR) as frame:
                target = frame.goal(direction, step)
                assert TICK_MIN <= target <= TICK_MAX, (direction, step)
                bus.write_goal_position(MOTOR, target)  # would raise if it were not


def test_the_frame_refuses_a_step_no_frame_could_EVER_promise(tmp_path):
    bus = open_bus()
    with RollingFrame(bus, make_journal(tmp_path), joint=JOINT, motor=MOTOR) as frame:
        with pytest.raises(CliError) as excinfo:
            frame.goal(+1, MAX_HEADROOM + 1)
        assert excinfo.value.code == EXIT_USER_ERROR
        with pytest.raises(CliError):
            frame.goal(+1, 0)
        with pytest.raises(CliError):
            frame.goal(0, 25)


def test_ensure_headroom_reports_whether_it_had_to_roll(tmp_path):
    bus = open_bus(raw=2048)
    with RollingFrame(bus, make_journal(tmp_path), joint=JOINT, motor=MOTOR) as frame:
        assert frame.ensure_headroom(+1, 100) is False  # freshly centred: nothing to do
        assert frame.ensure_headroom(+1, MAX_HEADROOM) is False  # exactly what it promises
        assert frame.recentres == 0
        assert not bus.offset_writes[1:]  # the opening shift, and no more


# ---------------------------------------------------------------------------
# Closing the transaction
# ---------------------------------------------------------------------------


def ends_in(journal: CalibrationJournal) -> list:
    """``(motor, disposition)`` for every closed transaction still on disk."""
    return [
        (record["motor"], record["disposition"])
        for record in journal.records()
        if record["event"] == "end"
    ]


def test_restore_puts_the_original_offset_back_and_closes_the_journal(tmp_path):
    bus = open_bus(raw=2048)
    journal = make_journal(tmp_path)
    # A second joint's transaction, left open. Two things fall out of it: the journal
    # is not truncated when this frame closes (a journal with nothing in flight clears
    # itself, by design), so the disposition is still readable — and closing THIS frame
    # is shown to close only THIS frame's transaction.
    journal.begin(joint="wrist_roll", motor=MOTOR + 1, original_offset=7)

    with RollingFrame(bus, journal, joint=JOINT, motor=MOTOR) as frame:
        creep(frame, bus, direction=+1, ticks=2500)

    assert bus.read_offset(MOTOR) == FACTORY_ENCODER_OFFSET
    assert journal.dirty_entry_for(MOTOR) is None
    assert journal.dirty_entry_for(MOTOR + 1) is not None  # the other joint, untouched
    assert ends_in(journal) == [(MOTOR, DISPOSITION_RESTORED)]
    # ...and the standing goal names where the joint is IN THE RESTORED FRAME.
    assert bus.reported_goal(MOTOR) == bus.read_position(MOTOR)


def test_restore_clears_a_latched_overload_first(tmp_path):
    """A probe that found a wall is a probe whose servo may be latched. Put it back anyway."""
    bus = open_bus(raw=2048)
    journal = make_journal(tmp_path)

    frame = RollingFrame(bus, journal, joint=JOINT, motor=MOTOR)
    frame.open()
    bus.fail_with_overload_on_op(1)  # every op raises from here until the latch is cleared
    with pytest.raises(OverloadError):
        bus.read_position(MOTOR)

    frame.restore()

    assert bus.read_offset(MOTOR) == FACTORY_ENCODER_OFFSET
    assert not journal.is_dirty()


class _OffsetSticks(RollingServoBus):
    """A servo that ACCEPTS the restore and goes on holding the temporary offset anyway."""

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.sticky = False

    def write_offset(self, motor: int, offset: int) -> None:
        if self.sticky:
            return  # accepted... and quietly ignored
        super().write_offset(motor, offset)


def test_an_UNVERIFIED_restore_keeps_the_journal_dirty(tmp_path):
    """A write the bus accepted but the servo is not holding is not a restore.

    The entry stays open on purpose: its ``original_offset`` is still the only record
    of the truth, and closing it here would destroy that record while the joint is
    still mis-calibrated.
    """
    bus = _OffsetSticks(positions={MOTOR: 2048}, offsets={MOTOR: FACTORY_ENCODER_OFFSET})
    bus.open()
    journal = make_journal(tmp_path)

    frame = RollingFrame(bus, journal, joint=JOINT, motor=MOTOR)
    frame.open()
    bus.sticky = True

    with pytest.raises(CliError) as excinfo:
        frame.restore()

    assert excinfo.value.code == EXIT_ENV_ERROR
    assert str(FACTORY_ENCODER_OFFSET) in excinfo.value.message
    assert journal.dirty_entry_for(MOTOR) is not None


def test_an_exception_inside_the_frame_still_restores_it_and_is_NOT_masked(tmp_path):
    bus = open_bus(raw=2048)
    journal = make_journal(tmp_path)

    class Boom(RuntimeError):
        pass

    with pytest.raises(Boom):
        with RollingFrame(bus, journal, joint=JOINT, motor=MOTOR) as frame:
            frame.recentre()
            raise Boom("the probe fell over")

    assert bus.read_offset(MOTOR) == FACTORY_ENCODER_OFFSET
    assert not journal.is_dirty()


def test_a_failing_restore_never_masks_the_exception_that_caused_it(tmp_path):
    bus = _OffsetSticks(positions={MOTOR: 2048}, offsets={MOTOR: FACTORY_ENCODER_OFFSET})
    bus.open()
    journal = make_journal(tmp_path)

    class Boom(RuntimeError):
        pass

    with pytest.raises(Boom):  # NOT the CliError the failed restore raises
        with RollingFrame(bus, journal, joint=JOINT, motor=MOTOR):
            bus.sticky = True
            raise Boom("the probe fell over")

    # ...and the journal is still dirty, so the next run's require_clean retries.
    assert journal.dirty_entry_for(MOTOR) is not None


def test_commit_KEEPS_the_rolled_calibration(tmp_path):
    """The deliberate other ending: this frame IS the new truth, so don't undo it."""
    bus = open_bus(raw=2048)
    journal = make_journal(tmp_path)
    journal.begin(joint="wrist_roll", motor=MOTOR + 1, original_offset=7)  # keeps the file alive

    with RollingFrame(bus, journal, joint=JOINT, motor=MOTOR) as frame:
        creep(frame, bus, direction=+1, ticks=1800)
        frame.recentre()
        kept = frame.offset
        frame.commit()

    assert kept != FACTORY_ENCODER_OFFSET
    assert bus.read_offset(MOTOR) == kept  # the rolled calibration is still in the servo
    assert journal.dirty_entry_for(MOTOR) is None
    assert ends_in(journal) == [(MOTOR, DISPOSITION_COMMITTED)]


def test_a_frame_that_never_opened_refuses_to_do_anything(tmp_path):
    bus = open_bus()
    frame = RollingFrame(bus, make_journal(tmp_path), joint=JOINT, motor=MOTOR)
    for call in (lambda: frame.sync(), lambda: frame.goal(+1, 25), lambda: frame.recentre()):
        with pytest.raises(CliError) as excinfo:
            call()
        assert excinfo.value.code == EXIT_USER_ERROR
    assert not bus.offset_writes


def test_restoring_twice_is_a_no_op_not_a_second_transaction(tmp_path):
    bus = open_bus()
    journal = make_journal(tmp_path)
    frame = RollingFrame(bus, journal, joint=JOINT, motor=MOTOR)
    frame.open()
    frame.restore()
    frame.restore()

    assert not journal.is_dirty()
    assert len(bus.offset_writes) == 2  # the shift, and the one restore


def test_the_frame_reports_the_state_t6_has_to_build_an_observation_from(tmp_path):
    """The public surface, exercised as a consumer will read it."""
    bus = open_bus(raw=1500)
    frame = RollingFrame(bus, make_journal(tmp_path), joint=JOINT, motor=MOTOR, margin=64)

    assert frame.joint == JOINT
    assert frame.motor == MOTOR
    assert frame.margin == 64
    assert frame.closed is False

    with frame:
        assert frame.original_offset == FACTORY_ENCODER_OFFSET
        assert frame.offset != FACTORY_ENCODER_OFFSET
        assert frame.full_turn is False
        assert frame.travelled == 0
        creep(frame, bus, direction=+1, ticks=300)
        assert frame.travelled == abs(frame.displacement) > 0

    assert frame.closed is True


def test_a_frame_refuses_a_joint_or_a_margin_it_cannot_honour(tmp_path):
    journal = make_journal(tmp_path)
    with pytest.raises(ValueError):
        RollingFrame(open_bus(), journal, joint="", motor=MOTOR)
    with pytest.raises(ValueError):
        RollingFrame(open_bus(), journal, joint=JOINT, motor=MOTOR, margin=MAX_HEADROOM + 1)
    with pytest.raises(ValueError):
        RollingFrame(open_bus(), journal, joint=JOINT, motor=MOTOR, margin=-1)


def test_a_frame_is_opened_once_and_used_until_it_is_closed(tmp_path):
    bus = open_bus()
    frame = RollingFrame(bus, make_journal(tmp_path), joint=JOINT, motor=MOTOR)
    frame.open()

    with pytest.raises(CliError) as excinfo:
        frame.open()  # a second shift would journal a TEMPORARY offset as the original
    assert excinfo.value.code == EXIT_USER_ERROR

    frame.restore()

    with pytest.raises(CliError) as excinfo:  # ...and a closed frame is not a frame
        frame.sync()
    assert excinfo.value.code == EXIT_USER_ERROR
    assert len(bus.offset_writes) == 2  # the shift and the restore, and nothing else


def test_ensure_headroom_refuses_to_promise_what_no_frame_could_deliver(tmp_path):
    bus = open_bus()
    with RollingFrame(bus, make_journal(tmp_path), joint=JOINT, motor=MOTOR) as frame:
        with pytest.raises(CliError) as excinfo:
            frame.ensure_headroom(+1, MAX_HEADROOM + 1)
        assert excinfo.value.code == EXIT_USER_ERROR
        assert frame.recentres == 0  # it did not spend an EEPROM write finding out


def test_a_joint_that_MOVES_while_it_is_limp_is_reported_not_papered_over(tmp_path):
    """The frame can roll the seam. It cannot outrun a joint that is falling.

    A re-centre de-energises the joint (``write_offset`` must). If gravity drags it
    across most of the frame while it is limp, the fresh frame is not fresh by the
    time it is measured — and that is a fact about the arm, so it is raised, not
    silently re-tried until the EEPROM wears out.
    """

    class _SagsWhileLimp(RollingServoBus):
        def write_offset(self, motor: int, offset: int) -> None:
            super().write_offset(motor, offset)
            # De-energised, and the joint falls most of a turn.
            self._positions[motor] = (self.true_raw(motor) + 2000) % ENCODER_TICKS

    bus = _SagsWhileLimp(positions={MOTOR: 2048}, offsets={MOTOR: FACTORY_ENCODER_OFFSET})
    bus.open()

    with RollingFrame(bus, make_journal(tmp_path), joint=JOINT, motor=MOTOR) as frame:
        with pytest.raises(CliError) as excinfo:
            frame.goal(+1, 300)

    assert excinfo.value.code == EXIT_ENV_ERROR
    assert "moved" in excinfo.value.message


def test_a_failed_settle_rolls_the_shift_back_WITHIN_the_same_process(tmp_path):
    """``open`` is not allowed to half-happen and then walk away.

    A failure after the EEPROM write leaves the servo in the temporary frame. The
    journal would recover it — on the NEXT run. Inside this one the frame puts it
    back itself, so a caught error does not leave the rest of the run reading ticks
    in a frame nobody chose.
    """

    class _GoalWriteFails(RollingServoBus):
        def write_goal_position(self, motor: int, position: int) -> None:
            raise CliError(code=EXIT_ENV_ERROR, message="wire fault", remediation="check the cable")

    bus = _GoalWriteFails(positions={MOTOR: 900}, offsets={MOTOR: FACTORY_ENCODER_OFFSET})
    bus.open()
    journal = make_journal(tmp_path)

    with pytest.raises(CliError):
        RollingFrame(bus, journal, joint=JOINT, motor=MOTOR).open()

    assert bus.read_offset(MOTOR) == FACTORY_ENCODER_OFFSET
    assert not journal.is_dirty()
