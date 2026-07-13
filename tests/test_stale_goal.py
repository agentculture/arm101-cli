"""Issue #47 — a stale ``Goal_Position`` is a LURCH waiting for the next torque-on.

The hazard, in one sentence
==========================
``Goal_Position`` (addr 42) is a **REPORTED** tick. Write a joint's encoder offset
and that same stored number names a **different physical angle** — without anything
having written to addr 42. The servo does not notice while it is limp. It notices the
instant torque comes back, and it bolts.

On ``elbow_flex`` the offset delta is ~988 ticks. On ``shoulder_lift`` — which carries
the whole arm — a lurch of that size is not a diagnostic curiosity.

Why it is a RACE and not a certainty (and why that makes it worse, not better)
-----------------------------------------------------------------------------
``gentle_move``/``compliant_move`` enable torque and then write their first goal. The
servo's motion-onset latency is ~95-127 ms, so in practice the correct goal usually
lands before the shaft has done anything. Nothing in the code bounds that window: it
is a race that is *usually* won, which is the worst kind of bug to leave in a path
that is about to re-zero five more joints.

The fix, and where it lives
===========================
**Every offset write re-points the standing goal at the joint's own position, while
torque is still off.** Then enabling torque is a no-op instead of a command.
:func:`arm101.hardware.safety.hold_in_place` is that operation, and this module pins
it at EVERY door in the package through which an offset reaches a servo:

* :func:`arm101.hardware.rezero.apply_rezero` — the deliberate, permanent re-zero;
* :func:`arm101.hardware.rezero.commit_rezero` — the journalled re-zero ``arm limits
  --commit`` performs;
* :meth:`arm101.hardware.rolling_frame.RollingFrame.open` / ``recentre`` / ``restore``
  — the temporary frame (already did this; pinned here so it stays true);
* :func:`arm101.hardware.journal.restore_dirty` — **the crash-recovery path**, which
  runs at the start of EVERY motion verb via ``require_clean``. It writes the original
  offset back and therefore moves the frame too. This one was missed.

The bus MUST be a :class:`~tests._rolling_servo.RollingServoBus`
===============================================================
``tests/_fakes.py::ServoModelBus`` **cannot see this bug.** It converts a goal into a
raw count at *write* time, so a later offset change moves the goal and the shaft
together and the hazard cancels itself out. ``RollingServoBus`` models the servo the
way the servo is — the error is a plain subtraction in the REPORTED frame — so a stale
goal really does name a different angle, and really does make the shaft run for it.

``test_a_stale_goal_really_WOULD_lurch`` is the mutation check that keeps this whole
file honest: it puts the stale goal back by hand and watches the joint bolt. A test
suite that could not produce the failure cannot vouch for its absence.
"""

from __future__ import annotations

import pytest

from arm101.hardware import rezero
from arm101.hardware.arm_spec import FACTORY_ENCODER_OFFSET, REZERO_ARCS
from arm101.hardware.journal import CalibrationJournal, require_clean, restore_dirty, shift_offset
from arm101.hardware.rolling_frame import RollingFrame
from arm101.hardware.safety import hold_in_place
from arm101.hardware.ticks import ENCODER_TICKS
from tests._rolling_servo import RollingServoBus

JOINT = "elbow_flex"
MOTOR = 3

#: The offset a fresh re-zero of ``elbow_flex`` writes — derived from the shipped arc,
#: never typed. The delta from the factory offset is the size of the lurch this file
#: exists to prevent (~1000 ticks on the real arm).
TARGET_OFFSET = REZERO_ARCS[JOINT].offset


def _bus(raw: int = 1000, offset: int = FACTORY_ENCODER_OFFSET) -> RollingServoBus:
    bus = RollingServoBus(positions={MOTOR: raw}, offsets={MOTOR: offset}, ids=[MOTOR])
    bus.open()
    return bus


def _journal(tmp_path) -> CalibrationJournal:
    return CalibrationJournal(tmp_path / "journal.jsonl")


#: The **floor** on how far the shaft runs if a re-zero leaves the standing goal stale:
#: the offset delta, i.e. the distance the reported frame slid under the stored number.
#: Derived from the arc, not typed — ~1072 ticks on ``elbow_flex``.
#:
#: A floor rather than an equality, because the servo can do **worse**: its error is a
#: plain subtraction in the reported frame and it cannot drive across its own reported
#: seam, so a stale goal on the far side of that seam sends it the LONG way round. That
#: is exactly the "rotates ``elbow_flex`` the long way round, through its whole travel,
#: into a wall" hazard :mod:`arm101.hardware.rezero`'s docstring forbids commanding on
#: purpose — reached here by commanding nothing at all.
LURCH_TICKS = abs(TARGET_OFFSET - FACTORY_ENCODER_OFFSET)


def _settle(bus: RollingServoBus, motor: int = MOTOR) -> int:
    """Energise the joint and let the servo chase whatever goal it is holding, to a stop.

    The moment of truth: this is what the *next mover* does. If the standing goal is
    stale, the shaft runs for it here — and it is given enough polls to run a WHOLE TURN,
    so the test measures the lurch rather than how long we happened to watch for.
    """
    bus.enable_torque(motor, True)
    for _ in range(ENCODER_TICKS // bus.ticks_per_poll + 8):
        bus.read_info(motor)
    return bus.true_raw(motor)


# ---------------------------------------------------------------------------
# The hazard is real — the mutation check
# ---------------------------------------------------------------------------


def test_a_stale_goal_really_WOULD_lurch() -> None:
    """The bug, reproduced. Without it, nothing below proves anything.

    The joint sits at raw 1000 holding a goal that means "stay here" in the factory
    frame. Re-zero it — nothing writes addr 42 — and the *same* number now names an
    angle ~1000 ticks away. Energise, and the shaft goes there.
    """
    bus = _bus(raw=1000)
    stale = bus.read_position(MOTOR)  # "hold station", in the OLD frame
    bus.write_goal_position(MOTOR, stale)

    bus.write_offset(MOTOR, TARGET_OFFSET)  # the raw primitive: no hold-in-place

    assert bus.read_offset(MOTOR) == TARGET_OFFSET  # the write landed...
    assert bus.true_raw(MOTOR) == 1000  # ...and the shaft has not moved. Yet.

    _settle(bus)

    assert bus.true_raw(MOTOR) != 1000, "the fake cannot see the bug it is here to catch"
    assert abs(bus.net_travel(MOTOR)) >= LURCH_TICKS, (
        "the shaft should run at LEAST the offset delta — the amount by which the frame "
        "moved under the stored goal. If it does not, this fake is not modelling the servo"
    )


# ---------------------------------------------------------------------------
# The primitive
# ---------------------------------------------------------------------------


def test_hold_in_place_points_the_standing_goal_at_the_joints_own_position() -> None:
    bus = _bus(raw=1000)
    bus.write_offset(MOTOR, TARGET_OFFSET)

    hold_in_place(bus, MOTOR)

    assert bus.reported_goal(MOTOR) == bus.read_position(MOTOR)
    assert _settle(bus) == 1000  # nothing to chase


def test_hold_in_place_energises_nothing() -> None:
    """It is called on a LIMP joint, and it must leave it limp."""
    bus = _bus(raw=1000)
    hold_in_place(bus, MOTOR)
    assert bus.torque_on(MOTOR) is False


# ---------------------------------------------------------------------------
# Every door an offset reaches a servo through
# ---------------------------------------------------------------------------


def test_apply_rezero_leaves_no_stale_goal_behind() -> None:
    """The deliberate re-zero (``arm rezero <joint> --apply``). Issue #47's headline case."""
    bus = _bus(raw=1000)
    bus.write_goal_position(MOTOR, bus.read_position(MOTOR))  # a standing "hold station"

    read_back = rezero.apply_rezero(bus, MOTOR, TARGET_OFFSET)

    assert read_back == TARGET_OFFSET
    assert bus.reported_goal(MOTOR) == bus.read_position(MOTOR)
    assert _settle(bus) == 1000, "the joint LURCHED after a re-zero"


def test_apply_rezero_still_commands_no_motion_of_its_own() -> None:
    """The hold-in-place goal is not a motion command — and must not become one.

    ``rezero``'s whole module docstring turns on writing NO goal that could drive the
    joint anywhere. A goal that names the joint's *own* position is the one goal that
    cannot: it is the servo's definition of "stay". This asserts the distinction is
    real, not a technicality — the shaft does not move, torque stays off, and the only
    goal ever written is the one the joint is already at.
    """
    bus = _bus(raw=1000)
    rezero.apply_rezero(bus, MOTOR, TARGET_OFFSET)

    assert bus.torque_on(MOTOR) is False
    assert bus.true_raw(MOTOR) == 1000
    assert bus.net_travel(MOTOR) == 0
    assert bus.reported_goal(MOTOR) == bus.read_position(MOTOR)


def test_commit_rezero_leaves_no_stale_goal_behind(tmp_path) -> None:
    """The journalled re-zero ``arm limits --commit`` performs."""
    bus = _bus(raw=1000)
    bus.write_goal_position(MOTOR, bus.read_position(MOTOR))
    journal = _journal(tmp_path)

    read_back = rezero.commit_rezero(bus, journal, joint=JOINT, motor=MOTOR, offset=TARGET_OFFSET)

    assert read_back == TARGET_OFFSET
    assert bus.reported_goal(MOTOR) == bus.read_position(MOTOR)
    assert _settle(bus) == 1000


def test_the_crash_recovery_path_leaves_no_stale_goal_behind_either(tmp_path) -> None:
    """``require_clean`` runs at the start of EVERY motion verb — and it writes an offset.

    A crashed run left a temporary offset in EEPROM. The next run restores the original
    — which moves the frame back, and makes the goal the crashed run left behind stale
    all over again, in the *other* direction. This is the door that was missed: the
    restore is the one offset write that happens on the path of every verb, including
    the ones that never asked to touch a calibration at all.
    """
    journal = _journal(tmp_path)
    bus = _bus(raw=1000)

    # A crashed run: it shifted the offset, wrote a hold-in-place goal in THAT frame,
    # and died before restoring.
    shift_offset(bus, journal, joint=JOINT, motor=MOTOR, offset=TARGET_OFFSET)
    bus.write_goal_position(MOTOR, bus.read_position(MOTOR))
    assert journal.dirty_entries()

    report = restore_dirty(bus, journal)

    assert report.complete
    assert bus.read_offset(MOTOR) == FACTORY_ENCODER_OFFSET
    assert bus.reported_goal(MOTOR) == bus.read_position(MOTOR)
    assert _settle(bus) == 1000, "the recovery path re-pointed the offset but not the goal"


def test_require_clean_is_the_guard_the_verbs_actually_call(tmp_path) -> None:
    """Same hazard, through the entry point a verb uses."""
    journal = _journal(tmp_path)
    bus = _bus(raw=1000)
    shift_offset(bus, journal, joint=JOINT, motor=MOTOR, offset=TARGET_OFFSET)
    bus.write_goal_position(MOTOR, bus.read_position(MOTOR))

    require_clean(bus, journal)

    assert _settle(bus) == 1000


def test_the_rolling_frame_holds_at_every_offset_write(tmp_path) -> None:
    """open() and recentre() and restore(). Already true; pinned so it stays true."""
    bus = _bus(raw=1000)
    with RollingFrame(bus, _journal(tmp_path), joint=JOINT, motor=MOTOR) as frame:
        assert bus.reported_goal(MOTOR) == frame.reported
        frame.recentre()
        assert bus.reported_goal(MOTOR) == frame.reported
    assert bus.reported_goal(MOTOR) == bus.read_position(MOTOR)
    assert _settle(bus) == 1000


# ---------------------------------------------------------------------------
# The invariant, stated once, over every door at once
# ---------------------------------------------------------------------------

#: Every public operation in ``arm101`` that can put an offset into a servo. Each is a
#: closure that performs it against a freshly built bus; the invariant below is asserted
#: over all of them. **A new offset-writing path belongs in this list** — that is the
#: whole point of enumerating them, rather than testing three of them and hoping.
#:
#: ``bus.write_offset`` itself is deliberately NOT here: it is the register primitive,
#: one layer below policy, and it is what ``test_a_stale_goal_really_WOULD_lurch`` uses
#: to *produce* the bug. The rule is that nothing above it may leave the goal stale.
_OFFSET_DOORS = {
    "rezero.apply_rezero": lambda bus, journal: rezero.apply_rezero(bus, MOTOR, TARGET_OFFSET),
    "rezero.commit_rezero": lambda bus, journal: rezero.commit_rezero(
        bus, journal, joint=JOINT, motor=MOTOR, offset=TARGET_OFFSET
    ),
    "journal.shift_offset + restore_dirty": lambda bus, journal: (
        shift_offset(bus, journal, joint=JOINT, motor=MOTOR, offset=TARGET_OFFSET),
        restore_dirty(bus, journal),
    ),
    "rolling_frame.RollingFrame": lambda bus, journal: _roll(bus, journal),
}


def _roll(bus: RollingServoBus, journal: CalibrationJournal) -> None:
    with RollingFrame(bus, journal, joint=JOINT, motor=MOTOR) as frame:
        frame.recentre()


@pytest.mark.parametrize("door", sorted(_OFFSET_DOORS))
def test_no_door_into_a_servos_offset_register_leaves_the_goal_stale(door, tmp_path) -> None:
    """The invariant, over every path at once: an offset write ALWAYS re-points the goal.

    Each door is exercised on a joint that was holding station. Afterwards the servo is
    energised — the thing the next mover does — and the shaft must not move a tick.
    """
    bus = _bus(raw=1000)
    bus.write_goal_position(MOTOR, bus.read_position(MOTOR))

    _OFFSET_DOORS[door](bus, _journal(tmp_path / door.replace("/", "_")))

    assert bus.reported_goal(MOTOR) == bus.read_position(MOTOR), (
        f"{door} left a standing goal that does not name the joint's own position — "
        "the next mover to energise this joint will drive it there"
    )
    assert _settle(bus) == 1000, f"{door} left a stale goal: the joint LURCHED"
