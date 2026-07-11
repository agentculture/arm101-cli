"""Regression guard: t4's measure-the-arm rewrite must not have disturbed the
overload-safety proven on hardware at an earlier t7 (see
``arm101/hardware/gentle.py``'s module docstring, claim c6).

``gentle_move`` layers two overload-safety measures around the whole move,
independent of its own ``present_load``-threshold contact check:

1. it caps the servo's own RAM ``Torque_Limit`` to ``_CONTACT_TORQUE_LIMIT``
   (500) for the duration of the move, restoring the pre-move value in a
   ``finally`` — so the servo's OWN hardware overload protection trips at a
   lower load than its factory rating, no matter how the move ends; and
2. it catches a mid-move ``OverloadError`` (the STS3215's own error=32 latch)
   and returns ``overloaded=True`` instead of letting the exception propagate,
   calling ``bus.clear_overload(motor)`` to recover.

Both measures predate the t4 rewrite (gentle_move now MEASURES arrival/stall
via polling present_position/present_load, rather than trusting the goal
tether) and are an explicit spec boundary that rewrite must not touch. This
file re-proves them against :class:`tests._fakes.ServoModelBus` — the fake
that actually models servo travel latency and load saturation — for all
three ways a move can end: clean arrival, a genuine stall-detected contact,
and a mid-move overload latch. ``tests/test_gentle_overload.py`` already
covers the overload/torque-cap contract against the plain (pre-travel-model)
``FakeBus``; this file is the post-t4 regression pin, exercised against the
fake that can actually produce a real CONTACT ending.

Also guarded here (a real hardware constraint discovered this session, per
``tests/_fakes.py``'s module docstring): ``present_load`` saturates at the
active ``Torque_Limit``. Since ``gentle_move`` caps that limit to 500 for the
duration of a move, a contact threshold >= 500 can never fire — the load
simply cannot climb high enough to exceed it. ``tests/test_gentle.py`` already
has a STATIC guard that every ``DEFAULT_CONTACT_THRESHOLDS`` entry sits below
the cap; the behavioural version of that guard lives at the bottom of this
file, made hang-proof by an explicit small ``timeout=`` plus reliance on the
``_MAX_POLLS_PER_MOVE`` backstop (see that test's docstring for the numbers).
"""

from __future__ import annotations

from arm101.hardware.bus import FakeBus
from arm101.hardware.gentle import _CONTACT_TORQUE_LIMIT, _DEFAULT_LOAD_THRESHOLD, gentle_move
from tests._fakes import ServoModelBus

# ---------------------------------------------------------------------------
# Geometry — mirrors tests/test_gentle.py's START/TARGET/OBSTACLE/GIVE (kept
# local rather than imported across test modules, matching the pattern
# already used in tests/test_demo_overload.py and tests/test_gentle_overload.py).
# ---------------------------------------------------------------------------

START = 2048
TARGET = 2448
OBSTACLE = START + 200
STIFFNESS = 20
#: Ticks the servo can creep past the obstacle's contact point before its
#: load saturates at the _CONTACT_TORQUE_LIMIT cap and it can push no further.
GIVE = _CONTACT_TORQUE_LIMIT // STIFFNESS


class _CapTrackingServoModelBus(ServoModelBus):
    """ServoModelBus that snapshots the live Torque_Limit at each goal write.

    Proves the ``_CONTACT_TORQUE_LIMIT`` cap is actually in force at every
    goal-position write made DURING a move — not merely bookkept before and
    after it — the same thing ``_CapTrackingBus`` in
    ``tests/test_gentle_overload.py`` proves against the plain (non-travel-
    modelling) ``FakeBus``.
    """

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.torque_limit_at_writes: list[int] = []

    def write_goal_position(self, motor: int, position: int) -> None:
        self.torque_limit_at_writes.append(self.active_torque_limit(motor))
        super().write_goal_position(motor, position)


def _cap_tracking_bus(position: int = START) -> _CapTrackingServoModelBus:
    bus = _CapTrackingServoModelBus(positions={1: position}, obstacle_stiffness=STIFFNESS)
    bus.open()
    return bus


# ---------------------------------------------------------------------------
# 1. Torque_Limit cap held DURING and restored AFTER — all three endings.
# ---------------------------------------------------------------------------


def test_torque_limit_capped_during_move_and_restored_after_clean_arrival():
    bus = _cap_tracking_bus()
    original = bus.read_torque_limit(1)
    assert original == 1000  # FakeBus factory default, sanity check

    result = gentle_move(
        bus, motor=1, target=TARGET, min_angle=0, max_angle=4095, allow_motion=True
    )

    assert result["contacted"] is False
    assert result["overloaded"] is False

    # The cap was live at EVERY goal-position write during the move.
    assert bus.torque_limit_at_writes
    assert all(v == _CONTACT_TORQUE_LIMIT for v in bus.torque_limit_at_writes)

    # ...and restored to the pre-move value once the move finished cleanly.
    assert bus.torque_limit_writes[0] == {"motor": 1, "value": _CONTACT_TORQUE_LIMIT}
    assert bus.torque_limit_writes[-1] == {"motor": 1, "value": original}
    assert bus.read_torque_limit(1) == original


def test_torque_limit_capped_during_move_and_restored_after_contact():
    bus = _cap_tracking_bus()
    bus.place_obstacle(1, OBSTACLE)
    original = bus.read_torque_limit(1)

    result = gentle_move(
        bus, motor=1, target=TARGET, min_angle=0, max_angle=4095, allow_motion=True
    )

    # This must be a REAL stall-detected contact (not the overload path), or
    # the test would be vacuous.
    assert result["contacted"] is True
    assert result["overloaded"] is False
    assert OBSTACLE <= result["contact_position"] <= OBSTACLE + GIVE

    # The cap was live at every write — the approach, AND the backoff retreat
    # write that follows a detected contact.
    assert bus.torque_limit_at_writes
    assert all(v == _CONTACT_TORQUE_LIMIT for v in bus.torque_limit_at_writes)

    # Restored once the joint backed off and held — a contact ending must not
    # leave the servo's own overload protection permanently lowered.
    assert bus.torque_limit_writes[0] == {"motor": 1, "value": _CONTACT_TORQUE_LIMIT}
    assert bus.torque_limit_writes[-1] == {"motor": 1, "value": original}
    assert bus.read_torque_limit(1) == original


def test_torque_limit_capped_during_move_and_restored_after_overload():
    bus = _cap_tracking_bus().fail_with_overload_on_op(11)
    original = bus.read_torque_limit(1)

    # Op sequence against ServoModelBus (verified against the implementation):
    # ops 1-6 are gentle_move's setup (read_torque_limit, write_torque_limit
    # cap, write_acceleration, write_goal_speed, enable_torque, read_info for
    # start_position); each loop iteration from there is a write_goal_position
    # then a read_info. Op 11 lands on the THIRD loop write, so two prior
    # writes succeed under the cap before the overload strikes on the third —
    # proving the cap held across genuine simulated travel, not just a single
    # instant.
    result = gentle_move(
        bus, motor=1, target=TARGET, min_angle=0, max_angle=4095, allow_motion=True
    )

    assert result["overloaded"] is True
    assert result["contacted"] is False
    assert len(bus.torque_limit_at_writes) >= 2
    assert all(v == _CONTACT_TORQUE_LIMIT for v in bus.torque_limit_at_writes)

    # Restored in the finally even though the move ended in an overload, not
    # a clean stop — this is the crux of the c6 safety boundary.
    assert bus.torque_limit_writes[0] == {"motor": 1, "value": _CONTACT_TORQUE_LIMIT}
    assert bus.torque_limit_writes[-1] == {"motor": 1, "value": original}
    assert bus.read_torque_limit(1) == original


def test_torque_limit_restore_uses_the_actual_pre_move_value_not_a_constant():
    """The restore must read back what THIS motor had, never a hardcoded 1000."""
    bus = ServoModelBus(positions={1: START}, obstacle_stiffness=STIFFNESS, torque_limits={1: 700})
    bus.open()

    gentle_move(bus, motor=1, target=TARGET, min_angle=0, max_angle=4095, allow_motion=True)

    assert bus.torque_limit_writes[0] == {"motor": 1, "value": _CONTACT_TORQUE_LIMIT}
    assert bus.torque_limit_writes[-1] == {"motor": 1, "value": 700}
    assert bus.read_torque_limit(1) == 700


# ---------------------------------------------------------------------------
# 2. Mid-move OverloadError: caught, recovered via clear_overload, reported
#    via the result dict — never raised.
# ---------------------------------------------------------------------------


def test_mid_move_overload_against_travel_modelled_bus_is_caught_not_raised():
    """The whole point: an overload mid-travel is recovered from, never propagated.

    Uses ServoModelBus (not the plain FakeBus tests/test_gentle_overload.py
    already covers) so the overload path is exercised against the SAME fake
    that drives the new stall-detection state machine (moving/stalled/onset
    counters) t4 introduced — proving those additions did not sneak the
    exception past the except clause.
    """
    bus = ServoModelBus(
        positions={1: START}, obstacle_stiffness=STIFFNESS
    ).fail_with_overload_on_op(9)
    bus.open()

    # No pytest.raises here on purpose — a raise would fail this test.
    result = gentle_move(
        bus, motor=1, target=TARGET, min_angle=0, max_angle=4095, allow_motion=True
    )

    assert isinstance(result, dict)
    assert result["overloaded"] is True
    assert result["contacted"] is False
    assert result["motor"] == 1


def test_mid_move_overload_calls_clear_overload_and_disarms_the_seam():
    bus = ServoModelBus(
        positions={1: START}, obstacle_stiffness=STIFFNESS
    ).fail_with_overload_on_op(9)
    bus.open()

    gentle_move(bus, motor=1, target=TARGET, min_angle=0, max_angle=4095, allow_motion=True)

    # clear_overload() records a torque-off entry and disarms the seam — proof
    # the recovery action actually ran, not merely that the exception vanished.
    assert bus.torque_writes[-1] == {"motor": 1, "on": False}
    assert bus.overload_after_ops is None


def test_mid_move_overload_final_position_is_traceable_to_a_real_poll():
    """Even on the overload path, whatever position is reported must be one the
    bus actually measured — never a value invented after the exception."""
    bus = ServoModelBus(
        positions={1: START}, obstacle_stiffness=STIFFNESS
    ).fail_with_overload_on_op(9)
    bus.open()

    result = gentle_move(
        bus, motor=1, target=TARGET, min_angle=0, max_angle=4095, allow_motion=True
    )

    assert result["overloaded"] is True
    assert result["final_position"] in bus.polled_positions()


def test_mid_move_overload_on_the_very_first_setup_write_still_restores_nothing_bogus():
    """An overload during the torque-limit CAP write itself (before any motion
    setup has completed) must not attempt to restore a value that was never
    successfully read — this is the ``original_torque_limit is None`` guard in
    gentle_move's finally block."""
    bus = FakeBus(positions={1: START}).fail_with_overload_on_op(1)
    bus.open()

    result = gentle_move(
        bus, motor=1, target=TARGET, min_angle=0, max_angle=4095, allow_motion=True
    )

    assert result["overloaded"] is True
    # The read_torque_limit call itself was the one that overloaded, so no
    # cap write and no restore write ever happened.
    assert bus.torque_limit_writes == []


# ---------------------------------------------------------------------------
# 3. present_load saturates at the active Torque_Limit: a contact threshold
#    at or above _CONTACT_TORQUE_LIMIT can never fire.
#
# tests/test_gentle.py::test_every_default_contact_threshold_sits_below_the_torque_cap
# already pins this STATICALLY (every DEFAULT_CONTACT_THRESHOLDS entry must
# sit below the cap). The test below is the BEHAVIOURAL version: it drives an
# actual gentle_move at threshold == _CONTACT_TORQUE_LIMIT into a real
# obstacle and proves the load-based check can never trip, so the joint is
# neither reported as "contacted" nor able to reach its target — it can only
# terminate via the timeout / _MAX_POLLS_PER_MOVE backstop.
#
# Hang-safety: FakeBus (and ServoModelBus, a subclass) is paced with
# poll_interval=0 regardless of what is passed in (see gentle.py's
# _needs_pacing — a simulated bus never sleeps between polls), so every
# iteration of the step loop costs microseconds of real CPU time, not wall-
# clock time. The loop is bounded on TWO independent axes: an explicit small
# `timeout=` here (0.05s of *wall clock*), and gentle_move's own
# `_MAX_POLLS_PER_MOVE` (4000) hard backstop, which does not depend on the
# clock at all. Measured directly against this exact scenario: the backstop
# trips first, after 4000 polls in single-digit milliseconds of real time —
# nowhere near either bound, so this cannot hang CI even on a slow runner or
# if a future change altered one of the two limits.
# ---------------------------------------------------------------------------


def test_threshold_at_the_torque_cap_can_never_detect_a_real_obstacle():
    bus = ServoModelBus(positions={1: START}, obstacle_stiffness=STIFFNESS)
    bus.place_obstacle(1, OBSTACLE)
    bus.open()

    result = gentle_move(
        bus,
        motor=1,
        target=TARGET,
        min_angle=0,
        max_angle=4095,
        threshold=_CONTACT_TORQUE_LIMIT,  # 500 -- present_load can never exceed this
        allow_motion=True,
        timeout=0.05,  # belt: wall-clock bound, in case the poll backstop ever changes
    )

    # The load genuinely reached the cap (the obstacle is real and was hit)...
    assert max(bus.polled_loads()) == _CONTACT_TORQUE_LIMIT
    # ...but "load > threshold" is never true when threshold == the load's own
    # ceiling, so the stall was never classified as a contact...
    assert result["contacted"] is False
    assert result["overloaded"] is False
    # ...and the joint is physically stuck against the obstacle, so it also
    # never reached the target -- neither ending condition the loop knows
    # about was ever satisfied, and only the hang-proof backstop ended it.
    assert result["final_position"] < result["clamped_target"]
    assert bus.true_position(1) == OBSTACLE + GIVE

    # This is exactly why every entry in DEFAULT_CONTACT_THRESHOLDS must sit
    # strictly below _CONTACT_TORQUE_LIMIT (the static half of this guard, in
    # tests/test_gentle.py) -- a threshold placed AT the cap silently loses
    # all contact detection, as reproduced above.
    assert _DEFAULT_LOAD_THRESHOLD < _CONTACT_TORQUE_LIMIT
