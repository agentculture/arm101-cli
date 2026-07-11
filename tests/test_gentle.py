"""Tests for arm101.hardware.gentle — load-watch back-off-then-hold compliant move.

These tests drive ``gentle_move`` against :class:`tests._fakes.ServoModelBus`, a
fake bus that models a real STS3215's **travel latency**: a goal-write commands
the servo but does not move it, the shaft advances toward its goal over
successive ``read_info`` polls, and ``present_load`` exists only while the joint
is actually moving or actually pushing against something — saturating at the
motor's active ``Torque_Limit``.

That replaces the fake this file used to carry, whose ``write_goal_position``
teleported ``present_position`` to the goal and materialised load on the same
call. It made the suite structurally incapable of seeing ``gentle_move``'s two
real defects, both measured on the follower arm on 2026-07-12:

1. it samples ``present_load`` ~1 ms after commanding a step, before the servo
   has mechanically responded — so every load read it takes reflects the state
   BEFORE the move (0-68, while real travel load peaked at 272); and
2. it tracks the COMMANDED tick, not a read-back position, so it terminates when
   the goal-writes run out rather than when the arm arrives — a 400-tick
   wrist_roll move returned in 71 ms claiming ``final_position=3548`` while the
   joint was still sitting at 3148, with ~900 ms of real travel still to come.

The tests marked ``xfail(strict=True)`` below are the regression tests for
exactly that. They FAIL against the pre-fix ``gentle_move`` — that failure is
the point — and t4's rewrite (measure, don't assume) is what removes the
markers.
"""

from __future__ import annotations

import pytest

from arm101.cli._errors import EXIT_USER_ERROR, CliError
from arm101.hardware.arm_spec import DEFAULT_CONTACT_THRESHOLDS
from arm101.hardware.bus import load_magnitude
from arm101.hardware.gentle import (
    _CONTACT_TORQUE_LIMIT,
    _DEFAULT_BACKOFF_TICKS,
    _DEFAULT_LOAD_THRESHOLD,
    _DEFAULT_STEP_TICKS,
    gentle_move,
)
from tests._fakes import CONTACT, FRICTION_STALL, IDLE, TRAVELLING, ServoModelBus

# ---------------------------------------------------------------------------
# Geometry shared by the moving tests.
#
# START -> TARGET is 400 ticks, the same move the hardware timing probe ran on
# wrist_roll. At the fake's default 10 ticks per poll that is 40 polls of real
# travel; the pre-fix loop, which polls exactly once per commanded 25-tick step,
# spends only 16 — so it returns with the shaft at ~40% of the commanded travel,
# having watched none of the motion that matters. OBSTACLE sits 200 ticks in:
# genuinely mid-travel, and deliberately BEYOND the ~160 ticks the pre-fix loop
# ever lets the servo cover, so a contact there is one the pre-fix code cannot
# possibly see. That is the point of the fixture, not an accident of it.
# ---------------------------------------------------------------------------

START = 2048
TARGET = 2448
DISTANCE = TARGET - START
OBSTACLE = START + 200

#: Ticks the servo can creep past the obstacle's contact point before its load
#: saturates at gentle_move's Torque_Limit cap and it can push no further:
#: ``500 // 20``. The bench recording shows this creep-under-load is real (the
#: gripper kept moving 3022 -> 3001 while load climbed past its threshold), so
#: contact lands somewhere in [OBSTACLE, OBSTACLE + GIVE] depending on the
#: stall rule's tolerance — never at a single knife-edge tick.
STIFFNESS = 20
GIVE = _CONTACT_TORQUE_LIMIT // STIFFNESS

#: wrist_roll's measured FREE-motion load profile: an acceleration transient
#: peaking at 272 — above the 250 default contact threshold — that then settles.
#: The joint is advancing the whole time, so this is emphatically NOT contact.
FREE_MOTION_TRANSIENT = (272, 272, 200, 120, 60)

_PRE_FIX_REASON = (
    "pre-fix gentle_move samples load before motion and returns commanded ticks — fixed in t4"
)


def _bus(position: int = START, **kwargs) -> ServoModelBus:
    """An opened single-motor ServoModelBus with the shaft at *position*."""
    bus = ServoModelBus(positions={1: position}, obstacle_stiffness=STIFFNESS, **kwargs)
    bus.open()
    return bus


# ---------------------------------------------------------------------------
# The fake itself — it is the linchpin, so pin down its physics.
#
# If these ever go green while asserting the old behaviour, the fake has drifted
# back into teleporting and every test below it is worthless.
# ---------------------------------------------------------------------------


def test_goal_write_does_not_move_the_servo():
    """A goal-write COMMANDS; it does not move anything. Travel costs polls."""
    bus = _bus()

    bus.write_goal_position(1, TARGET)

    assert bus.true_position(1) == START  # no teleport
    assert bus.poll_count == 0

    # The shaft only advances when time passes — i.e. on a poll.
    assert bus.read_info(1)["present_position"] == START + 10
    assert bus.read_info(1)["present_position"] == START + 20
    assert bus.true_position(1) == START + 20


def test_load_is_zero_at_rest_and_rises_only_while_travelling():
    bus = _bus()

    at_rest = bus.read_info(1)  # no goal written yet
    assert at_rest["present_load"] == 0
    assert bus.poll_log[-1]["state"] == IDLE

    bus.write_goal_position(1, TARGET)
    moving = bus.read_info(1)
    assert moving["present_load"] == 60
    assert bus.poll_log[-1]["state"] == TRAVELLING


def test_reaching_an_obstacle_stalls_the_joint_with_a_high_sustained_load():
    """Contact = the shaft creeps in, load ramps, then it stops with load pinned."""
    bus = _bus()
    bus.place_obstacle(1, OBSTACLE)
    bus.write_torque_limit(1, _CONTACT_TORQUE_LIMIT)  # what gentle_move applies
    bus.write_goal_position(1, TARGET)

    for _ in range(60):
        bus.read_info(1)

    # It pushed INTO the obstacle (a compliant contact, as on the bench) and
    # stopped there — never reaching the commanded target.
    assert bus.true_position(1) == OBSTACLE + GIVE
    assert bus.true_position(1) < TARGET

    # ...and it is not advancing any more, while the load stays high.
    tail_positions = bus.polled_positions()[-5:]
    assert len(set(tail_positions)) == 1
    assert bus.polled_loads()[-5:] == [_CONTACT_TORQUE_LIMIT] * 5
    assert bus.poll_log[-1]["state"] == CONTACT


def test_present_load_saturates_at_the_active_torque_limit():
    """Proven twice on hardware: limit 300 -> load pins at 300; limit 600 -> 600.

    The Present_Load register is clamped by Torque_Limit, so a contact threshold
    at or above the active limit can never fire. gentle_move caps the limit at
    _CONTACT_TORQUE_LIMIT for the duration of a move, which puts a hard ceiling
    of 500 on any load it can ever observe.
    """
    for limit in (300, 500, 600):
        bus = _bus()
        bus.place_obstacle(1, OBSTACLE)
        bus.write_torque_limit(1, limit)
        bus.write_goal_position(1, TARGET)

        loads = [bus.read_info(1)["present_load"] for _ in range(80)]

        assert max(loads) == limit, f"load must saturate AT Torque_Limit={limit}"
        assert all(load <= limit for load in loads)


def test_every_default_contact_threshold_sits_below_the_torque_cap():
    """A threshold >= the active Torque_Limit is unfirable — load can never reach it.

    gentle_move pins Torque_Limit to _CONTACT_TORQUE_LIMIT for the whole move,
    so the usable band for every joint is (free-motion peak, _CONTACT_TORQUE_LIMIT).
    This guards t7's re-derivation of the table from putting a threshold above
    the ceiling, where it would silently never trigger.
    """
    assert DEFAULT_CONTACT_THRESHOLDS
    for joint, threshold in DEFAULT_CONTACT_THRESHOLDS.items():
        assert threshold < _CONTACT_TORQUE_LIMIT, (
            f"{joint}'s threshold {threshold} is at/above the {_CONTACT_TORQUE_LIMIT} "
            "Torque_Limit cap, so present_load can never exceed it"
        )


def test_under_torqued_joint_stalls_in_free_space_like_a_contact():
    """An under-torqued joint stalls in EMPTY AIR and looks exactly like contact.

    Bench, 2026-07-12: Torque_Limit=300 sits below the gripper's own gear
    friction (~320), so the gripper could not move at all — load pinned at the
    limit, position frozen. That is the same signature (load over threshold AND
    position not advancing) that the stall rule reads as contact.

    This test pins the SIGNATURE down; it deliberately does NOT assert what
    gentle_move should do about it, because that is a judgement call, not a
    fact — see the t2 report. The practical consequence for t4/t7: the
    Torque_Limit cap must stay ABOVE every joint's gear friction, or the
    primitive will report a contact against thin air.
    """
    bus = _bus(friction_load=320)
    bus.write_torque_limit(1, 300)  # below the friction floor
    bus.write_goal_position(1, TARGET)  # empty air — no obstacle placed

    samples = [bus.read_info(1) for _ in range(10)]

    assert [s["present_position"] for s in samples] == [START] * 10  # never budged
    assert [s["present_load"] for s in samples] == [300] * 10  # pinned at the limit
    assert bus.poll_log[-1]["state"] == FRICTION_STALL


def test_load_direction_bit_is_reported_on_a_negative_load():
    """The STS3215 puts load DIRECTION in bit 10; callers must mask it off."""
    bus = _bus(direction_bit=True)
    bus.write_goal_position(1, START - 200)

    raw = bus.read_info(1)["present_load"]

    assert raw >= 1024  # direction bit set
    assert load_magnitude(raw) == 60


# ---------------------------------------------------------------------------
# allow_motion gate
# ---------------------------------------------------------------------------


def test_gentle_move_without_allow_motion_raises_and_writes_nothing():
    bus = _bus()

    with pytest.raises(CliError) as exc:
        gentle_move(bus, motor=1, target=3000, min_angle=0, max_angle=4095)
    assert exc.value.code == EXIT_USER_ERROR

    assert bus.accel_writes == []
    assert bus.speed_writes == []
    assert bus.torque_writes == []
    assert bus.position_writes == []


def test_gentle_move_allow_motion_false_explicit_raises_and_writes_nothing():
    bus = _bus()

    with pytest.raises(CliError) as exc:
        gentle_move(bus, motor=1, target=3000, min_angle=0, max_angle=4095, allow_motion=False)
    assert exc.value.code == EXIT_USER_ERROR
    assert bus.position_writes == []


# ---------------------------------------------------------------------------
# step / backoff validation (guards against a non-progressing infinite loop)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("bad_step", [0, -1, -25])
def test_gentle_move_non_positive_step_raises_and_writes_nothing(bad_step):
    """A step <= 0 never advances toward the target: reject it up front rather
    than spin forever issuing bus writes."""
    bus = _bus()

    with pytest.raises(CliError) as exc:
        gentle_move(
            bus,
            motor=1,
            target=3000,
            min_angle=0,
            max_angle=4095,
            step=bad_step,
            allow_motion=True,
        )
    assert exc.value.code == EXIT_USER_ERROR
    assert "step" in exc.value.message
    # The guard fires before any hardware interaction.
    assert bus.accel_writes == []
    assert bus.speed_writes == []
    assert bus.torque_writes == []
    assert bus.position_writes == []


@pytest.mark.parametrize("bad_backoff", [-1, -50])
def test_gentle_move_negative_backoff_raises_and_writes_nothing(bad_backoff):
    bus = _bus()

    with pytest.raises(CliError) as exc:
        gentle_move(
            bus,
            motor=1,
            target=3000,
            min_angle=0,
            max_angle=4095,
            backoff=bad_backoff,
            allow_motion=True,
        )
    assert exc.value.code == EXIT_USER_ERROR
    assert "backoff" in exc.value.message
    assert bus.position_writes == []


# ---------------------------------------------------------------------------
# THE REGRESSION TESTS — measured arrival, not assumed arrival.
#
# Every test in this block fails against the pre-fix loop, which returns as soon
# as its goal-writes are exhausted and then reports the commanded target as if
# the joint were there. t4 makes them pass by measuring.
# ---------------------------------------------------------------------------


def test_gentle_move_final_position_is_the_joints_actual_position():
    """``final_position`` must be where the arm IS, not where it was told to go.

    The hardware finding in one assertion: the pre-fix loop returns
    ``final_position=TARGET`` while the shaft is still ~240 ticks short of it.
    """
    bus = _bus()

    result = gentle_move(
        bus, motor=1, target=TARGET, min_angle=0, max_angle=4095, allow_motion=True
    )

    assert result["contacted"] is False
    assert result["final_position"] == bus.true_position(1)
    assert bus.true_position(1) == TARGET  # the joint really did arrive


def test_gentle_move_final_position_is_traceable_to_a_bus_read():
    """Honesty condition: every reported position is a value read off the servo.

    An implementation that computes ``final_position`` from the commanded target
    cannot satisfy this — that number never appears in the bus's poll log.
    """
    bus = _bus()

    result = gentle_move(
        bus, motor=1, target=TARGET, min_angle=0, max_angle=4095, allow_motion=True
    )

    assert result["final_position"] in bus.polled_positions()


def test_gentle_move_returns_only_after_the_joint_could_have_arrived():
    """A move cannot honestly finish faster than the servo can physically travel.

    The fake's exchange rate is one poll = one poll-interval of real time, so a
    400-tick move at 10 ticks/poll takes at least 40 polls. The pre-fix loop
    spends 17 — the fake's equivalent of returning in 71 ms against ~900 ms of
    real travel.
    """
    bus = _bus()

    gentle_move(bus, motor=1, target=TARGET, min_angle=0, max_angle=4095, allow_motion=True)

    assert bus.poll_count >= DISTANCE // bus.ticks_per_poll


def test_gentle_move_detects_a_contact_created_mid_travel():
    """The whole point of the primitive: an obstacle the MOVE ITSELF runs into.

    The pre-fix loop cannot see this contact. It exhausts its goal-writes and
    returns while the shaft is still 40 ticks short of the obstacle — the load
    watch only ever caught a joint that was ALREADY loaded before the probe
    began.
    """
    bus = _bus()
    bus.place_obstacle(1, OBSTACLE)

    result = gentle_move(
        bus, motor=1, target=TARGET, min_angle=0, max_angle=4095, allow_motion=True
    )

    assert result["contacted"] is True
    assert result["overloaded"] is False

    # Contact is reported where the joint actually met resistance (allowing for
    # the few ticks of creep-under-load the bench recording shows), never at the
    # commanded target.
    assert OBSTACLE <= result["contact_position"] <= OBSTACLE + GIVE
    assert result["contact_position"] < result["clamped_target"]

    # And both readings are traceable to real bus reads.
    assert result["contact_position"] in bus.polled_positions()
    assert result["contact_load"] in bus.polled_loads()
    assert result["contact_load"] > _DEFAULT_LOAD_THRESHOLD
    assert result["contact_load"] <= _CONTACT_TORQUE_LIMIT  # load saturates at the cap


def test_gentle_move_backs_off_the_contact_point_and_holds_there():
    """Contact -> retreat a bounded distance -> HOLD with torque still on."""
    bus = _bus()
    bus.place_obstacle(1, OBSTACLE)

    result = gentle_move(
        bus, motor=1, target=TARGET, min_angle=0, max_angle=4095, allow_motion=True
    )

    assert result["contacted"] is True
    assert result["retreat_position"] == result["contact_position"] - _DEFAULT_BACKOFF_TICKS

    # The last thing commanded is the retreat, not the target and not the
    # contact point (which would keep pressing).
    assert bus.position_writes[-1] == {"motor": 1, "position": result["retreat_position"]}

    # The joint really did come off the obstacle, and we report where it is.
    assert result["final_position"] == bus.true_position(1)
    assert result["final_position"] in bus.polled_positions()
    assert result["retreat_position"] <= result["final_position"] < result["contact_position"]

    # Hold, not limp: torque is never released.
    assert bus.torque_writes[-1] == {"motor": 1, "on": True}
    assert not any(w == {"motor": 1, "on": False} for w in bus.torque_writes)


def test_gentle_move_free_motion_load_transient_is_not_contact():
    """A joint ACCELERATING is not a joint BLOCKED, however high the load reads.

    wrist_roll's free-motion load peaks at 272 — above the 250 default threshold
    — purely from accelerating through empty air. A magnitude check alone calls
    that contact and stops dead; the stall rule (load high AND position not
    advancing) does not, because the joint is plainly still moving.
    """
    bus = _bus(travel_load=FREE_MOTION_TRANSIENT)

    result = gentle_move(
        bus, motor=1, target=TARGET, min_angle=0, max_angle=4095, allow_motion=True
    )

    # The transient really was above threshold — this test would be vacuous
    # otherwise.
    assert max(bus.polled_loads()) > _DEFAULT_LOAD_THRESHOLD

    assert result["contacted"] is False
    assert result["contact_position"] is None
    assert result["final_position"] == bus.true_position(1) == TARGET


def test_gentle_move_oversized_backoff_retreat_is_clamped_to_bounds():
    """The retreat write never lands outside [min_angle, max_angle]."""
    bus = _bus()
    bus.place_obstacle(1, OBSTACLE)

    result = gentle_move(
        bus,
        motor=1,
        target=TARGET,
        min_angle=0,
        max_angle=4095,
        backoff=9000,  # deliberately oversized to prove clamping still bounds it
        allow_motion=True,
    )

    assert result["contacted"] is True
    assert result["retreat_position"] == 0  # clamped to min_angle, not negative
    for write in bus.position_writes:
        assert 0 <= write["position"] <= 4095


# ---------------------------------------------------------------------------
# No-contact path
# ---------------------------------------------------------------------------


def test_gentle_move_no_contact_reaches_clamped_target():
    """Free travel at a load well under threshold: no contact, target commanded."""
    bus = _bus()

    result = gentle_move(bus, motor=1, target=2200, min_angle=0, max_angle=4095, allow_motion=True)

    assert result["contacted"] is False
    assert result["final_position"] == 2200
    assert result["contact_position"] is None
    assert result["contact_load"] is None
    assert result["retreat_position"] is None

    # Torque is still enabled (hold), never released.
    assert bus.torque_writes[-1] == {"motor": 1, "on": True}

    # The target was actually commanded.
    assert bus.position_writes[-1] == {"motor": 1, "position": 2200}


def test_gentle_move_no_contact_moving_downward_reaches_target():
    bus = _bus(position=2200)

    result = gentle_move(bus, motor=1, target=2000, min_angle=0, max_angle=4095, allow_motion=True)

    assert result["contacted"] is False
    assert result["final_position"] == 2000
    assert bus.position_writes[-1] == {"motor": 1, "position": 2000}


def test_gentle_move_no_contact_downward_with_the_load_direction_bit_set():
    """A negative-direction load reads >= 1024 raw; masking it off is mandatory.

    Without :func:`load_magnitude` this ordinary free move trips contact on its
    very first sample.
    """
    bus = _bus(position=2200, direction_bit=True)

    result = gentle_move(bus, motor=1, target=2000, min_angle=0, max_angle=4095, allow_motion=True)

    assert max(bus.polled_loads()) >= 1024  # the raw register really was negative
    assert result["contacted"] is False


def test_gentle_move_torque_is_never_released():
    """Hold, never limp — for every ending, torque stays enabled."""
    bus = _bus()

    gentle_move(bus, motor=1, target=TARGET, min_angle=0, max_angle=4095, allow_motion=True)

    assert bus.torque_writes  # torque was enabled at least once
    assert bus.torque_writes[-1] == {"motor": 1, "on": True}
    assert not any(w == {"motor": 1, "on": False} for w in bus.torque_writes)


def test_gentle_move_target_equals_start_is_a_no_op_move():
    """Target already at the current position: direction is zero, so the
    stepping loop never runs — setup happens, but no goal-position write and
    no contact. final_position is the (unchanged) clamped target."""
    bus = _bus()

    result = gentle_move(bus, motor=1, target=START, min_angle=0, max_angle=4095, allow_motion=True)

    assert result["contacted"] is False
    assert result["overloaded"] is False
    assert result["final_position"] == START
    assert result["contact_position"] is None
    # Compliant setup still ran (torque enabled to hold), but no step was written.
    assert bus.torque_writes[-1] == {"motor": 1, "on": True}
    assert bus.position_writes == []


# ---------------------------------------------------------------------------
# Threshold: default + override
#
# Note what is NOT here any more. The old suite asserted that a higher threshold
# "travels further before contact" and a lower one "trips sooner" — properties of
# the old fake, whose load ramped with every goal-WRITE, not properties of a
# servo. Against an honest model the obstacle decides WHERE contact happens and
# the threshold decides only WHETHER it is recognised; and since load saturates
# at Torque_Limit, a threshold at or above the cap can never fire at all (see
# test_every_default_contact_threshold_sits_below_the_torque_cap).
# ---------------------------------------------------------------------------


def test_gentle_move_default_threshold_used_when_unspecified():
    bus = _bus()

    result = gentle_move(bus, motor=1, target=2200, min_angle=0, max_angle=4095, allow_motion=True)

    assert result["threshold"] == _DEFAULT_LOAD_THRESHOLD


def test_gentle_move_threshold_override_is_reported():
    bus = _bus()

    result = gentle_move(
        bus,
        motor=1,
        target=2200,
        min_angle=0,
        max_angle=4095,
        threshold=420,
        allow_motion=True,
    )

    assert result["threshold"] == 420
    assert result["contacted"] is False


# ---------------------------------------------------------------------------
# Bounds / clamping
# ---------------------------------------------------------------------------


def test_gentle_move_target_beyond_max_is_clamped():
    bus = _bus()

    result = gentle_move(bus, motor=1, target=9000, min_angle=0, max_angle=3000, allow_motion=True)

    assert result["was_clamped"] is True
    assert result["clamped_target"] == 3000
    assert result["contacted"] is False
    assert result["final_position"] == 3000

    for write in bus.position_writes:
        assert 0 <= write["position"] <= 3000


def test_gentle_move_target_below_min_is_clamped():
    bus = _bus()

    result = gentle_move(
        bus, motor=1, target=-500, min_angle=1000, max_angle=4095, allow_motion=True
    )

    assert result["was_clamped"] is True
    assert result["clamped_target"] == 1000
    assert result["final_position"] == 1000

    for write in bus.position_writes:
        assert 1000 <= write["position"] <= 4095


def test_gentle_move_steps_never_exceed_step_increment():
    """Consecutive goal writes never jump by more than `step` ticks.

    Goal-stepping stays part of the contract after the rewrite: ``step`` is a
    public parameter and is echoed in the result dict, and small increments are
    what keep the approach gentle. Only the *sampling* changes.
    """
    bus = _bus(position=0)

    gentle_move(
        bus,
        motor=1,
        target=300,
        min_angle=0,
        max_angle=4095,
        step=_DEFAULT_STEP_TICKS,
        allow_motion=True,
    )

    prev = 0
    for write in bus.position_writes:
        assert write["position"] - prev <= _DEFAULT_STEP_TICKS
        prev = write["position"]
    assert bus.position_writes[-1]["position"] == 300


# ---------------------------------------------------------------------------
# Compliant setup writes
# ---------------------------------------------------------------------------


def test_gentle_move_writes_acceleration_and_speed_once():
    bus = _bus()

    result = gentle_move(
        bus,
        motor=1,
        target=2200,
        min_angle=0,
        max_angle=4095,
        acceleration=15,
        speed=350,
        allow_motion=True,
    )

    assert bus.accel_writes == [{"motor": 1, "value": 15}]
    assert bus.speed_writes == [{"motor": 1, "value": 350}]
    assert result["acceleration"] == 15
    assert result["speed"] == 350
