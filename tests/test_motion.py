"""Tests for arm101.hardware.motion — clamp_goal + compliant_move, and the new
bus.py write primitives (write_acceleration / write_goal_speed) they depend on.

TDD: written before motion.py existed (and before the new bus primitives were
added) to drive the implementation.
"""

from __future__ import annotations

import pytest

from arm101.cli._errors import EXIT_ENV_ERROR, EXIT_USER_ERROR, CliError
from arm101.hardware.bus import FakeBus
from arm101.hardware.motion import clamp_goal, compliant_move

# ---------------------------------------------------------------------------
# 1. FakeBus.write_acceleration
# ---------------------------------------------------------------------------


def test_fakebus_records_write_acceleration():
    """FakeBus records every write_acceleration call in accel_writes."""
    bus = FakeBus()
    bus.open()

    bus.write_acceleration(motor=1, value=20)
    bus.write_acceleration(motor=2, value=0)

    assert len(bus.accel_writes) == 2
    assert bus.accel_writes[0] == {"motor": 1, "value": 20}
    assert bus.accel_writes[1] == {"motor": 2, "value": 0}


def test_fakebus_accel_writes_empty_on_init():
    """FakeBus starts with no recorded acceleration writes."""
    bus = FakeBus()
    assert bus.accel_writes == []


def test_fakebus_write_acceleration_not_open_raises():
    """write_acceleration raises CliError(EXIT_ENV_ERROR) when bus is not open."""
    bus = FakeBus()
    with pytest.raises(CliError) as exc:
        bus.write_acceleration(motor=1, value=20)
    assert exc.value.code == EXIT_ENV_ERROR


def test_fakebus_write_acceleration_out_of_range_raises():
    """write_acceleration raises CliError(EXIT_USER_ERROR) for out-of-range value."""
    bus = FakeBus()
    bus.open()

    with pytest.raises(CliError) as exc:
        bus.write_acceleration(motor=1, value=255)
    assert exc.value.code == EXIT_USER_ERROR

    with pytest.raises(CliError) as exc:
        bus.write_acceleration(motor=1, value=-1)
    assert exc.value.code == EXIT_USER_ERROR

    # An out-of-range call must not be recorded.
    assert bus.accel_writes == []


def test_fakebus_write_acceleration_boundary_values():
    """write_acceleration accepts boundary values 0 and 254."""
    bus = FakeBus()
    bus.open()

    bus.write_acceleration(motor=1, value=0)
    bus.write_acceleration(motor=1, value=254)

    assert bus.accel_writes[0]["value"] == 0
    assert bus.accel_writes[1]["value"] == 254


# ---------------------------------------------------------------------------
# 2. FakeBus.write_goal_speed
# ---------------------------------------------------------------------------


def test_fakebus_records_write_goal_speed():
    """FakeBus records every write_goal_speed call in speed_writes."""
    bus = FakeBus()
    bus.open()

    bus.write_goal_speed(motor=1, value=400)
    bus.write_goal_speed(motor=2, value=0)

    assert len(bus.speed_writes) == 2
    assert bus.speed_writes[0] == {"motor": 1, "value": 400}
    assert bus.speed_writes[1] == {"motor": 2, "value": 0}


def test_fakebus_speed_writes_empty_on_init():
    """FakeBus starts with no recorded goal-speed writes."""
    bus = FakeBus()
    assert bus.speed_writes == []


def test_fakebus_write_goal_speed_not_open_raises():
    """write_goal_speed raises CliError(EXIT_ENV_ERROR) when bus is not open."""
    bus = FakeBus()
    with pytest.raises(CliError) as exc:
        bus.write_goal_speed(motor=1, value=400)
    assert exc.value.code == EXIT_ENV_ERROR


def test_fakebus_write_goal_speed_out_of_range_raises():
    """write_goal_speed raises CliError(EXIT_USER_ERROR) for out-of-range value."""
    bus = FakeBus()
    bus.open()

    with pytest.raises(CliError) as exc:
        bus.write_goal_speed(motor=1, value=4096)
    assert exc.value.code == EXIT_USER_ERROR

    with pytest.raises(CliError) as exc:
        bus.write_goal_speed(motor=1, value=-1)
    assert exc.value.code == EXIT_USER_ERROR

    # An out-of-range call must not be recorded.
    assert bus.speed_writes == []


def test_fakebus_write_goal_speed_boundary_values():
    """write_goal_speed accepts boundary values 0 and 4095."""
    bus = FakeBus()
    bus.open()

    bus.write_goal_speed(motor=1, value=0)
    bus.write_goal_speed(motor=1, value=4095)

    assert bus.speed_writes[0]["value"] == 0
    assert bus.speed_writes[1]["value"] == 4095


# ---------------------------------------------------------------------------
# 3. clamp_goal
# ---------------------------------------------------------------------------


def test_clamp_goal_in_range_returns_unchanged():
    """A target already inside [lo, hi] is returned unchanged, was_clamped=False."""
    assert clamp_goal(2048, 0, 4095) == (2048, False)
    assert clamp_goal(0, 0, 4095) == (0, False)
    assert clamp_goal(4095, 0, 4095) == (4095, False)


def test_clamp_goal_below_lo_clamps_up():
    """A target below lo is clamped to lo, was_clamped=True."""
    assert clamp_goal(-100, 0, 4095) == (0, True)
    assert clamp_goal(50, 100, 4095) == (100, True)


def test_clamp_goal_above_hi_clamps_down():
    """A target above hi is clamped to hi, was_clamped=True."""
    assert clamp_goal(9999, 0, 4095) == (4095, True)
    assert clamp_goal(4096, 0, 4095) == (4095, True)


@pytest.mark.parametrize(
    "target,lo,hi",
    [
        (-5000, 0, 4095),
        (-1, 0, 4095),
        (0, 0, 4095),
        (2048, 0, 4095),
        (4095, 0, 4095),
        (4096, 0, 4095),
        (10_000, 0, 4095),
        (500, 200, 800),
        (199, 200, 800),
        (801, 200, 800),
    ],
)
def test_clamp_goal_result_never_outside_bounds(target, lo, hi):
    """Property check: clamp_goal's result is always within [lo, hi]."""
    clamped, _was_clamped = clamp_goal(target, lo, hi)
    assert lo <= clamped <= hi


def test_clamp_goal_lo_greater_than_hi_raises():
    """lo > hi is a programming error -> CliError, not a silent swap."""
    with pytest.raises(CliError) as exc:
        clamp_goal(100, 800, 200)
    assert exc.value.code == EXIT_USER_ERROR


# ---------------------------------------------------------------------------
# 4. compliant_move
# ---------------------------------------------------------------------------


def test_compliant_move_without_allow_motion_raises_and_writes_nothing():
    """allow_motion defaults to False; the call must raise and record no writes."""
    bus = FakeBus()
    bus.open()

    with pytest.raises(CliError) as exc:
        compliant_move(bus, motor=1, target=2048, min_angle=0, max_angle=4095)
    assert exc.value.code == EXIT_USER_ERROR

    assert bus.accel_writes == []
    assert bus.speed_writes == []
    assert bus.torque_writes == []
    assert bus.position_writes == []


def test_compliant_move_allow_motion_false_explicit_raises_and_writes_nothing():
    """allow_motion=False explicitly also raises and writes nothing."""
    bus = FakeBus()
    bus.open()

    with pytest.raises(CliError) as exc:
        compliant_move(bus, motor=1, target=2048, min_angle=0, max_angle=4095, allow_motion=False)
    assert exc.value.code == EXIT_USER_ERROR
    assert bus.position_writes == []


def test_compliant_move_clamps_beyond_max_target():
    """A target beyond max_angle is clamped to max_angle before the move."""
    bus = FakeBus()
    bus.open()

    result = compliant_move(
        bus,
        motor=1,
        target=9000,
        min_angle=0,
        max_angle=4095,
        allow_motion=True,
    )

    assert result["was_clamped"] is True
    assert result["clamped_target"] == 4095
    assert result["requested_target"] == 9000
    assert bus.position_writes[-1]["position"] == 4095


def test_compliant_move_clamps_below_min_target():
    """A target below min_angle is clamped up to min_angle before the move."""
    bus = FakeBus()
    bus.open()

    result = compliant_move(
        bus,
        motor=1,
        target=-500,
        min_angle=200,
        max_angle=4000,
        allow_motion=True,
    )

    assert result["was_clamped"] is True
    assert result["clamped_target"] == 200
    assert bus.position_writes[-1]["position"] == 200


def test_compliant_move_in_range_target_not_clamped():
    """A target already within range is not clamped."""
    bus = FakeBus()
    bus.open()

    result = compliant_move(
        bus,
        motor=1,
        target=2048,
        min_angle=0,
        max_angle=4095,
        allow_motion=True,
    )

    assert result["was_clamped"] is False
    assert result["clamped_target"] == 2048


def test_compliant_move_writes_in_order_accel_speed_torque_position():
    """The four writes happen in the documented order: accel -> speed -> torque -> position.

    Verified by interleaving a shared sequence counter via monkeypatching each
    FakeBus method's call, captured indirectly through the per-list lengths at
    each step using a simple call-order list built from the four lists.
    """
    bus = FakeBus()
    bus.open()

    compliant_move(
        bus,
        motor=1,
        target=2048,
        min_angle=0,
        max_angle=4095,
        allow_motion=True,
    )

    assert len(bus.accel_writes) == 1
    assert len(bus.speed_writes) == 1
    assert len(bus.torque_writes) == 1
    assert len(bus.position_writes) == 1

    # Reconstruct call order by wrapping the bus methods and recording the
    # order of invocation for a second call (the first already proved each
    # write happened exactly once; this proves order).
    call_order: list[str] = []
    orig_accel = bus.write_acceleration
    orig_speed = bus.write_goal_speed
    orig_torque = bus.enable_torque
    orig_position = bus.write_goal_position

    def _accel(*args, **kwargs):
        call_order.append("acceleration")
        return orig_accel(*args, **kwargs)

    def _speed(*args, **kwargs):
        call_order.append("speed")
        return orig_speed(*args, **kwargs)

    def _torque(*args, **kwargs):
        call_order.append("torque")
        return orig_torque(*args, **kwargs)

    def _position(*args, **kwargs):
        call_order.append("position")
        return orig_position(*args, **kwargs)

    bus.write_acceleration = _accel
    bus.write_goal_speed = _speed
    bus.enable_torque = _torque
    bus.write_goal_position = _position

    compliant_move(
        bus,
        motor=1,
        target=2048,
        min_angle=0,
        max_angle=4095,
        allow_motion=True,
    )

    assert call_order == ["acceleration", "speed", "torque", "position"]


def test_compliant_move_uses_gentle_defaults_when_unspecified():
    """Default acceleration/speed are gentle and get written through to the bus."""
    bus = FakeBus()
    bus.open()

    result = compliant_move(
        bus,
        motor=1,
        target=2048,
        min_angle=0,
        max_angle=4095,
        allow_motion=True,
    )

    assert bus.accel_writes[-1]["value"] == result["acceleration"]
    assert bus.speed_writes[-1]["value"] == result["speed"]
    # Sane, gentle bounds — not asserting an exact magic number, just that the
    # defaults stay within the documented gentle envelope.
    assert 0 < result["acceleration"] <= 50
    assert 0 < result["speed"] <= 1000


def test_compliant_move_custom_acceleration_and_speed_are_used():
    """Caller-supplied acceleration/speed override the defaults and are written."""
    bus = FakeBus()
    bus.open()

    result = compliant_move(
        bus,
        motor=1,
        target=2048,
        min_angle=0,
        max_angle=4095,
        acceleration=10,
        speed=300,
        allow_motion=True,
    )

    assert result["acceleration"] == 10
    assert result["speed"] == 300
    assert bus.accel_writes[-1]["value"] == 10
    assert bus.speed_writes[-1]["value"] == 300


def test_compliant_move_return_dict_reports_motor_and_targets():
    """The returned dict reports motor, requested target, and clamped target."""
    bus = FakeBus()
    bus.open()

    result = compliant_move(
        bus,
        motor=3,
        target=5000,
        min_angle=0,
        max_angle=4095,
        allow_motion=True,
    )

    assert result["motor"] == 3
    assert result["requested_target"] == 5000
    assert result["clamped_target"] == 4095
    assert result["was_clamped"] is True
