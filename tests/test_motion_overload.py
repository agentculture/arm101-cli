"""Tests for arm101.hardware.motion.compliant_move — gentler default + graceful overload.

Covers task t3 of the "arm motion is overload-safe" plan:

* the default goal-speed is lowered to a conservative value (<=150, matching
  ``gentle_move``'s gentle default) instead of the previous, snappier value.
* a mid-move :class:`~arm101.hardware.bus.OverloadError` raised by ANY of the
  four bus calls inside :func:`compliant_move` is caught, recovered from (via
  ``bus.clear_overload(motor)``), and surfaced as ``overloaded=True`` in the
  returned dict instead of propagating as an exception.
* the happy path carries ``overloaded=False`` and is otherwise unaffected.

Driven entirely through the t1 ``FakeBus`` overload-simulation seam
(``fail_with_overload_on_op`` / ``overload_after_ops``) — no real hardware.

TDD: written before the corresponding motion.py changes; they must fail
against the current code (default speed 400, no ``overloaded`` key, no
overload recovery) and drive the implementation.
"""

from __future__ import annotations

import pytest

from arm101.cli._errors import EXIT_USER_ERROR, CliError
from arm101.hardware.bus import FakeBus
from arm101.hardware.motion import compliant_move

# ---------------------------------------------------------------------------
# 1. Gentler default goal-speed
# ---------------------------------------------------------------------------


def test_compliant_move_default_speed_is_150():
    """The default goal-speed is lowered to the conservative value 150."""
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

    assert result["speed"] == 150
    assert result["speed"] <= 150


def test_compliant_move_default_speed_is_written_to_the_bus():
    """The lowered default speed is actually written through to the bus."""
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

    assert bus.speed_writes[-1] == {"motor": 1, "value": 150}


def test_compliant_move_custom_speed_still_overrides_default():
    """A caller-supplied speed still overrides the (now-lower) default."""
    bus = FakeBus()
    bus.open()

    result = compliant_move(
        bus,
        motor=1,
        target=2048,
        min_angle=0,
        max_angle=4095,
        speed=300,
        allow_motion=True,
    )

    assert result["speed"] == 300
    assert bus.speed_writes[-1]["value"] == 300


# ---------------------------------------------------------------------------
# 2. Happy path carries overloaded=False
# ---------------------------------------------------------------------------


def test_compliant_move_happy_path_overloaded_is_false():
    """On a normal move, the result dict reports overloaded=False."""
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

    assert result["overloaded"] is False
    # Existing contract keys are all still present alongside the new one.
    assert result["motor"] == 1
    assert result["requested_target"] == 2048
    assert result["clamped_target"] == 2048
    assert result["was_clamped"] is False
    assert result["acceleration"] == 20


# ---------------------------------------------------------------------------
# 3. Mid-move OverloadError is caught and recovered from
# ---------------------------------------------------------------------------


def test_compliant_move_overload_on_enable_torque_returns_gracefully():
    """An overload on the 3rd bus op (enable_torque) does not raise.

    Bus op order inside compliant_move is: write_acceleration (1),
    write_goal_speed (2), enable_torque (3), write_goal_position (4).
    Arming the seam for op 3 simulates the fault landing squarely mid-move,
    after the gentle accel/speed setup but before torque/position.
    """
    bus = FakeBus().fail_with_overload_on_op(3)
    bus.open()

    result = compliant_move(
        bus,
        motor=2,
        target=3000,
        min_angle=0,
        max_angle=4095,
        allow_motion=True,
    )

    assert result["overloaded"] is True
    # The move never reached the goal-position write.
    assert bus.position_writes == []


def test_compliant_move_overload_calls_clear_overload():
    """Recovery explicitly calls bus.clear_overload(motor) on the affected motor."""
    bus = FakeBus().fail_with_overload_on_op(3)
    bus.open()

    compliant_move(
        bus,
        motor=4,
        target=1000,
        min_angle=0,
        max_angle=4095,
        allow_motion=True,
    )

    # clear_overload disables torque (addr 40, value 0) as its recovery action.
    assert bus.torque_writes[-1] == {"motor": 4, "on": False}
    assert bus.register_writes[-1] == {"motor": 4, "addr": 40, "value": 0}
    # clear_overload also disarms the FakeBus overload-simulation seam.
    assert bus.overload_after_ops is None


def test_compliant_move_overload_on_first_op_still_recovers():
    """Even an overload on the very first bus call (write_acceleration) recovers cleanly."""
    bus = FakeBus().fail_with_overload_on_op(1)
    bus.open()

    result = compliant_move(
        bus,
        motor=5,
        target=2048,
        min_angle=0,
        max_angle=4095,
        allow_motion=True,
    )

    assert result["overloaded"] is True
    assert bus.torque_writes[-1] == {"motor": 5, "on": False}


def test_compliant_move_overload_on_last_op_still_recovers():
    """An overload on the final bus call (write_goal_position) still recovers cleanly."""
    bus = FakeBus().fail_with_overload_on_op(4)
    bus.open()

    result = compliant_move(
        bus,
        motor=6,
        target=2048,
        min_angle=0,
        max_angle=4095,
        allow_motion=True,
    )

    assert result["overloaded"] is True
    assert bus.torque_writes[-1] == {"motor": 6, "on": False}


def test_compliant_move_overload_result_preserves_existing_contract_keys():
    """The overloaded-result dict still carries the pre-existing contract keys."""
    bus = FakeBus().fail_with_overload_on_op(3)
    bus.open()

    result = compliant_move(
        bus,
        motor=7,
        target=9000,
        min_angle=0,
        max_angle=4095,
        acceleration=10,
        speed=100,
        allow_motion=True,
    )

    assert result["motor"] == 7
    assert result["requested_target"] == 9000
    assert result["clamped_target"] == 4095
    assert result["was_clamped"] is True
    assert result["acceleration"] == 10
    assert result["speed"] == 100
    assert result["overloaded"] is True


# ---------------------------------------------------------------------------
# 4. Non-overload failures still propagate (not swallowed)
# ---------------------------------------------------------------------------


def test_compliant_move_non_overload_error_still_propagates():
    """A non-overload CliError (bad caller input) is not swallowed as a graceful overload."""
    bus = FakeBus()
    bus.open()

    with pytest.raises(CliError) as exc:
        compliant_move(
            bus,
            motor=1,
            target=2048,
            min_angle=0,
            max_angle=4095,
            acceleration=999,  # out of range -> plain CliError(EXIT_USER_ERROR), not overload
            allow_motion=True,
        )
    assert exc.value.code == EXIT_USER_ERROR


def test_compliant_move_still_gated_by_allow_motion_when_overload_seam_armed():
    """allow_motion=False still raises before any bus call, even with the seam armed."""
    bus = FakeBus().fail_with_overload_on_op(1)
    bus.open()

    with pytest.raises(CliError) as exc:
        compliant_move(bus, motor=1, target=2048, min_angle=0, max_angle=4095)
    assert exc.value.code == EXIT_USER_ERROR
    assert bus.position_writes == []
