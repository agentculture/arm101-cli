"""Tests for gentle.py's gentler default speed + graceful overload + torque-cap.

Covers (arm101.hardware.gentle):

* ``_DEFAULT_SPEED`` lowered to a conservative value (<= 150), down from the
  original 400.
* A mid-move ``OverloadError`` (raised by any bus call inside the step
  loop — the ``present_load`` read or a ``write_goal_position``) is caught:
  ``gentle_move`` calls ``bus.clear_overload(motor)`` and RETURNS its result
  dict with ``overloaded=True`` instead of raising. The happy path's result
  dict gets ``overloaded=False``.
* ``gentle_move`` caps the RAM ``Torque_Limit`` at the start of the move
  (below the servo's factory trip point) and restores the pre-move value in
  a ``finally``, on both the happy path and the overload path.

TDD: written before the corresponding gentle.py changes; they must fail
against the current code and drive the implementation. Builds on the t1
bus-layer overload/torque-limit API (arm101.hardware.bus: is_overload,
OverloadError, read_torque_limit/write_torque_limit, clear_overload,
FakeBus's overload_after_ops / fail_with_overload_on_op seam).
"""

from __future__ import annotations

from arm101.hardware.bus import FakeBus
from arm101.hardware.gentle import _CONTACT_TORQUE_LIMIT, _DEFAULT_SPEED, gentle_move

# ---------------------------------------------------------------------------
# Local test double — records the live Torque_Limit at every goal-position
# write, so a test can prove the cap was actually in effect DURING the move
# (not merely bookkept before/after).
# ---------------------------------------------------------------------------


class _CapTrackingBus(FakeBus):
    """FakeBus that snapshots the current Torque_Limit at each goal-position write."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.torque_limit_at_writes: list[int] = []

    def write_goal_position(self, motor: int, position: int) -> None:
        self.torque_limit_at_writes.append(self.read_torque_limit(motor))
        super().write_goal_position(motor, position)


# ---------------------------------------------------------------------------
# 1. Gentler default speed
# ---------------------------------------------------------------------------


def test_default_speed_constant_is_conservative():
    assert _DEFAULT_SPEED <= 150


def test_gentle_move_uses_default_speed_when_unspecified():
    bus = FakeBus(positions={1: 2048})
    bus.open()

    result = gentle_move(
        bus,
        motor=1,
        target=2200,
        min_angle=0,
        max_angle=4095,
        allow_motion=True,
    )

    assert result["speed"] == _DEFAULT_SPEED
    assert bus.speed_writes == [{"motor": 1, "value": _DEFAULT_SPEED}]


# ---------------------------------------------------------------------------
# 2. Happy path: overloaded=False
# ---------------------------------------------------------------------------


def test_gentle_move_happy_path_result_includes_overloaded_false():
    bus = FakeBus(positions={1: 2048})
    bus.open()

    result = gentle_move(
        bus,
        motor=1,
        target=2200,
        min_angle=0,
        max_angle=4095,
        allow_motion=True,
    )

    assert result["overloaded"] is False
    # Existing contract keys are all still present.
    for key in (
        "motor",
        "requested_target",
        "clamped_target",
        "was_clamped",
        "start_position",
        "threshold",
        "step",
        "backoff_ticks",
        "acceleration",
        "speed",
        "contacted",
        "contact_position",
        "contact_load",
        "retreat_position",
        "final_position",
    ):
        assert key in result


# ---------------------------------------------------------------------------
# 3. Mid-move OverloadError -> graceful stop, no raise
# ---------------------------------------------------------------------------


def test_gentle_move_mid_move_overload_on_goal_position_write_returns_gracefully():
    """Overload raised BY write_goal_position itself, mid-loop, does not propagate."""
    bus = FakeBus(positions={1: 2048}).fail_with_overload_on_op(7)
    bus.open()

    # Ops 1-6 are: read_torque_limit, write_torque_limit(cap), write_acceleration,
    # write_goal_speed, enable_torque, read_info(start_position). Op 7 is the
    # FIRST write_goal_position inside the step loop.
    result = gentle_move(
        bus,
        motor=1,
        target=2500,
        min_angle=0,
        max_angle=4095,
        allow_motion=True,
    )

    assert result["overloaded"] is True
    assert result["motor"] == 1
    assert result["contacted"] is False


def test_gentle_move_mid_move_overload_on_present_load_read_returns_gracefully():
    """Overload raised by the present_load read (bus.read_info) after a successful step."""
    bus = FakeBus(positions={1: 2048}).fail_with_overload_on_op(8)
    bus.open()

    # Op 7 (first write_goal_position) succeeds; op 8 (the present_load read
    # that follows it) is where the overload fires.
    result = gentle_move(
        bus,
        motor=1,
        target=2500,
        min_angle=0,
        max_angle=4095,
        allow_motion=True,
    )

    assert result["overloaded"] is True
    # The step write itself succeeded before the load read overloaded.
    assert bus.position_writes  # at least one goal-position write happened


def test_gentle_move_overload_calls_clear_overload():
    bus = FakeBus(positions={1: 2048}).fail_with_overload_on_op(7)
    bus.open()

    gentle_move(
        bus,
        motor=1,
        target=2500,
        min_angle=0,
        max_angle=4095,
        allow_motion=True,
    )

    # clear_overload() records a torque-off entry and disarms the seam.
    assert bus.torque_writes[-1] == {"motor": 1, "on": False}
    assert bus.overload_after_ops is None


def test_gentle_move_overload_does_not_raise():
    """The whole point: a mid-move overload is recovered from, never propagated."""
    bus = FakeBus(positions={1: 2048}).fail_with_overload_on_op(7)
    bus.open()

    # No pytest.raises here on purpose — a raise would fail this test.
    result = gentle_move(
        bus,
        motor=1,
        target=2500,
        min_angle=0,
        max_angle=4095,
        allow_motion=True,
    )
    assert isinstance(result, dict)


# ---------------------------------------------------------------------------
# 4. Torque_Limit cap applied at the start, restored in a finally
# ---------------------------------------------------------------------------


def test_gentle_move_caps_torque_limit_during_move_happy_path():
    bus = _CapTrackingBus(positions={1: 2048})
    bus.open()

    gentle_move(
        bus,
        motor=1,
        target=2200,
        min_angle=0,
        max_angle=4095,
        allow_motion=True,
    )

    # Every goal-position write during the move saw the CAPPED limit, not
    # the factory default (1000) -- proves the cap was live DURING motion.
    assert bus.torque_limit_at_writes
    assert all(v == _CONTACT_TORQUE_LIMIT for v in bus.torque_limit_at_writes)


def test_gentle_move_restores_torque_limit_after_happy_path():
    bus = FakeBus(positions={1: 2048})
    bus.open()
    original = bus.read_torque_limit(1)
    assert original == 1000  # FakeBus factory default

    gentle_move(
        bus,
        motor=1,
        target=2200,
        min_angle=0,
        max_angle=4095,
        allow_motion=True,
    )

    assert bus.torque_limit_writes[0] == {"motor": 1, "value": _CONTACT_TORQUE_LIMIT}
    assert bus.torque_limit_writes[-1] == {"motor": 1, "value": original}
    assert bus.read_torque_limit(1) == original


def test_gentle_move_restores_torque_limit_after_overload_path():
    bus = FakeBus(positions={1: 2048}, torque_limits={1: 1000}).fail_with_overload_on_op(7)
    bus.open()

    result = gentle_move(
        bus,
        motor=1,
        target=2500,
        min_angle=0,
        max_angle=4095,
        allow_motion=True,
    )

    assert result["overloaded"] is True
    # Cap applied at the start...
    assert bus.torque_limit_writes[0] == {"motor": 1, "value": _CONTACT_TORQUE_LIMIT}
    # ...and restored in the finally, even though the move overloaded.
    assert bus.torque_limit_writes[-1] == {"motor": 1, "value": 1000}
    assert bus.read_torque_limit(1) == 1000


def test_gentle_move_torque_limit_restored_with_non_default_original_value():
    """The restore uses the motor's ACTUAL pre-move value, not a hardcoded 1000."""
    bus = FakeBus(positions={1: 2048}, torque_limits={1: 700})
    bus.open()

    gentle_move(
        bus,
        motor=1,
        target=2200,
        min_angle=0,
        max_angle=4095,
        allow_motion=True,
    )

    assert bus.torque_limit_writes[0] == {"motor": 1, "value": _CONTACT_TORQUE_LIMIT}
    assert bus.torque_limit_writes[-1] == {"motor": 1, "value": 700}
    assert bus.read_torque_limit(1) == 700
