"""Tests for arm101.hardware.gentle — load-watch back-off-then-hold compliant move.

TDD: written before gentle.py existed. Drives gentle_move's contract:
step incrementally toward target, watch present_load, stop + back off a
bounded number of ticks on contact, and HOLD there with torque still
enabled (never a limp release, never a hard freeze at the contact point).
"""

from __future__ import annotations

import pytest

from arm101.cli._errors import EXIT_USER_ERROR, CliError
from arm101.hardware.bus import FakeBus
from arm101.hardware.gentle import (
    _DEFAULT_BACKOFF_TICKS,
    _DEFAULT_LOAD_THRESHOLD,
    _DEFAULT_STEP_TICKS,
    gentle_move,
)

# ---------------------------------------------------------------------------
# Local test double — NOT in bus.py (per task scope, kept local to this file).
# ---------------------------------------------------------------------------


class RampLoadBus(FakeBus):
    """FakeBus whose present_load ramps with each commanded goal-position write.

    Models a gripper closing on an obstacle: every write_goal_position call
    bumps a simulated load counter by a fixed increment, and read_info()
    reports BOTH that ramped load and the most recently commanded position
    (the base FakeBus.write_goal_position only records to position_writes —
    it does not update the position read_info reports) — so gentle_move's
    stepping/backoff math is exercised against a present_position that
    actually tracks the last commanded goal, not a frozen start value.
    """

    def __init__(self, *args, load_increment: int = 40, **kwargs):
        super().__init__(*args, **kwargs)
        self._load_increment = load_increment
        self._load = 0

    def write_goal_position(self, motor: int, position: int) -> None:
        super().write_goal_position(motor, position)
        self._positions[motor] = position
        self._load += self._load_increment

    def read_info(self, motor: int) -> dict:
        info = super().read_info(motor)
        info["present_load"] = self._load
        return info


# ---------------------------------------------------------------------------
# allow_motion gate
# ---------------------------------------------------------------------------


def test_gentle_move_without_allow_motion_raises_and_writes_nothing():
    bus = RampLoadBus(positions={1: 2048})
    bus.open()

    with pytest.raises(CliError) as exc:
        gentle_move(bus, motor=1, target=3000, min_angle=0, max_angle=4095)
    assert exc.value.code == EXIT_USER_ERROR

    assert bus.accel_writes == []
    assert bus.speed_writes == []
    assert bus.torque_writes == []
    assert bus.position_writes == []


def test_gentle_move_allow_motion_false_explicit_raises_and_writes_nothing():
    bus = RampLoadBus(positions={1: 2048})
    bus.open()

    with pytest.raises(CliError) as exc:
        gentle_move(bus, motor=1, target=3000, min_angle=0, max_angle=4095, allow_motion=False)
    assert exc.value.code == EXIT_USER_ERROR
    assert bus.position_writes == []


# ---------------------------------------------------------------------------
# Contact detected -> back off + hold
# ---------------------------------------------------------------------------


def test_gentle_move_detects_contact_and_backs_off():
    """Load ramps past threshold mid-travel -> contacted, retreat write recorded."""
    bus = RampLoadBus(positions={1: 2048}, load_increment=40)
    bus.open()

    result = gentle_move(
        bus,
        motor=1,
        target=3500,
        min_angle=0,
        max_angle=4095,
        allow_motion=True,
    )

    assert result["contacted"] is True
    assert result["contact_load"] > _DEFAULT_LOAD_THRESHOLD

    # Contact tripped on the 7th step (40*7=280 > 250 default threshold):
    # 2048 + 7*25 = 2223.
    assert result["contact_position"] == 2048 + 7 * _DEFAULT_STEP_TICKS

    # Retreat is exactly backoff ticks back along the direction of travel.
    expected_retreat = result["contact_position"] - _DEFAULT_BACKOFF_TICKS
    assert result["retreat_position"] == expected_retreat
    assert result["final_position"] == expected_retreat

    # The LAST position write recorded on the bus is the retreat, not the
    # contact point and not the original target.
    assert bus.position_writes[-1] == {"motor": 1, "position": expected_retreat}


def test_gentle_move_hold_not_limp_after_contact():
    """Torque stays enabled after a contact -> back-off -> hold; never released."""
    bus = RampLoadBus(positions={1: 2048}, load_increment=40)
    bus.open()

    gentle_move(
        bus,
        motor=1,
        target=3500,
        min_angle=0,
        max_angle=4095,
        allow_motion=True,
    )

    assert bus.torque_writes  # torque was enabled at least once
    assert bus.torque_writes[-1] == {"motor": 1, "on": True}
    assert not any(w == {"motor": 1, "on": False} for w in bus.torque_writes)


def test_gentle_move_backoff_is_bounded_and_within_joint_bounds():
    """The retreat write never lands outside [min_angle, max_angle]."""
    bus = RampLoadBus(positions={1: 50}, load_increment=40)
    bus.open()

    # min_angle is close to the contact point so the unclamped retreat target
    # would fall below 0; the bounded backoff must be clamped, not negative.
    result = gentle_move(
        bus,
        motor=1,
        target=3500,
        min_angle=0,
        max_angle=4095,
        backoff=9000,  # deliberately oversized to prove clamping still bounds it
        allow_motion=True,
    )

    assert result["contacted"] is True
    assert 0 <= result["retreat_position"] <= 4095
    for write in bus.position_writes:
        assert 0 <= write["position"] <= 4095


# ---------------------------------------------------------------------------
# Threshold: default + override
# ---------------------------------------------------------------------------


def test_gentle_move_default_threshold_used_when_unspecified():
    bus = RampLoadBus(positions={1: 2048}, load_increment=40)
    bus.open()

    result = gentle_move(
        bus,
        motor=1,
        target=3500,
        min_angle=0,
        max_angle=4095,
        allow_motion=True,
    )

    assert result["threshold"] == _DEFAULT_LOAD_THRESHOLD
    assert result["contacted"] is True


def test_gentle_move_higher_threshold_travels_further_before_contact():
    bus_default = RampLoadBus(positions={1: 2048}, load_increment=40)
    bus_default.open()
    result_default = gentle_move(
        bus_default,
        motor=1,
        target=3500,
        min_angle=0,
        max_angle=4095,
        allow_motion=True,
    )

    bus_high = RampLoadBus(positions={1: 2048}, load_increment=40)
    bus_high.open()
    result_high = gentle_move(
        bus_high,
        motor=1,
        target=3500,
        min_angle=0,
        max_angle=4095,
        threshold=600,
        allow_motion=True,
    )

    assert result_high["contacted"] is True
    assert result_high["contact_position"] > result_default["contact_position"]


def test_gentle_move_lower_threshold_trips_sooner():
    bus_default = RampLoadBus(positions={1: 2048}, load_increment=40)
    bus_default.open()
    result_default = gentle_move(
        bus_default,
        motor=1,
        target=3500,
        min_angle=0,
        max_angle=4095,
        allow_motion=True,
    )

    bus_low = RampLoadBus(positions={1: 2048}, load_increment=40)
    bus_low.open()
    result_low = gentle_move(
        bus_low,
        motor=1,
        target=3500,
        min_angle=0,
        max_angle=4095,
        threshold=100,
        allow_motion=True,
    )

    assert result_low["contacted"] is True
    assert result_low["contact_position"] < result_default["contact_position"]


# ---------------------------------------------------------------------------
# No-contact path
# ---------------------------------------------------------------------------


def test_gentle_move_no_contact_reaches_clamped_target():
    """Load stays low (plain FakeBus always reports present_load=0) -> no contact."""
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

    assert result["contacted"] is False
    assert result["final_position"] == 2200
    assert result["contact_position"] is None
    assert result["contact_load"] is None
    assert result["retreat_position"] is None

    # Torque is still enabled (hold), never released.
    assert bus.torque_writes[-1] == {"motor": 1, "on": True}

    # The motor actually arrived at the (unclamped) target.
    assert bus.position_writes[-1] == {"motor": 1, "position": 2200}


def test_gentle_move_no_contact_moving_downward_reaches_target():
    bus = FakeBus(positions={1: 2200})
    bus.open()

    result = gentle_move(
        bus,
        motor=1,
        target=2000,
        min_angle=0,
        max_angle=4095,
        allow_motion=True,
    )

    assert result["contacted"] is False
    assert result["final_position"] == 2000
    assert bus.position_writes[-1] == {"motor": 1, "position": 2000}


# ---------------------------------------------------------------------------
# Bounds / clamping
# ---------------------------------------------------------------------------


def test_gentle_move_target_beyond_max_is_clamped():
    bus = FakeBus(positions={1: 2048})
    bus.open()

    result = gentle_move(
        bus,
        motor=1,
        target=9000,
        min_angle=0,
        max_angle=3000,
        allow_motion=True,
    )

    assert result["was_clamped"] is True
    assert result["clamped_target"] == 3000
    assert result["contacted"] is False
    assert result["final_position"] == 3000

    for write in bus.position_writes:
        assert 0 <= write["position"] <= 3000


def test_gentle_move_target_below_min_is_clamped():
    bus = FakeBus(positions={1: 2048})
    bus.open()

    result = gentle_move(
        bus,
        motor=1,
        target=-500,
        min_angle=1000,
        max_angle=4095,
        allow_motion=True,
    )

    assert result["was_clamped"] is True
    assert result["clamped_target"] == 1000
    assert result["final_position"] == 1000

    for write in bus.position_writes:
        assert 1000 <= write["position"] <= 4095


def test_gentle_move_steps_never_exceed_step_increment():
    """Consecutive position writes never jump by more than `step` ticks."""
    bus = FakeBus(positions={1: 0})
    bus.open()

    gentle_move(
        bus,
        motor=1,
        target=300,
        min_angle=0,
        max_angle=4095,
        step=25,
        allow_motion=True,
    )

    prev = 0
    for write in bus.position_writes:
        assert write["position"] - prev <= 25
        prev = write["position"]
    assert bus.position_writes[-1]["position"] == 300


# ---------------------------------------------------------------------------
# Compliant setup writes
# ---------------------------------------------------------------------------


def test_gentle_move_writes_acceleration_and_speed_once():
    bus = FakeBus(positions={1: 2048})
    bus.open()

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
