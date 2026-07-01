"""Regression tests for the STS3215 present_load DIRECTION bit (bit 10 / 0x400).

Present_Load (register 60) encodes load *direction* in bit 10 and magnitude in
bits 0-9. `gentle_move`'s contact check must threshold on the MAGNITUDE — before
the fix it compared the raw register value, so a load pointing the "negative"
direction (raw >= 1024) tripped a spurious contact on the very first step. This
was caught on the physical follower during the t9/t7 hardware run, not by the
existing FakeBus tests (whose `present_load` never set the direction bit).
"""

from __future__ import annotations

from arm101.hardware.bus import FakeBus, load_magnitude
from arm101.hardware.gentle import gentle_move

_DIR_BIT = 0x400  # bit 10 — load direction sign


# ---------------------------------------------------------------------------
# load_magnitude() helper
# ---------------------------------------------------------------------------


def test_load_magnitude_strips_direction_bit() -> None:
    # 1064 == 0x428 == direction bit (1024) + magnitude 40
    assert load_magnitude(1064) == 40
    assert load_magnitude(1048) == 24
    assert load_magnitude(_DIR_BIT) == 0  # pure direction bit, zero magnitude


def test_load_magnitude_passthrough_when_direction_clear() -> None:
    assert load_magnitude(0) == 0
    assert load_magnitude(40) == 40
    assert load_magnitude(1023) == 1023  # max magnitude, direction clear


# ---------------------------------------------------------------------------
# Test doubles: a fixed present_load, with the direction bit set
# ---------------------------------------------------------------------------


class DirectionLoadBus(FakeBus):
    """FakeBus that reports a fixed `present_load` (direction bit set) on read.

    Mirrors real hardware: a joint loaded in the negative direction reports
    `0x400 | magnitude`. `load` is the desired MAGNITUDE; the direction bit is
    OR-ed in so the raw register value is `0x400 | load`.
    """

    def __init__(self, *args, load: int = 0, **kwargs):
        super().__init__(*args, **kwargs)
        self._raw_load = _DIR_BIT | (load & 0x3FF)

    def write_goal_position(self, motor: int, position: int) -> None:
        super().write_goal_position(motor, position)
        self._positions[motor] = position

    def read_info(self, motor: int) -> dict:
        info = super().read_info(motor)
        info["present_load"] = self._raw_load
        return info


def test_direction_bit_low_magnitude_does_not_false_contact() -> None:
    # magnitude 40 (raw 0x428 == 1064) is well below the default threshold 250;
    # before the fix the raw 1064 > 250 tripped an immediate spurious contact.
    bus = DirectionLoadBus(positions={1: 2048}, load=40)
    bus.open()
    result = gentle_move(bus, 1, 2200, min_angle=0, max_angle=4095, allow_motion=True)
    assert result["contacted"] is False
    assert result["overloaded"] is False
    assert result["final_position"] == 2200  # reached target, no spurious stop


def test_direction_bit_high_magnitude_still_contacts() -> None:
    # magnitude 400 (raw 0x400|400) exceeds threshold 250 -> real contact,
    # regardless of the direction bit.
    bus = DirectionLoadBus(positions={1: 2048}, load=400)
    bus.open()
    result = gentle_move(bus, 1, 2200, min_angle=0, max_angle=4095, allow_motion=True)
    assert result["contacted"] is True
    # contact_load is reported as the MAGNITUDE (direction bit masked off),
    # not the raw register value.
    assert result["contact_load"] == 400
    assert result["contact_load"] < _DIR_BIT
