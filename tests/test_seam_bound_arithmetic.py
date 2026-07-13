"""The commandable bound IS the seam — pinned as arithmetic (issue #43).

The STS3215 carries a 12-bit magnetic encoder: 4096 raw ticks to a turn, and no
tick 4096. The servo does not report the raw count. It reports::

    Present = (Actual - Ofs) mod 4096

where ``Ofs`` is the EEPROM homing-offset register (addr 31,
:data:`~arm101.hardware.bus.ADDR_HOMING_OFFSET`). Two consequences follow, and
the whole of issue #43 is contained in them:

* The **seam** — the 4095->0 discontinuity in the REPORTED frame — is not a
  property of the shaft. It sits wherever ``Actual == Ofs``
  (:func:`~arm101.hardware.arm_spec.seam_tick`). Move the offset, move the seam.
* Therefore reported **0** is not "the bottom of the travel"; it is *the seam
  itself*. And reported **4095** — the bound every goal-position write in this
  codebase is validated against — is the tick immediately *below* the seam.

Five of the six joints on this arm still hold the factory
:data:`~arm101.hardware.arm_spec.FACTORY_ENCODER_OFFSET` (85, measured
2026-07-12). Under ``Ofs = 85`` the commandable bound, reported 4095, is raw
**84** — one tick below the seam at raw 85. The arm has been reporting the seam
as its boundary: the two ends of its "range" are not two walls, they are two
sides of one cut, physically adjacent on the shaft.

None of this is a hardware measurement. It is the arithmetic of a modular
encoding, so these tests pin the numbers directly — ``4096``, ``raw == Ofs``,
"one tick below" — and they hold for every offset the register can physically
hold, not just for the one this arm happens to ship with. Where a *measured*
constant is involved (the factory offset, the tick bounds), the expectation is
DERIVED from :mod:`~arm101.hardware.arm_spec` so that re-measuring it keeps the
suite green.
"""

from __future__ import annotations

import pytest

from arm101.cli._errors import EXIT_USER_ERROR, CliError
from arm101.hardware import bus as bus_module
from arm101.hardware.arm_spec import (
    ENCODER_TICKS,
    FACTORY_ENCODER_OFFSET,
    TICK_MAX,
    TICK_MIN,
    seam_tick,
)
from arm101.hardware.bus import OFFSET_MAX_MAGNITUDE, FakeBus
from arm101.hardware.rezero import raw_from_reported

#: Every offset the sign-magnitude offset register can physically hold
#: (``±OFFSET_MAX_MAGNITUDE``, see :func:`~arm101.hardware.bus.encode_offset`).
#: The properties below are asserted across ALL of them — an invariant that
#: held only for the offsets we happen to use would not be an invariant.
_EVERY_OFFSET = range(-OFFSET_MAX_MAGNITUDE, OFFSET_MAX_MAGNITUDE + 1)

#: A spread of offsets for the more expensive per-offset sweeps: zero (the state
#: no servo ships in), the factory offset, its negation, our re-zero's magnitude,
#: and both extremes of the register.
_SAMPLE_OFFSETS = [
    0,
    FACTORY_ENCODER_OFFSET,
    -FACTORY_ENCODER_OFFSET,
    1073,
    OFFSET_MAX_MAGNITUDE,
    -OFFSET_MAX_MAGNITUDE,
]


def _reported_from_raw(raw: int, offset: int) -> int:
    """The servo's own correction, ``Present = (Actual - Ofs) mod 4096``.

    The inverse of :func:`~arm101.hardware.rezero.raw_from_reported`, written
    out here rather than imported because it is the *thing under test* on the
    round-trip: if the two functions were the same code, the round-trip would
    prove nothing.
    """
    return (raw - offset) % ENCODER_TICKS


# ===========================================================================
# The encoding: a 4096-tick circle, agreed on by both modules that model it
# ===========================================================================


def test_the_encoder_is_a_4096_tick_circle():
    """12 bits, 4096 ticks, and the tick bounds are the two ends of exactly that.

    Not a measurement — the resolution of the magnetic encoder is fixed by the
    part. ``TICK_MIN``/``TICK_MAX`` are the inclusive bounds of the SAME circle,
    so a re-typed bound that no longer spans 4096 ticks would be an error in the
    table, not a re-measurement, and this catches it.
    """
    assert ENCODER_TICKS == 1 << 12 == 4096
    assert ENCODER_TICKS == TICK_MAX - TICK_MIN + 1
    assert (TICK_MIN, TICK_MAX) == (0, 4095)


def test_both_modules_that_model_the_encoder_agree_on_its_size():
    """``arm_spec.ENCODER_TICKS`` and ``bus.ENCODER_RESOLUTION`` are one fact, twice.

    ``arm_spec`` deliberately imports nothing from ``bus`` (and vice versa), so
    the 4096 is stated in both. Nothing but a test can keep them in step — and
    if they ever drift, every conversion in this file becomes a coin toss.
    """
    assert bus_module.ENCODER_RESOLUTION == ENCODER_TICKS


# ===========================================================================
# Issue #43, verbatim: reported 4095 under Ofs=85 is raw 84
# ===========================================================================


def test_issue_43_the_worked_example():
    """Reported 4095 under ``Ofs = 85`` is raw 84 — one tick below the seam at raw 85.

    The literals here are the issue's own arithmetic, not a reading off a servo:
    given ``Present = (Actual - Ofs) mod 4096``, ``Ofs = 85`` puts the seam at
    raw 85 and the top reported tick at raw ``(4095 + 85) mod 4096 == 84``. That
    remains true whatever this arm's servos are later measured to hold, which is
    why it is written out flat. The companion test below is the one that binds it
    to the arm.
    """
    ofs = 85

    # The seam — the reported 4095->0 cut — sits where the raw count equals Ofs.
    assert seam_tick(ofs) == 85

    # The commandable bound (reported 4095) is raw 84 …
    assert raw_from_reported(4095, ofs) == 84

    # … which is exactly one tick BELOW the seam. The software bound is not a
    # wall in the shaft; it is the last tick before the cut.
    assert raw_from_reported(4095, ofs) == (seam_tick(ofs) - 1) % ENCODER_TICKS

    # And the other end of the "range", reported 0, IS the seam.
    assert raw_from_reported(0, ofs) == seam_tick(ofs) == 85


def test_the_arm_we_have_is_that_worked_example():
    """The factory offset this arm ships with puts its bound on the seam.

    Derived from ``arm_spec`` rather than copying 85/84, so re-measuring the
    factory offset re-derives the expectation instead of breaking the test — but
    the *claim* (bound == seam − 1, and it is NOT at raw 0) still bites.
    """
    seam = seam_tick(FACTORY_ENCODER_OFFSET)
    bound_raw = raw_from_reported(TICK_MAX, FACTORY_ENCODER_OFFSET)

    # A servo at the factory offset is NOT in the "reported == raw" state. That
    # fiction is what made reported 4095 look like the top of the shaft.
    assert FACTORY_ENCODER_OFFSET != 0
    assert seam == FACTORY_ENCODER_OFFSET  # 0 <= 85 < 4096, so the mod is a no-op
    assert bound_raw == (seam - 1) % ENCODER_TICKS
    assert raw_from_reported(TICK_MIN, FACTORY_ENCODER_OFFSET) == seam


def test_the_commandable_bound_really_is_reported_tick_4095():
    """``TICK_MAX`` is not decoration: it is the goal-position bound the bus enforces.

    The finding above only matters because reported 4095 is genuinely the largest
    value any code path may command. Pinned against the bus's own validation, so
    "the commandable bound" is a fact about the code, not a claim in a docstring.
    """
    fake = FakeBus(positions={1: 2048})
    fake.open()

    fake.write_goal_position(1, TICK_MAX)  # accepted: the bound itself
    fake.write_goal_position(1, TICK_MIN)  # accepted: the other end

    with pytest.raises(CliError) as exc:
        fake.write_goal_position(1, TICK_MAX + 1)
    assert exc.value.code == EXIT_USER_ERROR


# ===========================================================================
# The general property — for EVERY offset the register can hold
# ===========================================================================


def test_the_seam_sits_at_raw_equals_offset_for_every_offset():
    """``seam_tick(Ofs) == Ofs mod 4096``, and reported 0 lands exactly on it.

    The seam is a property of the OFFSET, not of the joint. Anything that reasons
    about "where the encoder wraps" without knowing Ofs is guessing.
    """
    for offset in _EVERY_OFFSET:
        seam = seam_tick(offset)
        assert seam == offset % ENCODER_TICKS
        assert TICK_MIN <= seam <= TICK_MAX
        assert raw_from_reported(TICK_MIN, offset) == seam


def test_the_top_commandable_tick_is_always_one_tick_below_the_seam():
    """For EVERY offset, reported ``TICK_MAX`` is the raw tick just under the seam.

    The general form of the #43 finding: this is not a quirk of ``Ofs = 85``.
    There is no offset — none, including 0 — for which the top of the reported
    range is anything other than the last tick before the cut. A re-zero moves
    the seam somewhere harmless; it can never *remove* it from the bound.
    """
    for offset in _EVERY_OFFSET:
        assert raw_from_reported(TICK_MAX, offset) == (seam_tick(offset) - 1) % ENCODER_TICKS


def test_the_two_ends_of_the_commandable_range_are_physically_adjacent():
    """Reported 0 and reported 4095 are ONE raw tick apart on the shaft. Always.

    This is why a ``[min, max]`` pair cannot describe a joint whose travel spans
    the seam, and why the bound was never a wall: the "far end" of the range and
    the "near end" are neighbours on the magnet. The number 4095 tells you where
    the encoding cuts, and nothing whatsoever about where the joint stops.
    """
    for offset in _EVERY_OFFSET:
        low = raw_from_reported(TICK_MIN, offset)
        high = raw_from_reported(TICK_MAX, offset)
        assert (low - high) % ENCODER_TICKS == 1


# ===========================================================================
# The conversion is a bijection — nothing is lost, nothing is invented
# ===========================================================================


@pytest.mark.parametrize("offset", _SAMPLE_OFFSETS)
def test_reported_to_raw_is_a_bijection_mod_4096(offset):
    """The reported frame is the raw frame ROTATED — a relabelling, not a rescale.

    Every reported tick maps to a distinct raw tick and the image is the whole
    circle. So a re-zero can never widen or narrow the reachable set: it can only
    move where the discontinuity falls. (An implementation that clamped instead
    of wrapping — ``min(reported + offset, 4095)`` — would collapse two ticks onto
    one and fail here.)
    """
    image = [raw_from_reported(reported, offset) for reported in range(ENCODER_TICKS)]

    assert len(set(image)) == ENCODER_TICKS
    assert set(image) == set(range(ENCODER_TICKS))


@pytest.mark.parametrize("offset", _SAMPLE_OFFSETS)
def test_raw_and_reported_round_trip_through_the_servos_own_correction(offset):
    """``raw_from_reported`` inverts ``Present = (Actual - Ofs) mod 4096`` exactly.

    Both directions, over the whole circle — the guarantee every "and then the
    servo will report…" sentence in :mod:`~arm101.hardware.rezero` rests on.
    """
    for reported in range(ENCODER_TICKS):
        assert _reported_from_raw(raw_from_reported(reported, offset), offset) == reported

    for raw in range(ENCODER_TICKS):
        assert raw_from_reported(_reported_from_raw(raw, offset), offset) == raw
