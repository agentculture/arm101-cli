"""arm101.hardware.ticks — the ONE boundary between the servo's two tick frames.

Every encoder tick in this codebase is in one of exactly two frames, and mixing
them is the single bug class that has now shipped twice (the ``(126, 2020)``
re-zero arc measured in the wrong frame, 2026-07-12; and the soft limit this
module's arrival finally corrects). So the frames get a module, the conversion
gets one implementation, and every call site that crosses between them names the
frame it is in.

The two frames
--------------
The STS3215 has a 12-bit magnetic encoder: 4096 ticks, one full turn, and it
carries a signed *homing offset* register (``Ofs``, EEPROM addr 31) that shifts
what it reports::

    reported = (raw - Ofs) mod 4096
    raw      = (reported + Ofs) mod 4096

* **RAW** — the magnet on the shaft. A raw tick IS a physical angle: the same
  raw tick always means the same place, on this joint, forever. Writing the
  offset register changes nothing about it — the shaft does not move when you
  write EEPROM. The joint's mechanical walls, its
  :class:`~arm101.hardware.arm_spec.UnreachableArc`, and its
  :class:`~arm101.hardware.arm_spec.SoftLimit` are all claims about *physical
  angles*, so they are all RAW.
* **REPORTED** — what comes back over the wire from
  :meth:`~arm101.hardware.bus.MotorBus.read_position` and what a goal write is
  interpreted in. It is a **view through the current offset**, and it is only
  equal to raw when the register holds 0 — which no servo ships doing (the
  factory default is
  :data:`~arm101.hardware.arm_spec.FACTORY_ENCODER_OFFSET` = 85, measured
  uniform across all six follower joints on 2026-07-12).

Why RAW is the persistence frame — the rule this module exists to enforce
------------------------------------------------------------------------
**Persist RAW. Treat REPORTED as a display/wire view, converted at the bus
edge.**

A reported tick is only meaningful *alongside the offset it was read through*.
Store one on its own — in a table, a calibration profile, a reachability map —
and the first re-zero silently changes what it means: the number is unchanged,
the physical angle it names has moved by the offset delta, and nothing anywhere
fails. That is the whole failure mode. Raw ticks survive a re-zero untouched, so
a raw tick stored today still names the same physical angle after any number of
future re-zeros; the *view* of it changes, and the view is recomputed on demand
by :func:`reported_from_raw` with the offset in force at that moment.

The seam — and there are TWO of them
------------------------------------
A "seam" is where a tick axis wraps, i.e. where two adjacent physical angles get
tick values 4095 apart. Every piece of code that treats ticks as a linear axis
(``gentle_move``'s arrival check, ``clamp_goal``, a ``[min, max]`` range) is
wrong across a seam. There is one seam per frame, and they are *different
places*:

* the **raw seam**, :data:`RAW_SEAM_TICK` — where the magnet's own count rolls
  ``4095 -> 0``. Fixed, immovable, a property of the encoder. Any code comparing
  RAW ticks must stay clear of it.
* the **reported seam** — where ``(raw - Ofs) mod 4096`` rolls ``4095 -> 0``,
  which happens exactly where ``raw == Ofs``: :func:`seam_tick`. It MOVES when
  the offset is written, and moving it is precisely what an ``arm rezero`` does.
  Any code comparing REPORTED ticks — which is every mover in this codebase,
  because a goal write must be in the servo's own frame — must stay clear of
  *this* one.

So "does the dead arc contain the seam?" is not one question but two, and a
range that clears one can sit right on the other. That is why
:func:`raw_interval_to_reported` refuses to convert an interval that straddles
the reported seam instead of returning a plausible-looking, wrong pair: an
interval that wraps in the reported frame has NO honest ``(min, max)``
representation there, and inventing one is exactly how a "clamp" ends up
commanding a joint the long way round.

No imports, on purpose
----------------------
This module is the bottom of the hardware dependency stack: it imports nothing,
so :mod:`arm101.hardware.arm_spec` (which may not import the bus — see
``test_arm_spec_module_never_imports_the_bus``) can depend on it, and so can the
bus itself. Frame arithmetic must be reachable from both sides of the boundary
it describes.
"""

from __future__ import annotations

# ---------------------------------------------------------------------------
# The encoder, as a circle
# ---------------------------------------------------------------------------

#: Lowest / highest tick the 12-bit encoder can report, in either frame.
TICK_MIN: int = 0
TICK_MAX: int = 4095

#: One full turn, in ticks (4096). Derived from the bounds rather than typed, so
#: the three cannot drift apart. It is the modulus of both conversions below —
#: the encoder is a *circle*, and every tick expression in this codebase folds
#: back onto it.
#:
#: Deliberately re-stated here rather than imported from
#: :data:`arm101.hardware.bus.ENCODER_RESOLUTION`: this module imports nothing so
#: that ``arm_spec`` (which is forbidden from importing the bus) can use it. The
#: two constants are pinned equal by a cross-module test.
ENCODER_TICKS: int = TICK_MAX - TICK_MIN + 1

#: Widest magnitude the offset register (``Ofs``/``Homing_Offset``, EEPROM addr
#: 31) can hold: it is SIGN-MAGNITUDE on bit 11, so the magnitude field is 11
#: bits and the representable range is ``[-2047, +2047]``. (LeRobot
#: ``encode_sign_magnitude``: ``max_magnitude = (1 << 11) - 1``; confirmed the
#: hard way on a real SO-101 — LeRobot issue #3193 raised ``ValueError: Magnitude
#: 2073 exceeds 2047``.)
#:
#: Modulo 4096 that covers **every** seam placement except exactly one: raw 2048.
#: (``-2047`` is congruent to ``2049``, so residues ``0..2047`` and ``2049..4095``
#: are all reachable; neither ``+2048`` nor ``-2048`` is representable.) See
#: :func:`offset_for_seam_at`.
#:
#: Mirrors :data:`arm101.hardware.bus.OFFSET_MAX_MAGNITUDE` — same fact, stated in
#: the module that must not import the other, and pinned equal by a cross-module
#: test.
MAX_ENCODER_OFFSET: int = 2047

#: The RAW tick at which the RAW axis itself wraps: 0, i.e. the ``4095 -> 0``
#: rollover of the magnet's own count.
#:
#: Unlike the reported seam (:func:`seam_tick`) this one does not move, ever. No
#: offset can relocate it, because the offset does not touch the encoder — it
#: only changes the arithmetic applied on the way out. It is a constant rather
#: than a bare ``0`` at the call sites so that "keep clear of the raw seam" reads
#: as the physical claim it is, and so a reader can tell the two seams apart
#: without inferring which frame the surrounding code is in.
RAW_SEAM_TICK: int = TICK_MIN


# ---------------------------------------------------------------------------
# The conversion — the only bridge between the frames
# ---------------------------------------------------------------------------


def raw_from_reported(reported: int, offset: int) -> int:
    """Recover the shaft's RAW encoder count from what the servo REPORTS.

    ``raw = (reported + Ofs) mod 4096`` — the inverse of the correction the servo
    applies (``reported = raw - Ofs``, confirmed on hardware 2026-07-12 by a
    reversible probe: writing ``Ofs 85 -> 185`` dropped the reported position by
    exactly 100, and ``85 -> 0`` raised it by exactly 85).

    **The one bridge between the two frames**, and it is on every path, not just
    the exotic ones. It is tempting to think of it as a special case for an
    already-re-zeroed servo — that is what an earlier version of the re-zero code
    thought, treating the factory state as "offset 0, so reported IS raw". The
    factory state is ``Ofs = 85``
    (:data:`~arm101.hardware.arm_spec.FACTORY_ENCODER_OFFSET`), so that identity
    never held, on any servo, ever; the conversion has to happen every time.

    The ``mod 4096`` is load-bearing, not defensive. ``reported + offset``
    genuinely runs past 4096 for positions near the top of the travel (a joint
    reporting 4000 on a servo holding ``+200`` is physically at raw 104, not 4200
    — there is no tick 4200), and it genuinely runs below 0 for a negative
    offset. Both fold back onto the circle, because the encoder is a circle.
    """
    return (reported + offset) % ENCODER_TICKS


def reported_from_raw(raw: int, offset: int) -> int:
    """Render a RAW tick as the servo holding *offset* would REPORT it.

    ``reported = (raw - Ofs) mod 4096`` — the servo's own correction, and the
    exact inverse of :func:`raw_from_reported`.

    This is the direction that makes RAW storage *usable*: a stored raw tick is
    the durable fact, and this is the view of it that a goal write, an arrival
    check, or a human-facing display needs **at this moment, through this
    servo's current offset**. Recomputed on demand rather than stored, because
    the moment it is stored it stops being true (see the module docstring).
    """
    return (raw - offset) % ENCODER_TICKS


def seam_tick(offset: int) -> int:
    """The RAW tick at which a servo holding *offset* carries its REPORTED seam.

    The inverse of :func:`offset_for_seam_at`, and the single piece of arithmetic
    the whole re-zero turns on. With ``reported = (raw - Ofs) mod 4096`` the
    reported value rolls ``4095 -> 0`` exactly where ``raw == Ofs``, so the seam's
    raw tick simply **is** the offset — reduced modulo 4096, because the register
    is SIGNED (sign-magnitude on bit 11, range ``[-2047, +2047]``) while the
    encoder is not.

    That reduction is not a formality. A servo holding ``-1096`` carries its seam
    at raw **3000**, not at "-1096": those are the same residue and only one of
    them is a tick. Comparing the signed number straight against a raw arc would
    place the seam a whole turn away from where it physically is, and every "is
    the seam evicted?" answer downstream would be wrong.
    """
    return offset % ENCODER_TICKS


def offset_for_seam_at(tick: int) -> int:
    """Return the SIGNED offset ``H`` that places the REPORTED seam at raw *tick*.

    With ``reported = (raw - H) mod 4096``, the reported value rolls ``4095 -> 0``
    exactly where ``raw == H``, so the offset simply *is* the seam's raw tick —
    but expressed in the signed form the register can hold. Ticks above
    :data:`MAX_ENCODER_OFFSET` are unrepresentable as positive magnitudes and are
    re-expressed as their negative congruent (``tick - 4096``): raw 3000 becomes
    ``H = -1096``, which is the same residue and fits comfortably.

    Raw 2048 is the single seam placement the encoding cannot express at all
    (``+2048`` overflows the 11-bit magnitude and ``-2048`` does too). It is not
    silently rounded — ``arm_spec._require_evictable_seam`` rejects a table entry
    whose midpoint lands there, loudly, at import time.
    """
    return tick if tick <= MAX_ENCODER_OFFSET else tick - ENCODER_TICKS


# ---------------------------------------------------------------------------
# Intervals — where the frames stop being interchangeable
# ---------------------------------------------------------------------------


def crosses_reported_seam(low: int, high: int, offset: int) -> bool:
    """``True`` iff the RAW interval ``[low, high]`` WRAPS in the reported frame.

    A raw interval is a plain, well-ordered ``low <= high`` pair — it cannot cross
    the raw seam by construction. Its *reported* image is a different question
    entirely: it wraps iff the reported seam (:func:`seam_tick`, at raw ``==
    offset``) lies inside the interval, because that is exactly the raw tick where
    the reported value rolls ``4095 -> 0``.

    An interval for which this is ``True`` has **no** honest ``(min, max)``
    representation in the reported frame: the set of reported ticks it covers is
    the union of two arcs at opposite ends of the scale. Sorting them into a pair
    yields precisely the *complement* — the region the joint must never go — which
    is the same shape of bug that made ``elbow_flex``'s ``[min, max]`` describe
    the arc it cannot reach.

    The seam's own tick is deliberately treated as *inside* the interval when it
    equals ``high`` and *outside* when it equals ``low``: reported rolls over
    between ``low - 1`` and ``low``, so an interval starting exactly at the seam
    starts cleanly at reported 0 and stays monotone, while one ending exactly at
    the seam has already wrapped by its last tick.
    """
    return low < seam_tick(offset) <= high


def raw_interval_to_reported(low: int, high: int, offset: int) -> "tuple[int, int]":
    """Convert the RAW interval ``[low, high]`` into the frame a servo reports in.

    The bus-edge conversion for *ranges* — what turns a stored RAW soft limit into
    the ``(min, max)`` pair a goal-writing mover can actually clamp against.

    Returns ``(reported_from_raw(low, offset), reported_from_raw(high, offset))``,
    which is monotone (``low_reported < high_reported``) and covers exactly the
    same ``high - low + 1`` physical angles as the input.

    Raises
    ------
    ValueError
        If ``low > high`` (not an interval), if either endpoint is outside
        ``[TICK_MIN, TICK_MAX]``, or if the interval
        :func:`crosses_reported_seam` — in which case its reported image is two
        disjoint arcs and returning any single pair would be a lie. This is the
        error a caller *wants*: it means the range and the servo's offset
        contradict each other (the dead arc no longer contains the reported
        seam), and the fix is to correct one of them, not to clamp through it.
    """
    if not (TICK_MIN <= low <= high <= TICK_MAX):
        raise ValueError(
            f"({low}, {high}) is not a valid raw tick interval: requires "
            f"{TICK_MIN} <= low <= high <= {TICK_MAX}."
        )
    if crosses_reported_seam(low, high, offset):
        raise ValueError(
            f"The raw interval ({low}, {high}) straddles the reported seam of a servo "
            f"holding offset {offset} (its seam is at raw tick {seam_tick(offset)}), so it "
            "has no (min, max) representation in the reported frame — its reported image is "
            "two arcs at opposite ends of the scale. Either the offset is wrong for this "
            "joint or the interval's dead arc no longer contains the seam."
        )
    return (reported_from_raw(low, offset), reported_from_raw(high, offset))
