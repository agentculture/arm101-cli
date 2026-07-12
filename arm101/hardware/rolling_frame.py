"""The ROLLING FRAME — keep the seam ahead of the joint, and travel stops being bounded.

The chicken-and-egg this module breaks
======================================
The STS3215 reports ``reported = (raw - Ofs) mod 4096``, so the **reported seam** —
the 4095->0 discontinuity — sits at ``raw == Ofs`` (:func:`~arm101.hardware.ticks.seam_tick`).
A goal write is *linear* in the reported frame: the servo subtracts ``goal - present``
and drives the shaft in the direction of that error. It therefore **cannot be
commanded across its own seam**. Ask it to, and it goes the long way round, or it
runs out of scale and simply stops.

At the factory offset (:data:`~arm101.hardware.arm_spec.FACTORY_ENCODER_OFFSET` = 85)
the seam sits at raw 85 — which is **inside** several joints' physical travel. Those
joints hit the *commandable* bound while still physically free, and their real travel
has never been seen (issue #43). And measuring the arc a joint cannot reach is what
would tell you where to put the seam:

    you cannot measure a joint's unreachable arc until you can see past the seam,
    and you cannot evict the seam until you know the arc.

The escape: centre the joint, then ROLL
=======================================
Before probing, write a **temporary** offset that maps the joint's *current* raw
position to reported :data:`CENTRE_TICK` (2048). The seam is then **half a turn away
— the farthest it can possibly be** — and the joint has ~2048 ticks of clear,
commandable frame in each direction.

Then, whenever the creep runs low on frame, **RE-CENTRE**: write a fresh temporary
offset that re-centres the joint *where it now is*. The seam is thereby rolled ahead
of the joint and never obstructs it. Travel becomes **unbounded by the frame** — a
joint can be creeped through a full turn, or several, in a scale that only holds 4096
ticks, because the scale keeps sliding along underneath it.

Nothing here needs to know the joint's arc. That is the point: the rolling frame is
what *measures* the arc, so it cannot presuppose one.

The trap: residue 2048 is not representable
===========================================
Putting raw ``r`` at reported ``C`` requires ``Ofs ≡ r - C (mod 4096)``. The offset
register is **sign-magnitude on bit 11**, range ``[-2047, +2047]``
(:data:`~arm101.hardware.ticks.MAX_ENCODER_OFFSET`), so residues ``0..2047`` and
``2049..4095`` are all reachable and **2048 is not** — neither ``+2048`` (overflows
the 11-bit magnitude) nor ``-2048`` (same) fits.

With ``C = 2048`` the required residue is 2048 exactly when ``r == 0``. So the
"centre at the half-turn" rule is impossible for **exactly one raw position out of
4096**, and a naive implementation either crashes there or — far worse — has
``write_offset`` reject the value while the probe carries on believing it is in a
frame it is not in, and every tick it records afterwards is a lie.

:func:`centring_offset` handles it deliberately: raw 0 is centred at
:data:`FALLBACK_CENTRE_TICK` (2047) instead. **This costs nothing.** A frame centred
at 2048 clears 2047 ticks upward and 2048 downward; one centred at 2047 clears 2048
upward and 2047 downward. The *worst* direction is 2047 ticks either way — the
fallback is the mirror image of the nominal centre, not a degraded version of it.
(2049 would have been the degraded one: 2046 in its worst direction. Hence 2047.)
``test_exactly_one_raw_position_cannot_be_centred_at_the_half_turn`` enumerates all
4096 raw ticks rather than trusting that reasoning.

The rules a mover must obey
===========================
* **Re-centre BETWEEN moves, never during one.** A ``gentle_move`` whose reported
  frame shifted mid-flight would check its arrival against a target that no longer
  means what it did when it was computed. So a creep is a sequence of short moves,
  each asking :meth:`RollingFrame.goal` for a target guaranteed to be inside the
  current frame.
* **Sync often enough.** :meth:`RollingFrame.sync` folds movement into
  :attr:`~RollingFrame.displacement` by taking the SHORT way round the circle
  (:func:`~arm101.hardware.limits.signed_delta`). That is exact only while the joint
  moves less than half a turn between syncs — which a step-wise creep always does,
  and a 3000-tick unattended move would not.
* **The frame never energises anything.** Every offset write goes through
  ``bus.write_offset``, whose first act is ``enable_torque(motor, False)`` — a joint
  must not be *holding* while its frame of reference moves under it. The frame leaves
  it limp and lets the mover turn torque back on.

Why the standing goal is rewritten after every shift
====================================================
``Goal_Position`` is a **reported** tick. Move the frame under it and the very same
number names a *different physical angle* — and the servo notices the instant torque
comes back, and bolts for it. So after every offset write the frame reads where the
joint is and writes **that** as the standing goal, while torque is still off. The next
mover to energise the joint finds it already holding station.

The transaction
===============
Every offset write goes through :func:`~arm101.hardware.journal.shift_offset`, which
makes it durable on disk **before** it goes on the wire. A SIGKILL, a power cut or a
yanked cable mid-roll therefore leaves an arm whose original calibration is still
named somewhere — and :func:`~arm101.hardware.journal.require_clean` puts it back at
the start of the next run. The frame is a *transaction*, and it has two endings:
:meth:`RollingFrame.restore` (the default — put it back exactly as it was found) and
:meth:`RollingFrame.commit` (keep the rolled calibration deliberately).

Zero third-party imports, and no arm table: this module knows about *an* encoder and
*a* servo, not about which joint is which.
"""

from __future__ import annotations

import contextlib
from typing import TYPE_CHECKING, Tuple

from arm101.cli._errors import EXIT_ENV_ERROR, EXIT_USER_ERROR, CliError
from arm101.hardware.journal import (
    DISPOSITION_COMMITTED,
    DISPOSITION_RESTORED,
    CalibrationJournal,
    shift_offset,
)
from arm101.hardware.limits import signed_delta
from arm101.hardware.ticks import (
    ENCODER_TICKS,
    MAX_ENCODER_OFFSET,
    TICK_MAX,
    TICK_MIN,
    raw_from_reported,
    reported_from_raw,
    seam_tick,
)

if TYPE_CHECKING:  # pragma: no cover - typing only; never imported at runtime
    from arm101.hardware.bus import MotorBus

__all__ = [
    "CENTRE_TICK",
    "DEFAULT_RECENTRE_MARGIN",
    "FALLBACK_CENTRE_TICK",
    "MAX_HEADROOM",
    "RollingFrame",
    "centring_offset",
    "headroom_at",
]


# ---------------------------------------------------------------------------
# Where a centred joint sits, and how much room that buys it
# ---------------------------------------------------------------------------

#: The reported tick a joint is centred at: the half-turn, 2048. Derived, not typed —
#: it is *the* place that maximises the distance to the seam, and it is the half-turn
#: because the seam is exactly antipodal to it.
CENTRE_TICK: int = ENCODER_TICKS // 2

#: The centre used for the one raw position (0) whose nominal centring offset the
#: register cannot express (see the module docstring). One tick short of
#: :data:`CENTRE_TICK`, which is the *mirror image* of it — same worst-direction
#: headroom, :data:`MAX_HEADROOM`. Not a degradation; a reflection.
FALLBACK_CENTRE_TICK: int = CENTRE_TICK - 1

#: The clear commandable travel a centred frame promises **in its worst direction**.
#: Both centres above give exactly this (2047), which is why the fallback is free.
#: No :meth:`RollingFrame.goal` may ask for a step larger than this: no frame, however
#: placed, could deliver it.
MAX_HEADROOM: int = min(TICK_MAX - CENTRE_TICK, CENTRE_TICK - TICK_MIN)

#: Ticks of clear frame the joint must have *beyond* the move it is about to make,
#: before the frame will decline to re-centre.
#:
#: The number is a claim about the mover, and it is sized against the worst
#: ``gentle_move`` can do in one call: a 25-tick goal step, the servo's own overshoot
#: past that goal, and the 50-tick back-off it performs on contact. 256 clears all of
#: them with an order of magnitude to spare — and, just as importantly, it keeps the
#: joint from ever *settling* within a hair of the reported bound, where a few ticks
#: of overshoot would wrap the report from 4095 to 0 and hand the mover's arrival
#: check a 4095-tick error to chase.
#:
#: The cost of being generous is EEPROM writes, which are finite. At 256 the frame
#: re-centres every ~1792 ticks of travel instead of every ~2048 — about 14% more
#: writes, i.e. roughly two or three per full turn of joint travel. That is the whole
#: price. ``test_the_margin_leaves_room_for_everything_gentle_move_can_do_in_one_go``
#: pins both halves of the trade against ``gentle``'s own constants.
DEFAULT_RECENTRE_MARGIN: int = 256

#: The one residue the sign-magnitude offset register cannot hold: 2048. Both
#: ``+2048`` and ``-2048`` overflow its 11-bit magnitude field. Named, because the
#: single special case in this module exists solely because of it.
_UNREPRESENTABLE_RESIDUE: int = MAX_ENCODER_OFFSET + 1


def headroom_at(reported: int, direction: int) -> int:
    """Clear commandable ticks between *reported* and the end of the scale, going *direction*.

    The reported scale is ``[0, 4095]`` and a goal write cannot name a tick outside it
    — so this is, quite literally, how much further the joint can be *told* to go
    before the frame runs out. It says nothing about whether the joint can physically
    get there; that is what the probe is for.
    """
    return TICK_MAX - reported if direction > 0 else reported - TICK_MIN


def centring_offset(raw: int) -> Tuple[int, int]:
    """The signed offset that puts a joint sitting at *raw* in the middle of its scale.

    Returns ``(offset, centre)``: the value to write to the servo's ``Ofs`` register
    (EEPROM addr 31), and the reported tick the joint will then report — which is
    :data:`CENTRE_TICK` for 4095 of the 4096 raw positions, and
    :data:`FALLBACK_CENTRE_TICK` for the one where the register cannot express the
    nominal answer (``raw == 0``; see the module docstring for why, and why the
    fallback costs nothing).

    The resulting frame always has these three properties, and
    ``tests/test_rolling_frame.py`` enumerates all 4096 raw ticks rather than
    trusting the algebra:

    * the joint reports *centre*;
    * the seam (``raw == offset``) is exactly :data:`HALF_TURN
      <arm101.hardware.limits.HALF_TURN>` ticks away — as far as anything on a circle
      can be;
    * at least :data:`MAX_HEADROOM` ticks of clear frame lie in **each** direction.

    Raises
    ------
    ValueError
        If *raw* is not a raw encoder tick. Nothing else can be centred, and guessing
        what the caller meant is how a frame ends up describing a joint that is not
        where it says it is.
    """
    raw = int(raw)
    if not (TICK_MIN <= raw <= TICK_MAX):
        raise ValueError(f"raw {raw} is not a raw encoder tick — it must lie in [0, {TICK_MAX}].")

    centre = CENTRE_TICK
    residue = (raw - centre) % ENCODER_TICKS
    if residue == _UNREPRESENTABLE_RESIDUE:
        # THE trap, and it fires for exactly one raw tick (0). The register is
        # sign-magnitude on bit 11, so residue 2048 is the sole value it cannot hold.
        # Centring one tick short instead is a deliberate, and free, retreat: it is
        # the mirror image of the nominal centre, with identical worst-direction
        # headroom. What it must NOT be is silent — a write the servo rejects while
        # the caller believes it landed poisons every tick measured afterwards.
        centre = FALLBACK_CENTRE_TICK
        residue = (raw - centre) % ENCODER_TICKS

    # Re-express the residue in the SIGNED form the register holds: residues above
    # 2047 are unreachable as positive magnitudes and go on the wire as their negative
    # congruent (2049 -> -2047), which is the same seam and fits comfortably.
    offset = residue if residue <= MAX_ENCODER_OFFSET else residue - ENCODER_TICKS
    return offset, centre


# ---------------------------------------------------------------------------
# The frame itself
# ---------------------------------------------------------------------------


class RollingFrame:
    """A joint's temporary encoder frame, re-centred as often as the creep needs.

    Open it, creep, close it::

        with RollingFrame(bus, journal, joint="elbow_flex", motor=3) as frame:
            while probing:
                target = frame.goal(direction, step)   # re-centres if the frame is short
                gentle_move(bus, frame.motor, target, min_angle=0, max_angle=4095, ...)
                frame.sync()                           # fold the travel in
            observation = EndObservation(
                joint=frame.joint,
                end=TravelEnd.HIGH,
                verdict=verdict,
                origin_raw=frame.origin_raw,           # a RAW tick: the anchor
                displacement=frame.displacement,       # signed RAW ticks from it
            )

    :attr:`origin_raw` and :attr:`displacement` are exactly the pair
    :class:`~arm101.hardware.limits.EndObservation` is built from, and for the same
    reason: raw ticks cannot be compared across a seam (raw 4000 + 200 reads as raw
    104, which looks like a 3896-tick *retreat*), so the honest unit is a signed
    displacement from a named origin. This class is where that displacement is
    accumulated — one small, unambiguous delta at a time, invariant under every offset
    rewrite it performs.

    **Entering the context does real work**: it reads the servo, journals its original
    calibration, writes a temporary offset to EEPROM, and leaves the joint limp with a
    standing goal at its own position. Exiting **restores** the original calibration
    (unless :meth:`commit` was called), and does so even when an exception is on its
    way out — without ever masking it.

    Parameters
    ----------
    bus:
        An open :class:`~arm101.hardware.bus.MotorBus`.
    journal:
        The :class:`~arm101.hardware.journal.CalibrationJournal` this frame's writes
        are recorded in. The caller must already have run
        :func:`~arm101.hardware.journal.require_clean` against it — the frame checks,
        and refuses a motor whose calibration is still in flight.
    joint:
        The joint's name. Carried for the journal and for the error messages a human
        has to act on; nothing here branches on it.
    motor:
        The Feetech servo id.
    margin:
        Ticks of clear frame required *beyond* the move about to be made, before the
        frame will decline to re-centre. Defaults to :data:`DEFAULT_RECENTRE_MARGIN`.
    """

    def __init__(
        self,
        bus: "MotorBus",
        journal: CalibrationJournal,
        *,
        joint: str,
        motor: int,
        margin: int = DEFAULT_RECENTRE_MARGIN,
    ) -> None:
        if not isinstance(joint, str) or not joint:
            raise ValueError("joint must be a non-empty name.")
        margin = int(margin)
        if not (0 <= margin <= MAX_HEADROOM):
            raise ValueError(
                f"margin {margin} must lie in [0, {MAX_HEADROOM}] — a frame cannot reserve "
                "more headroom than it has."
            )

        self._bus = bus
        self._journal = journal
        self._joint = joint
        self._motor = int(motor)
        self._margin = margin

        self._original_offset: int = 0
        self._offset: int = 0
        self._centre: int = CENTRE_TICK
        self._raw: int = 0
        self._origin_raw: int = 0
        self._displacement: int = 0
        self._recentres: int = 0

        # Set the moment a shift is ATTEMPTED — not when it succeeds. A write that
        # raised may still have landed, and a frame that would not try to put that
        # back would be trusting the very bus that just failed it.
        self._in_transaction: bool = False
        self._opened: bool = False
        self._closed: bool = False

    # -- what the frame knows ------------------------------------------------

    @property
    def joint(self) -> str:
        return self._joint

    @property
    def motor(self) -> int:
        return self._motor

    @property
    def margin(self) -> int:
        return self._margin

    @property
    def original_offset(self) -> int:
        """The offset the servo held before this frame touched it. Restored on exit."""
        return self._original_offset

    @property
    def offset(self) -> int:
        """The temporary offset in force right now. Changes on every re-centre."""
        return self._offset

    @property
    def centre(self) -> int:
        """The reported tick the last centring put the joint at (2048, or 2047 at raw 0)."""
        return self._centre

    @property
    def seam_raw(self) -> int:
        """Where the reported seam currently sits, as a RAW tick. Half a turn from the joint."""
        return seam_tick(self._offset)

    @property
    def raw(self) -> int:
        """The joint's RAW encoder tick, as of the last :meth:`sync`. Frame-independent."""
        return self._raw

    @property
    def reported(self) -> int:
        """What the servo reports for the joint right now, through the offset in force."""
        return reported_from_raw(self._raw, self._offset)

    @property
    def origin_raw(self) -> int:
        """The RAW tick the frame was anchored at — where the probe started.

        Read **after** the opening EEPROM write, not before: the joint is limp for that
        write and may sag a tick or two, and sag that happened before the probe began
        is not travel the probe performed.
        """
        return self._origin_raw

    @property
    def displacement(self) -> int:
        """Signed RAW ticks travelled from :attr:`origin_raw`. **Not** bounded by the frame.

        Positive means the joint went up the raw scale, negative down. It passes 4096
        without flinching (a continuous joint really can turn twice), which is exactly
        why it is a displacement and not a tick.

        Note :class:`~arm101.hardware.limits.EndObservation` refuses a displacement
        larger than one full turn — past that there is nothing left to learn — so a
        probe should stop when :attr:`full_turn` goes true.
        """
        return self._displacement

    @property
    def travelled(self) -> int:
        """:attr:`displacement`'s magnitude — how far the joint got, direction aside."""
        return abs(self._displacement)

    @property
    def full_turn(self) -> bool:
        """The joint has gone all the way round. There is nothing further to learn."""
        return self.travelled >= ENCODER_TICKS

    @property
    def recentres(self) -> int:
        """How many times the frame has rolled. Each one is an EEPROM write."""
        return self._recentres

    @property
    def closed(self) -> bool:
        """The transaction is over — restored, or committed."""
        return self._closed

    def headroom(self, direction: int) -> int:
        """Clear commandable ticks left in *direction*, as of the last :meth:`sync`."""
        return headroom_at(self.reported, self._require_direction(direction))

    # -- opening -------------------------------------------------------------

    def open(self) -> "RollingFrame":
        """Centre the joint: read where it is, journal its calibration, roll the seam away.

        Afterwards the joint reports :attr:`centre`, the seam is half a turn away, and
        the servo is **limp** (``write_offset`` disables torque, and this does not turn
        it back on) with a standing goal at its own position — so the first mover to
        energise it finds it holding station rather than bolting for a goal written in
        a frame that no longer exists.

        Raises
        ------
        CliError(EXIT_USER_ERROR)
            If the frame is already open.
        CliError(EXIT_ENV_ERROR)
            If this motor already has an unresolved calibration transaction — run
            :func:`~arm101.hardware.journal.require_clean` first. Stacking a fresh
            temporary offset on top of one nobody restored is how the *original*
            offset stops being knowable.
        CliError
            Whatever the bus raises. The journal is durable by then, and the frame
            rolls its own shift back before the error leaves this method.
        """
        if self._opened:
            raise CliError(
                code=EXIT_USER_ERROR,
                message=(
                    f"The rolling frame for {self._joint} (motor {self._motor}) is already open."
                ),
                remediation="Open a frame once; call restore() or commit() to close it.",
            )
        self._require_no_transaction_in_flight()

        # Read the frame; never assume it. A factory servo holds 85, not 0.
        original = self._bus.read_offset(self._motor)
        reported = self._bus.read_position(self._motor)

        self._original_offset = original
        self._offset = original
        self._raw = raw_from_reported(reported, original)

        try:
            self._roll_to_centre()
        except BaseException:
            # The write may have landed even though the call raised. Put it back HERE,
            # in this process, rather than leaving the rest of the run reading ticks in
            # a frame nobody chose — the journal is the backstop for a crash, not an
            # excuse to strand a caught error.
            with contextlib.suppress(Exception):
                self.restore()
            raise

        # Anchor AFTER the write: the joint was limp for it and may have sagged, and
        # sag that happened before the probe began is not travel the probe performed.
        self._origin_raw = self._raw
        self._displacement = 0
        self._opened = True
        return self

    def __enter__(self) -> "RollingFrame":
        return self.open()

    def __exit__(self, exc_type, exc, traceback) -> bool:
        if self._closed:
            return False
        if exc_type is None:
            self.restore()
            return False
        # An exception is already on its way out. Put the arm back — but a failure to
        # do so must NEVER replace the reason the caller is unwinding. The journal
        # entry stays dirty, and the next run's require_clean retries it.
        #
        # suppress(Exception), never BaseException: a KeyboardInterrupt raised by the
        # operator during the restore is the operator asking to stop.
        with contextlib.suppress(Exception):
            self.restore()
        return False

    # -- reading, and rolling ------------------------------------------------

    def sync(self) -> int:
        """Read the joint and fold whatever it has done since the last read into the record.

        Returns its RAW tick. The movement is accumulated into :attr:`displacement` as
        a :func:`~arm101.hardware.limits.signed_delta` — the SHORT way round the circle
        — so a joint that rolled from raw 4090 to raw 10 is recorded as having moved
        **+16 ticks**, not -4080. That is exact while the joint moves less than half a
        turn between syncs, which a step-wise creep always does.
        """
        self._require_open()
        reported = self._bus.read_position(self._motor)
        self._absorb(raw_from_reported(reported, self._offset))
        return self._raw

    def ensure_headroom(self, direction: int, ticks: "int | None" = None) -> bool:
        """Guarantee *ticks* of clear commandable frame in *direction*. Roll if there is not.

        Returns ``True`` if it had to re-centre (i.e. an EEPROM write happened),
        ``False`` if the frame already had the room.

        Raises
        ------
        CliError(EXIT_USER_ERROR)
            If *ticks* exceeds :data:`MAX_HEADROOM` — no frame, however placed, could
            promise that, so re-centring would be a lie rather than a fix.
        CliError(EXIT_ENV_ERROR)
            If even a freshly centred frame does not have the room (the joint drifted
            while it was limp).
        """
        want = self._margin if ticks is None else int(ticks)
        return self._ensure(direction, want=want, need=want)

    def goal(self, direction: int, step: int) -> int:
        """The REPORTED tick *step* ticks further out in *direction* — guaranteed commandable.

        The mover's entry point, and the only one it needs. It syncs, re-centres if the
        frame can no longer promise ``step + margin`` ticks of clear travel, and hands
        back a target the servo's goal register can actually hold.

        Because a re-centre happens **here**, between moves, and never inside one, the
        mover always runs in a frame that stands still for the whole of its move.

        Raises
        ------
        CliError(EXIT_USER_ERROR)
            If *step* is not in ``[1, MAX_HEADROOM]``, or *direction* is not a
            direction.
        CliError(EXIT_ENV_ERROR)
            If even a freshly centred frame cannot fit *step*.
        """
        direction = self._require_direction(direction)
        step = int(step)
        if not (1 <= step <= MAX_HEADROOM):
            raise CliError(
                code=EXIT_USER_ERROR,
                message=(
                    f"A creep step of {step} ticks is not commandable: it must lie in "
                    f"[1, {MAX_HEADROOM}]. A centred frame clears {MAX_HEADROOM} ticks in its "
                    "worst direction, so no re-centring could make a larger step fit."
                ),
                remediation=(
                    f"Creep in steps of at most {MAX_HEADROOM} ticks — a probe should be using "
                    "far smaller ones anyway, so that contact is detected as it happens rather "
                    "than after the joint has driven into something."
                ),
            )

        # Ask for step + margin; insist only on step. The margin is the comfort buffer
        # that keeps the joint away from the bound, not a hard requirement — a joint
        # that sagged a tick while limp should not fail a move it can plainly make.
        self._ensure(direction, want=min(step + self._margin, MAX_HEADROOM), need=step)
        return self.reported + direction * step

    def recentre(self) -> None:
        """Roll the seam: write a fresh temporary offset that re-centres the joint HERE.

        Journalled before it is written (:func:`~arm101.hardware.journal.shift_offset`),
        so a crash between the two leaves an arm that can be put back. Afterwards the
        joint reports :attr:`centre` again, the seam is half a turn away again, and the
        joint has its full clear frame back in both directions — while not having moved
        a single tick. **An EEPROM write is not a motion command.**

        Torque is off when this returns (``write_offset`` disables it and this does not
        re-enable it), and the standing goal names the joint's own position, so nothing
        lurches when the next mover energises it.
        """
        self._require_open()
        self.sync()
        self._roll_to_centre()
        self._recentres += 1

    # -- closing -------------------------------------------------------------

    def restore(self) -> None:
        """Put the servo back in the calibration it was found in, and close the transaction.

        The default ending. Idempotent: calling it twice does nothing the second time.

        A latched overload is cleared first — a probe that found a wall is exactly the
        probe whose servo is latched, and ``write_offset``'s opening
        ``enable_torque(motor, False)`` is the very packet a latched servo answers with
        the overload bit still set. Best-effort: if the bus is too far gone to accept
        even that, the offset write below is still worth trying, and *its* failure is
        what gets reported.

        The journal entry is closed **only** once the servo is provably holding the
        original offset again. A write the bus accepted but the servo is not holding is
        not a restore, and closing the entry on one would destroy the only record of
        the truth while the joint is still mis-calibrated.

        Raises
        ------
        CliError(EXIT_ENV_ERROR)
            If the original offset could not be put back and verified. The journal
            entry stays dirty on purpose, so the next run's
            :func:`~arm101.hardware.journal.require_clean` tries again.
        """
        if self._closed or not self._in_transaction:
            self._closed = True
            return

        # contextlib.suppress, NOT try/except/pass — bandit B110 fails CI on the latter.
        with contextlib.suppress(Exception):
            self._bus.clear_overload(self._motor)

        shift_offset(
            self._bus,
            self._journal,
            joint=self._joint,
            motor=self._motor,
            offset=self._original_offset,
        )
        read_back = self._bus.read_offset(self._motor)
        if read_back != self._original_offset:
            raise CliError(
                code=EXIT_ENV_ERROR,
                message=(
                    f"{self._joint} (motor {self._motor}) was left holding encoder offset "
                    f"{read_back}: the restore wrote {self._original_offset} (EEPROM addr 31) "
                    "and the servo did not take it. Every tick this joint reports is now in a "
                    "frame nobody chose."
                ),
                remediation=(
                    f"The journal at {self._journal.path} still names the original offset "
                    f"({self._original_offset}) — it is the only record of it, so do not delete "
                    "it. Check the bus is healthy ('arm101 arm read'), power-cycle the servo if "
                    "it is latched, and re-run: the restore is retried automatically at startup."
                ),
            )
        self._offset = self._original_offset

        # The standing goal is a REPORTED tick and the frame just moved under it, so it
        # names a different angle than it did a moment ago. Re-point it at the joint's
        # own position before anything can energise the joint against it. Best-effort:
        # the calibration is what the journal protects, and it is provably back — a
        # failure here must not hold the transaction open (which would spend a
        # redundant EEPROM write on the next run) for a joint that is already correct.
        with contextlib.suppress(Exception):
            self._hold_in_place()

        self._journal.end(motor=self._motor, disposition=DISPOSITION_RESTORED)
        self._closed = True

    def commit(self) -> None:
        """Close the transaction and KEEP the offset now in force. Writes nothing.

        The deliberate other ending: the caller has decided the rolled frame *is* this
        joint's calibration from here on (a re-zero it asked for and has verified). All
        this does is stop the next run from helpfully undoing it.

        Note what is being kept: whatever offset the **last re-centre** happened to
        write, which centres the joint wherever it *finished*. That is a perfectly good
        calibration — any offset whose seam lands outside the joint's travel is — but it
        is not a considered one, so a caller that cares where the seam ends up should
        :meth:`recentre` from a pose it has chosen, then commit.
        """
        self._require_open()
        self._journal.end(motor=self._motor, disposition=DISPOSITION_COMMITTED)
        self._closed = True

    # -- internals -----------------------------------------------------------

    def _roll_to_centre(self) -> None:
        """Shift the offset so the joint reports :data:`CENTRE_TICK`, then settle in that frame."""
        offset, centre = centring_offset(self._raw)

        # Set BEFORE the write, not after: a write that raises may still have landed,
        # and restore() must know there is something to put back.
        self._in_transaction = True
        shift_offset(
            self._bus,
            self._journal,
            joint=self._joint,
            motor=self._motor,
            offset=offset,
        )
        self._offset = offset
        self._centre = centre

        # Re-read in the NEW frame. The joint was limp for the write and may have
        # sagged; that is real movement of a real shaft, and it is folded into the
        # displacement like any other — reading it back is the only honest way to know.
        reported = self._bus.read_position(self._motor)
        self._absorb(raw_from_reported(reported, self._offset))
        self._hold_in_place()

    def _hold_in_place(self) -> None:
        """Point the servo's standing goal at the joint's own position, in the current frame.

        Torque is off when this runs. ``Goal_Position`` is a REPORTED tick, so the frame
        change just moved it: the number is unchanged and the physical angle it names is
        not. Left alone, the servo drives to it the instant torque comes back.
        """
        self._bus.write_goal_position(self._motor, self.reported)

    def _absorb(self, raw: int) -> None:
        """Fold a fresh RAW reading into the displacement, the short way round the circle."""
        self._displacement += signed_delta(raw, self._raw)
        self._raw = raw

    def _ensure(self, direction: int, *, want: int, need: int) -> bool:
        """Re-centre if the frame is short of *want*; fail only if it is short of *need*."""
        direction = self._require_direction(direction)
        if not (0 <= want <= MAX_HEADROOM):
            raise CliError(
                code=EXIT_USER_ERROR,
                message=(
                    f"Cannot reserve {want} ticks of headroom: a centred frame clears at most "
                    f"{MAX_HEADROOM} in its worst direction, so no offset could deliver it."
                ),
                remediation=f"Ask for at most {MAX_HEADROOM} ticks.",
            )

        self.sync()
        if self.headroom(direction) >= want:
            return False

        self.recentre()
        if self.headroom(direction) < need:
            raise CliError(
                code=EXIT_ENV_ERROR,
                message=(
                    f"{self._joint} (motor {self._motor}) has only {self.headroom(direction)} "
                    f"ticks of commandable frame left going {'up' if direction > 0 else 'down'}, "
                    f"even after re-centring — {need} were needed. The joint must have moved "
                    "between the re-centre and the check."
                ),
                remediation=(
                    "Check the joint is not being driven by something else (gravity on an "
                    "un-energised arm will do it), and creep in smaller steps."
                ),
            )
        return True

    def _require_open(self) -> None:
        if not self._opened:
            raise CliError(
                code=EXIT_USER_ERROR,
                message=(
                    f"The rolling frame for {self._joint} (motor {self._motor}) is not open: the "
                    "joint has not been centred, so no tick it reports means what this frame "
                    "would say it means."
                ),
                remediation="Call open(), or use the frame as a context manager.",
            )
        if self._closed:
            raise CliError(
                code=EXIT_USER_ERROR,
                message=(
                    f"The rolling frame for {self._joint} (motor {self._motor}) is closed — its "
                    "temporary calibration has already been restored or committed."
                ),
                remediation="Open a fresh frame.",
            )

    def _require_no_transaction_in_flight(self) -> None:
        entry = self._journal.dirty_entry_for(self._motor)
        if entry is None:
            return
        raise CliError(
            code=EXIT_ENV_ERROR,
            message=(
                f"Motor {self._motor} ({entry.joint}) already has an unresolved calibration "
                f"transaction: it may be holding a temporary encoder offset, and its ORIGINAL "
                f"({entry.original_offset}) is recorded only in the journal. Opening a fresh "
                "frame on top of it would overwrite the only record of where this joint's zero "
                "used to be."
            ),
            remediation=(
                "Run journal.require_clean(bus, journal) first — it restores a calibration a "
                "crashed run left behind, and it is the guard every motion verb is supposed to "
                f"call before it touches the arm. The journal is at {self._journal.path}."
            ),
        )

    @staticmethod
    def _require_direction(direction: int) -> int:
        """Normalise *direction* to ``+1`` or ``-1``. Zero is not a direction."""
        direction = int(direction)
        if direction == 0:
            raise CliError(
                code=EXIT_USER_ERROR,
                message="A creep direction must be +1 (up the tick scale) or -1 (down), not 0.",
                remediation="Pass +1 for the HIGH end of the travel, -1 for the LOW end.",
            )
        return 1 if direction > 0 else -1
