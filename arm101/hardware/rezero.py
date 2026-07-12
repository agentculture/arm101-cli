"""Encoder re-zero — the one EEPROM write that makes ``elbow_flex``'s tick axis linear.

The problem (issue #35)
-----------------------
``elbow_flex``'s 12-bit encoder **wraps inside its own physical travel**. Driven
far enough it crosses the raw 4095->0 seam and reads back near zero, so its
reported position is *not monotonic with joint angle*: two different angles
report similar ticks, the joint's two measured endpoints sort into a ``[min,
max]`` pair that describes exactly the arc it CANNOT reach, and every position
comparison in this codebase — ``gentle_move``'s arrival check, ``clamp_goal``,
the reachability map's ranges — is silently wrong for it. It currently rests at
raw ~126, i.e. **past** its wrap.

The fix is to shift the encoder's zero (``Ofs``/``Homing_Offset``, EEPROM addr
31) so the seam falls inside the arc the joint physically cannot reach
(:class:`~arm101.hardware.arm_spec.UnreachableArc`). Then every tick the joint
can actually reach lies on one side of the seam, and the linear-axis assumption
the whole codebase already makes becomes TRUE rather than merely assumed.

The bootstrap problem — why nothing here commands motion
--------------------------------------------------------
**The tool that makes the axis linear cannot itself rely on the axis being
linear.** That single sentence dictates the shape of this module.

The obvious procedure — "drive the joint to mid-travel, then write the offset
that centres it" — is exactly the thing that must not happen. ``elbow_flex``
rests at raw ~126, on the far side of its wrap. A goal of, say, 3121 (its
mid-travel) looks like a modest move in tick-space and is in fact a rotation
the *long way round*: the servo would drive from 126 down through 0, across the
whole 1894-tick arc it cannot reach, and into a wall. The commanded number is
sane; the physical consequence is not; and the discrepancy is precisely the
non-linearity this write exists to remove. So:

* :func:`apply_rezero` **writes no goal position, ever** — the wire surface is
  torque-off, unlock, addr 31, re-lock, and nothing else. It reads where the
  joint physically is, computes the offset from the joint's *known unreachable
  arc* (a table fact, not a live measurement), and writes it.
* :func:`sweep` also commands nothing: the joint is de-energised and a **human
  hand** moves it. The one instrument that can prove the seam moved is a human
  arm, because it is the only actuator in the building that does not need a
  linear tick axis to work.

Torque is off for the write (``bus.write_offset`` disables it first) and stays
off — a joint must not be *holding* when its own frame of reference changes
underneath it.

The unproven assumption — and how ``--verify`` settles it
---------------------------------------------------------
Everything above rests on one undocumented bit of firmware semantics
(``docs/spikes/sts3215-offset-register.md`` §4)::

    Present = (raw - Ofs) mod 4096     seam RELOCATES  -> the fix works
    Present =  raw - Ofs   (signed)    seam STAYS      -> the fix does NOTHING

Under the second reading the offset merely *relabels* positions: the
discontinuity stays pinned to the physical angle where the magnet rolls over,
and the re-zero achieves nothing at all. Every source and LeRobot's shipped
SO-101 calibration imply the first, but no primary Feetech source states the
formula — so :class:`~arm101.hardware.bus.FakeBus` models BOTH
(``offset_wraps=True`` / ``False``) and this module is tested against both.

**Reading the offset back only proves it was APPLIED. It does not prove the
seam MOVED.** Only a sweep does — which is why :func:`sweep` exists and why it
is not optional garnish on the write. A sweep that finds a discontinuity is a
STOP condition: the re-zero did not solve issue #35, and the plan has to go back
to the user for a re-decision.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, Optional, Sequence

from arm101.cli._errors import EXIT_ENV_ERROR, EXIT_USER_ERROR, CliError
from arm101.hardware import arm_spec
from arm101.hardware.bus import FakeBus

if TYPE_CHECKING:  # pragma: no cover - typing only
    from arm101.hardware.bus import MotorBus


# ---------------------------------------------------------------------------
# Sweep tuning — every number here is a claim about the hardware
# ---------------------------------------------------------------------------

#: A single-sample position jump at or above this many ticks is a
#: **discontinuity**: the seam, crossed. It is not sensor noise and it is not a
#: fast hand.
#:
#: Sized against the three ways the seam can show up in a sweep, and against the
#: one thing it must never be confused with:
#:
#: * Seam still in the travel (no offset, or an offset that did nothing): the
#:   report jumps 4095 -> 0, a delta of ~4095.
#: * Offset written, firmware does a plain signed subtraction, and
#:   :meth:`~arm101.hardware.bus.FeetechBus.read_position`'s ``& 0x0FFF`` folds
#:   the negative result back into range: the jump is ~1949 ticks for
#:   ``H = 1073``. **This is the smallest discontinuity we can be shown**, and it
#:   is why the threshold is not the tempting 2048.
#: * The same, unmasked (what :class:`~arm101.hardware.bus.FakeBus` reports with
#:   ``offset_wraps=False``): the position goes NEGATIVE, a delta of ~4095, and
#:   :attr:`SweepReport.out_of_range` catches it independently anyway.
#:
#: Against a human hand: the whole 2202-tick travel moved in a brisk 2 s, polled
#: every 50 ms, is ~55 ticks per sample. Even a yank is well under 500. The gap
#: between "fastest plausible hand" and "smallest possible seam crossing" is
#: nearly 4x, and 500 sits in the middle of it.
DISCONTINUITY_TICKS: int = 500

#: A direction reversal smaller than this is the operator's hand, not a signal.
#: Encoder read jitter is a few ticks; a human holding a joint still wobbles.
#: Matches ``gentle_move``'s arrival tolerance, which was sized against the same
#: noise floor. Used ONLY for the *descriptive* :attr:`SweepReport.monotonic`
#: flag — never for the verdict, which turns on continuity alone.
REVERSAL_TOLERANCE: int = 12

#: How much of the joint's expected travel the sweep must actually cover before
#: "no discontinuity" is allowed to mean anything. A sweep that moved the joint
#: 200 ticks and saw no seam has proved **nothing** — of course it saw no seam;
#: it never went near where the seam would be. Claiming a pass from that would
#: be the single most damaging thing this verb could do, because it would close
#: the open question with a lie. Hence :attr:`SweepReport.conclusive`.
MIN_COVERAGE: float = 0.8

#: Default wall-clock length of a ``--verify`` sweep, in seconds. Long enough
#: for a human to move one joint, hand-over-hand, from one hard stop to the
#: other without hurrying — hurrying is how you skip past the seam between two
#: samples.
DEFAULT_SWEEP_DURATION: float = 30.0

#: Seconds between position polls during a sweep. 50 ms gives ~600 samples over
#: the default duration and ~55 ticks/sample at a brisk hand speed — dense
#: enough that a seam crossing cannot hide between two reads.
DEFAULT_SWEEP_INTERVAL: float = 0.05

#: Ticks of slack allowed between the position predicted after a re-zero and the
#: one actually observed. The joint is limp while this is measured, so gravity,
#: backlash and a nudged cable all move it a little between the pre-write read
#: and the post-write read. Generous on purpose: this check exists to catch an
#: offset that did nothing (delta ~1073) or went the wrong way, not to police
#: encoder jitter.
POSITION_TOLERANCE: int = 30

#: Verdicts a :class:`SweepReport` can return. Deliberately four, not two —
#: "did not fail" is not the same claim as "proved the fix works", and a verb
#: that conflated them would report a pass for a sweep of a joint that was never
#: re-zeroed, or one the human barely moved.
VERDICT_SEAM_EVICTED = "seam-evicted"
VERDICT_SEAM_NOT_EVICTED = "seam-not-evicted"
VERDICT_SEAM_PRESENT_BASELINE = "seam-present-baseline"
VERDICT_INCONCLUSIVE = "inconclusive"


# ---------------------------------------------------------------------------
# Eligibility — answered with no hardware attached
# ---------------------------------------------------------------------------


def require_rezeroable(joint: str) -> "tuple[int, arm_spec.UnreachableArc]":
    """Return *joint*'s ``(offset, arc)``, or raise explaining why there isn't one.

    Called FIRST by the CLI verb — before consent, before a port is resolved,
    before a bus is opened — so that ``arm rezero wrist_roll`` answers the
    question on a laptop with no arm plugged in. "Why can't I re-zero this
    joint?" is a question about the arm's geometry, not about the servo in front
    of you, and it deserves an answer that does not depend on one being there.

    Returning both halves together is deliberate: every caller that wants the
    offset also wants the arc it was derived from (to check the joint is not
    somewhere it cannot be, to size the sweep, to render the plan), and handing
    them back as a pair means no caller has to re-look-up an
    ``Optional[UnreachableArc]`` it has just proved is not ``None``.

    Parameters
    ----------
    joint:
        One of the six joint names in :data:`arm101.hardware.arm_spec.JOINTS`.

    Returns
    -------
    tuple[int, UnreachableArc]
        The signed encoder offset that evicts *joint*'s seam (``+1073`` for
        ``elbow_flex``, the only re-zeroable joint on this arm), and the
        unreachable arc it was derived from.

    Raises
    ------
    CliError(EXIT_USER_ERROR)
        If *joint* is not a joint name at all, or is a joint that cannot (or
        need not) be re-zeroed. The message is
        :func:`~arm101.hardware.arm_spec.rezero_refusal`'s — which distinguishes
        "impossible" (``wrist_roll``: no unreachable arc exists, so no offset can
        evict its seam; a soft limit handles it) from "unnecessary" (the other
        four: their encoders do not wrap inside their travel at all).
    """
    try:
        offset = arm_spec.rezero_offset(joint)
        arc = arm_spec.rezero_arc(joint)
    except ValueError as exc:
        raise CliError(
            code=EXIT_USER_ERROR,
            message=str(exc),
            remediation=f"Valid joints: {', '.join(arm_spec.JOINTS)}.",
        ) from exc

    if offset is None or arc is None:
        refusal = arm_spec.rezero_refusal(joint)
        raise CliError(
            code=EXIT_USER_ERROR,
            message=f"{joint} is not re-zeroable.\n\n{refusal}",
            remediation=(
                "Only elbow_flex wraps inside its travel, and only elbow_flex can be "
                "re-zeroed: run 'arm101 arm rezero elbow_flex'. Inspect any joint's live "
                "encoder offset (read-only) with 'arm101 arm read --json'."
            ),
        )
    return offset, arc


# ---------------------------------------------------------------------------
# The plan — read where the joint IS, decide nothing about where it should GO
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class RezeroPlan:
    """Everything the write will do, derived from live reads and the arc table.

    Built by :func:`plan_rezero` from **reads only**. It is the answer to "what
    is about to happen to this servo", and it is computed before anything
    happens to it.

    Attributes
    ----------
    joint, motor:
        Which joint, and the servo id carrying it.
    current_offset:
        The signed offset the servo holds RIGHT NOW (0 on a factory servo).
    target_offset:
        The signed offset about to be written — derived from the joint's
        unreachable arc, never typed.
    reported_position:
        What the servo reports today, i.e. already corrected by
        *current_offset*.
    raw_position:
        Where the shaft physically is, in the encoder's own frame:
        ``(reported + current_offset) mod 4096``. On a factory servo
        (*current_offset* == 0) this is an identity and carries no assumption
        at all — which is the case the whole procedure is designed around.
    predicted_position:
        What the servo will report once the write lands, IF the corrected
        position is reduced modulo 4096: ``(raw - target) mod 4096``. This is
        the first, cheapest test of the open question — see
        :func:`describe_shift`.
    already_applied:
        The servo already holds *target_offset*. The write is a no-op; say so
        rather than performing it again.
    """

    joint: str
    motor: int
    current_offset: int
    target_offset: int
    reported_position: int
    raw_position: int
    predicted_position: int
    already_applied: bool

    def as_dict(self) -> dict[str, object]:
        """JSON-serialisable form, for a verb's ``--json`` payload."""
        return {
            "joint": self.joint,
            "motor": self.motor,
            "current_offset": self.current_offset,
            "target_offset": self.target_offset,
            "reported_position": self.reported_position,
            "raw_position": self.raw_position,
            "predicted_position": self.predicted_position,
            "already_applied": self.already_applied,
        }


def raw_from_reported(reported: int, offset: int) -> int:
    """Recover the shaft's raw encoder count from what the servo REPORTS.

    ``Actual = (Present + Ofs) mod 4096`` — the inverse of the correction the
    servo applies. With the factory offset of 0 this is the identity, and the
    only case that matters for a first re-zero needs no inverse at all; the
    function exists so that a servo which has ALREADY been re-zeroed can still
    be located in the raw frame the arc table is written in (e.g. to re-run the
    reachability checks, or to detect that the joint is somewhere it should not
    physically be able to be).
    """
    return (reported + offset) % arm_spec.ENCODER_TICKS


def plan_rezero(bus: "MotorBus", motor: int, joint: str) -> RezeroPlan:
    """Read the joint's live state and work out exactly what the re-zero will write.

    **Reads only.** No torque write, no goal, no EEPROM. Two guards make this
    more than a formatting exercise, and both refuse rather than guess:

    *An unknown frame.* If the servo already holds an offset that is neither the
    factory ``0`` nor the target, we do not know what frame its reported
    positions are in, so we cannot honestly convert them to raw ticks and cannot
    honestly check anything below. Writing a new offset on top of an unknown one
    would bury the problem instead of surfacing it.

    *A physically impossible position.* If the joint's raw position lands
    strictly inside the arc it supposedly cannot reach, then either the arc is
    wrong or this servo is not the joint we think it is. Either way the offset
    about to be written is derived from a table that does not describe the
    hardware in front of us, and writing it would put the seam somewhere the
    joint CAN go — making issue #35 worse, not better, and doing it persistently
    in EEPROM.

    Raises
    ------
    CliError(EXIT_USER_ERROR)
        If *joint* is not re-zeroable (see :func:`require_rezeroable`).
    CliError(EXIT_ENV_ERROR)
        If the servo holds an unrecognised offset, or reports a raw position
        inside its own unreachable arc.
    """
    target, arc = require_rezeroable(joint)

    current = bus.read_offset(motor)
    reported = bus.read_position(motor)

    if current not in (0, target):
        raise CliError(
            code=EXIT_ENV_ERROR,
            message=(
                f"{joint} (motor {motor}) already holds an encoder offset of {current}, "
                f"which is neither the factory 0 nor this joint's computed {target}. "
                "Its reported positions are in a frame this tool did not set and cannot "
                "interpret, so it will not write a new offset on top of it."
            ),
            remediation=(
                "Inspect the live offset with 'arm101 arm read --json' (the 'offset' "
                "column). A re-zero must start from a known frame: restore the servo's "
                "offset register (EEPROM addr 31) to 0 — or to this joint's computed "
                f"{target} — and re-run. If {current} was deliberate, the arc table in "
                "arm101/hardware/arm_spec.py is what needs updating, not the servo."
            ),
        )

    raw = raw_from_reported(reported, current)

    if arc.contains(raw):
        raise CliError(
            code=EXIT_ENV_ERROR,
            message=(
                f"{joint} (motor {motor}) reports raw encoder position {raw}, which is "
                f"INSIDE the arc it is supposed to be physically unable to reach "
                f"({arc.low}, {arc.high}). The joint cannot be where it says it is, so the "
                "arc — and the offset derived from it — does not describe this hardware. "
                "Refusing to write."
            ),
            remediation=(
                "Check that motor "
                f"{motor} really is {joint} ('arm101 arm read'), and that the arm is "
                "assembled as the arc was measured on. If the joint's travel has genuinely "
                "changed, re-measure its walls and correct REZERO_ARCS in "
                "arm101/hardware/arm_spec.py — the offset is derived from that table, so "
                "correcting the table corrects the offset."
            ),
        )

    return RezeroPlan(
        joint=joint,
        motor=motor,
        current_offset=current,
        target_offset=target,
        reported_position=reported,
        raw_position=raw,
        predicted_position=(raw - target) % arm_spec.ENCODER_TICKS,
        already_applied=current == target,
    )


# ---------------------------------------------------------------------------
# The write — an EEPROM write, and NOT a move
# ---------------------------------------------------------------------------


def apply_rezero(bus: "MotorBus", motor: int, offset: int) -> int:
    """Write *offset* to *motor*'s EEPROM and read it back. Commands NO motion.

    The whole write, in two lines, and both of them matter:

    1. :meth:`~arm101.hardware.bus.MotorBus.clear_overload` — disables torque,
       tolerating (and clearing) a latched overload. Not optional and not
       merely defensive: ``write_offset``'s own first act is a plain
       ``enable_torque(motor, False)``, which a servo latched in overload
       answers with the overload bit still set — so the write would raise
       :class:`~arm101.hardware.bus.OverloadError` before it ever opened the
       EEPROM. A joint that has just been driven into a wall (which is how
       ``elbow_flex``'s arc was measured in the first place) is exactly the
       joint you are then asked to re-zero.
    2. :meth:`~arm101.hardware.bus.MotorBus.write_offset` — torque-off, unlock
       (addr 55 -> 0), addr 31, re-lock. That primitive owns the Lock dance
       (without it the write reads back fine and silently REVERTS on the next
       power-cycle — PR #21), the sign-magnitude encoding, and the range check.

    What is NOT here is the point: **no ``write_goal_position``, at any stage.**
    The joint is not driven anywhere before, during, or after. See the module
    docstring — a linear command issued while the axis is still non-linear
    rotates ``elbow_flex`` the long way round, through its whole travel, into a
    wall.

    Returns
    -------
    int
        The offset read back from EEPROM. The caller MUST check it equals
        *offset*: the read-back is what proves the write landed. (It does not
        prove the write PERSISTS — only a power-cycle does — and it does not
        prove the seam MOVED — only :func:`sweep` does.)

    Raises
    ------
    CliError(EXIT_USER_ERROR)
        If *offset* is outside the register's ``[-2047, +2047]`` — raised by
        ``encode_offset`` before any wire traffic, so the joint is not even
        de-energised by a rejected call.
    CliError(EXIT_ENV_ERROR)
        If the bus is not open or any write fails.
    """
    bus.clear_overload(motor)
    bus.write_offset(motor, offset)
    return bus.read_offset(motor)


def describe_shift(plan: RezeroPlan, observed: int) -> dict[str, object]:
    """Compare the position observed after the write against the one predicted.

    The cheapest possible probe of the open question, taken for free the moment
    the write lands — and worth taking, because two of its outcomes are already
    damning without waiting for a sweep:

    * ``observed`` equals the pre-write reading: the offset changed nothing
      about what the servo reports. The register took the value (the read-back
      says so) and the firmware ignored it.
    * ``observed`` is outside ``[0, 4095]``: the corrected position is a plain
      signed subtraction — the seam is pinned where it always was, and the
      re-zero cannot work.

    It is a probe, not a proof. A reading consistent with the prediction shows
    the offset is *applied*, in the *modular* sense, at *one* point. That the
    seam actually MOVED — that the joint's whole travel is now continuous — is a
    statement about every point in the travel, and only :func:`sweep` can make
    it.

    Returns a dict for the verb's payload: the predicted and observed positions,
    their difference, whether they agree within :data:`POSITION_TOLERANCE`,
    whether the reading is even in range, and whether the report moved at all.
    """
    delta = observed - plan.predicted_position
    return {
        "predicted_position": plan.predicted_position,
        "observed_position": observed,
        "delta": delta,
        "as_predicted": abs(delta) <= POSITION_TOLERANCE,
        "in_range": arm_spec.TICK_MIN <= observed <= arm_spec.TICK_MAX,
        "unchanged": abs(observed - plan.reported_position) <= POSITION_TOLERANCE,
    }


# ---------------------------------------------------------------------------
# The sweep — the only thing that can prove the seam MOVED
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class SweepReport:
    """What a torque-off, hand-driven sweep of one joint actually measured.

    Reports what it saw, then draws a conclusion — and keeps the two apart. The
    measurements (:attr:`samples`, :attr:`minimum`, :attr:`maximum`,
    :attr:`largest_jump`) are facts. The :attr:`verdict` is an interpretation,
    and it is allowed to be "I cannot tell" — see :attr:`conclusive`.

    Attributes
    ----------
    joint, motor:
        What was swept.
    offset_in_force, expected_offset:
        The offset the servo actually held during the sweep, and the one that
        evicts this joint's seam. When they differ the sweep is a **baseline**,
        not a proof — a perfectly useful thing to run (it SHOWS you the seam
        before you fix it), but not the same claim.
    samples:
        Every position read, in order. The measurement record; every number
        below is derived from it.
    minimum, maximum, span:
        The extent the joint was actually moved through. ``span`` is what
        :attr:`conclusive` is judged on — and, incidentally, the first
        measurement anyone has ever made of ``elbow_flex``'s far wall.
    monotonic:
        The reported position never both rose and fell by more than
        :data:`REVERSAL_TOLERANCE`. **Descriptive, not decisive**: a human hand
        that backs up mid-sweep makes this ``False`` without anything being
        wrong. The verdict turns on :attr:`continuous`, which a hand cannot fake
        either way.
    largest_jump, largest_jump_at:
        The biggest single-sample change, and the sample index it happened at.
        A seam crossing is ~1949-4095 ticks; sensor noise and a human hand are
        tens of ticks. The gap is not subtle.
    discontinuities:
        ``(index, before, after)`` for every jump at or above
        :data:`DISCONTINUITY_TICKS`. Non-empty means the seam is still inside
        the joint's travel.
    out_of_range:
        Any sample outside ``[0, 4095]``. A position register cannot hold such
        a value — seeing one means the corrected position is an unwrapped signed
        subtraction, which independently proves the re-zero cannot work.
    expected_travel:
        The joint's travel in ticks, from its unreachable arc — the yardstick
        :attr:`conclusive` measures :attr:`span` against.
    """

    joint: str
    motor: int
    offset_in_force: int
    expected_offset: int
    samples: tuple[int, ...]
    minimum: int
    maximum: int
    monotonic: bool
    largest_jump: int
    largest_jump_at: int
    discontinuities: tuple[tuple[int, int, int], ...]
    out_of_range: tuple[int, ...]
    expected_travel: int

    @property
    def span(self) -> int:
        """Ticks between the extremes reached — how far the joint was actually moved."""
        return self.maximum - self.minimum

    @property
    def rezeroed(self) -> bool:
        """The joint was carrying the seam-evicting offset while it was swept."""
        return self.offset_in_force == self.expected_offset

    @property
    def continuous(self) -> bool:
        """No discontinuity and no impossible reading — the seam was never crossed."""
        return not self.discontinuities and not self.out_of_range

    @property
    def conclusive(self) -> bool:
        """The sweep covered enough travel for its answer to mean anything.

        A sweep that barely moved the joint and saw no seam has proved nothing —
        of course it saw no seam, it never went near where the seam would be.
        Requires the span to reach :data:`MIN_COVERAGE` of the joint's expected
        travel. A DISCONTINUOUS sweep is always conclusive, whatever its span:
        seeing the seam is proof it is there, and no amount of extra travel can
        un-see it.
        """
        if not self.continuous:
            return True
        return self.span >= self.expected_travel * MIN_COVERAGE

    @property
    def seam_evicted(self) -> bool:
        """The one claim the whole plan hangs on, and it is only true three ways at once.

        The joint was re-zeroed, the sweep found no discontinuity anywhere, and
        the sweep actually covered the travel. Drop any one and the claim is not
        earned.
        """
        return self.rezeroed and self.continuous and self.conclusive

    @property
    def verdict(self) -> str:
        """One of the four :data:`VERDICT_SEAM_EVICTED` constants.

        * :data:`VERDICT_SEAM_EVICTED` — re-zeroed, continuous, and covered. The
          fix works; issue #35 is settled on hardware.
        * :data:`VERDICT_SEAM_NOT_EVICTED` — re-zeroed and STILL discontinuous.
          **The stop condition.** The corrected position is not modularly
          reduced; the re-zero achieves nothing; the plan goes back to the user.
        * :data:`VERDICT_SEAM_PRESENT_BASELINE` — not re-zeroed, discontinuous.
          Exactly what an un-fixed ``elbow_flex`` should look like: this is the
          bug, photographed. Useful, expected, not a failure.
        * :data:`VERDICT_INCONCLUSIVE` — continuous, but either the joint was
          not re-zeroed (so there was no seam to evict and nothing was tested)
          or the sweep did not cover enough travel to have met one.
        """
        if not self.continuous:
            return VERDICT_SEAM_NOT_EVICTED if self.rezeroed else VERDICT_SEAM_PRESENT_BASELINE
        if self.seam_evicted:
            return VERDICT_SEAM_EVICTED
        return VERDICT_INCONCLUSIVE

    @property
    def failed(self) -> bool:
        """``True`` iff this sweep is the STOP condition — a re-zero that did nothing."""
        return self.verdict == VERDICT_SEAM_NOT_EVICTED

    def describe(self) -> str:
        """A verdict a human standing at the arm can act on, without a legend."""
        headline = {
            VERDICT_SEAM_EVICTED: (
                "PASS — the seam is GONE from this joint's travel. The sweep ran "
                f"{self.span} ticks with no discontinuity: the corrected position IS "
                "reduced modulo 4096, the re-zero works, and issue #35 is fixed for "
                f"{self.joint}."
            ),
            VERDICT_SEAM_NOT_EVICTED: (
                "*** STOP — THE RE-ZERO DID NOT WORK. ***\n\n"
                f"{self.joint} carries the seam-evicting offset ({self.offset_in_force}) "
                "and its reported position STILL jumps discontinuously mid-travel "
                f"(largest jump: {self.largest_jump} ticks, at sample "
                f"{self.largest_jump_at}). The servo therefore does NOT reduce the "
                "corrected position modulo 4096 — it reports a plain signed subtraction, "
                "so the offset only RELABELS positions and the discontinuity stays pinned "
                "to the physical angle where the magnet rolls over.\n\n"
                "The re-zero cannot fix issue #35. Do not build anything on top of it. "
                "This needs a decision from the user, not a workaround."
            ),
            VERDICT_SEAM_PRESENT_BASELINE: (
                f"BASELINE — the seam is present, as expected. {self.joint} is not "
                f"re-zeroed (offset in force: {self.offset_in_force}) and its position "
                f"jumps {self.largest_jump} ticks at sample {self.largest_jump_at}. That "
                "jump IS issue #35. Write the offset "
                f"('arm101 arm rezero {self.joint} --apply'), power-cycle, and sweep again "
                "— the jump should be gone."
            ),
            VERDICT_INCONCLUSIVE: (
                "INCONCLUSIVE — this sweep proves nothing." + self._why_inconclusive()
            ),
        }[self.verdict]

        lines = [
            headline,
            "",
            f"- joint            : {self.joint} (motor {self.motor})",
            f"- offset in force  : {self.offset_in_force}"
            f" (seam-evicting offset for this joint: {self.expected_offset})",
            f"- samples          : {len(self.samples)}",
            f"- range reached    : {self.minimum} .. {self.maximum}  (span {self.span} ticks"
            f", expected travel ~{self.expected_travel})",
            f"- monotonic        : {self.monotonic}",
            f"- largest jump     : {self.largest_jump} ticks (at sample {self.largest_jump_at})",
            f"- discontinuities  : {len(self.discontinuities)}"
            f" (threshold {DISCONTINUITY_TICKS} ticks)",
        ]
        if self.out_of_range:
            lines.append(
                f"- IMPOSSIBLE READS : {len(self.out_of_range)} sample(s) outside [0, 4095]"
                f" (e.g. {self.out_of_range[0]}) — the position register cannot hold these,"
                " so the corrected position is an UNWRAPPED signed subtraction."
            )
        if self.rezeroed and self.continuous and self.conclusive:
            lines += [
                "",
                f"Far wall measured for the first time: {self.joint}'s travel spans "
                f"{self.span} ticks ({self.minimum} .. {self.maximum} in the corrected "
                "frame). The arc table in arm101/hardware/arm_spec.py was built on a "
                f"LOWER BOUND of {self.expected_travel}; this is the real number.",
            ]
        return "\n".join(lines)

    def _why_inconclusive(self) -> str:
        """Spell out which of the two ways this sweep failed to test anything."""
        if not self.rezeroed:
            return (
                f" {self.joint} was NOT re-zeroed during it (offset in force: "
                f"{self.offset_in_force}, expected {self.expected_offset}) and no "
                "discontinuity was seen — so either the sweep never reached the seam, or "
                "this joint does not wrap where we think it does. It does NOT show the "
                "re-zero working, because no re-zero was in force."
            )
        return (
            f" The joint moved only {self.span} ticks of its ~{self.expected_travel}-tick "
            f"travel ({MIN_COVERAGE:.0%} required). Seeing no seam across a fraction of the "
            "travel proves nothing — the seam may simply be in the part you did not visit. "
            "Move the joint from one hard stop ALL THE WAY to the other and sweep again."
        )

    def as_dict(self) -> dict[str, object]:
        """JSON-serialisable form, for a verb's ``--json`` payload."""
        return {
            "joint": self.joint,
            "motor": self.motor,
            "offset_in_force": self.offset_in_force,
            "expected_offset": self.expected_offset,
            "rezeroed": self.rezeroed,
            "samples": len(self.samples),
            "minimum": self.minimum,
            "maximum": self.maximum,
            "span": self.span,
            "expected_travel": self.expected_travel,
            "monotonic": self.monotonic,
            "continuous": self.continuous,
            "conclusive": self.conclusive,
            "seam_evicted": self.seam_evicted,
            "largest_jump": self.largest_jump,
            "largest_jump_at": self.largest_jump_at,
            "discontinuity_threshold": DISCONTINUITY_TICKS,
            "discontinuities": [
                {"index": i, "before": before, "after": after}
                for i, before, after in self.discontinuities
            ],
            "out_of_range": list(self.out_of_range),
            "verdict": self.verdict,
            "failed": self.failed,
        }


def analyse_sweep(
    positions: Sequence[int],
    *,
    joint: str,
    motor: int,
    offset_in_force: int,
    expected_offset: int,
    expected_travel: int,
) -> SweepReport:
    """Turn a list of polled positions into a :class:`SweepReport`. Pure — no bus.

    Split out from :func:`sweep` so the *judgement* can be tested exhaustively
    against hand-written position sequences (a clean sweep, a 4095->0 wrap, the
    ~1949-tick masked-signed jump, negative readings, a two-sample nothing)
    without a bus, a clock, or a servo anywhere near it.

    Raises
    ------
    CliError(EXIT_ENV_ERROR)
        If fewer than two positions were sampled. One sample has no deltas, so
        it cannot be continuous OR discontinuous — there is no sweep to judge,
        and returning a cheerful "no discontinuities found" for it would be the
        exact false pass this module is built to prevent.
    """
    if len(positions) < 2:
        raise CliError(
            code=EXIT_ENV_ERROR,
            message=(
                f"The sweep collected {len(positions)} position sample(s) — too few to "
                "judge. Continuity is a statement about the change BETWEEN samples, so at "
                "least two are needed before anything can be concluded."
            ),
            remediation=(
                "Re-run with a longer --duration, and check the joint is answering: "
                "'arm101 arm read'."
            ),
        )

    samples = tuple(int(p) for p in positions)
    deltas = [b - a for a, b in zip(samples, samples[1:])]

    rose = any(d > REVERSAL_TOLERANCE for d in deltas)
    fell = any(d < -REVERSAL_TOLERANCE for d in deltas)

    largest_jump_at = max(range(len(deltas)), key=lambda i: abs(deltas[i]))
    discontinuities = tuple(
        (i, samples[i], samples[i + 1])
        for i, d in enumerate(deltas)
        if abs(d) >= DISCONTINUITY_TICKS
    )
    out_of_range = tuple(p for p in samples if not (arm_spec.TICK_MIN <= p <= arm_spec.TICK_MAX))

    return SweepReport(
        joint=joint,
        motor=motor,
        offset_in_force=offset_in_force,
        expected_offset=expected_offset,
        samples=samples,
        minimum=min(samples),
        maximum=max(samples),
        monotonic=not (rose and fell),
        largest_jump=abs(deltas[largest_jump_at]),
        largest_jump_at=largest_jump_at,
        discontinuities=discontinuities,
        out_of_range=out_of_range,
        expected_travel=expected_travel,
    )


def _needs_pacing(bus: "MotorBus") -> bool:
    """Does this bus need real time to pass between samples?

    A physical servo does — the human's hand advances it by wall-clock, so the
    loop must wait between reads or it samples the same tick six hundred times.
    A simulated bus advances *per read*, so pacing it would only make the suite
    sleep. Same seam, same reasoning, as
    :func:`arm101.hardware.gentle._needs_pacing`.
    """
    return not isinstance(bus, FakeBus)


def sweep(
    bus: "MotorBus",
    motor: int,
    joint: str,
    *,
    samples: int,
    interval: float = DEFAULT_SWEEP_INTERVAL,
    on_sample: Optional[Callable[[int, int], None]] = None,
) -> SweepReport:
    """De-energise *motor* and poll its position while a HUMAN hand-moves the joint.

    The proof step, and the only one that can settle the open question. It
    commands nothing: the joint goes limp and the operator walks it from one
    hard stop to the other while this watches. That is not a fallback for
    missing automation — it is the *right* instrument, because a human arm is
    the only actuator available that does not need a linear tick axis to work,
    and a linear tick axis is precisely what is in doubt.

    Torque is disabled first, via
    :meth:`~arm101.hardware.bus.MotorBus.clear_overload` (which also clears a
    latch left by a previous run), and is **never re-enabled** — the verb
    deliberately ends with the joint limp, because the operator's hand is on it.
    The caller wraps this in a
    :func:`~arm101.hardware.safety.torque_guard` anyway, so an abnormal exit
    mid-sweep cannot leave a motor hot.

    Parameters
    ----------
    bus:
        An open :class:`~arm101.hardware.bus.MotorBus`.
    motor, joint:
        The servo id, and the joint name it carries (used to look up the
        expected offset and travel).
    samples:
        How many positions to poll. With *interval*, this sets the wall-clock
        length of the sweep.
    interval:
        Seconds between polls on real hardware. Skipped entirely for a
        :class:`~arm101.hardware.bus.FakeBus` (see :func:`_needs_pacing`), whose
        simulated shaft advances per read rather than per second.
    on_sample:
        Optional ``(index, position)`` callback, invoked after every poll — the
        verb uses it to show the operator the position moving in real time, so
        they can see they are actually driving the joint and not merely holding
        it.

    Raises
    ------
    CliError(EXIT_USER_ERROR)
        If *samples* is less than 2 (see :func:`analyse_sweep`).
    CliError(EXIT_ENV_ERROR)
        If the bus is not open or a read fails.
    """
    if samples < 2:
        raise CliError(
            code=EXIT_USER_ERROR,
            message=f"A sweep needs at least 2 samples to have any deltas, got {samples}.",
            remediation="Increase the sweep duration so it collects at least two samples.",
        )

    expected_offset, arc = require_rezeroable(joint)

    # Limp FIRST. A joint the human is about to hand-move must not be holding —
    # and one that fought a wall on the way here may be latched in overload, in
    # which case a plain enable_torque(False) would raise instead of releasing.
    bus.clear_overload(motor)

    # Read the offset AFTER de-energising and BEFORE the first sample, so the
    # report says which frame the samples below are actually in — rather than
    # assuming the one we hoped for.
    offset_in_force = bus.read_offset(motor)

    pace = interval if _needs_pacing(bus) else 0.0
    positions: list[int] = []
    for index in range(samples):
        position = bus.read_position(motor)
        positions.append(position)
        if on_sample is not None:
            on_sample(index, position)
        if pace:
            time.sleep(pace)

    return analyse_sweep(
        positions,
        joint=joint,
        motor=motor,
        offset_in_force=offset_in_force,
        expected_offset=expected_offset,
        expected_travel=arc.travel_ticks,
    )


def samples_for(duration: float, interval: float = DEFAULT_SWEEP_INTERVAL) -> int:
    """How many polls fit in *duration* seconds at *interval* seconds apart.

    Raises
    ------
    CliError(EXIT_USER_ERROR)
        If *duration* is not long enough to collect two samples — the minimum
        for any delta to exist at all.
    """
    count = int(duration / interval)
    if count < 2:
        raise CliError(
            code=EXIT_USER_ERROR,
            message=(
                f"A sweep of {duration}s at {interval}s per sample collects {count} "
                "sample(s) — too few to detect a discontinuity, which is a change BETWEEN "
                "two samples."
            ),
            remediation=(
                f"Pass a longer --duration (at least {2 * interval:.2f}s; the default "
                f"{DEFAULT_SWEEP_DURATION:.0f}s is sized for a human to walk the joint "
                "from one hard stop to the other without hurrying)."
            ),
        )
    return count
