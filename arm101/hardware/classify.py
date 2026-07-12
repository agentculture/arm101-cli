"""arm101.hardware.classify — BOUNDED, CONTINUOUS, or honestly UNDETERMINED.

Pure module. **No bus, no I/O** (pinned by ``test_classify_module_never_imports_the_bus``
and its transitive twin). It reads a :class:`~arm101.hardware.limits.JointTravel` —
accumulated RAW displacement and four-verdict ends — and answers the one question
that decides what can be done about a joint's encoder seam.

The question
===========

The encoder's 4095 -> 0 rollover — the **seam** — is a discontinuity in a number
that every position comparison in this codebase treats as linear. There are exactly
two instruments for it, and which one applies is a property of the *joint*, not of
anybody's preference:

* **Re-zero.** Move the seam into an arc the joint physically cannot reach. Needs
  such an arc to exist.
* **Soft limit.** Fence off a software-only dead arc containing the seam, and never
  command the joint into it. Costs travel, but needs nothing from the joint.

So: **does the joint have an unreachable arc?** That is the whole classification.

* :attr:`TravelKind.BOUNDED` — a **WALL at both ends**. The travel is known exactly,
  its complement is a real arc of ticks the joint can never visit, and the seam can
  be parked in there. This is ``elbow_flex``, and it is why it was fixable.
* :attr:`TravelKind.CONTINUOUS` — the joint **swept a full turn** (4096 raw ticks).
  Its travel covers every angle, so **there is no angle to put the seam at**. Re-zero
  is impossible *in principle*, not merely unavailable, and a soft limit is the only
  instrument. This is ``wrist_roll`` — and it must come back CONTINUOUS **because it
  is**, not because it is named.
* :attr:`TravelKind.UNDETERMINED` — anything else. One wall and one torque-limited
  end; two torque-limited ends; an EDGE; a TIMEOUT. You cannot site an arc without
  two ends you can vouch for, and you have not seen a full turn. It is neither, and
  forcing it into one of the two would be inventing a measurement.

NO JOINT NAMES. THIS IS THE POINT, NOT A STYLE PREFERENCE
=========================================================

``wrist_roll`` is *known* continuous and ``elbow_flex`` is *known* bounded — both
established by a long human session on the physical arm. The acceptance test for
this verb is that it **re-derives both answers from measurement alone**. A special
case keyed on a joint's name would make that test vacuous: the verb would recite the
answer it was handed, and no verdict it then gave on the four *unknown* joints could
be believed. So this module holds no joint table, reads no joint-keyed table, and
branches on no name (``test_the_classifier_s_logic_names_no_joint`` parses its AST
and proves it; ``test_the_classification_is_invariant_under_the_joint_s_name`` proves
it behaviourally, by classifying the same measurement under every joint's name).

Coverage of the circle, not absence of walls
============================================

CONTINUOUS is decided by **how far the joint swept**, never by "no wall was found"
and never by "we got back to where we started". Two consequences worth stating:

* A joint that turns 4096 ticks *and has hard stops* (an over-rotating joint whose
  travel exceeds a full turn) is still CONTINUOUS: every angle is reachable, so no
  offset can help it. A rule keyed on "no wall" would call that BOUNDED and then
  fail to find an arc that does not exist.
* "The reported position came back to its starting tick" is true of a joint that
  swept the whole circle **and** of a joint that never moved at all. Accumulated
  displacement separates them; raw ticks cannot.

The narrow-arc cutoff
=====================

A BOUNDED joint's arc can still be too tight to use. The seam must sit clear of both
walls by :data:`ARC_MARGIN_TICKS` — ``arm_spec``'s own inset, imported rather than
re-typed so the two cannot drift — because a wall is not a crisp number and at least
one of ``elbow_flex``'s was the *table* rather than the joint's own stop. An arc
narrower than :data:`MIN_EVICTABLE_ARC_TICKS` cannot carry those margins and still
have a tick left to put the seam on, and re-zeroing into a sliver is precisely what
broke the first ``elbow_flex`` attempt (the joint came to rest eleven ticks past an
edge taken from a sweep somebody had stopped short of). Such a joint is reported as
BOUNDED — that much *is* measured — with **no arc**: soft-limit territory.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Dict, Iterable, List, Optional, Tuple

# ``ARC_MARGIN_TICKS`` is ``arm_spec``'s own inset — imported rather than re-typed,
# and RE-EXPORTED here so this module's readers and tests can reach it by the name they
# already use. It is the SAME rule ("inset the measured envelope before declaring an
# arc"), and two copies of a safety margin is one copy too many: a re-measurement that
# widens it there must widen the cutoff here, without anyone remembering to. (It was
# spelled ``_ARC_MARGIN_TICKS`` while ``REZERO_ARCS`` was the only thing applying it; a
# number two modules share should not be private, so t8 promoted it.)
#
# Ticks of clearance the seam must keep from EACH measured wall: a hand-found wall moves
# 206..218 depending on how hard you push, an ARM-found wall can still be the *table*
# rather than the joint's stop, and an environmental wall makes the true travel WIDER
# than measured — hence the true unreachable arc NARROWER. Insetting is conservative
# against all three: it can neither false-refuse a legal position nor claim a tick the
# joint can actually reach.
from arm101.hardware.arm_spec import ARC_MARGIN_TICKS, UnreachableArc
from arm101.hardware.limits import (
    ENCODER_TICKS,
    EndObservation,
    JointTravel,
    MeasuredEnd,
    merge_joint_travel,
)
from arm101.hardware.ticks import MAX_ENCODER_OFFSET, TICK_MAX, TICK_MIN

#: The narrowest measured unreachable arc a re-zero may still be offered for.
#:
#: **The cutoff, and why it sits exactly here.** Inset :data:`ARC_MARGIN_TICKS` from
#: each measured wall and the arc must *still* have a tick strictly inside it to put
#: the seam on — an arc one tick wide has no interior, which is exactly what
#: ``arm_spec._require_evictable_seam`` refuses ("there is nowhere to evict the seam
#: TO"). Two margins plus those two ticks of interior is the floor:
#: ``2 * ARC_MARGIN_TICKS + 2``.
#:
#: At the cutoff, the seam lands a full margin clear of both walls — i.e. **the same
#: risk profile the shipped ``elbow_flex`` re-zero already runs**, and no worse. One
#: tick narrower and the seam would sit closer to a wall than the margin ``arm_spec``
#: insists on, which is the failure that broke the first re-zero attempt. That is the
#: whole justification: the line is drawn where the guarantee this repo already
#: accepts stops holding, rather than at a number somebody liked.
#:
#: (Not doubled up with ``SEAM_CLEARANCE_TICKS``, the soft limit's analogous number:
#: they are the same concern — keep the seam away from where the joint actually goes
#: — seen from the two sides. Requiring both would demand ~400 ticks of arc for no
#: additional physical guarantee.)
MIN_EVICTABLE_ARC_TICKS: int = 2 * ARC_MARGIN_TICKS + 2


class TravelKind(str, Enum):
    """What a joint's measured travel says about its encoder seam.

    A ``str`` subclass so a member serializes to JSON as its plain string value.
    """

    #: A WALL at both ends. The complement of the travel is a real, permanently
    #: unreachable arc — so the seam can be evicted into it (if it is wide enough:
    #: see :data:`MIN_EVICTABLE_ARC_TICKS`).
    BOUNDED = "bounded"

    #: The joint swept a full turn: every angle is reachable. There is nowhere to put
    #: the seam, so a re-zero is impossible **in principle**. A soft limit — a
    #: software-only dead arc containing the seam — is the only instrument left.
    CONTINUOUS = "continuous"

    #: Neither established. Not two walls (so no arc can be sited), and not a full
    #: turn (so continuity is not shown either). The honest answer, and a real one:
    #: it says *measure again*, not *pick one*.
    UNDETERMINED = "undetermined"


class SeamRemedy(str, Enum):
    """Which instrument can deal with this joint's seam — if it even has a problem.

    Derived entirely from the classification; stored nowhere. Note what it does NOT
    claim: that the joint *needs* the instrument. A bounded joint whose seam already
    lies outside its travel needs nothing at all, and that is a different answer from
    "nothing can be done" — collapsing the two would teach an operator the wrong
    thing about their arm.
    """

    #: An unreachable arc exists and can carry the seam with margin to spare.
    REZERO = "rezero"

    #: The seam is inside the travel and cannot be evicted — either because the joint
    #: reaches every angle (CONTINUOUS) or because its arc is too narrow to hold the
    #: seam clear of both walls. A software dead arc is what is left.
    SOFT_LIMIT = "soft_limit"

    #: The seam is not inside the joint's travel. The tick axis is already linear for
    #: it; there is nothing to fix.
    NONE_NEEDED = "none_needed"

    #: The travel is UNDETERMINED. Choosing an instrument on this evidence would be
    #: guessing.
    UNKNOWN = "unknown"


# ---------------------------------------------------------------------------
# The record
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class TravelClassification:
    """What one joint's measured travel means for its seam.

    Follows :mod:`arm101.hardware.limits`' discipline rather than inventing a parallel
    one: the fields that would let a caller mistake a non-result for a measurement are
    ``None`` **by construction**, and the constructor refuses the combinations that
    cannot be true. In particular a CONTINUOUS or UNDETERMINED classification cannot
    carry an unreachable arc — not "does not happen to", *cannot* — and an arc that
    could not actually take the seam cannot be stored at all.

    Attributes
    ----------
    joint:
        The joint. An opaque label: **nothing in this module branches on it.**
    kind:
        :class:`TravelKind`.
    reason:
        Why, in prose, for a report or an operator.
    swept_ticks:
        The most raw travel the joint was ever seen to cover — the number the
        BOUNDED/CONTINUOUS decision turns on. ``>= ENCODER_TICKS`` iff CONTINUOUS.
    seam_in_travel:
        ``True`` if the joint was OBSERVED to cross the raw seam (a fact, whatever
        the verdicts). ``False`` only when both walls are vouched for and the travel
        provably misses it. ``None`` when the travel is a lower bound that did not
        happen to cross it — the unmeasured part may well.
    reachable_raw_ends:
        ``(low, high)`` RAW ticks of the two walls. ``None`` unless BOUNDED. **May
        wrap** (``low > high``) — that is the whole reason a ``[min, max]`` pair
        cannot describe a joint like this.
    unreachable_width:
        Ticks the joint can never reach: ``ENCODER_TICKS - swept_ticks``. ``None``
        unless BOUNDED.
    unreachable_arc:
        The arc to evict the seam into, already inset by :data:`ARC_MARGIN_TICKS` on
        each side. ``None`` unless BOUNDED **and** the seam is in the travel **and**
        the arc can carry the seam clear of both walls.
    """

    joint: str
    kind: TravelKind
    reason: str
    swept_ticks: int
    seam_in_travel: Optional[bool]
    reachable_raw_ends: Optional[Tuple[int, int]] = None
    unreachable_width: Optional[int] = None
    unreachable_arc: Optional[UnreachableArc] = None

    def __post_init__(self) -> None:
        if not isinstance(self.joint, str) or not self.joint:
            raise ValueError("joint must be a non-empty name.")
        object.__setattr__(self, "kind", TravelKind(self.kind))
        object.__setattr__(self, "swept_ticks", int(self.swept_ticks))
        if self.swept_ticks < 0:
            raise ValueError(
                f"swept_ticks {self.swept_ticks} is a distance — it cannot be negative."
            )

        self._require_the_definition_of_continuous()
        self._require_bounded_carries_its_walls()
        self._require_the_arc_can_take_the_seam()

    # -- the invariants, each stating one thing that cannot be true ----------

    def _require_the_definition_of_continuous(self) -> None:
        """CONTINUOUS iff a full turn was swept. The definition, pinned into the type."""
        swept_a_full_turn = self.swept_ticks >= ENCODER_TICKS
        if swept_a_full_turn and self.kind is not TravelKind.CONTINUOUS:
            raise ValueError(
                f"{self.kind.value.upper()} claims a travel of {self.swept_ticks} ticks — a "
                f"full turn ({ENCODER_TICKS}) or more. A joint that sweeps the whole circle "
                "reaches every angle, so there is no angle left to put the seam at: it is "
                "CONTINUOUS, whatever else was found at its ends."
            )
        if self.kind is TravelKind.CONTINUOUS:
            if not swept_a_full_turn:
                raise ValueError(
                    f"CONTINUOUS claims a travel of only {self.swept_ticks} ticks, short of the "
                    f"full turn ({ENCODER_TICKS}) that would prove it. A joint is not continuous "
                    "for want of a wall — it is continuous because it was SEEN to cover every "
                    "angle."
                )
            if self.seam_in_travel is not True:
                raise ValueError(
                    "a CONTINUOUS joint reaches every angle, so the seam is necessarily inside "
                    "its travel. seam_in_travel must be True."
                )

    def _require_bounded_carries_its_walls(self) -> None:
        """Two walls is what BOUNDED means — and what nothing else may claim."""
        vouched = (self.reachable_raw_ends, self.unreachable_width)
        if self.kind is not TravelKind.BOUNDED:
            if any(value is not None for value in vouched) or self.unreachable_arc is not None:
                raise ValueError(
                    f"a {self.kind.value.upper()} travel cannot carry walls, a width or an arc: "
                    "none of them was vouched for. Only a WALL at BOTH ends bounds a joint, and "
                    "a record that says otherwise is the laundering this type exists to prevent."
                )
            return

        if any(value is None for value in vouched):
            raise ValueError(
                "a BOUNDED travel must carry both of its walls and its unreachable width — "
                "that pair IS the measurement."
            )

        low_raw, high_raw = self.reachable_raw_ends
        for name, tick in (("low", low_raw), ("high", high_raw)):
            if not (TICK_MIN <= tick <= TICK_MAX):
                raise ValueError(
                    f"the {name} wall {tick} is not a raw tick — it must lie in "
                    f"[{TICK_MIN}, {TICK_MAX}]."
                )
        if self.unreachable_width != ENCODER_TICKS - self.swept_ticks:
            raise ValueError(
                f"unreachable_width {self.unreachable_width} is not the rest of the circle "
                f"({ENCODER_TICKS} - {self.swept_ticks} = {ENCODER_TICKS - self.swept_ticks}). "
                "The arc is the COMPLEMENT of the travel; a width that says otherwise describes "
                "neither."
            )
        # Walking up from the high wall, across the arc, lands on the low wall.
        if (low_raw - high_raw) % ENCODER_TICKS != self.unreachable_width % ENCODER_TICKS:
            raise ValueError(
                f"the walls ({low_raw}, {high_raw}) are not {self.unreachable_width} ticks apart "
                "the way round the joint cannot go. The width and the walls disagree, so neither "
                "can be trusted."
            )

    def _require_the_arc_can_take_the_seam(self) -> None:
        """An arc that cannot hold the seam is not an arc — it is a false promise."""
        arc = self.unreachable_arc
        if arc is None:
            return
        if self.seam_in_travel is not True:
            raise ValueError(
                "an unreachable arc is the answer to a seam INSIDE the joint's travel. With the "
                "seam already outside it there is no question to answer, and no re-zero to offer."
            )
        if not _arc_can_take_the_seam(arc):
            raise ValueError(
                f"the arc ({arc.low}, {arc.high}) cannot take the seam: its midpoint "
                f"{arc.midpoint} is either not strictly inside it or needs an offset outside "
                f"the register's [-{MAX_ENCODER_OFFSET}, +{MAX_ENCODER_OFFSET}] range. An arc "
                "a re-zero cannot actually use must not be handed out as one."
            )

    # -- what the record says ------------------------------------------------

    @property
    def can_be_rezeroed(self) -> bool:
        """``True`` iff the seam is in this joint's travel AND an arc can take it."""
        return self.unreachable_arc is not None

    @property
    def rezero_offset(self) -> Optional[int]:
        """The signed register value a re-zero would write, or ``None`` if there is none."""
        return None if self.unreachable_arc is None else self.unreachable_arc.offset

    @property
    def remedy(self) -> SeamRemedy:
        """Which instrument can deal with this joint's seam (:class:`SeamRemedy`)."""
        if self.kind is TravelKind.UNDETERMINED:
            return SeamRemedy.UNKNOWN
        if self.unreachable_arc is not None:
            return SeamRemedy.REZERO
        if self.seam_in_travel:
            return SeamRemedy.SOFT_LIMIT
        return SeamRemedy.NONE_NEEDED

    def to_dict(self) -> Dict[str, object]:
        """A plain-JSON-serializable view (no enums, no dataclasses).

        Serialized one way only, deliberately. A classification is *derived* — the
        durable fact is the travel it was computed from, and re-deriving it is free.
        A ``from_dict`` would invite a hand-edited verdict back in through the file,
        which is the loophole :meth:`~arm101.hardware.limits.MeasuredEnd.from_dict`
        goes to some trouble to close.
        """
        arc = self.unreachable_arc
        return {
            "joint": self.joint,
            "kind": self.kind.value,
            "remedy": self.remedy.value,
            "reason": self.reason,
            "swept_ticks": self.swept_ticks,
            "seam_in_travel": self.seam_in_travel,
            "reachable_raw_ends": (
                None if self.reachable_raw_ends is None else list(self.reachable_raw_ends)
            ),
            "unreachable_width": self.unreachable_width,
            "unreachable_arc": (
                None
                if arc is None
                else {
                    "low": arc.low,
                    "high": arc.high,
                    "midpoint": arc.midpoint,
                    "offset": arc.offset,
                }
            ),
        }


# ---------------------------------------------------------------------------
# The geometry — all of it in RAW ticks and accumulated displacement
# ---------------------------------------------------------------------------


def _arc_can_take_the_seam(arc: UnreachableArc) -> bool:
    """``True`` iff a re-zero can actually park the seam in *arc*.

    The three conditions ``arm_spec._require_evictable_seam`` enforces on the shipped
    table, asked of a *candidate* arc instead of raised over one — composed from the
    arc's own primitives rather than restated, so there is one definition of "this arc
    can hold the seam":

    1. it has a tick strictly inside it (a one-tick arc has nowhere to put the seam);
    2. the seam placement is expressible in the sign-magnitude offset register (raw
       2048 is the single placement it cannot hold);
    3. the raw -> signed -> raw round-trip closes, so the offset written and the arc it
       came from are talking about the same tick.
    """
    if not arc.contains(arc.midpoint):
        return False
    offset = arc.offset
    if abs(offset) > MAX_ENCODER_OFFSET:
        return False
    return arc.evicts(offset)


def _evictable_arc(bottom_edge: int, top_edge: int) -> Optional[UnreachableArc]:
    """Inset the MEASURED unreachable arc ``(bottom_edge, top_edge)`` into a usable one.

    ``None`` when the measured arc is too narrow to carry a margin at each wall and
    still leave a tick to put the seam on — soft-limit territory, not a sliver to
    re-zero into.

    The candidates are tried widest-first and every one of them **shrinks** the arc,
    never widens it: the result is always a strict subset of the region the joint was
    measured unable to reach, so it can neither false-refuse a legal position nor claim
    a tick the joint can actually get to. The two one-tick shrinks exist for a single
    corner — an arc whose midpoint lands on the one raw tick the offset register cannot
    express. Refusing a wide arc over a one-tick encoding hole would be a false verdict;
    moving the seam one tick off centre costs nothing physical. (When the arc is at its
    narrowest the shrinks have nowhere to go, and the refusal that follows is the
    correct answer: the only tick available is one the register cannot name.)
    """
    if top_edge - bottom_edge < MIN_EVICTABLE_ARC_TICKS:
        return None

    low = bottom_edge + ARC_MARGIN_TICKS
    high = top_edge - ARC_MARGIN_TICKS
    for candidate_low, candidate_high in ((low, high), (low, high - 1), (low + 1, high)):
        if candidate_high - candidate_low < 2:
            continue  # no tick strictly inside — nowhere to put the seam
        arc = UnreachableArc(low=candidate_low, high=candidate_high)
        if _arc_can_take_the_seam(arc):
            return arc
    return None


def _crosses_the_seam(low_raw: int, span: int) -> bool:
    """``True`` iff walking *span* ticks UP from *low_raw* rolls over 4095 -> 0.

    Which is what "the seam is inside the travel" means: not that the joint sits near
    tick 0, but that its own travel carries it across the discontinuity — the crossing
    that makes its reported position non-monotonic with joint angle. Touching the top
    tick is not rolling over it.
    """
    return low_raw + span > TICK_MAX


def _swept_by(record: MeasuredEnd) -> int:
    """The longest contiguous raw sweep behind one merged end, in ticks."""
    return abs(record.evidence.displacement)


# ---------------------------------------------------------------------------
# The classification
# ---------------------------------------------------------------------------


def _continuous(joint: str, swept: int) -> TravelClassification:
    """A joint that covered the whole circle. No arc exists; a soft limit is all there is."""
    return TravelClassification(
        joint=joint,
        kind=TravelKind.CONTINUOUS,
        reason=(
            f"{joint} swept {swept} raw ticks — a full turn ({ENCODER_TICKS}) or more — so its "
            "travel covers every angle. There is no angle left to put the encoder seam at, which "
            "means a re-zero cannot help it even in principle: an offset RELOCATES the seam, it "
            "never EVICTS it. Only a soft limit (a software-only dead arc the joint is never "
            "commanded into, with the seam inside it) can fence the discontinuity off."
        ),
        swept_ticks=swept,
        seam_in_travel=True,
    )


def classify_travel(travel: JointTravel) -> TravelClassification:
    """Classify one joint's measured travel: BOUNDED, CONTINUOUS, or UNDETERMINED.

    Reads accumulated RAW displacement and the four-verdict ends — nothing else. In
    particular it does not know, and cannot ask, which joint it is looking at.
    """
    joint = travel.joint
    span = travel.span
    swept = max(span, _swept_by(travel.low), _swept_by(travel.high))

    if swept >= ENCODER_TICKS:
        return _continuous(joint, swept)

    low_raw, high_raw = travel.low.reach, travel.high.reach
    crossed = _crosses_the_seam(low_raw, span)
    walls = travel.mechanical_raw_ends

    if walls is None:
        unvouched = ", ".join(
            f"{record.end.value} ({record.verdict.value})"
            for record in (travel.low, travel.high)
            if record.mechanical_limit is None
        )
        return TravelClassification(
            joint=joint,
            kind=TravelKind.UNDETERMINED,
            reason=(
                f"{joint} was driven {span} ticks between its two ends, but no wall vouches for "
                f"the {unvouched} end. An unreachable arc cannot be sited without a wall at BOTH "
                f"ends, and a full turn ({ENCODER_TICKS} ticks) was not seen either — so the "
                "joint is neither shown bounded nor shown continuous. Measure again; do not pick."
            ),
            swept_ticks=swept,
            # A crossing that was OBSERVED is a fact. Not observing one proves nothing:
            # the travel is a lower bound, and the part that was not measured may well
            # contain the seam.
            seam_in_travel=True if crossed else None,
        )

    width = ENCODER_TICKS - span
    arc = _evictable_arc(high_raw, low_raw) if crossed else None

    if not crossed:
        reason = (
            f"{joint} has a WALL at both ends, {span} ticks apart (raw {low_raw}..{high_raw}), so "
            f"{width} ticks of the circle are permanently out of its reach. Its travel does not "
            "cross the encoder seam, so its reported position is already monotonic with joint "
            "angle: there is nothing to evict and nothing to fence off."
        )
    elif arc is not None:
        reason = (
            f"{joint} has a WALL at both ends, {span} ticks apart, and its travel WRAPS the "
            f"encoder seam (raw {low_raw} up across 4095->0 to {high_raw}). The {width} ticks it "
            f"cannot reach are wide enough to take the seam with {ARC_MARGIN_TICKS} ticks of "
            f"clearance at each wall: re-zero to offset {arc.offset} puts the seam at raw "
            f"{arc.midpoint}, out of the joint's reach, and the tick axis is linear again."
        )
    else:
        reason = (
            f"{joint} has a WALL at both ends, {span} ticks apart, and its travel WRAPS the "
            f"encoder seam — but the {width} ticks it cannot reach are narrower than the "
            f"{MIN_EVICTABLE_ARC_TICKS} a seam needs ({ARC_MARGIN_TICKS} of clearance at each "
            "wall, plus a tick to sit on). Re-zeroing into a sliver is how a joint comes to rest "
            "just past an edge and gets refused a position it can plainly reach. Use a soft limit."
        )

    return TravelClassification(
        joint=joint,
        kind=TravelKind.BOUNDED,
        reason=reason,
        swept_ticks=swept,
        seam_in_travel=crossed,
        reachable_raw_ends=walls,
        unreachable_width=width,
        unreachable_arc=arc,
    )


def classify_observations(observations: Iterable[EndObservation]) -> TravelClassification:
    """Classify from raw per-pose observations — what the probe actually holds.

    Equivalent to :func:`classify_travel` over
    :func:`~arm101.hardware.limits.merge_joint_travel`, with one exception that is the
    whole reason this function exists: **a single end that swept a full turn settles the
    question on its own.** The probe stops at the cap (accumulated displacement, never
    "we got back to where we started"), so for a continuous joint it will never HAVE a
    second end to offer — demanding one would force it to fabricate an observation, and
    a fabricated observation is exactly what this module family exists to make
    impossible.

    Every other single-ended batch still raises: an end that was never probed is not an
    end with nothing there.
    """
    records: List[EndObservation] = list(observations)
    if not records:
        raise ValueError("cannot classify no observations — there is no travel to classify.")

    joints = {record.joint for record in records}
    if len(joints) != 1:
        raise ValueError(f"every observation must be for the same joint; got {sorted(joints)}.")

    swept = max(abs(record.displacement) for record in records)
    if swept >= ENCODER_TICKS:
        return _continuous(records[0].joint, swept)

    # Anything else needs both ends, and merge_joint_travel already says why in the
    # right words: "an end that was never probed is not an end with nothing there."
    return classify_travel(merge_joint_travel(records))


__all__ = [
    "ARC_MARGIN_TICKS",
    "MIN_EVICTABLE_ARC_TICKS",
    "SeamRemedy",
    "TravelClassification",
    "TravelKind",
    "classify_observations",
    "classify_travel",
]
