"""arm101.hardware.limits — the FOUR-VERDICT record of a joint's measured travel.

Pure data module. **No bus, no I/O** (pinned by ``test_limits_module_never_imports_the_bus``
and its transitive twin): a record of what the hardware said must not be able to
talk back to the hardware. It imports only stdlib and :mod:`arm101.hardware.arm_spec`
(itself pure) for the encoder constants, so the tick modulus has one definition.

Why this module exists
======================

``gentle_move`` calls **contact** when ``present_load`` is high *and* the joint has
stopped advancing. That rule cannot tell a **wall** from **the arm being too weak**.
``shoulder_lift`` carries the whole arm; lifting it against ``gentle_move``'s 500
``Torque_Limit`` cap can make it stall at high load with *nothing in front of it*.
Record that as a mechanical limit and you have written a permanent lie into
``arm_spec``. Two further non-results must not be laundered into measurements
either: the probe can run out of commandable frame, or simply never arrive.

So each **END** of each joint's travel — not each joint — carries one of four
verdicts (:class:`LimitVerdict`):

============== ============================================================
``WALL``       Load saturated against something solid. **A real limit.**
``TORQUE_LIMITED`` Stalled at high load, but the torque cap is the likeliest
               reason, not a wall. **A LOWER BOUND, never a wall.**
``EDGE``       Ran out of commandable frame. Learned nothing yet.
``TIMEOUT``    Never got there. Learned nothing.
============== ============================================================

Environmental vs mechanical — and the asymmetry that shapes this API
====================================================================

A limit found in ONE pose is **environmental**: it depends on where the other
joints happen to be. The **mechanical** limit is the widest envelope ever seen
across poses — because *an obstacle can only ever make a range smaller*, so the
maximum over poses converges on the mechanical truth.

**That reasoning is sound for a WALL and FALSE for a TORQUE-LIMITED end.** The
arm's own weakness is not an obstacle you can pose your way out of. It shrinks the
range in *every* pose, so no number of poses ever escapes it, and a torque-limited
end stays a lower bound **forever**.

The types encode that asymmetry rather than documenting it:

* :class:`EndObservation` — what ONE probe found at ONE end, in ONE pose. Always
  environmental. It is *evidence*, not a limit.
* :class:`MeasuredEnd` — the merge of every pose's observation of one end. It is
  **abstract**, and it has exactly two concrete forms:

  * :class:`WallEnd` — a wall was found. ``mechanical_limit`` is an ``int``.
    **Cannot be constructed from non-WALL evidence — it raises.**
  * :class:`LowerBoundEnd` — the arm got at least this far and was stopped by
    something that is not known to be a wall. ``mechanical_limit`` is ``None``.
    **Cannot be constructed from WALL evidence — it raises.**

* :class:`JointTravel` — the two ends of one joint. ``mechanical_extent`` /
  ``mechanical_raw_ends`` return ``None`` unless **both** ends are walls; the only
  pair you can always read out is ``envelope_extent``, whose name claims nothing it
  cannot back.

There is therefore no accessor anywhere that turns a torque-limited stall into a
``(min, max)`` a caller could mistake for a wall, and no merge — over any number of
poses — that produces a :class:`WallEnd` without a :class:`LimitVerdict.WALL`
observation behind it. The wrong thing is unrepresentable, not merely discouraged.

The frame: RAW displacement, because the seam must not masquerade as a limit
===========================================================================

Every observation is stored as ``origin_raw`` (where the probe started, a raw tick)
plus a **signed accumulated RAW displacement**. Two reasons, both load-bearing:

* **The seam.** A probe that creeps from raw 4000 up by 200 ticks ends at raw 104.
  Compare raw *ticks* and that reads as a 3896-tick retreat — the encoder seam
  wearing a limit's clothes, which is the entire bug this feature exists to kill.
  Displacement says what happened: it went 200 ticks further out.
* **The rolling frame.** The probe rewrites the servo's offset mid-run to keep the
  seam half a turn away, so *reported* ticks mean different angles at different
  moments. Raw displacement is invariant under that; reported anything is not.

Displacement is measured from each pose's **own** start, so it cannot be compared
across poses directly. :func:`merge_end_observations` therefore re-expresses every
observation against one shared ``reference_raw`` — taking the short way round the
circle (:func:`signed_delta`) — before asking which pose got furthest. Displacement
is capped at one full turn (:data:`ENCODER_TICKS`): past that there is nothing left
to learn (the joint is continuous), and the cap is also what keeps every observation
within one lap of the reference, so the comparison is well-defined.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import ClassVar, Dict, Iterable, List, Optional, Sequence, Tuple

from arm101.hardware.arm_spec import ENCODER_TICKS, TICK_MAX, TICK_MIN

#: Half a full turn of the 12-bit encoder (2048 ticks) — the largest distance any
#: two raw ticks can be apart once you are allowed to go the short way round.
HALF_TURN: int = ENCODER_TICKS // 2


def signed_delta(tick: int, reference: int) -> int:
    """Return ``tick - reference`` taken the SHORT way round the encoder circle.

    The result lands in ``[-HALF_TURN, HALF_TURN)``. Raw 4090 and raw 10 are **16
    ticks apart**, not 4080 — the seam is a labelling artefact, not a distance, and
    plain subtraction is how it gets mistaken for one.

    (The single ambiguous case, two ticks exactly antipodal at 2048 apart, resolves
    to ``-HALF_TURN`` — deterministic, and the same ±2048 corner
    :func:`arm101.hardware.arm_spec._offset_for_seam_at` already documents.)
    """
    return ((tick - reference + HALF_TURN) % ENCODER_TICKS) - HALF_TURN


def _require_raw_tick(value: int, name: str) -> int:
    """Return *value* as an ``int`` if it is a raw encoder tick, else raise."""
    value = int(value)
    if not (TICK_MIN <= value <= TICK_MAX):
        raise ValueError(f"{name} {value} is not a raw tick — it must lie in [0, {TICK_MAX}].")
    return value


# ---------------------------------------------------------------------------
# LimitVerdict — the four names, one of which vouches
# ---------------------------------------------------------------------------


class LimitVerdict(str, Enum):
    """What a probe found at ONE end of ONE joint's travel, in ONE pose.

    A ``str`` subclass so a member serializes to JSON as its plain string value with
    no custom encoder, while still comparing and hashing distinctly per member.

    Exactly one of the four — :attr:`WALL` — is evidence of a mechanical limit. The
    other three are all reasons the probe stopped that say nothing about what, if
    anything, is in front of the joint. Keeping them apart is the point: collapsing
    them into "no contact" is what let the encoder seam, and the arm's own weakness,
    both pass themselves off as walls.
    """

    #: Load saturated against something solid. A real limit — the joint is
    #: mechanically stopped here (in this pose).
    WALL = "wall"

    #: Stalled at high load, but the ``Torque_Limit`` cap is the likeliest reason,
    #: not a wall. ``shoulder_lift`` carrying the whole arm is the canonical case.
    #: **A LOWER BOUND, never a wall** — and no amount of posing changes that,
    #: because the arm's weakness is present in every pose.
    TORQUE_LIMITED = "torque_limited"

    #: Ran out of commandable frame before anything stopped the joint. Learned
    #: nothing yet — the travel continues past here, unmeasured.
    EDGE = "edge"

    #: Never got there: the probe did not reach its target within its budget.
    #: Learned nothing.
    TIMEOUT = "timeout"

    @property
    def vouches_for_a_wall(self) -> bool:
        """``True`` for :attr:`WALL` alone — the one verdict that backs a limit.

        Every "is this a mechanical limit?" question in this module routes through
        this property, so there is one place where the answer is decided.
        """
        return self is LimitVerdict.WALL


# ---------------------------------------------------------------------------
# TravelEnd — which end of the travel, since the verdict is carried PER END
# ---------------------------------------------------------------------------


class TravelEnd(str, Enum):
    """Which end of a joint's travel an observation or record describes.

    The verdict is carried **per END, not per joint**: a joint routinely has a solid
    wall one way and a torque-limited stall the other (``shoulder_lift`` again —
    gravity helps it down and fights it up), and a record that averaged those into a
    single per-joint verdict would be wrong at one end by construction.
    """

    #: The decreasing-raw-tick direction. Displacements are ``<= 0``.
    LOW = "low"

    #: The increasing-raw-tick direction. Displacements are ``>= 0``.
    HIGH = "high"

    @property
    def sign(self) -> int:
        """``+1`` for :attr:`HIGH`, ``-1`` for :attr:`LOW`.

        Multiplying an extent by this turns "further out" into "larger", so the two
        ends share one comparison instead of two mirror-image code paths.
        """
        return 1 if self is TravelEnd.HIGH else -1

    @property
    def opposite(self) -> "TravelEnd":
        """The other end of the same travel."""
        return TravelEnd.LOW if self is TravelEnd.HIGH else TravelEnd.HIGH


# ---------------------------------------------------------------------------
# EndObservation — ONE probe, ONE end, ONE pose. Evidence, not a limit.
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class EndObservation:
    """What one probe found at one END of one joint, in one POSE.

    This is **environmental** by construction: it was measured with the other joints
    where they happened to be, and an obstacle in that pose narrows it. It is
    *evidence*. Only :func:`merge_end_observations` — folding every pose together —
    can produce something that speaks about the joint's mechanics, and even then only
    when the evidence is a :attr:`LimitVerdict.WALL`.

    Attributes
    ----------
    joint:
        The joint probed. An opaque non-empty name — this module holds no joint
        table and branches on no joint (the classifier that consumes it must be able
        to say ``wrist_roll`` is continuous *because it is*, not because it is named).
    end:
        Which end of the travel (:class:`TravelEnd`).
    verdict:
        What stopped the probe (:class:`LimitVerdict`).
    origin_raw:
        The RAW encoder tick the probe started from, in ``[0, 4095]``.
    displacement:
        Signed accumulated RAW displacement from ``origin_raw``. Positive at the
        :attr:`TravelEnd.HIGH` end, negative at :attr:`TravelEnd.LOW`, zero if the
        joint stalled before it moved. Never more than one full turn in magnitude.
    load:
        Direction-independent ``present_load`` magnitude at the stop, if sampled.
        ``None`` when the verdict is one that did not involve a load reading.
    pose:
        Opaque label for the pose this was measured in. ``None`` if the caller does
        not track poses.

    Raises
    ------
    ValueError
        On an unnamed joint, an ``origin_raw`` that is not a raw tick, a displacement
        whose sign contradicts its end or that exceeds one full turn, or a negative
        load.
    """

    joint: str
    end: TravelEnd
    verdict: LimitVerdict
    origin_raw: int
    displacement: int
    load: Optional[int] = None
    pose: Optional[str] = None

    def __post_init__(self) -> None:
        if not isinstance(self.joint, str) or not self.joint:
            raise ValueError("joint must be a non-empty name.")
        object.__setattr__(self, "end", TravelEnd(self.end))
        object.__setattr__(self, "verdict", LimitVerdict(self.verdict))
        object.__setattr__(self, "origin_raw", _require_raw_tick(self.origin_raw, "origin_raw"))

        displacement = int(self.displacement)
        if displacement * self.end.sign < 0:
            raise ValueError(
                f"displacement {displacement} contradicts the {self.end.value} end it "
                f"claims to probe: a {self.end.value} end travels "
                f"{'up' if self.end is TravelEnd.HIGH else 'down'}, so its displacement "
                f"must be {'>= 0' if self.end is TravelEnd.HIGH else '<= 0'}. A probe that "
                "moved the other way did not measure this end."
            )
        if abs(displacement) > ENCODER_TICKS:
            raise ValueError(
                f"displacement {displacement} exceeds one full turn ({ENCODER_TICKS} ticks). "
                "A joint that travels a full turn without a wall is CONTINUOUS and the probe "
                "stops — there is nothing further to learn, and going round twice would put "
                "the observation more than one lap from any reference it is compared against."
            )
        object.__setattr__(self, "displacement", displacement)

        if self.load is not None:
            load = int(self.load)
            if load < 0:
                raise ValueError(f"load {load} must be non-negative — it is a magnitude.")
            object.__setattr__(self, "load", load)

    @property
    def raw_tick(self) -> int:
        """The RAW encoder tick the probe stopped at — where the joint actually got to."""
        return (self.origin_raw + self.displacement) % ENCODER_TICKS

    @property
    def is_wall(self) -> bool:
        """``True`` iff this observation is evidence of a wall."""
        return self.verdict.vouches_for_a_wall

    def extent_from(self, reference_raw: int) -> int:
        """How far out this probe got, measured from a shared *reference_raw*.

        Displacement alone is measured from this pose's OWN start, so it cannot be
        compared with another pose's. Re-expressing both against one reference — going
        the short way round the circle, so a start either side of the seam does not
        blow the comparison up — makes "which pose got furthest?" a plain ``max``.
        """
        return signed_delta(self.origin_raw, _require_raw_tick(reference_raw, "reference_raw")) + (
            self.displacement
        )

    def to_dict(self) -> Dict[str, object]:
        """Return a plain-JSON-serializable representation (no enums, no dataclasses)."""
        return {
            "joint": self.joint,
            "end": self.end.value,
            "verdict": self.verdict.value,
            "origin_raw": self.origin_raw,
            "displacement": self.displacement,
            "load": self.load,
            "pose": self.pose,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, object]) -> "EndObservation":
        """Inverse of :meth:`to_dict`."""
        load = data.get("load")
        return cls(
            joint=str(data["joint"]),
            end=TravelEnd(data["end"]),
            verdict=LimitVerdict(data["verdict"]),
            origin_raw=int(data["origin_raw"]),  # type: ignore[arg-type]
            displacement=int(data["displacement"]),  # type: ignore[arg-type]
            load=None if load is None else int(load),  # type: ignore[arg-type]
            pose=None if data.get("pose") is None else str(data["pose"]),
        )


# ---------------------------------------------------------------------------
# MeasuredEnd — the merged, per-END record. Abstract: a WALL or a LOWER BOUND.
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class MeasuredEnd:
    """One END of one joint's travel, folded over every pose it was probed in.

    **Abstract.** Constructing a bare ``MeasuredEnd`` raises :exc:`TypeError`: an end
    that holds a tick without saying whether it can *vouch* for it is exactly the
    ambiguity this module exists to remove. Every instance is a :class:`WallEnd` or a
    :class:`LowerBoundEnd`, and which one it is *is* the verdict — enforced at the
    constructor, so no merge, no deserializer and no future helper can produce a
    record whose type and evidence disagree.

    Attributes
    ----------
    joint, end:
        The joint and which end of its travel. Must agree with ``evidence``.
    reference_raw:
        The RAW tick every pose's observation was re-expressed against, so that
        ``extent`` from different poses (and from the two ends) are commensurable.
    extent:
        Signed distance from ``reference_raw`` to the furthest the joint was ever
        seen to get at this end. Not a raw tick: it is unwrapped, so it stays ordered
        across the seam. See :attr:`raw_tick` for the tick.
    evidence:
        The single :class:`EndObservation` that got furthest — the one this record
        stands on. Keeping it (rather than just its numbers) is what lets a report
        name the pose, the load and the verdict behind a limit.
    pose_count:
        How many observations were folded in. Note that a large ``pose_count`` is
        **not** evidence of anything on its own: a thousand torque-limited poses are
        still a lower bound.
    """

    joint: str
    end: TravelEnd
    reference_raw: int
    extent: int
    evidence: EndObservation
    pose_count: int = 1

    #: Discriminator written into :meth:`to_dict` and dispatched on by
    #: :meth:`from_dict`. Empty on the abstract base, which is never instantiated.
    KIND: ClassVar[str] = ""

    def __post_init__(self) -> None:
        if type(self) is MeasuredEnd:
            raise TypeError(
                "MeasuredEnd is abstract — build a WallEnd (an end the arm can vouch "
                "for) or a LowerBoundEnd (an end it cannot). An end that says neither "
                "is the ambiguity this type exists to remove."
            )
        if not isinstance(self.joint, str) or not self.joint:
            raise ValueError("joint must be a non-empty name.")
        object.__setattr__(self, "end", TravelEnd(self.end))
        object.__setattr__(
            self, "reference_raw", _require_raw_tick(self.reference_raw, "reference_raw")
        )
        object.__setattr__(self, "extent", int(self.extent))

        if not isinstance(self.evidence, EndObservation):
            raise ValueError("evidence must be an EndObservation — the probe that got furthest.")
        if self.evidence.joint != self.joint:
            raise ValueError(f"evidence is for joint {self.evidence.joint!r}, not {self.joint!r}.")
        if self.evidence.end is not self.end:
            raise ValueError(
                f"evidence is for the {self.evidence.end.value} end, not the {self.end.value} end."
            )
        if self.evidence.extent_from(self.reference_raw) != self.extent:
            raise ValueError(
                f"extent {self.extent} does not match its own evidence "
                f"({self.evidence.extent_from(self.reference_raw)} from reference "
                f"{self.reference_raw}). A record whose number and whose evidence disagree "
                "cannot be trusted by either."
            )
        if int(self.pose_count) < 1:
            raise ValueError("pose_count must be at least 1 — a record needs evidence behind it.")
        object.__setattr__(self, "pose_count", int(self.pose_count))

    # -- what the record says ------------------------------------------------

    @property
    def verdict(self) -> LimitVerdict:
        """The verdict of the observation this record stands on."""
        return self.evidence.verdict

    @property
    def raw_tick(self) -> int:
        """The RAW encoder tick this end was last seen at."""
        return (self.reference_raw + self.extent) % ENCODER_TICKS

    @property
    def reach(self) -> int:
        """The furthest RAW tick the joint was ever seen to REACH at this end.

        Always honest, whatever the verdict: the joint demonstrably got here. What it
        does *not* say is whether anything is in front of it — that is
        :attr:`mechanical_limit`'s job, and only a :class:`WallEnd` answers it.
        """
        return self.raw_tick

    @property
    def is_wall(self) -> bool:
        """``True`` only on a :class:`WallEnd`."""
        raise NotImplementedError  # pragma: no cover - abstract; base is not instantiable

    @property
    def mechanical_limit(self) -> Optional[int]:
        """The RAW tick of the joint's mechanical limit at this end, or ``None``.

        ``None`` on a :class:`LowerBoundEnd` — **always**, and by virtue of the type
        rather than a check a caller might skip. This is the single accessor any
        consumer should use to ask "is there a wall here?", and it is why a
        torque-limited end cannot be read as one.
        """
        raise NotImplementedError  # pragma: no cover - abstract; base is not instantiable

    # -- serialization -------------------------------------------------------

    def to_dict(self) -> Dict[str, object]:
        """Return a plain-JSON-serializable representation, tagged with :attr:`KIND`."""
        return {
            "kind": self.KIND,
            "joint": self.joint,
            "end": self.end.value,
            "reference_raw": self.reference_raw,
            "extent": self.extent,
            "evidence": self.evidence.to_dict(),
            "pose_count": self.pose_count,
        }

    @staticmethod
    def from_dict(data: Dict[str, object]) -> "MeasuredEnd":
        """Inverse of :meth:`to_dict`, dispatching on ``kind``.

        **The file is not a loophole.** The concrete class's own validation runs on the
        way back in, so a payload hand-edited to relabel a torque-limited end as a
        ``"wall"`` raises rather than deserializing into a :class:`WallEnd`.
        """
        kind = data.get("kind")
        cls = _KIND_TO_CLASS.get(kind)  # type: ignore[arg-type]
        if cls is None:
            known = ", ".join(sorted(repr(k) for k in _KIND_TO_CLASS))
            raise ValueError(f"unknown MeasuredEnd kind {kind!r} — expected one of {known}.")
        return cls(
            joint=str(data["joint"]),
            end=TravelEnd(data["end"]),
            reference_raw=int(data["reference_raw"]),  # type: ignore[arg-type]
            extent=int(data["extent"]),  # type: ignore[arg-type]
            evidence=EndObservation.from_dict(data["evidence"]),  # type: ignore[arg-type]
            pose_count=int(data.get("pose_count", 1)),  # type: ignore[arg-type]
        )


@dataclass(frozen=True)
class WallEnd(MeasuredEnd):
    """An end the arm CAN vouch for: a wall was found, and this is the widest one.

    Sound because an obstacle can only ever make a range *smaller* — so the maximum
    over poses converges on the mechanical truth, and the widest wall ever seen is the
    best estimate of the joint's mechanical limit at this end.

    **Cannot hold non-WALL evidence.** Handing this constructor a torque-limited stall
    raises :exc:`ValueError`. That refusal is the whole mechanism: there is no path —
    no merge, no promotion, no deserializer, no helper anyone adds later — by which
    the arm's own weakness gets recorded as a wall, because the type will not carry it.
    """

    KIND: ClassVar[str] = "wall"

    def __post_init__(self) -> None:
        super().__post_init__()
        if not self.evidence.verdict.vouches_for_a_wall:
            raise ValueError(
                f"a WallEnd needs WALL evidence; got {self.evidence.verdict.value.upper()}. "
                "Only a load saturated against something solid vouches for a mechanical "
                "limit — a stall against the torque cap, an exhausted frame and a probe "
                "that never arrived are all LOWER BOUNDS. Build a LowerBoundEnd."
            )

    @property
    def is_wall(self) -> bool:
        """Always ``True``."""
        return True

    @property
    def mechanical_limit(self) -> Optional[int]:
        """The RAW tick of the wall — the joint's mechanical limit at this end."""
        return self.raw_tick


@dataclass(frozen=True)
class LowerBoundEnd(MeasuredEnd):
    """An end the arm CANNOT vouch for: it got at least this far, and no wall was found.

    Produced by :attr:`LimitVerdict.TORQUE_LIMITED` (the arm was too weak),
    :attr:`LimitVerdict.EDGE` (it ran out of frame) and :attr:`LimitVerdict.TIMEOUT`
    (it never arrived) — three quite different reasons to have learned nothing, kept
    apart in :attr:`verdict` so a report can say which.

    :attr:`mechanical_limit` is ``None``, permanently. In particular a TORQUE-LIMITED
    end does **not** become a wall by being observed in more poses: the arm's weakness
    is not an obstacle you can pose your way out of — it shrinks the range in *every*
    pose — so the max-over-poses rule that promotes a wall has nothing to bite on here.
    The value under-claims the arm's reach, which is the correct direction to be wrong.

    **Cannot hold WALL evidence** — the two types partition the verdict space between
    them, so every :class:`MeasuredEnd` is exactly one of them.
    """

    KIND: ClassVar[str] = "lower_bound"

    def __post_init__(self) -> None:
        super().__post_init__()
        if self.evidence.verdict.vouches_for_a_wall:
            raise ValueError(
                "a LowerBoundEnd cannot hold WALL evidence — a wall vouches for a "
                "mechanical limit, so it belongs in a WallEnd. Recording it as a mere "
                "lower bound would throw away the one verdict that actually measured "
                "something."
            )

    @property
    def is_wall(self) -> bool:
        """Always ``False``."""
        return False

    @property
    def mechanical_limit(self) -> Optional[int]:
        """Always ``None`` — there is no wall here to report."""
        return None


#: ``kind`` discriminator -> concrete class, for :meth:`MeasuredEnd.from_dict`.
_KIND_TO_CLASS: Dict[str, type] = {
    WallEnd.KIND: WallEnd,
    LowerBoundEnd.KIND: LowerBoundEnd,
}


# ---------------------------------------------------------------------------
# JointTravel — the two ends of one joint
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class JointTravel:
    """A joint's measured travel: two :class:`MeasuredEnd`\\ s, one per direction.

    The record a consumer reads. Note what it does and does not offer:

    * :attr:`envelope_extent` and :attr:`span` are **always** available. They describe
      what was OBSERVED, and their names claim nothing more.
    * :attr:`mechanical_extent` and :attr:`mechanical_raw_ends` are ``None`` unless
      **both** ends are :class:`WallEnd`\\ s. One torque-limited end is enough to
      withhold the pair — because a ``(min, max)`` handed to ``arm_spec`` is
      indistinguishable from a measured wall once it lands there, and that is precisely
      the lie this record exists to prevent.

    The classification that follows from all this — BOUNDED vs CONTINUOUS — is
    deliberately *not* here. It is the classifier's job, and it must reach it from the
    measurement alone.
    """

    joint: str
    low: MeasuredEnd
    high: MeasuredEnd

    def __post_init__(self) -> None:
        if not isinstance(self.joint, str) or not self.joint:
            raise ValueError("joint must be a non-empty name.")
        for name, record in (("low", self.low), ("high", self.high)):
            if not isinstance(record, MeasuredEnd):
                raise ValueError(f"{name} must be a MeasuredEnd (a WallEnd or a LowerBoundEnd).")
            if record.joint != self.joint:
                raise ValueError(
                    f"the {name} end belongs to joint {record.joint!r}, not {self.joint!r}."
                )
        if self.low.end is not TravelEnd.LOW:
            raise ValueError(
                f"the low end of a JointTravel must be a LOW end; got {self.low.end.value!r}."
            )
        if self.high.end is not TravelEnd.HIGH:
            raise ValueError(
                f"the high end of a JointTravel must be a HIGH end; got {self.high.end.value!r}."
            )
        if self.low.reference_raw != self.high.reference_raw:
            raise ValueError(
                f"the two ends were measured against different reference frames "
                f"({self.low.reference_raw} vs {self.high.reference_raw}); their extents are "
                "not commensurable. Merge them with merge_joint_travel, which shares one."
            )
        if self.low.extent > self.high.extent:
            raise ValueError(
                f"the low end ({self.low.extent}) sits above the high end "
                f"({self.high.extent}) — that is not a travel, it is a bug."
            )

    # -- what was OBSERVED (always available; claims nothing it cannot back) --

    @property
    def envelope_extent(self) -> Tuple[int, int]:
        """``(low, high)`` extents of the widest envelope ever OBSERVED.

        Unwrapped and relative to the shared reference, so the pair is always ordered
        even when the travel crosses the seam. Available whatever the verdicts — an
        observed envelope is a fact regardless of what stopped the probe.
        """
        return (self.low.extent, self.high.extent)

    @property
    def span(self) -> int:
        """Total observed travel, in ticks. Always ``>= 0``."""
        return self.high.extent - self.low.extent

    @property
    def unvouched_ends(self) -> Tuple[TravelEnd, ...]:
        """The ends with no wall behind them — the ones that are lower bounds."""
        return tuple(
            record.end for record in (self.low, self.high) if record.mechanical_limit is None
        )

    @property
    def is_fully_vouched(self) -> bool:
        """``True`` iff BOTH ends found a wall."""
        return not self.unvouched_ends

    # -- what can be VOUCHED for (None unless both ends are walls) -----------

    @property
    def mechanical_raw_ends(self) -> Optional[Tuple[int, int]]:
        """``(low, high)`` RAW ticks of the joint's mechanical limits, or ``None``.

        ``None`` unless both ends are walls. The ``None`` is not a guard bolted on
        here — it falls out of :attr:`MeasuredEnd.mechanical_limit`, which is ``None``
        on a :class:`LowerBoundEnd` by type.

        May wrap (``low > high`` as raw ticks) when the travel crosses the seam. That
        is not a defect: it is why a joint's travel cannot be described by a sorted
        ``[min, max]`` pair, and why the seam has to be evicted or fenced off.
        """
        low, high = self.low.mechanical_limit, self.high.mechanical_limit
        if low is None or high is None:
            return None
        return (low, high)

    @property
    def mechanical_extent(self) -> Optional[Tuple[int, int]]:
        """:attr:`envelope_extent`, but only when both ends are walls — else ``None``."""
        if self.mechanical_raw_ends is None:
            return None
        return self.envelope_extent

    # -- serialization -------------------------------------------------------

    def to_dict(self) -> Dict[str, object]:
        """Return a plain-JSON-serializable representation of this travel."""
        return {
            "joint": self.joint,
            "low": self.low.to_dict(),
            "high": self.high.to_dict(),
        }

    @classmethod
    def from_dict(cls, data: Dict[str, object]) -> "JointTravel":
        """Inverse of :meth:`to_dict`."""
        return cls(
            joint=str(data["joint"]),
            low=MeasuredEnd.from_dict(data["low"]),  # type: ignore[arg-type]
            high=MeasuredEnd.from_dict(data["high"]),  # type: ignore[arg-type]
        )


# ---------------------------------------------------------------------------
# The merge — the ONE place per-pose evidence becomes a per-joint record
# ---------------------------------------------------------------------------


def merge_end_observations(
    observations: Sequence[EndObservation],
    reference_raw: Optional[int] = None,
) -> MeasuredEnd:
    """Fold every pose's observation of ONE end of ONE joint into the widest envelope.

    **The rule: the OUTERMOST observation decides, and only a WALL can vouch.**

    Both halves of that matter, and the second is the whole point of the module:

    * *Outermost decides.* If pose A hit a wall 400 ticks out but pose B *stalled* 700
      ticks out, then the joint demonstrably travelled past pose A's wall — so that
      wall was an obstacle (environmental), not the mechanical limit, and promoting it
      would record a limit the arm has already been seen to cross. The end is a lower
      bound at 700.
    * *Only a WALL vouches.* If the outermost observation is TORQUE-LIMITED, EDGE or
      TIMEOUT, the result is a :class:`LowerBoundEnd` — **no matter how many poses were
      recorded**. The max-over-poses rule that legitimately promotes a wall has nothing
      to bite on: the arm's weakness narrows the range in every pose, so the maximum
      never escapes it.

    A tie at the same extent goes to the WALL: it is the stronger evidence, the stall
    is consistent with the wall being exactly there, and the choice is deterministic.

    Every observation is re-expressed against a single *reference_raw* (defaulting to
    the first observation's ``origin_raw``) before being compared, since displacement
    is measured from each pose's own start. Pass *reference_raw* explicitly to share one
    frame across both ends of a joint — :func:`merge_joint_travel` does exactly that.

    Raises
    ------
    ValueError
        If *observations* is empty, or mixes joints, or mixes ends. Each of those is a
        caller bug that would otherwise yield a record quietly averaging things that
        are not comparable.
    """
    records: List[EndObservation] = list(observations)
    if not records:
        raise ValueError(
            "cannot merge no observations — a record with no evidence behind it is the "
            "purest form of the laundering this module exists to prevent."
        )

    joints = {record.joint for record in records}
    if len(joints) != 1:
        raise ValueError(
            f"every observation must be for the same joint; got {sorted(joints)}. "
            "Ranges from different joints are not poses of one another."
        )
    ends = {record.end for record in records}
    if len(ends) != 1:
        raise ValueError(
            f"every observation must be for the same end; got "
            f"{sorted(end.value for end in ends)}. The verdict is carried PER END — "
            "folding two ends together is what this record exists to stop."
        )

    end = records[0].end
    reference = (
        records[0].origin_raw
        if reference_raw is None
        else _require_raw_tick(reference_raw, "reference_raw")
    )

    def outwardness(record: EndObservation) -> Tuple[int, bool]:
        # Further out wins; on a tie, the WALL wins (True > False).
        return (record.extent_from(reference) * end.sign, record.is_wall)

    winner = max(records, key=outwardness)
    cls = WallEnd if winner.is_wall else LowerBoundEnd
    return cls(
        joint=winner.joint,
        end=end,
        reference_raw=reference,
        extent=winner.extent_from(reference),
        evidence=winner,
        pose_count=len(records),
    )


def merge_joint_travel(observations: Iterable[EndObservation]) -> JointTravel:
    """Fold every pose's observations of BOTH ends of one joint into a :class:`JointTravel`.

    Both ends are merged against **one shared reference frame** (the first
    observation's ``origin_raw``), so that the two extents — and therefore
    :attr:`JointTravel.span` — are commensurable.

    Raises
    ------
    ValueError
        If *observations* is empty, mixes joints, or does not cover both ends. A travel
        with one end missing is not a travel; leaving the other end to be inferred is
        how "we never probed it" turns into "there is nothing there".
    """
    records: List[EndObservation] = list(observations)
    if not records:
        raise ValueError("cannot merge no observations into a JointTravel.")

    joints = {record.joint for record in records}
    if len(joints) != 1:
        raise ValueError(f"every observation must be for the same joint; got {sorted(joints)}.")

    by_end: Dict[TravelEnd, List[EndObservation]] = {TravelEnd.LOW: [], TravelEnd.HIGH: []}
    for record in records:
        by_end[record.end].append(record)
    missing = [end.value for end, found in by_end.items() if not found]
    if missing:
        raise ValueError(
            f"a joint's travel needs observations at both ends; the {', '.join(missing)} "
            "end has none. An end that was never probed is not an end with nothing there."
        )

    reference = records[0].origin_raw
    return JointTravel(
        joint=records[0].joint,
        low=merge_end_observations(by_end[TravelEnd.LOW], reference_raw=reference),
        high=merge_end_observations(by_end[TravelEnd.HIGH], reference_raw=reference),
    )


__all__ = [
    "ENCODER_TICKS",
    "HALF_TURN",
    "EndObservation",
    "JointTravel",
    "LimitVerdict",
    "LowerBoundEnd",
    "MeasuredEnd",
    "TravelEnd",
    "WallEnd",
    "merge_end_observations",
    "merge_joint_travel",
    "signed_delta",
]
