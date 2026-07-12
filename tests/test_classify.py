"""Tests for arm101.hardware.classify — BOUNDED, CONTINUOUS, or honestly UNDETERMINED (t7).

The contract this file pins:

* **Two walls => BOUNDED.** An unreachable arc exists, so a re-zero can evict the
  seam into it.
* **A full turn in one direction with no wall => CONTINUOUS.** The travel covers
  every angle, so there is no angle left to put the seam at: no offset can ever
  help, in principle. A soft limit is the only instrument.
* **Everything in between is UNDETERMINED** — one wall and one torque-limited end,
  two torque-limited ends, an EDGE. You cannot site an arc without two walls you
  can vouch for, and you have not seen a full turn. Saying so is the honest answer.
* **NO JOINT NAME appears anywhere in the classifier's logic.** ``wrist_roll`` must
  come back CONTINUOUS because it *is*, not because it is named — and
  ``elbow_flex`` BOUNDED because it *is*. That is what makes the hardware
  acceptance test (re-deriving what a long human session established) non-vacuous.
  Enforced three ways here: an AST sweep of the module's executable code, an AST
  check that it consults no joint-keyed table, and a behavioural check that the
  same measurement classifies the same under *every* joint's name — including the
  two swapped.
* **Termination is by accumulated raw displacement, never by "we got back to where
  we started".** Two travels that end on the same raw tick classify differently.

Every expectation below is DERIVED from ``arm_spec`` / ``ticks`` / ``limits``.
Change a constant there and this suite stays green.
"""

from __future__ import annotations

import ast
import contextlib
import inspect
import json
import subprocess  # nosec B404 — used only to prove an import graph, with a fixed argv
import sys

import pytest

from arm101.hardware import arm_spec
from arm101.hardware.arm_spec import UnreachableArc
from arm101.hardware.classify import (
    ARC_MARGIN_TICKS,
    MIN_EVICTABLE_ARC_TICKS,
    SeamRemedy,
    TravelClassification,
    TravelKind,
    classify_observations,
    classify_travel,
)
from arm101.hardware.limits import (
    ENCODER_TICKS,
    EndObservation,
    JointTravel,
    LimitVerdict,
    TravelEnd,
    merge_joint_travel,
)
from arm101.hardware.ticks import MAX_ENCODER_OFFSET, TICK_MAX

# ---------------------------------------------------------------------------
# Helpers — every travel here is built from RAW ticks and DISPLACEMENTS, the
# only two things the probe actually records.
# ---------------------------------------------------------------------------

#: A deliberately meaningless joint name. The classifier must not care, and the
#: tests that DO name a real joint below do it to prove the classifier ignores it.
ANON = "a_joint"

#: The three verdicts that are not a wall — none of them can site an arc.
NOT_A_WALL = (
    LimitVerdict.TORQUE_LIMITED,
    LimitVerdict.EDGE,
    LimitVerdict.TIMEOUT,
)

#: The raw tick the register cannot place a seam at: +2048 overflows the 11-bit
#: sign-magnitude field and -2048 does too. Derived, not typed.
UNREPRESENTABLE_SEAM_TICK = MAX_ENCODER_OFFSET + 1


def travel(
    *,
    joint: str = ANON,
    origin_raw: int = 2048,
    low_displacement: int = -500,
    low_verdict: LimitVerdict = LimitVerdict.WALL,
    high_displacement: int = 500,
    high_verdict: LimitVerdict = LimitVerdict.WALL,
) -> JointTravel:
    """One joint's travel, probed outward from *origin_raw* to both ends."""
    return merge_joint_travel(
        [
            EndObservation(
                joint=joint,
                end=TravelEnd.LOW,
                verdict=low_verdict,
                origin_raw=origin_raw,
                displacement=low_displacement,
            ),
            EndObservation(
                joint=joint,
                end=TravelEnd.HIGH,
                verdict=high_verdict,
                origin_raw=origin_raw,
                displacement=high_displacement,
            ),
        ]
    )


def travel_leaving_arc(low: int, high: int, *, joint: str = ANON) -> JointTravel:
    """A WALL-at-both-ends travel whose measured walls leave exactly ``(low, high)`` unreachable.

    The joint's travel therefore WRAPS the raw seam: driven upward it stops at
    *low* (the arc's bottom edge) having crossed 4095 -> 0 on the way; driven
    downward it stops at *high*. This is ``elbow_flex``'s shape, and the reason a
    ``[min, max]`` pair cannot describe it — sorting the two walls gives precisely
    the arc the joint CANNOT reach.
    """
    low_wall_raw, high_wall_raw = high, low
    span = ENCODER_TICKS - (high - low)
    origin_raw = (low_wall_raw + span // 2) % ENCODER_TICKS
    return merge_joint_travel(
        [
            EndObservation(
                joint=joint,
                end=TravelEnd.LOW,
                verdict=LimitVerdict.WALL,
                origin_raw=origin_raw,
                displacement=-((origin_raw - low_wall_raw) % ENCODER_TICKS),
            ),
            EndObservation(
                joint=joint,
                end=TravelEnd.HIGH,
                verdict=LimitVerdict.WALL,
                origin_raw=origin_raw,
                displacement=(high_wall_raw - origin_raw) % ENCODER_TICKS,
            ),
        ]
    )


def a_full_turn(*, joint: str = ANON, verdict: LimitVerdict = LimitVerdict.EDGE) -> JointTravel:
    """A joint driven a full turn one way with nothing stopping it. No wall anywhere."""
    return travel(
        joint=joint,
        origin_raw=0,
        low_displacement=0,
        low_verdict=verdict,
        high_displacement=ENCODER_TICKS,
        high_verdict=verdict,
    )


#: The joint the hardware proved BOUNDED — re-zeroed, and its arc is in arm_spec.
REZEROED_JOINT = "elbow_flex"

#: The joint the hardware proved CONTINUOUS — soft-limited, re-zero impossible.
FREE_JOINT = "wrist_roll"


def the_measured_arc() -> UnreachableArc:
    """The unreachable arc the ARM measured, before ``arm_spec`` insets its margins.

    ``REZERO_ARCS`` stores the arc already inset by ``_ARC_MARGIN_TICKS`` on each
    side (that inset is the whole point — a wall is not a crisp number). Undo the
    inset and you have the walls the probe actually reported, which is the input
    the classifier is handed. Derived from the table, so a re-measurement moves
    this with it.
    """
    declared = arm_spec.rezero_arc(REZEROED_JOINT)
    assert declared is not None
    return UnreachableArc(
        low=declared.low - ARC_MARGIN_TICKS,
        high=declared.high + ARC_MARGIN_TICKS,
    )


# ---------------------------------------------------------------------------
# Criterion 1a — TWO WALLS => BOUNDED
# ---------------------------------------------------------------------------


def test_a_wall_at_both_ends_is_bounded():
    """Two walls: the travel is known exactly, so its complement is an arc."""
    result = classify_travel(travel(low_displacement=-500, high_displacement=500))

    assert result.kind is TravelKind.BOUNDED
    assert result.swept_ticks == 1000
    assert result.unreachable_width == ENCODER_TICKS - 1000
    assert result.reachable_raw_ends == (2048 - 500, 2048 + 500)


def test_a_bounded_joint_s_unreachable_width_is_the_rest_of_the_circle():
    """The arc is the complement of the travel: every tick the joint cannot reach."""
    for span in (10, 1000, ENCODER_TICKS - 1):
        result = classify_travel(travel(low_displacement=0, high_displacement=span))
        assert result.kind is TravelKind.BOUNDED
        assert result.unreachable_width == ENCODER_TICKS - span


def test_a_bounded_joint_that_wraps_the_seam_can_be_rezeroed():
    """The arc exists, it can carry the seam, and the offset that puts it there evicts it."""
    measured = the_measured_arc()
    result = classify_travel(travel_leaving_arc(measured.low, measured.high))

    assert result.kind is TravelKind.BOUNDED
    assert result.seam_in_travel is True
    assert result.can_be_rezeroed is True
    assert result.remedy is SeamRemedy.REZERO

    arc = result.unreachable_arc
    assert arc is not None
    assert arc.evicts(arc.offset)
    assert result.rezero_offset == arc.offset


# ---------------------------------------------------------------------------
# Criterion 1b — A FULL TURN WITH NO WALL => CONTINUOUS
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("verdict", NOT_A_WALL)
def test_a_full_turn_with_no_wall_is_continuous(verdict):
    """4096 ticks in one direction and nothing stopped it: every angle is reachable."""
    result = classify_travel(a_full_turn(verdict=verdict))

    assert result.kind is TravelKind.CONTINUOUS
    assert result.swept_ticks >= ENCODER_TICKS


def test_a_continuous_joint_has_no_arc_to_evict_the_seam_into():
    """Not "we don't have an arc for it" — there IS no arc. The seam has nowhere to go."""
    result = classify_travel(a_full_turn())

    assert result.unreachable_arc is None
    assert result.unreachable_width is None
    assert result.can_be_rezeroed is False
    assert result.rezero_offset is None
    assert result.seam_in_travel is True
    assert result.remedy is SeamRemedy.SOFT_LIMIT


def test_one_tick_short_of_a_full_turn_is_not_continuous():
    """The threshold is a FULL turn. A joint that swept 4095 ticks has one angle left."""
    result = classify_travel(
        travel(
            origin_raw=0,
            low_displacement=0,
            low_verdict=LimitVerdict.EDGE,
            high_displacement=ENCODER_TICKS - 1,
            high_verdict=LimitVerdict.EDGE,
        )
    )
    assert result.kind is not TravelKind.CONTINUOUS


def test_walls_more_than_a_full_turn_apart_are_still_continuous():
    """A joint with hard stops that still over-rotates past 360 deg reaches EVERY angle.

    So there is no angle to put the seam at, and no offset can help — which is what
    CONTINUOUS *means*. The rule is about coverage of the circle, not about the
    absence of walls, and a classifier keyed on "no wall" would get this one wrong.
    """
    result = classify_travel(
        travel(
            origin_raw=0,
            low_displacement=-ENCODER_TICKS // 2,
            high_displacement=ENCODER_TICKS // 2,
        )
    )
    assert result.kind is TravelKind.CONTINUOUS
    assert result.can_be_rezeroed is False


# ---------------------------------------------------------------------------
# Criterion 1c — THE CASES IN BETWEEN. Not BOUNDED, not CONTINUOUS: UNDETERMINED.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("verdict", NOT_A_WALL)
def test_a_wall_at_one_end_and_a_non_wall_at_the_other_is_undetermined(verdict):
    """One wall is not two. You cannot site an arc without two ends you can vouch for."""
    assert classify_travel(travel(high_verdict=verdict)).kind is TravelKind.UNDETERMINED
    assert classify_travel(travel(low_verdict=verdict)).kind is TravelKind.UNDETERMINED


@pytest.mark.parametrize("low_verdict", NOT_A_WALL)
@pytest.mark.parametrize("high_verdict", NOT_A_WALL)
def test_two_non_wall_ends_are_undetermined(low_verdict, high_verdict):
    """Two lower bounds are still two lower bounds — no number of them makes a wall."""
    result = classify_travel(travel(low_verdict=low_verdict, high_verdict=high_verdict))
    assert result.kind is TravelKind.UNDETERMINED


def test_an_undetermined_joint_claims_nothing_it_cannot_back():
    """No arc, no width, no walls, no remedy — and it says WHY, per end."""
    result = classify_travel(travel(high_verdict=LimitVerdict.TORQUE_LIMITED))

    assert result.unreachable_arc is None
    assert result.unreachable_width is None
    assert result.reachable_raw_ends is None
    assert result.can_be_rezeroed is False
    assert result.remedy is SeamRemedy.UNKNOWN
    assert LimitVerdict.TORQUE_LIMITED.value in result.reason


def test_undetermined_does_not_pretend_to_know_where_the_seam_is():
    """The unmeasured part of the travel may well contain the seam. ``None`` says so."""
    result = classify_travel(travel(high_verdict=LimitVerdict.EDGE))
    assert result.seam_in_travel is None


def test_undetermined_still_reports_an_observed_seam_crossing_as_a_fact():
    """If the probe DID cross the seam, that is a fact regardless of what stopped it."""
    result = classify_travel(
        travel(
            origin_raw=TICK_MAX,
            low_displacement=-100,
            low_verdict=LimitVerdict.TORQUE_LIMITED,
            high_displacement=100,
            high_verdict=LimitVerdict.TORQUE_LIMITED,
        )
    )
    assert result.kind is TravelKind.UNDETERMINED
    assert result.seam_in_travel is True


# ---------------------------------------------------------------------------
# Criterion 2 — NO JOINT NAME IN THE LOGIC. Three independent proofs.
# ---------------------------------------------------------------------------


def _executable_ast(module) -> ast.Module:
    """The module's AST with every docstring removed — i.e. its LOGIC.

    Comments never reach the AST, and docstrings are stripped here, so what remains
    is exactly the code that runs. The classifier is free to *explain* wrist_roll and
    elbow_flex in prose; what it must never do is *branch* on them.
    """
    tree = ast.parse(inspect.getsource(module))
    for node in ast.walk(tree):
        if not isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        body = node.body
        if (
            body
            and isinstance(body[0], ast.Expr)
            and isinstance(body[0].value, ast.Constant)
            and isinstance(body[0].value.value, str)
        ):
            node.body = body[1:]
    return tree


def _spoken_names(tree: ast.Module) -> set[str]:
    """Every name and string literal the code utters — identifiers, attributes, constants."""
    spoken: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            spoken.add(node.value)
        elif isinstance(node, ast.Name):
            spoken.add(node.id)
        elif isinstance(node, ast.Attribute):
            spoken.add(node.attr)
        elif isinstance(node, ast.arg):
            spoken.add(node.arg)
        elif isinstance(node, ast.keyword) and node.arg:
            spoken.add(node.arg)
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            spoken.add(node.name)
        elif isinstance(node, ast.alias):
            spoken.add(node.name)
            if node.asname:
                spoken.add(node.asname)
    return spoken


def test_the_classifier_s_logic_names_no_joint():
    """THE point of the whole task, asserted against the module's own AST.

    A special case for the free joint would make the hardware acceptance test
    vacuous: the verb would "re-derive" what it was told. So the executable code may
    not utter a joint's name — not as a literal, not as an identifier, not as an
    attribute. The joint names come from ``arm_spec.JOINTS``, so adding a seventh
    joint extends this test for free.
    """
    import arm101.hardware.classify as classify_module

    spoken = _spoken_names(_executable_ast(classify_module))
    offenders = {
        name: joint for name in spoken for joint in arm_spec.JOINTS if joint.lower() in name.lower()
    }
    assert not offenders, (
        f"the classifier's logic names a joint: {offenders}. It must reach BOUNDED / "
        "CONTINUOUS from the measurement alone — a joint name in the code is the verb "
        "reciting the answer it was supposed to re-derive."
    )


def _arm_spec_members_used(tree: ast.Module) -> set[str]:
    """Every ``arm_spec`` member the code imports or reaches through the module object."""
    module_name = arm_spec.__name__
    package, _, leaf = module_name.rpartition(".")

    aliases: set[str] = set()
    used: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module == module_name:
            used.update(alias.name for alias in node.names)  # from ...arm_spec import X
        elif isinstance(node, ast.ImportFrom) and node.module == package:
            aliases.update(  # from arm101.hardware import arm_spec [as s]
                alias.asname or alias.name for alias in node.names if alias.name == leaf
            )
        elif isinstance(node, ast.Import):
            aliases.update(  # import arm101.hardware.arm_spec as s
                alias.asname for alias in node.names if alias.name == module_name and alias.asname
            )

    for node in ast.walk(tree):
        if (
            isinstance(node, ast.Attribute)
            and isinstance(node.value, ast.Name)
            and node.value.id in aliases
        ):
            used.add(node.attr)
    return used


def test_the_classifier_consults_nothing_keyed_by_a_joint():
    """Not naming a joint is not enough: it must not READ anything keyed by one.

    ``REZERO_ARCS``, ``SOFT_LIMITS`` and friends hold answers a human already worked
    out, and ``rezero_arc(joint)`` / ``rezero_refusal(joint)`` hand them straight
    back — the same cheat by a different route, and one the name sweep above would
    not see. Both forbidden sets are DERIVED from ``arm_spec`` itself: every dict
    keyed by a joint, and every callable that takes one. Add a seventh joint, or a
    new per-joint helper, and this test extends itself.

    What the classifier IS allowed from ``arm_spec`` is its joint-agnostic geometry —
    the ``UnreachableArc`` type and the margin it insets by. Those hold no answers.
    """
    import arm101.hardware.classify as classify_module

    joint_keyed = {
        name
        for name, value in vars(arm_spec).items()
        if isinstance(value, dict) and set(value) & set(arm_spec.JOINTS)
    }
    for name, value in vars(arm_spec).items():
        if not callable(value) or isinstance(value, type):
            continue
        with contextlib.suppress(TypeError, ValueError):
            if "joint" in inspect.signature(value).parameters:
                joint_keyed.add(name)

    assert {"REZERO_ARCS", "rezero_arc"} <= joint_keyed, "expected arm_spec's own per-joint API"

    used = _arm_spec_members_used(_executable_ast(classify_module))
    assert used, "expected the classifier to use arm_spec's joint-agnostic geometry"
    assert not (used & joint_keyed), (
        f"the classifier consults something keyed by a joint: {sorted(used & joint_keyed)}. "
        "Those hold answers a human already worked out; the classifier's job is to reach "
        "them from measurement."
    )


@pytest.mark.parametrize("joint", (*arm_spec.JOINTS, ANON, "", " "))
def test_the_classification_is_invariant_under_the_joint_s_name(joint):
    """The same measurement classifies the same whatever the joint is called.

    The behavioural twin of the AST test above, and the one that would catch a
    special case smuggled in through a lookup the AST test did not think of.
    """
    name = joint or "unnamed"
    assert classify_travel(travel(joint=name)).kind is TravelKind.BOUNDED
    assert classify_travel(a_full_turn(joint=name)).kind is TravelKind.CONTINUOUS
    assert (
        classify_travel(travel(joint=name, high_verdict=LimitVerdict.EDGE)).kind
        is TravelKind.UNDETERMINED
    )


def test_the_free_joint_would_come_back_bounded_if_it_had_walls():
    """Name it ``wrist_roll`` and give it two walls: the classifier says BOUNDED.

    Which is the strongest form of criterion 2. The classifier reads the
    measurement, not the nameplate — so when the real ``wrist_roll`` comes back
    CONTINUOUS on the physical arm, that is a measurement, not a recitation.
    """
    measured = the_measured_arc()
    result = classify_travel(travel_leaving_arc(measured.low, measured.high, joint=FREE_JOINT))

    assert result.kind is TravelKind.BOUNDED
    assert result.can_be_rezeroed is True


def test_the_rezeroed_joint_would_come_back_continuous_if_it_turned_freely():
    """And the inverse: ``elbow_flex``, driven a full turn with no wall, is CONTINUOUS."""
    result = classify_travel(a_full_turn(joint=REZEROED_JOINT))

    assert result.kind is TravelKind.CONTINUOUS
    assert result.can_be_rezeroed is False
    assert result.remedy is SeamRemedy.SOFT_LIMIT


# ---------------------------------------------------------------------------
# The acceptance shape (t12, on the physical arm) — re-derive what is already known.
# ---------------------------------------------------------------------------


def test_the_arm_s_own_wall_measurements_reproduce_arm_spec_s_declared_arc():
    """Feed the classifier the walls the ARM measured; it must emit arm_spec's arc EXACTLY.

    ``REZERO_ARCS`` was written by a human from a long hardware session. This test
    hands the classifier the same two wall positions and demands the same arc back —
    same edges, same midpoint, same offset. That is what "it re-derives it from
    measurement alone" has to mean, and it is checked against the table rather than
    against copied numbers, so a re-measurement moves both together.
    """
    declared = arm_spec.rezero_arc(REZEROED_JOINT)
    measured = the_measured_arc()

    result = classify_travel(travel_leaving_arc(measured.low, measured.high))

    assert result.kind is TravelKind.BOUNDED
    assert result.unreachable_arc == declared
    assert result.rezero_offset == declared.offset
    assert result.unreachable_width == measured.width
    assert result.remedy is SeamRemedy.REZERO


def test_the_free_joint_s_full_turn_reproduces_arm_spec_s_impossibility():
    """A joint measured free all the way round: arm_spec refuses to re-zero it, and so do we.

    ``rezero_arc`` returns ``None`` for it and ``rezero_refusal`` explains that no
    offset can help. The classifier reaches the same verdict from the sweep alone.
    """
    assert arm_spec.rezero_arc(FREE_JOINT) is None
    assert arm_spec.rezero_refusal(FREE_JOINT) is not None

    result = classify_travel(a_full_turn(joint=FREE_JOINT))

    assert result.kind is TravelKind.CONTINUOUS
    assert result.can_be_rezeroed is False
    assert result.remedy is SeamRemedy.SOFT_LIMIT


# ---------------------------------------------------------------------------
# Criterion 3 — accumulated DISPLACEMENT decides, never "we're back where we started".
# ---------------------------------------------------------------------------


def test_a_full_turn_and_a_dead_stall_end_on_the_same_raw_tick_and_classify_differently():
    """Both end where they began. One swept the circle; the other never moved.

    A classifier that watched the reported POSITION could not tell these apart —
    which is exactly the trap, because "we got back to where we started" is true of
    both. Accumulated displacement separates them.
    """
    swept_the_circle = travel(
        origin_raw=0,
        low_displacement=0,
        low_verdict=LimitVerdict.EDGE,
        high_displacement=ENCODER_TICKS,
        high_verdict=LimitVerdict.EDGE,
    )
    never_budged = travel(
        origin_raw=0,
        low_displacement=0,
        low_verdict=LimitVerdict.WALL,
        high_displacement=0,
        high_verdict=LimitVerdict.WALL,
    )

    assert swept_the_circle.high.reach == never_budged.high.reach
    assert swept_the_circle.low.reach == never_budged.low.reach

    assert classify_travel(swept_the_circle).kind is TravelKind.CONTINUOUS
    assert classify_travel(never_budged).kind is TravelKind.BOUNDED


def test_a_travel_that_crosses_the_seam_is_measured_by_displacement_not_by_ticks():
    """Raw 4000 + 200 ticks = raw 104. Subtract the ticks and you get a 3896-tick retreat.

    The seam wearing a limit's clothes. The classifier must see a 300-tick travel.
    """
    result = classify_travel(travel(origin_raw=4000, low_displacement=-100, high_displacement=200))

    assert result.kind is TravelKind.BOUNDED
    assert result.swept_ticks == 300
    assert result.unreachable_width == ENCODER_TICKS - 300
    assert result.reachable_raw_ends == (3900, 104)
    assert result.seam_in_travel is True


def test_the_span_of_a_wrapping_travel_is_not_the_difference_of_its_raw_ticks():
    """The pair that a [min, max] would sort into the arc the joint CANNOT reach."""
    measured = the_measured_arc()
    result = classify_travel(travel_leaving_arc(measured.low, measured.high))

    low_raw, high_raw = result.reachable_raw_ends
    assert high_raw < low_raw  # the travel wraps: sorting these two inverts it
    assert result.swept_ticks == ENCODER_TICKS - measured.width


# ---------------------------------------------------------------------------
# The narrow-arc cutoff — an arc too tight to hold the seam is soft-limit territory.
# ---------------------------------------------------------------------------


def test_the_cutoff_is_a_margin_at_each_wall_plus_a_tick_to_put_the_seam_on():
    """The rule, stated as arithmetic over the constants it is built from.

    ``ARC_MARGIN_TICKS`` of clearance from EACH measured wall (arm_spec's own inset —
    a wall is not a crisp number, and at least one of elbow_flex's was the table
    rather than the joint), leaving an arc with at least one tick STRICTLY inside it
    to put the seam on. Anything narrower cannot be re-zeroed with the margins
    arm_spec already insists on.
    """
    assert ARC_MARGIN_TICKS == arm_spec._ARC_MARGIN_TICKS
    assert MIN_EVICTABLE_ARC_TICKS == 2 * ARC_MARGIN_TICKS + 2


def test_an_arc_exactly_at_the_cutoff_still_takes_the_seam():
    """The narrowest arc we will re-zero into: the seam gets a full margin either side."""
    low = 1000
    result = classify_travel(travel_leaving_arc(low, low + MIN_EVICTABLE_ARC_TICKS))

    arc = result.unreachable_arc
    assert result.kind is TravelKind.BOUNDED
    assert arc is not None
    assert arc.evicts(arc.offset)
    assert result.remedy is SeamRemedy.REZERO
    # The seam sits at least a full margin clear of BOTH measured walls.
    assert arc.midpoint - low >= ARC_MARGIN_TICKS
    assert (low + MIN_EVICTABLE_ARC_TICKS) - arc.midpoint >= ARC_MARGIN_TICKS


def test_an_arc_one_tick_under_the_cutoff_is_soft_limit_territory():
    """A sliver. The joint is still BOUNDED — we just cannot re-zero it into that.

    This is the failure that broke the first elbow_flex re-zero: the joint came to
    rest just past an edge taken from a sweep somebody had stopped short of. An arc
    that cannot carry the margins is an arc that will false-refuse a legal position.
    """
    low = 1000
    result = classify_travel(travel_leaving_arc(low, low + MIN_EVICTABLE_ARC_TICKS - 1))

    assert result.kind is TravelKind.BOUNDED  # it IS bounded — that is measured
    assert result.seam_in_travel is True
    assert result.unreachable_arc is None  # but there is nowhere safe to put the seam
    assert result.can_be_rezeroed is False
    assert result.rezero_offset is None
    assert result.remedy is SeamRemedy.SOFT_LIMIT
    assert str(MIN_EVICTABLE_ARC_TICKS) in result.reason


def test_the_declared_arc_is_a_strict_subset_of_the_measured_one():
    """We only ever SHRINK the arc. Never claim a tick the joint might actually reach.

    Conservative in both directions that matter: it cannot false-refuse a legal
    position, and it cannot park the seam somewhere the joint can get to.
    """
    measured = the_measured_arc()
    arc = classify_travel(travel_leaving_arc(measured.low, measured.high)).unreachable_arc

    assert arc is not None
    assert measured.low < arc.low
    assert arc.high < measured.high
    assert measured.contains(arc.midpoint)


@pytest.mark.parametrize("parity", (0, 1))
def test_a_seam_placement_the_register_cannot_express_is_nudged_not_refused(parity):
    """Raw 2048 is the one seam the sign-magnitude register cannot hold. Shrink by a tick.

    Refusing a 1800-tick arc over a one-tick encoding hole would be a false SOFT_LIMIT
    verdict. Shrinking the arc by one tick moves the seam to 2047 or 2049, keeps the arc
    a strict subset of the unreachable region, and costs nothing physical.
    """
    half_span = 1000
    low = UNREPRESENTABLE_SEAM_TICK - half_span
    high = UNREPRESENTABLE_SEAM_TICK + half_span + parity
    # The midpoint of the measured arc — and of its inset, which shares the same sum —
    # lands exactly on the tick the register cannot express.
    assert UnreachableArc(low=low, high=high).midpoint == UNREPRESENTABLE_SEAM_TICK

    result = classify_travel(travel_leaving_arc(low, high))
    arc = result.unreachable_arc

    assert result.kind is TravelKind.BOUNDED
    assert arc is not None
    assert arc.midpoint != UNREPRESENTABLE_SEAM_TICK
    assert abs(arc.offset) <= MAX_ENCODER_OFFSET
    assert arc.evicts(arc.offset)
    assert low < arc.low and arc.high < high  # still a strict subset


def test_an_arc_at_the_cutoff_centred_on_the_unrepresentable_tick_is_refused():
    """The one corner where the nudge has nowhere to go — and SOFT_LIMIT is then CORRECT.

    At exactly the cutoff there is a single tick inside the inset arc. If that tick is
    the one the register cannot express, the seam genuinely cannot be evicted: this is
    not a shortcoming of the code, it is the joint's answer.
    """
    low = UNREPRESENTABLE_SEAM_TICK - MIN_EVICTABLE_ARC_TICKS // 2
    result = classify_travel(travel_leaving_arc(low, low + MIN_EVICTABLE_ARC_TICKS))

    assert result.kind is TravelKind.BOUNDED
    assert result.unreachable_arc is None
    assert result.remedy is SeamRemedy.SOFT_LIMIT


# ---------------------------------------------------------------------------
# A bounded joint whose travel never meets the seam needs nothing at all.
# ---------------------------------------------------------------------------


def test_a_bounded_joint_that_never_crosses_the_seam_needs_no_rezero():
    """Four of the six joints. There is no seam in the way — nothing to evict.

    This is arm_spec's ``_REZERO_UNNECESSARY``, re-derived: "you don't need one" is a
    different answer from "you can't have one", and collapsing them would teach an
    operator the wrong thing about their arm.
    """
    result = classify_travel(travel(origin_raw=2048, low_displacement=-500, high_displacement=500))

    assert result.kind is TravelKind.BOUNDED
    assert result.seam_in_travel is False
    assert result.unreachable_arc is None
    assert result.remedy is SeamRemedy.NONE_NEEDED


def test_a_travel_that_stops_exactly_on_the_top_tick_has_not_crossed_the_seam():
    """Touching 4095 is not rolling over it. The tick axis is still linear."""
    result = classify_travel(
        travel(origin_raw=TICK_MAX, low_displacement=-500, high_displacement=0)
    )

    assert result.kind is TravelKind.BOUNDED
    assert result.seam_in_travel is False
    assert result.remedy is SeamRemedy.NONE_NEEDED


# ---------------------------------------------------------------------------
# The type refuses to hold a claim it cannot back (limits.py's own discipline).
# ---------------------------------------------------------------------------


def test_a_continuous_classification_cannot_carry_an_arc():
    """A continuous joint has no unreachable arc BY DEFINITION. Not "we didn't find one"."""
    with pytest.raises(ValueError, match="arc"):
        TravelClassification(
            joint=ANON,
            kind=TravelKind.CONTINUOUS,
            reason="—",
            swept_ticks=ENCODER_TICKS,
            seam_in_travel=True,
            unreachable_arc=UnreachableArc(low=1000, high=2000),
        )


def test_an_undetermined_classification_cannot_carry_walls():
    """Nothing was vouched for, so there is no envelope to hand out."""
    with pytest.raises(ValueError):
        TravelClassification(
            joint=ANON,
            kind=TravelKind.UNDETERMINED,
            reason="—",
            swept_ticks=100,
            seam_in_travel=None,
            reachable_raw_ends=(100, 200),
        )


def test_a_bounded_classification_must_carry_its_walls():
    """Two walls is what BOUNDED MEANS — a record without them is a bug, not a record."""
    with pytest.raises(ValueError):
        TravelClassification(
            joint=ANON,
            kind=TravelKind.BOUNDED,
            reason="—",
            swept_ticks=100,
            seam_in_travel=False,
        )


def test_a_continuous_classification_must_have_swept_a_full_turn():
    """The definition, pinned into the type: less than a full turn is not continuous."""
    with pytest.raises(ValueError, match="full turn"):
        TravelClassification(
            joint=ANON,
            kind=TravelKind.CONTINUOUS,
            reason="—",
            swept_ticks=ENCODER_TICKS - 1,
            seam_in_travel=True,
        )


def test_a_bounded_classification_cannot_have_swept_a_full_turn():
    """And the converse: a joint that swept the circle is not bounded, whatever it says."""
    with pytest.raises(ValueError, match="full turn"):
        TravelClassification(
            joint=ANON,
            kind=TravelKind.BOUNDED,
            reason="—",
            swept_ticks=ENCODER_TICKS,
            seam_in_travel=True,
            reachable_raw_ends=(0, 0),
            unreachable_width=1,
        )


def test_an_arc_that_cannot_take_the_seam_is_unrepresentable():
    """The three conditions arm_spec's ``_require_evictable_seam`` enforces, at the door."""
    with pytest.raises(ValueError):
        TravelClassification(
            joint=ANON,
            kind=TravelKind.BOUNDED,
            reason="—",
            swept_ticks=ENCODER_TICKS - 2,
            seam_in_travel=True,
            reachable_raw_ends=(2, 0),
            unreachable_width=2,
            unreachable_arc=UnreachableArc(
                low=UNREPRESENTABLE_SEAM_TICK - 1, high=UNREPRESENTABLE_SEAM_TICK + 1
            ),
        )


def test_a_one_tick_arc_has_nowhere_to_put_the_seam():
    """An arc with no interior cannot hold a seam. arm_spec refuses one; so does this."""
    with pytest.raises(ValueError, match="cannot take the seam"):
        TravelClassification(
            joint=ANON,
            kind=TravelKind.BOUNDED,
            reason="—",
            swept_ticks=ENCODER_TICKS - 1,
            seam_in_travel=True,
            reachable_raw_ends=(1, 0),
            unreachable_width=1,
            unreachable_arc=UnreachableArc(low=1000, high=1001),
        )


def test_a_continuous_classification_must_admit_the_seam_is_in_its_travel():
    """It reaches every angle — the seam is necessarily one of them."""
    with pytest.raises(ValueError, match="seam_in_travel"):
        TravelClassification(
            joint=ANON,
            kind=TravelKind.CONTINUOUS,
            reason="—",
            swept_ticks=ENCODER_TICKS,
            seam_in_travel=False,
        )


def test_a_wall_that_is_not_a_raw_tick_is_refused():
    """A wall is a position on the encoder, and the encoder has 4096 of them."""
    with pytest.raises(ValueError, match="raw tick"):
        TravelClassification(
            joint=ANON,
            kind=TravelKind.BOUNDED,
            reason="—",
            swept_ticks=1000,
            seam_in_travel=False,
            reachable_raw_ends=(ENCODER_TICKS, 100),
            unreachable_width=ENCODER_TICKS - 1000,
        )


def test_a_width_that_is_not_the_rest_of_the_circle_is_refused():
    """The arc is the COMPLEMENT of the travel. A width that says otherwise describes neither."""
    with pytest.raises(ValueError, match="rest of the circle"):
        TravelClassification(
            joint=ANON,
            kind=TravelKind.BOUNDED,
            reason="—",
            swept_ticks=1000,
            seam_in_travel=False,
            reachable_raw_ends=(1000, 2000),
            unreachable_width=5,
        )


def test_walls_and_a_width_that_disagree_are_refused():
    """Both are the same measurement seen from two sides. If they differ, neither is true."""
    with pytest.raises(ValueError, match="ticks apart"):
        TravelClassification(
            joint=ANON,
            kind=TravelKind.BOUNDED,
            reason="—",
            swept_ticks=2000,
            seam_in_travel=False,
            reachable_raw_ends=(1000, 2000),
            unreachable_width=ENCODER_TICKS - 2000,
        )


def test_a_classification_needs_a_joint_to_be_about():
    with pytest.raises(ValueError, match="joint"):
        TravelClassification(
            joint="",
            kind=TravelKind.UNDETERMINED,
            reason="—",
            swept_ticks=0,
            seam_in_travel=None,
        )


def test_a_negative_sweep_is_not_a_distance():
    with pytest.raises(ValueError, match="negative"):
        TravelClassification(
            joint=ANON,
            kind=TravelKind.UNDETERMINED,
            reason="—",
            swept_ticks=-1,
            seam_in_travel=None,
        )


def test_an_arc_needs_the_seam_to_be_in_the_travel():
    """An arc is the answer to a seam INSIDE the travel. Without one there is no question."""
    with pytest.raises(ValueError):
        TravelClassification(
            joint=ANON,
            kind=TravelKind.BOUNDED,
            reason="—",
            swept_ticks=1000,
            seam_in_travel=False,
            reachable_raw_ends=(1000, 2000),
            unreachable_width=ENCODER_TICKS - 1000,
            unreachable_arc=UnreachableArc(low=2100, high=3900),
        )


# ---------------------------------------------------------------------------
# Serialization
# ---------------------------------------------------------------------------


def test_to_dict_is_plain_json():
    """No enums, no dataclasses — a report has to be able to print this."""
    measured = the_measured_arc()
    payload = classify_travel(travel_leaving_arc(measured.low, measured.high)).to_dict()

    round_tripped = json.loads(json.dumps(payload))
    assert round_tripped["kind"] == TravelKind.BOUNDED.value
    assert round_tripped["remedy"] == SeamRemedy.REZERO.value
    assert round_tripped["unreachable_arc"]["offset"] == arm_spec.rezero_arc(REZEROED_JOINT).offset


def test_to_dict_of_a_continuous_joint_says_so_in_plain_json():
    payload = json.loads(json.dumps(classify_travel(a_full_turn()).to_dict()))

    assert payload["kind"] == TravelKind.CONTINUOUS.value
    assert payload["unreachable_arc"] is None
    assert payload["remedy"] == SeamRemedy.SOFT_LIMIT.value


# ---------------------------------------------------------------------------
# classify_observations — the entry point the probe actually has
# ---------------------------------------------------------------------------


def test_classify_observations_merges_then_classifies():
    """The probe holds observations, not a JointTravel. Same answer either way."""
    observations = [
        EndObservation(
            joint=ANON,
            end=TravelEnd.LOW,
            verdict=LimitVerdict.WALL,
            origin_raw=2048,
            displacement=-500,
        ),
        EndObservation(
            joint=ANON,
            end=TravelEnd.HIGH,
            verdict=LimitVerdict.WALL,
            origin_raw=2048,
            displacement=500,
        ),
    ]
    assert classify_observations(observations) == classify_travel(merge_joint_travel(observations))


def test_a_single_end_that_turned_a_full_circle_needs_no_second_end():
    """Once a joint has swept the whole circle there is nothing the other end can add.

    And the probe, which stops at the cap, will never HAVE a second end for it — so
    demanding one would force it to fabricate an observation. Termination is by
    accumulated displacement; this is what that buys.
    """
    result = classify_observations(
        [
            EndObservation(
                joint=ANON,
                end=TravelEnd.HIGH,
                verdict=LimitVerdict.EDGE,
                origin_raw=1234,
                displacement=ENCODER_TICKS,
            )
        ]
    )
    assert result.kind is TravelKind.CONTINUOUS
    assert result.remedy is SeamRemedy.SOFT_LIMIT


def test_a_single_end_that_found_a_wall_is_not_a_travel():
    """One wall says nothing about the other end — and an unprobed end is not an empty one."""
    with pytest.raises(ValueError, match="both ends"):
        classify_observations(
            [
                EndObservation(
                    joint=ANON,
                    end=TravelEnd.LOW,
                    verdict=LimitVerdict.WALL,
                    origin_raw=2048,
                    displacement=-500,
                )
            ]
        )


def test_classify_observations_rejects_an_empty_batch():
    with pytest.raises(ValueError):
        classify_observations([])


def test_classify_observations_rejects_a_batch_that_mixes_joints():
    """Two joints' travels folded together would be a measurement of neither."""
    with pytest.raises(ValueError, match="same joint"):
        classify_observations(
            [
                EndObservation(
                    joint=REZEROED_JOINT,
                    end=TravelEnd.LOW,
                    verdict=LimitVerdict.WALL,
                    origin_raw=2048,
                    displacement=-500,
                ),
                EndObservation(
                    joint=FREE_JOINT,
                    end=TravelEnd.HIGH,
                    verdict=LimitVerdict.WALL,
                    origin_raw=2048,
                    displacement=500,
                ),
            ]
        )


# ---------------------------------------------------------------------------
# Purity — a classifier that cannot touch the hardware it classifies.
# ---------------------------------------------------------------------------


def test_classify_module_never_imports_the_bus():
    """Mirrors ``test_arm_spec_module_never_imports_the_bus``. Pure in, pure out."""
    import arm101.hardware.classify as classify_module

    tree = ast.parse(inspect.getsource(classify_module))
    imported: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module)

    assert not any(
        "bus" in name.lower() for name in imported
    ), f"classify must not import a bus module; found {imported}"


def test_importing_classify_does_not_drag_in_the_bus_transitively():
    """The AST guard sees direct imports only. This one sees the whole graph."""
    code = (
        "import sys; import arm101.hardware.classify; "
        "print(sorted(m for m in sys.modules if m.startswith('arm101') and 'bus' in m))"
    )
    result = subprocess.run(  # nosec B603 — fixed argv, no shell, no user input
        [sys.executable, "-c", code],
        capture_output=True,
        text=True,
        check=True,
    )
    assert result.stdout.strip() == "[]", (
        f"importing arm101.hardware.classify pulled a bus module into sys.modules "
        f"({result.stdout.strip()}) — the classifier must stay pure"
    )
