"""A re-zero derived from a LIVE MEASUREMENT, not from a hand-typed table (t8).

What was missing, and what these tests pin
==========================================

``REZERO_ARCS`` held exactly one entry — ``elbow_flex`` — typed by a human after a
long hardware session, and ``rezero.require_rezeroable`` could only ever *apply* an
arc a human had already found. **Nothing measured one.** So the procedure that fixed
``elbow_flex`` did not generalise: repeating it for another joint meant repeating that
session, by hand, joint by joint.

The missing piece is a derivation:

    a probe's observations
      -> ``limits.merge_joint_travel``   (evidence, four verdicts, RAW displacement)
      -> ``classify.classify_travel``    (BOUNDED | CONTINUOUS | UNDETERMINED, + arc)
      -> ``arm_spec.arc_from_measurement``  <- THIS, the bridge
      -> ``arm_spec.rezero_offset(joint, measured=...)``  (the SAME derivation the
         table already goes through: ``UnreachableArc.offset``)
      -> ``rezero.require_rezeroable(joint, measured=...)`` -> plan -> write

The table survives as the **shipped default** — it encodes a real measurement of
``elbow_flex`` — but it has stopped being the only possible source of an arc.

One derivation, never two
=========================

The offset is *never* typed and *never* recomputed on a second path: it is
``UnreachableArc.offset``, whether the arc came out of the table or off the arm five
seconds ago. :func:`test_re_measuring_a_wall_in_the_table_moves_the_derived_offset`
is the acid test — it re-executes ``arm_spec``'s own source with **one measured wall
moved** and demands the offset move with it, no other edit.

Nothing here copies a number out of ``arm_spec``. Every expectation is derived from
it, because the table exists in order to be re-measured — and a suite that copies it
turns a re-measurement into 34 test failures (which is exactly what happened on
2026-07-12).
"""

from __future__ import annotations

import inspect
import sys
import types

import pytest

from arm101.cli._errors import EXIT_USER_ERROR, CliError
from arm101.hardware import arm_spec, rezero
from arm101.hardware.bus import FakeBus
from arm101.hardware.classify import TravelKind, classify_travel
from arm101.hardware.limits import (
    ENCODER_TICKS,
    EndObservation,
    JointTravel,
    LimitVerdict,
    TravelEnd,
    merge_joint_travel,
)

#: The joint whose arc is in the shipped table, and the motor carrying it.
ELBOW = "elbow_flex"
ELBOW_MOTOR = 3

#: A joint with NO arc in the table — the whole point of the exercise. If a measured
#: arc only worked for the joint that already had one, nothing would have generalised.
UNMEASURED_JOINT = "shoulder_pan"

#: The joint a re-zero can never help: its travel covers the whole circle.
FREE_JOINT = "wrist_roll"


# ---------------------------------------------------------------------------
# Helpers — a travel is RAW ticks and DISPLACEMENTS, which is all a probe records
# ---------------------------------------------------------------------------


def travel_leaving_arc(low: int, high: int, *, joint: str) -> JointTravel:
    """A WALL-at-both-ends travel whose measured walls leave ``(low, high)`` unreachable.

    The joint's travel therefore WRAPS the raw seam — ``elbow_flex``'s shape, and the
    one a ``[min, max]`` pair cannot express.
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


def travel_clear_of_the_seam(*, joint: str) -> JointTravel:
    """Two walls, and the travel never goes near the seam. Nothing to evict."""
    return merge_joint_travel(
        [
            EndObservation(
                joint=joint,
                end=TravelEnd.LOW,
                verdict=LimitVerdict.WALL,
                origin_raw=2048,
                displacement=-500,
            ),
            EndObservation(
                joint=joint,
                end=TravelEnd.HIGH,
                verdict=LimitVerdict.WALL,
                origin_raw=2048,
                displacement=500,
            ),
        ]
    )


def a_full_turn(*, joint: str) -> JointTravel:
    """Driven a full turn with nothing stopping it: every angle is reachable."""
    return merge_joint_travel(
        [
            EndObservation(
                joint=joint,
                end=TravelEnd.LOW,
                verdict=LimitVerdict.EDGE,
                origin_raw=0,
                displacement=0,
            ),
            EndObservation(
                joint=joint,
                end=TravelEnd.HIGH,
                verdict=LimitVerdict.EDGE,
                origin_raw=0,
                displacement=ENCODER_TICKS,
            ),
        ]
    )


def an_undetermined_travel(*, joint: str) -> JointTravel:
    """One wall, one stall against the torque cap. No arc can be sited on that."""
    return merge_joint_travel(
        [
            EndObservation(
                joint=joint,
                end=TravelEnd.LOW,
                verdict=LimitVerdict.WALL,
                origin_raw=2048,
                displacement=-500,
            ),
            EndObservation(
                joint=joint,
                end=TravelEnd.HIGH,
                verdict=LimitVerdict.TORQUE_LIMITED,
                origin_raw=2048,
                displacement=500,
            ),
        ]
    )


def the_walls_behind_the_shipped_arc() -> tuple[int, int]:
    """The two RAW walls ``REZERO_ARCS`` was inset from — derived, never copied."""
    return (arm_spec._LOW_WALL_OBSERVED, arm_spec._HIGH_WALL_OBSERVED)


def a_wrapping_measurement(joint: str):
    """A BOUNDED, seam-wrapping measurement of *joint*, shaped like the real elbow_flex."""
    low_wall, high_wall = the_walls_behind_the_shipped_arc()
    return classify_travel(travel_leaving_arc(low_wall, high_wall, joint=joint))


# ---------------------------------------------------------------------------
# The margin is one number, and it is now public
# ---------------------------------------------------------------------------


def test_the_arc_margin_is_public_and_the_private_alias_still_names_it():
    """``classify`` shares this inset with ``arm_spec``; a shared name should not be private."""
    assert arm_spec.ARC_MARGIN_TICKS == arm_spec._ARC_MARGIN_TICKS

    from arm101.hardware import classify

    assert classify.ARC_MARGIN_TICKS == arm_spec.ARC_MARGIN_TICKS


# ---------------------------------------------------------------------------
# The derivation: a measured travel yields an arc, and the arc yields the offset
# ---------------------------------------------------------------------------


def test_a_measured_wrapping_joint_yields_an_arc_even_with_no_table_entry():
    """The whole point. ``shoulder_pan`` has no arc in the table — measure it, and it has one.

    Before this, the ONLY way a joint got an arc was for a human to type one. So the
    fix for ``elbow_flex`` did not generalise: four more joints meant four more human
    sessions. It generalises now.
    """
    assert arm_spec.rezero_arc(UNMEASURED_JOINT) is None  # nothing in the table

    measured = a_wrapping_measurement(UNMEASURED_JOINT)
    arc = arm_spec.rezero_arc(UNMEASURED_JOINT, measured=measured)

    assert arc is not None
    assert arc == measured.unreachable_arc
    assert arm_spec.rezero_refusal(UNMEASURED_JOINT, measured=measured) is None


def test_the_offset_of_a_measured_arc_is_the_arcs_own_offset_not_a_second_derivation():
    """One derivation. ``rezero_offset`` is ``arc.offset``, whatever the arc's provenance."""
    measured = a_wrapping_measurement(UNMEASURED_JOINT)
    arc = arm_spec.rezero_arc(UNMEASURED_JOINT, measured=measured)

    offset = arm_spec.rezero_offset(UNMEASURED_JOINT, measured=measured)

    assert offset == arc.offset == arm_spec._offset_for_seam_at(arc.midpoint)
    assert arc.evicts(offset)  # the goal is a PLACE: the seam lands out of reach


def test_a_measurement_beats_the_shipped_table_for_the_same_joint():
    """The table is the shipped DEFAULT, not the last word. A fresh measurement wins.

    Otherwise re-measuring a joint could never change its re-zero, which is precisely
    the trap the table was in.
    """
    low_wall, high_wall = the_walls_behind_the_shipped_arc()
    shifted = classify_travel(travel_leaving_arc(low_wall + 60, high_wall, joint=ELBOW))

    assert arm_spec.rezero_arc(ELBOW, measured=shifted) == shifted.unreachable_arc
    assert arm_spec.rezero_arc(ELBOW, measured=shifted) != arm_spec.rezero_arc(ELBOW)
    assert arm_spec.rezero_offset(ELBOW, measured=shifted) != arm_spec.rezero_offset(ELBOW)


def test_the_shipped_table_reproduces_itself_when_re_measured_identically():
    """Feed the derivation the walls the table was built from: the same arc comes back.

    The table and a live measurement are the same computation over the same walls —
    not two conventions that happen to agree today.
    """
    measured = a_wrapping_measurement(ELBOW)

    assert arm_spec.rezero_arc(ELBOW, measured=measured) == arm_spec.rezero_arc(ELBOW)
    assert arm_spec.rezero_offset(ELBOW, measured=measured) == arm_spec.rezero_offset(ELBOW)


def test_a_measurement_of_another_joint_is_refused_loudly():
    """A measurement carries the joint it was taken on. Applying it to another is a bug."""
    measured = a_wrapping_measurement(UNMEASURED_JOINT)

    with pytest.raises(ValueError, match="shoulder_pan"):
        arm_spec.rezero_arc(ELBOW, measured=measured)


def test_an_unknown_joint_still_raises_even_with_a_measurement():
    measured = a_wrapping_measurement("knee")
    with pytest.raises(ValueError, match="Unknown joint"):
        arm_spec.rezero_arc("knee", measured=measured)


# ---------------------------------------------------------------------------
# A measurement can also REFUSE — in its own words, which are the honest ones
# ---------------------------------------------------------------------------


def test_a_continuous_measurement_refuses_a_rezero_in_principle():
    """Measured free all the way round: no arc exists, so no offset can evict the seam."""
    measured = classify_travel(a_full_turn(joint=FREE_JOINT))
    assert measured.kind is TravelKind.CONTINUOUS

    assert arm_spec.rezero_arc(FREE_JOINT, measured=measured) is None
    assert arm_spec.rezero_offset(FREE_JOINT, measured=measured) is None

    refusal = arm_spec.rezero_refusal(FREE_JOINT, measured=measured)
    assert refusal == measured.reason  # the measurement's own words, not the table's
    assert "soft limit" in refusal.lower()


def test_an_undetermined_measurement_refuses_rather_than_guessing():
    """One wall and one stall sites no arc. "Measure again" is the answer, not "pick"."""
    measured = classify_travel(an_undetermined_travel(joint=UNMEASURED_JOINT))
    assert measured.kind is TravelKind.UNDETERMINED

    assert arm_spec.rezero_offset(UNMEASURED_JOINT, measured=measured) is None
    assert arm_spec.rezero_refusal(UNMEASURED_JOINT, measured=measured) == measured.reason


def test_a_measured_joint_whose_travel_misses_the_seam_needs_nothing():
    """ "You don't need one" survives — but it is now EARNED by a measurement, not asserted.

    This is the distinction ``arm_spec`` has always drawn and must keep drawing:
    "you don't need a re-zero" is a different answer from "you can't have one". What
    changed is who is entitled to say it — a measurement, not a table.
    """
    measured = classify_travel(travel_clear_of_the_seam(joint=UNMEASURED_JOINT))
    assert measured.seam_in_travel is False

    assert arm_spec.rezero_offset(UNMEASURED_JOINT, measured=measured) is None

    refusal = arm_spec.rezero_refusal(UNMEASURED_JOINT, measured=measured)
    assert refusal == measured.reason
    assert "nothing to evict" in refusal


# ---------------------------------------------------------------------------
# The retraction (issue #43): with no measurement, the arc is UNKNOWN
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("joint", ["shoulder_pan", "shoulder_lift", "wrist_flex", "gripper"])
def test_an_unmeasured_joint_is_refused_as_UNKNOWN_never_as_unnecessary(joint):
    """The old message told the operator, confidently, something HARDWARE CONTRADICTS.

    It said these four joints "do not wrap inside their travel at all". Issue #43:
    three of them reached the commandable bound with no contact — still physically
    free — 2, 3 and 11 raw ticks from the seam, and ``shoulder_lift`` then sagged
    THROUGH the seam under gravity. The bound sat ON the seam, which is exactly why
    nobody could see past it.

    Until the arc is measured, the only honest answer is that it is UNKNOWN — and
    "unknown" must not be dressed up as either "unnecessary" or "it wraps".
    """
    assert arm_spec.rezero_offset(joint) is None

    refusal = arm_spec.rezero_refusal(joint)
    assert joint in refusal
    # It says what IS known: there is no measured arc, so no re-zero can be derived.
    assert "no MEASURED unreachable arc" in refusal
    assert "UNKNOWN, NOT UNNECESSARY" in refusal
    # It names the retraction and the evidence for it.
    assert "WITHDRAWN" in refusal
    assert "#43" in refusal
    # And it does NOT make the old claim.
    assert "does not need a re-zero" not in refusal
    # Nor the opposite one: we do not know that it wraps either.
    assert "We do not know" in refusal


def test_the_unmeasured_refusal_is_not_the_impossible_one():
    """Two structurally different "no"s, and they must not collapse into each other."""
    unknown = arm_spec.rezero_refusal(UNMEASURED_JOINT)
    impossible = arm_spec.rezero_refusal(FREE_JOINT)

    assert unknown != impossible
    assert "RELOCATES" in impossible and "EVICT" in impossible  # proven, and it stays
    assert "WITHDRAWN" not in impossible


def test_wrist_rolls_proven_impossibility_survives_the_retraction():
    """``wrist_roll``'s refusal is PROVEN — its travel covers the circle. Nothing retracts it."""
    assert arm_spec.rezero_offset(FREE_JOINT) is None
    assert arm_spec.soft_limit(FREE_JOINT) is not None

    refusal = arm_spec.rezero_refusal(FREE_JOINT)
    assert "SOFT LIMIT" in refusal
    assert "no MEASURED unreachable arc" not in refusal


# ---------------------------------------------------------------------------
# rezero.require_rezeroable consumes a measurement
# ---------------------------------------------------------------------------


def test_require_rezeroable_accepts_a_measured_arc_for_an_untabled_joint():
    """The table is no longer the only source of an arc a re-zero can be planned from."""
    measured = a_wrapping_measurement(UNMEASURED_JOINT)

    offset, arc = rezero.require_rezeroable(UNMEASURED_JOINT, measured=measured)

    assert arc == measured.unreachable_arc
    assert offset == arc.offset
    assert arc.evicts(offset)


def test_require_rezeroable_still_answers_from_the_table_with_no_measurement():
    """The shipped default keeps working with no hardware, no probe, no arm plugged in."""
    offset, arc = rezero.require_rezeroable(ELBOW)

    assert arc == arm_spec.rezero_arc(ELBOW)
    assert offset == arm_spec.rezero_offset(ELBOW)


def test_require_rezeroable_refuses_a_measurement_that_supports_no_arc():
    """A CONTINUOUS measurement is a refusal — carried up in the measurement's own words."""
    measured = classify_travel(a_full_turn(joint=UNMEASURED_JOINT))

    with pytest.raises(CliError) as exc:
        rezero.require_rezeroable(UNMEASURED_JOINT, measured=measured)

    assert exc.value.code == EXIT_USER_ERROR
    assert measured.reason in exc.value.message


def test_plan_rezero_plans_from_a_measured_arc():
    """The plumbing runs end to end: measurement -> arc -> offset -> the write that is planned."""
    measured = a_wrapping_measurement(UNMEASURED_JOINT)
    arc = measured.unreachable_arc
    bus = FakeBus(positions={1: 0}, offsets={1: arm_spec.FACTORY_ENCODER_OFFSET})
    bus.open()

    plan = rezero.plan_rezero(bus, 1, UNMEASURED_JOINT, measured=measured)

    assert plan.target_offset == arc.offset
    assert plan.already_applied is False  # the factory seam is in the travel
    assert not arc.contains(plan.raw_position)


def test_plan_rezero_calls_an_already_evicting_offset_done_even_when_measured():
    """The goal is a PLACE, not a number: any offset inside the arc is already the fix.

    Our own follower holds 1073 against a derived target of 1157, and it is correctly
    re-zeroed. A measured arc must not tighten that into ``current == target``.
    """
    measured = a_wrapping_measurement(UNMEASURED_JOINT)
    arc = measured.unreachable_arc
    already = (arc.low + arc.offset) // 2  # some other offset whose seam is inside the arc
    assert arc.evicts(already) and already != arc.offset

    # A reported position whose RAW image is OUTSIDE the arc — the joint cannot be
    # somewhere it physically cannot go, and plan_rezero rightly refuses that.
    reported = (3000 - already) % arm_spec.ENCODER_TICKS
    bus = FakeBus(positions={1: reported}, offsets={1: already})
    bus.open()

    plan = rezero.plan_rezero(bus, 1, UNMEASURED_JOINT, measured=measured)

    assert plan.already_applied is True
    assert plan.target_offset == already  # nothing is written, so nothing is promised


# ---------------------------------------------------------------------------
# THE ACID TEST — move a measured wall, and the offset moves with it
# ---------------------------------------------------------------------------


def test_re_measuring_a_wall_in_the_table_moves_the_derived_offset():
    """Change one measured wall in ``arm_spec``'s source; the offset must follow. No other edit.

    This is the property the whole module hangs on, and it is checked by *actually doing
    it*: the module's source is re-executed with ``_LOW_WALL_OBSERVED`` moved outward,
    and the offset a re-zero would write is required to move by exactly half of that (the
    arc is inset from both walls, and the seam goes to its midpoint).

    Nothing is allowed to hard-code an offset that a re-measurement would then have to
    chase — the 2026-07-12 correction moved the target from 1073 to 1157 without a line
    of ``rezero_offset`` changing, and that must remain true of every path here.
    """
    shift = 40  # even, so the midpoint moves by exactly shift // 2
    source = inspect.getsource(arm_spec)
    old = f"_LOW_WALL_OBSERVED = {arm_spec._LOW_WALL_OBSERVED}"
    assert source.count(old) == 1, "the wall is measured in exactly one place"

    re_measured = source.replace(old, f"_LOW_WALL_OBSERVED = {arm_spec._LOW_WALL_OBSERVED + shift}")

    # A real module object, registered while it executes: ``dataclass`` resolves the
    # module's own annotations through ``sys.modules``, so a bare dict is not enough.
    module = types.ModuleType("arm101.hardware.arm_spec_re_measured")
    sys.modules[module.__name__] = module
    try:
        # The table's import-time guards run again here, on the moved arc — so this
        # also proves the re-measured arc is still one a seam can actually be evicted to.
        exec(compile(re_measured, arm_spec.__file__, "exec"), module.__dict__)  # nosec B102
        moved_arc = module.rezero_arc(ELBOW)
        moved_offset = module.rezero_offset(ELBOW)
    finally:
        sys.modules.pop(module.__name__, None)

    assert moved_arc.low == arm_spec.rezero_arc(ELBOW).low + shift
    assert moved_offset == moved_arc.offset  # still derived, not typed
    assert moved_offset == arm_spec.rezero_offset(ELBOW) + shift // 2
