"""Tests for arm101.hardware.limits — the FOUR-VERDICT per-END limit record (t2).

The contract this file pins, in the spec's own words:

* Four verdicts, each **producible by a test**. "A verdict no test can produce is
  a verdict the code cannot actually tell apart from its neighbours, and shipping
  it is a lie with four names."
* A range whose end is TORQUE-LIMITED **cannot be read as a wall** — and that must
  be *structural*, not a comment.
* "A TORQUE-LIMITED end NEVER becomes a mechanical limit on the evidence of poses
  alone, no matter how many poses are recorded."

The distinction those three encode: a limit found in ONE pose is ENVIRONMENTAL (it
depends on where the other joints happen to be). The MECHANICAL limit is the widest
envelope ever seen across poses — sound for a WALL, because an obstacle can only
ever make a range *smaller*, so the max over poses converges on the mechanical
truth. That reasoning is FALSE for a TORQUE-LIMITED end: the arm's own weakness is
not an obstacle you can pose your way out of. It shrinks the range in *every* pose,
so no number of poses ever escapes it, and the end stays a lower bound forever.
"""

from __future__ import annotations

import ast
import inspect
import json
import subprocess  # nosec B404 — used only to prove an import graph, with a fixed argv
import sys

import pytest

from arm101.hardware.limits import (
    ENCODER_TICKS,
    HALF_TURN,
    EndObservation,
    JointTravel,
    LimitVerdict,
    LowerBoundEnd,
    MeasuredEnd,
    TravelEnd,
    WallEnd,
    merge_end_observations,
    merge_joint_travel,
    signed_delta,
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

#: Every verdict that is NOT a wall. These are the three that must never be
#: readable as a mechanical limit.
_NOT_A_WALL = (
    LimitVerdict.TORQUE_LIMITED,
    LimitVerdict.EDGE,
    LimitVerdict.TIMEOUT,
)


def obs(
    verdict: LimitVerdict,
    *,
    joint: str = "shoulder_lift",
    end: TravelEnd = TravelEnd.HIGH,
    origin_raw: int = 2048,
    displacement: int = 500,
    load: int | None = None,
    pose: str | None = None,
) -> EndObservation:
    """One probe's finding at one END of one joint, in one pose."""
    return EndObservation(
        joint=joint,
        end=end,
        verdict=verdict,
        origin_raw=origin_raw,
        displacement=displacement,
        load=load,
        pose=pose,
    )


# ---------------------------------------------------------------------------
# Criterion 1 — each of the four verdicts is PRODUCED by its own test.
#
# One test per verdict. If a verdict cannot be generated here, the code cannot
# tell it from its neighbours.
# ---------------------------------------------------------------------------


def test_verdict_wall_is_produced_and_is_the_only_one_that_vouches():
    """WALL — load saturated against something solid. A real limit."""
    end = merge_end_observations([obs(LimitVerdict.WALL, load=500)])

    assert isinstance(end, WallEnd)
    assert end.verdict is LimitVerdict.WALL
    assert end.is_wall is True
    # A wall is the ONE verdict that yields a mechanical limit.
    assert end.mechanical_limit == 2548
    assert end.reach == 2548


def test_verdict_torque_limited_is_produced_and_is_only_ever_a_lower_bound():
    """TORQUE-LIMITED — stalled at high load, but the cap is the likeliest reason.

    A LOWER BOUND, never a wall. The joint got AT LEAST this far; whether there
    is anything in front of it is unknown, and more poses will never settle it.
    """
    end = merge_end_observations([obs(LimitVerdict.TORQUE_LIMITED, load=500)])

    assert isinstance(end, LowerBoundEnd)
    assert end.verdict is LimitVerdict.TORQUE_LIMITED
    assert end.is_wall is False
    # There is no mechanical limit to read out of it — structurally, not by note.
    assert end.mechanical_limit is None
    # What it DOES establish: the joint reached this tick. That is honest.
    assert end.reach == 2548


def test_verdict_edge_is_produced_and_learned_nothing():
    """EDGE — ran out of commandable frame. Learned nothing yet."""
    end = merge_end_observations([obs(LimitVerdict.EDGE)])

    assert isinstance(end, LowerBoundEnd)
    assert end.verdict is LimitVerdict.EDGE
    assert end.is_wall is False
    assert end.mechanical_limit is None


def test_verdict_timeout_is_produced_and_learned_nothing():
    """TIMEOUT — never got there. Learned nothing."""
    end = merge_end_observations([obs(LimitVerdict.TIMEOUT)])

    assert isinstance(end, LowerBoundEnd)
    assert end.verdict is LimitVerdict.TIMEOUT
    assert end.is_wall is False
    assert end.mechanical_limit is None


def test_the_five_verdicts_are_mutually_distinguishable():
    """Five members, five distinct values, and exactly one of them vouches.

    The failure this rules out is five names for the same thing.

    UNFIRABLE_THRESHOLD is the newest (issue #43) and the least intuitive: it is not a
    fact about the joint at all, it is a fact about the INSTRUMENT — the contact rule
    needed a load the joint could not produce, so it could not have fired whatever the
    joint did. Like the other three non-WALL verdicts it vouches for nothing, which is
    the whole point: a wall nobody could hear is still a wall nobody measured.
    """
    verdicts = list(LimitVerdict)
    assert len(verdicts) == 5
    assert len({v.value for v in verdicts}) == 5

    vouching = [v for v in verdicts if v.vouches_for_a_wall]
    assert vouching == [LimitVerdict.WALL]

    # And the merged RECORD keeps them apart, not just the enum.
    merged = [merge_end_observations([obs(v)]) for v in verdicts]
    assert len({m.verdict for m in merged}) == 5
    assert [type(m) for m in merged].count(WallEnd) == 1
    assert [type(m) for m in merged].count(LowerBoundEnd) == 4


# ---------------------------------------------------------------------------
# Criterion 2 — a TORQUE-LIMITED end CANNOT BE READ AS A WALL.
#
# Structural: the wrong thing is unrepresentable, not merely documented.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("verdict", _NOT_A_WALL)
def test_a_wall_end_cannot_be_constructed_from_non_wall_evidence(verdict: LimitVerdict):
    """The laundering is impossible at the constructor: WallEnd REFUSES the evidence.

    This is what makes the distinction structural. There is no code path — no
    merge, no promotion, no helper — that can hand a WallEnd a torque-limited
    stall, because the type itself will not hold one.
    """
    evidence = obs(verdict)
    with pytest.raises(ValueError, match="WALL"):
        WallEnd(
            joint=evidence.joint,
            end=evidence.end,
            reference_raw=evidence.origin_raw,
            extent=evidence.displacement,
            evidence=evidence,
        )


def test_a_lower_bound_end_cannot_be_constructed_from_wall_evidence():
    """The converse also holds — the two types partition the verdict space."""
    evidence = obs(LimitVerdict.WALL)
    with pytest.raises(ValueError, match="WALL"):
        LowerBoundEnd(
            joint=evidence.joint,
            end=evidence.end,
            reference_raw=evidence.origin_raw,
            extent=evidence.displacement,
            evidence=evidence,
        )


def test_the_base_measured_end_cannot_be_instantiated():
    """An 'unclassified' end is exactly the laundering this type exists to stop.

    Every MeasuredEnd is a WallEnd or a LowerBoundEnd. There is no third state in
    which a record holds a tick without saying whether it can vouch for it.
    """
    evidence = obs(LimitVerdict.WALL)
    with pytest.raises(TypeError, match="WallEnd|LowerBoundEnd"):
        MeasuredEnd(
            joint=evidence.joint,
            end=evidence.end,
            reference_raw=evidence.origin_raw,
            extent=evidence.displacement,
            evidence=evidence,
        )


def test_a_range_with_a_torque_limited_end_yields_no_mechanical_range():
    """Criterion 2, on the RANGE: one bad end poisons the pair, and says so."""
    low = merge_end_observations([obs(LimitVerdict.WALL, end=TravelEnd.LOW, displacement=-800)])
    high = merge_end_observations([obs(LimitVerdict.TORQUE_LIMITED, displacement=500)])
    travel = JointTravel(joint="shoulder_lift", low=low, high=high)

    # The OBSERVED envelope is always available — it claims nothing it cannot back.
    assert travel.envelope_extent == (-800, 500)
    assert travel.span == 1300

    # But there is no mechanical range to read. Not a warning field. None.
    assert travel.mechanical_extent is None
    assert travel.mechanical_raw_ends is None
    assert travel.is_fully_vouched is False
    assert travel.unvouched_ends == (TravelEnd.HIGH,)


def test_both_ends_walled_is_the_only_way_to_get_a_mechanical_range():
    """The positive case — so the None above is a refusal, not a broken accessor."""
    low = merge_end_observations([obs(LimitVerdict.WALL, end=TravelEnd.LOW, displacement=-800)])
    high = merge_end_observations([obs(LimitVerdict.WALL, displacement=500)])
    travel = JointTravel(joint="shoulder_lift", low=low, high=high)

    assert travel.is_fully_vouched is True
    assert travel.unvouched_ends == ()
    assert travel.mechanical_extent == (-800, 500)
    assert travel.mechanical_raw_ends == (1248, 2548)


@pytest.mark.parametrize("bad_end", [TravelEnd.LOW, TravelEnd.HIGH])
def test_no_mechanical_attribute_survives_an_unvouched_end(bad_end: TravelEnd):
    """THE BACK-DOOR GUARD: nothing named 'mechanical' returns a value when unvouched.

    The structural claim is only as good as its smallest hole. A future
    ``mechanical_range`` property that forgot to check the verdict would hand a
    caller a (min, max) pair indistinguishable from a real wall — the exact lie
    this record exists to prevent. So rather than enumerate today's accessors,
    reflect over EVERY public attribute of the record and require that any of
    them claiming to be 'mechanical' is None whenever an end is not a wall.
    """
    walled = merge_end_observations(
        [obs(LimitVerdict.WALL, end=bad_end.opposite, displacement=500 * bad_end.opposite.sign)]
    )
    stalled = merge_end_observations(
        [obs(LimitVerdict.TORQUE_LIMITED, end=bad_end, displacement=500 * bad_end.sign)]
    )
    low, high = (stalled, walled) if bad_end is TravelEnd.LOW else (walled, stalled)
    travel = JointTravel(joint="shoulder_lift", low=low, high=high)

    mechanical_attrs = [name for name in dir(travel) if "mechanical" in name.lower()]
    assert mechanical_attrs, "the record must expose a 'mechanical' accessor to guard"

    for name in mechanical_attrs:
        value = getattr(travel, name)
        assert value is None, (
            f"JointTravel.{name} returned {value!r} on a range whose {bad_end.value} end is "
            f"TORQUE-LIMITED. A torque-limited end is a LOWER BOUND — it must never be "
            f"readable as a mechanical limit."
        )

    # The same guard, on the END itself.
    for name in [n for n in dir(stalled) if "mechanical" in n.lower()]:
        assert getattr(stalled, name) is None


# ---------------------------------------------------------------------------
# Criterion 3 — no number of poses promotes a TORQUE-LIMITED end.
# ---------------------------------------------------------------------------


def test_a_thousand_torque_limited_poses_still_do_not_make_a_wall():
    """ "...no matter how many poses are recorded."

    The arm's own weakness shrinks the range in EVERY pose, so the max over poses
    never escapes it. Piling on evidence must not tip it over.
    """
    observations = [
        obs(LimitVerdict.TORQUE_LIMITED, displacement=400 + i, pose=f"pose-{i}")
        for i in range(1000)
    ]
    end = merge_end_observations(observations)

    assert isinstance(end, LowerBoundEnd)
    assert end.mechanical_limit is None
    assert end.pose_count == 1000
    # It DID widen the observed envelope — the merge is doing its job, it just
    # refuses to call the result a wall.
    assert end.extent == 1399


def test_the_widest_wall_across_poses_is_the_mechanical_limit():
    """The max-over-poses rule, where it IS sound: an obstacle only ever narrows.

    Three poses, three walls, at 400 / 900 / 650 ticks out. The widest is the one
    that converges on the mechanical truth — the narrower two were obstacles.
    """
    end = merge_end_observations(
        [
            obs(LimitVerdict.WALL, displacement=400, pose="a"),
            obs(LimitVerdict.WALL, displacement=900, pose="b"),
            obs(LimitVerdict.WALL, displacement=650, pose="c"),
        ]
    )

    assert isinstance(end, WallEnd)
    assert end.extent == 900
    assert end.mechanical_limit == 2048 + 900
    assert end.evidence.pose == "b"


def test_a_torque_limited_stall_BEYOND_the_widest_wall_demotes_the_end():
    """The subtle case, and the reason 'outermost observation wins' is the right rule.

    Pose A hits a wall 400 ticks out. Pose B *stalls* 700 ticks out — which proves
    the joint physically travelled PAST pose A's wall, so that wall was an obstacle
    (environmental), not the mechanical limit. The end is therefore a LOWER BOUND at
    700: we know it reaches at least that far, and we know of no wall there.

    Promoting pose A's wall to the mechanical limit here would be strictly worse
    than useless — it would record a limit the arm has already been observed to
    cross.
    """
    end = merge_end_observations(
        [
            obs(LimitVerdict.WALL, displacement=400, pose="a"),
            obs(LimitVerdict.TORQUE_LIMITED, displacement=700, pose="b"),
        ]
    )

    assert isinstance(end, LowerBoundEnd)
    assert end.verdict is LimitVerdict.TORQUE_LIMITED
    assert end.mechanical_limit is None
    assert end.extent == 700


def test_a_wall_beyond_a_torque_limited_stall_is_the_mechanical_limit():
    """The mirror of the above: the outermost observation is a WALL, so it vouches."""
    end = merge_end_observations(
        [
            obs(LimitVerdict.TORQUE_LIMITED, displacement=400, pose="a"),
            obs(LimitVerdict.WALL, displacement=700, pose="b"),
        ]
    )

    assert isinstance(end, WallEnd)
    assert end.extent == 700
    assert end.mechanical_limit == 2748


def test_a_wall_and_a_stall_at_the_same_tick_resolve_to_the_wall():
    """A tie at the same extent goes to the WALL — it is the stronger evidence.

    Deterministic, and in the safe direction: the stall is consistent with the wall
    being exactly there, and the wall is the only observation that saw anything.
    """
    end = merge_end_observations(
        [
            obs(LimitVerdict.TORQUE_LIMITED, displacement=500, pose="a"),
            obs(LimitVerdict.WALL, displacement=500, pose="b"),
        ]
    )
    assert isinstance(end, WallEnd)
    assert end.evidence.pose == "b"


@pytest.mark.parametrize("verdict", _NOT_A_WALL)
def test_merging_never_invents_a_wall_from_verdicts_that_have_none(verdict: LimitVerdict):
    """Whatever the mix of NON-wall verdicts, the merge cannot produce a WallEnd."""
    observations = [obs(verdict, displacement=d, pose=f"p{d}") for d in (100, 800, 450)]
    end = merge_end_observations(observations)

    assert isinstance(end, LowerBoundEnd)
    assert end.mechanical_limit is None
    assert end.extent == 800


def test_merge_joint_travel_folds_both_ends_against_one_reference():
    """The whole joint: both ends merged across poses, on a single shared frame."""
    travel = merge_joint_travel(
        [
            obs(LimitVerdict.WALL, end=TravelEnd.LOW, displacement=-300, pose="a"),
            obs(LimitVerdict.WALL, end=TravelEnd.LOW, displacement=-900, pose="b"),
            obs(LimitVerdict.TORQUE_LIMITED, end=TravelEnd.HIGH, displacement=600, pose="a"),
            obs(LimitVerdict.TORQUE_LIMITED, end=TravelEnd.HIGH, displacement=700, pose="b"),
        ]
    )

    assert travel.joint == "shoulder_lift"
    assert isinstance(travel.low, WallEnd)
    assert isinstance(travel.high, LowerBoundEnd)
    assert travel.envelope_extent == (-900, 700)
    assert travel.span == 1600
    # One end unvouched -> no mechanical range for the joint.
    assert travel.mechanical_extent is None
    assert travel.unvouched_ends == (TravelEnd.HIGH,)


# ---------------------------------------------------------------------------
# The frame: displacement accumulates in RAW ticks, and the seam must not bite.
# ---------------------------------------------------------------------------


def test_signed_delta_takes_the_short_way_round_the_seam():
    """raw 4090 and raw 10 are 16 ticks apart, not 4080 — the seam is not a distance."""
    assert signed_delta(10, 4090) == 16
    assert signed_delta(4090, 10) == -16
    assert signed_delta(2048, 2048) == 0


def test_travel_across_the_seam_is_measured_by_displacement_not_by_tick_order():
    """A probe that crosses raw 4095->0 has NOT gone backwards.

    This is the seam masquerading as a limit, in miniature. The joint starts at raw
    4000 and creeps 200 ticks up: it ends at raw 104. A record that compared raw
    TICKS would call that a retreat of 3896. Displacement (raw, accumulated) says
    what actually happened: it went 200 ticks further out.
    """
    crossed = obs(LimitVerdict.WALL, origin_raw=4000, displacement=200)
    assert crossed.raw_tick == 104
    assert crossed.extent_from(4000) == 200

    end = merge_end_observations([crossed])
    assert end.raw_tick == 104
    assert end.mechanical_limit == 104

    # And it is judged FURTHER OUT than a pose that stopped at raw 4090.
    stopped_short = obs(LimitVerdict.WALL, origin_raw=4000, displacement=90)
    widest = merge_end_observations([stopped_short, crossed])
    assert widest.extent == 200
    assert widest.raw_tick == 104


def test_poses_that_start_at_different_raw_positions_are_compared_on_one_frame():
    """Two poses whose probes started 100 ticks apart still compare correctly.

    Displacement alone is measured from each pose's OWN start, so it cannot be
    compared across poses; the merge re-expresses every observation against one
    shared reference before asking which went furthest.
    """
    near = obs(LimitVerdict.WALL, origin_raw=2000, displacement=500, pose="near")  # raw 2500
    far = obs(LimitVerdict.WALL, origin_raw=2100, displacement=450, pose="far")  # raw 2550

    # 'far' has the SMALLER displacement but ends further out. Displacement alone
    # would pick the wrong one.
    assert far.displacement < near.displacement
    end = merge_end_observations([near, far])
    assert end.evidence.pose == "far"
    assert end.raw_tick == 2550


def test_poses_that_start_either_side_of_the_seam_are_compared_correctly():
    """The same, when the two poses' start positions straddle the seam."""
    below = obs(LimitVerdict.WALL, origin_raw=4090, displacement=100, pose="below")  # raw 94
    above = obs(LimitVerdict.WALL, origin_raw=10, displacement=100, pose="above")  # raw 110

    end = merge_end_observations([below, above])
    assert end.evidence.pose == "above"
    assert end.raw_tick == 110


# ---------------------------------------------------------------------------
# Validation — a record that cannot be trusted must not be constructible.
# ---------------------------------------------------------------------------


def test_displacement_sign_must_match_the_end_it_probes():
    """A HIGH end that travelled DOWN is not a measurement, it is a bug."""
    with pytest.raises(ValueError, match="displacement"):
        obs(LimitVerdict.WALL, end=TravelEnd.HIGH, displacement=-100)
    with pytest.raises(ValueError, match="displacement"):
        obs(LimitVerdict.WALL, end=TravelEnd.LOW, displacement=100)
    # Zero is legal at either end — a probe can stall before it moves.
    assert obs(LimitVerdict.WALL, end=TravelEnd.HIGH, displacement=0).displacement == 0
    assert obs(LimitVerdict.WALL, end=TravelEnd.LOW, displacement=0).displacement == 0


def test_displacement_may_not_exceed_one_full_turn():
    """Past a full turn there is nothing left to learn — the joint is CONTINUOUS.

    Bounding displacement at one turn is also what keeps the cross-pose comparison
    honest: every observation lands within one lap of the shared reference.
    """
    assert obs(LimitVerdict.EDGE, displacement=ENCODER_TICKS).displacement == 4096
    with pytest.raises(ValueError, match="full turn"):
        obs(LimitVerdict.EDGE, displacement=ENCODER_TICKS + 1)


def test_origin_must_be_a_raw_tick():
    with pytest.raises(ValueError, match="origin_raw"):
        obs(LimitVerdict.WALL, origin_raw=4096)
    with pytest.raises(ValueError, match="origin_raw"):
        obs(LimitVerdict.WALL, origin_raw=-1)


def test_a_negative_load_is_not_a_load():
    with pytest.raises(ValueError, match="load"):
        obs(LimitVerdict.WALL, load=-1)


def test_a_joint_must_be_named():
    with pytest.raises(ValueError, match="joint"):
        obs(LimitVerdict.WALL, joint="")


def test_a_record_whose_number_disagrees_with_its_own_evidence_is_refused():
    """A MeasuredEnd must be exactly as far out as the observation it stands on.

    Without this the record could carry any extent it liked while pointing at an
    unrelated probe — and every downstream report would name a pose and a load that
    had nothing to do with the number beside them.
    """
    evidence = obs(LimitVerdict.WALL, origin_raw=2048, displacement=500)
    with pytest.raises(ValueError, match="does not match its own evidence"):
        WallEnd(
            joint="shoulder_lift",
            end=TravelEnd.HIGH,
            reference_raw=2048,
            extent=900,  # the evidence says 500
            evidence=evidence,
        )


def test_a_record_cannot_point_at_evidence_from_another_joint_or_another_end():
    evidence = obs(LimitVerdict.WALL, joint="gripper")
    with pytest.raises(ValueError, match="evidence is for joint"):
        WallEnd(
            joint="elbow_flex",
            end=TravelEnd.HIGH,
            reference_raw=2048,
            extent=500,
            evidence=evidence,
        )

    low_evidence = obs(LimitVerdict.WALL, end=TravelEnd.LOW, displacement=-500)
    with pytest.raises(ValueError, match="evidence is for the low end"):
        WallEnd(
            joint="shoulder_lift",
            end=TravelEnd.HIGH,
            reference_raw=2048,
            extent=-500,
            evidence=low_evidence,
        )


def test_a_record_needs_evidence_and_at_least_one_pose_behind_it():
    evidence = obs(LimitVerdict.WALL)
    with pytest.raises(ValueError, match="must be an EndObservation"):
        WallEnd(
            joint="shoulder_lift",
            end=TravelEnd.HIGH,
            reference_raw=2048,
            extent=500,
            evidence="a wall, honest",  # type: ignore[arg-type]
        )
    with pytest.raises(ValueError, match="pose_count"):
        WallEnd(
            joint="shoulder_lift",
            end=TravelEnd.HIGH,
            reference_raw=2048,
            extent=500,
            evidence=evidence,
            pose_count=0,
        )


def test_two_ends_measured_against_different_frames_are_not_commensurable():
    """Extents from different references cannot be compared, so the pair is refused.

    A JointTravel whose ends were measured in different frames would report a span
    that is off by the distance between them — a fabricated number wearing a
    measurement's clothes.
    """
    low = merge_end_observations(
        [obs(LimitVerdict.WALL, end=TravelEnd.LOW, displacement=-500)], reference_raw=2048
    )
    high = merge_end_observations([obs(LimitVerdict.WALL, displacement=500)], reference_raw=2000)
    with pytest.raises(ValueError, match="different reference frames"):
        JointTravel(joint="shoulder_lift", low=low, high=high)


def test_a_low_end_that_sits_above_its_high_end_is_refused():
    """Correctly-labelled ends, impossible geometry — still not a travel."""
    low = WallEnd(
        joint="shoulder_lift",
        end=TravelEnd.LOW,
        reference_raw=2048,
        extent=352,  # this pose's LOW end sits ABOVE the reference
        evidence=obs(LimitVerdict.WALL, end=TravelEnd.LOW, origin_raw=2500, displacement=-100),
    )
    high = WallEnd(
        joint="shoulder_lift",
        end=TravelEnd.HIGH,
        reference_raw=2048,
        extent=0,
        evidence=obs(LimitVerdict.WALL, origin_raw=2048, displacement=0),
    )
    with pytest.raises(ValueError, match="low end"):
        JointTravel(joint="shoulder_lift", low=low, high=high)


def test_a_joint_travel_end_must_be_a_measured_end():
    high = merge_end_observations([obs(LimitVerdict.WALL)])
    with pytest.raises(ValueError, match="must be a MeasuredEnd"):
        JointTravel(joint="shoulder_lift", low=2048, high=high)  # type: ignore[arg-type]


def test_an_unnamed_joint_is_refused_by_every_record():
    evidence = obs(LimitVerdict.WALL)
    with pytest.raises(ValueError, match="joint"):
        WallEnd(joint="", end=TravelEnd.HIGH, reference_raw=2048, extent=500, evidence=evidence)
    low = merge_end_observations([obs(LimitVerdict.WALL, end=TravelEnd.LOW, displacement=-500)])
    high = merge_end_observations([obs(LimitVerdict.WALL)])
    with pytest.raises(ValueError, match="joint"):
        JointTravel(joint="", low=low, high=high)


def test_merge_joint_travel_refuses_an_empty_list_or_mixed_joints():
    with pytest.raises(ValueError, match="no observations"):
        merge_joint_travel([])
    with pytest.raises(ValueError, match="same joint"):
        merge_joint_travel(
            [
                obs(LimitVerdict.WALL, joint="elbow_flex", end=TravelEnd.LOW, displacement=-100),
                obs(LimitVerdict.WALL, joint="gripper", end=TravelEnd.HIGH, displacement=100),
            ]
        )


def test_merging_an_empty_list_is_a_refusal_not_an_empty_record():
    """Zero observations must not yield a record. A record with no evidence behind
    it is the purest form of the laundering this module exists to prevent."""
    with pytest.raises(ValueError, match="no observations"):
        merge_end_observations([])


def test_merging_mixed_joints_or_mixed_ends_is_refused():
    with pytest.raises(ValueError, match="joint"):
        merge_end_observations(
            [obs(LimitVerdict.WALL, joint="elbow_flex"), obs(LimitVerdict.WALL, joint="gripper")]
        )
    with pytest.raises(ValueError, match="end"):
        merge_end_observations(
            [
                obs(LimitVerdict.WALL, end=TravelEnd.HIGH, displacement=100),
                obs(LimitVerdict.WALL, end=TravelEnd.LOW, displacement=-100),
            ]
        )


def test_merge_joint_travel_needs_both_ends():
    with pytest.raises(ValueError, match="both ends"):
        merge_joint_travel([obs(LimitVerdict.WALL, end=TravelEnd.HIGH)])


def test_a_joint_travel_whose_ends_disagree_about_the_joint_is_refused():
    low = merge_end_observations(
        [obs(LimitVerdict.WALL, joint="elbow_flex", end=TravelEnd.LOW, displacement=-100)]
    )
    high = merge_end_observations([obs(LimitVerdict.WALL, joint="gripper", displacement=100)])
    with pytest.raises(ValueError, match="joint"):
        JointTravel(joint="elbow_flex", low=low, high=high)


def test_a_joint_travel_whose_ends_are_the_wrong_way_round_is_refused():
    high = merge_end_observations([obs(LimitVerdict.WALL, end=TravelEnd.HIGH, displacement=100)])
    low = merge_end_observations([obs(LimitVerdict.WALL, end=TravelEnd.LOW, displacement=-100)])
    with pytest.raises(ValueError, match="low end"):
        JointTravel(joint="shoulder_lift", low=high, high=low)
    # ...and a 'high' slot holding a second LOW end is refused too, rather than
    # quietly recording a joint that only ever travelled one way.
    with pytest.raises(ValueError, match="high end"):
        JointTravel(joint="shoulder_lift", low=low, high=low)


# ---------------------------------------------------------------------------
# Serialization — the JSON door is not a back door.
# ---------------------------------------------------------------------------


def test_records_round_trip_through_plain_json():
    travel = merge_joint_travel(
        [
            obs(LimitVerdict.WALL, end=TravelEnd.LOW, displacement=-900, load=480, pose="a"),
            obs(LimitVerdict.TORQUE_LIMITED, end=TravelEnd.HIGH, displacement=700, load=500),
        ]
    )
    payload = travel.to_dict()
    # Plain JSON only — no enums, no dataclasses, nothing that needs a custom encoder.
    restored = JointTravel.from_dict(json.loads(json.dumps(payload)))

    assert restored == travel
    assert isinstance(restored.low, WallEnd)
    assert isinstance(restored.high, LowerBoundEnd)
    assert restored.mechanical_extent is None


def test_a_hand_edited_payload_cannot_smuggle_in_a_wall():
    """Relabelling a torque-limited end as a wall in the JSON must not deserialize.

    The type refuses it on the way back in, exactly as it refuses it on the way in.
    A file is not a loophole.
    """
    stalled = merge_end_observations([obs(LimitVerdict.TORQUE_LIMITED)])
    payload = stalled.to_dict()
    assert payload["kind"] == "lower_bound"

    payload["kind"] = "wall"  # the lie
    with pytest.raises(ValueError, match="WALL"):
        MeasuredEnd.from_dict(payload)


def test_an_unknown_kind_is_refused_rather_than_guessed():
    payload = merge_end_observations([obs(LimitVerdict.WALL)]).to_dict()
    payload["kind"] = "probably_a_wall"
    with pytest.raises(ValueError, match="kind"):
        MeasuredEnd.from_dict(payload)


# ---------------------------------------------------------------------------
# Purity — this module never touches a bus.
# ---------------------------------------------------------------------------


def test_limits_module_never_imports_the_bus():
    """A pure record of what the hardware said — with no handle on the hardware.

    Mirrors ``test_arm_spec_module_never_imports_the_bus``: a module that never
    imports a bus has no way to issue a register read or write in the first place,
    which rules out the whole category rather than one call site.
    """
    import arm101.hardware.limits as limits_module

    tree = ast.parse(inspect.getsource(limits_module))
    imported: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module)

    assert not any(
        "bus" in name.lower() for name in imported
    ), f"limits must not import a bus module; found {imported}"


def test_importing_limits_does_not_drag_in_the_bus_transitively():
    """The AST guard above only sees DIRECT imports. This one sees the whole graph.

    Run in a fresh interpreter so nothing another test already imported can mask a
    real dependency.
    """
    code = (
        "import sys; import arm101.hardware.limits; "
        "print(sorted(m for m in sys.modules if m.startswith('arm101') and 'bus' in m))"
    )
    result = subprocess.run(  # nosec B603 — fixed argv, no shell, no user input
        [sys.executable, "-c", code],
        capture_output=True,
        text=True,
        check=True,
    )
    assert result.stdout.strip() == "[]", (
        f"importing arm101.hardware.limits pulled a bus module into sys.modules "
        f"({result.stdout.strip()}) — the record must stay pure"
    )


# ===========================================================================
# The regression, in the numbers the ARM produced (2026-07-13)
# ===========================================================================


def test_a_travel_LONGER_than_half_a_turn_reconstructs_exactly__elbow_flex_on_hardware():
    """The bug that tripped the stop-gate, pinned with the follower's own measurements.

    `arm limits elbow_flex` found BOTH walls, correctly, to within 8 ticks of values a
    human and a previous session had established — and then classified the joint
    CONTINUOUS, which is the one thing it certainly is not.

    The measurement was right; the reconstruction was wrong. The two probes share one
    frame, so the second starts where the first stopped. Relating those origins by a
    SHORTEST-signed-delta is a guess:

        low probe : from raw 76, travelled -2114 (down, THROUGH the seam) to its wall
                    at raw 2058.
        high probe: therefore started at raw 2106 (the wall, plus the back-off).

        short way from 76 to 2106 = +2030
        the way the joint ACTUALLY went = -2066
        ... 36 ticks apart, so `min` picked the wrong SIGN.

    Wrong sign => span 6393 instead of 2297. Over by exactly one full turn — which is
    precisely the threshold `CONTINUOUS` is decided on.

    The shortest-path assumption fails for ANY joint whose travel exceeds half a turn
    (2048). elbow_flex's is 2297. Those are the only joints worth measuring.
    """
    anchor = 76  # the frame's origin, shared by both probes

    low = EndObservation(
        joint="elbow_flex",
        end=TravelEnd.LOW,
        verdict=LimitVerdict.WALL,
        origin_raw=anchor,
        origin_offset=0,  # the first probe began AT the anchor
        displacement=-2114,  # ... and went the long way round, through the seam
        load=500,
    )
    high = EndObservation(
        joint="elbow_flex",
        end=TravelEnd.HIGH,
        verdict=LimitVerdict.WALL,
        origin_raw=anchor,
        origin_offset=-2066,  # THE PATH, not the +2030 shortest-way guess
        displacement=2249,
        load=500,
    )

    travel = merge_joint_travel([low, high])

    # The walls, back in raw ticks — and they are the ones the arm actually touched.
    assert travel.low.reach == 2058
    assert travel.high.reach == 259

    # THE FIX: 2297, not 6393. Under a full turn, so this joint is BOUNDED.
    assert travel.span == 2297
    assert travel.span > HALF_TURN, "past the line where a shortest-path guess breaks"
    assert travel.span < ENCODER_TICKS, "and emphatically NOT continuous"

    # And the guess that would have been made instead, spelled out so nobody re-derives it:
    assert signed_delta(2106, anchor) == 2030  # the short way...
    assert 2030 + 2249 - (-2114) == 6393  # ... and the full-turn error it produces
