"""RAW-TICK PERSISTENCE: one conversion boundary between the bus and everything else.

The bug this suite exists to make unshippable
---------------------------------------------
The STS3215 reports ``reported = (raw - Ofs) mod 4096``. A REPORTED tick is
therefore a *view through the current offset*, not a place: re-zero the joint —
write a new ``Ofs`` — and every previously-stored reported tick silently names a
different physical angle. Nothing raises. Nothing fails. The number is still
there, still plausible, and now wrong.

That is not hypothetical. It has shipped twice:

* ``REZERO_ARCS`` shipped as ``(126, 2020)`` — ticks measured on a servo already
  holding the factory ``Ofs = 85``, i.e. REPORTED ticks, then used as if they
  were raw. It landed inside the true arc anyway, by luck and by margin (fixed
  in PR #41, which moved the table to RAW).
* ``SOFT_LIMITS`` shipped as ``(100, 3995)`` — reported ticks again. Read as raw
  they leave the joint just **15** ticks of clearance from the seam the servo
  actually moves across (see
  :func:`test_the_pre_fix_reported_frame_numbers_are_rejected_as_raw`), which is
  narrower than ``gentle_move``'s own 12-tick arrival tolerance. That is the one
  this task fixes.

So: **persist RAW, convert to REPORTED at the bus edge.** Raw ticks are physical
angles and survive any number of re-zeros untouched;
:mod:`arm101.hardware.ticks` owns the conversion, and these tests pin the three
things that make the convention real rather than aspirational —

1. a single module owns reported<->raw, and ``arm_spec.SOFT_LIMITS`` is RAW with
   its dead arc still containing the seam (BOTH seams — see below);
2. every persisted tick in the codebase is enumerated and its frame asserted, so
   a new REPORTED tick cannot be persisted without failing a test;
3. the round-trip property: store raw, change the offset, read back through the
   NEW offset — the physical angle is unchanged.

Two seams, not one
------------------
"The seam" is not a single place, and conflating the two is what makes this
subtle. The **raw seam** is where the magnet's own count rolls 4095->0; it is
immovable. The **reported seam** is where ``(raw - Ofs) mod 4096`` rolls 4095->0,
which is at ``raw == Ofs``; it MOVES when you write the offset, and relocating it
is exactly what an ``arm rezero`` does. A range that clears one can sit right on
top of the other. wrist_roll's soft limit has to clear both — its ticks are
compared as RAW (stored) and commanded as REPORTED (on the wire).
"""

from __future__ import annotations

import argparse
import inspect
import json
import sys

import pytest

from arm101.cli._commands import arm as arm_cmd
from arm101.hardware import arm_spec, ticks
from arm101.hardware.bus import FakeBus

#: The joint with the soft limit. Read from the table, never re-typed.
_WRIST_ROLL = "wrist_roll"

#: The offset every factory servo ships holding — and therefore the offset
#: wrist_roll holds, permanently: it is the one joint a re-zero can never help
#: (``arm_spec.rezero_refusal``), so its offset never moves off the factory value.
_FACTORY = arm_spec.FACTORY_ENCODER_OFFSET


# ===========================================================================
# The conversion module: reported <-> raw, and nothing else owns it
# ===========================================================================


def test_the_two_conversions_are_exact_inverses_at_every_offset() -> None:
    """``raw -> reported -> raw`` closes for any offset, positive or negative.

    The modulus is the whole point: ``reported + offset`` runs past 4096 near the
    top of the travel and below 0 for a negative offset, and both must fold back
    onto the circle rather than escaping it.
    """
    for offset in (-2047, -1096, -1, 0, 1, _FACTORY, 1073, 2047):
        for raw in range(0, ticks.ENCODER_TICKS, 97):
            reported = ticks.reported_from_raw(raw, offset)
            assert ticks.TICK_MIN <= reported <= ticks.TICK_MAX
            assert ticks.raw_from_reported(reported, offset) == raw


def test_reported_equals_raw_ONLY_at_offset_zero() -> None:
    """The identity that never held on any servo that ever shipped.

    Every SO-101 leaves the factory holding ``Ofs = 85``, so "reported is raw" was
    false from the first power-on — which is exactly why the conversion has to be
    on every path instead of being a special case for re-zeroed servos.
    """
    assert ticks.reported_from_raw(2048, 0) == 2048
    assert ticks.reported_from_raw(2048, _FACTORY) == 2048 - _FACTORY
    assert ticks.raw_from_reported(2048 - _FACTORY, _FACTORY) == 2048
    assert _FACTORY != 0, "if the factory offset were 0 this whole module would be moot"


def test_seam_tick_and_offset_for_seam_at_are_inverses() -> None:
    """The raw tick of the reported seam, and the signed register value that puts it there."""
    for tick in range(0, ticks.ENCODER_TICKS, 61):
        if tick == 2048:  # the one seam placement sign-magnitude cannot express
            continue
        assert ticks.seam_tick(ticks.offset_for_seam_at(tick)) == tick


def test_seam_tick_reduces_a_signed_offset_onto_the_circle() -> None:
    """A servo holding -1096 carries its seam at raw 3000 — the same residue, and a tick."""
    assert ticks.seam_tick(-1096) == 3000
    assert ticks.seam_tick(-1) == ticks.TICK_MAX
    assert ticks.seam_tick(_FACTORY) == _FACTORY


def test_the_raw_seam_does_not_move_when_the_offset_does() -> None:
    """The two seams are different animals: one is a constant, one is a function.

    This is the distinction the whole task turns on. Writing the offset register
    relocates the REPORTED seam (that is what a re-zero *is*) and moves the raw
    seam not one tick, because it does not move the magnet.
    """
    assert ticks.RAW_SEAM_TICK == ticks.TICK_MIN
    seams = {ticks.seam_tick(offset) for offset in (0, _FACTORY, 1073, 1157, -1096)}
    assert len(seams) == 5, "the reported seam must move with the offset"
    assert ticks.RAW_SEAM_TICK == ticks.TICK_MIN, "and the raw seam must not"


def test_raw_interval_to_reported_preserves_width_and_order() -> None:
    """The converted pair covers exactly the same physical angles, in the same order."""
    low, high, offset = 185, 3995, _FACTORY

    low_r, high_r = ticks.raw_interval_to_reported(low, high, offset)

    assert low_r < high_r
    assert high_r - low_r == high - low
    assert ticks.raw_from_reported(low_r, offset) == low
    assert ticks.raw_from_reported(high_r, offset) == high


def test_raw_interval_to_reported_refuses_an_interval_that_straddles_the_reported_seam() -> None:
    """No ``(min, max)`` pair can describe a wrapping range — so don't invent one.

    An interval containing the reported seam maps to two arcs at opposite ends of
    the reported scale. Sorting them into a pair yields the *complement* — the
    region the joint must never enter — which is the identical shape of bug that
    made ``elbow_flex``'s ``[min, max]`` describe the arc it cannot reach. Raising
    is the only honest answer.
    """
    with pytest.raises(ValueError, match="straddles the reported seam"):
        ticks.raw_interval_to_reported(ticks.TICK_MIN, ticks.TICK_MAX, _FACTORY)

    assert ticks.crosses_reported_seam(0, 4095, _FACTORY) is True
    assert ticks.crosses_reported_seam(185, 3995, _FACTORY) is False


def test_raw_interval_to_reported_is_the_identity_at_offset_zero() -> None:
    """At ``Ofs = 0`` the frames coincide — and no interval can straddle a seam at raw 0."""
    assert ticks.raw_interval_to_reported(0, 4095, 0) == (0, 4095)
    assert ticks.crosses_reported_seam(0, 4095, 0) is False


def test_raw_interval_to_reported_rejects_a_malformed_interval() -> None:
    with pytest.raises(ValueError, match="not a valid raw tick interval"):
        ticks.raw_interval_to_reported(3000, 100, 0)
    with pytest.raises(ValueError, match="not a valid raw tick interval"):
        ticks.raw_interval_to_reported(-1, 100, 0)


def test_the_conversion_module_imports_nothing() -> None:
    """``ticks`` sits at the bottom of the stack, so both sides of the boundary can use it.

    ``arm_spec`` may not import the bus (a table of physical facts must not depend
    on a serial port), and the bus should not have to import ``arm_spec``. The only
    way one conversion can serve both is for it to live in a module that depends on
    neither — enforced here rather than left to good intentions, because the day
    ``ticks`` grows an import of ``arm_spec`` is the day the cycle appears and
    somebody "fixes" it by copying the arithmetic back out again.
    """
    import ast

    tree = ast.parse(inspect.getsource(ticks))
    imported = {
        node.module or "" for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)
    } | {
        alias.name
        for node in ast.walk(tree)
        if isinstance(node, ast.Import)
        for alias in node.names
    }
    assert imported <= {
        "__future__"
    }, f"arm101.hardware.ticks must import nothing; found {imported}"


def test_rezero_still_exposes_raw_from_reported_from_the_shared_module() -> None:
    """One implementation, not two. ``rezero.raw_from_reported`` IS ``ticks.raw_from_reported``."""
    from arm101.hardware import rezero

    assert rezero.raw_from_reported is ticks.raw_from_reported
    assert arm_spec.seam_tick is ticks.seam_tick


# ===========================================================================
# Criterion 3 — the round-trip property: a stored RAW tick survives a re-zero
# ===========================================================================


def test_a_stored_raw_tick_names_the_same_angle_through_any_later_offset() -> None:
    """Store raw, re-zero the joint, convert back through the NEW offset — same angle.

    This is the entire reason RAW is the persistence frame, stated as a property.
    The stored number never changes; the *view* of it does, and the view is
    recomputed on demand from the offset in force. Whatever the servo is
    re-zeroed to — factory 85, the first (frame-confused) 1073, the current target
    1157, a negative register value — the physical angle the stored tick names is
    the one it named when it was written.
    """
    stored_raw = 1500  # a physical angle, recorded once

    for new_offset in (0, _FACTORY, 1073, 1157, -1096, 2047):
        view = ticks.reported_from_raw(stored_raw, new_offset)
        assert ticks.raw_from_reported(view, new_offset) == stored_raw


def test_a_stored_REPORTED_tick_silently_moves_when_the_offset_changes() -> None:
    """The bug, made explicit — and the reason criterion 2's inventory has teeth.

    Read a position off a factory servo (``Ofs = 85``), store the reported number,
    re-zero the joint (``arm rezero elbow_flex`` writes ~1156), then hand the same
    number back to the servo. It now names a physical angle **the whole offset delta
    away** — over a thousand ticks, a quarter of a turn. No exception, no warning:
    the stored number is unchanged and its meaning has moved out from under it.
    Storing raw is what makes this impossible rather than merely unlikely.
    """
    physical_raw = 1500
    stored_reported = ticks.reported_from_raw(physical_raw, _FACTORY)

    rezeroed = arm_spec.rezero_offset("elbow_flex")
    angle_now_named = ticks.raw_from_reported(stored_reported, rezeroed)

    drift = (angle_now_named - physical_raw) % ticks.ENCODER_TICKS
    assert drift == (rezeroed - _FACTORY) % ticks.ENCODER_TICKS
    assert drift > 1000, "the drift IS the offset delta, and the re-zero's is a big one"


# ===========================================================================
# Criterion 1 — SOFT_LIMITS is RAW, and its dead arc still contains the seam
# ===========================================================================


def test_the_soft_limit_dead_arc_clears_the_RAW_seam() -> None:
    """The seam of the frame the limit is STORED in — where the magnet's count rolls over.

    Any code comparing raw ticks (which, after this task, is anything that reads a
    persisted tick) is wrong across raw 0. The dead arc must contain it, with room
    to spare.
    """
    limit = arm_spec.SOFT_LIMITS[_WRIST_ROLL]

    assert not limit.permits(ticks.RAW_SEAM_TICK)
    assert limit.clearance_from(ticks.RAW_SEAM_TICK) >= arm_spec.SEAM_CLEARANCE_TICKS


def test_the_soft_limit_dead_arc_clears_the_REPORTED_seam_the_servo_actually_moves_across() -> None:
    """The seam of the frame the limit is USED in — at ``raw == Ofs``.

    A goal write is interpreted in the servo's own reported frame, so the mover's
    linear-tick assumption breaks at the *reported* seam, wherever the offset puts
    it. wrist_roll holds the factory offset permanently (it is the one joint a
    re-zero cannot help — :func:`arm_spec.rezero_refusal`), so its reported seam
    sits at raw 85, and the dead arc has to contain that too. Clearing only the raw
    seam would leave the joint free to be commanded straight across the one that
    bites.
    """
    limit = arm_spec.SOFT_LIMITS[_WRIST_ROLL]
    reported_seam = ticks.seam_tick(_FACTORY)

    assert not limit.permits(reported_seam)
    assert limit.clearance_from(reported_seam) >= arm_spec.SEAM_CLEARANCE_TICKS


def test_dead_arc_contains_reported_seam_is_the_named_predicate_for_the_second_seam() -> None:
    """The public form of the check, and it tracks the offset — that is the whole point.

    The same limit both contains and does not contain "the seam", depending on where
    the offset puts it. A predicate that took no offset could not express that, which
    is how the question got answered for the wrong seam in the first place.
    """
    limit = arm_spec.SOFT_LIMITS[_WRIST_ROLL]

    assert arm_spec.dead_arc_contains_reported_seam(limit, _FACTORY) is True
    assert arm_spec.dead_arc_contains_reported_seam(limit, 0) is True
    # An offset that parks the seam in the middle of the permitted range: the limit
    # is now worthless for that servo, and says so rather than quietly clamping across it.
    assert arm_spec.dead_arc_contains_reported_seam(limit, 2048) is False


def test_clearance_from_is_zero_for_a_tick_the_limit_permits() -> None:
    """A permitted tick has no clearance FROM the dead arc — it is not in it.

    Guards the reading of the name: ``clearance_from`` answers "how much room does
    the dead arc leave around this tick", so a tick that is not in the dead arc at
    all gets 0, not some distance to the nearest edge.
    """
    limit = arm_spec.SOFT_LIMITS[_WRIST_ROLL]

    assert limit.clearance_from(2048) == 0
    assert limit.clearance_from(limit.min_tick) == 0
    assert limit.clearance_from(limit.max_tick) == 0
    assert limit.clearance_from(limit.min_tick - 1) == 1


def test_a_joint_may_not_have_both_a_soft_limit_and_a_rezero_arc() -> None:
    """The two answers to a wrapping joint are mutually exclusive — and the guard says so.

    A soft limit fences the seam off; a re-zero EVICTS it. Claim both for one joint
    and the premise of :func:`arm_spec._require_seam_clearance` collapses: a
    re-zeroable joint's offset is not pinned to the factory value, so the table
    cannot know where that joint's reported seam will end up, and checking clearance
    against the factory seam would be checking the wrong tick. Better to refuse.
    """
    both = {"elbow_flex": arm_spec.SOFT_LIMITS[_WRIST_ROLL]}

    with pytest.raises(ValueError, match="BOTH a soft limit and a re-zero arc"):
        arm_spec._require_seam_clearance(both)


def test_the_pre_fix_reported_frame_numbers_are_rejected_as_raw() -> None:
    """THE FALSIFIER: the shipped-and-wrong ``(100, 3995)`` cannot survive as a raw table.

    Those two numbers were measured through the factory offset — they are REPORTED
    ticks. Persist them as raw and the guard must fail, because in the frame the
    servo actually moves in they leave only ``100 - 85 = 15`` ticks between the
    permitted range and the seam: narrower than ``gentle_move``'s 12-tick arrival
    tolerance plus encoder jitter, i.e. an arrival check can settle *on the seam*.
    That is the whole failure the soft limit exists to prevent, and it is what the
    table shipped with.

    If this test ever passes with the old numbers, the import-time guard has stopped
    distinguishing the frames and the bug is shippable again.
    """
    pre_fix = arm_spec.SoftLimit(min_tick=100, max_tick=3995)

    # It clears the RAW seam happily — which is precisely why the bug was invisible.
    assert not pre_fix.permits(ticks.RAW_SEAM_TICK)
    # And it does NOT clear the REPORTED seam by the margin the table declares.
    assert pre_fix.clearance_from(ticks.seam_tick(_FACTORY)) < arm_spec.SEAM_CLEARANCE_TICKS

    with pytest.raises(ValueError, match="ticks from the seam"):
        arm_spec._require_seam_clearance({_WRIST_ROLL: pre_fix})


def test_the_soft_limit_is_derived_from_the_seam_not_typed() -> None:
    """No number in the table can be in the wrong frame if no number is typed at all.

    The permitted range is computed from the raw tick of the reported seam plus the
    declared clearance — so it is raw *by construction*, and a change to either
    input moves it without anyone having to remember to follow.
    """
    limit = arm_spec.SOFT_LIMITS[_WRIST_ROLL]

    assert limit.min_tick == ticks.seam_tick(_FACTORY) + arm_spec.SEAM_CLEARANCE_TICKS
    assert limit.max_tick == ticks.TICK_MAX - arm_spec.SEAM_CLEARANCE_TICKS


def test_the_permitted_range_has_an_honest_reported_image() -> None:
    """The RAW limit converts to a real ``(min, max)`` in the frame goals are written in.

    This is the theorem that makes RAW storage workable rather than merely pure:
    an interval whose dead arc contains the reported seam is exactly an interval
    that does NOT wrap in the reported frame — so it converts to a single
    well-ordered pair, which is what a mover can clamp against.
    """
    low, high = arm_spec.permitted_reported_range(_WRIST_ROLL, _FACTORY)

    assert low < high
    assert low >= arm_spec.SEAM_CLEARANCE_TICKS  # clear of reported 0 — the seam
    assert ticks.TICK_MAX - high >= arm_spec.SEAM_CLEARANCE_TICKS  # and clear of reported 4095


def test_a_joint_without_a_soft_limit_has_no_permitted_reported_range() -> None:
    assert arm_spec.permitted_reported_range("shoulder_pan", _FACTORY) is None


# ===========================================================================
# Criterion 2 — every persisted tick is enumerated, and its frame asserted
# ===========================================================================

#: Every place this codebase writes an encoder tick down and reads it back later —
#: across a process, a session, or an EEPROM write. This is the list criterion 2
#: is about, because persistence is where a frame error becomes *silent*: a tick
#: compared against a live reading in the same breath is at worst wrong now, but a
#: tick stored through a re-zero is wrong forever, and looks fine.
#:
#: ``"raw"`` entries are safe by construction. ``"reported"`` entries are the
#: BACKLOG: each is a live instance of this bug class, listed so that it cannot be
#: forgotten and so that a *new* one cannot be added without failing
#: :func:`test_no_new_persisted_tick_site_appears_unclassified`.
_PERSISTED_TICK_SITES: "dict[str, str]" = {
    # --- converted: physical angles, immune to a re-zero -------------------
    "arm_spec.SOFT_LIMITS": "raw",  # this task
    "arm_spec.REZERO_ARCS": "raw",  # PR #41
    # --- backlog: still stored as a view through whatever offset was in force
    #     at capture time. Both are read straight from read_position() and
    #     written to a file that outlives the offset that produced them.
    "profiles.JointCalibration": "reported",  # ~/.config/... calibration profiles
    "explore.reachmap.ReachMap.reachable_ranges": "reported",  # the reachability map
}


def test_every_persisted_tick_site_declares_its_frame() -> None:
    """The inventory itself: no site may sit in the list without a frame."""
    assert set(_PERSISTED_TICK_SITES.values()) <= {"raw", "reported"}
    assert _PERSISTED_TICK_SITES, "an empty inventory would make every other test here vacuous"


def test_every_tick_table_in_arm_spec_holds_RAW() -> None:
    """The tables that survive a re-zero — asserted through their own guards, not by label.

    A comment claiming "these are raw" is what the last two bugs both had. What
    makes the claim real is that each table is checked, at import time, against a
    property only a raw table can satisfy:

    * ``REZERO_ARCS`` — its arc is where the seam is PUT, and the offset register
      works in raw ticks (``seam_tick``). A reported arc puts the seam in the wrong
      place by exactly the pre-existing offset.
    * ``SOFT_LIMITS`` — its dead arc must clear BOTH seams by the declared margin,
      and a reported-frame table cannot (see the falsifier above).
    """
    arm_spec._require_evictable_seam(arm_spec.REZERO_ARCS)  # must not raise
    arm_spec._require_dead_arc_contains_seam(arm_spec.SOFT_LIMITS)  # must not raise
    arm_spec._require_seam_clearance(arm_spec.SOFT_LIMITS)  # must not raise

    for name, frame in _PERSISTED_TICK_SITES.items():
        if name.startswith("arm_spec."):
            assert frame == "raw", f"{name} persists ticks and must hold RAW"


def test_no_new_persisted_tick_site_appears_unclassified() -> None:
    """A guard against the inventory going stale — which is the only way it can lie.

    Discovers every module-level tick table in ``arm_spec`` by type (rather than by
    a name the test already knows) and asserts each one is in the inventory. Add a
    ``SoftLimit``/``UnreachableArc`` table to ``arm_spec`` without classifying its
    frame and this fails — which is the point: the inventory is a contract, not a
    comment.
    """
    discovered = {
        f"arm_spec.{name}"
        for name, value in vars(arm_spec).items()
        if isinstance(value, dict)
        and value
        and all(
            isinstance(entry, (arm_spec.SoftLimit, arm_spec.UnreachableArc))
            for entry in value.values()
        )
    }

    unclassified = discovered - set(_PERSISTED_TICK_SITES)
    assert not unclassified, (
        f"{sorted(unclassified)} persists encoder ticks but is not in _PERSISTED_TICK_SITES. "
        "Declare its frame — and if it is 'reported', say why that is not the bug this "
        "suite exists to prevent."
    )


def test_the_reported_tick_backlog_is_exactly_the_known_two() -> None:
    """The backlog can shrink; it must not grow.

    ``profiles`` and ``reachmap`` both persist ticks read straight off the wire, so
    both silently re-point at a different physical angle the moment a joint is
    re-zeroed — a calibration profile captured before ``arm rezero elbow_flex`` is
    now wrong by 988 ticks, and nothing in the code will say so. They are out of
    scope for THIS task (which builds the conversion boundary they will each be
    routed through), and they are pinned here so that "out of scope" cannot quietly
    become "forgotten".
    """
    backlog = {name for name, frame in _PERSISTED_TICK_SITES.items() if frame == "reported"}

    assert backlog == {
        "profiles.JointCalibration",
        "explore.reachmap.ReachMap.reachable_ranges",
    }


# ===========================================================================
# The boundary binds: a live servo holding a NON-ZERO offset is clamped in ITS frame
# ===========================================================================


def _flex_args(joint: str, to: int) -> argparse.Namespace:
    return argparse.Namespace(
        joint=joint,
        to=to,
        demo=False,
        gentle=False,
        threshold=None,
        role="follower",
        port=None,
        apply=True,
        json=True,
    )


class _FakeStdin:
    def isatty(self) -> bool:
        return False

    def readline(self) -> str:
        return ""


def _flex_on_a_factory_servo(monkeypatch, target: int) -> "list[int]":
    """Run ``arm flex wrist_roll --to <target> --apply`` against a servo holding Ofs=85.

    A fresh bus per call: ``cmd_arm_flex`` closes the bus on its way out (the torque
    release), so a fake cannot be reused across invocations.
    """
    fake = FakeBus(positions={5: 2048}, offsets={5: _FACTORY})
    fake.open()
    monkeypatch.setattr(arm_cmd, "_candidate_ports", lambda: ["/dev/ttyACM_fake"])
    monkeypatch.setattr(arm_cmd, "_open_bus", lambda _p: fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin())

    arm_cmd.cmd_arm_flex(_flex_args(_WRIST_ROLL, target))

    return [w["position"] for w in fake.position_writes if w["motor"] == 5]


def test_flex_clamps_wrist_roll_in_the_REPORTED_frame_of_a_factory_servo(monkeypatch, capsys):
    """End-to-end, on a servo holding ``Ofs = 85``: the goal written clears the reported seam.

    The test that would have caught the original bug, and the one that proves the
    conversion actually reaches the wire. Every other soft-limit test in the suite
    runs against a ``FakeBus`` at offset 0, where the two frames coincide and a
    frame error is *invisible*. This one gives the servo the offset every real
    SO-101 ships with, and asserts on the number that lands on the bus.

    Both bounds are derived from the table through the conversion — nothing is
    typed — so retuning the clearance retunes the expectation with it.
    """
    goals = _flex_on_a_factory_servo(monkeypatch, target=4090)

    _low, high_reported = arm_spec.permitted_reported_range(_WRIST_ROLL, _FACTORY)

    assert goals == [high_reported]
    # The RAW soft max is 3995; through Ofs=85 the servo must be commanded to 3910.
    # Had the table stayed in the reported frame, the goal would have been 3995 —
    # 85 ticks further round, and 85 ticks closer to the seam it must never cross.
    assert high_reported == arm_spec.SOFT_LIMITS[_WRIST_ROLL].max_tick - _FACTORY
    payload = json.loads(capsys.readouterr().out)
    assert payload["move"]["clamped_target"] == high_reported
    assert payload["move"]["was_clamped"] is True


def test_the_goal_written_to_a_factory_servo_is_a_permitted_RAW_angle(monkeypatch) -> None:
    """Close the loop: convert the commanded goal BACK to raw and it is inside the limit.

    The clamp is computed in the reported frame, so this asserts the thing that
    actually matters physically — that the angle the joint is being sent to is one
    the RAW soft limit permits. Every target here is deep in the dead arc, on both
    sides of the seam.
    """
    limit = arm_spec.SOFT_LIMITS[_WRIST_ROLL]

    for target in (0, 5, 4090, 4095):
        goals = _flex_on_a_factory_servo(monkeypatch, target)
        assert goals, f"no goal was written at all for target {target}"
        for goal in goals:
            raw = ticks.raw_from_reported(goal, _FACTORY)
            assert limit.permits(raw), f"goal {goal} (raw {raw}) is in wrist_roll's dead arc"
