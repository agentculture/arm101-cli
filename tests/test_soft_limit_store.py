"""The measured soft limit: where it is DERIVED, where it LANDS, and what READS it.

Three modules, one question
===========================
``arm limits --commit`` can measure that a joint turns all the way round — that no offset
can ever evict its seam, and a software dead arc is the only instrument left. Then it has
to put the answer somewhere. :data:`arm101.hardware.arm_spec.SOFT_LIMITS` is a checked-in
source table, and **a CLI does not rewrite its own source.** So:

* :func:`~arm101.hardware.arm_spec.soft_limit_for_offset` **derives** the limit — the same
  derivation the shipped ``wrist_roll`` entry was written by hand, now a function, so a
  measured limit and a shipped one cannot mean different things;
* :mod:`arm101.hardware.soft_limit_store` **stores** it — append-only JSONL, RAW ticks, at
  a default path, with the provenance that makes the number believable;
* :func:`~arm101.hardware.arm_spec.resolve_soft_limits` **merges** it over the shipped
  table, and :func:`~arm101.hardware.arm_spec.resolve_bounds` **binds** it — the one
  function every mover takes its bounds from.

The last of those is the only one that matters, and it is the one this repo has already
got wrong once: the shipped ``wrist_roll`` soft limit was **inert data for a whole
release**, because every mover sourced its bounds from the servo's EEPROM
``min_angle``/``max_angle`` registers — the untouched factory ``0-4095`` — and never
consulted the table at all. A committed value that nothing reads is not committed. It is
filed.
"""

from __future__ import annotations

import json

import pytest

from arm101.cli._errors import EXIT_ENV_ERROR, EXIT_USER_ERROR, CliError
from arm101.hardware import arm_spec
from arm101.hardware.arm_spec import (
    FACTORY_ENCODER_OFFSET,
    JOINTS,
    REZERO_ARCS,
    SEAM_CLEARANCE_TICKS,
    SOFT_LIMITS,
    SoftLimit,
    dead_arc_contains_reported_seam,
    dead_arc_contains_seam,
    resolve_soft_limits,
    soft_limit_for_offset,
)
from arm101.hardware.soft_limit_store import (
    SOFT_LIMIT_ENV_VAR,
    MeasuredSoftLimit,
    default_soft_limit_path,
    load_soft_limits,
    record_soft_limit,
    summarise,
)
from arm101.hardware.ticks import ENCODER_TICKS, MAX_ENCODER_OFFSET, RAW_SEAM_TICK, seam_tick

#: A joint with no shipped soft limit and no re-zero arc — the ordinary case a measurement
#: has to be able to serve.
FRESH = "shoulder_pan"


@pytest.fixture()
def store(tmp_path, monkeypatch):
    """A store of this test's own. The default lives in ``~/.arm101`` and is the operator's."""
    path = tmp_path / "soft-limits.jsonl"
    monkeypatch.setenv(SOFT_LIMIT_ENV_VAR, str(path))
    return path


# ---------------------------------------------------------------------------
# The derivation — one rule, and the shipped entry obeys it
# ---------------------------------------------------------------------------


def test_the_shipped_wrist_roll_limit_is_what_this_function_derives() -> None:
    """The hand-written entry and the derived one are the SAME limit. Not similar. Same.

    ``wrist_roll`` is the one joint a re-zero can never help — its travel covers the whole
    circle, so there is no angle to put the seam at — which means its offset never moves off
    the factory value, and its soft limit is exactly what this function computes for that
    offset. If these two ever come apart, one of them is wrong and nobody would know which:
    the table would have stopped describing the rule, and the rule would have stopped
    explaining the table.
    """
    assert SOFT_LIMITS["wrist_roll"] == soft_limit_for_offset(FACTORY_ENCODER_OFFSET)


@pytest.mark.parametrize("offset", [0, 1, FACTORY_ENCODER_OFFSET, 500, 2047, -1, -500, -2047])
def test_every_derivable_limit_fences_off_BOTH_seams_with_clearance(offset: int) -> None:
    """The whole promise a soft limit makes, checked over the offsets a servo can hold.

    * the **RAW** seam — the magnet's own 4095->0 rollover, which no offset moves;
    * the **REPORTED** seam — at raw ``Ofs``, and the one a *goal write* actually crosses.
      This is the one the shipped table originally missed, by being written in the wrong
      frame: it cleared the raw seam by a comfortable 101 ticks and the reported seam by
      **15** — under ``gentle_move``'s own arrival tolerance plus encoder jitter, so an
      arrival check could settle *on the seam*.
    """
    limit = soft_limit_for_offset(offset)

    assert dead_arc_contains_seam(limit.min_tick, limit.max_tick)
    assert dead_arc_contains_reported_seam(limit, offset)
    assert limit.clearance_from(RAW_SEAM_TICK) >= SEAM_CLEARANCE_TICKS
    assert limit.clearance_from(seam_tick(offset)) >= SEAM_CLEARANCE_TICKS


def test_a_factory_servo_pays_almost_nothing_for_its_fence() -> None:
    """The cost is the OFFSET's, not the joint's — and every SO-101 ships at Ofs = 85.

    The dead arc has to be contiguous through raw 0, so a reported seam sitting near raw 0
    can be fenced off with a sliver. One near mid-scale cannot: the permitted band is then
    one of the two halves of the circle, and the joint loses the other. That is geometry,
    not a flaw — but an operator is entitled to see the price before accepting it, which is
    why ``dead_arc_ticks`` is in the payload.
    """
    cheap = soft_limit_for_offset(FACTORY_ENCODER_OFFSET)
    assert cheap.dead_arc_ticks < ENCODER_TICKS // 10  # ~285 of 4096: under 7%

    dear = soft_limit_for_offset(ENCODER_TICKS // 2 - 1)  # the seam at mid-scale
    assert dear.dead_arc_ticks > ENCODER_TICKS // 2  # more than half the circle, gone


def test_the_wider_of_the_two_candidate_bands_is_taken() -> None:
    """A seam in the UPPER half is fenced from below, and vice versa. Nothing is given away."""
    low_seam = soft_limit_for_offset(200)  # seam at raw 200 -> permit everything above it
    assert low_seam.min_tick == 200 + SEAM_CLEARANCE_TICKS

    high_seam = soft_limit_for_offset(-200)  # seam at raw 3896 -> permit everything below it
    assert high_seam.max_tick == seam_tick(-200) - SEAM_CLEARANCE_TICKS
    assert high_seam.min_tick == SEAM_CLEARANCE_TICKS


def test_a_clearance_too_wide_to_leave_anything_is_REFUSED__not_quietly_narrowed() -> None:
    """Manufacturing a limit by shrinking the clearance would put the seam back in reach.

    The clearance is not decoration: it is ~8x the worst case of encoder jitter plus
    ``gentle_move``'s 12-tick arrival tolerance, and it is what stops a move that "arrived"
    from having settled on the discontinuity. A derivation that quietly gave up on it to
    return *something* would be handing back a fence with a hole in it.
    """
    with pytest.raises(ValueError, match="nothing would be left to permit"):
        soft_limit_for_offset(FACTORY_ENCODER_OFFSET, clearance=ENCODER_TICKS)

    with pytest.raises(ValueError, match="fences off nothing"):
        soft_limit_for_offset(FACTORY_ENCODER_OFFSET, clearance=0)


def test_EVERY_offset_the_register_can_hold_can_be_fenced() -> None:
    """Enumerated over all 4095 of them, not reasoned about. No servo is un-fenceable.

    The offset register is sign-magnitude on bit 11, so it holds ``[-2047, +2047]`` — and
    every one of those puts the reported seam somewhere a dead arc can be built around, with
    the full clearance, in one of the two candidate bands. There is no servo this instrument
    cannot be applied to, which matters: the soft limit is the LAST resort (it is what a
    joint gets when a re-zero is impossible even in principle), and a last resort with a
    hole in it is not one.

    The cost is what varies, and wildly — from ~285 ticks at the factory offset to over half
    the circle for a seam near mid-scale. It is reported, never hidden.
    """
    worst = 0
    for offset in range(-MAX_ENCODER_OFFSET, MAX_ENCODER_OFFSET + 1):
        limit = soft_limit_for_offset(offset)
        assert dead_arc_contains_seam(limit.min_tick, limit.max_tick)
        assert dead_arc_contains_reported_seam(limit, offset)
        assert limit.clearance_from(RAW_SEAM_TICK) >= SEAM_CLEARANCE_TICKS
        assert limit.clearance_from(seam_tick(offset)) >= SEAM_CLEARANCE_TICKS
        worst = max(worst, limit.dead_arc_ticks)

    # The worst case is a seam at the half-turn, which costs the joint the far half of its
    # circle. That is geometry — the dead arc must be contiguous through raw 0 — not a bug.
    assert worst > ENCODER_TICKS // 2


# ---------------------------------------------------------------------------
# The store — append-only, durable, RAW
# ---------------------------------------------------------------------------


def test_an_absent_store_is_an_empty_dict__not_an_error(store) -> None:
    """The state of every fresh checkout. Loading it must be free to do unconditionally."""
    assert not store.exists()
    assert load_soft_limits() == {}


def test_a_record_round_trips_and_carries_its_PROVENANCE(store) -> None:
    """The pair of ticks is what binds. Everything else is why anyone should believe it.

    A soft limit is a claim about a joint *in a pose*, derived against a *particular
    offset*. A record that could not say which pose, or which offset, would be two numbers
    with nothing attached — and the next person would have no way to tell a measurement
    from a guess.
    """
    limit = soft_limit_for_offset(FACTORY_ENCODER_OFFSET)
    measured = MeasuredSoftLimit(
        joint=FRESH,
        limit=limit,
        offset=FACTORY_ENCODER_OFFSET,
        kind="continuous",
        swept_ticks=ENCODER_TICKS,
        reason="swept a full turn",
        pose="elbow folded",
    )

    written = record_soft_limit(measured)
    assert written == store == default_soft_limit_path()
    assert load_soft_limits() == {FRESH: limit}

    record = json.loads(store.read_text().strip())
    assert record["frame"] == "raw"  # NOT the ticks a servo reports
    assert record["offset"] == FACTORY_ENCODER_OFFSET
    assert record["dead_arc_ticks"] == limit.dead_arc_ticks
    assert record["pose"] == "elbow folded"
    assert record["kind"] == "continuous"
    assert record["ts"]


def test_the_store_is_APPEND_ONLY_and_the_LAST_record_wins(store) -> None:
    """Re-measuring corrects the arm. It does not erase what the last run believed.

    An append is the smallest durable write there is, and keeping the history costs a line
    of JSON. A store that rewrote itself would have a window in which it held neither the
    old truth nor the new one — and would throw away the only record of what changed.
    """
    first = SoftLimit(min_tick=185, max_tick=3995)
    second = SoftLimit(min_tick=200, max_tick=3900)
    for limit in (first, second):
        record_soft_limit(MeasuredSoftLimit(joint=FRESH, limit=limit, offset=85))

    assert len(store.read_text().strip().splitlines()) == 2
    assert load_soft_limits() == {FRESH: second}


def test_a_DAMAGED_line_is_REFUSED__never_skipped(store) -> None:
    """A fence somebody meant to put up, and cannot be read, must not be silently dropped.

    The calibration journal tolerates a truncated FINAL line, because a crash mid-append
    writes one. Nothing here is written mid-motion, so a line that will not parse is
    damage — and skipping it would leave the mover free to drive across the seam while the
    file on disk says otherwise.
    """
    store.write_text('{"joint": "shoulder_pan", "min_tick": 185, "max_tick": 3995}\nnot json\n')

    with pytest.raises(CliError) as excinfo:
        load_soft_limits()
    assert excinfo.value.code == EXIT_ENV_ERROR
    assert "refused rather than skipped" in excinfo.value.remediation


def test_a_range_that_is_not_a_valid_RAW_soft_limit_is_REFUSED(store) -> None:
    """The type does the refusing, which is why there is nowhere for a bad pair to hide."""
    store.write_text('{"joint": "shoulder_pan", "min_tick": 3995, "max_tick": 185}\n')

    with pytest.raises(CliError) as excinfo:
        load_soft_limits()
    assert excinfo.value.code == EXIT_USER_ERROR
    assert "RAW encoder ticks" in excinfo.value.remediation


def test_an_unknown_joint_is_REFUSED(store) -> None:
    store.write_text('{"joint": "elbow", "min_tick": 185, "max_tick": 3995}\n')

    with pytest.raises(CliError) as excinfo:
        load_soft_limits()
    assert excinfo.value.code == EXIT_USER_ERROR
    assert "elbow" in excinfo.value.message


def test_the_table_entry_a_human_pastes_is_RENDERED_from_the_measurement(store) -> None:
    """The store makes the limit true for THIS arm. The source table is how it stops being local.

    A measurement that lives only in one operator's home directory has been made once and
    will be made again, by hand, by the next person. So the entry is printed — and it is
    rendered from the record, so it cannot claim a different number than the store holds.
    """
    limit = soft_limit_for_offset(FACTORY_ENCODER_OFFSET)
    entry = MeasuredSoftLimit(
        joint=FRESH, limit=limit, offset=85, kind="continuous", swept_ticks=4096
    ).table_entry()

    assert f'"{FRESH}": SoftLimit(min_tick={limit.min_tick}, max_tick={limit.max_tick})' in entry
    assert "continuous" in entry and "4096 ticks swept" in entry and "Ofs = 85" in entry


def test_summarise_names_the_price(store) -> None:
    assert summarise({}) == "(none)"
    text = summarise({FRESH: SoftLimit(min_tick=185, max_tick=3995)})
    assert "permitted raw [185, 3995]" in text
    assert "285 ticks fenced off" in text


# ---------------------------------------------------------------------------
# The merge — and the contradiction it refuses
# ---------------------------------------------------------------------------


def test_the_shipped_table_answers_when_nothing_was_measured() -> None:
    assert resolve_soft_limits() == SOFT_LIMITS
    assert resolve_soft_limits(from_file={}) == SOFT_LIMITS


def test_a_MEASURED_limit_beats_the_shipped_one() -> None:
    """The table is a default. The arm is the truth."""
    measured = SoftLimit(min_tick=300, max_tick=3800)
    resolved = resolve_soft_limits(from_file={"wrist_roll": measured})

    assert resolved["wrist_roll"] == measured
    assert SOFT_LIMITS["wrist_roll"] != measured  # the source table is untouched


def test_a_joint_may_not_have_BOTH_a_soft_limit_and_a_REZERO_ARC() -> None:
    """The two mutually exclusive answers to a wrapping joint. Holding both describes no arm.

    A re-zeroable joint EVICTS its seam, so its offset does not stay put — and a soft limit
    whose dead arc was placed around the seam of *one* offset fences off the wrong part of
    the circle the moment the joint is re-zeroed to another. This is enforced on the shipped
    table at import; it is enforced here on a **measured** override, where the collision is
    live rather than hypothetical.
    """
    (rezeroable,) = REZERO_ARCS  # elbow_flex — the one joint with a measured arc

    with pytest.raises(ValueError, match="BOTH a soft limit and a re-zero arc"):
        resolve_soft_limits(from_file={rezeroable: SoftLimit(min_tick=185, max_tick=3995)})


def test_a_range_that_fences_off_NOTHING_is_refused() -> None:
    """The degenerate case: a "soft limit" spanning the whole circle. It buys nothing."""
    with pytest.raises(ValueError, match="does not contain the encoder seam"):
        resolve_soft_limits(from_file={FRESH: SoftLimit(min_tick=0, max_tick=4095)})


def test_an_override_for_a_joint_that_does_not_exist_is_refused() -> None:
    with pytest.raises(ValueError, match="Unknown joint"):
        resolve_soft_limits(from_file={"elbow": SoftLimit(min_tick=185, max_tick=3995)})


# ---------------------------------------------------------------------------
# The BIND — the only part that makes any of the above mean anything
# ---------------------------------------------------------------------------


def test_a_measured_limit_NARROWS_the_bounds_every_mover_takes(store) -> None:
    """``resolve_bounds`` is the one function ``arm flex``, ``arm explore`` and the demo all use.

    On this arm the servo's own ``min_angle``/``max_angle`` are the untouched factory
    ``0-4095`` — they know nothing about the joint's real travel — so if a soft limit does
    not reach THIS function, it reaches nothing, and the fence is a file nobody opens.
    """
    limits = resolve_soft_limits(from_file={FRESH: soft_limit_for_offset(FACTORY_ENCODER_OFFSET)})

    wide = arm_spec.resolve_bounds(FRESH, 0, 4095, FACTORY_ENCODER_OFFSET)
    fenced = arm_spec.resolve_bounds(FRESH, 0, 4095, FACTORY_ENCODER_OFFSET, limits=limits)

    assert wide == (0, 4095)  # what the servo alone would permit: the whole circle
    assert fenced != wide
    low, high = fenced
    assert low > 0 and high < 4095

    # And the bounds it hands a mover are REPORTED ticks, converted from the RAW limit by
    # this servo's own live offset. A limit compared against a live read without that
    # crossing is the bug this whole module family exists to prevent.
    expected = arm_spec.permitted_reported_range(FRESH, FACTORY_ENCODER_OFFSET, limits=limits)
    assert fenced == expected


def test_a_measured_limit_NEVER_WIDENS_a_servo_that_is_configured_narrower(store) -> None:
    """Intersection, not replacement. The servo's own limits are a physical constraint.

    "Never go outside this range" is not "always permit this range". A servo whose EEPROM
    limits are genuinely narrower — an operator's calibration, a fixture, a cable route —
    is telling the truth about something a software table has no business overruling.
    """
    limits = resolve_soft_limits(from_file={FRESH: soft_limit_for_offset(FACTORY_ENCODER_OFFSET)})

    assert arm_spec.resolve_bounds(FRESH, 1000, 2000, FACTORY_ENCODER_OFFSET, limits=limits) == (
        1000,
        2000,
    )


def test_a_joint_with_no_limit_gets_its_EEPROM_bounds_back_VERBATIM(store) -> None:
    """The table must never quietly narrow a joint it says nothing about."""
    limits = resolve_soft_limits(from_file={FRESH: soft_limit_for_offset(FACTORY_ENCODER_OFFSET)})

    for joint in JOINTS:
        if joint in limits:
            continue
        assert arm_spec.resolve_bounds(joint, 0, 4095, FACTORY_ENCODER_OFFSET, limits=limits) == (
            0,
            4095,
        )
