"""t10 — ``arm limits --commit``: keeping what was measured, and REFUSING to keep what was not.

The measurement is done and the servo is back exactly as it was found (that is
``tests/test_arm_limits_cli.py``'s subject, and it is unchanged). This file is about the
separate, explicitly gated act that follows it — and about the four ways it can end.

THE SWEEP IS THE ARBITER. THE READ-BACK IS NOT.
===============================================
An encoder offset that reads back *exactly right* proves the register took the value. It
proves **nothing whatever** about whether the discontinuity moved with it. Two live
mechanisms separate those claims:

* the firmware may do a plain signed subtraction rather than reducing the corrected
  position modulo 4096 — the offset then merely RELABELS positions and the seam stays
  pinned to the physical angle where the magnet rolls over (settled on one arm, one
  firmware revision, which is not the same thing as settled);
* **the arc may be wrong.** The arm finds a joint's walls by driving into them, and an arm
  can be stopped by the table, by a cable, or by the pose it is in — anywhere short of the
  joint's real stop. Every tick of that error makes the "unreachable" arc *wider* than it
  truly is, and the seam gets parked somewhere the joint can plainly go. This is not a
  hypothetical: the first ``elbow_flex`` re-zero took an arc edge from a hand sweep
  somebody had stopped short of, and the joint came to rest eleven ticks past it.

So :class:`_ObstructedServo` models exactly that — an arm stopped short of a wall a HAND
can still reach — and ``test_an_offset_that_reads_back_PERFECTLY_still_FAILS_if_the_seam_
did_not_move`` is the acceptance criterion, run end to end through the verb.

AND A SHORT CLEAN SWEEP IS NOT A PASS
=====================================
:data:`~arm101.hardware.rezero.MIN_COVERAGE` — the ≥80% rule — is what stopped **three
empty sweeps from being declared a pass** on ``elbow_flex``: coverage of 0, 0 and 376
ticks against an expected ~2202. A naive "no discontinuity seen => PASS" would have
claimed victory on a sweep in which the arm was never touched. It is reused here whole,
and ``test_an_UNATTENDED_commit_cannot_pass_itself`` is the case it exists for: an agent
running ``--apply --commit`` with nobody at the arm polls six hundred samples of a
stationary joint, sees no seam (of course it does not), and is REFUSED.

WHERE A MEASURED SOFT LIMIT GOES
================================
A re-zero is an EEPROM write, so "commit" is obvious. A soft limit is software-only, and
``arm_spec.SOFT_LIMITS`` is a checked-in source table — a CLI does not rewrite its own
source. So it lands in :mod:`arm101.hardware.soft_limit_store`, and the tests below prove
the thing that actually matters: **``arm flex`` then honours it.** A committed value that
nothing reads at runtime is not committed, it is filed — and this repo has already shipped
an inert soft limit once.

Every bus here is a :class:`~tests._rolling_servo.RollingServoBus` descendant. Numbers are
DERIVED from ``arm_spec`` / ``rezero`` / ``classify``, never copied.
"""

from __future__ import annotations

import argparse
import json

import pytest

from arm101.cli._commands import arm as arm_cmd
from arm101.cli._errors import EXIT_ENV_ERROR, CliError
from arm101.hardware import arm_spec, rezero
from arm101.hardware.arm_spec import (
    ARC_MARGIN_TICKS,
    FACTORY_ENCODER_OFFSET,
    REZERO_ARCS,
    SEAM_CLEARANCE_TICKS,
    UnreachableArc,
    joint_ids,
)
from arm101.hardware.classify import MIN_EVICTABLE_ARC_TICKS, SeamRemedy, TravelKind
from arm101.hardware.journal import (
    DISPOSITION_COMMITTED,
    DISPOSITION_RESTORED,
    CalibrationJournal,
    default_journal_path,
)
from arm101.hardware.soft_limit_store import default_soft_limit_path, load_soft_limits
from arm101.hardware.ticks import ENCODER_TICKS, seam_tick
from tests._rolling_servo import RollingServoBus
from tests.test_arm_limits_cli import (
    ELBOW,
    ELBOW_MOTOR,
    PAN,
    PAN_MOTOR,
    ROLE,
    _BoundedServo,
    _patch_bus,
    _servo,
)
from tests.test_probe import _GravityServo

IDS = joint_ids(ROLE)

# ---------------------------------------------------------------------------
# The servos. Each is a different physical story about what the sweep will find.
# ---------------------------------------------------------------------------

#: Consecutive LIMP position polls before the operator's hand is modelled as being on the
#: joint. The measurement's own torque-off windows (an offset write, a hold-in-place goal,
#: a frame re-centre) are two or three polls long; a ``--verify`` sweep is six hundred. So
#: a settle of a dozen separates "the tool is calibrating" from "a human is sweeping"
#: without either having to tell the fake which is happening — and it costs the sweep a
#: dozen stationary samples out of six hundred, which is what a real hand costs too.
_HAND_SETTLE_POLLS = 12


class _HandSweptServo(_BoundedServo):
    """A bounded joint that a HUMAN can also move — while it is limp, and only then.

    The instrument the whole seam-eviction proof rests on, modelled honestly: a hand walks
    the joint back and forth between the stops it can actually reach, at a hand's pace,
    while the servo is de-energised and commanding nothing.

    ``hand_overreach`` is the gap between where the ARM stopped and where the JOINT
    actually stops, on the HIGH side:

    * ``0`` — the arm found the joint's real wall. The measured arc is right.
    * ``> 0`` — **the arm was stopped short.** By the table, by a cable, by the pose. The
      joint really does go further, and a hand finds out. Every tick of that gap makes the
      "unreachable" arc wider than it truly is — which is how a seam ends up parked
      somewhere the joint can reach, with every read-back looking perfect.

    A hand can move a joint the servo's own goal register cannot command it to, and that
    is precisely why the hand is the instrument: it does not need a linear tick axis to
    work, and a linear tick axis is exactly what is in doubt.
    """

    def __init__(self, *args, hand_ticks: int = 40, hand_overreach: int = 0, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.hand_ticks = int(hand_ticks)
        self.hand_overreach = int(hand_overreach)
        self.hand_direction = +1
        self._limp_polls: dict[int, int] = {}

    # -- when the hand is on the joint --------------------------------------

    def enable_torque(self, motor: int, on: bool) -> None:
        if on:
            self._limp_polls[motor] = 0  # a powered joint is the tool's, not the human's
        super().enable_torque(motor, on)

    def write_offset(self, motor: int, offset: int) -> None:
        # An offset write is the TOOL calibrating, not a human sweeping. Reset, so the
        # handful of limp polls a frame shift performs can never accumulate into a phantom
        # hand halfway through a measurement.
        self._limp_polls[motor] = 0
        super().write_offset(motor, offset)

    def _hand_is_on_it(self, motor: int) -> bool:
        if self.torque_on(motor):
            return False
        self._limp_polls[motor] = self._limp_polls.get(motor, 0) + 1
        return self._limp_polls[motor] > _HAND_SETTLE_POLLS

    # -- the hand -----------------------------------------------------------

    def _hand_step(self, motor: int) -> None:
        """Walk the joint one hand-pace, reversing at the stops the HAND can reach."""
        low = -self.down
        high = self.up + self.hand_overreach
        travelled = self.net_travel(motor)
        if travelled >= high:
            self.hand_direction = -1
        elif travelled <= low:
            self.hand_direction = +1

        target = max(low, min(high, travelled + self.hand_direction * self.hand_ticks))
        advance = target - travelled
        self._positions[motor] = (self.true_raw(motor) + advance) % ENCODER_TICKS
        self._net_travel[motor] = travelled + advance

    def read_position(self, motor: int) -> int:
        """The sweep's poll. A hand advances the joint BETWEEN polls; the servo does not."""
        if self._hand_is_on_it(motor):
            self._hand_step(motor)
        return super().read_position(motor)

    def read_info(self, motor: int) -> dict:
        if self._hand_is_on_it(motor):
            self._hand_step(motor)
        return super().read_info(motor)


class _AbsentHandServo(_HandSweptServo):
    """Nobody is at the arm. The joint is limp and it simply does not move.

    What an agent's ``--apply --commit`` actually looks like on a real bench with no human
    in the room — and the sweep it produces (six hundred identical samples, no
    discontinuity anywhere) is exactly the sweep a naive "no seam seen => PASS" rule would
    have called a success.
    """

    def _hand_is_on_it(self, motor: int) -> bool:
        return False


# The two geometries. Both DERIVED — the walls are declared as travel and everything the
# tests assert about arcs, seams and offsets is computed from them by the shipped code.

#: A joint with a comfortable unreachable arc. Its seam ends up ~1148 ticks past the high
#: wall, so a hand that reaches the same wall the arm did never goes near it. The honest
#: PASS.
_ROOMY = {"joints": {ELBOW: 4000}, "down": 300, "up": 1500}

#: A joint whose measured arc is NARROW — 496 raw ticks, barely over the
#: ``MIN_EVICTABLE_ARC_TICKS`` floor once ``ARC_MARGIN_TICKS`` is taken off each side. The
#: seam lands only ~248 ticks past the high wall. An arm stopped 300 ticks short of that
#: wall therefore parks the seam inside travel the joint can plainly reach — and every
#: read-back still looks perfect. THE TRAP.
_NARROW = {"joints": {ELBOW: 1000}, "down": 1200, "up": 2400}


#: How far past the arm's high wall the obstructed joint really goes. Derived, not chosen:
#: it must clear the margin the arc is inset by, plus half the inset arc, or the seam is
#: still out of the hand's reach and the trap does not spring.
def _overreach_for(down: int, up: int) -> int:
    """Ticks the hand must reach past the arm's wall to touch the seam the arm's arc implies."""
    unreachable = ENCODER_TICKS - (down + up)
    # The cutoff applies to the FULL measured arc, before the margins come off it — same
    # comparison classify._arc_for makes. (This used to assert it against the INSET arc,
    # a stricter rule than the shipped one; the #43 tightening of the cutoff to
    # 3 * ARC_MARGIN_TICKS is what made the two disagree and surfaced the mismatch.)
    assert unreachable >= MIN_EVICTABLE_ARC_TICKS, "the geometry must still yield a usable arc"
    inset = unreachable - 2 * ARC_MARGIN_TICKS
    return ARC_MARGIN_TICKS + inset // 2 + 1  # one tick past the seam is enough to cross it


_NARROW_OVERREACH = _overreach_for(_NARROW["down"], _NARROW["up"])  # type: ignore[arg-type]


def _stalling_servo() -> _GravityServo:
    """A joint that runs out of TORQUE at both ends and finds a wall at neither.

    It must LIFT, and it eventually cannot: its load climbs with how far out it has been
    driven, so it is already working hard while it is still advancing, and it simply stops.
    At the moment of the stop it reads IDENTICALLY to a wall — same saturated load, same
    joint not advancing — which is why the probe rules on the APPROACH, and why this joint
    comes back UNDETERMINED rather than BOUNDED. There is nothing here to site an arc on.

    ``lift=-1`` so the resistance is on the way DOWN; without it the free direction sweeps
    a full turn and the joint reads CONTINUOUS, which is a different (and true) answer to a
    different question. The slope is derived so the loaded run is comfortably wider than a
    real contact's give — that gap is the whole basis of the TORQUE_LIMITED verdict.
    """
    from arm101.hardware.gentle import CONTACT_LOAD_CEILING
    from arm101.hardware.probe import wall_compliance

    threshold = arm_spec.DEFAULT_CONTACT_THRESHOLDS[PAN]
    slope = 1.0
    assert (CONTACT_LOAD_CEILING - threshold) / slope > wall_compliance()
    return _servo(_GravityServo, joints={PAN: 2048}, load_per_tick=slope, lift=-1)


# ---------------------------------------------------------------------------
# Harness
# ---------------------------------------------------------------------------


def _args(
    joint: "list[str] | None" = None,
    *,
    commit: bool = True,
    apply: bool = True,
    json_mode: bool = True,
    pose: "str | None" = None,
    sweep_duration: "float | None" = None,
    soft_limit_file: "str | None" = None,
    max_travel: "int | None" = None,
) -> argparse.Namespace:
    return argparse.Namespace(
        joint=list(joint) if joint else [],
        role=ROLE,
        port=None,
        apply=apply,
        commit=commit,
        json=json_mode,
        step=None,
        max_travel=max_travel,
        compliance=None,
        pose=pose,
        threshold=None,
        threshold_joint=None,
        threshold_file=None,
        sweep_duration=sweep_duration,
        soft_limit_file=soft_limit_file,
    )


@pytest.fixture(autouse=True)
def _isolated_state(monkeypatch, tmp_path):
    """Every run gets its OWN journal and soft-limit store.

    The suite is parallel (xdist) and both stores default into ``~/.arm101``. A test that
    shared them would race its neighbours — and, far worse, would write a fabricated soft
    limit onto the machine of whoever ran it.
    """
    monkeypatch.setenv("ARM101_CALIBRATION_JOURNAL", str(tmp_path / "journal.jsonl"))
    monkeypatch.setenv("ARM101_SOFT_LIMITS", str(tmp_path / "soft-limits.jsonl"))
    return tmp_path


def _run(monkeypatch, capsys, bus, **kwargs) -> dict:
    _patch_bus(monkeypatch, bus)
    arm_cmd.cmd_arm_limits(_args(**kwargs))
    return json.loads(capsys.readouterr().out)


def _run_expecting_stop(monkeypatch, capsys, bus, **kwargs) -> "tuple[dict, CliError]":
    """Drive a commit that must FAIL. Returns the report it still emitted, and the error.

    The report comes out on **stdout, in full, before** the error is raised. That ordering
    is the point: the numbers are why the operator ran the command, and they are worth
    exactly as much when the answer is "it did not work" as when it is "it worked". Failing
    without showing them would leave a human standing at an arm with an exit code.
    """
    _patch_bus(monkeypatch, bus)
    with pytest.raises(CliError) as excinfo:
        arm_cmd.cmd_arm_limits(_args(**kwargs))
    return json.loads(capsys.readouterr().out), excinfo.value


def _one(payload: dict, joint: str) -> dict:
    (found,) = [entry for entry in payload["joints"] if entry["joint"] == joint]
    return found


# ---------------------------------------------------------------------------
# AC1 — THE SWEEP IS THE ARBITER, NOT THE OFFSET READ-BACK
# ---------------------------------------------------------------------------


def test_an_offset_that_reads_back_PERFECTLY_still_FAILS_if_the_seam_did_not_move(
    monkeypatch, capsys
) -> None:
    """**The acceptance criterion.** A perfect read-back is not a passing grade.

    The arm measured this joint's walls by driving into them — and on the high side it was
    stopped 1149 ticks short of the joint's real stop, by something that is not the joint.
    So the arc it derived is too wide, and the offset it derives from that arc parks the
    seam inside travel the joint can plainly reach.

    Every check short of a sweep passes: the offset is written, it reads back **exactly**
    the value written, and it evicts the seam *according to the arc*. The arithmetic is
    immaculate. The arc is wrong.

    A human hand then walks the joint past where the arm stopped, crosses the seam, and the
    sweep sees the discontinuity. That is a FAILURE, it is reported as one, and the joint
    is put back exactly as it was found.
    """
    bus = _servo(_HandSweptServo, hand_overreach=_NARROW_OVERREACH, **_NARROW)

    payload, error = _run_expecting_stop(monkeypatch, capsys, bus, joint=[ELBOW])
    commit = _one(payload, ELBOW)["commit"]

    # The write LANDED. This is the trap: everything here is green.
    assert commit["applied"] is True
    assert commit["offset_read_back"] == commit["offset_written"]
    assert commit["sweep"]["rezeroed"] is True  # the offset evicts the seam — per the ARC

    # And the sweep says the seam is still in the travel. That is the only opinion that counts.
    assert commit["committed"] is False
    assert commit["sweep"]["verdict"] == rezero.VERDICT_SEAM_NOT_EVICTED
    assert commit["sweep"]["continuous"] is False
    assert commit["sweep"]["conclusive"] is True  # seeing the seam IS proof it is there
    assert commit["sweep"]["largest_jump"] >= rezero.DISCONTINUITY_TICKS

    # The joint is back exactly as it was found, and the failure is loud.
    assert bus.read_offset(ELBOW_MOTOR) == FACTORY_ENCODER_OFFSET
    assert commit["restored"] is True
    assert error.code == EXIT_ENV_ERROR
    assert "SEAM NOT EVICTED" in error.message
    assert "read back correctly" in error.message
    assert "ARC IS WRONG" in error.remediation  # the second cause, named


def test_the_failure_names_BOTH_causes__the_firmware_AND_the_arc(monkeypatch, capsys) -> None:
    """A stop condition that offered only one explanation would send the operator one way.

    "The servo does not reduce modulo 4096" and "your arc is wrong" produce the SAME sweep
    and need OPPOSITE responses — abandon the re-zero entirely, or re-measure with the
    workspace clear. The remediation has to carry both, and say how to tell them apart.
    """
    bus = _servo(_HandSweptServo, hand_overreach=_NARROW_OVERREACH, **_NARROW)
    _, error = _run_expecting_stop(monkeypatch, capsys, bus, joint=[ELBOW])

    assert "modulo 4096" in error.remediation  # cause 1: the firmware
    assert "ARC IS WRONG" in error.remediation  # cause 2: the measurement
    assert "obstacle, not the joint's stop" in error.remediation
    assert "compare the span" in error.remediation  # how to tell which one you have


def test_a_FAILED_commit_leaves_NOTHING_in_flight(monkeypatch, capsys, tmp_path) -> None:
    """The transaction closes RESTORED, not committed — and the journal is clean.

    Anything less would leave the next run to discover a dirty entry for a joint that has
    already been put back, and spend an EEPROM write undoing nothing.
    """
    dispositions: list[str] = []
    real_end = CalibrationJournal.end

    def _spy(self, *, motor: int, disposition: str) -> None:
        dispositions.append(disposition)
        real_end(self, motor=motor, disposition=disposition)

    monkeypatch.setattr(CalibrationJournal, "end", _spy)
    bus = _servo(_HandSweptServo, hand_overreach=_NARROW_OVERREACH, **_NARROW)

    _run_expecting_stop(monkeypatch, capsys, bus, joint=[ELBOW])

    assert DISPOSITION_COMMITTED not in dispositions
    assert set(dispositions) == {DISPOSITION_RESTORED}
    assert CalibrationJournal(default_journal_path()).dirty_entries() == []


def test_a_FAILED_commit_STOPS_the_run__it_does_not_carry_on_to_the_next_joint(
    monkeypatch, capsys
) -> None:
    """A re-zero that cannot be proven is a claim about the FIRMWARE and about the METHOD.

    Both are the ground every remaining joint's commit would stand on. Carrying on would be
    re-zeroing five more joints on a premise that has just failed in front of us — and one
    of them (``shoulder_lift``) carries the whole arm.
    """
    bus = _servo(
        _HandSweptServo,
        joints={PAN: 1000, ELBOW: 1000},
        down=1200,
        up=2400,
        hand_overreach=_NARROW_OVERREACH,
    )

    payload, _ = _run_expecting_stop(monkeypatch, capsys, bus, joint=[PAN, ELBOW])

    # JOINTS order puts shoulder_pan before elbow_flex, and the verb measures in that order.
    # PAN's sweep found the seam, so the run STOPS THERE.
    measured = [entry["joint"] for entry in payload["joints"]]
    assert measured == [PAN]
    assert payload["commits"]["failed"] == 1

    # elbow_flex was never measured, never energised, never written. Its calibration is
    # untouched — and that is the point: a re-zero that could not be proven has just told
    # us something about the FIRMWARE and about the METHOD, and both are the ground the
    # next joint's commit would stand on.
    assert not [w for w in bus.offset_writes if w["motor"] == ELBOW_MOTOR]
    assert bus.read_offset(ELBOW_MOTOR) == FACTORY_ENCODER_OFFSET


# ---------------------------------------------------------------------------
# AC2 — the >=80% coverage rule. A short clean sweep is INCONCLUSIVE, never a pass.
# ---------------------------------------------------------------------------


def test_an_UNATTENDED_commit_cannot_pass_itself(monkeypatch, capsys) -> None:
    """Nobody at the arm. Six hundred samples of a stationary joint. No discontinuity.

    This is the sweep a naive "no seam seen => PASS" rule would call a success — and it is
    the exact shape of the three empty sweeps (0, 0 and 376 ticks against an expected
    ~2202) that nearly closed the ``elbow_flex`` question with a lie.

    An agent running ``--apply --commit`` with no human in the room gets a clean, honest
    REFUSAL. That is not a limitation of the verb; it is the verb working.
    """
    bus = _servo(_AbsentHandServo, **_ROOMY)

    payload, error = _run_expecting_stop(monkeypatch, capsys, bus, joint=[ELBOW])
    commit = _one(payload, ELBOW)["commit"]
    sweep = commit["sweep"]

    assert sweep["continuous"] is True  # of course it is. Nothing moved.
    assert sweep["conclusive"] is False  # ...and that is why it proves nothing
    assert sweep["verdict"] == rezero.VERDICT_INCONCLUSIVE
    assert sweep["span"] < sweep["expected_travel"] * rezero.MIN_COVERAGE

    assert commit["committed"] is False
    assert bus.read_offset(ELBOW_MOTOR) == FACTORY_ENCODER_OFFSET  # put back
    assert error.code == EXIT_ENV_ERROR
    assert "UNPROVEN" in error.message
    assert "REQUIRES A HUMAN" in error.remediation


def test_the_coverage_rule_is_the_SHIPPED_one__not_a_second_copy(monkeypatch, capsys) -> None:
    """``rezero.MIN_COVERAGE`` is reused whole. Weakening it there weakens it here.

    Asserted by driving a sweep whose coverage sits just BELOW the shipped bar and watching
    it be refused — with the bar itself read from ``rezero``, so a change to that constant
    moves this test rather than breaking it.
    """
    bus = _servo(_HandSweptServo, **_ROOMY)
    arc = _measured_arc(bus)

    # A sweep too short to cover the travel: the hand gets a fraction of the samples it
    # needs. Derived from the shipped constants, never typed.
    short = (arc.travel_ticks * rezero.MIN_COVERAGE * 0.5) / bus.hand_ticks
    duration = (short + _HAND_SETTLE_POLLS) * rezero.DEFAULT_SWEEP_INTERVAL

    payload, error = _run_expecting_stop(
        monkeypatch, capsys, bus, joint=[ELBOW], sweep_duration=duration
    )
    sweep = _one(payload, ELBOW)["commit"]["sweep"]

    assert sweep["continuous"] is True
    assert sweep["conclusive"] is False
    assert sweep["verdict"] == rezero.VERDICT_INCONCLUSIVE
    assert f"{rezero.MIN_COVERAGE:.0%}" in error.message
    assert bus.read_offset(ELBOW_MOTOR) == FACTORY_ENCODER_OFFSET


def _measured_arc(bus: _HandSweptServo) -> UnreachableArc:
    """The arc the classifier WILL derive from this servo's declared walls. Derived, not typed."""
    span = bus.span()
    unreachable = ENCODER_TICKS - span
    inset = unreachable - 2 * ARC_MARGIN_TICKS
    assert inset >= MIN_EVICTABLE_ARC_TICKS
    return UnreachableArc(low=0, high=inset)  # only .travel_ticks / .width are read from this


# ---------------------------------------------------------------------------
# AC3 — BOUNDED => a re-zero. And it is PROVEN before it is kept.
# ---------------------------------------------------------------------------


def test_a_BOUNDED_joint_commits_a_REZERO__proven_by_the_sweep(monkeypatch, capsys) -> None:
    """The happy path, and the only one that ends in an EEPROM write being kept.

    Walls at both ends, an arc wide enough to take the seam with margin to spare, a human
    who actually sweeps the joint, and a sweep that comes back continuous across its whole
    travel. THEN, and only then, the calibration is the truth from here.
    """
    bus = _servo(_HandSweptServo, **_ROOMY)

    payload = _run(monkeypatch, capsys, bus, joint=[ELBOW])
    entry = _one(payload, ELBOW)
    commit = entry["commit"]

    assert entry["kind"] == TravelKind.BOUNDED.value
    assert entry["remedy"] == SeamRemedy.REZERO.value

    assert commit["committed"] is True
    assert commit["action"] == "rezero"
    assert commit["sweep"]["verdict"] == rezero.VERDICT_SEAM_EVICTED
    assert commit["sweep"]["seam_evicted"] is True
    assert commit["sweep"]["conclusive"] is True

    # The offset is DERIVED from the arc the arm measured — not from the shipped table, and
    # not typed here. The seam now sits inside the arc.
    written = commit["offset_written"]
    assert bus.read_offset(ELBOW_MOTOR) == written  # KEPT — not restored
    assert written != FACTORY_ENCODER_OFFSET
    assert commit["seam_now_at_raw_tick"] == seam_tick(written)
    low, high = entry["unreachable_arc"]["low"], entry["unreachable_arc"]["high"]
    assert low < commit["seam_now_at_raw_tick"] < high


def test_the_committed_offset_comes_from_the_MEASUREMENT__not_the_shipped_table(
    monkeypatch, capsys
) -> None:
    """The table is a default. The arm is the truth.

    ``REZERO_ARCS`` has exactly one entry, hand-measured in a long human session — and it
    is the arc for a joint whose walls are where THAT arm's were. A commit that wrote the
    table's offset would be re-zeroing this arm to somebody else's geometry, and the whole
    point of ``arm limits`` is that it does not have to.
    """
    bus = _servo(_HandSweptServo, **_ROOMY)

    payload = _run(monkeypatch, capsys, bus, joint=[ELBOW])
    entry = _one(payload, ELBOW)
    commit = entry["commit"]

    from_the_table = REZERO_ARCS[ELBOW].offset
    measured = UnreachableArc(
        low=entry["unreachable_arc"]["low"], high=entry["unreachable_arc"]["high"]
    )
    assert commit["offset_written"] == measured.offset
    assert commit["offset_written"] != from_the_table, (
        "this servo's walls are not the shipped arm's, so its offset must not be either — "
        "if these ever coincide, the geometry in this test has stopped testing anything"
    )


def test_the_committed_rezero_leaves_NO_STALE_GOAL(monkeypatch, capsys) -> None:
    """Issue #47, through the verb. The commit moved the frame; the standing goal followed.

    The offset delta here is over a thousand ticks. A joint left holding a goal written in
    the old frame drives that far the instant the next mover energises it — and the next
    mover enables torque BEFORE it writes its first goal.
    """
    bus = _servo(_HandSweptServo, **_ROOMY)
    _run(monkeypatch, capsys, bus, joint=[ELBOW])

    # Where the joint ACTUALLY is, in the frame the commit left it speaking in. Read from
    # the simulation, not through read_position — this fake's hand advances the shaft on
    # every poll, and an assertion that moved the joint it was measuring would be measuring
    # itself. (The servo's own arithmetic, so no frame is assumed: reported = raw - Ofs.)
    settled = bus.true_raw(ELBOW_MOTOR)
    standing = (settled - bus.read_offset(ELBOW_MOTOR)) % ENCODER_TICKS

    assert bus.reported_goal(ELBOW_MOTOR) == standing, (
        "the standing goal does not name where the joint is — the commit moved the frame "
        "(and the human moved the joint), and the goal did not follow either"
    )

    bus.enable_torque(ELBOW_MOTOR, True)
    for _ in range(ENCODER_TICKS // bus.ticks_per_poll + 8):
        bus.read_info(ELBOW_MOTOR)
    assert bus.true_raw(ELBOW_MOTOR) == settled, "the joint LURCHED after a committed re-zero"


def test_a_committed_rezero_journals_the_original_BEFORE_the_wire_write(
    monkeypatch, capsys
) -> None:
    """The commit is a TRANSACTION, and an unverified re-zero must not survive a crash.

    The offset is durable on disk before it reaches the servo, and only a PASSING sweep
    closes the entry as ``committed``. Kill the process between the two and the next run's
    ``require_clean`` puts the original back — which is right: "it died before it could
    check" is not evidence that it would have passed.
    """
    dispositions: list[str] = []
    real_end = CalibrationJournal.end

    def _spy(self, *, motor: int, disposition: str) -> None:
        dispositions.append(disposition)
        real_end(self, motor=motor, disposition=disposition)

    monkeypatch.setattr(CalibrationJournal, "end", _spy)
    bus = _servo(_HandSweptServo, **_ROOMY)

    _run(monkeypatch, capsys, bus, joint=[ELBOW])

    # RESTORED closes the measurement's rolling frame; COMMITTED closes the re-zero.
    assert dispositions == [DISPOSITION_RESTORED, DISPOSITION_COMMITTED]
    assert CalibrationJournal(default_journal_path()).dirty_entries() == []


def test_the_operator_is_told_the_write_is_not_yet_PERSISTENT(monkeypatch, capsys) -> None:
    """PR #21: an EEPROM write can read back correctly and revert on the next power-up."""
    bus = _servo(_HandSweptServo, **_ROOMY)
    payload = _run(monkeypatch, capsys, bus, joint=[ELBOW])

    assert _one(payload, ELBOW)["commit"]["persistence_proven"] is False


# ---------------------------------------------------------------------------
# AC3 — CONTINUOUS => a SOFT LIMIT whose dead arc contains the seam. Never addr 9/11.
# ---------------------------------------------------------------------------


def test_a_CONTINUOUS_joint_commits_a_SOFT_LIMIT_whose_dead_arc_contains_the_seam(
    monkeypatch, capsys
) -> None:
    """A joint that turns all the way round has NO angle to put the seam at.

    Re-zero is impossible for it *in principle* — an offset RELOCATES a seam, it never
    EVICTS one — so the only instrument left is a software dead arc the joint is never
    commanded into. The claim a soft limit makes is geometric, and it is checked as such:
    the dead arc must contain both seams, with clearance.
    """
    bus = _servo(RollingServoBus, joints={PAN: 2048})

    payload = _run(monkeypatch, capsys, bus, joint=[PAN])
    entry = _one(payload, PAN)
    commit = entry["commit"]

    assert entry["kind"] == TravelKind.CONTINUOUS.value
    assert entry["remedy"] == SeamRemedy.SOFT_LIMIT.value
    assert commit["committed"] is True
    assert commit["action"] == "soft_limit"

    limit = arm_spec.SoftLimit(
        min_tick=commit["soft_limit"]["min_tick"], max_tick=commit["soft_limit"]["max_tick"]
    )
    offset = commit["soft_limit"]["derived_for_offset"]
    assert offset == FACTORY_ENCODER_OFFSET  # read off the servo, not assumed

    # The two seams, and both are fenced off — with clearance.
    assert arm_spec.dead_arc_contains_seam(limit.min_tick, limit.max_tick)
    assert arm_spec.dead_arc_contains_reported_seam(limit, offset)
    assert limit.clearance_from(seam_tick(offset)) >= SEAM_CLEARANCE_TICKS
    assert limit.clearance_from(arm_spec.RAW_SEAM_TICK) >= SEAM_CLEARANCE_TICKS


def test_a_soft_limit_commit_writes_NO_SERVO_REGISTER_AT_ALL(monkeypatch, capsys) -> None:
    """It is SOFTWARE. In particular it is not addrs 9/11.

    The obvious-looking alternative — write ``Min/Max_Position_Limit`` and let the firmware
    clamp every goal for free — is forbidden, and the forbidding is not stylistic. Those
    registers are EEPROM: a fence written there outlives the pose that produced it and
    travels with the servo onto the next arm. ``tests/test_eeprom_limit_write_guard.py``
    pins the package's whole wire surface shut against them; this asserts the same thing
    from the other end, on the one verb that would most plausibly reach for them.
    """
    bus = _servo(RollingServoBus, joints={PAN: 2048})

    payload = _run(monkeypatch, capsys, bus, joint=[PAN])

    assert _one(payload, PAN)["commit"]["registers_written"] == []
    touched = {write["addr"] for write in bus.register_writes}
    assert 9 not in touched and 11 not in touched
    # ...and no EEPROM offset was kept either. A soft-limited joint's calibration is its own.
    assert bus.read_offset(PAN_MOTOR) == FACTORY_ENCODER_OFFSET


def test_a_soft_limit_needs_no_sweep__because_a_sweep_could_not_prove_it(
    monkeypatch, capsys
) -> None:
    """A sweep answers "did the seam MOVE?". A soft limit does not move the seam; it FENCES it.

    Demanding one here would be ceremony — and worse, it would make the soft-limit path
    unusable in exactly the situation it exists for. So the commit succeeds with nobody's
    hand on the joint, and the payload carries no sweep at all.
    """
    bus = _servo(RollingServoBus, joints={PAN: 2048})
    payload = _run(monkeypatch, capsys, bus, joint=[PAN])
    commit = _one(payload, PAN)["commit"]

    assert commit["committed"] is True
    assert "sweep" not in commit


# ---------------------------------------------------------------------------
# WHERE A MEASURED SOFT LIMIT LANDS — and what reads it at runtime
# ---------------------------------------------------------------------------


def test_the_measured_soft_limit_lands_in_the_STORE(monkeypatch, capsys, tmp_path) -> None:
    """``arm_spec.SOFT_LIMITS`` is checked-in source. A CLI does not rewrite its own source.

    So the measurement goes to the store — and the store is a real file, at a real default
    path, with the provenance that makes the number believable: the offset it was derived
    against, what the arm measured, and the pose it measured in.
    """
    bus = _servo(RollingServoBus, joints={PAN: 2048})

    payload = _run(monkeypatch, capsys, bus, joint=[PAN], pose="elbow folded, gripper clear")
    commit = _one(payload, PAN)["commit"]

    assert commit["stored_at"] == str(default_soft_limit_path())
    stored = load_soft_limits()
    assert PAN in stored
    assert stored[PAN].min_tick == commit["soft_limit"]["min_tick"]
    assert stored[PAN].max_tick == commit["soft_limit"]["max_tick"]

    record = json.loads(default_soft_limit_path().read_text().strip().splitlines()[-1])
    assert record["frame"] == "raw"  # NOT the ticks a servo reports
    assert record["offset"] == FACTORY_ENCODER_OFFSET
    assert record["pose"] == "elbow folded, gripper clear"
    assert record["kind"] == TravelKind.CONTINUOUS.value


def test_a_MEASURED_soft_limit_actually_BINDS__arm_flex_honours_it(
    monkeypatch, capsys, tmp_path
) -> None:
    """**The test that says the commit is real.**

    A value nothing reads at runtime is not committed, it is filed. This repo has already
    shipped that bug once: the ``wrist_roll`` soft limit was inert data for a whole release,
    because every mover took its bounds from the servo's EEPROM ``min_angle``/``max_angle``
    registers — the untouched factory ``0-4095`` — until a follow-up routed them all through
    ``arm_spec.resolve_bounds``.

    So: commit a soft limit on a joint that has none, then ask ``arm flex`` to drive that
    joint straight into the dead arc, and watch it be clamped out of it.
    """
    bus = _servo(RollingServoBus, joints={PAN: 2048})
    _run(monkeypatch, capsys, bus, joint=[PAN])

    # The joint has no soft limit in the SHIPPED table. It has one now.
    assert arm_spec.soft_limit(PAN) is None
    assert PAN in arm_spec.resolve_soft_limits(from_file=load_soft_limits())

    # A goal is a REPORTED tick and the limit is stored in RAW ones, so the target has to be
    # named in the frame the servo is actually commanded in — the crossing that this whole
    # codebase keeps getting wrong, made explicit here rather than assumed.
    resolved = arm_spec.resolve_soft_limits(from_file=load_soft_limits())
    permitted_low, _ = arm_spec.permitted_reported_range(
        PAN, FACTORY_ENCODER_OFFSET, limits=resolved
    )
    into_the_dead_arc = permitted_low - 1  # one tick over the fence
    assert into_the_dead_arc >= 0
    # ...and the EEPROM alone would happily allow it: the factory registers are 0-4095, so
    # ONLY the soft limit can stop this move. That is what makes this test mean something.
    assert 0 <= into_the_dead_arc <= 4095

    flex = _servo(RollingServoBus, joints={PAN: 2048})
    _patch_bus(monkeypatch, flex)
    arm_cmd.cmd_arm_flex(
        argparse.Namespace(
            role=ROLE,
            joint=PAN,
            to=into_the_dead_arc,
            demo=False,
            gentle=False,
            threshold=None,
            port=None,
            apply=True,
            json=True,
            soft_limit_file=None,
        )
    )
    move = json.loads(capsys.readouterr().out)["move"]

    assert move["requested_target"] == into_the_dead_arc  # we really did ask for it
    assert move["clamped_target"] == permitted_low, (
        "arm flex drove into the dead arc: the measured soft limit is INERT, which is the "
        "one thing a committed soft limit must never be"
    )
    assert move["clamped_target"] != into_the_dead_arc


def test_the_commit_also_prints_the_arm_spec_ENTRY_a_human_would_check_in(
    monkeypatch, capsys
) -> None:
    """The store makes the limit true for THIS arm. The table is how it stops being local.

    A measurement that lives only in one operator's home directory has been made once and
    will be made again, by hand, by the next person. So the commit prints the exact source
    line — and it is rendered from the measurement, so it cannot say a different number
    than the store holds.
    """
    bus = _servo(RollingServoBus, joints={PAN: 2048})
    payload = _run(monkeypatch, capsys, bus, joint=[PAN])
    commit = _one(payload, PAN)["commit"]

    entry = commit["arm_spec_entry"]
    assert f'"{PAN}": SoftLimit(' in entry
    assert f"min_tick={commit['soft_limit']['min_tick']}" in entry
    assert f"max_tick={commit['soft_limit']['max_tick']}" in entry


def test_the_TEXT_report_shows_the_table_entry_and_where_the_store_is(monkeypatch, capsys) -> None:
    bus = _servo(RollingServoBus, joints={PAN: 2048})
    _patch_bus(monkeypatch, bus)
    arm_cmd.cmd_arm_limits(_args(joint=[PAN], json_mode=False))
    out = capsys.readouterr().out

    assert "SOFT-LIMITED (software only; no servo register written)" in out
    assert str(default_soft_limit_path()) in out
    assert "SoftLimit(" in out  # the line to paste into arm_spec


def test_the_store_is_APPEND_ONLY_and_the_LAST_record_for_a_joint_wins(
    monkeypatch, capsys, tmp_path
) -> None:
    """Re-measuring a joint corrects it. It does not destroy what the last run believed."""
    for _ in range(2):
        bus = _servo(RollingServoBus, joints={PAN: 2048})
        _run(monkeypatch, capsys, bus, joint=[PAN])

    lines = default_soft_limit_path().read_text().strip().splitlines()
    assert len(lines) == 2  # both kept
    assert len(load_soft_limits()) == 1  # one in force


# ---------------------------------------------------------------------------
# AC3 — UNDETERMINED commits NOTHING. And so does a joint with nothing wrong.
# ---------------------------------------------------------------------------


def test_an_UNDETERMINED_joint_commits_NOTHING__and_is_told_to_measure_again(
    monkeypatch, capsys
) -> None:
    """No wall vouches for either end. An arc sited on this would be an INVENTION.

    A gravity-loaded joint stalls at a saturated load with nothing in front of it: it reads
    exactly like a wall at the moment it stops, which is why the probe rules on the
    approach. Neither instrument can be chosen on this evidence, and choosing one anyway
    would burn an EEPROM write — or fence off real travel — on a measurement that supports
    neither.
    """
    bus = _stalling_servo()

    payload = _run(monkeypatch, capsys, bus, joint=[PAN], max_travel=600)
    entry = _one(payload, PAN)
    commit = entry["commit"]

    assert entry["kind"] == TravelKind.UNDETERMINED.value
    assert entry["remedy"] == SeamRemedy.UNKNOWN.value
    assert commit["committed"] is False
    assert commit["action"] == "refused_undetermined"
    assert "REFUSED" in commit["reason"]
    assert "MEASURE AGAIN" in commit["reason"]

    # NOTHING was kept: not an offset, not a soft limit.
    assert bus.read_offset(PAN_MOTOR) == FACTORY_ENCODER_OFFSET
    assert load_soft_limits() == {}
    assert payload["commits"]["refused"] == [PAN]
    assert payload["commits"]["committed"] == 0
    assert payload["commits"]["failed"] == 0  # a refusal is NOT a failure


def test_a_joint_whose_travel_MISSES_the_seam_commits_nothing_either(monkeypatch, capsys) -> None:
    """ "Nothing to do" is a real answer, and a different one from "nothing can be done".

    This joint's travel does not cross the seam at all: its reported position is already
    monotonic with joint angle. There is nothing to evict and nothing to fence off, and
    committing something here would be inventing a problem to solve.
    """
    bus = _servo(_HandSweptServo, joints={ELBOW: 2048}, down=300, up=1500)

    payload = _run(monkeypatch, capsys, bus, joint=[ELBOW])
    entry = _one(payload, ELBOW)

    assert entry["kind"] == TravelKind.BOUNDED.value
    assert entry["seam_in_travel"] is False
    assert entry["remedy"] == SeamRemedy.NONE_NEEDED.value
    assert entry["commit"]["committed"] is False
    assert entry["commit"]["action"] == "none_needed"
    assert bus.read_offset(ELBOW_MOTOR) == FACTORY_ENCODER_OFFSET
    assert load_soft_limits() == {}


def test_a_refusal_and_a_FAILURE_do_not_look_alike_in_the_payload(monkeypatch, capsys) -> None:
    """Both commit nothing. Only one of them means something went WRONG.

    A run that wrote an offset and could not prove the seam moved must never be able to
    read like a run that quietly had nothing to do — and the empty ``committed`` list they
    share is exactly how it would.
    """
    quiet = _stalling_servo()
    payload = _run(monkeypatch, capsys, quiet, joint=[PAN], max_travel=600)
    assert payload["commits"]["committed"] == 0
    assert payload["commits"]["failed"] == 0  # nothing went wrong. Nothing was provable.

    loud = _servo(_HandSweptServo, hand_overreach=_NARROW_OVERREACH, **_NARROW)
    payload, _ = _run_expecting_stop(monkeypatch, capsys, loud, joint=[ELBOW])
    assert payload["commits"]["committed"] == 0
    assert payload["commits"]["failed"] == 1  # something went very wrong indeed


# ---------------------------------------------------------------------------
# Consent — a commit is a second, explicit act
# ---------------------------------------------------------------------------


def test_a_dry_run_commit_opens_NO_BUS_and_states_the_DECISION_PROCEDURE(
    monkeypatch, capsys
) -> None:
    """It cannot name the offsets — the measurement has not been taken.

    A plan that guessed would be guessing from the shipped table, which is the very table
    this verb exists to correct. What it CAN state, and does, is how each answer will be
    decided, that a re-zero is persistent, and that a human hand is required.
    """

    def _explode(_port):  # pragma: no cover - the point is that it is never reached
        raise AssertionError("a dry run must not open a bus")

    monkeypatch.setattr(arm_cmd, "_open_bus", _explode)
    monkeypatch.setattr(arm_cmd, "_candidate_ports", lambda: ["/dev/null"])

    arm_cmd.cmd_arm_limits(_args(joint=[ELBOW], apply=False))
    plan = json.loads(capsys.readouterr().out)["plan"]

    assert plan["commit"] is True
    assert "PERSISTENT" in plan["note"]
    assert "hand-move" in plan["note"]
    assert "RE-ZERO" in plan["decision"]["bounded"]
    assert "SOFT LIMIT" in plan["decision"]["continuous"]
    assert "NOTHING" in plan["decision"]["undetermined"]
    assert "9/11" in plan["decision"]["never"]  # the registers this verb never writes


def test_a_TTY_operator_is_warned_that_the_change_is_PERMANENT(monkeypatch, capsys) -> None:
    """A measure run puts everything back. A commit run changes the arm forever.

    The operator has to be told which one they are about to authorise, in those words,
    BEFORE the prompt — not discover it in the report afterwards.
    """
    import sys

    monkeypatch.setattr(sys.stdin, "isatty", lambda: True, raising=False)
    monkeypatch.setattr(arm_cmd, "_prompt", lambda _p: "no")

    arm_cmd.cmd_arm_limits(_args(joint=[ELBOW], apply=False))
    captured = capsys.readouterr()

    assert "PERSISTENT EEPROM WRITE" in captured.err
    assert "HAND-MOVE" in captured.err
    assert "UNDONE" in captured.err  # what happens if you do not
    assert "addrs 9/11" in captured.err
    assert "aborted" in captured.out.lower()


def test_declining_at_the_prompt_commits_and_measures_NOTHING(monkeypatch, capsys) -> None:
    import sys

    def _explode(_port):  # pragma: no cover
        raise AssertionError("declining must not open a bus")

    monkeypatch.setattr(sys.stdin, "isatty", lambda: True, raising=False)
    monkeypatch.setattr(arm_cmd, "_prompt", lambda _p: "no")
    monkeypatch.setattr(arm_cmd, "_open_bus", _explode)

    arm_cmd.cmd_arm_limits(_args(joint=[ELBOW], apply=False))
    assert load_soft_limits() == {}


def test_a_sweep_duration_too_short_to_sample_is_a_USER_error__caught_before_consent(
    monkeypatch,
) -> None:
    """Caught up front — not after the operator has said yes and the arm has creeped for ages."""

    def _explode(_port):  # pragma: no cover
        raise AssertionError("a bad --sweep-duration must not reach the bus")

    monkeypatch.setattr(arm_cmd, "_open_bus", _explode)

    with pytest.raises(CliError) as excinfo:
        arm_cmd.cmd_arm_limits(_args(joint=[ELBOW], sweep_duration=rezero.DEFAULT_SWEEP_INTERVAL))
    assert "too few to detect a discontinuity" in excinfo.value.message


def test_a_soft_limit_that_would_CONTRADICT_the_shipped_arc_is_refused__and_NOTHING_written(
    monkeypatch, capsys
) -> None:
    """A joint cannot have both a re-zero arc and a soft limit. Only a human can settle it.

    ``elbow_flex`` has a MEASURED unreachable arc in ``REZERO_ARCS``, hand-taken on a
    physical arm. Suppose a fresh measurement of it comes back "your arc is too narrow to
    hold the seam clear of both walls — use a soft limit". That is a genuine contradiction
    between the shipped table and the arm in front of us, and it has two opposite
    resolutions: either the arm is being stopped short of its real walls (so the MEASUREMENT
    is wrong), or the table's arc is stale (so the TABLE is wrong).

    The verb refuses, writes NOTHING, and says which two things to compare. Anything else
    would poison the store: a record that makes the next run's soft-limit resolution raise
    would break every motion verb on the machine — *after* this run had reported success.
    """
    # A joint whose travel is enormous: its unreachable arc is one tick too narrow to take
    # the seam with a margin at each wall, so the classifier routes it to a soft limit
    # instead of a re-zero. Derived from the shipped cutoff, not typed — a re-tuned margin
    # moves this fixture with it rather than breaking it.
    unreachable = MIN_EVICTABLE_ARC_TICKS - 1
    span = ENCODER_TICKS - unreachable
    bus = _servo(_HandSweptServo, joints={ELBOW: 500}, down=600, up=span - 600)

    _patch_bus(monkeypatch, bus)
    with pytest.raises(CliError) as excinfo:
        arm_cmd.cmd_arm_limits(_args(joint=[ELBOW]))

    assert "would contradict the shipped tables" in excinfo.value.message
    assert "nothing has been written" in excinfo.value.message
    assert "mutually exclusive" in excinfo.value.remediation
    assert "compare the span" in excinfo.value.remediation

    # NOTHING was written: not the store, not the servo.
    assert load_soft_limits() == {}
    assert bus.read_offset(ELBOW_MOTOR) == FACTORY_ENCODER_OFFSET
