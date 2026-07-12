"""t9/t11 — ``arm limits``: the verb that turns the probe/frame/classifier into a run.

What this verb is, and the four things it must never do
=======================================================
``arm limits [<joint>...]`` measures each joint's true travel: it rolls the encoder
seam out of the joint's way (:class:`~arm101.hardware.rolling_frame.RollingFrame`),
creeps outward under contact detection (:func:`~arm101.hardware.probe.probe_end`),
and classifies what it found (:func:`~arm101.hardware.classify.classify_observations`).

Four properties are asserted here over and over, because each of them is a way the
verb could quietly become something nobody asked it to be:

1. **MEASURE-ONLY.** It restores the offset it borrowed and leaves the servo exactly
   as it found it. A verb that silently re-calibrated five joints because you asked
   it to *look* at them is not one anybody should run — so the restore is asserted
   at the servo (``read_offset``), at the journal (no dirty entry, disposition
   ``restored``), and at the parser (**there is no ``--commit`` flag**).
2. **The whole probe runs inside a torque guard.** Asserted for THIS verb, against a
   bus that dies mid-probe: every motor it ever energised — including one whose frame
   closed long ago — must end up de-energised.
3. **Per-joint bounds and verdicts ONLY.** No cells, no reachability score, no map.
   Asserted by making the reachability machinery explode if it is so much as touched,
   and by pinning the payload's key set.
4. **The evidence reaches the operator.** ``loaded_run_ticks`` is the measurement that
   decides WALL vs TORQUE_LIMITED and its cutoff is currently derived from a
   *simulation*. The first hardware run exists to retune it from real data, so that
   number — with ``free_run``, ``peak_load``, the verdict and the reason — must be in
   ``--json``, per joint per end. If the operator has to re-instrument to get it, the
   run is wasted.

Every bus here is a :class:`~tests._rolling_servo.RollingServoBus`, never
``ServoModelBus``: the latter clamps in the raw frame and literally cannot model a
shaft crossing the seam, which is the motion this whole feature exists to enable.

Numbers are DERIVED — from ``arm_spec``, ``gentle``, ``ticks``, ``probe`` — never
copied. A servo's walls are declared as travel and the expected span is computed from
them, so a re-tuned constant moves the expectation with it.
"""

from __future__ import annotations

import argparse
import json
import sys

import pytest

from arm101.cli._commands import arm as arm_cmd
from arm101.cli._errors import EXIT_USER_ERROR, CliError
from arm101.hardware import gentle
from arm101.hardware.arm_spec import (
    DEFAULT_CONTACT_THRESHOLDS,
    FACTORY_ENCODER_OFFSET,
    JOINTS,
    joint_ids,
)
from arm101.hardware.classify import SeamRemedy, TravelKind
from arm101.hardware.journal import (
    DISPOSITION_COMMITTED,
    DISPOSITION_RESTORED,
    CalibrationJournal,
    default_journal_path,
)
from arm101.hardware.limits import ENCODER_TICKS, LimitVerdict
from arm101.hardware.probe import DEFAULT_CREEP_TICKS, wall_compliance
from tests._rolling_servo import RollingServoBus
from tests.test_probe import _GravityServo, _Recording

ROLE = "follower"
IDS = joint_ids(ROLE)

#: The two joints most of these tests drive. Named, not numbered — the motor id comes
#: from ``arm_spec``, like every other number here.
PAN = "shoulder_pan"
ELBOW = "elbow_flex"
PAN_MOTOR = IDS[PAN]
ELBOW_MOTOR = IDS[ELBOW]

#: Where ``present_load`` saturates: a wall and an exhausted arm BOTH read exactly
#: this at the stop, which is why the verdict is taken from the APPROACH.
CEILING = gentle.CONTACT_LOAD_CEILING


# ---------------------------------------------------------------------------
# The servos. Each is a different physical story about why a joint stopped.
# ---------------------------------------------------------------------------


class _BoundedServo(_Recording):
    """A joint with a REAL mechanical wall at each end of its travel.

    The walls are declared as **ticks of travel from where the shaft started**, which
    is what a physical joint's stops are, relative to a probe that begins at rest. So
    the joint's true span is ``down + up`` and every expectation in this file is
    computed from those two numbers rather than typed next to them.

    Seed the shaft near the raw seam and its travel WRAPS it — which is the whole
    reason the rolling frame exists, and the case a ``[min, max]`` pair cannot hold.
    """

    def __init__(self, *args, down: int, up: int, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.down = int(down)
        self.up = int(up)

    def span(self) -> int:
        """The travel this joint actually has, in ticks. The ground truth."""
        return self.down + self.up

    def _remaining(self, motor: int, direction: int) -> int:
        """Ticks of free travel left before the shaft meets the stop in *direction*."""
        travelled = self.net_travel(motor)
        return self.up - travelled if direction > 0 else travelled + self.down

    def _advance(self, motor: int) -> "tuple[int, int]":
        direction = self._commanded(motor)
        if direction and self._remaining(motor, direction) <= 0:
            # Pressed into the stop. The shaft does not move and the load saturates —
            # exactly what an exhausted joint reads like, which is why the probe rules
            # on the approach and not on this.
            return self.true_raw(motor), CEILING

        raw, load = super()._advance(motor)
        if direction:
            overshoot = -self._remaining(motor, direction)
            if overshoot > 0:  # the step would have driven THROUGH the stop
                raw = (raw - overshoot * direction) % ENCODER_TICKS
                self._positions[motor] = raw
                self._net_travel[motor] = direction * (self.up if direction > 0 else -self.down)
        return raw, load


class _DyingBus(_BoundedServo):
    """A bounded joint on a bus that DIES — the incident this feature's guard exists for.

    An ``arm explore`` run died on an unhandled ``serial.SerialException`` (a second
    process had opened the port) and left all six motors energised, holding the arm up
    against gravity at ~50 C. So the failure modelled here is deliberately **not** a
    ``CliError``: it is a raw exception out of the SDK, which is what actually happened.
    """

    def __init__(self, *args, dies_on_motor: int, after_reads: int, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.dies_on_motor = int(dies_on_motor)
        self.after_reads = int(after_reads)
        self._reads = 0
        #: Every motor ``clear_overload`` was called on, in order — the ONE thing a
        #: torque release does that nothing else in the run does for a motor whose
        #: frame closed long ago.
        self.cleared: list[int] = []

    def clear_overload(self, motor: int) -> None:
        self.cleared.append(motor)
        super().clear_overload(motor)

    def read_info(self, motor: int) -> dict:
        if motor == self.dies_on_motor:
            self._reads += 1
            if self._reads > self.after_reads:
                raise RuntimeError("could not open /dev/ttyACM0: device reports readiness")
        return super().read_info(motor)


def _servo(cls, *, joints: "dict[str, int]", **kwargs):
    """Build *cls* with one motor per named joint, each seeded at its own raw tick."""
    positions = {IDS[joint]: raw for joint, raw in joints.items()}
    bus = cls(
        positions=positions,
        offsets={motor: FACTORY_ENCODER_OFFSET for motor in positions},
        ids=sorted(positions),
        **kwargs,
    )
    bus.open()
    return bus


# ---------------------------------------------------------------------------
# Harness
# ---------------------------------------------------------------------------


def _patch_bus(monkeypatch, bus, port: str = "/dev/ttyACM_fake") -> None:
    monkeypatch.setattr(arm_cmd, "_candidate_ports", lambda: [port])
    monkeypatch.setattr(arm_cmd, "_open_bus", lambda _p: bus)
    # A FakeBus's close() is a no-op on the simulation, but the verb calls it — and a
    # test that then read the bus would be reading a closed one. Keep it open.
    monkeypatch.setattr(type(bus), "close", lambda self: None)


def _args(
    joint: "list[str] | None" = None,
    *,
    role: str = ROLE,
    port: "str | None" = None,
    apply: bool = True,
    json_mode: bool = True,
    step: "int | None" = None,
    max_travel: "int | None" = None,
    compliance: "int | None" = None,
    pose: "str | None" = None,
    threshold: "int | None" = None,
) -> argparse.Namespace:
    return argparse.Namespace(
        joint=list(joint) if joint else [],
        role=role,
        port=port,
        apply=apply,
        json=json_mode,
        step=step,
        max_travel=max_travel,
        compliance=compliance,
        pose=pose,
        threshold=threshold,
        threshold_joint=None,
        threshold_file=None,
    )


def _run(monkeypatch, capsys, bus, **kwargs) -> dict:
    """Drive the verb against *bus* and return the parsed ``--json`` payload."""
    _patch_bus(monkeypatch, bus)
    arm_cmd.cmd_arm_limits(_args(**kwargs))
    return json.loads(capsys.readouterr().out)


def _one(payload: dict, joint: str) -> dict:
    (found,) = [entry for entry in payload["joints"] if entry["joint"] == joint]
    return found


# ---------------------------------------------------------------------------
# Registration + the CLI surface
# ---------------------------------------------------------------------------


def test_limits_is_registered_under_the_arm_noun() -> None:
    from arm101.cli import _build_parser

    args = _build_parser().parse_args(["arm", "limits"])
    assert args.func is arm_cmd.cmd_arm_limits
    assert args.joint == []  # no joint named -> every joint
    assert args.role == ROLE
    assert args.apply is False
    assert args.json is False
    assert args.step is None
    assert args.max_travel is None
    assert args.compliance is None


def test_limits_takes_any_number_of_joints() -> None:
    from arm101.cli import _build_parser

    args = _build_parser().parse_args(["arm", "limits", PAN, ELBOW, "--apply", "--json"])
    assert args.joint == [PAN, ELBOW]
    assert args.apply is True
    assert args.json is True


def test_a_bad_joint_is_a_user_error_and_never_opens_a_bus(monkeypatch) -> None:
    def _explode(_port):  # pragma: no cover - the point is that it is never reached
        raise AssertionError("a bad joint name must be caught before any bus is opened")

    monkeypatch.setattr(arm_cmd, "_open_bus", _explode)
    with pytest.raises(CliError) as exc:
        arm_cmd.cmd_arm_limits(_args(joint=["elbow"]))
    assert exc.value.code == EXIT_USER_ERROR
    assert "elbow" in exc.value.message


# ---------------------------------------------------------------------------
# AC1 — MEASURE-ONLY. It looks; it does not re-calibrate.
# ---------------------------------------------------------------------------


def test_there_is_NO_commit_flag__committing_a_rezero_is_a_separate_gated_act(capsys) -> None:
    """The one flag this verb must not have.

    Committing a re-zero is a separate, explicitly gated act. A measure verb that can
    also keep the calibration it borrowed is one ``--commit`` typo away from silently
    re-zeroing every joint the operator asked it to *look* at.
    """
    from arm101.cli import _build_parser

    with pytest.raises(SystemExit) as exc:
        _build_parser().parse_args(["arm", "limits", ELBOW, "--commit"])
    assert exc.value.code == EXIT_USER_ERROR
    assert "unrecognized arguments: --commit" in capsys.readouterr().err


def test_the_run_puts_every_joints_ORIGINAL_offset_back(monkeypatch, capsys) -> None:
    """The servo ends the run in the calibration it started it in. Exactly."""
    bus = _servo(
        _BoundedServo,
        joints={PAN: 4000, ELBOW: 4000},
        down=300,
        up=1500,
    )
    before = {motor: bus.read_offset(motor) for motor in (PAN_MOTOR, ELBOW_MOTOR)}

    _run(monkeypatch, capsys, bus, joint=[PAN, ELBOW])

    after = {motor: bus.read_offset(motor) for motor in (PAN_MOTOR, ELBOW_MOTOR)}
    assert (
        after == before == {PAN_MOTOR: FACTORY_ENCODER_OFFSET, ELBOW_MOTOR: FACTORY_ENCODER_OFFSET}
    )


def test_it_really_DID_roll_the_seam__and_really_DID_put_it_back(monkeypatch, capsys) -> None:
    """ "Restored" must not be satisfied by never having touched the offset at all.

    The frame's whole job is to move the seam out of the joint's way, so the run MUST
    write the offset register — and the last thing it writes to it must be the original.
    """
    bus = _servo(_BoundedServo, joints={ELBOW: 4000}, down=300, up=1500)

    _run(monkeypatch, capsys, bus, joint=[ELBOW])

    writes = [w["offset"] for w in bus.offset_writes if w["motor"] == ELBOW_MOTOR]
    assert writes, "the frame never rolled the seam — nothing was measured across it"
    assert any(offset != FACTORY_ENCODER_OFFSET for offset in writes), (
        "the offset was never actually shifted, so the seam was never moved out of the "
        "joint's way and the measurement crossed it"
    )
    assert writes[-1] == FACTORY_ENCODER_OFFSET


def test_the_journal_closes_RESTORED__never_committed(monkeypatch, capsys, tmp_path) -> None:
    """The calibration transaction ends the way a MEASUREMENT ends.

    ``restore`` and ``commit`` are the only two ways to close a frame, and they mean
    opposite things: one puts the servo back, the other declares the borrowed offset to
    be the joint's calibration from now on. The journal is truncated once nothing is in
    flight (both endings leave it clean), so the disposition is the observable — and it
    is the one that says which of the two happened.
    """
    monkeypatch.setenv("ARM101_CALIBRATION_JOURNAL", str(tmp_path / "journal.jsonl"))
    dispositions: list[str] = []
    real_end = CalibrationJournal.end

    def _spy(self, *, motor: int, disposition: str) -> None:
        dispositions.append(disposition)
        real_end(self, motor=motor, disposition=disposition)

    monkeypatch.setattr(CalibrationJournal, "end", _spy)
    bus = _servo(_BoundedServo, joints={ELBOW: 4000}, down=300, up=1500)

    _run(monkeypatch, capsys, bus, joint=[ELBOW])

    assert dispositions, "the calibration transaction was never closed at all"
    assert set(dispositions) == {DISPOSITION_RESTORED}
    assert DISPOSITION_COMMITTED not in dispositions
    # ...and nothing is left in flight for the next run to have to recover.
    assert CalibrationJournal(default_journal_path()).dirty_entries() == []


def test_a_dirty_journal_from_a_CRASHED_run_is_recovered_before_anything_is_touched(
    monkeypatch, capsys, tmp_path
) -> None:
    """``require_clean`` runs at startup — before the verb touches the arm.

    A previous run died holding a temporary offset. The joint is still in a frame
    nobody chose, and no tick it reports means what it says. The verb must put that
    back first, not layer a fresh frame on top of the only record of the truth.
    """
    monkeypatch.setenv("ARM101_CALIBRATION_JOURNAL", str(tmp_path / "journal.jsonl"))
    journal = CalibrationJournal(default_journal_path())
    journal.begin(joint=ELBOW, motor=ELBOW_MOTOR, original_offset=FACTORY_ENCODER_OFFSET)
    journal.record_offset(motor=ELBOW_MOTOR, offset=-1000)

    # ...and the servo really is holding the temporary offset the crash left behind.
    bus = _servo(_BoundedServo, joints={ELBOW: 4000}, down=300, up=1500)
    bus.write_offset(ELBOW_MOTOR, -1000)
    assert bus.read_offset(ELBOW_MOTOR) == -1000

    _run(monkeypatch, capsys, bus, joint=[ELBOW])

    assert bus.read_offset(ELBOW_MOTOR) == FACTORY_ENCODER_OFFSET
    assert CalibrationJournal(default_journal_path()).dirty_entries() == []


# ---------------------------------------------------------------------------
# AC2 — the whole probe runs inside a torque guard. Asserted for THIS verb.
# ---------------------------------------------------------------------------


def test_a_bus_that_DIES_mid_probe_leaves_EVERY_motor_it_energised_released(
    monkeypatch, capsys
) -> None:
    """The incident, reproduced: the bus dies while joint 2 of 2 is being probed.

    ``shoulder_pan``'s frame closed long ago — nothing in the measuring path would ever
    go back to it. Only the guard, which has owned it since the moment it could first go
    hot, sweeps it. So the final act of the run must be a release across BOTH motors,
    and the exception the operator sees must still be the one that actually happened.
    """
    bus = _servo(
        _DyingBus,
        joints={PAN: 4000, ELBOW: 4000},
        down=300,
        up=1500,
        dies_on_motor=ELBOW_MOTOR,
        after_reads=20,
    )
    _patch_bus(monkeypatch, bus)

    with pytest.raises(RuntimeError, match="device reports readiness"):
        arm_cmd.cmd_arm_limits(_args(joint=[PAN, ELBOW]))

    # The guard's sweep is the LAST thing that happens, and it covers every motor the
    # run ever energised — the one that died AND the one that finished minutes ago.
    assert bus.cleared[-2:] == [PAN_MOTOR, ELBOW_MOTOR]

    # ...and it told the operator, on stderr, while the exception was still unwinding.
    assert "torque_release" in capsys.readouterr().err


def test_the_guard_owns_the_joints_it_probes__and_NOT_the_ones_it_does_not(
    monkeypatch, capsys
) -> None:
    """A safety report that cries wolf teaches a human to ignore the one line that matters."""
    bus = _servo(
        _DyingBus,
        joints={PAN: 4000, ELBOW: 4000},
        down=300,
        up=1500,
        dies_on_motor=ELBOW_MOTOR,
        after_reads=0,  # dies on the FIRST read of elbow_flex
    )
    _patch_bus(monkeypatch, bus)

    with pytest.raises(RuntimeError):
        arm_cmd.cmd_arm_limits(_args(joint=[ELBOW]))

    # shoulder_pan was never named, never probed, never energised — and is never claimed.
    assert PAN_MOTOR not in bus.cleared


def test_a_CLEAN_run_performs_no_release_sweep__hold_on_success(monkeypatch, capsys) -> None:
    """Torque is exactly as the verb left it. The guard is a net, not a policy."""
    bus = _servo(_BoundedServo, joints={ELBOW: 4000}, down=300, up=1500)
    _patch_bus(monkeypatch, bus)

    arm_cmd.cmd_arm_limits(_args(joint=[ELBOW]))

    assert "torque_release" not in capsys.readouterr().err


# ---------------------------------------------------------------------------
# AC3 — per-joint bounds and verdicts ONLY. This is not issue #34.
# ---------------------------------------------------------------------------


def test_the_verb_never_touches_the_reachability_machinery(monkeypatch, capsys) -> None:
    """No cells, no reachability score, no map. If it does any of those it IS issue #34."""

    def _forbidden(*_args, **_kwargs):  # pragma: no cover - the point is it is never called
        raise AssertionError(
            "arm limits reached for the reachability machinery. It measures per-joint "
            "bounds and verdicts; enqueueing cells or emitting a map is issue #34, which "
            "is explicitly out of scope."
        )

    monkeypatch.setattr(arm_cmd.engine, "explore", _forbidden)
    monkeypatch.setattr(arm_cmd, "_build_grid_spec", _forbidden)
    monkeypatch.setattr(arm_cmd, "GridSpec", _forbidden)

    bus = _servo(_BoundedServo, joints={ELBOW: 4000}, down=300, up=1500)
    payload = _run(monkeypatch, capsys, bus, joint=[ELBOW])
    assert payload["joints"]


def test_the_payload_carries_bounds_and_verdicts__and_nothing_that_scores_reachability(
    monkeypatch, capsys
) -> None:
    bus = _servo(_BoundedServo, joints={ELBOW: 4000}, down=300, up=1500)
    payload = _run(monkeypatch, capsys, bus, joint=[ELBOW])

    assert set(payload) == {"verb", "role", "port", "pose", "joints", "bounds_diff"}

    # Every key ``arm explore`` emits and this verb must not. (``reachable_raw_ends`` is
    # a joint's two WALLS — a bounds fact, not a reachability score — so the check is on
    # explore's key NAMES, not on a substring that would collide with it.)
    flattened = json.dumps(payload)
    for forbidden in (
        "cells_visited",
        '"reachable"',
        "map_path",
        "log_path",
        "escapes_attempted",
        "escapes_succeeded",
        "budget_bounded",
    ):
        assert forbidden not in flattened, f"this is arm explore's, not arm limits': {forbidden}"


def test_the_verb_writes_no_files(monkeypatch, capsys, tmp_path) -> None:
    """``arm explore`` writes a map and an event log. This verb writes nothing."""
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("ARM101_CALIBRATION_JOURNAL", str(tmp_path / "j" / "journal.jsonl"))
    bus = _servo(_BoundedServo, joints={ELBOW: 4000}, down=300, up=1500)

    _run(monkeypatch, capsys, bus, joint=[ELBOW])

    # The calibration journal is the ONE file a measurement is allowed to write — it is
    # the crash-recovery record, and it lives outside the working directory.
    assert list(tmp_path.iterdir()) == [tmp_path / "j"]


# ---------------------------------------------------------------------------
# AC4 — the measurement itself, re-derived from the arm and nothing else
# ---------------------------------------------------------------------------


def test_a_joint_walled_at_both_ends_reads_BOUNDED__and_its_span_is_what_it_travelled(
    monkeypatch, capsys
) -> None:
    """The core measurement, across a seam a ``[min, max]`` pair could not have held."""
    down, up = 300, 1500
    bus = _servo(_BoundedServo, joints={ELBOW: 4000}, down=down, up=up)

    payload = _run(monkeypatch, capsys, bus, joint=[ELBOW])
    elbow = _one(payload, ELBOW)

    assert elbow["kind"] == TravelKind.BOUNDED.value
    assert elbow["swept_ticks"] == bus.span() == down + up
    assert elbow["seam_in_travel"] is True  # raw 3700 -> up across 4095->0 -> raw 1404
    assert elbow["ends"]["low"]["verdict"] == LimitVerdict.WALL.value
    assert elbow["ends"]["high"]["verdict"] == LimitVerdict.WALL.value
    assert elbow["remedy"] == SeamRemedy.REZERO.value


def test_a_joint_free_all_the_way_round_reads_CONTINUOUS__and_the_second_end_is_not_probed(
    monkeypatch, capsys
) -> None:
    """A full turn settles it. Demanding a second end would force a fabricated observation.

    And it must come back CONTINUOUS **because it is** — the classifier holds no joint
    table, so this is asserted under a joint that is not ``wrist_roll``.
    """
    bus = _servo(RollingServoBus, joints={PAN: 2048})

    payload = _run(monkeypatch, capsys, bus, joint=[PAN])
    pan = _one(payload, PAN)

    assert pan["kind"] == TravelKind.CONTINUOUS.value
    assert pan["swept_ticks"] >= ENCODER_TICKS
    assert set(pan["ends"]) == {"low"}, "the joint went all the way round; there is nothing left"
    assert pan["remedy"] == SeamRemedy.SOFT_LIMIT.value
    assert pan["unreachable_arc"] is None


def test_a_joint_that_runs_out_of_TORQUE_is_a_lower_bound__never_a_wall(
    monkeypatch, capsys
) -> None:
    """The verdict this whole module family exists to keep apart from a wall.

    A gravity-loaded joint stalls at a saturated load with NOTHING in front of it. It
    reads identically to a wall at the moment of the stop — so the probe rules on the
    approach, and the verb must carry that ruling through to the operator intact.
    """
    threshold = DEFAULT_CONTACT_THRESHOLDS[ELBOW]
    # Slope chosen so the loaded run — (CEILING - threshold) / slope — is comfortably
    # WIDER than a real contact's measured give (wall_compliance()), which is what makes
    # this a torque climb and not a wall. Derived, so a retuned compliance moves it.
    slope = 1.0
    loaded_run = (CEILING - threshold) / slope
    assert loaded_run > wall_compliance(), "the fixture must model a torque climb, not a wall"

    bus = _servo(_GravityServo, joints={ELBOW: 2048}, load_per_tick=slope, lift=-1)

    payload = _run(monkeypatch, capsys, bus, joint=[ELBOW], max_travel=600)
    elbow = _one(payload, ELBOW)

    assert elbow["ends"]["low"]["verdict"] == LimitVerdict.TORQUE_LIMITED.value
    assert elbow["kind"] == TravelKind.UNDETERMINED.value
    assert elbow["reachable_raw_ends"] is None  # a lower bound is not a wall
    assert elbow["remedy"] == SeamRemedy.UNKNOWN.value


def test_EVERY_end_carries_the_evidence_the_hardware_run_exists_to_collect(
    monkeypatch, capsys
) -> None:
    """The cutoff that decides WALL vs TORQUE_LIMITED comes from a SIMULATION today.

    The first hardware session is what replaces it with real data, and
    ``loaded_run_ticks`` is the number it has to be replaced FROM. If it is not in the
    payload, the operator has to re-instrument the run to get it — and the run is
    wasted. Same for the free run, the peak load, the verdict and the reason.
    """
    bus = _servo(_BoundedServo, joints={ELBOW: 4000}, down=300, up=1500)
    payload = _run(monkeypatch, capsys, bus, joint=[ELBOW])

    for end in ("low", "high"):
        evidence = _one(payload, ELBOW)["ends"][end]
        for field in (
            "loaded_run_ticks",
            "free_run_ticks",
            "compliance",
            "peak_load",
            "verdict",
            "reason",
            "moves",
            "recentres",
            "samples",
        ):
            assert field in evidence, f"{end} end is missing {field}"
        assert isinstance(evidence["loaded_run_ticks"], int)
        assert evidence["peak_load"] >= CEILING  # it pressed into a real wall
        assert evidence["compliance"] == wall_compliance()
        assert evidence["observation"]["joint"] == ELBOW


def test_the_compliance_cutoff_is_tunable_from_the_bench_without_a_code_change(
    monkeypatch, capsys
) -> None:
    """The point of shipping ``--compliance``: retune it from the data, not from a diff."""
    bus = _servo(_BoundedServo, joints={ELBOW: 4000}, down=300, up=1500)
    payload = _run(monkeypatch, capsys, bus, joint=[ELBOW], compliance=7)
    assert _one(payload, ELBOW)["ends"]["low"]["compliance"] == 7


def test_the_pose_is_recorded_on_every_observation(monkeypatch, capsys) -> None:
    """An observation is only ever evidence ABOUT A POSE. The label travels with it."""
    bus = _servo(_BoundedServo, joints={ELBOW: 4000}, down=300, up=1500)
    payload = _run(monkeypatch, capsys, bus, joint=[ELBOW], pose="arm-folded")

    assert payload["pose"] == "arm-folded"
    assert _one(payload, ELBOW)["ends"]["low"]["observation"]["pose"] == "arm-folded"
    assert _one(payload, ELBOW)["ends"]["high"]["observation"]["pose"] == "arm-folded"


def test_the_pose_is_named_in_the_TEXT_report_too(monkeypatch, capsys) -> None:
    """A limit found in one pose may be an obstacle. The human reading the table needs
    to know which pose it was, without going to the JSON for it."""
    bus = _servo(_BoundedServo, joints={ELBOW: 4000}, down=300, up=1500)
    _patch_bus(monkeypatch, bus)

    arm_cmd.cmd_arm_limits(_args(joint=[ELBOW], json_mode=False, pose="gripper clear of table"))

    assert "Pose: gripper clear of table" in capsys.readouterr().out


def test_every_joint_is_measured_when_none_is_named(monkeypatch, capsys) -> None:
    bus = _servo(
        _BoundedServo,
        joints={joint: 4000 for joint in JOINTS},
        down=300,
        up=1500,
    )
    payload = _run(monkeypatch, capsys, bus)
    assert [entry["joint"] for entry in payload["joints"]] == list(JOINTS)


# ---------------------------------------------------------------------------
# AC5 (t11) — the bounds diff, and the verdict it must be able to give AGAINST itself
# ---------------------------------------------------------------------------


def _diff(joint: str, *, measured: int, eeprom: int, vouched: bool = True) -> dict:
    delta = measured - eeprom
    return {
        "joint": joint,
        "eeprom_reported_bounds": [0, eeprom],
        "eeprom_span_ticks": eeprom,
        "measured_span_ticks": measured,
        "span_delta_ticks": delta,
        "material": abs(delta) > arm_cmd.MATERIAL_SPAN_DELTA_TICKS,
        "vouched": vouched,
    }


def test_the_diff_reports_the_delta_between_the_measured_span_and_the_one_explore_uses(
    monkeypatch, capsys
) -> None:
    """t11, on the real path: what does ``arm explore`` believe, and what is true?

    ``arm explore`` builds its grid from the servo's EEPROM angle limits (intersected
    with the soft limit) — untouched factory ``0-4095`` on this arm. The joint has
    nothing like that much travel, so the grid is enqueueing cells it cannot reach.
    """
    down, up = 300, 1500
    bus = _servo(_BoundedServo, joints={ELBOW: 4000}, down=down, up=up)

    payload = _run(monkeypatch, capsys, bus, joint=[ELBOW])
    bounds = _one(payload, ELBOW)["bounds"]

    eeprom_min, eeprom_max = bounds["eeprom_reported_bounds"]
    assert (eeprom_min, eeprom_max) == (0, 4095)  # the factory EEPROM, which knows nothing
    assert bounds["measured_span_ticks"] == down + up
    assert bounds["span_delta_ticks"] == (down + up) - bounds["eeprom_span_ticks"]
    assert bounds["span_delta_ticks"] < 0  # the EEPROM claims travel the arm does not have
    assert bounds["material"] is True

    diff = payload["bounds_diff"]
    assert diff["any_material"] is True
    assert diff["material_joints"] == [ELBOW]
    assert diff["material_threshold_ticks"] == arm_cmd.MATERIAL_SPAN_DELTA_TICKS


def test_when_NO_joint_differs_materially_the_report_SAYS_SO__against_its_own_interest(
    monkeypatch, capsys
) -> None:
    """The verdict this report must be capable of delivering, and must not bury.

    If no joint's measured span differs materially from the EEPROM-derived span, then
    the grid was NOT being fed artifacts — and the rationale for blocking issue #34 on
    this work is FALSE. A report that could only ever confirm its author's premise is
    not a measurement.

    Modelled by an operator who has already written honest angle limits into EEPROM.
    """
    down, up = 300, 1500
    span = down + up
    bus = _servo(
        _BoundedServo,
        joints={ELBOW: 4000},
        down=down,
        up=up,
        info={ELBOW_MOTOR: {"min_angle": 100, "max_angle": 100 + span}},
    )

    payload = _run(monkeypatch, capsys, bus, joint=[ELBOW])
    bounds = _one(payload, ELBOW)["bounds"]
    assert bounds["span_delta_ticks"] == 0
    assert bounds["material"] is False

    diff = payload["bounds_diff"]
    assert diff["any_material"] is False
    assert diff["material_joints"] == []
    verdict = diff["verdict"].lower()
    assert "#34" in verdict
    assert "does not hold" in verdict


def test_the_no_material_verdict_names_issue_34_and_refuses_to_bury_it() -> None:
    """The pure function, both branches, with no arm attached."""
    quiet = arm_cmd._bounds_diff([_diff(PAN, measured=4000, eeprom=4095)])
    assert quiet["any_material"] is False
    assert "does not hold" in quiet["verdict"].lower()
    assert "#34" in quiet["verdict"]

    loud = arm_cmd._bounds_diff([_diff(PAN, measured=1800, eeprom=4095)])
    assert loud["any_material"] is True
    assert loud["material_joints"] == [PAN]
    assert "#34" in loud["verdict"]


def test_a_span_with_no_wall_behind_it_is_flagged_as_a_LOWER_BOUND_in_the_diff() -> None:
    """A delta computed from an unvouched end is itself only a bound. Say so."""
    diff = arm_cmd._bounds_diff(
        [
            _diff(PAN, measured=4000, eeprom=4095, vouched=False),
            _diff(ELBOW, measured=4050, eeprom=4095, vouched=True),
        ]
    )
    assert diff["unvouched_joints"] == [PAN]
    assert "lower bound" in diff["verdict"].lower()


def test_the_diff_verdict_survives_a_run_that_measured_nothing() -> None:
    """No joints is not a verdict about the grid. It must not read like one."""
    diff = arm_cmd._bounds_diff([])
    assert diff["any_material"] is False
    assert diff["joints"] == []
    assert "no joint" in diff["verdict"].lower()


# ---------------------------------------------------------------------------
# Consent — it commands motion, so it is gated like every other motion verb
# ---------------------------------------------------------------------------


class _FakeStdin:
    """Scripted stdin controlling ``isatty()`` and ``readline()``."""

    def __init__(self, lines: "list[str]", tty: bool = True) -> None:
        self._lines = list(lines)
        self._tty = tty

    def isatty(self) -> bool:
        return self._tty

    def readline(self) -> str:
        return self._lines.pop(0) if self._lines else ""


def _no_bus(monkeypatch) -> None:
    def _explode(_port):  # pragma: no cover - the point is that it is never reached
        raise AssertionError("no bus may be opened on this path")

    monkeypatch.setattr(arm_cmd, "_open_bus", _explode)
    monkeypatch.setattr(arm_cmd, "_candidate_ports", lambda: ["/dev/ttyACM_fake"])


def test_a_dry_run_plans_the_measurement_and_opens_NO_bus(monkeypatch, capsys) -> None:
    _no_bus(monkeypatch)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_limits(_args(joint=[ELBOW], apply=False, json_mode=True))

    plan = json.loads(capsys.readouterr().out)["plan"]
    assert plan["verb"] == "arm limits"
    assert plan["joints"] == [ELBOW]
    assert plan["step"] == DEFAULT_CREEP_TICKS
    assert plan["compliance"] == wall_compliance()
    assert "MEASURE-ONLY" in plan["note"]


def test_the_TEXT_dry_run_plan_names_the_knobs_and_says_it_commanded_nothing(
    monkeypatch, capsys
) -> None:
    _no_bus(monkeypatch)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_limits(_args(joint=[ELBOW], apply=False, json_mode=False))

    out = capsys.readouterr().out
    assert "Dry-run plan: arm limits" in out
    assert f"compliance: {wall_compliance()}" in out
    assert "MEASURE-ONLY" in out
    assert "No motion commanded (dry-run)." in out


def test_a_human_at_a_TTY_is_WARNED_about_the_table_before_anything_moves(
    monkeypatch, capsys
) -> None:
    """The motion gate. This verb drives every named joint into whatever stops it.

    If that is the table rather than the joint's own end-stop, the run records the table
    — a wall that is not the joint's. The human has to be told BEFORE they say yes, not
    after, so the warning goes out ahead of the prompt and ahead of any bus.
    """
    bus = _servo(_BoundedServo, joints={ELBOW: 4000}, down=300, up=1500)
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["yes\n"], tty=True))

    arm_cmd.cmd_arm_limits(_args(joint=[ELBOW], apply=False, json_mode=True))

    captured = capsys.readouterr()
    assert "COMMANDS MOTION" in captured.err
    assert "table" in captured.err  # the specific hazard, named
    assert json.loads(captured.out)["joints"]  # ...and it went ahead


def test_declining_at_the_prompt_measures_NOTHING(monkeypatch, capsys) -> None:
    """'no' means no bus, no motion, no partial measurement — and exit 0."""
    _no_bus(monkeypatch)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["no\n"], tty=True))

    arm_cmd.cmd_arm_limits(_args(joint=[ELBOW], apply=False, json_mode=True))

    assert json.loads(capsys.readouterr().out) == {"aborted": True, "role": ROLE}


def test_declining_at_the_prompt_says_so_in_TEXT_mode_too(monkeypatch, capsys) -> None:
    _no_bus(monkeypatch)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["no\n"], tty=True))

    arm_cmd.cmd_arm_limits(_args(joint=[ELBOW], apply=False, json_mode=False))

    assert "Aborted; no motion commanded." in capsys.readouterr().out


def test_the_text_report_names_the_verdict_the_span_and_the_delta(monkeypatch, capsys) -> None:
    bus = _servo(_BoundedServo, joints={ELBOW: 4000}, down=300, up=1500)
    _patch_bus(monkeypatch, bus)

    arm_cmd.cmd_arm_limits(_args(joint=[ELBOW], json_mode=False))

    out = capsys.readouterr().out
    assert "arm limits" in out
    assert ELBOW in out
    assert TravelKind.BOUNDED.value in out
    assert "1800" in out  # the measured span
    assert "#34" in out  # the bounds-diff verdict is in the TEXT report too, not just JSON


# ---------------------------------------------------------------------------
# Catalog lockstep — a verb without docs is how the docs silently drift
# ---------------------------------------------------------------------------


def test_the_verb_is_documented_in_every_one_of_the_four_places() -> None:
    """`explain`, `overview`, `learn` (text) and `learn` (json), in lockstep."""
    from arm101.cli._commands import learn, overview
    from arm101.explain.catalog import ENTRIES

    assert ("arm", "limits") in ENTRIES
    entry = ENTRIES[("arm", "limits")]
    assert "arm limits" in entry
    assert "MEASURE-ONLY" in entry

    assert any(verb.startswith("arm limits") for verb in overview._VERBS)
    assert "arm101-cli arm limits" in learn._TEXT
    paths = [tuple(cmd["path"]) for cmd in learn._as_json_payload()["commands"]]
    assert ("arm", "limits") in paths
    assert "arm limits" in learn._as_json_payload()["hardware"]["verbs"]


def test_arm_overview_lists_limits_among_the_arm_verbs(capsys) -> None:
    arm_cmd.cmd_arm_overview(argparse.Namespace(json=True))
    payload = json.loads(capsys.readouterr().out)
    assert "limits" in payload["verbs"]
