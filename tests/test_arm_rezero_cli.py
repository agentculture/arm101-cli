"""Tests for the ``arm rezero`` CLI verb — the gated EEPROM write for issue #35,
and the ``--verify`` sweep that proves it actually worked.

All hardware is a :class:`~arm101.hardware.bus.FakeBus` injected through the
``arm._open_bus`` / ``arm._candidate_ports`` seam (the same seam ``read`` /
``flex`` / ``explore`` use), so no serial port is ever opened.

Two properties are asserted over and over, because they are the two a hardware
run cannot forgive:

1. **No motion, on any path.** ``elbow_flex`` rests at raw ~126, PAST its wrap, so
   a linear goal would rotate it the long way round through its whole travel and
   into a wall. The verb must never write a goal position and must never energise
   the joint.
2. **``--verify`` fails under ``offset_wraps=False``.** That is the pessimistic
   reading of the one undocumented firmware behaviour the re-zero rests on, and
   under it the re-zero achieves nothing. The verb must say so, loudly, with a
   non-zero exit — that failure is what stands between the operator and shipping
   on top of a fix that did nothing.
"""

from __future__ import annotations

import argparse
import json
import sys

import pytest

from arm101.cli._commands import arm as arm_cmd
from arm101.cli._errors import EXIT_ENV_ERROR, EXIT_USER_ERROR, CliError
from arm101.hardware import rezero
from arm101.hardware.bus import ADDR_HOMING_OFFSET, FakeBus, OverloadError
from tests.test_rezero import (
    ARC_HIGH,
    ARC_LOW,
    ELBOW,
    ELBOW_MOTOR,
    EXPECTED_OFFSET,
    FACTORY_OFFSET,
    OUR_ARM_OFFSET,
    HandMovedBus,
)

#: STS3215 Goal_Position — the register this verb must never, ever write.
ADDR_GOAL_POSITION = 42


# ---------------------------------------------------------------------------
# Helpers
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


def _patch_bus(monkeypatch, fake: FakeBus, port: str = "/dev/ttyACM_fake") -> None:
    fake.open()
    monkeypatch.setattr(arm_cmd, "_candidate_ports", lambda: [port])
    monkeypatch.setattr(arm_cmd, "_open_bus", lambda _p: fake)


def _args(
    joint: str = ELBOW,
    verify: bool = False,
    duration: "float | None" = None,
    role: str = "follower",
    port: "str | None" = None,
    apply: bool = False,
    json_mode: bool = False,
):
    return argparse.Namespace(
        joint=joint,
        verify=verify,
        duration=duration,
        role=role,
        port=port,
        apply=apply,
        json=json_mode,
    )


#: A sweep short enough to run instantly against a FakeBus (which needs no
#: pacing) but long enough to walk the hand-moved shaft across the seam:
#: 2.5s / 0.05s = 50 samples x 50 ticks = 2500 raw ticks, comfortably past the
#: 2196-tick travel.
SWEEP_SECONDS = 2.5
SWEEP_STEP = 50


def _swept_bus(*, offset_wraps: bool = True, rezeroed: bool = True) -> HandMovedBus:
    bus = HandMovedBus(
        ticks_per_read=SWEEP_STEP,
        offsets={ELBOW_MOTOR: EXPECTED_OFFSET} if rezeroed else None,
        offset_wraps=offset_wraps,
    )
    return bus


# ---------------------------------------------------------------------------
# Registration
# ---------------------------------------------------------------------------


def test_rezero_is_registered_under_the_arm_noun():
    from arm101.cli import _build_parser

    args = _build_parser().parse_args(["arm", "rezero", ELBOW])
    assert args.func is arm_cmd.cmd_arm_rezero
    assert args.joint == ELBOW
    assert args.verify is False
    assert args.apply is False
    assert args.json is False
    assert args.role == "follower"


def test_rezero_flags_parse():
    from arm101.cli import _build_parser

    args = _build_parser().parse_args(
        ["arm", "rezero", ELBOW, "--verify", "--duration", "45", "--apply", "--json"]
    )
    assert (args.verify, args.duration, args.apply, args.json) == (True, 45.0, True, True)


def test_arm_overview_lists_rezero(capsys):
    arm_cmd.cmd_arm_overview(argparse.Namespace(json=True))
    payload = json.loads(capsys.readouterr().out)
    assert "rezero" in payload["verbs"]


# ---------------------------------------------------------------------------
# Eligibility — answered with NO hardware attached
# ---------------------------------------------------------------------------


def test_wrist_roll_is_refused_with_the_reason_and_no_bus_is_opened(monkeypatch):
    """ "Why can't I re-zero this joint?" is a question about the arm's GEOMETRY.

    It deserves an answer on a laptop, with no servo plugged in — so the
    eligibility check runs before consent, before a port is resolved, and before
    a bus is opened. If it needed hardware, the one operator who most needs the
    explanation (the one who does not understand the arm yet) is the one least
    able to get it.
    """

    def _explode(_port):  # pragma: no cover - must never be reached
        raise AssertionError("a bus was opened for a joint that cannot be re-zeroed")

    monkeypatch.setattr(arm_cmd, "_open_bus", _explode)
    monkeypatch.setattr(arm_cmd, "_candidate_ports", lambda: [])
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    with pytest.raises(CliError) as exc:
        arm_cmd.cmd_arm_rezero(_args(joint="wrist_roll", apply=True))

    assert exc.value.code == EXIT_USER_ERROR
    assert "RELOCATES" in exc.value.message
    assert "EVICT" in exc.value.message
    assert "SOFT LIMIT" in exc.value.message


@pytest.mark.parametrize("joint", ["shoulder_pan", "shoulder_lift", "wrist_flex", "gripper"])
def test_a_joint_that_never_wraps_is_refused_as_unnecessary(joint, monkeypatch):
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))
    with pytest.raises(CliError) as exc:
        arm_cmd.cmd_arm_rezero(_args(joint=joint))
    assert exc.value.code == EXIT_USER_ERROR
    assert "does not need a re-zero" in exc.value.message


def test_an_unknown_joint_is_a_user_error(monkeypatch):
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))
    with pytest.raises(CliError) as exc:
        arm_cmd.cmd_arm_rezero(_args(joint="knee"))
    assert exc.value.code == EXIT_USER_ERROR


# ---------------------------------------------------------------------------
# Consent gate — dry-run / TTY prompt / --apply
# ---------------------------------------------------------------------------


def test_dry_run_prints_the_exact_writes_and_opens_NO_bus(monkeypatch, capsys):
    """Zero writes AND zero bus access — like ``flex``'s and ``explore``'s dry-runs.

    Everything a plan can honestly say about a re-zero is already known without a
    servo (the offset is derived from the arc table, the wire value from the
    offset). Everything it cannot say offline is a live fact, checked at apply
    time where it can actually be acted on.
    """

    def _explode(_port):  # pragma: no cover - must never be reached
        raise AssertionError("dry-run opened a bus")

    monkeypatch.setattr(arm_cmd, "_open_bus", _explode)
    monkeypatch.setattr(arm_cmd, "_candidate_ports", lambda: ["/dev/ttyACM_fake"])
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_rezero(_args())

    out = capsys.readouterr().out
    assert "Dry-run plan: arm rezero elbow_flex" in out
    assert "addr=31, value=1157" in out  # the exact wire value
    assert "addr=55, value=0" in out and "addr=55, value=1" in out  # the Lock dance
    assert "COMMANDS NO MOTION" in out
    assert "no goal position is ever written" in out


def test_dry_run_json_carries_the_whole_plan(monkeypatch, capsys):
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))
    monkeypatch.setattr(arm_cmd, "_candidate_ports", lambda: ["/dev/ttyACM_fake"])

    arm_cmd.cmd_arm_rezero(_args(json_mode=True))

    plan = json.loads(capsys.readouterr().out)["plan"]
    assert plan["joint"] == ELBOW
    assert plan["motor"] == ELBOW_MOTOR
    assert plan["target_offset"] == EXPECTED_OFFSET
    assert plan["wire_value"] == EXPECTED_OFFSET
    assert plan["unreachable_arc"] == [ARC_LOW, ARC_HIGH]
    assert plan["seam_moves_to_raw_tick"] == EXPECTED_OFFSET
    assert plan["expected_travel_ticks"] == 2196
    assert plan["mode"] == "write"


def test_the_dry_run_plan_SAYS_the_arc_is_raw_ticks_and_the_factory_offset_is_85(
    monkeypatch, capsys
):
    """The plan is where an operator meets these numbers. It must not mislead them.

    Both halves of the 2026-07-12 bug were invisible in the old plan: it printed an
    arc without saying which frame it was in, and it implied a factory servo holds
    0. An operator re-measuring the arc from that plan would reproduce the bug
    exactly.
    """
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))
    monkeypatch.setattr(arm_cmd, "_candidate_ports", lambda: ["/dev/ttyACM_fake"])

    arm_cmd.cmd_arm_rezero(_args(json_mode=True))

    plan = json.loads(capsys.readouterr().out)["plan"]
    assert plan["unreachable_arc_frame"] == "raw encoder ticks (NOT the ticks a servo reports)"
    assert "raw = reported + offset, mod 4096" in plan["note"]
    assert f"a factory servo holds {FACTORY_OFFSET}, not 0" in plan["note"]
    assert "writes NOTHING" in plan["note"]  # the no-op path is announced up front


def test_verify_dry_run_warns_that_the_arm_will_SAG(monkeypatch, capsys):
    """De-energising a joint that is holding a pose is a physical hazard in itself.

    ``--verify`` is not "read-only and therefore safe": it drops the torque and
    leaves it dropped. An operator who does not know that is an operator standing
    under a falling arm.
    """
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))
    monkeypatch.setattr(arm_cmd, "_candidate_ports", lambda: ["/dev/ttyACM_fake"])

    arm_cmd.cmd_arm_rezero(_args(verify=True, json_mode=True))

    plan = json.loads(capsys.readouterr().out)["plan"]
    assert plan["mode"] == "verify"
    assert "sag" in plan["note"]
    assert "DE-ENERGISES" in plan["note"]
    assert plan["writes"] == ["write1ByteTxRx(addr=40, value=0)    # Torque_Enable OFF"]


def test_tty_prompt_confirms_before_the_write(monkeypatch, capsys):
    bus = FakeBus(positions={ELBOW_MOTOR: 126})
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["yes\n"], tty=True))

    arm_cmd.cmd_arm_rezero(_args())

    captured = capsys.readouterr()
    assert "PERSISTENT" in captured.err
    assert bus.offset_writes == [{"motor": ELBOW_MOTOR, "offset": EXPECTED_OFFSET}]


def test_tty_prompt_declined_writes_NOTHING(monkeypatch, capsys):
    bus = FakeBus(positions={ELBOW_MOTOR: 126})
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["no\n"], tty=True))

    arm_cmd.cmd_arm_rezero(_args())

    assert bus.register_writes == []
    assert "Aborted" in capsys.readouterr().out


def test_non_tty_without_apply_never_writes(monkeypatch):
    bus = FakeBus(positions={ELBOW_MOTOR: 126})
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_rezero(_args(apply=False))

    assert bus.register_writes == []


# ---------------------------------------------------------------------------
# The write path
# ---------------------------------------------------------------------------


def test_apply_writes_the_offset_and_reports_the_read_back(monkeypatch, capsys):
    bus = FakeBus(positions={ELBOW_MOTOR: 126})
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_rezero(_args(apply=True, json_mode=True))

    payload = json.loads(capsys.readouterr().out)
    assert payload["read_back_offset"] == EXPECTED_OFFSET
    assert payload["applied"] is True
    assert payload["plan"]["raw_position"] == 126
    assert payload["shift"]["observed_position"] == 3065  # (126 − 1157) mod 4096
    assert payload["shift"]["as_predicted"] is True
    # Applied is not persistent, and applied is not evicted. Neither is claimed.
    assert payload["persistence_proven"] is False
    assert payload["seam_eviction_proven"] is False


def test_a_FACTORY_FRESH_ARM_at_offset_85_can_finally_BE_RE_ZEROED(monkeypatch, capsys):
    """THE case this fix exists for: the state every SO-101 ships in.

    All six servos hold ``Ofs = 85`` out of the box. The old verb refused this
    outright — *"already holds an encoder offset of 85, which is neither the
    factory 0 nor this joint's computed 1073 … cannot interpret"* — so on real,
    unmodified hardware ``arm rezero elbow_flex --apply`` did not work at all. It
    could only ever have run on a servo somebody had already hand-zeroed.

    Now it reads 85, converts out of it, finds the shaft at raw 126, and writes.
    """
    bus = FakeBus(positions={ELBOW_MOTOR: 126}, offsets={ELBOW_MOTOR: FACTORY_OFFSET})
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_rezero(_args(apply=True, json_mode=True))  # must NOT raise

    payload = json.loads(capsys.readouterr().out)
    assert payload["plan"]["current_offset"] == FACTORY_OFFSET
    assert payload["plan"]["current_seam_tick"] == 85  # inside the [0, 207] band: the bug
    assert payload["plan"]["reported_position"] == 41  # 126 − 85 — a REPORTED tick
    assert payload["plan"]["raw_position"] == 126  # ...and the raw one behind it
    assert payload["plan"]["already_applied"] is False
    assert payload["read_back_offset"] == EXPECTED_OFFSET
    assert payload["applied"] is True
    assert bus.offset_writes == [{"motor": ELBOW_MOTOR, "offset": EXPECTED_OFFSET}]


def test_the_write_path_COMMANDS_NO_MOTION(monkeypatch):
    """The one thing a hardware run cannot forgive, pinned at the register level.

    ``elbow_flex`` rests PAST its wrap: any linear goal rotates it the long way
    round, through its whole travel, into a wall. Asserted here on the CLI verb's
    complete write surface — not just on the primitive it calls.
    """
    bus = FakeBus(positions={ELBOW_MOTOR: 126})
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_rezero(_args(apply=True))

    assert bus.position_writes == []
    assert all(w["addr"] != ADDR_GOAL_POSITION for w in bus.register_writes)
    assert bus.speed_writes == []
    assert bus.accel_writes == []
    # Torque was touched, and only ever downward.
    assert bus.torque_writes and all(w["on"] is False for w in bus.torque_writes)


def test_the_result_TELLS_the_operator_to_power_cycle(monkeypatch, capsys):
    """PR #21 exists because an EEPROM write read back fine and reverted anyway.

    The read-back proves the value was APPLIED. Only a power-cycle proves it
    PERSISTS. A result that ended on a success line would let the operator walk
    away with neither fact established.
    """
    bus = FakeBus(positions={ELBOW_MOTOR: 126})
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_rezero(_args(apply=True))

    out = capsys.readouterr().out
    assert "POWER-CYCLE" in out
    assert "BUS POWER" in out
    assert "--verify" in out
    assert "does NOT prove the seam" in out


def test_a_second_run_on_an_already_re_zeroed_joint_writes_nothing(monkeypatch, capsys):
    """The procedure sends the operator away to power-cycle. Coming back is normal."""
    bus = FakeBus(positions={ELBOW_MOTOR: 126}, offsets={ELBOW_MOTOR: EXPECTED_OFFSET})
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_rezero(_args(apply=True, json_mode=True))

    payload = json.loads(capsys.readouterr().out)
    assert payload["written"] is False
    assert payload["reason"] == "already-applied"
    assert bus.register_writes == []


def test_OUR_ARM_at_1073_is_a_clean_NO_OP_not_a_rewrite_to_1157(monkeypatch, capsys):
    """The arm on the bench, and the reason "already-applied" is not "offset == target".

    Our follower holds 1073 — written by the first, frame-confused re-zero. Its
    seam sits at raw 1073, strictly inside the unreachable (207, 2107), and a hand
    sweep proved its travel continuous. **It is fixed.** ``arm rezero elbow_flex
    --apply`` on it must therefore write NOTHING: rewriting a working calibration
    to 1157 would spend an EEPROM write on a finite-write part to slide a seam
    from one tick the joint can never reach to another tick the joint can never
    reach. Cosmetic centring is not worth a write.
    """
    bus = FakeBus(positions={ELBOW_MOTOR: 126}, offsets={ELBOW_MOTOR: OUR_ARM_OFFSET})
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_rezero(_args(apply=True, json_mode=True))

    payload = json.loads(capsys.readouterr().out)
    assert payload["written"] is False
    assert payload["reason"] == "already-applied"
    assert payload["plan"]["already_applied"] is True
    assert payload["plan"]["current_offset"] == OUR_ARM_OFFSET
    assert payload["plan"]["current_seam_tick"] == 1073
    assert payload["plan"]["target_offset"] == OUR_ARM_OFFSET  # NOT 1157
    # It still TELLS you what a fresh re-zero would have written — it just doesn't.
    assert payload["fresh_rezero_would_write"] == EXPECTED_OFFSET
    assert payload["unreachable_arc"] == [ARC_LOW, ARC_HIGH]

    # Zero writes. Not the offset, not the lock, not torque. Nothing.
    assert bus.offset_writes == []
    assert bus.register_writes == []
    assert bus.torque_writes == []


def test_the_no_op_report_EXPLAINS_itself_in_text_mode(monkeypatch, capsys):
    """An operator who ran --apply and saw nothing happen deserves to know why."""
    bus = FakeBus(positions={ELBOW_MOTOR: 126}, offsets={ELBOW_MOTOR: OUR_ARM_OFFSET})
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_rezero(_args(apply=True))

    out = capsys.readouterr().out
    assert "already re-zeroed" in out
    assert "raw tick 1073" in out
    assert f"({ARC_LOW}, {ARC_HIGH})" in out  # ...and where that is
    assert "Nothing written" in out
    assert "1157" in out  # what a fresh re-zero WOULD write, stated, not hidden


def test_a_write_that_does_not_stick_is_an_ENV_error(monkeypatch, capsys):
    """The servo took the packet and is not holding the value — that is PR #21's ghost."""

    class _AmnesiacBus(FakeBus):
        """Accepts the offset write, then reports the factory 0 forever after."""

        def read_offset(self, motor: int) -> int:
            return 0

    bus = _AmnesiacBus(positions={ELBOW_MOTOR: 126})
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    with pytest.raises(CliError) as exc:
        arm_cmd.cmd_arm_rezero(_args(apply=True))

    assert exc.value.code == EXIT_ENV_ERROR
    assert "did NOT take" in exc.value.message
    assert "Lock register" in exc.value.remediation


def test_the_write_path_WARNS_when_the_position_does_not_move_as_predicted(monkeypatch, capsys):
    """The free, early probe: under the pessimistic firmware reading it fires at once.

    ``offset_wraps=False`` makes the servo report −947 from rest — a value the
    position register cannot hold. The write itself still succeeded (it read
    back), so this is a warning and not an error; the STOP verdict belongs to
    ``--verify``. But the operator is told, 30 seconds before the sweep could
    tell them.
    """
    bus = FakeBus(positions={ELBOW_MOTOR: 126}, offset_wraps=False)
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_rezero(_args(apply=True))

    out = capsys.readouterr().out
    assert "WARNING" in out
    assert "OUTSIDE [0, 4095]" in out
    assert "UNWRAPPED signed subtraction" in out


def test_a_servo_holding_an_UNFAMILIAR_but_EVICTING_offset_is_a_no_op_not_a_refusal(
    monkeypatch, capsys
):
    """777 is nobody's computed target — and its seam is already out of the travel.

    The old verb refused any offset that was "neither the factory 0 nor this
    joint's computed 1073", because it could not convert frames and would not
    guess. It can convert now, so the question is answerable: raw 777 is strictly
    inside (207, 2107), the joint cannot reach it, cannot cross it, and is
    therefore already linear. Nothing to do, and nothing to refuse.
    """
    bus = FakeBus(positions={ELBOW_MOTOR: 126}, offsets={ELBOW_MOTOR: 777})
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_rezero(_args(apply=True, json_mode=True))  # must NOT raise

    payload = json.loads(capsys.readouterr().out)
    assert payload["reason"] == "already-applied"
    assert payload["plan"]["current_seam_tick"] == 777
    assert bus.offset_writes == []


def test_a_servo_whose_seam_is_STILL_IN_ITS_TRAVEL_is_re_zeroed_from_that_frame(
    monkeypatch, capsys
):
    """An offset of −1096 puts the seam at raw 3000 — in the far travel. Not evicted.

    So there IS work to do, and the verb must do it from that servo's own frame
    rather than refusing because the number is unfamiliar.
    """
    bus = FakeBus(positions={ELBOW_MOTOR: 126}, offsets={ELBOW_MOTOR: -1096})
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_rezero(_args(apply=True, json_mode=True))

    payload = json.loads(capsys.readouterr().out)
    assert payload["plan"]["current_seam_tick"] == 3000
    assert payload["plan"]["raw_position"] == 126  # converted out of a NEGATIVE offset
    assert payload["read_back_offset"] == EXPECTED_OFFSET
    assert bus.offset_writes == [{"motor": ELBOW_MOTOR, "offset": EXPECTED_OFFSET}]


def test_a_joint_reporting_an_impossible_position_is_refused(monkeypatch):
    bus = FakeBus(positions={ELBOW_MOTOR: 1500})  # inside its own unreachable arc
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    with pytest.raises(CliError) as exc:
        arm_cmd.cmd_arm_rezero(_args(apply=True))

    assert exc.value.code == EXIT_ENV_ERROR
    assert "INSIDE the arc" in exc.value.message
    assert bus.offset_writes == []


# ---------------------------------------------------------------------------
# Overload recovery while PLANNING — a latched motor can still be re-zeroed
# ---------------------------------------------------------------------------
#
# ``plan_rezero`` is reads-only (``read_offset`` then ``read_position``), but a
# read is not exempt from the overload latch: ``FeetechBus._read_register``
# raises through ``_status_error``, which returns ``OverloadError`` off the
# STATUS BYTE the servo replies with — not off which direction the packet that
# tripped it was. So a motor latched in overload fails ``plan_rezero`` exactly
# as it would fail a write, and the verb used to abort before it ever reached
# ``apply_rezero`` (the only call site that clears the latch). That is not a
# corner case: ``elbow_flex``'s unreachable arc was measured by driving the
# joint into a wall, which is precisely how a Feetech servo latches an
# overload — so an operator running ``arm rezero`` right after that
# measurement, exactly the order ``docs/hardware-rezero-procedure.md``
# describes, hit this every single time.


class _ClearOverloadSpyBus(FakeBus):
    """A :class:`FakeBus` that counts calls to :meth:`clear_overload`.

    Distinguishes the pre-existing ``clear_overload`` call inside
    :func:`rezero.apply_rezero` (expected on every real write) from the NEW
    one this fix adds around ``plan_rezero`` (expected ONLY when the plan
    read actually raised ``OverloadError``). A plain write-writes count of
    ``register_writes``/``torque_writes`` cannot tell those two apart; a call
    counter on the method itself can.
    """

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.clear_overload_calls = 0

    def clear_overload(self, motor: int) -> None:
        self.clear_overload_calls += 1
        super().clear_overload(motor)


def test_a_latched_motor_can_still_be_re_zeroed(monkeypatch, capsys):
    """THE test this bug exists for.

    ``fail_with_overload_on_op(1)`` makes the very FIRST bus operation raise
    ``OverloadError`` — ``plan_rezero``'s own ``read_offset``, before this verb
    has done anything else. Without the fix this aborts the whole verb with an
    ``OverloadError`` and never reaches the write. With it: the latch is
    cleared, the plan is re-read, and the offset is written exactly as it
    would be on an un-latched motor.
    """
    bus = FakeBus(positions={ELBOW_MOTOR: 126}).fail_with_overload_on_op(1)
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_rezero(_args(apply=True, json_mode=True))

    payload = json.loads(capsys.readouterr().out)
    assert payload["read_back_offset"] == EXPECTED_OFFSET
    assert payload["applied"] is True
    assert bus.offset_writes == [{"motor": ELBOW_MOTOR, "offset": EXPECTED_OFFSET}]
    # The simulated latch was actually disarmed by clear_overload, not merely
    # dodged — a real servo's latch does not clear itself.
    assert bus.overload_after_ops is None


def test_the_operator_is_told_the_joint_went_limp_when_a_latch_is_cleared(monkeypatch, capsys):
    """A joint going limp mid-command must never be a silent side effect.

    ``clear_overload`` disables torque as its mechanism for clearing the
    latch — the operator standing at the arm needs to know THAT happened and
    WHY, on stderr, before the write proceeds.
    """
    bus = FakeBus(positions={ELBOW_MOTOR: 126}).fail_with_overload_on_op(1)
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_rezero(_args(apply=True, json_mode=True))

    err = capsys.readouterr().err
    assert "latched" in err.lower()
    assert "LIMP" in err
    assert ELBOW in err


def test_the_happy_path_issues_NO_recovery_clear_overload_call(monkeypatch):
    """Pins "only recover when actually overloaded".

    A healthy, un-latched motor must plan on the FIRST read. If the fix
    unconditionally cleared overload ahead of every plan — rather than only in
    response to a caught ``OverloadError`` — a joint that was holding fine
    would be silently de-energised for no reason connected to anything that
    went wrong. ``apply_rezero`` itself calls ``clear_overload`` once, always
    (that is pre-existing, correct behaviour, guarding the write itself) —
    this asserts the total stays at exactly that one call, i.e. the new
    recovery path around ``plan_rezero`` never fired.
    """
    bus = _ClearOverloadSpyBus(positions={ELBOW_MOTOR: 126})
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_rezero(_args(apply=True))

    assert bus.clear_overload_calls == 1  # apply_rezero's own guard, and nothing more


def test_the_already_applied_noop_path_does_not_de_energise_the_joint(monkeypatch, capsys):
    """A second run against an already-re-zeroed joint must leave it exactly alone.

    This is the path the procedure sends operators back to on purpose (see
    ``test_a_second_run_on_an_already_re_zeroed_joint_writes_nothing``), and it
    is the clearest possible case for why the overload recovery must be
    conditional: nothing here is latched, nothing is written, and NOTHING
    should touch torque — a joint mid-way through being held in position by
    an operator must not go limp because a re-zero happened to be re-run
    against it.
    """
    bus = _ClearOverloadSpyBus(positions={ELBOW_MOTOR: 126}, offsets={ELBOW_MOTOR: EXPECTED_OFFSET})
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_rezero(_args(apply=True, json_mode=True))

    payload = json.loads(capsys.readouterr().out)
    assert payload["written"] is False
    assert payload["reason"] == "already-applied"
    assert bus.clear_overload_calls == 0
    assert bus.torque_writes == []
    assert bus.register_writes == []


def test_a_retry_that_still_raises_overload_propagates_and_does_not_loop(monkeypatch):
    """The negative control: recovery is ONE retry, not a loop.

    Models a fault ``clear_overload`` cannot actually clear — every read stays
    latched no matter how many times torque is cycled. The fix must not spin
    forever against a servo that is never going to answer differently; it
    retries exactly once and then lets the second ``OverloadError`` propagate.
    """

    class _StubbornlyLatchedBus(FakeBus):
        """A servo whose overload never releases, however many times it is cleared."""

        def read_offset(self, motor: int) -> int:
            raise OverloadError(
                motor=motor,
                error_byte=32,
                message=f"FakeBus: motor {motor} refuses to un-latch.",
            )

    bus = _StubbornlyLatchedBus(positions={ELBOW_MOTOR: 126})
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    with pytest.raises(OverloadError):
        arm_cmd.cmd_arm_rezero(_args(apply=True))

    assert bus.offset_writes == []  # the write was never reached


# ---------------------------------------------------------------------------
# --verify: the seam-eviction proof, under BOTH firmware readings
# ---------------------------------------------------------------------------


def test_verify_PASSES_when_the_offset_wraps(monkeypatch, capsys):
    """The world we hope we live in — and the run that finally settles the caveat."""
    bus = _swept_bus(offset_wraps=True)
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_rezero(_args(verify=True, duration=SWEEP_SECONDS, apply=True, json_mode=True))

    sweep = json.loads(capsys.readouterr().out)["sweep"]
    assert sweep["verdict"] == rezero.VERDICT_SEAM_EVICTED
    assert sweep["seam_evicted"] is True
    assert sweep["failed"] is False
    assert sweep["continuous"] is True
    assert sweep["conclusive"] is True
    assert sweep["discontinuities"] == []
    # It also measured the far wall — a number nobody has ever had.
    assert sweep["span"] >= sweep["expected_travel"]


def test_verify_FAILS_LOUDLY_when_the_offset_does_NOT_wrap(monkeypatch, capsys):
    """THE test this whole task exists for.

    Under ``offset_wraps=False`` the firmware reports a plain signed subtraction:
    the offset only relabels positions, the seam stays pinned where it always
    was, and the re-zero achieves NOTHING. The verb must not shrug at that. It
    must fail, with a non-zero exit, in words the operator cannot misread — and
    it must still hand them the numbers, because those numbers are what the
    re-decision will be made on.
    """
    bus = _swept_bus(offset_wraps=False)
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    with pytest.raises(CliError) as exc:
        arm_cmd.cmd_arm_rezero(_args(verify=True, duration=SWEEP_SECONDS, apply=True))

    # A stop condition, and it exits non-zero so no script can read it as success.
    assert exc.value.code == EXIT_ENV_ERROR
    assert "SEAM NOT EVICTED" in exc.value.message
    assert "does NOT reduce the corrected position modulo 4096" in exc.value.message
    assert "STOP" in exc.value.remediation
    assert "re-decision" in exc.value.remediation

    # ...and the report still went to stdout, BEFORE the raise. The numbers are
    # exactly as valuable when the answer is "the fix does not work".
    out = capsys.readouterr().out
    assert "STOP — THE RE-ZERO DID NOT WORK" in out
    assert "largest jump" in out


def test_verify_failure_emits_the_full_report_as_json_before_raising(monkeypatch, capsys):
    bus = _swept_bus(offset_wraps=False)
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    with pytest.raises(CliError):
        arm_cmd.cmd_arm_rezero(
            _args(verify=True, duration=SWEEP_SECONDS, apply=True, json_mode=True)
        )

    sweep = json.loads(capsys.readouterr().out)["sweep"]
    assert sweep["verdict"] == rezero.VERDICT_SEAM_NOT_EVICTED
    assert sweep["failed"] is True
    assert sweep["out_of_range"]  # positions no register could hold


def test_verify_on_a_FACTORY_joint_is_a_BASELINE_and_exits_zero(monkeypatch, capsys):
    """Running it before the write photographs the bug — useful, and not a failure."""
    bus = _swept_bus(rezeroed=False)
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_rezero(
        _args(verify=True, duration=SWEEP_SECONDS, apply=True, json_mode=True)
    )  # must NOT raise

    sweep = json.loads(capsys.readouterr().out)["sweep"]
    assert sweep["verdict"] == rezero.VERDICT_SEAM_PRESENT_BASELINE
    assert sweep["failed"] is False
    assert sweep["rezeroed"] is False
    assert sweep["largest_jump"] > 4000  # the raw seam, in the open


def test_verify_COMMANDS_NO_MOTION_and_leaves_the_joint_LIMP(monkeypatch):
    """The verb deliberately ENDS with the joint limp — a human's hand is on it."""
    bus = _swept_bus()
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_rezero(_args(verify=True, duration=SWEEP_SECONDS, apply=True))

    assert bus.position_writes == []
    assert all(w["addr"] != ADDR_GOAL_POSITION for w in bus.register_writes)
    assert bus.torque_writes and all(w["on"] is False for w in bus.torque_writes)
    # And it wrote no EEPROM: --verify is a measurement, not a write.
    assert all(w["addr"] != ADDR_HOMING_OFFSET for w in bus.register_writes)
    assert bus.offset_writes == []


def test_verify_tells_the_operator_to_start_moving_the_joint(monkeypatch, capsys):
    """A human hand-moving a limp joint with no feedback cannot tell "the tool is
    watching me" from "the tool wedged and I am wobbling a dead arm"."""
    bus = _swept_bus()
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_rezero(_args(verify=True, duration=SWEEP_SECONDS, apply=True))

    err = capsys.readouterr().err
    assert "Torque is now OFF" in err
    assert "ENTIRE travel" in err
    assert "sample" in err  # the live position feed


def test_verify_progress_goes_to_stderr_and_never_pollutes_the_json_stdout(monkeypatch, capsys):
    """Under ``--json`` stdout must carry exactly ONE document — the report.

    A progress line interleaved into stdout would wedge a partial object between
    the reader and the result, and an agent parsing it would see neither.
    """
    bus = _swept_bus()
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_rezero(_args(verify=True, duration=SWEEP_SECONDS, apply=True, json_mode=True))

    captured = capsys.readouterr()
    json.loads(captured.out)  # exactly one document; must not raise
    assert "sample" in captured.err
    for line in captured.err.strip().splitlines():
        if line.startswith("{"):
            json.loads(line)  # structured diagnostics stay structured


def test_a_duration_too_short_to_have_a_delta_is_a_user_error(monkeypatch):
    """Caught BEFORE the consent prompt — do not make the operator say yes to nothing."""
    bus = _swept_bus()
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["yes\n"], tty=True))

    with pytest.raises(CliError) as exc:
        arm_cmd.cmd_arm_rezero(_args(verify=True, duration=0.01, apply=True))

    assert exc.value.code == EXIT_USER_ERROR
    assert bus.register_writes == []  # not even the torque-off


def test_verify_declined_at_the_prompt_does_nothing(monkeypatch, capsys):
    bus = _swept_bus()
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["no\n"], tty=True))

    arm_cmd.cmd_arm_rezero(_args(verify=True, duration=SWEEP_SECONDS))

    assert bus.register_writes == []
    assert "Aborted" in capsys.readouterr().out


# ---------------------------------------------------------------------------
# Torque ownership + failure hygiene
# ---------------------------------------------------------------------------


def test_a_bus_failure_mid_sweep_releases_torque_and_raises_a_structured_error(monkeypatch, capsys):
    """The joint must never be left hot by a crash — and no traceback may leak."""

    class _DyingBus(HandMovedBus):
        def read_position(self, motor: int) -> int:
            if len(self.torque_writes) and self._op_count > 6:
                raise CliError(
                    code=EXIT_ENV_ERROR,
                    message="simulated serial failure mid-sweep",
                    remediation="retry",
                )
            return super().read_position(motor)

    bus = _DyingBus(ticks_per_read=SWEEP_STEP, offsets={ELBOW_MOTOR: EXPECTED_OFFSET})
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    with pytest.raises(CliError) as exc:
        arm_cmd.cmd_arm_rezero(_args(verify=True, duration=SWEEP_SECONDS, apply=True))

    assert exc.value.code == EXIT_ENV_ERROR
    assert "simulated serial failure" in exc.value.message
    # The torque guard fired and announced it on stderr.
    assert "Torque released" in capsys.readouterr().err
    assert all(w["on"] is False for w in bus.torque_writes)


def test_the_torque_guard_owns_only_the_joint_being_re_zeroed(monkeypatch, capsys):
    """One joint is touched, so one joint is claimed — no crying wolf over five others."""
    owned: "list[tuple[int, ...]]" = []
    real_guard = arm_cmd.torque_guard

    def _spy(bus, motors=(), **kwargs):
        owned.append(tuple(motors))
        return real_guard(bus, motors, **kwargs)

    monkeypatch.setattr(arm_cmd, "torque_guard", _spy)
    bus = FakeBus(positions={ELBOW_MOTOR: 126})
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_rezero(_args(apply=True))

    assert owned == [(ELBOW_MOTOR,)]


def test_no_port_is_an_env_error_not_a_traceback(monkeypatch):
    monkeypatch.setattr(arm_cmd, "_candidate_ports", lambda: [])
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    with pytest.raises(CliError) as exc:
        arm_cmd.cmd_arm_rezero(_args(apply=True))

    assert exc.value.code == EXIT_ENV_ERROR


def test_the_leader_role_resolves_the_same_motor_id(monkeypatch, capsys):
    """Both roles carry elbow_flex on id 3 (LeRobot: follower == leader for ids)."""
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))
    monkeypatch.setattr(arm_cmd, "_candidate_ports", lambda: ["/dev/ttyACM_fake"])

    arm_cmd.cmd_arm_rezero(_args(role="leader", json_mode=True))

    plan = json.loads(capsys.readouterr().out)["plan"]
    assert plan["role"] == "leader"
    assert plan["motor"] == ELBOW_MOTOR


# ---------------------------------------------------------------------------
# Catalog lockstep
# ---------------------------------------------------------------------------


def test_explain_arm_rezero_resolves_and_names_the_stop_condition():
    from arm101.explain.catalog import ENTRIES

    entry = ENTRIES[("arm", "rezero")]
    assert "seam-not-evicted" in entry
    assert "wrist_roll" in entry
    assert "commands no motion" in entry.lower()
    assert "bootstrap problem" in entry.lower()


def test_learn_and_overview_both_mention_rezero():
    from arm101.cli._commands.learn import _TEXT, _as_json_payload
    from arm101.cli._commands.overview import _VERBS

    assert "arm rezero" in _TEXT
    assert any("rezero" in v for v in _VERBS)
    payload = _as_json_payload()
    assert ["arm", "rezero"] in [c["path"] for c in payload["commands"]]
    assert "arm rezero" in payload["hardware"]["verbs"]
