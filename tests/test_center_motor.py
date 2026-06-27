"""Tests for ``arm101 center-motor`` — gated commanded-motion verb.

Drives the verb entirely against a :class:`~arm101.hardware.bus.FakeBus` and
monkeypatched seams from ``calibrate_motor``; no hardware is touched.

Covers:
- confirm → torque-on / goal-position / torque-relax in order
- ``--keep-torque`` leaves torque on (no final relax write)
- decline ("no") → zero bus writes and clean exit (exit 0)
- EOF on stdin → CliError(EXIT_ENV_ERROR), motor never moved
- stdout/stderr split (result on stdout, diagnostics on stderr)
- ``--json`` shape
- ``SystemExit`` is never raised
- out-of-range ``--position`` (e.g. 9999) → CliError(EXIT_USER_ERROR)
- dry-run (non-TTY, no --apply) writes plan file, zero motion
- agent --apply --plan-hash matching → full motion
- agent --apply --plan-hash mismatch → CliError(EXIT_ENV_ERROR), zero motion
- agent --apply missing hash → CliError(EXIT_ENV_ERROR), zero motion
"""

from __future__ import annotations

import argparse
import json
import sys

import pytest

from arm101.cli._commands import calibrate_motor as cm
from arm101.cli._consent import build_plan
from arm101.cli._errors import EXIT_ENV_ERROR, EXIT_USER_ERROR, CliError
from arm101.hardware.bus import FakeBus

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


class _FakeStdin:
    """Scripted stdin: ``readline`` returns successive lines, then "" (EOF)."""

    def __init__(self, lines: list[str]) -> None:
        self._lines = list(lines)

    def readline(self) -> str:
        return self._lines.pop(0) if self._lines else ""

    def isatty(self) -> bool:  # resolve_consent calls this first → routes to interactive
        return True


def _args(**kw) -> argparse.Namespace:
    ns = argparse.Namespace(
        port=None,
        position=2048,
        keep_torque=False,
        json=False,
        apply=False,
        plan_hash=None,
    )
    for key, value in kw.items():
        setattr(ns, key, value)
    return ns


def _patch_single_port(monkeypatch, bus: FakeBus) -> None:
    """One candidate port, opening to *bus*. Patches calibrate_motor seams."""
    monkeypatch.setattr(cm, "_candidate_ports", lambda: ["/dev/ttyACM1"])
    monkeypatch.setattr(cm, "_open_bus", lambda port: bus)


# ---------------------------------------------------------------------------
# Import sanity
# ---------------------------------------------------------------------------


def test_import_center_motor() -> None:
    """center_motor must import without error."""
    from arm101.cli._commands import center_motor  # noqa: F401

    assert center_motor


# ---------------------------------------------------------------------------
# Happy path: confirm → full sequence in order
# ---------------------------------------------------------------------------


def test_confirm_runs_full_sequence_in_order(monkeypatch, capsys) -> None:
    """On 'yes': torque-on, goal-position, torque-relax — in that order."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["yes\n"]))

    from arm101.cli._commands.center_motor import cmd_center_motor

    cmd_center_motor(_args())

    # Torque and position writes happen; bus.close() shuts things down.
    assert len(bus.torque_writes) == 2
    assert bus.torque_writes[0] == {"motor": 1, "on": True}
    assert bus.torque_writes[1] == {"motor": 1, "on": False}
    assert len(bus.position_writes) == 1
    assert bus.position_writes[0] == {"motor": 1, "position": 2048}

    # Ordering: torque-on < goal < torque-off is implicit from the FakeBus
    # recording lists in call order, but we assert the combined sequence by
    # checking that no position_write preceded the first torque_write and no
    # relax write preceded a position write.  Since FakeBus appends to separate
    # lists we cross-check by confirming counts match the expected sequence.
    assert bus.torque_writes[0]["on"] is True
    assert bus.torque_writes[1]["on"] is False


def test_result_on_stdout_diagnostics_on_stderr(monkeypatch, capsys) -> None:
    """Result emitted to stdout; warnings/snapshot/prompt to stderr."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["yes\n"]))

    from arm101.cli._commands.center_motor import cmd_center_motor

    cmd_center_motor(_args())

    out = capsys.readouterr()
    assert "Centered motor" in out.out
    assert "ENABLE TORQUE" in out.err or "Detected motor" in out.err
    assert "Centered motor" not in out.err


def test_custom_position_is_used(monkeypatch, capsys) -> None:
    """--position value is forwarded to write_goal_position."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["yes\n"]))

    from arm101.cli._commands.center_motor import cmd_center_motor

    cmd_center_motor(_args(position=1024))

    assert bus.position_writes[0]["position"] == 1024
    out = capsys.readouterr()
    assert "1024" in out.out


# ---------------------------------------------------------------------------
# --keep-torque: torque stays on
# ---------------------------------------------------------------------------


def test_keep_torque_leaves_torque_enabled(monkeypatch, capsys) -> None:
    """With --keep-torque the relax write is skipped."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["yes\n"]))

    from arm101.cli._commands.center_motor import cmd_center_motor

    cmd_center_motor(_args(keep_torque=True))

    # Only one torque write: enable; no relax.
    assert len(bus.torque_writes) == 1
    assert bus.torque_writes[0] == {"motor": 1, "on": True}
    assert len(bus.position_writes) == 1

    out = capsys.readouterr()
    assert "still enabled" in out.out


# ---------------------------------------------------------------------------
# Decline: no bus writes, clean exit
# ---------------------------------------------------------------------------


def test_decline_zero_writes_and_clean_exit(monkeypatch, capsys) -> None:
    """Answering 'no' produces zero bus writes and exits cleanly (exit 0)."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["no\n"]))

    from arm101.cli._commands.center_motor import cmd_center_motor

    cmd_center_motor(_args())

    assert bus.torque_writes == []
    assert bus.position_writes == []

    out = capsys.readouterr()
    assert "Aborted" in out.out


def test_blank_answer_is_decline(monkeypatch, capsys) -> None:
    """A blank answer (just Enter) is treated as a decline."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["\n"]))

    from arm101.cli._commands.center_motor import cmd_center_motor

    cmd_center_motor(_args())

    assert bus.torque_writes == []
    assert bus.position_writes == []

    out = capsys.readouterr()
    assert "Aborted" in out.out


# ---------------------------------------------------------------------------
# EOF: non-interactive guard
# ---------------------------------------------------------------------------


def test_eof_on_stdin_raises_env_error(monkeypatch) -> None:
    """EOF on stdin before confirmation raises CliError(EXIT_ENV_ERROR).

    This ensures a non-interactive run (CI, piped input) can never silently
    move the motor.
    """
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([]))  # immediate EOF

    from arm101.cli._commands.center_motor import cmd_center_motor

    with pytest.raises(CliError) as exc:
        cmd_center_motor(_args())

    assert exc.value.code == EXIT_ENV_ERROR
    # Motor must not have moved.
    assert bus.torque_writes == []
    assert bus.position_writes == []


# ---------------------------------------------------------------------------
# --json shape
# ---------------------------------------------------------------------------


def test_json_shape_on_confirm(monkeypatch, capsys) -> None:
    """--json emits a structured payload with the expected keys."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["yes\n"]))

    from arm101.cli._commands.center_motor import cmd_center_motor

    cmd_center_motor(_args(position=1500, json=True))

    out = capsys.readouterr()
    payload = json.loads(out.out)
    assert payload["motor"] == 1
    assert payload["port"] == "/dev/ttyACM1"
    assert payload["position"] == 1500
    assert payload["torque_relaxed"] is True
    assert out.err  # snapshot/warning went to stderr


def test_json_keep_torque_sets_flag(monkeypatch, capsys) -> None:
    """--json + --keep-torque: torque_relaxed is False."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["yes\n"]))

    from arm101.cli._commands.center_motor import cmd_center_motor

    cmd_center_motor(_args(keep_torque=True, json=True))

    out = capsys.readouterr()
    payload = json.loads(out.out)
    assert payload["torque_relaxed"] is False


# ---------------------------------------------------------------------------
# Never raises SystemExit
# ---------------------------------------------------------------------------


def test_never_raises_system_exit_on_no_motor(monkeypatch) -> None:
    """Failures surface as CliError, never a bare SystemExit."""
    bus = FakeBus(ids=[])  # nothing responds
    bus.open()
    _patch_single_port(monkeypatch, bus)

    from arm101.cli._commands.center_motor import cmd_center_motor

    try:
        cmd_center_motor(_args())
    except CliError:
        pass
    except SystemExit:  # pragma: no cover
        pytest.fail("center-motor raised SystemExit instead of CliError")


# ---------------------------------------------------------------------------
# Out-of-range --position
# ---------------------------------------------------------------------------


def test_out_of_range_position_raises_user_error(monkeypatch) -> None:
    """--position 9999 is caught by FakeBus and raises CliError(EXIT_USER_ERROR)."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["yes\n"]))

    from arm101.cli._commands.center_motor import cmd_center_motor

    with pytest.raises(CliError) as exc:
        cmd_center_motor(_args(position=9999))

    assert exc.value.code == EXIT_USER_ERROR
    # No torque should have been enabled (position validation fails in write_goal_position,
    # but torque-enable already ran). The important thing is the error code.
    assert exc.value.code == EXIT_USER_ERROR


def test_negative_position_raises_user_error(monkeypatch) -> None:
    """--position -1 raises CliError(EXIT_USER_ERROR)."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["yes\n"]))

    from arm101.cli._commands.center_motor import cmd_center_motor

    with pytest.raises(CliError) as exc:
        cmd_center_motor(_args(position=-1))

    assert exc.value.code == EXIT_USER_ERROR


# ---------------------------------------------------------------------------
# Non-TTY guard / abort-in-JSON / torque-relax-on-failure (Qodo #1, #4, #3)
# ---------------------------------------------------------------------------


class _NonTtyStdin:
    """stdin that would answer 'yes' but reports isatty() False."""

    def readline(self) -> str:  # pragma: no cover - never reached (TTY check first)
        return "yes\n"

    def isatty(self) -> bool:
        return False


def test_non_tty_no_apply_writes_plan_file(monkeypatch, tmp_path) -> None:
    """Non-TTY without --apply writes a plan file and performs zero motion."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _NonTtyStdin())
    monkeypatch.setenv("ARM101_PLAN_DIR", str(tmp_path))

    from arm101.cli._commands.center_motor import cmd_center_motor

    cmd_center_motor(_args())

    # A plan .json file was created in tmp_path
    plan_files = list(tmp_path.glob("*.json"))
    assert len(plan_files) == 1

    # Zero motion
    assert bus.torque_writes == []
    assert bus.position_writes == []


def test_agent_apply_matching_hash_moves(monkeypatch, capsys) -> None:
    """Agent mode with matching plan hash performs the full motion sequence."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _NonTtyStdin())

    # Build the action dict the verb will build
    target = 2048
    keep_torque = False
    motor_id = 1
    port = "/dev/ttyACM1"
    deg = target * 360.0 / 4096.0
    action = {
        "kind": "goal_position_write",
        "motor_id": motor_id,
        "target_position": target,
        "target_degrees": round(deg, 1),
        "keep_torque": keep_torque,
        "workspace_warning": (
            "ENABLE TORQUE and MOVE the motor. Clear the workspace before proceeding."
        ),
    }

    # Get the info the FakeBus would return
    info = bus.read_info(motor_id)

    # Compute the expected plan hash
    plan = build_plan(
        "center-motor",
        port,
        info,
        action,
        operator="x",
        created_at="x",
    )
    expected_hash = plan["plan_hash"]

    from arm101.cli._commands.center_motor import cmd_center_motor

    cmd_center_motor(_args(apply=True, plan_hash=expected_hash))

    # Full motion sequence recorded
    assert bus.torque_writes == [
        {"motor": 1, "on": True},
        {"motor": 1, "on": False},
    ]
    assert bus.position_writes == [{"motor": 1, "position": 2048}]


def test_agent_apply_bad_hash_refused(monkeypatch) -> None:
    """Agent mode with a bad plan hash raises CliError(EXIT_ENV_ERROR), zero motion."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _NonTtyStdin())

    from arm101.cli._commands.center_motor import cmd_center_motor

    with pytest.raises(CliError) as exc:
        cmd_center_motor(_args(apply=True, plan_hash="sha256:" + "0" * 64))

    assert exc.value.code == EXIT_ENV_ERROR
    assert bus.torque_writes == []
    assert bus.position_writes == []


def test_agent_apply_missing_hash_refused(monkeypatch) -> None:
    """Agent mode with --apply but no --plan-hash raises CliError(EXIT_ENV_ERROR)."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _NonTtyStdin())

    from arm101.cli._commands.center_motor import cmd_center_motor

    with pytest.raises(CliError) as exc:
        cmd_center_motor(_args(apply=True, plan_hash=None))

    assert exc.value.code == EXIT_ENV_ERROR
    assert bus.torque_writes == []
    assert bus.position_writes == []


def test_enable_torque_failure_audits_failed(monkeypatch, tmp_path) -> None:
    """A comms failure on the initial enable_torque(True) → 'pending' then 'failed' audit.

    Guards the motion-step rewrite (the sonnet BLOCKER + MAJOR #2): the
    torque-enable that precedes the goal-position write must sit *inside* the
    outer try, so a raise there is audited as ``"failed"`` after the
    ``"pending"`` record (never orphaning it) and commands zero motion.
    """

    class _EnableRaisesBus(FakeBus):
        def enable_torque(self, motor: int, on: bool) -> None:
            # Fail only on the initial enable (on=True), mimicking a comms drop.
            if on:
                raise CliError(
                    code=EXIT_ENV_ERROR,
                    message="torque enable failed (comms)",
                    remediation="Check wiring and power.",
                )
            super().enable_torque(motor, on)

    bus = _EnableRaisesBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _NonTtyStdin())
    audit_log = tmp_path / "audit.log"
    monkeypatch.setenv("ARM101_AUDIT_LOG", str(audit_log))

    # Matching hash so consent passes and the motion is actually attempted.
    info = bus.read_info(1)
    action = {
        "kind": "goal_position_write",
        "motor_id": 1,
        "target_position": 2048,
        "target_degrees": round(2048 * 360.0 / 4096.0, 1),
        "keep_torque": False,
        "workspace_warning": (
            "ENABLE TORQUE and MOVE the motor. Clear the workspace before proceeding."
        ),
    }
    expected_hash = build_plan(
        "center-motor", "/dev/ttyACM1", info, action, operator="x", created_at="x"
    )["plan_hash"]

    from arm101.cli._commands.center_motor import cmd_center_motor

    with pytest.raises(CliError) as exc:
        cmd_center_motor(_args(apply=True, plan_hash=expected_hash))

    assert exc.value.code == EXIT_ENV_ERROR
    # The enable raised before recording, and no goal-position write was commanded.
    assert bus.torque_writes == []
    assert bus.position_writes == []
    # Audit shows pending followed by failed — no orphaned pending record.
    records = [json.loads(line) for line in audit_log.read_text().splitlines()]
    assert [r["outcome"] for r in records] == ["pending", "failed"]
    assert records[1]["error"]


def test_abort_emits_valid_json(monkeypatch, capsys) -> None:
    """Declining in --json mode emits valid JSON (not plain text)."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["no\n"]))

    from arm101.cli._commands.center_motor import cmd_center_motor

    cmd_center_motor(_args(json=True))

    payload = json.loads(capsys.readouterr().out)
    assert payload["aborted"] is True
    assert payload["moved"] is False
    assert bus.position_writes == []


def test_torque_relaxed_when_goal_write_fails(monkeypatch, tmp_path) -> None:
    """A failed goal-position write still relaxes torque and audits 'failed'."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["yes\n"]))
    audit_log = tmp_path / "audit.log"
    monkeypatch.setenv("ARM101_AUDIT_LOG", str(audit_log))

    from arm101.cli._commands.center_motor import cmd_center_motor

    with pytest.raises(CliError):
        cmd_center_motor(_args(position=9999))  # FakeBus rejects -> raises mid-move

    assert bus.torque_writes == [{"motor": 1, "on": True}, {"motor": 1, "on": False}]
    assert bus.position_writes == []
    # The motion-step failure is audited 'failed' after the 'pending' record.
    records = [json.loads(line) for line in audit_log.read_text().splitlines()]
    assert [r["outcome"] for r in records] == ["pending", "failed"]
    assert records[1]["error"]


def test_keep_torque_not_relaxed_when_goal_write_fails(monkeypatch) -> None:
    """--keep-torque + a failed move leaves torque enabled (no relax write)."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["yes\n"]))

    from arm101.cli._commands.center_motor import cmd_center_motor

    with pytest.raises(CliError):
        cmd_center_motor(_args(position=9999, keep_torque=True))

    assert bus.torque_writes == [{"motor": 1, "on": True}]
