"""Tests for ``arm101 setup-motors`` — per-motor EEPROM id/baudrate assignment.

Covers three consent modes:
- dry_run (non-TTY, no --apply): emits plan, zero writes, includes baudrate
- interactive (TTY): per-motor detect + before/after card + Enter gate + audit
- agent (non-TTY + --apply): headless walk with per-motor detection + audit

New in this revision:
- Per-motor re-detection via calibrate_motor._open_bus / _candidate_ports seams
- --baudrate flag: default 1M, validated against BAUD_MAP, plumbed into write
- before/after motor cards emitted to stderr
- --current-id is now a safety assertion: if given, detected id must match

Seam strategy:
  - ``calibrate_motor._candidate_ports`` -> returns ``["/dev/ttyACM_fake"]``
  - ``calibrate_motor._open_bus`` -> factory that re-opens and returns the shared
    FakeBus (so eeprom_writes accumulate across all 6 motors)
  - ``setup_motors._open_bus_after`` -> returns a secondary FakeBus for the
    after-read when baudrate != 1_000_000

Stdin seam: ``sys.stdin`` is replaced with a fake object exposing
``.isatty()`` and ``.readline()`` so a single ``monkeypatch.setattr`` controls
both TTY detection and line input.
"""

from __future__ import annotations

import argparse
import json
import sys

import pytest

from arm101.cli._commands import calibrate_motor as cm
from arm101.cli._commands import setup_motors
from arm101.cli._errors import EXIT_ENV_ERROR, EXIT_USER_ERROR, CliError
from arm101.hardware.bus import FakeBus

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_BAUDRATE = 1_000_000

_MOTORS = [
    (6, "gripper"),
    (5, "wrist_roll"),
    (4, "wrist_flex"),
    (3, "elbow_flex"),
    (2, "shoulder_lift"),
    (1, "shoulder_pan"),
]


def _make_args(
    json_mode: bool = False,
    port: "str | None" = None,
    current_id: "int | None" = None,
    apply: bool = False,
    baudrate: int = _BAUDRATE,
) -> argparse.Namespace:
    """Return a minimal Namespace as if parsed by argparse."""
    return argparse.Namespace(
        json=json_mode,
        port=port,
        current_id=current_id,
        apply=apply,
        baudrate=baudrate,
    )


class _FakeStdin:
    """Fake stdin that controls both isatty() and readline() independently."""

    def __init__(self, lines: "list[str]", tty: bool = True) -> None:
        self._lines = iter(lines)
        self._tty = tty

    def isatty(self) -> bool:
        return self._tty

    def readline(self) -> str:
        try:
            return next(self._lines)
        except StopIteration:
            return ""  # EOF


def _patch_detection(monkeypatch, fake: FakeBus) -> None:
    """Patch calibrate_motor seams so _detect_one_motor returns *fake*.

    The factory re-opens *fake* on each call so it is still ``_open`` after
    the previous motor's ``bus.close()`` call set it to False.
    """
    monkeypatch.setattr(cm, "_candidate_ports", lambda: ["/dev/ttyACM_fake"])

    def _open(port: str) -> FakeBus:
        fake.open()  # re-open; close() in the walk sets _open=False
        return fake

    monkeypatch.setattr(cm, "_open_bus", _open)


# ---------------------------------------------------------------------------
# dry_run mode: non-TTY, no --apply -> plan only, zero writes
# ---------------------------------------------------------------------------


def test_dry_run_text_output(monkeypatch, capsys):
    """Non-TTY without --apply emits the full 6-joint plan in text."""
    fake = FakeBus(ids=[1])
    fake.open()
    _patch_detection(monkeypatch, fake)

    fake_stdin = _FakeStdin([], tty=False)
    monkeypatch.setattr(sys, "stdin", fake_stdin)

    setup_motors.cmd_setup_motors(_make_args())

    captured = capsys.readouterr()

    # All six joint names must appear in stdout
    for _, joint in _MOTORS:
        assert joint in captured.out, f"Joint '{joint}' missing from dry-run text"

    # Must mention dry-run plan
    assert "Dry-run plan" in captured.out

    # Zero writes
    assert fake.eeprom_writes == []


def test_dry_run_json_output(monkeypatch, capsys):
    """Non-TTY without --apply and --json emits structured plan."""
    fake = FakeBus(ids=[1])
    fake.open()
    _patch_detection(monkeypatch, fake)

    fake_stdin = _FakeStdin([], tty=False)
    monkeypatch.setattr(sys, "stdin", fake_stdin)

    setup_motors.cmd_setup_motors(_make_args(json_mode=True))

    captured = capsys.readouterr()

    payload = json.loads(captured.out)
    assert "plan" in payload
    plan = payload["plan"]
    assert len(plan) == 6

    for i, (motor_id, joint_name) in enumerate(_MOTORS):
        entry = plan[i]
        assert entry["joint"] == joint_name
        assert entry["from_id"] == 1
        assert entry["new_id"] == motor_id
        assert entry["baudrate"] == _BAUDRATE

    # Zero writes
    assert fake.eeprom_writes == []


def test_dry_run_no_bus_opened(monkeypatch):
    """dry_run mode must not open the bus at all."""
    bus_opened = [False]

    def fake_open(port):
        bus_opened[0] = True
        b = FakeBus(ids=[1])
        b.open()
        return b

    monkeypatch.setattr(cm, "_open_bus", fake_open)
    monkeypatch.setattr(cm, "_candidate_ports", lambda: ["/dev/ttyACM_fake"])

    fake_stdin = _FakeStdin([], tty=False)
    monkeypatch.setattr(sys, "stdin", fake_stdin)

    setup_motors.cmd_setup_motors(_make_args())

    assert bus_opened[0] is False, "dry_run must not open the bus"


def test_dry_run_includes_baudrate(monkeypatch, capsys):
    """Dry-run plan reflects the --baudrate flag."""
    fake = FakeBus(ids=[1])
    fake.open()
    _patch_detection(monkeypatch, fake)

    fake_stdin = _FakeStdin([], tty=False)
    monkeypatch.setattr(sys, "stdin", fake_stdin)

    setup_motors.cmd_setup_motors(_make_args(json_mode=True, baudrate=500_000))

    captured = capsys.readouterr()
    payload = json.loads(captured.out)
    for entry in payload["plan"]:
        assert entry["baudrate"] == 500_000


# ---------------------------------------------------------------------------
# interactive mode: TTY, per-motor detect + before/after cards + Enter gate
# ---------------------------------------------------------------------------


def test_full_walk_writes_in_order(monkeypatch, capsys):
    """All 6 motors are assigned in gripper->shoulder_pan order."""
    fake = FakeBus(ids=[1])
    fake.open()
    _patch_detection(monkeypatch, fake)

    # 6 Enters, one per motor
    fake_stdin = _FakeStdin(["\n"] * 6, tty=True)
    monkeypatch.setattr(sys, "stdin", fake_stdin)

    setup_motors.cmd_setup_motors(_make_args())

    assert len(fake.eeprom_writes) == 6
    expected_order = [motor_id for motor_id, _ in _MOTORS]
    for i, (expected_id, write) in enumerate(zip(expected_order, fake.eeprom_writes)):
        assert write["motor"] == 1, f"Write {i}: should address the auto-detected id (1)"
        assert write["new_id"] == expected_id, f"Write {i}: new_id mismatch"
        assert write["baudrate"] == _BAUDRATE, f"Write {i}: baudrate mismatch"


def test_detection_called_per_motor(monkeypatch, capsys):
    """_open_bus is called once per motor (6 times), not once globally."""
    call_count = [0]
    fake = FakeBus(ids=[1])

    def _open(port: str) -> FakeBus:
        call_count[0] += 1
        fake.open()
        return fake

    monkeypatch.setattr(cm, "_candidate_ports", lambda: ["/dev/ttyACM_fake"])
    monkeypatch.setattr(cm, "_open_bus", _open)

    fake_stdin = _FakeStdin(["\n"] * 6, tty=True)
    monkeypatch.setattr(sys, "stdin", fake_stdin)

    setup_motors.cmd_setup_motors(_make_args())

    assert call_count[0] == 6, f"Expected 6 bus opens (one per motor), got {call_count[0]}"


def test_before_and_after_cards_emitted_to_stderr(monkeypatch, capsys):
    """Before and after motor cards appear on stderr (not stdout)."""
    fake = FakeBus(ids=[1])
    fake.open()
    _patch_detection(monkeypatch, fake)

    fake_stdin = _FakeStdin(["\n"] * 6, tty=True)
    monkeypatch.setattr(sys, "stdin", fake_stdin)

    setup_motors.cmd_setup_motors(_make_args())

    captured = capsys.readouterr()
    # The card header is "Detected motor"
    assert "Detected motor" in captured.err, "Before/after card missing from stderr"
    # Baudrate in bps appears in card
    assert "1,000,000 bps" in captured.err, "Baudrate line missing from card"
    # Summary goes to stdout
    assert "Motors assigned" in captured.out


def test_prompt_omits_false_current_id_when_unasserted(monkeypatch, capsys):
    """With --current-id omitted, the prompt must not claim "currently at id 1".

    Under auto-detect semantics the connected id is unknown until detection, so
    asserting a specific id (the old `_FACTORY_DEFAULT_ID` text) misled the
    operator. The "currently at id" phrase only appears when an id is asserted.
    """
    fake = FakeBus(ids=[1])
    fake.open()
    _patch_detection(monkeypatch, fake)

    fake_stdin = _FakeStdin(["\n"] * 6, tty=True)
    monkeypatch.setattr(sys, "stdin", fake_stdin)

    setup_motors.cmd_setup_motors(_make_args(current_id=None))

    err = capsys.readouterr().err
    assert "currently at id" not in err  # no false id claim when unasserted
    assert "connect the gripper motor ONLY" in err  # guidance still present


def test_prompt_shows_current_id_when_asserted(monkeypatch, capsys):
    """With --current-id given, the prompt names that id ("currently at id N")."""
    fake = FakeBus(ids=[1])
    fake.open()
    _patch_detection(monkeypatch, fake)

    fake_stdin = _FakeStdin(["\n"] * 6, tty=True)
    monkeypatch.setattr(sys, "stdin", fake_stdin)

    setup_motors.cmd_setup_motors(_make_args(current_id=1))

    err = capsys.readouterr().err
    assert "currently at id 1" in err


def test_partial_walk_stops_at_stdin_eof(monkeypatch, capsys):
    """EOF mid-walk raises CliError(EXIT_ENV_ERROR); exactly the gated writes happen."""
    fake = FakeBus(ids=[1])
    fake.open()
    _patch_detection(monkeypatch, fake)

    # Only 2 Enters -> motors 6 and 5 are written, then EOF on the 3rd prompt.
    fake_stdin = _FakeStdin(["\n", "\n"], tty=True)
    monkeypatch.setattr(sys, "stdin", fake_stdin)

    with pytest.raises(CliError) as exc_info:
        setup_motors.cmd_setup_motors(_make_args())

    assert exc_info.value.code == EXIT_ENV_ERROR
    # Exactly the 2 gated writes occurred -- the EOF aborts before any further write.
    assert len(fake.eeprom_writes) == 2
    for i, write in enumerate(fake.eeprom_writes):
        assert write["motor"] == 1
        assert write["new_id"] == _MOTORS[i][0]


def test_stdout_stderr_split(monkeypatch, capsys):
    """Prompts land on stderr; result summary lands on stdout."""
    fake = FakeBus(ids=[1])
    fake.open()
    _patch_detection(monkeypatch, fake)

    fake_stdin = _FakeStdin(["\n"] * 6, tty=True)
    monkeypatch.setattr(sys, "stdin", fake_stdin)

    setup_motors.cmd_setup_motors(_make_args(json_mode=False))

    captured = capsys.readouterr()

    for _, joint in _MOTORS:
        assert joint in captured.err, f"Prompt for '{joint}' missing from stderr"

    assert captured.out.strip(), "stdout must have non-empty result"
    assert "press Enter" not in captured.out, "Prompts must not appear on stdout"


def test_json_output_shape(monkeypatch, capsys):
    """--json emits {assigned: [{joint, from_id, new_id, baudrate}, ...]}."""
    fake = FakeBus(ids=[1])
    fake.open()
    _patch_detection(monkeypatch, fake)

    fake_stdin = _FakeStdin(["\n"] * 6, tty=True)
    monkeypatch.setattr(sys, "stdin", fake_stdin)

    setup_motors.cmd_setup_motors(_make_args(json_mode=True))

    captured = capsys.readouterr()

    payload = json.loads(captured.out)
    assert "assigned" in payload
    assigned = payload["assigned"]
    assert len(assigned) == 6

    for i, (motor_id, joint_name) in enumerate(_MOTORS):
        entry = assigned[i]
        assert entry["joint"] == joint_name
        assert entry["from_id"] == 1
        assert entry["new_id"] == motor_id
        assert entry["baudrate"] == _BAUDRATE

    for _, joint in _MOTORS:
        assert joint in captured.err


# ---------------------------------------------------------------------------
# Audit: pending -> success per write
# ---------------------------------------------------------------------------


def test_audit_pending_success_per_motor(monkeypatch, tmp_path):
    """Each motor write produces a pending->success audit pair."""
    audit_log = tmp_path / "audit.log"
    monkeypatch.setenv("ARM101_AUDIT_LOG", str(audit_log))

    fake = FakeBus(ids=[1])
    fake.open()
    _patch_detection(monkeypatch, fake)

    fake_stdin = _FakeStdin(["\n"] * 6, tty=True)
    monkeypatch.setattr(sys, "stdin", fake_stdin)

    setup_motors.cmd_setup_motors(_make_args())

    lines = audit_log.read_text().strip().splitlines()
    records = [json.loads(line) for line in lines]

    # 6 motors x 2 records (pending + success) = 12
    assert len(records) == 12

    for motor_id, joint_name in _MOTORS:
        pending = [
            r
            for r in records
            if r["outcome"] == "pending"
            and r["action"]["to_id"] == motor_id
            and r["action"]["joint"] == joint_name
        ]
        success = [
            r
            for r in records
            if r["outcome"] == "success"
            and r["action"]["to_id"] == motor_id
            and r["action"]["joint"] == joint_name
        ]
        assert len(pending) == 1, f"Expected 1 pending for {joint_name}"
        assert len(success) == 1, f"Expected 1 success for {joint_name}"
        # consent_mode and operator present
        assert pending[0]["consent_mode"] == "interactive"
        assert "operator" in pending[0]
        # baudrate in audit
        assert pending[0]["action"]["baudrate"] == _BAUDRATE


# ---------------------------------------------------------------------------
# agent mode: non-TTY + --apply -> headless walk
# ---------------------------------------------------------------------------


def test_agent_apply_writes_all_motors(monkeypatch):
    """Non-TTY + --apply drives the full 6->1 walk without readline."""
    fake = FakeBus(ids=[1])
    fake.open()
    _patch_detection(monkeypatch, fake)

    fake_stdin = _FakeStdin([], tty=False)
    monkeypatch.setattr(sys, "stdin", fake_stdin)

    setup_motors.cmd_setup_motors(_make_args(apply=True))

    assert len(fake.eeprom_writes) == 6
    for i, (motor_id, _) in enumerate(_MOTORS):
        assert fake.eeprom_writes[i]["motor"] == 1
        assert fake.eeprom_writes[i]["new_id"] == motor_id
        assert fake.eeprom_writes[i]["baudrate"] == _BAUDRATE


def test_agent_apply_audit(monkeypatch, tmp_path):
    """Agent mode writes audit with consent_mode=agent."""
    audit_log = tmp_path / "audit.log"
    monkeypatch.setenv("ARM101_AUDIT_LOG", str(audit_log))

    fake = FakeBus(ids=[1])
    fake.open()
    _patch_detection(monkeypatch, fake)

    fake_stdin = _FakeStdin([], tty=False)
    monkeypatch.setattr(sys, "stdin", fake_stdin)

    setup_motors.cmd_setup_motors(_make_args(apply=True))

    lines = audit_log.read_text().strip().splitlines()
    records = [json.loads(line) for line in lines]

    # 6 motors x 2 records = 12
    assert len(records) == 12

    for r in records:
        assert r["consent_mode"] == "agent"
        assert "operator" in r

    # Check pending->success pairs
    pending = [r for r in records if r["outcome"] == "pending"]
    success = [r for r in records if r["outcome"] == "success"]
    assert len(pending) == 6
    assert len(success) == 6


def test_agent_apply_no_refusal(monkeypatch):
    """Non-TTY + --apply must NOT raise CliError (no hard refusal)."""
    fake = FakeBus(ids=[1])
    fake.open()
    _patch_detection(monkeypatch, fake)

    fake_stdin = _FakeStdin([], tty=False)
    monkeypatch.setattr(sys, "stdin", fake_stdin)

    # Must complete without raising
    setup_motors.cmd_setup_motors(_make_args(apply=True))
    assert len(fake.eeprom_writes) == 6


# ---------------------------------------------------------------------------
# --baudrate: plumbed into write, validated
# ---------------------------------------------------------------------------


def test_baudrate_plumbed_into_write(monkeypatch, capsys):
    """--baudrate is forwarded to write_id_baudrate for every motor."""
    fake = FakeBus(ids=[1])
    fake.open()
    _patch_detection(monkeypatch, fake)

    fake_stdin = _FakeStdin(["\n"] * 6, tty=True)
    monkeypatch.setattr(sys, "stdin", fake_stdin)

    setup_motors.cmd_setup_motors(_make_args(baudrate=500_000))

    assert len(fake.eeprom_writes) == 6
    for write in fake.eeprom_writes:
        assert write["baudrate"] == 500_000, "Non-default baudrate must be forwarded to write"


def test_baudrate_invalid_raises_user_error(monkeypatch):
    """An unsupported --baudrate raises CliError(EXIT_USER_ERROR) before any bus open."""
    bus_opened = [False]

    def _open(port):
        bus_opened[0] = True
        b = FakeBus(ids=[1])
        b.open()
        return b

    monkeypatch.setattr(cm, "_open_bus", _open)
    monkeypatch.setattr(cm, "_candidate_ports", lambda: ["/dev/ttyACM_fake"])

    fake_stdin = _FakeStdin([], tty=True)
    monkeypatch.setattr(sys, "stdin", fake_stdin)

    with pytest.raises(CliError) as exc_info:
        setup_motors.cmd_setup_motors(_make_args(baudrate=99999, apply=True))

    assert exc_info.value.code == EXIT_USER_ERROR
    assert bus_opened[0] is False, "Bus must not be opened for an invalid baudrate"


def test_baudrate_in_json_output(monkeypatch, capsys):
    """--baudrate appears in the JSON assigned list."""
    fake = FakeBus(ids=[1])
    fake.open()
    _patch_detection(monkeypatch, fake)

    fake_stdin = _FakeStdin(["\n"] * 6, tty=True)
    monkeypatch.setattr(sys, "stdin", fake_stdin)

    setup_motors.cmd_setup_motors(_make_args(json_mode=True, baudrate=500_000))

    payload = json.loads(capsys.readouterr().out)
    for entry in payload["assigned"]:
        assert entry["baudrate"] == 500_000


# ---------------------------------------------------------------------------
# --current-id: safety assertion (not address override)
# ---------------------------------------------------------------------------


def test_current_id_matching_detected_passes(monkeypatch, capsys):
    """--current-id equal to the detected id proceeds normally."""
    fake = FakeBus(ids=[1])  # scan returns id=1
    fake.open()
    _patch_detection(monkeypatch, fake)

    fake_stdin = _FakeStdin(["\n"] * 6, tty=True)
    monkeypatch.setattr(sys, "stdin", fake_stdin)

    # current_id=1 matches detected id=1
    setup_motors.cmd_setup_motors(_make_args(current_id=1))
    assert len(fake.eeprom_writes) == 6


def test_current_id_mismatch_raises_user_error(monkeypatch):
    """--current-id that doesn't match detected id raises CliError(EXIT_USER_ERROR)."""
    fake = FakeBus(ids=[1])  # scan returns id=1
    fake.open()
    _patch_detection(monkeypatch, fake)

    fake_stdin = _FakeStdin(["\n"] * 6, tty=True)
    monkeypatch.setattr(sys, "stdin", fake_stdin)

    # current_id=5 does not match detected id=1 -> error on first motor
    with pytest.raises(CliError) as exc_info:
        setup_motors.cmd_setup_motors(_make_args(current_id=5))

    assert exc_info.value.code == EXIT_USER_ERROR
    assert "5" in exc_info.value.message or "1" in exc_info.value.message


def test_current_id_none_auto_detects(monkeypatch, capsys):
    """Omitting --current-id accepts any detected id without asserting."""
    fake = FakeBus(ids=[1])
    fake.open()
    _patch_detection(monkeypatch, fake)

    fake_stdin = _FakeStdin(["\n"] * 6, tty=True)
    monkeypatch.setattr(sys, "stdin", fake_stdin)

    # No current_id assertion -> proceeds normally
    setup_motors.cmd_setup_motors(_make_args(current_id=None))
    assert len(fake.eeprom_writes) == 6


def test_current_id_zero_raises_user_error(monkeypatch):
    """--current-id 0 is out of range -> CliError(EXIT_USER_ERROR), before any bus open."""
    fake_stdin = _FakeStdin([], tty=False)
    monkeypatch.setattr(sys, "stdin", fake_stdin)

    with pytest.raises(CliError) as exc_info:
        setup_motors.cmd_setup_motors(_make_args(current_id=0))

    assert exc_info.value.code == EXIT_USER_ERROR


def test_current_id_254_raises_user_error(monkeypatch):
    """--current-id 254 is the broadcast id -> CliError(EXIT_USER_ERROR), before bus open."""
    fake_stdin = _FakeStdin([], tty=False)
    monkeypatch.setattr(sys, "stdin", fake_stdin)

    with pytest.raises(CliError) as exc_info:
        setup_motors.cmd_setup_motors(_make_args(current_id=254))

    assert exc_info.value.code == EXIT_USER_ERROR


# ---------------------------------------------------------------------------
# register() wires the subparser correctly
# ---------------------------------------------------------------------------


def test_register_creates_subparser():
    """register() attaches setup-motors to a subparsers group."""
    top = argparse.ArgumentParser(prog="arm101")
    sub = top.add_subparsers()
    setup_motors.register(sub)

    args = top.parse_args(["setup-motors", "--json"])
    assert args.json is True
    assert args.func is setup_motors.cmd_setup_motors


def test_register_has_apply_flag():
    """register() includes --apply flag."""
    top = argparse.ArgumentParser(prog="arm101")
    sub = top.add_subparsers()
    setup_motors.register(sub)

    args = top.parse_args(["setup-motors", "--apply"])
    assert args.apply is True


def test_register_has_baudrate_flag():
    """register() includes --baudrate flag with default 1_000_000."""
    top = argparse.ArgumentParser(prog="arm101")
    sub = top.add_subparsers()
    setup_motors.register(sub)

    args = top.parse_args(["setup-motors"])
    assert args.baudrate == 1_000_000

    args2 = top.parse_args(["setup-motors", "--baudrate", "500000"])
    assert args2.baudrate == 500_000


def test_register_port_defaults_to_none():
    """register() --port default is None (auto-detect)."""
    top = argparse.ArgumentParser(prog="arm101")
    sub = top.add_subparsers()
    setup_motors.register(sub)

    args = top.parse_args(["setup-motors"])
    assert args.port is None


def test_register_current_id_defaults_to_none():
    """register() --current-id default is None (no assertion)."""
    top = argparse.ArgumentParser(prog="arm101")
    sub = top.add_subparsers()
    setup_motors.register(sub)

    args = top.parse_args(["setup-motors"])
    assert args.current_id is None
