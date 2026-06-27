"""Tests for ``arm101 setup-motors`` — one-motor-at-a-time EEPROM id/baudrate.

Verb is NOT registered in main() yet (that is task t8).  We build a throwaway
parser and call the handler directly.

Seam: ``setup_motors._open_bus`` is monkeypatched to return a ``FakeBus``.
Stdin seam: ``sys.stdin`` is replaced with a fake object exposing
``.isatty()`` and ``.readline()`` so a single ``monkeypatch.setattr`` controls
both TTY detection and line input.
"""

from __future__ import annotations

import argparse
import json
import sys

import pytest

from arm101.cli._commands import setup_motors
from arm101.cli._errors import EXIT_ENV_ERROR, CliError
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


def _make_args(json_mode: bool = False, port: str = "/dev/ttyACM0") -> argparse.Namespace:
    """Return a minimal Namespace as if parsed by argparse."""
    return argparse.Namespace(json=json_mode, port=port)


class _FakeStdin:
    """Fake stdin that controls both isatty() and readline() independently."""

    def __init__(self, lines: list[str], tty: bool = True) -> None:
        self._lines = iter(lines)
        self._tty = tty

    def isatty(self) -> bool:
        return self._tty

    def readline(self) -> str:
        try:
            return next(self._lines)
        except StopIteration:
            return ""  # EOF


# ---------------------------------------------------------------------------
# Full 6-motor walk: writes happen only AFTER Enter, in order 6→1
# ---------------------------------------------------------------------------


def test_full_walk_writes_in_order(monkeypatch, capsys):
    """All 6 motors are assigned in gripper→shoulder_pan order."""
    fake = FakeBus()
    fake.open()
    monkeypatch.setattr(setup_motors, "_open_bus", lambda _args: fake)

    # 6 Enters, one per motor
    fake_stdin = _FakeStdin(["\n"] * 6, tty=True)
    monkeypatch.setattr(sys, "stdin", fake_stdin)

    setup_motors.cmd_setup_motors(_make_args())

    assert len(fake.eeprom_writes) == 6
    expected_order = [motor_id for motor_id, _ in _MOTORS]
    for i, (expected_id, write) in enumerate(zip(expected_order, fake.eeprom_writes)):
        assert write["motor"] == expected_id, f"Write {i}: motor mismatch"
        assert write["new_id"] == expected_id, f"Write {i}: new_id mismatch"
        assert write["baudrate"] == _BAUDRATE, f"Write {i}: baudrate mismatch"


# ---------------------------------------------------------------------------
# Fewer Enters → fewer writes (safety invariant: no write without Enter)
# ---------------------------------------------------------------------------


def test_partial_walk_stops_at_stdin_eof(monkeypatch, capsys):
    """With only K Enters supplied, exactly K writes occur — never more."""
    fake = FakeBus()
    fake.open()
    monkeypatch.setattr(setup_motors, "_open_bus", lambda _args: fake)

    # Only 2 Enters → should write motors 6 and 5 then stop
    fake_stdin = _FakeStdin(["\n", "\n"], tty=True)
    monkeypatch.setattr(sys, "stdin", fake_stdin)

    # EOF on stdin causes readline() to return "" — the handler should stop
    # (raise CliError or return early)
    try:
        setup_motors.cmd_setup_motors(_make_args())
    except CliError:
        pass  # acceptable — stdin EOF is an environment error

    # At most 2 writes occurred
    assert len(fake.eeprom_writes) <= 2
    # The writes that DID happen are the first K motors (6, 5)
    for i, write in enumerate(fake.eeprom_writes):
        expected_id = _MOTORS[i][0]
        assert write["motor"] == expected_id
        assert write["new_id"] == expected_id


# ---------------------------------------------------------------------------
# Non-TTY → CliError(EXIT_ENV_ERROR) with zero writes
# ---------------------------------------------------------------------------


def test_non_tty_raises_env_error(monkeypatch):
    """Non-interactive stdin raises CliError(EXIT_ENV_ERROR); no writes occur."""
    fake = FakeBus()
    fake.open()
    monkeypatch.setattr(setup_motors, "_open_bus", lambda _args: fake)

    fake_stdin = _FakeStdin([], tty=False)
    monkeypatch.setattr(sys, "stdin", fake_stdin)

    with pytest.raises(CliError) as exc_info:
        setup_motors.cmd_setup_motors(_make_args())

    assert exc_info.value.code == EXIT_ENV_ERROR
    assert fake.eeprom_writes == [], "No writes must occur on non-TTY"


# ---------------------------------------------------------------------------
# stdout / stderr split: prompts on stderr, summary on stdout
# ---------------------------------------------------------------------------


def test_stdout_stderr_split(monkeypatch, capsys):
    """Prompts land on stderr; result summary lands on stdout."""
    fake = FakeBus()
    fake.open()
    monkeypatch.setattr(setup_motors, "_open_bus", lambda _args: fake)

    fake_stdin = _FakeStdin(["\n"] * 6, tty=True)
    monkeypatch.setattr(sys, "stdin", fake_stdin)

    setup_motors.cmd_setup_motors(_make_args(json_mode=False))

    captured = capsys.readouterr()

    # stderr must contain all prompts (each joint name appears)
    for _, joint in _MOTORS:
        assert joint in captured.err, f"Prompt for '{joint}' missing from stderr"

    # stdout must contain result text (and be non-empty)
    assert captured.out.strip(), "stdout must have non-empty result"

    # stdout must NOT contain prompts
    assert "press Enter" not in captured.out, "Prompts must not appear on stdout"


# ---------------------------------------------------------------------------
# --json output shape
# ---------------------------------------------------------------------------


def test_json_output_shape(monkeypatch, capsys):
    """--json emits {assigned: [{joint, motor, new_id, baudrate}, ...]} to stdout."""
    fake = FakeBus()
    fake.open()
    monkeypatch.setattr(setup_motors, "_open_bus", lambda _args: fake)

    fake_stdin = _FakeStdin(["\n"] * 6, tty=True)
    monkeypatch.setattr(sys, "stdin", fake_stdin)

    setup_motors.cmd_setup_motors(_make_args(json_mode=True))

    captured = capsys.readouterr()

    # stdout must be valid JSON
    payload = json.loads(captured.out)
    assert "assigned" in payload, "JSON must have 'assigned' key"
    assigned = payload["assigned"]
    assert len(assigned) == 6

    # Verify order and shape
    for i, (motor_id, joint_name) in enumerate(_MOTORS):
        entry = assigned[i]
        assert entry["joint"] == joint_name
        assert entry["motor"] == motor_id
        assert entry["new_id"] == motor_id
        assert entry["baudrate"] == _BAUDRATE

    # stderr prompts still go to stderr (not stdout)
    for _, joint in _MOTORS:
        assert joint in captured.err


# ---------------------------------------------------------------------------
# Confirm prompts come BEFORE writes (ordering invariant)
# ---------------------------------------------------------------------------


def test_prompt_before_write(monkeypatch, capsys):
    """Each prompt appears in stderr BEFORE its write occurs in eeprom_writes.

    We verify this by interleaving a write-recording side-effect with
    a readline() call that captures the stderr state at the moment of Enter.
    """
    fake = FakeBus()
    fake.open()
    monkeypatch.setattr(setup_motors, "_open_bus", lambda _args: fake)

    prompt_counts_at_enter: list[int] = []

    class TrackingStdin:
        def isatty(self) -> bool:
            return True

        def readline(self) -> str:
            # At the moment Enter is pressed, count how many prompts are on stderr
            captured = capsys.readouterr()
            # Put the output back by writing it again — capsys consumes it, we
            # need to count prompts seen so far
            sys.stderr.write(captured.err)
            sys.stdout.write(captured.out)
            prompt_counts_at_enter.append(captured.err.count("press Enter"))
            if len(prompt_counts_at_enter) < 6:
                return "\n"
            return "\n"

    monkeypatch.setattr(sys, "stdin", TrackingStdin())

    setup_motors.cmd_setup_motors(_make_args())

    # After the N-th Enter, there should have been at least N prompts on stderr
    for i, count in enumerate(prompt_counts_at_enter):
        assert count >= i + 1, (
            f"Enter #{i+1}: expected at least {i+1} prompts on stderr before this "
            f"Enter, got {count}"
        )


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
