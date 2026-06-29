"""Tests for ``arm101 set-baudrate`` — gated EEPROM baud-rate change for STS3215.

Drives the verb entirely against a :class:`~arm101.hardware.bus.FakeBus` and
monkeypatched detection seams; no hardware is touched.  Covers: gated confirm →
baud_write recorded (id unchanged); decline → no write; non-TTY dry-run; non-TTY
--apply; agent --apply with audit log; bad baud → CliError(EXIT_USER_ERROR);
--json shape; before/after cards on stderr; no SystemExit leaks.

Patches go to :mod:`arm101.cli._commands.calibrate_motor` because
``_detect_one_motor`` resolves ``_open_bus`` and ``_candidate_ports`` as
module-level globals there — even though ``set_baudrate`` imports them from
the same module, patching the origin is what makes the seam work.

The ``_open_bus_after`` seam in :mod:`arm101.cli._commands.set_baudrate` is also
patched to inject a :class:`~arm101.hardware.bus.FakeBus` for the after-card read
without opening a real serial port.
"""

from __future__ import annotations

import argparse
import json
import sys

import pytest

from arm101.cli._commands import calibrate_motor as cm
from arm101.cli._commands import set_baudrate as sb
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

    def isatty(self) -> bool:  # pragma: no cover
        return True


def _args(**kw) -> argparse.Namespace:
    ns = argparse.Namespace(baud=None, port=None, json=False, apply=False)
    for key, value in kw.items():
        setattr(ns, key, value)
    return ns


def _patch_single_port(monkeypatch, bus: FakeBus) -> None:
    """One candidate port at /dev/ttyACM1, opening to *bus*."""
    monkeypatch.setattr(cm, "_candidate_ports", lambda: ["/dev/ttyACM1"])
    monkeypatch.setattr(cm, "_open_bus", lambda port: bus)


def _patch_after_bus(monkeypatch, after_bus: FakeBus) -> None:
    """Patch _open_bus_after to return *after_bus* (already open)."""
    monkeypatch.setattr(sb, "_open_bus_after", lambda port, baud: after_bus)


def _make_after_bus(motor_ids: list[int]) -> FakeBus:
    """Create a pre-opened FakeBus for the after-card."""
    b = FakeBus(ids=motor_ids)
    b.open()
    return b


# ---------------------------------------------------------------------------
# Happy path: confirm → baud_write recorded, id unchanged
# ---------------------------------------------------------------------------


def test_confirm_writes_baud(monkeypatch, capsys) -> None:
    """Typing 'yes' at the gate causes exactly one baud_write entry; id is unchanged."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    _patch_after_bus(monkeypatch, _make_after_bus([1]))
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["yes\n"]))

    sb.cmd_set_baudrate(_args(baud=500_000))

    assert bus.baud_writes == [{"motor": 1, "baudrate": 500_000}]
    # ID register must NOT have been written.
    assert bus.eeprom_writes == []


def test_confirm_writes_baud_default_rate(monkeypatch, capsys) -> None:
    """1 000 000 baud (the Feetech default) is accepted and written."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    _patch_after_bus(monkeypatch, _make_after_bus([1]))
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["yes\n"]))

    sb.cmd_set_baudrate(_args(baud=1_000_000))

    assert bus.baud_writes == [{"motor": 1, "baudrate": 1_000_000}]
    assert bus.eeprom_writes == []


def test_baud_supplied_via_prompt(monkeypatch, capsys) -> None:
    """Target baud supplied interactively (no positional arg) is read from stdin."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    _patch_after_bus(monkeypatch, _make_after_bus([1]))
    # baud prompt, then confirmation
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["500000\n", "yes\n"]))

    sb.cmd_set_baudrate(_args())  # no positional baud

    assert bus.baud_writes == [{"motor": 1, "baudrate": 500_000}]
    assert bus.eeprom_writes == []


# ---------------------------------------------------------------------------
# Non-TTY guard / dry-run
# ---------------------------------------------------------------------------


class _NonTtyStdin:
    """Non-TTY stdin: isatty() is False and readline() must never be called."""

    def readline(self) -> str:  # pragma: no cover - asserts it is never reached
        raise AssertionError("readline() called in non-TTY mode — consent routing bug")

    def isatty(self) -> bool:
        return False


def test_non_tty_stdin_is_dry_run(monkeypatch, capsys) -> None:
    """A non-interactive stdin without --apply resolves to dry_run (plan only)."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _NonTtyStdin())

    sb.cmd_set_baudrate(_args(baud=500_000))

    out = capsys.readouterr()
    assert "Dry-run plan" in out.out
    assert "set-baudrate" in out.out
    assert bus.baud_writes == []


def test_non_tty_apply_without_baud_refused(monkeypatch) -> None:
    """Non-TTY with --apply but no baud raises CliError(EXIT_USER_ERROR)."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _NonTtyStdin())

    with pytest.raises(CliError) as exc:
        sb.cmd_set_baudrate(_args(apply=True))  # no baud

    assert exc.value.code == EXIT_USER_ERROR
    assert bus.baud_writes == []


def test_non_tty_apply_writes_baud(monkeypatch, tmp_path) -> None:
    """Non-TTY with --apply and baud writes EEPROM and produces audit log."""
    audit_log = tmp_path / "audit.log"
    monkeypatch.setenv("ARM101_AUDIT_LOG", str(audit_log))

    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    _patch_after_bus(monkeypatch, _make_after_bus([1]))
    monkeypatch.setattr(sys, "stdin", _NonTtyStdin())

    sb.cmd_set_baudrate(_args(baud=500_000, apply=True))

    assert bus.baud_writes == [{"motor": 1, "baudrate": 500_000}]
    assert bus.eeprom_writes == []

    # Audit log must have pending before success
    lines = audit_log.read_text().strip().splitlines()
    assert len(lines) >= 2
    records = [json.loads(line) for line in lines]
    pending_records = [r for r in records if r["outcome"] == "pending"]
    success_records = [r for r in records if r["outcome"] == "success"]
    assert len(pending_records) == 1
    assert len(success_records) == 1
    pending_idx = next(i for i, r in enumerate(records) if r["outcome"] == "pending")
    success_idx = next(i for i, r in enumerate(records) if r["outcome"] == "success")
    assert pending_idx < success_idx


# ---------------------------------------------------------------------------
# Decline: no write, clean exit 0
# ---------------------------------------------------------------------------


def test_decline_no_baud_write(monkeypatch, capsys) -> None:
    """Answering 'no' aborts cleanly: no baud_writes, 'Aborted' to stdout."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["no\n"]))

    sb.cmd_set_baudrate(_args(baud=500_000))

    assert bus.baud_writes == []
    out = capsys.readouterr()
    assert "Aborted" in out.out
    assert "EEPROM" in out.out
    # Result goes to stdout, not stderr
    assert "Aborted" not in out.err


def test_blank_answer_is_declined(monkeypatch, capsys) -> None:
    """An empty confirmation answer (not 'yes') is treated as decline."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["\n"]))

    sb.cmd_set_baudrate(_args(baud=500_000))

    assert bus.baud_writes == []
    assert "Aborted" in capsys.readouterr().out


def test_abort_emits_valid_json(monkeypatch, capsys) -> None:
    """Declining in --json mode emits valid JSON (aborted), not plain text."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["no\n"]))

    sb.cmd_set_baudrate(_args(baud=500_000, json=True))

    payload = json.loads(capsys.readouterr().out)
    assert payload["aborted"] is True
    assert payload["baudrate"] == 500_000
    assert bus.baud_writes == []


# ---------------------------------------------------------------------------
# EOF guard
# ---------------------------------------------------------------------------


def test_eof_at_confirmation_raises_env_error(monkeypatch) -> None:
    """EOF at the confirmation prompt refuses the write (EXIT_ENV_ERROR)."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([]))  # immediate EOF

    with pytest.raises(CliError) as exc:
        sb.cmd_set_baudrate(_args(baud=500_000))
    assert exc.value.code == EXIT_ENV_ERROR
    assert bus.baud_writes == []


# ---------------------------------------------------------------------------
# baud validation: unsupported values
# ---------------------------------------------------------------------------


def test_bad_baud_raises_user_error(monkeypatch) -> None:
    """An unsupported baudrate raises CliError(EXIT_USER_ERROR)."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([]))

    with pytest.raises(CliError) as exc:
        sb.cmd_set_baudrate(_args(baud=9600))  # not in BAUD_MAP
    assert exc.value.code == EXIT_USER_ERROR
    assert bus.baud_writes == []


def test_zero_baud_raises_user_error(monkeypatch) -> None:
    """Baudrate 0 is not in BAUD_MAP and must be refused."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([]))

    with pytest.raises(CliError) as exc:
        sb.cmd_set_baudrate(_args(baud=0))
    assert exc.value.code == EXIT_USER_ERROR
    assert bus.baud_writes == []


def test_bad_baud_fails_before_bus_opened(monkeypatch) -> None:
    """An unsupported positional baud is rejected before any port is opened/scanned.

    The CHANGELOG promises invalid baud fails "before any bus is opened", so the
    detection seam must never be reached. We arm ``_open_bus`` to fail loudly if
    it is.
    """

    def _boom(_port):  # pragma: no cover - asserts it is never reached
        raise AssertionError("_open_bus called — bad baud should fail before bus open")

    monkeypatch.setattr(cm, "_candidate_ports", lambda: ["/dev/ttyACM1"])
    monkeypatch.setattr(cm, "_open_bus", _boom)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([]))

    with pytest.raises(CliError) as exc:
        sb.cmd_set_baudrate(_args(baud=9600))  # not in BAUD_MAP
    assert exc.value.code == EXIT_USER_ERROR


def test_non_tty_no_baud_fails_before_bus_opened(monkeypatch) -> None:
    """Non-TTY with no baud is rejected before any port is opened/scanned."""

    def _boom(_port):  # pragma: no cover - asserts it is never reached
        raise AssertionError("_open_bus called — missing baud should fail before bus open")

    monkeypatch.setattr(cm, "_candidate_ports", lambda: ["/dev/ttyACM1"])
    monkeypatch.setattr(cm, "_open_bus", _boom)
    monkeypatch.setattr(sys, "stdin", _NonTtyStdin())

    with pytest.raises(CliError) as exc:
        sb.cmd_set_baudrate(_args(apply=True))  # no baud
    assert exc.value.code == EXIT_USER_ERROR


def test_bad_baud_from_prompt_raises_user_error(monkeypatch) -> None:
    """A non-integer baud from the interactive prompt raises EXIT_USER_ERROR."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["notabaud\n"]))

    with pytest.raises(CliError) as exc:
        sb.cmd_set_baudrate(_args())  # no positional → prompts
    assert exc.value.code == EXIT_USER_ERROR
    assert bus.baud_writes == []


# ---------------------------------------------------------------------------
# stdout / stderr split
# ---------------------------------------------------------------------------


def test_stdout_stderr_split(monkeypatch, capsys) -> None:
    """Result lands on stdout; snapshot and warning land on stderr."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    _patch_after_bus(monkeypatch, _make_after_bus([1]))
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["yes\n"]))

    sb.cmd_set_baudrate(_args(baud=500_000))

    out = capsys.readouterr()
    assert "baudrate" in out.out.lower() or "Set baudrate" in out.out
    assert "EEPROM written" in out.out
    # Diagnostics (snapshot + warning) must be on stderr only
    assert "Detected motor" in out.err
    assert "WRITES EEPROM" in out.err
    # Result must NOT appear on stderr
    assert "EEPROM written" not in out.err


def test_result_text_mentions_port_motor_baudrate(monkeypatch, capsys) -> None:
    """Result line names the port, motor id, and baudrate."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    _patch_after_bus(monkeypatch, _make_after_bus([1]))
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["yes\n"]))

    sb.cmd_set_baudrate(_args(baud=500_000))

    out = capsys.readouterr().out
    assert "/dev/ttyACM1" in out
    assert "1" in out  # motor id
    assert "500000" in out


# ---------------------------------------------------------------------------
# Before / after cards on stderr
# ---------------------------------------------------------------------------


def test_before_card_on_stderr(monkeypatch, capsys) -> None:
    """The BEFORE card (register snapshot) goes to stderr before the prompt."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    _patch_after_bus(monkeypatch, _make_after_bus([1]))
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["yes\n"]))

    sb.cmd_set_baudrate(_args(baud=500_000))

    err = capsys.readouterr().err
    assert "Detected motor" in err


def test_after_card_on_stderr(monkeypatch, capsys) -> None:
    """The AFTER card header goes to stderr after a successful write."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    _patch_after_bus(monkeypatch, _make_after_bus([1]))
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["yes\n"]))

    sb.cmd_set_baudrate(_args(baud=500_000))

    err = capsys.readouterr().err
    assert "AFTER write" in err


# ---------------------------------------------------------------------------
# --json shape
# ---------------------------------------------------------------------------


def test_json_shape(monkeypatch, capsys) -> None:
    """--json emits structured payload to stdout; diagnostics still go to stderr."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    _patch_after_bus(monkeypatch, _make_after_bus([1]))
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["yes\n"]))

    sb.cmd_set_baudrate(_args(baud=500_000, json=True))

    out = capsys.readouterr()
    payload = json.loads(out.out)
    assert payload["port"] == "/dev/ttyACM1"
    assert payload["motor"] == 1
    assert payload["baudrate"] == 500_000
    # Diagnostics still go to stderr, not stdout
    assert out.err  # snapshot/warning on stderr


def test_dry_run_json_shape(monkeypatch, capsys) -> None:
    """Dry-run --json emits a plan object with verb/port/motor/baudrate fields."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _NonTtyStdin())

    sb.cmd_set_baudrate(_args(baud=500_000, json=True))

    payload = json.loads(capsys.readouterr().out)
    assert "plan" in payload
    assert payload["plan"]["verb"] == "set-baudrate"
    assert payload["plan"]["to_baudrate"] == 500_000
    assert "apply_command" in payload
    assert bus.baud_writes == []


# ---------------------------------------------------------------------------
# Never raises SystemExit
# ---------------------------------------------------------------------------


def test_never_raises_systemexit_on_failure(monkeypatch) -> None:
    """Failures surface as CliError, never a bare SystemExit / traceback."""
    bus = FakeBus(ids=[])  # nothing responds → env error
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([]))

    try:
        sb.cmd_set_baudrate(_args(baud=500_000))
    except CliError:
        pass
    except SystemExit:  # pragma: no cover
        pytest.fail("set-baudrate raised SystemExit instead of CliError")


def test_never_raises_systemexit_on_bad_baud(monkeypatch) -> None:
    """Bad baud must surface as CliError, not SystemExit."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([]))

    try:
        sb.cmd_set_baudrate(_args(baud=9600))
    except CliError:
        pass
    except SystemExit:  # pragma: no cover
        pytest.fail("set-baudrate raised SystemExit instead of CliError")


# ---------------------------------------------------------------------------
# Boundary: all supported baud rates are accepted
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("baud", [38_400, 57_600, 76_800, 115_200, 128_000, 250_000, 500_000])
def test_supported_baud_rates_accepted(monkeypatch, capsys, baud: int) -> None:
    """Every rate in BAUD_MAP is accepted (interactive confirm path)."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    _patch_after_bus(monkeypatch, _make_after_bus([1]))
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["yes\n"]))

    sb.cmd_set_baudrate(_args(baud=baud))

    assert bus.baud_writes == [{"motor": 1, "baudrate": baud}]
    assert bus.eeprom_writes == []
    capsys.readouterr()
