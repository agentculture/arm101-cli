"""Tests for ``arm101 set-motor-id`` — gated EEPROM ID assignment for STS3215.

Drives the verb entirely against a :class:`~arm101.hardware.bus.FakeBus` and
monkeypatched detection seams; no hardware is touched.  Covers: gated confirm
→ EEPROM write recorded; decline → no write; EOF at confirmation → CliError;
new_id out of range and non-integer → CliError(EXIT_USER_ERROR); positional
vs prompted ID; stdout/stderr split; ``--json`` shape; no SystemExit leaks.

Patches go to :mod:`arm101.cli._commands.calibrate_motor` because
``_detect_one_motor`` resolves ``_open_bus`` and ``_candidate_ports`` as
module-level globals there — even though ``set_motor_id`` imports them from
the same module, patching the origin is what makes the seam work.
"""

from __future__ import annotations

import argparse
import json
import sys

import pytest

from arm101.cli._commands import calibrate_motor as cm
from arm101.cli._commands import set_motor_id as sm
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
    ns = argparse.Namespace(new_id=None, port=None, baudrate=1_000_000, json=False, apply=False)
    for key, value in kw.items():
        setattr(ns, key, value)
    return ns


def _patch_single_port(monkeypatch, bus: FakeBus) -> None:
    """One candidate port at /dev/ttyACM1, opening to *bus*."""
    monkeypatch.setattr(cm, "_candidate_ports", lambda: ["/dev/ttyACM1"])
    monkeypatch.setattr(cm, "_open_bus", lambda port: bus)


# ---------------------------------------------------------------------------
# Happy path: confirm → EEPROM write recorded
# ---------------------------------------------------------------------------


def test_confirm_writes_eeprom(monkeypatch, capsys) -> None:
    """Typing 'yes' at the gate causes exactly one eeprom_write entry."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    # confirmation prompt
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["yes\n"]))

    sm.cmd_set_motor_id(_args(new_id="5"))

    assert bus.eeprom_writes == [{"motor": 1, "new_id": 5, "baudrate": 1_000_000}]


def test_confirm_writes_eeprom_custom_baudrate(monkeypatch, capsys) -> None:
    """Custom --baudrate is forwarded to write_id_baudrate."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["yes\n"]))

    sm.cmd_set_motor_id(_args(new_id="3", baudrate=500_000))

    assert bus.eeprom_writes == [{"motor": 1, "new_id": 3, "baudrate": 500_000}]


def test_id_supplied_via_prompt(monkeypatch, capsys) -> None:
    """New ID supplied interactively (no positional arg) is read from stdin."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    # new_id prompt, then confirmation
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["7\n", "yes\n"]))

    sm.cmd_set_motor_id(_args())  # no positional new_id

    assert bus.eeprom_writes == [{"motor": 1, "new_id": 7, "baudrate": 1_000_000}]


# ---------------------------------------------------------------------------
# Non-TTY guard / abort-in-JSON (Qodo #1, #4)
# ---------------------------------------------------------------------------


class _NonTtyStdin:
    """Non-TTY stdin: isatty() is False and readline() must never be called.

    In agent/dry-run mode the verb must never prompt, so a readline() here is a
    consent-routing bug — fail loudly instead of silently handing back 'yes'.
    """

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

    sm.cmd_set_motor_id(_args(new_id="5"))

    out = capsys.readouterr()
    assert "Dry-run plan" in out.out
    assert "set-motor-id" in out.out
    assert bus.eeprom_writes == []


def test_non_tty_apply_without_target_refused(monkeypatch) -> None:
    """Non-TTY with --apply but no new_id raises CliError(EXIT_USER_ERROR)."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _NonTtyStdin())

    with pytest.raises(CliError) as exc:
        sm.cmd_set_motor_id(_args(apply=True))  # no new_id

    assert exc.value.code == EXIT_USER_ERROR
    assert bus.eeprom_writes == []


def test_non_tty_apply_writes_eeprom(monkeypatch, tmp_path) -> None:
    """Non-TTY with --apply and new_id writes EEPROM and produces audit log."""
    audit_log = tmp_path / "audit.log"
    monkeypatch.setenv("ARM101_AUDIT_LOG", str(audit_log))

    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _NonTtyStdin())

    sm.cmd_set_motor_id(_args(new_id="6", apply=True))

    assert bus.eeprom_writes == [{"motor": 1, "new_id": 6, "baudrate": 1_000_000}]

    # Audit log must have pending before success
    lines = audit_log.read_text().strip().splitlines()
    assert len(lines) >= 2
    records = [json.loads(line) for line in lines]
    pending_records = [r for r in records if r["outcome"] == "pending"]
    success_records = [r for r in records if r["outcome"] == "success"]
    assert len(pending_records) == 1
    assert len(success_records) == 1
    # pending must precede success in the log
    pending_idx = next(i for i, r in enumerate(records) if r["outcome"] == "pending")
    success_idx = next(i for i, r in enumerate(records) if r["outcome"] == "success")
    assert pending_idx < success_idx


def test_abort_emits_valid_json(monkeypatch, capsys) -> None:
    """Declining in --json mode emits valid JSON (aborted), not plain text."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["no\n"]))

    sm.cmd_set_motor_id(_args(new_id="5", json=True))

    payload = json.loads(capsys.readouterr().out)
    assert payload["aborted"] is True
    assert payload["to_id"] == 5
    assert bus.eeprom_writes == []


# ---------------------------------------------------------------------------
# Decline: no EEPROM write, clean exit 0
# ---------------------------------------------------------------------------


def test_decline_no_eeprom_write(monkeypatch, capsys) -> None:
    """Answering 'no' aborts cleanly: no eeprom_writes, 'Aborted' to stdout."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["no\n"]))

    sm.cmd_set_motor_id(_args(new_id="5"))

    assert bus.eeprom_writes == []
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

    sm.cmd_set_motor_id(_args(new_id="2"))

    assert bus.eeprom_writes == []
    assert "Aborted" in capsys.readouterr().out


# ---------------------------------------------------------------------------
# EOF / non-interactive guard
# ---------------------------------------------------------------------------


def test_eof_at_confirmation_raises_env_error(monkeypatch) -> None:
    """EOF at the confirmation prompt refuses the write (EXIT_ENV_ERROR)."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([]))  # immediate EOF

    with pytest.raises(CliError) as exc:
        sm.cmd_set_motor_id(_args(new_id="5"))
    assert exc.value.code == EXIT_ENV_ERROR
    assert bus.eeprom_writes == []


def test_eof_at_new_id_prompt_raises_env_error(monkeypatch) -> None:
    """EOF when prompting for new_id also refuses the write (EXIT_ENV_ERROR)."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([]))

    with pytest.raises(CliError) as exc:
        sm.cmd_set_motor_id(_args())  # no positional → needs stdin
    assert exc.value.code == EXIT_ENV_ERROR
    assert bus.eeprom_writes == []


# ---------------------------------------------------------------------------
# new_id validation: out-of-range integers
# ---------------------------------------------------------------------------


def test_new_id_zero_is_rejected(monkeypatch) -> None:
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([]))

    with pytest.raises(CliError) as exc:
        sm.cmd_set_motor_id(_args(new_id="0"))
    assert exc.value.code == EXIT_USER_ERROR
    assert bus.eeprom_writes == []


def test_new_id_254_is_rejected(monkeypatch) -> None:
    """254 is the broadcast ID and must be refused."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([]))

    with pytest.raises(CliError) as exc:
        sm.cmd_set_motor_id(_args(new_id="254"))
    assert exc.value.code == EXIT_USER_ERROR
    assert bus.eeprom_writes == []


def test_new_id_999_is_rejected(monkeypatch) -> None:
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([]))

    with pytest.raises(CliError) as exc:
        sm.cmd_set_motor_id(_args(new_id="999"))
    assert exc.value.code == EXIT_USER_ERROR
    assert bus.eeprom_writes == []


# ---------------------------------------------------------------------------
# new_id validation: non-integer
# ---------------------------------------------------------------------------


def test_new_id_non_integer_is_rejected(monkeypatch) -> None:
    """A non-integer new_id raises EXIT_USER_ERROR."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([]))

    with pytest.raises(CliError) as exc:
        sm.cmd_set_motor_id(_args(new_id="abc"))
    assert exc.value.code == EXIT_USER_ERROR
    assert bus.eeprom_writes == []


def test_new_id_non_integer_from_prompt_is_rejected(monkeypatch) -> None:
    """A non-integer answer at the new_id prompt is EXIT_USER_ERROR."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["notanumber\n"]))

    with pytest.raises(CliError) as exc:
        sm.cmd_set_motor_id(_args())
    assert exc.value.code == EXIT_USER_ERROR
    assert bus.eeprom_writes == []


# ---------------------------------------------------------------------------
# stdout / stderr split
# ---------------------------------------------------------------------------


def test_stdout_stderr_split(monkeypatch, capsys) -> None:
    """Result lands on stdout; warnings, snapshot, and prompts land on stderr."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["yes\n"]))

    sm.cmd_set_motor_id(_args(new_id="4"))

    out = capsys.readouterr()
    assert "Set motor ID" in out.out
    assert "EEPROM written" in out.out
    # Diagnostics (snapshot + warning) must be on stderr only
    assert "Detected motor" in out.err
    assert "WRITES EEPROM" in out.err
    # Result must NOT appear on stderr
    assert "Set motor ID" not in out.err


def test_result_text_mentions_port_and_ids(monkeypatch, capsys) -> None:
    """Result line names the port, both IDs, and baudrate."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["yes\n"]))

    sm.cmd_set_motor_id(_args(new_id="6"))

    out = capsys.readouterr().out
    assert "/dev/ttyACM1" in out
    assert "1" in out  # from_id
    assert "6" in out  # to_id
    assert "1000000" in out


# ---------------------------------------------------------------------------
# --json shape
# ---------------------------------------------------------------------------


def test_json_shape(monkeypatch, capsys) -> None:
    """--json emits structured payload to stdout; nothing structured to stderr."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["yes\n"]))

    sm.cmd_set_motor_id(_args(new_id="8", json=True))

    out = capsys.readouterr()
    payload = json.loads(out.out)
    assert payload["port"] == "/dev/ttyACM1"
    assert payload["from_id"] == 1
    assert payload["to_id"] == 8
    assert payload["baudrate"] == 1_000_000
    # Diagnostics still go to stderr, not stdout
    assert out.err  # snapshot/warning on stderr


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
        sm.cmd_set_motor_id(_args(new_id="5"))
    except CliError:
        pass
    except SystemExit:  # pragma: no cover
        pytest.fail("set-motor-id raised SystemExit instead of CliError")


def test_never_raises_systemexit_on_bad_id(monkeypatch) -> None:
    """Bad new_id must surface as CliError, not SystemExit."""
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([]))

    try:
        sm.cmd_set_motor_id(_args(new_id="abc"))
    except CliError:
        pass
    except SystemExit:  # pragma: no cover
        pytest.fail("set-motor-id raised SystemExit instead of CliError")


# ---------------------------------------------------------------------------
# Boundary: id 1 and id 253 are both valid
# ---------------------------------------------------------------------------


def test_boundary_id_1_is_valid(monkeypatch, capsys) -> None:
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["yes\n"]))

    sm.cmd_set_motor_id(_args(new_id="1"))

    assert bus.eeprom_writes == [{"motor": 1, "new_id": 1, "baudrate": 1_000_000}]


def test_boundary_id_253_is_valid(monkeypatch, capsys) -> None:
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["yes\n"]))

    sm.cmd_set_motor_id(_args(new_id="253"))

    assert bus.eeprom_writes == [{"motor": 1, "new_id": 253, "baudrate": 1_000_000}]
