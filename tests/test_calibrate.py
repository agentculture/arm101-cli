"""Tests for ``arm101 calibrate`` — interactive per-joint calibration verb.

TDD: this file was written before calibrate.py existed and drives the
implementation.  Tests cover:

* Missing ``id`` positional → SystemExit(1) via _CliArgumentParser
* Full capture → persist → reload loop against a FakeBus with sequenced reads
* CliError propagation when the bus/SDK is unavailable
* stdout/stderr split in both text and ``--json`` modes
* ``--json`` output shape
* Three consent modes: dry_run (non-TTY no --apply), agent (non-TTY + --apply),
  and interactive (TTY)
* EOF mid-capture raises CliError without writing a profile
"""

from __future__ import annotations

import argparse
import io
import json
import sys

import pytest

from arm101.cli import _CliArgumentParser
from arm101.cli._commands import calibrate
from arm101.cli._errors import EXIT_ENV_ERROR, EXIT_USER_ERROR, CliError
from arm101.hardware import profiles as profiles_mod
from arm101.hardware.bus import FakeBus

# ---------------------------------------------------------------------------
# Stdin test doubles
# ---------------------------------------------------------------------------


class _TtyStringIO(io.StringIO):
    """StringIO that reports ``isatty() == True`` (an interactive terminal)."""

    def isatty(self) -> bool:  # noqa: D401 - trivial override
        return True


class _NonTtyStringIO(io.StringIO):
    """StringIO that reports ``isatty() == False`` (a pipe/redirect)."""

    def isatty(self) -> bool:  # noqa: D401 - trivial override
        return False


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _parse(argv: list[str]) -> argparse.Namespace:
    """Build a throwaway parser that exposes only the calibrate subcommand.

    Missing required positionals trigger ``_CliArgumentParser.error()`` →
    ``SystemExit(EXIT_USER_ERROR)``, exactly as in the real CLI.
    """
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(parser_class=_CliArgumentParser)
    calibrate.register(sub)
    return parser.parse_args(argv)


def _make_seq_bus(rounds: list[dict[int, int]]) -> FakeBus:
    """Return an opened FakeBus whose ``read_position`` cycles through *rounds*.

    *rounds* is a list of ``{motor_id: position}`` dicts, one per read round.
    Each motor advances independently through the list on successive calls;
    motors absent from a round dict fall back to 2048.
    """
    fake = FakeBus()
    fake.open()
    call_counts: dict[int, int] = {}

    def _read_position(motor: int) -> int:
        idx = call_counts.get(motor, 0)
        call_counts[motor] = idx + 1
        if idx < len(rounds):
            return rounds[idx].get(motor, 2048)
        return 2048

    fake.read_position = _read_position  # type: ignore[method-assign]
    return fake


# ---------------------------------------------------------------------------
# 1. Missing id positional → SystemExit(1)
# ---------------------------------------------------------------------------


def test_missing_id_exits_1(capsys: pytest.CaptureFixture[str]) -> None:
    """``calibrate`` with no id positional must exit with code EXIT_USER_ERROR."""
    with pytest.raises(SystemExit) as exc:
        _parse(["calibrate"])
    assert exc.value.code == EXIT_USER_ERROR


# ---------------------------------------------------------------------------
# 2. Full capture → persist → reload loop
# ---------------------------------------------------------------------------


def test_full_calibration_loop(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: pytest.TempPathFactory,
    capsys: pytest.CaptureFixture[str],
) -> None:
    """Three-pose capture, save, and reload produce consistent min<=mid<=max.

    Positions per round (motor 1..6, JOINTS order):
      round 0 (mid):  2048 / 2100 / 2200 / 2300 / 2400 / 2500
      round 1 (min):  1000 / 1100 / 1200 / 1300 / 1400 / 1500
      round 2 (max):  3000 / 3100 / 3200 / 3300 / 3400 / 3500
    """
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))

    rounds = [
        {1: 2048, 2: 2100, 3: 2200, 4: 2300, 5: 2400, 6: 2500},  # mid
        {1: 1000, 2: 1100, 3: 1200, 4: 1300, 5: 1400, 6: 1500},  # min
        {1: 3000, 2: 3100, 3: 3200, 4: 3300, 5: 3400, 6: 3500},  # max
    ]
    fake = _make_seq_bus(rounds)
    monkeypatch.setattr(calibrate, "_open_bus", lambda args: fake)
    monkeypatch.setattr(sys, "stdin", _TtyStringIO("\n\n\n"))

    args = _parse(["calibrate", "my-arm"])
    calibrate.cmd_calibrate(args)

    # Reload the saved profile and verify every joint
    profile = profiles_mod.load("my-arm")

    # shoulder_pan: sorted([2048, 1000, 3000]) → min=1000, mid=2048, max=3000
    sp = profile.joints["shoulder_pan"]
    assert sp.min == 1000
    assert sp.mid == 2048
    assert sp.max == 3000

    # shoulder_lift: sorted([2100, 1100, 3100]) → min=1100, mid=2100, max=3100
    sl = profile.joints["shoulder_lift"]
    assert sl.min == 1100
    assert sl.mid == 2100
    assert sl.max == 3100

    # All joints: invariant min <= mid <= max
    for joint in profiles_mod.JOINTS:
        c = profile.joints[joint]
        assert c.min <= c.mid <= c.max, f"{joint}: {c.min} <= {c.mid} <= {c.max} violated"


# ---------------------------------------------------------------------------
# 3. CliError propagates when bus is unavailable (no SDK / no hardware)
# ---------------------------------------------------------------------------


def test_bus_unavailable_raises_cli_error(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: pytest.TempPathFactory,
) -> None:
    """When _open_bus raises CliError(EXIT_ENV_ERROR) it must propagate unmolested."""
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))

    def _fail_open(args: object) -> None:
        raise CliError(
            code=EXIT_ENV_ERROR,
            message="Feetech SDK 'scservo_sdk' is not installed.",
            remediation="pip install 'arm101[seeed]'",
        )

    monkeypatch.setattr(calibrate, "_open_bus", _fail_open)
    # TTY stdin so resolve_consent returns "interactive" and reaches _open_bus
    monkeypatch.setattr(sys, "stdin", _TtyStringIO(""))

    args = _parse(["calibrate", "my-arm"])
    with pytest.raises(CliError) as exc:
        calibrate.cmd_calibrate(args)
    assert exc.value.code == EXIT_ENV_ERROR


# ---------------------------------------------------------------------------
# 4a. stdout/stderr split — text mode
# ---------------------------------------------------------------------------


def test_stdout_stderr_split_text_mode(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: pytest.TempPathFactory,
    capsys: pytest.CaptureFixture[str],
) -> None:
    """Summary table goes to stdout; operator prompts go to stderr (text mode)."""
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))

    # All positions default to 2048 (empty round dicts)
    fake = _make_seq_bus([{}, {}, {}])
    monkeypatch.setattr(calibrate, "_open_bus", lambda args: fake)
    monkeypatch.setattr(sys, "stdin", _TtyStringIO("\n\n\n"))

    args = _parse(["calibrate", "my-arm"])
    calibrate.cmd_calibrate(args)

    captured = capsys.readouterr()

    # Result on stdout
    assert "Calibration saved" in captured.out
    assert "shoulder_pan" in captured.out

    # Prompts on stderr, not on stdout
    assert "centered/rest pose" in captured.err
    assert "MINIMUM" in captured.err
    assert "MAXIMUM" in captured.err
    assert "centered/rest pose" not in captured.out
    assert "MINIMUM" not in captured.out
    assert "MAXIMUM" not in captured.out


# ---------------------------------------------------------------------------
# 4b. stdout/stderr split — JSON mode
# ---------------------------------------------------------------------------


def test_stdout_stderr_split_json_mode(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: pytest.TempPathFactory,
    capsys: pytest.CaptureFixture[str],
) -> None:
    """JSON result goes to stdout; operator prompts stay on stderr."""
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))

    fake = _make_seq_bus([{}, {}, {}])
    monkeypatch.setattr(calibrate, "_open_bus", lambda args: fake)
    monkeypatch.setattr(sys, "stdin", _TtyStringIO("\n\n\n"))

    args = _parse(["calibrate", "my-arm", "--json"])
    calibrate.cmd_calibrate(args)

    captured = capsys.readouterr()

    # stdout is valid JSON
    data = json.loads(captured.out)
    assert data["id"] == "my-arm"

    # Prompts on stderr only
    assert "centered/rest pose" in captured.err
    assert "centered/rest pose" not in captured.out
    assert "MINIMUM" not in captured.out
    assert "MAXIMUM" not in captured.out


# ---------------------------------------------------------------------------
# 5. --json output shape
# ---------------------------------------------------------------------------


def test_json_output_shape(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: pytest.TempPathFactory,
    capsys: pytest.CaptureFixture[str],
) -> None:
    """--json emits {id, joints: {<joint>: {min, mid, max}}, path} with valid values."""
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))

    rounds = [
        {1: 2048, 2: 2048, 3: 2048, 4: 2048, 5: 2048, 6: 2048},  # mid
        {1: 500, 2: 500, 3: 500, 4: 500, 5: 500, 6: 500},  # min
        {1: 3500, 2: 3500, 3: 3500, 4: 3500, 5: 3500, 6: 3500},  # max
    ]
    fake = _make_seq_bus(rounds)
    monkeypatch.setattr(calibrate, "_open_bus", lambda args: fake)
    monkeypatch.setattr(sys, "stdin", _TtyStringIO("\n\n\n"))

    args = _parse(["calibrate", "test-robot", "--json"])
    calibrate.cmd_calibrate(args)

    captured = capsys.readouterr()
    data = json.loads(captured.out)

    # Top-level keys
    assert set(data.keys()) >= {"id", "joints", "path"}
    assert data["id"] == "test-robot"
    assert "test-robot" in data["path"]

    # Per-joint structure and values
    # sorted([2048, 500, 3500]) → min=500, mid=2048, max=3500
    for joint in profiles_mod.JOINTS:
        assert joint in data["joints"], f"joint {joint!r} missing from JSON output"
        jd = data["joints"][joint]
        assert set(jd.keys()) == {"min", "mid", "max"}
        assert jd["min"] == 500
        assert jd["mid"] == 2048
        assert jd["max"] == 3500
        assert jd["min"] <= jd["mid"] <= jd["max"]


# ---------------------------------------------------------------------------
# 6. dry_run mode: non-TTY without --apply
# ---------------------------------------------------------------------------


def test_dry_run_no_bus_no_write(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: pytest.TempPathFactory,
    capsys: pytest.CaptureFixture[str],
) -> None:
    """Non-TTY without --apply: _open_bus NOT called, no profile written, preview on stdout."""
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))

    bus_opened: list[bool] = []

    def _spy_open(args: object) -> FakeBus:
        bus_opened.append(True)
        bus = FakeBus()
        bus.open()
        return bus

    monkeypatch.setattr(calibrate, "_open_bus", _spy_open)
    monkeypatch.setattr(sys, "stdin", _NonTtyStringIO(""))

    args = _parse(["calibrate", "dry-arm"])
    calibrate.cmd_calibrate(args)

    # Bus must NOT have been opened
    assert bus_opened == [], "dry_run mode must not open the bus"

    # No profile written
    assert not profiles_mod.profile_path("dry-arm").exists(), "dry_run must not write a profile"

    # stdout contains the dry-run preview
    out, _ = capsys.readouterr()
    assert "dry-arm" in out
    assert "shoulder_pan" in out
    assert "centered/rest" in out


def test_dry_run_json_shape(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: pytest.TempPathFactory,
    capsys: pytest.CaptureFixture[str],
) -> None:
    """Non-TTY without --apply in --json mode: correct JSON shape with would_write=False."""
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))

    bus_opened: list[bool] = []

    def _spy_open(args: object) -> FakeBus:
        bus_opened.append(True)
        bus = FakeBus()
        bus.open()
        return bus

    monkeypatch.setattr(calibrate, "_open_bus", _spy_open)
    monkeypatch.setattr(sys, "stdin", _NonTtyStringIO(""))

    args = _parse(["calibrate", "dry-arm", "--json"])
    calibrate.cmd_calibrate(args)

    assert bus_opened == [], "dry_run --json mode must not open the bus"
    assert not profiles_mod.profile_path(
        "dry-arm"
    ).exists(), "dry_run --json must not write a profile"

    out, _ = capsys.readouterr()
    data = json.loads(out)

    assert data["id"] == "dry-arm"
    assert data["would_write"] is False
    assert set(data["joints"]) == set(profiles_mod.JOINTS)
    assert data["poses"] == ["centered/rest", "minimum", "maximum"]
    assert "dry-arm" in data["path"]


# ---------------------------------------------------------------------------
# 7. EOF mid-capture raises CliError, no profile written
# ---------------------------------------------------------------------------


def test_eof_mid_capture_raises_env_error(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: pytest.TempPathFactory,
) -> None:
    """TTY stdin returning EOF on the 2nd pose raises CliError(EXIT_ENV_ERROR), no profile."""
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))

    fake = _make_seq_bus([{}, {}, {}])
    closed: list[bool] = []
    orig_close = fake.close

    def _spy_close() -> None:
        closed.append(True)
        orig_close()

    fake.close = _spy_close  # type: ignore[method-assign]
    monkeypatch.setattr(calibrate, "_open_bus", lambda args: fake)
    # Returns one line (1st pose OK), then EOF on 2nd readline
    monkeypatch.setattr(sys, "stdin", _TtyStringIO("\n"))

    args = _parse(["calibrate", "eof-arm"])
    with pytest.raises(CliError) as exc:
        calibrate.cmd_calibrate(args)

    assert exc.value.code == EXIT_ENV_ERROR
    # No profile must have been saved
    assert not profiles_mod.profile_path("eof-arm").exists(), "profile must not be written on EOF"
    # The bus must be closed via the finally block even when EOF aborts capture.
    assert closed == [True], "bus must be closed (finally) when EOF aborts capture"


# ---------------------------------------------------------------------------
# 8. Agent mode: non-TTY + --apply raises EXIT_USER_ERROR, no bus, no profile
# ---------------------------------------------------------------------------


def test_agent_apply_raises_user_error(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: pytest.TempPathFactory,
) -> None:
    """Non-TTY + --apply raises CliError(EXIT_USER_ERROR); no bus opened, no profile written."""
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))

    bus_opened: list[bool] = []

    def _spy_open(args: object) -> FakeBus:
        bus_opened.append(True)
        bus = FakeBus()
        bus.open()
        return bus

    monkeypatch.setattr(calibrate, "_open_bus", _spy_open)
    monkeypatch.setattr(sys, "stdin", _NonTtyStringIO(""))

    args = _parse(["calibrate", "agent-arm", "--apply"])
    with pytest.raises(CliError) as exc:
        calibrate.cmd_calibrate(args)

    assert exc.value.code == EXIT_USER_ERROR
    assert bus_opened == [], "agent mode must not open the bus"
    assert not profiles_mod.profile_path(
        "agent-arm"
    ).exists(), "agent mode must not write a profile"


# ---------------------------------------------------------------------------
# 9. TTY wins: --apply is ignored under a TTY (interactive capture proceeds)
# ---------------------------------------------------------------------------


def test_tty_apply_ignored_proceeds_interactive(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: pytest.TempPathFactory,
) -> None:
    """A TTY operator passing --apply still gets interactive capture (flag ignored, profile saved).

    Guards the ``resolve_consent`` TTY-wins rule at the handler level: if mode
    resolution ever regressed to treat ``--apply`` before ``isatty()``, a human
    at a terminal would be wrongly diverted to the agent-mode refusal.
    """
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))

    fake = _make_seq_bus([{}, {}, {}])
    monkeypatch.setattr(calibrate, "_open_bus", lambda args: fake)
    monkeypatch.setattr(sys, "stdin", _TtyStringIO("\n\n\n"))

    args = _parse(["calibrate", "tty-apply-arm", "--apply"])
    calibrate.cmd_calibrate(args)

    # TTY wins over --apply: interactive capture ran and saved the profile.
    assert profiles_mod.profile_path(
        "tty-apply-arm"
    ).exists(), "TTY --apply must still save (flag ignored under a TTY)"
