"""Tests for ``arm101 calibrate`` — interactive per-joint calibration verb.

TDD: this file was written before calibrate.py existed and drives the
implementation.  Tests cover:

* Missing ``id`` positional → SystemExit(1) via _CliArgumentParser
* Full capture → persist → reload loop against a FakeBus with sequenced reads
* CliError propagation when the bus/SDK is unavailable
* stdout/stderr split in both text and ``--json`` modes
* ``--json`` output shape
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
    monkeypatch.setattr(sys, "stdin", io.StringIO("\n\n\n"))

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
    monkeypatch.setattr(sys, "stdin", io.StringIO("\n\n\n"))

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
    monkeypatch.setattr(sys, "stdin", io.StringIO("\n\n\n"))

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
    monkeypatch.setattr(sys, "stdin", io.StringIO("\n\n\n"))

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
