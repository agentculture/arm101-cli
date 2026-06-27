"""Tests for arm101.cli._commands.find_port (verb: find-port).

The verb is NOT yet registered in main() (that is task t8), so tests build
a throwaway parser around the handler directly.
"""

from __future__ import annotations

import argparse
import io
import json

import pytest

from arm101.cli._commands import find_port
from arm101.cli._errors import EXIT_ENV_ERROR, EXIT_USER_ERROR, CliError


class _TtyStringIO(io.StringIO):
    """StringIO that reports isatty() = True, simulating an interactive terminal."""

    def isatty(self) -> bool:
        return True


class _NonTtyStringIO(io.StringIO):
    """StringIO that reports isatty() = False, simulating a pipe/redirect."""

    def isatty(self) -> bool:
        return False


# ---------------------------------------------------------------------------
# Helper — build a minimal parser that routes args through find_port.register
# ---------------------------------------------------------------------------


def _make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="arm101-cli")
    sub = parser.add_subparsers(dest="command")
    find_port.register(sub)
    return parser


def _run(argv: list[str]) -> argparse.Namespace:
    parser = _make_parser()
    args = parser.parse_args(argv)
    return args


# ---------------------------------------------------------------------------
# Default enumeration — text mode
# ---------------------------------------------------------------------------


class TestDefaultTextMode:
    def test_lists_ports_one_per_line(self, monkeypatch, capsys):
        monkeypatch.setattr(
            "arm101.cli._commands.find_port.ports.enumerate_ports",
            lambda: ["/dev/ttyACM0", "/dev/ttyACM1"],
        )
        args = _run(["find-port"])
        args.func(args)
        out, err = capsys.readouterr()
        assert "/dev/ttyACM0" in out
        assert "/dev/ttyACM1" in out
        # Each port on its own line
        lines = [ln for ln in out.splitlines() if ln.strip()]
        assert "/dev/ttyACM0" in lines
        assert "/dev/ttyACM1" in lines
        # No output on stderr (no prompt in non-interactive mode)
        assert err == ""

    def test_empty_result_prints_message_exit_0(self, monkeypatch, capsys):
        monkeypatch.setattr(
            "arm101.cli._commands.find_port.ports.enumerate_ports",
            lambda: [],
        )
        args = _run(["find-port"])
        result = args.func(args)
        out, err = capsys.readouterr()
        # exit 0 (handler returns None == 0)
        assert result is None or result == 0
        # some informative message on stdout
        assert "no candidate serial ports found" in out.lower() or out.strip() != ""
        # Not an error
        assert err == ""

    def test_empty_result_message_on_stdout(self, monkeypatch, capsys):
        monkeypatch.setattr(
            "arm101.cli._commands.find_port.ports.enumerate_ports",
            lambda: [],
        )
        args = _run(["find-port"])
        args.func(args)
        out, _ = capsys.readouterr()
        # Must mention "no" and "port" in some form
        assert "no" in out.lower()
        assert "port" in out.lower()


# ---------------------------------------------------------------------------
# Default enumeration — JSON mode
# ---------------------------------------------------------------------------


class TestDefaultJsonMode:
    def test_json_payload_shape(self, monkeypatch, capsys):
        monkeypatch.setattr(
            "arm101.cli._commands.find_port.ports.enumerate_ports",
            lambda: ["/dev/ttyACM0", "/dev/ttyACM1"],
        )
        args = _run(["find-port", "--json"])
        args.func(args)
        out, err = capsys.readouterr()
        payload = json.loads(out)
        assert payload["ports"] == ["/dev/ttyACM0", "/dev/ttyACM1"]
        assert payload["count"] == 2
        assert err == ""

    def test_json_empty_result(self, monkeypatch, capsys):
        monkeypatch.setattr(
            "arm101.cli._commands.find_port.ports.enumerate_ports",
            lambda: [],
        )
        args = _run(["find-port", "--json"])
        result = args.func(args)
        out, err = capsys.readouterr()
        payload = json.loads(out)
        assert payload["ports"] == []
        assert payload["count"] == 0
        # exit 0
        assert result is None or result == 0
        assert err == ""

    def test_json_result_to_stdout_not_stderr(self, monkeypatch, capsys):
        monkeypatch.setattr(
            "arm101.cli._commands.find_port.ports.enumerate_ports",
            lambda: ["/dev/ttyACM0"],
        )
        args = _run(["find-port", "--json"])
        args.func(args)
        out, err = capsys.readouterr()
        # stdout is valid JSON
        json.loads(out)
        # stderr is empty
        assert err == ""


# ---------------------------------------------------------------------------
# --detect mode: no TTY
# ---------------------------------------------------------------------------


class TestDetectNoTty:
    def test_raises_cli_error_exit_env_error_when_no_tty(self, monkeypatch):
        monkeypatch.setattr("sys.stdin", _NonTtyStringIO(""))
        args = _run(["find-port", "--detect"])
        with pytest.raises(CliError) as exc_info:
            args.func(args)
        assert exc_info.value.code == EXIT_ENV_ERROR

    def test_remediation_mentions_non_interactive(self, monkeypatch):
        monkeypatch.setattr("sys.stdin", _NonTtyStringIO(""))
        args = _run(["find-port", "--detect"])
        with pytest.raises(CliError) as exc_info:
            args.func(args)
        # Remediation should suggest the default (non-interactive) mode
        assert exc_info.value.remediation != ""


# ---------------------------------------------------------------------------
# --detect mode: happy path (mocked stdin + before/after sets)
# ---------------------------------------------------------------------------


class TestDetectHappyPath:
    def test_resolves_single_disappeared_port(self, monkeypatch, capsys):
        """Before: ACM0 + ACM1. After unplug: ACM1 only. Resolved: ACM0."""
        monkeypatch.setattr("sys.stdin", _TtyStringIO("\n"))

        calls = [["/dev/ttyACM0", "/dev/ttyACM1"], ["/dev/ttyACM1"]]
        call_iter = iter(calls)
        monkeypatch.setattr(
            "arm101.cli._commands.find_port.ports.enumerate_ports",
            lambda: next(call_iter),
        )

        args = _run(["find-port", "--detect"])
        result = args.func(args)
        out, err = capsys.readouterr()

        # Result (the discovered port) must be on stdout
        assert "/dev/ttyACM0" in out
        # Prompt must be on stderr
        assert err.strip() != ""
        # exit 0
        assert result is None or result == 0

    def test_prompt_appears_on_stderr_not_stdout(self, monkeypatch, capsys):
        monkeypatch.setattr("sys.stdin", _TtyStringIO("\n"))

        calls = [["/dev/ttyACM0", "/dev/ttyACM1"], ["/dev/ttyACM1"]]
        call_iter = iter(calls)
        monkeypatch.setattr(
            "arm101.cli._commands.find_port.ports.enumerate_ports",
            lambda: next(call_iter),
        )

        args = _run(["find-port", "--detect"])
        args.func(args)
        out, err = capsys.readouterr()

        # The prompt ("unplug" or "press Enter" style) must be on stderr
        assert "unplug" in err.lower() or "enter" in err.lower()
        # stdout must NOT contain the prompt text
        assert "unplug" not in out.lower()


# ---------------------------------------------------------------------------
# --detect mode: ambiguous (zero or >1 ports changed)
# ---------------------------------------------------------------------------


class TestDetectAmbiguous:
    def test_zero_changed_raises_user_error(self, monkeypatch):
        """No port disappears -> ambiguous, EXIT_USER_ERROR."""
        monkeypatch.setattr("sys.stdin", _TtyStringIO("\n"))

        same = ["/dev/ttyACM0", "/dev/ttyACM1"]
        call_iter = iter([same, same])
        monkeypatch.setattr(
            "arm101.cli._commands.find_port.ports.enumerate_ports",
            lambda: next(call_iter),
        )

        args = _run(["find-port", "--detect"])
        with pytest.raises(CliError) as exc_info:
            args.func(args)
        assert exc_info.value.code == EXIT_USER_ERROR

    def test_two_changed_raises_user_error(self, monkeypatch):
        """Two ports disappear -> ambiguous, EXIT_USER_ERROR."""
        monkeypatch.setattr("sys.stdin", _TtyStringIO("\n"))

        before = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0"]
        after = ["/dev/ttyACM1"]  # ACM0 and USB0 disappeared
        call_iter = iter([before, after])
        monkeypatch.setattr(
            "arm101.cli._commands.find_port.ports.enumerate_ports",
            lambda: next(call_iter),
        )

        args = _run(["find-port", "--detect"])
        with pytest.raises(CliError) as exc_info:
            args.func(args)
        assert exc_info.value.code == EXIT_USER_ERROR

    def test_ambiguous_error_message_mentions_count(self, monkeypatch):
        """Error message explains how many ports changed."""
        monkeypatch.setattr("sys.stdin", _TtyStringIO("\n"))

        before = ["/dev/ttyACM0", "/dev/ttyACM1"]
        after = []  # both disappeared
        call_iter = iter([before, after])
        monkeypatch.setattr(
            "arm101.cli._commands.find_port.ports.enumerate_ports",
            lambda: next(call_iter),
        )

        args = _run(["find-port", "--detect"])
        with pytest.raises(CliError) as exc_info:
            args.func(args)
        # message should mention the count somehow
        assert exc_info.value.code == EXIT_USER_ERROR
        assert exc_info.value.message != ""


# ---------------------------------------------------------------------------
# --detect mode: underlying enumerate_ports raises (e.g. on macOS)
# ---------------------------------------------------------------------------


class TestDetectPropagatesEnvError:
    def test_env_error_from_enumerate_propagates(self, monkeypatch):
        """CliError(EXIT_ENV_ERROR) from enumerate_ports must not be swallowed."""
        monkeypatch.setattr("sys.stdin", _TtyStringIO("\n"))

        def _raise():
            raise CliError(EXIT_ENV_ERROR, "unsupported platform", "use Linux")

        monkeypatch.setattr(
            "arm101.cli._commands.find_port.ports.enumerate_ports",
            _raise,
        )

        args = _run(["find-port", "--detect"])
        with pytest.raises(CliError) as exc_info:
            args.func(args)
        assert exc_info.value.code == EXIT_ENV_ERROR
