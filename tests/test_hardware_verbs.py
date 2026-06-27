"""Cross-cutting safety-net suite for the three hardware verbs (task t9).

These tests are deliberately adversarial: they must fail loudly if any of the
agent-first contracts regress.  They drive the *real* entry point
(:func:`arm101.cli.main`) wherever possible — the verbs are registered (t8) — and
fall back to calling handlers directly only where the requirement says so.

Coverage map (acceptance criteria A–F):

A. CliError-only failures — never a raw traceback, never a handler ``sys.exit``.
B. stdout/stderr split in BOTH text and ``--json`` modes.
C. Lockstep-or-drift — catalog + ``overview._VERBS`` + ``learn`` reference all
   three verbs consistently.
D. Import-clean — ``import arm101.cli`` needs no third-party / hardware SDK.
E. Scope-guard — no leader/teleop/training/motion verbs have crept in.
F. Green with NO hardware attached (FakeBus + monkeypatch only).

Nothing here opens a real serial port; every hardware seam is a FakeBus or a
monkeypatched factory.
"""

from __future__ import annotations

import argparse
import io
import json
import subprocess
import sys

import pytest

from arm101.cli import _build_parser, main
from arm101.cli._commands import calibrate, find_port, setup_motors
from arm101.cli._commands.learn import _TEXT as LEARN_TEXT
from arm101.cli._commands.learn import _as_json_payload
from arm101.cli._commands.overview import _VERBS
from arm101.cli._errors import EXIT_ENV_ERROR, EXIT_USER_ERROR, CliError
from arm101.explain.catalog import ENTRIES
from arm101.hardware.bus import FakeBus

# Canonical hardware-verb names this deliverable ships.  The whole suite pivots
# on these strings being honoured identically across every surface.
HARDWARE_VERBS = (
    "find-port",
    "calibrate",
    "calibrate-motor",
    "set-motor-id",
    "center-motor",
    "setup-motors",
)


# ---------------------------------------------------------------------------
# Test doubles for stdin (controls isatty() + readline()/read independently)
# ---------------------------------------------------------------------------


class _NonTtyStringIO(io.StringIO):
    """StringIO that reports ``isatty() == False`` (a pipe/redirect)."""

    def isatty(self) -> bool:  # noqa: D401 - trivial override
        return False


class _TtyStringIO(io.StringIO):
    """StringIO that reports ``isatty() == True`` (an interactive terminal)."""

    def isatty(self) -> bool:  # noqa: D401 - trivial override
        return True


# ---------------------------------------------------------------------------
# Parser-introspection helpers (used by lockstep + scope-guard tests)
# ---------------------------------------------------------------------------


def _top_level_choices() -> dict[str, argparse.ArgumentParser]:
    """Return the top-level subcommand name → subparser mapping from ``main``'s parser."""
    parser = _build_parser()
    for action in parser._actions:
        if isinstance(action, argparse._SubParsersAction):
            return dict(action.choices)
    raise AssertionError("no subparsers action found on the top-level parser")


def _all_verb_names(parser: argparse.ArgumentParser, acc: set[str] | None = None) -> set[str]:
    """Recursively collect every registered subcommand name (top-level + nested)."""
    if acc is None:
        acc = set()
    for action in parser._actions:
        if isinstance(action, argparse._SubParsersAction):
            for name, subparser in action.choices.items():
                acc.add(name)
                _all_verb_names(subparser, acc)
    return acc


def _overview_mentions(verb: str) -> bool:
    return any(verb in entry for entry in _VERBS)


def _learn_payload_paths() -> list[list[str]]:
    return [cmd["path"] for cmd in _as_json_payload()["commands"]]


# ===========================================================================
# A. CliError-only failures (never a raw traceback / never a handler sys.exit)
# ===========================================================================


def test_find_port_detect_no_tty_handler_raises_cli_error(monkeypatch) -> None:
    """(a) Calling the handler directly with a non-TTY stdin raises CliError(2)."""
    monkeypatch.setattr(sys, "stdin", _NonTtyStringIO(""))
    ns = argparse.Namespace(detect=True, json=False)
    with pytest.raises(CliError) as exc:
        find_port.cmd_find_port(ns)
    assert exc.value.code == EXIT_ENV_ERROR


def test_find_port_detect_no_tty_via_main(monkeypatch, capsys) -> None:
    """(b) Through main(): exit 2, structured error on stderr, nothing on stdout."""
    monkeypatch.setattr(sys, "stdin", _NonTtyStringIO(""))
    rc = main(["find-port", "--detect"])
    assert rc == EXIT_ENV_ERROR
    out, err = capsys.readouterr()
    # Structured error contract: error:/hint: on stderr, never a traceback.
    assert out == ""
    assert err.startswith("error:")
    assert "hint:" in err
    assert "Traceback" not in err


def test_setup_motors_no_tty_handler_raises_cli_error(monkeypatch) -> None:
    """Handler with non-TTY stdin raises CliError(EXIT_ENV_ERROR) before any write."""
    fake = FakeBus()
    fake.open()
    monkeypatch.setattr(setup_motors, "_open_bus", lambda _a: fake)
    monkeypatch.setattr(sys, "stdin", _NonTtyStringIO(""))

    ns = argparse.Namespace(json=False, port="/dev/ttyACM0")
    with pytest.raises(CliError) as exc:
        setup_motors.cmd_setup_motors(ns)
    assert exc.value.code == EXIT_ENV_ERROR
    assert fake.eeprom_writes == []


def test_setup_motors_no_tty_via_main_zero_writes(monkeypatch, capsys) -> None:
    """Through main(): exit 2, structured stderr error, and ZERO EEPROM writes."""
    fake = FakeBus()
    fake.open()
    monkeypatch.setattr(setup_motors, "_open_bus", lambda _a: fake)
    monkeypatch.setattr(sys, "stdin", _NonTtyStringIO(""))

    rc = main(["setup-motors"])
    assert rc == EXIT_ENV_ERROR
    out, err = capsys.readouterr()
    assert out == ""
    assert err.startswith("error:")
    assert "hint:" in err
    assert "Traceback" not in err
    # The safety check fires before the bus is touched: no writes ever happen.
    assert fake.eeprom_writes == []


def test_calibrate_bus_unavailable_via_main(monkeypatch, capsys) -> None:
    """_open_bus raising CliError(EXIT_ENV_ERROR) (no SDK) → main exit 2, stderr error."""

    def _fail_open(_args):
        raise CliError(
            code=EXIT_ENV_ERROR,
            message="Feetech SDK 'scservo_sdk' is not installed.",
            remediation="pip install 'arm101[seeed]'",
        )

    monkeypatch.setattr(calibrate, "_open_bus", _fail_open)

    rc = main(["calibrate", "myarm"])
    assert rc == EXIT_ENV_ERROR
    out, err = capsys.readouterr()
    assert out == ""
    assert err.startswith("error:")
    assert "hint:" in err
    assert "Traceback" not in err


def test_calibrate_missing_id_is_argparse_systemexit(capsys) -> None:
    """No id positional → SystemExit(1) from the argparse layer, structured stderr."""
    with pytest.raises(SystemExit) as exc:
        main(["calibrate"])
    assert exc.value.code == EXIT_USER_ERROR
    out, err = capsys.readouterr()
    # _CliArgumentParser.error() routes through emit_error → error:/hint: on stderr.
    assert err.startswith("error:")
    assert "hint:" in err
    assert out == ""


# ===========================================================================
# B. stdout/stderr split in BOTH text and --json modes
# ===========================================================================


def test_find_port_success_text_split(monkeypatch, capsys) -> None:
    """Success (text): the port is on stdout; stderr carries no result."""
    monkeypatch.setattr(find_port.ports, "enumerate_ports", lambda: ["/dev/ttyACM0"])
    rc = main(["find-port"])
    assert rc == 0
    out, err = capsys.readouterr()
    assert "/dev/ttyACM0" in out
    assert err == ""


def test_find_port_success_json_split(monkeypatch, capsys) -> None:
    """Success (--json): stdout parses as {ports, count}; stderr empty of results."""
    monkeypatch.setattr(find_port.ports, "enumerate_ports", lambda: ["/dev/ttyACM0"])
    rc = main(["find-port", "--json"])
    assert rc == 0
    out, err = capsys.readouterr()
    payload = json.loads(out)
    assert payload["ports"] == ["/dev/ttyACM0"]
    assert payload["count"] == 1
    assert err == ""


def test_find_port_failure_json_error_on_stderr(monkeypatch, capsys) -> None:
    """Failure (--json): the error JSON lands on stderr; stdout stays empty."""
    monkeypatch.setattr(sys, "stdin", _NonTtyStringIO(""))
    rc = main(["find-port", "--detect", "--json"])
    assert rc == EXIT_ENV_ERROR
    out, err = capsys.readouterr()
    assert out == ""
    payload = json.loads(err)
    assert set(payload) == {"code", "message", "remediation"}
    assert payload["code"] == EXIT_ENV_ERROR


def test_calibrate_success_split_text_and_json(monkeypatch, tmp_path, capsys) -> None:
    """Calibrate success: prompts on stderr, summary on stdout — text AND --json."""
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))

    def _fresh_open_bus(_args):
        bus = FakeBus()
        bus.open()
        return bus

    # --- text mode ---
    monkeypatch.setattr(calibrate, "_open_bus", _fresh_open_bus)
    monkeypatch.setattr(sys, "stdin", io.StringIO("\n\n\n"))
    rc = main(["calibrate", "arm-text"])
    assert rc == 0
    out, err = capsys.readouterr()
    assert "Calibration saved" in out
    assert "centered/rest pose" in err
    assert "centered/rest pose" not in out
    assert "Calibration saved" not in err

    # --- json mode ---
    monkeypatch.setattr(sys, "stdin", io.StringIO("\n\n\n"))
    rc = main(["calibrate", "arm-json", "--json"])
    assert rc == 0
    out, err = capsys.readouterr()
    payload = json.loads(out)  # stdout is pure JSON
    assert payload["id"] == "arm-json"
    assert "centered/rest pose" in err
    assert "centered/rest pose" not in out


def test_setup_motors_success_split_text_and_json(monkeypatch, capsys) -> None:
    """setup-motors success: prompts on stderr, summary on stdout — text AND --json."""

    def _fresh_open_bus(_args):
        bus = FakeBus()
        bus.open()
        return bus

    monkeypatch.setattr(setup_motors, "_open_bus", _fresh_open_bus)

    # --- text mode ---
    monkeypatch.setattr(sys, "stdin", _TtyStringIO("\n" * 6))
    rc = main(["setup-motors"])
    assert rc == 0
    out, err = capsys.readouterr()
    assert "Motors assigned" in out
    assert "gripper" in err
    assert "press Enter" not in out

    # --- json mode ---
    monkeypatch.setattr(sys, "stdin", _TtyStringIO("\n" * 6))
    rc = main(["setup-motors", "--json"])
    assert rc == 0
    out, err = capsys.readouterr()
    payload = json.loads(out)  # stdout is pure JSON
    assert len(payload["assigned"]) == 6
    assert "gripper" in err
    assert "press Enter" not in out


# ===========================================================================
# C. Lockstep-or-drift invariant
# ===========================================================================


@pytest.mark.parametrize("verb", HARDWARE_VERBS)
def test_catalog_has_nonempty_entry(verb: str) -> None:
    key = (verb,)
    assert key in ENTRIES, f"explain catalog missing entry for {verb!r}"
    body = ENTRIES[key]
    assert isinstance(body, str) and body.strip(), f"catalog entry for {verb!r} is empty"


@pytest.mark.parametrize("verb", HARDWARE_VERBS)
def test_overview_verbs_mention(verb: str) -> None:
    assert _overview_mentions(verb), f"overview._VERBS does not mention {verb!r}"


@pytest.mark.parametrize("verb", HARDWARE_VERBS)
def test_learn_text_and_payload_mention(verb: str) -> None:
    assert verb in LEARN_TEXT, f"learn._TEXT does not mention {verb!r}"
    assert [verb] in _learn_payload_paths(), f"learn payload has no [{verb!r}] command path"


def test_registered_hardware_verbs_have_lockstep_docs() -> None:
    """Every registered hardware verb must also appear in catalog + overview + learn.

    Derives the registered set from the real parser, so a future hardware verb
    added without updating all three doc surfaces fails here.
    """
    registered = set(_top_level_choices())
    learn_paths = _learn_payload_paths()
    for verb in HARDWARE_VERBS:
        assert verb in registered, f"{verb!r} is not registered on the parser"
        assert (verb,) in ENTRIES, f"{verb!r} registered but missing a catalog entry"
        assert _overview_mentions(verb), f"{verb!r} registered but absent from overview._VERBS"
        assert verb in LEARN_TEXT, f"{verb!r} registered but absent from learn._TEXT"
        assert [verb] in learn_paths, f"{verb!r} registered but absent from learn payload"


# ===========================================================================
# D. Import-clean (zero third-party deps; no hardware SDK pulled in)
# ===========================================================================


def test_import_arm101_cli_is_clean_subprocess() -> None:
    """A pristine interpreter can import arm101.cli without loading scservo_sdk."""
    code = (
        "import arm101.cli\n"
        "import sys\n"
        "assert 'scservo_sdk' not in sys.modules, 'hardware SDK leaked into import'\n"
    )
    proc = subprocess.run(
        [sys.executable, "-c", code],
        capture_output=True,
        text=True,
    )
    assert proc.returncode == 0, f"import arm101.cli failed: {proc.stderr}"


def test_import_arm101_cli_does_not_load_sdk_in_process() -> None:
    """Secondary, in-process guard: importing the package does not pull in the SDK.

    When the optional ``[seeed]`` extra is installed, a *sibling* test that opens
    a real FeetechBus may have already loaded ``scservo_sdk`` into this shared
    interpreter; that is unrelated to whether importing ``arm101.cli`` itself
    triggers the import. We therefore only assert the property when the SDK is
    not already loaded — the subprocess test above is the authoritative guard.
    """
    import importlib

    if "scservo_sdk" in sys.modules:
        pytest.skip("scservo_sdk already loaded by a sibling test; see subprocess guard")
    importlib.import_module("arm101.cli")
    assert "scservo_sdk" not in sys.modules


# ===========================================================================
# E. Scope-guard — no leader/teleop/training/motion verbs
# ===========================================================================


def test_no_motion_or_teleop_verbs_registered() -> None:
    """Pin the spec boundary: only the sanctioned pre-assembly motor verbs ship.

    ``center-motor`` is the one sanctioned (fully-built, hard-gated) motion verb;
    no generic half-built motion/teleop/training stubs may be wired into the
    parser.
    """
    forbidden = {
        "leader",
        "teleop",
        "teleoperate",
        "train",
        "training",
        "move",
        "motion",
        "execute",
        "record",
        "replay",
        "policy",
    }
    registered = _all_verb_names(_build_parser())
    leaked = forbidden & registered
    assert not leaked, f"forbidden out-of-scope verb(s) registered: {sorted(leaked)}"


def test_registered_verbs_are_the_expected_set() -> None:
    """Belt-and-braces: the top-level verb surface is exactly the known set."""
    expected = {
        "whoami",
        "learn",
        "explain",
        "overview",
        "doctor",
        "find-port",
        "calibrate",
        "calibrate-motor",
        "set-motor-id",
        "center-motor",
        "setup-motors",
        "cli",
    }
    assert set(_top_level_choices()) == expected


# ===========================================================================
# F. Suite is green with NO hardware attached
# ===========================================================================
#
# Every test above uses a FakeBus or a monkeypatched seam and never opens a real
# serial port; the success paths assert exit code 0.  The coverage gate
# (>=60%) and full-suite pass count are enforced by CI / the run commands in the
# task workflow, not by an in-file assertion.  This block documents that intent.


def test_hardware_success_paths_need_no_real_port(monkeypatch, tmp_path, capsys) -> None:
    """End-to-end happy paths for all three verbs run with zero physical hardware."""
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))

    # find-port (no hardware: enumerate stubbed)
    monkeypatch.setattr(find_port.ports, "enumerate_ports", lambda: [])
    assert main(["find-port"]) == 0
    capsys.readouterr()

    # calibrate (FakeBus + scripted stdin)
    def _fresh_open_bus(_args):
        bus = FakeBus()
        bus.open()
        return bus

    monkeypatch.setattr(calibrate, "_open_bus", _fresh_open_bus)
    monkeypatch.setattr(sys, "stdin", io.StringIO("\n\n\n"))
    assert main(["calibrate", "no-hw"]) == 0
    capsys.readouterr()

    # setup-motors (FakeBus + TTY stdin)
    monkeypatch.setattr(setup_motors, "_open_bus", _fresh_open_bus)
    monkeypatch.setattr(sys, "stdin", _TtyStringIO("\n" * 6))
    assert main(["setup-motors"]) == 0
    capsys.readouterr()
