"""Tests for ``arm101 calibrate-motor`` — single-motor identify + catalog.

Drives the verb entirely against a :class:`~arm101.hardware.bus.FakeBus` and a
monkeypatched port-detection seam; no hardware is touched.  Covers detection
(skip busy / non-motor ports, verify-Feetech, single-motor enforcement), the
manual and automatic flows, the stdout/stderr split, ``--json`` shape, and the
EOF/interactive guard.
"""

from __future__ import annotations

import argparse
import json
import sys

import pytest

from arm101.cli._commands import calibrate_motor as cm
from arm101.cli._errors import EXIT_ENV_ERROR, EXIT_USER_ERROR, CliError
from arm101.hardware import motor_catalog
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

    def isatty(self) -> bool:  # pragma: no cover - not relied upon by the verb
        return True


def _args(**kw) -> argparse.Namespace:
    ns = argparse.Namespace(label=None, auto=False, port=None, json=False)
    for key, value in kw.items():
        setattr(ns, key, value)
    return ns


def _patch_single_port(monkeypatch, bus: FakeBus) -> None:
    """One candidate port, opening to *bus*."""
    monkeypatch.setattr(cm, "_candidate_ports", lambda: ["/dev/ttyACM1"])
    monkeypatch.setattr(cm, "_open_bus", lambda port: bus)


def _xdg(monkeypatch, tmp_path) -> None:
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))


# ---------------------------------------------------------------------------
# Manual mode — happy path, persistence, output split, --json
# ---------------------------------------------------------------------------


def test_manual_registers_and_persists(monkeypatch, tmp_path, capsys) -> None:
    _xdg(monkeypatch, tmp_path)
    bus = FakeBus(positions={1: 3939}, ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    # servo (blank -> default), gear, joint
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["\n", "1:191\n", "shoulder_pan\n"]))

    cm.cmd_calibrate_motor(_args(label="F1"))

    out = capsys.readouterr()
    # Result on stdout; prompts + motor snapshot on stderr.
    assert "Registered F1" in out.out
    assert "Detected motor" in out.err
    assert "Registered F1" not in out.err

    catalog = motor_catalog.load_catalog()
    assert "F1" in catalog
    entry = catalog["F1"]
    assert entry.gear_ratio == "1:191"
    assert entry.joint == "shoulder_pan"
    assert entry.servo_model == "STS3215 (model 777)"  # detected default
    assert entry.detected_id == 1
    assert entry.detected_model == 777
    assert entry.recorded  # date stamped


def test_manual_json_shape(monkeypatch, tmp_path, capsys) -> None:
    _xdg(monkeypatch, tmp_path)
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["ST-3215-C044\n", "1:345\n", "L1\n"]))

    cm.cmd_calibrate_motor(_args(label="L2", json=True))

    out = capsys.readouterr()
    payload = json.loads(out.out)
    assert payload["motors"][0]["label"] == "L2"
    assert payload["motors"][0]["servo_model"] == "ST-3215-C044"
    assert payload["motors"][0]["gear_ratio"] == "1:345"
    assert payload["motors"][0]["detected"]["model"] == 777
    assert payload["catalog"].endswith("motors.json")
    assert out.err  # snapshot/prompts went to stderr, not stdout


def test_label_prompted_when_omitted(monkeypatch, tmp_path, capsys) -> None:
    _xdg(monkeypatch, tmp_path)
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    # label, servo(default), gear, joint
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["F3\n", "\n", "1:191\n", "elbow_flex\n"]))

    cm.cmd_calibrate_motor(_args())  # no label positional

    assert "F3" in motor_catalog.load_catalog()


# ---------------------------------------------------------------------------
# Detection: verify-Feetech, single motor, multiple ports, busy ports
# ---------------------------------------------------------------------------


def test_non_feetech_model_is_rejected(monkeypatch, tmp_path) -> None:
    """A responding servo whose model != 777 is not catalogued (verify-Feetech)."""
    _xdg(monkeypatch, tmp_path)
    bus = FakeBus(ids=[1], info={1: {"model": 999}})
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["\n", "1:191\n", "x\n"]))

    with pytest.raises(CliError) as exc:
        cm.cmd_calibrate_motor(_args(label="F1"))
    assert exc.value.code == EXIT_ENV_ERROR  # "no STS3215 detected"


def test_no_motor_detected_raises_env_error(monkeypatch, tmp_path) -> None:
    _xdg(monkeypatch, tmp_path)
    bus = FakeBus(ids=[])  # nothing responds
    bus.open()
    _patch_single_port(monkeypatch, bus)

    with pytest.raises(CliError) as exc:
        cm.cmd_calibrate_motor(_args(label="F1"))
    assert exc.value.code == EXIT_ENV_ERROR


def test_more_than_one_motor_on_port_rejected(monkeypatch, tmp_path) -> None:
    _xdg(monkeypatch, tmp_path)
    bus = FakeBus(ids=[1, 2])  # two servos on one bus
    bus.open()
    _patch_single_port(monkeypatch, bus)

    with pytest.raises(CliError) as exc:
        cm.cmd_calibrate_motor(_args(label="F1"))
    assert exc.value.code == EXIT_USER_ERROR  # connect one at a time


def test_multiple_ports_with_motors_is_ambiguous(monkeypatch, tmp_path) -> None:
    _xdg(monkeypatch, tmp_path)
    monkeypatch.setattr(cm, "_candidate_ports", lambda: ["/dev/a", "/dev/b"])

    def _open(port):
        b = FakeBus(ids=[1])
        b.open()
        return b

    monkeypatch.setattr(cm, "_open_bus", _open)
    with pytest.raises(CliError) as exc:
        cm.cmd_calibrate_motor(_args(label="F1"))
    assert exc.value.code == EXIT_USER_ERROR


def test_busy_port_is_skipped(monkeypatch, tmp_path, capsys) -> None:
    """A port that fails to open (e.g. held by another robot) is skipped."""
    _xdg(monkeypatch, tmp_path)
    good = FakeBus(ids=[1])
    good.open()
    monkeypatch.setattr(cm, "_candidate_ports", lambda: ["/dev/busy", "/dev/ttyACM1"])

    def _open(port):
        if port == "/dev/busy":
            raise CliError(EXIT_ENV_ERROR, "Device or resource busy", "")
        return good

    monkeypatch.setattr(cm, "_open_bus", _open)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["\n", "1:191\n", "gripper\n"]))

    cm.cmd_calibrate_motor(_args(label="F6"))
    assert "F6" in motor_catalog.load_catalog()  # detection succeeded past the busy port


# ---------------------------------------------------------------------------
# Interactive guard + automatic mode
# ---------------------------------------------------------------------------


def test_eof_on_stdin_raises_env_error(monkeypatch, tmp_path) -> None:
    _xdg(monkeypatch, tmp_path)
    bus = FakeBus(ids=[1])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([]))  # immediate EOF at first prompt

    with pytest.raises(CliError) as exc:
        cm.cmd_calibrate_motor(_args(label="F1"))
    assert exc.value.code == EXIT_ENV_ERROR


def test_auto_mode_walks_then_quits(monkeypatch, tmp_path, capsys) -> None:
    _xdg(monkeypatch, tmp_path)

    def _open(port):
        b = FakeBus(ids=[1])
        b.open()
        return b

    monkeypatch.setattr(cm, "_candidate_ports", lambda: ["/dev/ttyACM1"])
    monkeypatch.setattr(cm, "_open_bus", _open)
    # F1: connect(Enter), servo(default), gear, joint ; then F2 connect -> 'q'
    monkeypatch.setattr(
        sys,
        "stdin",
        _FakeStdin(["\n", "\n", "1:191\n", "shoulder_pan\n", "q\n"]),
    )

    cm.cmd_calibrate_motor(_args(auto=True))

    catalog = motor_catalog.load_catalog()
    assert "F1" in catalog
    assert "F2" not in catalog  # quit before F2
    assert "Connect the F1 motor" in capsys.readouterr().err


def test_register_never_raises_systemexit(monkeypatch, tmp_path) -> None:
    """Failures surface as CliError, never a bare SystemExit / traceback."""
    _xdg(monkeypatch, tmp_path)
    bus = FakeBus(ids=[])
    bus.open()
    _patch_single_port(monkeypatch, bus)
    try:
        cm.cmd_calibrate_motor(_args(label="F1"))
    except CliError:
        pass
    except SystemExit:  # pragma: no cover
        pytest.fail("calibrate-motor raised SystemExit instead of CliError")
