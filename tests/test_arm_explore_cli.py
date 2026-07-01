"""Tests for ``arm explore`` — the t9 CLI verb that drives the reachability
flood-fill :func:`arm101.explore.engine.explore` and renders its result.

All hardware is a :class:`~arm101.hardware.bus.FakeBus` injected through the
``arm._open_bus`` / ``arm._candidate_ports`` seam (the same seam the ``read`` /
``flex`` handlers use), so no serial port is ever opened. Runs are kept fast by
capping ``--max-moves`` low.

Coverage (plan targets c1, c2, c7, h6, h7, h12):
- registration: ``arm explore`` under the arm noun, with its flags/defaults.
- motion gating (mirrors flex): non-TTY without ``--apply`` -> dry-run plan,
  zero motion, zero bus; non-TTY ``--apply`` -> runs; TTY -> prompt (yes runs /
  no aborts).
- artifacts: the run writes BOTH the JSONL event log AND the compact map, and
  prints the map path (text AND ``--json``).
- ``--port`` and ``--map`` are honored; the live thermal guard halts a run.
- a simulated bus failure raises a structured :class:`CliError` (no traceback).
- the ``arm flex`` handler is untouched (behavioral smoke test).
"""

from __future__ import annotations

import argparse
import json
import sys

import pytest

from arm101.cli._commands import arm as arm_cmd
from arm101.cli._errors import EXIT_ENV_ERROR, CliError
from arm101.hardware.bus import FakeBus

# ---------------------------------------------------------------------------
# Test doubles + helpers
# ---------------------------------------------------------------------------


class _FakeStdin:
    """Scripted stdin controlling ``isatty()`` and ``readline()``."""

    def __init__(self, lines: "list[str]", tty: bool = True) -> None:
        self._lines = list(lines)
        self._tty = tty

    def isatty(self) -> bool:
        return self._tty

    def readline(self) -> str:
        return self._lines.pop(0) if self._lines else ""  # "" == EOF


class _FlakyReadBus(FakeBus):
    """FakeBus whose ``read_info`` permanently raises for chosen motor ids."""

    def __init__(self, *args, fail_ids: "set[int] | None" = None, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._fail_ids = set(fail_ids or set())

    def read_info(self, motor: int) -> "dict[str, int]":
        if motor in self._fail_ids:
            raise CliError(
                code=EXIT_ENV_ERROR,
                message=f"simulated read failure for motor {motor}",
                remediation="retry",
            )
        return super().read_info(motor)


def _patch_bus(monkeypatch, fake: FakeBus, port: str = "/dev/ttyACM_fake") -> None:
    """Patch the arm bus seam so it opens *fake* at *port*."""
    fake.open()
    monkeypatch.setattr(arm_cmd, "_candidate_ports", lambda: [port])
    monkeypatch.setattr(arm_cmd, "_open_bus", lambda _p: fake)


def _explore_args(
    role: str = "follower",
    port: "str | None" = None,
    map: "str | None" = None,
    threshold: "int | None" = None,
    max_moves: "int | None" = 5,
    resolution: "int | None" = 512,
    apply: bool = False,
    json_mode: bool = False,
):
    return argparse.Namespace(
        role=role,
        port=port,
        map=map,
        threshold=threshold,
        max_moves=max_moves,
        resolution=resolution,
        apply=apply,
        json=json_mode,
    )


def _flex_args(
    joint=None,
    to=None,
    demo=False,
    gentle=False,
    threshold=None,
    role="follower",
    port=None,
    apply=False,
    json_mode=False,
):
    return argparse.Namespace(
        joint=joint,
        to=to,
        demo=demo,
        gentle=gentle,
        threshold=threshold,
        role=role,
        port=port,
        apply=apply,
        json=json_mode,
    )


# ===========================================================================
# Registration
# ===========================================================================


def test_register_explore_verb() -> None:
    top = argparse.ArgumentParser(prog="arm101")
    sub = top.add_subparsers()
    arm_cmd.register(sub)

    args = top.parse_args(["arm", "explore", "--apply"])
    assert args.func is arm_cmd.cmd_arm_explore
    assert args.role == "follower"
    assert args.apply is True
    # documented safe defaults for the open-question knobs
    assert args.threshold is None
    assert args.max_moves is None
    assert args.resolution is None


def test_register_explore_all_flags() -> None:
    top = argparse.ArgumentParser(prog="arm101")
    sub = top.add_subparsers()
    arm_cmd.register(sub)

    args = top.parse_args(
        [
            "arm",
            "explore",
            "--role",
            "leader",
            "--port",
            "/dev/ttyACM3",
            "--map",
            "/tmp/x.map.json",
            "--threshold",
            "300",
            "--max-moves",
            "10",
            "--resolution",
            "256",
            "--json",
        ]
    )
    assert args.role == "leader"
    assert args.port == "/dev/ttyACM3"
    assert args.map == "/tmp/x.map.json"
    assert args.threshold == 300
    assert args.max_moves == 10
    assert args.resolution == 256
    assert args.json is True


# ===========================================================================
# Motion gating — mirrors flex
# ===========================================================================


def test_explore_dry_run_aborts_no_motion(monkeypatch, capsys, tmp_path) -> None:
    """Non-TTY without --apply: a plan only — zero motion, zero bus access."""
    opened = {"v": False}

    def _open(_p):
        opened["v"] = True
        raise AssertionError("dry-run must not open a bus")

    monkeypatch.setattr(arm_cmd, "_candidate_ports", lambda: ["/dev/ttyACM_fake"])
    monkeypatch.setattr(arm_cmd, "_open_bus", _open)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    map_path = tmp_path / "m.map.json"
    arm_cmd.cmd_arm_explore(_explore_args(map=str(map_path), apply=False))

    out = capsys.readouterr().out
    assert "Dry-run" in out
    assert opened["v"] is False, "dry-run must not open a bus"
    assert not map_path.exists(), "dry-run must not write the map"


def test_explore_dry_run_json(monkeypatch, capsys, tmp_path) -> None:
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))
    map_path = tmp_path / "m.map.json"

    arm_cmd.cmd_arm_explore(_explore_args(map=str(map_path), apply=False, json_mode=True))

    plan = json.loads(capsys.readouterr().out)["plan"]
    assert plan["role"] == "follower"
    assert plan["map_path"] == str(map_path)
    assert plan["log_path"].endswith(".events.jsonl")


def test_explore_apply_runs_writes_artifacts(monkeypatch, capsys, tmp_path) -> None:
    """Non-TTY with --apply: runs the engine, writing BOTH log + map, prints map path."""
    fake = FakeBus(positions={i: 2048 for i in range(1, 7)})
    _patch_bus(monkeypatch, fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    map_path = tmp_path / "explore.map.json"
    log_path = tmp_path / "explore.events.jsonl"

    arm_cmd.cmd_arm_explore(_explore_args(map=str(map_path), apply=True, max_moves=5))

    out = capsys.readouterr().out
    assert str(map_path) in out, "the map path must be printed"
    assert map_path.exists(), "the compact map must be written"
    assert log_path.exists(), "the JSONL event log must be written"
    assert len(fake.position_writes) > 0, "motion must have been commanded"
    assert fake._open is False, "bus must be closed in finally"


def test_explore_apply_json_shape(monkeypatch, capsys, tmp_path) -> None:
    fake = FakeBus(positions={i: 2048 for i in range(1, 7)})
    _patch_bus(monkeypatch, fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    map_path = tmp_path / "explore.map.json"
    arm_cmd.cmd_arm_explore(
        _explore_args(map=str(map_path), apply=True, max_moves=5, json_mode=True)
    )

    payload = json.loads(capsys.readouterr().out)
    assert payload["role"] == "follower"
    assert payload["port"] == "/dev/ttyACM_fake"
    assert payload["map_path"] == str(map_path)
    assert payload["log_path"].endswith(".events.jsonl")
    for key in (
        "cells_visited",
        "moves",
        "reachable",
        "contacts",
        "escapes_attempted",
        "escapes_succeeded",
        "budget_bounded",
    ):
        assert key in payload, f"{key} missing from explore JSON payload"
    assert isinstance(payload["moves"], int)


def test_explore_interactive_yes_runs(monkeypatch, tmp_path) -> None:
    fake = FakeBus(positions={i: 2048 for i in range(1, 7)})
    _patch_bus(monkeypatch, fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["yes\n"], tty=True))

    map_path = tmp_path / "explore.map.json"
    arm_cmd.cmd_arm_explore(_explore_args(map=str(map_path), max_moves=5))

    assert map_path.exists()
    assert len(fake.position_writes) > 0


def test_explore_interactive_no_aborts(monkeypatch, capsys, tmp_path) -> None:
    fake = FakeBus(positions={i: 2048 for i in range(1, 7)})
    opened = {"v": False}

    def _open(_p):
        opened["v"] = True
        fake.open()
        return fake

    monkeypatch.setattr(arm_cmd, "_candidate_ports", lambda: ["/dev/ttyACM_fake"])
    monkeypatch.setattr(arm_cmd, "_open_bus", _open)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["no\n"], tty=True))

    map_path = tmp_path / "explore.map.json"
    arm_cmd.cmd_arm_explore(_explore_args(map=str(map_path)))

    out = capsys.readouterr().out
    assert "Aborted" in out
    assert opened["v"] is False, "a declined prompt must not open a bus"
    assert not map_path.exists()


# ===========================================================================
# --port / --map honored
# ===========================================================================


def test_explore_honors_explicit_port(monkeypatch, tmp_path) -> None:
    fake = FakeBus(positions={i: 2048 for i in range(1, 7)})
    fake.open()
    seen: "dict[str, str]" = {}

    def _open(port: str) -> FakeBus:
        seen["port"] = port
        return fake

    monkeypatch.setattr(arm_cmd, "_open_bus", _open)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    map_path = tmp_path / "explore.map.json"
    arm_cmd.cmd_arm_explore(
        _explore_args(map=str(map_path), port="/dev/ttyACM7", apply=True, max_moves=3)
    )

    assert seen["port"] == "/dev/ttyACM7"


def test_explore_default_map_path_uses_role(monkeypatch, capsys, tmp_path) -> None:
    """Without --map, the map path defaults to ``./arm-explore-<role>.map.json``."""
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))
    monkeypatch.chdir(tmp_path)

    arm_cmd.cmd_arm_explore(_explore_args(map=None, apply=False, json_mode=True))

    plan = json.loads(capsys.readouterr().out)["plan"]
    assert plan["map_path"].endswith("arm-explore-follower.map.json")
    assert plan["log_path"].endswith("arm-explore-follower.events.jsonl")


# ===========================================================================
# Live thermal guard halts a run
# ===========================================================================


def test_explore_thermal_guard_halts(monkeypatch, capsys, tmp_path) -> None:
    """An over-temperature joint halts the run immediately (budget-bounded, 0 moves)."""
    hot = {i: {"present_temperature": 99} for i in range(1, 7)}
    fake = FakeBus(positions={i: 2048 for i in range(1, 7)}, info=hot)
    _patch_bus(monkeypatch, fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    map_path = tmp_path / "explore.map.json"
    arm_cmd.cmd_arm_explore(
        _explore_args(map=str(map_path), apply=True, max_moves=100, json_mode=True)
    )

    payload = json.loads(capsys.readouterr().out)
    assert payload["budget_bounded"] is True
    assert payload["moves"] == 0
    assert fake.position_writes == [], "thermal halt must command no motion"
    assert map_path.exists()


# ===========================================================================
# Structured failure — no traceback
# ===========================================================================


def test_explore_bus_failure_raises_clierror(monkeypatch, tmp_path) -> None:
    fake = _FlakyReadBus(positions={i: 2048 for i in range(1, 7)}, fail_ids={1})
    _patch_bus(monkeypatch, fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    map_path = tmp_path / "explore.map.json"
    with pytest.raises(CliError) as exc:
        arm_cmd.cmd_arm_explore(_explore_args(map=str(map_path), apply=True))
    assert exc.value.code == EXIT_ENV_ERROR
    assert fake._open is False, "bus must be closed in finally even on failure"


def test_explore_no_port_raises_env_error(monkeypatch) -> None:
    monkeypatch.setattr(arm_cmd, "_candidate_ports", lambda: [])
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))
    with pytest.raises(CliError) as exc:
        arm_cmd.cmd_arm_explore(_explore_args(apply=True))
    assert exc.value.code == EXIT_ENV_ERROR
    assert "no serial port" in exc.value.message


# ===========================================================================
# arm explore is registered under the arm noun overview list is out of scope
# (t10 owns overview/learn/explain) — but the flex handler must be untouched.
# ===========================================================================


def test_flex_handler_still_works(monkeypatch) -> None:
    """t9 must not touch the flex handler: a bounded --apply move still executes."""
    fake = FakeBus(positions={1: 2048})
    _patch_bus(monkeypatch, fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_flex(_flex_args(joint="shoulder_pan", to=3000, apply=True))

    assert fake.position_writes == [{"motor": 1, "position": 3000}]
