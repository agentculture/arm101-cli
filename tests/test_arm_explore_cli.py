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
from arm101.cli._errors import EXIT_ENV_ERROR, EXIT_USER_ERROR, CliError
from arm101.hardware import arm_spec
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
    threshold_joint: "list[str] | None" = None,
    threshold_file: "str | None" = None,
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
        threshold_joint=threshold_joint,
        threshold_file=threshold_file,
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
    assert args.threshold_joint is None
    assert args.threshold_file is None
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
            "--threshold-joint",
            "shoulder_lift=350",
            "--threshold-joint",
            "gripper=380",
            "--threshold-file",
            "/tmp/thresholds.jsonl",
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
    assert args.threshold_joint == ["shoulder_lift=350", "gripper=380"]
    assert args.threshold_file == "/tmp/thresholds.jsonl"
    assert args.max_moves == 10
    assert args.resolution == 256
    assert args.json is True


# ===========================================================================
# Per-joint thresholds (issue #26) — flag parsing, file parsing, precedence
# ===========================================================================


def test_parse_threshold_joint_flags_none_returns_empty() -> None:
    assert arm_cmd._parse_threshold_joint_flags(None) == {}


def test_parse_threshold_joint_flags_valid() -> None:
    result = arm_cmd._parse_threshold_joint_flags(["shoulder_lift=350", "gripper=380"])
    assert result == {"shoulder_lift": 350, "gripper": 380}


def test_parse_threshold_joint_flags_missing_equals_raises() -> None:
    with pytest.raises(CliError) as exc:
        arm_cmd._parse_threshold_joint_flags(["shoulder_lift350"])
    assert exc.value.code == EXIT_USER_ERROR


def test_parse_threshold_joint_flags_unknown_joint_raises() -> None:
    with pytest.raises(CliError) as exc:
        arm_cmd._parse_threshold_joint_flags(["not_a_joint=100"])
    assert exc.value.code == EXIT_USER_ERROR
    assert "not_a_joint" in exc.value.message


def test_parse_threshold_joint_flags_malformed_value_raises() -> None:
    with pytest.raises(CliError) as exc:
        arm_cmd._parse_threshold_joint_flags(["shoulder_lift=not_an_int"])
    assert exc.value.code == EXIT_USER_ERROR


def test_parse_threshold_file_none_returns_empty() -> None:
    assert arm_cmd._parse_threshold_file(None) == {}


def test_parse_threshold_file_valid_jsonl(tmp_path) -> None:
    path = tmp_path / "thresholds.jsonl"
    path.write_text(
        '{"joint": "shoulder_lift", "threshold": 350}\n'
        "\n"  # blank lines are skipped
        '{"joint": "gripper", "threshold": 380}\n'
    )
    result = arm_cmd._parse_threshold_file(str(path))
    assert result == {"shoulder_lift": 350, "gripper": 380}


def test_parse_threshold_file_missing_path_raises() -> None:
    with pytest.raises(CliError) as exc:
        arm_cmd._parse_threshold_file("/nonexistent/thresholds.jsonl")
    assert exc.value.code == EXIT_USER_ERROR


def test_parse_threshold_file_malformed_json_line_raises_with_line_number(tmp_path) -> None:
    path = tmp_path / "thresholds.jsonl"
    path.write_text('{"joint": "gripper", "threshold": 380}\nnot json\n')
    with pytest.raises(CliError) as exc:
        arm_cmd._parse_threshold_file(str(path))
    assert exc.value.code == EXIT_USER_ERROR
    assert "line 2" in exc.value.message


def test_parse_threshold_file_unknown_joint_raises(tmp_path) -> None:
    path = tmp_path / "thresholds.jsonl"
    path.write_text('{"joint": "not_a_joint", "threshold": 100}\n')
    with pytest.raises(CliError) as exc:
        arm_cmd._parse_threshold_file(str(path))
    assert exc.value.code == EXIT_USER_ERROR
    assert "line 1" in exc.value.message


def test_parse_threshold_file_non_int_threshold_raises(tmp_path) -> None:
    path = tmp_path / "thresholds.jsonl"
    path.write_text('{"joint": "gripper", "threshold": "not-an-int"}\n')
    with pytest.raises(CliError) as exc:
        arm_cmd._parse_threshold_file(str(path))
    assert exc.value.code == EXIT_USER_ERROR
    assert "line 1" in exc.value.message


def test_parse_threshold_file_missing_keys_raises(tmp_path) -> None:
    path = tmp_path / "thresholds.jsonl"
    path.write_text('{"joint": "gripper"}\n')
    with pytest.raises(CliError) as exc:
        arm_cmd._parse_threshold_file(str(path))
    assert exc.value.code == EXIT_USER_ERROR


def test_resolve_explore_thresholds_default_only() -> None:
    args = _explore_args()
    resolved = arm_cmd._resolve_contact_thresholds(args)
    assert resolved == dict(zip(arm_spec.JOINTS, arm_spec.resolve_contact_thresholds()))


def test_resolve_explore_thresholds_blanket_overrides_file(tmp_path) -> None:
    path = tmp_path / "thresholds.jsonl"
    path.write_text('{"joint": "shoulder_lift", "threshold": 999}\n')
    args = _explore_args(threshold=111, threshold_file=str(path))
    resolved = arm_cmd._resolve_contact_thresholds(args)
    assert resolved["shoulder_lift"] == 111
    assert all(v == 111 for v in resolved.values())


def test_resolve_explore_thresholds_per_joint_overrides_blanket_and_file(tmp_path) -> None:
    path = tmp_path / "thresholds.jsonl"
    path.write_text('{"joint": "gripper", "threshold": 999}\n')
    args = _explore_args(
        threshold=111,
        threshold_joint=["shoulder_lift=350"],
        threshold_file=str(path),
    )
    resolved = arm_cmd._resolve_contact_thresholds(args)
    assert resolved["shoulder_lift"] == 350  # per-joint flag wins
    assert resolved["gripper"] == 111  # blanket wins over file
    assert resolved["elbow_flex"] == 111  # blanket applies to everything else


def test_resolve_explore_thresholds_file_overrides_default_with_no_blanket(tmp_path) -> None:
    path = tmp_path / "thresholds.jsonl"
    path.write_text('{"joint": "shoulder_lift", "threshold": 777}\n')
    args = _explore_args(threshold_file=str(path))
    resolved = arm_cmd._resolve_contact_thresholds(args)
    assert resolved["shoulder_lift"] == 777
    assert resolved["gripper"] == arm_spec.DEFAULT_CONTACT_THRESHOLDS["gripper"]


def test_cmd_arm_explore_unknown_threshold_joint_raises(monkeypatch) -> None:
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))
    with pytest.raises(CliError) as exc:
        arm_cmd.cmd_arm_explore(_explore_args(threshold_joint=["not_a_joint=100"]))
    assert exc.value.code == EXIT_USER_ERROR


def test_explore_dry_run_plan_surfaces_resolved_per_joint_thresholds(
    monkeypatch, capsys, tmp_path
) -> None:
    """The dry-run plan must surface the RESOLVED per-joint threshold map."""
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))
    map_path = tmp_path / "m.map.json"

    arm_cmd.cmd_arm_explore(
        _explore_args(
            map=str(map_path),
            threshold_joint=["shoulder_lift=350"],
            apply=False,
            json_mode=True,
        )
    )

    plan = json.loads(capsys.readouterr().out)["plan"]
    assert plan["thresholds"]["shoulder_lift"] == 350
    # Every other joint falls through to its built-in default.
    assert plan["thresholds"]["gripper"] == arm_spec.DEFAULT_CONTACT_THRESHOLDS["gripper"]
    assert set(plan["thresholds"].keys()) == set(arm_spec.JOINTS)


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
