"""Caller-contract regression guard for the t4/t7 ``gentle_move`` rewrite (t6).

``arm101/hardware/gentle.py::gentle_move`` was rewritten to MEASURE the arm
(poll position/load during travel, terminate on measured arrival / contact /
timeout, return read-back values) instead of assuming it, and gained new
keyword-only arguments (``poll_interval``, ``timeout``, ``arrival_tolerance``,
``stall_eps``, ``stall_samples``, ``onset_ticks``). Spec claim c2 / honesty
condition h5 require that every EXISTING caller — ``arm flex --gentle``,
``arm flex --demo``, and ``arm explore`` — keeps working through the
corrected primitive with NO change to its CLI contract, consent gating, or
JSON payload keys.

This file does not re-test ``gentle_move``'s internal measurement behaviour
(that is ``tests/test_gentle.py``'s job) or the overload-recovery path (t5's
``tests/test_gentle_overload_guard.py``). It only pins:

1. The exact JSON payload KEY SETS of ``gentle_move`` itself and of every
   caller — a missing or renamed key is a broken contract.
2. The three-mode consent gating is unchanged for the gated motion verbs:
   without ``--apply`` in non-TTY mode, each emits a PLAN and commands no
   motion (no bus is even opened, so certainly no ``position_writes``).
3. ``gentle_move``'s new keyword arguments are all OPTIONAL, so a pre-rewrite
   call site (positional ``bus, motor, target`` plus
   ``min_angle``/``max_angle``/``allow_motion`` only) still works unmodified.
"""

from __future__ import annotations

import argparse
import inspect
import json
import sys

from arm101.cli._commands import arm as arm_cmd
from arm101.hardware.bus import FakeBus
from arm101.hardware.gentle import gentle_move
from tests._fakes import ServoModelBus

# ---------------------------------------------------------------------------
# Pinned key sets — the contract surface. Change these ONLY in lockstep with
# a deliberate, reviewed contract change; a drift here is exactly the kind of
# silent break this file exists to catch.
# ---------------------------------------------------------------------------

#: `gentle_move`'s own result dict — see its docstring "Returns" section.
GENTLE_MOVE_KEYS = {
    "motor",
    "requested_target",
    "clamped_target",
    "was_clamped",
    "start_position",
    "threshold",
    "step",
    "backoff_ticks",
    "acceleration",
    "speed",
    "contacted",
    "contact_position",
    "contact_load",
    "retreat_position",
    "final_position",
    "overloaded",
}

#: `arm flex --gentle --json` envelope (arm101/cli/_commands/arm.py::_emit_flex_move).
FLEX_GENTLE_ENVELOPE_KEYS = {"role", "port", "joint", "gentle", "move"}

#: `arm flex --demo --json` envelope (arm101/cli/_commands/arm.py::_emit_flex_demo).
FLEX_DEMO_ENVELOPE_KEYS = {"role", "port", "demo"}

#: `demo_sweep`'s result dict — see arm101/hardware/demo.py::demo_sweep docstring.
DEMO_SWEEP_KEYS = {
    "fraction",
    "threshold",
    "joints",
    "aborted_on_contact",
    "aborted_joint",
    "aborted_on_overload",
    "overloaded_joint",
}

#: One `demo_sweep` per-joint report entry's keys (same docstring).
DEMO_JOINT_ENTRY_KEYS = {
    "motor",
    "min_angle",
    "max_angle",
    "start_position",
    "planned_targets",
    "targets_attempted",
    "moves",
    "contacted",
    "overloaded",
    "final_position",
}

#: `arm explore --json` envelope (arm101/cli/_commands/arm.py::_emit_explore_result).
EXPLORE_JSON_KEYS = {
    "verb",
    "role",
    "port",
    "cells_visited",
    "moves",
    "reachable",
    "contacts",
    "escapes_attempted",
    "escapes_succeeded",
    "budget_bounded",
    "errors",
    "map_path",
    "log_path",
}

#: gentle_move's new-in-the-rewrite keyword-only arguments — must ALL default,
#: so a pre-rewrite call site (bus, motor, target, min_angle=, max_angle=,
#: allow_motion=) still works unmodified.
NEW_GENTLE_MOVE_KWARGS = (
    "poll_interval",
    "timeout",
    "arrival_tolerance",
    "stall_eps",
    "stall_samples",
    "onset_ticks",
)


# ---------------------------------------------------------------------------
# Test doubles + helpers (mirrors tests/test_arm_read_flex.py and
# tests/test_arm_explore_cli.py — not imported from there so this file stays
# a self-contained regression guard).
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


def _patch_bus(monkeypatch, fake: FakeBus, port: str = "/dev/ttyACM_fake") -> None:
    """Patch the arm bus seam so it opens *fake* at *port*."""
    fake.open()
    monkeypatch.setattr(arm_cmd, "_candidate_ports", lambda: [port])
    monkeypatch.setattr(arm_cmd, "_open_bus", lambda _p: fake)


def _flex_args(
    joint: "str | None" = None,
    to: "int | None" = None,
    demo: bool = False,
    gentle: bool = False,
    threshold: "int | None" = None,
    role: str = "follower",
    port: "str | None" = None,
    apply: bool = False,
    json_mode: bool = False,
) -> argparse.Namespace:
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
) -> argparse.Namespace:
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


# ===========================================================================
# 1. JSON payload key sets — gentle_move itself
# ===========================================================================


def test_gentle_move_result_keys_pinned_no_contact() -> None:
    """The happy-path (no contact) result dict carries exactly the pinned keys."""
    bus = ServoModelBus(positions={1: 2048})
    bus.open()

    result = gentle_move(bus, 1, 2148, min_angle=0, max_angle=4095, allow_motion=True)

    assert set(result.keys()) == GENTLE_MOVE_KEYS


def test_gentle_move_result_keys_pinned_on_contact() -> None:
    """The contact branch populates different values but the SAME key set —
    the dict shape must not vary by code path."""
    bus = ServoModelBus(positions={1: 2048}, obstacle_stiffness=20).place_obstacle(1, 2248)
    bus.open()

    result = gentle_move(bus, 1, 2448, min_angle=0, max_angle=4095, allow_motion=True)

    assert set(result.keys()) == GENTLE_MOVE_KEYS
    assert result["contacted"] is True
    assert result["contact_position"] is not None
    assert result["contact_load"] is not None
    assert result["retreat_position"] is not None


# ===========================================================================
# 2. JSON payload key sets — arm flex --gentle --json
# ===========================================================================


def test_flex_gentle_json_keys_pinned(monkeypatch, capsys) -> None:
    fake = FakeBus(positions={1: 1000})
    _patch_bus(monkeypatch, fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_flex(
        _flex_args(joint="shoulder_pan", to=1100, gentle=True, apply=True, json_mode=True)
    )

    payload = json.loads(capsys.readouterr().out)
    assert set(payload.keys()) == FLEX_GENTLE_ENVELOPE_KEYS
    assert set(payload["move"].keys()) == GENTLE_MOVE_KEYS


# ===========================================================================
# 3. JSON payload key sets — arm flex --demo --json
# ===========================================================================


def test_flex_demo_json_keys_pinned(monkeypatch, capsys) -> None:
    fake = FakeBus(positions={i: 2048 for i in range(1, 7)})
    _patch_bus(monkeypatch, fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_flex(_flex_args(demo=True, apply=True, json_mode=True))

    payload = json.loads(capsys.readouterr().out)
    assert set(payload.keys()) == FLEX_DEMO_ENVELOPE_KEYS

    report = payload["demo"]
    assert set(report.keys()) == DEMO_SWEEP_KEYS
    assert report["joints"], "expected at least one joint report to inspect"
    first_joint_report = next(iter(report["joints"].values()))
    assert set(first_joint_report.keys()) == DEMO_JOINT_ENTRY_KEYS
    # each attempted move is itself a raw gentle_move result dict.
    assert first_joint_report["moves"], "expected at least one gentle_move attempt"
    assert set(first_joint_report["moves"][0].keys()) == GENTLE_MOVE_KEYS


# ===========================================================================
# 4. JSON payload key set — arm explore --json
# ===========================================================================


def test_explore_json_keys_pinned(monkeypatch, capsys, tmp_path) -> None:
    fake = FakeBus(positions={i: 2048 for i in range(1, 7)})
    _patch_bus(monkeypatch, fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    map_path = tmp_path / "explore.map.json"
    arm_cmd.cmd_arm_explore(
        _explore_args(map=str(map_path), apply=True, max_moves=5, json_mode=True)
    )

    payload = json.loads(capsys.readouterr().out)
    assert set(payload.keys()) == EXPLORE_JSON_KEYS


# ===========================================================================
# 5. Consent gating unchanged — dry-run (non-TTY, no --apply): zero motion,
#    zero bus access, for every gated motion caller.
# ===========================================================================


def test_flex_gentle_single_joint_dry_run_no_motion(monkeypatch, capsys) -> None:
    opened = {"v": False}

    def _open(_p):
        opened["v"] = True
        raise AssertionError("dry-run must not open a bus")

    monkeypatch.setattr(arm_cmd, "_candidate_ports", lambda: ["/dev/ttyACM_fake"])
    monkeypatch.setattr(arm_cmd, "_open_bus", _open)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_flex(_flex_args(joint="shoulder_pan", to=3000, gentle=True, apply=False))

    out = capsys.readouterr().out
    assert "Dry-run" in out
    assert opened["v"] is False, "dry-run must not open a bus"


def test_flex_demo_dry_run_no_motion(monkeypatch, capsys) -> None:
    opened = {"v": False}

    def _open(_p):
        opened["v"] = True
        raise AssertionError("dry-run must not open a bus")

    monkeypatch.setattr(arm_cmd, "_candidate_ports", lambda: ["/dev/ttyACM_fake"])
    monkeypatch.setattr(arm_cmd, "_open_bus", _open)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_flex(_flex_args(demo=True, apply=False))

    out = capsys.readouterr().out
    assert "Dry-run" in out
    assert opened["v"] is False, "dry-run must not open a bus"


def test_explore_dry_run_no_motion(monkeypatch, capsys, tmp_path) -> None:
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


# ===========================================================================
# 6. gentle_move's new poll/stall kwargs are all OPTIONAL — a pre-rewrite
#    call site (positional bus/motor/target + min_angle/max_angle/
#    allow_motion only) still works unmodified.
# ===========================================================================


def test_gentle_move_new_kwargs_all_have_defaults() -> None:
    sig = inspect.signature(gentle_move)
    for name in NEW_GENTLE_MOVE_KWARGS:
        param = sig.parameters[name]
        assert param.default is not inspect.Parameter.empty, (
            f"gentle_move's {name!r} must be optional — an existing caller " "never passes it"
        )


def test_gentle_move_pre_rewrite_call_shape_still_works() -> None:
    """The exact call shape every existing caller uses (bus, motor, target,
    min_angle=, max_angle=, allow_motion=True; NO new kwargs) still works and
    returns a well-formed result — i.e. no caller had to change."""
    bus = ServoModelBus(positions={1: 2048})
    bus.open()

    result = gentle_move(bus, 1, 2148, min_angle=0, max_angle=4095, allow_motion=True)

    assert result["contacted"] is False
    assert result["final_position"] == 2148
