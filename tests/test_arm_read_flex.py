"""Tests for ``arm read`` and ``arm flex`` — the t8 wiring of the whole-arm
read snapshot and the gated motion verbs into the ``arm`` noun.

All hardware is a :class:`~arm101.hardware.bus.FakeBus` injected through the
``arm._open_bus`` / ``arm._candidate_ports`` seam (the read/flex handlers call
those module-level names directly, so patching them on the ``arm`` module is
what wires in the fake). No serial port is ever opened.

Coverage:
- ``arm read`` text + ``--json``: six joints; a flaky joint marked
  ``failed``/``partial`` while the rest still read; no-port -> env error.
- ``arm flex`` validation (joint+--demo, neither, missing --to, unknown joint).
- ``arm flex`` consent: dry-run (zero motion / zero bus), interactive
  yes/no, agent ``--apply``.
- ``arm flex`` execution: bounded compliant move, clamp-to-max, gentle path
  (threshold honored), demo sweep across all joints.
- Doc lockstep: explain/learn/overview mention the new verbs; the arm-overview
  verb list carries them.
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

    def __init__(self, lines: list[str], tty: bool = True) -> None:
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
    """Patch the arm read/flex seam so it opens *fake* at *port*."""
    fake.open()
    monkeypatch.setattr(arm_cmd, "_candidate_ports", lambda: [port])
    monkeypatch.setattr(arm_cmd, "_open_bus", lambda _p: fake)


def _read_args(role: str = "follower", json_mode: bool = False, port: "str | None" = None):
    return argparse.Namespace(role=role, json=json_mode, port=port)


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
# arm read — read-only
# ===========================================================================


def test_read_text_renders_six_joints(monkeypatch, capsys) -> None:
    fake = FakeBus(positions={i: 1000 + i for i in range(1, 7)})
    _patch_bus(monkeypatch, fake)

    arm_cmd.cmd_arm_read(_read_args())

    out = capsys.readouterr().out
    for joint in arm_spec.JOINTS:
        assert joint in out, f"{joint} missing from arm read table"
    assert "health" in out
    assert "complete" in out


def test_read_json_structure(monkeypatch, capsys) -> None:
    fake = FakeBus(positions={i: 1000 + i for i in range(1, 7)})
    _patch_bus(monkeypatch, fake)

    arm_cmd.cmd_arm_read(_read_args(json_mode=True))

    payload = json.loads(capsys.readouterr().out)
    assert payload["role"] == "follower"
    assert payload["complete"] is True
    assert payload["port"] == "/dev/ttyACM_fake"
    assert len(payload["joints"]) == 6
    first = payload["joints"][0]
    assert first["joint"] == "shoulder_pan"
    assert first["id"] == 1
    assert first["health"] == "ok"
    assert first["position"] == 1001


def test_read_leader_role_uses_leader_ids(monkeypatch, capsys) -> None:
    fake = FakeBus(positions={i: 2000 for i in range(1, 7)})
    _patch_bus(monkeypatch, fake)

    arm_cmd.cmd_arm_read(_read_args(role="leader", json_mode=True))

    payload = json.loads(capsys.readouterr().out)
    assert payload["role"] == "leader"
    by = {j["joint"]: j for j in payload["joints"]}
    assert by["gripper"]["id"] == arm_spec.joint_ids("leader")["gripper"]


def test_read_one_failed_joint_others_ok_json(monkeypatch, capsys) -> None:
    fake = _FlakyReadBus(positions={i: 1000 + i for i in range(1, 7)}, fail_ids={3})
    _patch_bus(monkeypatch, fake)

    arm_cmd.cmd_arm_read(_read_args(json_mode=True))

    payload = json.loads(capsys.readouterr().out)
    by = {j["joint"]: j for j in payload["joints"]}
    assert by["elbow_flex"]["health"] == "failed"
    assert by["elbow_flex"]["position"] is None
    assert by["shoulder_pan"]["health"] == "ok"
    assert payload["complete"] is False


def test_read_failed_joint_marked_in_text(monkeypatch, capsys) -> None:
    fake = _FlakyReadBus(positions={i: 1000 + i for i in range(1, 7)}, fail_ids={6})
    _patch_bus(monkeypatch, fake)

    arm_cmd.cmd_arm_read(_read_args())

    out = capsys.readouterr().out
    assert "failed" in out
    assert "incomplete" in out
    # the failed joint's register cells render as '-'
    assert "| gripper | 6 | failed | - |" in out


def test_read_no_port_raises_env_error(monkeypatch) -> None:
    monkeypatch.setattr(arm_cmd, "_candidate_ports", lambda: [])
    with pytest.raises(CliError) as exc:
        arm_cmd.cmd_arm_read(_read_args())
    assert exc.value.code == EXIT_ENV_ERROR
    assert "no serial port" in exc.value.message


def test_read_closes_bus(monkeypatch) -> None:
    fake = FakeBus(positions={1: 100})
    _patch_bus(monkeypatch, fake)
    arm_cmd.cmd_arm_read(_read_args())
    assert fake._open is False, "arm read must close the bus in its finally"


def test_read_explicit_port_used(monkeypatch, capsys) -> None:
    fake = FakeBus(positions={1: 100})
    fake.open()
    seen: dict[str, str] = {}

    def _open(port: str) -> FakeBus:
        seen["port"] = port
        return fake

    monkeypatch.setattr(arm_cmd, "_open_bus", _open)
    arm_cmd.cmd_arm_read(_read_args(port="/dev/ttyACM7", json_mode=True))

    payload = json.loads(capsys.readouterr().out)
    assert payload["port"] == "/dev/ttyACM7"
    assert seen["port"] == "/dev/ttyACM7"


# ---------------------------------------------------------------------------
# arm read — the encoder offset (Ofs / Homing_Offset, addr 31) is INSPECTABLE
# ---------------------------------------------------------------------------


def test_read_json_exposes_the_encoder_offset(monkeypatch, capsys) -> None:
    """Issue #35: a human must be able to see addr 31 before anyone writes it.

    Reported signed (-1073), never as the raw sign-magnitude wire value (3121).
    """
    fake = FakeBus(positions={i: 1000 + i for i in range(1, 7)}, offsets={3: -1073})
    _patch_bus(monkeypatch, fake)

    arm_cmd.cmd_arm_read(_read_args(json_mode=True))

    payload = json.loads(capsys.readouterr().out)
    by = {j["joint"]: j for j in payload["joints"]}
    assert by["elbow_flex"]["offset"] == -1073
    assert by["shoulder_pan"]["offset"] == 0


def test_read_text_renders_an_offset_column(monkeypatch, capsys) -> None:
    fake = FakeBus(positions={i: 1000 + i for i in range(1, 7)}, offsets={3: -1073})
    _patch_bus(monkeypatch, fake)

    arm_cmd.cmd_arm_read(_read_args())

    out = capsys.readouterr().out
    assert "offset" in out
    assert "-1073" in out


def test_read_is_read_only_even_with_the_offset_surfaced(monkeypatch) -> None:
    """Surfacing addr 31 must not write addr 31 — ``arm read`` has no consent gate."""
    fake = FakeBus(positions={i: 1000 + i for i in range(1, 7)}, offsets={3: -1073})
    _patch_bus(monkeypatch, fake)

    arm_cmd.cmd_arm_read(_read_args())

    assert fake.register_writes == []
    assert fake.offset_writes == []


# ===========================================================================
# arm flex — validation
# ===========================================================================


def test_flex_joint_and_demo_conflict(monkeypatch) -> None:
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))
    with pytest.raises(CliError) as exc:
        arm_cmd.cmd_arm_flex(_flex_args(joint="shoulder_pan", to=2048, demo=True, apply=True))
    assert exc.value.code == EXIT_USER_ERROR


def test_flex_demo_with_to_conflict(monkeypatch) -> None:
    """--demo + --to is contradictory: --to would be silently ignored, so reject
    it up front (regression for the qodo #22 finding)."""
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))
    with pytest.raises(CliError) as exc:
        arm_cmd.cmd_arm_flex(_flex_args(demo=True, to=123, apply=True))
    assert exc.value.code == EXIT_USER_ERROR
    assert "not both" in exc.value.message


def test_flex_neither_joint_nor_demo(monkeypatch) -> None:
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))
    with pytest.raises(CliError) as exc:
        arm_cmd.cmd_arm_flex(_flex_args(apply=True))
    assert exc.value.code == EXIT_USER_ERROR


def test_flex_missing_to(monkeypatch) -> None:
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))
    with pytest.raises(CliError) as exc:
        arm_cmd.cmd_arm_flex(_flex_args(joint="shoulder_pan", to=None, apply=True))
    assert exc.value.code == EXIT_USER_ERROR


def test_flex_unknown_joint(monkeypatch) -> None:
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))
    with pytest.raises(CliError) as exc:
        arm_cmd.cmd_arm_flex(_flex_args(joint="nope", to=2048, apply=True))
    assert exc.value.code == EXIT_USER_ERROR


# ===========================================================================
# arm flex — execution (agent --apply, non-TTY)
# ===========================================================================


def test_flex_apply_executes_bounded_move(monkeypatch) -> None:
    fake = FakeBus(positions={1: 2048})
    _patch_bus(monkeypatch, fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_flex(_flex_args(joint="shoulder_pan", to=3000, apply=True))

    assert fake.position_writes == [{"motor": 1, "position": 3000}]
    assert fake.accel_writes == [{"motor": 1, "value": 20}]
    assert fake.speed_writes == [{"motor": 1, "value": 150}]
    assert fake.torque_writes == [{"motor": 1, "on": True}]
    assert fake._open is False  # bus closed in finally


def test_flex_apply_clamps_to_max(monkeypatch, capsys) -> None:
    fake = FakeBus(positions={1: 2048}, info={1: {"min_angle": 0, "max_angle": 3000}})
    _patch_bus(monkeypatch, fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_flex(_flex_args(joint="shoulder_pan", to=4000, apply=True, json_mode=True))

    # the written goal-position must be clamped to the joint's max_angle (3000)
    assert fake.position_writes[-1]["position"] == 3000
    payload = json.loads(capsys.readouterr().out)
    assert payload["move"]["clamped_target"] == 3000
    assert payload["move"]["was_clamped"] is True
    assert payload["gentle"] is False


def test_flex_gentle_threshold_honored(monkeypatch, capsys) -> None:
    fake = FakeBus(positions={1: 1000})
    _patch_bus(monkeypatch, fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_flex(
        _flex_args(
            joint="shoulder_pan",
            to=1100,
            gentle=True,
            threshold=300,
            apply=True,
            json_mode=True,
        )
    )

    payload = json.loads(capsys.readouterr().out)
    move = payload["move"]
    assert payload["gentle"] is True
    assert move["threshold"] == 300  # override honored
    assert move["start_position"] == 1000
    assert "contacted" in move  # gentle-specific key -> gentle path taken
    assert move["contacted"] is False  # load 0 < 300, no contact
    assert len(fake.position_writes) >= 1  # stepped toward the target


def test_flex_gentle_default_threshold(monkeypatch, capsys) -> None:
    fake = FakeBus(positions={1: 1000})
    _patch_bus(monkeypatch, fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_flex(
        _flex_args(joint="shoulder_pan", to=1100, gentle=True, apply=True, json_mode=True)
    )

    payload = json.loads(capsys.readouterr().out)
    assert payload["move"]["threshold"] == 250  # _DEFAULT_THRESHOLD


def test_flex_gentle_explicit_threshold_zero_honored(monkeypatch, capsys) -> None:
    """`--threshold 0` is a valid (falsy) override and must NOT collapse to the
    default 250 via an `or` fallback (regression for the qodo #22 finding)."""
    fake = FakeBus(positions={1: 1000})
    _patch_bus(monkeypatch, fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_flex(
        _flex_args(
            joint="shoulder_pan",
            to=1100,
            gentle=True,
            threshold=0,
            apply=True,
            json_mode=True,
        )
    )

    payload = json.loads(capsys.readouterr().out)
    assert payload["move"]["threshold"] == 0  # explicit 0 honored, not 250


def test_flex_demo_runs_all_joints(monkeypatch, capsys) -> None:
    fake = FakeBus(positions={i: 2048 for i in range(1, 7)})
    _patch_bus(monkeypatch, fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_flex(_flex_args(demo=True, apply=True, json_mode=True))

    payload = json.loads(capsys.readouterr().out)
    report = payload["demo"]
    assert report["aborted_on_contact"] is False
    assert set(report["joints"]) == set(arm_spec.JOINTS)
    assert len(report["joints"]) == 6
    assert len(fake.position_writes) > 0
    assert fake._open is False


# ===========================================================================
# arm flex — dry-run (non-TTY, no --apply): zero motion, zero bus access
# ===========================================================================


def test_flex_dry_run_zero_writes_and_no_bus(monkeypatch, capsys) -> None:
    fake = FakeBus(positions={1: 2048})
    opened = {"v": False}

    def _open(_p):
        opened["v"] = True
        fake.open()
        return fake

    monkeypatch.setattr(arm_cmd, "_candidate_ports", lambda: ["/dev/ttyACM_fake"])
    monkeypatch.setattr(arm_cmd, "_open_bus", _open)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_flex(_flex_args(joint="shoulder_pan", to=3000, apply=False))

    out = capsys.readouterr().out
    assert "Dry-run" in out
    assert "shoulder_pan" in out
    assert "3000" in out
    assert opened["v"] is False, "dry-run must not open a bus"
    assert fake.position_writes == []
    assert fake.accel_writes == []


def test_flex_dry_run_json(monkeypatch, capsys) -> None:
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_flex(_flex_args(joint="shoulder_pan", to=2500, apply=False, json_mode=True))

    payload = json.loads(capsys.readouterr().out)
    plan = payload["plan"]
    assert plan["joint"] == "shoulder_pan"
    assert plan["target"] == 2500
    assert plan["role"] == "follower"
    assert plan["mode"] == "compliant"


def test_flex_dry_run_demo_json(monkeypatch, capsys) -> None:
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_flex(_flex_args(demo=True, apply=False, json_mode=True))

    payload = json.loads(capsys.readouterr().out)
    plan = payload["plan"]
    assert plan["mode"] == "demo"
    assert plan["joints"] == list(arm_spec.JOINTS)


# ===========================================================================
# arm flex — interactive (TTY) consent
# ===========================================================================


def test_flex_interactive_confirm_yes_moves(monkeypatch) -> None:
    fake = FakeBus(positions={1: 2048})
    _patch_bus(monkeypatch, fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["yes\n"], tty=True))

    arm_cmd.cmd_arm_flex(_flex_args(joint="shoulder_pan", to=3000))

    assert fake.position_writes == [{"motor": 1, "position": 3000}]


def test_flex_interactive_decline_no_move(monkeypatch, capsys) -> None:
    fake = FakeBus(positions={1: 2048})
    _patch_bus(monkeypatch, fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["no\n"], tty=True))

    arm_cmd.cmd_arm_flex(_flex_args(joint="shoulder_pan", to=3000))

    assert fake.position_writes == []
    out = capsys.readouterr().out
    assert "Aborted" in out


# ===========================================================================
# Registration
# ===========================================================================


def test_register_read_verb() -> None:
    top = argparse.ArgumentParser(prog="arm101")
    sub = top.add_subparsers()
    arm_cmd.register(sub)

    args = top.parse_args(["arm", "read"])
    assert args.func is arm_cmd.cmd_arm_read
    assert args.role == "follower"

    args2 = top.parse_args(["arm", "read", "--role", "leader", "--json"])
    assert args2.role == "leader"
    assert args2.json is True


def test_register_flex_verb() -> None:
    top = argparse.ArgumentParser(prog="arm101")
    sub = top.add_subparsers()
    arm_cmd.register(sub)

    args = top.parse_args(["arm", "flex", "shoulder_pan", "--to", "2048", "--apply"])
    assert args.func is arm_cmd.cmd_arm_flex
    assert args.joint == "shoulder_pan"
    assert args.to == 2048
    assert args.apply is True
    assert args.demo is False


def test_register_flex_demo_and_threshold() -> None:
    top = argparse.ArgumentParser(prog="arm101")
    sub = top.add_subparsers()
    arm_cmd.register(sub)

    args = top.parse_args(["arm", "flex", "--demo", "--gentle", "--threshold", "300"])
    assert args.demo is True
    assert args.gentle is True
    assert args.threshold == 300


# ===========================================================================
# arm overview verb list now carries read + flex
# ===========================================================================


def test_overview_json_lists_read_and_flex(capsys) -> None:
    arm_cmd.cmd_arm_overview(argparse.Namespace(json=True, target=None))
    payload = json.loads(capsys.readouterr().out)
    assert "read" in payload["verbs"]
    assert "flex" in payload["verbs"]
    # existing verbs still present
    assert "overview" in payload["verbs"]
    assert "setup" in payload["verbs"]


def test_overview_text_mentions_read_and_flex(capsys) -> None:
    arm_cmd.cmd_arm_overview(argparse.Namespace(json=False, target=None))
    out = capsys.readouterr().out
    assert "read" in out
    assert "flex" in out


# ===========================================================================
# Doc lockstep — explain / learn / overview
# ===========================================================================


def test_explain_arm_read_and_flex_resolve() -> None:
    from arm101.explain import resolve

    read_body = resolve(("arm", "read"))
    flex_body = resolve(("arm", "flex"))
    assert isinstance(read_body, str) and read_body.strip()
    assert isinstance(flex_body, str) and flex_body.strip()
    assert "no consent gate" in read_body
    assert "--gentle" in flex_body


def test_learn_mentions_arm_read_and_flex() -> None:
    from arm101.cli._commands.learn import _TEXT, _as_json_payload

    assert "arm read" in _TEXT
    assert "arm flex" in _TEXT
    paths = [c["path"] for c in _as_json_payload()["commands"]]
    assert ["arm", "read"] in paths
    assert ["arm", "flex"] in paths
    assert "arm read" in _as_json_payload()["hardware"]["verbs"]
    assert "arm flex" in _as_json_payload()["hardware"]["verbs"]


def test_overview_verbs_list_mentions_arm_read_and_flex() -> None:
    from arm101.cli._commands.overview import _VERBS

    assert any("arm read" in v for v in _VERBS)
    assert any("arm flex" in v for v in _VERBS)
