"""Tests for t5 — ``arm.py`` CLI surfacing of a servo overload across
``arm read`` / ``arm flex`` (single-joint + ``--demo``).

The bus-layer overload primitive (``OverloadError``, ``is_overload``,
``FakeBus.fail_with_overload_on_op`` / ``overload_after_ops``) and the
hardware-layer graceful-recovery contract (``gentle_move``/``compliant_move``
return ``overloaded: bool`` instead of raising; ``demo_sweep`` propagates it
per-joint plus top-level ``aborted_on_overload``/``overloaded_joint``) are
already merged into this branch's base (see tests/test_bus_overload.py,
tests/test_gentle_overload.py, tests/test_motion_overload.py,
tests/test_demo_overload.py). This file is about the CLI SURFACE on top of
that: does an operator (or an agent parsing ``--json``) actually SEE the
overload, in both text and JSON, for all three verbs — and never as a raw
Python traceback.

Honesty conditions covered:

* h5 — an overload surfaces as *documented, structured* output (an
  ``overloaded``/``overloaded_joint`` JSON key, a text marker), not silence.
* h7 — an overload never crashes the CLI: no unhandled exception, no
  traceback on stderr, and the normal success exit path is taken (calling the
  ``cmd_arm_*`` handler directly must not raise).

Coverage:

- ``arm read``: a joint whose EVERY retry raises ``OverloadError`` gets
  ``JointReading.overloaded=True`` (health stays ``"failed"`` — additive,
  not a control-flow change); a joint failing for a NON-overload reason stays
  ``overloaded=False`` (regression: don't mislabel a generic comms failure).
  Both JSON (`joints[i]["overloaded"]`) and text (`[OVERLOAD]` marker + a
  summary ``overloaded:`` segment) are asserted.
- ``arm flex <joint> --to <t> --gentle --apply``: a mid-move overload (via
  the FakeBus op-counting seam) yields ``move["overloaded"] is True`` in
  JSON, and the same key/value renders in the text dict-dump — the normal
  success path, no raise.
- ``arm flex --demo --apply``: a mid-sweep overload yields
  ``demo["aborted_on_overload"] is True`` / ``demo["overloaded_joint"]`` in
  JSON, and text renders an ``[OVERLOAD]`` marker plus an overload-abort
  summary line (mirroring the pre-existing ``[CONTACT]``/contact-abort
  handling).
"""

from __future__ import annotations

import argparse
import json
import sys

from arm101.cli._commands import arm as arm_cmd
from arm101.cli._errors import EXIT_ENV_ERROR, CliError
from arm101.hardware.arm_read import JointReading, read_arm
from arm101.hardware.bus import FakeBus, OverloadError

# ---------------------------------------------------------------------------
# Test doubles + helpers (kept local to this file, mirroring the pattern in
# tests/test_arm_read_flex.py and tests/test_demo_overload.py rather than
# importing private helpers across test modules).
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


class _OverloadReadBus(FakeBus):
    """FakeBus whose ``read_info`` permanently raises ``OverloadError`` for chosen motors."""

    def __init__(self, *args, overload_ids: "set[int] | None" = None, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._overload_ids = set(overload_ids or set())

    def read_info(self, motor: int) -> "dict[str, int]":
        if motor in self._overload_ids:
            raise OverloadError(motor=motor, error_byte=32)
        return super().read_info(motor)


class _GenericFailReadBus(FakeBus):
    """FakeBus whose ``read_info`` permanently raises a plain CliError (not overload)."""

    def __init__(self, *args, fail_ids: "set[int] | None" = None, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._fail_ids = set(fail_ids or set())

    def read_info(self, motor: int) -> "dict[str, int]":
        if motor in self._fail_ids:
            raise CliError(
                code=EXIT_ENV_ERROR,
                message=f"simulated non-overload read failure for motor {motor}",
                remediation="retry",
            )
        return super().read_info(motor)


class _RampLoadBus(FakeBus):
    """FakeBus whose present_load ramps PER MOTOR as that motor is driven.

    Mirrors the RampLoadBus doubles in tests/test_gentle.py and
    tests/test_demo.py: a motor listed in ``load_increment_by_motor`` bumps its
    reported ``present_load`` by that increment on every ``write_goal_position``,
    so it ramps into a ``gentle_move`` contact; motors absent from the mapping
    never ramp (load stays 0). Used here to drive a real mid-sweep CONTACT
    through the CLI so the text-mode ``[CONTACT]`` marker + contact-abort summary
    in ``_emit_flex_demo`` are exercised (the overload path is covered above).
    """

    def __init__(self, *args, load_increment_by_motor=None, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._load_increment_by_motor: "dict[int, int]" = load_increment_by_motor or {}
        self._load_by_motor: "dict[int, int]" = {}

    def write_goal_position(self, motor: int, position: int) -> None:
        super().write_goal_position(motor, position)
        increment = self._load_increment_by_motor.get(motor, 0)
        self._load_by_motor[motor] = self._load_by_motor.get(motor, 0) + increment

    def read_info(self, motor: int) -> "dict[str, int]":
        info = super().read_info(motor)
        info["present_load"] = self._load_by_motor.get(motor, 0)
        return info


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
# arm read — overloaded joint surfacing
# ===========================================================================


def test_read_overloaded_joint_sets_overloaded_true_json(monkeypatch, capsys) -> None:
    fake = _OverloadReadBus(positions={i: 1000 + i for i in range(1, 7)}, overload_ids={3})
    _patch_bus(monkeypatch, fake)

    arm_cmd.cmd_arm_read(_read_args(json_mode=True))

    payload = json.loads(capsys.readouterr().out)
    by = {j["joint"]: j for j in payload["joints"]}
    assert by["elbow_flex"]["health"] == "failed"
    assert by["elbow_flex"]["overloaded"] is True
    # Every other joint is healthy and explicitly NOT overloaded.
    for name, j in by.items():
        if name == "elbow_flex":
            continue
        assert j["health"] == "ok"
        assert j["overloaded"] is False
    assert payload["complete"] is False


def test_read_overloaded_joint_marked_in_text(monkeypatch, capsys) -> None:
    fake = _OverloadReadBus(positions={i: 1000 + i for i in range(1, 7)}, overload_ids={6})
    _patch_bus(monkeypatch, fake)

    arm_cmd.cmd_arm_read(_read_args())

    out = capsys.readouterr().out
    assert "failed" in out
    assert "[OVERLOAD]" in out
    # the overloaded joint's row carries the marker
    gripper_line = next(line for line in out.splitlines() if line.startswith("| gripper "))
    assert "[OVERLOAD]" in gripper_line
    assert "overloaded: gripper" in out


def test_read_generic_failure_not_marked_overloaded_json(monkeypatch, capsys) -> None:
    """A plain (non-overload) read failure must NOT be mislabeled overloaded=True."""
    fake = _GenericFailReadBus(positions={i: 1000 + i for i in range(1, 7)}, fail_ids={3})
    _patch_bus(monkeypatch, fake)

    arm_cmd.cmd_arm_read(_read_args(json_mode=True))

    payload = json.loads(capsys.readouterr().out)
    by = {j["joint"]: j for j in payload["joints"]}
    assert by["elbow_flex"]["health"] == "failed"
    assert by["elbow_flex"]["overloaded"] is False


def test_read_generic_failure_no_overload_marker_in_text(monkeypatch, capsys) -> None:
    fake = _GenericFailReadBus(positions={i: 1000 + i for i in range(1, 7)}, fail_ids={3})
    _patch_bus(monkeypatch, fake)

    arm_cmd.cmd_arm_read(_read_args())

    out = capsys.readouterr().out
    assert "[OVERLOAD]" not in out
    assert "overloaded:" not in out


def test_read_never_raises_on_overloaded_joint(monkeypatch) -> None:
    """h7: an overloaded joint must not escape as an unhandled exception."""
    fake = _OverloadReadBus(positions={1: 1000}, overload_ids={1})
    _patch_bus(monkeypatch, fake)

    # Must not raise.
    arm_cmd.cmd_arm_read(_read_args())


# ---------------------------------------------------------------------------
# arm_read.read_arm / JointReading — direct hardware-layer checks
# ---------------------------------------------------------------------------


def test_joint_reading_overloaded_field_defaults_false() -> None:
    r = JointReading(joint="gripper", motor_id=6, health="ok")
    assert r.overloaded is False


def test_read_arm_overloaded_joint_flag_direct() -> None:
    fake = _OverloadReadBus(positions={1: 1000, 2: 2000}, overload_ids={1})
    fake.open()

    readings = read_arm(fake, {"shoulder_pan": 1, "shoulder_lift": 2}, retries=2)
    by = {r.joint: r for r in readings}

    assert by["shoulder_pan"].health == "failed"
    assert by["shoulder_pan"].overloaded is True
    assert by["shoulder_lift"].health == "ok"
    assert by["shoulder_lift"].overloaded is False


# ===========================================================================
# arm flex <joint> --to <t> --gentle --apply — single-joint overload
# ===========================================================================
#
# Op-counting note (verified against the CLI code path, not just the
# primitive): ``_execute_single`` issues its OWN ``bus.read_info`` (op 1) to
# source min/max_angle BEFORE calling ``gentle_move``, which then does
# ``read_torque_limit`` (op 2), ``write_torque_limit`` cap (op 3),
# ``write_acceleration`` (op 4), ``write_goal_speed`` (op 5),
# ``enable_torque`` (op 6), ``read_info`` start-position (op 7), and the
# FIRST ``write_goal_position`` inside the step loop (op 8) — the first op
# actually inside gentle_move's ``try/except OverloadError`` block once the
# pre-move torque-limit read/write (ops 2-3, deliberately outside the try)
# have already succeeded. Arming the seam at 8 reproduces a graceful,
# caught-and-recovered mid-move overload.


def test_flex_gentle_apply_overload_json_surfaces_overloaded_true(monkeypatch, capsys) -> None:
    fake = FakeBus(positions={1: 2048}).fail_with_overload_on_op(8)
    _patch_bus(monkeypatch, fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    # Must not raise (h7) — the normal success path is taken.
    arm_cmd.cmd_arm_flex(
        _flex_args(joint="shoulder_pan", to=2500, gentle=True, apply=True, json_mode=True)
    )

    captured = capsys.readouterr()
    assert captured.err == ""  # no diagnostic/error leaked to stderr
    payload = json.loads(captured.out)
    move = payload["move"]
    assert move["overloaded"] is True
    assert move["contacted"] is False
    # The overload-recovery action (clear_overload) must have actually run —
    # it disarms FakeBus's seam, proving the catch-and-recover path executed
    # rather than the exception merely vanishing.
    assert fake.overload_after_ops is None
    assert fake._open is False  # bus still closed in the finally


def test_flex_gentle_apply_overload_text_renders_overloaded_key(monkeypatch, capsys) -> None:
    fake = FakeBus(positions={1: 2048}).fail_with_overload_on_op(8)
    _patch_bus(monkeypatch, fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_flex(_flex_args(joint="shoulder_pan", to=2500, gentle=True, apply=True))

    out = capsys.readouterr().out
    assert "- overloaded: True" in out


def test_flex_non_gentle_happy_path_still_reports_overloaded_false(monkeypatch, capsys) -> None:
    """Regression: the ordinary compliant-move path keeps overloaded=False."""
    fake = FakeBus(positions={1: 2048})
    _patch_bus(monkeypatch, fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_flex(_flex_args(joint="shoulder_pan", to=2500, apply=True, json_mode=True))

    payload = json.loads(capsys.readouterr().out)
    assert payload["move"]["overloaded"] is False


# ===========================================================================
# arm flex --demo --apply — sweep-level overload
# ===========================================================================


def test_flex_demo_apply_overload_json_surfaces_report(monkeypatch, capsys) -> None:
    fake = FakeBus(positions={i: 2048 for i in range(1, 7)}).fail_with_overload_on_op(8)
    _patch_bus(monkeypatch, fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    # Must not raise (h7).
    arm_cmd.cmd_arm_flex(_flex_args(demo=True, apply=True, json_mode=True))

    captured = capsys.readouterr()
    assert captured.err == ""
    payload = json.loads(captured.out)
    report = payload["demo"]
    assert report["aborted_on_overload"] is True
    assert report["overloaded_joint"] == "shoulder_pan"
    assert report["joints"]["shoulder_pan"]["overloaded"] is True
    # No joint after the overloaded one was ever visited.
    assert set(report["joints"]) == {"shoulder_pan"}


def test_flex_demo_apply_overload_text_marks_joint_and_summarizes(monkeypatch, capsys) -> None:
    fake = FakeBus(positions={i: 2048 for i in range(1, 7)}).fail_with_overload_on_op(8)
    _patch_bus(monkeypatch, fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_flex(_flex_args(demo=True, apply=True))

    out = capsys.readouterr().out
    assert "[OVERLOAD]" in out
    assert "Sweep aborted on overload at joint: shoulder_pan." in out


def test_flex_demo_apply_contact_text_marks_joint_and_summarizes(monkeypatch, capsys) -> None:
    """Sibling of the overload text test above, for the CONTACT branch: the
    first swept joint ramps its present_load past threshold, so the sweep aborts
    on contact there and the text render carries the ``[CONTACT]`` marker plus a
    contact-abort summary (exercises _emit_flex_demo's elif/contact path)."""
    # shoulder_pan (motor 1) is first in follower sweep order; ramp it past the
    # default 250 threshold on the very first step so it contacts immediately.
    fake = _RampLoadBus(
        positions={i: 2048 for i in range(1, 7)},
        load_increment_by_motor={1: 300},
    )
    _patch_bus(monkeypatch, fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_flex(_flex_args(demo=True, apply=True))

    out = capsys.readouterr().out
    assert "[OVERLOAD]" not in out
    assert "[CONTACT]" in out
    assert "Sweep aborted on contact at joint: shoulder_pan." in out


def test_flex_demo_apply_happy_path_no_overload_marker(monkeypatch, capsys) -> None:
    """Regression: a clean sweep still renders the pre-existing no-contact summary,
    now updated to also mention "no overload" (see arm.py's _emit_flex_demo)."""
    fake = FakeBus(positions={i: 2048 for i in range(1, 7)})
    _patch_bus(monkeypatch, fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_flex(_flex_args(demo=True, apply=True))

    out = capsys.readouterr().out
    assert "[OVERLOAD]" not in out
    assert "[CONTACT]" not in out
    assert "no contact" in out


# ===========================================================================
# Non-TTY, --apply, --json overload: consumable JSON, never a traceback
# ===========================================================================


def test_flex_non_tty_apply_json_overload_never_raises_and_is_consumable(
    monkeypatch, capsys
) -> None:
    """h7, explicit: an agent-mode (non-TTY, --apply) overload must reach the
    operator as consumable JSON on stdout, never a raw traceback."""
    fake = FakeBus(positions={1: 2048}).fail_with_overload_on_op(8)
    _patch_bus(monkeypatch, fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    # No pytest.raises: a raise here would fail the test outright.
    arm_cmd.cmd_arm_flex(
        _flex_args(joint="shoulder_pan", to=2500, gentle=True, apply=True, json_mode=True)
    )

    captured = capsys.readouterr()
    assert captured.err == ""
    payload = json.loads(captured.out)  # must be valid, parseable JSON
    assert payload["move"]["overloaded"] is True
