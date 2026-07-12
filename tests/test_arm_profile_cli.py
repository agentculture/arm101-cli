"""Tests for ``arm profile`` — the CLI verb that finds each joint's safe max speed.

All hardware is a :class:`tests._fakes.ServoModelBus` injected through the
``arm._open_bus`` / ``arm._candidate_ports`` seam (the same seam ``read`` /
``flex`` / ``explore`` use), so no serial port is ever opened.

Coverage:
- registration: ``arm profile`` under the arm noun, with its flags/defaults, and
  ``--contact-to`` being genuinely required.
- motion gating (mirrors flex/explore): non-TTY without ``--apply`` -> dry-run
  plan, **zero bus writes**; non-TTY ``--apply`` -> runs; TTY -> prompt.
- the crux, at CLI level: a speed at which contact detection fails is REJECTED,
  the last good speed is reported, and BOTH the text table and the ``--json``
  payload say so.
- torque ownership: an abnormal exit releases the joint (wave-1 ``torque_guard``).
"""

from __future__ import annotations

import argparse
import json
import sys

import pytest

from arm101.cli._commands import arm as arm_cmd
from arm101.cli._errors import EXIT_USER_ERROR, CliError
from arm101.hardware import arm_spec
from arm101.hardware.bus import FakeBus
from arm101.hardware.gentle import CONTACT_LOAD_CEILING
from arm101.hardware.profile import (
    DEFAULT_SPEED_MAX,
    DEFAULT_SPEED_START,
    DEFAULT_SPEED_STEP,
    REASON_CONTACT_DETECTED,
    REASON_CONTACT_MISSED,
)

from ._fakes import ServoModelBus
from .test_profile import CONTACT_TARGET, HOME, OBSTACLE, _SpeedCoupledArm

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


def _patch_bus(monkeypatch, fake: FakeBus, port: str = "/dev/ttyACM_fake") -> None:
    """Patch the arm bus seam so it opens *fake* at *port*."""
    fake.open()
    monkeypatch.setattr(arm_cmd, "_candidate_ports", lambda: [port])
    monkeypatch.setattr(arm_cmd, "_open_bus", lambda _p: fake)


def _blocked_arm(detection_ceiling: int = 250) -> ServoModelBus:
    """The follower's shoulder_pan (motor 1) at HOME, blocked at OBSTACLE.

    ``detection_ceiling`` is the speed above which the joint drives so deep into
    the compliant contact that it never reads as stopped and the stall rule can no
    longer fire — see :class:`tests.test_profile._SpeedCoupledArm`.
    """
    bus = _SpeedCoupledArm(positions={1: HOME}, detection_ceiling=detection_ceiling)
    bus.open()
    bus.place_obstacle(1, OBSTACLE)
    return bus


def _profile_args(
    joint: "str | None" = "shoulder_pan",
    contact_to: "int | None" = CONTACT_TARGET,
    threshold: "int | None" = None,
    speed_start: "int | None" = 150,
    speed_step: "int | None" = 150,
    speed_max: "int | None" = 300,
    role: str = "follower",
    port: "str | None" = None,
    apply: bool = False,
    json_mode: bool = False,
):
    """A namespace for the handler. The default ladder is (150, 300) — two rungs, fast."""
    return argparse.Namespace(
        joint=joint,
        contact_to=contact_to,
        threshold=threshold,
        speed_start=speed_start,
        speed_step=speed_step,
        speed_max=speed_max,
        role=role,
        port=port,
        apply=apply,
        json=json_mode,
    )


# ===========================================================================
# Registration
# ===========================================================================


def test_register_profile_verb() -> None:
    top = argparse.ArgumentParser(prog="arm101")
    sub = top.add_subparsers()
    arm_cmd.register(sub)

    args = top.parse_args(["arm", "profile", "shoulder_pan", "--contact-to", "3500", "--apply"])
    assert args.func is arm_cmd.cmd_arm_profile
    assert args.joint == "shoulder_pan"
    assert args.contact_to == 3500
    assert args.role == "follower"
    assert args.apply is True
    # The ladder knobs default to None and fall through to the module defaults.
    assert args.speed_start is None
    assert args.speed_step is None
    assert args.speed_max is None
    assert args.threshold is None


def test_contact_to_is_required() -> None:
    """A profile without a real contact to detect is not a profile."""
    top = argparse.ArgumentParser(prog="arm101")
    sub = top.add_subparsers()
    arm_cmd.register(sub)

    with pytest.raises(SystemExit):
        top.parse_args(["arm", "profile", "shoulder_pan"])


def test_register_profile_all_flags() -> None:
    top = argparse.ArgumentParser(prog="arm101")
    sub = top.add_subparsers()
    arm_cmd.register(sub)

    args = top.parse_args(
        [
            "arm",
            "profile",
            "wrist_roll",
            "--contact-to",
            "1200",
            "--threshold",
            "400",
            "--speed-start",
            "200",
            "--speed-step",
            "100",
            "--speed-max",
            "800",
            "--role",
            "leader",
            "--port",
            "/dev/ttyACM3",
            "--json",
        ]
    )
    assert args.joint == "wrist_roll"
    assert args.contact_to == 1200
    assert args.threshold == 400
    assert args.speed_start == 200
    assert args.speed_step == 100
    assert args.speed_max == 800
    assert args.role == "leader"
    assert args.port == "/dev/ttyACM3"
    assert args.json is True


def test_profile_is_listed_on_the_arm_noun(capsys) -> None:
    arm_cmd.cmd_arm_overview(argparse.Namespace(json=True))
    payload = json.loads(capsys.readouterr().out)
    assert "profile" in payload["verbs"]


# ===========================================================================
# Argument resolution
# ===========================================================================


def test_unknown_joint_raises_before_any_bus(monkeypatch) -> None:
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))
    with pytest.raises(CliError) as exc:
        arm_cmd.cmd_arm_profile(_profile_args(joint="not_a_joint"))
    assert exc.value.code == EXIT_USER_ERROR


def test_missing_contact_to_raises(monkeypatch) -> None:
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))
    with pytest.raises(CliError) as exc:
        arm_cmd.cmd_arm_profile(_profile_args(contact_to=None))
    assert exc.value.code == EXIT_USER_ERROR
    assert "--contact-to" in exc.value.message


def test_threshold_defaults_to_the_joints_hardware_tuned_value() -> None:
    args = _profile_args(joint="wrist_roll", threshold=None)
    assert arm_cmd._profile_threshold(args, "wrist_roll") == 400
    assert arm_cmd._profile_threshold(args, "wrist_roll") == (
        arm_spec.DEFAULT_CONTACT_THRESHOLDS["wrist_roll"]
    )


def test_threshold_flag_overrides_the_default() -> None:
    args = _profile_args(threshold=333)
    assert arm_cmd._profile_threshold(args, "shoulder_pan") == 333


@pytest.mark.parametrize("threshold", [CONTACT_LOAD_CEILING, CONTACT_LOAD_CEILING + 1, 900, 1000])
def test_threshold_at_or_above_the_torque_cap_is_refused(threshold: int) -> None:
    """A threshold that can NEVER fire must be refused, not silently attempted.

    ``present_load`` saturates at the servo's ``Torque_Limit``, which ``gentle_move``
    caps to ``CONTACT_LOAD_CEILING`` for the duration of every move, and contact
    requires ``load > threshold``. So at ``threshold >= CONTACT_LOAD_CEILING`` the
    inequality is unsatisfiable no matter how hard the arm pushes.

    The damage is not merely "it wouldn't work": every probe would come back
    reporting no contact, and the verb would then declare the FIRST rung a void run
    — "there was nothing there to detect" — while the joint was in fact pressed
    hard against a very real obstacle. A silent impossibility that lies about the
    physical world is far worse than a loud refusal.
    """
    args = _profile_args(threshold=threshold)
    with pytest.raises(CliError) as exc:
        arm_cmd._profile_threshold(args, "shoulder_pan")
    assert exc.value.code == EXIT_USER_ERROR
    assert str(CONTACT_LOAD_CEILING) in exc.value.remediation


@pytest.mark.parametrize("threshold", [0, -1, -250])
def test_threshold_at_or_below_zero_is_refused(threshold: int) -> None:
    """A non-positive threshold fires on free air — every move would read as contact."""
    args = _profile_args(threshold=threshold)
    with pytest.raises(CliError) as exc:
        arm_cmd._profile_threshold(args, "shoulder_pan")
    assert exc.value.code == EXIT_USER_ERROR


def test_the_highest_detectable_threshold_is_still_accepted() -> None:
    """The band is open at the top: one below the ceiling must still be usable."""
    args = _profile_args(threshold=CONTACT_LOAD_CEILING - 1)
    assert arm_cmd._profile_threshold(args, "shoulder_pan") == CONTACT_LOAD_CEILING - 1


def test_ladder_defaults_come_from_the_profile_module() -> None:
    args = _profile_args(speed_start=None, speed_step=None, speed_max=None)
    ladder = arm_cmd._resolve_ladder(args)
    assert ladder[0] == DEFAULT_SPEED_START
    assert ladder[1] - ladder[0] == DEFAULT_SPEED_STEP
    assert ladder[-1] <= DEFAULT_SPEED_MAX


def test_a_bad_ladder_is_a_user_error(monkeypatch) -> None:
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))
    with pytest.raises(CliError) as exc:
        arm_cmd.cmd_arm_profile(_profile_args(speed_step=0))
    assert exc.value.code == EXIT_USER_ERROR


# ===========================================================================
# Motion gating — mirrors flex/explore
# ===========================================================================


def test_profile_dry_run_performs_zero_bus_writes(monkeypatch, capsys) -> None:
    """Non-TTY without --apply: a plan only — zero motion, and the bus is never opened."""
    opened = {"v": False}

    def _open(_p):
        opened["v"] = True
        raise AssertionError("dry-run must not open a bus")

    monkeypatch.setattr(arm_cmd, "_candidate_ports", lambda: ["/dev/ttyACM_fake"])
    monkeypatch.setattr(arm_cmd, "_open_bus", _open)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_profile(_profile_args(apply=False))

    out = capsys.readouterr().out
    assert "Dry-run" in out
    assert opened["v"] is False


def test_profile_dry_run_writes_no_register_even_with_a_live_bus(monkeypatch, capsys) -> None:
    """Belt and braces: hand it a real (fake) bus and prove not one register moved."""
    bus = _blocked_arm()
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_profile(_profile_args(apply=False))

    capsys.readouterr()
    assert bus.register_writes == [], "a dry-run must perform ZERO bus writes"
    assert bus.position_writes == []
    assert bus.speed_writes == []
    assert bus.torque_writes == []


def test_profile_dry_run_json_plan(monkeypatch, capsys) -> None:
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_profile(_profile_args(apply=False, json_mode=True))

    plan = json.loads(capsys.readouterr().out)["plan"]
    assert plan["verb"] == "arm profile"
    assert plan["joint"] == "shoulder_pan"
    assert plan["motor"] == 1
    assert plan["contact_to"] == CONTACT_TARGET
    assert plan["ladder"] == [150, 300]
    assert plan["threshold"] == arm_spec.DEFAULT_CONTACT_THRESHOLDS["shoulder_pan"]


def test_profile_tty_prompt_yes_runs(monkeypatch, capsys) -> None:
    bus = _blocked_arm(detection_ceiling=1000)
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["yes\n"], tty=True))

    arm_cmd.cmd_arm_profile(_profile_args())

    out = capsys.readouterr().out
    assert "arm profile shoulder_pan" in out
    assert len(bus.position_writes) > 0, "motion must have been commanded"


def test_profile_tty_prompt_no_aborts_with_zero_motion(monkeypatch, capsys) -> None:
    bus = _blocked_arm()
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["no\n"], tty=True))

    arm_cmd.cmd_arm_profile(_profile_args())

    out = capsys.readouterr().out
    assert "Aborted" in out
    assert bus.register_writes == [], "a declined prompt must command no motion"


# ===========================================================================
# THE CRUX, at CLI level
# ===========================================================================


def test_apply_rejects_the_speed_where_contact_detection_fails(monkeypatch, capsys) -> None:
    """The verb's whole reason for existing, asserted through the CLI surface.

    At 300 the joint moves — faster than at 150 — and the servo is perfectly
    happy. But it drives into a real obstacle, loads past its threshold, and the
    stall rule never fires. The verb must call that a FAILURE and report 150.
    """
    bus = _blocked_arm(detection_ceiling=250)
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_profile(_profile_args(apply=True, json_mode=True))

    payload = json.loads(capsys.readouterr().out)
    assert payload["verb"] == "arm profile"
    assert payload["joint"] == "shoulder_pan"
    assert payload["certified"] is True
    assert payload["safe_speed"] == 150, "the last speed at which contact was still DETECTED"
    assert payload["ceiling_speed"] == 300
    assert payload["ceiling_reason"] == REASON_CONTACT_MISSED

    reasons = [t["reason"] for t in payload["trials"]]
    assert reasons == [REASON_CONTACT_DETECTED, REASON_CONTACT_MISSED]

    # The rejected speed loaded the joint past its threshold — it MET the obstacle —
    # and the servo never overloaded. A "did it survive?" check would have passed it.
    rejected = payload["trials"][1]
    assert rejected["overloaded"] is False
    assert rejected["peak_load"] > payload["threshold"]
    assert rejected["contacted"] is False


def test_the_json_payload_carries_the_same_conclusions_as_the_text(monkeypatch, capsys) -> None:
    """Text and --json must agree, including on the uncomfortable parts."""
    bus = _blocked_arm(detection_ceiling=250)
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))
    arm_cmd.cmd_arm_profile(_profile_args(apply=True, json_mode=True))
    payload = json.loads(capsys.readouterr().out)

    bus2 = _blocked_arm(detection_ceiling=250)
    _patch_bus(monkeypatch, bus2)
    arm_cmd.cmd_arm_profile(_profile_args(apply=True, json_mode=False))
    text = capsys.readouterr().out

    assert str(payload["safe_speed"]) in text
    assert "CONTACT IS STILL DETECTED" in text
    assert str(payload["ceiling_speed"]) in text
    assert str(payload["ceiling_reason"]) in text
    # Every trial's verdict is visible in the table, not just the summary.
    for trial in payload["trials"]:
        assert str(trial["speed"]) in text
        assert trial["reason"] in text
    assert "REJECT" in text


def test_measurements_are_reported_for_the_safe_speed(monkeypatch, capsys) -> None:
    """The three numbers t14 needs: safe speed, ticks/second, motion-onset latency."""
    bus = _blocked_arm(detection_ceiling=1000)
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_profile(_profile_args(apply=True, json_mode=True))

    payload = json.loads(capsys.readouterr().out)
    assert payload["safe_speed"] == 300
    assert set(payload) >= {"safe_speed", "ticks_per_second", "motion_onset_seconds"}
    # A fake bus advances per read, not per second, so the verb has no wall-clock to
    # measure against — and says so, rather than reporting the ~4,000,000 ticks/s the
    # arithmetic would otherwise produce. Real timings come from the hardware run (t15);
    # the deterministic timing assertions live in tests/test_profile.py, where a clock
    # ticking one poll interval per sample is injected.
    assert payload["ticks_per_second"] is None
    assert payload["motion_onset_seconds"] is None


def test_a_reachable_contact_target_voids_the_run(monkeypatch, capsys) -> None:
    """No obstacle => no evidence => a loud user error, not a cheerful "safe speed"."""
    bus = _SpeedCoupledArm(positions={1: HOME}, detection_ceiling=1000)  # no obstacle
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    with pytest.raises(CliError) as exc:
        arm_cmd.cmd_arm_profile(_profile_args(apply=True))

    assert exc.value.code == EXIT_USER_ERROR
    assert "nothing there to detect" in exc.value.message
    capsys.readouterr()


# ===========================================================================
# Progress narration + stream split
# ===========================================================================


def test_progress_goes_to_stderr_and_the_result_to_stdout(monkeypatch, capsys) -> None:
    bus = _blocked_arm(detection_ceiling=250)
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_profile(_profile_args(apply=True))

    captured = capsys.readouterr()
    assert "speed 150: accepted" in captured.err
    assert "speed 300: REJECTED" in captured.err
    assert "speed 150: accepted" not in captured.out, "diagnostics must never reach stdout"
    assert "## arm profile" in captured.out


def test_json_progress_lines_are_json_objects(monkeypatch, capsys) -> None:
    bus = _blocked_arm(detection_ceiling=250)
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_profile(_profile_args(apply=True, json_mode=True))

    err = capsys.readouterr().err
    trials = [json.loads(line)["trial"] for line in err.splitlines() if line.strip()]
    assert [t["speed"] for t in trials] == [150, 300]
    assert trials[0]["accepted"] is True
    assert trials[1]["accepted"] is False


# ===========================================================================
# Torque ownership (wave-1 torque_guard) — #33
# ===========================================================================


def test_an_abnormal_exit_releases_the_joint(monkeypatch, capsys) -> None:
    """A bus fault mid-ramp must not walk away from a joint pressed into a wall."""

    class _DyingArm(_SpeedCoupledArm):
        def __init__(self, *args, **kwargs) -> None:
            super().__init__(*args, **kwargs)
            self.reads = 0

        def read_info(self, motor: int) -> dict:
            self.reads += 1
            if self.reads > 12:
                raise RuntimeError("simulated USB unplug")  # NOT a CliError — like pyserial
            return super().read_info(motor)

    bus = _DyingArm(positions={1: HOME}, detection_ceiling=1000)
    bus.open()
    bus.place_obstacle(1, OBSTACLE)
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    with pytest.raises(RuntimeError, match="simulated USB unplug"):
        arm_cmd.cmd_arm_profile(_profile_args(apply=True))

    err = capsys.readouterr().err
    assert "Torque released on motors 1" in err, "the guard must announce the release"
    assert bus.torque_writes[-1] == {"motor": 1, "on": False}


def test_the_guard_owns_only_the_profiled_joint(monkeypatch, capsys) -> None:
    """One joint is profiled, so one motor is energised — and exactly one is claimed."""

    class _DyingArm(_SpeedCoupledArm):
        def read_info(self, motor: int) -> dict:
            raise RuntimeError("dead on arrival")

    bus = _DyingArm(positions={2: HOME}, detection_ceiling=1000)
    bus.open()
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    with pytest.raises(RuntimeError):
        arm_cmd.cmd_arm_profile(_profile_args(joint="shoulder_lift", apply=True))

    err = capsys.readouterr().err
    # shoulder_lift is motor 2 — and no other motor is touched or claimed.
    assert "motors 2" in err
    assert {w["motor"] for w in bus.torque_writes} == {2}
