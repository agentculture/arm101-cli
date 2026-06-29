"""Tests for ``arm101 arm`` — arm noun group (overview + setup <role>).

Covers:
- ``arm overview``: exits 0 on any path, accepts ignored positional target,
  text and JSON output shape.
- ``arm setup follower/leader``: dry-run (zero catalog writes), agent mode
  (--apply, non-TTY), and interactive mode (TTY, Enter gate).
- Catalog contents after --apply: F1-F6 with gear 1:345 (follower), L1-L6
  with mixed gears 1:191/1:345/1:147 (leader), correct servo_model per joint.

Seam strategy (mirrors test_setup_motors.py):
- ``calibrate_motor._candidate_ports`` -> ``["/dev/ttyACM_fake"]``
- ``calibrate_motor._open_bus`` -> factory returning the shared FakeBus
  (re-opens on each call so bus.close() in the walk does not leave it closed)
- ``XDG_CONFIG_HOME`` -> ``str(tmp_path)`` to isolate the catalog
"""

from __future__ import annotations

import argparse
import json
import sys

from arm101.cli._commands import arm as arm_cmd
from arm101.cli._commands import calibrate_motor as cm
from arm101.hardware import arm_spec, motor_catalog
from arm101.hardware.bus import FakeBus

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


class _FakeStdin:
    """Scripted stdin for consent-mode control."""

    def __init__(self, lines: list[str], tty: bool = True) -> None:
        self._lines = iter(lines)
        self._tty = tty

    def isatty(self) -> bool:
        return self._tty

    def readline(self) -> str:
        try:
            return next(self._lines)
        except StopIteration:
            return ""  # EOF


def _make_args(
    role: str = "follower",
    json_mode: bool = False,
    port: "str | None" = None,
    apply: bool = False,
) -> argparse.Namespace:
    """Return a minimal Namespace matching the fields ``arm setup`` expects."""
    return argparse.Namespace(
        role=role,
        json=json_mode,
        port=port,
        apply=apply,
    )


def _patch_detection(monkeypatch, fake: FakeBus) -> None:
    """Patch calibrate_motor seams so _detect_one_motor returns *fake*.

    The factory re-opens *fake* on each call so the bus is still ``_open``
    after the previous motor's ``bus.close()`` set it to False.
    """
    monkeypatch.setattr(cm, "_candidate_ports", lambda: ["/dev/ttyACM_fake"])

    def _open(port: str) -> FakeBus:
        fake.open()
        return fake

    monkeypatch.setattr(cm, "_open_bus", _open)


# ---------------------------------------------------------------------------
# arm overview — always exits 0, accepts ignored target, text + JSON
# ---------------------------------------------------------------------------


def test_overview_exits_0() -> None:
    """arm overview exits 0 unconditionally."""
    ns = argparse.Namespace(json=False, target=None)
    assert arm_cmd.cmd_arm_overview(ns) == 0


def test_overview_exits_0_with_ignored_target() -> None:
    """arm overview with any positional target still exits 0."""
    ns = argparse.Namespace(json=False, target="something-irrelevant")
    assert arm_cmd.cmd_arm_overview(ns) == 0


def test_overview_text_mentions_verbs_and_roles(capsys) -> None:
    """Text output names 'overview', 'setup', 'follower', and 'leader'."""
    arm_cmd.cmd_arm_overview(argparse.Namespace(json=False, target=None))
    out = capsys.readouterr().out
    assert "overview" in out
    assert "setup" in out
    assert "follower" in out
    assert "leader" in out


def test_overview_text_mentions_all_joints(capsys) -> None:
    """Text output includes every joint name from arm_spec.JOINTS."""
    arm_cmd.cmd_arm_overview(argparse.Namespace(json=False, target=None))
    out = capsys.readouterr().out
    for joint in arm_spec.JOINTS:
        assert joint in out, f"Joint {joint!r} missing from arm overview text"


def test_overview_json_structure(capsys) -> None:
    """--json emits the expected top-level structure."""
    arm_cmd.cmd_arm_overview(argparse.Namespace(json=True, target=None))
    payload = json.loads(capsys.readouterr().out)

    assert payload["noun"] == "arm"
    assert "overview" in payload["verbs"]
    assert "setup" in payload["verbs"]
    assert set(payload["roles"]) == {"follower", "leader"}
    assert "follower" in payload["motor_map"]
    assert "leader" in payload["motor_map"]


def test_overview_json_follower_shoulder_pan(capsys) -> None:
    """Follower shoulder_pan: id=1, gear_ratio=1:345."""
    arm_cmd.cmd_arm_overview(argparse.Namespace(json=True, target=None))
    payload = json.loads(capsys.readouterr().out)
    entry = payload["motor_map"]["follower"]["shoulder_pan"]
    assert entry["id"] == 1
    assert entry["gear_ratio"] == "1:345"


def test_overview_json_leader_shoulder_pan(capsys) -> None:
    """Leader shoulder_pan: id=1, gear_ratio=1:191."""
    arm_cmd.cmd_arm_overview(argparse.Namespace(json=True, target=None))
    payload = json.loads(capsys.readouterr().out)
    entry = payload["motor_map"]["leader"]["shoulder_pan"]
    assert entry["id"] == 1
    assert entry["gear_ratio"] == "1:191"


def test_overview_registered_and_reachable(capsys) -> None:
    """``register`` attaches overview as a subparser with the right func."""
    top = argparse.ArgumentParser(prog="arm101")
    sub = top.add_subparsers()
    arm_cmd.register(sub)

    args = top.parse_args(["arm", "overview"])
    assert args.func is arm_cmd.cmd_arm_overview


def test_overview_accepts_target_positional() -> None:
    """``arm overview <target>`` parses target without error."""
    top = argparse.ArgumentParser(prog="arm101")
    sub = top.add_subparsers()
    arm_cmd.register(sub)

    args = top.parse_args(["arm", "overview", "anything"])
    assert args.target == "anything"
    assert args.func is arm_cmd.cmd_arm_overview


# ---------------------------------------------------------------------------
# arm setup — dry_run: non-TTY, no --apply -> plan only, zero writes
# ---------------------------------------------------------------------------


def test_setup_dry_run_writes_no_catalog_follower(monkeypatch, tmp_path) -> None:
    """Dry-run mode for follower writes nothing to the catalog."""
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_setup(_make_args(role="follower", apply=False))

    assert motor_catalog.load_catalog() == {}


def test_setup_dry_run_writes_no_catalog_leader(monkeypatch, tmp_path) -> None:
    """Dry-run mode for leader writes nothing to the catalog."""
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_setup(_make_args(role="leader", apply=False))

    assert motor_catalog.load_catalog() == {}


def test_setup_dry_run_no_eeprom_writes(monkeypatch, tmp_path) -> None:
    """Dry-run mode opens no bus and performs no EEPROM writes."""
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    bus_opened = [False]

    def _open(port: str) -> FakeBus:
        bus_opened[0] = True
        b = FakeBus(ids=[1])
        b.open()
        return b

    monkeypatch.setattr(cm, "_candidate_ports", lambda: ["/dev/ttyACM_fake"])
    monkeypatch.setattr(cm, "_open_bus", _open)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_setup(_make_args(role="follower", apply=False))

    assert bus_opened[0] is False, "dry_run must not open any bus"


def test_setup_dry_run_text_output_follower(monkeypatch, capsys) -> None:
    """Dry-run text output for follower mentions all joints and 'Dry-run plan'."""
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))
    arm_cmd.cmd_arm_setup(_make_args(role="follower"))
    out = capsys.readouterr().out
    assert "Dry-run plan" in out
    assert "follower" in out
    for joint in arm_spec.JOINTS:
        assert joint in out, f"Joint {joint!r} missing from dry-run output"


def test_setup_dry_run_json_follower(monkeypatch, capsys) -> None:
    """Dry-run JSON for follower: 6 entries, all gear_ratio 1:345, labels F1-F6."""
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))
    arm_cmd.cmd_arm_setup(_make_args(role="follower", json_mode=True))
    payload = json.loads(capsys.readouterr().out)

    assert payload["role"] == "follower"
    plan = payload["plan"]
    assert len(plan) == 6
    for entry in plan:
        assert entry["gear_ratio"] == "1:345"
        assert entry["label"].startswith("F")
        assert entry["baudrate"] == arm_spec.DEFAULT_BAUDRATE


def test_setup_dry_run_json_leader(monkeypatch, capsys) -> None:
    """Dry-run JSON for leader: 6 entries, mixed gear_ratios, labels L1-L6."""
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))
    arm_cmd.cmd_arm_setup(_make_args(role="leader", json_mode=True))
    payload = json.loads(capsys.readouterr().out)

    assert payload["role"] == "leader"
    plan = payload["plan"]
    assert len(plan) == 6
    for entry in plan:
        assert entry["label"].startswith("L")
    gear_ratios = {e["gear_ratio"] for e in plan}
    assert "1:191" in gear_ratios
    assert "1:345" in gear_ratios
    assert "1:147" in gear_ratios


# ---------------------------------------------------------------------------
# arm setup follower --apply (agent mode: non-TTY + --apply)
# ---------------------------------------------------------------------------


def test_setup_follower_apply_catalog_entries(monkeypatch, tmp_path) -> None:
    """After arm setup follower --apply, catalog holds F1-F6, all gear 1:345."""
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    fake = FakeBus(ids=[1])
    _patch_detection(monkeypatch, fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_setup(_make_args(role="follower", apply=True))

    catalog = motor_catalog.load_catalog()
    assert len(catalog) == 6

    for i in range(1, 7):
        label = f"F{i}"
        assert label in catalog, f"{label} missing from catalog"
        entry = catalog[label]
        assert (
            entry.gear_ratio == "1:345"
        ), f"{label}: expected gear_ratio 1:345, got {entry.gear_ratio!r}"
        assert (
            entry.servo_model == "ST-3215-C001/C018/C047"
        ), f"{label}: expected servo_model ST-3215-C001/C018/C047, got {entry.servo_model!r}"
        assert entry.detected_id == 1, f"{label}: FakeBus reports id=1"
        assert entry.detected_model == 777, f"{label}: FakeBus default model=777"
        assert entry.port == "/dev/ttyACM_fake", f"{label}: expected fake port"


def test_setup_follower_apply_eeprom_writes(monkeypatch, tmp_path) -> None:
    """arm setup follower --apply drives 6 EEPROM writes via the walk."""
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    fake = FakeBus(ids=[1])
    _patch_detection(monkeypatch, fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_setup(_make_args(role="follower", apply=True))

    assert len(fake.eeprom_writes) == 6


def test_setup_follower_apply_joint_names(monkeypatch, tmp_path) -> None:
    """Each follower catalog entry has the correct joint name from arm_spec."""
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    fake = FakeBus(ids=[1])
    _patch_detection(monkeypatch, fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_setup(_make_args(role="follower", apply=True))

    catalog = motor_catalog.load_catalog()
    for joint in arm_spec.JOINTS:
        spec = arm_spec.motor_spec("follower", joint)
        label = f"F{spec.id}"
        assert (
            catalog[label].joint == joint
        ), f"{label}: expected joint {joint!r}, got {catalog[label].joint!r}"


# ---------------------------------------------------------------------------
# arm setup leader --apply (agent mode: non-TTY + --apply)
# ---------------------------------------------------------------------------


def test_setup_leader_apply_catalog_entries(monkeypatch, tmp_path) -> None:
    """After arm setup leader --apply, catalog holds L1-L6 with correct specs."""
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    fake = FakeBus(ids=[1])
    _patch_detection(monkeypatch, fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_setup(_make_args(role="leader", apply=True))

    catalog = motor_catalog.load_catalog()
    assert len(catalog) == 6

    for joint in arm_spec.JOINTS:
        spec = arm_spec.motor_spec("leader", joint)
        label = f"L{spec.id}"
        assert label in catalog, f"{label} missing from catalog"
        entry = catalog[label]
        assert entry.gear_ratio == spec.gear_ratio, (
            f"{label} ({joint}): expected gear_ratio {spec.gear_ratio!r},"
            f" got {entry.gear_ratio!r}"
        )
        assert entry.servo_model == spec.servo_model, (
            f"{label} ({joint}): expected servo_model {spec.servo_model!r},"
            f" got {entry.servo_model!r}"
        )
        assert entry.joint == joint
        assert entry.detected_id == 1
        assert entry.detected_model == 777
        assert entry.port == "/dev/ttyACM_fake"


def test_setup_leader_gear_ratios_all_present(monkeypatch, tmp_path) -> None:
    """Leader catalog has 1:191, 1:345, and 1:147 gears among L1-L6."""
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    fake = FakeBus(ids=[1])
    _patch_detection(monkeypatch, fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_setup(_make_args(role="leader", apply=True))

    catalog = motor_catalog.load_catalog()
    gears = {catalog[f"L{i}"].gear_ratio for i in range(1, 7)}
    assert "1:191" in gears, "Expected at least one 1:191 gear in leader catalog"
    assert "1:345" in gears, "Expected at least one 1:345 gear in leader catalog"
    assert "1:147" in gears, "Expected at least one 1:147 gear in leader catalog"


def test_setup_leader_apply_eeprom_writes(monkeypatch, tmp_path) -> None:
    """arm setup leader --apply drives 6 EEPROM writes."""
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    fake = FakeBus(ids=[1])
    _patch_detection(monkeypatch, fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_setup(_make_args(role="leader", apply=True))

    assert len(fake.eeprom_writes) == 6


# ---------------------------------------------------------------------------
# arm setup interactive (TTY, Enter gate)
# ---------------------------------------------------------------------------


def test_setup_follower_interactive_catalog(monkeypatch, tmp_path) -> None:
    """Interactive TTY mode gates on Enter per motor and writes F1-F6 catalog."""
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    fake = FakeBus(ids=[1])
    _patch_detection(monkeypatch, fake)
    # 6 Enters, one per motor
    monkeypatch.setattr(sys, "stdin", _FakeStdin(["\n"] * 6, tty=True))

    arm_cmd.cmd_arm_setup(_make_args(role="follower"))

    assert len(fake.eeprom_writes) == 6
    catalog = motor_catalog.load_catalog()
    assert len(catalog) == 6
    for i in range(1, 7):
        assert f"F{i}" in catalog


# ---------------------------------------------------------------------------
# arm setup JSON output (agent mode)
# ---------------------------------------------------------------------------


def test_setup_follower_apply_json_output(monkeypatch, tmp_path, capsys) -> None:
    """--json emits {role, assigned:[{label, joint, servo_model, gear_ratio, ...}]}."""
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    fake = FakeBus(ids=[1])
    _patch_detection(monkeypatch, fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin([], tty=False))

    arm_cmd.cmd_arm_setup(_make_args(role="follower", apply=True, json_mode=True))

    payload = json.loads(capsys.readouterr().out)
    assert payload["role"] == "follower"
    assigned = payload["assigned"]
    assert len(assigned) == 6
    for entry in assigned:
        assert entry["label"].startswith("F")
        assert entry["gear_ratio"] == "1:345"
        assert entry["servo_model"] == "ST-3215-C001/C018/C047"
        assert entry["detected_model"] == 777
        assert entry["port"] == "/dev/ttyACM_fake"


# ---------------------------------------------------------------------------
# register
# ---------------------------------------------------------------------------


def test_register_arm_noun_group() -> None:
    """register() attaches 'arm' noun group with _no_verb as default handler."""
    top = argparse.ArgumentParser(prog="arm101")
    sub = top.add_subparsers()
    arm_cmd.register(sub)

    args = top.parse_args(["arm"])
    assert args.func is arm_cmd._no_verb


def test_register_setup_role_choices() -> None:
    """arm setup accepts 'follower' and 'leader'."""
    top = argparse.ArgumentParser(prog="arm101")
    sub = top.add_subparsers()
    arm_cmd.register(sub)

    args = top.parse_args(["arm", "setup", "follower"])
    assert args.role == "follower"
    assert args.func is arm_cmd.cmd_arm_setup

    args2 = top.parse_args(["arm", "setup", "leader"])
    assert args2.role == "leader"


def test_register_setup_apply_flag() -> None:
    """arm setup --apply flag is registered and defaults to False."""
    top = argparse.ArgumentParser(prog="arm101")
    sub = top.add_subparsers()
    arm_cmd.register(sub)

    args_no_apply = top.parse_args(["arm", "setup", "follower"])
    assert args_no_apply.apply is False

    args_apply = top.parse_args(["arm", "setup", "follower", "--apply"])
    assert args_apply.apply is True


def test_register_setup_json_flag() -> None:
    """arm setup --json flag is registered."""
    top = argparse.ArgumentParser(prog="arm101")
    sub = top.add_subparsers()
    arm_cmd.register(sub)

    args = top.parse_args(["arm", "setup", "follower", "--json"])
    assert args.json is True
