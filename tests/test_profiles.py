"""Tests for arm101.hardware.profiles — calibration Profile schema + XDG persistence."""

from __future__ import annotations

import json

import pytest

from arm101.cli._errors import CliError
from arm101.hardware.profiles import (
    JOINTS,
    JointCalibration,
    Profile,
    load,
    profile_path,
    save,
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_profile() -> Profile:
    """Build a Profile with distinct per-joint values so equality is meaningful."""
    joints = {}
    for i, name in enumerate(JOINTS):
        # Use distinct, in-order values: min < mid < max, all 12-bit valid
        base = i * 100
        joints[name] = JointCalibration(min=base, mid=base + 50, max=base + 99)
    return Profile(joints=joints)


# ---------------------------------------------------------------------------
# Import-clean guard
# ---------------------------------------------------------------------------


def test_import_clean():
    """arm101.hardware.profiles must import with no third-party packages."""
    import arm101.hardware.profiles  # noqa: F401 — just verifying no ImportError


# ---------------------------------------------------------------------------
# JOINTS constant
# ---------------------------------------------------------------------------


def test_joints_constant_order():
    expected = [
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
        "gripper",
    ]
    assert list(JOINTS) == expected


# ---------------------------------------------------------------------------
# Round-trip: save → load yields identical per-joint values
# ---------------------------------------------------------------------------


def test_round_trip(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    original = _make_profile()
    save(original, "test_arm")
    loaded = load("test_arm")

    for joint in JOINTS:
        orig_cal = original.joints[joint]
        load_cal = loaded.joints[joint]
        assert load_cal.min == orig_cal.min, f"{joint}.min mismatch"
        assert load_cal.mid == orig_cal.mid, f"{joint}.mid mismatch"
        assert load_cal.max == orig_cal.max, f"{joint}.max mismatch"


# ---------------------------------------------------------------------------
# XDG_CONFIG_HOME override — file lands in the right place
# ---------------------------------------------------------------------------


def test_xdg_config_home_honored(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    profile = _make_profile()
    save(profile, "xdg_test")

    expected = tmp_path / "arm101" / "calibrations" / "xdg_test.json"
    assert expected.exists(), f"Expected file not found: {expected}"


def test_profile_path_uses_xdg(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    path = profile_path("my_arm")
    assert path == tmp_path / "arm101" / "calibrations" / "my_arm.json"


def test_profile_path_falls_back_to_home(monkeypatch):
    monkeypatch.delenv("XDG_CONFIG_HOME", raising=False)
    monkeypatch.setenv("HOME", "/fakehome")
    path = profile_path("arm")
    assert str(path) == "/fakehome/.config/arm101/calibrations/arm.json"


# ---------------------------------------------------------------------------
# Missing id → CliError, not bare OSError
# ---------------------------------------------------------------------------


def test_load_missing_raises_cli_error(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    with pytest.raises(CliError) as exc_info:
        load("does_not_exist")
    # Must be a user-level or env-level error (code 1 or 2), with a remediation hint
    assert exc_info.value.code in (1, 2)
    assert exc_info.value.remediation  # non-empty


def test_load_missing_not_file_not_found_error(tmp_path, monkeypatch):
    """Ensure FileNotFoundError does NOT propagate (must be wrapped in CliError)."""
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    with pytest.raises(CliError):
        load("ghost_profile")
    # If we got here the bare FileNotFoundError was suppressed — test passes


# ---------------------------------------------------------------------------
# JSON file structure sanity
# ---------------------------------------------------------------------------


def test_json_file_structure(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    profile = _make_profile()
    save(profile, "struct_test")

    path = tmp_path / "arm101" / "calibrations" / "struct_test.json"
    data = json.loads(path.read_text())

    for joint in JOINTS:
        assert joint in data, f"Joint {joint!r} missing from JSON"
        entry = data[joint]
        assert set(entry.keys()) >= {"min", "mid", "max"}
        assert isinstance(entry["min"], int)
        assert isinstance(entry["mid"], int)
        assert isinstance(entry["max"], int)


# ---------------------------------------------------------------------------
# Parent dirs created on save (idempotent)
# ---------------------------------------------------------------------------


def test_save_creates_parent_dirs(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    profile = _make_profile()
    # Calibrations dir should not exist yet
    cal_dir = tmp_path / "arm101" / "calibrations"
    assert not cal_dir.exists()
    save(profile, "dir_test")
    assert cal_dir.is_dir()


# ---------------------------------------------------------------------------
# Profile id validation — reject path traversal (security)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "bad_id",
    [
        "../evil",
        "../../etc/passwd",
        "sub/dir",
        "a/../b",
        "..",
        "",
        ".hidden",
        "back\\slash",
        "name with space",
    ],
)
def test_profile_path_rejects_unsafe_ids(bad_id, monkeypatch, tmp_path):
    """Path separators, '..', leading-dot, empty, and odd chars are rejected."""
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    with pytest.raises(CliError) as exc:
        profile_path(bad_id)
    assert exc.value.code == 1  # EXIT_USER_ERROR
    # save()/load() route through profile_path, so they reject too.
    with pytest.raises(CliError):
        load(bad_id)


@pytest.mark.parametrize("ok_id", ["my-arm", "arm_1", "L1", "a.b", "Arm01"])
def test_profile_path_accepts_safe_ids(ok_id, monkeypatch, tmp_path):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    path = profile_path(ok_id)
    assert path.name == f"{ok_id}.json"
    assert path.parent == tmp_path / "arm101" / "calibrations"
