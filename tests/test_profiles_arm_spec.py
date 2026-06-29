"""Tests that profiles.JOINTS is derived from arm_spec (t4, #17).

Acceptance criteria:
  1. profiles.JOINTS equals the canonical six-joint tuple — unchanged value.
  2. profiles.JOINTS is the same object as arm_spec.JOINTS (derived, not copied).
  3. A profile round-trips through save/load with equal per-joint values.
"""

from __future__ import annotations

from arm101.hardware import arm_spec, profiles
from arm101.hardware.profiles import JointCalibration, Profile, load, save

# ---------------------------------------------------------------------------
# 1 + 2: JOINTS derivation
# ---------------------------------------------------------------------------


def test_profiles_joints_value():
    """profiles.JOINTS must equal the canonical six-joint tuple."""
    expected = (
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
        "gripper",
    )
    assert profiles.JOINTS == expected


def test_profiles_joints_is_arm_spec_joints():
    """profiles.JOINTS must be derived from arm_spec.JOINTS (same object)."""
    assert profiles.JOINTS is arm_spec.JOINTS


# ---------------------------------------------------------------------------
# 3: Round-trip via profiles read/write helpers
# ---------------------------------------------------------------------------


def _make_profile() -> Profile:
    joints = {}
    for i, name in enumerate(profiles.JOINTS):
        base = (i + 1) * 200
        joints[name] = JointCalibration(min=base, mid=base + 100, max=base + 199)
    return Profile(joints=joints)


def test_profile_round_trip(tmp_path, monkeypatch):
    """save → load must return a profile with identical per-joint tick values."""
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    original = _make_profile()
    save(original, "arm_spec_rt")
    loaded = load("arm_spec_rt")

    for joint in profiles.JOINTS:
        orig_cal = original.joints[joint]
        load_cal = loaded.joints[joint]
        assert load_cal.min == orig_cal.min, f"{joint}.min mismatch"
        assert load_cal.mid == orig_cal.mid, f"{joint}.mid mismatch"
        assert load_cal.max == orig_cal.max, f"{joint}.max mismatch"
