"""Tests for arm101.hardware.arm_spec — single-source per-role motor map.

Verifies:
* Both roles ("follower", "leader") are present.
* Every role maps all six joints to MotorSpec instances.
* joint→id is {shoulder_pan:1 .. gripper:6} for both roles.
* Baud is 1_000_000 for every motor in both roles.
* Follower gears are all "1:345" and servo model "ST-3215-C001/C018/C047".
* Leader's six (servo_model, gear_ratio) pairs match the Seeed SO-101 wiki BOM exactly.
* Accessor helpers (roles, joint_ids, motor_spec, role_motors) work correctly.
* Unknown role raises ValueError; unknown joint raises ValueError.
"""

import pytest

from arm101.hardware.arm_spec import (
    ARM_SPEC,
    DEFAULT_BAUDRATE,
    DEFAULT_CONTACT_THRESHOLDS,
    JOINTS,
    MotorSpec,
    joint_ids,
    motor_spec,
    resolve_contact_thresholds,
    role_motors,
    roles,
)
from arm101.hardware.gentle import _CONTACT_TORQUE_LIMIT

# ---------------------------------------------------------------------------
# Structural invariants
# ---------------------------------------------------------------------------


def test_both_roles_present():
    """ARM_SPEC must contain exactly the 'follower' and 'leader' roles."""
    assert set(roles()) == {"follower", "leader"}


def test_all_joints_present_in_each_role():
    """Each role must map all six joints defined in JOINTS."""
    for role in roles():
        assert set(ARM_SPEC[role].keys()) == set(JOINTS), f"Role {role!r} is missing some joints"


def test_joint_ids_are_1_through_6_for_both_roles():
    """joint_ids() must return {shoulder_pan:1 .. gripper:6} for both roles.

    Source: LeRobot so_follower.py / so_leader.py motors dict (commit 2f2b567).
    Follower and leader share identical id assignments.
    """
    expected = {
        "shoulder_pan": 1,
        "shoulder_lift": 2,
        "elbow_flex": 3,
        "wrist_flex": 4,
        "wrist_roll": 5,
        "gripper": 6,
    }
    for role in roles():
        assert joint_ids(role) == expected, f"joint_ids mismatch for role {role!r}"


def test_default_baudrate_constant():
    """DEFAULT_BAUDRATE must equal 1_000_000.

    Source: LeRobot feetech.py DEFAULT_BAUDRATE = 1_000_000 (commit 2f2b567).
    """
    assert DEFAULT_BAUDRATE == 1_000_000


def test_baud_is_1_000_000_for_all_motors():
    """All motors in all roles must have baud == 1_000_000.

    Source: LeRobot feetech.py DEFAULT_BAUDRATE = 1_000_000 (commit 2f2b567).
    """
    for role in roles():
        for joint, spec in ARM_SPEC[role].items():
            assert (
                spec.baud == 1_000_000
            ), f"role={role!r} joint={joint!r}: baud {spec.baud} != 1_000_000"


# ---------------------------------------------------------------------------
# Follower-specific assertions
# ---------------------------------------------------------------------------


def test_follower_all_gears_1_345():
    """Follower role: every joint must have gear_ratio '1:345'.

    Source: Seeed SO-101 wiki BOM, follower column
    (https://wiki.seeedstudio.com/lerobot_so100m_new/#configure-the-motors).
    """
    for joint, spec in ARM_SPEC["follower"].items():
        assert (
            spec.gear_ratio == "1:345"
        ), f"follower joint {joint!r}: gear_ratio {spec.gear_ratio!r} != '1:345'"


def test_follower_all_model_c001_c018_c047():
    """Follower role: every joint must use the C001/C018/C047 servo model.

    Source: Seeed SO-101 wiki BOM, follower column.
    """
    for joint, spec in ARM_SPEC["follower"].items():
        assert (
            spec.servo_model == "ST-3215-C001/C018/C047"
        ), f"follower joint {joint!r}: model {spec.servo_model!r}"


# ---------------------------------------------------------------------------
# Leader-specific assertions (exact Seeed BOM values, parametrised)
# ---------------------------------------------------------------------------

#: Expected (servo_model, gear_ratio) per leader joint.
#: Source: Seeed SO-101 wiki BOM, leader column.
#: https://wiki.seeedstudio.com/lerobot_so100m_new/#configure-the-motors
_LEADER_EXPECTED: dict[str, tuple[str, str]] = {
    "shoulder_pan": ("ST-3215-C044", "1:191"),  # C044, 1:191 — Seeed wiki BOM
    "shoulder_lift": ("ST-3215-C001", "1:345"),  # C001, 1:345 — Seeed wiki BOM
    "elbow_flex": ("ST-3215-C044", "1:191"),  # C044, 1:191 — Seeed wiki BOM
    "wrist_flex": ("ST-3215-C046", "1:147"),  # C046, 1:147 — Seeed wiki BOM
    "wrist_roll": ("ST-3215-C046", "1:147"),  # C046, 1:147 — Seeed wiki BOM
    "gripper": ("ST-3215-C046", "1:147"),  # C046, 1:147 — Seeed wiki BOM
}


@pytest.mark.parametrize("joint,expected", list(_LEADER_EXPECTED.items()))
def test_leader_motor_spec(joint: str, expected: tuple[str, str]):
    """Each leader joint must have the exact (servo_model, gear_ratio) from the Seeed BOM."""
    spec = motor_spec("leader", joint)
    expected_model, expected_gear = expected
    assert (
        spec.servo_model == expected_model
    ), f"leader {joint!r}: model {spec.servo_model!r} != {expected_model!r}"
    assert (
        spec.gear_ratio == expected_gear
    ), f"leader {joint!r}: gear {spec.gear_ratio!r} != {expected_gear!r}"


# ---------------------------------------------------------------------------
# Accessor helpers
# ---------------------------------------------------------------------------


def test_role_motors_returns_all_six():
    """role_motors() must return a dict with all six joints mapping to MotorSpec."""
    for role in roles():
        motors = role_motors(role)
        assert set(motors.keys()) == set(JOINTS)
        assert all(isinstance(v, MotorSpec) for v in motors.values())


def test_motor_spec_returns_correct_type_and_values():
    """motor_spec() must return a MotorSpec with the correct id and baud."""
    spec = motor_spec("follower", "shoulder_pan")
    assert isinstance(spec, MotorSpec)
    assert spec.id == 1  # LeRobot so_follower.py (commit 2f2b567)
    assert spec.baud == 1_000_000  # LeRobot feetech.py (commit 2f2b567)


def test_roles_returns_sorted_list():
    """roles() must return a sorted list containing both roles."""
    result = roles()
    assert result == sorted(result)
    assert "follower" in result
    assert "leader" in result


def test_joint_ids_values_are_contiguous_1_to_6():
    """For both roles the id set must be exactly {1, 2, 3, 4, 5, 6}."""
    for role in roles():
        ids = set(joint_ids(role).values())
        assert ids == {1, 2, 3, 4, 5, 6}, f"role {role!r}: ids {ids}"


# ---------------------------------------------------------------------------
# Error handling
# ---------------------------------------------------------------------------


def test_unknown_role_in_joint_ids_raises():
    """joint_ids() must raise ValueError for an unknown role."""
    with pytest.raises(ValueError, match="Unknown arm role"):
        joint_ids("invalid_role")


def test_unknown_role_in_motor_spec_raises():
    """motor_spec() must raise ValueError for an unknown role."""
    with pytest.raises(ValueError, match="Unknown arm role"):
        motor_spec("invalid_role", "shoulder_pan")


def test_unknown_joint_in_motor_spec_raises():
    """motor_spec() must raise ValueError for an unknown joint."""
    with pytest.raises(ValueError, match="Unknown joint"):
        motor_spec("follower", "nonexistent_joint")


def test_unknown_role_in_role_motors_raises():
    """role_motors() must raise ValueError for an unknown role."""
    with pytest.raises(ValueError, match="Unknown arm role"):
        role_motors("invalid_role")


# ---------------------------------------------------------------------------
# DEFAULT_CONTACT_THRESHOLDS — structural invariants
# ---------------------------------------------------------------------------


def test_default_contact_thresholds_covers_every_joint():
    """DEFAULT_CONTACT_THRESHOLDS must have exactly one entry per JOINTS."""
    assert set(DEFAULT_CONTACT_THRESHOLDS.keys()) == set(JOINTS)


#: Peak load each joint develops merely ACCELERATING through open space,
#: measured on the follower on 2026-07-12 through the fixed load-during-travel
#: sampling. A threshold at or below its joint's figure calls contact on free
#: motion; one at or above 500 can never fire, because present_load saturates
#: at gentle_move's Torque_Limit cap.
MEASURED_FREE_MOTION_PEAK = {
    "shoulder_pan": 88,
    "shoulder_lift": 92,
    "elbow_flex": 148,
    "wrist_flex": 96,
    "wrist_roll": 300,
    "gripper": 76,
}


def test_default_contact_thresholds_values():
    """The specific hardware-tuned per-joint default values."""
    assert DEFAULT_CONTACT_THRESHOLDS == {
        "shoulder_pan": 250,
        "shoulder_lift": 250,
        "elbow_flex": 280,
        "wrist_flex": 250,
        "wrist_roll": 400,
        "gripper": 250,
    }


def test_every_threshold_sits_inside_its_measured_band():
    """Each threshold must clear its joint's free-motion peak, with margin.

    This is the invariant the previous values violated: wrist_roll's 180 sat
    BELOW its own 300 free-motion peak, so a correctly-sampled load watch would
    have called contact on every move that joint made. They were tuned against
    the pre-fix code's near-zero load reads, which measured nothing real.
    """
    for joint, peak in MEASURED_FREE_MOTION_PEAK.items():
        threshold = DEFAULT_CONTACT_THRESHOLDS[joint]
        assert threshold > peak, (
            f"{joint}: threshold {threshold} is at or below its measured "
            f"free-motion peak {peak} — free travel would false-trigger contact"
        )


def test_every_threshold_sits_below_the_torque_cap():
    """present_load saturates at Torque_Limit, so a threshold >= the cap is dead."""
    for joint, threshold in DEFAULT_CONTACT_THRESHOLDS.items():
        assert threshold < _CONTACT_TORQUE_LIMIT, (
            f"{joint}: threshold {threshold} >= the {_CONTACT_TORQUE_LIMIT} torque cap, "
            "so present_load can never reach it and contact could never fire"
        )


# ---------------------------------------------------------------------------
# resolve_contact_thresholds — precedence: per_joint > blanket > from_file > default
# ---------------------------------------------------------------------------


def test_resolve_defaults_only():
    """With no overrides, every joint resolves to its built-in default."""
    resolved = resolve_contact_thresholds()
    assert resolved == tuple(DEFAULT_CONTACT_THRESHOLDS[j] for j in JOINTS)


def test_resolve_blanket_only_broadcasts_to_every_joint():
    """An explicit blanket threshold overrides every joint's default."""
    resolved = resolve_contact_thresholds(blanket=300)
    assert resolved == tuple(300 for _ in JOINTS)


def test_resolve_blanket_none_does_not_broadcast():
    """blanket=None (flag absent) must NOT collapse to a fixed value — each
    joint falls through to file/default instead."""
    resolved = resolve_contact_thresholds(blanket=None)
    assert resolved == tuple(DEFAULT_CONTACT_THRESHOLDS[j] for j in JOINTS)


def test_resolve_blanket_zero_is_honored_explicitly():
    """An explicit blanket of 0 (falsy) must be honored, not treated as absent."""
    resolved = resolve_contact_thresholds(blanket=0)
    assert resolved == tuple(0 for _ in JOINTS)


def test_resolve_file_only_applies_named_joints_and_default_for_rest():
    """from_file values apply only to named joints; others use the default."""
    resolved = resolve_contact_thresholds(from_file={"shoulder_lift": 400})
    expected = []
    for joint in JOINTS:
        if joint == "shoulder_lift":
            expected.append(400)
        else:
            expected.append(DEFAULT_CONTACT_THRESHOLDS[joint])
    assert resolved == tuple(expected)


def test_resolve_per_joint_only_applies_named_joints_and_default_for_rest():
    """per_joint values apply only to named joints; others use the default."""
    resolved = resolve_contact_thresholds(per_joint={"gripper": 500})
    expected = []
    for joint in JOINTS:
        if joint == "gripper":
            expected.append(500)
        else:
            expected.append(DEFAULT_CONTACT_THRESHOLDS[joint])
    assert resolved == tuple(expected)


def test_resolve_blanket_overrides_file():
    """A blanket threshold beats a file entry for the same joint."""
    resolved = resolve_contact_thresholds(blanket=111, from_file={"shoulder_lift": 999})
    idx = JOINTS.index("shoulder_lift")
    assert resolved[idx] == 111


def test_resolve_per_joint_overrides_blanket():
    """A per-joint flag beats the blanket for that one joint, but not others."""
    resolved = resolve_contact_thresholds(
        blanket=111, per_joint={"shoulder_lift": 350}, from_file={"gripper": 999}
    )
    idx_sl = JOINTS.index("shoulder_lift")
    idx_gr = JOINTS.index("gripper")
    idx_other = JOINTS.index("elbow_flex")
    assert resolved[idx_sl] == 350  # per_joint wins over blanket
    assert resolved[idx_gr] == 111  # blanket wins over file
    assert resolved[idx_other] == 111  # blanket applies to everything else


def test_resolve_per_joint_overrides_file_when_no_blanket():
    """Full precedence chain: per_joint > file > default, blanket absent."""
    resolved = resolve_contact_thresholds(
        per_joint={"shoulder_lift": 350},
        from_file={"shoulder_lift": 999, "gripper": 400},
    )
    idx_sl = JOINTS.index("shoulder_lift")
    idx_gr = JOINTS.index("gripper")
    idx_other = JOINTS.index("elbow_flex")
    assert resolved[idx_sl] == 350  # per_joint wins over file
    assert resolved[idx_gr] == 400  # file applies (no blanket, no per_joint)
    assert resolved[idx_other] == DEFAULT_CONTACT_THRESHOLDS["elbow_flex"]


def test_resolve_returns_tuple_length_matches_joints():
    resolved = resolve_contact_thresholds()
    assert len(resolved) == len(JOINTS)
    assert isinstance(resolved, tuple)


def test_resolve_unknown_joint_in_per_joint_raises():
    with pytest.raises(ValueError, match="Unknown joint"):
        resolve_contact_thresholds(per_joint={"not_a_joint": 100})


def test_resolve_unknown_joint_in_from_file_raises():
    with pytest.raises(ValueError, match="Unknown joint"):
        resolve_contact_thresholds(from_file={"not_a_joint": 100})
