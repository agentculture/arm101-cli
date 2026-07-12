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
* wrist_roll's SoftLimit, dead_arc_contains_seam, and soft_limit() — the
  encoder-wrap dead-arc machinery (issue #35 / plan task t9).
"""

import pytest

from arm101.hardware import arm_spec, ticks
from arm101.hardware.arm_spec import (
    ARM_SPEC,
    DEFAULT_BAUDRATE,
    DEFAULT_CONTACT_THRESHOLDS,
    FACTORY_ENCODER_OFFSET,
    JOINTS,
    SEAM_CLEARANCE_TICKS,
    SOFT_LIMITS,
    TICK_MAX,
    TICK_MIN,
    MotorSpec,
    SoftLimit,
    _require_dead_arc_contains_seam,
    dead_arc_contains_seam,
    joint_ids,
    motor_spec,
    resolve_contact_thresholds,
    role_motors,
    roles,
    soft_limit,
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


# ---------------------------------------------------------------------------
# Encoder wrap — TICK_MIN / TICK_MAX
# ---------------------------------------------------------------------------


def test_tick_bounds_are_12_bit_encoder_range():
    """The STS3215 is a 12-bit encoder: ticks run [0, 4095]."""
    assert TICK_MIN == 0
    assert TICK_MAX == 4095


# ---------------------------------------------------------------------------
# SoftLimit — structural validation
# ---------------------------------------------------------------------------


def test_soft_limit_accepts_a_well_ordered_interval():
    limit = SoftLimit(min_tick=100, max_tick=3995)
    assert limit.min_tick == 100
    assert limit.max_tick == 3995


@pytest.mark.parametrize(
    "min_tick,max_tick",
    [
        (100, 100),  # min == max: zero-width, not a valid interval
        (200, 100),  # min > max: the "wrap means min>max" encoding this module bans
        (-1, 100),  # below TICK_MIN
        (100, 4096),  # above TICK_MAX
        (0, 4095),  # structurally valid (full range) — SoftLimit alone allows this
    ],
)
def test_soft_limit_rejects_malformed_intervals(min_tick, max_tick):
    if (min_tick, max_tick) == (0, 4095):
        # The full range is a well-formed interval structurally — SoftLimit's
        # own validation is purely "is this a well-ordered interval", not the
        # seam-containment guarantee. That guarantee is a SEPARATE check (see
        # dead_arc_contains_seam below) precisely so the two concerns are
        # independently testable.
        SoftLimit(min_tick=min_tick, max_tick=max_tick)
    else:
        with pytest.raises(ValueError):
            SoftLimit(min_tick=min_tick, max_tick=max_tick)


def test_soft_limit_dead_arc_ticks_matches_both_excluded_sides():
    limit = SoftLimit(min_tick=100, max_tick=3995)
    # low side [0, 100) = 100 ticks, high side (3995, 4095] = 100 ticks
    assert limit.dead_arc_ticks == 200


def test_soft_limit_dead_arc_ticks_zero_for_full_range():
    limit = SoftLimit(min_tick=TICK_MIN, max_tick=TICK_MAX)
    assert limit.dead_arc_ticks == 0


def test_soft_limit_permits_checks_inclusive_bounds():
    limit = SoftLimit(min_tick=100, max_tick=3995)
    assert limit.permits(100) is True
    assert limit.permits(3995) is True
    assert limit.permits(2000) is True
    assert limit.permits(99) is False
    assert limit.permits(3996) is False
    assert limit.permits(0) is False
    assert limit.permits(4095) is False


# ---------------------------------------------------------------------------
# dead_arc_contains_seam — the enforceable predicate
# ---------------------------------------------------------------------------


def test_dead_arc_contains_seam_true_for_a_genuine_restriction():
    """Any interval narrower than the full turn excludes an arc through the seam."""
    assert dead_arc_contains_seam(100, 3995) is True


def test_dead_arc_contains_seam_true_when_only_the_low_side_is_excluded():
    assert dead_arc_contains_seam(50, TICK_MAX) is True


def test_dead_arc_contains_seam_true_when_only_the_high_side_is_excluded():
    assert dead_arc_contains_seam(TICK_MIN, 4000) is True


def test_dead_arc_contains_seam_false_for_the_full_range():
    """The degenerate case: nothing excluded, so the seam is not contained anywhere.

    This is the exact "a soft limit that still spans the seam buys nothing"
    case the task exists to guard against. If a future edit ever widens
    wrist_roll's SOFT_LIMITS entry back to (TICK_MIN, TICK_MAX), this is the
    predicate that must say False — see
    test_widening_the_soft_limit_to_the_full_range_fails_validation below for
    the module-load-time enforcement built on top of it.
    """
    assert dead_arc_contains_seam(TICK_MIN, TICK_MAX) is False


@pytest.mark.parametrize(
    "min_tick,max_tick",
    [
        (100, 100),
        (200, 100),
        (-1, 100),
        (100, 4096),
    ],
)
def test_dead_arc_contains_seam_rejects_malformed_intervals(min_tick, max_tick):
    with pytest.raises(ValueError):
        dead_arc_contains_seam(min_tick, max_tick)


# ---------------------------------------------------------------------------
# _require_dead_arc_contains_seam — the import-time guard, and how a future
# widening of the range is caught
# ---------------------------------------------------------------------------


def test_require_dead_arc_contains_seam_passes_the_real_soft_limits_table():
    """The table this module actually ships must satisfy its own guarantee."""
    _require_dead_arc_contains_seam(SOFT_LIMITS)  # must not raise


def test_widening_the_soft_limit_to_the_full_range_fails_validation():
    """A future edit that widens a joint's range back to the full turn must be
    rejected — not silently accepted — which is exactly what would happen if
    SOFT_LIMITS["wrist_roll"] were ever "simplified" back to (TICK_MIN, TICK_MAX).

    This directly exercises the guard :func:`_require_dead_arc_contains_seam`
    runs at import time against the real :data:`SOFT_LIMITS`, using a
    reconstructed table so the test does not have to mutate (and restore) the
    module-level constant to prove the point.
    """
    widened = {"wrist_roll": SoftLimit(min_tick=TICK_MIN, max_tick=TICK_MAX)}
    with pytest.raises(ValueError, match="spans the full"):
        _require_dead_arc_contains_seam(widened)


def test_require_dead_arc_contains_seam_empty_table_is_fine():
    _require_dead_arc_contains_seam({})  # must not raise


# ---------------------------------------------------------------------------
# SOFT_LIMITS / soft_limit() — the shipped wrist_roll value
# ---------------------------------------------------------------------------


def test_soft_limits_has_exactly_one_entry_wrist_roll():
    """Only wrist_roll gets a soft limit; elbow_flex takes the re-zero path instead."""
    assert set(SOFT_LIMITS.keys()) == {"wrist_roll"}


def test_wrist_roll_soft_limit_is_derived_from_the_seam_and_the_clearance():
    """The shipped value, asserted as the DERIVATION rather than as two numbers.

    It used to be typed — ``(100, 3995)`` — and typed is how it came to be in the
    wrong frame (those were reported ticks, read off a servo holding the factory
    offset, and stored as if they were raw). Derived from
    ``seam_tick(FACTORY_ENCODER_OFFSET)`` there is no number left to be in a frame
    at all. See tests/test_tick_frames.py for the properties this buys.
    """
    limit = SOFT_LIMITS["wrist_roll"]
    assert limit.min_tick == arm_spec.seam_tick(FACTORY_ENCODER_OFFSET) + SEAM_CLEARANCE_TICKS
    assert limit.max_tick == TICK_MAX - SEAM_CLEARANCE_TICKS


def test_wrist_roll_soft_limit_dead_arc_contains_the_seam():
    """The load-bearing property, checked directly against the shipped value."""
    limit = SOFT_LIMITS["wrist_roll"]
    assert dead_arc_contains_seam(limit.min_tick, limit.max_tick) is True


def test_wrist_roll_soft_limit_is_narrower_than_a_full_turn():
    """A genuine dead arc exists — the range is strictly narrower than [0, 4095]."""
    limit = SOFT_LIMITS["wrist_roll"]
    assert limit.dead_arc_ticks > 0
    assert (limit.min_tick, limit.max_tick) != (TICK_MIN, TICK_MAX)


def test_the_measured_free_envelope_constrains_nothing_because_it_wraps():
    """The "envelope" that used to justify the range is a REPORTED reading, and it wraps.

    The t9 sweep reported ``[21, 4073]`` and an earlier version of this test asserted
    the soft limit sat inside it, as if those were walls. They are not walls, and they
    are not raw. Converted into the frame they were measured in (a factory servo,
    ``Ofs = 85``, so ``raw = reported + 85``) the envelope covers ``[106, 4095]`` AND
    ``[0, 62]`` — all but a 43-tick raw gap at ``(62, 106)``, which straddles raw 85.

    That gap IS the seam: the sweep simply never crossed it. So the envelope confirms
    wrist_roll turns essentially all the way round (which is exactly why a re-zero can
    never help it) and constrains the soft limit not at all. Reading it as a wall was
    the same frame confusion, in miniature — and the dead arc swallows the whole gap
    anyway, which is what this asserts instead.
    """
    limit = SOFT_LIMITS["wrist_roll"]
    raw_lo = ticks.raw_from_reported(21, FACTORY_ENCODER_OFFSET)
    raw_hi = ticks.raw_from_reported(4073, FACTORY_ENCODER_OFFSET)

    assert raw_hi < raw_lo, "the measured envelope wraps in raw ticks — it is not an interval"
    for unvisited in range(raw_hi + 1, raw_lo):  # the raw ticks the sweep never reached
        assert not limit.permits(unvisited), f"raw {unvisited} was never swept, yet is permitted"


def test_soft_limit_returns_none_for_joints_without_a_wrap_problem():
    for joint in JOINTS:
        if joint == "wrist_roll":
            continue
        assert soft_limit(joint) is None


def test_soft_limit_returns_the_soft_limit_for_wrist_roll():
    result = soft_limit("wrist_roll")
    assert isinstance(result, SoftLimit)
    assert result is SOFT_LIMITS["wrist_roll"]


def test_soft_limit_unknown_joint_raises():
    with pytest.raises(ValueError, match="Unknown joint"):
        soft_limit("not_a_joint")


# ---------------------------------------------------------------------------
# Acceptance criterion: a sweep across the permitted range is monotonic in
# ticks, and never enters the dead arc / crosses the seam.
# ---------------------------------------------------------------------------


def test_wrist_roll_soft_limit_sweep_is_monotonic_in_ticks():
    """Sweeping wrist_roll's permitted range end-to-end never jumps through the seam.

    This is the direct proof the task calls for: a sweep across the whole
    permitted range, in ascending tick order, is strictly increasing (no
    4095->0 jump anywhere inside it) and never visits a dead-arc tick. If
    wrist_roll's soft limit still spanned the seam (the exact bug this task
    fixes), the analogous sweep would have to wrap through 0 to cover the
    joint's full travel and could not be expressed as one monotonic tick
    sequence at all.
    """
    limit = SOFT_LIMITS["wrist_roll"]
    sweep = list(range(limit.min_tick, limit.max_tick + 1, 37))
    if sweep[-1] != limit.max_tick:
        sweep.append(limit.max_tick)

    assert len(sweep) > 2, "sweep must cover more than just the two endpoints"

    for previous, current in zip(sweep, sweep[1:]):
        assert current > previous, "sweep must be strictly increasing"

    for tick in sweep:
        assert limit.permits(tick), f"tick {tick} outside the permitted range"
        assert TICK_MIN <= tick <= TICK_MAX

    # The defining property: no tick in the sweep falls in the dead arc, i.e.
    # the sweep never has to cross [TICK_MIN, min_tick) or (max_tick, TICK_MAX].
    dead_low = range(TICK_MIN, limit.min_tick)
    dead_high = range(limit.max_tick + 1, TICK_MAX + 1)
    assert not any(tick in dead_low for tick in sweep)
    assert not any(tick in dead_high for tick in sweep)


def test_full_range_sweep_would_have_to_cross_the_seam_to_be_monotonic():
    """Contrast case: sweeping the WHOLE [TICK_MIN, TICK_MAX] range in one
    direction never needs to wrap either — because a single unbroken sweep
    from 0 to 4095 doesn't have to come back around. The seam only bites a
    controller that must go from a point near one edge to a point near the
    other by the SHORT way (e.g. 4 -> 304, the real hardware failure in
    docs/hardware-validation-arm-read-flex.md). This test documents why
    dead_arc_contains_seam — not "can I enumerate 0..4095 in order" — is the
    right predicate: monotonicity of a single full sweep is necessary but not
    sufficient, whereas an explicit dead arc rules out the wrap entirely,
    for every pair of permitted ticks, not just an end-to-end sweep.
    """
    full_sweep = list(range(TICK_MIN, TICK_MAX + 1, 500))
    for previous, current in zip(full_sweep, full_sweep[1:]):
        assert current > previous
    # And yet the full range does NOT contain a dead arc around the seam:
    assert dead_arc_contains_seam(TICK_MIN, TICK_MAX) is False


# ---------------------------------------------------------------------------
# Spec boundary: nothing in this module writes to a servo register.
# ---------------------------------------------------------------------------


def test_arm_spec_module_never_imports_the_bus():
    """arm_spec stays a pure data module — no EEPROM/register-writing capability.

    The soft limit is a SOFTWARE bound only (this task's spec boundary): the
    servo's EEPROM min_angle/max_angle registers stay at the factory 0-4095.
    Parsing the module's own AST for any import mentioning "bus" is a coarse
    but effective guardrail — a module with no import of
    arm101.hardware.bus (or MotorBus) has no handle to issue a register write
    with in the first place, so this rules the whole category out rather than
    checking for one specific call site.
    """
    import ast
    import inspect

    import arm101.hardware.arm_spec as arm_spec_module

    tree = ast.parse(inspect.getsource(arm_spec_module))
    imported_names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported_names.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported_names.add(node.module)

    assert not any(
        "bus" in name.lower() for name in imported_names
    ), f"arm_spec must not import a bus module; found {imported_names}"
