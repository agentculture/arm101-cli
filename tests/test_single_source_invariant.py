"""Single-source invariant tests for the SO-101 joint→id mapping (t7, #17).

Verifies two complementary guarantees:

A1. The hardcoded joint→id literal (shoulder_pan→1 … gripper→6) exists in
    EXACTLY ONE place across arm101/ — arm101/hardware/arm_spec.py — and no
    other module repeats a duplicate of it.

A2. All three downstream consumers (calibrate._JOINT_MOTOR, setup_motors via
    _MOTOR_ORDER, and profiles.JOINTS) resolve to the SAME canonical joint→id
    mapping, proving that refactoring onto arm_spec is behavior-preserving.
"""

from __future__ import annotations

import pathlib
import re

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_ARM101_ROOT = pathlib.Path(__file__).parent.parent / "arm101"

#: The canonical joint→motor-id mapping for the SO-101.
#: Source: arm_spec.ARM_SPEC (both roles share ids 1–6 per LeRobot commit 2f2b567).
_CANONICAL: dict[str, int] = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}

# Pattern: a quoted joint name immediately followed by a colon and an integer
# literal (dict-literal style).  Whitespace between the colon and int is optional.
# Examples it matches:
#   "shoulder_pan": 1
#   'shoulder_pan': 2
#   "gripper":6
_JOINT_INT_LITERAL_RE = re.compile(
    r"""['"](shoulder_pan|shoulder_lift|elbow_flex|wrist_flex|wrist_roll|gripper)['"]\s*:\s*\d"""
)


# ---------------------------------------------------------------------------
# A1 — single source of truth scan
# ---------------------------------------------------------------------------


def test_joint_id_literal_in_exactly_one_place():
    """The joint→id literal must appear ONLY in arm_spec.py, nowhere else in arm101/.

    Strategy: scan every .py file in arm101/ for the pattern
    '<joint_name>': <int> (dict-literal style).  Any match in a file other
    than arm_spec.py is a duplicate that breaks the single-source invariant.

    This catches the scenario where a future developer accidentally re-introduces
    the mapping inline instead of importing from arm_spec.
    """
    arm_spec_path = _ARM101_ROOT / "hardware" / "arm_spec.py"
    offending: list[tuple[pathlib.Path, int, str]] = []

    for py_file in sorted(_ARM101_ROOT.rglob("*.py")):
        if py_file.resolve() == arm_spec_path.resolve():
            continue  # arm_spec.py is the authorised home — skip
        if "__pycache__" in py_file.parts:
            continue  # skip compiled bytecode siblings

        source = py_file.read_text(encoding="utf-8")
        for lineno, line in enumerate(source.splitlines(), start=1):
            if _JOINT_INT_LITERAL_RE.search(line):
                offending.append((py_file, lineno, line.strip()))

    assert offending == [], (
        "Found hardcoded joint→id literal(s) outside arm_spec.py — "
        "these modules must derive the mapping from arm_spec.joint_ids() instead:\n"
        + "\n".join(f"  {p}:{n}: {text}" for p, n, text in offending)
    )


def test_arm_spec_defines_all_canonical_joints():
    """arm_spec.py must define a mapping entry for each of the six canonical joints.

    This confirms the authorised source file actually contains what we're
    guarding — the scan in the companion test is meaningless if arm_spec itself
    doesn't own the mapping.
    """
    arm_spec_path = _ARM101_ROOT / "hardware" / "arm_spec.py"
    source = arm_spec_path.read_text(encoding="utf-8")
    # arm_spec uses MotorSpec(id=N) style rather than "joint": N dict-literal
    # style, so name-only presence is the right check here.
    missing_names = [joint for joint in _CANONICAL if joint not in source]
    assert missing_names == [], (
        f"arm_spec.py is missing these canonical joint names: {missing_names}. "
        "All six joints must be defined there."
    )


# ---------------------------------------------------------------------------
# A2 — behavior-preserving: all three consumers resolve the same mapping
# ---------------------------------------------------------------------------


def test_calibrate_joint_motor_equals_canonical():
    """calibrate._JOINT_MOTOR must equal the canonical {shoulder_pan:1 … gripper:6} mapping.

    calibrate._JOINT_MOTOR is defined as arm_spec.joint_ids("follower"), which
    must resolve to the same mapping that previously was hard-coded inline.
    """
    from arm101.cli._commands import calibrate  # noqa: PLC0415

    assert calibrate._JOINT_MOTOR == _CANONICAL, (
        f"calibrate._JOINT_MOTOR does not match the canonical mapping.\n"
        f"  got:      {calibrate._JOINT_MOTOR}\n"
        f"  expected: {_CANONICAL}"
    )


def test_setup_motors_order_implies_canonical_mapping():
    """setup_motors._MOTOR_ORDER must encode the canonical joint→id mapping.

    _MOTOR_ORDER is a list of (id, joint) tuples derived from arm_spec — when
    converted to a {joint: id} dict it must equal the canonical mapping.
    """
    from arm101.cli._commands import setup_motors  # noqa: PLC0415

    derived = {joint: motor_id for motor_id, joint in setup_motors._MOTOR_ORDER}
    assert derived == _CANONICAL, (
        f"setup_motors._MOTOR_ORDER does not encode the canonical joint→id mapping.\n"
        f"  derived:  {derived}\n"
        f"  expected: {_CANONICAL}"
    )


def test_profiles_joints_order_implies_canonical_mapping():
    """profiles.JOINTS must yield the canonical joint→id mapping by position (index+1).

    profiles.JOINTS is derived from arm_spec.JOINTS.  The canonical mapping
    assigns shoulder_pan (index 0) → id 1, shoulder_lift (index 1) → id 2, …,
    gripper (index 5) → id 6.  Converting by index+1 must reproduce the canonical
    dict exactly.
    """
    from arm101.hardware import profiles  # noqa: PLC0415

    derived = {joint: i + 1 for i, joint in enumerate(profiles.JOINTS)}
    assert derived == _CANONICAL, (
        f"profiles.JOINTS positional mapping does not match the canonical joint→id mapping.\n"
        f"  derived:  {derived}\n"
        f"  expected: {_CANONICAL}"
    )


def test_all_three_consumers_agree():
    """All three consumers must derive the SAME canonical joint→id mapping.

    This is the parallel==serial behavior-preserving proof: calibrate,
    setup_motors, and profiles all resolve an identical mapping, so
    refactoring them onto arm_spec preserved the original behavior.
    """
    from arm101.cli._commands import calibrate, setup_motors  # noqa: PLC0415
    from arm101.hardware import profiles  # noqa: PLC0415

    calibrate_map = calibrate._JOINT_MOTOR
    setup_map = {joint: motor_id for motor_id, joint in setup_motors._MOTOR_ORDER}
    profiles_map = {joint: i + 1 for i, joint in enumerate(profiles.JOINTS)}

    assert calibrate_map == setup_map, (
        "calibrate._JOINT_MOTOR and setup_motors._MOTOR_ORDER disagree:\n"
        f"  calibrate:    {calibrate_map}\n"
        f"  setup_motors: {setup_map}"
    )
    assert calibrate_map == profiles_map, (
        "calibrate._JOINT_MOTOR and profiles.JOINTS positional map disagree:\n"
        f"  calibrate: {calibrate_map}\n"
        f"  profiles:  {profiles_map}"
    )
    assert calibrate_map == _CANONICAL, (
        "All three consumers agree but do NOT match the canonical mapping:\n"
        f"  consumers: {calibrate_map}\n"
        f"  canonical: {_CANONICAL}"
    )
