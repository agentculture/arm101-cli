"""arm101.hardware.arm_spec — single-source, per-role motor map for the SO-101.

Provenance
----------
This module is the single source of truth for the SO-101 arm's motor identity
and physical specifications.  All values come from two public sources:

* **LeRobot** (lerobot/lerobot, commit 2f2b567):
  ``lerobot/common/robot_devices/robots/so_follower.py`` and
  ``so_leader.py`` define the ``motors`` dict with joint names, ids 1–6, and
  servo model ``"sts3215"``; ``feetech.py`` sets ``DEFAULT_BAUDRATE = 1_000_000``.
  Follower and leader share *identical* id and baud assignments per LeRobot.

* **Seeed SO-101 wiki BOM**
  (https://wiki.seeedstudio.com/lerobot_so100m_new/#configure-the-motors):
  Physical BOM listing the servo model variant (C001/C018/C044/C046/C047) and
  gear ratio per joint per role.  These hardware facts are NOT present in the
  LeRobot software — they are physical-BOM facts only.

Rationale
---------
Future-proof structure with cited values and no assumed divergence between
roles; ids and baud are shared by both roles, while servo_model and gear_ratio
differ per role (and per joint for the leader).  Downstream modules
(calibrate, setup_motors) should import from here instead of duplicating the
joint→id map.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Optional

# ---------------------------------------------------------------------------
# Public constants
# ---------------------------------------------------------------------------

#: Canonical joint names for the SO-101, in hardware order (shoulder_pan → gripper).
#: Source: LeRobot so_follower.py / so_leader.py motors dict (commit 2f2b567).
JOINTS: tuple[str, ...] = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)

#: Default bus baud rate for all Feetech STS3215 servos on the SO-101.
#: Source: LeRobot feetech.py DEFAULT_BAUDRATE = 1_000_000 (commit 2f2b567).
DEFAULT_BAUDRATE: int = 1_000_000

#: Follower servo model variant — uniform across all 6 follower joints.
#: Source: Seeed SO-101 wiki BOM (follower column).
_FOLLOWER_SERVO_MODEL: str = "ST-3215-C001/C018/C047"

#: Default per-joint contact-load threshold (STS3215 ``Present_Load``
#: register units) used by ``arm explore``'s gentle-move contact detection
#: (see :func:`resolve_contact_thresholds`).
#:
#: Free-motion load differs a lot per joint, so one global threshold is
#: always wrong for someone: too high misses real contacts on light joints,
#: too low false-triggers a heavy joint on its own gravity/friction load.
#: These values are keyed to two hardware-validation runs:
#:
#: * ``docs/hardware-validation-arm-explore.md`` (t11, live follower run):
#:   ``--threshold 250`` never registered a contact even at joint-range
#:   limits on the light joints driven that run — a floor of 250 is too HIGH
#:   for them, real contacts need a LOWER floor to trip reliably.
#:   ``--threshold 150`` misread ``shoulder_lift``'s own gravity load (152)
#:   as a contact — ``shoulder_lift`` needs a HIGHER floor than 150 so
#:   ordinary gravity load never false-triggers.
#: * ``docs/hardware-validation-arm-read-flex.md``: a genuine gravity
#:   contact on ``shoulder_lift`` was measured at ``present_load`` magnitude
#:   **252** (a real, physically-confirmed number) — the floor must sit
#:   comfortably ABOVE that band.
#:
#: ``shoulder_lift`` (350, floored well above the ~250 gravity band) and
#: ``gripper`` (380, floored above the ~320 gear-friction ceiling noted in
#: :mod:`arm101.hardware.gentle`) are the two joints with a hard numeric
#: band behind them. **HARDWARE-TUNED, PARTIALLY OPEN QUESTION:** the
#: remaining four joints (``shoulder_pan``, ``elbow_flex``, ``wrist_flex``,
#: ``wrist_roll``) are conservative ESTIMATES, not yet individually
#: validated on hardware — confirming (or correcting) them is deferred to a
#: follow-up hardware run (plan task t12). Override any joint's default via
#: ``arm explore --threshold-joint NAME=VALUE`` or ``--threshold-file``
#: without waiting on that follow-up.
DEFAULT_CONTACT_THRESHOLDS: dict[str, int] = {
    "shoulder_pan": 200,
    "shoulder_lift": 350,
    "elbow_flex": 220,
    "wrist_flex": 200,
    "wrist_roll": 180,
    "gripper": 380,
}


# ---------------------------------------------------------------------------
# Data model
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class MotorSpec:
    """Hardware specification for one motor in one role.

    Attributes
    ----------
    id : int
        EEPROM servo ID (1–6).
        Source: LeRobot so_follower.py / so_leader.py (commit 2f2b567).
    baud : int
        Bus baud rate (always 1_000_000 for STS3215 on SO-101).
        Source: LeRobot feetech.py DEFAULT_BAUDRATE (commit 2f2b567).
    servo_model : str
        Physical servo model variant string.
        Source: Seeed SO-101 wiki BOM.
    gear_ratio : str
        Physical gear reduction ratio (e.g. ``"1:345"``).
        Source: Seeed SO-101 wiki BOM.
    """

    id: int
    baud: int
    servo_model: str
    gear_ratio: str


# ---------------------------------------------------------------------------
# Role-keyed arm spec
# ---------------------------------------------------------------------------

#: Per-role, per-joint motor specification.
#:
#: Structure: ``ARM_SPEC[role][joint_name] -> MotorSpec``
#:
#: Both roles share ids 1–6 and baud 1_000_000.
#: Source ids/baud: LeRobot so_follower.py / so_leader.py (commit 2f2b567).
#: Source servo_model/gear_ratio: Seeed SO-101 wiki BOM
#:   https://wiki.seeedstudio.com/lerobot_so100m_new/#configure-the-motors
ARM_SPEC: dict[str, dict[str, MotorSpec]] = {
    # -----------------------------------------------------------------------
    # Follower arm: uniform STS3215 C001/C018/C047 series, all 1:345 gear.
    # Source: Seeed SO-101 wiki BOM (follower column).
    # -----------------------------------------------------------------------
    "follower": {
        "shoulder_pan": MotorSpec(
            id=1,  # LeRobot so_follower.py motors["shoulder_pan"].id (commit 2f2b567)
            baud=1_000_000,  # LeRobot feetech.py DEFAULT_BAUDRATE (commit 2f2b567)
            servo_model=_FOLLOWER_SERVO_MODEL,  # Seeed SO-101 wiki BOM, follower
            gear_ratio="1:345",  # Seeed SO-101 wiki BOM, follower
        ),
        "shoulder_lift": MotorSpec(
            id=2,  # LeRobot so_follower.py motors["shoulder_lift"].id (commit 2f2b567)
            baud=1_000_000,  # LeRobot feetech.py DEFAULT_BAUDRATE (commit 2f2b567)
            servo_model=_FOLLOWER_SERVO_MODEL,  # Seeed SO-101 wiki BOM, follower
            gear_ratio="1:345",  # Seeed SO-101 wiki BOM, follower
        ),
        "elbow_flex": MotorSpec(
            id=3,  # LeRobot so_follower.py motors["elbow_flex"].id (commit 2f2b567)
            baud=1_000_000,  # LeRobot feetech.py DEFAULT_BAUDRATE (commit 2f2b567)
            servo_model=_FOLLOWER_SERVO_MODEL,  # Seeed SO-101 wiki BOM, follower
            gear_ratio="1:345",  # Seeed SO-101 wiki BOM, follower
        ),
        "wrist_flex": MotorSpec(
            id=4,  # LeRobot so_follower.py motors["wrist_flex"].id (commit 2f2b567)
            baud=1_000_000,  # LeRobot feetech.py DEFAULT_BAUDRATE (commit 2f2b567)
            servo_model=_FOLLOWER_SERVO_MODEL,  # Seeed SO-101 wiki BOM, follower
            gear_ratio="1:345",  # Seeed SO-101 wiki BOM, follower
        ),
        "wrist_roll": MotorSpec(
            id=5,  # LeRobot so_follower.py motors["wrist_roll"].id (commit 2f2b567)
            baud=1_000_000,  # LeRobot feetech.py DEFAULT_BAUDRATE (commit 2f2b567)
            servo_model=_FOLLOWER_SERVO_MODEL,  # Seeed SO-101 wiki BOM, follower
            gear_ratio="1:345",  # Seeed SO-101 wiki BOM, follower
        ),
        "gripper": MotorSpec(
            id=6,  # LeRobot so_follower.py motors["gripper"].id (commit 2f2b567)
            baud=1_000_000,  # LeRobot feetech.py DEFAULT_BAUDRATE (commit 2f2b567)
            servo_model=_FOLLOWER_SERVO_MODEL,  # Seeed SO-101 wiki BOM, follower
            gear_ratio="1:345",  # Seeed SO-101 wiki BOM, follower
        ),
    },
    # -----------------------------------------------------------------------
    # Leader arm: mixed variants per joint.
    # Source: Seeed SO-101 wiki BOM (leader column).
    # -----------------------------------------------------------------------
    "leader": {
        "shoulder_pan": MotorSpec(
            id=1,  # LeRobot so_leader.py motors["shoulder_pan"].id (commit 2f2b567)
            baud=1_000_000,  # LeRobot feetech.py DEFAULT_BAUDRATE (commit 2f2b567)
            servo_model="ST-3215-C044",  # Seeed SO-101 wiki BOM, leader
            gear_ratio="1:191",  # Seeed SO-101 wiki BOM, leader
        ),
        "shoulder_lift": MotorSpec(
            id=2,  # LeRobot so_leader.py motors["shoulder_lift"].id (commit 2f2b567)
            baud=1_000_000,  # LeRobot feetech.py DEFAULT_BAUDRATE (commit 2f2b567)
            servo_model="ST-3215-C001",  # Seeed SO-101 wiki BOM, leader
            gear_ratio="1:345",  # Seeed SO-101 wiki BOM, leader
        ),
        "elbow_flex": MotorSpec(
            id=3,  # LeRobot so_leader.py motors["elbow_flex"].id (commit 2f2b567)
            baud=1_000_000,  # LeRobot feetech.py DEFAULT_BAUDRATE (commit 2f2b567)
            servo_model="ST-3215-C044",  # Seeed SO-101 wiki BOM, leader
            gear_ratio="1:191",  # Seeed SO-101 wiki BOM, leader
        ),
        "wrist_flex": MotorSpec(
            id=4,  # LeRobot so_leader.py motors["wrist_flex"].id (commit 2f2b567)
            baud=1_000_000,  # LeRobot feetech.py DEFAULT_BAUDRATE (commit 2f2b567)
            servo_model="ST-3215-C046",  # Seeed SO-101 wiki BOM, leader
            gear_ratio="1:147",  # Seeed SO-101 wiki BOM, leader
        ),
        "wrist_roll": MotorSpec(
            id=5,  # LeRobot so_leader.py motors["wrist_roll"].id (commit 2f2b567)
            baud=1_000_000,  # LeRobot feetech.py DEFAULT_BAUDRATE (commit 2f2b567)
            servo_model="ST-3215-C046",  # Seeed SO-101 wiki BOM, leader
            gear_ratio="1:147",  # Seeed SO-101 wiki BOM, leader
        ),
        "gripper": MotorSpec(
            id=6,  # LeRobot so_leader.py motors["gripper"].id (commit 2f2b567)
            baud=1_000_000,  # LeRobot feetech.py DEFAULT_BAUDRATE (commit 2f2b567)
            servo_model="ST-3215-C046",  # Seeed SO-101 wiki BOM, leader
            gear_ratio="1:147",  # Seeed SO-101 wiki BOM, leader
        ),
    },
}

#: Known arm roles (frozen set for O(1) membership tests).
_KNOWN_ROLES: frozenset[str] = frozenset(ARM_SPEC.keys())


# ---------------------------------------------------------------------------
# Accessor helpers
# ---------------------------------------------------------------------------


def roles() -> list[str]:
    """Return the list of known arm roles (``["follower", "leader"]``)."""
    return sorted(ARM_SPEC.keys())


def joint_ids(role: str) -> dict[str, int]:
    """Return a ``{joint_name: id}`` mapping for *role*.

    Parameters
    ----------
    role:
        One of ``"follower"`` or ``"leader"``.

    Raises
    ------
    ValueError
        If *role* is not one of the known roles.
    """
    _require_role(role)
    return {joint: spec.id for joint, spec in ARM_SPEC[role].items()}


def motor_spec(role: str, joint: str) -> MotorSpec:
    """Return the :class:`MotorSpec` for *role* and *joint*.

    Parameters
    ----------
    role:
        One of ``"follower"`` or ``"leader"``.
    joint:
        One of the six joint names in :data:`JOINTS`.

    Raises
    ------
    ValueError
        If *role* or *joint* is unknown.
    """
    _require_role(role)
    specs = ARM_SPEC[role]
    if joint not in specs:
        raise ValueError(
            f"Unknown joint {joint!r} for role {role!r}. Valid joints: {list(JOINTS)}."
        )
    return specs[joint]


def resolve_contact_thresholds(
    *,
    blanket: Optional[int] = None,
    per_joint: Optional[Mapping[str, int]] = None,
    from_file: Optional[Mapping[str, int]] = None,
) -> tuple[int, ...]:
    """Resolve one contact threshold per joint, in :data:`JOINTS` order.

    Precedence per joint (first match wins): ``per_joint`` > ``blanket`` >
    ``from_file`` > :data:`DEFAULT_CONTACT_THRESHOLDS`.

    ``blanket`` (the ``arm explore --threshold`` flag) is deliberately NOT a
    fallback default — it only takes effect for a joint when EXPLICITLY
    given (i.e. not ``None``), broadcasting that one value to every joint
    that ``per_joint`` doesn't already cover. Passing ``blanket=None`` (the
    flag simply absent) means every joint falls through to ``from_file``/the
    built-in default instead of collapsing to a fixed number — mirroring the
    "explicit ``None`` check" idiom used elsewhere in this codebase (e.g. an
    explicit ``--threshold 0`` must be honoured, not treated as falsy).

    Parameters
    ----------
    blanket:
        An explicit all-joints override (``--threshold N``), or ``None`` if
        the flag was not given.
    per_joint:
        Per-joint overrides keyed by joint name (``--threshold-joint
        NAME=VAL``, repeatable). Highest precedence.
    from_file:
        Per-joint values parsed from a ``--threshold-file``, keyed by joint
        name. Lowest precedence above the built-in default.

    Returns
    -------
    tuple[int, ...]
        One threshold per joint, length ``len(JOINTS)``, indexed 0-based to
        match the explore engine's ``joint`` int (``motor = joint + 1``).

    Raises
    ------
    ValueError
        If ``per_joint`` or ``from_file`` names a joint not in
        :data:`JOINTS`. This module stays free of CLI concerns — callers at
        the CLI layer translate this into a :class:`CliError`.
    """
    per_joint = per_joint or {}
    from_file = from_file or {}

    for name in (*per_joint, *from_file):
        if name not in JOINTS:
            raise ValueError(f"Unknown joint {name!r}. Valid joints: {list(JOINTS)}.")

    resolved: list[int] = []
    for joint in JOINTS:
        if joint in per_joint:
            resolved.append(per_joint[joint])
        elif blanket is not None:
            resolved.append(blanket)
        elif joint in from_file:
            resolved.append(from_file[joint])
        else:
            resolved.append(DEFAULT_CONTACT_THRESHOLDS[joint])
    return tuple(resolved)


def role_motors(role: str) -> dict[str, MotorSpec]:
    """Return a ``{joint_name: MotorSpec}`` mapping for *role*.

    Parameters
    ----------
    role:
        One of ``"follower"`` or ``"leader"``.

    Raises
    ------
    ValueError
        If *role* is not one of the known roles.
    """
    _require_role(role)
    return dict(ARM_SPEC[role])


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------


def _require_role(role: str) -> None:
    """Raise :class:`ValueError` if *role* is not a known arm role."""
    if role not in _KNOWN_ROLES:
        raise ValueError(f"Unknown arm role {role!r}. Valid roles: {sorted(_KNOWN_ROLES)}.")
