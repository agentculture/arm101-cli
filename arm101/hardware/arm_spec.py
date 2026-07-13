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

``wrist_roll``'s soft limit — why a joint can need a SOFTWARE-only dead arc
----------------------------------------------------------------------------
The STS3215 encoder is 12-bit: raw position ticks run ``[0, 4095]`` and then
**wrap** — commanding the servo past 4095 rolls it over to read back near 0,
and vice versa. Every piece of code that reasons about a joint's position as a
linear tick axis (``gentle_move``'s arrival check, ``arm explore``'s grid and
reachability map, a plain ``[min, max]`` range) silently assumes that axis
never crosses that wrap point — call it the **seam**.

``docs/hardware-validation-arm-read-flex.md`` (t9, 2026-07-01) caught this
live: ``wrist_roll``, parked at raw position **4** (i.e. sitting almost
exactly *on* the seam) after an earlier mapping run, was commanded 300 ticks
to 304 — a linear-controller-trivial move — and instead ran its full 7.0 s
timeout, converging nowhere, because the straight-line path from 4 to 304
never has to cross the seam but the servo's *actual* physical position that
session did, so the arrival check kept comparing ticks across a discontinuity
that isn't really there in angle-space. A later re-run *clear of* the seam
(3049 → 2749) converged in 2078 ms, exactly as expected. The seam, not the
joint, was the fault.

``elbow_flex`` has real mechanical walls (a measured 2196-tick travel, running
raw ``2107 -> 4095 -> 0 -> 207``), so it is fixed by an encoder **re-zero**:
relocate the seam into the arc the joint physically cannot reach, and every
reachable tick is then on one side of it — linear again. ``wrist_roll`` cannot
take that path. Exploration found **no
wall anywhere** in its travel — measured free range ``[21, 4073]`` — meaning
it rotates freely all the way round. A re-zero only *relocates* the seam;
it can never *evict* it, because eviction requires an arc the joint cannot
reach, and a joint whose travel covers the whole circle has no such arc by
definition — every angle, including wherever the seam is moved to, is inside
its travel. So ``wrist_roll`` instead gets a **soft limit**: a SOFTWARE-only
restriction (never an EEPROM ``min_angle``/``max_angle`` write — see
:class:`SoftLimit` and :func:`dead_arc_contains_seam`) that shrinks its
*permitted* travel to strictly less than a full turn, carving out a **dead
arc** — ticks the joint is simply never commanded into — and placing the seam
inside that dead arc rather than inside the permitted range. A soft limit
whose dead arc does *not* contain the seam buys nothing: the permitted range
would still cross the wrap, and the exact failure above would still happen.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Mapping, Optional

from arm101.hardware.ticks import (
    ENCODER_TICKS,
    MAX_ENCODER_OFFSET,
    RAW_SEAM_TICK,
    TICK_MAX,
    TICK_MIN,
    offset_for_seam_at,
    raw_interval_to_reported,
    seam_tick,
)

if TYPE_CHECKING:  # pragma: no cover - typing only
    # TYPE_CHECKING-only, and it has to be: :mod:`arm101.hardware.classify` imports
    # THIS module (for :class:`UnreachableArc` and :data:`ARC_MARGIN_TICKS`), so a
    # runtime import here would close a cycle. The same shape ``safety`` and
    # ``journal`` use for the bus: the live thing is **passed in as a parameter**, never
    # reached for. That is what keeps this module pure — it can consume a measurement
    # without acquiring the ability to take one (``test_arm_spec_module_never_imports_the_bus``).
    from arm101.hardware.classify import TravelClassification

# Re-exported so that ``arm_spec.seam_tick`` / ``arm_spec.TICK_MAX`` keep naming the
# one implementation rather than a second copy of it. The frame ARITHMETIC lives in
# :mod:`arm101.hardware.ticks` (which imports nothing, so this module — forbidden
# from importing the bus — can depend on it); the frame FACTS, measured on hardware,
# live here. Private alias kept because it is the name the re-zero tests reach for.
_offset_for_seam_at = offset_for_seam_at

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
#: **THE CEILING IS NOT 500. It is whatever the joint can PUSH (issue #43).**
#: Every value here used to be chosen inside a band ``(free-motion peak, 500)``,
#: taking 500 — where ``present_load`` saturates at ``gentle_move``'s
#: ``Torque_Limit`` cap — as the upper bound. That is where the *sensor* stops,
#: not where the *joint* stops. A threshold only fires if the joint can develop
#: more load than it; the real ceiling is therefore the load at that joint's
#: WEAKEST WALL, which is physics, and can be far below saturation.
#:
#: ``wrist_roll`` is the proof and the warning. It was set to 400 inside the
#: fictional band ``(300, 500)``. Its walls press at **272 and 288**. The threshold
#: sat above anything the joint could produce, so contact could never fire — and
#: the joint was catalogued as turning freely through its whole travel, with no
#: wall anywhere, for an entire session. It has two real walls. See
#: :attr:`~arm101.hardware.limits.LimitVerdict.UNFIRABLE_THRESHOLD`.
#:
#: **The floor is lower than it looks, too.** The old floor was each joint's
#: free-motion peak — the load it develops merely ACCELERATING through open space.
#: That number is irrelevant: ``_StallDetector.is_contact`` requires ``load >
#: threshold`` AND a stall, and a joint that is *advancing* never stalls, however
#: hard it is pulling. What actually constrains the floor is the load during the
#: servo's ~95-127 ms pre-onset DEAD WINDOW, when the joint is loaded but has not
#: moved yet — set the threshold under that and the detector arms before the joint
#: does, and phantom-contacts on every move. Both old errors pushed the threshold
#: UP, and up is the direction that blinds it.
#:
#: **STATUS PER JOINT — the ceiling is what matters, and four are unmeasured:**
#:
#:   joint          threshold  wall load (= ceiling)   verdict
#:   elbow_flex     280        >= 500 (saturates)      VALIDATED — fires
#:   wrist_roll     150        272 / 288               VALIDATED — fires (was 400: could not)
#:   shoulder_pan   250        UNKNOWN                 unvalidated ceiling
#:   shoulder_lift  250        UNKNOWN (see below)     unvalidated ceiling — SUSPECT
#:   wrist_flex     250        UNKNOWN                 unvalidated ceiling
#:   gripper        250        UNKNOWN                 unvalidated ceiling
#:
#: ``shoulder_lift`` is the one to distrust first. The only genuine contact ever
#: measured on it loaded to **252**, and its threshold is **250** — a two-tick
#: margin against never firing at all. That is not a margin, it is a coin flip, on
#: the joint that carries the whole arm. Probe it with a deliberately low
#: ``--threshold-joint shoulder_lift=<n>``, read the wall load the probe reports,
#: and set the threshold from that — do not trust 250 to stop it.
DEFAULT_CONTACT_THRESHOLDS: dict[str, int] = {
    "shoulder_pan": 250,
    "shoulder_lift": 250,
    "elbow_flex": 280,
    "wrist_flex": 250,
    # 150, not 400. Its walls press at 272 and 288 (measured 2026-07-13), so 400
    # could never fire. Sits below both, and above its dead-window load: proven on
    # hardware to recover the joint's true BOUNDED travel of 3887 ticks.
    "wrist_roll": 150,
    "gripper": 250,
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
# The encoder's two frames — and the factory offset that separates them
# ---------------------------------------------------------------------------
#
# :data:`TICK_MIN`, :data:`TICK_MAX`, :data:`ENCODER_TICKS`,
# :data:`MAX_ENCODER_OFFSET`, :data:`RAW_SEAM_TICK`, :func:`seam_tick` and
# :func:`_offset_for_seam_at` are imported from :mod:`arm101.hardware.ticks`,
# which owns the reported<->raw arithmetic outright. Read that module's docstring
# before touching anything below: **every tick persisted in this module is RAW** —
# a physical angle, unchanged by any number of re-zeros — and the conversion to
# the REPORTED frame a servo speaks happens at exactly one place per call path.


#: The encoder offset a factory-fresh STS3215 actually ships holding: **85**.
#: **Not 0** — which is what every source, and this codebase's first re-zero,
#: assumed.
#:
#: Measured on the follower on 2026-07-12, before anything had been written to
#: any servo's addr 31: **all six joints read ``Ofs = 85``.** Uniform across six
#: independently-manufactured servos, so it is a vendor default baked into the
#: firmware image — not a per-servo calibration, and not something a user did.
#: Confirmed to be a genuine correction rather than a stale register by a
#: reversible probe: writing ``85 -> 185`` dropped the reported position by
#: exactly 100, and zeroing ``85 -> 0`` raised it by exactly 85. That is
#: ``reported = raw - Ofs``, exactly as :func:`seam_tick` assumes.
#:
#: Two consequences, and both are load-bearing:
#:
#: * **A "factory" servo's reported positions are NOT raw ticks.** They are
#:   already shifted by −85. Anything that treats a reported tick as a raw tick
#:   is wrong by 85 on a brand-new arm — which is the default state of every
#:   SO-101. Both tick tables in this module shipped with exactly that error
#:   (``REZERO_ARCS``, fixed 2026-07-12; ``SOFT_LIMITS``, fixed here).
#: * **The factory seam is at raw 85**, which is inside ``elbow_flex``'s travel
#:   (its travel includes the raw band ``[0, 207]``) — that is issue #35 — and it
#:   is where ``wrist_roll``'s reported seam sits *permanently*, since ``wrist_roll``
#:   is the one joint a re-zero can never help (:func:`rezero_refusal`) and its
#:   offset therefore never moves off this value. :data:`SOFT_LIMITS` is placed
#:   around it.
#:
#: Kept here rather than in ``ticks.py`` because it is a *fact about the hardware*,
#: in the module that holds facts about the hardware. Nothing in the re-zero path
#: branches on it — :func:`rezero_arc` and ``plan_rezero`` read the servo's live
#: offset and convert, rather than assuming any particular starting value, and that
#: is the whole point of the fix.
FACTORY_ENCODER_OFFSET: int = 85


# ---------------------------------------------------------------------------
# Encoder wrap — software-only soft limits
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class SoftLimit:
    """A SOFTWARE-only travel restriction in **RAW** ticks, as a plain interval.

    **RAW TICKS — the same rule as :class:`UnreachableArc`, and for the same reason.**
    A soft limit is a claim about *physical angles*: "never drive this joint into
    that part of the circle". A physical angle is a raw tick — the magnet on the
    shaft — and it does not move when the offset register is written. Store the
    limit in reported ticks and it is only true for the offset it was measured
    through; re-zero the joint and the same two numbers now fence off a different
    piece of the circle, silently. (``SOFT_LIMITS`` shipped in the reported frame
    and was corrected to raw; ``REZERO_ARCS`` made the identical mistake and was
    corrected on 2026-07-12. Twice is a pattern, so the frame is now in the type's
    name and enforced by :func:`_require_seam_clearance`.)

    Anything read off a live servo is REPORTED and must be converted
    (:func:`arm101.hardware.ticks.raw_from_reported`) before it is compared against
    a limit — and the limit must be converted the other way
    (:func:`permitted_reported_range`) before it becomes a goal a servo can be
    given. Both crossings go through :mod:`arm101.hardware.ticks`.

    This is deliberately the *only* shape a soft limit is ever expressed in:
    a ``min_tick < max_tick`` interval within ``[TICK_MIN, TICK_MAX]`` — never
    a "min greater than max means wrap around" encoding. That second, more
    "clever" representation is exactly what this module exists to avoid: it
    would let a range *cross* the seam by construction, silently reintroducing
    the bug documented in the module docstring. Because every ``SoftLimit`` is
    a plain interval, the excluded region — the **dead arc** — is always
    ``[TICK_MIN, min_tick) ∪ (max_tick, TICK_MAX]``: the arc that runs
    *through* the raw seam. See :func:`dead_arc_contains_seam`, which checks that
    this dead arc is non-empty rather than assuming it, and
    :func:`_require_seam_clearance`, which checks that it also contains the
    *reported* seam — a different tick, and the one the servo actually moves
    across.

    Attributes
    ----------
    min_tick, max_tick : int
        The permitted (non-dead) range in RAW ticks, inclusive on both ends.

    Raises
    ------
    ValueError
        If the pair is not a well-ordered interval within
        ``[TICK_MIN, TICK_MAX]`` (i.e. NOT
        ``TICK_MIN <= min_tick < max_tick <= TICK_MAX``).
    """

    min_tick: int
    max_tick: int

    def __post_init__(self) -> None:
        if not (TICK_MIN <= self.min_tick < self.max_tick <= TICK_MAX):
            raise ValueError(
                f"Invalid soft limit ({self.min_tick}, {self.max_tick}): requires "
                f"{TICK_MIN} <= min_tick < max_tick <= {TICK_MAX}."
            )

    @property
    def dead_arc_ticks(self) -> int:
        """Combined width, in ticks, of the excluded arc on both sides of the seam.

        ``(min_tick - TICK_MIN) + (TICK_MAX - max_tick)``. This is the
        concrete number an operator is trading away for linearity — the exact
        width is a tunable (see :data:`SOFT_LIMITS`'s docstring), but this
        property makes "how much" legible rather than buried in two subtractions.
        """
        return (self.min_tick - TICK_MIN) + (TICK_MAX - self.max_tick)

    def permits(self, tick: int) -> bool:
        """``True`` iff RAW *tick* falls inside the permitted range, inclusive.

        Takes a **raw** tick, like everything else on this class. Hand it a
        position read straight off a servo and you have compared the wrong frame:
        convert first (:func:`arm101.hardware.ticks.raw_from_reported`).
        """
        return self.min_tick <= tick <= self.max_tick

    def clearance_from(self, tick: int) -> int:
        """Ticks between RAW *tick* and the nearest permitted tick, measured round the circle.

        ``0`` if *tick* is permitted (no clearance: it is not in the dead arc at
        all). Otherwise the smaller of the two ways round from *tick* to the edge
        of the permitted range — so it answers "how much room does the dead arc
        actually leave here?", which is the question a seam needs answered.

        Circular, not linear, because both ends of the dead arc are the same arc:
        a tick at 4090 is 6 ticks from tick 0, not 4090. Computing that with a
        subtraction is how a "200-tick dead arc" turns out to leave 15 ticks of
        margin on the side that matters.
        """
        if self.permits(tick):
            return 0
        upward = (self.min_tick - tick) % ENCODER_TICKS
        downward = (tick - self.max_tick) % ENCODER_TICKS
        return min(upward, downward)


def dead_arc_contains_seam(min_tick: int, max_tick: int) -> bool:
    """Return ``True`` iff excluding ``[min_tick, max_tick]`` leaves a dead arc over the RAW seam.

    **The RAW seam** (:data:`~arm101.hardware.ticks.RAW_SEAM_TICK`) — the
    ``TICK_MAX -> TICK_MIN`` rollover of the magnet's own count, which no offset
    can move. This predicate says nothing about the *reported* seam (at ``raw ==
    Ofs``, a different tick, and the one a goal write crosses); that is
    :func:`dead_arc_contains_reported_seam`, and a range can clear one while
    sitting squarely on the other. Both matter, so both are checked.

    This is the enforceable version of the argument in the module docstring,
    made concrete rather than left as a comment someone can outrun with an
    edit. For any well-ordered interval ``TICK_MIN <= min_tick < max_tick <=
    TICK_MAX``, the excluded region is exactly
    ``[TICK_MIN, min_tick) ∪ (max_tick, TICK_MAX]`` — the arc that runs
    through the ``TICK_MAX -> TICK_MIN`` wrap, i.e. through the raw seam. That
    region is non-empty, and therefore genuinely contains that seam, iff
    ``min_tick > TICK_MIN or max_tick < TICK_MAX``. The one case where it is
    EMPTY is ``(min_tick, max_tick) == (TICK_MIN, TICK_MAX)``: the full turn,
    nothing excluded, seam still fully reachable. That degenerate case is
    exactly "a soft limit that still spans the seam" — it buys nothing, and
    this function is what makes that failure mode fail a test instead of
    quietly reintroducing the bug the next time someone "simplifies" a range.

    Parameters
    ----------
    min_tick, max_tick : int
        A candidate permitted range in RAW ticks, in the plain-interval form
        :class:`SoftLimit` always uses.

    Returns
    -------
    bool
        ``True`` if the dead arc is non-empty (contains the raw seam), ``False``
        only for the degenerate full-range case.

    Raises
    ------
    ValueError
        If ``min_tick``/``max_tick`` do not form a well-ordered interval
        within ``[TICK_MIN, TICK_MAX]``. A malformed interval isn't a range
        this predicate can classify one way or the other.
    """
    if not (TICK_MIN <= min_tick < max_tick <= TICK_MAX):
        raise ValueError(
            f"({min_tick}, {max_tick}) is not a valid tick interval: requires "
            f"{TICK_MIN} <= min_tick < max_tick <= {TICK_MAX}."
        )
    return min_tick > TICK_MIN or max_tick < TICK_MAX


def dead_arc_contains_reported_seam(limit: SoftLimit, offset: int) -> bool:
    """``True`` iff *limit*'s dead arc contains the seam of a servo holding *offset*.

    The second of the two questions, and the one the shipped table got wrong.

    A goal position is written in the servo's own REPORTED frame, so the mover's
    linear-tick assumption — ``gentle_move``'s arrival check, ``clamp_goal``, the
    whole idea of a ``(min, max)`` pair — breaks at the *reported* seam, which sits
    at raw tick ``Ofs`` (:func:`~arm101.hardware.ticks.seam_tick`) and moves
    whenever the offset is written. A soft limit that excludes only the raw seam
    leaves the joint perfectly free to be commanded straight across that one.

    Equivalently, and this is why the check earns its place rather than being a
    nicety: an interval whose dead arc contains the reported seam is *exactly* an
    interval that does not wrap in the reported frame — i.e. one that
    :func:`permitted_reported_range` can convert into a single well-ordered pair a
    mover can clamp against. Fail this and there is no honest ``(min, max)`` to
    hand the servo at all.
    """
    return not limit.permits(seam_tick(offset))


def _require_dead_arc_contains_seam(table: Mapping[str, SoftLimit]) -> None:
    """Raise ``ValueError`` if any entry in *table* does not satisfy :func:`dead_arc_contains_seam`.

    Called once, below, against :data:`SOFT_LIMITS` at import time — so a
    future edit that widens a joint's range back out to the full turn (even a
    "harmless-looking" one-line change) fails LOUDLY the moment the module is
    imported, for every caller and every test, rather than silently shipping
    a soft limit that no longer excludes the seam. This is the difference
    between the guarantee being enforced and the guarantee being merely
    documented.
    """
    for joint, limit in table.items():
        if not dead_arc_contains_seam(limit.min_tick, limit.max_tick):
            raise ValueError(
                f"Soft limit for {joint!r} is ({limit.min_tick}, {limit.max_tick}), "
                f"which spans the full {TICK_MIN}-{TICK_MAX} encoder range — its dead "
                "arc is empty, so it does not contain the encoder seam and the limit "
                "buys nothing. Narrow the range so the dead arc contains the seam."
            )


def _require_no_soft_limit_and_arc(table: Mapping[str, SoftLimit]) -> None:
    """Raise ``ValueError`` if any joint in *table* ALSO has an unreachable arc.

    A soft limit and a re-zero are the two **mutually exclusive** answers to a wrapping
    joint. A re-zeroable joint EVICTS its seam, so its offset does not stay put — and a
    soft limit whose dead arc was placed around the seam of one offset fences off the
    wrong part of the circle the moment the joint is re-zeroed to another. Holding both
    is not belt-and-braces; it is two tables describing different arms.

    Extracted from :func:`_require_seam_clearance` (which enforces it on the shipped
    table at import) so :func:`resolve_soft_limits` can enforce the same rule on a
    **measured** override — where the collision is live rather than hypothetical: a
    joint whose arc is in :data:`REZERO_ARCS` and whose fresh measurement came back
    "arc too narrow, use a soft limit" is a genuine contradiction between the table and
    the arm, and the operator has to settle it. Silently letting the override win would
    leave ``rezero_arc`` still offering an offset for a joint the mover is now fencing.
    """
    for joint in table:
        if joint in REZERO_ARCS:
            raise ValueError(
                f"Joint {joint!r} has BOTH a soft limit and a re-zero arc. Those are the two "
                "mutually exclusive answers to a wrapping joint: a re-zeroable joint EVICTS "
                "its seam, so its offset is not pinned to the factory value and this table "
                "cannot know where its reported seam will be. Pick one."
            )


def _require_seam_clearance(table: Mapping[str, SoftLimit]) -> None:
    """Raise ``ValueError`` unless every entry clears BOTH seams by :data:`SEAM_CLEARANCE_TICKS`.

    **This is the check that makes "``SOFT_LIMITS`` is RAW" a fact rather than a
    comment**, and it is the one the previous, reported-frame table cannot pass.

    A soft-limited joint is by definition one an encoder re-zero cannot help — it
    turns freely all the way round, so there is no unreachable arc to evict the
    seam into (:data:`_REZERO_IMPOSSIBLE`). Its offset therefore never moves off
    the factory value, and its reported seam sits, permanently, at raw
    :data:`FACTORY_ENCODER_OFFSET`. So the dead arc must contain **two** ticks with
    room to spare:

    * :data:`~arm101.hardware.ticks.RAW_SEAM_TICK` — the frame the limit is
      *stored and compared* in;
    * ``seam_tick(FACTORY_ENCODER_OFFSET)`` — the frame the limit is *used* in,
      i.e. the one a goal write crosses.

    They are 85 ticks apart, which is precisely small enough for a table written in
    the wrong frame to look right: the shipped ``(100, 3995)`` clears the raw seam
    by a comfortable 101 ticks and the reported seam by **15** — under
    ``gentle_move``'s own 12-tick arrival tolerance plus encoder jitter, so an
    arrival check could settle *on the seam*. It is the same 85-tick error that put
    ``REZERO_ARCS`` a whole factory offset out (fixed 2026-07-12), and it survived
    every existing test because ``FakeBus`` defaults to ``Ofs = 0``, where the two
    frames coincide and the bug is invisible.

    Also rejects a joint that has both a soft limit and a re-zero arc
    (:func:`_require_no_soft_limit_and_arc`): those are the two mutually exclusive
    answers to a wrapping joint, and if the joint really can be re-zeroed then its
    offset is NOT pinned to the factory value and the whole premise of the check above
    evaporates. Better to say so than to check the wrong seam.
    """
    _require_no_soft_limit_and_arc(table)
    for joint, limit in table.items():
        for seam, what in (
            (RAW_SEAM_TICK, "the RAW seam (the magnet's own 4095->0 rollover)"),
            (
                seam_tick(FACTORY_ENCODER_OFFSET),
                f"the REPORTED seam of a factory servo (raw == Ofs == {FACTORY_ENCODER_OFFSET}), "
                "which is the seam a goal write actually crosses",
            ),
        ):
            clearance = limit.clearance_from(seam)
            if clearance < SEAM_CLEARANCE_TICKS:
                raise ValueError(
                    f"Soft limit for {joint!r} is ({limit.min_tick}, {limit.max_tick}) and sits "
                    f"{clearance} ticks from the seam at raw {seam} — {what} — but "
                    f"{SEAM_CLEARANCE_TICKS} ticks of clearance are required. If those numbers "
                    "were measured off a live servo they are REPORTED ticks, and this table is "
                    "RAW: convert them (ticks.raw_from_reported) before storing them."
                )


#: How far a soft limit's permitted range must stay clear of a seam, in ticks.
#:
#: The one tunable behind :data:`SOFT_LIMITS`, and the number that must survive
#: contact with the two things that can put a tick near the edge by accident:
#:
#: * encoder read jitter — a handful of ticks;
#: * ``gentle_move``'s arrival tolerance — 12 ticks
#:   (``arm101.hardware.gentle._DEFAULT_ARRIVAL_TOLERANCE``), so a move that
#:   "arrived" may be sitting up to 12 ticks from where it was told to go.
#:
#: 100 ticks (~8.8°) is ~8x that combined worst case: an arrival check settling
#: near either edge of the permitted range cannot land on a seam by accident. An
#: operator may revisit it with more hardware data; what must never change is that
#: the dead arc *contains* both seams, which is enforced
#: (:func:`_require_seam_clearance`), not merely documented.
SEAM_CLEARANCE_TICKS: int = 100

#: Per-joint software travel restrictions for joints whose encoder wraps
#: within (or across the whole of) their physical travel. **RAW ticks** — see
#: :class:`SoftLimit`. A joint absent from
#: this table has no soft limit — its full ``[TICK_MIN, TICK_MAX]`` range is
#: permitted. Read that as "this table restricts nothing here", **not** as "that joint
#: does not wrap": the four joints with neither a soft limit nor a measured arc are
#: simply unmeasured (issue #43, and :data:`_REZERO_ARC_UNKNOWN`).
#: ``elbow_flex`` is the other joint known to wrap, but
#: it takes the encoder RE-ZERO path instead: it
#: has real mechanical walls, so its seam can be relocated into an arc it
#: physically cannot reach, and it therefore needs no entry here.
#:
#: ``wrist_roll`` — the only entry — has none: exploration drove it across
#: its entire measured free range with **no wall found anywhere**
#: (``docs/hardware-validation-arm-read-flex.md``), i.e. it rotates freely all the
#: way round, so re-zero cannot fix it even in principle (see the module
#: docstring's impossibility argument), and its offset therefore stays at
#: :data:`FACTORY_ENCODER_OFFSET` forever.
#:
#: **The range is DERIVED, not typed** — from the seam and the clearance, in RAW
#: ticks. That is not tidiness: a typed pair is a pair that can be in the wrong
#: frame, and this table shipped in the wrong frame. The shipped value was
#: ``(100, 3995)``, measured off a live servo and therefore REPORTED; read as raw
#: it leaves 15 ticks between the permitted range and the seam the servo actually
#: moves across. Derived from ``seam_tick(FACTORY_ENCODER_OFFSET)`` there is no
#: number left to get wrong, and :func:`_require_seam_clearance` proves it.
#:
#: The resulting dead arc — ``[0, 185) ∪ (3995, 4095]``, 285 ticks — spans BOTH
#: seams (they are 85 ticks apart) with :data:`SEAM_CLEARANCE_TICKS` to spare on
#: each outer side, and leaves 3811 of 4096 ticks (~93%) usable. The exclusion is
#: real (this is not "spans the seam" in disguise: see
#: :func:`dead_arc_contains_seam`) but deliberately small, since nothing measured
#: suggests the joint needs more room than that.
#:
#: On the "measured free envelope". The t9 sweep reported ``[21, 4073]``, and the
#: old entry claimed to sit inside it. Converted to the raw frame it was measured
#: in (``raw = reported + 85``) that envelope is ``[106 .. 4095] ∪ [0 .. 62]`` —
#: it *wraps*, covering all but a 43-tick raw gap at ``(62, 106)``. The gap
#: straddles raw 85. Of course it does: that is the seam, and the sweep never
#: crossed it. So the envelope confirms wrist_roll reaches essentially the whole
#: circle and constrains the soft limit not at all — the only thing that places
#: this range is the seam. Reading it as a wall was the frame confusion in
#: miniature.
SOFT_LIMITS: dict[str, SoftLimit] = {
    "wrist_roll": SoftLimit(
        min_tick=seam_tick(FACTORY_ENCODER_OFFSET) + SEAM_CLEARANCE_TICKS,
        max_tick=TICK_MAX - SEAM_CLEARANCE_TICKS,
    ),
}

_require_dead_arc_contains_seam(SOFT_LIMITS)
# _require_seam_clearance(SOFT_LIMITS) is run at the FOOT of this module: it
# cross-checks against REZERO_ARCS (a joint may not have both), which is not
# defined yet at this point in the file.


# ---------------------------------------------------------------------------
# Encoder re-zero — EVICTING the seam from a joint's travel (issue #35)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class UnreachableArc:
    """The contiguous arc of **RAW** encoder ticks a joint physically CANNOT reach.

    The mirror image of a :class:`SoftLimit`, and the thing that makes an
    encoder re-zero possible at all. A joint with real mechanical walls cannot
    rotate all the way round, so some arc of the encoder's 4096 ticks is
    permanently out of reach — and *that* is where the seam belongs. Move the
    seam into an arc the joint can never visit and it can never be crossed;
    every tick the joint can actually reach then lies on one side of it, and
    the tick axis is linear again. A joint with no such arc (``wrist_roll``,
    which turns freely through the whole circle) cannot be helped this way at
    all — there is nowhere to put the seam. See :func:`rezero_refusal`.

    **RAW TICKS. NOT REPORTED TICKS. THIS IS THE WHOLE POINT.**
    ---------------------------------------------------------
    Say it loudly because getting it wrong is silent, and it *was* got wrong
    (fixed 2026-07-12; the arc this table originally shipped with —
    ``(126, 2020)`` — was measured on a servo already holding the factory
    :data:`FACTORY_ENCODER_OFFSET` of 85, i.e. in the REPORTED frame, and then
    used as if it were raw). An arc is a claim about **the magnet on the shaft**:
    which physical angles the joint's own mechanical walls forbid. It is a
    property of the *joint*, and it is **independent of whatever number happens
    to be sitting in the offset register** — writing an offset changes what the
    servo *reports*, and moves nothing at all. Re-zero the joint twice, or
    ten times, and the arc is the same arc.

    The offset register operates in this same raw frame (the seam lands where
    ``Actual == Ofs``, see :func:`seam_tick`), which is exactly why the arc must
    be raw: an arc expressed in reported ticks would be off by the pre-existing
    offset, and the target computed from it would place the seam by that much
    error. It happens to be survivable when the arc is wide and the pre-existing
    offset is small — the ``(126, 2020)`` bug shipped a target that still landed
    inside the true arc, by luck and with margin to spare — and it is fatal when
    either of those stops being true.

    So: **anything read off a live servo is REPORTED and must be converted**
    (``raw = (reported + offset) mod 4096``,
    :func:`arm101.hardware.ticks.raw_from_reported`) before it is compared
    against an arc. There is no frame in which both readings are the same except
    the one where the register happens to hold 0 — and no servo ships that way.

    Expressed as the OPEN interval ``(low, high)``: both endpoints are positions
    the joint *can* reach (they are its hard walls, or the last positions
    measured before them), and everything strictly between them is unreachable.

    Attributes
    ----------
    low, high : int
        The reachable RAW ticks bounding the unreachable arc, ``low < high``.

    Raises
    ------
    ValueError
        If ``(low, high)`` is not a well-ordered interval within
        ``[TICK_MIN, TICK_MAX]``.
    """

    low: int
    high: int

    def __post_init__(self) -> None:
        if not (TICK_MIN <= self.low < self.high <= TICK_MAX):
            raise ValueError(
                f"Invalid unreachable arc ({self.low}, {self.high}): requires "
                f"{TICK_MIN} <= low < high <= {TICK_MAX}."
            )

    @property
    def width(self) -> int:
        """Width of the unreachable arc in ticks (``high - low``)."""
        return self.high - self.low

    @property
    def travel_ticks(self) -> int:
        """The joint's physical travel: everything the arc does NOT exclude.

        ``ENCODER_TICKS - width``. For ``elbow_flex`` this is **2196 ticks**, and
        as of 2026-07-12 that is a *measurement*, not a bound: the far wall was
        finally seen by a torque-off hand sweep with the seam evicted (the one
        instrument that can cross where ``arm explore`` could not).
        """
        return ENCODER_TICKS - self.width

    @property
    def midpoint(self) -> int:
        """The raw tick dead-centre of the arc — where the seam gets the most clearance.

        The *target*, not the *goal*. The goal is :meth:`evicts`: the seam must be
        somewhere the joint can never reach. Any tick strictly inside the arc
        achieves that, and the midpoint is merely the one that maximises the
        margin on both sides — so an arc endpoint that is off by a few ticks
        still leaves the seam safely out of reach. It is what a re-zero writes
        when it writes anything at all; it is emphatically NOT a number a servo
        has to be holding for the joint to be considered fixed.
        """
        return (self.low + self.high) // 2

    def contains(self, tick: int) -> bool:
        """``True`` iff RAW *tick* is strictly inside the arc — i.e. unreachable.

        Strict, because the endpoints are exactly the ticks the joint CAN
        reach. A joint reporting a raw position for which this is ``True`` is
        reporting a position it should be physically incapable of holding — the
        arc is wrong, or the servo is not the one we think it is.

        Takes a **raw** tick, like everything else on this class. Hand it a
        position read straight off a servo and you have compared the wrong frame:
        convert first (:func:`arm101.hardware.ticks.raw_from_reported`).
        """
        return self.low < tick < self.high

    def evicts(self, offset: int) -> bool:
        """``True`` iff a servo holding *offset* has its seam OUT of the joint's travel.

        The actual goal of a re-zero, stated once and in one place: not "the
        register holds a particular number" but "the discontinuity is somewhere
        the joint can never go". An offset evicts the seam iff its seam tick
        (:func:`seam_tick` — the offset reduced modulo 4096, because the register
        is signed and the encoder is not) lands strictly inside this arc.

        Two things follow, and they are why this method exists rather than an
        ``== midpoint`` comparison scattered at the call sites:

        * **A servo already holding a *different* evicting offset is already
          fixed.** Rewriting its EEPROM to centre the seam more prettily would
          buy nothing physical and spend a write on a finite-write part. (Our
          follower holds ``1073``, from the first — frame-confused — re-zero; its
          seam sits at raw 1073, comfortably inside ``(207, 2107)``, and a hand
          sweep proved the travel continuous. It is done. Leave it alone.)
        * **A servo holding the factory** :data:`FACTORY_ENCODER_OFFSET` **is
          not.** Its seam sits at raw 85 — inside ``elbow_flex``'s ``[0, 207]``
          reachable band, which is precisely issue #35.
        """
        return self.contains(seam_tick(offset))

    @property
    def offset(self) -> int:
        """The signed register value that puts the seam at this arc's :attr:`midpoint`.

        The single source of the "what would a fresh re-zero write?" answer:
        :func:`rezero_offset` is this, and so is every report that quotes a
        target. Derived, so an arc correction propagates to the offset without
        anybody remembering to follow it — which is what let the 2026-07-12
        re-measurement move the target from 1073 to 1157 by editing one tuple.
        """
        return _offset_for_seam_at(self.midpoint)


#: Per-joint unreachable arcs — the joints an encoder re-zero can actually fix.
#: **RAW ticks** (:class:`UnreachableArc`), which is the correction this table
#: carries: the numbers it shipped with were not.
#:
#: **The SHIPPED DEFAULT, no longer the only source.** An arc is a *measurement*, and
#: :func:`arc_from_measurement` now derives one from a live
#: :class:`~arm101.hardware.classify.TravelClassification` — so a joint can get an arc
#: (and therefore a re-zero) without a human typing one in, which is the only way the
#: procedure that fixed ``elbow_flex`` was ever going to generalise. Pass ``measured=``
#: to :func:`rezero_arc` / :func:`rezero_offset` / :func:`rezero_refusal` and the
#: measurement wins over this table; pass nothing and this table answers, as it always
#: did, with no arm plugged in.
#:
#: ``elbow_flex`` is the only entry — the only joint whose arc anybody has **measured**.
#: That is a statement about our measurements, and **not** a statement about the other
#: five joints' encoders (issue #43: three joints reached their commandable bound with
#: no contact at all, 2, 3 and 11 raw ticks from the seam, because at the factory offset
#: the bound lands *on* the seam — so their travel may well wrap it and nothing could
#: see past it). A joint absent from this table has an **unknown** arc, not a
#: nonexistent one; see :data:`_REZERO_ARC_UNKNOWN`.
#:
#: ``elbow_flex``'s encoder WRAPS inside its physical travel (issue #35): driven far enough it
#: crosses the raw 4095->0 seam and reads back near zero, so its reported
#: position is **not monotonic with joint angle** and every position comparison
#: in this codebase — ``gentle_move``'s arrival check, ``clamp_goal``, the
#: reachability map's ``(min, max)`` ranges — is silently wrong for it. Worse,
#: sorting its two measured endpoints into a ``[min, max]`` pair yields exactly
#: the arc it CANNOT reach.
#:
#: Measured on hardware, follower ``/dev/ttyACM1``, **2026-07-12** — a torque-off
#: hand sweep of the joint's *entire* travel with the seam already evicted
#: (``Ofs = 1073``), which is the first instrument that could ever see across the
#: wrap. It reported a **monotonic, discontinuity-free** run of **2196 ticks**,
#: spanning **1034 .. 3230 in that corrected frame**. Converting back to the raw
#: frame the shaft actually lives in (``raw = (reported + 1073) mod 4096``):
#:
#: * near wall  : reported 1034 -> raw **2107**
#: * far wall   : reported 3230 -> raw **207**  (``3230 + 1073 = 4303 mod 4096``)
#: * so travel runs ``2107 -> 4095 -> |raw seam| -> 0 -> 207``, i.e. the raw set
#:   ``[2107, 4095] ∪ [0, 207]`` — **2196 ticks**, and it WRAPS, which is exactly
#:   the fact a ``[min, max]`` pair cannot express and this arc exists to state.
#: * the complement — the arc it cannot reach — is therefore **(207, 2107)**:
#:   **1900 ticks**, and *this is a measurement*, not the upper bound the old
#:   entry was.
#:
#: **The far wall had never been measured before this run** (nothing could see
#: across the seam — that was the whole problem), so the old entry was built from
#: the near wall plus the rest position and was a bound in both directions.
#:
#: **Why the old ``(126, 2020)`` was WRONG, not merely loose** (this is the bug
#: fixed here, and it is a reasoning bug, not an arithmetic one): 126 and 2020
#: were read off a servo that was *already holding the factory
#: :data:`FACTORY_ENCODER_OFFSET` of 85*. They are **REPORTED** ticks. The offset
#: register works in RAW ticks (:func:`seam_tick`), so the target derived from
#: them — midpoint 1073 — was computed in the wrong frame and was off by the
#: pre-existing offset. **It worked anyway, by luck:** raw 1073 lands inside the
#: true arc ``(207, 2107)`` with ~866/1034 ticks of margin to spare. A narrower
#: arc, or a larger factory offset, and that same "correct" read-back would have
#: parked the seam back inside the joint's travel with nothing to show for it.
#:
#: Midpoint **1157** — where a *fresh* re-zero puts the seam: **950 ticks of
#: clearance on each side**, the most the arc allows. Note that the arm this was
#: measured on is NOT re-written to 1157: it holds 1073, which
#: :meth:`UnreachableArc.evicts` — the seam is out of the travel, the axis is
#: linear, and the job is done. See :func:`rezero_offset`.
#: The furthest into the LOW band ``elbow_flex`` can reach (raw ticks) — measured
#: BY THE ARM ITSELF, not by hand.
#:
#: ``gentle_move`` was driven past the known travel and let the load-watch find the
#: wall: it contacted at corrected 3274 (raw 251) with ``present_load`` SATURATED at
#: the 500 torque cap, backed off, and held. A saturated load is the signature of a
#: real wall, not of an operator deciding it felt firm enough.
#:
#: THE ARM BEATS THE HAND. Successive human sweeps put this wall at 206, then 218 —
#: it moves with how hard you push. The arm presses to a fixed load threshold every
#: time, so it is both further (251) and REPEATABLE. An earlier arc edge set from a
#: hand sweep false-refused within minutes when the joint came to rest one tick past
#: it. Take the machine's number.
_LOW_WALL_OBSERVED = 251

#: The deepest into the HIGH band ``elbow_flex`` can reach (raw ticks) — again
#: measured by the arm: contact at corrected 988 (raw 2061), load saturated at 500.
#: Hand sweeps only ever reached 2107/2118; the arm pushed 46 ticks further.
_HIGH_WALL_OBSERVED = 2061

#: Inset applied to EACH side of a measured envelope before declaring an arc.
#:
#: A hand-found wall is not a crisp number (206..218 depending on push force), and
#: at least one of these limits may be the TABLE rather than the joint's mechanical
#: stop — the operator had to move the gripper aside "or we would hit the table".
#: An environmental wall makes the true travel WIDER than measured, so the true
#: unreachable arc is NARROWER. Insetting is conservative against both.
#:
#: The cost is nothing: the arc must merely CONTAIN the seam — a single tick would
#: suffice — and after the inset it still spans ~1700.
#:
#: **PUBLIC, because it is shared.** :mod:`arm101.hardware.classify` applies this same
#: inset to a *live* measurement (and sizes its narrow-arc cutoff from it), and it
#: imports the number rather than re-typing it: two copies of a safety margin is one
#: copy too many — a re-measurement that widens it here must widen it there, without
#: anyone remembering to. It was private while ``REZERO_ARCS`` was the only thing that
#: applied it; it is not any more.
ARC_MARGIN_TICKS: int = 100

#: Deprecated private alias for :data:`ARC_MARGIN_TICKS`. Kept because it is the name
#: some existing callers and tests reach for; it names the same number, not a copy.
_ARC_MARGIN_TICKS = ARC_MARGIN_TICKS

REZERO_ARCS: dict[str, UnreachableArc] = {
    # RAW ticks, hardware-measured on the follower, 2026-07-12, with a DELIBERATE
    # MARGIN. Read the margin note below before "tightening" this to the measured
    # numbers — the tight version is what broke.
    #
    # REACHABLE ENVELOPE, measured BY THE ARM (gentle_move driven past the known
    # travel until the load-watch found each wall; both contacts saturated at the
    # 500 torque cap, which is what a real wall looks like):
    #     raw reachable = [2061, 4095] ∪ [0, 251]
    #     => the true unreachable arc is AT MOST (251, 2061), width 1810
    #
    # The arm out-measured the human on BOTH sides (hand: 218 / 2107). It presses to
    # a fixed load every time instead of to whatever felt firm, so its walls are
    # further out AND repeatable. This is the arm feeling its own body — the same
    # primitive `arm explore` uses, which is the point of the whole exercise.
    #
    # A first cut declared exactly (207, 2107) — the extremes of one sweep — and
    # it FALSE-REFUSED within minutes: the joint came to rest at raw 218, eleven
    # ticks past an edge taken from a sweep the operator had simply stopped short
    # of, and `arm rezero` correctly reported that the joint "cannot be where it
    # says it is". A wall is not a crisp number: it moved 206..218 depending on
    # how hard a human pushed. An arc set AT a measured extreme is an arc that
    # contradicts the arm the first time someone pushes harder.
    #
    # So the declared arc is a STRICT SUBSET of the unreachable region, inset by
    # ARC_MARGIN_TICKS on each side. Shrinking is conservative in BOTH directions
    # that matter: it cannot false-refuse a legal position, and it cannot claim a
    # tick the joint can actually reach.
    #
    # THE SECOND REASON FOR MARGIN, and the deeper one. These walls were found by
    # hand, and the operator had to move the GRIPPER out of the way "or we would
    # hit the table". So at least one limit may be ENVIRONMENTAL (the table) rather
    # than MECHANICAL (the joint's own stop) — and an environmental wall makes the
    # true travel WIDER than measured, hence the true unreachable arc NARROWER than
    # measured. That is exactly the failure issue #34 is about: the table is the
    # wall, and the table is not in the servo's EEPROM. Margin absorbs it.
    #
    # What keeps this honest rather than hopeful: the seam sits ~855 ticks (~75deg)
    # from the nearest wall ever observed, and the acceptance sweep ran 2196 ticks
    # MONOTONIC with 0 discontinuities — it would have SHOWN a seam crossing had
    # raw 1073 been reachable. The arc only has to CONTAIN the seam; one tick would
    # do, and it keeps ~1700.
    "elbow_flex": UnreachableArc(
        low=_LOW_WALL_OBSERVED + ARC_MARGIN_TICKS,
        high=_HIGH_WALL_OBSERVED - ARC_MARGIN_TICKS,
    ),
}


def _require_evictable_seam(table: Mapping[str, UnreachableArc]) -> None:
    """Raise ``ValueError`` if any arc in *table* cannot actually take the seam.

    Three ways a table entry can be nonsense, all caught at import time — for
    every caller and every test — rather than discovered halfway through an
    EEPROM write on a physical servo:

    1. **The arc does not contain its own seam.** Vacuously true for the
       midpoint of a well-ordered open interval — *unless* the arc is only one
       tick wide (``high == low + 1``), in which case there is no tick strictly
       inside it and the "seam goes here" claim is empty. A one-tick arc means a
       joint whose travel is 4095 of 4096 ticks: essentially ``wrist_roll``, and
       a re-zero is the wrong tool (see :func:`rezero_refusal`).
    2. **The offset is unrepresentable.** The register holds ``[-2047, +2047]``
       (:data:`MAX_ENCODER_OFFSET`); the one seam placement it cannot express is
       raw 2048. An arc whose midpoint lands there needs a human, not a rounding
       rule.
    3. **The signed offset does not land back on the raw tick it came from** —
       i.e. the RAW -> SIGNED -> RAW round-trip
       (:func:`_offset_for_seam_at` then :meth:`UnreachableArc.evicts`, which
       goes through :func:`seam_tick`) does not close. This is the check the
       frame bug of 2026-07-12 would have wanted: the offset written to the
       register and the arc it was derived from must be talking about the same
       tick. It cannot fail while ``_offset_for_seam_at`` and ``seam_tick`` stay
       true inverses — which is exactly why it is worth pinning, because they are
       the two halves of the arithmetic that got confused, and a future edit to
       either that quietly breaks the correspondence would otherwise surface as a
       seam written into a joint's live travel.

    This mirrors :func:`_require_dead_arc_contains_seam` for :data:`SOFT_LIMITS`,
    and for the same reason: the guarantee is *enforced*, not merely documented,
    so a future "harmless" edit to the table fails loudly instead of quietly
    writing a useless offset into a servo's EEPROM.
    """
    for joint, arc in table.items():
        seam = arc.midpoint
        if not arc.contains(seam):
            raise ValueError(
                f"Unreachable arc for {joint!r} is ({arc.low}, {arc.high}), which has no "
                "tick strictly inside it — there is nowhere to evict the seam TO. A joint "
                "whose travel covers all but a sliver of the circle needs a soft limit, "
                "not a re-zero."
            )
        offset = _offset_for_seam_at(seam)
        if abs(offset) > MAX_ENCODER_OFFSET:
            raise ValueError(
                f"Unreachable arc for {joint!r} is ({arc.low}, {arc.high}), whose midpoint "
                f"{seam} needs an offset of {offset} — outside the register's "
                f"[-{MAX_ENCODER_OFFSET}, +{MAX_ENCODER_OFFSET}] range. Raw 2048 is the one "
                "seam placement the sign-magnitude encoding cannot express; pick another "
                "tick inside the arc."
            )
        if not arc.evicts(offset):
            raise ValueError(  # pragma: no cover - unreachable while the inverses hold
                f"Unreachable arc for {joint!r} is ({arc.low}, {arc.high}), but the offset "
                f"{offset} derived from its midpoint {seam} puts the seam at raw tick "
                f"{seam_tick(offset)} — which is NOT inside the arc. The raw<->signed "
                "round-trip is broken: _offset_for_seam_at and seam_tick are no longer "
                "inverses, so the offset written to the register and the arc it came from "
                "are describing different ticks."
            )


_require_evictable_seam(REZERO_ARCS)


#: Why a joint that is MEASURED and still cannot be re-zeroed cannot be re-zeroed —
#: keyed by joint. A cached refusal for joints ``arm limits`` has already probed; a
#: live :class:`~arm101.hardware.classify.TravelClassification` passed to
#: :func:`rezero_refusal` always wins over it.
#:
#: **This table used to be called ``_REZERO_IMPOSSIBLE`` and it used to be wrong.**
#: It carried one entry — ``wrist_roll`` — asserting a *logical* impossibility: that
#: the joint "turns freely all the way round (measured free range [21, 4073])", has
#: no unreachable arc "by definition", and therefore no offset could ever evict its
#: seam. It said, in as many words, "**This refusal is PROVEN and it stays.**"
#:
#: It was not proven. It was an artifact of an instrument that could not fire.
#:
#: ``wrist_roll``'s contact threshold was 400. Re-probed on 2026-07-13 it develops a
#: peak load of **272** at one wall and **288** at the other — so the contact rule
#: (``load > threshold``) could never have called a contact, however hard the joint
#: pressed. The "free range [21, 4073]" was not a measurement of a joint that turns
#: freely. It was a measurement of a joint driving into two real walls with the
#: software watching for a load no torque it has could produce. See
#: :attr:`~arm101.hardware.limits.LimitVerdict.UNFIRABLE_THRESHOLD`, which exists so
#: this class of mistake announces itself instead of being written down as fact.
#:
#: **What is actually true.** ``wrist_roll`` is BOUNDED: walls at raw 1700 and raw
#: 1491, travel 3887 ticks, and an unreachable arc of **209 ticks** between them. So
#: an arc does exist, and the old reason is void. The refusal nevertheless SURVIVES —
#: for an entirely different, and this time measured, reason: 209 ticks is narrower
#: than the :data:`~arm101.hardware.classify.MIN_EVICTABLE_ARC_TICKS` (300) a seam
#: needs, so there is nowhere safe to put it. Same outcome — a SOFT LIMIT
#: (:data:`SOFT_LIMITS`) — reached honestly.
#:
#: The two reasons are not interchangeable and the difference is not academic: the old
#: one said *no arc exists*, which foreclosed the question forever. The new one says
#: *the arc is 209 ticks and that is too few*, which is a number, and numbers can be
#: re-measured. Every other ineligible joint gets :data:`_REZERO_ARC_UNKNOWN` — a
#: third answer again ("nobody has measured your arc") that must not be confused with
#: either.
_REZERO_REFUSED: dict[str, str] = {
    "wrist_roll": (
        "wrist_roll cannot be re-zeroed: its unreachable arc is 209 ticks (raw 1491-1700, "
        "between walls measured at raw 1700 and raw 1491, travel 3887), and a seam needs 300 "
        "to sit in with a margin's clearance at each wall AND a margin of interior to be "
        "placed in. Re-zeroing into a 209-tick arc would park the seam in a window narrower "
        "than the ~12 ticks this arm's walls have been seen to shift by. wrist_roll is handled "
        "instead by a SOFT LIMIT (arm_spec.SOFT_LIMITS): a software-only travel restriction "
        "that carves out a dead arc the joint is never commanded into, and puts the seam in "
        "there. That is already in force. (NOTE: this joint was previously recorded as turning "
        "freely through its whole travel with no wall anywhere. That was false — an artifact of "
        "a 400 contact threshold on a joint whose walls press at only 272 and 288, so contact "
        "could not fire. It has real walls. See LimitVerdict.UNFIRABLE_THRESHOLD.)"
    ),
}

#: The other, ordinary reason a joint is not offered a re-zero: **nobody has measured
#: its unreachable arc**, so there is nothing to derive an offset from.
#:
#: **This message is a RETRACTION (issue #43), and the retraction is the point.** It
#: used to tell the operator, in their face, that "four of the six joints do not wrap
#: inside their travel at all, so there is no seam in the way … Only elbow_flex wraps
#: mid-travel". Hardware contradicted it: probed by feel, ``shoulder_lift``,
#: ``gripper`` and ``shoulder_pan`` each reached the commandable bound **with no
#: contact** — still physically free — 2, 3 and 11 raw ticks from the seam, and
#: ``shoulder_lift`` then sagged *through* the seam under gravity with its torque off.
#: At the factory offset the commandable bound (reported 4095 == raw 84) sits one tick
#: below the seam (raw 85), so the arm was reporting the *seam* as its boundary and
#: nothing could see past it.
#:
#: So the confident answer was withdrawn — and **not replaced with the opposite
#: confident answer**. We do not know that these joints wrap; we know we cannot see.
#: Until a measurement says otherwise, the arc is UNKNOWN, and an unknown arc is
#: exactly as un-re-zeroable as a nonexistent one — for a completely different reason,
#: and the reason is what the operator needs.
#:
#: "You don't need one" is still a real answer — it is just no longer this table's to
#: give. A *measurement* can earn it (a BOUNDED travel that misses the seam: see
#: :func:`arc_from_measurement` and ``classify.SeamRemedy.NONE_NEEDED``), and then
#: :func:`rezero_refusal` says so in the measurement's own words.
_REZERO_ARC_UNKNOWN = (
    "{joint} has no MEASURED unreachable arc, so no re-zero can be derived for it. An offset "
    "EVICTS the encoder seam into an arc the joint physically cannot reach; with no measurement "
    "of that arc there is no tick to put the seam at, and nothing here will invent one.\n\n"
    "UNKNOWN, NOT UNNECESSARY. This message used to say that {joint}'s encoder does not wrap "
    "inside its travel and that only elbow_flex does. That claim is WITHDRAWN (issue #43). "
    "Probed by feel, three joints reached their commandable bound with NO contact — still "
    "physically free — 2, 3 and 11 raw ticks from the seam, and shoulder_lift then sagged "
    "THROUGH the seam under gravity with its torque off. At the factory offset the bound lands "
    "one tick below the seam, so the seam is precisely what nothing could see past. Their travel "
    "may well include it. We do not know — and telling you otherwise was the bug.\n\n"
    "Measure the joint's travel end to end, classify it (arm101.hardware.classify), and the "
    "re-zero follows from the measurement: a BOUNDED joint whose travel wraps the seam gets an "
    "offset derived from its unreachable arc; one whose travel misses the seam genuinely needs "
    "nothing; one that turns all the way round can never be re-zeroed at all and takes a soft "
    "limit instead (wrist_roll — the one refusal here that is PROVEN)."
)


#: The retraction, in one line, for an **operator-facing surface** — a ``--help`` string,
#: a ``learn`` prompt, an ``explain`` page.
#:
#: **PUBLIC, and it exists because a retraction that lives only in the table it corrects
#: is not a retraction.** :data:`_REZERO_ARC_UNKNOWN` withdrew the "four joints do not
#: wrap" claim from the message ``rezero_refusal`` returns — and the same false claim went
#: on being told to the operator's face by ``learn``, by ``explain arm rezero``, and by
#: ``arm rezero --help``, because each of those had re-typed it. Three copies of a claim is
#: three things to correct, and the hardware run only corrected one.
#:
#: So the surfaces RENDER this, and the wording lives HERE, next to the table it describes.
#: Change the table, change this, and every surface moves with it — which is exactly the
#: discipline the ``_ARM_REZERO`` catalog entry already applies to :data:`REZERO_ARCS`.
#:
#: Note what it is careful NOT to say. It does not claim the four joints **do** wrap: we do
#: not know that either, and swapping one confident answer for its opposite would be the
#: same bug in different clothes. It says the arc is **unknown**, which is a completely
#: different answer from ``wrist_roll``'s — and ``wrist_roll``'s is the one that is PROVEN.
REZERO_UNKNOWN_HEADLINE: str = (
    "UNKNOWN, not unnecessary: elbow_flex is the only joint whose unreachable arc has been "
    "measured AND is wide enough to take the seam, so it is the only one a re-zero can be "
    "derived for. wrist_roll is refused because its measured arc — 209 ticks — is TOO NARROW "
    "to park a seam in, and it takes a soft limit instead. (Issue #43 withdrew the previous "
    "claim that wrist_roll turns freely all the way round with no wall anywhere: that was an "
    "artifact of a contact threshold set above any load the joint can produce. It has two "
    "real walls.) The other four are refused because nobody has measured their arc: #43 also "
    "withdrew the claim that their encoders do not wrap inside their travel."
)

#: ``wrist_roll``'s refusal, for the surfaces that explain which joints can be re-zeroed.
#: RENDERED from the table rather than restated, so the prose an operator reads cannot drift
#: away from the reason the code acts on — which is exactly how the withdrawn claim survived
#: in ``explain`` after the table itself had been corrected once already.
REZERO_NARROW_ARC_SUMMARY: str = _REZERO_REFUSED["wrist_roll"]

#: :data:`REZERO_UNKNOWN_HEADLINE`, plus the evidence that withdrew the claim and the way
#: back. Long-form, for the surfaces that have room for it (``learn``, ``explain``).
#:
#: Literally built from the headline, so the short and the long form cannot drift into two
#: different claims (``test_the_headline_is_the_summarys_own_first_words__one_source_not_two``).
REZERO_ARC_UNKNOWN_SUMMARY: str = REZERO_UNKNOWN_HEADLINE + (
    " Probed by feel, three of them reached their commandable bound with NO contact — still "
    "physically free — 2, 3 and 11 raw ticks from the seam, and shoulder_lift then sagged "
    "THROUGH the seam under gravity with its torque off. At the factory offset the commandable "
    "bound lands one tick below the seam, so the seam is precisely what nothing could see past. "
    "Measure a joint's travel end to end ('arm101-cli arm limits <joint>') and the answer "
    "follows from the measurement: a BOUNDED travel that wraps the seam gets an offset derived "
    "from its unreachable arc; one that misses the seam genuinely needs nothing; one that turns "
    "all the way round can never be re-zeroed at all."
)


# ---------------------------------------------------------------------------
# The derivation: a live measurement -> an arc -> the offset (ONE path, not two)
# ---------------------------------------------------------------------------


def arc_from_measurement(measurement: "TravelClassification") -> Optional[UnreachableArc]:
    """Return the RAW :class:`UnreachableArc` a *measurement* supports, or ``None``.

    **The capability that was missing.** ``REZERO_ARCS`` is a hand-typed table with one
    entry, measured over a long human session on the physical arm; nothing in this
    codebase could MEASURE an arc, so the procedure that fixed ``elbow_flex`` did not
    generalise — repeating it for another joint meant repeating that session. This is
    the bridge from what the arm can feel for itself to what a re-zero needs.

    Takes the measurement **as a parameter**: this module may not import the bus
    (``test_arm_spec_module_never_imports_the_bus``), and it does not need to. A
    :class:`~arm101.hardware.classify.TravelClassification` is pure data — the four
    verdicts of :mod:`arm101.hardware.limits` folded into BOUNDED / CONTINUOUS /
    UNDETERMINED — and it already carries the arc, inset by :data:`ARC_MARGIN_TICKS`
    (this module's own inset, imported there rather than re-typed) and refused outright
    if it cannot actually take the seam. So this function *reads* the arc rather than
    re-deriving it: a second derivation is a second thing to drift.

    ``None`` — a real answer, not a failure — for every classification that supports no
    arc, and :attr:`~arm101.hardware.classify.TravelClassification.reason` says which
    of the four it is: the joint turns all the way round (no arc exists, ever); its
    travel misses the seam (no arc is needed); its arc is too narrow to hold the seam
    clear of both walls (soft-limit territory); or the travel is UNDETERMINED and an
    arc sited on it would be an invention. :func:`rezero_refusal` hands that reason
    straight to the operator.

    The returned arc is held to the same standard as a table entry
    (:func:`_require_evictable_seam`) — the arc a servo's EEPROM is about to be written
    from must be one the seam can genuinely be evicted to, and where it came from does
    not change that.
    """
    arc = measurement.unreachable_arc
    if arc is None:
        return None
    _require_evictable_seam({measurement.joint: arc})
    return arc


def rezero_arc(
    joint: str, *, measured: Optional["TravelClassification"] = None
) -> Optional[UnreachableArc]:
    """Return *joint*'s :class:`UnreachableArc`, or ``None`` if it has none.

    Two sources, and the live one wins:

    * *measured* — a :class:`~arm101.hardware.classify.TravelClassification` taken off
      the arm just now. Its arc (:func:`arc_from_measurement`) is the answer, whatever
      :data:`REZERO_ARCS` says: a table that could not be overruled by a re-measurement
      is a table nobody can correct, which is the trap this one was in.
    * no *measured* — :data:`REZERO_ARCS`, the shipped default. Answerable on a laptop
      with no arm plugged in, which is why the table survives at all.

    ``None`` is the common answer (five of six joints from the table alone) and it means
    **"no arc is known here"** — not "no arc exists". :func:`rezero_refusal` says which
    kind of "no" it is: impossible (``wrist_roll``), unnecessary (a measurement showed
    the seam is outside the travel), or simply unmeasured (issue #43).

    Raises
    ------
    ValueError
        If *joint* is not one of :data:`JOINTS`, or if *measured* is a measurement of a
        **different** joint — applying one joint's walls to another is not a fallback,
        it is a bug that would write a wrong seam into EEPROM.
    """
    if joint not in JOINTS:
        raise ValueError(f"Unknown joint {joint!r}. Valid joints: {list(JOINTS)}.")
    if measured is None:
        return REZERO_ARCS.get(joint)
    if measured.joint != joint:
        raise ValueError(
            f"The measurement is of {measured.joint!r}, not {joint!r}. A joint's unreachable "
            "arc is a fact about ITS mechanical walls; using another joint's would derive an "
            "offset that parks the seam somewhere this joint can reach."
        )
    return arc_from_measurement(measured)


def rezero_offset(
    joint: str, *, measured: Optional["TravelClassification"] = None
) -> Optional[int]:
    """Return the signed encoder offset a re-zero would WRITE to *joint*, or ``None``.

    ``None`` means the joint is not re-zeroable — call :func:`rezero_refusal`
    for the reason, which is never "no reason".

    The offset is DERIVED from the arc, never typed: it is the signed form
    (:func:`_offset_for_seam_at`) of the arc's midpoint, i.e.
    :attr:`UnreachableArc.offset`. **One derivation, and the arc's provenance does not
    change it** — the table's arc and an arc measured on the arm five seconds ago both
    reach the offset the same way, so there is no second path to drift. Correcting the
    arc corrects the offset with it, which is exactly what happened on 2026-07-12: the
    first sweep of ``elbow_flex``'s far wall replaced a reported-frame guess with a
    raw-frame measurement, and this function's answer moved from 1073 to 1157 **without
    a line of it changing**. Nothing may hard-code an offset that a re-measurement would
    then have to chase.

    **This is a target, not a requirement.** A servo does not have to hold *this*
    number to be correctly re-zeroed; it has to hold *an offset that evicts the
    seam* (:meth:`UnreachableArc.evicts` — any tick strictly inside the arc). Our
    own follower holds ``1073`` and is fine. Code that asks "is this joint
    re-zeroed?" must ask ``arc.evicts(current)``, never ``current ==
    rezero_offset(joint)``: the second question is a different, stricter, and
    physically meaningless one, and answering it instead would rewrite a working
    calibration to move a seam from one unreachable tick to another.

    Raises
    ------
    ValueError
        If *joint* is not one of :data:`JOINTS`, or *measured* is of another joint.
    """
    arc = rezero_arc(joint, measured=measured)  # validates *joint* and *measured*
    if arc is None:
        return None
    return arc.offset


def rezero_refusal(
    joint: str, *, measured: Optional["TravelClassification"] = None
) -> Optional[str]:
    """Return WHY *joint* cannot be re-zeroed, or ``None`` if it can be.

    A refusal is an explanation, not a shrug. Three structurally different answers hide
    behind "no", and the distinction between them is the thing an operator actually
    needs:

    * ``wrist_roll`` — **measured, and refused on the number** (:data:`_REZERO_REFUSED`).
      Its unreachable arc exists and is 209 ticks: too narrow to hold the seam, so a soft
      limit handles it instead. (This entry used to claim a permanent *impossibility* —
      "it turns freely all the way round". Issue #43 withdrew that too: the joint has real
      walls, and the reading that hid them came from a threshold it could never reach.)
    * with a *measured* travel — the measurement's **own words**
      (:attr:`~arm101.hardware.classify.TravelClassification.reason`). That is where
      "you don't need a re-zero" now comes from: a BOUNDED travel that misses the seam
      has *earned* that answer. So has "your arc is too narrow to hold the seam" and
      "this travel is UNDETERMINED — measure again, do not pick".
    * with no measurement — **unknown** (:data:`_REZERO_ARC_UNKNOWN`). Nobody has
      measured this joint's arc, so no offset can be derived. This used to claim the
      joint's encoder does not wrap at all; issue #43 withdrew that.

    Collapsing those into one message would teach the operator the wrong thing about
    their arm — it would make a permanent, provable impossibility read like an
    unimplemented feature, and (until #43) it dressed an unmeasured joint up as a
    healthy one.

    Raises
    ------
    ValueError
        If *joint* is not one of :data:`JOINTS`, or *measured* is of another joint.
    """
    if rezero_arc(joint, measured=measured) is not None:  # validates *joint*/*measured*
        return None
    if measured is not None:
        # The measurement refused. It knows why — better than any table does.
        return measured.reason
    refused = _REZERO_REFUSED.get(joint)
    if refused is not None:
        return refused
    return _REZERO_ARC_UNKNOWN.format(joint=joint)


def permitted_reported_range(
    joint: str, offset: int, *, limits: Optional[Mapping[str, SoftLimit]] = None
) -> Optional[tuple[int, int]]:
    """*joint*'s RAW soft limit, rendered in the REPORTED frame of a servo holding *offset*.

    The bus-edge conversion, for the one table that has to make the crossing. The
    soft limit is stored RAW because a raw tick is a physical angle
    (:class:`SoftLimit`); a *goal* is written in the servo's own reported frame; so
    exactly one conversion stands between them, and this is it. Recomputed from the
    live offset every time rather than cached, because the moment a reported tick is
    stored it stops being true.

    ``None`` means the joint has no soft limit — the same, common answer
    :func:`soft_limit` gives, and it means "the full range is permitted", not
    "unknown".

    Raises
    ------
    ValueError
        If *joint* is unknown, or if the limit's dead arc does NOT contain the
        reported seam of a servo holding *offset*
        (:func:`dead_arc_contains_reported_seam`). In that case the permitted range
        *wraps* in the reported frame and has no ``(min, max)`` representation there
        at all — the same shape as ``elbow_flex``'s ``[min, max]`` naming the arc it
        cannot reach. It means the servo's offset and this table contradict each
        other, and the answer is to fix one of them, not to invent a pair. In
        practice it cannot fire for a correctly configured arm:
        :func:`_require_seam_clearance` proves it at import time for the factory
        offset, and a soft-limited joint is by definition one that never gets
        re-zeroed off it.

    *limits* selects the table to read — see :func:`soft_limit`. A **measured** soft
    limit (:func:`resolve_soft_limits`) crosses into the reported frame right here, by
    exactly the same conversion as a shipped one: there is one bridge, and this is it.
    """
    limit = soft_limit(joint, limits=limits)  # validates *joint*
    if limit is None:
        return None
    return raw_interval_to_reported(limit.min_tick, limit.max_tick, offset)


def resolve_bounds(
    joint: str,
    eeprom_min: int,
    eeprom_max: int,
    offset: int,
    *,
    limits: Optional[Mapping[str, SoftLimit]] = None,
) -> tuple[int, int]:
    """Return the travel bounds a move may actually use for *joint*, in REPORTED ticks.

    **Frames, named — because this function is where they meet.** Every tick that
    goes in and comes out is REPORTED, because every one of them is a number a
    servo speaks: ``eeprom_min``/``eeprom_max`` are the ``min_angle``/``max_angle``
    registers, which the servo compares against a goal in its own corrected frame,
    and the returned pair is handed straight to ``clamp_goal`` /
    ``gentle_move`` / ``GridSpec.bounds``, which compare it against
    ``read_position``. The soft limit is the one input in the *other* frame — it is
    stored RAW, because it is a claim about a physical angle — and *offset* is what
    lets this function put them on the same axis
    (:func:`permitted_reported_range`). Pass the servo's live offset, from
    ``read_info(motor)["homing_offset"]``; do not default it, do not assume 0. No
    servo ships holding 0, and assuming it is how the last two frame bugs got in.

    This is the function that makes :data:`SOFT_LIMITS` **bind**. Without it the
    table is inert: every move in this codebase sources its bounds from the
    servo's EEPROM ``min_angle``/``max_angle`` registers, and on this arm those
    are the untouched factory ``0-4095`` on all six joints
    (``docs/hardware-validation-arm-explore.md`` — the EEPROM knows nothing
    about the arm's real travel). So ``arm flex wrist_roll --to 4090`` would
    have been perfectly happy to drive the joint into the arc the soft limit
    exists to exclude, straight across the encoder seam, reproducing the exact
    hang documented in this module's docstring. A soft limit nobody reads
    protects nothing. Every call site that turns a ``read_info`` result into
    move bounds — ``arm flex``, ``arm explore``'s grid, the demo sweep — routes
    through here.

    **Intersection, not replacement.** The soft limit says "never go outside
    ``(min_tick, max_tick)``". It does NOT say "always permit
    ``(min_tick, max_tick)``". If a servo's EEPROM limits are genuinely
    narrower than the soft limit — an operator's calibration, a fixture, a
    cable-routing constraint — those are a real physical constraint that a
    software table has no business widening, and replacing them would drive the
    joint somewhere the servo was explicitly configured not to go. So each end
    independently takes the **tighter** of the two: the higher low bound, the
    lower high bound. A joint with no soft limit (five of six — see
    :func:`soft_limit`) gets its EEPROM bounds back verbatim; this function must never
    quietly narrow a joint this table says nothing about.

    **Read-side only.** Nothing here — and nothing downstream of here — writes
    the resolved range back to the servo. That is the standing spec boundary
    for this whole line of work: measured and derived ranges are pose- and
    environment-dependent, so they live in this module and in the reachability
    map, never burnt into EEPROM where they would outlive the pose that
    produced them. This module cannot violate that even by accident: it imports
    no bus (pinned by ``test_arm_spec_module_never_imports_the_bus``).

    Parameters
    ----------
    joint:
        One of the six joint names in :data:`JOINTS`.
    eeprom_min, eeprom_max:
        The joint's angle limits as read from the servo, i.e.
        ``bus.read_info(motor)["min_angle"]`` / ``["max_angle"]``. REPORTED ticks:
        the servo compares them against a goal, which is in its corrected frame.
    offset:
        The servo's live signed homing offset, i.e.
        ``bus.read_info(motor)["homing_offset"]``. The one thing that relates the
        RAW soft limit to the REPORTED bounds above.
    limits:
        The resolved soft-limit table (:func:`resolve_soft_limits`) — the shipped
        :data:`SOFT_LIMITS` merged with anything ``arm limits --commit`` measured.
        Defaults to the shipped table. **This is where a measured soft limit BINDS**:
        it is the one function every mover's bounds pass through, so a limit that
        reaches here reaches ``arm flex``, ``arm explore``'s grid and the demo sweep,
        and one that does not reaches nothing at all.

    Returns
    -------
    tuple[int, int]
        ``(min_tick, max_tick)`` in REPORTED ticks — the bounds to hand to
        ``clamp_goal`` / ``compliant_move`` / ``gentle_move`` / ``GridSpec.bounds``.

    Raises
    ------
    ValueError
        If *joint* is unknown; if the soft limit's dead arc does not contain this
        servo's reported seam (see :func:`permitted_reported_range`); or if the
        intersection is EMPTY — i.e. the
        servo's configured range lies entirely inside the soft limit's dead
        arc, so the servo says "only ever go here" about precisely the arc the
        soft limit says "never go here" about. No pair of bounds honours both
        constraints, and returning the inverted pair would surface downstream
        as :func:`arm101.hardware.motion.clamp_goal`'s misleading "min/max were
        swapped" error, several frames from the real cause. This module stays
        free of CLI concerns — callers at the CLI/hardware layer translate this
        into a :class:`~arm101.cli._errors.CliError`. Note that an *inverted*
        EEPROM pair (``eeprom_min > eeprom_max``) is deliberately NOT caught
        here: intersection preserves inversion, and ``clamp_goal`` already owns
        that error with a message that names the real problem.
    """
    permitted = permitted_reported_range(joint, offset, limits=limits)  # validates *joint*
    if permitted is None:
        return (eeprom_min, eeprom_max)
    limit_min, limit_max = permitted

    low = max(eeprom_min, limit_min)
    high = min(eeprom_max, limit_max)
    # Only an EEPROM range that is itself well-ordered can produce an empty
    # intersection here; an inverted EEPROM pair stays inverted (low > high on
    # both sides of the max/min) and is clamp_goal's error to report, not ours.
    if eeprom_min <= eeprom_max and low > high:
        raise ValueError(
            f"Joint {joint!r} has no permitted travel: its servo angle limits "
            f"({eeprom_min}, {eeprom_max}) lie entirely inside the soft limit's dead arc "
            f"(permitted range is ({limit_min}, {limit_max}) in this servo's reported frame, "
            f"at offset {offset}). Widen the servo's angle limits or retune the soft limit — "
            "they currently contradict each other."
        )
    return (low, high)


def soft_limit(
    joint: str, *, limits: Optional[Mapping[str, SoftLimit]] = None
) -> Optional[SoftLimit]:
    """Return *joint*'s **RAW** :class:`SoftLimit`, or ``None`` if it has no travel restriction.

    RAW ticks — a claim about physical angles, unchanged by any re-zero. To get the
    range in the frame a servo is *commanded* in, go through
    :func:`permitted_reported_range` (or :func:`resolve_bounds`, which also
    intersects the servo's own EEPROM limits); never compare this against a live
    position read without converting one of them.

    ``None`` is the common answer — five of the six joints have no entry here — and it
    means "**this table restricts nothing for this joint**", i.e. the full ``[TICK_MIN,
    TICK_MAX]`` range is permitted. It does **not** mean the joint is known to have no
    wrap problem: ``elbow_flex`` has one and takes the re-zero path instead, and the
    other four have simply never had their travel measured (issue #43 — their
    commandable bound sat on the seam, so nothing could see past it). A joint's seam is
    settled by measuring its travel, never by its absence from a table.

    Parameters
    ----------
    joint:
        One of the six joint names in :data:`JOINTS`.
    limits:
        The resolved soft-limit table to read, from :func:`resolve_soft_limits` — the
        shipped :data:`SOFT_LIMITS` **merged with whatever a measurement committed**.
        Defaults to the shipped table alone, which is the right answer for a caller with
        no run behind it (a ``--help`` string, a unit test, a laptop with no arm).

        Threaded as a parameter rather than read from a module global because this
        module must not do file I/O — ``tests/test_scope_guard.py`` pins that: no file
        handles, no home directory, no config reader. So the store lives in
        :mod:`arm101.hardware.soft_limit_store`, and the table it produces arrives here
        as plain data.

    Raises
    ------
    ValueError
        If *joint* is not one of :data:`JOINTS`.
    """
    if joint not in JOINTS:
        raise ValueError(f"Unknown joint {joint!r}. Valid joints: {list(JOINTS)}.")
    table = SOFT_LIMITS if limits is None else limits
    return table.get(joint)


def soft_limit_for_offset(offset: int, *, clearance: int = SEAM_CLEARANCE_TICKS) -> SoftLimit:
    """Derive the RAW :class:`SoftLimit` that fences off the seams of a servo holding *offset*.

    **The other instrument.** A joint whose travel covers the whole circle
    (:attr:`~arm101.hardware.classify.TravelKind.CONTINUOUS`) has no unreachable arc, so
    no offset can evict its seam — a re-zero cannot help it *in principle*. Neither can
    one whose arc is too narrow to hold the seam clear of both walls. What is left is to
    stop **commanding** the joint across the discontinuity: a software-only dead arc,
    containing the seam, that no mover is allowed to drive into.

    This is the derivation ``SOFT_LIMITS["wrist_roll"]`` was written by hand, and it is
    now a function so that a *measured* soft limit reaches the same answer by the same
    route. ``test_the_shipped_wrist_roll_limit_is_what_this_function_derives`` pins the
    two together, which is the only way the shipped entry and a measured one cannot
    drift apart.

    Two seams, and both must land in the dead arc
    ---------------------------------------------
    * the **RAW** seam — the magnet's own 4095->0 rollover, which no offset moves. A
      :class:`SoftLimit` is a plain interval, so its dead arc always runs *through* raw
      0 and therefore always contains this one (:func:`dead_arc_contains_seam`).
    * the **REPORTED** seam — at raw ``Ofs`` (:func:`seam_tick`), the one a *goal write*
      actually crosses and the one the shipped table originally missed by being written
      in the wrong frame. This is what *offset* is for
      (:func:`dead_arc_contains_reported_seam`).

    Both are cleared by *clearance* ticks (:data:`SEAM_CLEARANCE_TICKS` by default:
    ~8x the worst case of encoder jitter plus ``gentle_move``'s 12-tick arrival
    tolerance, so an arrival check settling near an edge cannot land on a seam).

    The cost is the offset's, not the joint's
    -----------------------------------------
    The dead arc must be contiguous through raw 0, so the permitted range is one of two
    intervals: the arc *above* the reported seam, or the arc *below* it. The wider is
    taken — so a servo whose reported seam sits near raw 0 (the factory ``Ofs = 85``:
    every SO-101 ships this way) pays ~285 ticks of a 4096-tick circle, while one
    holding an offset that puts its seam near mid-scale would pay ~half the circle. That
    is not a flaw in the derivation; it is the geometry, and it is why an operator should
    see :attr:`SoftLimit.dead_arc_ticks` before accepting the limit.

    Raises
    ------
    ValueError
        If no interval clears both seams by *clearance* — i.e. *clearance* is so wide
        that nothing is left to permit. There is no soft limit for such a joint, and
        inventing a narrower clearance to manufacture one would be putting the seam back
        within reach of an arrival check.
    """
    seam = seam_tick(offset)
    clearance = int(clearance)
    if clearance < 1:
        raise ValueError(
            f"A seam clearance of {clearance} ticks fences off nothing: the permitted range "
            "would touch the seam it exists to exclude."
        )

    # The dead arc runs through raw 0 by construction (SoftLimit is a plain interval), so
    # the two candidates are "permit everything above the reported seam" and "permit
    # everything below it". Widest first — and every candidate is CHECKED, not trusted.
    candidates = (
        (seam + clearance, TICK_MAX - clearance),
        (TICK_MIN + clearance, seam - clearance),
    )
    best: Optional[SoftLimit] = None
    for min_tick, max_tick in candidates:
        if not (TICK_MIN <= min_tick < max_tick <= TICK_MAX):
            continue
        limit = SoftLimit(min_tick=min_tick, max_tick=max_tick)
        if not dead_arc_contains_seam(min_tick, max_tick):
            continue  # cannot happen for a strict sub-interval; asked anyway, never assumed
        if not dead_arc_contains_reported_seam(limit, offset):
            continue
        if (
            limit.clearance_from(RAW_SEAM_TICK) < clearance
            or limit.clearance_from(seam) < clearance
        ):
            continue
        if best is None or limit.max_tick - limit.min_tick > best.max_tick - best.min_tick:
            best = limit

    if best is None:
        raise ValueError(
            f"No soft limit can be derived for a servo holding offset {offset} (its reported "
            f"seam is at raw tick {seam}) with {clearance} ticks of clearance: nothing would be "
            "left to permit. The dead arc must contain BOTH the raw seam (raw "
            f"{RAW_SEAM_TICK}) and the reported seam (raw {seam}), each with clearance to spare."
        )
    return best


def resolve_soft_limits(
    *, from_file: Optional[Mapping[str, SoftLimit]] = None
) -> dict[str, SoftLimit]:
    """Resolve the soft-limit table in force: the shipped defaults, plus measured overrides.

    Precedence per joint: ``from_file`` > :data:`SOFT_LIMITS`. The same shape as
    :func:`resolve_contact_thresholds` — an ``arm_spec`` default table plus an optional
    file override — and for the same reason: a CLI does not rewrite its own source, so a
    value the arm *measured* needs somewhere to live that the runtime will actually
    read. :mod:`arm101.hardware.soft_limit_store` is that somewhere; this is what merges
    it in, and :func:`resolve_bounds` is where it BINDS.

    **A soft limit that nobody consults is not a soft limit.** The shipped
    ``wrist_roll`` entry was inert data for a whole release — every mover sourced its
    bounds straight from the servo's EEPROM registers, which on this arm hold the
    factory 0-4095 — until :func:`resolve_bounds` was routed through by every call site.
    An override that only bound when someone remembered to pass a flag would be exactly
    as inert, which is why the store has a default location and is loaded whether or not
    it is asked for.

    Validated, not merely merged: the result must still be a table that describes ONE
    arm. An override naming an unknown joint, a range whose dead arc excludes nothing
    (:func:`_require_dead_arc_contains_seam`), or a joint that also carries a re-zero arc
    (:func:`_require_no_soft_limit_and_arc` — the two mutually exclusive answers to a
    wrapping joint) is refused here, where a human can still act on it, rather than
    surfacing three frames away as a clamp that silently fences off the wrong arc.

    Raises
    ------
    ValueError
        If *from_file* names an unknown joint, or the merged table fails an invariant.
        This module stays free of CLI concerns — callers at the CLI layer translate it
        into a :class:`~arm101.cli._errors.CliError`.
    """
    overrides = dict(from_file or {})
    for name in overrides:
        if name not in JOINTS:
            raise ValueError(f"Unknown joint {name!r}. Valid joints: {list(JOINTS)}.")

    merged = {**SOFT_LIMITS, **overrides}
    _require_dead_arc_contains_seam(merged)
    _require_no_soft_limit_and_arc(merged)
    return merged


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------


def _require_role(role: str) -> None:
    """Raise :class:`ValueError` if *role* is not a known arm role."""
    if role not in _KNOWN_ROLES:
        raise ValueError(f"Unknown arm role {role!r}. Valid roles: {sorted(_KNOWN_ROLES)}.")


# ---------------------------------------------------------------------------
# Import-time invariants that span BOTH tick tables
# ---------------------------------------------------------------------------
#
# Deferred to the foot of the module only because it reads SOFT_LIMITS *and*
# REZERO_ARCS, and one is defined after the other. Everything it enforces is
# described on the function itself; what matters here is that it runs on import,
# for every caller and every test, so a soft limit written in the reported frame
# (which is how this table shipped) cannot survive `import arm101`.
_require_seam_clearance(SOFT_LIMITS)
