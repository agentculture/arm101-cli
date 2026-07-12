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
    # MEASURED on the follower (/dev/ttyACM1) on 2026-07-12, through the fixed
    # load-during-travel sampling. Each value sits inside that joint's usable
    # band — above the peak load it develops merely ACCELERATING through open
    # space (below), and below the 500 ceiling where present_load saturates at
    # gentle_move's Torque_Limit cap (a threshold >= 500 can never fire).
    #
    #   joint          free-motion peak   band          margin
    #   shoulder_pan    88                (88,  500)    +162
    #   shoulder_lift   92                (92,  500)    +158
    #   elbow_flex     148                (148, 500)    +132
    #   wrist_flex      96                (96,  500)    +154
    #   wrist_roll     300                (300, 500)    +100   <- worst joint
    #   gripper         76                (76,  500)    +174
    #
    # The previous values were tuned against the pre-fix code's near-zero load
    # reads and were wrong: wrist_roll's 180 sat BELOW its own 300 free-motion
    # peak, so it would have called contact on every move it made.
    "shoulder_pan": 250,
    "shoulder_lift": 250,
    "elbow_flex": 280,
    "wrist_flex": 250,
    "wrist_roll": 400,
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
# Encoder wrap — software-only soft limits
# ---------------------------------------------------------------------------

#: Encoder tick bounds shared by every SO-101 joint (STS3215, 12-bit encoder).
#: Every raw ``present_position``/``goal_position`` read or write is in this
#: range; the **seam** — the point that motivates this whole section — is the
#: wrap between :data:`TICK_MAX` and :data:`TICK_MIN`: driving a joint past
#: 4095 rolls it over to read back near 0, and driving it below 0 rolls it
#: over to read back near 4095.
TICK_MIN: int = 0
TICK_MAX: int = 4095


@dataclass(frozen=True)
class SoftLimit:
    """A SOFTWARE-only travel restriction, expressed as a plain tick interval.

    This is deliberately the *only* shape a soft limit is ever expressed in:
    a ``min_tick < max_tick`` interval within ``[TICK_MIN, TICK_MAX]`` — never
    a "min greater than max means wrap around" encoding. That second, more
    "clever" representation is exactly what this module exists to avoid: it
    would let a range *cross* the seam by construction, silently reintroducing
    the bug documented in the module docstring. Because every ``SoftLimit`` is
    a plain interval, the excluded region — the **dead arc** — is always
    ``[TICK_MIN, min_tick) ∪ (max_tick, TICK_MAX]``: the arc that runs
    *through* the seam. See :func:`dead_arc_contains_seam`, which checks that
    this dead arc is non-empty rather than assuming it.

    Attributes
    ----------
    min_tick, max_tick : int
        The permitted (non-dead) range, inclusive on both ends.

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
        """``True`` iff *tick* falls inside the permitted range, inclusive."""
        return self.min_tick <= tick <= self.max_tick


def dead_arc_contains_seam(min_tick: int, max_tick: int) -> bool:
    """Return ``True`` iff excluding ``[min_tick, max_tick]`` leaves a dead arc containing the seam.

    This is the enforceable version of the argument in the module docstring,
    made concrete rather than left as a comment someone can outrun with an
    edit. For any well-ordered interval ``TICK_MIN <= min_tick < max_tick <=
    TICK_MAX``, the excluded region is exactly
    ``[TICK_MIN, min_tick) ∪ (max_tick, TICK_MAX]`` — the arc that runs
    through the ``TICK_MAX -> TICK_MIN`` wrap, i.e. through the seam. That
    region is non-empty, and therefore genuinely contains the seam, iff
    ``min_tick > TICK_MIN or max_tick < TICK_MAX``. The one case where it is
    EMPTY is ``(min_tick, max_tick) == (TICK_MIN, TICK_MAX)``: the full turn,
    nothing excluded, seam still fully reachable. That degenerate case is
    exactly "a soft limit that still spans the seam" — it buys nothing, and
    this function is what makes that failure mode fail a test instead of
    quietly reintroducing the bug the next time someone "simplifies" a range.

    Parameters
    ----------
    min_tick, max_tick : int
        A candidate permitted range, in the plain-interval form
        :class:`SoftLimit` always uses.

    Returns
    -------
    bool
        ``True`` if the dead arc is non-empty (contains the seam), ``False``
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


#: Per-joint software travel restrictions for joints whose encoder wraps
#: within (or across the whole of) their physical travel. A joint absent from
#: this table has no soft limit — its full ``[TICK_MIN, TICK_MAX]`` range is
#: permitted, which is correct for every joint except the two wrapping ones
#: (see the module docstring). ``elbow_flex`` is the other wrapping joint, but
#: it takes the encoder RE-ZERO path instead (a different task/module): it
#: has real mechanical walls, so its seam can be relocated into an arc it
#: physically cannot reach, and it therefore needs no entry here.
#:
#: ``wrist_roll`` — the only entry — has none: exploration drove it across
#: its entire ``[21, 4073]``-tick measured free range with **no wall found
#: anywhere** (``docs/hardware-validation-arm-read-flex.md``), i.e. it
#: rotates freely all the way round, so re-zero cannot fix it even in
#: principle (see the module docstring's impossibility argument). The chosen
#: range, ``(100, 3995)``:
#:
#: * Splits a 200-tick dead arc evenly across the seam — 100 ticks
#:   (``[0, 100)``) below :data:`TICK_MIN`'s side, 100 ticks
#:   (``(3995, 4095]``) below :data:`TICK_MAX`'s side. 100 ticks is
#:   comfortably larger than both encoder read jitter (a handful of ticks)
#:   and ``gentle_move``'s default arrival tolerance of 12 ticks
#:   (``arm101.hardware.gentle._DEFAULT_ARRIVAL_TOLERANCE``), so an arrival
#:   check settling near either edge cannot land in the dead arc by accident.
#: * Lands with margin *inside* the empirically-confirmed free envelope
#:   ``[21, 4073]`` — 79 ticks of margin above 21, 78 below 4073 — so the
#:   permitted range never asks the servo to go anywhere exploration did not
#:   already drive it without incident.
#: * Leaves 3895 of 4095 ticks (~95%) usable — the exclusion is real (this is
#:   not "spans the seam" in disguise: see :func:`dead_arc_contains_seam`) but
#:   deliberately small, since nothing measured suggests the joint needs more
#:   room than that.
#:
#: The exact width is a tunable an operator may revisit with more hardware
#: data (e.g. if arrival tolerance or observed jitter ever changes); what must
#: never change is that the dead arc contains the seam — enforced immediately
#: below, not merely asserted here.
SOFT_LIMITS: dict[str, SoftLimit] = {
    "wrist_roll": SoftLimit(min_tick=100, max_tick=3995),
}

_require_dead_arc_contains_seam(SOFT_LIMITS)


# ---------------------------------------------------------------------------
# Encoder re-zero — EVICTING the seam from a joint's travel (issue #35)
# ---------------------------------------------------------------------------

#: One full turn of the STS3215's 12-bit magnetic encoder, in ticks (4096).
#: Derived from :data:`TICK_MIN`/:data:`TICK_MAX` rather than typed, so the two
#: cannot drift apart. It is the modulus the corrected position is reduced by —
#: the arithmetic that makes an offset *relocate* the seam instead of merely
#: relabelling positions.
#:
#: Deliberately re-stated here rather than imported from
#: :mod:`arm101.hardware.bus` (``ENCODER_RESOLUTION``): this module imports no
#: bus, by design and by test (``test_arm_spec_module_never_imports_the_bus``),
#: because a table of physical facts must not depend on a serial port. The two
#: constants are pinned equal by a cross-module test instead.
ENCODER_TICKS: int = TICK_MAX - TICK_MIN + 1

#: Widest magnitude the servo's offset register (``Ofs``/``Homing_Offset``,
#: EEPROM addr 31) can hold: it is SIGN-MAGNITUDE on bit 11, so the magnitude
#: field is 11 bits and the representable range is ``[-2047, +2047]``.
#: (LeRobot ``encode_sign_magnitude``: ``max_magnitude = (1 << 11) - 1``;
#: confirmed the hard way on a real SO-101 — LeRobot issue #3193 raised
#: ``ValueError: Magnitude 2073 exceeds 2047``.)
#:
#: Modulo 4096 that covers **every** seam placement except exactly one: raw
#: 2048. (``-2047`` is congruent to ``2049``, so residues ``0..2047`` and
#: ``2049..4095`` are all reachable; neither ``+2048`` nor ``-2048`` is
#: representable.) See :func:`_offset_for_seam_at`.
#:
#: Mirrors :data:`arm101.hardware.bus.OFFSET_MAX_MAGNITUDE` — same reason as
#: :data:`ENCODER_TICKS`: same fact, stated in the module that must not import
#: the other, and pinned equal by a cross-module test.
MAX_ENCODER_OFFSET: int = 2047

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
#: ``Present = Actual - Ofs``, exactly as :func:`seam_tick` assumes.
#:
#: Two consequences, and both are load-bearing:
#:
#: * **A "factory" servo's reported positions are NOT raw ticks.** They are
#:   already shifted by −85. Anything that treats a reported tick as a raw tick
#:   is wrong by 85 on a brand-new arm — which is the default state of every
#:   SO-101 (see the ``REZERO_ARCS`` note below; the arc this table shipped with
#:   was measured in exactly that shifted frame and then used as if it were raw).
#: * **The factory seam is at raw 85, which is INSIDE ``elbow_flex``'s travel.**
#:   Its travel includes the raw band ``[0, 207]``, so a factory-fresh
#:   ``elbow_flex`` carries its encoder discontinuity right in the middle of
#:   where it works. That is issue #35, and 85 is where it actually lives.
#:
#: Kept here rather than in ``rezero.py`` because it is a *fact about the
#: hardware*, in the module that holds facts about the hardware. Nothing in the
#: code branches on it — :func:`rezero_arc` and :func:`plan_rezero` read the
#: servo's live offset and convert, rather than assuming any particular starting
#: value, and that is the whole point of the fix. It is documented so that the
#: number in front of an operator ("offset: 85") has a name.
FACTORY_ENCODER_OFFSET: int = 85


def seam_tick(offset: int) -> int:
    """The RAW encoder tick at which a servo holding *offset* carries its seam.

    The inverse of :func:`_offset_for_seam_at`, and the single piece of
    arithmetic the whole re-zero turns on. With
    ``Present = (Actual - Ofs) mod 4096`` the reported value rolls ``4095 -> 0``
    exactly where ``Actual == Ofs``, so the seam's raw tick simply **is** the
    offset — reduced modulo 4096, because the register is SIGNED (it is
    sign-magnitude on bit 11, range ``[-2047, +2047]``) while the encoder is
    not.

    That reduction is not a formality. A servo holding ``-1096`` carries its seam
    at raw **3000**, not at "-1096": those are the same residue and only one of
    them is a tick. Comparing the signed number straight against a raw arc would
    place the seam a whole turn away from where it physically is, and every
    "is the seam evicted?" answer downstream would be wrong.
    """
    return offset % ENCODER_TICKS


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
    :func:`arm101.hardware.rezero.raw_from_reported`) before it is compared
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
        convert first (:func:`arm101.hardware.rezero.raw_from_reported`).
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


def _offset_for_seam_at(tick: int) -> int:
    """Return the SIGNED offset ``H`` that places the encoder seam at raw *tick*.

    With ``Present = (raw - H) mod 4096``, the reported value rolls 4095->0
    exactly where ``raw == H``, so the offset simply *is* the seam's raw tick —
    but expressed in the signed form the register can hold. Ticks above
    :data:`MAX_ENCODER_OFFSET` are unrepresentable as positive magnitudes and
    are re-expressed as their negative congruent (``tick - 4096``): raw 3000
    becomes ``H = -1096``, which is the same residue and fits comfortably.

    Raw 2048 is the single seam placement the encoding cannot express at all
    (``+2048`` overflows the 11-bit magnitude and ``-2048`` does too). It is not
    silently rounded — :func:`_require_evictable_seam` rejects a table entry
    whose midpoint lands there, loudly, at import time.
    """
    return tick if tick <= MAX_ENCODER_OFFSET else tick - ENCODER_TICKS


#: Per-joint unreachable arcs — the joints an encoder re-zero can actually fix.
#: **RAW ticks** (:class:`UnreachableArc`), which is the correction this table
#: carries: the numbers it shipped with were not.
#:
#: ``elbow_flex`` is the only entry, and the only joint that needs one. Its
#: encoder WRAPS inside its physical travel (issue #35): driven far enough it
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
REZERO_ARCS: dict[str, UnreachableArc] = {
    # RAW ticks. Hardware sweep, follower, 2026-07-12: travel 1034..3230 reported
    # at Ofs=1073 (span 2196, monotonic, 0 discontinuities) -> raw [2107, 4095] ∪
    # [0, 207] -> unreachable (207, 2107). The far wall, measured for the first
    # time. The previous (126, 2020) were REPORTED-frame ticks read at the factory
    # Ofs=85 and used as if they were raw.
    "elbow_flex": UnreachableArc(low=207, high=2107),
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


#: Why a joint that CANNOT be re-zeroed cannot be re-zeroed — keyed by joint.
#:
#: Only ``wrist_roll`` is here, and its reason is a genuine impossibility rather
#: than an omission, which is exactly why it is spelled out instead of being
#: left to a shrug. Every other ineligible joint gets
#: :data:`_REZERO_UNNECESSARY` instead: a completely different answer ("you
#: don't need one") that must never be confused with this one ("you can't have
#: one").
_REZERO_IMPOSSIBLE: dict[str, str] = {
    "wrist_roll": (
        "wrist_roll cannot be re-zeroed: a re-zero only RELOCATES the encoder seam, it "
        "can never EVICT it. Eviction needs an arc the joint physically cannot reach, and "
        "exploration found no wall anywhere in wrist_roll's travel (measured free range "
        "[21, 4073]) — it turns freely all the way round, so every angle is reachable, "
        "including whichever one the seam is moved to. Its unreachable arc is empty by "
        "definition. wrist_roll is handled instead by a SOFT LIMIT (arm_spec.SOFT_LIMITS): "
        "a software-only travel restriction that carves out a dead arc the joint is simply "
        "never commanded into, and puts the seam in there. That is already in force."
    ),
}

#: The other, ordinary reason a joint is not offered a re-zero: it never needed
#: one. Four of the six joints do not wrap inside their travel at all, so there
#: is no seam in the way and nothing to evict.
_REZERO_UNNECESSARY = (
    "{joint} does not need a re-zero: its encoder does not wrap inside its travel, so its "
    "reported position is already monotonic with joint angle and there is no seam to evict. "
    "Only elbow_flex wraps mid-travel (issue #35). Re-zeroing a joint that does not need it "
    "would shift its frame of reference for no benefit and invalidate every position "
    "previously recorded for it."
)


def rezero_arc(joint: str) -> Optional[UnreachableArc]:
    """Return *joint*'s :class:`UnreachableArc`, or ``None`` if it has none.

    ``None`` is the common answer (five of six joints) and means "this joint has
    no measured unreachable arc **in this table**" — either because it does not
    wrap and needs no re-zero, or because it wraps but cannot be re-zeroed at
    all. :func:`rezero_refusal` is what tells those two apart.

    Raises
    ------
    ValueError
        If *joint* is not one of :data:`JOINTS`.
    """
    if joint not in JOINTS:
        raise ValueError(f"Unknown joint {joint!r}. Valid joints: {list(JOINTS)}.")
    return REZERO_ARCS.get(joint)


def rezero_offset(joint: str) -> Optional[int]:
    """Return the signed encoder offset a re-zero would WRITE to *joint*, or ``None``.

    ``None`` means the joint is not re-zeroable — call :func:`rezero_refusal`
    for the reason, which is never "no reason".

    The offset is DERIVED from :data:`REZERO_ARCS`, never typed: it is the
    signed form (:func:`_offset_for_seam_at`) of the arc's midpoint. So
    correcting the arc corrects the offset with it and the two cannot drift
    apart — which is exactly what happened on 2026-07-12, when the first sweep of
    ``elbow_flex``'s far wall replaced a reported-frame guess with a raw-frame
    measurement, and this function's answer moved from 1073 to 1157 without a
    line of it changing. For ``elbow_flex`` today: arc ``(207, 2107)``, midpoint
    1157, offset **+1157** — 950 ticks of clearance on each side.

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
        If *joint* is not one of :data:`JOINTS`.
    """
    arc = rezero_arc(joint)  # validates *joint*
    if arc is None:
        return None
    return arc.offset


def rezero_refusal(joint: str) -> Optional[str]:
    """Return WHY *joint* cannot be re-zeroed, or ``None`` if it can be.

    A refusal is an explanation, not a shrug. Two structurally different
    answers hide behind "no":

    * ``wrist_roll`` — **impossible**. Its travel covers the whole circle, so
      there is no unreachable arc to evict the seam into; no offset can help,
      and a soft limit already handles it (:data:`_REZERO_IMPOSSIBLE`).
    * the other four — **unnecessary**. Their encoders do not wrap inside their
      travel, so there is no seam in the way (:data:`_REZERO_UNNECESSARY`).

    Collapsing those into one message would teach the operator the wrong thing
    about their arm — and would make "wrist_roll isn't supported yet" a
    plausible reading of a limit that is permanent and provable.

    Raises
    ------
    ValueError
        If *joint* is not one of :data:`JOINTS`.
    """
    if rezero_arc(joint) is not None:  # validates *joint*
        return None
    impossible = _REZERO_IMPOSSIBLE.get(joint)
    if impossible is not None:
        return impossible
    return _REZERO_UNNECESSARY.format(joint=joint)


def resolve_bounds(joint: str, eeprom_min: int, eeprom_max: int) -> tuple[int, int]:
    """Return the travel bounds a move may actually use for *joint*.

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
    lower high bound. A joint with no soft limit (four of six — see
    :func:`soft_limit`) gets its EEPROM bounds back verbatim; this function
    must never quietly narrow a joint that never had a wrap problem.

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
        ``bus.read_info(motor)["min_angle"]`` / ``["max_angle"]``.

    Returns
    -------
    tuple[int, int]
        ``(min_tick, max_tick)`` — the bounds to hand to ``clamp_goal`` /
        ``compliant_move`` / ``gentle_move`` / ``GridSpec.bounds``.

    Raises
    ------
    ValueError
        If *joint* is unknown, or if the intersection is EMPTY — i.e. the
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
    limit = soft_limit(joint)  # validates *joint*
    if limit is None:
        return (eeprom_min, eeprom_max)

    low = max(eeprom_min, limit.min_tick)
    high = min(eeprom_max, limit.max_tick)
    # Only an EEPROM range that is itself well-ordered can produce an empty
    # intersection here; an inverted EEPROM pair stays inverted (low > high on
    # both sides of the max/min) and is clamp_goal's error to report, not ours.
    if eeprom_min <= eeprom_max and low > high:
        raise ValueError(
            f"Joint {joint!r} has no permitted travel: its servo angle limits "
            f"({eeprom_min}, {eeprom_max}) lie entirely inside the soft limit's dead arc "
            f"(permitted range is ({limit.min_tick}, {limit.max_tick})). Widen the servo's "
            "angle limits or retune the soft limit — they currently contradict each other."
        )
    return (low, high)


def soft_limit(joint: str) -> Optional[SoftLimit]:
    """Return *joint*'s :class:`SoftLimit`, or ``None`` if it has no software travel restriction.

    ``None`` is a real, common answer — most joints (four of six) have no
    wrap problem and no soft limit; ``None`` means "the full ``[TICK_MIN,
    TICK_MAX]`` range is permitted", not "unknown".

    Parameters
    ----------
    joint:
        One of the six joint names in :data:`JOINTS`.

    Raises
    ------
    ValueError
        If *joint* is not one of :data:`JOINTS`.
    """
    if joint not in JOINTS:
        raise ValueError(f"Unknown joint {joint!r}. Valid joints: {list(JOINTS)}.")
    return SOFT_LIMITS.get(joint)


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------


def _require_role(role: str) -> None:
    """Raise :class:`ValueError` if *role* is not a known arm role."""
    if role not in _KNOWN_ROLES:
        raise ValueError(f"Unknown arm role {role!r}. Valid roles: {sorted(_KNOWN_ROLES)}.")
