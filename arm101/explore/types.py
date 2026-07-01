"""arm101.explore.types — shared data types for SO-101 reachability exploration.

Pure-data module: every downstream ``arm101.explore`` module (``grid``, ``log``,
``reachmap``, ``budget``, ``escape``, ``engine``) imports its shared types from
here. Zero third-party imports (stdlib only: ``dataclasses``, ``enum``,
``typing``) — ``arm101.explore`` is deliberately decoupled from
``arm101.hardware`` (only ``engine.py`` talks to ``arm101.hardware.bus``), so
the canonical joint order is duplicated here rather than imported from
``arm101.hardware.arm_spec``.

Contents
--------
:class:`JointConfig`
    An immutable snapshot of all 6 SO-101 joint positions, in encoder ticks.
    Hashable — usable as a dict/set key (downstream cell-visited tracking).
:class:`GridSpec`
    Discretization parameters (per-joint bucket size, origin, per-joint
    bounds). The tick<->cell math itself lives in the later ``grid.py``.
:class:`ContactResult`
    ``REACHABLE`` / ``BLOCKED`` outcome of a single probe.
:class:`ContactEvent`
    One recorded probe/contact event — the JSONL log line schema that
    ``log.py`` appends/reads.
:class:`ReachMap`
    The compact reachability map DATA container — build/query logic (e.g.
    ``is_reachable``, ``build_from_events``) lives in the later ``reachmap.py``.

Every type here round-trips losslessly through ``to_dict()`` / ``from_dict()``,
and every ``to_dict()`` payload is plain-JSON-serializable (``int``/``str``/
``list``/``dict`` only) so it can pass through ``json.dumps``/``json.loads``
unchanged — that is the exact contract the JSONL event log depends on.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, Optional, Tuple

# ---------------------------------------------------------------------------
# Public constants
# ---------------------------------------------------------------------------

#: Canonical joint names for the SO-101, in hardware order (shoulder_pan ->
#: gripper, motor ids 1-6). Matches ``arm101.hardware.arm_spec.JOINTS`` —
#: duplicated (not imported) so ``arm101.explore`` stays self-contained.
JOINT_NAMES: Tuple[str, str, str, str, str, str] = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)

#: Number of joints on the SO-101 (``len(JOINT_NAMES)``).
NUM_JOINTS: int = len(JOINT_NAMES)

#: Valid encoder tick bounds shared by every SO-101 joint (12-bit encoder).
TICK_MIN: int = 0
TICK_MAX: int = 4095

#: Default per-joint ``(min, max)`` bounds used by :class:`GridSpec` when the
#: caller does not supply its own.
_DEFAULT_BOUNDS: Tuple[Tuple[int, int], ...] = tuple(
    (TICK_MIN, TICK_MAX) for _ in range(NUM_JOINTS)
)


def _validate_tick(value: int, joint_name: str) -> int:
    """Return *value* as ``int`` if within ``[TICK_MIN, TICK_MAX]``, else raise ``ValueError``."""
    value = int(value)
    if not (TICK_MIN <= value <= TICK_MAX):
        raise ValueError(f"{joint_name} tick {value} out of range [{TICK_MIN}, {TICK_MAX}].")
    return value


# ---------------------------------------------------------------------------
# JointConfig
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class JointConfig:
    """Immutable snapshot of all 6 SO-101 joint positions, in encoder ticks.

    Each field is a raw 12-bit encoder tick in ``[0, 4095]``, validated on
    construction. Frozen (and therefore hashable via the dataclass-generated
    ``__hash__``) so it can be used directly as a ``dict``/``set`` key — this
    is load-bearing for downstream cell-visited tracking (``grid.py``,
    ``engine.py``).

    Attributes
    ----------
    shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper : int
        Raw encoder tick in ``[0, 4095]`` for each joint (motor ids 1-6, in
        that order — see :data:`JOINT_NAMES`).

    Raises
    ------
    ValueError
        If any joint's tick is outside ``[0, 4095]``.
    """

    shoulder_pan: int
    shoulder_lift: int
    elbow_flex: int
    wrist_flex: int
    wrist_roll: int
    gripper: int

    def __post_init__(self) -> None:
        for name in JOINT_NAMES:
            object.__setattr__(self, name, _validate_tick(getattr(self, name), name))

    @property
    def ticks(self) -> Tuple[int, int, int, int, int, int]:
        """Return all 6 joint ticks as a tuple, in :data:`JOINT_NAMES` order."""
        return tuple(getattr(self, name) for name in JOINT_NAMES)  # type: ignore[return-value]

    def __getitem__(self, index: int) -> int:
        """Return the tick at joint *index* (0-5, :data:`JOINT_NAMES` order)."""
        return self.ticks[index]

    def __iter__(self):
        """Iterate over the 6 joint ticks, in :data:`JOINT_NAMES` order."""
        return iter(self.ticks)

    def __len__(self) -> int:
        """Always ``6`` — the number of SO-101 joints."""
        return NUM_JOINTS

    @classmethod
    def from_ticks(cls, ticks: "Tuple[int, ...] | list") -> "JointConfig":
        """Build a :class:`JointConfig` from a 6-element sequence, :data:`JOINT_NAMES` order.

        Raises
        ------
        ValueError
            If *ticks* does not have exactly 6 elements, or any tick is
            outside ``[0, 4095]``.
        """
        ticks = tuple(ticks)
        if len(ticks) != NUM_JOINTS:
            raise ValueError(f"Expected {NUM_JOINTS} ticks, got {len(ticks)}.")
        return cls(*ticks)

    def to_dict(self) -> Dict[str, int]:
        """Return a plain-JSON-serializable ``{joint_name: tick}`` mapping."""
        return {name: getattr(self, name) for name in JOINT_NAMES}

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "JointConfig":
        """Inverse of :meth:`to_dict`. Raises ``KeyError``/``ValueError`` on malformed input."""
        return cls(**{name: int(data[name]) for name in JOINT_NAMES})


# ---------------------------------------------------------------------------
# GridSpec
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class GridSpec:
    """Discretization parameters for the joint-space exploration grid.

    Pure data holder — the tick<->cell mapping itself is implemented in the
    later ``arm101/explore/grid.py``, not here.

    Attributes
    ----------
    bucket_size : tuple[int, ...]
        Per-joint discretization step, in encoder ticks — one value per
        joint, :data:`JOINT_NAMES` order.
    origin : JointConfig
        The home/origin configuration the grid is discretized relative to.
    bounds : tuple[tuple[int, int], ...]
        Per-joint ``(min, max)`` encoder-tick bounds, :data:`JOINT_NAMES`
        order. Defaults to ``(0, 4095)`` for every joint.

    Raises
    ------
    ValueError
        If ``bucket_size`` or ``bounds`` does not have exactly 6 entries.
    """

    bucket_size: Tuple[int, ...]
    origin: JointConfig
    bounds: Tuple[Tuple[int, int], ...] = field(default_factory=lambda: _DEFAULT_BOUNDS)

    def __post_init__(self) -> None:
        bucket_size = tuple(int(v) for v in self.bucket_size)
        bounds = tuple(tuple(int(v) for v in b) for b in self.bounds)
        if len(bucket_size) != NUM_JOINTS:
            raise ValueError(f"Expected {NUM_JOINTS} bucket_size entries, got {len(bucket_size)}.")
        if len(bounds) != NUM_JOINTS:
            raise ValueError(f"Expected {NUM_JOINTS} bounds entries, got {len(bounds)}.")
        object.__setattr__(self, "bucket_size", bucket_size)
        object.__setattr__(self, "bounds", bounds)

    def to_dict(self) -> Dict[str, Any]:
        """Return a plain-JSON-serializable representation of this :class:`GridSpec`."""
        return {
            "bucket_size": list(self.bucket_size),
            "origin": self.origin.to_dict(),
            "bounds": [list(bound) for bound in self.bounds],
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "GridSpec":
        """Inverse of :meth:`to_dict`."""
        return cls(
            bucket_size=tuple(int(v) for v in data["bucket_size"]),
            origin=JointConfig.from_dict(data["origin"]),
            bounds=tuple(tuple(int(v) for v in bound) for bound in data["bounds"]),
        )


# ---------------------------------------------------------------------------
# ContactResult
# ---------------------------------------------------------------------------


class ContactResult(str, Enum):
    """Outcome of a single reachability probe.

    A ``str`` subclass so a :class:`ContactResult` member serializes to JSON
    as its plain string value with no extra conversion at call sites, while
    still comparing/hashing distinctly per member.
    """

    REACHABLE = "reachable"
    BLOCKED = "blocked"


# ---------------------------------------------------------------------------
# ContactEvent
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ContactEvent:
    """One recorded probe/contact event — the JSONL event-log line schema.

    Attributes
    ----------
    config : JointConfig
        The full joint configuration at the moment of the probe.
    moving_joint_index : int
        Index (0-5, :data:`JOINT_NAMES` order) of the joint that was moving
        when this event was recorded.
    load_magnitude : int
        Direction-independent servo load magnitude observed for the moving
        joint (see ``arm101.hardware.bus.load_magnitude`` for the equivalent
        hardware-side computation; this module holds no such coupling).
    result : ContactResult
        Whether the probe was reachable or blocked.
    step : int | None
        Optional monotonic step/sequence number. ``None`` if the caller does
        not track one.

    Raises
    ------
    ValueError
        If ``moving_joint_index`` is outside ``[0, 5]`` or ``load_magnitude``
        is negative.
    """

    config: JointConfig
    moving_joint_index: int
    load_magnitude: int
    result: ContactResult
    step: Optional[int] = None

    def __post_init__(self) -> None:
        if not (0 <= self.moving_joint_index < NUM_JOINTS):
            raise ValueError(
                f"moving_joint_index {self.moving_joint_index} out of range "
                f"[0, {NUM_JOINTS - 1}]."
            )
        if self.load_magnitude < 0:
            raise ValueError(f"load_magnitude {self.load_magnitude} must be non-negative.")
        if not isinstance(self.result, ContactResult):
            object.__setattr__(self, "result", ContactResult(self.result))

    def to_dict(self) -> Dict[str, Any]:
        """Return a plain-JSON-serializable representation — the JSONL line payload."""
        return {
            "config": self.config.to_dict(),
            "moving_joint_index": self.moving_joint_index,
            "load_magnitude": self.load_magnitude,
            "result": self.result.value,
            "step": self.step,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ContactEvent":
        """Inverse of :meth:`to_dict`."""
        return cls(
            config=JointConfig.from_dict(data["config"]),
            moving_joint_index=int(data["moving_joint_index"]),
            load_magnitude=int(data["load_magnitude"]),
            result=ContactResult(data["result"]),
            step=data.get("step"),
        )


# ---------------------------------------------------------------------------
# ReachMap
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ReachMap:
    """The compact reachability map DATA container.

    Pure data holder — build-from-events, serialize/deserialize-to-file, and
    the offline ``is_reachable`` query all live in the later
    ``arm101/explore/reachmap.py``, not here.

    Attributes
    ----------
    reachable_ranges : tuple[tuple[int, int], ...]
        Per-joint reachable ``(min, max)`` encoder-tick range, one entry per
        joint, :data:`JOINT_NAMES` order.
    blocked : tuple[JointConfig, ...]
        Sparse collection of blocked joint-combinations, each recorded as the
        full :class:`JointConfig` at which contact occurred. Defaults to
        empty.

    Raises
    ------
    ValueError
        If ``reachable_ranges`` does not have exactly 6 entries.
    """

    reachable_ranges: Tuple[Tuple[int, int], ...]
    blocked: Tuple[JointConfig, ...] = ()

    def __post_init__(self) -> None:
        ranges = tuple(tuple(int(v) for v in r) for r in self.reachable_ranges)
        if len(ranges) != NUM_JOINTS:
            raise ValueError(f"Expected {NUM_JOINTS} reachable_ranges entries, got {len(ranges)}.")
        object.__setattr__(self, "reachable_ranges", ranges)
        object.__setattr__(self, "blocked", tuple(self.blocked))

    def to_dict(self) -> Dict[str, Any]:
        """Return a plain-JSON-serializable representation of this :class:`ReachMap`."""
        return {
            "reachable_ranges": [list(r) for r in self.reachable_ranges],
            "blocked": [cfg.to_dict() for cfg in self.blocked],
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ReachMap":
        """Inverse of :meth:`to_dict`."""
        return cls(
            reachable_ranges=tuple(tuple(int(v) for v in r) for r in data["reachable_ranges"]),
            blocked=tuple(JointConfig.from_dict(cfg) for cfg in data["blocked"]),
        )


__all__ = [
    "JOINT_NAMES",
    "NUM_JOINTS",
    "TICK_MIN",
    "TICK_MAX",
    "JointConfig",
    "GridSpec",
    "ContactResult",
    "ContactEvent",
    "ReachMap",
]
