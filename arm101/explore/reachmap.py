"""arm101.explore.reachmap — derive, query, and persist the compact reach map.

This module is the DERIVED, compact, queryable reachability artifact of the
``arm explore`` feature: downstream code (and a future flex-gate) consults it
*offline* to decide whether a joint configuration is safe to command, without
touching hardware.

Three concerns live here, all operating purely on the ``arm101.explore.types``
data model (``ContactEvent`` / ``JointConfig`` / ``ReachMap``):

* :func:`build_from_events` — fold a stream of recorded :class:`ContactEvent`
  probes into a :class:`ReachMap`: a per-joint reachable ``(min, max)`` tick
  range (derived from REACHABLE events) plus the sparse, deduped set of BLOCKED
  joint-combinations.
* :func:`is_reachable` — the offline query. Reads ONLY the in-memory map; it
  never imports ``arm101.hardware``, opens a serial port, or moves a motor.
* :func:`save_map` / :func:`load_map_file` — serialize the compact map to / from
  a JSON file. The on-disk form is ``json.dumps(reach_map.to_dict())``, so a
  save-then-load round-trips to an identical :class:`ReachMap`.

Zero third-party imports (stdlib only) — same discipline as the rest of
``arm101.explore``.

Empty-range sentinel convention
--------------------------------
A joint with *no* reachable observation is recorded as :data:`EMPTY_RANGE`, an
INVERTED ``(TICK_MAX, TICK_MIN)`` span. Because ``min > max``, the inclusive
test ``min <= tick <= max`` is False for every valid tick, so an unobserved
joint makes :func:`is_reachable` return ``False`` rather than silently admitting
configurations. A ``None`` sentinel was rejected: ``ReachMap.__post_init__``
coerces every range entry through ``int(...)`` and ``None`` would crash there,
so the sentinel must be a real ``(int, int)`` tuple.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Iterable, List, Optional, Tuple, Union

from arm101.explore.types import (
    NUM_JOINTS,
    TICK_MAX,
    TICK_MIN,
    ContactEvent,
    ContactResult,
    JointConfig,
    ReachMap,
)

#: Sentinel ``(min, max)`` range for a joint with NO reachable observation.
#: Deliberately inverted (``min > max``) so ``min <= tick <= max`` is False for
#: every valid tick in ``[TICK_MIN, TICK_MAX]`` — an unobserved joint therefore
#: makes :func:`is_reachable` return ``False``. See the module docstring for why
#: a tuple sentinel (not ``None``) is required by the ``ReachMap`` contract.
EMPTY_RANGE: Tuple[int, int] = (TICK_MAX, TICK_MIN)

#: Accepted path types for :func:`save_map` / :func:`load_map_file`.
PathLike = Union[str, Path]


def _fold_reachable(
    mins: "List[Optional[int]]", maxs: "List[Optional[int]]", config: JointConfig
) -> None:
    """Widen the running per-joint ``(min, max)`` tick bounds for one REACHABLE config."""
    for j, tick in enumerate(config.ticks):
        if mins[j] is None or tick < mins[j]:  # type: ignore[operator]
            mins[j] = tick
        if maxs[j] is None or tick > maxs[j]:  # type: ignore[operator]
            maxs[j] = tick


def build_from_events(events: Iterable[ContactEvent]) -> ReachMap:
    """Fold recorded contact events into the compact :class:`ReachMap`.

    Consumes *events* exactly once (any iterable, including a generator).

    * ``reachable_ranges[j]`` is the ``(min, max)`` tick observed at joint ``j``
      across every event whose ``result`` is ``REACHABLE`` (the full config of a
      reachable event contributes to all 6 joint ranges). A joint that is never
      observed reachable is recorded as :data:`EMPTY_RANGE`.
    * ``blocked`` is the sparse set of configs from ``BLOCKED`` events, deduped
      and ordered by first appearance (deterministic given event order).

    Parameters
    ----------
    events : Iterable[ContactEvent]
        The recorded probe stream.

    Returns
    -------
    ReachMap
        The derived compact reachability map.
    """
    mins: List[Optional[int]] = [None] * NUM_JOINTS
    maxs: List[Optional[int]] = [None] * NUM_JOINTS
    # dict as an insertion-ordered set: dedup while preserving first-seen order.
    blocked: "dict[JointConfig, None]" = {}

    for event in events:
        if event.result == ContactResult.BLOCKED:
            blocked.setdefault(event.config, None)
            continue
        if event.result != ContactResult.REACHABLE:
            continue
        _fold_reachable(mins, maxs, event.config)

    ranges: Tuple[Tuple[int, int], ...] = tuple(
        EMPTY_RANGE if mins[j] is None else (mins[j], maxs[j])  # type: ignore[misc]
        for j in range(NUM_JOINTS)
    )
    return ReachMap(reachable_ranges=ranges, blocked=tuple(blocked))


def is_reachable(reach_map: ReachMap, config: JointConfig) -> bool:
    """Return whether *config* is reachable per *reach_map* — a pure, offline query.

    ``True`` iff every joint tick lies within that joint's reachable range
    (inclusive) AND *config* is not among the map's ``blocked`` combinations.

    Reads ONLY the in-memory *reach_map*: it opens no serial port, imports no
    hardware module, and moves no motor.

    Parameters
    ----------
    reach_map : ReachMap
        The compact map to query.
    config : JointConfig
        The candidate joint configuration.

    Returns
    -------
    bool
    """
    for tick, (low, high) in zip(config.ticks, reach_map.reachable_ranges):
        if not (low <= tick <= high):
            return False
    return config not in reach_map.blocked


def save_map(path: PathLike, reach_map: ReachMap) -> None:
    """Serialize *reach_map* to *path* as JSON (``json.dumps(to_dict())``).

    Writes the compact artifact — compact relative to the raw JSONL event log.
    Round-trips exactly with :func:`load_map_file`.

    Written **atomically**: the JSON goes to a sibling temp file in the same
    directory, then :func:`os.replace` swaps it into place. A crash mid-write
    therefore never leaves a truncated map that :func:`load_map_file` can't
    parse — the destination is either the old map or the complete new one.
    """
    dest = Path(path)
    tmp = dest.with_name(dest.name + ".tmp")
    tmp.write_text(json.dumps(reach_map.to_dict()), encoding="utf-8")
    os.replace(tmp, dest)


def load_map_file(path: PathLike) -> ReachMap:
    """Load a :class:`ReachMap` previously written by :func:`save_map`.

    Named ``load_map_file`` (not ``load_map``) to avoid colliding with the
    later ``default_map.load_map`` task, which loads a different, packaged map.
    """
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return ReachMap.from_dict(data)


__all__ = [
    "EMPTY_RANGE",
    "PathLike",
    "build_from_events",
    "is_reachable",
    "save_map",
    "load_map_file",
]
