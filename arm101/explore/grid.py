"""arm101.explore.grid — tick<->cell discretization over a GridSpec.

Turns a :class:`~arm101.explore.types.GridSpec` into a concrete joint-space
grid: mapping a continuous :class:`~arm101.explore.types.JointConfig` onto a
discrete :data:`Cell` (one bucket index per joint), back again, and
enumerating the 6-DOF grid neighborhood of a cell. This is the layer the
later ``engine.py`` walks over — it never talks to hardware itself.

Zero third-party imports (stdlib only), matching the rest of
``arm101.explore``.

Conventions
-----------
* A joint's bucket index is ``(tick - bound_min) // bucket_size``, clamped
  into ``[0, max_bucket]`` where ``max_bucket = (bound_max - bound_min) //
  bucket_size`` — the highest bucket index that fits inside the joint's
  ``GridSpec.bounds`` entry.
* The representative tick for a bucket is its **lower edge**:
  ``bound_min + bucket * bucket_size``. Because ``max_bucket`` is derived by
  floor division, this edge tick is always ``<= bound_max`` for any bucket in
  ``[0, max_bucket]``, so :func:`config_to_cell` and :func:`cell_to_config`
  round-trip exactly for every in-range cell.
"""

from __future__ import annotations

from typing import List, Tuple

from arm101.explore.types import NUM_JOINTS, GridSpec, JointConfig

#: A discrete grid cell: one bucket index per joint, in canonical
#: ``JOINT_NAMES`` order.
Cell = Tuple[int, ...]


def _max_bucket_index(bound_min: int, bound_max: int, bucket_size: int) -> int:
    """Return the highest valid bucket index for one joint's bound/bucket_size."""
    return (bound_max - bound_min) // bucket_size


def config_to_cell(config: JointConfig, spec: GridSpec) -> Cell:
    """Map *config* onto its discrete grid :data:`Cell` under *spec*.

    Each joint's tick is first clamped into that joint's ``spec.bounds``
    entry, then converted to a bucket index and clamped into
    ``[0, max_bucket]`` so every returned index is always in range.
    """
    cell = []
    for i in range(NUM_JOINTS):
        bound_min, bound_max = spec.bounds[i]
        bucket_size = spec.bucket_size[i]
        tick = max(bound_min, min(bound_max, config[i]))
        bucket = (tick - bound_min) // bucket_size
        max_bucket = _max_bucket_index(bound_min, bound_max, bucket_size)
        bucket = max(0, min(max_bucket, bucket))
        cell.append(bucket)
    return tuple(cell)


def cell_to_config(cell: Cell, spec: GridSpec) -> JointConfig:
    """Map *cell* back to a representative :class:`JointConfig` under *spec*.

    Uses each bucket's lower-edge tick, clamped into that joint's
    ``spec.bounds`` entry as a defensive measure (see module docstring for
    why this is a no-op for any in-range bucket index).
    """
    ticks = []
    for i in range(NUM_JOINTS):
        bound_min, bound_max = spec.bounds[i]
        bucket_size = spec.bucket_size[i]
        tick = bound_min + cell[i] * bucket_size
        tick = max(bound_min, min(bound_max, tick))
        ticks.append(tick)
    return JointConfig.from_ticks(tuple(ticks))


def neighbors(cell: Cell, spec: GridSpec) -> List[Cell]:
    """Return the 6-DOF +-1-bucket neighbors of *cell* under *spec*.

    Each of the 6 joints is perturbed by -1 and +1 bucket independently (up
    to 12 candidates); any candidate whose perturbed joint index would fall
    outside ``[0, max_bucket]`` for that joint is dropped.
    """
    result: List[Cell] = []
    for i in range(NUM_JOINTS):
        bound_min, bound_max = spec.bounds[i]
        max_bucket = _max_bucket_index(bound_min, bound_max, spec.bucket_size[i])
        for delta in (-1, 1):
            candidate = cell[i] + delta
            if 0 <= candidate <= max_bucket:
                neighbor = list(cell)
                neighbor[i] = candidate
                result.append(tuple(neighbor))
    return result


def clamp(config: JointConfig, spec: GridSpec) -> JointConfig:
    """Clamp every joint tick of *config* into its ``spec.bounds`` entry."""
    ticks = []
    for i in range(NUM_JOINTS):
        bound_min, bound_max = spec.bounds[i]
        ticks.append(max(bound_min, min(bound_max, config[i])))
    return JointConfig.from_ticks(tuple(ticks))


def home_cell(spec: GridSpec) -> Cell:
    """Return the :data:`Cell` of ``spec.origin``."""
    return config_to_cell(spec.origin, spec)


__all__ = [
    "Cell",
    "config_to_cell",
    "cell_to_config",
    "neighbors",
    "clamp",
    "home_cell",
]
