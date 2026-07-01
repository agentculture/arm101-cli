"""arm101.explore — reachability mapping for the SO-101 arm.

Drives the ``arm101 arm explore`` verb: safely walking the follower's
joint-space, auto-detecting self/environment contact, and recording the
reachable envelope to a map file. Zero third-party imports at module load
time (stdlib only), except for ``engine.py``, which is the sole module
permitted to talk to ``arm101.hardware.bus`` to actually drive motion.

Re-exports the shared data types from :mod:`arm101.explore.types` so callers
can write ``from arm101.explore import JointConfig`` instead of reaching into
the submodule directly.
"""

from __future__ import annotations

from arm101.explore.types import (
    JOINT_NAMES,
    NUM_JOINTS,
    TICK_MAX,
    TICK_MIN,
    ContactEvent,
    ContactResult,
    GridSpec,
    JointConfig,
    ReachMap,
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
