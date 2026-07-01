"""arm101.explore.budget — the run budget + thermal guard for SO-101 exploration.

Load-bearing module: this is what GUARANTEES an ``arm101 arm explore`` run
terminates. The exploration engine's combinatorial escape search over
joint-space has no natural stopping point of its own — without an external
bound it could run forever — so every long-running loop in ``engine.py``
(and its escape/backtrack search) must be driven by :meth:`Budget.should_continue`.

:class:`Budget` bounds a run by THREE independent limits, any one of which
can end it:

* **moves** — a hard cap on the number of probes/moves issued
  (:data:`DEFAULT_MAX_MOVES`).
* **wall-time** — a hard cap on elapsed real time, measured via an
  **injected clock** (:data:`DEFAULT_MAX_SECONDS`) so tests can drive time
  deterministically without a real ``time.sleep``.
* **thermal** — a servo-temperature ceiling (:data:`DEFAULT_MAX_TEMPERATURE_C`)
  checked against live per-joint readings, independent of the other two caps
  because it needs fresh hardware data on every check rather than being
  derivable from internal counters alone.

Zero third-party imports (stdlib only: ``time``, ``typing``) — consistent
with the rest of ``arm101.explore``, which stays decoupled from
``arm101.hardware`` except in ``engine.py``.

Open question (frame risk r3)
------------------------------
The exact default cap values below are a **hardware-tuned open question**:
they are chosen to be conservative for a first real run on physical SO-101
hardware, not derived from a benchmark. The STS3215 servo's own ``Max_Temp``
register defaults to 70C, so :data:`DEFAULT_MAX_TEMPERATURE_C` leaves a 10C
margin under that hardware ceiling. Expect these numbers to be revisited
once real exploration runs produce data on how long/how many moves a useful
run actually needs.
"""

from __future__ import annotations

import time
from typing import Callable, Optional, Sequence

# ---------------------------------------------------------------------------
# Default cap constants (frame risk r3 — hardware-tuned open question)
# ---------------------------------------------------------------------------

#: Conservative default cap on the number of moves/probes in a single run.
DEFAULT_MAX_MOVES: int = 2000

#: Conservative default cap on elapsed wall-clock time, in seconds (10 minutes).
DEFAULT_MAX_SECONDS: float = 600.0

#: Conservative default per-joint temperature ceiling, in degrees Celsius.
#: The SO-101's STS3215 servo Max_Temp register defaults to 70C; this leaves
#: a 10C safety margin under that hardware limit.
DEFAULT_MAX_TEMPERATURE_C: int = 60


class Budget:
    """Bounds an exploration run by moves, wall-time, and joint temperature.

    Any one of the three independent caps ending a run is enough to halt it.
    Thermal is checked separately from the other two (via
    :meth:`check_thermal`) because it needs a live per-joint temperature
    reading rather than being derivable from internal counters.

    Parameters
    ----------
    max_moves : int
        Hard cap on the number of moves/probes (see :data:`DEFAULT_MAX_MOVES`).
    max_seconds : float
        Hard cap on elapsed wall-clock time, in seconds (see
        :data:`DEFAULT_MAX_SECONDS`).
    max_temperature_c : int
        Per-joint temperature ceiling, in degrees Celsius (see
        :data:`DEFAULT_MAX_TEMPERATURE_C`).
    clock : Callable[[], float]
        Zero-arg callable returning a monotonically increasing time value,
        in seconds. Defaults to :func:`time.monotonic`. Tests inject a fake
        clock so wall-time behavior is deterministic (no real ``time.sleep``).
        The start time is captured immediately, at construction.

    Attributes
    ----------
    moves : int
        Number of moves recorded so far via :meth:`record_move`.
    elapsed_seconds : float
        Time elapsed since construction, per the injected ``clock``.
    """

    def __init__(
        self,
        max_moves: int = DEFAULT_MAX_MOVES,
        max_seconds: float = DEFAULT_MAX_SECONDS,
        max_temperature_c: int = DEFAULT_MAX_TEMPERATURE_C,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.max_moves = max_moves
        self.max_seconds = max_seconds
        self.max_temperature_c = max_temperature_c
        self._clock = clock
        self._moves = 0
        self._start_time = self._clock()

    @property
    def moves(self) -> int:
        """Number of moves recorded so far via :meth:`record_move`."""
        return self._moves

    @property
    def elapsed_seconds(self) -> float:
        """Elapsed wall-clock time since construction, per the injected clock."""
        return self._clock() - self._start_time

    def record_move(self) -> None:
        """Record that one move/probe was issued, incrementing the move counter."""
        self._moves += 1

    def exhausted(self) -> bool:
        """Return True if the move cap or the wall-time cap has been reached.

        Does NOT consult the thermal guard — that needs live temperature data
        and is checked separately via :meth:`check_thermal`.
        """
        return self._moves >= self.max_moves or self.elapsed_seconds >= self.max_seconds

    def check_thermal(self, temperatures: Sequence[int]) -> bool:
        """Return True (halt) if any joint temperature exceeds ``max_temperature_c``.

        Meeting the ceiling exactly does not halt — only strictly exceeding
        it does. An empty *temperatures* sequence never halts.
        """
        return any(t > self.max_temperature_c for t in temperatures)

    def should_continue(self, temperatures: Optional[Sequence[int]] = None) -> bool:
        """Convenience: combine :meth:`exhausted` and :meth:`check_thermal`.

        Returns False (stop) if either the move/time caps are exhausted, or
        *temperatures* is given and any joint exceeds the thermal ceiling.
        When *temperatures* is ``None``, the thermal guard is not consulted.
        """
        if self.exhausted():
            return False
        if temperatures is not None and self.check_thermal(temperatures):
            return False
        return True


__all__ = [
    "DEFAULT_MAX_MOVES",
    "DEFAULT_MAX_SECONDS",
    "DEFAULT_MAX_TEMPERATURE_C",
    "Budget",
]
