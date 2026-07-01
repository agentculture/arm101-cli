"""arm101.explore.engine — the flood-fill reachability engine (the integration).

This is the module that ties the whole ``arm explore`` feature together. It
flood-fills the SO-101's reachable joint-space outward from a home
configuration, driving EVERY physical move through the overload-safe
:func:`arm101.hardware.gentle.gentle_move`, recording a
:class:`~arm101.explore.types.ContactEvent` for every attempt to the JSONL
:mod:`~arm101.explore.log`, invoking the combination-:func:`~arm101.explore.escape.escape`
search whenever a joint is blocked, all bounded by a
:class:`~arm101.explore.budget.Budget` — then derives and saves the compact
:class:`~arm101.explore.reachmap.ReachMap`.

This is the ONE module in ``arm101.explore`` permitted to talk to
``arm101.hardware`` (via ``gentle_move``); every sibling module is deliberately
hardware-free. Even here the coupling is funnelled through a single injected
*move function* (:data:`MoveFn`, defaulting to ``gentle_move``) so the whole
engine is testable with no serial port at all — mirroring how
:func:`~arm101.explore.escape.escape` injects its :data:`~arm101.explore.escape.Probe`.

Design invariants
-----------------
* **The move function is the SOLE motion path.** The engine NEVER calls
  ``bus.write_goal_position`` (or any raw motion register) directly — every
  physical move, in both the flood-fill and the escape probe, goes through
  ``move_fn``. That is the entire reason ``gentle_move`` (with its load-watch
  back-off and its Torque_Limit cap) is the safe default.
* **A servo overload is never fatal.** Because ``move_fn`` is ``gentle_move``,
  the servo's own overload latch tripping mid-move RETURNS ``overloaded=True``
  (after clearing torque) rather than raising, so the engine treats it exactly
  like a contact/block and the run continues. No ``OverloadError`` escapes a
  run.
* **The run always terminates.** BFS dedups both cells and probed configs, so a
  finite grid is finite work; on top of that the shared ``Budget`` (moves,
  wall-time, thermal) bounds the run and stops it cleanly when spent.
* **Resume is free.** Configs already recorded in the log are not re-probed, and
  previously-reachable cells are re-enqueued so an interrupted run picks up where
  it left off.

Zero third-party imports (stdlib only: ``collections``, ``contextlib``,
``dataclasses``, ``pathlib``, ``typing``) beyond the sibling explore modules and
the injected ``gentle_move``.
"""

from __future__ import annotations

import contextlib
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional, Sequence, Union

from arm101.cli._errors import CliError
from arm101.explore.budget import Budget
from arm101.explore.escape import (
    DEFAULT_MAX_ESCAPE_BREADTH,
    DEFAULT_MAX_ESCAPE_DEPTH,
    ProbeResult,
    escape,
)
from arm101.explore.grid import (
    Cell,
    cell_to_config,
    config_to_cell,
    home_cell,
    neighbors,
)
from arm101.explore.log import append_event, read_events
from arm101.explore.reachmap import build_from_events, save_map
from arm101.explore.types import (
    NUM_JOINTS,
    TICK_MAX,
    TICK_MIN,
    ContactEvent,
    ContactResult,
    GridSpec,
    JointConfig,
    ReachMap,
)
from arm101.hardware.gentle import gentle_move

#: Path argument type accepted for the log/map artifacts.
PathLike = Union[str, Path]

#: The injected physical move. Same call shape and return dict as
#: :func:`arm101.hardware.gentle.gentle_move` — given ``(bus, motor, target)``
#: plus ``min_angle``/``max_angle``/``threshold``/``allow_motion`` keywords, it
#: performs one load-watched move and returns a result dict carrying at least
#: ``contacted``, ``overloaded``, ``contact_load`` and ``final_position``. The
#: default is ``gentle_move``; tests inject a synthetic closure.
MoveFn = Callable[..., dict]

#: A zero-arg provider of live per-joint temperatures (deg C) for the thermal
#: guard, or ``None`` to skip thermal checks. Injected (like ``move_fn``) so the
#: thermal path is reachable without hardware; the default engine call passes
#: ``None`` and relies on the move/time caps.
TemperatureFn = Callable[[], Sequence[int]]

#: Default contact-load threshold, mirroring ``gentle_move``'s own default.
_DEFAULT_THRESHOLD = 250


@dataclass(frozen=True)
class ExploreResult:
    """Summary of an :func:`explore` run — the map plus lightweight counters.

    The compact :attr:`reach_map` (also written to ``map_path``) and the JSONL
    event log (at ``log_path``) are the real deliverables; the remaining fields
    are a render-friendly summary for the ``arm explore`` CLI verb (t9).

    Attributes
    ----------
    reach_map : ReachMap
        The derived compact reachability map (also saved to :attr:`map_path`).
    cells_visited : int
        Number of distinct grid cells enqueued as reachable during the walk
        (includes the home cell and any cells freed by an escape).
    moves : int
        Total moves recorded on the budget — flood-fill moves PLUS every escape
        probe.
    reachable : int
        Count of REACHABLE flood-fill attempts recorded to the log.
    contacts : int
        Count of BLOCKED flood-fill attempts recorded to the log.
    escapes_attempted : int
        Number of times the combination-escape search was invoked (once per
        block).
    escapes_succeeded : int
        Number of those escapes that returned a path (freed the joint).
    budget_bounded : bool
        ``True`` if the run stopped because the budget was spent (move/time cap
        or thermal ceiling) rather than the frontier draining naturally.
    log_path : str
        Filesystem path of the JSONL event log.
    map_path : str
        Filesystem path of the saved compact map.
    """

    reach_map: ReachMap
    cells_visited: int
    moves: int
    reachable: int
    contacts: int
    escapes_attempted: int
    escapes_succeeded: int
    budget_bounded: bool
    log_path: str
    map_path: str


def _differing_joint(cell: Cell, neighbor: Cell) -> Optional[int]:
    """Return the single joint index at which *neighbor* differs from *cell*.

    ``neighbors`` only ever perturbs one joint by one bucket, so exactly one
    index differs; returns ``None`` only for the degenerate identical-cell case
    (which the caller skips).
    """
    for j in range(NUM_JOINTS):
        if cell[j] != neighbor[j]:
            return j
    return None  # pragma: no cover - neighbors() always perturbs exactly one joint


def _read_temperatures(temperatures: Optional[TemperatureFn]) -> Optional[Sequence[int]]:
    """Best-effort read of live joint temperatures for the thermal guard.

    Returns ``None`` (skip the thermal check this iteration) when no provider is
    injected or the provider raises a :class:`CliError` — including an
    ``OverloadError`` — so a flaky temperature read can never break the run.
    """
    if temperatures is None:
        return None
    with contextlib.suppress(CliError):
        return temperatures()
    return None


def _make_probe(bus: object, spec: GridSpec, threshold: int, move_fn: MoveFn):
    """Build the injected :data:`~arm101.explore.escape.Probe` for ``escape``.

    The probe advances one joint a single bucket step (in its exploration
    direction) via ``move_fn`` — the SAME overload-safe move path the flood-fill
    uses — and reports a :class:`~arm101.explore.escape.ProbeResult`.
    """

    def probe(from_config: JointConfig, joint: int) -> ProbeResult:
        bound_min, bound_max = spec.bounds[joint]
        current = from_config[joint]
        target = min(current + spec.bucket_size[joint], bound_max)
        if target == current:  # already at the top bucket — step the other way
            target = max(current - spec.bucket_size[joint], bound_min)

        result = move_fn(
            bus,
            motor=joint + 1,
            target=target,
            min_angle=bound_min,
            max_angle=bound_max,
            threshold=threshold,
            allow_motion=True,
        )
        reachable = (not result["contacted"]) and (not result["overloaded"])

        final = result.get("final_position")
        final_tick = current if final is None else int(final)
        final_tick = max(TICK_MIN, min(TICK_MAX, final_tick))
        ticks = list(from_config.ticks)
        ticks[joint] = final_tick
        return ProbeResult(reachable=reachable, position=JointConfig.from_ticks(ticks))

    return probe


def _load_from_result(result: dict) -> int:
    """Extract a non-negative load magnitude from a move result dict."""
    load = result.get("contact_load")
    return int(load) if load else 0


class _Walk:
    """Mutable working state + BFS driver for a single :func:`explore` run.

    Kept as a small class so the counters, the frontier, and the dedup sets do
    not have to be threaded through a chain of helper functions — the public
    entry point (:func:`explore`) constructs one, runs it, and reads back the
    summary.
    """

    def __init__(
        self,
        bus: object,
        spec: GridSpec,
        *,
        log_path: PathLike,
        threshold: int,
        budget: Budget,
        move_fn: MoveFn,
        max_escape_depth: int,
        max_escape_breadth: int,
        temperatures: Optional[TemperatureFn],
    ) -> None:
        self.bus = bus
        self.spec = spec
        self.log_path = log_path
        self.threshold = threshold
        self.budget = budget
        self.move_fn = move_fn
        self.max_escape_depth = max_escape_depth
        self.max_escape_breadth = max_escape_breadth
        self.temperatures = temperatures
        self.probe = _make_probe(bus, spec, threshold, move_fn)

        home = home_cell(spec)
        home_config = cell_to_config(home, spec)

        prior_events = read_events(log_path)
        #: Configs already probed — seeded from the log (resume) and grown as we
        #: go, so no config is ever probed twice. Home counts as already-known.
        self.probed = {event.config for event in prior_events}
        self.probed.add(home_config)

        self.visited_cells = {home}
        self.frontier: "deque[Cell]" = deque([home])
        # Resume: re-enqueue every previously-reachable cell so exploration
        # continues past where the interrupted run stopped.
        for event in prior_events:
            if event.result == ContactResult.REACHABLE:
                cell = config_to_cell(event.config, spec)
                if cell not in self.visited_cells:
                    self.visited_cells.add(cell)
                    self.frontier.append(cell)

        self.step_no = len(prior_events)
        self.reachable = 0
        self.contacts = 0
        self.escapes_attempted = 0
        self.escapes_succeeded = 0
        self.budget_bounded = False

    def _enqueue(self, cell: Cell) -> None:
        if cell not in self.visited_cells:
            self.visited_cells.add(cell)
            self.frontier.append(cell)

    def _probe_neighbor(self, cell: Cell, neighbor: Cell, temps: Optional[Sequence[int]]) -> None:
        """Attempt the single-joint move from *cell* to *neighbor* and record it."""
        neighbor_config = cell_to_config(neighbor, self.spec)
        if neighbor_config in self.probed:
            return  # already probed (resume, or reached from another cell)

        joint = _differing_joint(cell, neighbor)
        if joint is None:  # pragma: no cover - neighbors always differ by one joint
            return

        bound_min, bound_max = self.spec.bounds[joint]
        result = self.move_fn(
            self.bus,
            motor=joint + 1,
            target=neighbor_config[joint],
            min_angle=bound_min,
            max_angle=bound_max,
            threshold=self.threshold,
            allow_motion=True,
        )
        self.budget.record_move()
        self.probed.add(neighbor_config)

        reachable = (not result["contacted"]) and (not result["overloaded"])
        self.step_no += 1
        append_event(
            self.log_path,
            ContactEvent(
                config=neighbor_config,
                moving_joint_index=joint,
                load_magnitude=_load_from_result(result),
                result=ContactResult.REACHABLE if reachable else ContactResult.BLOCKED,
                step=self.step_no,
            ),
        )

        if reachable:
            self.reachable += 1
            self._enqueue(neighbor)
        else:
            self._on_block(neighbor, joint)

    def _on_block(self, blocked_cell: Cell, blocked_joint: int) -> None:
        """A joint was blocked — record the contact and search for an escape."""
        self.contacts += 1
        self.escapes_attempted += 1
        path = escape(
            blocked_cell,
            blocked_joint,
            self.spec,
            self.budget,
            self.probe,
            max_depth=self.max_escape_depth,
            max_breadth=self.max_escape_breadth,
        )
        if path is not None:
            self.escapes_succeeded += 1
            self._enqueue(config_to_cell(path.freed_config, self.spec))

    def run(self) -> None:
        """Drive the bounded BFS until the frontier drains or the budget is spent."""
        while self.frontier:
            temps = _read_temperatures(self.temperatures)
            if not self.budget.should_continue(temps):
                self.budget_bounded = True
                break
            cell = self.frontier.popleft()
            for neighbor in neighbors(cell, self.spec):
                if not self.budget.should_continue(temps):
                    self.budget_bounded = True
                    self.frontier.clear()
                    break
                self._probe_neighbor(cell, neighbor, temps)


def explore(
    bus: object,
    spec: GridSpec,
    *,
    log_path: PathLike,
    map_path: PathLike,
    threshold: int = _DEFAULT_THRESHOLD,
    budget: Optional[Budget] = None,
    move_fn: MoveFn = gentle_move,
    max_escape_depth: int = DEFAULT_MAX_ESCAPE_DEPTH,
    max_escape_breadth: int = DEFAULT_MAX_ESCAPE_BREADTH,
    temperatures: Optional[TemperatureFn] = None,
) -> ExploreResult:
    """Flood-fill the reachable joint-space of *spec*, mapping it safely.

    Walks outward (breadth-first) from ``spec.origin``'s home cell, moving one
    joint per grid step toward each neighbouring cell via *move_fn* (the
    overload-safe :func:`~arm101.hardware.gentle.gentle_move` by default).
    Every attempt is recorded to the JSONL log at *log_path*; a blocked joint
    triggers the combination-:func:`~arm101.explore.escape.escape` search, whose
    freed configuration (if any) is enqueued so exploration continues past the
    obstruction. The whole run is bounded by *budget*. Finally the recorded
    events are folded into a compact :class:`~arm101.explore.types.ReachMap`,
    saved to *map_path*, and returned inside an :class:`ExploreResult`.

    Parameters
    ----------
    bus : object
        An OPEN :class:`~arm101.hardware.bus.MotorBus` (real or ``FakeBus``).
        The engine never drives it directly — it is passed straight through to
        *move_fn* (and the escape probe).
    spec : GridSpec
        The discretization: bucket sizes, per-joint bounds, and the origin/home
        configuration the walk starts from.
    log_path : str | Path
        Append-only JSONL event log. Existing lines are honoured for resume.
    map_path : str | Path
        Destination for the derived compact map (JSON).
    threshold : int, optional
        Contact-load threshold handed to *move_fn*. Defaults to
        :data:`_DEFAULT_THRESHOLD` (matching ``gentle_move``).
    budget : Budget, optional
        Shared run budget (moves / wall-time / thermal). A fresh default
        :class:`~arm101.explore.budget.Budget` is constructed when ``None``.
    move_fn : MoveFn, optional
        The injected physical move (see :data:`MoveFn`). Defaults to
        ``gentle_move``.
    max_escape_depth, max_escape_breadth : int, optional
        Caps forwarded to :func:`~arm101.explore.escape.escape`.
    temperatures : TemperatureFn, optional
        Optional live-temperature provider for the thermal guard (see
        :data:`TemperatureFn`). ``None`` (default) skips the thermal check and
        relies on the move/time caps.

    Returns
    -------
    ExploreResult
        The derived map plus a render-friendly summary. The map and the event
        log are also written to disk as the durable artifacts.
    """
    if budget is None:
        budget = Budget()

    walk = _Walk(
        bus,
        spec,
        log_path=log_path,
        threshold=threshold,
        budget=budget,
        move_fn=move_fn,
        max_escape_depth=max_escape_depth,
        max_escape_breadth=max_escape_breadth,
        temperatures=temperatures,
    )
    walk.run()

    reach_map = build_from_events(read_events(log_path))
    save_map(map_path, reach_map)

    return ExploreResult(
        reach_map=reach_map,
        cells_visited=len(walk.visited_cells),
        moves=budget.moves,
        reachable=walk.reachable,
        contacts=walk.contacts,
        escapes_attempted=walk.escapes_attempted,
        escapes_succeeded=walk.escapes_succeeded,
        budget_bounded=walk.budget_bounded,
        log_path=str(log_path),
        map_path=str(map_path),
    )


__all__ = [
    "MoveFn",
    "TemperatureFn",
    "PathLike",
    "ExploreResult",
    "explore",
]
