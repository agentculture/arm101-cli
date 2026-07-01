"""arm101.explore.escape — the deeper multi-joint combination-escape search.

This is the riskiest, most combinatorial piece of ``arm101 arm explore`` (plan
risk r1). While walking the SO-101's reachable joint-space, a joint frequently
gets BLOCKED at some cell — it cannot advance because another joint is in the
way. Often a small COORDINATED move of OTHER joints FIRST clears the obstruction
so the blocked joint can then advance (e.g. raise ``shoulder_lift`` a little to
free ``elbow_flex``). :func:`escape` is the search that hunts for such a
coordinated escape.

The key word is *coordinated*: this is not merely single-joint perturbation. The
search explores sequences of perturbations of the OTHER joints — a bounded
breadth-first walk over multi-joint perturbation vectors — retrying the blocked
joint after each candidate. Because that space is combinatorial it is pruned
hard and bounded so it ALWAYS terminates:

* **depth cap** (``max_depth``) — how many coordinated perturbation steps deep
  the search may go (path length);
* **breadth cap** (``max_breadth``) — how many child perturbations are expanded
  per node (branching factor);
* **the shared :class:`~arm101.explore.budget.Budget`** — every probe both
  records a move (:meth:`Budget.record_move`) and is gated on
  :meth:`Budget.exhausted`, so a spent global budget ends the search immediately
  even mid-level.

Hardware-free by construction — dependency injection
----------------------------------------------------
The physical action is injected as a :data:`Probe` callable so the search is
fully testable with no hardware. A probe, given a *from* configuration and the
index of the joint to move, attempts that one move and reports a
:class:`ProbeResult` (did it advance, and where the arm ended up). The real
engine (t8) supplies a probe backed by the overload-safe ``bus.gentle_move``;
tests supply synthetic closures encoding a toy obstruction model. This module
never imports :mod:`arm101.hardware` — consistent with the rest of
``arm101.explore`` (only ``engine.py`` talks to the bus). Zero third-party
imports (stdlib only: ``typing``).

Determinism
-----------
Given the same inputs and probe, the search is deterministic: joints are always
tried in ascending index order, the frontier is processed in insertion order,
and the visited-set is used only for membership (never iterated for decisions).
There is no randomness and no wall-clock dependence in the search itself.

Open question (plan risk r1)
----------------------------
The exact default caps (:data:`DEFAULT_MAX_ESCAPE_DEPTH`,
:data:`DEFAULT_MAX_ESCAPE_BREADTH`) are a **hardware-tuned open question**. They
are chosen conservatively — deep/wide enough to find realistic two-to-three
joint escapes, small enough to keep the probe count (and therefore physical
motion) modest — but are not derived from a benchmark. Expect them to be
revisited once real exploration runs show how deep useful escapes actually are.
The single-direction-per-joint probe model (a probe advances a joint one step in
its exploration direction) is likewise a first cut: an escape that needs a joint
moved the *other* way is out of scope for this search and is left to the engine's
choice of probe direction at the frontier.
"""

from __future__ import annotations

from typing import Callable, List, NamedTuple, Optional, Tuple

from arm101.explore.budget import Budget
from arm101.explore.grid import Cell, cell_to_config
from arm101.explore.types import NUM_JOINTS, GridSpec, JointConfig

# ---------------------------------------------------------------------------
# Default cap constants (plan risk r1 — hardware-tuned open question)
# ---------------------------------------------------------------------------

#: Default cap on how many coordinated perturbation steps deep the search may go
#: (the maximum length of a returned :class:`EscapePath`). Deep enough for the
#: common two-to-three-joint escapes; small enough to bound the probe count.
DEFAULT_MAX_ESCAPE_DEPTH: int = 3

#: Default cap on how many child perturbations are expanded per search node (the
#: branching factor). With 5 non-blocked joints this still prunes the tree while
#: leaving room to try the joints most likely to help first.
DEFAULT_MAX_ESCAPE_BREADTH: int = 4


# ---------------------------------------------------------------------------
# Dependency-injection types (the hardware seam)
# ---------------------------------------------------------------------------


class ProbeResult(NamedTuple):
    """Outcome of a single injected probe move.

    Attributes
    ----------
    reachable : bool
        ``True`` if the moving joint advanced, ``False`` if it was blocked /
        made contact and did not move.
    position : JointConfig
        The full joint configuration after the attempt — the moving joint's new
        position when ``reachable``, or the unchanged *from* config when blocked.
    """

    reachable: bool
    position: JointConfig


#: The injected physical action. Given a *from* :class:`JointConfig` and the
#: index (0..``NUM_JOINTS``-1) of the joint to move, attempt that one move and
#: report a :class:`ProbeResult`. The real engine backs this with the
#: overload-safe ``bus.gentle_move``; tests back it with a synthetic closure.
Probe = Callable[[JointConfig, int], ProbeResult]


class EscapePath(NamedTuple):
    """A found coordinated escape: the perturbations that free the blocked joint.

    Attributes
    ----------
    steps : tuple[tuple[JointConfig, int], ...]
        The ordered coordinated perturbations, each a ``(from_config,
        moving_joint)`` pair — from ``from_config``, moving ``moving_joint`` one
        step advances the arm toward freeing the blocked joint. The first step
        starts from the blocked configuration; each subsequent step starts from
        the previous step's resulting config. An empty tuple means the blocked
        joint was already free (no perturbation needed).
    freed_config : JointConfig
        The configuration reached after applying every step — the config from
        which the blocked joint successfully advances.
    """

    steps: Tuple[Tuple[JointConfig, int], ...]
    freed_config: JointConfig

    @property
    def depth(self) -> int:
        """Number of coordinated perturbation steps (``len(self.steps)``)."""
        return len(self.steps)


# ---------------------------------------------------------------------------
# The search
# ---------------------------------------------------------------------------


def escape(
    blocked_cell: Cell,
    blocked_joint: int,
    spec: GridSpec,
    budget: Budget,
    probe: Probe,
    *,
    max_depth: int = DEFAULT_MAX_ESCAPE_DEPTH,
    max_breadth: int = DEFAULT_MAX_ESCAPE_BREADTH,
) -> Optional[EscapePath]:
    """Search for a coordinated multi-joint escape that frees ``blocked_joint``.

    Starting from the configuration of ``blocked_cell``, run a bounded
    breadth-first search over sequences of perturbations of the OTHER joints,
    retrying ``blocked_joint`` after each candidate perturbation. Breadth-first
    so the *shallowest* (gentlest, fewest-move) escape is returned first.

    Every probe records one move on ``budget`` and is gated on
    ``budget.exhausted()``, and the search is capped at ``max_depth``
    coordinated steps and ``max_breadth`` children per node — so it ALWAYS
    terminates, whichever bound bites first.

    Parameters
    ----------
    blocked_cell : Cell
        The grid cell at which ``blocked_joint`` is stuck. Its representative
        configuration (via :func:`~arm101.explore.grid.cell_to_config`) is the
        search origin.
    blocked_joint : int
        Index (0..``NUM_JOINTS``-1) of the joint to free.
    spec : GridSpec
        The discretization; used to resolve ``blocked_cell`` to a config.
    budget : Budget
        The shared run budget. Gates and counts every probe.
    probe : Probe
        The injected physical action (see :data:`Probe`).
    max_depth : int, optional
        Maximum coordinated perturbation steps (path length). Defaults to
        :data:`DEFAULT_MAX_ESCAPE_DEPTH`.
    max_breadth : int, optional
        Maximum child perturbations expanded per node. Defaults to
        :data:`DEFAULT_MAX_ESCAPE_BREADTH`.

    Returns
    -------
    EscapePath or None
        An :class:`EscapePath` whose steps, when applied, let ``blocked_joint``
        advance — possibly a depth-0 path if the joint was already free. ``None``
        if no escape is found within the caps or before the budget is spent.

    Raises
    ------
    ValueError
        If ``blocked_joint`` is outside ``[0, NUM_JOINTS)``.
    """
    if not (0 <= blocked_joint < NUM_JOINTS):
        raise ValueError(f"blocked_joint {blocked_joint} out of range [0, {NUM_JOINTS - 1}].")

    blocked_config = cell_to_config(blocked_cell, spec)
    # Fixed, deterministic order: every non-blocked joint, ascending index.
    perturb_joints: Tuple[int, ...] = tuple(j for j in range(NUM_JOINTS) if j != blocked_joint)

    def try_probe(config: JointConfig, joint: int) -> Optional[ProbeResult]:
        """Gate on the budget, then probe. Returns None if the budget is spent
        (the search must stop), else records a move and returns the result."""
        if budget.exhausted() or not budget.should_continue():
            return None
        budget.record_move()
        return probe(config, joint)

    # Base case: is the joint actually blocked here at all?
    base = try_probe(blocked_config, blocked_joint)
    if base is None:
        return None
    if base.reachable:
        return EscapePath(steps=(), freed_config=blocked_config)

    # Bounded BFS over coordinated perturbation sequences. A frontier node is
    # (config, steps): the config reached so far and the perturbations taken to
    # get there. ``visited`` dedups configs (JointConfig is hashable) so cyclic /
    # order-equivalent paths are not re-expanded.
    frontier: List[Tuple[JointConfig, Tuple[Tuple[JointConfig, int], ...]]] = [(blocked_config, ())]
    visited = {blocked_config}

    for _depth in range(max_depth):
        if not frontier:
            break
        next_frontier: List[Tuple[JointConfig, Tuple[Tuple[JointConfig, int], ...]]] = []
        for config, steps in frontier:
            children = 0
            for joint in perturb_joints:
                if children >= max_breadth:
                    break
                # Attempt the coordinated perturbation of this other joint.
                pert = try_probe(config, joint)
                if pert is None:
                    return None  # budget spent — stop, deterministically, now
                if not pert.reachable:
                    continue  # this joint cannot move from here — dead end
                new_config = pert.position
                if new_config in visited:
                    continue  # already reached by a shallower/earlier path
                visited.add(new_config)
                children += 1
                new_steps = steps + ((config, joint),)
                # Retry the blocked joint from the freshly perturbed config.
                retry = try_probe(new_config, blocked_joint)
                if retry is None:
                    return None  # budget spent
                if retry.reachable:
                    return EscapePath(steps=new_steps, freed_config=new_config)
                next_frontier.append((new_config, new_steps))
        frontier = next_frontier

    return None


__all__ = [
    "DEFAULT_MAX_ESCAPE_DEPTH",
    "DEFAULT_MAX_ESCAPE_BREADTH",
    "ProbeResult",
    "Probe",
    "EscapePath",
    "escape",
]
