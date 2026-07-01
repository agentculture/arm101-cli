"""Tests for arm101.explore.escape — the deeper multi-joint combination-escape search.

TDD: written before arm101/explore/escape.py existed and drive the implementation.

Why this module is the risky one (plan risk r1)
------------------------------------------------
When walking the SO-101's reachable joint-space, a joint often gets BLOCKED at
some cell, but a small COORDINATED move of OTHER joints first can clear the
obstruction so the blocked joint can then advance (e.g. raise shoulder_lift a
little to free elbow_flex). ``escape()`` is the DEEPER, multi-joint coordinated
search that finds such escapes — not merely single-joint perturbation. Because
that search is combinatorial it MUST be pruned (depth/breadth caps) AND bounded
by the shared Budget so it ALWAYS terminates.

Everything here is hardware-free: the physical action is dependency-injected as
a ``Probe`` callable. Tests supply synthetic probe closures encoding a toy
obstruction model; the real engine (t8) supplies a probe backed by the
overload-safe ``bus.gentle_move``.

Covers:
* Zero-dep import (arm101.explore.escape imports with no third-party packages).
* Public API surface (Probe, ProbeResult, EscapePath, escape, default caps).
* An already-free joint yields a depth-0 EscapePath (no perturbation needed).
* A-blocked-until-B (depth 1): a single coordinated perturbation frees A; the
  returned path, when replayed, lets A advance under the synthetic probe.
* Multi-joint (depth >= 2) escape found where a single-joint perturbation alone
  would fail — the core value of the deeper search.
* No-escape fixture (blocked regardless of any perturbation) returns None
  within the caps and records only a bounded number of moves.
* Budget-exhaustion early-exit: a tiny move budget bails out, returns None, and
  never records more moves than the cap.
* Every probe records exactly one budget move (record_move parity).
* Determinism: identical inputs yield an identical EscapePath.
* blocked_joint out of range raises ValueError.
"""

from __future__ import annotations

import sys

import pytest

from arm101.explore.budget import Budget
from arm101.explore.grid import cell_to_config, home_cell
from arm101.explore.types import NUM_JOINTS, TICK_MAX, GridSpec, JointConfig

# ---------------------------------------------------------------------------
# Fixtures / helpers — hardware-free synthetic probes
# ---------------------------------------------------------------------------


def _spec(bucket=64):
    """A uniform GridSpec centered at mid-range (2048) with room to perturb up."""
    origin = JointConfig.from_ticks((2048,) * NUM_JOINTS)
    return GridSpec(bucket_size=(bucket,) * NUM_JOINTS, origin=origin)


def make_probe(blocked_joint, unlock, *, step=64, advanceable=None):
    """Build a synthetic ``Probe`` closure encoding a toy obstruction model.

    Semantics (a pure function of the *from* config — no hidden state, so it is
    deterministic and safely replayable):

    * Probing ``blocked_joint``: reachable iff ``unlock(config)`` is True; on
      success the blocked joint advances by ``+step`` (clamped to TICK_MAX),
      otherwise it stays put and reports blocked.
    * Probing any other joint ``j``: reachable (advances ``+step``) iff ``j`` is
      in ``advanceable`` and there is head-room under TICK_MAX; otherwise blocked
      and unchanged.

    ``advanceable`` defaults to every joint except ``blocked_joint``.
    """
    from arm101.explore.escape import ProbeResult

    if advanceable is None:
        advanceable = tuple(j for j in range(NUM_JOINTS) if j != blocked_joint)
    advanceable = frozenset(advanceable)

    def probe(config, joint):
        ticks = list(config.ticks)
        if joint == blocked_joint:
            if unlock(config):
                ticks[joint] = min(TICK_MAX, ticks[joint] + step)
                return ProbeResult(reachable=True, position=JointConfig.from_ticks(ticks))
            return ProbeResult(reachable=False, position=config)
        if joint in advanceable and ticks[joint] + step <= TICK_MAX:
            ticks[joint] += step
            return ProbeResult(reachable=True, position=JointConfig.from_ticks(ticks))
        return ProbeResult(reachable=False, position=config)

    return probe


class CountingProbe:
    """Wrap a probe and count how many times it is invoked (record_move parity)."""

    def __init__(self, inner):
        self.inner = inner
        self.calls = 0

    def __call__(self, config, joint):
        self.calls += 1
        return self.inner(config, joint)


def _replay(path, blocked_config, blocked_joint, probe):
    """Replay an EscapePath's perturbations from *blocked_config*, then advance
    the blocked joint. Returns the final ProbeResult of the blocked joint.

    Asserts along the way that each recorded step starts from the config the
    replay is actually at and that each perturbation is reachable — i.e. the
    path is a real, applicable sequence of moves.
    """
    current = blocked_config
    for from_config, joint in path.steps:
        assert from_config == current, "escape step does not start from the replayed config"
        res = probe(from_config, joint)
        assert res.reachable, "escape step perturbation was not actually reachable"
        current = res.position
    assert current == path.freed_config, "replayed perturbations did not reach freed_config"
    return probe(current, blocked_joint)


# ---------------------------------------------------------------------------
# 1. Zero-dep import + public API surface
# ---------------------------------------------------------------------------


def test_import_arm101_explore_escape_zero_deps():
    """import arm101.explore.escape must work with no third-party packages installed."""
    import arm101.explore.escape  # noqa: F401

    assert "arm101.explore.escape" in sys.modules


def test_public_api_surface():
    """The module exposes the injection types, the search entrypoint, and default caps."""
    from arm101.explore import escape as mod

    for name in (
        "Probe",
        "ProbeResult",
        "EscapePath",
        "escape",
        "DEFAULT_MAX_ESCAPE_DEPTH",
        "DEFAULT_MAX_ESCAPE_BREADTH",
    ):
        assert hasattr(mod, name), f"missing public name: {name}"
    # Caps must be positive integers so the search is finitely bounded.
    assert isinstance(mod.DEFAULT_MAX_ESCAPE_DEPTH, int) and mod.DEFAULT_MAX_ESCAPE_DEPTH >= 1
    assert isinstance(mod.DEFAULT_MAX_ESCAPE_BREADTH, int) and mod.DEFAULT_MAX_ESCAPE_BREADTH >= 1


def test_probe_result_fields():
    """ProbeResult carries (reachable: bool, position: JointConfig)."""
    from arm101.explore.escape import ProbeResult

    cfg = JointConfig.from_ticks((2048,) * NUM_JOINTS)
    res = ProbeResult(reachable=True, position=cfg)
    assert res.reachable is True
    assert res.position == cfg


def test_escape_path_fields_and_depth():
    """EscapePath carries ordered (from_config, joint) steps + the freed_config, with depth."""
    from arm101.explore.escape import EscapePath

    a = JointConfig.from_ticks((2048,) * NUM_JOINTS)
    b = JointConfig.from_ticks((2112,) + (2048,) * (NUM_JOINTS - 1))
    path = EscapePath(steps=((a, 0),), freed_config=b)
    assert path.steps == ((a, 0),)
    assert path.freed_config == b
    assert path.depth == 1
    assert EscapePath(steps=(), freed_config=a).depth == 0


# ---------------------------------------------------------------------------
# 2. Degenerate: already free -> depth-0 path
# ---------------------------------------------------------------------------


def test_already_free_returns_depth_zero_path():
    """If the 'blocked' joint can actually advance immediately, escape needs no
    perturbation and returns a depth-0 EscapePath at the blocked config."""
    from arm101.explore.escape import escape

    spec = _spec()
    blocked_joint = 2
    cell = home_cell(spec)
    blocked_config = cell_to_config(cell, spec)
    probe = make_probe(blocked_joint, unlock=lambda c: True)  # never actually blocked
    budget = Budget(max_moves=1000)

    path = escape(cell, blocked_joint, spec, budget, probe)
    assert path is not None
    assert path.depth == 0
    assert path.freed_config == blocked_config


# ---------------------------------------------------------------------------
# 3. A-blocked-until-B: a single coordinated perturbation frees A (depth 1)
# ---------------------------------------------------------------------------


def test_single_joint_perturbation_frees_blocked_joint():
    """Joint A (elbow_flex, idx 2) is blocked until joint B (shoulder_lift, idx 1)
    is nudged up once; escape returns the freed-A path and replaying it advances A."""
    from arm101.explore.escape import escape

    spec = _spec()
    blocked_joint = 2  # elbow_flex
    cell = home_cell(spec)
    blocked_config = cell_to_config(cell, spec)  # all 2048

    # A is free only once shoulder_lift (idx 1) has advanced past its start tick.
    unlock = lambda c: c[1] >= 2048 + 64  # noqa: E731
    probe = make_probe(blocked_joint, unlock, advanceable=(1,))
    budget = Budget(max_moves=1000)

    path = escape(cell, blocked_joint, spec, budget, probe)
    assert path is not None
    assert path.depth == 1
    # The single perturbation was of joint 1 (shoulder_lift), from the blocked config.
    assert path.steps[0][0] == blocked_config
    assert path.steps[0][1] == 1

    # Replaying the path must actually free A under the same synthetic probe.
    freed = _replay(path, blocked_config, blocked_joint, probe)
    assert freed.reachable is True


# ---------------------------------------------------------------------------
# 4. Multi-joint (depth >= 2) escape where single-joint alone fails — core value
# ---------------------------------------------------------------------------


def test_multi_joint_depth2_escape_found_where_single_joint_fails():
    """A needs BOTH joint 0 and joint 1 nudged up before it frees — a single
    coordinated step can never free it, so a depth-1 search MUST return None while
    the deeper (depth >= 2) search finds the freed-A path. This is the whole point
    of the combination-escape search over single-joint perturbation."""
    from arm101.explore.escape import escape

    spec = _spec()
    blocked_joint = 2
    cell = home_cell(spec)
    blocked_config = cell_to_config(cell, spec)

    # A frees only when joints 0 AND 1 have each advanced once — two coordinated steps.
    unlock = lambda c: c[0] >= 2048 + 64 and c[1] >= 2048 + 64  # noqa: E731

    # Depth-1 search: single-joint perturbation alone cannot free A.
    probe1 = make_probe(blocked_joint, unlock, advanceable=(0, 1))
    budget1 = Budget(max_moves=1000)
    assert escape(cell, blocked_joint, spec, budget1, probe1, max_depth=1) is None

    # Full (deeper) search finds the coordinated two-step escape.
    probe2 = make_probe(blocked_joint, unlock, advanceable=(0, 1))
    budget2 = Budget(max_moves=1000)
    path = escape(cell, blocked_joint, spec, budget2, probe2, max_depth=3)
    assert path is not None
    assert path.depth == 2, "coordinated escape must require two perturbation steps"
    # Two DIFFERENT joints were perturbed (genuinely multi-joint, not one joint twice).
    moved = {joint for _cfg, joint in path.steps}
    assert moved == {0, 1}

    freed = _replay(path, blocked_config, blocked_joint, probe2)
    assert freed.reachable is True


# ---------------------------------------------------------------------------
# 5. No-escape: blocked regardless of any perturbation -> None, bounded moves
# ---------------------------------------------------------------------------


def test_no_escape_returns_none_within_caps_and_bounded_moves():
    """When A stays blocked no matter what other joints do, escape terminates,
    returns None, and records only a bounded number of moves (the depth/breadth
    caps bound the search independently of the Budget)."""
    from arm101.explore.escape import escape

    spec = _spec()
    blocked_joint = 2
    cell = home_cell(spec)

    unlock = lambda c: False  # noqa: E731  — never freeable
    probe = make_probe(blocked_joint, unlock)
    # A deliberately huge budget so ONLY the depth/breadth caps can stop the search.
    budget = Budget(max_moves=10**9)

    max_depth, max_breadth = 2, 2
    path = escape(
        cell, blocked_joint, spec, budget, probe, max_depth=max_depth, max_breadth=max_breadth
    )
    assert path is None
    assert budget.moves > 0

    # Loose analytic upper bound: 1 base probe + per node up to (all joints probed
    # once + a retry each). Nodes ever expanded <= 1 + Bd + Bd^2 + ...
    perturb_joints = NUM_JOINTS - 1
    nodes = sum(max_breadth**k for k in range(max_depth + 1))
    upper_bound = 1 + 2 * perturb_joints * nodes
    assert budget.moves <= upper_bound


def test_no_escape_terminates_even_at_default_caps():
    """The default caps alone (no tight budget) still make a no-escape search halt."""
    from arm101.explore.escape import escape

    spec = _spec()
    blocked_joint = 0
    cell = home_cell(spec)
    probe = make_probe(blocked_joint, unlock=lambda c: False)
    budget = Budget(max_moves=10**9)

    path = escape(cell, blocked_joint, spec, budget, probe)
    assert path is None
    assert budget.moves > 0  # it did real work, then stopped by the caps


# ---------------------------------------------------------------------------
# 6. Budget-exhaustion early-exit
# ---------------------------------------------------------------------------


def test_budget_exhaustion_bails_out_early():
    """A tiny move budget stops the search early: escape returns None and never
    records more moves than the cap (the shared Budget bounds the search)."""
    from arm101.explore.escape import escape

    spec = _spec()
    blocked_joint = 2
    cell = home_cell(spec)
    probe = make_probe(blocked_joint, unlock=lambda c: False)
    budget = Budget(max_moves=2)  # far too small to explore

    path = escape(cell, blocked_joint, spec, budget, probe, max_depth=5, max_breadth=5)
    assert path is None
    assert budget.moves <= 2


# ---------------------------------------------------------------------------
# 7. record_move parity — every probe records exactly one budget move
# ---------------------------------------------------------------------------


def test_every_probe_records_exactly_one_move():
    """The number of probe invocations equals budget.moves — no probe skips
    record_move and none double-counts."""
    from arm101.explore.escape import escape

    spec = _spec()
    blocked_joint = 2
    cell = home_cell(spec)
    unlock = lambda c: c[0] >= 2048 + 64 and c[1] >= 2048 + 64  # noqa: E731
    probe = CountingProbe(make_probe(blocked_joint, unlock, advanceable=(0, 1)))
    budget = Budget(max_moves=1000)

    escape(cell, blocked_joint, spec, budget, probe, max_depth=3)
    assert probe.calls == budget.moves
    assert probe.calls > 0


# ---------------------------------------------------------------------------
# 8. Determinism
# ---------------------------------------------------------------------------


def test_search_is_deterministic():
    """Identical inputs + probe yield an identical EscapePath (stable ordering —
    no reliance on set iteration order or randomness)."""
    from arm101.explore.escape import escape

    spec = _spec()
    blocked_joint = 2
    cell = home_cell(spec)
    unlock = lambda c: c[0] >= 2048 + 64 and c[1] >= 2048 + 64  # noqa: E731

    def run():
        probe = make_probe(blocked_joint, unlock, advanceable=(0, 1))
        budget = Budget(max_moves=1000)
        return escape(cell, blocked_joint, spec, budget, probe, max_depth=3)

    first = run()
    second = run()
    assert first is not None
    assert first == second


# ---------------------------------------------------------------------------
# 9. Validation
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("bad_joint", [-1, NUM_JOINTS, NUM_JOINTS + 3])
def test_blocked_joint_out_of_range_raises(bad_joint):
    """A blocked_joint index outside [0, NUM_JOINTS) is rejected up front."""
    from arm101.explore.escape import escape

    spec = _spec()
    cell = home_cell(spec)
    probe = make_probe(0, unlock=lambda c: False)
    budget = Budget(max_moves=10)

    with pytest.raises(ValueError):
        escape(cell, bad_joint, spec, budget, probe)
