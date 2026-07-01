"""Tests for arm101.explore.budget — the run budget + thermal guard.

TDD: written before arm101/explore/budget.py existed and drive the implementation.

The Budget is what GUARANTEES an `arm101 arm explore` run terminates: the
combinatorial escape search over joint-space could otherwise run forever, so
every long-running loop must be driven by ``Budget.should_continue()``.

Covers:
* Zero-dep import (arm101.explore.budget imports with no third-party packages).
* Default cap constants exist and are conservative (module-level).
* record_move() increments an internal move counter.
* exhausted() reports True at the move cap, False before it.
* exhausted() reports True at the wall-time cap (driven by an INJECTED fake
  clock — never real ``time.sleep``), False before it.
* check_thermal() halts (True) when any joint temperature exceeds the
  ceiling, and does not halt (False) at-or-under the ceiling.
* should_continue() combines exhausted()+check_thermal(); temperatures=None
  means "don't consult the thermal guard".
* A loop driven solely by Budget provably terminates: one test bounded by
  max_moves, one test bounded by max_seconds via a fake clock. Both loops
  have no other exit condition, so a hang here means Budget failed to bound
  the loop.
"""

from __future__ import annotations

import sys
import time

import pytest

# ---------------------------------------------------------------------------
# Fake-clock helpers (deterministic wall-time control for tests)
# ---------------------------------------------------------------------------


def _sequence_clock(times):
    """Return a zero-arg clock callable that yields *times* in order, one per call."""
    values = iter(times)
    return lambda: next(values)


def _frozen_clock(value=0.0):
    """Return a zero-arg clock callable that always returns *value* (time never advances)."""
    return lambda: value


# ---------------------------------------------------------------------------
# 1. Zero-dep import guarantee
# ---------------------------------------------------------------------------


def test_import_arm101_explore_budget_zero_deps():
    """import arm101.explore.budget must work with no third-party packages installed."""
    import arm101.explore.budget  # noqa: F401

    assert "arm101.explore.budget" in sys.modules


# ---------------------------------------------------------------------------
# 2. Default cap constants
# ---------------------------------------------------------------------------


def test_default_cap_constants_are_conservative():
    """Default caps exist, are positive, and the thermal ceiling stays safely
    under the STS3215 Max_Temp of 70C (frame risk r3: exact numbers are a
    hardware-tuned open question, but the defaults must be conservative)."""
    from arm101.explore.budget import (
        DEFAULT_MAX_MOVES,
        DEFAULT_MAX_SECONDS,
        DEFAULT_MAX_TEMPERATURE_C,
    )

    assert DEFAULT_MAX_MOVES > 0
    assert DEFAULT_MAX_SECONDS > 0
    assert 0 < DEFAULT_MAX_TEMPERATURE_C < 70


# ---------------------------------------------------------------------------
# 3. Construction + defaults
# ---------------------------------------------------------------------------


def test_budget_constructs_with_defaults():
    """A Budget with no args uses the module default caps and the real clock."""
    from arm101.explore.budget import (
        DEFAULT_MAX_MOVES,
        DEFAULT_MAX_SECONDS,
        DEFAULT_MAX_TEMPERATURE_C,
        Budget,
    )

    budget = Budget()
    assert budget.max_moves == DEFAULT_MAX_MOVES
    assert budget.max_seconds == DEFAULT_MAX_SECONDS
    assert budget.max_temperature_c == DEFAULT_MAX_TEMPERATURE_C
    # Default clock is real wall time: elapsed_seconds is a small non-negative float.
    assert isinstance(budget.elapsed_seconds, float)
    assert budget.elapsed_seconds >= 0.0


def test_budget_accepts_injected_clock_and_custom_caps():
    """Custom caps and an injected clock are stored as given."""
    from arm101.explore.budget import Budget

    clock = _frozen_clock(0.0)
    budget = Budget(max_moves=10, max_seconds=5.0, max_temperature_c=55, clock=clock)
    assert budget.max_moves == 10
    assert budget.max_seconds == 5.0
    assert budget.max_temperature_c == 55


# ---------------------------------------------------------------------------
# 4. record_move() / moves counter
# ---------------------------------------------------------------------------


def test_record_move_increments_counter():
    from arm101.explore.budget import Budget

    budget = Budget(clock=_frozen_clock(0.0))
    assert budget.moves == 0
    budget.record_move()
    assert budget.moves == 1
    budget.record_move()
    budget.record_move()
    assert budget.moves == 3


# ---------------------------------------------------------------------------
# 5. exhausted() — move cap
# ---------------------------------------------------------------------------


def test_exhausted_false_before_move_cap():
    from arm101.explore.budget import Budget

    budget = Budget(max_moves=3, max_seconds=1000.0, clock=_frozen_clock(0.0))
    budget.record_move()
    budget.record_move()
    assert budget.exhausted() is False


def test_exhausted_true_at_move_cap():
    from arm101.explore.budget import Budget

    budget = Budget(max_moves=3, max_seconds=1000.0, clock=_frozen_clock(0.0))
    budget.record_move()
    budget.record_move()
    budget.record_move()
    assert budget.exhausted() is True


def test_exhausted_true_beyond_move_cap():
    from arm101.explore.budget import Budget

    budget = Budget(max_moves=1, max_seconds=1000.0, clock=_frozen_clock(0.0))
    budget.record_move()
    budget.record_move()
    budget.record_move()
    assert budget.exhausted() is True


# ---------------------------------------------------------------------------
# 6. exhausted() — wall-time cap (injected fake clock, never real sleep)
# ---------------------------------------------------------------------------


def test_exhausted_false_before_time_cap():
    from arm101.explore.budget import Budget

    # Start time consumes the first value (0.0); the exhausted() check consumes
    # the second (5.0) -> elapsed 5.0s, under the 10s cap.
    clock = _sequence_clock([0.0, 5.0])
    budget = Budget(max_moves=1_000_000, max_seconds=10.0, clock=clock)
    assert budget.exhausted() is False


def test_exhausted_true_at_time_cap():
    from arm101.explore.budget import Budget

    # Start time = 0.0, then elapsed check lands exactly on the 10s cap.
    clock = _sequence_clock([0.0, 10.0])
    budget = Budget(max_moves=1_000_000, max_seconds=10.0, clock=clock)
    assert budget.exhausted() is True


def test_exhausted_true_beyond_time_cap():
    from arm101.explore.budget import Budget

    clock = _sequence_clock([0.0, 11.0])
    budget = Budget(max_moves=1_000_000, max_seconds=10.0, clock=clock)
    assert budget.exhausted() is True


def test_elapsed_seconds_reflects_injected_clock():
    from arm101.explore.budget import Budget

    clock = _sequence_clock([100.0, 107.5])
    budget = Budget(clock=clock)
    assert budget.elapsed_seconds == pytest.approx(7.5)


# ---------------------------------------------------------------------------
# 7. check_thermal()
# ---------------------------------------------------------------------------


def test_check_thermal_false_when_all_under_ceiling():
    from arm101.explore.budget import Budget

    budget = Budget(max_temperature_c=60, clock=_frozen_clock(0.0))
    assert budget.check_thermal([30, 40, 50, 45, 20, 35]) is False


def test_check_thermal_false_at_exact_ceiling():
    """Meeting the ceiling exactly does not halt — only *exceeding* it does."""
    from arm101.explore.budget import Budget

    budget = Budget(max_temperature_c=60, clock=_frozen_clock(0.0))
    assert budget.check_thermal([60, 60, 60, 60, 60, 60]) is False


def test_check_thermal_true_when_any_joint_exceeds_ceiling():
    from arm101.explore.budget import Budget

    budget = Budget(max_temperature_c=60, clock=_frozen_clock(0.0))
    # Only the 4th joint (wrist_flex) is over; the rest are comfortably under.
    assert budget.check_thermal([30, 40, 50, 61, 20, 35]) is True


def test_check_thermal_false_for_empty_temperatures():
    from arm101.explore.budget import Budget

    budget = Budget(max_temperature_c=60, clock=_frozen_clock(0.0))
    assert budget.check_thermal([]) is False


# ---------------------------------------------------------------------------
# 8. should_continue()
# ---------------------------------------------------------------------------


def test_should_continue_true_when_nothing_exhausted_and_no_temps_given():
    from arm101.explore.budget import Budget

    budget = Budget(max_moves=5, max_seconds=1000.0, clock=_frozen_clock(0.0))
    assert budget.should_continue() is True


def test_should_continue_false_when_moves_exhausted():
    from arm101.explore.budget import Budget

    budget = Budget(max_moves=1, max_seconds=1000.0, clock=_frozen_clock(0.0))
    budget.record_move()
    assert budget.should_continue() is False


def test_should_continue_ignores_thermal_when_temperatures_is_none():
    from arm101.explore.budget import Budget

    budget = Budget(max_moves=5, max_seconds=1000.0, max_temperature_c=60, clock=_frozen_clock(0.0))
    assert budget.should_continue(None) is True


def test_should_continue_false_when_thermal_over_ceiling():
    from arm101.explore.budget import Budget

    budget = Budget(max_moves=5, max_seconds=1000.0, max_temperature_c=60, clock=_frozen_clock(0.0))
    assert budget.should_continue([65, 30, 30, 30, 30, 30]) is False


def test_should_continue_true_when_thermal_under_ceiling():
    from arm101.explore.budget import Budget

    budget = Budget(max_moves=5, max_seconds=1000.0, max_temperature_c=60, clock=_frozen_clock(0.0))
    assert budget.should_continue([30, 30, 30, 30, 30, 30]) is True


# ---------------------------------------------------------------------------
# 9. Terminating-loop tests — the load-bearing guarantee
# ---------------------------------------------------------------------------


def test_move_bound_loop_terminates_at_move_cap():
    """A loop with NO exit condition other than the Budget must stop at max_moves."""
    from arm101.explore.budget import Budget

    budget = Budget(max_moves=5, max_seconds=1000.0, clock=_frozen_clock(0.0))
    moves_taken = 0
    while budget.should_continue():
        budget.record_move()
        moves_taken += 1
    assert moves_taken == 5
    assert budget.exhausted() is True


def test_time_bound_loop_terminates_at_time_cap():
    """A loop with NO exit condition other than the Budget must stop once the
    (fake) clock crosses max_seconds — proves the wall-time cap alone bounds
    a loop, independent of the move cap."""
    from arm101.explore.budget import Budget

    # Each call to should_continue() consumes one more clock reading: the
    # constructor consumes the first (0.0), and each exhausted() check inside
    # should_continue() consumes the next. Advancing by 1.0s per check
    # guarantees the loop exits once elapsed >= 3.0s.
    clock = _sequence_clock([float(i) for i in range(0, 1000)])
    budget = Budget(max_moves=1_000_000, max_seconds=3.0, clock=clock)
    moves_taken = 0
    while budget.should_continue():
        budget.record_move()
        moves_taken += 1
    assert budget.exhausted() is True
    # It must have stopped well short of the (effectively infinite) move cap.
    assert moves_taken < 1_000_000


def test_default_clock_is_time_monotonic():
    """The constructor's default clock parameter is time.monotonic (not e.g. time.time)."""
    import inspect

    from arm101.explore.budget import Budget

    sig = inspect.signature(Budget.__init__)
    assert sig.parameters["clock"].default is time.monotonic
