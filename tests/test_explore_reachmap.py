"""Tests for arm101.explore.reachmap — the compact, offline reachability map.

TDD contract for task t4. Covers:

* ``build_from_events`` — per-joint reachable ``(min, max)`` range derivation
  across REACHABLE events, plus the sparse, deduped, deterministically ordered
  set of BLOCKED joint-combinations.
* the no-reachable-observation edge case → the inverted ``EMPTY_RANGE`` sentinel.
* ``is_reachable`` — the pure, offline query: in-range-clean → True,
  in-range-but-blocked → False, out-of-range → False; and a subprocess proof
  that the query path imports no ``arm101.hardware`` code / opens no serial port.
* ``save_map`` / ``load_map_file`` — exact JSON round-trip via ``tmp_path``.
"""

from __future__ import annotations

import subprocess  # nosec B404 - used only to prove the query imports no hardware
import sys

import pytest

from arm101.explore import reachmap
from arm101.explore.reachmap import (
    EMPTY_RANGE,
    build_from_events,
    is_reachable,
    load_map_file,
    save_map,
)
from arm101.explore.types import (
    NUM_JOINTS,
    TICK_MAX,
    TICK_MIN,
    ContactEvent,
    ContactResult,
    JointConfig,
    ReachMap,
)


def _cfg(*ticks: int) -> JointConfig:
    """Build a JointConfig from 6 ticks."""
    return JointConfig.from_ticks(ticks)


def _reachable(config: JointConfig, moving: int = 0) -> ContactEvent:
    return ContactEvent(config, moving, 0, ContactResult.REACHABLE)


def _blocked(config: JointConfig, moving: int = 0, load: int = 40) -> ContactEvent:
    return ContactEvent(config, moving, load, ContactResult.BLOCKED)


# ---------------------------------------------------------------------------
# build_from_events — reachable range derivation
# ---------------------------------------------------------------------------


def test_build_from_events_derives_per_joint_min_max_from_reachable():
    events = [
        _reachable(_cfg(100, 200, 300, 400, 500, 600)),
        _reachable(_cfg(150, 100, 350, 400, 450, 600)),
        # a blocked event whose ticks must NOT widen the reachable ranges
        _blocked(_cfg(4000, 4000, 4000, 4000, 4000, 4000)),
    ]
    rm = build_from_events(events)
    assert isinstance(rm, ReachMap)
    assert rm.reachable_ranges == (
        (100, 150),
        (100, 200),
        (300, 350),
        (400, 400),
        (450, 500),
        (600, 600),
    )


def test_build_from_events_collects_blocked_configs():
    b1 = _cfg(4000, 10, 10, 10, 10, 10)
    b2 = _cfg(20, 4000, 20, 20, 20, 20)
    events = [
        _reachable(_cfg(100, 100, 100, 100, 100, 100)),
        _blocked(b1),
        _blocked(b2),
    ]
    rm = build_from_events(events)
    assert rm.blocked == (b1, b2)


def test_build_from_events_dedups_blocked_preserving_first_seen_order():
    b1 = _cfg(4000, 10, 10, 10, 10, 10)
    b2 = _cfg(20, 4000, 20, 20, 20, 20)
    events = [
        _blocked(b2),
        _blocked(b1),
        _blocked(b2),  # duplicate of the first-seen b2
        _blocked(b1),  # duplicate of b1
    ]
    rm = build_from_events(events)
    # deduped, and ordered by first appearance (b2 then b1)
    assert rm.blocked == (b2, b1)


def test_build_from_events_accepts_a_generator_iterable_once():
    def gen():
        yield _reachable(_cfg(100, 100, 100, 100, 100, 100))
        yield _reachable(_cfg(200, 200, 200, 200, 200, 200))

    rm = build_from_events(gen())
    assert rm.reachable_ranges[0] == (100, 200)


# ---------------------------------------------------------------------------
# build_from_events — no-reachable-observation edge case (EMPTY_RANGE sentinel)
# ---------------------------------------------------------------------------


def test_empty_range_sentinel_is_inverted_full_span():
    # documented convention: an inverted (max, min) span so nothing is ever
    # "in range" for an unobserved joint.
    assert EMPTY_RANGE == (TICK_MAX, TICK_MIN)
    lo, hi = EMPTY_RANGE
    assert lo > hi
    # no valid tick can satisfy lo <= t <= hi
    for t in (TICK_MIN, 1234, TICK_MAX):
        assert not (lo <= t <= hi)


def test_build_from_events_no_reachable_uses_empty_range_sentinel():
    rm = build_from_events([_blocked(_cfg(4000, 4000, 4000, 4000, 4000, 4000))])
    assert rm.reachable_ranges == tuple(EMPTY_RANGE for _ in range(NUM_JOINTS))
    # the blocked config is still captured
    assert len(rm.blocked) == 1


def test_build_from_events_empty_input_is_all_sentinel_no_blocked():
    rm = build_from_events([])
    assert rm.reachable_ranges == tuple(EMPTY_RANGE for _ in range(NUM_JOINTS))
    assert rm.blocked == ()


def test_unobserved_joint_makes_everything_unreachable():
    rm = build_from_events([_blocked(_cfg(4000, 4000, 4000, 4000, 4000, 4000))])
    # every plausible config is unreachable because every joint range is empty
    assert is_reachable(rm, _cfg(0, 0, 0, 0, 0, 0)) is False
    assert is_reachable(rm, _cfg(2000, 2000, 2000, 2000, 2000, 2000)) is False


# ---------------------------------------------------------------------------
# is_reachable — the offline query
# ---------------------------------------------------------------------------


def _sample_map() -> ReachMap:
    return ReachMap(
        reachable_ranges=(
            (100, 150),
            (100, 200),
            (300, 350),
            (400, 400),
            (450, 500),
            (600, 600),
        ),
        blocked=(_cfg(120, 150, 320, 400, 480, 600),),
    )


def test_is_reachable_in_range_and_clean_is_true():
    rm = _sample_map()
    assert is_reachable(rm, _cfg(110, 120, 310, 400, 460, 600)) is True


def test_is_reachable_returns_a_plain_bool():
    rm = _sample_map()
    result = is_reachable(rm, _cfg(110, 120, 310, 400, 460, 600))
    assert result is True and isinstance(result, bool)


def test_is_reachable_in_range_but_blocked_is_false():
    rm = _sample_map()
    # exact match of the blocked config, and it IS inside every joint range
    blocked_cfg = _cfg(120, 150, 320, 400, 480, 600)
    for j in range(NUM_JOINTS):
        lo, hi = rm.reachable_ranges[j]
        assert lo <= blocked_cfg[j] <= hi  # sanity: it really is in-range
    assert is_reachable(rm, blocked_cfg) is False


def test_is_reachable_out_of_range_low_is_false():
    rm = _sample_map()
    assert is_reachable(rm, _cfg(50, 120, 310, 400, 460, 600)) is False


def test_is_reachable_out_of_range_high_is_false():
    rm = _sample_map()
    assert is_reachable(rm, _cfg(110, 120, 310, 400, 460, 700)) is False


def test_is_reachable_boundary_ticks_are_inclusive():
    rm = _sample_map()
    # min and max of every joint range are themselves reachable (not blocked)
    assert is_reachable(rm, _cfg(100, 100, 300, 400, 450, 600)) is True
    assert is_reachable(rm, _cfg(150, 200, 350, 400, 500, 600)) is True


def test_is_reachable_does_not_import_hardware_or_open_serial():
    # Fresh interpreter: import ONLY the query module + pure types, run the
    # offline query, and assert no arm101.hardware module was pulled in and no
    # serial library was imported. Proves the query is pure/offline (h3, h10).
    code = (
        "import sys\n"
        "from arm101.explore.reachmap import is_reachable\n"
        "from arm101.explore.types import ReachMap, JointConfig\n"
        "m = ReachMap(reachable_ranges=tuple((0, 4095) for _ in range(6)))\n"
        "cfg = JointConfig.from_ticks((10, 10, 10, 10, 10, 10))\n"
        "assert is_reachable(m, cfg) is True\n"
        "bad = [n for n in sys.modules if 'arm101.hardware' in n or n == 'serial']\n"
        "assert not bad, bad\n"
        "print('OK')\n"
    )
    proc = subprocess.run(  # nosec B603
        [sys.executable, "-c", code],
        capture_output=True,
        text=True,
    )
    assert proc.returncode == 0, proc.stderr
    assert proc.stdout.strip() == "OK"


def test_reachmap_module_source_names_no_hardware_import():
    # Belt-and-suspenders static check: the query module must not IMPORT
    # arm101.hardware (prose mentions of the name in docstrings are fine).
    import ast
    import inspect

    tree = ast.parse(inspect.getsource(reachmap))
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module)
    assert not any(name.startswith("arm101.hardware") for name in imported), imported
    assert "serial" not in imported


# ---------------------------------------------------------------------------
# save_map / load_map_file — exact JSON round-trip
# ---------------------------------------------------------------------------


def test_save_load_round_trips_exactly(tmp_path):
    rm = build_from_events(
        [
            _reachable(_cfg(100, 200, 300, 400, 500, 600)),
            _reachable(_cfg(150, 100, 350, 400, 450, 600)),
            _blocked(_cfg(4000, 10, 10, 10, 10, 10)),
        ]
    )
    path = tmp_path / "reachmap.json"
    assert save_map(path, rm) is None
    loaded = load_map_file(path)
    assert loaded == rm
    assert isinstance(loaded, ReachMap)


def test_save_writes_plain_json(tmp_path):
    import json

    rm = _sample_map()
    path = tmp_path / "map.json"
    save_map(path, rm)
    # file content is valid JSON matching the ReachMap.to_dict() payload
    on_disk = json.loads(path.read_text(encoding="utf-8"))
    assert on_disk == rm.to_dict()


def test_save_load_accepts_str_path(tmp_path):
    rm = _sample_map()
    path = str(tmp_path / "as_str.json")
    save_map(path, rm)
    assert load_map_file(path) == rm


def test_round_trip_preserves_empty_range_sentinel(tmp_path):
    rm = build_from_events([_blocked(_cfg(4000, 4000, 4000, 4000, 4000, 4000))])
    path = tmp_path / "sentinel.json"
    save_map(path, rm)
    loaded = load_map_file(path)
    assert loaded == rm
    assert loaded.reachable_ranges[0] == EMPTY_RANGE


if __name__ == "__main__":  # pragma: no cover
    sys.exit(pytest.main([__file__, "-q"]))
