"""Tests for arm101.explore.grid — tick<->cell discretization over a GridSpec.

TDD: written before arm101/explore/grid.py existed and drive the implementation.

Covers:
* Zero-dep import (arm101.explore.grid imports with no third-party packages).
* config_to_cell: per-joint bucket-index computation, including clamping ticks
  that fall outside GridSpec.bounds and asymmetric per-joint bucket sizes.
* cell_to_config: representative tick per bucket, clamped into bounds, returns
  a JointConfig.
* Round-trip: config_to_cell(cell_to_config(cell, spec), spec) == cell for
  in-range cells (interior and boundary), under both default and custom
  bounds/bucket sizes.
* neighbors: the 6-DOF +-1-bucket neighborhood, dropping out-of-range
  neighbors at the grid boundaries.
* clamp: clamps every joint tick into its GridSpec bound.
* home_cell: the cell of GridSpec.origin.
"""

from __future__ import annotations

import sys

import pytest


def _uniform_spec(bucket_size=64, bounds=None):
    from arm101.explore.types import GridSpec, JointConfig

    origin = JointConfig.from_ticks((2048, 2048, 2048, 2048, 2048, 2048))
    kwargs = {"bucket_size": (bucket_size,) * 6, "origin": origin}
    if bounds is not None:
        kwargs["bounds"] = bounds
    return GridSpec(**kwargs)


# ---------------------------------------------------------------------------
# 1. Zero-dep import
# ---------------------------------------------------------------------------


def test_import_arm101_explore_grid_zero_deps():
    """import arm101.explore.grid must work with no third-party packages installed."""
    import arm101.explore.grid  # noqa: F401

    assert "arm101.explore.grid" in sys.modules


# ---------------------------------------------------------------------------
# 2. config_to_cell
# ---------------------------------------------------------------------------


def test_config_to_cell_basic_bucket_index():
    from arm101.explore.grid import config_to_cell
    from arm101.explore.types import JointConfig

    spec = _uniform_spec(bucket_size=64)
    cfg = JointConfig.from_ticks((130, 130, 130, 130, 130, 130))
    assert config_to_cell(cfg, spec) == (2, 2, 2, 2, 2, 2)


def test_config_to_cell_is_tuple_of_6_ints():
    from arm101.explore.grid import config_to_cell
    from arm101.explore.types import JointConfig

    spec = _uniform_spec(bucket_size=64)
    cell = config_to_cell(JointConfig.from_ticks((0, 100, 200, 300, 400, 4095)), spec)
    assert isinstance(cell, tuple)
    assert len(cell) == 6
    assert all(isinstance(v, int) for v in cell)


def test_config_to_cell_zero_tick_is_bucket_zero():
    from arm101.explore.grid import config_to_cell
    from arm101.explore.types import JointConfig

    spec = _uniform_spec(bucket_size=64)
    cfg = JointConfig.from_ticks((0, 0, 0, 0, 0, 0))
    assert config_to_cell(cfg, spec) == (0, 0, 0, 0, 0, 0)


def test_config_to_cell_max_tick_is_max_bucket():
    from arm101.explore.grid import config_to_cell
    from arm101.explore.types import JointConfig

    spec = _uniform_spec(bucket_size=64)
    cfg = JointConfig.from_ticks((4095, 4095, 4095, 4095, 4095, 4095))
    # 4095 // 64 == 63, and that is the max valid bucket for bounds (0, 4095).
    assert config_to_cell(cfg, spec) == (63, 63, 63, 63, 63, 63)


def test_config_to_cell_respects_custom_bounds():
    from arm101.explore.grid import config_to_cell
    from arm101.explore.types import GridSpec, JointConfig

    origin = JointConfig.from_ticks((1500,) * 6)
    spec = GridSpec(bucket_size=(50,) * 6, origin=origin, bounds=((1000, 3000),) * 6)
    cfg = JointConfig.from_ticks((1050, 1050, 1050, 1050, 1050, 1050))
    # bucket = (1050 - 1000) // 50 == 1
    assert config_to_cell(cfg, spec) == (1, 1, 1, 1, 1, 1)


def test_config_to_cell_clamps_tick_below_bound_min():
    from arm101.explore.grid import config_to_cell
    from arm101.explore.types import GridSpec, JointConfig

    origin = JointConfig.from_ticks((1500,) * 6)
    spec = GridSpec(bucket_size=(50,) * 6, origin=origin, bounds=((1000, 3000),) * 6)
    cfg = JointConfig.from_ticks((0, 0, 0, 0, 0, 0))
    assert config_to_cell(cfg, spec) == (0, 0, 0, 0, 0, 0)


def test_config_to_cell_clamps_tick_above_bound_max():
    from arm101.explore.grid import config_to_cell
    from arm101.explore.types import GridSpec, JointConfig

    origin = JointConfig.from_ticks((1500,) * 6)
    spec = GridSpec(bucket_size=(50,) * 6, origin=origin, bounds=((1000, 3000),) * 6)
    cfg = JointConfig.from_ticks((4095,) * 6)
    max_bucket = (3000 - 1000) // 50  # 40
    assert config_to_cell(cfg, spec) == (max_bucket,) * 6


def test_config_to_cell_handles_asymmetric_per_joint_bucket_sizes():
    from arm101.explore.grid import config_to_cell
    from arm101.explore.types import GridSpec, JointConfig

    origin = JointConfig.from_ticks((2048,) * 6)
    spec = GridSpec(bucket_size=(64, 32, 16, 8, 4, 2), origin=origin)
    cfg = JointConfig.from_ticks((128, 128, 128, 128, 128, 128))
    assert config_to_cell(cfg, spec) == (2, 4, 8, 16, 32, 64)


# ---------------------------------------------------------------------------
# 3. cell_to_config
# ---------------------------------------------------------------------------


def test_cell_to_config_returns_joint_config():
    from arm101.explore.grid import cell_to_config
    from arm101.explore.types import JointConfig

    spec = _uniform_spec(bucket_size=64)
    cfg = cell_to_config((2, 2, 2, 2, 2, 2), spec)
    assert isinstance(cfg, JointConfig)


def test_cell_to_config_ticks_within_bounds():
    from arm101.explore.grid import cell_to_config

    spec = _uniform_spec(bucket_size=64)
    cfg = cell_to_config((63, 63, 63, 63, 63, 63), spec)
    for tick in cfg.ticks:
        assert 0 <= tick <= 4095


def test_cell_to_config_ticks_within_custom_bounds():
    from arm101.explore.grid import cell_to_config
    from arm101.explore.types import GridSpec, JointConfig

    origin = JointConfig.from_ticks((1500,) * 6)
    spec = GridSpec(bucket_size=(50,) * 6, origin=origin, bounds=((1000, 3000),) * 6)
    max_bucket = (3000 - 1000) // 50
    cfg = cell_to_config((max_bucket,) * 6, spec)
    for tick in cfg.ticks:
        assert 1000 <= tick <= 3000


# ---------------------------------------------------------------------------
# 4. Round trip: config_to_cell(cell_to_config(cell)) == cell
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "cell",
    [
        (0, 0, 0, 0, 0, 0),
        (63, 63, 63, 63, 63, 63),
        (32, 32, 32, 32, 32, 32),
        (0, 63, 0, 63, 0, 63),
        (1, 2, 3, 4, 5, 6),
        (10, 20, 30, 40, 50, 60),
    ],
)
def test_round_trip_cell_to_config_to_cell_default_bounds(cell):
    from arm101.explore.grid import cell_to_config, config_to_cell

    spec = _uniform_spec(bucket_size=64)
    cfg = cell_to_config(cell, spec)
    assert config_to_cell(cfg, spec) == cell


@pytest.mark.parametrize(
    "cell",
    [
        (0, 0, 0, 0, 0, 0),
        (40, 40, 40, 40, 40, 40),
        (1, 1, 1, 1, 1, 1),
        (20, 40, 0, 15, 30, 40),
    ],
)
def test_round_trip_cell_to_config_to_cell_custom_bounds(cell):
    from arm101.explore.grid import cell_to_config, config_to_cell
    from arm101.explore.types import GridSpec, JointConfig

    origin = JointConfig.from_ticks((1500,) * 6)
    spec = GridSpec(bucket_size=(50,) * 6, origin=origin, bounds=((1000, 3000),) * 6)
    cfg = cell_to_config(cell, spec)
    assert config_to_cell(cfg, spec) == cell


def test_round_trip_asymmetric_bucket_sizes():
    from arm101.explore.grid import cell_to_config, config_to_cell
    from arm101.explore.types import GridSpec, JointConfig

    origin = JointConfig.from_ticks((2048,) * 6)
    spec = GridSpec(bucket_size=(64, 32, 16, 8, 4, 2), origin=origin)
    for cell in [(0,) * 6, (2, 4, 8, 16, 32, 64), (63, 127, 255, 511, 1023, 2047)]:
        cfg = cell_to_config(cell, spec)
        assert config_to_cell(cfg, spec) == cell


# ---------------------------------------------------------------------------
# 5. neighbors
# ---------------------------------------------------------------------------


def test_neighbors_interior_cell_has_12():
    from arm101.explore.grid import neighbors

    spec = _uniform_spec(bucket_size=64)
    result = neighbors((32, 32, 32, 32, 32, 32), spec)
    assert len(result) == 12
    assert len(set(result)) == 12  # no duplicates


def test_neighbors_interior_cell_contents():
    from arm101.explore.grid import neighbors

    spec = _uniform_spec(bucket_size=64)
    cell = (32, 32, 32, 32, 32, 32)
    result = set(neighbors(cell, spec))
    expected = set()
    for i in range(6):
        for delta in (-1, 1):
            neighbor = list(cell)
            neighbor[i] += delta
            expected.add(tuple(neighbor))
    assert result == expected


def test_neighbors_drops_out_of_range_at_zero_corner():
    from arm101.explore.grid import neighbors

    spec = _uniform_spec(bucket_size=64)
    result = neighbors((0, 0, 0, 0, 0, 0), spec)
    # every joint's -1 neighbor is out of range; only +1 remains, one per joint.
    assert len(result) == 6
    for cell in result:
        assert all(v >= 0 for v in cell)
        assert sum(cell) == 1


def test_neighbors_drops_out_of_range_at_max_corner():
    from arm101.explore.grid import neighbors

    spec = _uniform_spec(bucket_size=64)
    max_bucket = 63
    corner = (max_bucket,) * 6
    result = neighbors(corner, spec)
    assert len(result) == 6
    for cell in result:
        for v in cell:
            assert v <= max_bucket


def test_neighbors_edge_cell_has_11():
    from arm101.explore.grid import neighbors

    spec = _uniform_spec(bucket_size=64)
    # one joint pinned at 0 (only +1 valid), the other five interior (both valid).
    cell = (0, 32, 32, 32, 32, 32)
    result = neighbors(cell, spec)
    assert len(result) == 11


def test_neighbors_never_yields_cell_itself():
    from arm101.explore.grid import neighbors

    spec = _uniform_spec(bucket_size=64)
    cell = (32, 32, 32, 32, 32, 32)
    result = neighbors(cell, spec)
    assert cell not in result


def test_neighbors_returns_list():
    from arm101.explore.grid import neighbors

    spec = _uniform_spec(bucket_size=64)
    result = neighbors((32, 32, 32, 32, 32, 32), spec)
    assert isinstance(result, list)


# ---------------------------------------------------------------------------
# 6. clamp
# ---------------------------------------------------------------------------


def test_clamp_leaves_in_range_config_unchanged():
    from arm101.explore.grid import clamp
    from arm101.explore.types import JointConfig

    spec = _uniform_spec(bucket_size=64)
    cfg = JointConfig.from_ticks((100, 200, 300, 400, 500, 600))
    assert clamp(cfg, spec) == cfg


def test_clamp_pulls_low_ticks_up_to_bound_min():
    from arm101.explore.grid import clamp
    from arm101.explore.types import GridSpec, JointConfig

    origin = JointConfig.from_ticks((1500,) * 6)
    spec = GridSpec(bucket_size=(50,) * 6, origin=origin, bounds=((1000, 3000),) * 6)
    cfg = JointConfig.from_ticks((0, 500, 999, 1000, 1500, 0))
    clamped = clamp(cfg, spec)
    assert clamped.ticks == (1000, 1000, 1000, 1000, 1500, 1000)


def test_clamp_pulls_high_ticks_down_to_bound_max():
    from arm101.explore.grid import clamp
    from arm101.explore.types import GridSpec, JointConfig

    origin = JointConfig.from_ticks((1500,) * 6)
    spec = GridSpec(bucket_size=(50,) * 6, origin=origin, bounds=((1000, 3000),) * 6)
    cfg = JointConfig.from_ticks((4095, 3001, 3000, 2500, 4000, 3500))
    clamped = clamp(cfg, spec)
    assert clamped.ticks == (3000, 3000, 3000, 2500, 3000, 3000)


def test_clamp_returns_joint_config():
    from arm101.explore.grid import clamp
    from arm101.explore.types import JointConfig

    spec = _uniform_spec(bucket_size=64)
    cfg = JointConfig.from_ticks((100, 200, 300, 400, 500, 600))
    assert isinstance(clamp(cfg, spec), JointConfig)


# ---------------------------------------------------------------------------
# 7. home_cell
# ---------------------------------------------------------------------------


def test_home_cell_matches_config_to_cell_of_origin():
    from arm101.explore.grid import config_to_cell, home_cell

    spec = _uniform_spec(bucket_size=64)
    assert home_cell(spec) == config_to_cell(spec.origin, spec)


def test_home_cell_default_origin_2048():
    from arm101.explore.grid import home_cell

    spec = _uniform_spec(bucket_size=64)
    # origin ticks are all 2048; 2048 // 64 == 32.
    assert home_cell(spec) == (32, 32, 32, 32, 32, 32)


def test_home_cell_with_custom_origin_and_bounds():
    from arm101.explore.grid import home_cell
    from arm101.explore.types import GridSpec, JointConfig

    origin = JointConfig.from_ticks((1500,) * 6)
    spec = GridSpec(bucket_size=(50,) * 6, origin=origin, bounds=((1000, 3000),) * 6)
    # (1500 - 1000) // 50 == 10
    assert home_cell(spec) == (10, 10, 10, 10, 10, 10)
