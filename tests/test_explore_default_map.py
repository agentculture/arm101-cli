"""Tests for arm101.explore.default_map — bundled default self-collision map.

TDD contract for task t6. Covers:

* ``load_map()`` (no ``path``) returns the BUNDLED default self-collision
  :class:`ReachMap`, loaded via ``importlib.resources`` package data (not a
  ``__file__``/cwd path trick), and that default is the permissive baseline:
  full ``(0, 4095)`` per-joint ranges, empty ``blocked``.
* ``load_map(path)`` loads the user's override file instead (written via
  ``reachmap.save_map``), ignoring the bundled default entirely.
* loading the default never mutates the bundled asset on disk (bytes
  unchanged before/after).
* the bundled asset resolves as real package data via
  ``importlib.resources.files("arm101.explore")`` and round-trips cleanly
  through ``ReachMap.from_dict``.
"""

from __future__ import annotations

import importlib.resources
import json
import sys

import pytest

from arm101.explore import default_map, reachmap
from arm101.explore.default_map import default_map_path, load_map
from arm101.explore.types import NUM_JOINTS, TICK_MAX, TICK_MIN, JointConfig, ReachMap

_ASSET_RELATIVE_PATH = "arm101/explore/data/default_selfcollision_map.json"


def _asset_path():
    """Return the real on-disk path of the bundled default asset (editable install)."""
    return importlib.resources.files("arm101.explore").joinpath(
        "data/default_selfcollision_map.json"
    )


# ---------------------------------------------------------------------------
# load_map() with no path -> bundled default
# ---------------------------------------------------------------------------


def test_load_map_with_no_path_returns_bundled_default():
    rm = load_map()
    assert isinstance(rm, ReachMap)


def test_default_map_has_full_reachable_ranges_and_no_blocked():
    rm = load_map()
    assert rm.reachable_ranges == tuple((TICK_MIN, TICK_MAX) for _ in range(NUM_JOINTS))
    assert rm.blocked == ()


def test_load_map_none_matches_bundled_asset_contents_exactly():
    asset_text = _asset_path().read_text(encoding="utf-8")
    expected = ReachMap.from_dict(json.loads(asset_text))
    assert load_map() == expected
    assert load_map(None) == expected


# ---------------------------------------------------------------------------
# load_map(path) -> user override
# ---------------------------------------------------------------------------


def test_load_map_with_path_loads_user_override(tmp_path):
    user_rm = ReachMap(
        reachable_ranges=(
            (100, 200),
            (100, 200),
            (100, 200),
            (100, 200),
            (100, 200),
            (100, 200),
        ),
        blocked=(JointConfig.from_ticks((150, 150, 150, 150, 150, 150)),),
    )
    path = tmp_path / "user_map.json"
    reachmap.save_map(path, user_rm)

    loaded = load_map(path)
    assert loaded == user_rm
    # not the bundled default
    assert loaded != load_map()


def test_load_map_with_path_accepts_str_path(tmp_path):
    user_rm = ReachMap(reachable_ranges=tuple((0, 100) for _ in range(NUM_JOINTS)))
    path = tmp_path / "as_str.json"
    reachmap.save_map(path, user_rm)
    assert load_map(str(path)) == user_rm


# ---------------------------------------------------------------------------
# bundled asset is untouched by any load_map() call
# ---------------------------------------------------------------------------


def test_loading_default_does_not_mutate_bundled_asset_on_disk():
    asset = _asset_path()
    before = asset.read_bytes()
    load_map()
    load_map()  # twice, for good measure
    after = asset.read_bytes()
    assert before == after


def test_loading_user_override_does_not_mutate_bundled_asset_on_disk(tmp_path):
    asset = _asset_path()
    before = asset.read_bytes()
    user_rm = ReachMap(reachable_ranges=tuple((0, 10) for _ in range(NUM_JOINTS)))
    path = tmp_path / "user_map.json"
    reachmap.save_map(path, user_rm)
    load_map(path)
    after = asset.read_bytes()
    assert before == after


# ---------------------------------------------------------------------------
# bundled asset resolves as real package data (works from an installed wheel,
# not just a cwd-relative __file__ trick)
# ---------------------------------------------------------------------------


def test_default_map_path_resolves_via_importlib_resources():
    traversable = default_map_path()
    assert traversable.is_file()
    data = json.loads(traversable.read_text(encoding="utf-8"))
    # round-trips cleanly through ReachMap.from_dict (extra metadata keys, if
    # any, must not break this)
    rm = ReachMap.from_dict(data)
    assert isinstance(rm, ReachMap)


def test_default_map_path_matches_expected_relative_location():
    traversable = default_map_path()
    # importlib.resources Traversable objects stringify to their real path on
    # a standard (non-zip) install; confirm it's the file we expect.
    assert (
        str(traversable)
        .replace("\\", "/")
        .endswith("arm101/explore/data/default_selfcollision_map.json")
    )


def test_default_map_module_does_not_rely_on_dunder_file_for_asset_lookup():
    # Belt-and-suspenders static check: default_map.py must use
    # importlib.resources (not open()/__file__ path-joining) to find the
    # asset, so it also works from an installed wheel/zip.
    import ast
    import inspect

    tree = ast.parse(inspect.getsource(default_map))
    source = inspect.getsource(default_map)
    assert "importlib.resources" in source or "importlib_resources" in source
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, "id", None) == "open":
            pytest.fail("default_map.py must not call open() directly on __file__-relative paths")


if __name__ == "__main__":  # pragma: no cover
    sys.exit(pytest.main([__file__, "-q"]))
