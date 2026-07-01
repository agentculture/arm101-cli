"""Tests for arm101.explore.types — shared data types for reachability exploration.

TDD: written before arm101/explore/types.py existed and drive the implementation.

Covers:
* Zero-dep import (arm101.explore.types imports with no third-party packages).
* JointConfig: named access, tuple-of-6 storage, tick-range validation, hashability,
  to_dict/from_dict round-trip.
* GridSpec: to_dict/from_dict round-trip, default bounds.
* ContactEvent: to_dict/from_dict round-trip through an actual JSON string (the
  JSONL line schema downstream log.py will append/read).
* ReachMap: to_dict/from_dict round-trip, including a sparse blocked collection.
"""

from __future__ import annotations

import json
import sys

import pytest

# ---------------------------------------------------------------------------
# 1. Zero-dep import guarantee
# ---------------------------------------------------------------------------


def test_import_arm101_explore_types_zero_deps():
    """import arm101.explore.types must work with no third-party packages installed."""
    import arm101.explore.types  # noqa: F401

    assert "arm101.explore.types" in sys.modules


# ---------------------------------------------------------------------------
# 2. JointConfig
# ---------------------------------------------------------------------------


def test_joint_config_named_access():
    """JointConfig exposes each of the 6 joints by canonical name."""
    from arm101.explore.types import JointConfig

    cfg = JointConfig(
        shoulder_pan=100,
        shoulder_lift=200,
        elbow_flex=300,
        wrist_flex=400,
        wrist_roll=500,
        gripper=600,
    )
    assert cfg.shoulder_pan == 100
    assert cfg.shoulder_lift == 200
    assert cfg.elbow_flex == 300
    assert cfg.wrist_flex == 400
    assert cfg.wrist_roll == 500
    assert cfg.gripper == 600


def test_joint_config_ticks_is_tuple_of_6_in_canonical_order():
    """JointConfig.ticks returns the 6 joint positions as a tuple, canonical order."""
    from arm101.explore.types import JointConfig

    cfg = JointConfig(
        shoulder_pan=1,
        shoulder_lift=2,
        elbow_flex=3,
        wrist_flex=4,
        wrist_roll=5,
        gripper=6,
    )
    assert cfg.ticks == (1, 2, 3, 4, 5, 6)
    assert isinstance(cfg.ticks, tuple)
    assert len(cfg.ticks) == 6


def test_joint_config_indexable_by_joint_index():
    """JointConfig supports index-based access (0-5), matching ContactEvent's moving-joint index."""
    from arm101.explore.types import JointConfig

    cfg = JointConfig.from_ticks((10, 20, 30, 40, 50, 60))
    assert cfg[0] == 10
    assert cfg[5] == 60


def test_joint_config_from_ticks_roundtrips_to_ticks():
    from arm101.explore.types import JointConfig

    ticks = (0, 4095, 2048, 1, 4094, 2000)
    cfg = JointConfig.from_ticks(ticks)
    assert cfg.ticks == ticks


@pytest.mark.parametrize("bad_ticks", [-1, 4096, 100000, -100])
def test_joint_config_rejects_out_of_range_ticks(bad_ticks):
    """Each of the 6 joints must be validated against the 0-4095 encoder range."""
    from arm101.explore.types import JointConfig

    with pytest.raises(ValueError):
        JointConfig(
            shoulder_pan=bad_ticks,
            shoulder_lift=2048,
            elbow_flex=2048,
            wrist_flex=2048,
            wrist_roll=2048,
            gripper=2048,
        )


def test_joint_config_accepts_boundary_ticks():
    """0 and 4095 (inclusive bounds) must be accepted, not rejected."""
    from arm101.explore.types import JointConfig

    cfg = JointConfig(
        shoulder_pan=0,
        shoulder_lift=4095,
        elbow_flex=0,
        wrist_flex=4095,
        wrist_roll=0,
        gripper=4095,
    )
    assert cfg.ticks == (0, 4095, 0, 4095, 0, 4095)


def test_joint_config_is_hashable_and_usable_as_dict_key():
    """JointConfig must be hashable — downstream cell-visited tracking uses it as a dict/set key."""
    from arm101.explore.types import JointConfig

    cfg_a = JointConfig.from_ticks((1, 2, 3, 4, 5, 6))
    cfg_b = JointConfig.from_ticks((1, 2, 3, 4, 5, 6))
    cfg_c = JointConfig.from_ticks((9, 9, 9, 9, 9, 9))

    visited = {cfg_a}
    assert cfg_b in visited  # equal value hashes the same
    assert cfg_c not in visited

    as_dict_key = {cfg_a: "reachable"}
    assert as_dict_key[cfg_b] == "reachable"


def test_joint_config_equality():
    from arm101.explore.types import JointConfig

    cfg_a = JointConfig.from_ticks((1, 2, 3, 4, 5, 6))
    cfg_b = JointConfig.from_ticks((1, 2, 3, 4, 5, 6))
    cfg_c = JointConfig.from_ticks((1, 2, 3, 4, 5, 7))
    assert cfg_a == cfg_b
    assert cfg_a != cfg_c


def test_joint_config_to_dict_has_named_keys():
    from arm101.explore.types import JointConfig

    cfg = JointConfig.from_ticks((1, 2, 3, 4, 5, 6))
    d = cfg.to_dict()
    assert d == {
        "shoulder_pan": 1,
        "shoulder_lift": 2,
        "elbow_flex": 3,
        "wrist_flex": 4,
        "wrist_roll": 5,
        "gripper": 6,
    }


def test_joint_config_roundtrips_through_to_dict_from_dict():
    from arm101.explore.types import JointConfig

    cfg = JointConfig.from_ticks((10, 20, 30, 40, 50, 60))
    assert JointConfig.from_dict(cfg.to_dict()) == cfg


def test_joint_config_roundtrips_through_json():
    """to_dict() output must be plain-JSON-serializable (int/str only)."""
    from arm101.explore.types import JointConfig

    cfg = JointConfig.from_ticks((10, 20, 30, 40, 50, 60))
    blob = json.dumps(cfg.to_dict())
    restored = JointConfig.from_dict(json.loads(blob))
    assert restored == cfg


# ---------------------------------------------------------------------------
# 3. GridSpec
# ---------------------------------------------------------------------------


def test_grid_spec_roundtrips_through_to_dict_from_dict():
    from arm101.explore.types import GridSpec, JointConfig

    home = JointConfig.from_ticks((2048, 2048, 2048, 2048, 2048, 2048))
    spec = GridSpec(
        bucket_size=(64, 64, 64, 64, 64, 64),
        origin=home,
        bounds=((0, 4095),) * 6,
    )
    restored = GridSpec.from_dict(spec.to_dict())
    assert restored == spec


def test_grid_spec_defaults_bounds_to_full_encoder_range():
    from arm101.explore.types import GridSpec, JointConfig

    home = JointConfig.from_ticks((2048, 2048, 2048, 2048, 2048, 2048))
    spec = GridSpec(bucket_size=(32, 32, 32, 32, 32, 32), origin=home)
    assert spec.bounds == ((0, 4095),) * 6


def test_grid_spec_roundtrips_through_json():
    from arm101.explore.types import GridSpec, JointConfig

    home = JointConfig.from_ticks((100, 200, 300, 400, 500, 600))
    spec = GridSpec(bucket_size=(10, 20, 30, 40, 50, 60), origin=home)
    blob = json.dumps(spec.to_dict())
    restored = GridSpec.from_dict(json.loads(blob))
    assert restored == spec


# ---------------------------------------------------------------------------
# 4. ContactResult
# ---------------------------------------------------------------------------


def test_contact_result_has_reachable_and_blocked():
    from arm101.explore.types import ContactResult

    assert ContactResult.REACHABLE != ContactResult.BLOCKED
    # Must be usable where a plain string is expected (JSON-friendly).
    assert str(ContactResult.REACHABLE.value) == ContactResult.REACHABLE.value


# ---------------------------------------------------------------------------
# 5. ContactEvent — the JSONL log line schema
# ---------------------------------------------------------------------------


def test_contact_event_roundtrips_through_to_dict_from_dict():
    from arm101.explore.types import ContactEvent, ContactResult, JointConfig

    cfg = JointConfig.from_ticks((10, 20, 30, 40, 50, 60))
    event = ContactEvent(
        config=cfg,
        moving_joint_index=2,
        load_magnitude=350,
        result=ContactResult.BLOCKED,
        step=7,
    )
    restored = ContactEvent.from_dict(event.to_dict())
    assert restored == event


def test_contact_event_roundtrips_through_actual_jsonl_line():
    """to_dict() must serialize via plain json.dumps and read back identically —
    this is the exact JSONL line schema log.py will append/read."""
    from arm101.explore.types import ContactEvent, ContactResult, JointConfig

    cfg = JointConfig.from_ticks((1, 2, 3, 4, 5, 6))
    event = ContactEvent(
        config=cfg,
        moving_joint_index=0,
        load_magnitude=42,
        result=ContactResult.REACHABLE,
        step=1,
    )
    line = json.dumps(event.to_dict())
    restored = ContactEvent.from_dict(json.loads(line))
    assert restored == event
    assert restored.result == ContactResult.REACHABLE


def test_contact_event_step_is_optional():
    from arm101.explore.types import ContactEvent, ContactResult, JointConfig

    cfg = JointConfig.from_ticks((1, 2, 3, 4, 5, 6))
    event = ContactEvent(
        config=cfg,
        moving_joint_index=3,
        load_magnitude=0,
        result=ContactResult.REACHABLE,
    )
    restored = ContactEvent.from_dict(json.loads(json.dumps(event.to_dict())))
    assert restored == event
    assert restored.step is None


@pytest.mark.parametrize("bad_index", [-1, 6, 100])
def test_contact_event_rejects_out_of_range_joint_index(bad_index):
    from arm101.explore.types import ContactEvent, ContactResult, JointConfig

    cfg = JointConfig.from_ticks((1, 2, 3, 4, 5, 6))
    with pytest.raises(ValueError):
        ContactEvent(
            config=cfg,
            moving_joint_index=bad_index,
            load_magnitude=0,
            result=ContactResult.REACHABLE,
        )


# ---------------------------------------------------------------------------
# 6. ReachMap
# ---------------------------------------------------------------------------


def test_reach_map_roundtrips_through_to_dict_from_dict():
    from arm101.explore.types import JointConfig, ReachMap

    blocked_cfg = JointConfig.from_ticks((100, 100, 100, 100, 100, 100))
    rmap = ReachMap(
        reachable_ranges=((0, 4095), (500, 3500), (0, 4095), (0, 4095), (0, 4095), (0, 4095)),
        blocked=(blocked_cfg,),
    )
    restored = ReachMap.from_dict(rmap.to_dict())
    assert restored == rmap


def test_reach_map_roundtrips_with_empty_blocked_set():
    from arm101.explore.types import ReachMap

    rmap = ReachMap(reachable_ranges=((0, 4095),) * 6)
    restored = ReachMap.from_dict(rmap.to_dict())
    assert restored == rmap
    assert restored.blocked == ()


def test_reach_map_roundtrips_through_json():
    from arm101.explore.types import JointConfig, ReachMap

    blocked_a = JointConfig.from_ticks((10, 10, 10, 10, 10, 10))
    blocked_b = JointConfig.from_ticks((20, 20, 20, 20, 20, 20))
    rmap = ReachMap(
        reachable_ranges=((0, 4095),) * 6,
        blocked=(blocked_a, blocked_b),
    )
    blob = json.dumps(rmap.to_dict())
    restored = ReachMap.from_dict(json.loads(blob))
    assert restored == rmap
