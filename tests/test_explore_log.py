"""Tests for arm101.explore.log — append-only JSONL event log for reachability exploration.

TDD: written before arm101/explore/log.py existed and drive the implementation.

Covers:
* append_event: creates file+parent dir if missing, appends exactly one JSON
  line per call, opened+flushed+closed per call (durability — a killed process
  keeps every line already written).
* read_events: append -> read round-trip; tolerates a truncated/blank final
  line (a crash mid-write) instead of raising; missing file -> empty list.
* visited_configs: the set of JointConfigs already recorded, for resume;
  empty set for a missing file.
* append_events: convenience helper appending a sequence of events.
"""

from __future__ import annotations

import json

import pytest

from arm101.explore.types import ContactEvent, ContactResult, JointConfig


def _make_event(
    pan: int = 100,
    moving_joint_index: int = 0,
    load: int = 0,
    result: ContactResult = ContactResult.REACHABLE,
    step=None,
) -> ContactEvent:
    cfg = JointConfig(
        shoulder_pan=pan,
        shoulder_lift=200,
        elbow_flex=300,
        wrist_flex=400,
        wrist_roll=500,
        gripper=600,
    )
    return ContactEvent(
        config=cfg,
        moving_joint_index=moving_joint_index,
        load_magnitude=load,
        result=result,
        step=step,
    )


# ---------------------------------------------------------------------------
# append_event / read_events round-trip
# ---------------------------------------------------------------------------


def test_append_event_creates_file_and_parent_dir(tmp_path):
    """append_event creates the log file and any missing parent directories."""
    from arm101.explore.log import append_event

    path = tmp_path / "nested" / "dir" / "events.jsonl"
    assert not path.parent.exists()

    event = _make_event()
    append_event(path, event)

    assert path.exists()


def test_append_event_writes_one_json_line(tmp_path):
    """append_event writes exactly one JSON line carrying the full event schema."""
    from arm101.explore.log import append_event

    path = tmp_path / "events.jsonl"
    event = _make_event(
        pan=111, moving_joint_index=2, load=42, result=ContactResult.BLOCKED, step=7
    )
    append_event(path, event)

    lines = path.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 1
    data = json.loads(lines[0])
    assert data["config"]["shoulder_pan"] == 111
    assert data["moving_joint_index"] == 2
    assert data["load_magnitude"] == 42
    assert data["result"] == "blocked"
    assert data["step"] == 7


def test_append_event_then_read_events_round_trip(tmp_path):
    """append_event followed by read_events reconstructs equal ContactEvents."""
    from arm101.explore.log import append_event, read_events

    path = tmp_path / "events.jsonl"
    e1 = _make_event(pan=100, result=ContactResult.REACHABLE, step=1)
    e2 = _make_event(pan=200, result=ContactResult.BLOCKED, step=2)
    append_event(path, e1)
    append_event(path, e2)

    events = read_events(path)
    assert events == [e1, e2]


def test_read_events_missing_file_returns_empty_list(tmp_path):
    """read_events on a nonexistent path returns an empty list, not an error."""
    from arm101.explore.log import read_events

    events = read_events(tmp_path / "does-not-exist.jsonl")
    assert events == []


# ---------------------------------------------------------------------------
# Durability: N append_event calls (file reopened each time) -> N lines
# ---------------------------------------------------------------------------


def test_append_event_durability_n_calls_n_lines(tmp_path):
    """Each append_event call opens/flushes/closes independently — simulate a
    crash between calls: after N calls, exactly N lines are on disk."""
    from arm101.explore.log import append_event

    path = tmp_path / "events.jsonl"
    n = 10
    for i in range(n):
        # A fresh call each time, as if the process could be killed right
        # after this returns — no file handle held open across calls.
        append_event(path, _make_event(pan=1000 + i, step=i))
        lines = path.read_text(encoding="utf-8").splitlines()
        assert len(lines) == i + 1

    final_lines = path.read_text(encoding="utf-8").splitlines()
    assert len(final_lines) == n


def test_append_events_convenience_helper(tmp_path):
    """append_events appends a whole sequence of events, one line each."""
    from arm101.explore.log import append_events

    path = tmp_path / "events.jsonl"
    events = [_make_event(pan=i, step=i) for i in range(5)]
    append_events(path, events)

    lines = path.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 5


# ---------------------------------------------------------------------------
# visited_configs
# ---------------------------------------------------------------------------


def test_visited_configs_missing_file_returns_empty_set(tmp_path):
    """visited_configs on a nonexistent path returns an empty set."""
    from arm101.explore.log import visited_configs

    result = visited_configs(tmp_path / "does-not-exist.jsonl")
    assert result == set()


def test_visited_configs_returns_set_of_recorded_configs(tmp_path):
    """visited_configs reconstructs the set of already-probed JointConfigs, for resume."""
    from arm101.explore.log import append_event, visited_configs

    path = tmp_path / "events.jsonl"
    e1 = _make_event(pan=100)
    e2 = _make_event(pan=200)
    # Re-record the same config twice (e.g. probed at two different moving
    # joints) — the visited set must dedupe to one entry.
    e3 = _make_event(pan=100, moving_joint_index=1)
    append_event(path, e1)
    append_event(path, e2)
    append_event(path, e3)

    configs = visited_configs(path)
    assert configs == {e1.config, e2.config}
    assert len(configs) == 2


# ---------------------------------------------------------------------------
# Truncated / blank final line tolerance
# ---------------------------------------------------------------------------


def test_read_events_tolerates_truncated_final_line(tmp_path):
    """A truncated last line (crash mid-write) is skipped, not raised."""
    from arm101.explore.log import append_event, read_events

    path = tmp_path / "events.jsonl"
    e1 = _make_event(pan=100, step=1)
    append_event(path, e1)

    # Simulate a crash mid-write: append a partial JSON line with no
    # trailing newline.
    with path.open("a", encoding="utf-8") as fh:
        fh.write('{"config": {"shoulder_pan": 999, "shoulder_l')

    events = read_events(path)
    assert events == [e1]


def test_read_events_tolerates_blank_final_line(tmp_path):
    """A trailing blank line does not crash read_events."""
    from arm101.explore.log import append_event, read_events

    path = tmp_path / "events.jsonl"
    e1 = _make_event(pan=100, step=1)
    append_event(path, e1)

    with path.open("a", encoding="utf-8") as fh:
        fh.write("\n")

    events = read_events(path)
    assert events == [e1]


def test_visited_configs_tolerates_truncated_final_line(tmp_path):
    """visited_configs (built atop read_events) also survives a truncated tail."""
    from arm101.explore.log import append_event, visited_configs

    path = tmp_path / "events.jsonl"
    e1 = _make_event(pan=100)
    append_event(path, e1)

    with path.open("a", encoding="utf-8") as fh:
        fh.write('{"config": {"shoulder_pan": 4')

    configs = visited_configs(path)
    assert configs == {e1.config}


def test_read_events_raises_on_corrupt_non_final_line(tmp_path):
    """Corruption on a non-final line is a real error, not a mid-write crash — it still raises."""
    from arm101.explore.log import read_events

    path = tmp_path / "events.jsonl"
    path.write_text('not-json-at-all\n{"config": {}}\n', encoding="utf-8")

    with pytest.raises(Exception):
        read_events(path)
