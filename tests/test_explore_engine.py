"""Tests for arm101.explore.engine — the flood-fill reachability engine (t8).

TDD: written before arm101/explore/engine.py existed and drive its
implementation.

What the engine is
------------------
``engine.explore`` flood-fills the SO-101's reachable joint-space from a home
configuration, driving EVERY physical move through an injected move function
(defaulting to the overload-safe ``arm101.hardware.gentle.gentle_move``). It
records a ``ContactEvent`` for every attempt to the JSONL log, invokes the
combination-``escape`` search when a joint is blocked, is bounded by the shared
``Budget``, and finally derives + saves the compact ``ReachMap``.

The whole point of the injected move seam is that the engine can be exercised
with NO hardware: a synthetic ``move_fn`` encoding a toy obstruction map stands
in for the real gentle move, and a ``FakeBus`` stands in for a serial port.

Covers (coverage targets c4, c8, c10, h1, h2, h4/h13):
* Public API surface (explore, ExploreResult, MoveFn, default move_fn).
* Flood-fill maps the reachable region and walls off a blocked region; every
  attempt (reachable or blocked) is recorded to the log; a block triggers an
  ``escape`` attempt for the blocked joint (c4/c10).
* The injected ``move_fn`` is the SOLE motion path — the engine issues no raw
  ``write_goal_position`` (nor any raw register write) itself (h1).
* A full run against a ``FakeBus`` with the REAL ``gentle_move`` completes with
  ZERO ``OverloadError`` escaping, even with the servo's own overload latch
  armed mid-run, and writes a loadable map (c8).
* Budget bound: a tiny ``max_moves`` stops the run early yet still produces a
  valid partial map (c8/h13); a hot thermal reading halts it too.
* Escape extends the frontier: when ``escape`` returns a path, the freed cell is
  explored (h4).
* Resume: a second run over an existing log does not re-probe already-recorded
  configs (h2).
"""

from __future__ import annotations

from arm101.explore import engine
from arm101.explore.budget import Budget
from arm101.explore.engine import ExploreResult, explore
from arm101.explore.escape import EscapePath
from arm101.explore.grid import cell_to_config
from arm101.explore.log import append_event, read_events
from arm101.explore.reachmap import load_map_file
from arm101.explore.types import (
    NUM_JOINTS,
    ContactEvent,
    ContactResult,
    GridSpec,
    JointConfig,
    ReachMap,
)
from arm101.hardware.bus import FakeBus
from arm101.hardware.gentle import gentle_move

# ---------------------------------------------------------------------------
# Grid fixtures — small hardware-free specs with most joints pinned
# ---------------------------------------------------------------------------


def _two_joint_spec() -> GridSpec:
    """Joints 0 and 1 span 3 buckets (2000..2100 step 50); joints 2-5 pinned.

    A pinned joint has ``bounds == (2048, 2048)`` and ``bucket_size == 1`` so
    ``(max - min) // bucket_size == 0`` — a single bucket, no neighbors. This
    keeps the reachable grid a 3x3 plane so a full flood-fill is tiny and its
    reachable/blocked structure is easy to assert.
    """
    origin = JointConfig.from_ticks((2000, 2000, 2048, 2048, 2048, 2048))
    bucket_size = (50, 50, 1, 1, 1, 1)
    bounds = (
        (2000, 2100),
        (2000, 2100),
        (2048, 2048),
        (2048, 2048),
        (2048, 2048),
        (2048, 2048),
    )
    return GridSpec(bucket_size=bucket_size, origin=origin, bounds=bounds)


def _one_joint_spec() -> GridSpec:
    """Only joint 0 moves (3 buckets); joints 1-5 pinned. A 1-D line grid."""
    origin = JointConfig.from_ticks((2000, 2048, 2048, 2048, 2048, 2048))
    bucket_size = (50, 1, 1, 1, 1, 1)
    bounds = (
        (2000, 2100),
        (2048, 2048),
        (2048, 2048),
        (2048, 2048),
        (2048, 2048),
        (2048, 2048),
    )
    return GridSpec(bucket_size=bucket_size, origin=origin, bounds=bounds)


# ---------------------------------------------------------------------------
# Synthetic move_fn — same call shape as gentle_move, encodes a toy obstruction
# ---------------------------------------------------------------------------


def make_move_fn(*, wall=None, calls=None):
    """Build a synthetic ``move_fn`` matching ``gentle_move``'s signature/return.

    ``wall(motor, target) -> bool`` (optional) decides whether a move is a
    contact/block (``True``) rather than free motion. ``calls`` (optional list)
    records every invocation as ``{"motor", "target"}`` so a test can assert the
    move function is the sole motion path.
    """

    def move_fn(
        bus,
        motor,
        target,
        *,
        min_angle,
        max_angle,
        threshold=250,
        step=25,
        backoff=50,
        acceleration=20,
        speed=150,
        allow_motion=False,
        **_kwargs,
    ):
        if calls is not None:
            calls.append({"motor": motor, "target": target})
        blocked = bool(wall(motor, target)) if wall is not None else False
        clamped = min(max(int(target), int(min_angle)), int(max_angle))
        return {
            "motor": motor,
            "contacted": blocked,
            "overloaded": False,
            "contact_load": 300 if blocked else None,
            "start_position": int(min_angle),
            "contact_position": clamped if blocked else None,
            "retreat_position": int(min_angle) if blocked else None,
            "final_position": int(min_angle) if blocked else clamped,
        }

    return move_fn


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def test_public_api_and_default_move_fn():
    assert callable(explore)
    assert engine.gentle_move is gentle_move
    # MoveFn alias is exported for callers/type hints.
    assert hasattr(engine, "MoveFn")
    # ExploreResult is a data container bundling the map + summary.
    fields = ExploreResult.__dataclass_fields__
    assert "reach_map" in fields
    assert "budget_bounded" in fields


# ---------------------------------------------------------------------------
# c4 / c10 — flood-fill maps the reachable region and walls off blocks
# ---------------------------------------------------------------------------


def test_floodfill_maps_reachable_region_and_walls_off_blocked(tmp_path, monkeypatch):
    spec = _two_joint_spec()
    log_path = tmp_path / "events.jsonl"
    map_path = tmp_path / "map.json"

    calls = []
    # Wall: joint 1 (motor 2) cannot advance to its top bucket (tick 2100).
    move_fn = make_move_fn(wall=lambda m, t: m == 2 and t >= 2100, calls=calls)

    escape_calls = []

    def spy_escape(blocked_cell, blocked_joint, *args, **kwargs):
        escape_calls.append((blocked_cell, blocked_joint))
        return None  # do not extend the frontier in this test

    monkeypatch.setattr(engine, "escape", spy_escape)

    result = explore(
        FakeBus(),
        spec,
        log_path=log_path,
        map_path=map_path,
        move_fn=move_fn,
    )

    events = read_events(log_path)
    # Every attempt (reachable or blocked) is recorded exactly once, and the
    # log line count equals the number of flood-fill moves issued.
    assert len(events) == len(calls)
    assert len(events) > 0

    reachable_cfgs = {e.config for e in events if e.result == ContactResult.REACHABLE}
    blocked_cfgs = {e.config for e in events if e.result == ContactResult.BLOCKED}

    # The wall shows up as blocked configs at joint 1's top tick, and never as a
    # reachable config there.
    assert any(cfg[1] == 2100 for cfg in blocked_cfgs)
    assert all(cfg[1] != 2100 for cfg in reachable_cfgs)

    # Escape is attempted on every block, always for the blocked joint (index 1).
    assert escape_calls, "a block must trigger an escape attempt"
    assert all(joint == 1 for (_cell, joint) in escape_calls)

    # The derived map: joint 0 fully reachable, joint 1 reachable only up to the
    # bucket below the wall (tick 2050).
    assert result.reach_map.reachable_ranges[0] == (2000, 2100)
    assert result.reach_map.reachable_ranges[1] == (2000, 2050)

    # The map artifact is written and round-trips to the returned map.
    assert map_path.exists()
    assert load_map_file(map_path) == result.reach_map
    assert result.contacts > 0
    assert result.escapes_attempted == len(escape_calls)


# ---------------------------------------------------------------------------
# h1 — the injected move_fn is the SOLE motion path
# ---------------------------------------------------------------------------


def test_move_fn_is_the_only_motion_path(tmp_path):
    spec = _two_joint_spec()
    bus = FakeBus()
    bus.open()

    calls = []
    move_fn = make_move_fn(calls=calls)

    explore(
        bus,
        spec,
        log_path=tmp_path / "events.jsonl",
        map_path=tmp_path / "map.json",
        move_fn=move_fn,
    )

    assert calls, "the injected move_fn must be the motion path"
    # h1/c10: the engine issues NO raw goal-position write — every MOTION goes
    # through move_fn. Its ONLY direct bus writes are post-probe torque RELEASES
    # (limping each probed joint to keep the bus healthy), which move no joint:
    # torque-OFF writes to Torque_Enable (register 40, value 0), nothing else.
    assert bus.position_writes == []
    assert all(w["on"] is False for w in bus.torque_writes)
    assert all(w["addr"] == 40 and w["value"] == 0 for w in bus.register_writes)


# ---------------------------------------------------------------------------
# c8 — full FakeBus run with the REAL gentle_move, zero OverloadError escapes
# ---------------------------------------------------------------------------


def test_fakebus_run_with_real_gentle_move_completes(tmp_path):
    spec = _two_joint_spec()
    bus = FakeBus()
    bus.open()
    log_path = tmp_path / "events.jsonl"
    map_path = tmp_path / "map.json"

    result = explore(bus, spec, log_path=log_path, map_path=map_path)

    events = read_events(log_path)
    assert events
    # A plain FakeBus reports zero load, so every real gentle_move is reachable.
    assert all(e.result == ContactResult.REACHABLE for e in events)
    assert map_path.exists()
    assert load_map_file(map_path) == result.reach_map
    assert isinstance(result, ExploreResult)


def test_fakebus_run_survives_a_mid_run_hardware_overload(tmp_path):
    spec = _two_joint_spec()
    # Arm the servo's own overload latch to trip mid-way through the first real
    # gentle_move (op 8 lands inside its step loop). gentle_move must catch it,
    # recover, and report overloaded=True — never let OverloadError escape.
    bus = FakeBus(overload_after_ops=8)
    bus.open()
    log_path = tmp_path / "events.jsonl"
    map_path = tmp_path / "map.json"

    # If an OverloadError escaped the run it would fail here naturally, with the
    # original traceback pointing at the raise inside gentle_move — which is
    # exactly this test's thesis: zero OverloadError escapes a run.
    result = explore(bus, spec, log_path=log_path, map_path=map_path)

    # The run completed and produced a loadable map.
    assert map_path.exists()
    assert isinstance(load_map_file(map_path), ReachMap)
    # The recovery path disarmed the servo's latch (clear_overload was called).
    assert bus.overload_after_ops is None
    assert isinstance(result, ExploreResult)


# ---------------------------------------------------------------------------
# Resilience — a transient comm error on one probe never aborts the run.
# Regression: the first live follower run aborted when a gripper torque-limit
# write raised RX_TIMEOUT (result=-6) mid-flood-fill.
# ---------------------------------------------------------------------------


def _flaky_move_fn(*, fail_motor, fail_times, calls=None):
    """A move_fn that raises ``CliError`` on the first ``fail_times`` calls
    targeting ``fail_motor`` (a transient comm glitch), then behaves normally."""
    from arm101.cli._errors import EXIT_ENV_ERROR, CliError

    state = {"n": 0}
    inner = make_move_fn(calls=calls)

    def move_fn(bus, motor, target, **kwargs):
        if motor == fail_motor and state["n"] < fail_times:
            state["n"] += 1
            raise CliError(
                code=EXIT_ENV_ERROR,
                message=f"Write torque limit failed for motor {motor}: result=-6, error=0.",
                remediation="Check wiring, power, and that the motor ID is correct.",
            )
        return inner(bus, motor, target, **kwargs)

    return move_fn


def test_transient_comm_error_on_a_probe_is_skipped_not_fatal(tmp_path):
    spec = _two_joint_spec()
    log_path = tmp_path / "events.jsonl"
    map_path = tmp_path / "map.json"

    # Motor 2 (joint 1) fails on EVERY attempt (a transient that never clears) —
    # the run must NOT abort; those probes are skipped + counted, and the other
    # joint's exploration still yields a loadable map.
    move_fn = _flaky_move_fn(fail_motor=2, fail_times=10_000)

    result = explore(FakeBus(), spec, log_path=log_path, map_path=map_path, move_fn=move_fn)

    assert isinstance(result, ExploreResult)
    assert result.errors >= 1  # at least one probe was skipped, not fatal
    assert map_path.exists()
    assert isinstance(load_map_file(map_path), ReachMap)


def test_probe_retry_recovers_a_one_shot_comm_glitch(tmp_path):
    spec = _one_joint_spec()
    log_path = tmp_path / "events.jsonl"
    map_path = tmp_path / "map.json"

    calls = []
    # Motor 1 times out exactly ONCE then succeeds — the single retry absorbs the
    # glitch, so no error is counted and the probe is recorded normally.
    move_fn = _flaky_move_fn(fail_motor=1, fail_times=1, calls=calls)

    result = explore(FakeBus(), spec, log_path=log_path, map_path=map_path, move_fn=move_fn)

    assert result.errors == 0  # the retry absorbed the transient glitch
    assert any(c["motor"] == 1 for c in calls)  # motor 1 did eventually move
    assert len(read_events(log_path)) >= 1


def test_each_probe_releases_torque_to_keep_the_bus_healthy(tmp_path):
    # Regression (hardware): leaving joints holding torque across the flood-fill
    # accumulates active servos and wedges the bus (register-48 comms cascade
    # -fail). Every probe must limp its joint afterward.
    spec = _two_joint_spec()
    bus = FakeBus()
    bus.open()

    result = explore(
        bus,
        spec,
        log_path=tmp_path / "events.jsonl",
        map_path=tmp_path / "map.json",
        move_fn=make_move_fn(),  # synthetic: never touches the bus itself
        budget=Budget(max_moves=3),
    )

    # The synthetic move_fn ignores the bus, so every torque write recorded is
    # an engine release — all torque-OFF: one per probe, plus a final limp sweep
    # of all six motors so the arm is never left holding at the end of a run.
    assert bus.torque_writes, "engine must release torque after each probe"
    assert all(w["on"] is False for w in bus.torque_writes)
    assert len(bus.torque_writes) == result.moves + 6
    released_motors = {w["motor"] for w in bus.torque_writes}
    assert released_motors.issuperset(range(1, 7)), "final sweep limps every joint"


# ---------------------------------------------------------------------------
# c8 / h13 — budget bounds the run (moves cap, then thermal)
# ---------------------------------------------------------------------------


def test_budget_move_cap_stops_run_early_but_still_maps(tmp_path):
    spec = _two_joint_spec()
    log_path = tmp_path / "events.jsonl"
    map_path = tmp_path / "map.json"

    budget = Budget(max_moves=2)
    result = explore(
        FakeBus(),
        spec,
        log_path=log_path,
        map_path=map_path,
        budget=budget,
        move_fn=make_move_fn(),
    )

    # Exactly the capped number of moves were issued, and every one was logged.
    assert budget.moves == 2
    assert len(read_events(log_path)) == 2
    assert result.budget_bounded is True
    # A valid (partial) map is still written and loadable.
    assert map_path.exists()
    assert isinstance(load_map_file(map_path), ReachMap)


def test_budget_move_cap_of_one_stops_mid_cell(tmp_path):
    spec = _two_joint_spec()
    log_path = tmp_path / "events.jsonl"
    map_path = tmp_path / "map.json"

    budget = Budget(max_moves=1)
    result = explore(
        FakeBus(),
        spec,
        log_path=log_path,
        map_path=map_path,
        budget=budget,
        move_fn=make_move_fn(),
    )

    # The budget is spent partway through the home cell's neighbours: one move
    # issued, the frontier cleared, the run flagged budget-bounded.
    assert budget.moves == 1
    assert len(read_events(log_path)) == 1
    assert result.budget_bounded is True


def test_flaky_temperature_read_never_breaks_the_run(tmp_path):
    spec = _two_joint_spec()
    log_path = tmp_path / "events.jsonl"
    map_path = tmp_path / "map.json"

    from arm101.cli._errors import EXIT_ENV_ERROR, CliError

    def raising_temps():
        raise CliError(
            code=EXIT_ENV_ERROR,
            message="simulated flaky temperature read",
            remediation="ignored — the guard suppresses it",
        )

    # A temperature provider that always raises must be swallowed (treated as no
    # reading), so the run still completes and produces a map.
    result = explore(
        FakeBus(),
        spec,
        log_path=log_path,
        map_path=map_path,
        move_fn=make_move_fn(),
        temperatures=raising_temps,
    )

    assert result.budget_bounded is False
    assert read_events(log_path), "the run should have proceeded normally"
    assert map_path.exists()


def test_thermal_ceiling_halts_run(tmp_path):
    spec = _two_joint_spec()
    log_path = tmp_path / "events.jsonl"
    map_path = tmp_path / "map.json"

    budget = Budget(max_temperature_c=60)

    result = explore(
        FakeBus(),
        spec,
        log_path=log_path,
        map_path=map_path,
        budget=budget,
        move_fn=make_move_fn(),
        temperatures=lambda: [99] * NUM_JOINTS,  # over the ceiling
    )

    assert result.budget_bounded is True
    # Halted before issuing any move.
    assert read_events(log_path) == []
    assert map_path.exists()


# ---------------------------------------------------------------------------
# h4 — escape extends the reachable frontier
# ---------------------------------------------------------------------------


def test_escape_success_extends_the_frontier(tmp_path, monkeypatch):
    spec = _two_joint_spec()
    log_path = tmp_path / "events.jsonl"
    map_path = tmp_path / "map.json"

    # Wall joint 1's top bucket, exactly as before.
    move_fn = make_move_fn(wall=lambda m, t: m == 2 and t >= 2100)

    freed_cells = []

    def fake_escape(blocked_cell, blocked_joint, spec_, budget_, probe, **kwargs):
        # Pretend a coordinated move freed the joint: the freed config is the
        # blocked cell nudged one bucket up on joint 0 (a real, reachable cell).
        freed_cell = list(blocked_cell)
        if freed_cell[0] < 2:
            freed_cell[0] += 1
        freed = cell_to_config(tuple(freed_cell), spec_)
        freed_cells.append(tuple(freed_cell))
        return EscapePath(steps=(), freed_config=freed)

    monkeypatch.setattr(engine, "escape", fake_escape)

    result = explore(
        FakeBus(),
        spec,
        log_path=log_path,
        map_path=map_path,
        move_fn=move_fn,
    )

    assert freed_cells, "escape should have been consulted on a block"
    assert result.escapes_succeeded == len(freed_cells)
    # The freed cells were enqueued and explored (they are among the visited).
    assert result.cells_visited >= 1


# ---------------------------------------------------------------------------
# h2 — resume: a second run does not re-probe already-recorded configs
# ---------------------------------------------------------------------------


def test_resume_does_not_reprobe_recorded_configs(tmp_path):
    spec = _one_joint_spec()
    log_path = tmp_path / "events.jsonl"
    map_path = tmp_path / "map.json"

    # Pre-seed the log with the two non-home reachable configs of the 1-D grid.
    b1 = cell_to_config((1, 0, 0, 0, 0, 0), spec)
    b2 = cell_to_config((2, 0, 0, 0, 0, 0), spec)
    for cfg in (b1, b2):
        append_event(
            log_path,
            ContactEvent(
                config=cfg,
                moving_joint_index=0,
                load_magnitude=0,
                result=ContactResult.REACHABLE,
            ),
        )

    calls = []
    move_fn = make_move_fn(calls=calls)

    result = explore(
        FakeBus(),
        spec,
        log_path=log_path,
        map_path=map_path,
        move_fn=move_fn,
    )

    # Everything reachable is already in the log, so nothing is re-probed.
    assert calls == []
    # The map still derives from the pre-seeded events (the two recorded ticks;
    # the home tick 2000 is never logged as a move, so it is not in the range).
    assert result.reach_map.reachable_ranges[0] == (2050, 2100)


def test_resume_probes_only_new_configs(tmp_path):
    spec = _one_joint_spec()
    log_path = tmp_path / "events.jsonl"
    map_path = tmp_path / "map.json"

    # Pre-seed only the first step. The second step (b2) is still unexplored.
    b1 = cell_to_config((1, 0, 0, 0, 0, 0), spec)
    append_event(
        log_path,
        ContactEvent(
            config=b1,
            moving_joint_index=0,
            load_magnitude=0,
            result=ContactResult.REACHABLE,
        ),
    )
    b2 = cell_to_config((2, 0, 0, 0, 0, 0), spec)

    calls = []
    move_fn = make_move_fn(calls=calls)

    explore(
        FakeBus(),
        spec,
        log_path=log_path,
        map_path=map_path,
        move_fn=move_fn,
    )

    probed_targets = {c["target"] for c in calls}
    # b2 (tick 2100) is newly probed; the already-recorded b1 (tick 2050) is not.
    assert b2[0] in probed_targets
    assert b1[0] not in probed_targets
