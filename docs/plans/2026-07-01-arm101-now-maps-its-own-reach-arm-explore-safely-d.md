# Build Plan — arm101 now maps its own reach: 'arm explore' safely drives the SO-101 follower through its joint-space, auto-detecting every self/environment contact and recording the reachable envelope to a map file — shipped with a sensible default map and fully overridable per bench.

slug: `arm101-now-maps-its-own-reach-arm-explore-safely-d` · status: `exported` · from frame: `arm101-now-maps-its-own-reach-arm-explore-safely-d`

> arm101 now maps its own reach: 'arm explore' safely drives the SO-101 follower through its joint-space, auto-detecting every self/environment contact and recording the reachable envelope to a map file — shipped with a sensible default map and fully overridable per bench.

## Tasks

### t1 — Explore data types: JointConfig/GridSpec/ContactEvent/ReachMap dataclasses in arm101/explore/types.py

- acceptance:
  - arm101.explore.types defines JointConfig (6 joint ticks), GridSpec (per-joint bucket size + origin), ContactEvent, and ReachMap as pure-Python dataclasses (zero third-party import) that round-trip through to_dict/from_dict.

### t2 — Grid discretization in arm101/explore/grid.py: tick<->cell mapping, 6-DOF neighbor enumeration, home config, encoder-bound clamping

- depends on: t1
- acceptance:
  - grid maps a JointConfig to a discrete cell and back within one bucket, enumerates the 6-DOF neighbors of a cell, and clamps every joint to its 0-4095 encoder bound; zero third-party deps.

### t3 — JSONL event log in arm101/explore/log.py: append contact/probe events durably, read back, derive resume-state

- depends on: t1
- covers: c11, h2
- acceptance:
  - each explored cell appends one JSON line carrying the full JointConfig, the moving joint, the load magnitude, and result (reachable|blocked), flushed per event; the reader reconstructs the set of already-visited cells so a resumed run skips re-probing them.

### t4 — Compact reachability map in arm101/explore/reachmap.py: build-from-events, serialize/deserialize, offline is_reachable query

- depends on: t1
- covers: c12, h3, h9, h10
- acceptance:
  - build_from_events yields a ReachMap of per-joint reachable ranges plus a sparse set of blocked joint-combinations; is_reachable(config) returns a bool reading only the map with NO serial port opened and no motor moved; serialize then load round-trips to an identical map.

### t5 — Run budget + thermal guard in arm101/explore/budget.py: move/time caps and per-motor temperature ceiling that halts a run

- depends on: t1
- covers: h13
- acceptance:
  - a Budget tracks moves-issued and elapsed wall-time (injected clock) against configured caps and reports exhausted at the limit; a thermal check halts when a joint temperature exceeds the ceiling; a loop driven by the Budget provably terminates (a unit test proves the loop exits at the cap).

### t6 — Bundled default self-collision map + --map override loader in arm101/explore/default_map.py

- depends on: t4
- covers: c14, h5
- acceptance:
  - with no user file load_map() returns the bundled default self-collision ReachMap shipped as package data; load_map(path) loads the user file instead; neither path mutates the bundled default asset on disk.

### t7 — Deeper multi-joint combination-escape in arm101/explore/escape.py: pruned, budgeted search to free a blocked joint

- depends on: t1, t2, t5
- covers: c13, h4
- acceptance:
  - given a blocked cell, escape() searches multi-joint perturbation vectors (not just single-joint) for a configuration from which the blocked joint advances, bounded by depth/breadth caps AND the shared Budget so it always terminates; a fixture where joint A is blocked until joint B moves first yields the freed A path, and a no-escape fixture returns empty within the cap.

### t8 — Explorer engine in arm101/explore/engine.py: flood-fill from home over the grid via gentle_move, recording contacts, invoking escape

- depends on: t1, t2, t3, t5, t7
- covers: c4, c8, c10, h1
- acceptance:
  - explore() flood-fills reachable cells from the home config, issuing every motion through bus.gentle_move (test asserts no raw goal writes and only the passed port opened); on a contact it records the blocked cell via the log and calls escape to continue; against a FakeBus a full run completes with zero OverloadError/error=32, writes a complete event log, and is bounded by the Budget.

### t9 — arm explore CLI verb: wire into the arm noun group with --port/--map/--apply/--json in arm101/cli/_commands/arm.py

- depends on: t8, t4, t6
- covers: c1, c2, c7, h6, h7, h12
- acceptance:
  - arm101 arm explore registers under the arm noun; one invocation runs the engine and writes both the JSONL log and the compact map, printing the map path (text and --json); motion is gated (--apply required in non-TTY, ignored under a TTY) and honors --port and --map; it does NOT touch the arm flex handler (grep-asserted unchanged); failures raise CliError with no traceback.

### t10 — Docs surface for arm explore (lockstep): explain catalog entry, overview _VERBS, learn text, README section

- depends on: t9
- covers: c3, c5, h8
- acceptance:
  - the explain catalog has an (arm, explore) entry that renders; overview.py _VERBS lists explore; learn.py mentions explore; README documents the before/why (no persisted reachability artifact existed) and the produce+store+query scope; test_every_catalog_path_resolves passes.

### t11 — Hardware validation run-log: live fresh-bench arm explore run on the follower with ground-truth check (HUMAN-GATED)

- depends on: t9
- covers: c6, h11
- acceptance:
  - on the physical follower (/dev/ttyACM1, never ttyACM0) a real arm explore run produces a map whose blocked configs match manual-flex ground truth, including at least one combination case where perturbing another joint unblocks a previously-blocked joint, with zero error=32 across the run; results recorded in docs/hardware-validation-arm-explore.md.

## Risks

- [unknown_nonblocking] Deeper multi-joint escape is combinatorial. The pruning depth/breadth caps plus the global budget must be tuned to terminate in acceptable wall-time WITHOUT missing common single-perturbation unblocks. Exact caps are TBD and must be validated on hardware; a bad choice either misses reachable space or makes runs impractically long. (task t7)
- [unknown_nonblocking] Default grid resolution (per-joint tick-bucket size) is TBD: too coarse misses narrow passages, too fine explodes the number of physical moves. Tune on hardware (frame v1). (task t2)
- [unknown_nonblocking] Budget / thermal-guard default numbers (max moves, max wall-time, per-motor temperature ceiling) are TBD: must be validated as safe + practical on the physical follower (frame v2). (task t5)
