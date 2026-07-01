# arm101 now maps its own reach: 'arm explore' safely drives the SO-101 follower through its joint-space, auto-detecting every self/environment contact and recording the reachable envelope to a map file — shipped with a sensible default map and fully overridable per bench.

> arm101 now maps its own reach: 'arm explore' safely drives the SO-101 follower through its joint-space, auto-detecting every self/environment contact and recording the reachable envelope to a map file — shipped with a sensible default map and fully overridable per bench.

## Audience

- An operator or agent bringing up an SO-101 follower who must know its safe reachable joint-space before running autonomous motions, plus downstream motion code that queries the resulting map.

## Before → After

- Before: Today the only motion verbs are read/flex/setup; the arm's reachable envelope is unknown. You find limits by manually flexing one joint at a time and eyeballing contacts, and that knowledge is lost between sessions — nothing persists a map of where it is safe to move.
- After: Running 'arm explore --apply' autonomously walks the follower through its joint-space at safe speed (via the overload-safe gentle_move), detects every self/environment contact from real load, and writes a resumable, overridable reachability map of reachable ranges plus blocked configurations.

## Why it matters

- Autonomous motion needs to know which joint configurations are reachable without self-collision; discovering that by hand is slow, per-bench, and forgotten between sessions. A persisted map turns safe-motion planning from tribal knowledge into a queryable artifact.

## Requirements

- All motion during explore goes through the overload-safe gentle_move (speed 150, torque cap, graceful error=32); explore never trips a hardware overload latch, and touches only the follower on the given --port, never Reachy on ttyACM0.
  - honesty: A full explore run on the follower completes with zero error=32 latches (rides gentle_move's back-off/cap), and the process only ever opens the --port passed (never ttyACM0).
- Every contact is recorded with the full joint configuration, the moving joint, and the load magnitude, appended durably as it happens, so a killed run resumes without re-probing already-mapped cells (checkpoint/resume).
  - honesty: Killing the explore process mid-run and restarting it resumes from the last checkpointed state and does not re-issue motion for cells already recorded in the map/log.
- The map is queryable offline: 'is configuration X reachable?' is answerable from the file alone without re-running hardware.
  - honesty: A query path returns reachable|blocked for a given joint configuration reading only the map file — no serial port opened, no motor moved.
- Combination-escape: when a joint is blocked at a cell, the explorer perturbs other joints and retries, so it records reachable space beyond the first per-joint contact rather than stopping at a single min/max limit.
  - honesty: On a configuration where joint A is blocked until joint B moves first, explore records the reachable A range reached AFTER perturbing B, not merely A's first-contact limit.
- The default map ships bundled with the CLI and loads when no user file is present; a documented --map <path> (or override path) fully replaces it with the user's own bench map.
  - honesty: With no user map present the CLI loads the bundled default; passing --map <path> loads that file instead, and neither run mutates the bundled default in place.

## Honesty conditions

- 'arm explore' ships as a new arm verb that, on the physical follower, drives safe motion, detects contacts, and writes an overridable reachability-map file demonstrated end-to-end.
- One command yields the map for that audience and downstream code can load it — no manual per-joint measurement step is required of the operator.
- Verifiable that today no persisted reachability artifact or explore verb exists in arm101 — limits are only found by manual flexing and lost between sessions.
- After a run a map file exists on disk holding reachable ranges + blocked configurations discovered from real load, and every motion went through gentle_move (no raw goal writes).
- A downstream consumer can answer 'is configuration X reachable?' from the map alone, without re-running hardware or re-deriving limits by hand.
- On a fresh bench the produced map matches manual-flex ground truth on the blocked configs, including at least one combination case where perturbing another joint unblocks a previously-blocked joint.
- v1 ships explore + a query/report of the map but does NOT modify 'arm flex' to consume it; the flex-gate stays a separate follow-up (v4).
- Every explore run terminates within its configured move/time budget and never attempts full 6-DOF enumeration; the map is labelled best-effort, not a completeness guarantee.

## Success signals

- On a fresh bench an operator runs 'arm explore', and the resulting map — compared against manual flexing — correctly identifies the blocked configurations, INCLUDING at least one combination case where perturbing another joint first unblocks a previously-blocked joint.

## Scope / boundaries

- Not a motion planner. 'arm explore' PRODUCES and STORES the reachability map; consuming it to gate 'arm flex' targets (refuse/warn on unreachable) is a named follow-up, not v1.
- Not exhaustive 6-DOF grid enumeration. Exploration is bounded — flood-fill from a safe home over a coarse discretized grid with a move/time budget — so the map is a best-effort discovered envelope, never a completeness guarantee.

## Non-goals

- The bundled DEFAULT map is SELF-collision only (intrinsic to arm geometry, portable across benches). Environment/obstacle collisions are inherently per-bench and are the user-override layer, not the shipped default.

## Decisions

- Map storage (user q1) = DUAL: an append-only JSONL event log (raw contact/probe events — provenance + resume) PLUS a derived compact queryable map (per-joint reachable ranges + a sparse list of blocked joint-combinations, packed). The compact map is what downstream code queries; the JSONL is the source of truth it is rebuilt from.
- Combination-escape (user q2) = DEEPER multi-joint coordinated search in v1: when a joint is blocked, search over multi-joint perturbation vectors to free it — NOT merely single-joint local perturbation. Because that search is combinatorial it MUST be pruned (perturbation depth/breadth caps) and hard-bounded by a global move/time budget, so every run provably terminates and never wanders the full 6-DOF space.
- Default-map scope (user q3) = self-collision only. The bundled default maps arm-geometry self-collision (portable across every SO-101 follower); per-bench environment obstacles are the user-override layer (--map). Confirms c9.

## Open / follow-up

- Consuming the map to gate 'arm flex' targets (refuse/warn when a requested config is outside the discovered envelope) — follow-up once the map artifact exists.
