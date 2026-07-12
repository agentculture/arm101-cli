# Changelog

All notable changes to this project will be documented in this file.

Format follows [Keep a Changelog](https://keepachangelog.com/). This project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.22.1] - 2026-07-12

### Fixed

- `gentle_move`: a **corrupt `Torque_Limit` read** could raise out of the cleanup and mask the
  real failure. Hit on hardware — `read_torque_limit` returned 2048 on a healthy motor whose
  register actually held 500. The read *succeeded*; it just returned nonsense, and 2048 is
  outside the register's own 0–1000 band, so it cannot be a torque limit at all. That value was
  stashed as the pre-move limit, and the `finally` then tried to write it back, where the servo's
  range check rejected it and raised a `CliError` **out of the cleanup** — masking whatever the
  move had actually been doing.
  Now validated at the point of READ, not at the point of write: if we do not know the pre-move
  value we say so, rather than restoring a fiction. And the restore can no longer raise at all —
  with the pre-move value unknown, the conservative cap simply stays in place. A joint left
  slightly under-torqued is a nuisance; a cleanup that raises is a lie about why the move failed.

## [0.22.0] - 2026-07-12

### Added

- `docs/hardware-rezero-run-2026-07-12.md` — the hardware run-log. The re-zero is PROVEN on the follower: the STS3215 reduces the corrected position modulo 4096, so a homing offset genuinely RELOCATES the encoder seam. Same joint, same hand sweep, before/after: offset 0 gave monotonic=False with 1 discontinuity; offset 1073 gave monotonic=True with 0 discontinuities over 2196 ticks. It survived a power cycle. The t5 spike GO-WITH-CAVEAT is now closed by measurement.

### Changed

- `REZERO_ARCS` is derived from walls the ARM measured itself. `gentle_move` was driven past the known travel and let the load-watch find each wall by feel; both contacts saturated at the 500 torque cap. The arm out-measured the human on both sides (raw 251 vs 218, and 2061 vs 2107) because it presses to a fixed load every time instead of to whatever felt firm. The arc is INSET from those walls by a margin, so a harder push can never make the table contradict the arm.
- The rezero tests DERIVE every expectation from the arc table instead of copying it. ~125 arc-coupled literals were hard-coded across two files, so re-measuring a table that exists to be re-measured broke 34 tests. Changing a wall or the margin now leaves the suite green.
- Docstrings and the `explain` catalog no longer quote ticks at all — a document that names a measurement is a document that goes stale, and `explain arm rezero` is what prints to whoever is standing at the arm.

### Fixed

- `arm rezero`: the unreachable arc was in the WRONG FRAME. It held REPORTED ticks (read off a servo already carrying the factory offset) and used them as RAW. Every SO-101 ships with `Ofs = +85`, not the factory 0 the spike assumed — and that is precisely WHY `elbow_flex` wraps: the seam sits where `Actual == Ofs`, i.e. at raw 85, which is BELOW the unreachable arc and therefore INSIDE the joint travel. The old target landed inside the true arc anyway, with ~866 ticks of margin: it worked by luck, and every read-back looked correct.
- `arm rezero` refused outright on a servo holding the factory 85 — that is, on every fresh SO-101. Now that `raw = (reported + offset) mod 4096` is known, any readable frame converts, and the goal is stated as a PLACE (the seam is out of the travel) rather than a magic number: an arm already holding a seam-evicting offset is a clean no-op, not a rewrite.
- `bus`: a dropped packet raised a bare `IndexError` from inside the vendor SDK instead of a `CliError`, leaking a raw traceback — hit live on a healthy bus. Reads now convert SDK faults to `CliError` and retry (bounded); WRITES ARE NEVER RETRIED, because a failed write may in fact have landed (that ambiguity is exactly how #21 and #38 arose). `OverloadError` is never retried — it is a latched state, not a dropped packet.
- `bus`: a read issued immediately after an EEPROM write could return GARBAGE — a `read_position` 0.2s after `write_offset` returned 0 while the servo genuinely held 3387. A plausible-looking wrong value is far more dangerous than an error. EEPROM writes now settle before the next read.

## [0.21.0] - 2026-07-12

### Added

- `arm rezero <joint>` — a gated EEPROM write that evicts the encoder seam from a wrapping joint travel. Writes only the STS3215 `Ofs` register (addr 31, sign-magnitude on bit 11, range +/-2047, EEPROM) via the unlock -> write -> relock dance PR #21 established. The offset is DERIVED from the joint measured unreachable arc, never typed.
- `arm rezero <joint> --verify` — the seam-eviction proof. Torque off; the operator hand-moves the joint through its travel while the verb polls position and asserts MONOTONICITY. Reading the offset back only proves it was APPLIED; only the sweep proves the seam MOVED.
- `arm101/hardware/bus.py`: `read_offset` / `write_offset` / `encode_offset` / `decode_offset`, plus a `FakeBus` that models the offset effect on reported position. `arm read` now shows the signed offset (read-only).
- `docs/spikes/sts3215-offset-register.md` — the triple-sourced research behind the register, including the range arithmetic and the one semantic that is still unproven.
- `docs/hardware-rezero-procedure.md` — the followable procedure, including the mandatory power-cycle.

### Changed

- `wrist_roll` gains a software soft limit (100, 3995) whose dead arc CONTAINS the 4095->0 seam,
  and `arm_spec.resolve_bounds()` now INTERSECTS each joint's EEPROM limits with its soft limit.
  Every site that resolved move bounds from EEPROM (`arm flex`, `arm explore`'s grid, the demo
  sweep) routes through it, so the dead arc is genuinely unreachable rather than merely declared.

### Fixed

- `arm rezero` tolerates a motor latched in overload. Planning does register READS, and on a
  latched servo reads raise too — so the verb aborted before reaching the only code that clears
  the latch. Since `elbow_flex`'s unreachable arc is measured by driving the joint into a wall
  (exactly how a servo latches), this fired on the documented procedure every time. The latch is
  now cleared and planning retried once; the recovery is conditional, so a healthy holding joint
  is never silently de-energised. Found by qodo on PR #40.

## [0.20.1] - 2026-07-12

### Fixed

- `arm profile`: refuse a `--threshold` at or above the torque cap. `present_load` SATURATES at
  the servo's `Torque_Limit`, which `gentle_move` caps to 500 for the duration of every move, and
  contact requires load greater than threshold — so a threshold of 500 or more can never fire,
  however hard the arm pushes. Every probe would report no contact, and the verb would then
  declare the first rung a void run ("nothing there to detect") while the joint was pressed hard
  against a very real obstacle. Now rejected before the bus is even opened. The ceiling is exposed
  as `gentle.CONTACT_LOAD_CEILING` rather than duplicated as a literal. Found by qodo on PR #39.
- `profile_joint`: cleanup no longer masks the real fault. The `finally` retreat and the torque
  release suppressed only `CliError` — but the failure they exist to survive is a dead port, and
  pyserial's `SerialException` arrives from the SDK unwrapped, so it is not a `CliError` at all.
  The narrow suppress let the cleanup's own failure REPLACE the hardware fault the operator needed
  to see. Both now survive any `BaseException` (with `SystemExit` alone re-raised), matching
  `safety._release_motor`. Found by qodo on PR #39.

## [0.20.0] - 2026-07-12

### Added

- `arm profile <joint>` — a gated verb that ramps a joint's Goal_Speed and finds the highest
  speed at which CONTACT DETECTION STILL WORKS, not merely the highest speed the servo
  survives. At every candidate speed it drives the joint into a real, unreachable target and
  requires the shipped `gentle_move` to report `contacted=True`. A rung whose peak load crossed
  the joint's threshold (so it demonstrably MET the obstacle) but where the stall rule never
  fired is REJECTED as the ceiling. A probe that meets nothing raises: a speed "validated" on
  free motion alone proves nothing, and the code enforces that rather than documenting it.
  Records per joint: safe speed, measured ticks/second, and motion-onset latency.

### Changed

- `gentle_move` gains an optional, passive `TravelObserver` seam (default `None`, zero behaviour change) so a caller can watch the exact (position, load) sample stream the real `_StallDetector` is fed, without forking the poll loop.

## [0.19.1] - 2026-07-12

### Fixed

- `arm setup`: transfer torque ownership when the servo id write LANDS but the subsequent EEPROM relock fails. `write_id_baudrate()` re-locks from its `else:` branch, outside the try, so it can raise after the servo has already moved to its new address — leaving the guard owning a dead id, aiming the release sweep at a motor that no longer answers, and reporting a false `may still be energised` alarm about a motor that is limp and merely renamed. The failure path now probes (`bus.scan`) for the new address and moves the claim only on positive evidence. Found by qodo review on PR #38.
- `TorqueGuard`: a failing release announcement can no longer replace the original exception. The `on_release` hook ran inside `contextlib.suppress(Exception)`, which does not cover `BaseException` — so a second Ctrl-C landing while the CLI printed `released motors 1-6` escaped `__exit__` and became the error the operator saw instead of the real one. The hook now mirrors `_release_motor`s asymmetry exactly (swallow `KeyboardInterrupt`, re-raise `SystemExit`), and the torque release always happens BEFORE the announcement, so a broken diagnostic never costs the safety action. Found by qodo review on PR #38.

## [0.19.0] - 2026-07-12

### Added

- `arm101/hardware/safety.py` — `torque_guard`, a context manager that owns the motors a motion verb energizes and releases them on any ABNORMAL exit (unhandled exception, bus fault, SIGINT). Per-motor independent and survives its own failure: the bus that just threw is the bus the release must talk to. Closes #33.

### Changed

- `arm explore`, `arm flex`, `arm flex --demo` and `arm setup` now run inside `torque_guard`. Semantics are HOLD ON SUCCESS, RELEASE ON ABNORMAL: a successful `gentle_move` keeps its deliberate stop-and-hold (a gripper that closed on an object does not drop it), so a powered arm at process exit is always a deliberate state, never an accident. A clean exit issues ZERO release writes.
- The release routes through `bus.clear_overload()` rather than `enable_torque(motor, False)`. Both are the same wire write (Torque_Enable=0, addr 40) but `clear_overload` is overload-TOLERANT: a latched servo tags every packet response with the overload bit — including the response to the very write that clears the latch — so `enable_torque` would raise on exactly the motor that most needs de-energizing.

### Fixed

- `FakeBus.clear_overload` now routes its torque-disable through `FakeBus.enable_torque` instead of recording the write separately. On the wire the two are the identical `write1ByteTxRx(motor, 40, 0)`, so a fake in which a subclass can intercept one and silently miss the other models a bus that does not exist — and would let a test prove a torque-disable never happened when on real hardware it did.

## [0.18.1] - 2026-07-12

### Added

- docs/specs/2026-07-12-arm-explore-now-maps-the-arm-s-real-joint-space-it.md — converged devague spec for making `arm explore` work on the real arm, covering issues #33 (motion verbs leave the arm energized on an abnormal exit), #35 (elbow_flex and wrist_roll encoders wrap mid-travel), #34 (the search grid is derived from factory EEPROM bounds and a global bucket size), plus a new hardware-measured speed/limit profile. Four waves: safety release; encoder linearity (elbow_flex re-zeroed, wrist_roll soft-limited); speed profile; grid rebuilt on the measured reachable space.

### Changed

- Spec records the decision that `wrist_roll` CANNOT be fixed by re-zeroing — a joint whose travel covers the whole circle has no zero that moves the 4095->0 seam out of its travel — so it is soft-limited to create a dead arc containing the seam, while `elbow_flex` (which has real walls) is re-zeroed.

## [0.18.0] - 2026-07-12

### Added

- `scripts/probe_gentle_timing.py` — the diagnostic that reproduces the premature-return bug on hardware.
- Regression guards: a fake bus that models servo travel latency honestly (`tests/_fakes.py::ServoModelBus`), plus overload-safety and caller-contract guards. The previous test doubles teleported the shaft to its goal and materialised load on the commanding call, leaving the whole suite structurally blind to this class of bug.

### Changed

- Hardware run-log: the t3 regression baseline (contact going undetected on the pre-fix code), the t8 acceptance run that inverts it, the per-joint free-motion load profile, four real boundaries discovered on the follower, and the goal-tether experiment (tried, measured, removed — it starves gravity-loaded joints so they stall in open space).
- `gentle_move`'s poll/stall tuning is grouped into a single `LoadWatch` parameter object (`watch=`) instead of six loose keyword arguments, and the travel loop is extracted into `_travel` with the onset/stall bookkeeping owned by a `_StallDetector`. Behaviour is unchanged — re-verified on the follower, contact still caught mid-move, stopped, backed off and held at zero load — but `gentle_move` drops from 17 parameters to 12 and its stepping loop from a cognitive complexity of 41 to 3.

### Fixed

- `gentle_move` now MEASURES the arm instead of assuming it. It polls `present_position`/`present_load` during travel, terminates only on a measured condition (arrival within tolerance, a detected contact, or a timeout) rather than on "commanded ticks exhausted", and reports `final_position`/`contact_position`/`contact_load` as read-back values. Proven on the follower: a 400-tick move that used to return in 71 ms claiming arrival — while the joint had not moved a tick — now returns after 2755 ms reporting the position the servo actually reads back.
- Contact detection now works at all. It was blind to any contact the move itself caused, because every load sample was taken in the ~100 ms dead window before the servo mechanically responds; it could only ever catch a joint that was ALREADY loaded. Contact is now `load > threshold` AND the joint no longer advancing (a stall), armed only once the joint has actually moved — so a free-motion acceleration transient is not mistaken for contact, and the ~100 ms onset latency does not fire a phantom contact on every move.
- `DEFAULT_CONTACT_THRESHOLDS` re-derived from measured free-motion load profiles. The previous values were tuned against the pre-fix code's near-zero reads: `wrist_roll`'s threshold of 180 sat BELOW its own 300 free-motion peak, so a correctly-sampled load watch would have called contact on every move that joint made.
- A motor whose overload latch was ALREADY tripped raised on `gentle_move`'s very first bus call — before it had read a position — so it reported `final_position: None`, which then leaked into `demo`'s report where the contract says `int`. The latch is now cleared and the joint's position genuinely re-read, so the caller gets a real measurement; if the bus is still unreadable it stays `None` rather than inventing a value, and `demo` keeps the last position it actually measured.
- A `LoadWatch` that would disable the detection it configures can no longer be constructed. `stall_eps <= 0` made the stall condition unsatisfiable — contact detection silently switched **off**, so the arm would have pushed until the torque cap; `stall_samples < 1` removed the stall gate, so an acceleration transient read as contact; a negative `poll_interval` reached `time.sleep()` as a raw `ValueError`. All now raise a `CliError` with remediation at construction.

## [0.17.1] - 2026-07-12

### Added

- spec: `docs/specs/2026-07-11-arm101-s-gentle-move-now-actually-measures-the-arm.md` — converged devague frame for making `gentle_move` measure the arm instead of assuming it (load sampled DURING travel; termination on measured arrival).

### Fixed

- Documented a blocker proven on hardware (t12, 2026-07-12): `gentle_move` reads `present_load` ~1ms after each `write_goal_position`, i.e. before the servo has mechanically responded, and tracks the COMMANDED tick rather than the measured position — so it returns before the arm has moved (71ms vs ~900ms of real travel on a 400-tick `wrist_roll` move). Contact detection is therefore blind to contacts caused by the move itself, and every `arm explore` reachable verdict in v0.16/v0.17 is unverified. Fix is specced, not yet implemented.

## [0.17.0] - 2026-07-01

### Added

- arm explore: per-joint contact thresholds (#26). New DEFAULT_CONTACT_THRESHOLDS table in arm_spec.py (hardware-tuned per joint) plus resolve_contact_thresholds() resolver.
- arm explore --threshold-joint JOINT=LOAD (repeatable) to override one joint's contact threshold.
- arm explore --threshold-file PATH — a JSONL file of per-joint contact thresholds ({"joint": name, "threshold": N} per line).

### Changed

- arm explore --threshold is now a blanket all-joints override (was the sole threshold); each joint otherwise resolves independently with precedence --threshold-joint > --threshold > --threshold-file > built-in per-joint default. explore engine now threads a per-joint threshold tuple to both move call sites (flood-fill + escape probe). Dry-run plan now surfaces per-joint thresholds.

## [0.16.0] - 2026-07-01

### Added

- `arm explore` — a new arm noun verb that flood-fills and maps the SO-101 follower's reachable joint-space. Drives every move through the overload-safe `gentle_move`, detects self/environment contacts from real load, and writes a resumable dual artifact: an append-only JSONL event log plus a derived compact reachability map (per-joint reachable ranges + sparse blocked joint-combinations).
- New `arm101/explore/` module (zero runtime deps): `types` (JointConfig/GridSpec/ContactEvent/ReachMap), `grid` (tick<->cell discretization), `log` (JSONL event log + resume-set), `reachmap` (build-from-events, offline `is_reachable` query, save/load), `budget` (move/time caps + thermal guard that guarantees termination), `default_map` (bundled self-collision default + `--map` override), `escape` (deeper multi-joint coordinated combination-escape, pruned + budget-bounded), and `engine` (the flood-fill explorer).
- Bundled permissive self-collision default map shipped as package data, overridable per bench via `--map`.
- `arm explore` flags: `--role`, `--port`, `--map`, `--threshold`, `--max-moves`, `--resolution`, `--apply`, `--json`; gated motion via the three-mode consent (dry-run/interactive/agent) and a live per-joint thermal guard.
- Explorer bus-health hardening (found during live follower validation): each probe limps its joint afterward (accumulated holding-torque otherwise wedges the register-48 bus comms after ~6 held motors), and a transient probe comm error is retried once then skipped (counted in a new `errors` field) instead of aborting the whole run. Validated on hardware: a 40-move run completed with zero comm errors.

### Changed

- Documented `arm explore` in the explain catalog, both overview surfaces, learn, and README (produce+store+query scope; consuming the map to gate `arm flex` is a follow-up).

## [0.15.0] - 2026-07-01

### Added

- Bus layer: `OverloadError(CliError)` + `is_overload()` classifier (STS3215 status bit5), `read_torque_limit`/`write_torque_limit` (RAM addr 48) and `clear_overload()` on MotorBus/FeetechBus/FakeBus, plus a FakeBus overload test seam
- `overloaded` field surfaced across `arm read`/`arm flex`/`arm flex --demo` (JSON + text markers) so an overload is a consumable, structured outcome

### Changed

- `gentle_move`/`compliant_move` default goal-speed lowered 400 -> 150; `gentle_move` caps RAM `Torque_Limit` (~50%) during contact moves and restores it in a finally
- `gentle_move`/`compliant_move`/`demo_sweep` now treat a mid-move STS3215 overload (error=32) as a reported contact/fault: release torque to clear the latch and return `overloaded=True` instead of propagating a raw read error
- Internal: extracted helpers to clear SonarCloud cognitive-complexity/nesting smells (no behaviour change) — `gentle_move` gains `_require_gentle_args`/`_step_direction`, `demo_sweep` gains `_sweep_targets`, and `_emit_flex_demo`'s nested overload/contact ternary is now an explicit `if/elif/else`

### Fixed

- `arm flex`/`arm flex --demo` no longer crash with a raw `error=32` env error on a dynamic overload; the joint is auto-recovered (torque released) and the outcome is reported
- Gentle contact-detection now thresholds on `present_load` **magnitude** (new `bus.load_magnitude()` masks the STS3215 direction bit 10 / `0x400`); previously a negative-direction load read as ≥1024 and tripped a spurious contact on the first step — found on the physical follower during the t7 hardware run

## [0.14.1] - 2026-07-01

### Added

- docs/specs: converged /think spec `arm101 arm motion is overload-safe` — gentler motion defaults, RAM Torque_Limit capping during contact moves, and graceful STS3215 error=32 handling (coordinated move --joints verb parked as follow-up)
- docs/hardware-validation-arm-read-flex.md: t9 live-follower run-log for issue #20 — arm read / doctor --probe / per-joint + coordinated (GroupSyncWrite) flex validated, plus the two STS3215 overload reproductions that seed the overload-safe-motion spec

## [0.14.0] - 2026-07-01

### Added

- `arm read` — retry-tolerant whole-arm live-state snapshot (all six joints: position, load, speed, voltage, temperature, torque); a single flaky joint is marked partial/failed without aborting the snapshot (#20).
- `arm flex <joint> --to <tick>` — bounded per-joint move, clamped to the joint calibrated min/max, gated by the three-mode consent core (dry-run / interactive / agent --apply) (#20).
- `arm flex --demo` — scripted safe-exploration sweep over a conservative fraction of each joint reachable range, aborting cleanly on contact (#20).
- `--gentle` + `--threshold` on motion — load-watch back-off-then-hold: on contact the joint reverses a bounded number of ticks off the contact point and holds, instead of pushing through (#20).
- `doctor --probe [--port]` — multi-baud id/read probe that classifies each id at each supported baud as SUCCESS / CORRUPT (collision/duplicate-id) / TIMEOUT (absent), so a silent or misbauded bus is diagnosed instead of reported as no servo (closes #18).
- `FeetechBus.write_acceleration` (addr 41) and `FeetechBus.write_goal_speed` (addr 46) low-level motion primitives, plus `arm101.hardware.motion` / `gentle` / `demo` / `arm_read` / `baud_probe` modules (zero new runtime dependencies).

### Fixed

- `set-motor-id` and `setup-motors` now read the id back after the EEPROM write and fail loudly (exit 2) if it did not persist — the defense that catches the silent factory-id revert; `setup-motors` no longer continues the 6-to-1 walk past a motor that did not take its new id.
- Realigned `uv.lock` with the `pyproject.toml` version (was pinning a stale version).
- `doctor --probe` now pre-flights the optional Feetech SDK (`scservo_sdk`) and fails with a clear "not installed" error + `pip install` hint (exit 2) when it is absent, instead of degrading the missing SDK into a misleading "silent bus / no servo answered at any baud" diagnosis that exited 0 (qodo #22-1). New `arm101.hardware.bus.sdk_available()` / `require_sdk()` helpers back the pre-flight.
- `arm flex --threshold 0` is now honored as an explicit override instead of collapsing to the default 250 — the `x or DEFAULT` fallback treated the valid falsy `0` as unset (qodo #22-2).
- `arm flex --demo --to <tick>` is now rejected as a contradictory combination rather than silently ignoring `--to` and running the demo sweep anyway (qodo #22-3).
- `gentle_move()` now validates `step > 0` and `backoff >= 0` up front, so a non-positive step (which never advances toward the target) fails fast with a clear error instead of spinning forever while issuing bus writes (qodo #22-4).

## [0.13.2] - 2026-07-01

### Fixed

- `set-baudrate` no longer fails (and no longer strands the motor at Lock=0) when the target baud differs from the current bus baud. STS3215 fw 3.10 applies a baud change immediately, so `FeetechBus.write_baudrate()` now switches the host serial port to the new baud (mirroring `open()`) before re-locking the EEPROM, so the re-lock reaches the motor at the baud it is now listening on (qodo #2).
- EEPROM writes in `FeetechBus.write_id_baudrate()` and `write_baudrate()` are now exception-safe: if any id/baud write fails after the EEPROM is unlocked, a best-effort re-lock is attempted before the original error propagates, so a failed call never leaves the motor at Lock=0. In `write_id_baudrate()` the re-lock targets the new id only once the id write has actually committed; otherwise it targets the original motor id (qodo #3).

## [0.13.1] - 2026-06-30

### Fixed

- EEPROM id/baud writes now open the STS3215 Lock register (addr 55) before writing and restore it after, so an assigned id/baud persists across a power-cycle. Previously the write took effect on the live register but was never committed to EEPROM, so a motor silently reverted to its stored id (factory default 1) on the next power-up — which left an assembled arm with all motors colliding at id 1 and the bus apparently dead.

## [0.13.0] - 2026-06-29

### Added

- **`arm` noun group** with `arm setup <role>` (follower|leader) and `arm overview`. `arm setup` is number-free, role-aware arm setup: it drives the existing gated three-mode-consent `setup-motors` walk (ids 1-6 @ 1 000 000) and, per motor, records a role-correct motor-catalog entry (`F1`-`F6` / `L1`-`L6` with `servo_model` + `gear_ratio`) sourced from `arm_spec` — so a full arm is set up AND catalogued with zero numbers typed. Dry-run writes nothing.
- **`arm101/hardware/arm_spec.py`** — single-source per-role motor map keyed by role -> per-joint `{id, baud, servo_model, gear_ratio}`. Every value is cited, not assumed: ids (1-6) and baud (1 000 000) from LeRobot `so_follower.py`/`so_leader.py` + `feetech.py DEFAULT_BAUDRATE` (identical across both roles); per-joint `servo_model` + `gear_ratio` from the Seeed SO-101 BOM wiki (follower uniform `1:345`; leader mixed `1:191`/`1:345`/`1:147`).
- Lockstep docs for the `arm` noun/verbs (`explain` catalog + `overview._VERBS` + `learn`); `teken cli doctor . --strict` stays green (26/26).
- Hardware validation run-log template for `arm setup follower` (`docs/hardware-validation-arm-setup.md`).
- Single-source invariant + scope-guard test suites (joint->id literal lives in exactly one place; zero new runtime deps; calibration not yet gear-aware).

### Changed

- `calibrate`, `setup-motors`, and `profiles` now derive the SO-101 joint->id map (and setup-motors' default baud) from the single-source `arm_spec` instead of three duplicated hardcoded literals. Behavior-preserving — the resolved ids/baud are identical before and after, asserted by tests.

### Fixed

- **`arm setup <role>` catalogued the pre-write id** (PR #19 review, Qodo): the per-motor catalog entry saved `detected_id` from `from_id` (the id detected *before* the EEPROM write), so a motor programmed to id 6 could be recorded as `detected_id=1`. It now records the assigned id (`motor_id`), so `motors.json` reflects the post-setup state. Tests updated to assert the assigned id (1-6) for both follower and leader.
- Code-quality cleanups flagged by SonarCloud on the new `arm`/`arm_spec` modules (behavior-preserving): `cmd_arm_overview` returns `None` instead of a constant `0` (the dispatcher maps `None` -> exit 0; S3516); the dry-run plan builder is extracted to `_emit_dry_run_plan` to lower `cmd_arm_setup`'s cognitive complexity (S3776); the repeated follower servo-model literal is hoisted to a `_FOLLOWER_SERVO_MODEL` constant and the repeated `arm` `--json` help text to a `_JSON_HELP` constant (S1192); and two implicitly-concatenated f-strings are merged (S5799).

## [0.12.0] - 2026-06-29

### Added

- **`set-baudrate` verb** — change the EEPROM baud rate of the single connected
  Feetech STS3215 without altering its servo ID (`addr 6` only, `addr 5`
  untouched). Supports the same three-mode consent as `set-motor-id`: (1) TTY
  interactive with `yes` confirmation; (2) non-TTY without `--apply` emits a
  markdown dry-run plan (zero writes); (3) non-TTY with `--apply` executes
  the write (1-step tier). Shows a BEFORE card (register snapshot on stderr)
  and opens a fresh bus at the new baud for an AFTER card after the write.
  Headless writes are attributed and appended to `~/.arm101/audit.log`.
  Valid baud rates: 38400, 57600, 76800, 115200, 128000, 250000, 500000,
  1000000 (validated against `BAUD_MAP`; invalid value → `EXIT_USER_ERROR`
  before any bus is opened). Verified on hardware (STS3215 fw 3.10): the baud
  change takes effect **immediately**, so the after-card opens at the new baud
  and succeeds. Note the CLI always opens at 1000000, so once a motor is moved
  off 1000000 you must reach it at its new baud (e.g. a direct
  `FeetechBus(port, baudrate=…)`) to change it back.
- **`FakeBus.write_baudrate` + `baud_writes`** — in-memory implementation of
  the new `MotorBus.write_baudrate` abstract method; records each call in
  `baud_writes` so tests can assert the baud was written without touching the
  ID register (`eeprom_writes` stays empty). Mirrors `FeetechBus` by rejecting
  an unsupported baud (`CliError(EXIT_ENV_ERROR)`), so a value that would fail
  on hardware also fails against the fake.
- **`FeetechBus.write_baudrate`** — real implementation writing only the
  `Baud_Rate` EEPROM register (addr 6, 1 byte) via the Feetech SDK; validates
  the supplied baud against `BAUD_MAP` and raises `CliError(EXIT_ENV_ERROR)`
  for unsupported values.

## [0.11.0] - 2026-06-29

### Added

- **`setup-motors`: per-motor port auto-detection (fixes #12, #14)** — the bus
  is now re-detected (via `_detect_one_motor`) fresh for each motor in the 6→1
  walk. Unplugging one motor and plugging in the next can change the
  `/dev/ttyACM*` path; the old single-bus design left a stale file descriptor
  that died with `(5, 'Input/output error')`. Re-detection handles USB
  re-enumeration transparently. Pass `--port` to override with a fixed path.
- **`setup-motors --baudrate` flag** — validated EEPROM baud rate (default
  `1_000_000`). Valid values: 38400, 57600, 76800, 115200, 128000, 250000,
  500000, 1000000. Invalid value raises `CliError(EXIT_USER_ERROR)` before any
  bus is opened. Plumbed through every audit record and the final summary.
- **`setup-motors` before/after motor cards** — for each motor in the walk, a
  read-only register snapshot (using the shared `_show_info`) is shown BEFORE
  and AFTER the EEPROM write, both on stderr. The card now includes a
  human-readable baudrate line (`baudrate : 1,000,000 bps (index 0)` or
  `unknown (index N)` for unmapped indices) in addition to the raw index.
- **`BAUD_MAP` and `BAUD_INDEX_TO_BPS` exported from `arm101.hardware.bus`** —
  the `_BAUD_MAP` dict, previously buried inside `FeetechBus.write_id_baudrate`,
  is now a public module-level constant so `setup-motors` and `_show_info` can
  validate/render baudrates without duplicating the table.

### Changed

- **`setup-motors --port` default changed to `None`** (was `/dev/ttyACM0`) —
  auto-detection is now the default; pass `--port` to pin a fixed device.
- **`setup-motors --current-id` semantics changed to a safety assertion** — the
  flag is no longer the address used to target the motor; the detected id is.
  If `--current-id` is provided and differs from the auto-detected id, the walk
  aborts with `CliError(EXIT_USER_ERROR)`. Omit the flag to accept any detected
  id.
- **`_show_info` motor card enhanced with baudrate in bps** — the `baud index`
  line is replaced by `baudrate : N bps (index I)`, benefiting all verbs that
  show the card (`calibrate-motor`, `set-motor-id`, `center-motor`,
  `setup-motors`).
- **`setup-motors` interactive prompt no longer asserts a false current id** —
  when `--current-id` is omitted (auto-detect), the connect guidance dropped the
  misleading "currently at id 1" claim; it only names a specific id when one is
  asserted.

### Fixed

- **`setup-motors` USB re-enumeration bug (#12, #14)** — the old single-bus
  design reused one `FeetechBus` across all 6 motors. Unplugging and
  re-plugging between motors caused `(5, 'Input/output error')`. Fixed by
  per-motor detection.
- **Explicit `--port` open errors are surfaced, not masked** — when the operator
  names a port, a failure to open *that* port now propagates the real
  `"Failed to open serial port …"` `CliError` instead of being swallowed into
  the generic "No STS3215 servo detected" message. Auto-detection still skips
  busy/unopenable ports as before. Affects every verb that detects through
  `_detect_one_motor` (`calibrate-motor`, `set-motor-id`, `center-motor`,
  `setup-motors`).

## [0.10.0] - 2026-06-29

### Added

- **Memory-discipline "Conventions and workflow" section in `CLAUDE.md`** — a
  per-task *recall-before / remember-after* convention (scope localized to this
  repo's nick) so the vendored `remember` / `recall` skills are actually used,
  not just present: `/recall` before non-trivial work to build on prior
  decisions instead of re-deriving them, and `/remember` when a non-obvious
  decision, constraint, fix-and-why, or hard-won gotcha surfaces. The section
  documents this repo's memory as **in-repo and public** — records resolve to
  `<repo-root>/.eidetic/memory` (committed, team- and mesh-shared). Inserted
  idempotently (skipped if already present), slotted under an existing
  "Conventions and workflow" heading when one exists, else appended.

### Changed

- **Refreshed the `remember` + `recall` wrappers from eidetic-cli 0.10.0**
  (cite-don't-import) — picks up eidetic's **project-local store default**: the
  files backend now resolves per record by visibility — PUBLIC records inside a
  git repo go to `<repo-root>/.eidetic/memory` (committed, team-shared), PRIVATE
  records (or any record outside a repo) go to `$HOME/.eidetic/memory` (never
  committed), an explicit `EIDETIC_DATA_DIR` still wins, and recall reads both
  stores and merges. Also carries the 0.9.3 hardening (interactive-stdin guard,
  `help` as a search term, SIGPIPE-safe suffix parsing). **Recipe policy
  override (the wrappers here are NOT byte-verbatim):** the injected default
  visibility is flipped from eidetic's `private` to **`public`**, so a plain
  `/remember` lands the note in `./.eidetic/memory` in this repo, kept as part
  of the repo — pass `--visibility private` to route a record to `$HOME`
  instead. `remember` drives `eidetic remember` (idempotent upsert of one JSON
  record or an NDJSON batch on stdin); `recall` drives `eidetic recall` with
  four search modes (exact / approximate / keyword / hybrid). Each `SKILL.md` is
  localized only in the illustrative `--scope <nick>` examples (Provenance keeps
  "First-party to eidetic-cli"). Runtime dep: the `eidetic` CLI on PATH (else a
  local eidetic-cli checkout with `uv`) — **`eidetic >= 0.10.0`** for the
  in-repo routing; on an older CLI the public records still work but are stored
  in `$HOME/.eidetic/memory` instead of in-repo. Propagated by rollout-cli's
  `eidetic-memory` recipe.

## [0.9.0] - 2026-06-27

### Added

- `calibrate` non-TTY **dry-run preview** (joints + three poses + profile path) — opens no bus and writes no profile; supports `--json` (#10).

### Changed

- `calibrate` now routes through the shared `resolve_consent` three-mode core: interactive (TTY) preserves the pose-and-Enter flow but is now EOF-safe (`sys.stdin.readline()` instead of bare `input()`); non-TTY without `--apply` yields the dry-run preview; non-TTY `--apply` is refused with a clean `CliError(EXIT_USER_ERROR)` because full-arm pose capture cannot be automated headlessly (#10).

### Fixed

- `calibrate` no longer leaks a bare `EOFError`/traceback on non-TTY stdin — EOF mid-capture now raises the structured `CliError(EXIT_ENV_ERROR)` contract with no profile written (#10).

## [0.8.0] - 2026-06-27

### Added

- setup-motors agent mode: non-TTY `--apply` drives the headless 6→1 EEPROM walk, emitting connect-<joint> guidance before each write (1-step tier, no plan-hash); the physical motor swap stays the operator job (human / USB hub / future capability).
- setup-motors dry-run: a non-TTY invocation without `--apply` now prints the full 6→1 assignment table (joint/from_id/new_id/baudrate) in text and `--json` with zero EEPROM writes, instead of being hard-refused.

### Changed

- setup-motors now routes through `_consent.py:resolve_consent` (three modes: interactive / dry_run / agent), completing the consent migration of all gated hardware verbs; every EEPROM write emits a pending→success/failed audit pair carrying consent_mode + operator. Docs (explain catalog, overview, learn) updated in lockstep.

## [0.7.0] - 2026-06-27

### Added

- Three-mode consent for the gated hardware verbs (set-motor-id, center-motor): a new shared arm101/cli/_consent.py core auto-detects the operator from (TTY?, --apply?, --plan-hash?) and resolves one of human-interactive (type `yes` at a TTY), agent-interactive (a non-TTY agent consents with --apply), or non-interactive dry-run (prints a read-only write-plan; zero side effects). An AI agent can now drive an EEPROM write / commanded motion without faking a TTY, while a human still gets the typed-confirmation gate.
- Tiered consent matched to blast radius: set-motor-id (reversible EEPROM write) is 1-step (`set-motor-id <id> --apply`); center-motor (commanded motion) is a 2-step plan-file handshake — a dry-run writes a JSON plan under ~/.arm101/plans/ whose plan_hash the agent reads and passes back as `--apply --plan-hash <hash>`. The hash is recomputed from live motor state at apply time and refuses on mismatch (stale-state protection); it is written only to the plan file, never to stdout.
- Attribution + audit for headless writes: an operator identity (ARM101_OPERATOR env -> culture.yaml nick -> tty:$USER) is recorded, and every gated write appends a JSONL `pending` record before and a `success`/`failed` record after to ~/.arm101/audit.log (ARM101_AUDIT_LOG). Audit writes never raise.
- MotorBus.read_lock() (STS3215 EEPROM Lock register, addr 55) + FakeBus lock_register; the Lock state is surfaced in the center-motor plan snapshot (full unlock->write->relock deferred to a follow-on).

### Changed

- set-motor-id / center-motor no longer hard-refuse a non-TTY stdin (reverses the 0.6.0 up-front non-TTY rejection). A non-TTY caller now gets a read-only dry-run plan by default; the destructive write fires only with an explicit --apply (plus --plan-hash for motion). A piped `yes` still cannot drive a write — consent is an explicit flag against a named target, not stdin content.
- Default output (without --json) is markdown — the agent-readable format; --json is for application consumers. The explain catalog, overview verb list, and learn prompt were updated in lockstep to document the three modes, the tiers, and --apply/--plan-hash.

### Fixed

- Test isolation: an autouse `tests/conftest.py` fixture pins `ARM101_AUDIT_LOG` and `ARM101_PLAN_DIR` into each test's tmp dir, so the suite can no longer append test records to the operator's real `~/.arm101/audit.log` (the audit-write tests previously leaked there when they did not set the env var themselves). Found during the F1 live-test.
- Plan-hash verification now tolerates surrounding whitespace: `verify_plan_hash` strips the supplied `--plan-hash` the same way `resolve_consent` does, so a hash read from the plan file with a trailing newline (or copy-pasted with stray spaces) verifies instead of being falsely refused (Qodo).
- `center-motor` plans now surface the real EEPROM Lock register on hardware: `FeetechBus.read_info` reads addr 55 (`_INFO_REGISTERS`), so `motor_snapshot.lock_register` reflects the actual lock state instead of defaulting to 0 (previously only `FakeBus` injected it) (Qodo).
- `set-motor-id` `explain` docs no longer claim a non-interactive stdin without `--apply` exits 2 — it prints a read-only dry-run plan and exits 0 (Qodo).
- Refactored `cmd_center_motor` and `cmd_set_motor_id` into focused helpers (dry-run/confirm/motion/result/audit) to cut cognitive complexity below the gate, and normalized the `# noqa: BLE001` suppression comments (SonarCloud python:S3776, S3358, S7632).

## [0.6.0] - 2026-06-27

### Added

- calibrate-motor verb: identify the single connected Feetech servo before assembly and catalog it — auto-detects the one motor (skipping busy/non-motor ports so it never grabs an unrelated device such as a Reachy daemon), verifies it is a Feetech STS3215 (model 777), shows its full read-only register snapshot, then records Servo Model / Gear Ratio / Corresponding Joint keyed by a motor label (F1..F6, L1..L6) into an XDG motor catalog. Read-only on the motor (no torque, motion, or EEPROM writes); manual and --auto (walk F1..F6 then L1..L6) modes. Validated live against a physical F1 follower motor.
- set-motor-id verb: assign a new EEPROM id (1-253) to the single connected motor — the SO-101 pre-assembly step of connecting motors one at a time to give each its joint's id. Hard-gated: requires a typed `yes`, and a non-interactive stdin (EOF) refuses the persistent write unconditionally (CliError exit 2). Reuses the existing FeetechBus.write_id_baudrate primitive.
- center-motor verb: drive the single connected motor to a known home position (default encoder tick 2048) for horn mounting, then relax torque (--keep-torque to leave it engaged). Commanded motion, hard-gated the same way (typed `yes`; EOF refuses to move). Adds enable_torque / write_goal_position primitives to the MotorBus interface (FeetechBus real impl at registers 40/42; FakeBus records torque/position writes in order for tests).

### Changed

- Renamed the optional install extra [hardware] → [seeed] (named after the Seeed Studio SO-101 kit; the kit currently ships Feetech servos, verified at runtime by model 777 so a future kit revision with a different servo vendor only updates the extra). Touches pyproject, README, docs, and the SDK install hint.
- Registered set-motor-id/center-motor and updated the explain catalog, overview verb list, and learn prompt in lockstep so the documentation surfaces agree.
- learn now documents the hardware prerequisite — the `[seeed]` SDK extra (`pip install 'arm101-cli[seeed]'`) and that set-motor-id/center-motor/setup-motors are gated destructive ops needing an interactive terminal — in both the text and `--json` (a `hardware` key), so a fresh install is self-sufficient from `learn` alone.

### Fixed

- set-motor-id/center-motor now reject a non-TTY stdin up front (before opening the bus), so a piped `yes` can no longer drive a persistent EEPROM write or commanded motion non-interactively (Qodo: gated write/motion needs an interactive terminal).
- center-motor relaxes torque in a `finally` (unless `--keep-torque`) so a failed goal-position write never leaves the servo holding torque (Qodo: torque could remain enabled after an aborted move).
- write_id_baudrate writes the baud register before the id register, both at the motor's current id — writing id first changed the device address mid-call, so the later baud write hit a now-unreachable id (Qodo).
- set-motor-id/center-motor abort paths honour `--json` (emit a structured `aborted` payload instead of plain text) (Qodo).
- FeetechBus.scan sweeps the full 1–253 id space by default (no broadcastPing in this SDK build) so a motor previously re-id'd above 12 is still detected (Qodo).
- load_catalog rejects a non-object JSON root with a clean CliError instead of crashing on `.items()` (Qodo).
- Corrected the SDK install hint to the real distribution name `arm101-cli[seeed]` (was `arm101[seeed]`) (Qodo).
- Extracted three repeated CliError strings in bus.py to module constants (SonarCloud python:S1192).

## [0.5.0] - 2026-06-27

### Added

- find-port verb: lists candidate serial ports (stdlib /dev enumeration, Linux) non-interactively (text + --json, exit 0); --detect resolves the arm's port by diffing before/after an operator unplug, with a clean CliError(exit 2) when run without a TTY.
- calibrate <id> verb: reads per-joint min/mid/max (raw STS3215 ticks) through a Feetech MotorBus adapter and persists a named JSON profile under $XDG_CONFIG_HOME/arm101/calibrations/<id>.json (round-trips byte-identically; the documented clamp contract a future motion verb will read).
- setup-motors verb: walks gripper=6 down to shoulder_pan=1, prompting to connect each motor alone before writing its EEPROM id/baudrate — never writes without the per-motor Enter; non-TTY invocation exits CliError(2) with zero writes.
- arm101.hardware layer: stdlib serial-port enumeration (ports), a Feetech bus adapter that lazy-imports scservo_sdk with an in-memory FakeBus for tests (bus), and the calibration profile schema + XDG persistence (profiles) — all isolated so `import arm101.cli` keeps zero third-party runtime deps.
- Optional install extras [hardware] (feetech-servo-sdk), [mac]/[win] (pyserial placeholders); runtime dependencies stay [].
- docs/hardware-validation.md: the hardware-gated 'done' run-log procedure for validating the three verbs against a physical SO-101 follower arm.

### Changed

- Registered find-port/calibrate/setup-motors and updated the explain catalog, overview verb list, and learn prompt in lockstep so the documentation surfaces agree.
- markdownlint: relaxed MD026/MD033/MD037 so devague-exported specs/plans (literal <id>/<platform> placeholders, export-style emphasis) lint clean while all other rules stay active.

### Fixed

- setup-motors now addresses each motor at the factory/default id (1, override with --current-id) and reassigns it to its target id, so it works on fresh motors that all ship at the same id (Qodo: it previously addressed each motor at its *target* id, which cannot reach an unconfigured motor).
- calibrate profile ids are validated as a single safe filename component (allowlist; path separators and '..' rejected with CliError) so a crafted id can no longer read or overwrite files outside the calibrations directory (Qodo: path traversal).
- bus.py: reword a trailing comment that SonarCloud flagged as commented-out code (S125).

## [0.4.0] - 2026-06-23

### Added

- **Vendored the `remember` + `recall` memory skills from eidetic-cli**
  (cite-don't-import) — the write/read halves of eidetic's shared
  `~/.eidetic/memory` surface, so this agent (Claude and its colleague backend)
  can persist facts across sessions and recall them later, sharing one store.
  `remember` drives `eidetic remember` (idempotent upsert of one JSON record or
  an NDJSON batch on stdin, dedup by id + content hash); `recall` drives
  `eidetic recall` with four search modes — exact / approximate / keyword /
  hybrid — each hit carrying text, full provenance metadata, a relevance score,
  and a freshness signal. The `.sh` wrappers are byte-verbatim from eidetic-cli
  (their first-party origin); each `SKILL.md` is localized only in the
  illustrative `--scope <nick>` examples (Provenance keeps "First-party to
  eidetic-cli"). Both default to this agent's PRIVATE scope, reading the suffix
  from `culture.yaml`. Runtime dep: the `eidetic` CLI on PATH (else a local
  eidetic-cli checkout with `uv`). Propagated by rollout-cli's `eidetic-memory`
  recipe.

## [0.3.3] - 2026-06-20

### Changed

- CLAUDE.md: replaced the pre-/init seed placeholder with a full runtime guide — documents the actual repo state (mesh-agent scaffold, no arm/gripper code yet), the CLI dispatch/registration/error/output contracts, the explain-catalog lockstep rule, the agent-first rubric, and the CI-gating AgentCulture conventions.

### Fixed

- README: Quickstart now invokes the real console script `arm101` (was `arm101-cli`, which fails to spawn); mesh-identity line now reads `AGENTS.colleague.md` for this agent's `backend: colleague` (was a stale `CLAUDE.md` / `backend: claude`).
- explain catalog: add a root entry for the `arm101` console-script name. The agent-first rubric's `explain_self` check runs `explain <project-script-name>` (`arm101`), which was failing because the catalog only keyed the internal prog name `arm101-cli` — a latent scaffold bug the rubric gate only exercises via the script name. Locked in by a regression test asserting every `[project.scripts]` name resolves.

## [0.3.2] - 2026-06-18

### Added

- ask-colleague skill: `monitor`/`guide`/`stop` pilot verbs plus a `--watch`
  flag to dispatch, watch the live feed of, send mid-flight guidance to, and
  cooperatively stop a running colleague flight (re-vendored from colleague).

### Changed

- README: correct the License section from MIT to Apache 2.0 to match the
  `LICENSE` file.

## [0.3.1] - 2026-06-13

### Changed

- CLAUDE.md: add a convention to reach for the `ask-colleague` skill reflexively
  for explore/review/write/grade — read-only `review`/`explore` are always safe;
  side-effecting `write` needs the user's go-ahead.

## [0.3.0] - 2026-06-13

### Added

- AGENTS.colleague.md resident prompt file (backend colleague <-> AGENTS.colleague.md)

### Changed

- Promote agent identity to a colleague resident: culture.yaml backend
  claude -> colleague with a pinned model. The `doctor` backend-consistency
  map gains `colleague` -> AGENTS.colleague.md.

## [0.2.1] - 2026-06-12

### Changed

- **Re-vendored the `ask-colleague` skill from colleague (now 1.7.0, up from the
  0.39.2 sync)** — the wrapper had drifted multiple releases behind origin. Picks
  up the `clean` verb (reap stale/corrupt `colleague/*` branches + orphaned
  `.colleague/` artifacts a crashed run left behind), the `--json` flag on every
  verb (result JSON on stdout, diagnostics/digest on stderr), the
  `_colleague_via_uv` local-dev resolution that honors `--repo`, and the
  tri-state (0/1/2) exit-code contract. `scripts/ask-colleague.sh` + `prompts/`
  are byte-identical to the origin; `SKILL.md` diverges only in the one
  consumer-identifying Provenance clause (`arm101-cli vendors from
  guildmaster`). `docs/skill-sources.md` sync row updated to
  `2026-06-12 (colleague 1.7.0, direct)`. Refs: colleague#183, #186.

## [0.2.0] - 2026-06-06

### Added

- **`ask-colleague` skill** (`.claude/skills/ask-colleague/`) — the first-party front door to the `colleague` CLI (the renamed `convertible`). On top of `explore` / `review` / `write` it adds a `feedback` verb (grade a finished work item — the ROI loop), and `write` now **previews by default** in a throwaway worktree (no side effects) unless `--apply` / `--pr` is given. Reach for it reflexively — `review` for a diverse second opinion on a committed diff before opening a PR, `explore` for a fresh read of an unfamiliar area.

### Changed

- **Replaced the `outsource` skill with `ask-colleague`.** `outsource` was renamed to `ask-colleague` upstream ([colleague#148](https://github.com/agentculture/colleague/pull/148)). Because guildmaster has not re-broadcast the rename yet (its kit still ships the old `outsource`), `ask-colleague` is vendored **directly from the sibling `colleague` checkout** rather than from guildmaster — a tracked local divergence recorded in `docs/skill-sources.md`, parallel to the `agex` → `devex` one. Vendored verbatim except one consumer-identifying clause in the Provenance paragraph.
- **Ledger + CLAUDE.md + `.gitignore`:** point `docs/skill-sources.md` and the CLAUDE.md Skills section at `colleague` / `ask-colleague`, swap the *optional* runtime prerequisite `convertible` → `colleague` (env prefix `CONVERTIBLE_*` → `COLLEAGUE_*`, with the legacy names kept as a deprecated fallback), and gitignore the `.colleague/` run-artifact dir the skill writes (plus the stale `.agex/`).

## [0.1.4] - 2026-05-31

### Added

- **Vendor the `outsource` skill** (`.claude/skills/outsource/`) from
  guildmaster's canonical copy (origin
  [`agentculture/convertible`](https://github.com/agentculture/convertible),
  re-broadcast via guildmaster — guildmaster
  [#51](https://github.com/agentculture/guildmaster/pull/51)). Every agent
  cloned from this template now inherits the ability to hand a scoped task to a
  *different* engine/mind: `explore` (read-only investigation), `review` (a
  diverse second opinion on the committed diff), and `write` (delegate a small
  implementation). `explore`/`review` run isolated in a throwaway `git worktree`;
  `write` refuses a dirty tree. Fulfils
  [#8](https://github.com/agentculture/arm101-cli/issues/8).
- **Ledger + CLAUDE.md:** record `outsource` in `docs/skill-sources.md`
  (origin = convertible, re-broadcast via guildmaster; vendored verbatim — it
  already carries `type: command`) and document its *optional* runtime
  dependency on the `convertible` CLI (the skill exits with an install hint if
  absent, so a clone that never uses it is unaffected).

### Changed

### Fixed

## [0.1.3] - 2026-05-31

### Changed

- Expanded the clone-and-rename instructions in `CLAUDE.md`: added `README.md` to
  the rename targets and a portable `git grep` discovery command so a cloner can
  find every occurrence of the template name (hard-coded in ~100 places across the
  package, including the CLI command files and `_ISSUES_URL` in
  `arm101/cli/__init__.py`) rather than renaming by hand.
- Synced `README.md`'s "Make it your own" checklist with `CLAUDE.md`: it now lists
  `README.md` itself as a rename target and points to `CLAUDE.md`'s discovery
  command as the authoritative procedure, so the two onboarding checklists no
  longer drift.

## [0.1.2] - 2026-05-30

### Changed

- Renamed the PR-lifecycle CLI references `agex` / `agex-cli` to `devex` (same
  tool, new name) across `CLAUDE.md`, `docs/skill-sources.md`, `.gitignore`, and
  the vendored `cicd`, `assign-to-workforce`, and `communicate` skills — the
  `cicd` scripts now invoke `devex pr`.
- Logged the vendored-skill in-place patch as a local divergence in
  `docs/skill-sources.md`; the matching canonical rename is tracked upstream for
  guildmaster in
  [agentculture/guildmaster#48](https://github.com/agentculture/guildmaster/issues/48)
  so a future re-sync reconciles cleanly.
- Aligned the documented `devex` version floor to `>=0.21` across the vendored
  `cicd` `SKILL.md` and `workflow.sh` install hint (were `>=0.1`), matching
  `docs/skill-sources.md` and the `await`-era feature set; flagged upstream on
  guildmaster#48.

### Fixed

- SonarCloud now reports code coverage — added `relative_files = true` to
  `[tool.coverage.run]` so `coverage.xml` emits repo-relative paths that map to
  `sonar.sources=arm101` (absolute / `.venv` paths were dropped
  as unmappable). Mirrors the sibling `convertible` setup.

## [0.1.1] - 2026-05-26

### Changed

- **CI gates on the SonarCloud quality gate**
  ([issue #3](https://github.com/agentculture/arm101-cli/issues/3)) —
  added `sonar.qualitygate.wait=true` to `sonar-project.properties` so a failing
  gate fails the `test` job when `SONAR_TOKEN` is set. Token-less repos and fork
  PRs remain green (the scan step is guarded by `if: env.SONAR_TOKEN != ''`).

## [0.1.0] - 2026-05-26

### Added

- **Onboarded into the AgentCulture mesh** ([issue #1](https://github.com/agentculture/arm101-cli/issues/1)).
- **Agent-first CLI** cited from teken's (`afi-cli`) `python-cli` reference
  (`teken cli cite`) — verbs `whoami`, `learn`, `explain`, `overview`, `doctor`,
  and the `cli` noun group. Runtime is self-contained (`dependencies = []`);
  `teken>=0.8` is a dev dependency only. Passes the seven-bundle agent-first
  rubric (`teken cli doctor . --strict`). `doctor` checks the agent-identity
  invariants (prompt-file-present, backend-consistency, skills-present).
- **Mesh identity**: `culture.yaml` (`suffix: arm101-cli`,
  `backend: claude`) and the matching `CLAUDE.md` prompt file.
- **Canonical guildmaster skill kit** (11 skills) vendored under
  `.claude/skills/` (cite-don't-import): `agent-config`, `assign-to-workforce`,
  `cicd`, `communicate`, `doc-test-alignment`, `pypi-maintainer`, `run-tests`,
  `sonarclaude`, `spec-to-plan`, `think`, `version-bump`. Every `SKILL.md`
  carries `type: command` (load-bearing for the culture/claude backend);
  `cicd` / `communicate` consumer-identifying prose adapted, all script bodies
  verbatim. Provenance in `docs/skill-sources.md`. Three skills (`think`,
  `spec-to-plan`, `assign-to-workforce`) originate in `devague`, re-broadcast
  via guildmaster.
- **Build + deploy baseline**: `pyproject.toml` (hatchling), `tests/` (pytest,
  xdist, coverage), `.github/workflows/{tests,publish}.yml` (CI rubric/lint gate,
  PyPI Trusted Publishing), `.flake8`, `.markdownlint-cli2.yaml`,
  `sonar-project.properties`, and `.claude/skills.local.yaml.example`.

### Changed

### Fixed
