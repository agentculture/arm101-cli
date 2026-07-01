# Changelog

All notable changes to this project will be documented in this file.

Format follows [Keep a Changelog](https://keepachangelog.com/). This project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.15.0] - 2026-07-01

### Added

- Bus layer: `OverloadError(CliError)` + `is_overload()` classifier (STS3215 status bit5), `read_torque_limit`/`write_torque_limit` (RAM addr 48) and `clear_overload()` on MotorBus/FeetechBus/FakeBus, plus a FakeBus overload test seam
- `overloaded` field surfaced across `arm read`/`arm flex`/`arm flex --demo` (JSON + text markers) so an overload is a consumable, structured outcome

### Changed

- `gentle_move`/`compliant_move` default goal-speed lowered 400 -> 150; `gentle_move` caps RAM `Torque_Limit` (~50%) during contact moves and restores it in a finally
- `gentle_move`/`compliant_move`/`demo_sweep` now treat a mid-move STS3215 overload (error=32) as a reported contact/fault: release torque to clear the latch and return `overloaded=True` instead of propagating a raw read error

### Fixed

- `arm flex`/`arm flex --demo` no longer crash with a raw `error=32` env error on a dynamic overload; the joint is auto-recovered (torque released) and the outcome is reported

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
