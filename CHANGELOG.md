# Changelog

All notable changes to this project will be documented in this file.

Format follows [Keep a Changelog](https://keepachangelog.com/). This project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.6.0] - 2026-06-27

### Added

- calibrate-motor verb: identify the single connected Feetech servo before assembly and catalog it — auto-detects the one motor (skipping busy/non-motor ports so it never grabs an unrelated device such as a Reachy daemon), verifies it is a Feetech STS3215 (model 777), shows its full read-only register snapshot, then records Servo Model / Gear Ratio / Corresponding Joint keyed by a motor label (F1..F6, L1..L6) into an XDG motor catalog. Read-only on the motor (no torque, motion, or EEPROM writes); manual and --auto (walk F1..F6 then L1..L6) modes. Validated live against a physical F1 follower motor.
- set-motor-id verb: assign a new EEPROM id (1-253) to the single connected motor — the SO-101 pre-assembly step of connecting motors one at a time to give each its joint's id. Hard-gated: requires a typed `yes`, and a non-interactive stdin (EOF) refuses the persistent write unconditionally (CliError exit 2). Reuses the existing FeetechBus.write_id_baudrate primitive.
- center-motor verb: drive the single connected motor to a known home position (default encoder tick 2048) for horn mounting, then relax torque (--keep-torque to leave it engaged). Commanded motion, hard-gated the same way (typed `yes`; EOF refuses to move). Adds enable_torque / write_goal_position primitives to the MotorBus interface (FeetechBus real impl at registers 40/42; FakeBus records torque/position writes in order for tests).

### Changed

- Renamed the optional install extra [hardware] → [seeed] (named after the Seeed Studio SO-101 kit; the kit currently ships Feetech servos, verified at runtime by model 777 so a future kit revision with a different servo vendor only updates the extra). Touches pyproject, README, docs, and the SDK install hint.
- Registered set-motor-id/center-motor and updated the explain catalog, overview verb list, and learn prompt in lockstep so the documentation surfaces agree.

### Fixed

- set-motor-id/center-motor now reject a non-TTY stdin up front (before opening the bus), so a piped `yes` can no longer drive a persistent EEPROM write or commanded motion non-interactively (Qodo: gated write/motion needs an interactive terminal).
- center-motor relaxes torque in a `finally` (unless `--keep-torque`) so a failed goal-position write never leaves the servo holding torque (Qodo: torque could remain enabled after an aborted move).
- write_id_baudrate writes the baud register before the id register, both at the motor's current id — writing id first changed the device address mid-call, so the later baud write hit a now-unreachable id (Qodo).
- set-motor-id/center-motor abort paths honour `--json` (emit a structured `aborted` payload instead of plain text) (Qodo).
- FeetechBus.scan sweeps the full 1–253 id space by default (no broadcastPing in this SDK build) so a motor previously re-id'd above 12 is still detected (Qodo).
- load_catalog rejects a non-object JSON root with a clean CliError instead of crashing on `.items()` (Qodo).
- Corrected the SDK install hint to the real distribution name `arm101-cli[seeed]` (was `arm101[seeed]`) (Qodo).

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
