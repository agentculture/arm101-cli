# Build Plan — arm101's gated-write hardware verbs (set-motor-id, center-motor) now resolve consent in three driver modes — human-interactive (type yes at a TTY), agent-interactive (an AI reads a structured write-plan and consents explicitly), and non-interactive (scripted) — so an AI agent can safely drive an EEPROM write or a commanded motion without faking a TTY, while the unconsented non-TTY default still refuses.

slug: `arm101-s-gated-write-hardware-verbs-set-motor-id-c` · status: `exported` · from frame: `arm101-s-gated-write-hardware-verbs-set-motor-id-c`

> arm101's gated-write hardware verbs (set-motor-id, center-motor) now resolve consent in three driver modes — human-interactive (type yes at a TTY), agent-interactive (an AI reads a structured write-plan and consents explicitly), and non-interactive (scripted) — so an AI agent can safely drive an EEPROM write or a commanded motion without faking a TTY, while the unconsented non-TTY default still refuses.

## Tasks

### t1 — bus: add FeetechBus.read_lock() + FakeBus lock_register snapshot field

- acceptance:
  - FeetechBus.read_lock(motor_id) reads STS3215 register 55 (1 byte) and returns 0/1; unit test asserts it reads addr 55 via the fake packet handler.
  - FakeBus.read_info() includes a constructor-settable 'lock_register' key (default 0) so plan snapshots carry it; test asserts the key is present.
  - No behavior change to existing bus methods; existing tests/test_bus.py still pass.

### t2 — consent core: new arm101/cli/_consent.py (resolve_consent + plan build/write/verify + audit + operator identity)

- depends on: t1
- covers: c1, h4, c2, h7, c3, c5, c10, h3, c17, h5, c19, h6
- acceptance:
  - resolve_consent(args,*,verb,require_plan_hash) -> 'interactive' under a TTY; 'dry_run' when non-TTY and not args.apply; 'agent' when non-TTY+args.apply and (require_plan_hash False OR plan_hash is valid sha256:<64hex>); raises CliError(EXIT_ENV_ERROR) when non-TTY+apply and require_plan_hash but hash missing; CliError(EXIT_USER_ERROR) when hash malformed. test_consent.py pins every row for both flag values.
  - build_plan(verb,port,info,action) returns schema_version/verb/created_at/port/operator/action/motor_snapshot(id,model,present_position,torque_enable,lock_register)/plan_hash; plan_hash = sha256 over canonical json of {verb,port,action,motor_snapshot(id,model,present_position,torque_enable)} (excludes operator/created_at/volatile sensors). Test: same state->same hash, changed present_position->different hash.
  - write_plan_file(plan) writes JSON to $ARM101_PLAN_DIR or ~/.arm101/plans/<verb>-<portbase>-<utc>.json and returns the path; emit_plan_stdout() writes MARKDOWN naming the path but NOT the hash. Test asserts hash is in the file and absent from stdout.
  - verify_plan_hash(supplied,verb,port,action,info) recomputes from live state; raises CliError(EXIT_ENV_ERROR) on mismatch, returns None on match. Test: mismatch->exit2, match->ok.
  - resolve_operator() = ARM101_OPERATOR else culture.yaml nick else 'tty:'+getuser(); write_audit(record) appends one JSONL line to $ARM101_AUDIT_LOG or ~/.arm101/audit.log and NEVER raises on write failure. Test asserts pending-before/success-after ordering and that an unwritable log path is swallowed.
  - Module introduces NO ANSI/curses/redraw code; uses _output.emit_* only.

### t3 — set-motor-id: tiered 1-step --apply via resolve_consent(require_plan_hash=False); drop _require_tty/_confirm

- depends on: t2
- covers: c4, h9, c7, h12, h10, h8, c10, h3, c19, h6
- acceptance:
  - set_motor_id imports resolve_consent and no longer defines _require_tty or _confirm (grep test asserts absence); register() adds --apply (store_true) + --plan-hash with help noting --apply is non-TTY-only / ignored under a TTY.
  - Non-TTY + no --apply prints a markdown plan to stdout and performs ZERO eeprom writes (FakeBus.eeprom_writes == []).
  - Non-TTY + 'set-motor-id 6 --apply' (no hash needed, 1-step EEPROM tier) writes EEPROM exactly once after a 'pending' audit record; result names from_id->to_id, port, operator.
  - Non-TTY + --apply with new_id absent raises CliError(EXIT_USER_ERROR) and writes nothing (specific-target rule).
  - TTY path behaviorally unchanged: typed 'yes' writes, 'no'/EOF behave as today; the two old non-TTY-rejection tests are rewritten and result-text assertions updated for markdown.

### t4 — center-motor: 2-step plan-file handshake via resolve_consent(require_plan_hash=True); drop _require_tty/_confirm

- depends on: t2
- covers: c4, h9, h8, c17, h5, c19, h6, c10
- acceptance:
  - center_motor imports resolve_consent and no longer defines _require_tty/_confirm (grep test); register() adds --apply + --plan-hash.
  - Non-TTY + no --apply writes a JSON plan FILE (action carries workspace_warning; plan_hash in file) + markdown pointer to stdout; performs ZERO motion (FakeBus enable_torque/write_goal_position never called).
  - Non-TTY + --apply --plan-hash <matching> runs enable_torque->write_goal_position->relax (relax skipped with --keep-torque) after a 'pending' audit; <mismatched/stale> raises CliError(EXIT_ENV_ERROR) with no motion.
  - Non-TTY + --apply without --plan-hash raises CliError(EXIT_ENV_ERROR), no motion.
  - TTY path unchanged: workspace warning + typed 'yes' moves, 'no'/EOF aborts; existing tests pass after migration.

### t5 — docs lockstep: explain/catalog + overview + learn updated for --apply/--plan-hash and the three modes

- depends on: t3, t4
- covers: c6, h11
- acceptance:
  - explain catalog gains entries for --apply/--plan-hash/dry-run-plan on set-motor-id and center-motor; test_every_catalog_path_resolves passes.
  - overview._VERBS descriptions and learn text mention agent mode / --apply; existing lockstep tests pass.
  - Docs/strings only — no ANSI/TUI code introduced.

## Risks

- [follow_up] Full EEPROM Lock-register unlock/relock inside write_id_baudrate deferred to a follow-on PR; this plan only surfaces lock_register + warns when Lock=1.
- [unknown_nonblocking] Stale-plan TTL: hash-over-live-state catches state drift, but an idle motor with constant state means a hash never expires; a created_at TTL check is a soft (warn-only) mitigation.
- [unknown_nonblocking] Plan file is a TOCTOU surface; mitigated by computing the hash from live MOTOR state at apply time (not from file content), so file tampering can't authorize a different action.
