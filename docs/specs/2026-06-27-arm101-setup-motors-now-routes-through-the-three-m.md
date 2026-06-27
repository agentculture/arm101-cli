# arm101 setup-motors now routes through the three-mode consent core: a non-TTY agent gets a read-only dry-run of the full 6->1 EEPROM assignment plan instead of a hard refusal, while the human-interactive per-motor walk is preserved unchanged — completing the consent migration of all gated hardware verbs.

> arm101 setup-motors now routes through the three-mode consent core: a non-TTY agent gets a read-only dry-run of the full 6->1 EEPROM assignment plan instead of a hard refusal, while the human-interactive per-motor walk is preserved unchanged — completing the consent migration of all gated hardware verbs.

## Audience

- AI agents (no PTY) doing SO-101 pre-assembly motor setup, plus humans at a terminal — the same dual audience the other two gated verbs already serve.

## Before → After

- Before: setup-motors hard-refuses any non-TTY stdin up front (setup_motors.py:85) and blocks each motor on sys.stdin.readline() (:109), so a headless agent cannot drive it at all — inconsistent with set-motor-id and center-motor, which PR #7 already migrated.
- After: setup-motors routes through resolve_consent: non-TTY without --apply yields a read-only dry-run of the 6->1 assignment table (zero EEPROM writes); the TTY per-motor walk is preserved; all three gated hardware verbs are now consistent.

## Why it matters

- PR #7's whole point was removing the non-TTY hard-refusal for gated writes; leaving one of three verbs behind makes the surface inconsistent and blocks agents from even previewing the setup plan.

## Requirements

- Replace the up-front isatty() refusal at setup_motors.py:85 with a resolve_consent(args, verb='setup-motors', require_plan_hash=False) call; the three modes (interactive / dry_run / agent) branch off its result.
  - honesty: The verb calls resolve_consent(args, verb=setup-motors, require_plan_hash=False) like set-motor-id, and the old isatty() block at line 85 is deleted.
- dry_run mode (non-TTY, no --apply) emits the 6->1 assignment plan (joint -> from_id -> new_id -> baudrate) to stdout in both text and --json, and performs ZERO write_id_baudrate calls — asserted with FakeBus.
  - honesty: The dry-run path renders every joint in _MOTOR_ORDER with from_id/new_id/baudrate in both text and --json, and a FakeBus asserts zero writes.
- interactive mode (TTY) preserves today's behaviour exactly: per-motor diagnostic prompt, Enter gates each EEPROM write, and EOF mid-walk raises CliError(EXIT_ENV_ERROR) with no further writes.
  - honesty: The existing interactive tests (per-motor Enter prompt and EOF-mid-walk abort with no further writes) pass unchanged against the migrated verb.
- Each EEPROM write emits the pending->success/failed audit pair via build_audit_record/write_audit, carrying consent_mode + operator (resolve_operator), matching set-motor-id and center-motor.
  - honesty: Each write in both the TTY and --apply paths produces a pending record then a success/failed record carrying consent_mode and operator, asserted by a test.
- Documentation lockstep: the explain catalog entry, overview._VERBS, and learn._TEXT/_as_json_payload all describe setup-motors' three consent modes, kept consistent (the repo's three-place doc rule).
  - honesty: All three doc surfaces (explain catalog, overview _VERBS, learn _TEXT and _as_json_payload) describe setup-motors dry-run/interactive/apply modes, and the catalog-resolution and doc tests pass.
- agent mode (--apply, non-TTY) drives the gated 6->1 walk headless: before each EEPROM write it emits connect-<joint> guidance, then writes the connected motor assigned id+baud (1-step EEPROM tier, no plan-hash) emitting the pending->success/failed audit pair with consent_mode=agent and operator. The physical connect/disconnect stays the operator job (human / USB hub / future agent capability), never the CLI.
  - honesty: A FakeBus test drives setup-motors --apply with non-TTY stdin and observes at least one write_id_baudrate plus its pending->success audit pair tagged consent_mode=agent, proving the headless agent path works without a refusal.

## Honesty conditions

- After the change all three gated verbs (set-motor-id, center-motor, setup-motors) call resolve_consent; no verb keeps an independent isatty() gate.
- A non-TTY invocation yields useful output (the plan) instead of a hard refusal, and a TTY invocation still drives the interactive walk; one code path serves both audiences.
- On main today, piping empty stdin into arm101 setup-motors exits EXIT_ENV_ERROR at the isatty() check before any plan is shown.
- A FakeBus test proves non-TTY without --apply makes zero write_id_baudrate calls while printing the full 6->1 table.
- The inconsistency is observable on main: two gated verbs accept non-TTY, one rejects it; closing that gap is the stated point of issue #8.
- CI on the PR branch is green: the zero-write dry-run test, the EOF-mid-walk test, teken doctor, arm101 doctor, and pytest all pass at coverage >= 60 percent.
- setup-motors never actuates hardware beyond write_id_baudrate, carries no plan-hash flag (require_plan_hash=False), and its docs state the physical swap is the operator responsibility.

## Success signals

- A non-TTY 'arm101 setup-motors' prints the 6->1 assignment plan with zero write_id_baudrate calls (asserted via FakeBus); the TTY walk still gates each write on Enter and still raises CliError(EXIT_ENV_ERROR) on EOF mid-walk; teken cli doctor . --strict, arm101 doctor, and pytest stay green at coverage >=60%.

## Scope / boundaries

- The CLI never performs the physical motor connect/disconnect: each swap is done by a human at the bench (or a USB hub, or a future agent USB-swap capability). The agent drives only consent, the EEPROM writes, and the connect-next guidance. No 2-step plan-hash handshake either: setup-motors is the reversible 1-step EEPROM tier; the handshake is reserved for motion verbs like center-motor.

## Decisions

- q1 resolved: setup-motors --apply stays IN SCOPE as an agent-driven per-motor walk (not out-of-scope). The agent drives consent + EEPROM writes + connect-guidance; the physical motor swap is performed by a human / USB hub / future agent USB-swap capability.

## Hard questions

- What does non-TTY 'setup-motors --apply' do? (A) out of scope: reject with guidance pointing agents to 'set-motor-id --apply' for per-motor EEPROM writes — full walk impossible headless; or (B) per-motor: write the single currently-connected motor set-motor-id-style, human swaps between invocations (largely duplicates set-motor-id). (blocking)
