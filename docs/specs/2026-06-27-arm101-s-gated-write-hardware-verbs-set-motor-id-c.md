# arm101's gated-write hardware verbs (set-motor-id, center-motor) now resolve consent in three driver modes — human-interactive (type yes at a TTY), agent-interactive (an AI reads a structured write-plan and consents explicitly), and non-interactive (scripted) — so an AI agent can safely drive an EEPROM write or a commanded motion without faking a TTY, while the unconsented non-TTY default still refuses.

> arm101's gated-write hardware verbs (set-motor-id, center-motor) now resolve consent in three driver modes — human-interactive (type yes at a TTY), agent-interactive (an AI reads a structured write-plan and consents explicitly), and non-interactive (scripted) — so an AI agent can safely drive an EEPROM write or a commanded motion without faking a TTY, while the unconsented non-TTY default still refuses.

## Audience

- Three operator types of the SO-101 pre-assembly flow: a human at a bench terminal, an AI agent (Claude Code) driving the arm101 CLI, and scripted/CI callers with no one present.

## Before → After

- Before: set-motor-id/center-motor call _require_tty() and HARD-refuse any non-TTY stdin (exit 2). The only consent channel is a human typing yes at a TTY, so the agent operator can never drive the write — forcing PTY-faking hacks that defeat the very gate they bypass.
- After: Every gated side-effect verb resolves consent through ONE shared mode-aware helper; an agent can drive set-motor-id/center-motor to completion with explicit consent and machine-readable I/O; a human still types yes at a TTY; a bare non-TTY run with no consent flag still refuses (exit 2).

## Why it matters

- Here the operator IS an agent. A guard that blocks all non-TTY input also blocks the legitimate operator. A mode-aware consent model makes the agent a first-class, attributable driver without weakening the human safety gate.

## Requirements

- set-motor-id and center-motor both route their gate through ONE shared consent helper (e.g. arm101/cli/_consent.py: resolve_consent(args, plan) -> bool|raise). No verb-specific consent logic; the verbs differ only in the plan object they pass in.
  - honesty: The shared helper carries no per-verb branching: set-motor-id and center-motor pass different plan objects but call the identical resolve_consent().
- The plan file carries a plan-HASH; --apply must reference a matching hash, enforcing 'read the plan before applying' and rejecting a stale/mismatched plan. (mechanism for the 2-step handshake)
  - honesty: An --apply with a mismatched or stale plan-hash is refused (exit 2) and performs NO write — pinned by a test that mutates motor state between plan and apply.
- Headless callers get a ZERO-side-effect plan FIRST: set-motor-id prints a markdown plan to stdout; center-motor writes a JSON plan FILE (~/.arm101/plans/...) with the plan-hash INSIDE the file only (never stdout) plus a markdown pointer. The plan/dry-run path performs no bus write and no motion.
  - honesty: A FakeBus test asserts the plan/dry-run path calls zero bus writes and zero motion (write_id_baudrate / enable_torque / write_goal_position never invoked).

## Honesty conditions

- Each of the three modes is pinned by a test: TTY->interactive prompt; non-TTY+consent->execute (1-step --apply for set-motor-id, 2-step --apply --plan-hash for center-motor); non-TTY+no-consent->refuse (exit 2).
- All three operator types exercise the SAME verbs through one resolver — a test demonstrates a human-TTY prompt path, an agent --apply path, and a non-TTY no-consent refusal.
- After the change neither verb has its own duplicated gate — both import resolve_consent from arm101/cli/_consent.py (asserted by an import/grep test); the human path is behaviorally unchanged.
- The CURRENT code is accurately described: set_motor_id._require_tty and center_motor._require_tty both raise exit 2 on non-TTY today (the pre-change tests we are replacing assert exactly this).
- The agent operator genuinely has no TTY — proven live: invoking 'arm101 set-motor-id 1 < /dev/null' today returns exit 2 at the TTY guard.
- No ANSI/curses/redraw/per-keystroke code is added — the diff touches only consent resolution, plan files, audit, and the two verbs (verifiable by reviewing changed files).
- On F1 an agent completes 'set-motor-id <id> --apply' with no PTY hack; the existing human-TTY tests still pass; a bare non-TTY no-flag run still exits 2; grep shows BOTH verbs call resolve_consent.
- A bare 'set-motor-id 1' with no TTY and no consent flag still exits 2 (default-refuse preserved) — pinned by a test.

## Success signals

- An agent runs set-motor-id end-to-end on F1 with explicit consent and no PTY hack; a human terminal run is unchanged; a bare non-TTY no-flag run still exits refusing; set-motor-id AND center-motor both route through the one shared consent helper.

## Scope / boundaries

- NOT building colleague's full ANSI cockpit / redraw TUI / per-keystroke reader. Scope is the consent + mode-resolution contract for gated hardware verbs plus a dry-run/plan view — not a general UI framework. Read-only verbs (calibrate-motor, detection) are unchanged.

## Decisions

- Consent resolves via ONE shared strategy mirroring colleague's _resolve_decide: (a) explicit consent flag present -> consent granted WITHOUT a TTY; (b) else stdin is a TTY -> interactive prompt the human types yes; (c) else refuse with a hint. The bare no-flag + no-TTY path still refuses (exit 2) — we ADD the explicit-flag channel, we don't remove the default-refuse.
- Headless consent requires the SPECIFIC target named on the command line (e.g. 'set-motor-id 6 --apply' consents to id 6); a bare consent flag with no target is refused. [resolves v2]
- Commanded MOTION is gated HARDER than an EEPROM write: motion refuses the pure-CI tier — it needs a human at a TTY OR a present agent that verifies via the 2-step plan-file read; an EEPROM write is allowed headless with consent. [resolves v1]
- The consent/execute flag is --apply (NOT --yes). Default output is MARKDOWN (the agent-readable format); --json is for APPLICATION consumers, not agents. 'Agents verify' via --apply.
- Risky actions use a 2-STEP FILE HANDSHAKE, not a single flag: (1) the command writes a plan FILE describing exactly what will happen; (2) the agent must READ that plan file (direct file read) and only then run --apply. The handshake forces the agent to consume the plan before executing.
- Headless writes require an attributed operator identity (ARM101_OPERATOR env → culture.yaml nick → tty:$USER); a non-TTY write is REFUSED without an identity; identity recorded in the result + an append-only audit log. [resolves v3]
- Interaction mode is AUTO-DETECTED from (TTY?, output format, consent state); no explicit --mode flag. [resolves v4]
- TIERED consent weight: set-motor-id (reversible EEPROM) uses 1-step agent consent — 'set-motor-id <id> --apply' (specific target + flag, attributed + audited, no plan file). center-motor (physical motion / damage risk) requires the full 2-step plan-file handshake (dry-run writes JSON plan file → agent must Read it → '--apply --plan-hash <hash>'). Honors 'motion stricter than EEPROM'.

## Hard questions

- risk: Stale-state race: motor state can change between plan generation and --apply. Mitigation: plan embeds a state snapshot + timestamp + hash; --apply re-checks and refuses if stale/mismatched.
- risk: Identity env vars (ARM101_OPERATOR) are trivially spoofable — treat attribution as informational audit, never as authorization.

## Open / follow-up

- Full EEPROM Lock-register unlock/relock inside write_id_baudrate (STS3215 addr 55) is DEFERRED to a follow-on PR. This spec only SURFACES lock_register in the plan snapshot and warns when Lock=1 (a locked motor silently discards the write while the SDK returns result=0,error=0).
