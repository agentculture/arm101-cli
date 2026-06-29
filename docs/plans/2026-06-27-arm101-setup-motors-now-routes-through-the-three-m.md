# Build Plan — arm101 setup-motors now routes through the three-mode consent core: a non-TTY agent gets a read-only dry-run of the full 6->1 EEPROM assignment plan instead of a hard refusal, while the human-interactive per-motor walk is preserved unchanged — completing the consent migration of all gated hardware verbs.

slug: `arm101-setup-motors-now-routes-through-the-three-m` · status: `exported` · from frame: `arm101-setup-motors-now-routes-through-the-three-m`

> arm101 setup-motors now routes through the three-mode consent core: a non-TTY agent gets a read-only dry-run of the full 6->1 EEPROM assignment plan instead of a hard refusal, while the human-interactive per-motor walk is preserved unchanged — completing the consent migration of all gated hardware verbs.

## Tasks

### t1 — Migrate setup-motors onto resolve_consent and implement dry-run mode

- covers: c2, c3, c4, c5, c8, c9, h1, h2, h3, h4, h5, h7, h8
- acceptance:
  - The isatty() refusal block at setup_motors.py:85-93 is deleted; the handler calls resolve_consent(args, verb=setup-motors, require_plan_hash=False) and branches on interactive/dry_run/agent.
  - A --apply flag (store_true, default False) is registered on the subparser, mirroring set-motor-id; no --plan-hash flag is added.
  - dry_run mode (non-TTY, no --apply) prints the full 6->1 assignment table (every joint in _MOTOR_ORDER as joint/from_id/new_id/baudrate) in both text and --json, opening no bus.
  - A FakeBus test asserts ZERO write_id_baudrate calls in dry-run and that all six joints appear in both text and --json output.

### t2 — Preserve the interactive TTY per-motor walk under the interactive consent mode

- depends on: t1
- covers: c10, h9
- acceptance:
  - In interactive mode the per-motor diagnostic prompt and Enter gate are preserved; each EEPROM write still follows its prompt.
  - EOF mid-walk still raises CliError(EXIT_ENV_ERROR) with no further writes.
  - The existing interactive setup-motors tests pass unchanged.

### t3 — Audit every EEPROM write (interactive and agent paths) with pending->success/failed

- depends on: t1, t2
- covers: c11, h10
- acceptance:
  - Each write_id_baudrate in both the interactive and agent paths is wrapped with build_audit_record/write_audit: a pending record before, a success record after, or a failed record (with error) on exception before re-raising.
  - Every audit record carries consent_mode (the resolve_consent result) and operator (resolve_operator()).
  - A test asserts the pending->success pair is written for a successful walk, with consent_mode and operator present.

### t4 — Implement agent --apply headless 6->1 walk with connect-guidance and EEPROM tier boundary

- depends on: t1, t3
- covers: c13, c14, h12, h13
- acceptance:
  - In agent mode (non-TTY + --apply) the verb drives the 6->1 walk without blocking on stdin: before each write it emits connect-<joint> guidance to stderr and writes the connected motor (addressing --current-id) to its target id.
  - Each agent-mode write emits the pending->success/failed audit pair tagged consent_mode=agent and operator; require_plan_hash stays False (no --plan-hash; 1-step EEPROM tier).
  - A FakeBus test drives setup-motors --apply with non-TTY stdin and observes at least one write_id_baudrate plus its pending->success audit with consent_mode=agent, with no hard refusal.
  - The verb docstring states the physical motor connect/disconnect is the operator responsibility (human / USB hub / future capability), never the CLI.

### t5 — Update the three doc surfaces in lockstep for setup-motors three consent modes

- depends on: t1, t4
- covers: c1, c12, h11
- acceptance:
  - The explain catalog entry for (setup-motors,) describes all three consent modes (dry-run / interactive / agent --apply).
  - overview._VERBS and learn._TEXT plus _as_json_payload mention setup-motors three modes consistently.
  - The catalog-resolution test (test_every_catalog_path_resolves) and the learn/overview doc tests pass.

### t6 — All quality gates green (tests, coverage, doctors, lint)

- depends on: t2, t3, t4, t5
- covers: c6, h6
- acceptance:
  - pytest -n auto passes with coverage >= 60 percent.
  - teken cli doctor . --strict passes and arm101 doctor passes.
  - black, isort, flake8, bandit, and markdownlint-cli2 are clean.

## Risks

- [unknown_nonblocking] non-TTY --apply swap synchronisation on real hardware: the in-process walk emits guidance and writes each motor in 6->1 order but does not block between writes (no TTY). With manual swapping the operator must pre-connect all motors (USB hub) or pace via separate per-motor set-motor-id --apply invocations; an in-process wait/auto-detect of the next connected motor is a possible enhancement. Decide the safe default during implementation. (task t4)
