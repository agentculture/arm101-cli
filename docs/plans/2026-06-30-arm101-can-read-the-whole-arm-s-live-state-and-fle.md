# Build Plan — arm101 can read the whole arm's live state and flex it gently: one command streams every joint's position, load, voltage and temperature, and a compliant 'gentle' mode moves the arm while watching motor load so it yields the instant it meets resistance (a touch) instead of pushing through.

slug: `arm101-can-read-the-whole-arm-s-live-state-and-fle` · status: `exported` · from frame: `arm101-can-read-the-whole-arm-s-live-state-and-fle`

> arm101 can read the whole arm's live state and flex it gently: one command streams every joint's position, load, voltage and temperature, and a compliant 'gentle' mode moves the arm while watching motor load so it yields the instant it meets resistance (a touch) instead of pushing through.

## Tasks

### t1 — arm_read: retry-tolerant whole-arm snapshot module

- covers: c9, h1, c2, h7
- acceptance:
  - read_arm(bus, joints) reads all 6 joints via read_info with bounded per-joint retries; one joint's repeated RX timeout marks that joint partial/failed and still returns the other five — no exception aborts the snapshot
  - each joint result carries an explicit health flag (ok|partial|failed) plus position/load/speed/voltage/temperature/torque; lives in new file arm101/hardware/arm_read.py with unit tests against FakeBus

### t2 — motion: bounded, flag-gated per-joint move primitive

- covers: c11, h3
- acceptance:
  - clamp_goal(target,min_angle,max_angle) never returns outside [min,max]; an out-of-bound target is clamped/rejected per policy and reported, never silently driven past a calibrated bound
  - the move helper sets acceleration + goal-speed (compliant) limits and requires an explicit caller flag — no motion path executes without it; new file arm101/hardware/motion.py with unit tests

### t3 — gentle: load-watch back-off-then-hold compliant move

- depends on: t2
- covers: c14, h6
- acceptance:
  - gentle_move steps the goal incrementally; when present_load exceeds threshold (sane default + override) it stops, reverses a bounded N ticks off the contact point, and holds with torque on — not limp, not a hard freeze
  - default threshold and override both honored; back-off magnitude bounded; new file arm101/hardware/gentle.py with tests driving a FakeBus present_load ramp

### t4 — baud_probe: multi-baud id/read classifier (closes #18)

- covers: c12, h4
- acceptance:
  - probe_bus(port) sweeps every baud in BAUD_MAP, scans ids, and classifies each id at each baud as SUCCESS / CORRUPT (collision/garbled) / TIMEOUT (absent)
  - result names the port and, per id, which baud(s) answered; a fully silent bus yields all-TIMEOUT (diagnosed, not 'no servo'); new file arm101/hardware/baud_probe.py with tests

### t5 — hardening: read-back-after-write + refuse >1 motor on bus

- acceptance:
  - set-motor-id and setup-motors read the id back after writing and raise CliError if read-back id != written id
  - set-motor-id refuses (CliError, exit 1, with remediation) when scan finds more than one motor on the bus; edits set_motor_id.py + setup_motors.py only, with tests

### t6 — demo: scripted safe-exploration sweep (layers on motion+gentle)

- depends on: t2, t3
- covers: c13
- acceptance:
  - demo_sweep moves each joint through a safe fraction of its calibrated range via the gentle compliant primitive, never exceeding min/max, and aborts cleanly on contact
  - new file arm101/hardware/demo.py with tests; imports motion+gentle, touches no CLI file

### t7 — doctor-extended: wire baud_probe into the doctor command

- depends on: t4
- covers: c12, h4, c7
- acceptance:
  - doctor exposes a multi-baud probe path that prints, per id, SUCCESS/CORRUPT/TIMEOUT with port+baud (text + --json), routed through the structured error/exit contract
  - existing identity-invariant doctor checks (prompt-file/backend/skills) unchanged and still pass; edits arm101/cli/_commands/doctor.py only, with tests

### t8 — arm noun wiring: read/flex(--to/--demo/--gentle/--threshold) + explain lockstep

- depends on: t1, t2, t3, t6
- covers: c1, c13, c5, c6, c7, c4, h8, h9, h10, h12
- acceptance:
  - arm read prints six joints live state (text + --json) with partial joints marked; arm flex <joint> --to <tick> moves bounded; arm flex --demo runs the sweep; --gentle and --threshold apply on any motion; every failure raises CliError (structured, no traceback)
  - explain catalog + overview _VERBS + learn updated in lockstep (test_every_catalog_path_resolves and overview/learn tests pass); runtime dependencies = [] unchanged (no new third-party import); edits arm.py + explain/catalog.py + overview.py + learn.py

### t9 — hardware validation on a real SO-101 follower (operator-run)

- depends on: t7, t8
- covers: h5, h11
- acceptance:
  - on a powered follower: arm read returns all six joints with live load; arm flex --gentle demonstrably backs off on contact instead of pushing through; doctor diagnoses a silenced/misbauded bus — run-log captured into the PR
  - operator-executed (subagents cannot drive hardware); the live run-log is the evidence the honesty conditions h5/h11 are met

## Risks

- [unknown_nonblocking] present_load contact threshold is per-joint (gripper free-motion load ~140-208 from gear friction; larger joints lower) — needs sane default + --threshold override + a future calibration step (task t3)
- [unknown_nonblocking] back-off magnitude and speed tuning (how many ticks / how fast) to make the retreat feel gentle, not abrupt — tune on hardware (task t3)
- [unknown_nonblocking] honesty conditions h5/h11 require a powered physical follower and an operator; subagents cannot auto-test them — validation is human-gated (t9) (task t9)
- [follow_up] hardening fold-ins (t5) come from the issue's 'Hardening surfaced this session' section, not the converged spec's 22 coverage targets — included as a safety fold-in, droppable at the plan gate (task t5)
