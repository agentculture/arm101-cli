# Build Plan — arm101-cli grows its first hardware verbs: 'arm101 find-port' detects which USB serial port the SO-101 arm is on, and 'arm101 calibrate' records each joint's range of motion — both following the existing agent-first error/output/explain contracts.

slug: `arm101-cli-grows-its-first-hardware-verbs-arm101-f` · status: `exported` · from frame: `arm101-cli-grows-its-first-hardware-verbs-arm101-f`

> arm101-cli grows its first hardware verbs: 'arm101 find-port' detects which USB serial port the SO-101 arm is on, and 'arm101 calibrate' records each joint's range of motion — both following the existing agent-first error/output/explain contracts.

## Tasks

### t1 — Hardware serial-port enumeration module (arm101/hardware/ports.py)

- covers: c9
- acceptance:
  - enumerate_ports() returns sorted matches of /dev/ttyACM*, /dev/ttyUSB*, /dev/serial/by-id using only the stdlib (glob/os); on darwin/win it raises CliError(exit 2) 'serial enumeration unsupported on <platform> for now'
  - tests/test_ports.py covers: Linux match against a fake /dev tmp tree, empty-result case, and the macOS/Windows unsupported path — and asserts the module imports with zero third-party packages

### t2 — Feetech bus adapter with lazy SDK import + in-memory FakeBus (arm101/hardware/bus.py)

- covers: c9, h1
- acceptance:
  - a MotorBus adapter exposes read_position(motor)/write_id_baudrate(motor,...) ; the real impl lazy-imports the Feetech SDK only on first bus use; a FakeBus implements the same interface for tests
  - tests/test_bus.py asserts: 'import arm101.cli' and 'import arm101.hardware.bus' both succeed with no third-party packages installed, and constructing the real bus without the SDK raises CliError(exit 2)

### t3 — Calibration profile schema + XDG persistence (arm101/hardware/profiles.py)

- covers: h10, h16, c22
- acceptance:
  - a Profile stores per-joint {min,mid,max} for the 6 named joints; save(id)/load(id) round-trip to $XDG_CONFIG_HOME or ~/.config/arm101/calibrations/<id>.json and yield byte-identical per-joint values
  - the units/encoding of min/max are documented in the module docstring as the clamp contract a future motion verb reads; tests/test_profiles.py covers round-trip and XDG_CONFIG_HOME override

### t4 — Packaging optional extras in pyproject.toml ([hardware], [mac], [win])

- covers: c9, h1
- acceptance:
  - pyproject.toml defines [project.optional-dependencies] hardware (Feetech SDK), mac, and win; the runtime 'dependencies' list stays empty ([])
  - a base 'uv sync' (no extras) installs zero third-party RUNTIME deps; the README/CLAUDE note records the '.[hardware]' install for real motor I/O

### t5 — find-port verb (arm101/cli/_commands/find_port.py)

- depends on: t1
- covers: c21, c23, h14, c2
- acceptance:
  - default mode prints enumerated candidate ports (text + --json) fully non-interactively with exit 0; --detect prompts the operator to unplug, diffs ports before/after, and resolves the single changed port; --detect without a TTY raises CliError(exit 2)
  - all failures raise CliError (never sys.exit); results go to stdout and prompts/diagnostics to stderr in both text and --json modes; tests/test_find_port.py covers default enumeration, the diff resolution (mocked), and the no-TTY path

### t6 — calibrate verb (arm101/cli/_commands/calibrate.py)

- depends on: t2, t3
- covers: c21, c23, h14, c16
- acceptance:
  - calibrate <id> requires the id positional (missing -> CliError exit 1); walks min/mid/max capture per joint by reading real positions through the bus adapter, then persists a profile via the profiles module; no bus/SDK present -> CliError(exit 2)
  - tests/test_calibrate.py drives the full capture->persist->reload loop against the FakeBus; asserts CliError-only failures, the stdout/stderr split, and --json output shape

### t7 — setup-motors verb (arm101/cli/_commands/setup_motors.py)

- depends on: t2
- covers: c19, h9, c23, h14
- acceptance:
  - walks gripper=6 down to shoulder_pan=1, prompting 'connect <motor> only, press Enter' before each EEPROM id/baudrate write; NEVER writes EEPROM without the per-motor Enter; non-TTY invocation -> CliError(exit 2)
  - tests/test_setup_motors.py uses scripted stdin against the FakeBus to assert writes happen only after Enter, and that CliError/stdout-stderr/--json contracts hold

### t8 — Register verbs + lockstep docs + doctor (cli/__init__.py, explain/catalog.py, _commands/overview.py, _commands/learn.py)

- depends on: t5, t6, t7
- covers: c1, c23, h6, h11, c3
- acceptance:
  - _build_parser() imports and registers find-port, calibrate, setup-motors; each gets an explain catalog ENTRIES entry, appears in overview._VERBS, and is described in learn._TEXT and _as_json_payload — all updated in the same change
  - arm101 --help lists all three verbs; 'teken cli doctor . --strict' and the in-package 'arm101 doctor' both pass; test_every_catalog_path_resolves stays green

### t9 — Cross-cutting contract, lockstep, import-clean & scope-guard test suite (tests/test_hardware_verbs.py)

- depends on: t8
- covers: h6, h11, h12, h13, h15, c16, h1, c2, c15
- acceptance:
  - tests assert each of the three verbs raises CliError (never sys.exit) and keeps the stdout/stderr split in both text and --json; a lockstep test asserts catalog + overview._VERBS + learn all reference all three verbs; an import-clean test asserts 'import arm101.cli' works with no third-party deps
  - a scope-guard test asserts no leader/teleop/training/motion verbs exist; the full suite runs green with NO hardware attached and coverage stays >= 60 (the CI gate)

### t10 — Manual hardware validation run-log against a physical SO-101 follower (docs/hardware-validation.md)

- depends on: t8
- covers: h7, h13, c16
- acceptance:
  - a documented procedure + run-log captures find-port (returns the right /dev path), calibrate <id> (records real STS3215 min/mid/max), and setup-motors (sets per-motor id/baudrate) each exercised at least once against a physical follower arm
  - the run-log is referenced as the hardware-gated 'done' criterion; CI-only contributors are told the surface is green but this manual step is required before release
