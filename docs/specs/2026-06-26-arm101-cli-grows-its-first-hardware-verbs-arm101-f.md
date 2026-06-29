# arm101-cli grows its first hardware verbs: 'arm101 find-port' detects which USB serial port the SO-101 arm is on, and 'arm101 calibrate' records each joint's range of motion — both following the existing agent-first error/output/explain contracts.

> arm101-cli grows its first hardware verbs: 'arm101 find-port' detects which USB serial port the SO-101 arm is on, and 'arm101 calibrate' records each joint's range of motion — both following the existing agent-first error/output/explain contracts.

## Audience

- An operator (human or mesh agent) bringing up a physical SO-101 arm for the first time, who needs to know which /dev port the arm is on and to calibrate joint ranges before any motion.

## Before → After

- Before: arm101-cli has zero hardware verbs — it can introspect its own identity but cannot see, address, or calibrate the physical arm. Operators copy lerobot-find-port / lerobot-calibrate workflows by hand.
- After: find-port returns the arm's serial port; calibrate <id> walks the operator through recording min/mid/max for each of the 6 joints (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper) and persists a named profile that later motion commands consume; setup-motors assigns each motor's id/baudrate in EEPROM.

## Why it matters

- Calibration is the precondition for safe motion: without per-joint min/max, position commands can drive a joint into a hard stop. find-port removes the brittle hand-copied device path, and setup-motors makes a fresh-from-factory arm addressable at all. Together these three verbs are the on-ramp from 'introspection chassis' to 'controls a real arm'.

## Requirements

- The hardware-touching layer (serial enumeration + Feetech motor I/O) is isolated behind an adapter module so the introspection CLI still imports with zero third-party deps; hardware libs install via an optional extra (e.g. pip install '.[hardware]').
  - honesty: After the feature lands, 'python -c "import arm101.cli"' still succeeds in an env with no third-party packages installed, and the hardware adapter import is lazy (only triggered when a hardware verb actually talks to the bus).
- setup-motors walks the operator through one-motor-at-a-time EEPROM id/baudrate assignment (gripper=6 down to shoulder_pan=1), prompting to connect each motor alone before pressing Enter — mirroring lerobot-setup-motors. It is interactive and degrades to a clear CliError when run without a TTY.
  - honesty: setup-motors never proceeds to write EEPROM without an explicit operator Enter per motor, and a non-TTY invocation exits with CliError(2) rather than hanging or auto-writing.
- All three verbs follow the existing contracts verbatim: register() under _commands/, CliError-only failures, the emit_result/emit_error stdout-stderr split, a --json mode, and lockstep updates to the explain catalog + overview _VERBS + learn _TEXT.
  - honesty: A test asserts find-port, calibrate, and setup-motors each raise CliError (never sys.exit), keep the stdout/stderr split in both text and --json modes, and that the explain catalog, overview _VERBS, and learn _TEXT all reference all three verbs (the lockstep-or-drift invariant).

## Honesty conditions

- All three verbs (find-port, calibrate, setup-motors) exist as registered verbs that pass the in-package doctor, teken cli doctor --strict, and the CI pytest suite (surface + contracts + profile round-trip + find-port enumeration) with no hardware attached.
- calibrate's and setup-motors' real motor-I/O paths have been exercised against a physical SO-101 follower arm at least once before the feature is considered done.
- The verbs serve both a human at a terminal and a mesh agent: every prompt-driven flow (find-port --detect, setup-motors) has a documented non-interactive and/or --json path, so an agent is never forced through a blocking TTY prompt to obtain a result.
- On current main, 'arm101 --help' lists no find-port/calibrate/setup-motors verbs and no module under arm101/ opens a serial port — verifiable by grep before the change lands.
- find-port's default (enumeration) mode is fully non-interactive and agent-safe (no blocking prompt), and the interactive --detect mode degrades to a clear CliError when run without a TTY rather than hanging.
- The leader-arm variant, teleoperation, training, and motion-execution verbs are entirely absent from this spec's deliverable — no half-built stubs for them ship.
- CI is green on the surface/contract/round-trip/enumeration tests with no hardware attached, AND a manual run-log against a physical follower arm records find-port, calibrate, and setup-motors working end-to-end.
- calibrate's persisted profile round-trips: writing then reading a profile yields identical per-joint min/mid/max, and the schema is documented where motion commands will later read it.
- The persisted profile's per-joint min/max is stored in the documented units/encoding that a future motion verb will read to clamp commands — the schema is the actual safety contract, not merely a record.

## Success signals

- An operator with a connected SO-101 runs: find-port (returns the right /dev path), calibrate <id> (records real STS3215 min/mid/max to a profile), setup-motors (sets per-motor id/baudrate). Surface + contract + profile round-trip + find-port enumeration tests and the rubric gate pass in CI with NO hardware; real motor I/O is validated manually against a physical arm before 'done'.

## Scope / boundaries

- In scope: find-port, calibrate, and setup-motors for ONE follower SO-101 arm, Linux at launch. Out of scope: the leader-arm variant (differently-geared motors), teleoperation, training, and motion-execution verbs.

## Non-goals

- Does not vendor or wrap the full LeRobot stack; arm101-cli stays a standalone CLI, not a LeRobot plugin.

## Decisions

- find-port offers a non-interactive enumeration mode (list candidate serial ports, agent-friendly, default) AND an interactive disconnect-diff mode (--detect) that prompts the operator to unplug the arm and resolves the single changed port — mirroring lerobot-find-port.
- calibrate performs REAL Feetech STS3215 reads via the Feetech SDK, lazy-imported from the optional [hardware] install extra. With no bus/SDK present it raises CliError (exit 2, env error). The hardware adapter is tested in CI against a fake/in-memory bus; real reads are validated manually against a physical arm, and that manual validation is part of 'done'.
- Calibration profiles persist as JSON under an XDG/user config dir ($XDG_CONFIG_HOME or ~/.config/arm101/calibrations/<id>.json). The arm id is a REQUIRED positional argument, mirroring lerobot --robot.id. Profiles are reinstall-safe and not committed to the repo.
- Serial-port enumeration is Linux stdlib-only at launch (glob /dev/ttyACM*, /dev/ttyUSB*, /dev/serial/by-id). Optional [mac] and [win] install extras exist as placeholders; on macOS/Windows the hardware verbs return a clean 'unsupported for now' CliError rather than silently finding nothing.

## Hard questions

- Is setup-motors (writing per-motor IDs/baudrates to EEPROM) in scope, or a deliberate follow-up? It's the third LeRobot verb and a prerequisite to a fresh arm working at all.
- risk: Stdlib /dev globbing is Linux-only; macOS operators (the LeRobot docs show /dev/tty.usbmodem...) would get nothing until pyserial lands. Need to decide whether macOS support is required at launch.
- Where should calibration profiles live and be keyed — a calibrations/ dir next to the package, an XDG/user config dir, or under culture.yaml? And is the arm id a required positional or defaulted?
- Should calibrate fail-clean without hardware (define surface now, real I/O later), or must this spec include real Feetech STS3215 reads tested against a physical arm before it's 'done'?
