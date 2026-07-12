"""Markdown catalog for ``arm101-cli explain <path>``.

Each entry is verbatim markdown. Keys are command-path tuples. The empty tuple
resolves to the root entry, as do both names the CLI answers to: the console
script ``("arm101",)`` (from ``[project.scripts]``) and the internal prog name
``("arm101-cli",)``. The script-name key is load-bearing — the agent-first
rubric's ``explain_self`` check runs ``explain <project-script-name>``.

Keep bodies self-contained: an agent reading one entry should get enough
context without chaining reads.
"""

from __future__ import annotations

from arm101.hardware import arm_spec
from arm101.hardware.limits import MATERIAL_SPAN_DELTA_TICKS

_ROOT = """\
# arm101-cli

A clonable template for AgentCulture mesh agents. It carries an agent-first CLI
(cited from the teken `python-cli` reference), a mesh identity (`culture.yaml` +
`CLAUDE.md`), the canonical guildmaster skill kit under `.claude/skills/`, and a
buildable/deployable package baseline. Clone it, rename the package, edit
`culture.yaml`, and you have a new agent.

## Verbs

- `arm101-cli whoami` — identity probe from `culture.yaml`.
- `arm101-cli learn` — structured self-teaching prompt.
- `arm101-cli explain <path>` — markdown docs for any noun/verb.
- `arm101-cli overview` — descriptive snapshot of the agent.
- `arm101-cli doctor` — check the agent-identity invariants.
- `arm101-cli find-port` — list candidate serial ports (or `--detect` to resolve by unplug).
- `arm101-cli calibrate <id>` — capture min/mid/max (interactive; non-TTY = dry-run preview).
- `arm101-cli calibrate-motor` — identify a connected motor; catalog its model/gear/joint.
- `arm101-cli setup-motors` — assign per-motor EEPROM id/baudrate (interactive).
- `arm101-cli arm setup <role>` — gated number-free setup; assigns ids 1–6, catalogs F/L motors.
- `arm101-cli arm overview` — describe the arm noun surface (roles, joints, motor map).
- `arm101-cli arm read` — read every joint's live register state (read-only; no motion).
- `arm101-cli arm flex` — gated joint move (`--to`) or demo sweep (`--demo`); `--gentle`.
- `arm101-cli cli overview` — describe the CLI surface.

## Exit-code policy

- `0` success
- `1` user-input error
- `2` environment / setup error
- `3+` reserved

## See also

- `arm101-cli explain whoami`
- `arm101-cli explain doctor`
"""

_WHOAMI = """\
# arm101-cli whoami

Reports the agent's identity from `culture.yaml`: nick (`suffix`), backend,
served model, and the package version. Read-only.

## Usage

    arm101-cli whoami
    arm101-cli whoami --json
"""

_LEARN = """\
# arm101-cli learn

Prints a structured self-teaching prompt covering purpose, command map,
exit-code policy, `--json` support, and the `explain` pointer.

## Usage

    arm101-cli learn
    arm101-cli learn --json
"""

_EXPLAIN = """\
# arm101-cli explain <path>

Prints markdown documentation for any noun/verb path. Unlike `--help` (terse,
positional), `explain` is global and addressable by path.

## Usage

    arm101-cli explain arm101-cli
    arm101-cli explain whoami
    arm101-cli explain --json <path>
"""

_OVERVIEW = """\
# arm101-cli overview

Read-only descriptive snapshot of the agent: identity (from `culture.yaml`), the
verb surface, and the sibling-pattern artifacts the template carries. Accepts an
ignored `target` so a stray path never hard-fails.

## Usage

    arm101-cli overview
    arm101-cli overview --json
"""

_DOCTOR = """\
# arm101-cli doctor

Checks the agent-identity invariants `steward doctor` verifies:
prompt-file-present and backend-consistency (`claude` → `CLAUDE.md`), plus a
skills-present check. Exits 1 when unhealthy.

## Usage

    arm101-cli doctor
    arm101-cli doctor --json
"""

_FIND_PORT = """\
# arm101-cli find-port

Resolve the serial port the SO-ARM101 is attached to. The default mode is
agent-safe: it lists every candidate serial port non-interactively and exits 0
even when none are found. The `--detect` mode is an interactive
disconnect-diff (mirrors `lerobot-find-port`): it snapshots the ports, prompts
you to unplug the arm, then reports the single port that disappeared.

## Usage

    arm101-cli find-port
    arm101-cli find-port --json
    arm101-cli find-port --detect

## Hardware / TTY behavior

`--detect` requires an interactive terminal (a TTY on stdin); without one it
fails with a hardware/setup error (exit 2). The default listing mode needs no
hardware and never hard-fails — use it from an agent.
"""

_CALIBRATE = """\
# arm101-cli calibrate <id>

Interactively capture per-joint min/mid/max encoder positions and persist them
as a named calibration profile (stored under the XDG config dir). Walks you
through three poses (centered/rest, minimum, maximum), reads every joint from
the motor bus after each, and saves a `Profile` keyed by the required `id`
positional (mirrors lerobot's `--robot.id`).

## Usage

    arm101-cli calibrate my-arm
    arm101-cli calibrate my-arm --port /dev/ttyACM0
    arm101-cli calibrate my-arm --json
    arm101-cli calibrate my-arm          # non-TTY: prints a read-only dry-run preview

## Consent / TTY modes

Three modes are supported based on the terminal environment:

1. **Interactive (TTY)** — the default when stdin is a terminal. Walks through
   three poses (centered/rest → minimum → maximum), reads all 6 joints after each
   via the motor bus, then saves the profile to disk. Prompts go to stderr; the
   saved summary goes to stdout.
2. **Non-TTY without `--apply`** — read-only dry-run preview. Describes the id,
   the 6 joints that would be captured, the three poses, and the profile path.
   No bus is opened; no profile is written. Safe to run from an agent or a pipe.
3. **Non-TTY with `--apply`** — NOT SUPPORTED. Full-arm pose calibration requires
   physical arm poses that cannot be captured headlessly. Exits 1 with a clear
   error and remediation hint (run interactively or use the dry-run preview without
   `--apply`).

## Exit codes

- `0` success (interactive capture + save, or dry-run preview)
- `1` user/usage error (bad id format, or `--apply` in non-TTY mode)
- `2` hardware/setup error (SDK absent, port unavailable, or stdin closed mid-capture)

## Hardware / TTY behavior

Requires a real motor bus and the Feetech SDK (`[seeed]` extra) in interactive
mode. The dry-run preview requires no hardware and never opens a bus.
When the SDK is absent or the serial port cannot be opened, it fails with a
hardware/setup error (exit 2).
"""

_CALIBRATE_MOTOR = """\
# arm101-cli calibrate-motor

Identify a single connected Feetech servo before assembly and record its spec
into the motor catalog. Auto-detects the one motor (skipping busy or non-motor
serial ports, so it never grabs an unrelated device), shows its full read-only
register snapshot, then captures three operator-supplied fields — Servo Model,
Gear Ratio, and Corresponding Joint — keyed by a motor label (`F1`..`F6`
follower, `L1`..`L6` leader). Read-only on the motor: it pings and reads
registers but never enables torque, moves, or writes EEPROM.

## Usage

    arm101-cli calibrate-motor F1
    arm101-cli calibrate-motor --port /dev/ttyACM1
    arm101-cli calibrate-motor --auto
    arm101-cli calibrate-motor --json

## Hardware / TTY behavior

Requires a real motor bus and the Feetech SDK (the `[seeed]` extra). It verifies
each connected motor really is a Feetech STS3215 (model 777) before cataloging.
Manual
mode registers the one connected motor; `--auto` walks F1..F6 then L1..L6,
prompting to connect each. Inherently interactive — prompts and the motor
snapshot go to stderr, the saved record to stdout; with no input available it
fails with a hardware/setup error (exit 2).
"""

_SET_MOTOR_ID = """\
# arm101-cli set-motor-id

Assign a new EEPROM id to the single connected Feetech STS3215 — the SO-101
pre-assembly step of connecting motors one at a time and giving each its joint's
id. Auto-detects the one motor at its present id (skipping busy or non-motor
ports), shows its full read-only register snapshot, then writes the new id only
after an explicit typed `yes`.

## Consent modes

Three modes are supported:

1. **TTY (interactive)** — prompts the human to type `yes` to confirm the write.
2. **Non-TTY without `--apply`** — prints a markdown dry-run plan (zero writes).
3. **Non-TTY with `--apply`** — executes the write (1-step tier). The target id is
   required; a bare `--apply` with no id is refused.

Headless writes are attributed (`ARM101_OPERATOR` env / culture nick) and
appended to `~/.arm101/audit.log`.

## Usage

    arm101-cli set-motor-id 1
    arm101-cli set-motor-id 6 --apply
    arm101-cli set-motor-id --port /dev/ttyACM1
    arm101-cli set-motor-id --json

## Hardware / TTY behavior

Requires a real motor bus and the Feetech SDK (the `[seeed]` extra). Exit codes:
0 success, clean abort, or a non-TTY dry-run plan; 1 for a bad id (outside the
1-253 range or non-integer) or a missing id in non-interactive mode; 2 for a
hardware/setup error. `--json` emits `{"port", "from_id", "to_id", "baudrate"}`;
prompts and the snapshot go to stderr, the result to stdout.
"""

_CENTER_MOTOR = """\
# arm101-cli center-motor

Drive the single connected Feetech STS3215 to a known home position (default
encoder tick 2048, mid-range) so a horn can be mounted against a repeatable
zero, then relax torque. Auto-detects the one motor (skipping busy or non-motor
ports), shows its full read-only register snapshot, then — only after an
explicit typed `yes` — enables torque, moves to the target, and relaxes.

## Consent modes

Three modes are supported:

1. **TTY (interactive)** — prompts the human to type `yes` to confirm the motion.
2. **Non-TTY without `--apply`** — writes a JSON plan file under
   `~/.arm101/plans/` (zero motion).
3. **Non-TTY with `--apply`** — executes the motion (2-step tier). Read the plan
   file to obtain its `plan_hash`, then run
   `center-motor --position <p> --apply --plan-hash <hash>`. The hash is
   re-checked against live motor state and refused if it changed.

Headless writes are attributed (`ARM101_OPERATOR` env / culture nick) and
appended to `~/.arm101/audit.log`.

## Usage

    arm101-cli center-motor
    arm101-cli center-motor --position 2048 --apply --plan-hash sha256:...
    arm101-cli center-motor --keep-torque
    arm101-cli center-motor --json

## Hardware / TTY behavior

Requires a real motor bus and the Feetech SDK (the `[seeed]` extra). Exit codes:
0 success or clean abort, 1 for an out-of-range `--position`, 2 for a
hardware/setup error or non-interactive stdin without `--apply`. `--json` emits
`{"motor", "port", "position", "torque_relaxed"}`; prompts and the snapshot go to
stderr, the result to stdout.
"""

_SET_BAUDRATE = """\
# arm101-cli set-baudrate

Change the EEPROM baud rate of the single connected Feetech STS3215 without
altering its servo ID.  Auto-detects the one motor (skipping busy or non-motor
ports), shows its full read-only register snapshot, then writes the new baud
rate only after an explicit typed `yes`.  The change takes effect on the
motor's next power-up; the after-card opens a fresh bus at the new baud to
confirm the register was written.

## Consent modes

Three modes are supported:

1. **TTY (interactive)** — prompts the human to type `yes` to confirm the write.
2. **Non-TTY without `--apply`** — prints a markdown dry-run plan (zero writes).
3. **Non-TTY with `--apply`** — executes the write (1-step tier). The target baud
   rate is required; a bare `--apply` with no baud is refused.

Headless writes are attributed (`ARM101_OPERATOR` env / culture nick) and
appended to `~/.arm101/audit.log`.

## Usage

    arm101-cli set-baudrate 500000
    arm101-cli set-baudrate 500000 --apply
    arm101-cli set-baudrate --port /dev/ttyACM1
    arm101-cli set-baudrate --json

## Hardware / TTY behavior

Requires a real motor bus and the Feetech SDK (the `[seeed]` extra). Exit codes:
0 success, clean abort, or a non-TTY dry-run plan; 1 for an unsupported baud rate
or a missing baud in non-interactive mode; 2 for a hardware/setup error.
`--json` emits `{"port", "motor", "baudrate"}`; prompts and the snapshot go to
stderr, the result to stdout.

## Supported baud rates

38400, 57600, 76800, 115200, 128000, 250000, 500000, 1000000
"""

_SETUP_MOTORS = """\
# arm101-cli setup-motors

Assign each motor's EEPROM id and baudrate one at a time, walking the arm from
gripper (id 6) down to shoulder_pan (id 1). The port is **auto-detected per
motor** (via the same detection machinery as `set-motor-id`), so USB
re-enumeration when the operator unplugs one motor and plugs in the next is
handled transparently. Pass `--port` to override with a fixed path.

For each motor the verb shows a **before card** (read-only register snapshot,
including baudrate in bps) and — after the write — an **after card** confirming
the new id and baudrate. Both cards go to stderr; the final assignment summary
goes to stdout.

## Flags

- `--baudrate` — EEPROM baud rate to programme (default 1 000 000).
  Supported values: 38400, 57600, 76800, 115200, 128000, 250000, 500000,
  1000000. Validated before any bus is opened.
- `--current-id` — safety assertion: auto-detected motor id must equal this
  value or the walk is aborted. Omit to accept any detected id.
- `--port` — fixed serial port; omit for per-motor auto-detection.

## Consent modes

Three modes are supported:

1. **TTY (interactive)** — per-motor prompt; press Enter to confirm each EEPROM
   write.
2. **Non-TTY without `--apply`** — prints a read-only dry-run plan of the full
   6→1 assignment table including the baudrate (zero writes, no bus opened).
3. **Non-TTY with `--apply`** — executes the headless 6→1 walk (1-step tier).
   Before each write emits a "connect the <joint> motor now" guidance line.
   The physical motor connect/disconnect is the operator's responsibility.

Headless writes are attributed (`ARM101_OPERATOR` env / culture nick) and
appended to `~/.arm101/audit.log`.

## Usage

    arm101-cli setup-motors
    arm101-cli setup-motors --apply
    arm101-cli setup-motors --baudrate 500000
    arm101-cli setup-motors --port /dev/ttyACM0
    arm101-cli setup-motors --current-id 1
    arm101-cli setup-motors --json

## Hardware / TTY behavior

Requires a real motor bus and the Feetech SDK (the `[seeed]` extra). Exit codes:
0 success or a non-TTY dry-run plan; 1 for a bad `--baudrate` or `--current-id`
mismatch; 2 for a hardware/setup error. `--json` emits `{"assigned": [...]}`;
prompts, cards, and guidance go to stderr, the result to stdout.
"""

_CLI = """\
# arm101-cli cli

Noun group for CLI-surface introspection. `cli overview` describes the CLI
itself (distinct from the global `overview`, which describes the agent).

## Usage

    arm101-cli cli overview
    arm101-cli cli overview --json
"""

_ARM = """\
# arm101-cli arm

Noun group for arm-level operations on the SO-101 robotic arm. Provides a
read-only surface snapshot (`arm overview`), a read-only live-state read
(`arm read`), a gated motion verb (`arm flex`), a gated reachability-mapping
walk (`arm explore`), a gated speed-profiling ramp (`arm profile <joint>`), a
gated travel measurement (`arm limits [<joint>...]` — it measures and restores,
changing nothing), a gated encoder re-zero (`arm rezero <joint>` — an EEPROM write
that commands no motion), and a gated setup walk (`arm setup <role>`).

## Verbs

- `arm101-cli arm overview` — describe the arm noun surface (roles, joints,
  and the per-role id / baud / servo_model / gear_ratio map). Read-only;
  always exits 0.
- `arm101-cli arm read` — read every joint's live register state
  (position/load/speed/voltage/temperature/torque, plus the signed encoder
  `offset`). Read-only on the bus — no consent gate; a flaky joint is marked
  `partial`/`failed` while the rest still read.
- `arm101-cli arm flex` — command a bounded, gentle joint move (`--to`) or a
  demo sweep (`--demo`). Gated motion: three-mode consent + `--apply`, with
  `--gentle`/`--threshold` selecting the load-watch back-off-then-hold path.
- `arm101-cli arm explore` — flood-fill and map the reachable joint-space via
  the overload-safe gentle move, writing a resumable JSONL event log plus a
  compact, queryable reachability map (`--map` to resume/override). Gated
  motion: three-mode consent + `--apply`.
- `arm101-cli arm profile <joint>` — find the highest speed at which contact
  detection STILL WORKS, by driving the joint into a real contact
  (`--contact-to`) at every candidate speed and requiring the stall rule to fire.
  A speed the servo merely survives is a failure, not a pass. Records the joint's
  safe speed, ticks/second, and motion-onset latency. Gated motion: three-mode
  consent + `--apply`.
- `arm101-cli arm limits [<joint>...]` — **measure** each joint's true travel and
  change nothing: roll the encoder seam out of the way, creep to BOTH ends under
  contact detection, and rule on what stopped it (WALL / TORQUE_LIMITED / EDGE /
  TIMEOUT, per end — only WALL vouches for a limit). MEASURE-ONLY: the borrowed
  encoder offset is restored, and there is no `--commit` — keeping a re-zero is a
  separate, explicitly gated act. Also reports the delta against the EEPROM-derived
  bounds `arm explore` uses today. Gated motion: three-mode consent + `--apply`.
- `arm101-cli arm rezero <joint>` — shift the servo's encoder zero (EEPROM addr
  31) so the 4095->0 seam falls in the arc the joint cannot reach (issue #35;
  only `elbow_flex`, the one joint whose arc has been MEASURED — every other joint
  is refused *with the reason*, and the reasons differ: `wrist_roll` is impossible,
  the rest are unknown). **Commands no motion.** `--verify` runs the torque-off,
  hand-driven sweep that proves the seam actually moved. Gated: three-mode consent
  + `--apply`.
- `arm101-cli arm setup <role>` — assign EEPROM ids 1–6 at 1 000 000 baud for
  all 6 motors of the given role and auto-catalog each motor's servo_model and
  gear_ratio from `arm_spec`. Gated; uses the three-mode consent walk.

## Roles

- `follower` — labels F1–F6, all `ST-3215-C001/C018/C047`, gear ratio `1:345`.
- `leader` — labels L1–L6, mixed variants (C044 / C001 / C046), mixed gears.

## Usage

    arm101-cli arm overview
    arm101-cli arm read
    arm101-cli arm read --role leader --json
    arm101-cli arm flex shoulder_pan --to 2048 --apply
    arm101-cli arm flex --demo --apply
    arm101-cli arm profile shoulder_pan --contact-to 3500 --apply
    arm101-cli arm limits --apply
    arm101-cli arm limits elbow_flex --apply --json
    arm101-cli arm rezero elbow_flex --apply
    arm101-cli arm rezero elbow_flex --verify --apply
    arm101-cli arm setup follower
    arm101-cli arm setup follower --apply
"""

_ARM_READ = """\
# arm101-cli arm read

Read every joint's live register state for an arm role and print it as a table
(or `--json`). Read-only on the motor bus — it opens a bus and reads
`present_position`, `present_load`, `present_speed`, `present_voltage`,
`present_temperature`, `torque_enable`, and `Homing_Offset` for each of the six
joints, but commands no motion and writes no register. Because nothing is
mutated, there is **no consent gate** — unlike `arm flex`/`arm setup`.

The `offset` column / field is the servo's encoder offset (`Ofs` /
`Homing_Offset`, EEPROM address 31), shown **signed** — the bus decodes the
register's sign-magnitude wire form, so an offset of `-1073` reads as `-1073`
and not as the raw `3121`. It is surfaced because issue #35 fixes `elbow_flex`'s
mid-travel encoder wrap by re-zeroing that register, and inspecting the current
re-zero must never require performing one. `0` on a factory servo.

Retry-tolerant: each joint is read with bounded retries
(`arm101.hardware.arm_read.read_arm`). A joint whose first read succeeds is
`ok`; one that succeeds only after a retry is `partial`; one whose reads all
fail is `failed` (its register cells render as `-` / `null`). A single dead
joint never aborts the snapshot — the other joints still read, and the report
carries a `complete` flag (false when any joint failed).

## Flags

- `--role {follower,leader}` — which arm's joint→id map to read (default
  `follower`).
- `--port PORT` — serial port; default auto-detects the first candidate port.
- `--json` — emit `{"role", "port", "complete", "joints": [...]}` where each
  joint dict carries `joint`, `id`, `health`, and the six register fields.

## Usage

    arm101-cli arm read
    arm101-cli arm read --role leader
    arm101-cli arm read --port /dev/ttyACM0 --json

## Exit codes

- `0` success (even when some joints are `partial`/`failed` — that is data,
  not an error).
- `2` environment/setup error (no serial port found, SDK absent, or the port
  cannot be opened).

## Hardware / TTY behavior

Requires a real motor bus and the Feetech SDK (the `[seeed]` extra). The table
goes to stdout; diagnostics to stderr. Run from an agent freely — it never
moves the arm.
"""

_ARM_FLEX = """\
# arm101-cli arm flex

Command motion on the SO-101: move ONE joint to a target encoder tick
(`<joint> --to <tick>`), or sweep EVERY joint through a conservative safe
sub-range (`--demo`). This is a **gated motion verb** — it can physically move
the arm, so it uses the same three-mode consent as `arm setup`.

A single-joint move clamps the target to the joint's calibrated
`[min_angle, max_angle]` (read from the motor) and then either:

- **compliant** (default) — one gentle ramp-and-go move
  (`arm101.hardware.motion.compliant_move`); or
- **gentle** (`--gentle`) — a load-watch back-off-then-hold move
  (`arm101.hardware.gentle.gentle_move`): it steps toward the target watching
  `present_load` after each step, and on contact (load past `--threshold`,
  default 250) it stops, retreats a bounded back-off, and **holds with torque
  on** — never a limp release, never a hard press at the contact point.

`--demo` runs the scripted safe-exploration sweep
(`arm101.hardware.demo.demo_sweep`) across all joints; it is inherently gentle
(every sub-move is load-watched) and aborts cleanly on the first contact.

## Flags

- `joint` (positional, optional) — one of the six joints; required with `--to`
  unless `--demo` is given.
- `--to TICK` — target encoder tick for the single-joint move.
- `--demo` — sweep all joints instead of moving one (mutually exclusive with a
  joint + `--to`).
- `--gentle` — use the load-watch back-off-then-hold primitive.
- `--threshold N` — gentle contact-load threshold override (default 250).
- `--role {follower,leader}` — joint→id map to use (default `follower`).
- `--port PORT` — serial port; default auto-detects the first candidate.
- `--apply` — execute the motion in non-TTY (agent) mode.
- `--json` — emit the structured move/sweep result.

## Consent modes

1. **TTY (interactive)** — prints the planned motion, then prompts the human to
   type `yes` before any bus is opened. Declining aborts with zero motion.
2. **Non-TTY without `--apply`** — prints a dry-run plan (joint/target or the
   demo joint list, gentle/threshold settings) and stops: **zero motion, zero
   bus access**.
3. **Non-TTY with `--apply`** — proceeds (agent mode) and commands the motion.

## Usage

    arm101-cli arm flex shoulder_pan --to 2048 --apply
    arm101-cli arm flex gripper --to 2600 --gentle --threshold 300 --apply
    arm101-cli arm flex --demo --apply
    arm101-cli arm flex shoulder_pan --to 2048        # non-TTY: dry-run plan

## Exit codes

- `0` success, clean abort, or a non-TTY dry-run plan.
- `1` user/usage error (joint and `--demo` together, neither given, missing
  `--to`, or an unknown joint).
- `2` environment/setup error (no port, SDK absent, comms failure).

## Hardware / TTY behavior

Requires a real motor bus and the Feetech SDK (the `[seeed]` extra). The move
result goes to stdout; the confirmation prompt and warnings go to stderr.
"""

_ARM_EXPLORE = """\
# arm101-cli arm explore

Flood-fill and map the arm's reachable joint-space for a role, moving
outward from its live home pose via the same overload-safe `gentle_move`
primitive `arm flex --gentle` uses. This is a **gated motion verb** — it can
run many probe moves against the arm, so it uses the same three-mode consent
as `arm flex`/`arm setup`.

Each probe is watched for contact against a **per-joint contact threshold**
(see "Per-joint thresholds" below — free-motion load differs a lot per
joint, so one global number is always wrong for someone). When a joint is
blocked, the explorer does not stop at that single-joint limit: it runs a
bounded, pruned multi-joint **escape search** — perturbing other joints and
retrying — so combinations that unblock a joint (joint A blocked until
joint B moves first) get recorded too, not just A's first-contact limit.

## Dual artifacts

Every run writes two files:

- a **JSONL event log** (`<name>.events.jsonl`) — the append-only, resumable
  source of truth: every probe/contact event as it happens. A killed run
  resumes from this log instead of re-probing already-mapped cells.
- a **compact reachability map** (`<name>.map.json`) — derived from the log:
  per-joint reachable ranges plus a sparse list of blocked joint-combinations.
  This is what downstream code queries (`arm101.explore.reachmap.is_reachable`)
  offline, straight from the file — no bus opened, no motor moved.

## `--map` resume / override

`--map PATH` names the map file; if it already exists it is the resume input
for this run (already-mapped cells are not re-probed) as well as the write
target. Without `--map`, the default is `./arm-explore-<role>.map.json`, and
the JSONL log is always the sibling `<same-base>.events.jsonl`. A bundled
self-collision default map ships and loads automatically when no user map is
present.

## Per-joint thresholds

Free-motion load differs a lot per joint — gripper gear-friction alone can
run up to ~320, `shoulder_lift` holds the arm's own mass so its free-motion
gravity load sits around ~250, and the lighter joints load much less — so a
single global threshold is always wrong for someone: too high misses real
contacts on light joints, too low false-triggers `shoulder_lift` on its own
gravity. Each joint's threshold is resolved independently, **first match
wins**:

1. `--threshold-joint NAME=VAL` (repeatable) — highest precedence, one joint.
2. `--threshold N` — a blanket override broadcast to EVERY joint.
3. `--threshold-file PATH` — a JSONL file of per-joint thresholds.
4. the built-in per-joint default (hardware-tuned; see
   `arm101.hardware.arm_spec.DEFAULT_CONTACT_THRESHOLDS`) — used for any
   joint none of the above name.

`--threshold` only broadcasts when EXPLICITLY given: omitting it does not
collapse every joint to a fixed number — each joint instead falls through to
`--threshold-file`/the built-in default.

## Flags

- `--role {follower,leader}` — which arm's joint→id map to use (default
  `follower`).
- `--port PORT` — serial port; default auto-detects the first candidate.
- `--map PATH` — reachability-map file: resume input if it exists, and the
  written output (default `./arm-explore-<role>.map.json`).
- `--threshold N` — blanket contact-load threshold applied to EVERY joint,
  overriding `--threshold-file` and the per-joint defaults.
- `--threshold-joint JOINT=LOAD` — override one joint's contact threshold
  (repeatable), e.g. `--threshold-joint shoulder_lift=350`. Beats
  `--threshold` and `--threshold-file` for that joint.
- `--threshold-file PATH` — a JSONL file of per-joint contact thresholds,
  one `{"joint": "<name>", "threshold": <int>}` object per line.
- `--max-moves N` — budget cap on total moves/probes before the run stops
  (default 2000; hardware-tuned open question).
- `--resolution N` — per-joint grid bucket size in encoder ticks (default
  512; hardware-tuned open question).
- `--apply` — execute the exploration in non-TTY (agent) mode.
- `--json` — emit `{"verb", "role", "port", "cells_visited", "moves",
  "reachable", "contacts", "escapes_attempted", "escapes_succeeded",
  "budget_bounded", "map_path", "log_path"}`.

## Consent modes

1. **TTY (interactive)** — prints the planned run, then prompts the human to
   type `yes` before any bus is opened. Declining aborts with zero motion.
2. **Non-TTY without `--apply`** — prints a dry-run plan (role, map/log
   paths, per-joint thresholds, resolution, max-moves) and stops: **zero
   motion, zero bus access**.
3. **Non-TTY with `--apply`** — proceeds (agent mode) and drives the run.

## Usage

    arm101-cli arm explore --apply
    arm101-cli arm explore --role leader --map ./bench-a.map.json --apply
    arm101-cli arm explore --threshold-joint shoulder_lift=350 --apply
    arm101-cli arm explore --threshold 300 --max-moves 500 --apply
    arm101-cli arm explore --json --apply

## Scope (v1)

`arm explore` **produces and stores** the reachability map, and the map is
**queryable** offline straight from the file. It does not change any other
verb's behavior: consuming the map to gate `arm flex` targets (refuse/warn
on a request outside the discovered envelope) is a documented follow-up, not
part of this verb.

## Exit codes

- `0` success, clean abort, or a non-TTY dry-run plan.
- `1` user/usage error (an unknown joint name or non-integer value in
  `--threshold-joint`/`--threshold-file`, a malformed `--threshold-file`
  line, or a non-positive `--resolution`).
- `2` environment/setup error (no port, SDK absent, comms failure, an
  unreadable `--threshold-file`).

## Hardware / TTY behavior

Requires a real motor bus and the Feetech SDK (the `[seeed]` extra). The run
summary goes to stdout; the confirmation prompt and warnings go to stderr.
"""

_ARM_PROFILE = """\
# arm101-cli arm profile <joint>

Find the highest Goal_Speed at which the arm can **still detect a contact** for
one joint — and record that joint's measured travel rate and motion-onset
latency at it. This is a **gated motion verb**: it deliberately drives the joint
into an obstacle, over and over, at rising speeds, until it finds a speed at
which the software can no longer tell it has hit anything.

## Why it exists

`arm explore` mapped **2 cells in 25 minutes** and left the arm at ~50 C. Probe
cost is the bottleneck, and probe cost is dominated by TRAVEL TIME. But every
motion constant in the arm was hand-fitted in a single bench session
(`gentle`'s `_DEFAULT_SPEED = 150`, `_MIN_TICKS_PER_SECOND = 120`) — and at
speed 150 a 500-tick move measures ~930 ms on `wrist_roll` but ~3300 ms on the
shoulders. Nobody knows what the arm can actually sustain. This verb measures
it, per joint.

## The rule: a speed the servo SURVIVES is not a speed that WORKS

Contact detection and speed are **coupled**. The stall rule (see
`arm101.hardware.gentle`) calls CONTACT when load crosses a threshold *while the
joint has stopped advancing*. Both halves are needed: a joint merely
ACCELERATING through open air peaks at a load of 300 on `wrist_roll`, above its
own threshold, so the load gate alone would fire on every move. The stall gate
needs a *moving* joint to visibly ADVANCE between samples — at the 25 ms poll
interval the slowest joints cover ~4 ticks per sample, over the 2-tick
`stall_eps`. Drive faster and that margin erodes: the joint pressed into a
compliant contact keeps creeping *harder*, reads as "still moving", and the
stall counter never accumulates — or the servo's own overload latch (error=32)
trips first and cuts torque before the software rule has seen its 8 consecutive
stalled samples.

So:

> **A speed at which the arm moves but contact can no longer be detected is a
> FAILURE of that speed, not a pass. Free motion at a speed proves NOTHING.**

Every candidate speed is therefore certified against a **real contact**: the
joint is driven into `--contact-to` and the shipped `gentle_move` must come back
reporting `contacted=True`. Not a copy of the detector — the detector itself.

## `--contact-to` is required, and it must be UNREACHABLE

`--contact-to TICK` names a tick the joint genuinely **cannot** reach: its
mechanical end-stop, or a fixture you have clamped in its path. If the joint
sails to it through free air, the probe met nothing, nothing was proven, and the
run is **void** (exit 1) rather than quietly reporting a "safe speed" certified
against thin air.

Note `wrist_roll` (and `elbow_flex`) can rotate past their encoder wrap — a joint
with no reachable end-stop needs a physical fixture, or it cannot be profiled.

## The ramp

Candidates run low to high from `--speed-start` (default 150 — `gentle_move`'s
own default, and the only speed contact detection has ever been proven at on this
hardware) in `--speed-step` (default 50) up to `--speed-max` (default 600, which
brackets the speed 400 at which a one-shot overload was measured).

The ramp **stops at the first rejection**. Speed → detection is monotone (more
speed can only erode the margin), and probing above a speed already known to miss
contacts would mean slamming the arm into an obstacle at a speed where the
software cannot tell it has hit. The last ACCEPTED speed is the answer.

Between candidates the joint is retreated to its home pose at the last
**certified** speed — never at the untested candidate — and the run always ends
with the joint returned home and **de-energised**: a profiling run's last act must
not be to leave a joint holding itself against the wall it was just driven into.

## Verdicts

Each candidate ends in exactly one of four, and only the first is a pass:

- `contact_detected` — **ACCEPT**. The stall rule fired on a real obstacle.
- `contact_missed` — **REJECT**. The approach loaded past the joint's threshold —
  it demonstrably met something — and the rule never fired. At this speed the
  detector can no longer tell "blocked" from "accelerating".
- `overload` — **REJECT**. The servo's own latch (error=32) cut torque before the
  software rule could accumulate its 8 stalled samples. The contact was survived,
  not detected. Recovered gracefully; a legitimate ceiling.
- `no_contact` — **REJECT**. The probe never loaded past the threshold at all.
  On the first candidate this voids the run (see `--contact-to` above).

## Flags

- `<joint>` — which joint to profile (one of the six SO-101 joints).
- `--contact-to TICK` — **required**; a tick the joint cannot reach.
- `--threshold N` — contact-load threshold (default: the joint's hardware-tuned
  per-joint value, the same one `arm explore` uses). Must be **< 500** —
  `present_load` saturates at `gentle_move`'s Torque_Limit cap, so a threshold at
  or above it can never fire at any speed.
- `--speed-start N` / `--speed-step N` / `--speed-max N` — the ladder.
- `--role {follower,leader}` / `--port PORT` — as `arm read`/`arm flex`.
- `--apply` — execute in non-TTY (agent) mode.
- `--json` — emit `{"verb", "role", "port", "joint", "motor", "home",
  "contact_target", "threshold", "ladder", "certified", "safe_speed",
  "ticks_per_second", "motion_onset_seconds", "ceiling_speed", "ceiling_reason",
  "trials": [...]}`. Per-trial progress is emitted to **stderr** as it runs.

## Consent modes

1. **TTY (interactive)** — warns that this drives the joint into a contact at
   rising speeds, then prompts for `yes` before any bus is opened.
2. **Non-TTY without `--apply`** — prints a dry-run plan (joint, motor, contact
   target, threshold, ladder) and stops: **zero motion, zero bus access**.
3. **Non-TTY with `--apply`** — proceeds (agent mode) and drives the ramp.

## Usage

    arm101-cli arm profile shoulder_pan --contact-to 3500 --apply
    arm101-cli arm profile gripper --contact-to 3200 --speed-max 400 --apply
    arm101-cli arm profile elbow_flex --contact-to 500 --json --apply
    arm101-cli arm profile shoulder_lift --contact-to 3800   # non-TTY: dry-run plan

## Exit codes

- `0` success (including "no safe speed found" — that is a finding, not an error),
  a clean abort, or a non-TTY dry-run plan.
- `1` user/usage error: an unknown joint, a missing `--contact-to`, an
  out-of-range `--speed-*`, or a `--contact-to` the joint can actually reach
  (the void run — nothing was certified).
- `2` environment/setup error (no port, SDK absent, comms failure).

## Hardware / TTY behavior

Requires a real motor bus and the Feetech SDK (the `[seeed]` extra). The result
table goes to stdout; the confirmation prompt, per-trial progress, and any torque
release go to stderr. The whole run is wrapped in a torque guard: an abnormal exit
(bus fault, `Ctrl-C`) de-energises the joint before the error propagates.
"""

_ARM_OVERVIEW = """\
# arm101-cli arm overview

Read-only snapshot of the `arm` noun surface: known roles, joints, and the
complete per-role id / baud / servo_model / gear_ratio map from `arm_spec`.
Accepts an ignored positional `target` so a stray path argument never
hard-fails — it always exits 0.

## Usage

    arm101-cli arm overview
    arm101-cli arm overview --json
    arm101-cli arm overview <anything>  # ignored; still exits 0

## JSON output

Emits `{"noun": "arm", "verbs": [...], "roles": [...], "motor_map": {...}}`.
`motor_map` is keyed by role then joint, with each entry carrying `id`, `baud`,
`servo_model`, and `gear_ratio`.
"""

_ELBOW_ARC = arm_spec.REZERO_ARCS["elbow_flex"]

#: Rendered from :data:`arm101.hardware.arm_spec.REZERO_ARCS` at import, NOT copied
#: from it. The arc exists to be RE-MEASURED on hardware, and this text is what
#: prints to whoever is standing at the arm — so it must not be able to drift from
#: the table. A doc that names a measurement is a doc that goes stale; a doc that
#: RENDERS one cannot.
_ARM_REZERO = f"""\
# arm101-cli arm rezero <joint>

Shift a joint's **encoder zero** — the servo's `Ofs` / `Homing_Offset` register
(EEPROM addr 31) — so that the encoder's 4095->0 **seam** falls inside the arc
the joint physically cannot reach. **Commands no motion, on any path.**

## The bug this fixes (issue #35)

`elbow_flex`'s 12-bit encoder wraps *inside its own physical travel*. Driven far
enough it crosses the raw 4095->0 seam and reads back near zero, so its reported
position is **not monotonic with joint angle**: its two measured endpoints sort
into a `[min, max]` pair describing exactly the arc it CANNOT reach, and every
position comparison in this codebase — `gentle_move`'s arrival check,
`clamp_goal`, the reachability map's ranges — is silently wrong for it. It
currently rests at raw ~126, i.e. *past* its wrap.

Move the seam into the joint's unreachable arc and every tick it can actually
reach lies on one side of it. The tick axis is linear again — genuinely, not by
assumption.

## Two frames, and everything turns on keeping them apart

A servo reports `Present = (Actual - Ofs) mod 4096`, so there are two tick frames
and they coincide only at `Ofs == 0` — which **no servo ships doing**. The
factory default is **85**, measured uniform across all six joints of the follower
(2026-07-12).

- **RAW** — the magnet on the shaft. The joint's walls, its unreachable arc, and
  the seam (which lands where `Actual == Ofs`) all live here. Writing the offset
  register moves none of it.
- **REPORTED** — what comes back over the wire, i.e. *everything* `arm read` and
  `read_position` hand you. Shifted by whatever offset the servo holds.

`arm_spec.REZERO_ARCS` is **RAW ticks**, and the numbers below are RENDERED from
that table — not copied — so they cannot drift from it when the arm is
re-measured.

`elbow_flex`'s unreachable arc is currently `({_ELBOW_ARC.low}, {_ELBOW_ARC.high})`
(width {_ELBOW_ARC.high - _ELBOW_ARC.low}), leaving a reachable travel of about
{_ELBOW_ARC.travel_ticks} ticks: raw `[{_ELBOW_ARC.high}, 4095] ∪ [0, {_ELBOW_ARC.low}]`,
which *wraps* — which is exactly the fact a `[min, max]` pair cannot express, and the
whole of issue #35. Every live reading is converted (`raw = (reported + offset) mod
4096`) before it is compared against the arc.

Those walls were measured BY THE ARM, not by hand: `gentle_move` was driven past the
known travel and left to find each one by feel, stopping when `present_load` saturated.
A human stops when it *feels* firm; the arm presses to a fixed load every time, so its
walls are further out and repeatable. The arc is deliberately INSET from them, so a
harder push can never make the table contradict the arm.

## Why it commands no motion — the bootstrap problem

The tool that MAKES the axis linear cannot itself rely on the axis being linear.
"Drive the joint to mid-travel, then centre it" is the natural procedure and it
is exactly the one that must not run: from a rest position on the far side of
its wrap, a
linear goal at its mid-travel looks like a modest move and is in fact a rotation
*the long way round* — down through 0, across the whole arc the joint
cannot reach, and into a wall. So this verb reads where the joint physically
**is**, computes the offset from the joint's known unreachable arc (a measured
table fact, in `arm_spec.REZERO_ARCS`), and writes it. No goal position is ever
written.

Torque is disabled before the EEPROM write and left off: a servo must not be
*holding* while its own frame of reference changes underneath it.

## Which joints

Only `elbow_flex`. Every other joint is refused **with the reason** — and there
are two structurally different reasons, which the verb keeps apart:

- **`wrist_roll` — impossible.** A re-zero only *relocates* a seam; it can never
  *evict* one. Eviction needs an arc the joint cannot reach, and exploration
  found no wall anywhere in `wrist_roll`'s travel (measured free range
  `[21, 4073]`) — it turns freely all the way round, so every angle is reachable,
  including whichever one the seam is moved to. It is handled instead by a
  software **soft limit** (`arm_spec.SOFT_LIMITS`), already in force. **This
  refusal is PROVEN and permanent.**
- **The other four — UNKNOWN.** Rendered from `arm_spec`, so it cannot drift from
  the table again:

  > {arm_spec.REZERO_ARC_UNKNOWN_SUMMARY}

  This section used to say those four were *unnecessary* — that their encoders do
  not wrap inside their travel. Hardware said otherwise, and the claim was
  withdrawn (issue #43). "You don't need one" is still a real answer; it is just no
  longer a table's to give. A **measurement** can earn it — a BOUNDED travel that
  misses the seam — and `arm101-cli arm limits <joint>` is what takes it.

## The goal is a PLACE, not a number

"Re-zeroed" means **the seam is outside the joint's travel**, not "the register
holds the computed target". Any offset whose seam tick lands strictly inside the
arc has done the job; `arc.midpoint` is simply the one
with the most margin, and is what a *fresh* re-zero writes.

So a servo already holding a *different* evicting offset is **already fixed**, and
`--apply` reports a **no-op** and writes nothing. (Our follower holds `1073`, from
an earlier re-zero computed in the wrong frame; its seam sits at raw 1073, deep
inside the arc, and a sweep proved its travel continuous. Rewriting it
to the midpoint would spend an EEPROM write to slide a seam from one unreachable tick to
another.) A servo holding the **factory 85** is *not* fixed: raw 85 is inside
`elbow_flex`'s reachable `[0, 207]` band, which is issue #35 exactly.

## `--verify` — the seam-eviction proof

**Reading the offset back only proves it was APPLIED. It does not prove the seam
MOVED.** One undocumented bit of firmware semantics decides which:

    Present = (raw - Ofs) mod 4096     seam RELOCATES  -> the fix works
    Present =  raw - Ofs   (signed)    seam STAYS      -> the fix does NOTHING

**It is the first — settled on hardware, 2026-07-12.** With `Ofs = 0` the sweep
came back `monotonic: False, discontinuities: 1`; with `Ofs = 1073` (inside the
arc) it came back `monotonic: True, discontinuities: 0` across its whole travel. No
primary Feetech source states the formula, so `--verify` remains the check — one
arm and one firmware revision is not every arm, and a verification that cannot
fail is not a verification.

`--verify`: torque goes **off** and stays off, a **human hand-moves** the joint
through its entire travel, and the verb polls `present_position` and asserts there
is **no discontinuity anywhere**. A human arm is the right instrument precisely
because it is the only actuator available that does not need a linear tick axis to
work.

It reports the range reached (in both frames), whether the sweep was monotonic,
and the largest single-sample jump — a seam crossing is ~1781-4095 ticks; sensor
noise and a human hand are tens.

Four verdicts, because "did not fail" is not the same claim as "proved it works":

- `seam-evicted` — re-zeroed, continuous, and the sweep actually covered the
  travel. The fix works.
- `seam-not-evicted` — re-zeroed and **still** discontinuous. **STOP.** The
  re-zero achieves nothing; exit code 2, and the decision goes back to the user.
- `seam-present-baseline` — not re-zeroed, discontinuous. The bug, photographed.
  Expected before the write; not a failure.
- `inconclusive` — continuous, but either no offset was in force or the joint was
  not moved through enough of its travel for "no seam" to mean anything.

`--verify` deliberately ends with the joint **limp** — the operator's hand is on
it. If the arm is holding a pose it will sag: support it.

## Consent modes

Same three-mode gate as `arm flex` / `arm explore` (1-step tier):

1. **TTY (interactive)** — confirm at a prompt.
2. **Non-TTY without `--apply`** — dry-run: prints the exact register writes and
   opens **no bus at all**.
3. **Non-TTY with `--apply`** — executes.

## Usage

    arm101-cli arm rezero elbow_flex                   # dry-run: the exact writes
    arm101-cli arm rezero elbow_flex --apply           # write the offset
    arm101-cli arm rezero elbow_flex --verify --apply  # prove the seam moved
    arm101-cli arm rezero elbow_flex --verify --duration 45 --apply
    arm101-cli arm rezero wrist_roll                   # refused, with the reason
    arm101-cli arm rezero elbow_flex --json

## After the write — what is NOT yet proven

The read-back proves the offset was **applied**, not that it **persists**: PR #21
exists because id/baud EEPROM writes read back correctly and silently reverted on
the next power-cycle. **Power-cycle the servo** (cut and restore bus power, not
just the serial link), re-read with `arm read`, then run `--verify`. The full
hand-run procedure is in `docs/hardware-rezero-procedure.md`.

## Exit codes

- `0` success, clean abort, a non-TTY dry-run plan, or an informative sweep
  (baseline / inconclusive).
- `1` user/usage error (an unknown joint, a joint that cannot be re-zeroed, a
  `--duration` too short to collect two samples).
- `2` environment error (no port, SDK absent, comms failure), the offset failing
  to read back, the joint reporting a raw position inside its own unreachable arc
  — **and the `seam-not-evicted` verdict**, which is a stop condition, not a
  retryable error. (An *unfamiliar* offset is no longer an error: the verb reads
  whatever the register holds, converts out of it, and reasons in raw ticks.)

## Hardware / TTY behavior

Requires a real motor bus and the Feetech SDK (the `[seeed]` extra). The result
and the sweep report go to stdout; the prompt, the live sample feed, and every
warning go to stderr.
"""

_ARM_LIMITS = f"""\
# arm101-cli arm limits [<joint>...]

**Measure** each joint's true travel — and, with `--commit`, KEEP the remedy it points to.

Per joint: roll the encoder seam out of the joint's way, creep to BOTH ends of its
travel under contact detection, and rule on what stopped it. Then classify the
travel, and diff the measured span against the EEPROM-derived bounds `arm explore`
builds its grid from today.

This is the verb the rolling frame, the probe, and the classifier were built for.

## MEASURE-ONLY by default

The run borrows the servo's encoder offset (`Ofs`/`Homing_Offset`, EEPROM addr 31)
to keep the 4095->0 seam half a turn ahead of the creep — otherwise a joint whose
travel crosses the seam reports a 200-tick move as a 3896-tick retreat, and the seam
wears a limit's clothes. It **puts the original offset back** on every exit path,
clean or not, so the servo ends the run in the calibration it started it in.

Keeping the remedy is a **separate, explicitly gated act**: `--commit`, and it is off
unless you type it. A verb that silently re-calibrated five joints because somebody
asked it to *look* at them is not one anybody should run.

## `--commit` — and THE SWEEP IS THE ARBITER, NOT THE READ-BACK

The measurement is identical. What changes is that the **remedy the travel points to**
is then kept — and which remedy that is comes from the joint, not from a preference:

- **BOUNDED**, with an arc that can take the seam -> **RE-ZERO.** A persistent EEPROM
  write (`Ofs`, addr 31) moving the seam into the arc the joint cannot reach — and then
  **PROVEN BY A TORQUE-OFF HAND SWEEP.** You are asked to walk the joint from one hard
  stop to the other while the verb watches. Reading the offset back proves it was
  *applied*; it proves **nothing** about whether the seam *moved*. Only a sweep can, and
  a sweep is the only thing that gets to decide.
- **CONTINUOUS** (or an arc too narrow to hold the seam clear of both walls) -> **SOFT
  LIMIT.** Software only: a dead arc containing the seam, which no mover may enter. No
  servo register is written at all.
- **UNDETERMINED** -> **nothing.** Neither instrument is supported by the evidence, and
  choosing one anyway would be inventing a measurement. Measure again.

A sweep that finds a discontinuity is a **FAILURE**: the original offset is restored,
the journal is closed, and the verb exits non-zero. So is a sweep that was too *short* —
it must cover at least 80% of the joint's travel or it is `inconclusive`, never a pass.
That rule is not pedantry: it is what stopped three EMPTY sweeps of `elbow_flex` (0, 0
and 376 ticks against an expected ~2202) from being declared a success. **A clean sweep
of a joint nobody touched proves nothing** — of course it saw no seam; it never went near
where the seam would be. An unattended `--apply --commit` is refused for exactly this
reason, and that is the verb working, not a limitation of it.

Every commit is a TRANSACTION: the offset is durable on disk *before* it reaches the
wire, and only a passing sweep closes the journal as `committed`. A crash, a Ctrl-C or a
yanked cable in between leaves it dirty, and the next run restores the original. **An
unverified re-zero cannot survive** — "it died before it could check" is not evidence
that it would have passed.

## Where a measured SOFT LIMIT goes, and what reads it

A re-zero is an EEPROM write, so committing one is obvious. A soft limit is
software-only — and `arm_spec.SOFT_LIMITS` is a checked-in source table, which a CLI
does not rewrite. So a measured soft limit is **appended to a store**
(`~/.arm101/soft-limits.jsonl`; `$ARM101_SOFT_LIMITS` or `--soft-limit-file` to relocate),
in RAW ticks, with the offset it was derived against and the pose it was measured in.

**That store is loaded on every run of every motion verb** — `arm flex`, `arm explore`'s
grid, the demo sweep — merged over the shipped table by `arm_spec.resolve_soft_limits`
and bound by `arm_spec.resolve_bounds`, the one function all of them take their move
bounds from. It is loaded whether or not you pass the flag, because a fence that only
binds when you remember to ask for it is not a fence. (This repo shipped an inert soft
limit once already: the `wrist_roll` entry meant nothing for a whole release, because
every mover was reading the servo's factory `0-4095` EEPROM registers instead.)

The commit also **prints the `arm_spec` table entry to check in**. The store makes the
limit true for this arm today; the checked-in table is how it stops being local knowledge.

## What is NEVER written, on any path

The servo's `Min_Position_Limit` / `Max_Position_Limit` (addrs **9** and **11**). They
clamp every goal in firmware, which is exactly why they look tempting — and they are
EEPROM, so a fence written there outlives the pose that produced it and travels with the
servo onto the next arm. A measured range is a claim about *this arm in this pose*; it
belongs in software, where re-measuring can correct it. `tests/test_eeprom_limit_write_guard.py`
pins the whole package's wire surface shut against them.

A calibration a *crashed* run left behind is restored first (`require_clean`), before
this verb touches the arm at all. Layering a fresh temporary offset on top of one
nobody restored is how the original offset stops being knowable.

## Four verdicts, per END — because a wall and a weak arm look identical

`present_load` **saturates** at `gentle_move`'s Torque_Limit cap (500). So a joint
pressed against a mechanical limit and a joint that has simply run out of torque read
*exactly the same* at the moment they stop: load 500, not advancing. `shoulder_lift`
carries the whole arm, and recording its torque-limited stall as a mechanical limit
would write a permanent lie into `arm_spec` — a wall in a place the arm can, in
another pose, walk straight through.

What differs is the **approach**, not the stop:

- **`wall`** — free travel, then a sharp transition to a saturated, stalled load,
  inside the few tens of ticks of give a real contact on this arm was measured to
  have. **A real limit.** The only verdict that vouches for one.
- **`torque_limited`** — it was ALREADY pushing past its contact threshold *while it
  was still advancing*, for far longer than a contact's give. A joint working that
  hard while still moving is carrying a load, not meeting an obstacle. **A LOWER
  BOUND, never a wall** — and no number of poses promotes it, because the arm's own
  weakness is present in every pose.
- **`edge`** — ran out of travel budget without anything stopping it. Nothing bounds
  the joint here; the record is a lower bound.
- **`timeout`** — never arrived. A slipped gear, a servo not following, a bus not
  being heard. Nothing was learned about this end.

The verdict is carried **per end, not per joint**: gravity helps a joint down and
fights it up, so a record that averaged the two would be wrong at one end by
construction. **Every gap in the evidence falls toward `torque_limited`** — a false
lower bound under-claims the arm's reach and another pose can widen it; a false wall
is permanent and nothing can dislodge it.

## `loaded_run_ticks` — the number this verb exists to collect

WALL vs TORQUE_LIMITED turns on **how far the joint travelled while already pushing
past its own contact threshold**. A real contact's give is tens of ticks (`gentle`
measured it: a 30-70 tick retreat relieves one). A gravity climb spends *hundreds* of
ticks above its threshold on the way to running out.

**That cutoff is currently derived from a simulation, not from the arm** (default
`--compliance` = twice `gentle_move`'s measured contact-relief distance). Retuning it
from real data is the entire point of the first hardware session — so `--json` reports
`loaded_run_ticks`, `free_run_ticks`, `compliance`, `peak_load`, the verdict and the
reason **per joint, per end**. Nothing has to be re-instrumented to get them.

## The classification, and what can be done about the seam

- **`bounded`** — a WALL at both ends. The complement of the travel is a real,
  permanently unreachable arc, so the seam can be parked in it.
  Remedy: **`rezero`** if that arc is wide enough to hold the seam clear of both
  walls; **`soft_limit`** if it is only a sliver; **`none_needed`** if the travel
  never crossed the seam in the first place.
- **`continuous`** — the joint swept a FULL TURN. Every angle is reachable, so there
  is nowhere to put the seam and a re-zero cannot help *even in principle*: an offset
  RELOCATES a seam, it never EVICTS one. Remedy: **`soft_limit`** — a software-only
  dead arc the joint is simply never commanded into.
- **`undetermined`** — not two walls (so no arc can be sited) and not a full turn (so
  continuity is not shown either). Remedy: **`unknown`**. It says *measure again*, not
  *pick one*.

`continuous` is decided by **how far the joint swept**, never by "no wall was found"
and never by "it came back to where it started" (which is equally true of a joint that
never moved). A continuous joint is discovered by the FIRST probe — it sweeps a full
turn, the probe stops at the cap, and the second end is not driven at all.

**No joint names anywhere in the classifier.** `wrist_roll` must come back continuous
*because it is*, not because it is called `wrist_roll` — otherwise no verdict it gave
on the four unknown joints could be believed.

## The bounds diff (and the verdict it can lose)

`arm explore` builds its grid from `GridSpec.bounds` — the servo's EEPROM
`min_angle`/`max_angle`, intersected with the joint's soft limit. On this arm those
registers hold the untouched factory `0-4095`: **the EEPROM knows nothing about the
arm's real travel.** So the report gives, per joint, `measured - eeprom` in ticks:

- **negative** — the EEPROM claims travel the joint does not have, and every cell in
  that gap is one `arm explore`'s flood-fill will enqueue and the joint can never
  reach. That is issue #34's artifact, measured rather than assumed.
- **positive** — the arm reaches further than its configured limits permit, so moves
  are being clamped short of its real travel.

And if **no** joint differs materially (more than {MATERIAL_SPAN_DELTA_TICKS} ticks —
the bar is rendered from `arm101.hardware.limits`, not typed here), the report says so
plainly rather than burying it: that would mean the grid was NOT being fed artifacts
and the rationale for blocking issue #34 on this work is FALSE. A report that could
only ever confirm the reason it was commissioned is not a measurement.

A span with no wall behind it is a LOWER BOUND (the true travel can only be wider), so
those joints are flagged in the diff rather than dropped from it.

## Flags

- `[<joint>...]` — joints to measure; default **every** joint, in hardware order.
- `--threshold N` / `--threshold-joint JOINT=N` / `--threshold-file PATH` — contact-load
  thresholds, resolved exactly as `arm explore` resolves them (per-joint flag > blanket
  flag > file > hardware-tuned per-joint default).
- `--step TICKS` — ticks per creep step, i.e. the length of one gentle move.
- `--max-travel TICKS` — travel budget per END (default a full turn, past which the
  joint is CONTINUOUS and there is nothing left to learn).
- `--compliance TICKS` — the widest LOADED approach a WALL may show. **Raising it is the
  one change here that can manufacture a wall that is not there.**
- `--pose LABEL` — records which pose the OTHER joints were in. A limit found in one pose
  is *environmental*: it may be an obstacle, not the joint's own stop.
- `--commit` — KEEP the remedy each measurement points to (see above). **Requires a human
  at the arm**: every re-zero it writes must be proven by a hand sweep before it is kept.
- `--sweep-duration SECONDS` — how long you get to hand-sweep each re-zeroed joint
  (default 30). The sweep must cover ≥80% of the joint's travel or the commit is refused.
- `--soft-limit-file PATH` — the measured soft-limit store to read, and to append to under
  `--commit`. Loaded whether or not you pass it.
- `--role`, `--port`, `--apply`, `--json`.

## Consent modes

Gated motion — the same three-mode gate as `arm flex` / `arm explore` (1-step tier):

1. **TTY (interactive)** — confirm at a prompt.
2. **Non-TTY without `--apply`** — dry-run: prints the plan and opens **no bus at all**.
3. **Non-TTY with `--apply`** — executes.

Every motor the run may energise is owned by a `torque_guard` from the moment it can
first go hot, and never disowned: a bus that dies while joint 5 is being probed still
de-energises joints 1-4, whose frames closed minutes ago and whose servos may well
still be holding.

## What it does NOT do

No cells, no reachability score, no map. Producing a reachability map from these bounds
is `arm explore`'s job (and issue #34's) — this verb measures per-joint bounds and
verdicts, and stops there.

## Usage

    arm101-cli arm limits                              # dry-run plan for every joint
    arm101-cli arm limits --apply                      # measure every joint
    arm101-cli arm limits elbow_flex --apply --json    # one joint, with the full evidence
    arm101-cli arm limits shoulder_lift --pose "elbow folded, gripper clear" --apply
    arm101-cli arm limits elbow_flex --compliance 60 --apply   # retune the WALL cutoff
    arm101-cli arm limits --commit --apply             # measure, then KEEP each remedy
    arm101-cli arm limits elbow_flex --commit --apply  # re-zero one joint, sweep-verified

## Exit codes

- `0` success, a clean abort, or a non-TTY dry-run plan.
- `1` user/usage error (an unknown joint, a creep step that could not measure anything).
- `2` environment error (no port, SDK absent, comms failure, a calibration a previous
  run left dirty that could not be restored, a joint that cannot be held still long
  enough to re-centre its frame — **or a `--commit` whose sweep could not prove the seam
  moved**, which is a stop-and-return-to-the-user condition, not a retryable error).

## Hardware / TTY behavior

Requires a real motor bus and the Feetech SDK (the `[seeed]` extra). The report goes to
stdout; the prompt, the live sweep feed, the torque-release announcement, and every
warning go to stderr. `--commit` needs a **human hand** on the arm for each re-zeroed
joint — the seam-eviction proof is a torque-off sweep, and a human arm is the only
actuator in the building that does not need a linear tick axis to work, which is
precisely what is in doubt.
"""

_ARM_SETUP = """\
# arm101-cli arm setup <role>

Assign EEPROM ids 1–6 at 1 000 000 baud for all 6 motors of *role*
(`follower` or `leader`) and auto-catalog each motor's `servo_model` and
`gear_ratio` from `arm101.hardware.arm_spec`. Zero numbers typed by the
operator — all values come from the spec.

Reuses the existing `setup-motors` gated three-mode-consent walk (same serial
port auto-detection per motor), but records role-correct catalog entries
(`F{id}` for follower, `L{id}` for leader) with the physical BOM facts.

## Consent modes

1. **TTY (interactive)** — per-motor Enter-gated confirmation; catalog entries
   written after each motor.
2. **Non-TTY without `--apply`** — prints a dry-run plan table (zero writes,
   zero catalog entries).
3. **Non-TTY with `--apply`** — executes the headless 6→1 walk and saves
   catalog entries (1-step tier). Headless writes are attributed
   (`ARM101_OPERATOR` env / culture nick) and appended to
   `~/.arm101/audit.log`.

## Usage

    arm101-cli arm setup follower
    arm101-cli arm setup leader
    arm101-cli arm setup follower --apply
    arm101-cli arm setup follower --port /dev/ttyACM0
    arm101-cli arm setup follower --json

## Hardware / TTY behavior

Requires a real motor bus and the Feetech SDK (the `[seeed]` extra). Exit
codes: 0 success or non-TTY dry-run; 1 for an invalid role; 2 for a
hardware/setup error. `--json` emits `{"role", "assigned": [...]}` on success
or `{"role", "plan": [...]}` in dry-run; prompts and motor cards go to stderr.
"""


ENTRIES: dict[tuple[str, ...], str] = {
    (): _ROOT,
    ("arm101",): _ROOT,
    ("arm101-cli",): _ROOT,
    ("whoami",): _WHOAMI,
    ("learn",): _LEARN,
    ("explain",): _EXPLAIN,
    ("overview",): _OVERVIEW,
    ("doctor",): _DOCTOR,
    ("find-port",): _FIND_PORT,
    ("calibrate",): _CALIBRATE,
    ("calibrate-motor",): _CALIBRATE_MOTOR,
    ("set-motor-id",): _SET_MOTOR_ID,
    ("set-baudrate",): _SET_BAUDRATE,
    ("center-motor",): _CENTER_MOTOR,
    ("setup-motors",): _SETUP_MOTORS,
    ("cli",): _CLI,
    ("cli", "overview"): _CLI,
    ("arm",): _ARM,
    ("arm", "overview"): _ARM_OVERVIEW,
    ("arm", "read"): _ARM_READ,
    ("arm", "flex"): _ARM_FLEX,
    ("arm", "explore"): _ARM_EXPLORE,
    ("arm", "profile"): _ARM_PROFILE,
    ("arm", "limits"): _ARM_LIMITS,
    ("arm", "rezero"): _ARM_REZERO,
    ("arm", "setup"): _ARM_SETUP,
}
