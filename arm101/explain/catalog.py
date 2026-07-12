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
walk (`arm explore`), a gated speed-profiling ramp (`arm profile <joint>`), and
a gated setup walk (`arm setup <role>`).

## Verbs

- `arm101-cli arm overview` — describe the arm noun surface (roles, joints,
  and the per-role id / baud / servo_model / gear_ratio map). Read-only;
  always exits 0.
- `arm101-cli arm read` — read every joint's live register state
  (position/load/speed/voltage/temperature/torque). Read-only on the bus —
  no consent gate; a flaky joint is marked `partial`/`failed` while the rest
  still read.
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
    arm101-cli arm setup follower
    arm101-cli arm setup follower --apply
"""

_ARM_READ = """\
# arm101-cli arm read

Read every joint's live register state for an arm role and print it as a table
(or `--json`). Read-only on the motor bus — it opens a bus and reads
`present_position`, `present_load`, `present_speed`, `present_voltage`,
`present_temperature`, and `torque_enable` for each of the six joints, but
commands no motion and writes no register. Because nothing is mutated, there
is **no consent gate** — unlike `arm flex`/`arm setup`.

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
    ("arm", "setup"): _ARM_SETUP,
}
