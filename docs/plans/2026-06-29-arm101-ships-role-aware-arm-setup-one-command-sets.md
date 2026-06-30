# Build Plan — arm101 ships role-aware arm setup: one command sets up a full SO-101 arm for a chosen role (follower or leader) with zero numbers typed, driven by a single source-of-truth joint→id+baud arm spec that calibrate, setup-motors, and profiles all consume.

slug: `arm101-ships-role-aware-arm-setup-one-command-sets` · status: `exported` · from frame: `arm101-ships-role-aware-arm-setup-one-command-sets`

> arm101 ships role-aware arm setup: one command sets up a full SO-101 arm for a chosen role (follower or leader) with zero numbers typed, driven by a single source-of-truth joint→id+baud arm spec that calibrate, setup-motors, and profiles all consume.

## Tasks

### t1 — Create arm101/hardware/arm_spec.py — the single-source per-role motor map

- covers: c23, h18
- acceptance:
  - arm_spec exposes a role-keyed structure for follower and leader, each mapping all six joints (shoulder_pan..gripper) to id, baud, servo_model, gear_ratio; ids are 1..6 and baud is 1000000 for both roles
  - per-joint gears match the cited Seeed BOM: follower all 1:345; leader shoulder_pan and elbow_flex 1:191 (C044), shoulder_lift 1:345 (C001), wrist_flex/wrist_roll/gripper 1:147 (C046)
  - every value carries an inline source-citation comment (LeRobot so_follower/so_leader + feetech.py for id/baud; Seeed wiki for model/gear); a unit test asserts the accessor returns these exact values

### t2 — Refactor calibrate.py to import the joint-to-id map from arm_spec (remove _JOINT_MOTOR literal)

- depends on: t1
- covers: c3, h5
- acceptance:
  - calibrate.py no longer defines a local _JOINT_MOTOR literal; it derives the joint-to-id map from arm_spec
  - a behavior-preserving test asserts calibrate resolves the same map shoulder_pan:1..gripper:6 as before; existing calibrate tests stay green

### t3 — Refactor setup_motors.py to source walk order and default baud from arm_spec (remove _MOTOR_ORDER + _DEFAULT_BAUDRATE literals)

- depends on: t1
- covers: c3, h5, c5, h6
- acceptance:
  - setup_motors derives the 6-to-1 walk order and default baud from arm_spec instead of local literals
  - a behavior-preserving test asserts the walk order is gripper:6..shoulder_pan:1 and baud 1000000 unchanged; existing setup-motors tests stay green

### t4 — Refactor profiles.py to derive JOINTS from arm_spec (single joint-order source)

- depends on: t1
- covers: c3, h5
- acceptance:
  - profiles.JOINTS is derived from arm_spec joint order, not a separate literal; JOINTS still equals shoulder_pan..gripper in id order
  - a test asserts JOINTS is unchanged and profile read/write still round-trips

### t5 — Add the arm noun group (arm101/cli/_commands/arm.py): register, arm overview, and arm setup role

- depends on: t1, t3
- covers: c1, h1, c10, h3, c19, h15, c21, h17
- acceptance:
  - a new arm noun group is registered in _build_parser via _commands/arm.py:register() using parser_class=type(p) so child parse errors route through the structured error contract; arm overview accepts an ignored positional target, exits 0 on any path, supports --json
  - arm setup follower|leader drives the existing setup-motors gated three-mode-consent walk (resolve_consent dry-run/TTY/--apply) using arm_spec ids+baud, introducing no new consent code path
  - during the walk each motor is recorded into the motor catalog with its role-correct F/L label + servo_model + gear_ratio from arm_spec, with zero numbers typed
  - FakeBus tests prove: after arm setup leader the catalog holds L1-L6 with gears 1:191/1:345/1:147; after arm setup follower it holds F1-F6 at 1:345; dry-run writes nothing

### t6 — Lockstep docs for the arm noun/verbs (explain catalog + overview._VERBS + learn)

- depends on: t5
- covers: c15, h12
- acceptance:
  - explain catalog ENTRIES gains entries for the arm noun and the arm overview + arm setup paths; test_every_catalog_path_resolves passes
  - overview._VERBS lists the arm verbs and learn _TEXT/_as_json_payload mention arm setup + arm overview; teken cli doctor . --strict passes (arm noun exposes overview; descriptive verbs do not hard-fail)

### t7 — Cross-cutting verification: single-source dedup regression + scope guard + full gate

- depends on: t2, t3, t4, t5, t6
- covers: c2, h4, c5, h6, c14, h11, c15, h12
- acceptance:
  - a regression test asserts the joint-to-id literal exists in exactly one place (arm_spec) — grep finds no fourth copy — and calibrate/setup_motors/profiles all resolve identical maps (parallel==serial behavior-preserving)
  - a scope-guard test asserts deferred items are absent: no XDG arm-profile module, pyproject dependencies stay empty (zero new runtime dep), calibrate range math unchanged (not gear-aware)
  - full gate green: black/isort/flake8/bandit, pytest -n auto coverage >=60, teken cli doctor . --strict

### t8 — Hardware validation run-log: arm setup follower on a physical SO-101 follower

- depends on: t5, t6
- covers: c15, h12
- acceptance:
  - a documented run-log captures arm setup follower --apply exercised on a real follower arm (ids 1-6 assigned, catalog F1-F6 auto-filled), signed off by a human operator
  - the run-log notes the leader path is FakeBus-proven only pending physical leader hardware

## Risks

- [unknown_nonblocking] Leader hardware may be unavailable; the follower path is hardware-validated (t8) while the leader walk is FakeBus-proven only until a physical leader arm is on hand (task t8)
- [follow_up] Gear-corrected calibration math is deferred — once arm_spec carries gear ratios, a follow-up should make calibrate range math gear-aware
- [out_of_scope] User-editable XDG arm profile (Layer 3) is deferred to a later iteration
