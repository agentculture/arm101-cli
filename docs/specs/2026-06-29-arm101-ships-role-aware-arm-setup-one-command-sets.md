# arm101 ships role-aware arm setup: one command sets up a full SO-101 arm for a chosen role (follower or leader) with zero numbers typed, driven by a single source-of-truth joint→id+baud arm spec that calibrate, setup-motors, and profiles all consume.

> arm101 ships role-aware arm setup: one command sets up a full SO-101 arm for a chosen role (follower or leader) with zero numbers typed, driven by a single source-of-truth joint→id+baud arm spec that calibrate, setup-motors, and profiles all consume.

## Audience

- An operator (human or agent) setting up an SO-101 arm's motors, plus the three call sites that today each hardcode the joint→id map: calibrate, setup-motors, and profiles.

## Before → After

- Before: The SO-101 joint→id map is hardcoded and duplicated 3× with no single source: calibrate._JOINT_MOTOR (ascending dict), setup_motors._MOTOR_ORDER (descending walk), profiles.JOINTS (tuple order implies id 1..6). Baud is a flat setup_motors._DEFAULT_BAUDRATE=1_000_000. There is no follower/leader role concept; F1-F6/L1-L6 labels live only in calibrate-motor's catalog, never in the id/baud assignment path.
- After: An operator runs 'arm101 arm setup follower|leader'; the gated three-mode walk assigns the six servo ids 1-6 (baud left at the existing 1_000_000 default the walk already writes) AND records each motor as F1-F6 / L1-L6 with that role's servo_model + gear_ratio from arm_spec — no numbers typed. The same arm_spec is the single source consumed by calibrate, setup-motors, and profiles.

## Why it matters

- Number-free, role-aware setup removes a whole class of fumble errors (mistyped ids/bauds). A single source of truth removes silent drift between the three duplicated copies of the joint→id map.

## Requirements

- A new 'arm' noun group exposing 'arm setup <role>' (drives the existing setup-motors gated three-mode-consent walk using the role's ids+baud) and 'arm overview' (rubric: a noun with action-verbs must expose overview, and must not hard-fail on a bad path).
  - honesty: 'arm setup <role>' reuses the existing setup-motors gated walk + consent machinery (resolve_consent three-mode dry-run/TTY/--apply); it introduces no new consent code path.
- 'arm setup <role>' reuses the existing setup-motors gated three-mode walk to assign ids 1-6 (baud unchanged) AND records each motor's role-correct catalog entry (F/L label + servo_model + gear_ratio from arm_spec), so setup and cataloging happen in one number-free pass.
  - honesty: After 'arm setup leader' the catalog holds L1-L6 with the leader's mixed gears (1:191/1:345/1:147); after 'arm setup follower' it holds F1-F6 at 1:345 — without the operator typing any id, baud, model, or gear.
- A single-source arm_spec module (arm101/hardware/arm_spec.py) keyed by role -> per-joint {id, baud, servo_model, gear_ratio} — a full per-role motor map. Every value is CITED, not assumed: id (1-6) and baud (1_000_000) from lerobot so_follower.py/so_leader.py + feetech.py DEFAULT_BAUDRATE (identical across BOTH roles today); servo_model + gear_ratio from the Seeed SO-101 BOM wiki (follower uniform 1:345; leader mixed shoulder_pan&elbow 1:191/C044, lift 1:345/C001, wrist_flex/roll/gripper 1:147/C046). Imported by calibrate, setup-motors, and profiles so no duplicated joint->id literals remain; supplies the F/L identity + model/gear for calibrate-motor's catalog.
  - honesty: Every arm_spec value traces to a cited source: id+baud to lerobot so_follower/so_leader + feetech.py DEFAULT_BAUDRATE=1_000_000; servo_model+gear_ratio to the Seeed SO-101 wiki. id (1-6) and baud (1M) are identical across roles AND match the existing calibrate/setup-motors/profiles literals; only model/gear differ. No unsourced magic numbers remain.

## Honesty conditions

- A full SO-101 arm can be set up for a role by typing zero id/baud numbers — the role's ids+baud come entirely from arm_spec.
- Operators are human OR agent; the three call sites that hardcode the joint->id map today are exactly calibrate, setup-motors, and profiles — a grep confirms no fourth copy exists.
- The joint->id map literally appears in three places today (calibrate._JOINT_MOTOR, setup_motors._MOTOR_ORDER, profiles.JOINTS) and baud is a single _DEFAULT_BAUDRATE constant — verifiable by reading the code.
- After de-duplication a future joint->id change is made in exactly one place, and the role-setup path eliminates mistyped ids/bauds because no number is typed.
- None of the deferred items (XDG config, gear-corrected calibration math, power-supply handling, new runtime dep, set-motor-id/set-baudrate changes) is required to set up AND catalog a follower or leader arm number-free.
- Each success signal is independently checkable: a grep shows no duplicated joint->id literal; arm_spec's values match the Seeed/lerobot spec; 'arm overview' resolves; motors.json holds the role's F/L entries after a walk; CI gates + hardware pass.
- Both 'arm setup follower' and 'arm setup leader' complete the gated walk assigning ids 1-6 (baud at the unchanged 1M default) and recording the role's F/L catalog, with zero numbers typed.

## Success signals

- calibrate/setup-motors/profiles consume one arm_spec with no duplicated joint->id literals; follower (uniform 1:345) and leader (mixed: pan&elbow 1:191, lift 1:345, wrist_flex/roll/gripper 1:147) are both defined with correct per-joint servo_model+gear_ratio; 'arm setup <role>' assigns ids 1-6 @ 1_000_000 and catalogs the role's F/L motors with zero numbers typed; the new 'arm' noun exposes setup + overview; tests + lockstep docs (catalog + overview._VERBS + learn) + 'teken cli doctor . --strict' all green; follower path validated on hardware.

## Scope / boundaries

- Out of scope this iteration: a user-editable XDG JSON/YAML arm profile (Layer 3); gear-ratio-CORRECTED calibration math (arm_spec records the gear ratios, but calibrate's range math is not yet gear-aware); power-supply differences (leader 5V / follower 12V on the Pro edition) which are hardware, not CLI config; any new runtime third-party dependency; and changing set-motor-id / set-baudrate.

## Decisions

- command surface is a noun group 'arm setup <role>' (mirrors the existing 'cli' noun), not a flat 'setup-arm' verb.
- follower and leader share ids 1-6 @ 1_000_000 baud; they differ ONLY in per-joint servo_model + gear_ratio (follower uniform 1:345; leader mixed: shoulder_pan&elbow_flex 1:191/C044, shoulder_lift 1:345/C001, wrist_flex/wrist_roll/gripper 1:147/C046) and in power supply (Pro: leader 5V / follower 12V). arm_spec is the full per-role motor map; the role difference flows to models/gear + the F/L catalog, not to the id/baud writes.
- arm_spec keeps the FULL per-role structure (id+baud+model+gear per joint per role) as future-proofing, even though the cited lerobot source shows id+baud are presently IDENTICAL across roles. We do NOT assume divergence: identical values are recorded as identical and cited to lerobot; only model/gear differ today (cited to Seeed). The per-role schema lets a future role diverge in id/baud without a schema change.

## Sources / Provenance

Every value that lands in `arm101/hardware/arm_spec.py` must trace to one of
these. LeRobot links are pinned to commit `2f2b567` so the cited lines are
immutable.

**Motor ids, type, and the number-free setup walk** — from LeRobot
(`huggingface/lerobot`); identical for follower and leader:

- Follower motors (ids 1–6, all `sts3215`): [`so_follower.py:53–59`](https://github.com/huggingface/lerobot/blob/2f2b5679510a35aa83fdd8e9f986e134666618bc/src/lerobot/robots/so_follower/so_follower.py#L53)
- Leader motors (byte-for-byte identical dict) + the reverse `setup_motors()` walk: [`so_leader.py:45–51`](https://github.com/huggingface/lerobot/blob/2f2b5679510a35aa83fdd8e9f986e134666618bc/src/lerobot/teleoperators/so_leader/so_leader.py#L45), [`setup_motors():139`](https://github.com/huggingface/lerobot/blob/2f2b5679510a35aa83fdd8e9f986e134666618bc/src/lerobot/teleoperators/so_leader/so_leader.py#L139)
- Config carries **no** baud/gear field (only `port`, torque, target, cameras, `use_degrees`): [`config_so_follower.py`](https://github.com/huggingface/lerobot/blob/2f2b5679510a35aa83fdd8e9f986e134666618bc/src/lerobot/robots/so_follower/config_so_follower.py)

**Baud** — a Feetech-bus default, not a per-arm value:

- `DEFAULT_BAUDRATE = 1_000_000`: [`feetech.py:44`](https://github.com/huggingface/lerobot/blob/2f2b5679510a35aa83fdd8e9f986e134666618bc/src/lerobot/motors/feetech/feetech.py#L44)
- `_find_single_motor` multi-baud probe over `SCAN_BAUDRATES` — the reference for the `--from-baudrate` follow-up (arm101-cli #18): [`feetech.py:159`](https://github.com/huggingface/lerobot/blob/2f2b5679510a35aa83fdd8e9f986e134666618bc/src/lerobot/motors/feetech/feetech.py#L159)

**Per-joint servo model + gear ratio** — the only role-varying data; a physical
BOM fact **not** present in LeRobot software (which treats every joint as
`sts3215` and absorbs gearing through calibration):

- Seeed SO-101 motor configuration / BOM: <https://wiki.seeedstudio.com/lerobot_so100m_new/#configure-the-motors>
  - Follower F1–F6: `ST-3215-C001/C018/C047`, gear **1:345** (uniform)
  - Leader L1/L3 (shoulder_pan/elbow_flex): `ST-3215-C044`, **1:191**; L2 (shoulder_lift): `ST-3215-C001`, **1:345**; L4–L6 (wrist_flex/wrist_roll/gripper): `ST-3215-C046`, **1:147**
  - Power supply (Pro edition): leader **5V**, follower **12V** — hardware, out of CLI scope
