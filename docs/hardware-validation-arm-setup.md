# Hardware Validation Run-Log: `arm setup` (role-aware) — SO-101

This run-log validates the **`arm setup <role>`** verb (issue #17) against
physical hardware. It is the human-in-the-loop gate for the role-aware setup
feature: the FakeBus test suite proves the logic, and this log proves the real
EEPROM/catalog path on an actual arm.

> **Status: TEMPLATE — awaiting a hardware run + operator sign-off.**
> Fill the run-log table and the sign-off line after exercising the commands
> below on a physical SO-101 **follower** arm. The **leader** path is
> FakeBus-proven only until a physical leader arm is on hand (see the note at
> the end).

## What `arm setup <role>` does

`arm setup follower|leader` drives the existing gated `setup-motors` walk
(6→1, per-motor re-detection, three-mode consent) using ids + baud from
`arm101/hardware/arm_spec.py`, and **additionally records each motor into the
motor catalog** (`motors.json`) with its role-correct label (`F1`–`F6` /
`L1`–`L6`) plus the `servo_model` and `gear_ratio` from `arm_spec` — so a full
arm is set up **and catalogued** with **zero numbers typed**. Both roles share
ids 1–6 @ 1 000 000; the role difference is the per-joint model/gear (follower
uniform `1:345`; leader mixed `1:191 / 1:345 / 1:147`).

## Prerequisites

- A physical SO-101 **follower** arm (6× Feetech STS3215), USB-to-serial
  adapter connected; note its `/dev/ttyACM*` (or `/dev/ttyUSB*`) port.
- User in the `dialout` group (or equivalent) to reach `/dev/ttyACM*` without
  `sudo` — else `sudo chmod 666 /dev/ttyACM*`.
- The `[seeed]` extra installed so the real Feetech SDK is present
  (`uv sync --extra seeed`), otherwise the bus cannot open.
- ⚠️ EEPROM writes are persistent. Connect **one motor at a time** as prompted
  by the walk (gripper first / id 6 → shoulder_pan / id 1).

## Procedure

### Step 1 — Dry-run (number-free, zero writes)

Non-interactive, no `--apply` → prints the assignment plan and writes nothing:

```bash
arm101 arm setup follower            # markdown plan to stdout
arm101 arm setup follower --json     # same plan as JSON
```

**Expected:** an F1–F6 table — ids 1–6, baud 1000000, servo_model
`ST-3215-C001/C018/C047`, gear `1:345` for every joint — and **no** bus opened,
**no** catalog entry written.

### Step 2 — Apply (TTY interactive, or `--apply` headless)

```bash
# Interactive (a TTY): walks 6→1, prompts to connect each motor, typed 'yes' to confirm
arm101 arm setup follower

# Headless agent mode (non-TTY): executes the walk without prompts
arm101 arm setup follower --apply
```

**Expected:** each motor is detected, assigned its id @ 1 000 000, a BEFORE/AFTER
card shown, and an `F{n}` catalog entry written. The operator types **no** id,
baud, model, or gear.

### Step 3 — Verify the catalog

```bash
# motors.json under $XDG_CONFIG_HOME/arm101 (or ~/.config/arm101)
cat "${XDG_CONFIG_HOME:-$HOME/.config}/arm101/motors.json"
```

**Expected:** entries `F1`–`F6` present, each with the correct `joint`,
`servo_model` `ST-3215-C001/C018/C047`, `gear_ratio` `1:345`, and the
`detected_id` read back from hardware.

## Run-Log Table

| Date | Operator | Arm | Serial Port | Step | Command | Observed Result | Pass/Fail |
|------|----------|-----|-------------|------|---------|-----------------|-----------|
|      |          | follower |        | 1 dry-run | `arm setup follower` | | |
|      |          | follower |        | 2 apply   | `arm setup follower [--apply]` | | |
|      |          | follower |        | 3 catalog | inspect `motors.json` | | |

### Extended notes

```text
Operator name/contact:
Date / time:
Software: branch feat/role-aware-arm-setup (version), [seeed] extra present? Y/N
Hardware: STS3215 firmware, starting ids/baud, voltage, temp:
Anything unexpected:
```

## "Done" gate statement

> On __________ (date), operator __________ ran `arm101 arm setup follower`
> end-to-end against a physical SO-101 follower arm: the dry-run produced the
> F1–F6 plan with zero writes, `--apply` assigned ids 1–6 @ 1 000 000, and the
> catalog (`motors.json`) recorded F1–F6 with gear `1:345` — **with no id, baud,
> model, or gear typed**. Result: ____ (PASS / FAIL).

## Leader note

The **leader** role (`arm setup leader`) is exercised by the FakeBus test suite
(`tests/test_arm.py`) — which asserts the catalog holds `L1`–`L6` with the mixed
leader gears (`1:191 / 1:345 / 1:147`) — but has **not** been validated on
physical leader hardware. When a physical SO-101 leader arm is available, add a
`leader` row to the run-log above and a matching sign-off. (Tracked as plan risk
`r1`.)
