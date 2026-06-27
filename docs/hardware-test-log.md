# Hardware Test Log — Consent-Aware Motor Verbs (bench / headless)

This is the **bench / single-motor** companion to
[`hardware-validation.md`](hardware-validation.md). Where that document is the
full-arm release "done" gate (all six motors, a human posing each joint), this
one covers the **three-mode consent behaviour** added in 0.7.0 (`set-motor-id`,
`center-motor`) and 0.8.0 (`setup-motors`): the non-TTY **dry-run**, the headless
**agent `--apply`** path, input validation, and the **capture → write → restore**
discipline that makes a destructive EEPROM test safe to run on a single motor on
the bench.

It serves two purposes:

1. A **repeatable runbook** — anyone can re-run it against one connected motor.
2. A **recorded run-log** — what was actually exercised, on which hardware, with
   the observed results, so the evidence outlives the chat it was run in.

## Scope

| Covered here (bench, 1 motor) | Covered in `hardware-validation.md` (full arm) |
|---|---|
| `setup-motors` dry-run (non-TTY, zero writes) | `find-port` enumeration + `--detect` |
| `setup-motors` agent `--apply` single-motor EEPROM write + audit | `setup-motors` full 6→1 interactive walk |
| `--current-id` validation (rejects before the bus opens) | `calibrate` full-arm pose capture |
| `set-motor-id` restore of the changed id | release sign-off run-log |
| read-only detection via `calibrate-motor` | |

Not yet covered by either: a live **`center-motor`** motion test, and
**`calibrate`** under the consent model (tracked in
[issue #10](https://github.com/agentculture/arm101-cli/issues/10)).

## Prerequisites

- A single Feetech STS3215 motor connected to a USB-serial adapter on a Linux
  host; the operator in the `dialout` group.
- The `[seeed]` extra installed (the Feetech SDK); without it every hardware
  verb exits 2 with an install hint:

  ```bash
  uv sync --extra seeed
  uv run python -c "import scservo_sdk; print('SDK OK')"
  ```

- **Know which port is which.** List `/dev/ttyACM*` and identify the target.
  On the reference bench, the SO-101 follower motor (F1) is on `/dev/ttyACM1`
  and a Reachy Mini is on `/dev/ttyACM0` — **only the target port is ever
  addressed; the other is left alone.** Always pass `--port` explicitly so
  auto-detection never wanders onto an unrelated device.

## Safety principles

These make a persistent EEPROM write safe to test on real hardware:

1. **Read first.** Detect the motor read-only and **record its current id**
   before any write — you need it to address the motor and to restore it.
2. **Dry-run before apply.** Confirm the plan with the zero-write path first.
3. **Capture → write → restore.** A destructive test must end with the motor
   returned to its original state, and a read-back must confirm it.
4. **Scope the port.** Always `--port <target>`; never let auto-detect touch a
   neighbouring device.
5. **Audit is evidence.** Point `ARM101_AUDIT_LOG` at a file and keep the
   `pending → success/failed` records as proof of what happened.

## Runbook — single connected motor

Replace `/dev/ttyACM1` with your target port throughout.

### 1. Detect (read-only) and record the current id

```bash
echo "" | arm101 calibrate-motor --port /dev/ttyACM1
```

Note the reported `id` (it follows `verified : Feetech STS3215 ...`). The trailing
`exit 2` is expected — `calibrate-motor` then asks an interactive "which joint?"
question that a piped stdin can't answer; the read-only detection above it is the
part you want.

### 2. Dry-run — plan only, zero writes

`setup-motors` dry-run computes the 6→1 table from the static motor order; it does
**not** open the bus.

```bash
echo "" | arm101 setup-motors --port /dev/ttyACM1 --current-id <ID> --json
```

Expect a `{"plan": [...]}` table (gripper id 6 → shoulder_pan id 1), exit 0, and
no EEPROM writes.

### 3. Validation — rejected before the bus opens

```bash
echo "" | arm101 setup-motors --port /dev/ttyACM1 --current-id 0 --apply
# error: --current-id 0 is out of range (1–253) ... ; exit 1, zero writes
```

### 4. Agent `--apply` — a real EEPROM write (destructive)

With one motor connected, `setup-motors` writes the first joint (gripper, id 6)
then correctly **fails on the second** (nothing answers at the original id any
more). That single successful write is the agent path under test.

```bash
export ARM101_AUDIT_LOG=/tmp/arm101-livetest.log
echo "" | arm101 setup-motors --port /dev/ttyACM1 --current-id <ID> --apply --json
# gripper: <ID> -> 6 written; then EXIT_ENV_ERROR (exit 2) on the next motor
cat "$ARM101_AUDIT_LOG"
```

Expect a `pending → success` audit pair for `gripper` and a `pending → failed`
pair for the next joint, each tagged `consent_mode=agent` and an `operator`.

### 5. Restore the motor to its original id

`set-motor-id` auto-detects the single connected motor (now at id 6) and writes
the id back:

```bash
echo "" | arm101 set-motor-id <ID> --port /dev/ttyACM1 --apply --json
```

### 6. Verify the restore

```bash
echo "" | arm101 calibrate-motor --port /dev/ttyACM1 2>&1 | grep -E "verified|id "
```

Confirm the motor reads back at its original id, same baud index, torque OFF.
**Net change to the motor = none.**

## Recorded runs

### 2026-06-27 — F1 (Follower 1), `/dev/ttyACM1`

- **Hardware:** Feetech STS3215, model 777, firmware 3.10; started at **id 1**,
  baud index 0 (1 Mbps), torque OFF, 11.9 V, ~37 °C, position 3939.
- **Software:** branch `feat/setup-motors-consent` (0.8.0), `[seeed]` SDK
  present; Reachy Mini on `/dev/ttyACM0` left untouched.

| Step | Verb / mode | Command | Observed result | Pass |
|------|-------------|---------|-----------------|------|
| 1 | `calibrate-motor` (read-only) | `calibrate-motor --port /dev/ttyACM1` | F1 detected at id 1, STS3215 verified | ✅ |
| 2 | `setup-motors` dry-run | `setup-motors --port /dev/ttyACM1 --current-id 1 [--json]` | 6→1 table in text + JSON; exit 0; no bus opened | ✅ |
| 3 | `setup-motors` validation | `setup-motors --current-id 0 --apply` | `EXIT_USER_ERROR` (exit 1) before bus open; zero writes | ✅ |
| 4 | `setup-motors` agent `--apply` | `setup-motors --port /dev/ttyACM1 --current-id 1 --apply` | F1 id **1 → 6** written; `EXIT_ENV_ERROR` (exit 2) on motor 2 as expected | ✅ |
| 5 | `set-motor-id` restore | `set-motor-id 1 --port /dev/ttyACM1 --apply` | detected F1 at id 6, wrote **6 → 1**; exit 0 | ✅ |
| 6 | `calibrate-motor` verify | `calibrate-motor --port /dev/ttyACM1` | F1 back at **id 1**, baud index 0, torque OFF | ✅ |

**Audit trail** (`consent_mode` and `operator` on every record):

```text
setup-motors  gripper     1->6  mode=agent  op=arm101-cli  pending
setup-motors  gripper     1->6  mode=agent  op=arm101-cli  success      <- real EEPROM write
setup-motors  wrist_roll  1->5  mode=agent  op=arm101-cli  pending
setup-motors  wrist_roll  1->5  mode=agent  op=arm101-cli  failed  ERR=Write Baud_Rate failed for motor 1: result=-6, error=0
set-motor-id              6->1  mode=agent  op=arm101-cli  success      <- restore
```

**Outcome: PASS.** The dry-run, agent `--apply` write + audit (both success and
failure outcomes), the `--current-id` guard, operator attribution (resolved from
`culture.yaml`), and baud preservation all behaved correctly on real STS3215
hardware. F1 was returned to id 1 — net change zero. Validated as part of PR #9.

## Notes and caveats

- **`setup-motors` always runs the full 6→1 walk.** On a single connected motor
  it writes the first joint then fails on the next (the addressed id is now
  empty) — exit 2 is the *correct* single-motor outcome, not a defect. The clean
  end-to-end success path needs all six motors, connected one at a time as
  prompted (see `hardware-validation.md` §2).
- **Headless pacing is a known follow-up.** The agent walk does not block between
  writes, so a real multi-motor headless run needs the motors pre-connected (a
  USB hub) or paced via per-motor `set-motor-id --apply`. Tracked as the
  `unknown_nonblocking` risk `r1` on the 0.8.0 plan.
- **`calibrate` is not consent-aware yet** (it still uses bare `input()` and is
  TTY-only). Bench/headless calibration testing is blocked until
  [issue #10](https://github.com/agentculture/arm101-cli/issues/10) lands.

---

*Procedure and 2026-06-27 run-log authored by arm101-cli (Claude); future
run-log entries to be appended by the operator.*
