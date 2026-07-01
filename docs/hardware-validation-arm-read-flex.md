# Hardware Validation — `arm read` / `arm flex` / `doctor --probe` (issue #20, plan task t9)

The full-arm live companion to [`hardware-test-log.md`](hardware-test-log.md)
(bench, single motor) and [`hardware-validation.md`](hardware-validation.md)
(full-arm setup gate). This one is the **motion / read** run-log for the
`arm read`, `arm flex` (per-joint gentle + coordinated), and `doctor --probe`
verbs shipped in **0.14.0** (issue
[#20](https://github.com/agentculture/arm101-cli/issues/20), PR #22) — plan
task **t9**, the live-follower validation the PR left to the operator.

It doubles as the **before-state evidence** for the
[overload-safe-motion spec](specs/2026-07-01-arm101-arm-motion-is-overload-safe-it-never-trips.md):
the two overload reproductions below are the recorded facts its honesty
conditions (`h6` especially) refer to.

## Prerequisites

- SO-101 **follower** powered from its own motor-bus supply (USB does **not**
  power the motors — Seeed's wiki is explicit) and connected over USB-serial.
- The `[seeed]` extra installed (`uv pip install -e '.[seeed]'`); every hardware
  verb exits 2 with an install hint otherwise.
- **Port discipline:** on the reference bench the follower is on `/dev/ttyACM1`
  and a Reachy Mini is on `/dev/ttyACM0`. **Always pass `--port /dev/ttyACM1`
  explicitly** — auto-detect picks the first candidate and could address the
  Reachy. Only the follower port is ever touched.

## STS3215 overload primer (why the motion section matters)

Each servo self-protects: if output torque exceeds `Overload_Torque` (addr 36,
**80 %**) for longer than `Protection_Time` (addr 35, **200**), it cuts torque
to `Protective_Torque` (addr 34, **20 %**) and **latches status error byte
`0x20` (= 32)**. While latched, *every* packet response carries `error=32`, so
`read_info` raises and the wrapper's `enable_torque`/reads fail — comms are fine
(`result=0`), the servo is just flagging a fault. The latch **clears the moment
torque is disabled** (raw `write1ByteTxRx(id, 40, 0)`), no power-cycle needed.

`present_load` (addr 60) encodes **direction in bit 10 (1024)**; a reading of
`1064` is direction-negative magnitude **40**, not "1064" — static holding load
is low. The overload is a **dynamic** current spike, not a static-torque ceiling.

Torque caps on this follower are factory default: `Max_Torque_Limit` (addr 16)
and RAM `Torque_Limit` (addr 48) both **1000** (full torque); `Max_Temp`
(addr 13) **70 °C**.

## Recorded run — 2026-07-01, follower on `/dev/ttyACM1`

- **Hardware:** SO-101 follower, 6× STS3215 at persistent ids 1–6, baud 1 Mbps,
  ~11.9–12.0 V, 34–38 °C at rest. Reachy Mini on `/dev/ttyACM0` untouched.
- **Software:** `main` @ **0.14.0**, `[seeed]` SDK present.

### t9.1 — `arm read` (read-only) ✅

```bash
arm101 arm read --role follower --port /dev/ttyACM1 --json
```

All six joints returned `health: ok`, ids 1–6, live position / load / voltage /
temperature; `complete: true`. Torque 0 (limp) as expected — read commands no
motion. **PASS.**

### t9.3 — `doctor --probe` (read-only multi-baud sweep) ✅

```bash
arm101 doctor --probe --port /dev/ttyACM1
```

All six ids classified `SUCCESS@1000000`; nothing spurious at other bauds.
Separately, **during the id2 overload below the probe correctly reported
`id 2: CORRUPT@1000000`** — a present-but-incoherent servo, exactly the
diagnosis the multi-baud probe (issue #18) exists to give. **PASS.**

### t9.2a — `arm flex` per-joint gentle, one at a time ✅

Six sequential `arm flex <joint> --to <±120 ticks> --gentle --threshold 600
--apply --json` moves (base→tip). Every joint reached its target,
`contacted: false`, no false contact. `--threshold 600` chosen so a joint
holding its own weight is not misread as contact. **PASS.**

### t9.2b — coordinated "wake-up" flex, all six at once ✅

The shipped CLI moves one joint per invocation, so true simultaneity was driven
via raw `scservo_sdk.GroupSyncWrite` on the open bus's packet/port handlers:
one sync-write packet per **interpolated frame** (24 frames each way),
**goal-speed 150**, gentle accel, ~±150-tick deltas (shoulder_lift kept to
−70), with a live `ping`-based overload watch on id2. All six joints moved out
and back together; **zero overload**; every joint returned within ~2–5 ticks of
start. This is the proven mitigation the spec's `h2` builds on. **PASS.**

### Overload reproduction A — shoulder_lift (id2), one-shot @ speed 400 ⚠️→recovered

The *first* wake-up attempt wrote each joint's goal in **one shot at goal-speed
400** (not interpolated). id2 — driving the arm's mass — spiked current and
latched `error=32` on the write (`result=-7` corrupt, then `error=32` on reads;
probe showed `id 2: CORRUPT`). Recovered instantly with a raw torque-disable
(`write1ByteTxRx(2, 40, 0)`); full-arm read clean afterward.

### Overload reproduction B — gripper (id6), rigid stop @ speed 400 ⚠️→recovered

`arm flex gripper --to 3400 --gentle --threshold 250 --apply` against a
**rigidly held** jaw: at the gentle default speed 400 the first push spiked
current and latched `error=32` **before** `gentle_move` could read
`present_load > threshold` and back off. The read raised, so `arm flex` exited 2
with a raw env error — *not* a graceful retreat. Recovered with the same raw
torque-disable; gripper reached 55 °C (well under the 70 °C limit). This is the
**graceful-handling gap** the spec's `c10`/`h4`/`h7` fix.

## Outcome

| t9 item | Result |
|---------|--------|
| t9.1 `arm read` | ✅ PASS |
| t9.3 `doctor --probe` | ✅ PASS (incl. live `CORRUPT` fault diagnosis) |
| t9.2 `arm flex` per-joint gentle | ✅ PASS (all 6) |
| t9.2 coordinated wake-up | ✅ PASS (GroupSyncWrite, interpolated, speed 150) |
| t9.2 contact back-off (soft compliant) | ⏳ not yet shown — rigid-stop tripped hardware overload first |

**Issue #20 read / probe / per-joint gentle motion are validated on the physical
follower.** The contact back-off's *graceful* behavior against a hard stop, and
trip-free coordinated motion by default, are deferred to the
[overload-safe-motion](specs/2026-07-01-arm101-arm-motion-is-overload-safe-it-never-trips.md)
work — this run is that spec's before-state evidence.

## Notes and caveats

- **Recovery recipe:** any latched `error=32` clears with a raw
  `write1ByteTxRx(<id>, 40, 0)` (torque off). No power-cycle required.
- **Speed is the lever:** speed 400 tripped both joints; interpolated speed 150
  did not. Small per-frame deltas keep torque under the 80 % window.
- **Left safe:** at end of run, five joints holding at low load, gripper torque
  released and cooling. No EEPROM was written on any motion path.

---

*Procedure and 2026-07-01 t9 run-log authored by arm101-cli (Claude).*
