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

### After-fix t9 re-run — 2026-07-01, plan task t7 (PASS)

Run on the physical follower (`/dev/ttyACM1`) against the overload-safe-motion
fix (`gentle_move` default speed 150, `_CONTACT_TORQUE_LIMIT=500` cap, graceful
`error=32` handling). Torque_Limit baseline read `1000` on all six joints first.

**Test B — gripper into a rigidly-held jaw** (the move that raw-crashed at t9,
above): `arm flex gripper --to 3600 --gentle --threshold 250 --apply` →
`contacted: true` (stopped at 3202, retreated to 3152, held), **`overloaded:
false`, exit 0** — the torque cap kept output below the 80 % trip so the
software back-off engaged first. `Torque_Limit` read back **1000** after; the
joint read fine with **no manual torque-disable** needed. Satisfies **h3, h7,
h8** and the rigid-stop half of **h1**.

**Test A — whole-arm `arm flex --demo`** (the shipped "wake-up" path):
`overloaded: false` / `aborted_on_overload: false`, **exit 0 — zero `error=32`
trips** (**h2, h1**). The first run also exposed a *contact-detection* bug
(unrelated to overload): STS3215 `present_load` carries load direction in bit 10
(`0x400`), and `gentle_move` compared the **raw** value, so a load in the
negative direction (raw ≥ 1024) tripped a spurious "contact" on the first step —
the sweep aborted on `shoulder_pan`'s up-move. Fixed in the same PR by masking
`& 0x3FF` (magnitude) before the threshold compare (new `bus.load_magnitude()`
helper + regression tests). **After the fix**, the demo re-run swept
`shoulder_pan` through its **full** sub-range (`contacted: false`, reached 4095)
and moved `shoulder_lift` a real ~475 ticks before a genuine magnitude-based
contact — still `overloaded: false`, exit 0.

**shoulder_lift direct gentle move** (the joint that overloaded at t9):
`overloaded: false`, exit 0, `Torque_Limit` 1000, not latched.

**Outcome: t7 PASS.** The exact dynamic move that latched `error=32` and crashed
the CLI at t9 now stops / stalls / recovers with no hardware overload, no raw
traceback, torque-limit restored, and no manual recovery. Honesty conditions
**h1, h2, h3, h7, h8** verified on hardware (h7's graceful `overloaded=true`
path stays unit-verified — the cap *prevented* the overload on hardware, a
superior outcome). This section is **h10**.

### Post-merge full-flex re-run — 2026-07-01, merged `main` @ v0.15.0 (PASS)

The t7 section above was recorded on the `feat/overload-safe-motion-build`
branch. This run re-confirms the whole read → probe → per-joint → coordinated
surface on **merged `main` @ 0.15.0** (PR #24), follower on `/dev/ttyACM1`,
Reachy Mini on `/dev/ttyACM0` untouched (`--port /dev/ttyACM1` passed on every
command). Rest state before: 6 joints `health: ok`, ids 1–6, ~11.9–12.1 V,
38–42 °C, all `torque: 0`.

| Step | Command | Result |
|------|---------|--------|
| `arm read` (before) | `arm read --role follower --port /dev/ttyACM1 --json` | ✅ 6 joints `ok`, unlatched |
| `doctor --probe` | `doctor --probe --port /dev/ttyACM1` | ✅ 6× `SUCCESS@1000000` |
| per-joint gentle × 6 | `arm flex <joint> --to <±120> --gentle --threshold 600 --apply` | ✅ each reached target, `contacted: false`, `overloaded: false` |
| coordinated demo | `arm flex --demo --role follower --port /dev/ttyACM1 --apply` | ✅ exit **0**, `overloaded: false`, `aborted_on_overload: false` |
| `arm read` (after) | same as before | ✅ all `ok`, `overloaded: false`, `torque: 0` (limp), temps ≤ 43 °C |

Per-joint gentle covered all six joints including `shoulder_lift` (id 2) — the
exact joint that latched `error=32` at t9 — which reached 3303 clean with no
trip. In the demo sweep, `shoulder_pan` swept its full sub-range to 4095
(`contacted: false`), and `shoulder_lift`'s down-move made a **genuine gravity
contact** (`present_load` magnitude **252** > the demo's threshold 250) and
**backed off 50 ticks and held** (`contacted: true`, `aborted_on_contact: true`)
— a graceful software back-off, **not** an overload (`overloaded: false`). Zero
`error=32` across the entire run, no raw tracebacks, no manual torque-disable.
The load-magnitude mask (`& 0x3FF`) is re-confirmed: 252 is a true magnitude, not
the direction-bit artifact that caused the spurious step-1 contact before the
t7 fix. Arm left limp and safe (all joints `torque: 0`, nothing latched).

### Regression baseline — 2026-07-12, plan task t3 (the load-watch does NOT work)

**Purpose.** The before-picture for the `gentle_move` measurement fix. Drives a
joint into an obstacle **mid-travel on the pre-fix code** and records what
`gentle_move` reports. This run is unreproducible once the fix lands, so it is
captured first, deliberately.

**Setup.** Follower on `/dev/ttyACM1`, base clamped. A soft, compliant object
placed between the gripper fingers. Gripper (id 6), threshold **380** (its own
`DEFAULT_CONTACT_THRESHOLD`). `Torque_Limit` pre-set to 600 so the joint can
travel (gripper gear-friction alone runs ~320) and so a contact above 380 is
physically expressible. Commanded travel: **-690 ticks** (decreasing ticks
closes the gripper — established by this run).

**What `gentle_move` reported:**

```text
gentle_move RETURNED after 129 ms
  contacted       = False
  contact_load    = None
  final_position  = 2396   (claimed)
  overloaded      = False
```

**What the arm actually did** (polled every ~25 ms *after* the call returned):

```text
  t (ms)  position   load
     133      3076     60      <- gentle_move has already returned
     693      3022    320
     809      3014    388      <- load CROSSES the 380 threshold here
     954      3011    548
    1073      3001    600      <- stalled against the object, load saturated
    1512      3001    600
```

**Result — baseline confirmed.** The gripper travelled only **85 of the 690**
commanded ticks and stalled against the object. Its load crossed the configured
380 contact threshold at **t ≈ 809 ms — some 680 ms after `gentle_move` had
already returned reporting `contacted=False`** — and went on to 600. The contact
was real, it exceeded the threshold by a wide margin, and the code detected
nothing, because it had stopped watching two-thirds of a second earlier.
`final_position` was reported as 2396; the joint was at 3001.

This is the failure the fix must invert: after the fix, this same run must stop
and hold **on the contact itself** (plan task t8).

**New constraint discovered — `present_load` saturates at `Torque_Limit`.** A
first attempt at this run used `Torque_Limit = 300`; the load climbed and then
pinned at exactly 300, never reaching the 380 threshold, and the gripper stalled
because 300 sits *below* its own ~320 gear-friction. Both effects are real and
load-bearing:

- **A contact threshold at or above the active `Torque_Limit` can never fire.**
  `gentle_move` caps `Torque_Limit` to `_CONTACT_TORQUE_LIMIT = 500` for the
  duration of a move, so **every per-joint threshold must sit strictly below
  500** or it is undetectable by construction.
- Combined with the free-motion peaks measured during travel (wrist_roll alone
  peaks at ~272), the usable band for each joint's threshold is
  `(free-motion peak, 500)`. Plan task **t7** must derive the re-tuned
  `DEFAULT_CONTACT_THRESHOLDS` inside that band and confirm it is non-empty for
  every joint — this is precisely the risk parked as frame `q1` / plan `r1`.

**Left safe:** torque released and the original `Torque_Limit` restored in a
`finally` on both runs; no EEPROM written; no `error=32` latched.

### Acceptance — 2026-07-12, plan tasks t7 + t8 (PASS, both halves)

The fix for the measurement bug recorded in the t3 baseline above. Same arm,
same soft object in the gripper.

**Half 1 — free-space move returns only after MEASURED arrival.**

```text
wrist_roll 3551 -> 3151
  returned after   : 2755 ms      (pre-fix: 71 ms)
  final_position   : 3153         (claimed)
  actually at      : 3153         (read back now)
  contacted        : False
```

The call now takes the servo's real travel time, and the position it reports is
one it read off the servo rather than the target it was told to aim for.

**Half 2 — stop-and-hold on a contact the move ITSELF caused.**

```text
gripper 3010 -> 2320 (closing onto the soft object)
  returned after   : 1995 ms      (pre-fix: 129 ms)
  contacted        : True         (pre-fix: False)
  contact_load     : 500          (pre-fix: None)
  contact_position : 2956
  final_position   : 3005         (backed off 50 ticks)
  holding at       : 3005, load 0 (pressure relieved)
```

This is the exact move the t3 baseline recorded going **completely undetected**.
It is now caught during the approach, stopped, backed off, and held.

### The goal tether — tried, measured, and REMOVED

Worth recording, because it looks like a safety improvement and is not.

The first version of the fix tethered the goal to the MEASURED position (never
more than `step` = 25 ticks ahead), to bound how hard the servo could push.
Measured consequence: it pins the servo's position error at 25 ticks, which caps
its torque, giving a stalled load of **~208 on every joint** — a constant of the
design, not a property of the joint. And gravity-loaded joints then cannot break
away at all:

```text
joint           FREE-space result        blocked load
shoulder_pan    arrived      peak 124    212
shoulder_lift   STALLED@3381 peak 188    208     <- stalled in OPEN SPACE
elbow_flex      STALLED@2301 peak 156    -       <- stalled in OPEN SPACE
wrist_flex      arrived      peak  56    208
wrist_roll      arrived      peak  80    -
gripper         arrived      peak  76    208
```

`shoulder_lift` and `elbow_flex` stall in **open space** — the "under-torqued
joint stalls in free space and looks exactly like a contact" failure. It also
crushed `shoulder_lift`'s usable band to (188, 208), 20 units wide.

Pressing force never needed the tether: it is already bounded by the
`_CONTACT_TORQUE_LIMIT` (500) cap held for the duration of a move. That is the
hardware-proven safety, and it is what `present_load` saturates against on a
real contact.

### Free-motion load profile (untethered — the shipped dynamics)

The floor of each joint's contact-threshold band: the peak load it develops
merely ACCELERATING through open space. Ceiling is 500 (saturation at the cap).

| joint | free-motion peak | band | threshold (old → new) |
|-------|------------------|------|------------------------|
| shoulder_pan | 88 | (88, 500) | 200 → **250** |
| shoulder_lift | 92 | (92, 500) | 350 → **250** |
| elbow_flex | 148 | (148, 500) | 220 → **280** |
| wrist_flex | 96 | (96, 500) | 200 → **250** |
| wrist_roll | **300** | (300, 500) | 180 → **400** |
| gripper | 76 | (76, 500) | 380 → **250** |

Every joint has a usable band, so risk `q1` / `r1` does **not** fire.
`wrist_roll`'s old threshold of 180 sat **below its own 300 free-motion peak** —
correctly sampled, it would have called contact on every move it ever made.

### Boundaries discovered

Real hard stops, each confirmed by a saturated load with the joint refusing to
advance (and `wrist_flex` re-confirmed against a raised 900 torque cap, so it is
physical, not under-torque):

| joint | boundary | direction |
|-------|----------|-----------|
| shoulder_pan | ~3430 | decreasing |
| shoulder_lift | ~3311 | decreasing |
| wrist_flex | ~2223 | increasing |
| gripper | ~2945 (bare) / ~2975 (soft object) | closing |

## Notes and caveats

- **Recovery recipe:** any latched `error=32` clears with a raw
  `write1ByteTxRx(<id>, 40, 0)` (torque off). No power-cycle required.
- **Motion-onset latency is ~95-127 ms.** The servo does not begin moving for
  ~100 ms after a goal write. A stall detector live during that dead window
  reports a phantom contact on EVERY move — arm it on measured motion, never on
  a fixed timer. Retreating FROM a contact, onset stretches to ~1.0-1.2 s.
- **Travel is slow and very uneven:** 500 ticks takes ~930 ms on `wrist_roll`
  but ~3300 ms on the shoulder joints. Size probe timeouts from the slow end.
- **Thresholds live under the torque cap:** `present_load` saturates at
  `Torque_Limit`, which `gentle_move` caps to 500 during a move — so a contact
  threshold ≥ 500 can never trigger, whatever the sampling does (t3, 2026-07-12).
- **Speed is the lever:** speed 400 tripped both joints; interpolated speed 150
  did not. Small per-frame deltas keep torque under the 80 % window.
- **Left safe:** at end of run, five joints holding at low load, gripper torque
  released and cooling. No EEPROM was written on any motion path.

---

*Procedure and 2026-07-01 t9 run-log authored by arm101-cli (Claude).*
