# Hardware Validation — `arm explore` (reachability mapping, plan task t11)

The live-follower validation of the `arm explore` verb shipped in **0.16.0**
(the reachability-mapping feature: flood-fill the arm's joint-space via the
overload-safe `gentle_move`, detect contacts from real load, and write a
resumable JSONL event log + a derived compact map). Companion to
[`hardware-validation-arm-read-flex.md`](hardware-validation-arm-read-flex.md).

This run **found and fixed three robustness bugs** that only surface on real
hardware — the whole point of the human-gated t11 gate.

## Prerequisites

- SO-101 **follower** on its own motor-bus supply, over USB-serial.
- `[seeed]` extra installed.
- **Port discipline:** follower on `/dev/ttyACM1`, a Reachy Mini on
  `/dev/ttyACM0`. **Always pass `--port /dev/ttyACM1`.** The engine receives an
  already-open bus, so only the `--port` given is ever touched.

## Recorded run — 2026-07-01, follower on `/dev/ttyACM1`

- **Hardware:** SO-101 follower, 6× STS3215 ids 1–6 @ 1 Mbps, ~11.9–12.1 V,
  37–42 °C throughout (never near the 70 °C limit).
- **Software:** `feat/arm-explore` @ 0.16.0, `[seeed]` present.

### Bug 1 — a transient probe comm error aborted the whole run ⚠️→fixed

The first live run flood-filled **6 clean moves**, then a gripper (id 6)
`write_torque_limit` returned `RX_TIMEOUT` (`result=-6`); `gentle_move` raised a
`CliError` and the **entire explore aborted**. A flood-fill issues hundreds of
moves, so one flaky servo read must not be fatal. **Fix:** `_safe_move` retries a
probe once, then skips it (counted in the new `ExploreResult.errors`) and keeps
exploring.

### Bug 2 — held-torque accumulation wedged the bus (the big one) ⚠️→fixed

With bug 1 handled, a 15-move run reported **`errors: 8`** — over half the probes
failing. Instrumentation showed the failures were **all on register 48
(`Torque_Limit`)** and **cascaded**: moves 1–6 succeeded, then move 7 returned
`error=2`, and from there **every** register-48 op on **every** motor timed out —
a classic serial-buffer desync. Per-motor, in isolation, register 48 read/wrote
**8/8 clean on all six motors** — so it was not hardware flakiness.

Root cause: `gentle_move` leaves each moved joint **holding torque**. As the
flood-fill advances, active servos pile up; past ~6 held motors the bus comms
cascade-fail. An explorer only needs to **probe** reachability, not hold poses.
**Fix:** each probe **limps its joint afterward**. Impact, on hardware:

| Run | max-moves | threshold | errors | result |
|-----|-----------|-----------|--------|--------|
| before fix | 15 | 500 | **8** | half the probes failed |
| after fix | 40 | 500 | **0** | 40/40 clean, 41 cells mapped |
| after fix | 120 | 500 | **0** | 120/120 clean, arm still 41 °C |

Limping between probes also keeps the arm **cool** (no sustained holding) — 120
moves left every joint at ≤ 42 °C.

### Bug 3 — an escape-triggering run left a joint holding ⚠️→fixed

The flood-fill releases each probed joint, but the **escape** search holds its
perturbations while probing. A run that hit a contact and ran an escape left
`shoulder_pan` energised (torque=1, low load) at the end. **Fix:** a final
release sweep over all joints in `explore()` — the arm is always left **limp**.
Verified: an escape-triggering run afterward read all six joints `torque=0`.

### Contact + combination-escape ✅

At `--threshold 150` (below the joints' free-motion load), a probe read
`shoulder_lift` load **152 > 150** and classified it **BLOCKED** — the contact
was recorded to the map and **fired the combination-escape** search
(`escapes_attempted: 1`), which perturbed other joints, retried, and terminated
safely (`errors: 0`). The escape did **not** succeed (`escapes_succeeded: 0`) —
correctly, since that "contact" is the joint's own gravity load, which no
perturbation can remove. This exercises the full path on hardware: read servo
load → classify blocked → record → bounded escape → terminate.

## Outcome

| t11 item | Result |
|----------|--------|
| Flood-fill + `gentle_move` on the physical arm | ✅ PASS (runs of 40 & 120 moves) |
| Bus stays healthy at scale | ✅ PASS — **0 comm errors** across 40 + 120 moves |
| JSONL log + compact map + resume + budget | ✅ PASS |
| Contact detection inside explore | ✅ PASS (`shoulder_lift` load 152) |
| Combination-escape triggered + terminates | ✅ PASS (ran on hardware, bounded) |
| Arm left safe (limp, unlatched, ≤ 42 °C) | ✅ PASS |
| Transient-comm-error resilience | ✅ PASS (retry-then-skip) |

**`arm explore` is validated on the physical follower.** Three hardware-only
robustness bugs were found, fixed, unit-tested, and re-validated live.

## Known limitations / follow-ups

- **Single global `--threshold`.** Free-motion load differs per joint (gripper
  ~140–320, `shoulder_lift` gravity ~250, lighter joints lower), so one
  threshold either misses real contacts (too high) or false-triggers on
  gravity/friction (too low). **Per-joint thresholds** are the right next step
  (echoes the flex-era finding that contact thresholds must be per-joint).
- **Escape *success* (a real combination-unblock) not physically demonstrated.**
  It requires a contrived setup where joint A is blocked until joint B moves; the
  escape *mechanism* is unit-tested and ran on hardware, but a successful unblock
  on the bench is deferred.
- **The bundled default map is still a permissive placeholder.** Producing the
  real self-collision default needs a thorough run (per-joint thresholds, enough
  budget to reach actual self-collisions), which undirected flood-fill did not
  reach in ≤120 moves. That map-generation pass is future work.

## Notes and caveats

- **Left safe:** every run ends with all six joints limp (the final release
  sweep); no EEPROM written on any path.
- **Recovery:** the register-48 desync self-cleared once the process closed and
  the bus reopened; the limp-between-probes fix prevents it recurring.

---

*Procedure and 2026-07-01 t11 run-log authored by arm101-cli (Claude).*
