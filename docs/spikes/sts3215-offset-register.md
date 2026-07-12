# STS3215 offset register — spike (issue #35, task t5)

**VERDICT: GO-WITH-CAVEAT.**

The mechanism exists and is exactly what we hoped for: a **2-byte, EEPROM-backed,
sign-magnitude offset register at address 31** (`Ofs` / `Homing_Offset`), applied by
the servo to the position it *reports*. Its ±2047 range is roughly **twice** what
`elbow_flex` needs (we need ≈1073). Feetech's own SDK, Feetech's own Python library,
and LeRobot — which drives this exact arm — all agree on the facts below.

**But one load-bearing assumption is undocumented and must be proven on hardware
before t7 writes anything:** that the corrected `Present_Position` is reduced
**modulo 4096** (so the seam *relocates* to `raw == Ofs`), rather than reported as a
plain signed subtraction (which would leave the discontinuity pinned at the physical
raw 4095→0 point and make the re-zero **useless**). Every source is consistent with
the modular reading and LeRobot's shipped SO-101 calibration silently depends on it,
but no primary source states the firmware's formula. See
[The caveat](#4-the-caveat--what-must-be-proven-on-hardware) and
[the hardware test](#5-the-hardware-persistence--seam-eviction-test).

This is a spike, not a build. Nothing was written to any servo; no Python file was
touched.

---

## 1. Does an offset mechanism exist?

Yes — and there are **two** of them.

`arm101/hardware/bus.py` implements neither. Confirmed by reading it: the only
addresses it knows are 0, 1, 3 (read-only ident), 5 (ID), 6 (Baud_Rate), 9/11
(Min/Max_Angle_Limit), 40 (Torque_Enable), 41 (Acceleration), 42 (Goal_Position),
46 (Goal_Speed), 48 (Torque_Limit), 55 (Lock), 56 (Present_Position), 58/60/62/63
(present speed/load/voltage/temperature). **Address 31 appears nowhere in the
codebase.**

### Mechanism A — write the offset register directly (address 31)

The one t7 should build on: it gives arbitrary, exact control over where the seam
lands.

### Mechanism B — firmware midpoint calibration (write `128` to address 40)

Feetech's SDK exposes `CalibrationOfs(id)`, which is literally one byte:

```cpp
int SMS_STS::CalibrationOfs(u8 ID)
{
    return writeByte(ID, SMS_STS_TORQUE_ENABLE, SMS_STS_CALIBRATION_CMD);
}
// #define SMS_STS_TORQUE_ENABLE 40
// #define SMS_STS_CALIBRATION_CMD 128   // Command value for midpoint calibration
```

The firmware reads the current physical position, computes the offset that makes it
the centre, and commits that to EEPROM itself. Feetech's own example documents the
procedure: *"Physically position servo shaft to desired center/zero point → run this
program → CalibrationOfs() reads current position and calculates offset → Offset is
written to EEPROM (persists across power cycles) → All future position commands are
relative to this new center."*

Mechanism B is a genuinely attractive **fallback**, because it sidesteps a
chicken-and-egg problem: it needs no wrap-aware move and no arithmetic — a human
parks the joint by hand (torque off) at mid-travel and sends one byte. Its downsides
are that it always centres on 2048 (no control over the exact offset) and that
whether its firmware-internal EEPROM commit honours the Lock register is **untested**.

Sources: [`SMS_STS.h`][smsh] (Feetech memory table + `SMS_STS_CALIBRATION_CMD`),
[`SMS_STS.cpp`][smscpp] (`CalibrationOfs` body),
[`CalibrationOfs.cpp` example][smsex].

---

## 2. Register facts (every fact cited)

| Fact | Value | Source |
| --- | --- | --- |
| Name | `Ofs` (Feetech) / `Homing_Offset` (LeRobot) | [`SMS_STS.h`][smsh], [LeRobot `tables.py`][lrtables] |
| Address | **31** (`OFS_L`), 32 (`OFS_H`) — one 2-byte register at 31 | `#define SMS_STS_OFS_L 31` / `SMS_STS_OFS_H 32` ([`SMS_STS.h`][smsh]); `SMS_STS_OFS_L = 31` ([Feetech `sms_sts.py`][ftpy]); `"Homing_Offset": (31, 2)` ([LeRobot][lrtables]) |
| Width | **2 bytes** | as above (L/H pair; LeRobot's `(31, 2)`) |
| Encoding | **SIGN-MAGNITUDE, sign bit = bit 11.** Wire value = `(sign << 11) \| magnitude`. **NOT two's complement.** | `"Homing_Offset": 11` in `STS_SMS_SERIES_ENCODINGS_TABLE` ([LeRobot][lrtables]) + `encode_sign_magnitude` ([LeRobot][lrenc]) |
| Usable range | **−2047 … +2047** | `max_magnitude = (1 << sign_bit_index) - 1` = `(1 << 11) - 1` = **2047** ([LeRobot][lrenc]); confirmed empirically by a real SO-101 failure: `ValueError: Magnitude 2073 exceeds 2047 (max for sign_bit_index=11)` ([LeRobot issue #3193][lr3193]) |
| **EEPROM or RAM** | **EEPROM — persistent.** It sits under the `//-------EEPROM (Read and Write)--------` heading; the `//-------SRAM (Read and Write)--------` region does not begin until address 40. | [`SMS_STS.h`][smsh] |
| Lock (addr 55) | **Unlock → write → re-lock required.** Same EEPROM region as ID (5) and Baud_Rate (6) — the exact registers PR #21 proved revert on power-cycle when Lock is left closed. | [`SMS_STS.h`][smsh]; LeRobot's `disable_torque` writes `Torque_Enable=0` **and `Lock=0`**, `enable_torque` writes `Torque_Enable=1` **and `Lock=1`**, and `write_calibration` (which writes `Homing_Offset`) runs between them ([LeRobot `feetech.py`][lrfeetech]) |
| **Effect on `Present_Position` (addr 56)** | **YES — the servo applies it to what it reports.** `Present_Position = Actual_Position − Homing_Offset` | LeRobot `_get_half_turn_homings` docstring ([LeRobot `feetech.py`][lrfeetech]); independently, an unrelated STS3215 spec memo: *"補正後の現在位置 = 生のエンコーダ値 − Homing_Offset"* (corrected present position = raw encoder value − Homing_Offset) ([Zenn][zenn]) |

Two further facts worth carrying into t7:

- **`Present_Position` is itself sign-magnitude on bit 15.** Feetech's `ReadPos`
  decodes it as `scs_tohost(value, 15)` ([Feetech `sms_sts.py`][ftpy]); LeRobot lists
  `"Present_Position": 15` in its encodings table ([LeRobot][lrtables]). Our
  `bus.py:read_position` currently does `int(value) & 0x0FFF`, which discards that
  sign bit. In servo mode with limits 0/4095 that mask is harmless, but it is exactly
  the mask that would *hide* a negative reading — see the caveat.
- **The SDK this repo already depends on ships the codec but no register table.**
  `scservo_sdk` (`feetech-servo-sdk` 1.0.0) contains `SCS_TOHOST(a, b)` /
  `SCS_TOSCS(a, b)` in `scservo_def.py` — a sign-magnitude codec where bit `b` is the
  sign — but defines **no control-table constants at all**. So t7 must supply address
  31 itself; there is nothing to import.

---

## 3. Range arithmetic for `elbow_flex`, step by step

**Measured facts** (follower on `/dev/ttyACM1`, 2026-07-12 — from
`arm-explore-follower.map.json` and issue #35):

- Hard wall (driving *decreasing*) at raw **2020**.
- Recorded contiguous, wrap-free region: **[2040, 4060]** (span 2020 ticks — this is
  the "~2020-tick span", and it is a *lower bound* on true travel: the map note says
  it deliberately stops short of the wrap).
- Driven *increasing*, it crossed the 4095→0 seam and read back as **~1**.
- It currently **rests at raw ~126** — *past* its wrap (issue #35: *"elbow_flex is
  currently resting at ~126, i.e. past its wrap. A linear command will rotate it the
  long way round."*).

**Step 1 — reconstruct the true travel arc in raw ticks.** Going up from the wall:

```text
2020  →  4095  →│seam│→  0  →  ~126
```

**Step 2 — arc length.**

```text
travel  =  (4096 − 2020)  +  126
        =       2076      +  126
        =  2202 ticks          (a LOWER BOUND — the far wall was never measured)
```

**Step 3 — the unreachable arc**, which is where we must evict the seam to:

```text
unreachable  =  4096 − 2202  =  1894 ticks,  spanning raw (126, 2020)
```

**Step 4 — where the seam lands.** With `Present = (raw − H) mod 4096`, the reported
value wraps 4095→0 exactly where `raw == H`. So we need:

```text
H  ∈  (126, 2020)
```

**Step 5 — pick the midpoint for maximum margin.**

```text
H*  =  (126 + 2020) / 2  =  1073
```

**Step 6 — does it fit in ±2047?**

```text
|H*|  =  1073   ≤   2047   ✓        head-room = 2047 − 1073 = 974 ticks
```

**Yes — with ~2× margin.** The answer to "if the offset is limited to ±2047, can it
move the seam clear of a ~2020-tick travel?" is **comfortably yes**, because the
required offset is bounded by the *unreachable* arc's position (≈1073), not by the
travel's length.

**Step 7 — verify the result is monotonic.** With `H = 1073`:

| raw | reported = `(raw − 1073) mod 4096` |
| --- | --- |
| 2020 (wall) | **947** |
| 4060 | 2987 |
| 4095 | 3022 |
| 0 | 3023 |
| 126 (rest) | **3149** |

Reported values along the travel: `947 → 2987 → 3022 → 3023 → 3149` — **strictly
increasing, no discontinuity.** The reachable set becomes the single interval
**[947, 3149]**, which a `(min, max)` pair *can* honestly describe. The seam sits at
raw 1073, dead centre of the 1894-tick unreachable arc, with **947 ticks of clearance
on each side**.

**Step 8 — cross-check against LeRobot's rule.** LeRobot's `set_half_turn_homings`
computes `H = pos − 2047` with the joint parked at mid-range. Mid-travel here is
`2020 + 2202/2 = 3121`, giving `H = 3121 − 2047 = 1074` — within **one tick** of the
1073 derived independently above. Two different routes, same answer, both far inside
±2047.

**Generality of the ±2047 bound.** Sign-magnitude on bit 11 yields
`H ∈ {−2047 … +2047}`. Modulo 4096 that covers **every residue except 2048**
(since `−2047 ≡ 2049`, and neither `+2048` nor `−2048` is representable). So there is
exactly *one* seam placement the encoding cannot express — raw 2048 — and it is not
one we need. Our required window `(126, 2020)` lies wholly inside `0…2047`.

**Why this cannot rescue `wrist_roll`** (handled separately, not this task): its
travel covers the whole circle, so the unreachable arc is *empty* and step 3 yields
nothing to evict the seam into. No choice of `H` can remove a seam from a travel that
includes every angle. That is a soft-limit problem, not a re-zero problem.

---

## 4. The caveat — what must be proven on hardware

Steps 4 and 7 above both assume:

```text
Present_Position  =  (raw − Ofs)  mod 4096          ← seam MOVES to raw == Ofs   ✅ fix works
```

The alternative is:

```text
Present_Position  =  raw − Ofs     (plain signed)   ← seam STAYS at raw 4095→0   ❌ fix is useless
```

Under the second reading the offset merely *relabels* positions: the discontinuity
remains pinned to the physical angle where the magnet's raw count rolls over, and
re-zeroing achieves **nothing**. This is not a paranoid hypothetical — `Present_Position`
really is decoded as sign-magnitude on bit 15 by both Feetech's SDK and LeRobot, which
means the register *can* carry negative values, which is precisely what an unwrapped
signed subtraction would produce.

**Evidence for the modular (correct-for-us) reading — strong, but circumstantial:**

- LeRobot's SO-101 calibration procedure is *literally the fix we are proposing*:
  *"First you need to move the robot to the position where all joints are in the middle
  of their ranges"*, then it writes homing offsets ([LeRobot SO-101 docs][lr101]).
- The community framing of the identical bug is that centring **prevents** the wrap:
  *"The wrist axis uses almost the entire motor rotation, so if it's not properly
  centered, you may encounter encoder overflow / underflow"* ([LeRobot issue
  #3193][lr3193]). Centring can only prevent overflow if the offset **relocates** the
  seam.
- Feetech's own example: *"All future position commands are relative to this new
  center"* ([`CalibrationOfs.cpp`][smsex]).

**Evidence I could not obtain:** any primary Feetech statement of the firmware's
actual formula, or of whether the corrected value is reduced mod 4096 in servo mode.
Waveshare's ST3215 wiki (which publishes the full memory table with min/max columns)
returned HTTP 403, and the Feetech PDF was not machine-readable.

Because the entire re-zero rests on this one bit of semantics, it is a **caveat, not a
GO**. It is cheap to settle — step 10 of the test below settles it in about two
minutes with the torque off.

---

## 5. The hardware persistence + seam-eviction test

Run on the follower (`/dev/ttyACM1`), `elbow_flex` = **motor id 3**. PR #21 exists
precisely because an EEPROM write *appeared* to work and then silently reverted on
power-cycle, so persistence is proven by power-cycling, not by reading back.

Encoding reminder: for a **positive** `H`, the wire value is just `H` (sign bit
clear). For a **negative** `H` it is `(1 << 11) | abs(H)` = `2048 + abs(H)`. For
`H = 1073` the wire value is simply **1073**.

1. **Baseline.** Read addr 31 (2 bytes) on motor 3 → record it (expect `0` from
   factory). Read addr 56 → record (expect ~126, the rest position).
2. **Torque off.** Write addr 40 = `0`. *(Do not write the offset with torque on —
   the servo would instantly re-interpret its own position and could lurch toward its
   goal. This is inferred, not documented; treat it as a hard rule.)*
3. **Unlock EEPROM.** Write addr 55 = `0` (this repo's `_set_lock(motor, False)`).
4. **Write the offset.** `write2ByteTxRx(id=3, addr=31, value=1073)`.
5. **Re-lock.** Write addr 55 = `1`.
6. **Read back addr 31** → must be `1073`. *(Proves the write landed.)*
7. **Read addr 56** → must now read **~3149**, i.e. shifted by −1073 mod 4096 from the
   ~126 baseline. It must **not** still read ~126 (offset ignored) and must **not**
   read negative / ~64589 (unwrapped signed — see the caveat).
8. **POWER-CYCLE the servo.** Cut and restore *bus power* — not merely close and
   reopen the serial port.
9. **Re-read addr 31** → must **still** be `1073`. If it reverted to `0`, the Lock
   dance failed exactly as id/baud did before PR #21. Re-read addr 56 → still ~3149.
10. **The seam-eviction proof (settles the caveat).** With torque **off**, hand-move
    `elbow_flex` slowly through its **entire** travel, from the wall to the far stop,
    polling addr 56 throughout. The reported value must climb **monotonically** from
    ~947 to ~3149 with **no 4095→0 jump anywhere**. If a jump appears, the corrected
    position is not modularly reduced, the re-zero does **not** solve issue #35, and
    wave 2a must **STOP** and return to the user.
    - Bonus, free: this sweep also measures `elbow_flex`'s **far wall** for the first
      time (see unknown 5 below).

**Do not skip step 10 for step 7.** Step 7 shows the offset is *applied*; only step 10
shows the seam actually *moved*, and that is the thing the whole plan depends on.

---

## 6. What I could NOT determine

Stated plainly — these are the point of the spike.

1. **Whether the corrected `Present_Position` is reduced modulo 4096** (seam moves) or
   reported as an unwrapped signed value (seam stays). *The decisive unknown.* All
   circumstantial evidence points to modular; no primary source states it. Settled by
   test step 10.
2. **Whether `Ofs` is applied to `Goal_Position` (addr 42) as well as to
   `Present_Position`.** If the servo corrected what it *reports* but not what it
   *accepts*, our commands and its feedback would live in different frames and the
   re-zero would be worse than useless. Strong inference that both are corrected —
   LeRobot commands goals in the corrected frame on this exact arm and it works — but
   I found no primary statement. A move commanded after step 6 that lands where
   expected would confirm it.
3. **Feetech's own documented min/max for register 31.** The ±2047 bound is derived
   from the bit-11 sign-magnitude encoding (LeRobot) and confirmed empirically by the
   `ValueError` in issue #3193 — but I never saw the datasheet's own min/max column.
   Waveshare's ST3215 wiki (which has it) 403'd. The bound is well-attested; its
   *provenance* is second-hand.
4. **Whether Mechanism B (`128` → addr 40) needs the Lock open to persist.** Feetech's
   own example does no unlock/relock and still claims EEPROM persistence, which
   suggests the firmware commits it internally — but PR #21 is a standing warning
   about exactly this class of assumption.
5. **`elbow_flex`'s far wall.** We know the travel reaches *at least* raw ~126; the
   upper mechanical stop has never been measured, because `arm explore` cannot see
   across the seam. So the 2202-tick travel is a **lower bound** and the 1894-tick
   unreachable arc is an **upper bound**. This does not threaten the verdict (the
   re-zero works for any travel < 4096, and 2202 ≪ 4096) but `H` should be re-derived
   once the far wall is known. Test step 10 measures it as a side-effect.
6. **Whether the STS3215 requires torque off for EEPROM writes in general.** Assumed
   (and made a hard rule above) rather than confirmed.

---

## 7. Guidance for t7, if the hardware test passes

- Write **only** address 31. **Do not** copy LeRobot's `write_calibration`, which also
  writes `Min_Position_Limit` (9) and `Max_Position_Limit` (11): in servo mode the
  firmware clamps goals to that window, and our factory values are the wide-open
  0/4095 that we *want* to keep. Narrowing them would clamp the reachable set we are
  trying to recover.
- Reuse the existing `_set_lock` unlock→write→relock pattern from
  `bus.py:write_id_baudrate` verbatim, including its best-effort re-lock on the
  failure path — address 31 is in the same EEPROM region and carries the same
  power-cycle-revert hazard.
- `read_offset` / `write_offset` must encode/decode **sign-magnitude on bit 11**, not
  two's complement. Guard the magnitude: `abs(H) > 2047` must raise a `CliError`, not
  silently corrupt bit 11 into the sign.
- Note that `bus.py:read_position` masks `& 0x0FFF`. Revisit that mask deliberately:
  it is what would silently swallow a negative reading if unknown 1 resolves the wrong
  way.

---

## Sources

- [`SMS_STS.h`][smsh] — Feetech SMS/STS memory table: `SMS_STS_OFS_L 31` /
  `SMS_STS_OFS_H 32` under the **EEPROM (Read and Write)** heading; SRAM starts at 40;
  `SMS_STS_LOCK 55`; `SMS_STS_CALIBRATION_CMD 128`.
- [`SMS_STS.cpp`][smscpp] — `CalibrationOfs`, `unLockEeprom`, `LockEeprom` bodies.
- [`CalibrationOfs.cpp`][smsex] — Feetech's official midpoint-calibration example and
  its EEPROM-persistence claim.
- [Feetech `sms_sts.py`][ftpy] — Feetech's official Python SDK: `SMS_STS_OFS_L = 31`,
  `SMS_STS_LOCK = 55`, `ReadPos` decoding via `scs_tohost(value, 15)`.
- [LeRobot `feetech/tables.py`][lrtables] — `"Homing_Offset": (31, 2)`;
  `STS_SMS_SERIES_ENCODINGS_TABLE = {"Present_Load": 10, "Homing_Offset": 11,
  "Goal_Position": 15, …, "Present_Position": 15}`.
- [LeRobot `feetech/feetech.py`][lrfeetech] — `Present_Position = Actual_Position −
  Homing_Offset`; `enable_torque`/`disable_torque` writing `Lock` 1/0 around the
  calibration write.
- [LeRobot `encoding_utils.py`][lrenc] — `encode_sign_magnitude`:
  `max_magnitude = (1 << sign_bit_index) - 1`.
- [LeRobot issue #3193][lr3193] — real SO-101 `wrist_roll` failure:
  `ValueError: Magnitude 2073 exceeds 2047 (max for sign_bit_index=11)`; encoder
  overflow/underflow when not centred.
- [LeRobot SO-101 docs][lr101] — calibration procedure: park all joints mid-range,
  then write homing offsets.
- [STS3215 spec memo (Zenn, ja)][zenn] — independent confirmation:
  corrected present position = raw encoder value − `Homing_Offset`; address 31 in
  EEPROM.
- `arm101/hardware/bus.py` (this repo) — confirmed: no offset register implemented.
- `arm-explore-follower.map.json` + issue #35 (this repo) — the measured `elbow_flex`
  numbers used in the arithmetic.

[smsh]: https://github.com/adityakamath/SCServo_Linux/blob/main/include/scservo/SMS_STS.h
[smscpp]: https://github.com/adityakamath/SCServo_Linux/blob/main/src/SMS_STS.cpp
[smsex]: https://github.com/adityakamath/SCServo_Linux/blob/main/examples/SMS_STS/CalibrationOfs/CalibrationOfs.cpp
[ftpy]: https://github.com/ftservo/FTServo_Python/blob/main/scservo_sdk/sms_sts.py
[lrtables]: https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/feetech/tables.py
[lrfeetech]: https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/feetech/feetech.py
[lrenc]: https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/encoding_utils.py
[lr3193]: https://github.com/huggingface/lerobot/issues/3193
[lr101]: https://huggingface.co/docs/lerobot/so101
[zenn]: https://zenn.dev/usagi1975/articles/2026-05-16-000_sts3215-spec
