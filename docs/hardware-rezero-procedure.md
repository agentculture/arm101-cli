# Hardware procedure — re-zeroing `elbow_flex`'s encoder (issue #35)

**Who this is for:** a human standing at the follower arm, with one hand free
for the joint and one for the keyboard. It should be followable start to finish
without reading anything else.

**What you are doing:** writing one number into one servo's EEPROM, so that the
encoder's 4095→0 **seam** falls in the arc `elbow_flex` physically cannot reach,
instead of in the middle of its travel. Then *proving* it worked — which takes
your hand, not the motor.

**How long:** about 15 minutes, most of it the two sweeps.

---

## Before you start

| Thing | Value |
| --- | --- |
| Arm | follower |
| Port | `/dev/ttyACM1` |
| Joint | `elbow_flex` |
| Motor id | **3** |
| Register | `Ofs` / `Homing_Offset` — EEPROM **addr 31**, 2 bytes |
| Offset to be written | **+1157** (wire value `1157`) |
| Where the seam ends up | raw tick 1157 — dead centre of the arc `(207, 2107)` the joint cannot reach, with 950 ticks of clearance on each side |
| The joint's real travel | **2196 ticks**, raw `[2107, 4095] ∪ [0, 207]` — it *wraps*, which is the whole problem |

You do **not** type any of those numbers. They come from
`arm101/hardware/arm_spec.py` (`REZERO_ARCS`), and the offset is *derived* from
the measured arc rather than typed, so it cannot drift away from it.

**Those are RAW ticks — not the ticks a servo reports.** A servo reports
`Present = (Actual − Ofs) mod 4096`, so the two frames coincide only when the
offset register holds 0, and **no servo ships that way**: the factory default is
**85**, on all six joints. To go from what you read to what the arc is talking
about: `raw = (reported + offset) mod 4096`. Getting this backwards is the bug
this procedure was rewritten to fix, on 2026-07-12 — the arc used to say
`(126, 2020)`, which were *reported* ticks measured at `Ofs = 85` and then used as
though they were raw.

Two things to have ready:

- **A way to cut and restore power to the servo bus** — not just unplugging USB.
  Step 5 needs a real power-cycle, and there is no way around it (see
  [Why the power-cycle is not optional](#why-the-power-cycle-is-not-optional)).
- **A hand on the elbow.** Steps 3 and 6 de-energise the joint. If the arm is
  holding a pose it **will sag** when torque drops. Support it.

### Sanity check the arm is alive

```bash
uv run arm101 arm read
```

Every joint should read `ok`. Note `elbow_flex`'s `position` and `offset`
columns.

On a servo that has never been re-zeroed the `offset` reads **85** — the factory
default, and you should see it on *all six* joints. That is normal and expected;
it is not something anyone did. It also means the joint's seam currently sits at
raw tick 85 — which is **inside** `elbow_flex`'s travel (the raw band `[0, 207]`
is reachable). That is issue #35, and 85 is where it actually lives.

If the `offset` is anything else, you are not starting from a fresh servo — which
is fine, the verb reads whatever is there and converts. See [If something looks
wrong](#if-something-looks-wrong).

---

## Step 1 — Look at the plan (writes nothing, opens no bus)

```bash
uv run arm101 arm rezero elbow_flex
```

This is a dry-run. It prints the exact register writes it *would* perform:

```text
write1ByteTxRx(addr=40, value=0)     # Torque_Enable OFF
write1ByteTxRx(addr=55, value=0)     # Lock OPEN
write2ByteTxRx(addr=31, value=1073)  # Ofs/Homing_Offset = +1073
write1ByteTxRx(addr=55, value=1)     # Lock CLOSED
(no goal position is ever written — this verb commands NO motion)
```

Read that last line twice. **This procedure never commands the joint to move.**
That is not a simplification, it is the safety property — see
[Why nothing here moves the joint](#why-nothing-here-moves-the-joint).

`arm101 arm rezero wrist_roll` will refuse and explain why (it cannot be
re-zeroed at all, and that is permanent, not a missing feature). Try it if you
want to understand the shape of the problem.

---

## Step 2 — (Optional but recommended) Photograph the bug first

Sweep the joint **before** you change anything. This shows you the seam with
your own eyes, so that when it is gone in step 6 you know it was actually you
who moved it.

```bash
uv run arm101 arm rezero elbow_flex --verify --apply
```

Follow the on-screen instructions (they are the same as
[step 6](#step-6--prove-the-seam-actually-moved)). Expect the verdict
**`seam-present-baseline`** and a jump of roughly **4000 ticks** somewhere in the
middle of the travel. That jump *is* issue #35. The command exits 0 — a baseline
is not a failure.

---

## Step 3 — Write the offset

Support the elbow. Torque is about to drop.

```bash
uv run arm101 arm rezero elbow_flex --apply
```

At a terminal it prompts; type `yes`. What it does, in order:

1. Reads the offset the servo is *currently* holding (85 on a fresh one) and the
   position it currently reports, and converts the two into a **raw** tick:
   `raw = (reported + offset) mod 4096`.
2. Checks that raw position is somewhere the joint is actually *able* to be. (If
   not, it refuses — the table would not describe your arm.)
3. Asks the only question that matters: **is the seam already out of the joint's
   travel?** If the offset in force already puts it inside `(207, 2107)`, the
   joint is already fixed and the verb writes **nothing** — see [If something
   looks wrong](#if-something-looks-wrong).
4. Otherwise: clears any latched overload and **disables torque**. A servo must
   not be *holding* while its own frame of reference moves underneath it.
5. Opens the EEPROM lock, writes **addr 31 = 1157**, closes the lock.
6. Reads the offset back and reports it.

On a factory-fresh arm (offset 85, elbow resting at raw ~126, so reporting ~41)
you should see:

```text
- offset before    : 85  (seam was at raw tick 85 — inside this joint's travel)
- offset written   : +1157 (wire value 1157, EEPROM addr 31)
- offset read back : 1157  <- the write LANDED
- seam now at      : raw tick 1157 — inside the arc the joint cannot reach
- reported before  : 41
- reported after   : 3065  (predicted 3065, delta 0)
```

The `reported after` value is the first evidence: the servo's own report jumped to
~3065, which is `(126 − 1157) mod 4096` — the raw position, corrected by the new
offset. That is the modular correction working.

**If you see a WARNING here** — the position did not change, or it went negative
— the offset was applied but the firmware is not using it the way we need.
Continue to step 6 anyway; the sweep is what settles it, and its verdict is the
one that counts.

---

## Step 4 — Confirm the joint did not move

Look at the arm. `elbow_flex` should be exactly where it was.

Nothing commanded it to move, and nothing should have. The *reported number*
changed by ~3000 ticks; the *physical joint* changed by nothing. If the joint
moved, stop and report it — something is commanding motion that should not be.

---

## Step 5 — POWER-CYCLE the servo

**Cut power to the servo bus and restore it.** Not the USB cable — the *bus
power*. Then re-read:

```bash
uv run arm101 arm read --json
```

`elbow_flex`'s `offset` must **still be 1157**.

If it reverted to **85** (the factory value), the write did not persist, and the
EEPROM Lock dance failed. Stop and see [If something looks
wrong](#if-something-looks-wrong).

### Why the power-cycle is not optional

PR #21 of this repo exists because exactly this went wrong before. Servo `id` and
`baudrate` writes *appeared* to work — they read back correctly, every time — and
then silently **reverted on the next power-up**, because the EEPROM Lock register
(addr 55) had not been opened around the write. On this firmware a write while
`Lock=1` updates the *live* register but is never committed to EEPROM.

`addr 31` sits in the same EEPROM region and fails the same way. The read-back in
step 3 proves the servo **accepted** the value. Only the power-cycle proves it
**kept** it. These are different claims, and only one of them survives the night.

---

## Step 6 — Prove the seam actually moved

This is the step the whole exercise exists for. **Do not skip it, and do not
substitute step 3's read-back for it.**

Reading the offset back proves it was *applied*. It does **not** prove the seam
*moved*. Only a sweep does — see [The thing nobody knew — and what settled
it](#the-thing-nobody-knew--and-what-settled-it).

Support the elbow. Torque is about to drop and **stay** dropped.

```bash
uv run arm101 arm rezero elbow_flex --verify --apply
```

Then, for the 30 seconds it gives you:

1. **Put one hand on the elbow link.** It is limp.
2. **Move it slowly, all the way to one hard stop.** Gently — you are looking for
   the wall, not fighting it.
3. **Move it slowly, all the way back to the other hard stop.** *This is the part
   that matters.* You must traverse the **entire** travel — the whole way, end to
   end. A sweep that covers only part of the travel proves nothing, and the tool
   will tell you so rather than pretend otherwise.
4. **Do not hurry.** Hurrying is how a seam crossing hides between two samples.

You will see the position ticking past on screen as you move it. That feed is
there so you can tell "the tool is watching me" from "the tool has wedged and I
am wobbling a dead arm".

### Reading the verdict

| Verdict | Exit | What it means |
| --- | --- | --- |
| **`seam-evicted`** | 0 | **Done.** The joint's reported position climbed steadily across its whole travel with no jump. The seam is out of the travel, the tick axis is linear, and issue #35 is fixed. |
| **`seam-not-evicted`** | 2 | **STOP.** See below. |
| `inconclusive` | 0 | You did not move the joint through enough of its travel (or the offset was not in force). Nothing was proved either way. Just do it again, and go end to end. |
| `seam-present-baseline` | 0 | The joint was not re-zeroed when you swept it. If you got this *after* step 5, the offset did not persist — go back to step 5. |

The passing report also states **the span of `elbow_flex`'s travel**, in both
frames — the reported ticks it actually saw, and the raw ticks they convert to.
This is how the arc in `arm_spec.REZERO_ARCS` was measured in the first place (a
2196-tick sweep on 2026-07-12, the first time anything had ever seen across the
seam). If your span differs from 2196 by more than a hand's slop at the walls, the
arc is what to correct — **and it is raw ticks**, so convert before you touch it.

### If the verdict is `seam-not-evicted`

**Stop. Do not work around this, and do not build anything on top of it.**

The offset is in force, and the joint's reported position *still* jumps
discontinuously in the middle of its travel. That means the servo does **not**
reduce the corrected position modulo 4096 — it reports a plain signed
subtraction, so the offset only *relabels* positions and the seam stays pinned to
the physical angle where the magnet rolls over.

The re-zero cannot fix issue #35. The command exits 2 and says so. Take the
report to the user: this needs a decision (a software soft limit, as `wrist_roll`
uses, or unwrapping the encoder in software), not a retry.

---

## Background

### Why nothing here moves the joint

**The tool that makes the tick axis linear cannot itself rely on the tick axis
being linear.**

The natural procedure would be "drive the joint to mid-travel, then centre it
there". It is exactly the procedure that must not run. `elbow_flex` rests at raw
tick **~126** — which is *past* its wrap. A goal at its mid-travel looks like a
modest move in tick-space, and is in fact a rotation **the long way round**: from
126 down through 0, across the whole 1900-tick arc the joint cannot reach, and
into a wall. The commanded number is sane; the physical consequence is not — and
that gap *is* the bug being fixed.

So the verb reads where the joint physically **is**, computes the offset from the
joint's known unreachable arc, and writes it. No goal position is written at any
point, on any path. The only thing that moves the joint in this entire procedure
is **your hand**, in step 6 — and a human arm is the right instrument precisely
because it is the one actuator in the building that does not need a linear tick
axis to work.

### The thing nobody knew — and what settled it

One bit of firmware behaviour decides whether this whole fix works, and Feetech
have never documented it:

```text
Present = (raw − Ofs) mod 4096      the seam RELOCATES  →  the fix works
Present =  raw − Ofs   (signed)     the seam STAYS      →  the fix does NOTHING
```

Every source we could find — Feetech's own SDK, LeRobot, and LeRobot's shipped
SO-101 calibration procedure, which is *literally this fix* — implied the first.
None of them stated it. The evidence was strong and entirely circumstantial.

**It is the first. Settled on this arm, 2026-07-12, by exactly the sweep in step
6.** With `Ofs = 0` it came back `monotonic: False, discontinuities: 1` — the
seam, sitting in the travel, photographed. With `Ofs = 1073` (inside the
unreachable arc) the same sweep came back `monotonic: True, discontinuities: 0`
across all 2196 ticks. The correction *is* reduced modulo 4096; the seam *does*
relocate.

Step 6 stays anyway, and is not a formality: that is one arm and one firmware
revision, the `seam-not-evicted` verdict is what would catch a servo that behaves
differently, and a verification that cannot fail is not a verification. Full
write-up: `docs/spikes/sts3215-offset-register.md`, section 4.

### The other thing nobody knew — the factory offset is 85, not 0

Everything above (and the spike) assumed a factory servo holds `Ofs = 0`. It does
not: **all six joints of this follower shipped holding 85**, and uniformly, so it
is a vendor default rather than anything anyone did.

That mattered more than it sounds. The arc in `REZERO_ARCS` was originally
measured by reading positions off servos that were *already* correcting by −85 —
so it was in the **reported** frame, and it was then used as if it were **raw**.
The target it produced (1073) was therefore computed a whole factory-offset away
from where it was meant to be. It landed inside the true arc regardless, with
~866 ticks of margin to spare — which is the most dangerous way for a frame bug
to behave, because every read-back looks correct. A narrower arc, or a bigger
factory offset, and it would have parked the seam back inside the joint's travel
while still reporting success.

The verb now reads whatever offset a servo holds, converts out of it, and reasons
entirely in raw ticks.

### Why `wrist_roll` is not on this list

A re-zero only ever **relocates** a seam. It can never **evict** one.

Eviction needs an arc the joint physically cannot reach — somewhere to put the
seam where the joint will never follow it. `elbow_flex` has one: real mechanical
walls, and a 1900-tick arc between them that it cannot enter. `wrist_roll` has
none: exploration drove it right around and found **no wall anywhere** (measured
free range `[21, 4073]`). Every angle is reachable, including whichever one you
move the seam to.

So `wrist_roll` gets a **soft limit** instead — a software-only travel
restriction that carves out a dead arc the joint is simply never *commanded*
into, and puts the seam in there. That is already in force
(`arm_spec.SOFT_LIMITS`). It is a different fix for a genuinely different
problem, and no amount of re-zeroing would have helped.

---

## If something looks wrong

| Symptom | What it means | What to do |
| --- | --- | --- |
| `arm read` shows `offset` = **85** before you start | Nothing is wrong. That is the factory default, on every joint. The seam is at raw 85, inside the travel — which is the bug you are here to fix. | Carry on with step 1. |
| `--apply` says **"already re-zeroed"** and writes nothing | The offset in force already puts the seam inside `(207, 2107)`, so the seam is already out of the joint's travel — which is the entire goal. It does **not** have to equal 1157. (This arm holds **1073**, from an earlier re-zero: still evicting, still fine.) | Nothing. Skip to step 6 if you have not proved it with a sweep yet. Do not "fix" the number — rewriting a working calibration spends an EEPROM write to move a seam from one unreachable tick to another. |
| `arm read` shows some other `offset` | The verb reads whatever is there and converts (`raw = reported + offset, mod 4096`) — an unfamiliar number is not an error. | Carry on. It will either report a no-op (seam already evicted) or re-zero from that frame. |
| `"reports raw encoder position N, which is INSIDE the arc"` | The joint says it is somewhere it should be physically unable to be. The arc table does not describe this arm — wrong motor, or the travel has genuinely changed. | Check motor 3 really is `elbow_flex` (`arm read`). Do **not** override it. If the travel changed, re-measure the walls **in raw ticks** (`raw = reported + offset, mod 4096`) and correct `REZERO_ARCS` in `arm101/hardware/arm_spec.py`; the offset is derived from that table. |
| `"The encoder offset did NOT take"` | The servo accepted the write and is not holding the value. | The Lock register (addr 55) is being re-closed by something else, or motor 3 is not the servo you think it is. This is PR #21's failure mode. |
| Offset reverted to 85 after the power-cycle | The write did not persist — the EEPROM Lock dance failed, and the servo fell back to its factory default. | Re-run step 3. If it reverts again, that is a real bug in the Lock handling; capture it and report it. |
| The arm sagged when you ran step 3 or 6 | Working as designed. Both steps de-energise the joint deliberately. | Support the arm before running them. |
| The sweep says `inconclusive` | You did not move the joint far enough — over 80% of the travel is required before "no seam" means anything. | Run it again and go from one hard stop **all the way** to the other. |

---

## What to record

When you are done, note these in the run log — the first three are numbers
nobody has:

- The verdict, and the **span** the sweep measured. This is `elbow_flex`'s real
  travel, and the first measurement of its far wall.
- The `minimum` and `maximum` reported positions across the sweep. Together with
  the span they give the joint's true reachable interval in the *corrected*
  frame — which is now a single interval, which is the whole point.
- The **largest single-sample jump**. On a pass this should be tens of ticks
  (your hand). Anything in the thousands is the seam.
- Whether the offset survived the power-cycle.

If the verdict was `seam-evicted`, the arc table in `arm101/hardware/arm_spec.py`
can now be corrected from a lower bound to a measured fact — and the offset,
being derived from it, will follow automatically.
