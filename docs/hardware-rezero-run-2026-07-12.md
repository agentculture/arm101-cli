# Hardware run-log — encoder re-zero on the follower (2026-07-12)

Arm: SO-101 follower, `/dev/ttyACM1`, bolted to a table. Operator present throughout;
every joint hand-moved by a human. arm101-cli **v0.21.0**.

**Headline: the re-zero works.** The STS3215 reduces the corrected position **modulo
4096**, so a homing offset genuinely *relocates* the encoder seam rather than merely
relabelling positions. That was the one unproven assumption the whole of issue #35 rested
on ([the t5 spike](spikes/sts3215-offset-register.md) §4 called it GO-**WITH-CAVEAT**), and
it is now settled by measurement rather than inference.

## The result, before and after

The same joint, swept the same way by hand, torque off:

| | offset in force | monotonic | discontinuities | span |
|---|---|---|---|---|
| **before** | 0 | **False** | **1** | 4093 ticks |
| **after** | 1073 | **True** | **0** | 2196 ticks |

Largest single-sample jump after the re-zero: **28 ticks** — noise, against a 500-tick
discontinuity threshold. `elbow_flex` no longer wraps within its travel.

## The finding that mattered: the servos are not at factory zero

**All six servos ship with `Ofs = +85`** (EEPROM addr 31). Uniform across six joints, so it
is a vendor default, not a per-joint calibration. The spike assumed a factory `0`.

This is not a footnote — **it is the mechanism of the bug**. The reported seam sits where
`Actual == Ofs`, i.e. at **raw 85**. `elbow_flex`'s unreachable arc starts at **raw 207**.
So the seam sat *below* the arc — **inside the joint's travel**. That is exactly why
`elbow_flex` wrapped.

The register semantics were confirmed twice, reversibly, before anything was committed:

- `Ofs 85 → 185` dropped the reported position by **exactly 100**
- `Ofs 85 → 0` raised it by **exactly 85**

So `Present_Position = Actual − Ofs`, on this firmware, measured.

## `elbow_flex`'s far wall — measured for the first time

Nothing could see across the seam before, so the arc table carried only a *lower bound*.
The passing sweep measured the real thing:

- travel: **1034 .. 3230** in the corrected frame (`Ofs=1073`) — span **2196 ticks**
- in raw ticks: reachable = `[2107, 4095] ∪ [0, 207]`
- therefore the **true raw unreachable arc = (207, 2107)**, width 1900, midpoint **1157**

The shipped table said `(126, 2020)`, midpoint 1073 — but those were **reported**-frame
ticks recorded in the `Ofs=85` frame, used as if they were raw. The written offset of 1073
does land inside `(207, 2107)`, with 866/1034 ticks of margin — so **the shipped code
worked, but by luck rather than by construction**. Fixed separately; see the defects below.

## What the run validated beyond the re-zero itself

**PR #38's torque guard, on real hardware.** The first `arm rezero` attempt failed (the
frame guard, correctly), and the CLI printed:

```text
Torque released on motors 3 after an abnormal exit.
```

The arm was de-energised on the way out instead of left holding. Issue #33's fix, observed
on the physical arm rather than in a test.

**The false-pass guards earned their keep.** Three sweeps were rejected as `INCONCLUSIVE`
before the real one — coverage of 0, 0 and 376 ticks against an expected ~2202. A naive
"no discontinuity seen ⇒ PASS" would have declared victory on the very first *empty* sweep,
with the arm never touched. The ≥80%-coverage rule is what stopped it.

## Defects this run exposed

1. **The unreachable arc was in the wrong frame.** `REZERO_ARCS` held reported-frame ticks
   and used them as raw. Worked here only because the arc is 1900 ticks wide.
2. **The `current_offset in (0, target)` guard was too strict.** It refused outright on a
   servo holding the factory `85` — i.e. on *every fresh SO-101*. Now that
   `raw = (reported + offset) mod 4096` is known, any readable frame can be converted.
3. **A dropped packet raised `IndexError`, not `CliError`.** The vendor SDK indexes
   `data[1]` on a short packet while `result == COMM_SUCCESS`, so a raw traceback escaped —
   violating the repo's "no traceback ever leaks" contract. Hit live on a healthy bus.
4. **A read immediately after an EEPROM write can return garbage.** A `read_position` ~0.2 s
   after `write_offset` returned **0** while the servo genuinely held 3387 (re-reads were
   stable and correct). A plausible-looking wrong value is far more dangerous than an error.

## Persistence — CONFIRMED

The 12 V bus power was cut and restored, and the offset was re-read **cold**:

```text
| elbow_flex | 3 | ok | 3241 | ... | offset 1073 |
```

**It survived.** The other five joints still read 85, untouched. This closes the last open
risk on the re-zero: PR #21 exists precisely because an EEPROM write on these servos once
read back perfectly and then silently reverted on the next power-up when the Lock register
was mishandled. The unlock → write → re-lock dance held.

Both halves of issue #35 are therefore settled for `elbow_flex` **on hardware**: the seam is
evicted (proven by a hand sweep) *and* the fix persists across a power cycle (proven cold).

## The arm measured its own walls — better than the human did

Once the axis was linear, `gentle_move` could be driven **past** the known travel and
allowed to find each wall by feel: creep under torque, watch the load, stop on contact,
back off, hold. The blind-person-in-a-room primitive — and the same one `arm explore`
uses, which is the whole point of the exercise.

```text
arm flex elbow_flex --to  900 --gentle   ->  contacted 988   load 500 (SATURATED)
arm flex elbow_flex --to 3400 --gentle   ->  contacted 3274  load 500 (SATURATED)
```

**The second of those is the proof of the entire day's work.** Commanding 3400 crosses the
raw 4095→0 boundary — and in the corrected frame it is simply a linear climb, which
converged. That exact command, before the re-zero, would have rotated the elbow *the long
way round* into a wall.

The machine out-measured the hand on **both** sides:

| wall (raw ticks) | by hand | by the arm |
|---|---|---|
| low band | 218 | **251** |
| high band | 2107 | **2061** |

A human pushes until it *feels* firm — successive sweeps put the low wall at 206, then
218. The arm presses to a fixed load threshold every time, so its walls are both **further
out** and **repeatable**. Both contacts saturated `present_load` at the 500 torque cap,
which is the signature of a real wall rather than of an operator's judgement.

True unreachable arc: **(251, 2061)**, width 1810. The previously declared `(318, 2007)`
was *still* a strict subset — the margin absorbed the difference, with 67 and 54 ticks to
spare. It held, which is exactly why it was there.

### The lesson that outlives the numbers

The tests used to **copy** the arc — ~125 arc-coupled literals across two files — so
re-measuring a table *that exists to be re-measured* broke 34 tests. They now derive every
expectation from `arm_spec`, and the acid test is explicit: change `_ARC_MARGIN_TICKS` or
either wall, and the suite stays green. The measured numbers live in **one** place.

The same rule now applies to prose: the docstrings and the `explain` catalog no longer
quote ticks at all. A document that names a measurement is a document that goes stale — and
the `explain` text is what prints to whoever is standing at the arm.

## Current state of the arm

| joint | motor | offset | note |
|---|---|---|---|
| shoulder_pan | 1 | 85 | untouched (factory) |
| shoulder_lift | 2 | 85 | untouched (factory) |
| **elbow_flex** | **3** | **1073** | **re-zeroed — seam evicted, verified by sweep AND by the arm's own wall-find** |
| wrist_flex | 4 | 85 | untouched (factory) |
| wrist_roll | 5 | 85 | untouched — handled by a soft limit, not a re-zero |
| gripper | 6 | 85 | untouched (factory) |

Every joint limp (torque 0), EEPROM re-locked on every write.
