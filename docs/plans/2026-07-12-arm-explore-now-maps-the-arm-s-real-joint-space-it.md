# Build Plan — arm explore now maps the arm's real joint-space: it never leaves the arm energized, it moves at speeds measured from the hardware rather than guessed, its two wrapping joints have been made linear (elbow_flex re-zeroed, wrist_roll soft-limited) so a linear tick axis is finally true, and its search grid is built from the arm's measured reachable space instead of the servos' factory EEPROM limits.

slug: `arm-explore-now-maps-the-arm-s-real-joint-space-it` · status: `exported` · from frame: `arm-explore-now-maps-the-arm-s-real-joint-space-it`

> arm explore now maps the arm's real joint-space: it never leaves the arm energized, it moves at speeds measured from the hardware rather than guessed, its two wrapping joints have been made linear (elbow_flex re-zeroed, wrist_roll soft-limited) so a linear tick axis is finally true, and its search grid is built from the arm's measured reachable space instead of the servos' factory EEPROM limits.

## Tasks

### t1 — Reproduce #33: a regression test that proves an abnormal exit today leaves torque ON

- instruction: tests/test_arm_safety.py (new). The FakeBus in tests/ records enable_torque calls — assert against that call log. Write it RED against today's code.
- covers: c3, h11
- acceptance:
  - tests/test_arm_safety.py drives a motion verb against a FakeBus that raises mid-run, and asserts (against CURRENT code) that torque is left enabled on every motor -- i.e. the test FAILS to find the bug fixed, documenting the defect
  - The test is written so that it flips to green once t3 lands, without being rewritten

### t2 — arm101/hardware/safety.py: a torque_guard context manager that owns the motors it energized

- instruction: arm101/hardware/safety.py (new, file-disjoint from everything else in this wave). Zero third-party deps. Do NOT touch gentle.py — its Torque_Limit finally is a different layer and stays exactly as it is.
- covers: c22, h1, c23, h2
- acceptance:
  - Releases torque on ABNORMAL exit only (exception propagating, including KeyboardInterrupt); a clean return leaves torque as the verb left it, preserving gentle_move's stop-and-hold
  - Release is per-motor independent: a FakeBus that raises on motor 1's release still de-energizes motors 2..6
  - Uses contextlib.suppress, never a bare try/except/pass (bandit B110 fails CI lint); bandit clean
  - Pure unit-testable against FakeBus; no hardware needed

### t3 — Wire torque_guard into every gated motion verb + SIGINT

- instruction: arm101/cli/_commands/arm.py only. The guard wraps the whole run at the verb level, above the primitive. Release ONLY when exiting with an exception (or SIGINT) — a clean return must leave gentle_move's stop-and-hold intact.
- depends on: t2
- covers: c10, h21, c8, h16
- acceptance:
  - arm explore, arm flex, arm flex --demo and arm setup each run inside the guard
  - Three distinct abnormal paths each leave all six motors de-energized in tests: an injected exception, a KeyboardInterrupt, and a bus error raised from the release path itself
  - The t1 reproduction test now passes

### t4 — HARDWARE (human-gated): SIGINT and USB-yank acceptance for the safety release

- instruction: HUMAN-GATED. The arm is bolted to a table in a safe area. Do not run this yourself — prepare the procedure, hand it to the user, transcribe the result.
- depends on: t3
- covers: c19, h25
- acceptance:
  - SIGINT during a live arm explore run on the bolted follower: a subsequent arm read reports torque disabled on all six motors
  - A physical USB yank mid-run: after re-plugging, arm read reports torque disabled on all six -- this is the case that exercises the fault-tolerant release, because the bus the release needs is the bus that just broke
  - Run-log appended to docs/hardware-validation-arm-read-flex.md

### t5 — SPIKE (read-only): identify the STS3215 encoder-offset mechanism, or stop the wave

- instruction: READ-ONLY. No EEPROM write on a guess. Cross-check the STS3215 memory table against LeRobot's feetech tables. A STOP recommendation is a successful outcome — say so plainly rather than forcing a register that isn't there.
- depends on: t3
- acceptance:
  - Names the register: address, width, EEPROM-vs-RAM, and usable range -- sourced from the STS3215 memory table and cross-checked against LeRobot's feetech tables, not from memory
  - Proves on hardware that a written offset SURVIVES A POWER CYCLE (PR #21's bug was exactly a write that looked fine until power-cycled)
  - STRICTLY READ-ONLY until the register is identified: no EEPROM write happens on a guess
  - If no such register exists, has too little range, or is RAM-only: the task's output is a STOP recommendation and wave 2a halts for a re-decision. This is a legitimate, successful outcome of the spike

### t6 — HARDWARE: re-confirm elbow_flex's wrap BEFORE re-zeroing destroys the evidence

- instruction: HUMAN-GATED hardware. Must run BEFORE t8 — the re-zero destroys the evidence this task records.
- depends on: t3
- covers: c6, h14
- acceptance:
  - Driving elbow_flex toward 4095 is observed to roll past the seam and read back near 1
  - Sorting its two probe endpoints is shown to yield a range whose interior the joint cannot enter -- the inversion is recorded, not just asserted
  - Captured in the run-log before any re-zero is attempted

### t7 — bus.py: offset-register read/write primitives with the EEPROM unlock -> write -> relock dance

- instruction: arm101/hardware/bus.py. Mirror write_torque_limit's shape and error contract. Extend FakeBus so the write path is unit-testable. The addr-55 Lock dance is PR #21 — read that diff first.
- depends on: t5
- covers: c43, h30
- acceptance:
  - read_offset / write_offset primitives mirror the existing register helpers (write_torque_limit et al) in shape and error contract
  - The write is wrapped in the addr-55 Lock unlock -> write -> relock sequence PR #21 established
  - FakeBus models the offset register so the write path is unit-testable without hardware

### t8 — arm rezero: a gated calibration verb that re-zeros elbow_flex WITHOUT assuming a linear axis

- instruction: The bootstrap problem is the hard part: the tool that MAKES the axis linear cannot itself assume the axis is linear. elbow_flex rests at ~126, past its wrap.
- depends on: t7
- covers: c25, h4
- acceptance:
  - The procedure establishes where the joint PHYSICALLY is before commanding it anywhere -- elbow_flex currently rests at ~126, PAST its wrap, so a naive linear command rotates it the long way round
  - Validated from the arm's CURRENT resting state, not from a conveniently chosen pose
  - Routes through the existing three-mode consent gate (dry-run / TTY / agent --apply) like every other EEPROM-writing verb
  - Only elbow_flex is re-zeroed; the verb must NOT offer to re-zero wrist_roll, and says why

### t9 — arm_spec: wrist_roll soft limit whose dead arc CONTAINS the 4095->0 seam

- instruction: arm101/hardware/arm_spec.py + the map. File-disjoint from t7/t8, so it can run in parallel with them. NEVER write min_angle/max_angle to the servo — that is an explicit spec boundary.
- depends on: t3
- covers: c37, h19, c38, h28
- acceptance:
  - The soft limit is a SOFTWARE bound in arm_spec (and the map); the servo's EEPROM min_angle/max_angle stay at the factory 0-4095, verified by reading them back
  - The excluded dead arc is asserted to contain the seam: the permitted range is contiguous in ticks with no 4095->0 boundary inside it
  - A unit test proves a sweep across the permitted range is monotonic in ticks

### t10 — Calibration identity on reachability maps; explore refuses a map from a different calibration

- instruction: arm101/explore/types.py + reachmap.py + the explore verb. Test the guard against the REAL pre-re-zero map committed in #32 — that artifact is the thing this protects against.
- depends on: t8
- covers: c26, h5
- acceptance:
  - ReachMap carries a calibration identity (the per-joint offsets/soft-limits it was measured under) and round-trips through save_map/load_map_file
  - explore RAISES a structured CliError naming the mismatch when handed a map measured under a different calibration -- it never silently misreads stale ticks as if they still meant the same angles
  - The map committed in #32 (pre-re-zero) is rejected after the re-zero, proving the guard fires on the real artifact

### t11 — HARDWARE (human-gated): wave 2a acceptance -- the linear axis is now TRUE for all six joints

- instruction: HUMAN-GATED. Includes a full POWER CYCLE — PR #21's bug was a write that looked fine until power-cycled.
- depends on: t8, t9, t10
- covers: c42, h29, c21, h27, c43, h30
- acceptance:
  - elbow_flex is re-zeroed and, after a FULL POWER CYCLE, present_position reads back the new zero
  - Each of the six joints is swept across its full permitted travel and yields MONOTONIC encoder reads -- no 4095->0 jump anywhere inside permitted travel
  - wrist_roll, commanded 300 ticks from a position that previously sat on the seam, arrives inside arrival tolerance instead of exhausting its travel budget (the exact move that timed out on hardware: parked at 4, commanded to 304, gave up at 3055)
  - Run-log appended to docs/hardware-validation-arm-read-flex.md

### t12 — HARDWARE: baseline timing -- reproduce today's per-joint travel rate at speed 150

- instruction: HUMAN-GATED hardware, read-mostly. Establishes the baseline the profile must beat, or honestly fail to beat (risk r2).
- depends on: t3
- covers: c7, h15
- acceptance:
  - A 500-tick move is timed on each joint at speed 150 and reproduces the ~930ms (wrist_roll) to ~3300ms (shoulders) spread, confirming the shipped constants were fitted to one bench session
  - This is the number the speed profile must beat -- or honestly fail to beat (see risk r2)

### t13 — arm profile: a gated verb that ramps goal speed and finds where CONTACT DETECTION still works

- instruction: arm101/hardware/profile.py (new) + the CLI verb. Runs inside the wave-1 torque_guard. The contact-at-every-candidate-speed requirement is the whole point — a free-motion-only ramp is the wrong verb.
- depends on: t3
- covers: c27, h6
- acceptance:
  - For each candidate speed the profile exercises a REAL CONTACT and confirms the stall rule still detects it. A speed validated on free motion alone is NOT accepted
  - A speed the servo survives but at which the stall rule can no longer tell blocked from accelerating is reported as a FAILURE of that speed, not a pass -- this distinction is the whole point of the verb
  - Records per joint: highest safe speed, measured ticks/second, and motion-onset latency
  - Runs inside the wave-1 torque_guard and honours the same consent gate as the other motion verbs

### t14 — arm_spec speed defaults + --speed-profile override; delete the hand-set motion constants

- instruction: arm101/hardware/arm_spec.py + gentle.py. Mirror resolve_contact_thresholds() from #26 exactly: flag > blanket > file > per-joint default.
- depends on: t13
- covers: c12, h22
- acceptance:
  - Measured per-joint speeds ship as checked-in defaults in arm_spec, with an optional --speed-profile file override -- the SAME precedence as the per-joint contact thresholds from #26 (flag > blanket > file > default)
  - gentle_move's speed default, _MIN_TICKS_PER_SECOND and _travel_timeout are all sourced from the profile; grepping for the magic numbers 150, 120 and the 2.0/2.0/6.0 timeout terms finds none left hand-set in the motion path
  - A fresh clone can still move the arm with no profile file present

### t15 — HARDWARE (human-gated): run arm profile on the follower; land the measured numbers

- instruction: HUMAN-GATED. A negative result (150 is already the ceiling) is a VALID outcome — record it and hand it to wave 3 as a constraint.
- depends on: t13, t14
- acceptance:
  - arm profile runs all six joints on the bolted follower and produces per-joint safe max speed + ticks/second
  - The measured values are landed as the arm_spec defaults
  - If the profile finds speed 150 is ALREADY at or above the ceiling for reliable contact detection (risk r2), that negative result is recorded and handed to wave 3 as a constraint -- it is a valid outcome, not a failure of the task
  - Run-log appended to docs/hardware-validation-arm-read-flex.md

### t16 — explore/grid.py: per-joint bucket size derived from span and a target bucket COUNT

- instruction: arm101/explore/grid.py + tests. Pure functions, no hardware, no CLI. Write the defect test RED first: 512 collapses shoulder_pan's 613-tick span to one bucket.
- depends on: t3
- covers: c5, h13, c29, h8
- acceptance:
  - A unit test first PROVES the defect: at bucket_size 512, config_to_cell maps every tick of shoulder_pan's measured 613-tick span to the same bucket, and grid.neighbors offers it no in-range step -- the joint cannot move in grid terms at all
  - For a target bucket count N, every joint yields N buckets (+-1) -- shoulder_pan's 613-tick span and wrist_roll's 4052-tick span alike -- so no joint is collapsed to a single bucket
  - Pure functions, no hardware, no CLI changes in this task

### t17 — _build_grid_spec: bounds from the supplied map (a PRIOR), EEPROM only as fallback

- instruction: arm101/cli/_commands/arm.py:_build_grid_spec. GridSpec ALREADY carries the per-joint bucket_size tuple — the CLI is what throws it away with tuple(resolution for _ in JOINTS).
- depends on: t16, t10, t9
- covers: c13, h23, c28, h7, c4, h12
- acceptance:
  - Given the committed follower map, the resulting GridSpec bounds equal that map's measured ranges
  - Given NO map, bounds fall back to the servos' EEPROM angle limits and the verb SAYS SO in its output -- the fallback is as load-bearing as the prior and is tested in both directions
  - The six measured spans (shoulder_pan 613 ... wrist_roll 4052) are re-derived from the map, not hard-coded
  - The per-joint bucket_size tuple GridSpec already carries is actually populated -- the CLI stops throwing it away

### t18 — Probe-cost model: honest --max-moves default + a PRE-FLIGHT report before any torque

- instruction: arm101/explore/budget.py + the verb. Pre-flight must be emitted BEFORE torque is enabled — ordering is the requirement, not a nicety. See risk r3: the real limit may be duty cycle, not moves.
- depends on: t17, t14
- covers: c14, h24, c30, h9
- acceptance:
  - The move budget is derived from the MEASURED probe cost (travel + retreat + the escape search a blocked cell triggers), not from the 2000 default set when a probe appeared to cost 60ms because the arm never moved
  - Before any torque is enabled, explore reports the grid it will search (buckets per joint, total cells) and the wall-clock the run will cost -- ordering is the requirement: a prediction printed after the arm energizes is a receipt, not a prediction
  - A run that would exceed the operator's budget can be abandoned at the pre-flight point with the arm never having moved
  - The predicted wall-clock lands within a stated factor of the observed wall-clock on the t21 hardware run; if it cannot, the prediction is a false reassurance and the task is not done

### t19 — Guard test: nothing in the shipped change ever writes the servos' angle limits

- instruction: tests/ only. Small, file-disjoint. Guards the one boundary both #34 and #35 tempt you to cross.
- depends on: t17
- covers: c15, h18
- acceptance:
  - A test fails if any code path writes min_angle/max_angle to a servo -- the measured ranges live in the map and in arm_spec, NEVER burned into the EEPROM
  - Covers both the reachable ranges (#34) and wrist_roll's soft limit (#35), which are the two temptations to do exactly this

### t20 — Output legibility: the JSON payload tells an agent what the text tells a human

- instruction: The audience test from the spec: an agent reading only the JSON must reach the same conclusion a human reading the text does.
- depends on: t18
- covers: c2, h20
- acceptance:
  - From the JSON payload alone, an agent can determine which calibration the map was measured under, how much of the grid was actually probed, and what the run cost -- the same conclusions a human draws from the text output
  - Applies to arm explore and arm profile alike; --json keeps the stdout/stderr split

### t21 — HARDWARE (human-gated): the acceptance run for the whole spec

- instruction: HUMAN-GATED. This is the acceptance run for the WHOLE spec — everything else is a means to it. Commit the resulting map with its calibration identity.
- depends on: t18, t19, t20, t11, t15
- covers: c1, h10, c9, h17, c20, h26
- acceptance:
  - arm explore, seeded with the measured map, predicts its cost BEFORE moving, and finishes inside the interval it predicted
  - The event log contains probes in which MORE THAN ONE joint differs from home -- explore finally answers the question it exists to answer (which joint COMBINATIONS are reachable), rather than the single-joint range measurement shipped in #32
  - The run ends with the arm limp
  - Run-log appended to docs/hardware-validation-arm-read-flex.md; the resulting map is committed with its calibration identity

## Risks

- [unknown_nonblocking] The STS3215 offset register may not exist, may be RAM-only, or may lack the range to move a seam. bus.py implements no such register today and the mechanism is UNVERIFIED. If the t5 spike fails, wave 2a STOPS -- and the fallback (modelling the reachable set as ARCS) is one the user explicitly did not pick, so this returns to the user as a re-decision, not to the builder as an improvisation. [DOWNGRADED to non-blocking: the plan manages this structurally -- t5 is a read-only spike gate, and every task that depends on the register's existence (t7, t8, t10, t11) sits behind it in the graph. Waves 0-2 (the whole #33 safety fix) do not depend on the answer.] (task t5)
- [unknown_nonblocking] The speed profile may find that today's speed 150 is ALREADY at or above the ceiling for reliable contact detection on the slow shoulder joints. Then probe cost is IRREDUCIBLE, and the only levers left for making explore affordable are a coarser grid or a smaller region -- i.e. wave 2b returns a NEGATIVE result that constrains wave 3 rather than helping it. Plan for that outcome; do not treat it as a failed task. (task t15)
- [unknown_nonblocking] The THERMAL ceiling may bind before the move budget does -- the first honest run hit 50C in 25 minutes. If so, a budget expressed in MOVES is the wrong unit entirely and the real limit is DUTY CYCLE, which would change what t18 is even modelling. (task t18)
- [unknown_nonblocking] The default target bucket COUNT is unchosen. Comparable per-joint granularity is the requirement; the specific N that trades cell count against usefulness wants a real run behind it. (task t16)
- [follow_up] explore may need to seed its flood-fill from a CENTRAL pose rather than wherever the arm happens to rest. Starting at a corner of the reachable space means the first probes are all walls -- part of why the first run mapped 2 cells. (task t18)
- [follow_up] The bounded multi-joint ESCAPE search needs re-costing and pruning. Every blocked cell triggers it, and at the honest post-#32 probe cost that multiplier is what turned a 150-move run into 25 minutes. (task t18)
