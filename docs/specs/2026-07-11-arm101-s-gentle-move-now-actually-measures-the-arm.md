# arm101's gentle_move now actually measures the arm: it watches load DURING travel and returns only when the joint has really arrived — so contact detection stops being blind and 'reachable' stops being a guess

> arm101's gentle_move now actually measures the arm: it watches load DURING travel and returns only when the joint has really arrived — so contact detection stops being blind and 'reachable' stops being a guess
> instruction: Rewrite the stepping loop in arm101/hardware/gentle.py so it measures instead of assumes; everything below is in service of that.

## Audience

- arm101-cli's own motion stack and everything built on it: 'arm flex --gentle', 'arm explore', and any future gripper-control verb — plus the human standing next to a real SO-101 who is trusting the stop-and-hold claim
  - instruction: Do not change the CLI surface: 'arm flex --gentle', 'arm explore' and the demo sweep keep their flags, consent gating and JSON keys. Only the primitive's internals and its timing behaviour change.

## Before → After

- Before: gentle_move fires ~20 goal-writes at bus speed and reads present_load ~1ms after each one, before the servo has mechanically responded; it tracks 'current' as the COMMANDED tick and exits when the writes are exhausted, not when the arm arrives. Measured on hardware 2026-07-12: the call returned in 71ms claiming final_position=3548 while wrist_roll was still sitting at its start of 3148; real travel took ~900ms, entirely after the function stopped watching.
  - instruction: Keep the timing probe as a committed diagnostic (scripts/ or docs/) so the bug's evidence is reproducible: gentle_move on wrist_roll, 400 ticks, poll position+load after return.
- After: gentle_move polls present_position and present_load DURING travel, decides contact from measurements taken while the joint is actually moving, and returns only on measured arrival (or a timeout/contact), with final_position being a value it read off the servo rather than one it assumed
  - instruction: In the move loop: write the goal, then POLL bus.read_info(motor) on an interval, tracking present_position and load_magnitude(present_load) during travel. Exit on measured arrival (|present_position - goal| <= tolerance), on contact, or on timeout.

## Why it matters

- the stop-and-hold contact safety that 'arm flex --gentle' and 'arm explore' both advertise has never actually worked on hardware — the load-watch can only catch a joint that was ALREADY loaded before the probe began. The only thing that has really been protecting the arm is the Torque_Limit cap of 500. Every 'reachable' verdict in the v0.16/v0.17 reachability map is an assertion about commanded ticks that was never checked against the physical arm.
  - instruction: Before fixing, prove the failure: drive a joint into an obstacle mid-travel on the CURRENT code and record that no contact is detected. That recording is the regression baseline.

## Requirements

- gentle_move terminates on a MEASURED condition — the joint's read-back present_position reaching the goal within a tolerance, a detected contact, or a timeout — and never on 'the commanded ticks have been written'
  - instruction: Terminate on: measured arrival within tolerance, detected contact, or timeout. Never on 'commanded ticks exhausted'. The loop variable must be a read-back present_position, not the commanded next_position.
  - honesty: on real hardware a gentle_move into free space takes the servo's REAL travel time, not the bus-write time: a 400-tick move at speed 150 returns in roughly 900ms with a read-back present_position within tolerance of the goal — never in ~70ms
- the reported final_position, contact_position and contact_load are values read off the servo, never assumed from the commanded target
  - instruction: Result dict: final_position <- last read present_position; contact_position/contact_load <- the readings at the sample that tripped the stall+threshold rule.
  - honesty: an implementation that reports the commanded target instead of a read-back value FAILS the suite: the fake bus models travel (the servo does not arrive on the write that commands it), so assumed-vs-measured is observable in a test
- the per-joint DEFAULT_CONTACT_THRESHOLDS are re-derived from real free-motion travel load profiles measured on hardware, since the PR #31 values were tuned against the near-zero reads this bug produced
  - instruction: Hardware profiling run (human-gated): sweep each of the 6 joints through free space recording its load profile, then press each into a blocked state. Derive DEFAULT_CONTACT_THRESHOLDS from the measured free-motion peak vs blocked load and record the band per joint in arm_spec. Verify a usable band EXISTS for each joint (see q1).
  - honesty: for EVERY joint a usable band actually exists: the load it develops when genuinely blocked is separable from the peak load it develops merely accelerating through free space (wrist_roll's free-motion peak measured 272 against its current 180 threshold)

## Honesty conditions

- the shipped gentle_move can be shown, on the physical SO-101, to (a) return only after measured arrival and (b) stop-and-hold on a contact created BY the move itself — the two things the current implementation provably cannot do
- every existing caller — 'arm flex --gentle', 'arm explore', the demo sweep — keeps working through the corrected primitive with no change to its CLI contract, consent gating, or JSON payload keys
- the diagnosis is reproducible, not anecdotal: re-running the timing probe on wrist_roll reproduces a ~71ms return against ~900ms of real travel, and the fake bus in tests/test_gentle.py is confirmed to teleport the servo inside write_goal_position
- final_position, contact_position and contact_load in the returned dict can each be traced to a bus read taken during or after the motion — never to the commanded target
- the claim that the load-watch never worked is verified rather than inferred: a deliberate mid-travel obstacle test on the CURRENT code fails to detect contact, and the same test on the fixed code detects it
- the t7-proven overload safety is demonstrably intact after the change: the Torque_Limit cap of 500 is still applied for the duration of the move and restored in the finally, and a servo error=32 latch is still caught and cleared via clear_overload rather than raised
- both halves are demonstrated on real hardware in one session and written up in the run-log doc: the free-space move takes real travel time, and the obstacle move stops on the contact it caused
- the new tests are shown to be genuine regression tests: they FAIL when run against the pre-fix gentle_move and PASS against the fixed one

## Success signals

- on real hardware, a gentle_move into free space returns only after the joint has measurably arrived (final_position read back within tolerance of the goal, taking ~900ms for a 400-tick travel, not 71ms), and a gentle_move that drives the joint into an obstacle stops and holds on the CONTACT ITSELF — not merely on a load that was already present before the probe started
  - instruction: Hardware acceptance, human-gated: (1) free-space move returns in real travel time with a read-back final_position; (2) obstacle move stops and holds on the contact it caused. Write both up in docs/hardware-validation-arm-read-flex.md.
- the test suite can tell the difference: the fake bus models travel latency (the servo does NOT arrive on the write that commands it), so the current early-sampling code FAILS the new tests — a regression test that would have caught this bug on day one
  - instruction: Replace the teleporting fake bus in tests/test_gentle.py with one that models travel: present_position advances toward the goal over successive read_info calls, and load rises only while the joint is actually moving/blocked. Assert the new tests FAIL against the pre-fix loop.

## Scope / boundaries

- not a rewrite of the motion stack: the overload-safety already proven on hardware in t7 (the Torque_Limit cap of 500 for the duration of the move, and the graceful catch of the servo's own error=32 latch via clear_overload) stays exactly as it is. Not a new trajectory planner, not closed-loop control, not a change to the three-mode consent gate.
  - instruction: Leave the Torque_Limit cap (_CONTACT_TORQUE_LIMIT=500), its finally-restore, and the OverloadError/clear_overload path structurally untouched. They are the only safety that has actually been working.

## Non-goals

- not re-running the full 'arm explore' reachability map as part of this work — the map is downstream and stays blocked until the primitive is trustworthy

## Decisions

- USER RULING (q1): contact = load above the joint's threshold AND position failing to advance across N consecutive samples. The stall check is what separates 'pushing against something' from 'merely accelerating' — a transient free-swing load of 272 is ignored because the joint is still advancing. Thresholds are re-tuned as the magnitude gate on top of the stall check, not instead of it. This is also the only rule that survives a joint whose transient load overlaps its blocked load.
  - instruction: Implement contact as: load_magnitude(present_load) > threshold[joint] AND present_position has advanced by less than eps over the last N consecutive samples. N, eps, poll interval and arrival tolerance are fixed empirically during the profiling run (see v2).
- USER RULING (q2): the ~20x slowdown of a genuinely-blocking gentle_move is ACCEPTED. arm explore becomes slow but honest. Whether its 2000-move / 512-tick defaults are still right is a SEPARATE follow-up issue, to be sized from real measured timings once the first honest run exists.
  - instruction: Do NOT re-tune arm explore's --max-moves/--resolution here. File the follow-up issue (see v1) once the first honest run gives real per-probe timings.
- USER RULING (q3): the hardware threshold-profiling run is IN SCOPE for this work — sweep each joint through free space, record its real load profile, derive each threshold from the measured free-motion peak vs blocked load, and verify per joint that a usable band actually exists. Shipping corrected sampling on top of the old bogus-tuned thresholds would false-trigger, so the fix is not trustworthy without it. Requires a hardware session with the user present.
  - instruction: The profiling run is part of THIS work item and needs a hardware session with the user present. Its output is the new DEFAULT_CONTACT_THRESHOLDS table plus the per-joint band evidence.

## Hard questions

- risk: making gentle_move genuinely blocking multiplies every probe's cost by ~20x (40ms of bus-writes becomes ~1s of real travel); 'arm explore' at its default 2000-move budget goes from ~80 seconds to well over half an hour, which may force the explore budget/resolution defaults to be revisited
- risk: the band may NOT exist for some joint — if a light joint's transient acceleration load overlaps the load it shows when blocked, then NO magnitude threshold can separate contact from free motion, and stall-detection (load high AND position not advancing) becomes mandatory rather than optional

## Open / follow-up

- arm explore's --max-moves 2000 / --resolution 512 defaults are almost certainly wrong once each probe really costs ~1s instead of ~40ms; re-size them from measured timings
