# Build Plan — arm101's gentle_move now actually measures the arm: it watches load DURING travel and returns only when the joint has really arrived — so contact detection stops being blind and 'reachable' stops being a guess

slug: `arm101-s-gentle-move-now-actually-measures-the-arm` · status: `exported` · from frame: `arm101-s-gentle-move-now-actually-measures-the-arm`

> arm101's gentle_move now actually measures the arm: it watches load DURING travel and returns only when the joint has really arrived — so contact detection stops being blind and 'reachable' stops being a guess

## Tasks

### t1 — Commit the timing-probe diagnostic that proves the bug

- instruction: New file scripts/probe_gentle_timing.py. Do not touch arm101/ or tests/. Autonomous — no hardware needed to write it; its hardware output is captured in t3's session.
- covers: c3, h6
- acceptance:
  - scripts/probe_gentle_timing.py drives one gentle_move on wrist_roll (400 ticks) and polls present_position+present_load AFTER the call returns
  - re-running it on the PRE-FIX code reproduces the evidence: the call returns in ~70ms while the joint is still at its start, and real travel takes ~900ms
  - its output is pasted into the run-log doc so the diagnosis is reproducible, not anecdotal

### t2 — Replace the teleporting fake bus with one that models travel latency

- instruction: Own tests/test_gentle.py and any new tests/_fakes helper. Do NOT edit arm101/hardware/gentle.py — the tests must fail against the current implementation; that failure IS the deliverable. Autonomous.
- covers: c9, h2, h11
- acceptance:
  - the fake bus no longer sets present_position inside write_goal_position: the servo advances toward the goal across successive read_info calls, and load rises only while it is actually moving or blocked
  - a test asserts final_position is a READ-BACK value: an implementation returning the commanded target fails it
  - the new tests FAIL against the pre-fix gentle_move and are demonstrated to do so (genuine regression tests)
  - the existing 604 tests still pass

### t3 — Record the regression baseline: mid-travel contact goes UNDETECTED on the current code

- instruction: HUMAN-GATED HARDWARE RUN — do not fan out to a subagent. Follower arm on /dev/ttyACM1. Runs against the PRE-FIX code, so it must happen before t4 merges. Writes only to the run-log doc.
- covers: c5, h8
- acceptance:
  - HUMAN-GATED hardware run: a joint is driven into an obstacle mid-travel on the PRE-FIX code and the run-log records that no contact is detected
  - this is the before-picture the fixed code must invert; it verifies the load-watch never worked rather than inferring it

### t4 — Rewrite the gentle_move stepping loop to measure instead of assume

- instruction: Owns arm101/hardware/gentle.py exclusively — the only task in its wave, because it is the only one touching that file. TDD: make t2's failing tests pass without weakening them. N/eps/poll-interval/arrival-tolerance start as documented provisional constants and are finalised by t7 (risk r3).
- depends on: t2
- covers: c4, c10, c11, h7
- acceptance:
  - the loop writes the goal then POLLS bus.read_info(motor) on an interval, tracking present_position and load_magnitude(present_load) DURING travel
  - it terminates on a measured condition only: |present_position - goal| <= tolerance (arrival), a detected contact, or a timeout — never on 'commanded ticks exhausted'
  - contact = load_magnitude(present_load) > threshold[joint] AND present_position advanced < eps across N consecutive samples (the stall rule), so a transient free-swing load like wrist_roll's 272 does NOT trip it
  - final_position, contact_position and contact_load in the result dict are all read-back values, traceable to a bus read taken during or after the motion
  - the Torque_Limit cap (_CONTACT_TORQUE_LIMIT=500), its finally-restore, and the OverloadError/clear_overload path are structurally untouched

### t5 — Guard the t7-proven overload safety against regression

- instruction: New test file (e.g. tests/test_gentle_overload_guard.py). Must NOT edit tests/test_gentle.py (t2 owns it) or arm101/hardware/gentle.py (t4 owns it). Autonomous.
- depends on: t4
- covers: c6, h9
- acceptance:
  - a test asserts the Torque_Limit cap of 500 is applied for the duration of the move and restored in the finally, however the move ends (clean, contact, or overload)
  - a test asserts a servo error=32 latch is still caught and cleared via clear_overload and REPORTED as overloaded=True, never raised
  - lives in its own test file so it does not collide with the fake-bus work in t2

### t6 — Guard every existing caller's contract against the rewrite

- instruction: New test file (e.g. tests/test_gentle_caller_compat.py). Must NOT edit tests/test_gentle.py, the t5 file, or arm101/hardware/gentle.py. Autonomous.
- depends on: t4
- covers: c2, h5
- acceptance:
  - 'arm flex --gentle', 'arm explore' and the demo sweep keep their flags, three-mode consent gating and JSON payload keys unchanged
  - tests assert the JSON keys of each caller's payload are identical pre- and post-rewrite
  - lives in its own test file so it does not collide with t2 or t5

### t7 — Hardware profiling run: re-derive DEFAULT_CONTACT_THRESHOLDS from real load profiles

- instruction: HUMAN-GATED HARDWARE RUN — do not fan out to a subagent. Follower arm on /dev/ttyACM1. Owns arm101/hardware/arm_spec.py (DEFAULT_CONTACT_THRESHOLDS) plus the run-log. If a joint shows no usable band (risk r1), record that explicitly rather than inventing a threshold.
- depends on: t4
- covers: c12, h3
- acceptance:
  - HUMAN-GATED hardware run: each of the 6 joints is swept through free space with its load profile recorded, then pressed into a blocked state
  - each threshold in arm101/hardware/arm_spec.py DEFAULT_CONTACT_THRESHOLDS is derived from the MEASURED free-motion peak vs blocked load — not carried over from the PR #31 values, which were tuned against the bug's near-zero reads
  - a usable band is shown to EXIST for every joint (free-motion peak < blocked load); any joint where it does not is recorded explicitly, and the stall rule from t4 carries the contact decision there
  - the per-joint band evidence is written into the run-log doc

### t8 — Hardware acceptance: measured arrival in free space, stop-and-hold on real contact

- instruction: HUMAN-GATED HARDWARE RUN — do not fan out to a subagent. Final acceptance; writes only docs/hardware-validation-arm-read-flex.md. Its measured per-probe timings are the input to the explore-defaults follow-up issue (risk r2).
- depends on: t5, t6, t7
- covers: c1, c8, h1, h4, h10
- acceptance:
  - HUMAN-GATED hardware run, both halves in one session: (1) a gentle_move into free space returns only after MEASURED arrival — a 400-tick move at speed 150 takes ~900ms and reports a read-back final_position within tolerance, never ~70ms
  - (2) a gentle_move driven into an obstacle stops and holds on the contact IT CAUSED — inverting the t3 baseline, where the same contact went undetected
  - both are written up in docs/hardware-validation-arm-read-flex.md

## Risks

- [unknown_nonblocking] the per-joint load band may NOT exist for some joint: if a light joint's transient acceleration load overlaps the load it shows when blocked, no magnitude threshold can separate contact from free motion and the stall rule alone carries the contact decision (frame q1) (task t7)
- [follow_up] arm explore's --max-moves 2000 / --resolution 512 defaults are almost certainly wrong once each probe really costs ~1s instead of ~40ms — re-size them in a FOLLOW-UP issue from the timings t8 produces (frame v1)
- [unknown_nonblocking] the exact N (consecutive non-advancing samples), eps (minimum tick advance), poll interval and arrival tolerance are hardware constants to be fixed empirically during t7's profiling run, not guessed at implementation time (frame v2) (task t4)
