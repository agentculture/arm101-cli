# arm101 can read the whole arm's live state and flex it gently: one command streams every joint's position, load, voltage and temperature, and a compliant 'gentle' mode moves the arm while watching motor load so it yields the instant it meets resistance (a touch) instead of pushing through.

> arm101 can read the whole arm's live state and flex it gently: one command streams every joint's position, load, voltage and temperature, and a compliant 'gentle' mode moves the arm while watching motor load so it yields the instant it meets resistance (a touch) instead of pushing through.

## Audience

- An operator or agent driving an assembled SO-101 follower arm over the Feetech STS3215 bus on a single serial port (e.g. /dev/ttyACM1).

## Before → After

- Before: The CLI can only provision motors one at a time (set id, set baud, calibrate, center); there is no whole-arm read, no motion or flex, and no contact sensing. Detection is hardwired to 1 Mbps, so a motor at another baud is invisible and reported as no servo.
- After: arm read returns all six joints live registers (position, load, speed, voltage, temperature, torque flag) with retry-tolerant reads. arm flex <joint> --to <tick> moves one joint to a bounded target. arm flex --demo runs a scripted safe exploration that moves the arm gently around its reachable space. In any motion, --gentle watches present_load and, on contact, backs the joint a few ticks off the contact point then holds, so the arm feels alive and gentle rather than pushing through.

## Why it matters

- Whole-arm read plus safe compliant motion is the foundation every higher-level arm behavior builds on, and load-based touch detection makes motion safe around people and stops the arm pushing through an obstacle.

## Requirements

- Whole-arm read tolerates transient RX timeouts with bounded retries and reports per-joint read health, rather than aborting the whole snapshot on one flaky read.
  - honesty: A single flaky read does not abort the whole-arm snapshot; partial results are clearly marked per joint.
- All motion is torque- and speed-limited and bounded by calibrated joint min/max, safe by default; an explicit flag is required to move.
  - honesty: No motion command can drive a joint past its calibrated min/max or unsafe speed; motion requires an explicit flag.
- Doctor sweeps the supported baud rates and reports which ids answer at which baud, so a baud/id misconfiguration is diagnosed instead of reported as no servo.
  - honesty: doctor distinguishes no-servo-at-any-baud (likely power or data-line) from servo-found-at-unexpected-baud (config), naming the port and baud.
- Gentle mode thresholds contact on present_load (sane default plus --threshold override) and, on contact, backs the joint a bounded number of ticks off the contact point and holds; the back-off magnitude and speed are tuned so motion feels alive and gentle, not abrupt.
  - honesty: present_load rises detectably on contact before mechanical damage, the back-off reliably retreats from the contact, and the response feels gentle (small smooth retreat) rather than a hard stop or violent recoil.

## Honesty conditions

- On healthy hardware, one read command returns all six joints live state and gentle mode demonstrably yields on contact rather than pushing through, both verified on the real arm.
- The intended user really is an operator or agent driving an assembled SO-101 follower over a single Feetech serial bus, not a multi-arm or non-Feetech setup.
- Today the CLI genuinely has no whole-arm read, no motion or flex, and no contact sensing, and motor detection is hardwired to 1 Mbps (verified against the current code).
- Whole-arm read plus safe compliant motion really is the prerequisite for higher-level arm behavior, and load-based touch detection meaningfully improves safety around people.
- IK, trajectory planning, teleoperation, new runtime dependencies, and grasping are genuinely out of scope for this spec and not silently required by the four commands.
- These signals (six joints with live load; gentle backs off within a bounded threshold and latency; doctor names port, ids and baud) are observable and sufficient to call the feature shipped.
- On healthy hardware the described commands (arm read, arm flex per-joint, arm flex --demo, --gentle back-off-then-hold) are all achievable on the existing FeetechBus primitives with no new runtime dependencies.

## Success signals

- On a comms-healthy powered arm, arm read prints six joints with live load values; arm flex --gentle stops within a bounded load threshold and latency of meeting resistance instead of pushing through; and doctor reports a clear hardware-comms diagnosis (which port, which ids answer, at which baud) when the bus is silent.

## Scope / boundaries

- Not inverse kinematics, trajectory planning, or teleoperation; no new third-party runtime dependencies in the introspection core; gripper object-manipulation grasping is a later layer.

## Decisions

- The hardware-comms diagnostic (serial port plus multi-baud id/read probe) lives inside the CLI doctor command as a first-class check, not as throwaway scripts.
- Contact response is back-off-then-hold: reverse a few ticks from the contact point, then hold. Not limp, not a hard freeze. Quality bar: motion feels alive and gentle.
- arm flex ships parametric per-joint move first (arm flex <joint> --to <tick>), then a scripted safe-exploration demo layered on the compliant gentle sensing so it explores without collisions.
