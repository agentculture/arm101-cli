# arm101 arm motion is overload-safe: it never trips the STS3215 hardware overload latch during normal moves, and when a joint does overload it reports and recovers gracefully instead of crashing

> arm101 arm motion is overload-safe: it never trips the STS3215 hardware overload latch during normal moves, and when a joint does overload it reports and recovers gracefully instead of crashing

## Audience

- the arm101 operator and the mesh agent that call arm read/flex/(move) on an SO-101 follower — both need motion that won't latch a joint or crash the CLI mid-task

## Before → After

- Before: at goal-speed 400 a dynamic move (one-shot goal accelerating arm mass, or driving into a rigid stop) spikes current past the servo's 80% overload threshold; the STS3215 latches error=32, read_info raises CliError, and 'arm flex' exits 2 with a raw env error leaving the joint limp/latched
- After: arm flex/demo/read and any coordinated move never crash on overload: normal gentle moves stay below the 80% trip, and a joint that does trip error=32 is reported (overloaded=true) and auto-recovered (torque released so the latch clears) instead of raising a raw read error

## Why it matters

- an overload that crashes the CLI leaves a joint latched and the operator staring at a cryptic error=32 traceback; graceful, trip-resistant motion is table stakes for trusting the arm to move autonomously on the mesh

## Requirements

- gentler motion defaults: the motion primitives use a conservative default goal-speed/acceleration and advance the goal in small steps/interpolated frames (never a one-shot far goal at speed 400), keeping output torque under the 80% overload threshold for ordinary moves
  - honesty: with the new gentle defaults, the all-joint wake-up move that tripped shoulder_lift today completes with zero error=32 trips on the physical follower
- torque-limit capping: during compliant/contact moves the primitive lowers the RAM Torque_Limit (addr 48) below the overload threshold so driving into a stop stalls softly instead of tripping, and restores it after (RAM-only, resets on power-cycle, no EEPROM Lock dance)
  - honesty: with Torque_Limit (addr48) capped during the move, driving a joint into a rigidly-held stop stalls WITHOUT tripping error=32 — verified on hardware; and the cap is restored to 1000 afterward (verified by a read-back)
- graceful overload handling: gentle_move, demo_sweep, and arm read/flex treat a mid-operation STS3215 error=32 as a contact/fault event — release torque (clearing the latch) or retreat, set overloaded=true in the result, and return via the structured contact/env path, never a raw read-error traceback
  - honesty: a FakeBus that raises error=32 mid-move drives gentle_move to return overloaded=true with torque released (latch cleared) and NO raw traceback — asserted by a unit test; and the same graceful path is confirmed once on the physical follower

## Honesty conditions

- on the physical follower, the t9 rigid-stop contact move AND a coordinated all-joint move both complete with no latched error=32 CLI crash (verified in a hardware run-log)
- a non-TTY mesh agent calling 'arm flex --apply --json' gets a structured result carrying an 'overloaded' field on a trip (consumable outcome), not a crash/traceback
- today's t9 hardware run-log documents BOTH overload reproductions (shoulder_lift one-shot @speed400; gripper rigid-stop @speed400) — the before-state is a recorded fact, not hypothetical
- an intentional overload on the follower is reported overloaded=true with torque released, and the CLI returns via the structured contact/env path (never a raw read-error traceback)
- after the fix, recovering from an overload needs NO manual torque-disable or power-cycle — the CLI self-clears the latch by releasing torque (shown on hardware)
- no motion path writes ANY EEPROM protection register (Overload_Torque/Protection_Time/Protective_Torque) — verified by a test asserting the FakeBus records no such write, plus code review
- a committed hardware run-log shows the all-joint wake-up move and a rigid-stop contact both finishing with no latched error=32 crash

## Success signals

- on the physical follower: the t9 all-joint wake-up move and a rigid-stop contact both finish with no latched error=32 CLI crash — either no trip, or a trip caught+reported+recovered

## Scope / boundaries

- NOT rewriting servo EEPROM protection registers (Overload_Torque/Protection_Time/Protective_Torque), NOT a trajectory planner or IK, NOT multi-arm; scope is the single follower bus, the existing motion primitives, and their overload behavior

## Open / follow-up

- a full coordinated 'move --joints ... --time 800ms' verb using GroupSyncWrite + time-based interpolation (the sketched multi-joint motion API) — proven technique today but a larger surface; solve overload on existing primitives first
