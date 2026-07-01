# Build Plan — arm101 arm motion is overload-safe: it never trips the STS3215 hardware overload latch during normal moves, and when a joint does overload it reports and recovers gracefully instead of crashing

slug: `arm101-arm-motion-is-overload-safe-it-never-trips` · status: `exported` · from frame: `arm101-arm-motion-is-overload-safe-it-never-trips`

> arm101 arm motion is overload-safe: it never trips the STS3215 hardware overload latch during normal moves, and when a joint does overload it reports and recovers gracefully instead of crashing

## Tasks

### t1 — Bus-layer overload + torque-limit support (bus.py)

- covers: c3, c6, h9
- acceptance:
  - an is_overload(error_byte) helper (or typed OverloadError) classifies STS3215 status bit5 (0x20) as overload; FeetechBus gains read_torque_limit/write_torque_limit at RAM addr 48 and a clear_overload(motor) that disables torque (addr40=0) to clear the latch
  - FakeBus can be configured to surface error=32 on the Nth read/write op (test seam) and to round-trip torque_limit; a unit test asserts NO write ever targets the EEPROM protection registers (addr 34/35/36)

### t2 — gentle.py: gentler default + graceful overload + torque-cap

- depends on: t1
- covers: c8, c9, c10, c5, h4, h8
- acceptance:
  - gentle_move lowers its default goal-speed to a conservative value (<=150) and keeps stepping; on a mid-move error=32 it stops, releases/retreats, and returns overloaded=True in its result dict WITHOUT raising a raw read error
  - gentle_move caps RAM Torque_Limit(48) below the trip at the start of a contact move and restores it to 1000 in a finally (read-back asserted); unit tests via a FakeBus that raises error=32 mid-move cover both the graceful-return and the cap-and-restore paths

### t3 — motion.py: gentler default + graceful overload (compliant_move)

- depends on: t1
- covers: c8, c10
- acceptance:
  - compliant_move lowers its default goal-speed to match gentle_move's conservative default and, on a mid-move error=32, returns overloaded=True instead of raising; covered by a dedicated FakeBus test module

### t4 — demo.py: propagate graceful overload through the sweep

- depends on: t1, t2
- covers: c4, c10
- acceptance:
  - demo_sweep reports a joint overload (overloaded=True on that joint's report + aborted_on_overload) and stops the sweep cleanly with no exception; dedicated FakeBus test asserts the joint after the overloaded one is never touched

### t5 — arm.py CLI surfacing of overload (read/flex/demo)

- depends on: t1, t2
- covers: c2, c4, h5, h7
- acceptance:
  - arm flex/read/demo emit an 'overloaded' field in --json output and map an overload to the structured contact/env path (documented exit code), never a raw traceback; a non-TTY 'arm flex --apply --json' against a FakeBus overload yields a consumable result with overloaded=true

### t6 — Docs: t9 run-log before-state + after-fix section

- covers: h6
- acceptance:
  - docs/hardware-validation-arm-read-flex.md records both overload reproductions (before-state, already committed) and reserves an after-fix results section for the t9 re-run

### t7 — HARDWARE re-run of t9 on the physical follower (human-gated)

- depends on: t2, t3, t4, t5
- covers: c1, c7, h1, h2, h3, h7, h8, h10
- acceptance:
  - on the follower: the all-joint wake-up completes trip-free; a rigid-stop contact stalls WITHOUT tripping error=32 and Torque_Limit reads back 1000 after; an intentional overload self-clears with no manual torque-disable/power-cycle; results appended to the run-log; coordinated move + rigid stop both finish with no latched crash

## Risks

- [unknown_nonblocking] the hardware-verified honesty conditions (h1/h2/h3/h7/h8/h10, task t7) require a powered physical follower and a human operator; they cannot be closed by CI — human-gated sign-off (task t7)
