# Build Plan — arm101 finds its own joint limits. A new gated verb drives each joint outward under contact detection until it meets a real wall or proves it turns freely — rolling the encoder seam out of the way ahead of it, so the seam can never masquerade as a limit again. From that measurement the arm decides per joint whether to evict the seam permanently (re-zero) or fence it off (soft limit), and records the widest envelope it has ever seen as the joint's MECHANICAL limit while keeping the narrower per-pose findings as ENVIRONMENTAL ones.

slug: `arm101-finds-its-own-joint-limits-a-new-gated-verb` · status: `exported` · from frame: `arm101-finds-its-own-joint-limits-a-new-gated-verb`

> arm101 finds its own joint limits. A new gated verb drives each joint outward under contact detection until it meets a real wall or proves it turns freely — rolling the encoder seam out of the way ahead of it, so the seam can never masquerade as a limit again. From that measurement the arm decides per joint whether to evict the seam permanently (re-zero) or fence it off (soft limit). It records the widest envelope it has ever OBSERVED — and says which ends it can actually vouch for, because an end the arm was merely too weak to push past is a lower bound, not a wall.

## Tasks

### t1 — RAW-TICK PERSISTENCE: one conversion boundary between the bus and everything else

- instruction: New module owns reported<->raw conversion (rezero.raw_from_reported already exists — build on it). Convert arm_spec.SOFT_LIMITS to RAW and re-assert _require_dead_arc_contains_seam under the new frame. Files: arm101/hardware/ (new conversion module) + arm_spec.py.
- covers: c2, h7
- acceptance:
  - A single module owns reported<->raw conversion; arm_spec.SOFT_LIMITS is expressed in RAW ticks and wrist_roll's dead arc still contains its seam under the new frame
  - A test enumerates every call site that stores or compares a tick and asserts each holds RAW — falsified by any persisted REPORTED tick
  - Round-trip property: store raw, change the offset, convert back through the NEW offset, and the physical angle is unchanged

### t2 — THE FOUR-VERDICT RECORD: WALL / TORQUE-LIMITED / EDGE / TIMEOUT, carried per END not per joint

- instruction: New pure module: the verdict enum + a per-END limit record. No bus import. Files: arm101/hardware/ (new). This is the type that makes 'a lower bound' structurally different from 'a wall'.
- covers: c9, h14, c12, h17
- acceptance:
  - Each of the four verdicts is produced by its own test — a verdict no test can generate is one the code cannot tell from its neighbours
  - A range whose end is TORQUE-LIMITED cannot be read as a wall: the record makes the distinction structural, not a comment
  - A TORQUE-LIMITED end is never promoted to a mechanical limit on the evidence of poses alone, no matter how many poses are recorded

### t3 — THE CALIBRATION JOURNAL: a temporary offset is a transaction — journal, write, restore

- instruction: New pure module: journal to disk + fsync BEFORE any write_offset. Test the SIGKILL path for real (subprocess + os.kill), not by reasoning about finally-blocks. GOTCHA: bandit B110 fails CI on bare try/except/pass — use contextlib.suppress.
- covers: c17, h3, c18, h4
- acceptance:
  - The journal is durable on disk BEFORE the first write_offset — proven by a bus whose write_offset ALWAYS fails: the journal must already name the original offset
  - SIGKILL the process mid-probe with a temporary offset in force; the NEXT invocation detects the dirty calibration, names the original offset, and restores it before doing anything else
  - The restore survives its own failure — per-motor, independent, contextlib.suppress not bare try/except/pass (bandit B110 fails CI)

### t4 — GUARD TESTS: the seam arithmetic, and the prohibition on writing servo addrs 9/11

- instruction: Tests only. Two files: the seam arithmetic, and a repo-wide assertion that nothing writes servo addr 9 or 11. Do NOT port LeRobot's write_calibration — it writes both, which would clamp goals and narrow the very reachable set this work recovers.
- covers: c3, h8, c20, h19
- acceptance:
  - Unit test: reported 4095 under Ofs=85 is raw 84, one tick below the seam at raw 85
  - A test asserts NO code path in arm101/ writes servo address 9 or 11 — falsified by a single such write, anywhere, for any reason

### t5 — THE ROLLING FRAME: re-centre the joint's current raw position to reported 2048, and again whenever the creep nears the bound

- instruction: New module. offset such that current raw reports as 2048. Re-centre on nearing the bound. Every write journalled first (t3). This is THE mechanism that breaks the chicken-and-egg — get it right and the rest follows.
- depends on: t1, t3
- acceptance:
  - Given any current raw position, the temporary offset places it at reported 2048 — so the seam is half a turn away, the farthest it can be, with ~2048 clear ticks each side
  - A creep that reaches the reported bound RE-CENTRES and continues; total travel is therefore unbounded by the frame
  - Every offset write is journalled first (t3), so a crash mid-roll is recoverable

### t6 — THE CREEP: step outward from where the joint IS, never command an absolute bound; emit a per-end verdict

- instruction: New module. Step outward from the CURRENT position in the CURRENT frame — never compute a target in a stale frame (that bug fired live during the #43 session). gentle_move is the primitive; contact = load high AND stalled. A stall against the torque cap is TORQUE-LIMITED, not WALL.
- depends on: t2, t5
- acceptance:
  - The probe steps outward from the CURRENT position in the CURRENT frame — a target is never computed in a stale frame (the exact bug that fired during the #43 probe session)
  - A stall at high load against the torque cap is reported TORQUE-LIMITED, not WALL; a saturated load against something solid is WALL; an exhausted frame is EDGE; an unreached target is TIMEOUT
  - Displacement accumulates in RAW ticks, so the measurement survives the frame moving underneath it

### t7 — THE CLASSIFIER: BOUNDED or CONTINUOUS, from accumulated raw displacement alone

- instruction: New module, PURE (no bus). Two walls => BOUNDED. A full 4096 raw ticks one way with no wall => CONTINUOUS. A test must assert NO joint name appears in the logic — wrist_roll comes back CONTINUOUS because it IS, not because it is named.
- depends on: t2, t6
- covers: c10, h15, c22, h21
- acceptance:
  - Two walls => BOUNDED (an unreachable arc exists, so a re-zero can evict the seam into it); a full 4096 ticks in one direction with no wall => CONTINUOUS (no offset can ever help it)
  - NO JOINT NAME appears anywhere in the classifier's logic — asserted by a test. wrist_roll must come back CONTINUOUS because it IS, not because it is named
  - Termination is by accumulated raw displacement, never by 'we got back to where we started'

### t8 — MEASURED ARCS: derive an UnreachableArc from live bus readings, and retract the false claim in arm_spec

- instruction: Extend rezero.py + arm_spec.py. Derive the arc from live readings; feed arm_spec's EXISTING _offset_for_seam_at derivation — do not add a second path. ALSO: delete the confident sentence in _REZERO_UNNECESSARY; until the verb measures, the code may only say the arc is UNKNOWN.
- depends on: t1, t7
- covers: c7, h12, c11, h16, c6, h11
- acceptance:
  - A function DERIVES an UnreachableArc from live readings; rezero.require_rezeroable consumes a measured arc, not a literal
  - The measured arc feeds arm_spec's EXISTING derivation (_offset_for_seam_at on the arc midpoint) — no second path. Acid test: change a measured wall and the derived offset moves with it, no other edit, suite stays green
  - arm_spec._REZERO_UNNECESSARY no longer asserts WHICH joints wrap. Until the verb measures, it may only say the arc is UNKNOWN — the code has no right to the confident sentence

### t9 — THE VERB: 'arm limits' — gated, MEASURE-ONLY by default, under torque_guard

- instruction: New verb in arm101/cli/_commands/arm.py. MEASURE-ONLY default: measure, restore the original offset, report. Wrap in torque_guard. CATALOG LOCKSTEP — explain/catalog.py + overview.py _VERBS + learn.py must all be updated or 'teken cli doctor . --strict' fails CI.
- depends on: t6, t7, t8
- covers: c23, h5, c21, h20
- acceptance:
  - The default run measures, RESTORES the original offset, and reports — leaving the servo exactly as it found it. Committing is a separate, explicitly gated act
  - The whole probe runs inside torque_guard (arm101/hardware/safety.py); an exception raised mid-probe leaves EVERY motor it energised with torque released
  - The verb emits per-joint bounds and verdicts ONLY — it does not enqueue cells, score reachability, or emit a map. If it does any of the three it has quietly become #34
  - Catalog lockstep: explain/catalog.py + overview.py _VERBS + learn.py, and 'uv run teken cli doctor . --strict' passes

### t10 — THE COMMIT PATH: the SWEEP is the arbiter, not the offset read-back

- instruction: The sweep is the arbiter, not the offset read-back. Reuse rezero.sweep/analyse_sweep INCLUDING the >=80% coverage rule — that rule is what stopped three empty sweeps being declared a pass on elbow_flex. BOUNDED => re-zero. CONTINUOUS => soft limit. NEVER an EEPROM angle-limit write.
- depends on: t8, t9
- covers: c13, h18
- acceptance:
  - A joint whose offset reads back exactly right but whose torque-off sweep still shows a discontinuity is a FAILURE and is reported as one
  - Reuses rezero.sweep/analyse_sweep INCLUDING its >=80% coverage rule — the rule that stopped three empty sweeps being declared a pass on elbow_flex
  - A BOUNDED joint commits a re-zero; a CONTINUOUS joint commits a SOFT LIMIT whose dead arc contains the seam. Never an EEPROM angle-limit write

### t11 — THE BOUNDS DIFF: measured span vs the EEPROM-derived span arm explore uses today

- instruction: Report the per-joint delta between the measured span and the EEPROM-derived span arm explore uses today. If NO joint differs by >100 ticks, say so plainly: c8's rationale for blocking #34 on this work would be FALSE.
- depends on: t9
- covers: c8, h13
- acceptance:
  - The verb reports, per joint, the delta between its measured span and the EEPROM-derived span
  - If NO joint differs materially (>100 ticks), c8's rationale for blocking #34 on this work is FALSE and the report says so plainly rather than burying it

### t12 — HARDWARE — THE GROUND-TRUTH RUN: the verb must rediscover what we already know

- instruction: HUMAN-GATED HARDWARE. Bolted-down SO-101 follower, /dev/ttyACM1. THIS IS A STOP-GATE: elbow_flex must come back BOUNDED (arc contains its seam) and wrist_roll CONTINUOUS. If either fails, HALT and return to the user — do not proceed to t13.
- depends on: t10, t11
- covers: c1, h1
- acceptance:
  - Run against the two joints whose answers are already established: elbow_flex => BOUNDED with a measured arc containing its seam; wrist_roll => CONTINUOUS, no wall in a full turn
  - If it cannot re-derive BOTH, it is not measuring — it is guessing, and NO verdict it gives on the other four joints may be believed or written anywhere. STOP and return to the user

### t13 — HARDWARE — THE THREE UNMEASURED JOINTS: shoulder_lift, gripper, shoulder_pan

- instruction: HUMAN-GATED HARDWARE. Probe gripper and shoulder_pan first; shoulder_lift LAST and with the operator watching — it carries the whole arm and is where a torque-limited stall will masquerade as a wall. GRIPPER MUST BE EMPTY. If a joint has a WALL at the old bound, c4 collapses for it — report that, do not explain it away.
- depends on: t12
- covers: c4, h9, c5, h10
- acceptance:
  - Each of the three is probed under the rolling frame and real travel is found BEYOND the old bound. If any has a WALL at the old bound, its 'no contact' reading was wrong and c4 collapses for that joint — report it, do not explain it away
  - shoulder_lift's measured travel CONTAINS raw 85 (it sagged through the seam under gravity). If the measurement disagrees, TRUST THE MEASUREMENT and retract c5
  - shoulder_lift is probed LAST and with the operator watching — it carries the whole arm, and it is the joint where a torque-limited stall will masquerade as a wall

### t14 — HARDWARE — COMMIT + THE MONOTONIC SWEEPS

- instruction: HUMAN-GATED HARDWARE. Torque-off sweep per re-zeroed joint: MONOTONIC, 0 discontinuities. Then CUT AND RESTORE BUS POWER and re-read the offset cold — PR #21 exists because an EEPROM write once read back perfectly and silently reverted on the next power-up.
- depends on: t13
- covers: h2
- acceptance:
  - For every joint re-zeroed, a torque-off sweep across its full permitted travel reports MONOTONIC with ZERO discontinuities against the 500-tick threshold
  - The offset survives a POWER CYCLE, read back cold — the PR #21 lock dance held

### t15 — HARDWARE — THE ACCEPTANCE RUN + run-log

- instruction: HUMAN-GATED HARDWARE. Check all five numbers ON THE ARM and transcribe them into docs/ as a run-log, as was done for the elbow_flex re-zero.
- depends on: t14, t11
- covers: c27, h22
- acceptance:
  - All five numbers checked on the ARM, not in software: 6/6 joints measured (0 from a literal); both known answers re-derived with 0 special cases; >=80% sweep coverage with 0 discontinuities on every re-zeroed joint; >=1 joint's span differs from EEPROM by >100 ticks; 0 writes to addr 9 or 11
  - The five numbers are transcribed into a run-log doc, as was done for the elbow_flex re-zero

## Risks

- [unknown_nonblocking] SHOULDER_LIFT IS THE DANGEROUS ONE. It carries the whole arm, it is one of the three joints with unmeasured travel across the seam, AND it is exactly where a torque-limited stall will masquerade as a wall. Probe it LAST, with the operator watching. A wrong verdict here writes a permanent lie into arm_spec. (task t13)
- [unknown_nonblocking] THERMAL may be the real budget, not moves. 5 joints x 2 directions x N poses is a lot of travel; the first honest explore run hit 50 C in 25 minutes. If the ceiling binds first, the unit of work is DUTY CYCLE and the verb must be interruptible and resumable across sittings, not merely fast.
- [unknown_nonblocking] A NARROW ARC may be evictable in theory but not in practice. arm_spec keeps _ARC_MARGIN_TICKS=100 clearance per side because a tight arc is what broke the first elbow_flex attempt. A joint whose arc is narrower than the margins can carry should be treated as EFFECTIVELY CONTINUOUS and soft-limited, not re-zeroed into a sliver. Where that cutoff sits is unknown. (task t7)
- [unknown_nonblocking] WHICH POSES to probe in, to promote an environmental limit to a mechanical one. More poses = a tighter bound on the truth, but each costs travel and heat — and setting up a pose means moving OTHER joints whose bounds are exactly what is in question. Pose selection and bootstrapping order are unresolved; v1 may ship single-pose (environmental only) and defer promotion.
- [unknown_nonblocking] THE GRIPPER MUST BE PROBED EMPTY. Its closing travel ends wherever the object it is holding ends. A gripper limit measured while gripping is environmental in the strongest sense and would silently become the mechanical limit if recorded naively. (task t13)
- [unknown_nonblocking] STOP-GATE (encoded in t12's acceptance criteria, not an open unknown): if the verb cannot re-derive elbow_flex=BOUNDED and wrist_roll=CONTINUOUS from measurement alone, it is guessing. t13/t14/t15 HALT and the run returns to the user — no verdict on the four unknown joints may be written anywhere. The plan is buildable; the gate fires at RUN time, which is why it does not block convergence. (task t12)
