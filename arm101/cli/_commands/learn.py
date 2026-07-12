"""``arm101-cli learn`` — the learnability affordance.

Prints a structured self-teaching prompt. Must satisfy the agent-first rubric:
>=200 chars and mention purpose, command map, exit codes, --json, and explain.

One paragraph here is **rendered from** :mod:`arm101.hardware.arm_spec` rather than
written out: which joints can be re-zeroed, and why the rest cannot. That prose used
to be a hand-typed copy of the table, and when hardware withdrew the table's claim
(issue #43) the copy went on telling operators the retracted version. A doc that
RESTATES a measurement drifts from it; one that RENDERS it cannot. Hence the
concatenation below — ``_TEXT`` cannot be an f-string, because it contains literal
braces (the JSON error shape).
"""

from __future__ import annotations

import argparse

from arm101 import __version__
from arm101.cli._output import emit_result
from arm101.hardware import arm_spec

_TEXT = (
    """\
arm101-cli — a clonable template for AgentCulture mesh agents.

Purpose
-------
Scaffold for a new Culture mesh agent: an agent-first CLI (cited from the teken
`python-cli` reference), an identity (culture.yaml + CLAUDE.md), the canonical
guildmaster skill kit under .claude/skills/, and a deploy/CI baseline. Clone it,
rename the package, and edit culture.yaml to mint a new agent.

Commands
--------
  arm101-cli whoami             Identity from culture.yaml.
  arm101-cli learn              This self-teaching prompt.
  arm101-cli explain <path>...  Markdown docs for any noun/verb path.
  arm101-cli overview           Descriptive snapshot of the agent.
  arm101-cli doctor             Check the agent-identity invariants.
  arm101-cli find-port          List candidate serial ports; --detect resolves by unplug.
  arm101-cli calibrate <id>     Capture per-joint min/mid/max to a named profile
                                (interactive; non-TTY = read-only dry-run preview).
  arm101-cli calibrate-motor    Identify one connected motor (read-only); catalog model/gear/joint.
  arm101-cli set-motor-id       Assign EEPROM id (gated; TTY or agent via --apply).
  arm101-cli set-baudrate       Change EEPROM baud rate, id unchanged (gated; TTY or agent --apply).
  arm101-cli center-motor       Home motor to 2048 (gated; TTY or 2-step agent --apply).
  arm101-cli setup-motors       Assign per-motor EEPROM id/baudrate with per-motor
                                port auto-detect (dry-run / interactive / agent --apply;
                                --baudrate; before/after motor cards)
  arm101-cli arm setup <role>   Set up all 6 motors for follower|leader: assigns EEPROM
                                ids 1–6 at 1 000 000 baud and auto-catalogs F/L motors
                                from arm_spec (gated; dry-run / interactive / agent --apply).
  arm101-cli arm overview       Describe the arm noun surface (roles, joints, motor map).
  arm101-cli arm read           Read every joint's live register state (read-only; no motion).
  arm101-cli arm flex <joint>   Move a joint (--to) or sweep all (--demo); gated motion
                                (--gentle / --threshold; TTY prompt or agent via --apply).
  arm101-cli arm explore        Flood-fill + map the reachable joint-space via the
                                overload-safe gentle move; writes a resumable JSONL
                                event log + a compact, queryable map (--map to
                                resume/override); gated motion (TTY prompt or agent
                                via --apply).
  arm101-cli arm profile <j>    Find the highest speed at which CONTACT DETECTION
                                still works: ramps Goal_Speed and certifies each
                                candidate by driving the joint into a REAL contact
                                (--contact-to, required) and requiring the stall rule
                                to fire. A speed the servo merely survives is a
                                FAILURE, not a pass. Reports the joint's safe speed,
                                ticks/second, and motion-onset latency; gated motion
                                (TTY prompt or agent via --apply).
  arm101-cli arm limits [<j>..] MEASURE each joint's true travel and change nothing.
                                Rolls the encoder seam out of the joint's way, creeps to
                                BOTH ends under contact detection, and rules on what
                                stopped it: WALL / TORQUE_LIMITED / EDGE / TIMEOUT, per
                                END — and only WALL vouches for a limit. MEASURE-ONLY:
                                the borrowed encoder offset is restored and the servo is
                                left exactly as it was found. There is no --commit;
                                keeping a re-zero is a separate, explicitly gated act.
                                Reports the delta between each measured span and the
                                EEPROM-derived span 'arm explore' uses today (issue #34);
                                gated motion (TTY prompt or agent via --apply).
  arm101-cli arm rezero <joint> Shift a joint's encoder zero (Ofs/Homing_Offset, EEPROM
                                addr 31) so the 4095->0 encoder seam falls in the arc the
                                joint cannot reach — the issue-#35 fix, and elbow_flex is
                                the only joint it applies to (every other joint is refused
                                WITH the reason). Commands NO motion. --verify runs the
                                torque-off, hand-driven sweep that proves the seam actually
                                moved (gated; TTY prompt or agent via --apply).
  arm101-cli cli overview       Describe the CLI surface itself.

Hardware (SO-101 motor verbs)
-----------------------------
find-port, calibrate, calibrate-motor, set-motor-id, set-baudrate,
center-motor, setup-motors, arm setup, arm read, arm flex, arm explore,
arm profile, arm limits and arm rezero drive real Feetech STS3215 servos over a
serial bus. Install the SDK
extra to use them: pip install 'arm101-cli[seeed]' (or uv sync --extra seeed);
without it those verbs exit 2 with an install hint. arm read is read-only (no
consent gate): it opens a bus and reads every joint's live state but commands no
motion. arm flex is gated motion (three-mode consent + --apply): it moves one
joint (--to) or sweeps all (--demo), with --gentle/--threshold selecting the
load-watch back-off-then-hold path. arm explore is also gated motion: it
flood-fills the reachable joint-space via the same overload-safe gentle move,
writing a resumable JSONL event log plus a derived compact reachability map
(--map to resume from or override the default path); a bounded multi-joint
escape search finds combination-unblocks rather than stopping at the first
single-joint contact. v1 produces, stores, and lets you query the map;
consuming it to gate arm flex targets is a documented follow-up.
arm profile <joint> is gated motion too, and it exists because arm explore's
probe cost is dominated by travel time while every motion constant in the arm was
hand-fitted in one bench session. It ramps Goal_Speed and certifies each candidate
speed by driving the joint into a REAL contact (--contact-to, required: a tick the
joint genuinely cannot reach) and requiring the shipped stall rule to detect it.
Speed and contact detection are COUPLED — drive fast enough and the joint creeping
into an obstacle no longer reads as stopped, so the rule cannot tell "blocked" from
"accelerating" — so a speed the servo merely SURVIVES is a FAILURE of that speed,
not a pass, and free motion at a speed proves nothing. It reports the joint's
highest safe speed, its measured ticks/second, and its motion-onset latency; the
ramp stops at the first speed that fails, and a --contact-to the joint can actually
reach voids the run (exit 1) rather than certifying a speed against thin air.

arm limits is gated motion that MEASURES and changes nothing. Per joint it opens a
rolling frame — which keeps the encoder seam half a turn ahead of the creep, so a
joint whose travel crosses the seam does not report a 200-tick move as a 3896-tick
retreat — creeps to BOTH ends under contact detection, and rules on what stopped it.
The verdict is carried per END, not per joint: gravity helps a joint down and fights
it up, and present_load SATURATES at the torque cap, so a joint pressed against a
mechanical limit and a joint that has simply run out of torque read EXACTLY the same
at the moment they stop. What separates them is the approach — a real contact's give
is tens of ticks; a gravity climb spends hundreds of ticks above its threshold on the
way to running out. Only WALL vouches for a limit; TORQUE_LIMITED, EDGE and TIMEOUT
are all LOWER BOUNDS, and every gap in the evidence falls that way on purpose (a false
lower bound under-claims the arm's reach and another pose can widen it; a false wall
is permanent). MEASURE-ONLY: the borrowed encoder offset is restored on every exit
path and there is deliberately no --commit — a verb that silently re-calibrated five
joints because you asked it to LOOK at them is not one anybody should run. It also
reports, per joint, the delta between the measured span and the EEPROM-derived span
arm explore builds its grid from today, and it is written to be able to report that
there is NO material difference — which would mean the grid was not being fed
artifacts and the case for blocking issue #34 on this work is false.

arm rezero is a gated EEPROM write that commands NO motion: it shifts a joint's
encoder zero (Ofs/Homing_Offset, addr 31) so the 4095->0 encoder seam falls in
the arc the joint physically cannot reach (issue #35). Which joints, and why not
the others — rendered from arm_spec, so it cannot drift from the table again:

"""
    + arm_spec.REZERO_ARC_UNKNOWN_SUMMARY
    + """

It commands no motion on purpose: elbow_flex
rests PAST its wrap, so a linear goal would rotate it the long way round into a
wall — the tool that makes the axis linear cannot rely on the axis being linear.
--verify is the proof: torque off, a human hand-moves the joint through its whole
travel, and the verb asserts there is no discontinuity anywhere. Reading the
offset back proves only that it was APPLIED; only the sweep proves the seam
MOVED, and a discontinuity under a written offset exits 2 as a stop condition.
calibrate is a profile-write (disk only) verb with a dry-run preview on
non-TTY: TTY captures poses and saves, non-TTY without --apply emits a
read-only preview (no bus, no write), non-TTY with --apply exits 1 (physical
pose capture cannot be automated). set-motor-id (EEPROM id write), set-baudrate
(EEPROM baud write, id unchanged), center-motor (motion) and setup-motors are
gated and destructive — they use the three-mode consent core: (1) TTY prompts
the human; (2) non-TTY without --apply prints a read-only plan (set-motor-id /
set-baudrate: markdown dry-run; center-motor: JSON plan file under
~/.arm101/plans/; setup-motors: 6→1 assignment table); (3) non-TTY with
--apply executes (set-motor-id, set-baudrate and setup-motors are 1-step;
center-motor is 2-step with --plan-hash). Headless writes are attributed
(ARM101_OPERATOR env / culture nick) and appended to ~/.arm101/audit.log. Run
'explain <verb>' for each verb's contract.

Machine-readable output
-----------------------
Every command supports --json. Errors in JSON mode emit
{"code", "message", "remediation"} to stderr. Stdout and stderr never mix.

Exit-code policy
----------------
  0 success
  1 user-input error (bad flag, bad path, missing arg)
  2 environment / setup error
  3+ reserved

More detail
-----------
  arm101-cli explain arm101-cli
"""
)


def _as_json_payload() -> dict[str, object]:
    return {
        "tool": "arm101-cli",
        "version": __version__,
        "purpose": "Clonable scaffold for a new AgentCulture mesh agent.",
        "commands": [
            {"path": ["whoami"], "summary": "Identity probe from culture.yaml."},
            {"path": ["learn"], "summary": "Self-teaching prompt."},
            {"path": ["explain"], "summary": "Markdown docs by path."},
            {"path": ["overview"], "summary": "Descriptive snapshot of the agent."},
            {"path": ["doctor"], "summary": "Check the agent-identity invariants."},
            {
                "path": ["find-port"],
                "summary": "List candidate serial ports; --detect resolves by unplug.",
            },
            {
                "path": ["calibrate"],
                "summary": (
                    "Capture per-joint min/mid/max to a named profile "
                    "(interactive; non-TTY = read-only dry-run preview)."
                ),
            },
            {
                "path": ["calibrate-motor"],
                "summary": "Identify one connected motor (read-only); catalog model/gear/joint.",
            },
            {
                "path": ["set-motor-id"],
                "summary": "Assign EEPROM id (gated; TTY or agent via --apply).",
            },
            {
                "path": ["set-baudrate"],
                "summary": (
                    "Change EEPROM baud rate without reassigning id "
                    "(gated; TTY or agent via --apply)."
                ),
            },
            {
                "path": ["center-motor"],
                "summary": "Home motor to 2048 (gated; TTY or 2-step agent --apply).",
            },
            {
                "path": ["setup-motors"],
                "summary": (
                    "Assign per-motor EEPROM id/baudrate with per-motor port auto-detection "
                    "(dry-run / interactive / agent --apply; --baudrate flag; "
                    "before/after motor cards)."
                ),
            },
            {
                "path": ["arm", "setup"],
                "summary": (
                    "Set up all 6 motors for follower|leader: assigns EEPROM ids 1–6 at "
                    "1 000 000 baud and auto-catalogs F/L motors from arm_spec "
                    "(gated; dry-run / interactive / agent --apply)."
                ),
            },
            {
                "path": ["arm", "overview"],
                "summary": "Describe the arm noun surface (roles, joints, per-role motor map).",
            },
            {
                "path": ["arm", "read"],
                "summary": "Read every joint's live register state (read-only; no motion gate).",
            },
            {
                "path": ["arm", "flex"],
                "summary": (
                    "Move a joint (--to) or sweep all (--demo); gated motion "
                    "(--gentle / --threshold; TTY prompt or agent via --apply)."
                ),
            },
            {
                "path": ["arm", "explore"],
                "summary": (
                    "Flood-fill + map the reachable joint-space via the overload-safe "
                    "gentle move; writes a resumable JSONL log + a compact, queryable "
                    "map (--map to resume/override); gated motion (TTY prompt or agent "
                    "via --apply)."
                ),
            },
            {
                "path": ["arm", "profile"],
                "summary": (
                    "Find the highest speed at which CONTACT DETECTION still works: ramps "
                    "Goal_Speed and certifies each candidate against a REAL contact "
                    "(--contact-to, required), requiring the stall rule to fire. A speed "
                    "the servo merely survives is a FAILURE, not a pass. Reports the "
                    "joint's safe speed, ticks/second, and motion-onset latency; gated "
                    "motion (TTY prompt or agent via --apply)."
                ),
            },
            {
                "path": ["arm", "limits"],
                "summary": (
                    "MEASURE each joint's true travel and change nothing: roll the encoder "
                    "seam out of the way, creep to BOTH ends under contact detection, and "
                    "rule on what stopped it (WALL / TORQUE_LIMITED / EDGE / TIMEOUT, per "
                    "END — only WALL vouches for a limit). MEASURE-ONLY: the borrowed "
                    "encoder offset is restored and there is no --commit. Reports the delta "
                    "against the EEPROM-derived span 'arm explore' uses today (issue #34); "
                    "gated motion (TTY prompt or agent via --apply)."
                ),
            },
            {
                "path": ["arm", "rezero"],
                "summary": (
                    "Shift a joint's encoder zero (Ofs/Homing_Offset, EEPROM addr 31) so "
                    "the 4095->0 encoder seam falls in the arc the joint cannot reach — "
                    "the issue-#35 fix; elbow_flex only, every other joint refused WITH "
                    "the reason. Commands NO motion. --verify runs the torque-off, "
                    "hand-driven sweep that proves the seam moved (gated; TTY prompt or "
                    "agent via --apply)."
                ),
            },
            {"path": ["cli", "overview"], "summary": "Describe the CLI surface."},
        ],
        "exit_codes": {
            "0": "success",
            "1": "user-input error",
            "2": "environment/setup error",
        },
        "hardware": {
            "verbs": [
                "find-port",
                "calibrate",
                "calibrate-motor",
                "set-motor-id",
                "set-baudrate",
                "center-motor",
                "setup-motors",
                "arm setup",
                "arm read",
                "arm flex",
                "arm explore",
                "arm profile",
                "arm limits",
                "arm rezero",
            ],
            "sdk_extra": "pip install 'arm101-cli[seeed]'",
            "note": (
                "Motor verbs drive real Feetech STS3215 servos over a serial bus and "
                "need the [seeed] SDK extra (else exit 2). calibrate is a profile-write "
                "(disk only) verb with a dry-run preview on non-TTY: TTY captures poses "
                "and saves; non-TTY without --apply emits a read-only preview (no bus, "
                "no write); non-TTY with --apply exits 1 (physical pose capture cannot "
                "be automated). set-motor-id (EEPROM id write), set-baudrate (EEPROM "
                "baud write, id unchanged), center-motor (motion), setup-motors, "
                "arm setup, arm flex, arm explore, arm profile and arm rezero are gated, "
                "destructive, and use the three-mode consent core: TTY interactive, "
                "non-TTY dry-run plan, or non-TTY --apply (set-motor-id, set-baudrate, "
                "setup-motors, arm setup, arm flex, arm explore, arm profile and arm "
                "rezero are 1-step; center-motor is 2-step with --plan-hash). arm setup "
                "additionally auto-catalogs F/L motor entries "
                "from arm_spec (servo_model + gear_ratio) after each write. arm read is "
                "the one read-only motor verb (no consent gate): it reads every joint's "
                "live state but commands no motion. arm flex moves one joint (--to) or "
                "sweeps all (--demo), with --gentle/--threshold selecting the "
                "load-watch back-off-then-hold path. arm explore flood-fills the "
                "reachable joint-space via the same overload-safe gentle move, writing "
                "a resumable JSONL event log plus a derived compact reachability map "
                "(--map to resume/override the default path) and running a bounded "
                "multi-joint escape search for combination-unblocks; v1 produces, "
                "stores, and lets you query the map — consuming it to gate arm flex "
                "targets is a documented follow-up. arm profile ramps Goal_Speed for one "
                "joint and certifies each candidate speed by driving the joint into a REAL "
                "contact (--contact-to, required: a tick it genuinely cannot reach) and "
                "requiring the shipped stall rule to detect it — speed and contact "
                "detection are COUPLED, so a speed the servo merely SURVIVES is a failure "
                "of that speed, not a pass, and free motion at a speed proves nothing; it "
                "reports the joint's highest safe speed, its measured ticks/second, and its "
                "motion-onset latency. arm limits MEASURES each joint's true travel and "
                "changes nothing: it rolls the encoder seam out of the joint's way, creeps "
                "to BOTH ends under contact detection, and rules on what stopped it (WALL / "
                "TORQUE_LIMITED / EDGE / TIMEOUT, carried per END — only WALL vouches for a "
                "limit, because present_load SATURATES at the torque cap and a joint pressed "
                "into a wall reads identically to one that has run out of torque). It is "
                "MEASURE-ONLY — the borrowed encoder offset is restored on every exit path "
                "and there is deliberately no --commit — and it reports the delta between "
                "each measured span and the EEPROM-derived span arm explore builds its grid "
                "from today (issue #34). arm rezero is a gated EEPROM write that commands NO "
                "motion: it shifts a joint's encoder zero (addr 31) so the 4095->0 seam "
                "falls in the arc the joint cannot reach (issue #35). Which joints, and why "
                "not the others — rendered from arm_spec so it cannot drift from the table: "
                + arm_spec.REZERO_ARC_UNKNOWN_SUMMARY
                + " --verify is the proof: torque off, a human hand-moves the "
                "joint through its whole travel, and the verb asserts no discontinuity "
                "anywhere — the read-back proves the offset was APPLIED, only the sweep "
                "proves the seam MOVED. Headless writes are attributed "
                "(ARM101_OPERATOR / culture nick) and logged to ~/.arm101/audit.log."
            ),
        },
        "json_support": True,
        "explain_pointer": "arm101-cli explain <path>",
    }


def cmd_learn(args: argparse.Namespace) -> int:
    if getattr(args, "json", False):
        emit_result(_as_json_payload(), json_mode=True)
    else:
        emit_result(_TEXT, json_mode=False)
    return 0


def register(sub: argparse._SubParsersAction) -> None:
    p = sub.add_parser(
        "learn",
        help="Print a structured self-teaching prompt for agent consumers.",
    )
    p.add_argument("--json", action="store_true", help="Emit structured JSON.")
    p.set_defaults(func=cmd_learn)
