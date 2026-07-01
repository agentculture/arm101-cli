"""``arm101-cli learn`` — the learnability affordance.

Prints a structured self-teaching prompt. Must satisfy the agent-first rubric:
>=200 chars and mention purpose, command map, exit codes, --json, and explain.
"""

from __future__ import annotations

import argparse

from arm101 import __version__
from arm101.cli._output import emit_result

_TEXT = """\
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
  arm101-cli cli overview       Describe the CLI surface itself.

Hardware (SO-101 motor verbs)
-----------------------------
find-port, calibrate, calibrate-motor, set-motor-id, set-baudrate,
center-motor, setup-motors, arm setup, arm read, arm flex and arm explore drive
real Feetech STS3215 servos over a serial bus. Install the SDK extra to use
them: pip install 'arm101-cli[seeed]' (or uv sync --extra seeed); without it
those verbs exit 2 with an install hint. arm read is read-only (no consent
gate): it opens a bus and reads every joint's live state but commands no
motion. arm flex is gated motion (three-mode consent + --apply): it moves one
joint (--to) or sweeps all (--demo), with --gentle/--threshold selecting the
load-watch back-off-then-hold path. arm explore is also gated motion: it
flood-fills the reachable joint-space via the same overload-safe gentle move,
writing a resumable JSONL event log plus a derived compact reachability map
(--map to resume from or override the default path); a bounded multi-joint
escape search finds combination-unblocks rather than stopping at the first
single-joint contact. v1 produces, stores, and lets you query the map;
consuming it to gate arm flex targets is a documented follow-up.
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
                "arm setup, arm flex and arm explore are gated, destructive, and use the "
                "three-mode consent core: TTY interactive, non-TTY dry-run plan, or "
                "non-TTY --apply (set-motor-id, set-baudrate, setup-motors, arm setup, "
                "arm flex and arm explore are 1-step; center-motor is 2-step with "
                "--plan-hash). arm setup additionally auto-catalogs F/L motor entries "
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
                "targets is a documented follow-up. Headless writes are attributed "
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
