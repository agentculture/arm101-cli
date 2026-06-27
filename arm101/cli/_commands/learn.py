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
  arm101-cli calibrate <id>     Record per-joint min/mid/max to a named profile.
  arm101-cli calibrate-motor    Identify one connected motor (read-only); catalog model/gear/joint.
  arm101-cli set-motor-id       Assign a new EEPROM id to the single connected motor (gated write).
  arm101-cli center-motor       Home the single connected motor to 2048 (gated motion).
  arm101-cli setup-motors       Assign per-motor EEPROM id/baudrate (interactive).
  arm101-cli cli overview       Describe the CLI surface itself.

Hardware (SO-101 motor verbs)
-----------------------------
find-port, calibrate, calibrate-motor, set-motor-id, center-motor and
setup-motors drive real Feetech STS3215 servos over a serial bus. Install the
SDK extra to use them: pip install 'arm101-cli[seeed]' (or uv sync --extra
seeed); without it those verbs exit 2 with an install hint. set-motor-id
(EEPROM write), center-motor (motion) and setup-motors are gated and
destructive — they require an interactive terminal and refuse piped stdin.
Run 'explain <verb>' for each verb's contract.

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
            {"path": ["calibrate"], "summary": "Record per-joint min/mid/max to a named profile."},
            {
                "path": ["calibrate-motor"],
                "summary": "Identify one connected motor (read-only); catalog model/gear/joint.",
            },
            {
                "path": ["set-motor-id"],
                "summary": "Assign a new EEPROM id to the single connected motor (gated write).",
            },
            {
                "path": ["center-motor"],
                "summary": "Home the connected motor to 2048 for horn mounting (gated motion).",
            },
            {
                "path": ["setup-motors"],
                "summary": "Assign per-motor EEPROM id/baudrate (interactive).",
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
                "center-motor",
                "setup-motors",
            ],
            "sdk_extra": "pip install 'arm101-cli[seeed]'",
            "note": (
                "Motor verbs drive real Feetech STS3215 servos over a serial bus and "
                "need the [seeed] SDK extra (else exit 2). set-motor-id (EEPROM write), "
                "center-motor (motion) and setup-motors are gated, destructive, and "
                "require an interactive terminal — piped stdin is refused."
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
