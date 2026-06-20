"""Markdown catalog for ``arm101-cli explain <path>``.

Each entry is verbatim markdown. Keys are command-path tuples. The empty tuple
resolves to the root entry, as do both names the CLI answers to: the console
script ``("arm101",)`` (from ``[project.scripts]``) and the internal prog name
``("arm101-cli",)``. The script-name key is load-bearing — the agent-first
rubric's ``explain_self`` check runs ``explain <project-script-name>``.

Keep bodies self-contained: an agent reading one entry should get enough
context without chaining reads.
"""

from __future__ import annotations

_ROOT = """\
# arm101-cli

A clonable template for AgentCulture mesh agents. It carries an agent-first CLI
(cited from the teken `python-cli` reference), a mesh identity (`culture.yaml` +
`CLAUDE.md`), the canonical guildmaster skill kit under `.claude/skills/`, and a
buildable/deployable package baseline. Clone it, rename the package, edit
`culture.yaml`, and you have a new agent.

## Verbs

- `arm101-cli whoami` — identity probe from `culture.yaml`.
- `arm101-cli learn` — structured self-teaching prompt.
- `arm101-cli explain <path>` — markdown docs for any noun/verb.
- `arm101-cli overview` — descriptive snapshot of the agent.
- `arm101-cli doctor` — check the agent-identity invariants.
- `arm101-cli cli overview` — describe the CLI surface.

## Exit-code policy

- `0` success
- `1` user-input error
- `2` environment / setup error
- `3+` reserved

## See also

- `arm101-cli explain whoami`
- `arm101-cli explain doctor`
"""

_WHOAMI = """\
# arm101-cli whoami

Reports the agent's identity from `culture.yaml`: nick (`suffix`), backend,
served model, and the package version. Read-only.

## Usage

    arm101-cli whoami
    arm101-cli whoami --json
"""

_LEARN = """\
# arm101-cli learn

Prints a structured self-teaching prompt covering purpose, command map,
exit-code policy, `--json` support, and the `explain` pointer.

## Usage

    arm101-cli learn
    arm101-cli learn --json
"""

_EXPLAIN = """\
# arm101-cli explain <path>

Prints markdown documentation for any noun/verb path. Unlike `--help` (terse,
positional), `explain` is global and addressable by path.

## Usage

    arm101-cli explain arm101-cli
    arm101-cli explain whoami
    arm101-cli explain --json <path>
"""

_OVERVIEW = """\
# arm101-cli overview

Read-only descriptive snapshot of the agent: identity (from `culture.yaml`), the
verb surface, and the sibling-pattern artifacts the template carries. Accepts an
ignored `target` so a stray path never hard-fails.

## Usage

    arm101-cli overview
    arm101-cli overview --json
"""

_DOCTOR = """\
# arm101-cli doctor

Checks the agent-identity invariants `steward doctor` verifies:
prompt-file-present and backend-consistency (`claude` → `CLAUDE.md`), plus a
skills-present check. Exits 1 when unhealthy.

## Usage

    arm101-cli doctor
    arm101-cli doctor --json
"""

_CLI = """\
# arm101-cli cli

Noun group for CLI-surface introspection. `cli overview` describes the CLI
itself (distinct from the global `overview`, which describes the agent).

## Usage

    arm101-cli cli overview
    arm101-cli cli overview --json
"""


ENTRIES: dict[tuple[str, ...], str] = {
    (): _ROOT,
    ("arm101",): _ROOT,
    ("arm101-cli",): _ROOT,
    ("whoami",): _WHOAMI,
    ("learn",): _LEARN,
    ("explain",): _EXPLAIN,
    ("overview",): _OVERVIEW,
    ("doctor",): _DOCTOR,
    ("cli",): _CLI,
    ("cli", "overview"): _CLI,
}
