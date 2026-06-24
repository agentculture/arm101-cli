# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo actually is right now

The package name and description say "Agent and CLI for controlling SO-ARM101
robotic arm grippers," but **no arm/gripper control code exists yet.** What is
here is the AgentCulture **mesh-agent scaffold** (cloned from
`culture-agent-template`): an agent-first introspection CLI, a mesh identity, the
vendored guildmaster skill kit, and a build/CI/deploy baseline. The gripper
domain is the *destination*; treat the current CLI as the chassis you extend to
get there, following the patterns below.

Two consequences worth internalizing before you touch anything:

- **The runtime agent prompt is `AGENTS.colleague.md`, not this file.**
  `culture.yaml` declares `backend: colleague`, and the backend→prompt-file map
  (see `doctor` below) resolves `colleague` → `AGENTS.colleague.md`. This
  `CLAUDE.md` is guidance for *Claude Code working in the repo*; editing it does
  not change the mesh agent's runtime behavior. (Note: the pre-`/init` seed text
  and some README prose still say `backend: claude` / `CLAUDE.md` — that is stale;
  the CHANGELOG `0.3.0` entry records the promotion to `colleague`.)
- **The installed console script is `arm101`, not `arm101-cli`.**
  `[project.scripts]` defines `arm101 = "arm101.cli:main"`. The README quickstart
  (`uv run arm101-cli whoami`) is wrong and will fail with "Failed to spawn." Use
  `uv run arm101 …` or `python -m arm101 …`. The *internal* prog name is still
  `arm101-cli`, so `--help` text, error messages, and JSON payloads all say
  `arm101-cli` — that string is intentional in output, just not as the binary.

## Common commands

```bash
uv sync                                      # create .venv, install runtime + dev deps
uv run pytest -n auto                         # full test suite (xdist parallel)
uv run pytest tests/test_cli.py::test_whoami_text   # a single test
uv run pytest -n auto --cov=arm101 --cov-report=term # tests with coverage (CI gate: fail_under=60)
uv run arm101 whoami                          # run the CLI (note: 'arm101', not 'arm101-cli')

# Lint — CI runs all of these; run them before opening a PR
uv run black --check arm101 tests
uv run isort --check-only arm101 tests
uv run flake8 arm101 tests
uv run bandit -c pyproject.toml -r arm101
markdownlint-cli2 "**/*.md" "#node_modules" "#.local" "#.claude/skills" "#.teken"

uv run teken cli doctor . --strict            # the agent-first rubric gate (see below)
```

The runtime package has **zero third-party dependencies** (`dependencies = []`),
on purpose. `teken` and the test/lint tools are dev-only. Keep it that way unless
adding the gripper layer genuinely requires a hardware library — and if it does,
isolate that dependency so the introspection CLI still imports clean.

## CLI architecture

Everything hangs off `arm101/cli/__init__.py:main()`. The shape is worth
understanding before adding commands, because three cross-cutting contracts are
enforced by tests and by the rubric gate.

**Registration pattern.** `_build_parser()` imports each module under
`arm101/cli/_commands/` and calls its `register(sub)` function, which adds a
subparser and wires `func` + `--json` via `set_defaults`. To add a global verb:
write `_commands/<verb>.py` with a `register()`, then add one import + call in
`_build_parser()`. To add a **noun group** (a subcommand with its own verbs —
this is how gripper control will likely land, e.g. `arm101 gripper open`), mirror
`_commands/cli.py`: create the noun's subparsers with
`parser_class=type(p)` so child parse errors route through the structured error
contract instead of argparse's default `exit 2`.

**Error contract** (`_errors.py` + the `_dispatch`/`_CliArgumentParser` plumbing).
Every failure raises `CliError(code, message, remediation)`; `main()` catches it
and renders via `_output.emit_error`. Any *other* exception is wrapped so no
Python traceback ever leaks to stderr. Argparse-level errors (unknown verb,
missing arg) are also captured: `_CliArgumentParser.error()` emits the structured
form. Because parse errors happen before `args.json` exists, `main()` pre-scans
raw argv for `--json` and stashes it on the class-level `_CliArgumentParser._json_hint`.
**Handlers must raise `CliError` on failure — never `sys.exit`, never a bare
print-and-return.**

**Output contract** (`_output.py`). Results → stdout, errors/diagnostics →
stderr, **never mixed**; JSON mode keeps the same split. Every command takes
`--json`. Exit codes: `0` success, `1` user-input error, `2` environment error,
`3+` reserved (constants in `_errors.py`). Use `emit_result` / `emit_error` /
`emit_diagnostic` rather than calling `print`.

**The explain catalog** (`arm101/explain/catalog.py`). `ENTRIES` is a dict keyed
by command-path tuples (`("whoami",)`, `("cli", "overview")`, `()` and
`("arm101-cli",)` both = root). `explain` resolves a path to verbatim markdown.
The test `test_every_catalog_path_resolves` asserts every entry renders, but
nothing forces a *new* verb to have one — so when you add a verb, you must update
**three places in lockstep** or the docs silently drift: the catalog entry, the
`_VERBS` list in `overview.py`, and the `_TEXT`/`_as_json_payload` blocks in
`learn.py`.

**Identity reading** (`whoami.py`). `culture.yaml` is parsed by hand (no YAML
dependency, to keep runtime deps empty) — only the documented flat
`suffix`/`backend`/`model` shape is understood; anything fancier falls back to
defaults. `find_culture_yaml()` walks up from `__file__`, so identity is the
agent's own even when invoked from another directory; a wheel install (no
`culture.yaml` alongside the package) falls back to literal defaults and `doctor`
reports a single info check.

## The agent-first rubric (why some code looks the way it does)

`teken cli doctor . --strict` enforces a seven-bundle "agent-first" rubric in CI,
and several otherwise-odd shapes exist only to satisfy it — don't "simplify" them
away:

- `learn` must be ≥200 chars and mention purpose, command map, exit codes,
  `--json`, and `explain`.
- Any noun with action-verbs must also expose `overview` — that's the entire
  reason the `cli` noun group exists (`cli overview` describes the CLI surface,
  distinct from the global `overview`, which describes the *agent*).
- Descriptive verbs must never hard-fail on a bad path — hence `overview` accepts
  an ignored positional `target` and still exits 0.

This is separate from the in-package `arm101 doctor`, which checks **agent-identity
invariants**: `prompt-file-present` and `backend-consistency` (the
`backend → prompt file` map is `claude`→`CLAUDE.md`, `colleague`→`AGENTS.colleague.md`,
`acp`→`AGENTS.md`, `gemini`→`GEMINI.md`), plus `skills-present`. If you change the
backend in `culture.yaml`, teach `doctor` the matching prompt file or
`test_doctor_recognizes_declared_backend` fails.

## AgentCulture conventions that gate CI

- **Bump the version on every PR — even docs/config/CI-only changes.** The
  `version-check` job in `.github/workflows/tests.yml` fails the PR if
  `pyproject.toml`'s version equals `main`'s. Use the `version-bump` skill (updates
  `pyproject.toml` + prepends a Keep-a-Changelog entry to `CHANGELOG.md`).
- **PR lifecycle goes through the `cicd` skill**, which delegates to `devex pr` and
  adds `status` (SonarCloud quality gate) and `await` (blocks until green / threads
  resolved). PR comments auto-sign as `- arm101-cli (Claude)` via the skill's
  `_resolve-nick.sh` (resolved from `culture.yaml`); don't hand-sign inside `cicd`.
- **SonarCloud gates the `test` job** when `SONAR_TOKEN` is set
  (`sonar.qualitygate.wait=true`). Token-less repos and fork PRs skip the scan and
  stay green. `coverage.run.relative_files = true` is load-bearing — without it
  `coverage.xml` paths don't map to `sonar.sources=arm101` and coverage reports 0%.
- **PyPI publish is Trusted Publishing via OIDC** (`.github/workflows/publish.yml`):
  push to `main` → PyPI; PR (same-repo) → a `.devN` build to TestPyPI. No tokens.

## The vendored skill kit — cite-don't-import

`.claude/skills/` holds skills **vendored** from `guildmaster` (a few from
`colleague`/`devague`); `docs/skill-sources.md` is the authoritative provenance
ledger with the per-skill re-sync procedure. **Do not hand-edit skill script
bodies** — the only sanctioned local edits are (a) consumer-identifying prose in
`SKILL.md` and (b) adding `type: command` to frontmatter (load-bearing: the
culture backend's `core.skill_loader` silently skips any `SKILL.md` lacking it).
Two tracked divergences from "always cite guildmaster" are documented in the
ledger: the `agex`→`devex` rename and `outsource`→`ask-colleague` (vendored
directly from `colleague` until guildmaster re-broadcasts). Reach for
`ask-colleague` reflexively for a diverse second opinion — `review`/`explore` are
read-only and always safe; side-effecting `write --apply`/`--pr` needs the user's
go-ahead.

## Conventions and workflow

**Memory discipline — recall before, remember after.** This repo keeps its
eidetic memory **in-repo and public**: records resolve to
`<repo-root>/.eidetic/memory` — committed, and shared with the team and mesh
peers (the `claude` and `colleague` backends both read the same
`arm101-cli` scope), so memory travels with the repo, not a private
home-dir store. Make it a per-task habit:

- **`/recall` before you start.** Search the store for the area you're about
  to touch — prior decisions, gotchas, "have we done this before?" — so you
  build on what's already known instead of re-deriving it. Do this before
  non-trivial tasks, not just when asked.
- **`/remember` when something worth keeping surfaces.** A non-obvious
  decision and its rationale, a constraint, a fix and *why* it was needed, a
  gotcha that cost time, a fact the next session would otherwise re-learn.
  Capture it as it happens, not at the end when it's faded.

A plain `/remember` lands the note in `./.eidetic/memory` in this repo — no
flag needed (the wrappers here default to `--visibility public`; in-repo
routing needs `eidetic >= 0.10.0`, older CLIs keep records in `$HOME`). Keep
something out of the committed store only by passing `--visibility private`
(routes to `$HOME/.eidetic/memory`, never committed); `/recall` reads both
stores and merges. Don't store what the repo already records (code structure,
git history, what's already in this file or `CHANGELOG.md`) — store what you'd
have to re-derive. These are the `recall`/`remember` skills (`.claude/skills/`),
backed by the `eidetic` store.
