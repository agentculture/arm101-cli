# arm101-cli

Agent and CLI for controlling SO-ARM101 robotic arm grippers

## What you get

- **An agent-first CLI** cited from [teken](https://github.com/agentculture/teken)
  (`afi-cli`) — the runtime package has no third-party dependencies.
- **A mesh identity** — `culture.yaml` (`suffix` + `backend`) and the matching
  prompt file (`AGENTS.colleague.md` for this agent's `backend: colleague`).
- **The canonical guildmaster skill kit** (11 skills) under `.claude/skills/`,
  vendored cite-don't-import. See [`docs/skill-sources.md`](docs/skill-sources.md).
- **A build + deploy baseline** — pytest, lint, the agent-first rubric gate, and
  PyPI Trusted Publishing wired into GitHub Actions.

## Installation

Base install (zero third-party runtime dependencies — introspection only):

```bash
uv sync          # or: pip install arm101-cli
```

For real Feetech STS3215 motor I/O on the Seeed Studio SO-101 kit, add the
`[seeed]` extra (named by the kit provider; the CLI verifies each connected
motor really is a Feetech STS3215 at runtime):

```bash
uv sync --extra seeed          # or: pip install 'arm101-cli[seeed]'
```

This pulls in `feetech-servo-sdk` (import module `scservo_sdk`), which the bus
adapter lazy-imports. The base install stays zero-dep so the CLI and agent
introspection work on any machine without hardware attached.

## Quickstart

```bash
uv sync
uv run pytest -n auto                 # run the test suite
uv run arm101 whoami                 # identity from culture.yaml (console script is 'arm101')
uv run arm101 learn                  # self-teaching prompt (add --json)
uv run teken cli doctor . --strict    # the agent-first rubric gate CI runs
```

## CLI

| Verb | What it does |
|------|--------------|
| `whoami` | Report this agent's nick, version, backend, and model from `culture.yaml`. |
| `learn` | Print a structured self-teaching prompt. |
| `explain <path>` | Markdown docs for any noun/verb path. |
| `overview` | Read-only descriptive snapshot of the agent. |
| `doctor` | Check the agent-identity invariants (prompt-file-present, backend-consistency). |
| `cli overview` | Describe the CLI surface itself. |
| `arm overview` / `arm read` / `arm flex` / `arm explore` / `arm setup <role>` | Arm-level operations on the SO-101 — see below. |

Every command supports `--json`. Results go to stdout, errors/diagnostics to
stderr (never mixed). Exit codes: `0` success, `1` user error, `2` environment
error, `3+` reserved.

## Arm verbs

The `arm` noun group carries the SO-101 hardware surface: `arm overview`
(read-only spec snapshot), `arm read` (read-only live joint state), `arm flex`
(gated single-joint/demo motion), `arm explore` (gated reachability mapping —
documented below), and `arm setup <role>` (gated per-role motor bring-up).
`arm flex`/`arm explore`/`arm setup` are gated motion: a TTY prompts for
confirmation, a non-TTY run without `--apply` prints a dry-run plan (zero
motion, zero bus access), and non-TTY with `--apply` proceeds (agent mode).

### `arm explore` — map the arm's reachable joint-space

Before this verb, nothing persisted where the arm can safely move: limits
were found by manually flexing one joint at a time and eyeballing contacts,
and that knowledge was lost the moment the session ended. `arm explore`
autonomously flood-fills the follower's (or leader's) joint-space at safe
speed — via the same overload-safe `gentle_move` primitive `arm flex
--gentle` uses — detects every self/environment contact from real load, and
writes a resumable, overridable reachability map.

```bash
uv run arm101 arm explore --apply                       # follower, defaults
uv run arm101 arm explore --role leader --map ./bench-a.map.json --apply
uv run arm101 arm explore --threshold 300 --max-moves 500 --apply
```

Flags: `--role {follower,leader}` (default `follower`), `--port` (default
auto-detect), `--map PATH` (resume input if it exists, and the write target;
default `./arm-explore-<role>.map.json`), `--threshold` (contact-load
threshold, default `250`), `--max-moves` (move/probe budget cap, default
`2000` — hardware-tuned open question), `--resolution` (per-joint grid
bucket size in encoder ticks, default `512` — hardware-tuned open question),
`--apply`, `--json`.

Every run writes **two artifacts**: an append-only JSONL event log (the
resumable source of truth — a killed run resumes instead of re-probing
already-mapped cells) and a derived, compact reachability map (per-joint
reachable ranges plus a sparse list of blocked joint-combinations). When a
joint is blocked, a bounded multi-joint escape search perturbs other joints
to find combinations that unblock it, rather than stopping at the first
single-joint contact. A bundled self-collision default map ships and loads
automatically when no user `--map` is present.

**v1 scope:** `arm explore` *produces and stores* the reachability map, and
the map is queryable offline straight from the file — but v1 does **not**
change `arm flex`'s behavior. Consuming the map to gate `arm flex` targets
(refuse/warn on a request outside the discovered envelope) is a documented
follow-up, not part of this verb.

## Make it your own

1. Rename the package `arm101/` and the `arm101-cli`
   CLI/dist name throughout `pyproject.toml`, the package, `tests/`,
   `sonar-project.properties`, and this `README.md`. The name is hard-coded in
   ~100 places, so list every occurrence first — see the `git grep` discovery
   command in [`CLAUDE.md`](CLAUDE.md), the authoritative rename procedure.
2. Edit `culture.yaml` with your `suffix` and `backend`.
3. Rewrite `CLAUDE.md` for your agent and run `/init`.
4. Re-vendor only the skills you need from guildmaster (see
   [`docs/skill-sources.md`](docs/skill-sources.md)).

See [`CLAUDE.md`](CLAUDE.md) for the full conventions (version-bump-every-PR,
the `cicd` PR lane, deploy setup).

## License

Apache 2.0 — see [`LICENSE`](LICENSE).
