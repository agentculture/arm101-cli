"""``arm101-cli doctor`` — check the agent-identity invariants.

Mirrors the two invariants ``steward doctor`` verifies for a mesh agent:

* **prompt-file-present** — the repo declares an agent in ``culture.yaml`` and
  has the matching prompt file on disk;
* **backend-consistency** — the declared ``backend`` matches the prompt file
  (``claude`` → ``CLAUDE.md``, ``colleague`` → ``AGENTS.colleague.md``,
  ``acp`` → ``AGENTS.md``, ``gemini`` → ``GEMINI.md``).

Plus a **skills-present** check (the vendored ``.claude/skills/`` kit). Read-only.

Reports the rubric-shaped contract
``{healthy, checks: [{id, passed, severity, message, remediation}]}`` so the
agent-first rubric's bundle 7 passes. When run from a wheel install (no
``culture.yaml`` alongside the package), it reports a single info check and
exits 0 — there is nothing to diagnose.

``--probe`` switches to a second, unrelated diagnosis: a multi-baud sweep of
the Feetech bus (see :mod:`arm101.hardware.baud_probe`), reporting per-id
SUCCESS/CORRUPT/TIMEOUT classifications at every candidate baud. A fully
silent bus is itself a successful diagnosis (exit 0) — it answered the
question "is anything out there at any baud" with "no". Only an unresolvable
port (no ``--port`` given and none auto-detected) is a CliError.
"""

from __future__ import annotations

import argparse

from arm101.cli._commands.calibrate_motor import _candidate_ports
from arm101.cli._commands.whoami import find_culture_yaml, read_agent_fields
from arm101.cli._errors import EXIT_ENV_ERROR, CliError
from arm101.cli._output import emit_result
from arm101.hardware.baud_probe import probe_bus
from arm101.hardware.bus import require_sdk

# backend → required prompt file (the backend-consistency mapping).
_PROMPT_FILE = {
    "claude": "CLAUDE.md",
    "colleague": "AGENTS.colleague.md",
    "acp": "AGENTS.md",
    "gemini": "GEMINI.md",
}


def _diagnose() -> dict[str, object]:
    cfg = find_culture_yaml()
    if cfg is None:
        check = {
            "id": "source_checkout",
            "passed": True,
            "severity": "info",
            "message": "no culture.yaml found alongside the package; identity checks skipped",
            "remediation": "",
        }
        return {"healthy": True, "checks": [check]}

    root = cfg.parent
    fields = read_agent_fields()
    backend = fields["backend"]
    checks: list[dict[str, object]] = []

    # 1. backend-consistency: the prompt file for the declared backend exists.
    expected = _PROMPT_FILE.get(backend)
    if expected is None:
        checks.append(
            {
                "id": "backend_consistency",
                "passed": False,
                "severity": "error",
                "message": f"unknown backend '{backend}' in culture.yaml",
                "remediation": f"set backend to one of: {', '.join(sorted(_PROMPT_FILE))}",
            }
        )
    else:
        present = (root / expected).is_file()
        checks.append(
            {
                "id": "prompt_file_present",
                "passed": present,
                "severity": "error",
                "message": (
                    f"backend '{backend}' requires {expected} — "
                    + ("present" if present else "missing")
                ),
                "remediation": "" if present else f"create {expected} at the repo root",
            }
        )

    # 2. skills-present: the vendored skill kit is on disk.
    skills_dir = root / ".claude" / "skills"
    has_skills = skills_dir.is_dir() and any(skills_dir.iterdir())
    checks.append(
        {
            "id": "skills_present",
            "passed": has_skills,
            "severity": "warning",
            "message": (
                ".claude/skills/ vendored" if has_skills else ".claude/skills/ missing or empty"
            ),
            "remediation": (
                "" if has_skills else "vendor the skill kit (see docs/skill-sources.md)"
            ),
        }
    )

    healthy = all(c["passed"] for c in checks)
    return {"healthy": healthy, "checks": checks}


def _resolve_probe_port(args: argparse.Namespace) -> str:
    """Resolve the port to probe: ``--port`` if given, else the first auto-detected candidate.

    Raises ``CliError(EXIT_ENV_ERROR)`` when neither is available.
    """
    port = getattr(args, "port", None)
    if port:
        return port
    candidates = _candidate_ports()
    if not candidates:
        raise CliError(
            code=EXIT_ENV_ERROR,
            message="no serial port found to probe",
            remediation="connect the bus / pass --port",
        )
    return candidates[0]


def _run_probe(args: argparse.Namespace) -> int:
    """Run the multi-baud bus probe and emit its report.

    A completed probe — including a fully-silent one — is a successful
    diagnosis (exit 0); an unresolvable port or a missing Feetech SDK raises
    ``CliError(EXIT_ENV_ERROR)``. The SDK pre-flight matters because
    ``probe_bus`` degrades a bad ``open()`` at each baud to ``TIMEOUT``; without
    it, an absent ``scservo_sdk`` would be misreported as a "silent bus" rather
    than a clear "install the SDK" error.
    """
    json_mode = bool(getattr(args, "json", False))
    port = _resolve_probe_port(args)
    require_sdk()
    report = probe_bus(port)
    if json_mode:
        emit_result(report.to_dict(), json_mode=True)
    else:
        emit_result(report.summary(), json_mode=False)
    return 0


def cmd_doctor(args: argparse.Namespace) -> int:
    if getattr(args, "probe", False):
        return _run_probe(args)

    report = _diagnose()
    json_mode = bool(getattr(args, "json", False))
    if json_mode:
        emit_result(report, json_mode=True)
    else:
        status = "healthy" if report["healthy"] else "unhealthy"
        lines = [f"arm101-cli doctor: {status}", ""]
        for check in report["checks"]:
            mark = "ok" if check["passed"] else "FAIL"
            lines.append(f"[{mark}] {check['id']}: {check['message']}")
            if not check["passed"] and check["remediation"]:
                lines.append(f"  hint: {check['remediation']}")
        emit_result("\n".join(lines), json_mode=False)
    return 0 if report["healthy"] else 1


def register(sub: argparse._SubParsersAction) -> None:
    p = sub.add_parser(
        "doctor",
        help="Check the agent-identity invariants (prompt-file-present, backend-consistency).",
    )
    p.add_argument("--json", action="store_true", help="Emit structured JSON.")
    p.add_argument(
        "--probe",
        action="store_true",
        help="Run a multi-baud bus probe (SUCCESS/CORRUPT/TIMEOUT per id) instead of the "
        "identity diagnosis.",
    )
    p.add_argument(
        "--port",
        default=None,
        help="serial port to probe; default auto-detect",
    )
    p.set_defaults(func=cmd_doctor)
