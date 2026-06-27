"""Consent, plan, and audit helpers for hardware-mutating arm101 verbs.

Three consent modes
-------------------
``interactive``
    stdin is a TTY; the operator is physically present and will type a
    confirmation at the prompt.  This is the lowest-trust / highest-safety
    mode: every destructive step gates on human input.

``dry_run``
    stdin is not a TTY and ``--apply`` was not passed.  The verb MUST emit a
    plan file and stdout summary and then stop; nothing is written to hardware.

``agent``
    stdin is not a TTY, ``--apply`` was passed, and (when
    ``require_plan_hash`` is True) a valid ``--plan-hash`` matching the live
    motor state has been supplied.  The verb may proceed to write hardware.

Decision table (first match wins)::

    is_tty                                        → "interactive"
    not is_tty, not apply                         → "dry_run"
    not is_tty, apply, not require_plan_hash      → "agent"
    not is_tty, apply, require_plan_hash,
        plan_hash missing or empty                → CliError(EXIT_ENV_ERROR)
    not is_tty, apply, require_plan_hash,
        plan_hash present but malformed           → CliError(EXIT_USER_ERROR)
    not is_tty, apply, require_plan_hash,
        plan_hash well-formed                     → "agent"

Plan file and hash
------------------
A :func:`build_plan` call snapshots the motor state and intent, hashes the
deterministic canonical form, and writes a JSON file to
``$ARM101_PLAN_DIR`` (default ``~/.arm101/plans/``).  Only the plan FILE
PATH is shown to the operator on stdout; ``plan_hash`` lives inside the
file so they must ``cat`` it to obtain the value — preventing drive-by
``--plan-hash`` guessing from stdout alone.

:func:`verify_plan_hash` re-derives the hash from the live motor state when
``--apply --plan-hash`` is supplied and rejects any mismatch, ensuring the
plan was created for the current motor state.

Audit log
---------
:func:`write_audit` appends one JSONL record to ``$ARM101_AUDIT_LOG``
(default ``~/.arm101/audit.log``).  It NEVER raises on write failure — a
logging failure must not prevent a legitimate hardware operation.
"""

from __future__ import annotations

import getpass
import hashlib
import json
import os
import re
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Literal

from arm101.cli._errors import EXIT_ENV_ERROR, EXIT_USER_ERROR, CliError
from arm101.cli._output import emit_diagnostic, emit_result

# ---------------------------------------------------------------------------
# Plan hash format
# ---------------------------------------------------------------------------

#: Regex that a well-formed plan hash must match.
_HASH_RE = re.compile(r"^sha256:[0-9a-f]{64}$")


# ---------------------------------------------------------------------------
# Consent resolution
# ---------------------------------------------------------------------------


def resolve_consent(
    args: object,
    *,
    verb: str,
    require_plan_hash: bool,
) -> Literal["interactive", "agent", "dry_run"]:
    """Resolve the operator consent mode from *args* and the environment.

    Parameters
    ----------
    args:
        The parsed :class:`argparse.Namespace` (or any object with optional
        ``apply`` and ``plan_hash`` attributes).
    verb:
        The CLI verb name (used only in error messages).
    require_plan_hash:
        If *True*, agent-mode invocations must supply a matching
        ``--plan-hash`` (2-step tier: center-motor and friends).  If *False*
        a bare ``--apply`` flag suffices (1-step tier: set-motor-id).

    Returns
    -------
    Literal["interactive", "agent", "dry_run"]

    Raises
    ------
    CliError(EXIT_ENV_ERROR)
        When ``require_plan_hash`` is True, ``--apply`` is set, and no hash
        was provided.
    CliError(EXIT_USER_ERROR)
        When ``require_plan_hash`` is True, ``--apply`` is set, and the
        supplied hash does not match ``r"^sha256:[0-9a-f]{64}$"``.
    """
    if sys.stdin.isatty():
        return "interactive"

    apply_flag = bool(getattr(args, "apply", False))

    if not apply_flag:
        return "dry_run"

    # apply=True from here on — check whether a plan hash is required
    if not require_plan_hash:
        return "agent"

    # require_plan_hash is True — validate --plan-hash
    raw = getattr(args, "plan_hash", None)
    plan_hash = (raw.strip() if isinstance(raw, str) else "") if raw is not None else ""

    if not plan_hash:
        raise CliError(
            code=EXIT_ENV_ERROR,
            message=(
                f"{verb}: --plan-hash is required in non-interactive (agent) mode "
                "but was not supplied."
            ),
            remediation=(
                f"First run 'arm101 {verb}' without --apply to generate a plan file. "
                "Read the plan file to obtain plan_hash, then re-run: "
                f"arm101 {verb} --apply --plan-hash <hash>."
            ),
        )

    if not _HASH_RE.match(plan_hash):
        raise CliError(
            code=EXIT_USER_ERROR,
            message=(
                f"{verb}: malformed plan hash {plan_hash!r}. "
                "Expected format: sha256:<64 lowercase hex digits>."
            ),
            remediation="Copy the plan_hash value exactly from the plan file.",
        )

    return "agent"


# ---------------------------------------------------------------------------
# Canonical form and hash computation
# ---------------------------------------------------------------------------


def _canonical_for_hash(
    verb: str,
    port: str,
    action: dict,
    info: dict,
) -> dict:
    """Return the canonical dict used as the SHA-256 input.

    Deliberately excludes operator, created_at, lock_register, and volatile
    sensors — only the action + stable identity/position/torque define the hash.
    This means two operators running the same action against the same physical
    motor state get the same hash, which is what :func:`verify_plan_hash` relies on.
    """
    return {
        "verb": verb,
        "port": port,
        "action": action,
        "motor_snapshot": {
            "id": info["id"],
            "model": info["model"],
            "present_position": info["present_position"],
            "torque_enable": info["torque_enable"],
        },
    }


def _compute_hash(canonical: dict) -> str:
    """Return ``sha256:<hex>`` of the JSON-serialised *canonical* dict."""
    blob = json.dumps(canonical, sort_keys=True, ensure_ascii=True).encode("ascii")
    return "sha256:" + hashlib.sha256(blob).hexdigest()


# ---------------------------------------------------------------------------
# Plan construction
# ---------------------------------------------------------------------------


def build_plan(
    verb: str,
    port: str,
    info: dict,
    action: dict,
    *,
    operator: str,
    created_at: str,
) -> dict:
    """Build a consent plan dict.

    Parameters
    ----------
    verb:
        CLI verb that will perform the hardware write (e.g. ``"set-motor-id"``).
    port:
        Serial port path (e.g. ``"/dev/ttyACM1"``).
    info:
        Full ``read_info()`` snapshot for the motor.
    action:
        Verb-specific intent dict (e.g. ``{"new_id": 5}``).
    operator:
        Operator identity string from :func:`resolve_operator`.
    created_at:
        ISO 8601 UTC timestamp string.  Passed in so tests can control it
        deterministically; callers should supply
        ``datetime.now(timezone.utc).isoformat() + "Z"``.

    Returns
    -------
    dict
        Plan dict with keys ``schema_version``, ``verb``, ``created_at``,
        ``port``, ``operator``, ``action``, ``motor_snapshot``, and
        ``plan_hash``.

    Notes
    -----
    ``plan_hash`` covers ``verb``, ``port``, ``action``, and the stable motor
    identity fields (``id``, ``model``, ``present_position``, ``torque_enable``).
    It intentionally excludes ``operator``, ``created_at``, ``lock_register``,
    and volatile sensors so that two independent runs with the same physical
    motor state produce the same hash.
    """
    canonical = _canonical_for_hash(verb, port, action, info)
    plan_hash = _compute_hash(canonical)

    return {
        "schema_version": 1,
        "verb": verb,
        "created_at": created_at,
        "port": port,
        "operator": operator,
        "action": action,
        "motor_snapshot": {
            "id": info["id"],
            "model": info["model"],
            "present_position": info["present_position"],
            "torque_enable": info["torque_enable"],
            "lock_register": info.get("lock_register", 0),
        },
        "plan_hash": plan_hash,
    }


# ---------------------------------------------------------------------------
# Plan file I/O
# ---------------------------------------------------------------------------


def _plan_dir() -> Path:
    """Return the directory for plan files (``$ARM101_PLAN_DIR`` or default)."""
    env = os.environ.get("ARM101_PLAN_DIR", "").strip()
    if env:
        return Path(env)
    return Path.home() / ".arm101" / "plans"


def _compact_ts(created_at: str) -> str:
    """Convert an ISO 8601 UTC string to compact ``%Y%m%dT%H%M%SZ`` form.

    Examples
    --------
    ``"2024-01-15T12:30:45.123456Z"``  →  ``"20240115T123045Z"``
    """
    ts = created_at
    if ts.endswith("Z"):
        ts = ts[:-1] + "+00:00"
    try:
        dt = datetime.fromisoformat(ts)
    except ValueError:
        # Fallback: strip punctuation and grab the first 15 significant chars
        stripped = created_at.replace("-", "").replace(":", "").replace(".", "")
        return stripped[:15] + "Z"
    return dt.strftime("%Y%m%dT%H%M%SZ")


def write_plan_file(plan: dict) -> str:
    """Write *plan* as a pretty-printed JSON file and return its path.

    The file is placed in ``$ARM101_PLAN_DIR`` (or ``~/.arm101/plans/``).
    File name: ``<verb>-<basename(port)>-<created_at compact>.json``.
    The ``plan_hash`` IS included in this file (it is intentionally excluded
    from stdout via :func:`emit_plan_stdout`).
    """
    plan_dir = _plan_dir()
    plan_dir.mkdir(parents=True, exist_ok=True)

    port_base = Path(plan["port"]).name
    ts_compact = _compact_ts(plan["created_at"])
    stem = f"{plan['verb']}-{port_base}-{ts_compact}"
    path = plan_dir / f"{stem}.json"

    # The compact timestamp is second-resolution, so two dry-runs of the same
    # verb+port within one second would collide. Append a counter rather than
    # silently overwriting the earlier plan (an agent may still be reading it).
    counter = 1
    while path.exists():
        path = plan_dir / f"{stem}-{counter}.json"
        counter += 1

    path.write_text(json.dumps(plan, indent=2), encoding="utf-8")
    return str(path)


# ---------------------------------------------------------------------------
# Stdout plan summary
# ---------------------------------------------------------------------------


def emit_plan_stdout(plan: dict, plan_path: str, *, json_mode: bool) -> None:
    """Emit a plan summary to stdout.

    CRITICAL: ``plan_hash`` MUST NOT appear in this output — only the file
    path is shown so the operator must ``cat`` the file to obtain the hash.
    This prevents drive-by ``--plan-hash`` guessing from stdout alone.

    Parameters
    ----------
    plan:
        The plan dict returned by :func:`build_plan`.
    plan_path:
        Absolute path to the plan file written by :func:`write_plan_file`.
    json_mode:
        If *True*, emit a structured JSON object to stdout.  If *False*,
        emit a human-readable Markdown-style summary.
    """
    verb = plan["verb"]
    port = plan["port"]
    operator = plan["operator"]
    snap = plan["motor_snapshot"]
    action = plan["action"]
    created_at = plan["created_at"]

    if json_mode:
        payload = {
            "plan_file": plan_path,
            "verb": verb,
            "port": port,
            "operator": operator,
            "created_at": created_at,
            "action": action,
            "motor_snapshot": snap,
            # plan_hash intentionally omitted — read the file to obtain it
        }
        emit_result(payload, json_mode=True)
        return

    action_lines = [f"- {k}: {v}" for k, v in action.items()]
    lines = [
        f"## Dry-run plan: {verb}",
        "",
        f"- **port**     : {port}",
        f"- **operator** : {operator}",
        f"- **created**  : {created_at}",
        "",
        "### Action",
        "",
        *action_lines,
        "",
        "### Motor snapshot",
        "",
        f"- id               : {snap['id']}",
        f"- model            : {snap['model']}",
        f"- present_position : {snap['present_position']}",
        f"- torque_enable    : {snap['torque_enable']}",
        f"- lock_register    : {snap.get('lock_register', 0)}",
        "",
        "### Next step",
        "",
        f"Plan written to: {plan_path}",
        "",
        "To execute, read the plan file to obtain the plan_hash, then re-run:",
        f"  arm101 {verb} --apply --plan-hash <hash from plan file>",
        "",
        "  Example:",
        f"    cat {plan_path}",
        f"    arm101 {verb} --apply --plan-hash sha256:...",
    ]
    emit_result("\n".join(lines), json_mode=False)


# ---------------------------------------------------------------------------
# Hash verification
# ---------------------------------------------------------------------------


def verify_plan_hash(
    supplied: str,
    *,
    verb: str,
    port: str,
    action: dict,
    info: dict,
) -> None:
    """Verify *supplied* matches the hash recomputed from live motor state.

    Parameters
    ----------
    supplied:
        The ``--plan-hash`` value supplied by the operator.
    verb, port, action, info:
        Same values that would be passed to :func:`build_plan`.

    Raises
    ------
    CliError(EXIT_ENV_ERROR)
        If the recomputed hash differs from *supplied*, meaning the motor
        state has changed since the plan was created or the wrong hash was given.
    """
    canonical = _canonical_for_hash(verb, port, action, info)
    expected = _compute_hash(canonical)
    if supplied != expected:
        raise CliError(
            code=EXIT_ENV_ERROR,
            message=(
                "plan hash mismatch: the live motor state, or a command argument "
                "(e.g. --position / --keep-torque), differs from the plan this hash "
                "was generated for"
            ),
            remediation=(
                "Re-run the same command without --apply to generate a fresh plan for "
                "the current motor state and arguments, then read the plan file and "
                "supply its plan_hash with --apply --plan-hash."
            ),
        )


# ---------------------------------------------------------------------------
# Operator identity
# ---------------------------------------------------------------------------


def resolve_operator() -> str:
    """Return an operator identity string for audit and plan records.

    Resolution order (first non-empty wins):

    1. ``ARM101_OPERATOR`` environment variable.
    2. The ``suffix`` (nick) read from ``culture.yaml`` via the whoami seam
       — only when the file is actually found on disk.
    3. ``"tty:" + getpass.getuser()`` — the logged-in Unix user name.

    This value is informational only and does NOT affect ``plan_hash``.
    """
    env_op = os.environ.get("ARM101_OPERATOR", "").strip()
    if env_op:
        return env_op

    try:
        from arm101.cli._commands.whoami import find_culture_yaml, read_agent_fields

        if find_culture_yaml() is not None:
            fields = read_agent_fields()
            nick = fields.get("nick", "").strip()
            if nick:
                return nick
    except Exception:  # noqa: BLE001 # nosec B110
        pass

    return "tty:" + getpass.getuser()


# ---------------------------------------------------------------------------
# Audit log
# ---------------------------------------------------------------------------


def _audit_path() -> Path:
    """Return the path to the audit log file."""
    env = os.environ.get("ARM101_AUDIT_LOG", "").strip()
    if env:
        return Path(env)
    return Path.home() / ".arm101" / "audit.log"


def build_audit_record(
    *,
    verb: str,
    port: str,
    operator: str,
    consent_mode: str,
    action: dict,
    outcome: str,
    plan_hash: str | None = None,
    error: str | None = None,
) -> dict:
    """Build an audit record dict suitable for :func:`write_audit`.

    Parameters
    ----------
    verb:
        CLI verb name.
    port:
        Serial port path.
    operator:
        Operator identity string from :func:`resolve_operator`.
    consent_mode:
        One of ``"interactive"``, ``"agent"``, ``"dry_run"``.
    action:
        Verb-specific intent dict.
    outcome:
        One of ``"pending"``, ``"success"``, ``"failed"``.
    plan_hash:
        Optional plan hash; present only in agent mode.
    error:
        Optional error message; present when outcome is ``"failed"``.

    Returns
    -------
    dict
        Audit record with keys ``ts``, ``verb``, ``port``, ``operator``,
        ``consent_mode``, ``action``, ``outcome``, and optionally
        ``plan_hash`` and ``error``.
    """
    record: dict = {
        "ts": datetime.now(timezone.utc).isoformat(),
        "verb": verb,
        "port": port,
        "operator": operator,
        "consent_mode": consent_mode,
        "action": action,
        "outcome": outcome,
    }
    if plan_hash is not None:
        record["plan_hash"] = plan_hash
    if error is not None:
        record["error"] = error
    return record


def write_audit(record: dict) -> None:
    """Append *record* as a single JSONL line to the audit log.

    NEVER raises on write failure — a logging failure must not abort a
    legitimate hardware operation.  Any exception is swallowed and
    optionally surfaced as a diagnostic on stderr.
    """
    try:
        audit_path = _audit_path()
        audit_path.parent.mkdir(parents=True, exist_ok=True)
        with audit_path.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(record) + "\n")
    except Exception as exc:  # noqa: BLE001
        try:
            emit_diagnostic(f"[arm101] audit write failed (ignored): {exc}")
        except Exception:  # noqa: BLE001 # nosec B110
            pass
