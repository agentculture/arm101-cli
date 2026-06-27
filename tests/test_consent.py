"""Tests for arm101.cli._consent — three-mode consent, plan, and audit.

TDD: this file drives the implementation in arm101/cli/_consent.py.

Covers:
- resolve_consent: all six decision-table rows (interactive, dry_run,
  agent/no-hash-required, agent/good-hash, env-error/missing-hash,
  user-error/malformed-hash).
- build_plan: hash stability, hash sensitivity to position, hash independence
  from operator/created_at/lock_register, full key presence.
- write_plan_file + emit_plan_stdout: file contains plan_hash; stdout does
  NOT contain plan_hash but DOES contain the file path.
- verify_plan_hash: matching → ok; state change → EXIT_ENV_ERROR.
- resolve_operator: env var wins; tty: fallback when culture.yaml absent.
- write_audit: appends JSONL; unwritable path does not raise; creates dirs.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

from arm101.cli._errors import EXIT_ENV_ERROR, EXIT_USER_ERROR, CliError

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


class _TtyStdin:
    """Fake stdin that reports isatty() = True."""

    def isatty(self) -> bool:
        return True

    def readline(self) -> str:  # pragma: no cover
        return ""


class _NonTtyStdin:
    """Fake stdin that reports isatty() = False."""

    def isatty(self) -> bool:
        return False

    def readline(self) -> str:  # pragma: no cover
        return ""


def _args(**kw):
    """Build a minimal argparse.Namespace with apply/plan_hash."""
    import argparse

    ns = argparse.Namespace(apply=False, plan_hash=None)
    for k, v in kw.items():
        setattr(ns, k, v)
    return ns


def _make_info(present_position: int = 2048, torque_enable: int = 0) -> dict:
    """Return a minimal read_info()-style snapshot for motor 1."""
    return {
        "id": 1,
        "model": 777,
        "present_position": present_position,
        "torque_enable": torque_enable,
        "lock_register": 0,
        "firmware_major": 3,
        "firmware_minor": 10,
        "baud_index": 0,
        "min_angle": 0,
        "max_angle": 4095,
        "present_speed": 0,
        "present_load": 0,
        "present_voltage": 120,
        "present_temperature": 38,
    }


_CREATED_AT = "2024-01-15T12:30:45.000000Z"
_ACTION = {"new_id": 5}
_PORT = "/dev/ttyACM1"
_VERB = "set-motor-id"

# ---------------------------------------------------------------------------
# 1. resolve_consent — six decision-table rows
# ---------------------------------------------------------------------------


def test_resolve_consent_interactive_tty(monkeypatch) -> None:
    """TTY stdin → 'interactive' regardless of --apply or plan_hash."""
    from arm101.cli._consent import resolve_consent

    monkeypatch.setattr(sys, "stdin", _TtyStdin())
    result = resolve_consent(_args(apply=True, plan_hash=None), verb=_VERB, require_plan_hash=True)
    assert result == "interactive"


def test_resolve_consent_interactive_tty_apply_ignored(monkeypatch) -> None:
    """Under a TTY, --apply is ignored and the result is still 'interactive'."""
    from arm101.cli._consent import resolve_consent

    monkeypatch.setattr(sys, "stdin", _TtyStdin())
    result = resolve_consent(_args(apply=True), verb=_VERB, require_plan_hash=False)
    assert result == "interactive"


def test_resolve_consent_dry_run_no_apply(monkeypatch) -> None:
    """Non-TTY without --apply → 'dry_run'."""
    from arm101.cli._consent import resolve_consent

    monkeypatch.setattr(sys, "stdin", _NonTtyStdin())
    result = resolve_consent(_args(apply=False), verb=_VERB, require_plan_hash=True)
    assert result == "dry_run"


def test_resolve_consent_dry_run_default_args(monkeypatch) -> None:
    """Non-TTY with default args (apply absent) → 'dry_run'."""
    import argparse

    from arm101.cli._consent import resolve_consent

    monkeypatch.setattr(sys, "stdin", _NonTtyStdin())
    ns = argparse.Namespace()  # no apply attribute at all
    result = resolve_consent(ns, verb=_VERB, require_plan_hash=False)
    assert result == "dry_run"


def test_resolve_consent_agent_no_hash_required(monkeypatch) -> None:
    """Non-TTY + apply + require_plan_hash=False → 'agent' (1-step tier)."""
    from arm101.cli._consent import resolve_consent

    monkeypatch.setattr(sys, "stdin", _NonTtyStdin())
    result = resolve_consent(_args(apply=True), verb=_VERB, require_plan_hash=False)
    assert result == "agent"


def test_resolve_consent_agent_good_hash(monkeypatch) -> None:
    """Non-TTY + apply + well-formed hash + require_plan_hash=True → 'agent'."""
    from arm101.cli._consent import resolve_consent

    good_hash = "sha256:" + "a" * 64
    monkeypatch.setattr(sys, "stdin", _NonTtyStdin())
    result = resolve_consent(
        _args(apply=True, plan_hash=good_hash),
        verb=_VERB,
        require_plan_hash=True,
    )
    assert result == "agent"


def test_resolve_consent_env_error_missing_hash(monkeypatch) -> None:
    """Non-TTY + apply + require_plan_hash=True + None hash → EXIT_ENV_ERROR."""
    from arm101.cli._consent import resolve_consent

    monkeypatch.setattr(sys, "stdin", _NonTtyStdin())
    with pytest.raises(CliError) as exc:
        resolve_consent(
            _args(apply=True, plan_hash=None),
            verb=_VERB,
            require_plan_hash=True,
        )
    assert exc.value.code == EXIT_ENV_ERROR
    # Remediation must mention how to generate a plan
    assert "--apply" in exc.value.remediation
    assert "--plan-hash" in exc.value.remediation


def test_resolve_consent_env_error_empty_string_hash(monkeypatch) -> None:
    """Non-TTY + apply + require_plan_hash=True + empty string → EXIT_ENV_ERROR."""
    from arm101.cli._consent import resolve_consent

    monkeypatch.setattr(sys, "stdin", _NonTtyStdin())
    with pytest.raises(CliError) as exc:
        resolve_consent(
            _args(apply=True, plan_hash=""),
            verb=_VERB,
            require_plan_hash=True,
        )
    assert exc.value.code == EXIT_ENV_ERROR


def test_resolve_consent_user_error_malformed_hash(monkeypatch) -> None:
    """Non-TTY + apply + require_plan_hash=True + garbage hash → EXIT_USER_ERROR."""
    from arm101.cli._consent import resolve_consent

    monkeypatch.setattr(sys, "stdin", _NonTtyStdin())
    with pytest.raises(CliError) as exc:
        resolve_consent(
            _args(apply=True, plan_hash="notahash"),
            verb=_VERB,
            require_plan_hash=True,
        )
    assert exc.value.code == EXIT_USER_ERROR


def test_resolve_consent_user_error_wrong_prefix(monkeypatch) -> None:
    """sha256: prefix required; md5:... is rejected as EXIT_USER_ERROR."""
    from arm101.cli._consent import resolve_consent

    monkeypatch.setattr(sys, "stdin", _NonTtyStdin())
    with pytest.raises(CliError) as exc:
        resolve_consent(
            _args(apply=True, plan_hash="md5:" + "a" * 32),
            verb=_VERB,
            require_plan_hash=True,
        )
    assert exc.value.code == EXIT_USER_ERROR


def test_resolve_consent_user_error_truncated_hash(monkeypatch) -> None:
    """sha256 prefix with only 32 hex chars (truncated) → EXIT_USER_ERROR."""
    from arm101.cli._consent import resolve_consent

    monkeypatch.setattr(sys, "stdin", _NonTtyStdin())
    with pytest.raises(CliError) as exc:
        resolve_consent(
            _args(apply=True, plan_hash="sha256:" + "a" * 32),
            verb=_VERB,
            require_plan_hash=True,
        )
    assert exc.value.code == EXIT_USER_ERROR


# ---------------------------------------------------------------------------
# 2. build_plan — hash properties
# ---------------------------------------------------------------------------


def test_build_plan_returns_well_formed_hash() -> None:
    """build_plan includes a plan_hash matching the expected regex."""
    import re

    from arm101.cli._consent import build_plan

    info = _make_info()
    plan = build_plan(_VERB, _PORT, info, _ACTION, operator="op", created_at=_CREATED_AT)
    assert "plan_hash" in plan
    assert re.match(r"^sha256:[0-9a-f]{64}$", plan["plan_hash"])


def test_build_plan_hash_is_stable() -> None:
    """Two calls with identical arguments produce the same plan_hash."""
    from arm101.cli._consent import build_plan

    info = _make_info()
    plan1 = build_plan(_VERB, _PORT, info, _ACTION, operator="op", created_at=_CREATED_AT)
    plan2 = build_plan(_VERB, _PORT, info, _ACTION, operator="op", created_at=_CREATED_AT)
    assert plan1["plan_hash"] == plan2["plan_hash"]


def test_build_plan_hash_excludes_operator() -> None:
    """Changing operator does NOT change plan_hash."""
    from arm101.cli._consent import build_plan

    info = _make_info()
    plan_alice = build_plan(_VERB, _PORT, info, _ACTION, operator="alice", created_at=_CREATED_AT)
    plan_bob = build_plan(_VERB, _PORT, info, _ACTION, operator="bob", created_at=_CREATED_AT)
    assert plan_alice["plan_hash"] == plan_bob["plan_hash"]


def test_build_plan_hash_excludes_created_at() -> None:
    """Changing created_at does NOT change plan_hash."""
    from arm101.cli._consent import build_plan

    info = _make_info()
    plan1 = build_plan(
        _VERB,
        _PORT,
        info,
        _ACTION,
        operator="op",
        created_at=_CREATED_AT,
    )
    plan2 = build_plan(
        _VERB,
        _PORT,
        info,
        _ACTION,
        operator="op",
        created_at="2099-12-31T23:59:59.000000Z",
    )
    assert plan1["plan_hash"] == plan2["plan_hash"]


def test_build_plan_hash_changes_with_present_position() -> None:
    """Changing present_position produces a DIFFERENT plan_hash."""
    from arm101.cli._consent import build_plan

    plan_a = build_plan(
        _VERB,
        _PORT,
        _make_info(present_position=2048),
        _ACTION,
        operator="op",
        created_at=_CREATED_AT,
    )
    plan_b = build_plan(
        _VERB,
        _PORT,
        _make_info(present_position=100),
        _ACTION,
        operator="op",
        created_at=_CREATED_AT,
    )
    assert plan_a["plan_hash"] != plan_b["plan_hash"]


def test_build_plan_hash_excludes_lock_register() -> None:
    """lock_register does NOT affect plan_hash (excluded from canonical form)."""
    from arm101.cli._consent import build_plan

    info_unlocked = _make_info()
    info_unlocked["lock_register"] = 0
    info_locked = _make_info()
    info_locked["lock_register"] = 1

    plan_unlocked = build_plan(
        _VERB, _PORT, info_unlocked, _ACTION, operator="op", created_at=_CREATED_AT
    )
    plan_locked = build_plan(
        _VERB, _PORT, info_locked, _ACTION, operator="op", created_at=_CREATED_AT
    )
    assert plan_unlocked["plan_hash"] == plan_locked["plan_hash"]


def test_build_plan_motor_snapshot_includes_lock_register() -> None:
    """motor_snapshot in the plan dict includes lock_register even though it
    is excluded from the hash."""
    from arm101.cli._consent import build_plan

    info = _make_info()
    info["lock_register"] = 1
    plan = build_plan(_VERB, _PORT, info, _ACTION, operator="op", created_at=_CREATED_AT)
    assert plan["motor_snapshot"]["lock_register"] == 1


def test_build_plan_all_required_keys() -> None:
    """build_plan result contains all required top-level keys."""
    from arm101.cli._consent import build_plan

    info = _make_info()
    plan = build_plan(_VERB, _PORT, info, _ACTION, operator="op", created_at=_CREATED_AT)
    required = {
        "schema_version",
        "verb",
        "created_at",
        "port",
        "operator",
        "action",
        "motor_snapshot",
        "plan_hash",
    }
    assert required.issubset(plan.keys())
    assert plan["schema_version"] == 1
    assert plan["verb"] == _VERB
    assert plan["port"] == _PORT
    assert plan["created_at"] == _CREATED_AT
    assert plan["operator"] == "op"
    assert plan["action"] == _ACTION


# ---------------------------------------------------------------------------
# 3. write_plan_file + emit_plan_stdout
# ---------------------------------------------------------------------------


def test_write_plan_file_contains_plan_hash(tmp_path, monkeypatch) -> None:
    """The JSON file written by write_plan_file includes plan_hash."""
    from arm101.cli._consent import build_plan, write_plan_file

    monkeypatch.setenv("ARM101_PLAN_DIR", str(tmp_path))

    info = _make_info()
    plan = build_plan(_VERB, _PORT, info, _ACTION, operator="op", created_at=_CREATED_AT)
    plan_path = write_plan_file(plan)

    data = json.loads(Path(plan_path).read_text())
    assert "plan_hash" in data
    assert data["plan_hash"] == plan["plan_hash"]


def test_write_plan_file_name_format(tmp_path, monkeypatch) -> None:
    """File name is <verb>-<basename(port)>-<compact_ts>.json."""
    from arm101.cli._consent import build_plan, write_plan_file

    monkeypatch.setenv("ARM101_PLAN_DIR", str(tmp_path))

    info = _make_info()
    plan = build_plan(_VERB, _PORT, info, _ACTION, operator="op", created_at=_CREATED_AT)
    plan_path = write_plan_file(plan)

    fname = Path(plan_path).name
    # verb- and port basename
    assert fname.startswith("set-motor-id-ttyACM1-")
    assert fname.endswith(".json")
    # compact timestamp must appear in the name
    assert "20240115T123045Z" in fname


def test_write_plan_file_does_not_clobber_same_second_plan(tmp_path, monkeypatch) -> None:
    """Two plans with the same compact (second-resolution) timestamp get distinct files."""
    from arm101.cli._consent import build_plan, write_plan_file

    monkeypatch.setenv("ARM101_PLAN_DIR", str(tmp_path))

    plan = build_plan(_VERB, _PORT, _make_info(), _ACTION, operator="op", created_at=_CREATED_AT)
    first = write_plan_file(plan)
    second = write_plan_file(plan)  # identical created_at -> would collide on the name

    assert first != second
    assert Path(first).exists()
    assert Path(second).exists()
    assert len(list(tmp_path.glob("*.json"))) == 2


def test_write_plan_file_respects_plan_dir_env(tmp_path, monkeypatch) -> None:
    """write_plan_file uses $ARM101_PLAN_DIR when set."""
    from arm101.cli._consent import build_plan, write_plan_file

    plan_dir = tmp_path / "custom-plans"
    monkeypatch.setenv("ARM101_PLAN_DIR", str(plan_dir))

    plan = build_plan(_VERB, _PORT, _make_info(), _ACTION, operator="op", created_at=_CREATED_AT)
    plan_path = write_plan_file(plan)

    assert Path(plan_path).parent == plan_dir
    assert plan_dir.is_dir()


def test_emit_plan_stdout_text_excludes_plan_hash(tmp_path, monkeypatch, capsys) -> None:
    """Text-mode stdout MUST NOT contain plan_hash."""
    from arm101.cli._consent import build_plan, emit_plan_stdout, write_plan_file

    monkeypatch.setenv("ARM101_PLAN_DIR", str(tmp_path))

    info = _make_info()
    plan = build_plan(_VERB, _PORT, info, _ACTION, operator="op", created_at=_CREATED_AT)
    plan_path = write_plan_file(plan)
    emit_plan_stdout(plan, plan_path, json_mode=False)

    out = capsys.readouterr().out
    assert plan["plan_hash"] not in out


def test_emit_plan_stdout_text_contains_file_path(tmp_path, monkeypatch, capsys) -> None:
    """Text-mode stdout contains the plan file path."""
    from arm101.cli._consent import build_plan, emit_plan_stdout, write_plan_file

    monkeypatch.setenv("ARM101_PLAN_DIR", str(tmp_path))

    info = _make_info()
    plan = build_plan(_VERB, _PORT, info, _ACTION, operator="op", created_at=_CREATED_AT)
    plan_path = write_plan_file(plan)
    emit_plan_stdout(plan, plan_path, json_mode=False)

    out = capsys.readouterr().out
    assert plan_path in out


def test_emit_plan_stdout_text_instructs_apply(tmp_path, monkeypatch, capsys) -> None:
    """Text-mode stdout instructs the reader to re-run with --apply --plan-hash."""
    from arm101.cli._consent import build_plan, emit_plan_stdout, write_plan_file

    monkeypatch.setenv("ARM101_PLAN_DIR", str(tmp_path))

    plan = build_plan(_VERB, _PORT, _make_info(), _ACTION, operator="op", created_at=_CREATED_AT)
    plan_path = write_plan_file(plan)
    emit_plan_stdout(plan, plan_path, json_mode=False)

    out = capsys.readouterr().out
    assert "--apply --plan-hash" in out


def test_emit_plan_stdout_json_excludes_plan_hash(tmp_path, monkeypatch, capsys) -> None:
    """JSON-mode stdout MUST NOT include plan_hash key."""
    from arm101.cli._consent import build_plan, emit_plan_stdout, write_plan_file

    monkeypatch.setenv("ARM101_PLAN_DIR", str(tmp_path))

    info = _make_info()
    plan = build_plan(_VERB, _PORT, info, _ACTION, operator="op", created_at=_CREATED_AT)
    plan_path = write_plan_file(plan)
    emit_plan_stdout(plan, plan_path, json_mode=True)

    out = capsys.readouterr().out
    payload = json.loads(out)
    assert "plan_hash" not in payload
    assert payload["plan_file"] == plan_path


def test_emit_plan_stdout_json_contains_action(tmp_path, monkeypatch, capsys) -> None:
    """JSON-mode output includes action and motor_snapshot."""
    from arm101.cli._consent import build_plan, emit_plan_stdout, write_plan_file

    monkeypatch.setenv("ARM101_PLAN_DIR", str(tmp_path))

    plan = build_plan(_VERB, _PORT, _make_info(), _ACTION, operator="op", created_at=_CREATED_AT)
    plan_path = write_plan_file(plan)
    emit_plan_stdout(plan, plan_path, json_mode=True)

    payload = json.loads(capsys.readouterr().out)
    assert payload["action"] == _ACTION
    assert "motor_snapshot" in payload


# ---------------------------------------------------------------------------
# 4. verify_plan_hash
# ---------------------------------------------------------------------------


def test_verify_plan_hash_matching_does_not_raise() -> None:
    """verify_plan_hash with a correct hash returns None."""
    from arm101.cli._consent import build_plan, verify_plan_hash

    info = _make_info()
    plan = build_plan(_VERB, _PORT, info, _ACTION, operator="op", created_at=_CREATED_AT)
    # Must not raise
    result = verify_plan_hash(
        plan["plan_hash"],
        verb=_VERB,
        port=_PORT,
        action=_ACTION,
        info=info,
    )
    assert result is None


def test_verify_plan_hash_mismatch_position_raises() -> None:
    """Motor moved (present_position changed) → hash mismatch → EXIT_ENV_ERROR."""
    from arm101.cli._consent import build_plan, verify_plan_hash

    original_info = _make_info(present_position=2048)
    plan = build_plan(_VERB, _PORT, original_info, _ACTION, operator="op", created_at=_CREATED_AT)

    # Motor moved
    live_info = _make_info(present_position=100)
    with pytest.raises(CliError) as exc:
        verify_plan_hash(
            plan["plan_hash"],
            verb=_VERB,
            port=_PORT,
            action=_ACTION,
            info=live_info,
        )
    assert exc.value.code == EXIT_ENV_ERROR
    assert "mismatch" in exc.value.message.lower()


def test_verify_plan_hash_wrong_hash_raises() -> None:
    """A completely wrong hash → EXIT_ENV_ERROR."""
    from arm101.cli._consent import verify_plan_hash

    info = _make_info()
    with pytest.raises(CliError) as exc:
        verify_plan_hash(
            "sha256:" + "0" * 64,
            verb=_VERB,
            port=_PORT,
            action=_ACTION,
            info=info,
        )
    assert exc.value.code == EXIT_ENV_ERROR


def test_verify_plan_hash_operator_ignored() -> None:
    """Hash built with one operator verifies fine with different live state
    (operator is excluded from canonical form)."""
    from arm101.cli._consent import build_plan, verify_plan_hash

    info = _make_info()
    plan = build_plan(_VERB, _PORT, info, _ACTION, operator="alice", created_at=_CREATED_AT)
    # Must not raise even though "operator" is conceptually different
    verify_plan_hash(
        plan["plan_hash"],
        verb=_VERB,
        port=_PORT,
        action=_ACTION,
        info=info,
    )


def test_verify_plan_hash_tolerates_surrounding_whitespace() -> None:
    """A hash read from the plan file with a trailing newline/space still verifies.

    resolve_consent() strips --plan-hash before format-validating it; verify must
    normalize the same way, or a valid copy/file-read hash is falsely refused.
    """
    from arm101.cli._consent import build_plan, verify_plan_hash

    info = _make_info()
    plan = build_plan(_VERB, _PORT, info, _ACTION, operator="op", created_at=_CREATED_AT)

    # Trailing newline + leading/trailing spaces (realistic file-read / copy-paste).
    verify_plan_hash(
        f"  {plan['plan_hash']}\n",
        verb=_VERB,
        port=_PORT,
        action=_ACTION,
        info=info,
    )


# ---------------------------------------------------------------------------
# 5. resolve_operator
# ---------------------------------------------------------------------------


def test_resolve_operator_env_wins(monkeypatch) -> None:
    """ARM101_OPERATOR env var takes priority over everything else."""
    from arm101.cli._consent import resolve_operator

    monkeypatch.setenv("ARM101_OPERATOR", "robot-arm-1")
    assert resolve_operator() == "robot-arm-1"


def test_resolve_operator_env_wins_over_culture_yaml(monkeypatch) -> None:
    """Even when culture.yaml is present, ARM101_OPERATOR takes priority."""
    from arm101.cli._consent import resolve_operator

    monkeypatch.setenv("ARM101_OPERATOR", "override")
    result = resolve_operator()
    assert result == "override"


def test_resolve_operator_falls_back_when_env_unset(monkeypatch) -> None:
    """When ARM101_OPERATOR is unset, returns a non-empty string."""
    from arm101.cli._consent import resolve_operator

    monkeypatch.delenv("ARM101_OPERATOR", raising=False)
    result = resolve_operator()
    assert result
    assert isinstance(result, str)


def test_resolve_operator_tty_fallback_when_no_culture_yaml(monkeypatch) -> None:
    """When culture.yaml is absent, falls back to 'tty:<user>'."""
    import arm101.cli._commands.whoami as wa
    from arm101.cli._consent import resolve_operator

    monkeypatch.delenv("ARM101_OPERATOR", raising=False)
    monkeypatch.setattr(wa, "find_culture_yaml", lambda: None)
    result = resolve_operator()
    assert result.startswith("tty:")
    # Must contain a non-empty username
    assert len(result) > len("tty:")


# ---------------------------------------------------------------------------
# 6. write_audit
# ---------------------------------------------------------------------------


def test_write_audit_appends_jsonl(tmp_path, monkeypatch) -> None:
    """write_audit appends one JSONL line per call."""
    from arm101.cli._consent import build_audit_record, write_audit

    log_path = str(tmp_path / "audit.log")
    monkeypatch.setenv("ARM101_AUDIT_LOG", log_path)

    record = build_audit_record(
        verb=_VERB,
        port=_PORT,
        operator="op",
        consent_mode="agent",
        action=_ACTION,
        outcome="success",
    )
    write_audit(record)
    write_audit(record)  # two separate calls → two lines

    lines = Path(log_path).read_text().splitlines()
    assert len(lines) == 2
    parsed = json.loads(lines[0])
    assert parsed["verb"] == _VERB
    assert parsed["outcome"] == "success"


def test_write_audit_json_line_has_required_keys(tmp_path, monkeypatch) -> None:
    """Each JSONL line has the required keys."""
    from arm101.cli._consent import build_audit_record, write_audit

    log_path = str(tmp_path / "audit.log")
    monkeypatch.setenv("ARM101_AUDIT_LOG", log_path)

    record = build_audit_record(
        verb=_VERB,
        port=_PORT,
        operator="op",
        consent_mode="interactive",
        action=_ACTION,
        outcome="pending",
    )
    write_audit(record)

    parsed = json.loads(Path(log_path).read_text().strip())
    required = {"ts", "verb", "port", "operator", "consent_mode", "action", "outcome"}
    assert required.issubset(parsed.keys())


def test_write_audit_with_plan_hash(tmp_path, monkeypatch) -> None:
    """build_audit_record includes optional plan_hash when supplied."""
    from arm101.cli._consent import build_audit_record, write_audit

    log_path = str(tmp_path / "audit.log")
    monkeypatch.setenv("ARM101_AUDIT_LOG", log_path)

    record = build_audit_record(
        verb=_VERB,
        port=_PORT,
        operator="op",
        consent_mode="agent",
        action=_ACTION,
        outcome="pending",
        plan_hash="sha256:" + "a" * 64,
    )
    write_audit(record)

    parsed = json.loads(Path(log_path).read_text().strip())
    assert "plan_hash" in parsed
    assert parsed["plan_hash"].startswith("sha256:")


def test_write_audit_unwritable_path_does_not_raise(tmp_path, monkeypatch) -> None:
    """Pointing ARM101_AUDIT_LOG at a directory (not a file) does not raise."""
    from arm101.cli._consent import build_audit_record, write_audit

    # A directory path cannot be opened for writing as a file
    monkeypatch.setenv("ARM101_AUDIT_LOG", str(tmp_path))

    record = build_audit_record(
        verb=_VERB,
        port=_PORT,
        operator="op",
        consent_mode="dry_run",
        action=_ACTION,
        outcome="pending",
    )
    # Must not raise
    write_audit(record)


def test_write_audit_creates_parent_dirs(tmp_path, monkeypatch) -> None:
    """write_audit creates missing parent directories."""
    from arm101.cli._consent import build_audit_record, write_audit

    nested = str(tmp_path / "nested" / "deep" / "audit.log")
    monkeypatch.setenv("ARM101_AUDIT_LOG", nested)

    record = build_audit_record(
        verb=_VERB,
        port=_PORT,
        operator="op",
        consent_mode="interactive",
        action=_ACTION,
        outcome="success",
    )
    write_audit(record)
    assert Path(nested).is_file()


# ---------------------------------------------------------------------------
# 7. build_audit_record — shape
# ---------------------------------------------------------------------------


def test_build_audit_record_required_keys() -> None:
    """build_audit_record includes all required keys."""
    from arm101.cli._consent import build_audit_record

    record = build_audit_record(
        verb=_VERB,
        port=_PORT,
        operator="op",
        consent_mode="interactive",
        action=_ACTION,
        outcome="success",
    )
    required = {"ts", "verb", "port", "operator", "consent_mode", "action", "outcome"}
    assert required.issubset(record.keys())


def test_build_audit_record_optional_keys_absent_by_default() -> None:
    """plan_hash and error are absent when not supplied."""
    from arm101.cli._consent import build_audit_record

    record = build_audit_record(
        verb=_VERB,
        port=_PORT,
        operator="op",
        consent_mode="interactive",
        action=_ACTION,
        outcome="success",
    )
    assert "plan_hash" not in record
    assert "error" not in record


def test_build_audit_record_with_error() -> None:
    """build_audit_record includes error key when supplied."""
    from arm101.cli._consent import build_audit_record

    record = build_audit_record(
        verb=_VERB,
        port=_PORT,
        operator="op",
        consent_mode="agent",
        action=_ACTION,
        outcome="failed",
        error="something went wrong",
    )
    assert record["error"] == "something went wrong"
    assert record["outcome"] == "failed"


def test_build_audit_record_ts_is_iso8601() -> None:
    """build_audit_record 'ts' field is an ISO 8601 timestamp string."""
    from datetime import datetime

    from arm101.cli._consent import build_audit_record

    record = build_audit_record(
        verb=_VERB,
        port=_PORT,
        operator="op",
        consent_mode="dry_run",
        action=_ACTION,
        outcome="pending",
    )
    # Must parse without raising
    dt = datetime.fromisoformat(record["ts"])
    assert dt.tzinfo is not None
