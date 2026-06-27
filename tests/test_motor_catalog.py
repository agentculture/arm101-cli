"""Tests for arm101.hardware.motor_catalog — persistence + load robustness.

The happy-path round-trip is exercised through ``test_calibrate_motor``; these
focus on the defensive load paths so a corrupt or hand-edited catalog fails
with a clean CliError rather than leaking a traceback.
"""

from __future__ import annotations

import json

import pytest

from arm101.cli._errors import EXIT_ENV_ERROR, CliError
from arm101.hardware import motor_catalog


def _write_catalog(monkeypatch, tmp_path, text: str) -> None:
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    path = motor_catalog.catalog_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def test_load_catalog_missing_returns_empty(monkeypatch, tmp_path) -> None:
    """No catalog file → empty mapping (not an error)."""
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    assert motor_catalog.load_catalog() == {}


def test_load_catalog_non_dict_root_raises_env_error(monkeypatch, tmp_path) -> None:
    """A JSON root that is not an object fails cleanly (Qodo #6)."""
    _write_catalog(monkeypatch, tmp_path, json.dumps(["not", "a", "dict"]))

    with pytest.raises(CliError) as exc:
        motor_catalog.load_catalog()
    assert exc.value.code == EXIT_ENV_ERROR


def test_load_catalog_scalar_root_raises_env_error(monkeypatch, tmp_path) -> None:
    """A bare scalar JSON root is also rejected, not crashed on."""
    _write_catalog(monkeypatch, tmp_path, "42")

    with pytest.raises(CliError) as exc:
        motor_catalog.load_catalog()
    assert exc.value.code == EXIT_ENV_ERROR


def test_load_catalog_invalid_json_raises_env_error(monkeypatch, tmp_path) -> None:
    """Malformed JSON fails cleanly with an env error."""
    _write_catalog(monkeypatch, tmp_path, "{not valid json")

    with pytest.raises(CliError) as exc:
        motor_catalog.load_catalog()
    assert exc.value.code == EXIT_ENV_ERROR
