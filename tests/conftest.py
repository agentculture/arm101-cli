"""Shared pytest fixtures for the arm101 test suite."""

from __future__ import annotations

import pytest


@pytest.fixture(autouse=True)
def _isolate_arm101_home(tmp_path, monkeypatch):
    """Pin arm101's audit log and plan dir into a per-test tmp location.

    The hardware-verb tests exercise ``write_audit`` / ``write_plan_file``,
    which default to ``~/.arm101/``. Without isolation the suite appends test
    records to the operator's *real* audit log (and scatters plan files in their
    home dir). This autouse fixture redirects both into ``tmp_path`` for every
    test, so no test can ever touch the real home directory.

    Tests that assert on the audit log / plan dir still set ``ARM101_AUDIT_LOG``
    / ``ARM101_PLAN_DIR`` themselves; because the test body runs after this
    fixture, their explicit ``setenv`` overrides the safe default below.
    """
    monkeypatch.setenv("ARM101_AUDIT_LOG", str(tmp_path / "audit.log"))
    monkeypatch.setenv("ARM101_PLAN_DIR", str(tmp_path / "plans"))
