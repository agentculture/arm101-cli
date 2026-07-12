"""Shared pytest fixtures for the arm101 test suite."""

from __future__ import annotations

import pytest


@pytest.fixture(autouse=True)
def _isolate_arm101_home(tmp_path, monkeypatch):
    """Pin arm101's audit log, plan dir, and calibration journal into a per-test tmp location.

    The hardware-verb tests exercise ``write_audit`` / ``write_plan_file``,
    which default to ``~/.arm101/``. Without isolation the suite appends test
    records to the operator's *real* audit log (and scatters plan files in their
    home dir). This autouse fixture redirects both into ``tmp_path`` for every
    test, so no test can ever touch the real home directory.

    The calibration journal (:mod:`arm101.hardware.journal`) is pinned for the
    same reason and one sharper one: it defaults to ``~/.arm101/`` too, and it is
    the record of whether a REAL servo is holding a temporary encoder offset. A
    test that wrote a fake dirty entry into the operator's real journal would
    have the next real run "restore" a joint that was never shifted. Tests also
    run in PARALLEL under xdist, so a single shared path would have them racing
    each other's transactions; ``tmp_path`` is per-test.

    Tests that assert on any of the three still set the env var themselves;
    because the test body runs after this fixture, their explicit ``setenv``
    overrides the safe default below.
    """
    monkeypatch.setenv("ARM101_AUDIT_LOG", str(tmp_path / "audit.log"))
    monkeypatch.setenv("ARM101_PLAN_DIR", str(tmp_path / "plans"))
    monkeypatch.setenv("ARM101_CALIBRATION_JOURNAL", str(tmp_path / "calibration-journal.jsonl"))
