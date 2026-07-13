"""SIGKILL, for real — not a ``finally`` block reasoned about.

The acceptance criterion this file exists for::

    SIGKILL the process mid-probe with a temporary offset in force; the NEXT
    invocation detects the dirty calibration, names the original offset, and
    restores it before doing anything else.

Every cheaper way to "test" this is a lie. ``pytest.raises`` proves a ``finally``
runs on an *exception*; SIGKILL does not raise. ``KeyboardInterrupt`` proves the
Ctrl-C path; SIGKILL is not deliverable to Python at all — no handler, no
``atexit``, no cleanup, nothing. The failure mode this whole module defends
against is precisely the one where none of the code runs: a hard kill, an OOM
reap, a power cut. So the test spawns a genuine subprocess (``tests/_journal_
subject.py``), waits until it has genuinely written a temporary offset to a
persistent "EEPROM", and genuinely SIGKILLs it — then asks a *fresh process* to
put the arm back.

The EEPROM is a file (:class:`tests._fakes.EepromFileBus`), because the whole
premise of the journal is that the servo's addr-31 register outlives the process
that wrote it. Model it in memory and death would tidy up the mess for us.
"""

from __future__ import annotations

import json
import os
import signal
import subprocess  # nosec B404 - spawning our own test subject, no shell, fixed argv
import sys
import time
from pathlib import Path

import pytest

from arm101.hardware.journal import CalibrationJournal
from tests._fakes import EepromFileBus

SUBJECT = Path(__file__).parent / "_journal_subject.py"
REPO_ROOT = Path(__file__).resolve().parents[1]

JOINT = "elbow_flex"
MOTOR = 3
ORIGINAL = 85
TEMPORARY = 1963

#: The parent will not wait longer than this for the child to get dirty.
READY_TIMEOUT_SECONDS = 30.0


def _wait_for(path: Path, proc: subprocess.Popen) -> None:
    """Block until *path* appears, or fail loudly if the child dies or stalls."""
    deadline = time.monotonic() + READY_TIMEOUT_SECONDS
    while time.monotonic() < deadline:
        if path.exists():
            return
        if proc.poll() is not None:
            _, err = proc.communicate()
            pytest.fail(f"the probe died before it got dirty (rc={proc.returncode}): {err}")
        time.sleep(0.02)
    proc.kill()
    pytest.fail(f"the probe never reported ready within {READY_TIMEOUT_SECONDS}s")


def _run(args: list[str], env: dict) -> subprocess.CompletedProcess:
    return subprocess.run(  # nosec B603 - fixed argv, no shell, our own script
        [sys.executable, str(SUBJECT), *args],
        capture_output=True,
        text=True,
        env=env,
        check=False,
        timeout=60,
    )


@pytest.fixture
def env():
    """A child environment that can import ``arm101`` and ``tests`` from the repo."""
    child = dict(os.environ)
    child["PYTHONPATH"] = os.pathsep.join([str(REPO_ROOT), child.get("PYTHONPATH", "")]).rstrip(
        os.pathsep
    )
    return child


def test_sigkill_mid_probe_is_recovered_by_the_next_invocation(tmp_path, env):
    """Kill it dead with a temporary offset in force. The next run must undo it."""
    eeprom = tmp_path / "eeprom.json"
    journal_path = tmp_path / "calibration-journal.jsonl"
    ready = tmp_path / "ready"
    eeprom.write_text(json.dumps({str(MOTOR): ORIGINAL}), encoding="utf-8")

    proc = subprocess.Popen(  # nosec B603 - fixed argv, no shell, our own script
        [
            sys.executable,
            str(SUBJECT),
            "probe",
            str(eeprom),
            str(journal_path),
            str(ready),
            JOINT,
            str(MOTOR),
            str(TEMPORARY),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=env,
    )
    try:
        _wait_for(ready, proc)

        # SIGKILL. Not SIGTERM, not SIGINT — nothing in the child gets to run.
        os.kill(proc.pid, signal.SIGKILL)
        proc.wait(timeout=30)
    finally:
        if proc.poll() is None:  # pragma: no cover - only if the kill itself failed
            proc.kill()
            proc.wait(timeout=10)

    assert proc.returncode == -signal.SIGKILL, "the child was not actually SIGKILLed"

    # The arm is genuinely dirty: the servo is holding a calibration the
    # process that wrote it is no longer alive to remember.
    assert EepromFileBus.read_eeprom(eeprom) == {MOTOR: TEMPORARY}

    # And the ONLY record of the truth is the journal — durable, on disk,
    # written before the offset it names.
    dirty = CalibrationJournal(journal_path).dirty_entries()
    assert [e.motor for e in dirty] == [MOTOR]
    assert dirty[0].joint == JOINT
    assert dirty[0].original_offset == ORIGINAL
    assert dirty[0].temporary_offsets == (TEMPORARY,)

    # The NEXT invocation — a brand-new process, no shared memory, no fixtures.
    result = _run(["recover", str(eeprom), str(journal_path)], env)

    assert result.returncode == 0, result.stderr
    report = json.loads(result.stdout)
    assert report["complete"] is True
    assert report["restored"] == [MOTOR]
    assert report["outcomes"][0]["original_offset"] == ORIGINAL
    assert report["outcomes"][0]["joint"] == JOINT

    # The servo is back where it started, and the journal is clean.
    assert EepromFileBus.read_eeprom(eeprom) == {MOTOR: ORIGINAL}
    assert not CalibrationJournal(journal_path).is_dirty()


def test_recovery_on_a_clean_journal_is_a_no_op(tmp_path, env):
    """The recovery path is safe to run unconditionally at every startup."""
    eeprom = tmp_path / "eeprom.json"
    eeprom.write_text(json.dumps({str(MOTOR): ORIGINAL}), encoding="utf-8")

    result = _run(["recover", str(eeprom), str(tmp_path / "absent.jsonl")], env)

    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout) == {
        "complete": True,
        "outcomes": [],
        "restored": [],
        "failed": [],
        "errors": {},
    }
    assert EepromFileBus.read_eeprom(eeprom) == {MOTOR: ORIGINAL}
