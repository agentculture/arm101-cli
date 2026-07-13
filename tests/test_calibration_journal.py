"""The calibration journal — a temporary encoder offset is a TRANSACTION.

These tests pin the contract that makes ``arm limits`` safe to crash:

* the journal names the ORIGINAL offset on disk **before** the first
  ``write_offset`` — proven against a bus whose ``write_offset`` always fails;
* a dirty journal survives into a fresh process and the recovery restores the
  recorded original before anything else happens;
* the restore survives its OWN failure, per motor, independently.

The SIGKILL half of the contract lives in
``tests/test_calibration_journal_sigkill.py`` — it needs a real process to kill.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

import pytest

from arm101.cli._errors import EXIT_ENV_ERROR, EXIT_USER_ERROR, CliError
from arm101.hardware import journal as journal_mod
from arm101.hardware.bus import FakeBus, OverloadError
from arm101.hardware.journal import (
    JOURNAL_ENV_VAR,
    CalibrationJournal,
    commit,
    default_journal_path,
    require_clean,
    restore_dirty,
    shift_offset,
)

ELBOW = "elbow_flex"
MOTOR = 3
ORIGINAL = 85
TEMPORARY = 900


# ---------------------------------------------------------------------------
# Buses that fail the way real ones fail
# ---------------------------------------------------------------------------


class _OpenedFakeBus(FakeBus):
    """A :class:`FakeBus` that is already open — every test here writes offsets."""

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.open()


class _WriteAlwaysFailsBus(_OpenedFakeBus):
    """Its ``write_offset`` NEVER lands. The journal must already name the original."""

    def write_offset(self, motor: int, offset: int) -> None:
        raise CliError(
            code=EXIT_ENV_ERROR,
            message=f"Simulated EEPROM write fault on motor {motor}.",
            remediation="none — this bus exists to fail.",
        )


class _OneMotorRefusesBus(_OpenedFakeBus):
    """``write_offset`` works for every motor except *refuses* — which always faults."""

    def __init__(self, refuses: int, **kwargs) -> None:
        self.refuses = refuses
        super().__init__(**kwargs)

    def write_offset(self, motor: int, offset: int) -> None:
        if motor == self.refuses:
            raise CliError(
                code=EXIT_ENV_ERROR,
                message=f"Simulated EEPROM write fault on motor {motor}.",
                remediation="none — this bus exists to fail.",
            )
        super().write_offset(motor, offset)


class _LiesOnReadBackBus(_OpenedFakeBus):
    """Accepts the write, then reports a DIFFERENT offset — the write did not take."""

    def read_offset(self, motor: int) -> int:
        return super().read_offset(motor) + 1


class _LatchedOverloadBus(_OpenedFakeBus):
    """Latched in overload: ``enable_torque`` raises until ``clear_overload`` runs.

    Models the servo a crashed probe actually leaves behind — driven into a wall,
    overload bit set, answering every packet with it. ``write_offset``'s first act
    is ``enable_torque(motor, False)``, so a restore that does not clear the latch
    first never reaches the EEPROM at all.
    """

    def __init__(self, **kwargs) -> None:
        self.latched = True
        self.calls: list[str] = []
        super().__init__(**kwargs)

    def enable_torque(self, motor: int, enabled: bool) -> None:
        if self.latched and not enabled:
            raise OverloadError(motor=motor, error_byte=32)
        super().enable_torque(motor, enabled)

    def clear_overload(self, motor: int) -> None:
        self.calls.append("clear_overload")
        self.latched = False
        super().clear_overload(motor)

    def write_offset(self, motor: int, offset: int) -> None:
        self.calls.append("write_offset")
        super().write_offset(motor, offset)


# ---------------------------------------------------------------------------
# Where the journal lives
# ---------------------------------------------------------------------------


def test_default_journal_path_honours_the_env_var(monkeypatch, tmp_path):
    monkeypatch.setenv(JOURNAL_ENV_VAR, str(tmp_path / "elsewhere.jsonl"))
    assert default_journal_path() == tmp_path / "elsewhere.jsonl"


def test_default_journal_path_falls_back_to_the_arm101_home(monkeypatch, tmp_path):
    monkeypatch.delenv(JOURNAL_ENV_VAR, raising=False)
    monkeypatch.setattr(Path, "home", classmethod(lambda cls: tmp_path))
    assert default_journal_path() == tmp_path / ".arm101" / "calibration-journal.jsonl"


def test_an_explicit_path_beats_the_env_var(monkeypatch, tmp_path):
    """Injectable per-instance — xdist runs tests in parallel; no global path may be shared."""
    monkeypatch.setenv(JOURNAL_ENV_VAR, str(tmp_path / "env.jsonl"))
    explicit = tmp_path / "explicit.jsonl"
    assert CalibrationJournal(explicit).path == explicit


# ---------------------------------------------------------------------------
# Criterion 1 — DURABLE BEFORE THE WRITE
# ---------------------------------------------------------------------------


def test_the_journal_names_the_original_before_a_write_that_never_lands(tmp_path):
    """A bus whose ``write_offset`` ALWAYS fails.

    The write is the thing the journal exists to undo. If the journal were
    written *after* it, this is exactly the case in which it would protect
    nothing — so the journal must already be on disk, naming 85, by the time the
    write is even attempted.
    """
    path = tmp_path / "journal.jsonl"
    bus = _WriteAlwaysFailsBus(positions={MOTOR: 2048}, offsets={MOTOR: ORIGINAL})

    with pytest.raises(CliError):
        shift_offset(bus, CalibrationJournal(path), joint=ELBOW, motor=MOTOR, offset=TEMPORARY)

    # Read it back through a FRESH object — nothing in memory may be trusted here.
    dirty = CalibrationJournal(path).dirty_entries()
    assert len(dirty) == 1
    assert dirty[0].joint == ELBOW
    assert dirty[0].motor == MOTOR
    assert dirty[0].original_offset == ORIGINAL
    assert dirty[0].temporary_offsets == (TEMPORARY,)


def test_the_journal_is_fsynced_before_the_first_write_offset(tmp_path, monkeypatch):
    """Durability means fsync. A write still sitting in the page cache is not a journal.

    Records the interleaving of ``os.fsync`` calls and ``write_offset`` calls and
    asserts an fsync happened *first* — a `write()`+`flush()` that a power cut
    eats is indistinguishable from never having journalled at all.
    """
    events: list[str] = []
    real_fsync = os.fsync

    def spy_fsync(fd):
        events.append("fsync")
        return real_fsync(fd)

    monkeypatch.setattr(journal_mod.os, "fsync", spy_fsync)

    class _RecordingBus(_OpenedFakeBus):
        def write_offset(self, motor: int, offset: int) -> None:
            events.append("write_offset")
            super().write_offset(motor, offset)

    bus = _RecordingBus(positions={MOTOR: 2048}, offsets={MOTOR: ORIGINAL})
    shift_offset(
        bus,
        CalibrationJournal(tmp_path / "journal.jsonl"),
        joint=ELBOW,
        motor=MOTOR,
        offset=TEMPORARY,
    )

    assert "write_offset" in events
    assert events.index("fsync") < events.index("write_offset")


def test_every_temporary_offset_is_journalled_before_it_is_written(tmp_path):
    path = tmp_path / "journal.jsonl"
    bus = _OpenedFakeBus(positions={MOTOR: 2048}, offsets={MOTOR: ORIGINAL})
    journal = CalibrationJournal(path)

    shift_offset(bus, journal, joint=ELBOW, motor=MOTOR, offset=TEMPORARY)
    shift_offset(bus, journal, joint=ELBOW, motor=MOTOR, offset=1200)

    entry = CalibrationJournal(path).dirty_entries()[0]
    # The SECOND shift must not overwrite the original with the temporary one in
    # force at the time — that would journal a lie and lose the real calibration.
    assert entry.original_offset == ORIGINAL
    assert entry.temporary_offsets == (TEMPORARY, 1200)
    assert entry.last_written_offset == 1200


def test_recording_an_offset_with_no_open_transaction_is_refused(tmp_path):
    journal = CalibrationJournal(tmp_path / "journal.jsonl")
    with pytest.raises(CliError) as excinfo:
        journal.record_offset(motor=MOTOR, offset=TEMPORARY)
    assert excinfo.value.code == EXIT_USER_ERROR


def test_beginning_an_already_open_transaction_keeps_the_FIRST_original(tmp_path):
    """The guard that makes a RESUMED run safe.

    A second ``begin`` for the same motor — a resumed probe, a re-entered helper —
    must not re-read the servo and record whatever *temporary* offset is in force
    as if it were the original. That would journal a lie and destroy the only copy
    of the number the restore needs, which is the whole failure this module exists
    to prevent, committed by the module itself.
    """
    path = tmp_path / "journal.jsonl"
    journal = CalibrationJournal(path)

    journal.begin(joint=ELBOW, motor=MOTOR, original_offset=ORIGINAL)
    journal.record_offset(motor=MOTOR, offset=TEMPORARY)
    # A caller who wrongly believes the CURRENT (temporary) offset is the original.
    returned = journal.begin(joint=ELBOW, motor=MOTOR, original_offset=TEMPORARY)

    assert returned.original_offset == ORIGINAL
    entry = CalibrationJournal(path).dirty_entries()[0]
    assert entry.original_offset == ORIGINAL
    assert entry.temporary_offsets == (TEMPORARY,)  # no second 'begin' was written


def test_closing_a_motor_that_was_never_opened_is_refused(tmp_path):
    journal = CalibrationJournal(tmp_path / "journal.jsonl")
    with pytest.raises(CliError) as excinfo:
        journal.end(motor=MOTOR, disposition="restored")
    assert excinfo.value.code == EXIT_USER_ERROR


def test_clearing_a_journal_that_was_never_written_is_harmless(tmp_path):
    journal = CalibrationJournal(tmp_path / "journal.jsonl")
    journal.clear()
    assert not journal.path.exists()


def test_blank_lines_and_headless_records_are_skipped(tmp_path):
    """Journals are appended to by processes that get killed. Be generous reading them."""
    path = tmp_path / "journal.jsonl"
    journal = CalibrationJournal(path)
    journal.begin(joint=ELBOW, motor=MOTOR, original_offset=ORIGINAL)
    with path.open("a", encoding="utf-8") as handle:
        handle.write("\n")
        handle.write(json.dumps({"event": "offset", "offset": 999}) + "\n")  # no motor
        handle.write(json.dumps({"event": "offset", "motor": 99, "offset": 5}) + "\n")  # no begin
        handle.write(json.dumps({"event": "end", "motor": 99, "disposition": "restored"}) + "\n")

    dirty = CalibrationJournal(path).dirty_entries()
    assert [e.motor for e in dirty] == [MOTOR]
    assert dirty[0].temporary_offsets == ()


# ---------------------------------------------------------------------------
# Criterion 2 (in-process half) — a dirty journal is detected and restored
# ---------------------------------------------------------------------------


def test_a_dirty_journal_is_detected_and_the_original_restored(tmp_path):
    path = tmp_path / "journal.jsonl"
    bus = _OpenedFakeBus(positions={MOTOR: 2048}, offsets={MOTOR: ORIGINAL})

    shift_offset(bus, CalibrationJournal(path), joint=ELBOW, motor=MOTOR, offset=TEMPORARY)
    assert bus.read_offset(MOTOR) == TEMPORARY

    # A FRESH journal object — as a fresh process would have.
    journal = CalibrationJournal(path)
    assert journal.is_dirty()

    report = restore_dirty(bus, journal)

    assert report.complete
    assert [o.motor for o in report.outcomes] == [MOTOR]
    assert report.outcomes[0].original_offset == ORIGINAL
    assert report.outcomes[0].restored
    assert bus.read_offset(MOTOR) == ORIGINAL
    assert not CalibrationJournal(path).is_dirty()


def test_a_restored_journal_is_cleared_from_disk(tmp_path):
    path = tmp_path / "journal.jsonl"
    bus = _OpenedFakeBus(positions={MOTOR: 2048}, offsets={MOTOR: ORIGINAL})
    journal = CalibrationJournal(path)

    shift_offset(bus, journal, joint=ELBOW, motor=MOTOR, offset=TEMPORARY)
    restore_dirty(bus, journal)

    assert path.read_text(encoding="utf-8") == ""
    assert CalibrationJournal(path).dirty_entries() == []


def test_restore_clears_a_latched_overload_before_it_writes(tmp_path):
    """The joint a crashed probe leaves behind is the joint that is latched.

    ``write_offset`` opens with ``enable_torque(motor, False)``, which a latched
    servo answers with the overload bit still set. Restore therefore has to
    de-latch first or it never reaches addr 31.
    """
    path = tmp_path / "journal.jsonl"
    bus = _LatchedOverloadBus(positions={MOTOR: 2048}, offsets={MOTOR: ORIGINAL})
    journal = CalibrationJournal(path)
    journal.begin(joint=ELBOW, motor=MOTOR, original_offset=ORIGINAL)
    journal.record_offset(motor=MOTOR, offset=TEMPORARY)

    report = restore_dirty(bus, journal)

    assert report.complete
    assert bus.calls[0] == "clear_overload"
    assert "write_offset" in bus.calls
    assert bus.read_offset(MOTOR) == ORIGINAL


def test_require_clean_on_a_clean_journal_never_touches_the_bus(tmp_path):
    bus = _OpenedFakeBus(positions={MOTOR: 2048}, offsets={MOTOR: ORIGINAL})
    bus.close()  # a closed bus raises on any call — proof nothing was attempted

    report = require_clean(bus, CalibrationJournal(tmp_path / "journal.jsonl"))

    assert report.complete
    assert report.outcomes == ()


def test_require_clean_restores_before_anything_else_happens(tmp_path):
    path = tmp_path / "journal.jsonl"
    bus = _OpenedFakeBus(positions={MOTOR: 2048}, offsets={MOTOR: ORIGINAL})
    shift_offset(bus, CalibrationJournal(path), joint=ELBOW, motor=MOTOR, offset=TEMPORARY)

    report = require_clean(bus, CalibrationJournal(path))

    assert report.complete
    assert bus.read_offset(MOTOR) == ORIGINAL


def test_require_clean_refuses_to_continue_when_a_restore_fails(tmp_path):
    path = tmp_path / "journal.jsonl"
    bus = _OneMotorRefusesBus(refuses=MOTOR, positions={MOTOR: 2048}, offsets={MOTOR: ORIGINAL})
    journal = CalibrationJournal(path)
    journal.begin(joint=ELBOW, motor=MOTOR, original_offset=ORIGINAL)
    journal.record_offset(motor=MOTOR, offset=TEMPORARY)

    with pytest.raises(CliError) as excinfo:
        require_clean(bus, journal)

    assert excinfo.value.code == EXIT_ENV_ERROR
    # The message has to carry the number a human needs to fix this by hand.
    assert str(ORIGINAL) in excinfo.value.message
    assert ELBOW in excinfo.value.message
    assert str(path) in excinfo.value.remediation
    # And the entry stays dirty, so the NEXT run tries again.
    assert CalibrationJournal(path).is_dirty()


# ---------------------------------------------------------------------------
# Criterion 3 — the restore survives its own failure, per motor
# ---------------------------------------------------------------------------


def test_one_motors_failed_restore_does_not_strand_the_others(tmp_path):
    path = tmp_path / "journal.jsonl"
    bus = _OneMotorRefusesBus(
        refuses=2,
        positions={1: 2048, 2: 2048, 3: 2048},
        offsets={1: 10, 2: 20, 3: 30},
    )
    journal = CalibrationJournal(path)
    for motor, original in ((1, 10), (2, 20), (3, 30)):
        journal.begin(joint=f"joint{motor}", motor=motor, original_offset=original)
        journal.record_offset(motor=motor, offset=1500)
    # Force the temporary offset onto motors 1 and 3 (2 refuses every write).
    FakeBus.write_offset(bus, 1, 1500)
    FakeBus.write_offset(bus, 3, 1500)

    report = restore_dirty(bus, journal)

    assert not report.complete
    assert sorted(report.restored) == [1, 3]
    assert report.failed == (2,)
    assert 2 in report.errors
    # The bus refused motor 2 FIRST in id order — and 3 was still restored.
    assert bus.read_offset(1) == 10
    assert bus.read_offset(3) == 30
    # Motor 2 stays dirty: its original is the only record of the truth.
    dirty = CalibrationJournal(path).dirty_entries()
    assert [e.motor for e in dirty] == [2]
    assert dirty[0].original_offset == 20


def test_an_unverified_restore_never_closes_its_entry(tmp_path):
    """The write "succeeded" but the read-back disagrees. That is not a restore.

    Closing the entry here would discard the only record of the original offset
    while the servo is still holding something else — the exact silent
    mis-calibration the journal exists to prevent.
    """
    path = tmp_path / "journal.jsonl"
    bus = _LiesOnReadBackBus(positions={MOTOR: 2048}, offsets={MOTOR: ORIGINAL})
    journal = CalibrationJournal(path)
    journal.begin(joint=ELBOW, motor=MOTOR, original_offset=ORIGINAL)
    journal.record_offset(motor=MOTOR, offset=TEMPORARY)

    report = restore_dirty(bus, journal)

    assert not report.complete
    assert report.failed == (MOTOR,)
    assert not report.outcomes[0].restored
    assert CalibrationJournal(path).is_dirty()


def test_the_restore_report_renders_as_json(tmp_path):
    """The verb reports this to an agent — and the SIGKILL recovery ships it over a pipe."""
    path = tmp_path / "journal.jsonl"
    bus = _OpenedFakeBus(positions={MOTOR: 2048}, offsets={MOTOR: ORIGINAL})
    journal = CalibrationJournal(path)
    shift_offset(bus, journal, joint=ELBOW, motor=MOTOR, offset=TEMPORARY)

    payload = restore_dirty(bus, CalibrationJournal(path)).to_dict()

    assert payload == {
        "complete": True,
        "outcomes": [
            {
                "joint": ELBOW,
                "motor": MOTOR,
                "original_offset": ORIGINAL,
                "restored": True,
                "read_back": ORIGINAL,
                "error": None,
            }
        ],
        "restored": [MOTOR],
        "failed": [],
        "errors": {},
    }
    assert json.dumps(payload)  # it has to survive the pipe, so it has to serialise


# ---------------------------------------------------------------------------
# Committing — the OTHER way a journal entry closes
# ---------------------------------------------------------------------------


def test_commit_clears_the_entry_and_keeps_the_new_calibration(tmp_path):
    path = tmp_path / "journal.jsonl"
    bus = _OpenedFakeBus(positions={MOTOR: 2048}, offsets={MOTOR: ORIGINAL})
    journal = CalibrationJournal(path)
    shift_offset(bus, journal, joint=ELBOW, motor=MOTOR, offset=TEMPORARY)

    commit(journal, motor=MOTOR)

    assert not CalibrationJournal(path).is_dirty()
    assert bus.read_offset(MOTOR) == TEMPORARY  # deliberately kept


def test_committing_an_unknown_motor_is_refused(tmp_path):
    journal = CalibrationJournal(tmp_path / "journal.jsonl")
    with pytest.raises(CliError) as excinfo:
        commit(journal, motor=MOTOR)
    assert excinfo.value.code == EXIT_USER_ERROR


def test_only_the_closed_motor_is_cleared(tmp_path):
    path = tmp_path / "journal.jsonl"
    journal = CalibrationJournal(path)
    journal.begin(joint="a", motor=1, original_offset=10)
    journal.record_offset(motor=1, offset=1500)
    journal.begin(joint="b", motor=2, original_offset=20)
    journal.record_offset(motor=2, offset=1500)

    commit(journal, motor=1)

    dirty = CalibrationJournal(path).dirty_entries()
    assert [e.motor for e in dirty] == [2]
    assert dirty[0].original_offset == 20


# ---------------------------------------------------------------------------
# Reading a journal a crash wrote
# ---------------------------------------------------------------------------


def test_a_truncated_final_line_is_tolerated(tmp_path):
    """A crash mid-append leaves a partial last line. Everything before it is still true."""
    path = tmp_path / "journal.jsonl"
    journal = CalibrationJournal(path)
    journal.begin(joint=ELBOW, motor=MOTOR, original_offset=ORIGINAL)
    journal.record_offset(motor=MOTOR, offset=TEMPORARY)
    with path.open("a", encoding="utf-8") as handle:
        handle.write('{"event": "offset", "motor": 3, "off')

    entry = CalibrationJournal(path).dirty_entries()[0]
    assert entry.original_offset == ORIGINAL
    assert entry.temporary_offsets == (TEMPORARY,)


def test_corruption_in_the_middle_of_the_journal_is_loud(tmp_path):
    """A garbled line that is NOT the last one is not a crash artefact — it is damage.

    Silently skipping it could drop the very ``begin`` record that names an
    original offset, turning a recoverable arm into an unrecoverable one while
    reporting success.
    """
    path = tmp_path / "journal.jsonl"
    journal = CalibrationJournal(path)
    journal.begin(joint=ELBOW, motor=MOTOR, original_offset=ORIGINAL)
    with path.open("a", encoding="utf-8") as handle:
        handle.write("not json at all\n")
        handle.write(json.dumps({"event": "offset", "motor": MOTOR, "offset": TEMPORARY}) + "\n")

    with pytest.raises(CliError) as excinfo:
        CalibrationJournal(path).entries()
    assert excinfo.value.code == EXIT_ENV_ERROR


def test_a_missing_journal_is_simply_clean(tmp_path):
    journal = CalibrationJournal(tmp_path / "nothing" / "here.jsonl")
    assert not journal.is_dirty()
    assert journal.entries() == []
    assert journal.dirty_entries() == []


def test_entries_render_as_json_for_the_verb_to_report(tmp_path):
    journal = CalibrationJournal(tmp_path / "journal.jsonl")
    journal.begin(joint=ELBOW, motor=MOTOR, original_offset=ORIGINAL)
    journal.record_offset(motor=MOTOR, offset=TEMPORARY)

    payload = CalibrationJournal(journal.path).dirty_entries()[0].to_dict()

    assert payload == {
        "joint": ELBOW,
        "motor": MOTOR,
        "original_offset": ORIGINAL,
        "temporary_offsets": [TEMPORARY],
        "closed": False,
        "disposition": None,
    }
