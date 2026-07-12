"""The calibration journal — a temporary encoder offset is a TRANSACTION.

Measuring a joint's true travel means moving the encoder seam out of the way
ahead of the probe, and the only way to move the seam is to write the servo's
``Ofs`` register (``Homing_Offset``, EEPROM addr 31 — see
:meth:`arm101.hardware.bus.MotorBus.write_offset`). That write is **temporary**
in intent and **permanent** on the silicon: it is EEPROM, behind the addr-55
Lock dance, and it survives the process, the power cycle, and any opinion the
code had about how long it was meant to last.

Why this module exists
----------------------
If the process dies between the shift and the restore — SIGKILL, a Ctrl-C at the
wrong instant, an OOM reap, a yanked USB cable, a transient bus fault — the arm
is left holding a calibration **nobody recorded**. From that moment every tick in
:mod:`arm101.hardware.arm_spec`, every reachability map, every run-log, and every
goal position silently means a *different physical angle* for that joint. Nothing
raises. Nothing looks wrong. The arm just quietly lies, forever, and the only
cure is a human re-measuring a joint by hand.

This is not hypothetical. The bench script that first probed this feature did
exactly this dance without a journal, hit transient write faults mid-probe, and
left the arm in an unknown calibration. This module is the answer to that.

The contract, in the only order that works
------------------------------------------
1. **Journal first.** Read the joint's ORIGINAL offset off the servo, and write
   it to disk — ``write`` + ``flush`` + ``fsync``, plus an ``fsync`` of the
   parent directory the first time the file is created — **before** the first
   ``write_offset`` goes out on the wire.
2. **Write second.** Every temporary offset is journalled (durably) before it is
   written, so there is never an offset in a servo that is not already named on
   disk.
3. **Restore (or commit) third**, and only then is the entry cleared.

The ordering is the whole point and it is not a detail: *a journal written after
the write it exists to undo protects nothing.* :func:`shift_offset` is therefore
the only sanctioned way to write a temporary offset, and
``tests/test_calibration_journal.py`` pins the ordering against a bus whose
``write_offset`` always fails — the journal must already name the original.

Why ``fsync``, and why the directory too
----------------------------------------
"Durable" means *survives a power cut*, not *survives a return statement*. A
``write()`` that is still in the page cache when the machine loses power did not
happen. And a freshly created file is not durable until its **parent directory**
is also synced — the data can be on the platter while the directory entry that
names it is not. Both syncs are cheap (this is a handful of lines, a few times
per joint, against a run that takes minutes of motor travel) and each is the
difference between a recoverable arm and a bricked calibration.

Why SIGKILL is the test that matters
------------------------------------
A ``finally:`` block does not run on SIGKILL. Nor on a power cut, nor on an OOM
kill. ``atexit`` does not fire; context managers do not exit; signal handlers are
never even consulted. **The journal is the only thing that survives those** — so
``tests/test_calibration_journal_sigkill.py`` proves it by killing a real
subprocess that has really written a temporary offset to a really persistent
(file-backed) EEPROM, and then watching a fresh process put it back. Reasoning
about ``finally`` blocks would prove precisely nothing about the case this module
was built for.

The format on disk
------------------
Append-only JSONL — the same idiom as :mod:`arm101.explore.log`, and for the same
reason: an append is the smallest, least-destructive durable write there is. A
state machine that rewrote a whole document on every change would have a window
in which the document is neither the old truth nor the new one; an append never
does. Three record kinds, one per line::

    {"event": "begin",  "joint": "elbow_flex", "motor": 3, "original_offset": 85, "ts": ...}
    {"event": "offset", "motor": 3, "offset": 1963, "ts": ...}
    {"event": "end",    "motor": 3, "disposition": "restored", "ts": ...}

An entry is **dirty** when it has a ``begin`` and no ``end``: the servo may be
holding a temporary offset and the recorded ``original_offset`` is the only
record of what it used to be. When the last dirty entry closes, the file is
truncated — every state it passes through on the way (full-with-``end``, or
empty) is a *clean* state, so the truncation has no unsafe window.

Scope
-----
Zero third-party imports, and (like :mod:`arm101.hardware.safety`) no runtime
import of the bus at all: it only ever *calls* the ``MotorBus`` it is handed.
Single-process, single-arm: the journal is not a lock, and two ``arm101``
processes driving one bus at once is out of scope here (and already unsafe for
reasons that have nothing to do with calibration).
"""

from __future__ import annotations

import contextlib
import json
import os
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import TYPE_CHECKING, Union

from arm101.cli._errors import EXIT_ENV_ERROR, EXIT_USER_ERROR, CliError

if TYPE_CHECKING:  # pragma: no cover - typing only; never imported at runtime
    from arm101.hardware.bus import MotorBus

__all__ = [
    "JOURNAL_ENV_VAR",
    "DISPOSITION_COMMITTED",
    "DISPOSITION_RESTORED",
    "CalibrationJournal",
    "JournalEntry",
    "RestoreOutcome",
    "RestoreReport",
    "commit",
    "default_journal_path",
    "require_clean",
    "restore_dirty",
    "shift_offset",
]

#: Accepted path argument type throughout this module.
PathLike = Union[str, Path]

#: Environment variable that relocates the journal. Set it per-process (tests run
#: in PARALLEL under xdist — a single hard-coded global path would have them
#: racing each other's transactions, and would let a test scribble on the
#: operator's real arm state).
JOURNAL_ENV_VAR = "ARM101_CALIBRATION_JOURNAL"

#: Default journal file, under the same ``~/.arm101/`` the audit log and plan
#: files already live in (:mod:`arm101.cli._consent`).
DEFAULT_JOURNAL_NAME = "calibration-journal.jsonl"

# Record kinds. Deliberately not an Enum: these strings are the on-disk format,
# and an old journal written by an older arm101 must still be readable by a newer
# one — that file may be the only record of a real servo's real calibration.
EVENT_BEGIN = "begin"
EVENT_OFFSET = "offset"
EVENT_END = "end"

#: The joint was put back exactly as it was found. The temporary offset is gone.
DISPOSITION_RESTORED = "restored"

#: The new calibration was DELIBERATELY kept (a re-zero the operator asked for).
#: The servo still holds the last temporary offset — and that is now the truth.
DISPOSITION_COMMITTED = "committed"


def default_journal_path() -> Path:
    """Return the journal path: ``$ARM101_CALIBRATION_JOURNAL``, else the default."""
    env = os.environ.get(JOURNAL_ENV_VAR, "").strip()
    if env:
        return Path(env)
    return Path.home() / ".arm101" / DEFAULT_JOURNAL_NAME


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


# ---------------------------------------------------------------------------
# What a dirty journal says
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class JournalEntry:
    """One joint's in-flight (or finished) calibration transaction.

    Attributes
    ----------
    joint:
        Joint name, for the human who has to understand a recovery message.
    motor:
        Feetech servo id — the thing the restore actually writes to.
    original_offset:
        The offset the servo held **before** anything was written. This is the
        number the whole module exists to preserve: once the process that read
        it is dead, this record is the only place it exists.
    temporary_offsets:
        Every temporary offset written, in write order. The servo may be holding
        the last of them — or, if the process died mid-write, possibly not. The
        restore does not care: it writes :attr:`original_offset` either way.
    closed:
        ``True`` once the joint was restored or the calibration committed.
    disposition:
        :data:`DISPOSITION_RESTORED` or :data:`DISPOSITION_COMMITTED` once
        :attr:`closed`, else ``None``.
    """

    joint: str
    motor: int
    original_offset: int
    temporary_offsets: tuple[int, ...] = ()
    closed: bool = False
    disposition: "str | None" = None

    @property
    def last_written_offset(self) -> int:
        """The offset the servo is most likely holding right now.

        Best-effort and explicitly NOT trusted by :func:`restore_dirty` — a
        process killed *between* the journal append and the wire write leaves the
        servo holding the previous value while the journal already names the new
        one. Diagnostics only; the restore writes :attr:`original_offset`
        unconditionally, which is correct in both cases.
        """
        return self.temporary_offsets[-1] if self.temporary_offsets else self.original_offset

    def to_dict(self) -> dict:
        return {
            "joint": self.joint,
            "motor": self.motor,
            "original_offset": self.original_offset,
            "temporary_offsets": list(self.temporary_offsets),
            "closed": self.closed,
            "disposition": self.disposition,
        }


class CalibrationJournal:
    """The append-only, fsynced transaction log itself. Knows nothing about a bus.

    Parameters
    ----------
    path:
        Where the journal lives. ``None`` resolves :func:`default_journal_path`
        at construction time — pass an explicit path in tests, which run in
        parallel and must not share one.
    """

    def __init__(self, path: "PathLike | None" = None) -> None:
        self.path: Path = Path(path) if path is not None else default_journal_path()

    # -- writing -----------------------------------------------------------

    def _append(self, record: dict) -> None:
        """Append one record and make it DURABLE before returning.

        ``write`` -> ``flush`` -> ``fsync`` on the file, plus an ``fsync`` on the
        parent directory when the file has just been created (a file's *data* can
        be on the platter while the directory entry naming it is not — after a
        power cut that is a journal that does not exist).
        """
        parent = self.path.parent
        parent.mkdir(parents=True, exist_ok=True)
        is_new = not self.path.exists()

        with self.path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record) + "\n")
            handle.flush()
            os.fsync(handle.fileno())

        if is_new:
            _fsync_dir(parent)

    def begin(self, *, joint: str, motor: int, original_offset: int) -> JournalEntry:
        """Open a transaction for *motor*, recording the offset it holds RIGHT NOW.

        Idempotent by design: if *motor* already has a dirty entry (a resumed run,
        or a second shift within one run), the EXISTING entry is returned and
        nothing is written. Re-recording ``original_offset`` here would overwrite
        the true original with whatever *temporary* offset happens to be in force
        — journalling a lie, and destroying the only copy of the number the
        restore needs.
        """
        existing = self.dirty_entry_for(motor)
        if existing is not None:
            return existing
        self._append(
            {
                "event": EVENT_BEGIN,
                "joint": joint,
                "motor": motor,
                "original_offset": original_offset,
                "ts": _now(),
            }
        )
        return JournalEntry(joint=joint, motor=motor, original_offset=original_offset)

    def record_offset(self, *, motor: int, offset: int) -> None:
        """Record a temporary *offset* for *motor*. MUST be called before the wire write.

        Raises
        ------
        CliError(EXIT_USER_ERROR)
            If *motor* has no open transaction. A temporary offset written with
            no journalled original is the exact defect this module exists to
            make impossible, so it is refused rather than tolerated.
        """
        if self.dirty_entry_for(motor) is None:
            raise CliError(
                code=EXIT_USER_ERROR,
                message=(
                    f"Refusing to journal a temporary offset for motor {motor}: no open "
                    "calibration transaction, so its ORIGINAL offset was never recorded."
                ),
                remediation=(
                    "Call CalibrationJournal.begin() (or use journal.shift_offset(), which "
                    "does it for you) before writing any temporary offset."
                ),
            )
        self._append({"event": EVENT_OFFSET, "motor": motor, "offset": offset, "ts": _now()})

    def end(self, *, motor: int, disposition: str) -> None:
        """Close *motor*'s transaction — the joint is restored, or the change is kept.

        Clears the file once nothing is left in flight. Both the pre-truncate
        state (a journal whose every entry carries an ``end``) and the
        post-truncate state (an empty journal) are CLEAN, so there is no window
        in which a crash here could invent a dirty entry.

        Raises
        ------
        CliError(EXIT_USER_ERROR)
            If *motor* has no open transaction.
        """
        if self.dirty_entry_for(motor) is None:
            raise CliError(
                code=EXIT_USER_ERROR,
                message=f"No open calibration transaction for motor {motor} to close.",
                remediation=(
                    f"Inspect the journal at {self.path} — only a motor with an open "
                    "transaction can be restored or committed."
                ),
            )
        self._append({"event": EVENT_END, "motor": motor, "disposition": disposition, "ts": _now()})
        if not self.dirty_entries():
            self.clear()

    def clear(self) -> None:
        """Truncate the journal. Only ever called when nothing is in flight."""
        if not self.path.exists():
            return
        with self.path.open("w", encoding="utf-8") as handle:
            handle.flush()
            os.fsync(handle.fileno())

    # -- reading -----------------------------------------------------------

    def records(self) -> list[dict]:
        """Parse the raw records off disk, tolerating exactly one kind of damage.

        A crash *during* an append can leave a truncated final line — that line is
        skipped, because everything before it is still true and a partially
        written record describes nothing that happened.

        A garbled line anywhere *else* is not a crash artefact, it is damage, and
        it raises. Skipping it could silently drop a ``begin`` record — i.e.
        forget a servo's original offset — while cheerfully reporting the arm
        clean, which is worse than any error message.

        Raises
        ------
        CliError(EXIT_ENV_ERROR)
            If a non-final line is unparseable.
        """
        if not self.path.exists():
            return []

        lines = self.path.read_text(encoding="utf-8").splitlines()
        records: list[dict] = []
        last_index = len(lines) - 1
        for index, raw in enumerate(lines):
            line = raw.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
            except ValueError:  # also covers json.JSONDecodeError
                if index == last_index:
                    continue  # a crash mid-append; everything before it stands
                raise CliError(
                    code=EXIT_ENV_ERROR,
                    message=(
                        f"The calibration journal at {self.path} is corrupt: line "
                        f"{index + 1} is not valid JSON, and it is not the final line "
                        "(so it is damage, not an interrupted write)."
                    ),
                    remediation=(
                        "A servo may be holding a temporary encoder offset whose original "
                        f"is recorded ONLY in this file. Read {self.path} by hand, find the "
                        "'begin' record naming the joint's original_offset, and restore it "
                        "with 'arm101 arm rezero' before moving the arm."
                    ),
                ) from None
            if isinstance(record, dict):
                records.append(record)
        return records

    def entries(self) -> list[JournalEntry]:
        """Replay the log into one :class:`JournalEntry` per motor, in ``begin`` order."""
        entries: dict[int, JournalEntry] = {}
        for record in self.records():
            event = record.get("event")
            motor = record.get("motor")
            if not isinstance(motor, int):
                continue
            if event == EVENT_BEGIN:
                entries[motor] = JournalEntry(
                    joint=str(record.get("joint", "")),
                    motor=motor,
                    original_offset=int(record.get("original_offset", 0)),
                )
            elif event == EVENT_OFFSET and motor in entries:
                current = entries[motor]
                entries[motor] = JournalEntry(
                    joint=current.joint,
                    motor=motor,
                    original_offset=current.original_offset,
                    temporary_offsets=current.temporary_offsets + (int(record["offset"]),),
                    closed=current.closed,
                    disposition=current.disposition,
                )
            elif event == EVENT_END and motor in entries:
                current = entries[motor]
                entries[motor] = JournalEntry(
                    joint=current.joint,
                    motor=motor,
                    original_offset=current.original_offset,
                    temporary_offsets=current.temporary_offsets,
                    closed=True,
                    disposition=record.get("disposition"),
                )
        return list(entries.values())

    def dirty_entries(self) -> list[JournalEntry]:
        """Every entry with a ``begin`` and no ``end`` — the joints at risk."""
        return [entry for entry in self.entries() if not entry.closed]

    def dirty_entry_for(self, motor: int) -> "JournalEntry | None":
        """*motor*'s open transaction, or ``None``."""
        for entry in self.dirty_entries():
            if entry.motor == motor:
                return entry
        return None

    def is_dirty(self) -> bool:
        """``True`` if any joint may be holding an unrecorded calibration."""
        return bool(self.dirty_entries())


def _fsync_dir(directory: Path) -> None:
    """``fsync`` a directory so a newly created file's NAME is durable too.

    Best-effort: not every platform or filesystem permits opening a directory for
    ``fsync`` (Windows refuses outright). Where it fails, the file's *data* is
    still synced — we simply lose the guarantee on the directory entry, which is
    strictly better than refusing to journal at all.

    ``contextlib.suppress``, not ``try/except/pass``: bandit's B110 rejects the
    latter and CI enforces bandit.
    """
    with contextlib.suppress(OSError):
        fd = os.open(directory, os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)


# ---------------------------------------------------------------------------
# The transaction, against a real bus
# ---------------------------------------------------------------------------


def shift_offset(
    bus: "MotorBus",
    journal: CalibrationJournal,
    *,
    joint: str,
    motor: int,
    offset: int,
) -> None:
    """Write a TEMPORARY encoder *offset* to *motor* — journalled first, always.

    The only sanctioned way to move a servo's seam. In order:

    1. On the first shift of this transaction, read the ORIGINAL offset off the
       servo and ``begin`` the journal entry — durable on disk before step 3.
    2. Journal the temporary *offset* — also durable before step 3, so there can
       never be an offset in a servo that is not already named on disk.
    3. ``bus.write_offset`` — the EEPROM write.

    If step 3 fails (or the process is killed anywhere from step 1 onward), the
    journal already names the original, and :func:`restore_dirty` can put the arm
    back. That is the entire contract, and it only holds in this order.

    Deliberately does NOT clear a latched overload first. ``write_offset`` opens
    with ``enable_torque(motor, False)``, which a latched servo answers with the
    overload bit still set — so a shift against a latched joint WILL raise. That
    is correct here: a joint latched in overload has hit something, and silently
    de-energising it (which is what clearing the latch does) is a decision for the
    caller that knows why it is probing, not for the primitive. Compare
    :func:`restore_dirty`, which *does* clear the latch — because by then the
    joint is known-latched, and putting the calibration back matters more.

    Raises
    ------
    CliError
        Whatever the bus raises. The journal is durable by then.
    """
    if journal.dirty_entry_for(motor) is None:
        original = bus.read_offset(motor)
        journal.begin(joint=joint, motor=motor, original_offset=original)
    journal.record_offset(motor=motor, offset=offset)
    bus.write_offset(motor, offset)


@dataclass(frozen=True)
class RestoreOutcome:
    """What the restore actually managed to do for ONE motor. Asked, and achieved.

    Attributes
    ----------
    restored:
        ``True`` only when the write landed **and** the read-back agrees. A write
        the bus accepted but the servo is not holding is not a restore, and
        saying so would be the same class of lie this module exists to prevent
        (see :meth:`arm101.hardware.bus.MotorBus.write_offset` — an offset that
        reads back fine can still silently revert if the addr-55 Lock dance is
        skipped; PR #21).
    read_back:
        The offset the servo reported after the write, or ``None`` if it could
        not be read at all.
    error:
        Why it failed, for the human who now has to fix a joint by hand.
    """

    joint: str
    motor: int
    original_offset: int
    restored: bool
    read_back: "int | None" = None
    error: "str | None" = None

    def to_dict(self) -> dict:
        return {
            "joint": self.joint,
            "motor": self.motor,
            "original_offset": self.original_offset,
            "restored": self.restored,
            "read_back": self.read_back,
            "error": self.error,
        }


@dataclass(frozen=True)
class RestoreReport:
    """The result of a whole recovery sweep. ``complete`` is the only safe check."""

    outcomes: tuple[RestoreOutcome, ...] = ()
    errors: dict[int, str] = field(default_factory=dict)

    @property
    def restored(self) -> tuple[int, ...]:
        return tuple(o.motor for o in self.outcomes if o.restored)

    @property
    def failed(self) -> tuple[int, ...]:
        return tuple(o.motor for o in self.outcomes if not o.restored)

    @property
    def complete(self) -> bool:
        """``True`` when every dirty joint is back where it started (or none was)."""
        return not self.failed

    def to_dict(self) -> dict:
        return {
            "complete": self.complete,
            "outcomes": [o.to_dict() for o in self.outcomes],
            "restored": list(self.restored),
            "failed": list(self.failed),
            "errors": {str(m): text for m, text in self.errors.items()},
        }


def _restore_entry(
    bus: "MotorBus", journal: CalibrationJournal, entry: JournalEntry
) -> RestoreOutcome:
    """Put ONE joint back. Never raises for a bus failure — the next motor still gets its turn."""
    # A crashed probe is a prime way to leave a servo latched in overload, and
    # `write_offset`'s first act (`enable_torque(motor, False)`) is exactly the
    # packet a latched servo answers with the overload bit still set — so the
    # restore would raise before it ever opened the EEPROM. `clear_overload`
    # writes the same byte (addr 40) but tolerates that bit. Best-effort: if the
    # bus is too far gone to accept even this, the write below is still worth
    # trying, and its failure is what gets reported.
    #
    # contextlib.suppress, NOT try/except/pass — bandit B110 fails CI on the latter.
    with contextlib.suppress(Exception):
        bus.clear_overload(entry.motor)

    try:
        bus.write_offset(entry.motor, entry.original_offset)
        read_back = bus.read_offset(entry.motor)
    except Exception as exc:  # noqa: BLE001 - a bus failure on ONE motor must not strand the rest
        return RestoreOutcome(
            joint=entry.joint,
            motor=entry.motor,
            original_offset=entry.original_offset,
            restored=False,
            error=f"{type(exc).__name__}: {exc}",
        )

    if read_back != entry.original_offset:
        # The write was accepted and the servo is holding something else. The
        # entry stays DIRTY on purpose: its original_offset is still the only
        # record of the truth, and closing it here would destroy that record
        # while the joint is still mis-calibrated.
        return RestoreOutcome(
            joint=entry.joint,
            motor=entry.motor,
            original_offset=entry.original_offset,
            restored=False,
            read_back=read_back,
            error=(
                f"wrote offset {entry.original_offset} to motor {entry.motor} "
                f"(EEPROM addr 31) but it read back {read_back}"
            ),
        )

    journal.end(motor=entry.motor, disposition=DISPOSITION_RESTORED)
    return RestoreOutcome(
        joint=entry.joint,
        motor=entry.motor,
        original_offset=entry.original_offset,
        restored=True,
        read_back=read_back,
    )


def restore_dirty(bus: "MotorBus", journal: CalibrationJournal) -> RestoreReport:
    """Restore every joint the journal says may be holding a temporary offset.

    The recovery path — run it at startup, **before anything else**, every time.
    On a clean journal it touches the bus not at all, so it is free to call
    unconditionally.

    Per motor, independent, and the sweep always runs to the end: the bus that
    stranded one joint is the same bus the others need, so a failure on motor 2
    must never cost motors 3..6 their restore. Failures are captured into
    :class:`RestoreOutcome` rather than raised — and a motor whose restore could
    not be *verified* keeps its journal entry, so the next run tries again. The
    original offset is only ever forgotten once the servo is provably holding it.

    Does not raise for a bus failure. ``KeyboardInterrupt`` and ``SystemExit`` DO
    propagate: this runs at startup, not during unwinding, so an operator asking
    to abort gets to abort — and the journal is still on disk, still dirty, still
    correct, for the run that follows.
    """
    outcomes: list[RestoreOutcome] = []
    errors: dict[int, str] = {}
    for entry in sorted(journal.dirty_entries(), key=lambda e: e.motor):
        outcome = _restore_entry(bus, journal, entry)
        outcomes.append(outcome)
        if outcome.error is not None:
            errors[outcome.motor] = outcome.error
    return RestoreReport(outcomes=tuple(outcomes), errors=errors)


def require_clean(bus: "MotorBus", journal: "CalibrationJournal | None" = None) -> RestoreReport:
    """Recover a dirty calibration, and REFUSE to continue if the recovery failed.

    The guard a verb calls first. Either the arm is in a calibration somebody
    recorded, or this raises — there is no third outcome in which a probe is
    allowed to start layering fresh temporary offsets on top of an old one nobody
    can name.

    Raises
    ------
    CliError(EXIT_ENV_ERROR)
        If any joint could not be restored. The message carries the joint, the
        motor, and the original offset — the numbers a human needs to fix it by
        hand — because at that point the file is the only place they exist.
    """
    journal = journal if journal is not None else CalibrationJournal()
    report = restore_dirty(bus, journal)
    if report.complete:
        return report

    stranded = "; ".join(
        f"{o.joint} (motor {o.motor}) should hold offset {o.original_offset} — {o.error}"
        for o in report.outcomes
        if not o.restored
    )
    raise CliError(
        code=EXIT_ENV_ERROR,
        message=(
            "A previous run left a temporary encoder offset in EEPROM and it could NOT be "
            f"restored: {stranded}. Every tick this joint reports is in a frame nobody chose, "
            "so no measurement, map, or move may be trusted until it is put back."
        ),
        remediation=(
            f"The journal at {journal.path} still names the original offset — it is the only "
            "record of it, so do not delete it. Check the bus is healthy ('arm101 arm read'), "
            "power-cycle the servo if it is latched, and re-run: the restore is retried "
            "automatically at startup."
        ),
    )


def commit(journal: CalibrationJournal, *, motor: int) -> None:
    """Close *motor*'s transaction and KEEP the offset now in force.

    The deliberate other ending: the operator asked for this re-zero, the sweep
    proved the seam actually moved, and the new calibration is the truth from
    here. Writes nothing to the bus — the offset is already there. All this does
    is stop the next run from helpfully undoing it.

    Raises
    ------
    CliError(EXIT_USER_ERROR)
        If *motor* has no open transaction.
    """
    journal.end(motor=motor, disposition=DISPOSITION_COMMITTED)
