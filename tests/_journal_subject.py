"""A real, killable process that shifts a real (file-backed) encoder offset.

Driven by ``tests/test_calibration_journal_sigkill.py``. It is a *script*, not a
test module — pytest will not collect it (leading underscore), and it exists
because the acceptance criterion it serves cannot be met inside the test process:

    SIGKILL the process mid-probe with a temporary offset in force; the NEXT
    invocation detects the dirty calibration, names the original offset, and
    restores it before doing anything else.

SIGKILL cannot be caught, blocked, or handled. No ``finally`` runs, no
``atexit`` hook fires, no context manager exits. Reasoning about the happy path
proves nothing about it — the only honest test is to actually kill a process that
has actually written a temporary offset, and then watch a *fresh* process clean
up after it. Hence two verbs:

``probe``
    Journal the original offset, write a temporary one to the (persistent)
    EEPROM, touch a ready-file so the parent knows the arm is now dirty, and
    then block forever waiting to be killed.

``recover``
    What the NEXT invocation does: open the same journal and the same EEPROM,
    restore whatever the dead process left behind, and print the report as JSON.

The "EEPROM" is a JSON file (:class:`tests._fakes.EepromFileBus`) precisely
because a servo's addr-31 register outlives the process that wrote it. An
in-memory fake would forget the temporary offset at the moment of death and the
test would pass without ever exercising a recovery.
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

# Make the repo root importable when run as a bare script from a subprocess.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from arm101.hardware.journal import (  # noqa: E402
    CalibrationJournal,
    restore_dirty,
    shift_offset,
)
from tests._fakes import EepromFileBus  # noqa: E402

#: How long ``probe`` waits to be killed before giving up. Generously longer than
#: any sane parent takes to notice the ready-file, but finite: a bug in the test
#: must not leave a process spinning on somebody's machine forever.
KILL_TIMEOUT_SECONDS = 60.0


def _bus(eeprom: str) -> EepromFileBus:
    bus = EepromFileBus(eeprom)
    bus.open()
    return bus


def probe(eeprom: str, journal_path: str, ready: str, joint: str, motor: int, offset: int) -> int:
    """Shift *motor* to a temporary *offset*, announce it, and wait to be killed."""
    bus = _bus(eeprom)
    shift_offset(
        bus,
        CalibrationJournal(journal_path),
        joint=joint,
        motor=motor,
        offset=offset,
    )
    # Only NOW is the arm genuinely dirty: the journal names the original and the
    # EEPROM holds the temporary. The parent may kill us from this instant on.
    Path(ready).write_text(json.dumps({"offset": bus.read_offset(motor)}), encoding="utf-8")
    time.sleep(KILL_TIMEOUT_SECONDS)
    return 0  # pragma: no cover - only reached if nobody kills us


def recover(eeprom: str, journal_path: str) -> int:
    """The NEXT invocation: restore whatever the killed process left in force."""
    bus = _bus(eeprom)
    report = restore_dirty(bus, CalibrationJournal(journal_path))
    print(json.dumps(report.to_dict()))
    return 0 if report.complete else 1


def main(argv: list[str]) -> int:
    verb = argv[0]
    if verb == "probe":
        eeprom, journal_path, ready, joint, motor, offset = argv[1:]
        return probe(eeprom, journal_path, ready, joint, int(motor), int(offset))
    if verb == "recover":
        eeprom, journal_path = argv[1:]
        return recover(eeprom, journal_path)
    raise SystemExit(f"unknown verb: {verb!r}")  # pragma: no cover


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
