"""arm101.explore.log — append-only JSONL event log for reachability exploration.

The provenance + resume layer for ``arm101.explore``: every probed
:class:`~arm101.explore.types.ContactEvent` is appended as exactly one JSON
line, so a killed/interrupted exploration run keeps every line already
written to disk, and a resumed run can reconstruct which
:class:`~arm101.explore.types.JointConfig` values have already been probed
(:func:`visited_configs`) and skip re-probing them.

Zero third-party imports (stdlib only: ``json``, ``pathlib``) — matches the
zero-dependency contract of the rest of ``arm101.explore``.

Durability contract
--------------------
:func:`append_event` opens the log file in append mode, writes exactly one
line, flushes, and closes it again *within the call* — no file handle is
held open across calls. If the process is killed immediately after
:func:`append_event` returns, every previously appended line (including the
one just written) is durably on disk.

Corruption tolerance
---------------------
A crash *during* a write can leave a truncated or blank final line in the
file. :func:`read_events` tolerates exactly that: an unparseable *last*
line is skipped rather than raised. Corruption anywhere earlier in the file
is a genuine error and still raises, so silent data loss in the middle of a
log is never masked.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Iterable, List, Set, Union

from arm101.explore.types import ContactEvent, JointConfig

#: Accepted path argument type for every function in this module.
PathLike = Union[str, Path]

__all__ = ["append_event", "append_events", "read_events", "visited_configs"]


def append_event(path: PathLike, event: ContactEvent) -> None:
    """Append *event* to the JSONL log at *path* as exactly one JSON line.

    Creates *path*'s parent directory (and the file itself) if missing. The
    file is opened in append mode, written to, flushed, and closed within
    this call — no handle is held open across calls — so a process killed
    right after this returns keeps every line already written.

    Parameters
    ----------
    path : str | Path
        Location of the JSONL event log.
    event : ContactEvent
        The event to append.
    """
    log_path = Path(path)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("a", encoding="utf-8") as fh:
        fh.write(json.dumps(event.to_dict()) + "\n")
        fh.flush()


def append_events(path: PathLike, events: Iterable[ContactEvent]) -> None:
    """Append each event in *events* to the JSONL log at *path*, in order.

    Convenience wrapper over repeated :func:`append_event` calls — one JSON
    line per event, same durability contract as a standalone call.
    """
    for event in events:
        append_event(path, event)


def read_events(path: PathLike) -> List[ContactEvent]:
    """Parse the JSONL log at *path* back into a list of :class:`ContactEvent`.

    Returns an empty list if *path* does not exist. Blank lines are skipped
    everywhere. An unparseable *final* line — the signature of a crash
    mid-write — is skipped rather than raised; an unparseable line anywhere
    else in the file is a genuine error and propagates.
    """
    log_path = Path(path)
    if not log_path.exists():
        return []

    with log_path.open("r", encoding="utf-8") as fh:
        lines = fh.readlines()

    events: List[ContactEvent] = []
    last_index = len(lines) - 1
    for index, raw_line in enumerate(lines):
        line = raw_line.strip()
        if not line:
            continue
        try:
            data = json.loads(line)
            events.append(ContactEvent.from_dict(data))
        except (json.JSONDecodeError, KeyError, ValueError, TypeError):
            if index == last_index:
                # Tolerate a truncated/partial final line (crash mid-write).
                continue
            raise
    return events


def visited_configs(path: PathLike) -> Set[JointConfig]:
    """Return the set of :class:`JointConfig` values already recorded at *path*.

    Used to resume an interrupted exploration run: any config already in
    this set has already been probed and should be skipped. Returns an
    empty set if *path* does not exist.
    """
    return {event.config for event in read_events(path)}
