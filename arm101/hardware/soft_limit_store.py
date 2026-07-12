"""Where a MEASURED soft limit lands — and the thing the runtime reads it back out of.

The question this module answers
================================
A re-zero is an EEPROM write, so "commit it" is obvious: the value goes into the servo.
**A soft limit is software-only.** ``arm limits --commit`` can measure that a joint turns
all the way round — that no offset can ever evict its seam, and a software dead arc is
the only instrument left — and then it has nowhere to put the answer.

:data:`~arm101.hardware.arm_spec.SOFT_LIMITS` is a checked-in source table, and **a CLI
does not rewrite its own source.** So the measured limit goes here: an append-only JSONL
file at a default location, merged over the shipped table by
:func:`~arm101.hardware.arm_spec.resolve_soft_limits`, and read by every mover through
:func:`~arm101.hardware.arm_spec.resolve_bounds`.

The precedent, and why this one has a default path
--------------------------------------------------
Per-joint contact thresholds already ship exactly this way: ``arm_spec`` defaults plus an
optional ``--threshold-file`` override, resolved by ``arm_spec.resolve_contact_thresholds``.
This follows it — with **one deliberate difference**.

A contact threshold is a *tuning* input: forget to pass ``--threshold-file`` and you get
the default, which is a fine answer. A soft limit is a *safety constraint*: forget to pass
it and you drive the joint straight across the encoder seam the limit exists to fence off.
An override that only binds when someone remembers a flag is exactly as inert as a table
nobody consults — and **this repo has already shipped that bug once**: the ``wrist_roll``
soft limit was inert data for a whole release, because every mover sourced its bounds from
the servo's EEPROM ``min_angle``/``max_angle`` registers, which on this arm hold the
untouched factory ``0-4095``. It took a follow-up routing every mover through
``resolve_bounds`` to make the table mean anything.

So the store has a **default location** (:func:`default_soft_limit_path`) and is loaded
whether or not anybody asks for it. ``--soft-limit-file`` points somewhere else;
``--no-soft-limit-file`` is deliberately not a flag.

The format
----------
Append-only JSONL, one record per commit — the same idiom as
:mod:`arm101.hardware.journal` and :mod:`arm101.explore.log`, for the same reason: an
append is the smallest durable write there is, and it keeps the *history* of what was
measured rather than overwriting it. **The last record for a joint wins.**

::

    {"joint": "wrist_roll", "min_tick": 185, "max_tick": 3995, "offset": 85,
     "kind": "continuous", "swept_ticks": 4096, "dead_arc_ticks": 285,
     "reason": "...", "pose": null, "ts": "2026-07-12T..."}

Only ``joint``, ``min_tick`` and ``max_tick`` are load-bearing; they are the
:class:`~arm101.hardware.arm_spec.SoftLimit`, in **RAW** ticks, and reading them back
constructs one — so a hand-edited range that is not a valid limit is refused by the type
itself rather than fenced off silently. Everything else is provenance: the offset the
limit was derived against, what the arm measured, the pose it measured in, and when. A
soft limit is evidence about a *pose* as much as about a joint, and a record that could
not say which pose would be a number with no claim attached.

RAW ticks, and it matters
-------------------------
The stored pair is RAW — the magnet's own frame, a physical angle, unchanged by any
re-zero — because that is the only frame a soft limit can be *stored* in. The reported
frame moves whenever the offset register does, and a limit written in it is only true for
the offset it was measured through. ``SOFT_LIMITS`` shipped in the reported frame and had
to be corrected; ``REZERO_ARCS`` made the identical mistake. Twice is a pattern, so the
frame is named in the record and the crossing happens exactly once, in
:func:`~arm101.hardware.arm_spec.permitted_reported_range`.

Zero third-party imports. No bus.
"""

from __future__ import annotations

import contextlib
import json
import os
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Mapping, Optional, Union

from arm101.cli._errors import EXIT_ENV_ERROR, EXIT_USER_ERROR, CliError
from arm101.hardware.arm_spec import JOINTS, SoftLimit

__all__ = [
    "SOFT_LIMIT_ENV_VAR",
    "DEFAULT_SOFT_LIMIT_NAME",
    "MeasuredSoftLimit",
    "default_soft_limit_path",
    "load_soft_limits",
    "record_soft_limit",
]

PathLike = Union[str, Path]

#: Environment variable that relocates the store. Set it per-process — the tests run in
#: PARALLEL under xdist, and a single hard-coded global path would have them racing each
#: other's commits *and* would let a test scribble a fake soft limit onto the operator's
#: real arm. Same rule, same reason, as ``ARM101_CALIBRATION_JOURNAL``.
SOFT_LIMIT_ENV_VAR = "ARM101_SOFT_LIMITS"

#: Default store file, under the same ``~/.arm101/`` the journal, audit log and plan
#: files already live in.
DEFAULT_SOFT_LIMIT_NAME = "soft-limits.jsonl"


def default_soft_limit_path() -> Path:
    """Return the store path: ``$ARM101_SOFT_LIMITS``, else ``~/.arm101/soft-limits.jsonl``."""
    env = os.environ.get(SOFT_LIMIT_ENV_VAR, "").strip()
    if env:
        return Path(env)
    return Path.home() / ".arm101" / DEFAULT_SOFT_LIMIT_NAME


@dataclass(frozen=True)
class MeasuredSoftLimit:
    """One joint's soft limit, plus the evidence that earned it.

    Attributes
    ----------
    joint:
        The joint the limit restricts.
    limit:
        The :class:`~arm101.hardware.arm_spec.SoftLimit` itself, in **RAW** ticks. The
        only load-bearing part; everything else on this record exists so a human can
        decide whether to believe it.
    offset:
        The signed encoder offset the joint was holding when this was derived. The dead
        arc has to contain that servo's *reported* seam (at raw ``Ofs``), so a limit
        without the offset it was derived against is a range with no claim attached.
    kind, swept_ticks, reason:
        What ``arm limits`` measured — the
        :class:`~arm101.hardware.classify.TravelClassification` that concluded a soft
        limit is the instrument, in its own words.
    pose:
        The ``--pose`` label the measurement ran under, if any. A limit found with the
        other joints in one pose may be an *obstacle*, not the joint's own geometry.
    """

    joint: str
    limit: SoftLimit
    offset: int
    kind: str = ""
    swept_ticks: int = 0
    reason: str = ""
    pose: Optional[str] = None

    def to_record(self) -> dict:
        """The JSONL line. ``dead_arc_ticks`` is derived — the price, made legible."""
        return {
            "joint": self.joint,
            "min_tick": self.limit.min_tick,
            "max_tick": self.limit.max_tick,
            "frame": "raw",
            "offset": self.offset,
            "dead_arc_ticks": self.limit.dead_arc_ticks,
            "kind": self.kind,
            "swept_ticks": self.swept_ticks,
            "reason": self.reason,
            "pose": self.pose,
            "ts": datetime.now(timezone.utc).isoformat(),
        }

    def table_entry(self) -> str:
        """The ``arm_spec.SOFT_LIMITS`` line a human would check in, rendered verbatim.

        The store is what the **runtime** reads. This is what a **person** reads — and it
        is deliberately printed alongside every commit, because a measurement that stays
        in one operator's home directory has been made once and will be made again by
        the next person. Promoting it into the shipped table is how a measured fact stops
        being a local one, and this is the exact text to paste.
        """
        return (
            f'    "{self.joint}": SoftLimit(min_tick={self.limit.min_tick}, '
            f"max_tick={self.limit.max_tick}),  # measured: {self.kind}, "
            f"{self.swept_ticks} ticks swept, at Ofs = {self.offset}"
        )


def _parse_record(record: object, *, line_no: int, path: Path) -> Optional[MeasuredSoftLimit]:
    """Turn one JSONL record into a :class:`MeasuredSoftLimit`, or refuse it loudly.

    Refuses rather than skips. A soft limit is a *safety* constraint: a line this cannot
    read is a fence somebody meant to put up, and quietly dropping it would leave the
    mover free to drive across the seam while the file on disk says otherwise. The one
    thing that is *not* damage — a blank line — is handled by the caller.
    """
    if not isinstance(record, dict):
        raise CliError(
            code=EXIT_ENV_ERROR,
            message=f"{path}, line {line_no}: expected a JSON object, got {type(record).__name__}.",
            remediation="Each line must be an object: "
            '{"joint": "wrist_roll", "min_tick": 185, "max_tick": 3995}.',
        )

    joint = record.get("joint")
    if joint not in JOINTS:
        raise CliError(
            code=EXIT_USER_ERROR,
            message=f"{path}, line {line_no}: unknown joint {joint!r}.",
            remediation=f"Valid joints: {', '.join(JOINTS)}.",
        )

    try:
        limit = SoftLimit(min_tick=int(record["min_tick"]), max_tick=int(record["max_tick"]))
    except (KeyError, TypeError, ValueError) as exc:
        raise CliError(
            code=EXIT_USER_ERROR,
            message=f"{path}, line {line_no}: not a usable soft limit for {joint} — {exc}",
            remediation=(
                "min_tick and max_tick are RAW encoder ticks and must satisfy "
                "0 <= min_tick < max_tick <= 4095. They are NOT the ticks a servo reports: "
                "convert (raw = reported + offset, mod 4096) before storing them."
            ),
        ) from exc

    return MeasuredSoftLimit(
        joint=str(joint),
        limit=limit,
        offset=int(record.get("offset", 0)),
        kind=str(record.get("kind", "")),
        swept_ticks=int(record.get("swept_ticks", 0)),
        reason=str(record.get("reason", "")),
        pose=record.get("pose"),
    )


def load_soft_limits(path: "PathLike | None" = None) -> dict[str, SoftLimit]:
    """Read the store and return ``{joint: SoftLimit}``. An absent file is an empty dict.

    **The last record for a joint wins.** The file is append-only, so a joint re-measured
    today sits below the one measured last month, and the newer measurement is the one in
    force. The older lines are kept: a soft limit is a claim about an arm at a moment, and
    the history of what was claimed is worth more than the disk it costs.

    An absent file means "nothing has been measured" — not "something went wrong". That is
    the common case (it is the state of every fresh checkout), so this is free to call
    unconditionally, which is exactly what :func:`arm101.cli._commands.arm._soft_limits`
    does.

    Raises
    ------
    CliError(EXIT_ENV_ERROR)
        If a line is not valid JSON. Unlike the calibration journal — which tolerates a
        truncated FINAL line, because a crash mid-append writes one — nothing here is
        written mid-motion, so a damaged line is damage. Skipping it would silently drop
        a fence.
    CliError(EXIT_USER_ERROR)
        If a record names an unknown joint or a range that is not a valid RAW soft limit.
    """
    resolved = Path(path) if path is not None else default_soft_limit_path()
    if not resolved.exists():
        return {}

    limits: dict[str, SoftLimit] = {}
    for line_no, raw in enumerate(resolved.read_text(encoding="utf-8").splitlines(), start=1):
        line = raw.strip()
        if not line:
            continue
        try:
            record = json.loads(line)
        except ValueError as exc:
            raise CliError(
                code=EXIT_ENV_ERROR,
                message=f"{resolved}, line {line_no}: not valid JSON — {exc}",
                remediation=(
                    "This file fences a joint off from its own encoder seam. A line that "
                    "cannot be read is a fence that will not be put up, so it is refused "
                    "rather than skipped. Repair or delete the line, then re-run."
                ),
            ) from exc
        measured = _parse_record(record, line_no=line_no, path=resolved)
        if measured is not None:
            limits[measured.joint] = measured.limit  # last one wins
    return limits


def record_soft_limit(measured: MeasuredSoftLimit, path: "PathLike | None" = None) -> Path:
    """Append *measured* to the store, durably. Returns the path written to.

    ``write`` -> ``flush`` -> ``fsync``, and an ``fsync`` of the parent directory the
    first time the file is created — the same durability the calibration journal insists
    on, and for a milder version of the same reason. A soft limit that is still in the
    page cache when the machine loses power did not get measured: the operator would come
    back to an arm that still drives across its own seam, and no record that anybody had
    ever found out otherwise.
    """
    resolved = Path(path) if path is not None else default_soft_limit_path()
    parent = resolved.parent
    parent.mkdir(parents=True, exist_ok=True)
    is_new = not resolved.exists()

    with resolved.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(measured.to_record()) + "\n")
        handle.flush()
        os.fsync(handle.fileno())

    if is_new:
        _fsync_dir(parent)
    return resolved


def _fsync_dir(directory: Path) -> None:
    """``fsync`` a directory so a newly created file's NAME is durable too.

    Best-effort: not every platform permits opening a directory for ``fsync`` (Windows
    refuses outright). Where it fails, the file's *data* is still synced. Uses
    ``contextlib.suppress`` rather than ``try/except/pass`` — bandit's B110 fails CI on
    the latter. Same helper, same reasoning, as :mod:`arm101.hardware.journal`.
    """
    with contextlib.suppress(OSError):
        fd = os.open(directory, os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)


def summarise(limits: Mapping[str, SoftLimit]) -> str:
    """One line per joint: the permitted range, and the travel it costs. For an operator."""
    if not limits:
        return "(none)"
    return "\n".join(
        f"  {joint}: permitted raw [{limit.min_tick}, {limit.max_tick}] "
        f"— {limit.dead_arc_ticks} ticks fenced off"
        for joint, limit in sorted(limits.items())
    )
