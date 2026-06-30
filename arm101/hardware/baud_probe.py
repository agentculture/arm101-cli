"""Multi-baud id/read probe + classifier for the Feetech bus. Closes #18.

Today's motor detection (``arm101 doctor``, ``setup-motors``, etc.) opens the
bus at a single hardwired baud rate — 1 Mbps, :data:`FeetechBus`'s default —
so a motor that has drifted off that baud (a half-finished ``set-baudrate``,
a hand-edited EEPROM, a leftover factory-default servo on an otherwise
re-bauded chain) is invisible to that scan and gets reported as "no servo"
even though it is alive and listening at, say, 500 kbps. That is a misleading
diagnosis: it reads the same as a genuinely dead/disconnected motor.

:func:`probe_bus` fixes this by sweeping **every** baud rate in
:data:`arm101.hardware.bus.BAUD_MAP`, scanning a set of candidate ids at each
one, and classifying each ``(baud, id)`` pair as :data:`SUCCESS`,
:data:`CORRUPT`, or :data:`TIMEOUT`. A fully silent bus — TIMEOUT at every
baud for every id — is then a positive *diagnosis* ("no servo answered at
any baud — likely power or data-line") rather than the old single-baud
"no servo" non-answer.

Classification rule
--------------------
``scan()`` is the presence test; ``read_info()`` is the coherence test:

* If ``scan()`` does not return the id (or raises) -> :data:`TIMEOUT` — the
  bus saw no electrical response at all for that id at that baud, the
  signature of "absent" or "wrong baud".
* If ``scan()`` *does* confirm the id -> the servo is physically present and
  responsive at that baud, so anything that goes wrong from here on is a
  data-quality problem, not absence:
  * ``read_info()`` succeeds and is coherent (``id`` echoes the queried id
    and ``model`` is a plausible STS register value) -> :data:`SUCCESS`.
  * ``read_info()`` succeeds but is incoherent (mismatched ``id`` — a
    duplicate-id collision or cross-talk — or an implausible ``model``) ->
    :data:`CORRUPT`.
  * ``read_info()`` itself raises (garbled packet) -> :data:`CORRUPT`.

A failure opening or scanning the bus for an entire baud (the factory raises,
``bus.open()`` raises, or ``bus.scan()`` raises) degrades every candidate id
at *that baud only* to :data:`TIMEOUT` (we have no positive evidence anything
answered) and the sweep continues with the remaining bauds — one bad baud
never aborts the whole probe.

This module is pure logic and hardware-free: it only ever touches a bus
through the ``bus_factory(port, baudrate) -> MotorBus`` seam, so it is fully
testable with :class:`arm101.hardware.bus.FakeBus` (see
``tests/test_baud_probe.py``). A later CLI-layer task renders
:class:`ProbeReport` for ``arm101`` users; this module does not touch the CLI.
"""

from __future__ import annotations

import contextlib
from dataclasses import dataclass, field
from typing import Callable, Iterable, Literal

from arm101.hardware.bus import BAUD_MAP, FeetechBus, MotorBus

# ---------------------------------------------------------------------------
# Classification
# ---------------------------------------------------------------------------

#: One of "SUCCESS" / "CORRUPT" / "TIMEOUT" — see the module docstring for
#: the exact rule that assigns each (baud, id) probe a classification.
Classification = Literal["SUCCESS", "CORRUPT", "TIMEOUT"]

SUCCESS: Classification = "SUCCESS"
CORRUPT: Classification = "CORRUPT"
TIMEOUT: Classification = "TIMEOUT"

# scan() proved the servo is present and responsive at this baud; only a
# coherence problem (not absence) follows from here, so failures past this
# point classify CORRUPT rather than TIMEOUT.


def _is_plausible_model(model: object) -> bool:
    """Return True when *model* looks like a real STS register value.

    The STS3215 model register (address 3, 2 bytes) is a positive, non-zero
    16-bit value (the FakeBus/known-hardware default is 777, matching the
    ``_STS3215_MODEL`` constant in ``calibrate_motor.py``). A garbled or
    all-zero read is not a real model number, so it is treated as
    incoherent rather than asserting an exact model match — the probe's job
    is to detect *garbling*, not to gate on a specific servo SKU.
    """
    return isinstance(model, int) and not isinstance(model, bool) and 0 < model <= 0xFFFF


# ---------------------------------------------------------------------------
# Result records
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ProbeRecord:
    """One ``(baud, motor_id)`` classification produced by :func:`probe_bus`."""

    baud: int
    motor_id: int
    classification: Classification
    detail: str = ""

    def to_dict(self) -> dict[str, object]:
        """JSON-serializable form."""
        return {
            "baud": self.baud,
            "motor_id": self.motor_id,
            "classification": self.classification,
            "detail": self.detail,
        }


@dataclass
class ProbeReport:
    """Aggregate result of sweeping every baud x candidate id for one port."""

    port: str
    records: list[ProbeRecord] = field(default_factory=list)

    @property
    def is_fully_silent(self) -> bool:
        """True when every record, across every baud and id, is TIMEOUT.

        This is the "diagnosed" silent-bus case from issue #18: rather than
        a single-baud probe's ambiguous "no servo", a fully silent multi-baud
        sweep is positive evidence of a power or data-line problem.
        """
        return bool(self.records) and all(r.classification == TIMEOUT for r in self.records)

    def answering_ids(self) -> list[int]:
        """Sorted ids that produced at least one non-TIMEOUT record at any baud."""
        return sorted({r.motor_id for r in self.records if r.classification != TIMEOUT})

    def bauds_for_id(
        self, motor_id: int, *, classification: Classification | None = None
    ) -> list[int]:
        """Sorted bauds at which *motor_id* answered.

        With no *classification* filter, "answered" means any non-TIMEOUT
        record (SUCCESS or CORRUPT — the servo is physically present even if
        its read was incoherent). Pass ``classification=SUCCESS`` (or
        ``CORRUPT``) to narrow to one outcome.
        """
        if classification is None:
            wanted = (SUCCESS, CORRUPT)
        else:
            wanted = (classification,)
        return sorted(
            r.baud for r in self.records if r.motor_id == motor_id and r.classification in wanted
        )

    def to_dict(self) -> dict[str, object]:
        """JSON-serializable form: port, convenience summaries, and every record."""
        return {
            "port": self.port,
            "fully_silent": self.is_fully_silent,
            "answering_ids": self.answering_ids(),
            "records": [r.to_dict() for r in self.records],
        }

    def summary(self) -> str:
        """Concise human-readable summary naming the port and, per id, which
        baud(s) answered — or a positive silent-bus diagnosis."""
        lines = [f"Probe of {self.port}:"]
        if self.is_fully_silent:
            lines.append(
                "  ALL TIMEOUT at every baud for every id — no servo answered at "
                "any baud (likely power or data-line, not a baud mismatch)."
            )
            return "\n".join(lines)

        ids = self.answering_ids()
        if not ids:
            lines.append("  No candidate ids were probed.")
            return "\n".join(lines)

        for motor_id in ids:
            success = self.bauds_for_id(motor_id, classification=SUCCESS)
            corrupt = self.bauds_for_id(motor_id, classification=CORRUPT)
            parts = []
            if success:
                parts.append(f"SUCCESS@{','.join(str(b) for b in success)}")
            if corrupt:
                parts.append(f"CORRUPT@{','.join(str(b) for b in corrupt)}")
            lines.append(f"  id {motor_id}: {'; '.join(parts)}")
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# Probe
# ---------------------------------------------------------------------------


def probe_bus(
    port: str,
    *,
    bus_factory: Callable[[str, int], MotorBus] = FeetechBus,
    ids: Iterable[int] = range(1, 13),
) -> ProbeReport:
    """Sweep every baud in :data:`BAUD_MAP`, scan *ids* at each, and classify.

    Parameters
    ----------
    port:
        Serial device path; carried into the report, not otherwise inspected.
    bus_factory:
        ``(port, baudrate) -> MotorBus`` seam. Defaults to the real
        :class:`~arm101.hardware.bus.FeetechBus`; tests inject a factory
        returning a configured :class:`~arm101.hardware.bus.FakeBus`.
    ids:
        Candidate motor ids to scan at each baud. Defaults to ``1..12`` (an
        SO-101 follower plus leader, mirroring :meth:`MotorBus.scan`'s
        default range).

    Returns
    -------
    ProbeReport
        Never raises: a bad bus/factory for one baud degrades that baud's
        records to TIMEOUT and the sweep continues (see module docstring).
    """
    candidate_ids = list(ids)
    records: list[ProbeRecord] = []
    for baud in sorted(BAUD_MAP):
        records.extend(_probe_one_baud(port, baud, bus_factory, candidate_ids))
    return ProbeReport(port=port, records=records)


def _timeout_records(baud: int, candidate_ids: list[int], detail: str) -> list[ProbeRecord]:
    """Build a TIMEOUT record for every candidate id at *baud* (degrade path)."""
    return [ProbeRecord(baud, motor_id, TIMEOUT, detail) for motor_id in candidate_ids]


def _probe_one_baud(
    port: str,
    baud: int,
    bus_factory: Callable[[str, int], MotorBus],
    candidate_ids: list[int],
) -> list[ProbeRecord]:
    """Open one bus at *baud*, scan + classify *candidate_ids*, always close."""
    try:
        bus = bus_factory(port, baud)
    except Exception as exc:  # noqa: BLE001 - one bad baud must not abort the sweep
        return _timeout_records(baud, candidate_ids, f"bus_factory() raised: {exc!r}")

    try:
        return _probe_with_open_bus(bus, baud, candidate_ids)
    finally:
        with contextlib.suppress(Exception):
            bus.close()


def _probe_with_open_bus(bus: MotorBus, baud: int, candidate_ids: list[int]) -> list[ProbeRecord]:
    """``open()`` *bus*, ``scan()`` the candidates, classify each present id."""
    try:
        bus.open()
    except Exception as exc:  # noqa: BLE001 - degrade this baud only, not the sweep
        return _timeout_records(baud, candidate_ids, f"open() raised: {exc!r}")

    try:
        present = set(bus.scan(list(candidate_ids)))
    except Exception as exc:  # noqa: BLE001 - no confirmed response -> TIMEOUT
        return _timeout_records(baud, candidate_ids, f"scan() raised: {exc!r}")

    return [_classify(bus, baud, motor_id, present) for motor_id in candidate_ids]


def _classify(bus: MotorBus, baud: int, motor_id: int, present_ids: set[int]) -> ProbeRecord:
    """Classify one id once its baud's bus is open and scanned."""
    if motor_id not in present_ids:
        return ProbeRecord(baud, motor_id, TIMEOUT, "not present in scan()")

    # scan() confirmed presence: anything past here is CORRUPT, not TIMEOUT.
    try:
        info = bus.read_info(motor_id)
    except Exception as exc:  # noqa: BLE001 - presence confirmed -> CORRUPT, not TIMEOUT
        return ProbeRecord(
            baud, motor_id, CORRUPT, f"read_info() raised after scan confirmed presence: {exc!r}"
        )

    reported_id = info.get("id")
    model = info.get("model")
    if reported_id == motor_id and _is_plausible_model(model):
        return ProbeRecord(baud, motor_id, SUCCESS, f"id+model coherent (model={model})")
    return ProbeRecord(
        baud,
        motor_id,
        CORRUPT,
        f"incoherent read: reported id={reported_id!r}, model={model!r}",
    )
