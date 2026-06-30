"""Tests for arm101.hardware.baud_probe — multi-baud id/read probe + classifier.

TDD: written before baud_probe.py existed and drive the implementation.
Closes #18 (re-find a motor moved off 1 Mbps): today's single-baud scan can
only say "no servo"; these tests assert the probe instead sweeps every baud
in ``BAUD_MAP`` and tells SUCCESS apart from CORRUPT apart from TIMEOUT, and
that a fully silent bus is *diagnosed* rather than reported as "no servo".

All bus interaction is via a ``bus_factory(port, baud) -> bus`` seam — tests
inject FakeBus (or tiny hand-rolled duck-typed stand-ins that raise on
purpose) instead of touching real hardware.
"""

from __future__ import annotations

import inspect
import json

from arm101.hardware.baud_probe import CORRUPT, SUCCESS, TIMEOUT, probe_bus
from arm101.hardware.bus import BAUD_MAP, FakeBus, FeetechBus

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _factory_from_map(per_baud):
    """Build a bus_factory closure backed by a {baud: bus} dict."""

    def factory(port, baud):  # noqa: ARG001 - port unused by the fake lookup
        return per_baud[baud]

    return factory


# ---------------------------------------------------------------------------
# 1. SUCCESS at one baud, TIMEOUT elsewhere
# ---------------------------------------------------------------------------


def test_success_at_one_baud_other_bauds_timeout():
    """A motor that only answers at 500000 reports SUCCESS there and TIMEOUT
    at every other swept baud; the report names the port."""
    per_baud = {baud: FakeBus(ids=[]) for baud in BAUD_MAP}
    per_baud[500_000] = FakeBus(ids=[7])  # default read_info echoes id=7, model=777

    report = probe_bus("/dev/ttyFAKE0", bus_factory=_factory_from_map(per_baud), ids=[7])

    assert report.port == "/dev/ttyFAKE0"
    success_records = [r for r in report.records if r.classification == SUCCESS]
    assert len(success_records) == 1
    assert success_records[0].baud == 500_000
    assert success_records[0].motor_id == 7

    other = [r for r in report.records if r.baud != 500_000]
    assert other  # sanity: there are other bauds
    assert all(r.classification == TIMEOUT for r in other)

    assert report.bauds_for_id(7, classification=SUCCESS) == [500_000]
    assert report.is_fully_silent is False


# ---------------------------------------------------------------------------
# 2. CORRUPT — duplicate/mismatched id, distinct from TIMEOUT
# ---------------------------------------------------------------------------


def test_corrupt_duplicate_id_classified_distinct_from_timeout():
    """A servo that answers scan() but whose read_info() reports a different
    id (collision / cross-talk) is CORRUPT, never TIMEOUT."""
    per_baud = {baud: FakeBus(ids=[3], info={3: {"id": 9}}) for baud in BAUD_MAP}

    report = probe_bus("/dev/ttyFAKE1", bus_factory=_factory_from_map(per_baud), ids=[3])

    assert report.records
    assert all(r.classification == CORRUPT for r in report.records)
    assert report.is_fully_silent is False
    assert report.answering_ids() == [3]


def test_corrupt_when_id_matches_but_model_implausible():
    """Even when the reported id is correct, an implausible model value
    (garbled register read) is still CORRUPT, not SUCCESS."""
    per_baud = {baud: FakeBus(ids=[6], info={6: {"model": 0}}) for baud in BAUD_MAP}

    report = probe_bus("/dev/ttyFAKE2", bus_factory=_factory_from_map(per_baud), ids=[6])

    assert all(r.classification == CORRUPT for r in report.records)


def test_read_info_raising_after_presence_confirmed_is_corrupt_not_timeout():
    """scan() confirms the id is physically present; read_info() then raising
    is classified CORRUPT (responded-but-bad), not TIMEOUT (absent)."""

    class _BoomOnRead:
        def open(self):
            pass

        def close(self):
            pass

        def scan(self, ids=None):
            return [4]

        def read_info(self, motor):
            raise RuntimeError("garbled packet")

    report = probe_bus("/dev/ttyFAKE3", bus_factory=lambda port, baud: _BoomOnRead(), ids=[4])

    assert report.records
    assert all(r.classification == CORRUPT for r in report.records)


# ---------------------------------------------------------------------------
# 3. Fully silent bus -> ALL TIMEOUT, diagnosed (not "no servo")
# ---------------------------------------------------------------------------


def test_fully_silent_bus_is_diagnosed_as_all_timeout():
    per_baud = {baud: FakeBus(ids=[]) for baud in BAUD_MAP}

    report = probe_bus("/dev/ttyFAKE4", bus_factory=_factory_from_map(per_baud), ids=[1, 2, 3])

    assert report.records
    assert all(r.classification == TIMEOUT for r in report.records)
    assert report.is_fully_silent is True
    assert report.answering_ids() == []
    # every swept baud x every candidate id is represented
    assert {r.baud for r in report.records} == set(BAUD_MAP)
    assert {r.motor_id for r in report.records} == {1, 2, 3}


def test_summary_names_port_and_diagnoses_silence():
    per_baud = {baud: FakeBus(ids=[]) for baud in BAUD_MAP}

    report = probe_bus("/dev/ttyFAKE5", bus_factory=_factory_from_map(per_baud), ids=[1])
    text = report.summary()

    assert "/dev/ttyFAKE5" in text
    assert "no servo answered at any baud" in text.lower()


# ---------------------------------------------------------------------------
# 4. The probe never raises even when a factory/bus blows up for one baud
# ---------------------------------------------------------------------------


def test_probe_never_raises_when_factory_raises_for_one_baud():
    good_bus_by_baud = {baud: FakeBus(ids=[5]) for baud in BAUD_MAP if baud != 250_000}

    def factory(port, baud):
        if baud == 250_000:
            raise RuntimeError("simulated serial-port-busy")
        return good_bus_by_baud[baud]

    report = probe_bus("/dev/ttyFAKE6", bus_factory=factory, ids=[5])

    bad = [r for r in report.records if r.baud == 250_000]
    assert bad
    assert all(r.classification == TIMEOUT for r in bad)

    good = [r for r in report.records if r.baud != 250_000]
    assert any(r.classification == SUCCESS for r in good)


def test_open_raising_degrades_that_baud_to_timeout():
    class _BoomOnOpen:
        def open(self):
            raise RuntimeError("port busy")

        def close(self):
            pass

        def scan(self, ids=None):
            return []

        def read_info(self, motor):
            return {}

    def factory(port, baud):
        if baud == 1_000_000:
            return _BoomOnOpen()
        return FakeBus(ids=[])

    report = probe_bus("/dev/ttyFAKE7", bus_factory=factory, ids=[1])

    bad = [r for r in report.records if r.baud == 1_000_000]
    assert bad
    assert all(r.classification == TIMEOUT for r in bad)


def test_scan_raising_degrades_to_timeout_without_aborting_other_bauds():
    class _BoomOnScan:
        def open(self):
            pass

        def close(self):
            pass

        def scan(self, ids=None):
            raise RuntimeError("comm error")

        def read_info(self, motor):
            return {"id": motor, "model": 777}

    def factory(port, baud):
        if baud == 38_400:
            return _BoomOnScan()
        return FakeBus(ids=[2])

    report = probe_bus("/dev/ttyFAKE8", bus_factory=factory, ids=[2])

    bad = [r for r in report.records if r.baud == 38_400]
    assert bad
    assert all(r.classification == TIMEOUT for r in bad)

    good = [r for r in report.records if r.baud != 38_400]
    assert any(r.classification == SUCCESS for r in good)


def test_bus_close_called_even_when_read_info_raises():
    closed = []

    class _Bus:
        def open(self):
            pass

        def close(self):
            closed.append(True)

        def scan(self, ids=None):
            return [1]

        def read_info(self, motor):
            raise RuntimeError("boom")

    probe_bus("/dev/ttyFAKE9", bus_factory=lambda port, baud: _Bus(), ids=[1])

    assert len(closed) == len(BAUD_MAP)


def test_bus_close_called_even_when_factory_succeeds_but_open_raises():
    closed = []

    class _BoomOnOpen:
        def open(self):
            raise RuntimeError("port busy")

        def close(self):
            closed.append(True)

        def scan(self, ids=None):
            return []

        def read_info(self, motor):
            return {}

    probe_bus("/dev/ttyFAKEA", bus_factory=lambda port, baud: _BoomOnOpen(), ids=[1])

    assert len(closed) == len(BAUD_MAP)


# ---------------------------------------------------------------------------
# 5. Sweep coverage + signature/defaults
# ---------------------------------------------------------------------------


def test_every_baud_in_baud_map_is_swept():
    per_baud = {baud: FakeBus(ids=[]) for baud in BAUD_MAP}

    report = probe_bus("/dev/ttyFAKEB", bus_factory=_factory_from_map(per_baud), ids=[1])

    assert {r.baud for r in report.records} == set(BAUD_MAP)


def test_default_bus_factory_is_feetech_bus():
    sig = inspect.signature(probe_bus)
    assert sig.parameters["bus_factory"].default is FeetechBus


def test_default_ids_is_range_1_to_12():
    sig = inspect.signature(probe_bus)
    assert list(sig.parameters["ids"].default) == list(range(1, 13))


# ---------------------------------------------------------------------------
# 6. Output forms — human summary + JSON
# ---------------------------------------------------------------------------


def test_report_to_dict_is_json_serializable():
    per_baud = {baud: FakeBus(ids=[1]) for baud in BAUD_MAP}

    report = probe_bus("/dev/ttyFAKEC", bus_factory=_factory_from_map(per_baud), ids=[1])
    payload = report.to_dict()
    text = json.dumps(payload)  # must not raise

    decoded = json.loads(text)
    assert decoded["port"] == "/dev/ttyFAKEC"
    assert decoded["fully_silent"] is False
    assert decoded["answering_ids"] == [1]
    assert len(decoded["records"]) == len(report.records)


def test_summary_lists_answering_ids_with_their_bauds():
    per_baud = {baud: FakeBus(ids=[]) for baud in BAUD_MAP}
    per_baud[115_200] = FakeBus(ids=[2])

    report = probe_bus("/dev/ttyFAKED", bus_factory=_factory_from_map(per_baud), ids=[2])
    text = report.summary()

    assert "2" in text
    assert "115200" in text.replace(",", "")
