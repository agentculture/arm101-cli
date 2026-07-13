"""Tests for FeetechBus read robustness: a dropped packet is a CliError, not
an IndexError, and reads retry while writes never do.

TDD: written before the corresponding bus.py changes; they must fail against
the pre-fix code and drive the implementation.

Why this module exists — a real traceback, seen live on hardware today
------------------------------------------------------------------------
``FeetechBus._read_register`` calls the vendor SDK directly::

    value, result, error = self._packet_handler.read2ByteTxRx(self._port_handler, motor, addr)

and the SDK does this internally (``scservo_sdk/protocol_packet_handler.py``)::

    data_read = SCS_MAKEWORD(data[0], data[1]) if (result == COMM_SUCCESS) else 0

On a SHORT/CORRUPT packet, ``data`` can have fewer than 2 elements while
``result`` is still ``COMM_SUCCESS``, so the SDK raises a bare
``IndexError: list index out of range`` from *inside* ``read2ByteTxRx`` —
before this bus's own ``result != 0 or error != 0`` check ever runs. This was
observed live, mid-session, on a bus that was otherwise perfectly healthy.
This repo's one hard rule (``arm101/cli/_errors.py``) is that no Python
traceback ever leaks to stderr; that must hold for exceptions the *vendor
SDK* raises internally too, not only ones this codebase raises itself.

A second, related finding: a read issued immediately after an EEPROM write
can return silently-WRONG data (a plausible-looking ``0`` in place of the
real position) rather than failing loudly — covered by
``test_feetech_eeprom_write_settle.py``... no such file; the settle tests
live at the bottom of this module instead, next to the retry tests they are
easiest to reason about alongside.

See ``arm101/hardware/bus.py``'s ``_sdk_read`` / ``_retry_read`` / `_set_lock`
docstrings and the ``_READ_RETRY_*`` / ``_EEPROM_SETTLE_SECONDS`` constants
for the implementation this file drives.
"""

from __future__ import annotations

import pytest

from arm101.cli._errors import CliError
from arm101.hardware.bus import (
    _EEPROM_SETTLE_SECONDS,
    _IDEMPOTENT_WRITE_ATTEMPTS,
    _OFFSET_WRITE_ATTEMPTS,
    _READ_RETRY_ATTEMPTS,
    ADDR_HOMING_OFFSET,
    ADDR_LOCK,
    FeetechBus,
    OverloadError,
)

# ---------------------------------------------------------------------------
# Fake packet-handler — sequenced reads + fail-by-address writes
# ---------------------------------------------------------------------------


class _SequencedPacket:
    """Packet-handler stub whose READS play back a scripted, PER-CALL sequence.

    Every other packet-handler fake in this suite (``_ScriptedPacket`` in
    ``test_bus_overload.py``, ``_RecordingPacket`` in ``test_bus.py`` and
    ``test_bus_offset.py``) returns the SAME ``(result, error)`` /
    ``(value, result, error)`` on every call. Retry behaviour is defined by
    what happens ACROSS successive calls to the same register — fail, fail,
    succeed — and a fixture that cannot vary its answer by call number cannot
    express that, which is the one genuinely new capability this fake adds.

    Each element of *read_outcomes* is either:

    * an ``Exception`` INSTANCE — raised, modelling a bare SDK exception
      exactly as observed live (``IndexError`` from a short/corrupt packet —
      ``scservo_sdk/protocol_packet_handler.py:326``).
    * a ``(value, result, error)`` tuple — returned normally, matching the
      shape every other read fake here already uses.

    Calls past the end of *read_outcomes* repeat the LAST entry, so a test
    only has to script as many steps as it cares about.

    *write_fail_addrs* mirrors ``_RecordingPacket.fail_addrs`` in
    ``test_bus_offset.py``: addresses in the set report a comms failure
    (``result=1``) on EVERY write; everything else succeeds. Every write call
    is recorded in ``.writes``, in order — used by the "a write is never
    retried" tests, which assert directly on how many times a given address
    was written.
    """

    def __init__(self, read_outcomes, write_fail_addrs=None):
        self._read_outcomes = list(read_outcomes)
        self._write_fail_addrs = set(write_fail_addrs or ())
        self.read_calls = 0
        self.writes: list[tuple[int, int, int]] = []
        # A servo REMEMBERS what it was told. `write_offset` verifies itself by
        # reading addr 31 back (hardware, 2026-07-13: a "failed" write had in fact
        # landed), so a fake whose reads ignore its writes models a servo that
        # forgets — and the verify could never agree with it. The scripted
        # result/error still governs; only the VALUE comes from what was stored.
        self._stored: dict[int, int] = {}

    def _next_read(self):
        idx = min(self.read_calls, len(self._read_outcomes) - 1)
        outcome = self._read_outcomes[idx]
        self.read_calls += 1
        if isinstance(outcome, Exception):
            raise outcome
        return outcome

    def read1ByteTxRx(self, port, motor, addr):  # noqa: N802 - SDK spelling
        return self._next_read()

    def read2ByteTxRx(self, port, motor, addr):  # noqa: N802 - SDK spelling
        value, result, error = self._next_read()
        if addr in self._stored:
            value = self._stored[addr]
        return value, result, error

    def _write_result(self, addr):
        return (1, 0) if addr in self._write_fail_addrs else (0, 0)

    def write1ByteTxRx(self, port, motor, addr, val):  # noqa: N802 - SDK spelling
        self.writes.append((motor, addr, val))
        return self._write_result(addr)

    def write2ByteTxRx(self, port, motor, addr, val):  # noqa: N802 - SDK spelling
        self.writes.append((motor, addr, val))
        result, error = self._write_result(addr)
        if result == 0 and error == 0:
            self._stored[addr] = val
        return result, error


def _open_feetech(packet: "_SequencedPacket") -> FeetechBus:
    """A FeetechBus wired to *packet*, marked open, with no serial port involved."""
    bus = FeetechBus(port="/dev/ttyUSB_fake")
    bus._packet_handler = packet
    bus._port_handler = object()
    bus._open = True
    return bus


# ---------------------------------------------------------------------------
# 1. A short/corrupt packet becomes a CliError, never a raw IndexError
# ---------------------------------------------------------------------------


def test_feetech_read_position_indexerror_becomes_cli_error_not_a_raw_traceback():
    """The exact failure observed live: read2ByteTxRx raises bare IndexError.

    ``pytest.raises(CliError)`` only passes if a ``CliError`` — and nothing
    else — comes out of ``read_position``; an uncaught ``IndexError`` would
    fail this test with the raw traceback, which is exactly the bug.
    """
    packet = _SequencedPacket(read_outcomes=[IndexError("list index out of range")])
    bus = _open_feetech(packet)

    with pytest.raises(CliError) as exc:
        bus.read_position(motor=6)

    assert exc.value.code == 2  # EXIT_ENV_ERROR
    assert "IndexError" in exc.value.message
    assert "retry" in exc.value.remediation.lower() or "retry" in exc.value.message.lower()
    # Bounded: exhausted every retry attempt, not one and not unboundedly many.
    assert packet.read_calls == _READ_RETRY_ATTEMPTS


def test_feetech_read_lock_indexerror_becomes_cli_error_not_a_raw_traceback():
    """The 1-byte read path (read_lock -> read1ByteTxRx) has the identical shape."""
    packet = _SequencedPacket(read_outcomes=[IndexError("list index out of range")])
    bus = _open_feetech(packet)

    with pytest.raises(CliError) as exc:
        bus.read_lock(motor=6)

    assert exc.value.code == 2  # EXIT_ENV_ERROR
    assert "IndexError" in exc.value.message


def test_feetech_read_register_indexerror_becomes_cli_error_not_a_raw_traceback():
    """`_read_register` (read_info / read_offset / read_torque_limit's shared path)."""
    packet = _SequencedPacket(read_outcomes=[IndexError("list index out of range")])
    bus = _open_feetech(packet)

    with pytest.raises(CliError) as exc:
        bus.read_torque_limit(motor=4)

    assert exc.value.code == 2  # EXIT_ENV_ERROR


# ---------------------------------------------------------------------------
# 2. Retry: a read that eventually succeeds returns the value
# ---------------------------------------------------------------------------


def test_feetech_read_position_retries_and_returns_value_on_third_attempt():
    """Fails twice (one SDK exception, one comms failure), succeeds on the third."""
    packet = _SequencedPacket(
        read_outcomes=[
            IndexError("list index out of range"),  # attempt 1: bare SDK exception
            (0, 1, 0),  # attempt 2: comms failure (nonzero result)
            (1234, 0, 0),  # attempt 3: success
        ]
    )
    bus = _open_feetech(packet)

    assert bus.read_position(motor=6) == 1234
    assert packet.read_calls == 3


def test_feetech_read_register_retries_and_returns_value_on_third_attempt():
    """Same retry-then-succeed shape via `_read_register` (read_torque_limit)."""
    packet = _SequencedPacket(
        read_outcomes=[
            IndexError("list index out of range"),
            (0, 1, 0),
            (750, 0, 0),
        ]
    )
    bus = _open_feetech(packet)

    assert bus.read_torque_limit(motor=4) == 750
    assert packet.read_calls == 3


# ---------------------------------------------------------------------------
# 3. Retry is bounded: every attempt failing raises CliError
# ---------------------------------------------------------------------------


def test_feetech_read_position_all_attempts_fail_raises_cli_error():
    packet = _SequencedPacket(read_outcomes=[(0, 1, 0)])  # comms failure, every call
    bus = _open_feetech(packet)

    with pytest.raises(CliError) as exc:
        bus.read_position(motor=6)

    assert exc.value.code == 2  # EXIT_ENV_ERROR
    assert packet.read_calls == _READ_RETRY_ATTEMPTS  # bounded — not infinite


def test_feetech_read_lock_all_attempts_fail_raises_cli_error():
    packet = _SequencedPacket(read_outcomes=[IndexError("boom")])
    bus = _open_feetech(packet)

    with pytest.raises(CliError):
        bus.read_lock(motor=6)

    assert packet.read_calls == _READ_RETRY_ATTEMPTS


# ---------------------------------------------------------------------------
# 4. A WRITE is never retried — pinned hard, by call count
# ---------------------------------------------------------------------------


def test_feetech_write_goal_position_IS_retried__it_is_idempotent():
    """A goal write IS retried now — because repeating it changes nothing.

    THE PREMISE OF THIS TEST'S PREDECESSOR WAS WRONG. It asserted "exactly one
    attempt — no retry", inheriting the blanket rule that a write is never safe to
    repeat. That rule is right about id and baud (issue #38: the second packet is
    addressed to a motor that has already moved) and right about EEPROM. It was
    never right about a goal.

    Writing goal=2048 twice leaves the servo wanting 2048, exactly as once did. The
    dividing line is IDEMPOTENCE, not RAM-vs-EEPROM.

    And it matters: on 2026-07-13 a dropped ACK on this very write (``result=-7``)
    aborted a live probe on the arm. A follow-up sweep found the same write
    succeeding at every delay from 0 to 0.6 s — a dropped packet, not a busy servo.
    Reads have been retried all along, which is exactly why the loss rate went
    unnoticed for so long. An unretried write on a lossy bus is not a safety
    property; it is an outage.
    """
    packet = _SequencedPacket(read_outcomes=[(0, 0, 0)], write_fail_addrs={42})
    bus = _open_feetech(packet)

    with pytest.raises(CliError):
        bus.write_goal_position(motor=3, position=2048)

    goal_writes = [w for w in packet.writes if w[1] == 42]
    assert len(goal_writes) == _IDEMPOTENT_WRITE_ATTEMPTS  # bounded — not one, not forever
    assert {w[2] for w in goal_writes} == {2048}  # and every repeat says the SAME thing


def test_feetech_write_offset_is_never_retried_BLIND__only_ever_VERIFIED():
    """The addr-31 write is the ONE write that retries — because it can VERIFY.

    THE PREMISE OF THIS TEST'S PREDECESSOR WAS DISPROVEN ON HARDWARE (2026-07-13).
    It asserted the offset write is "never retried, specifically", on the rule that
    a failed write may in fact have landed and re-applying it is unsafe. The first
    half of that is true — and truer than we knew: an offset write returns
    ``result=-6`` (RX timeout) with real frequency and the servo had applied it
    EVERY observed time. Only the ACK went missing.

    But the conclusion does not follow. "May have landed" is a statement of
    IGNORANCE, and ignorance is curable: read addr 31 back. A read is idempotent
    and already retried, so it costs nothing to ask, and it converts an unsafe
    BLIND retry into a safe VERIFIED one — we repeat only what the register has
    just told us is absent.

    The other EEPROM writes (id at addr 5, baud at addr 6) still retry NOTHING and
    the tests below still pin that. The asymmetry is principled, not an oversight:
    an id write MOVES the address you would have to read back from, so it cannot
    verify itself. The offset write can.
    """
    packet = _SequencedPacket(read_outcomes=[(0, 0, 0)], write_fail_addrs={ADDR_HOMING_OFFSET})
    bus = _open_feetech(packet)

    with pytest.raises(CliError):
        bus.write_offset(motor=3, offset=100)

    # The write failed AND did not land (the read-back keeps saying 0), so it was
    # retried — bounded, and each attempt verified.
    offset_writes = [w for w in packet.writes if w[1] == ADDR_HOMING_OFFSET]
    assert len(offset_writes) == _OFFSET_WRITE_ATTEMPTS

    # THE INVARIANT THAT SURVIVES: it never gave up quietly, and never hammered.
    assert _OFFSET_WRITE_ATTEMPTS == _READ_RETRY_ATTEMPTS  # one policy, one number


def test_feetech_write_id_baudrate_baud_write_is_never_retried_on_failure():
    """The addr-6 baud write inside write_id_baudrate is never retried either."""
    packet = _SequencedPacket(read_outcomes=[(0, 0, 0)], write_fail_addrs={6})

    bus = _open_feetech(packet)

    with pytest.raises(CliError):
        bus.write_id_baudrate(motor=1, new_id=2, baudrate=1_000_000)

    baud_writes = [w for w in packet.writes if w[1] == 6]
    assert len(baud_writes) == 1  # exactly one attempt — no retry


# ---------------------------------------------------------------------------
# 5. OverloadError is never retried, and never converted
# ---------------------------------------------------------------------------


def test_feetech_read_position_overload_propagates_immediately_not_retried():
    """A latched overload (status error bit 5) is a REAL fault, not a dropped packet.

    Callers (``gentle_move``, ``arm rezero``) rely on catching it immediately
    to run their own recovery, so it must reach them on the FIRST attempt —
    silently eating one or two attempts on it here would only delay that
    recovery, and the latch will not clear itself between retries anyway.
    """
    packet = _SequencedPacket(read_outcomes=[(0, 0, 32)])  # overload bit set
    bus = _open_feetech(packet)

    with pytest.raises(OverloadError) as exc:
        bus.read_position(motor=6)

    assert exc.value.motor == 6
    assert exc.value.error_byte == 32
    assert packet.read_calls == 1  # NOT retried


def test_feetech_read_register_overload_propagates_immediately_not_retried():
    packet = _SequencedPacket(read_outcomes=[(0, 0, 32)])
    bus = _open_feetech(packet)

    with pytest.raises(OverloadError):
        bus.read_torque_limit(motor=4)

    assert packet.read_calls == 1


# ---------------------------------------------------------------------------
# 6. EEPROM write settle — a brief pause after the closing re-lock
# ---------------------------------------------------------------------------


def test_feetech_set_lock_relock_settles_after_a_successful_write(monkeypatch):
    """Closing the Lock (locked=True) sleeps _EEPROM_SETTLE_SECONDS before returning."""
    import arm101.hardware.bus as bus_module

    sleeps: list[float] = []
    monkeypatch.setattr(bus_module.time, "sleep", lambda seconds: sleeps.append(seconds))

    packet = _SequencedPacket(read_outcomes=[(0, 0, 0)])
    bus = _open_feetech(packet)

    bus._set_lock(motor=3, locked=True)

    assert sleeps == [_EEPROM_SETTLE_SECONDS]


def test_feetech_set_lock_unlock_does_not_settle(monkeypatch):
    """Opening the Lock (locked=False) is followed by MORE EEPROM traffic, not a
    caller's read — it must not pay the settle cost."""
    import arm101.hardware.bus as bus_module

    sleeps: list[float] = []
    monkeypatch.setattr(bus_module.time, "sleep", lambda seconds: sleeps.append(seconds))

    packet = _SequencedPacket(read_outcomes=[(0, 0, 0)])
    bus = _open_feetech(packet)

    bus._set_lock(motor=3, locked=False)

    assert sleeps == []


def test_feetech_write_offset_settles_exactly_once_after_the_relock(monkeypatch):
    """End to end: write_offset's closing re-lock settles; nothing else does."""
    import arm101.hardware.bus as bus_module

    sleeps: list[float] = []
    monkeypatch.setattr(bus_module.time, "sleep", lambda seconds: sleeps.append(seconds))

    packet = _SequencedPacket(read_outcomes=[(0, 0, 0)])
    bus = _open_feetech(packet)

    bus.write_offset(motor=3, offset=100)

    assert sleeps == [_EEPROM_SETTLE_SECONDS]


def test_feetech_failed_offset_write_still_settles_on_the_best_effort_relock(monkeypatch):
    """Even on a FAILED EEPROM write, the best-effort re-lock still settles —
    the servo may still be recovering from whatever landed before the failure."""
    import arm101.hardware.bus as bus_module

    sleeps: list[float] = []
    monkeypatch.setattr(bus_module.time, "sleep", lambda seconds: sleeps.append(seconds))

    packet = _SequencedPacket(read_outcomes=[(0, 0, 0)], write_fail_addrs={ADDR_HOMING_OFFSET})
    bus = _open_feetech(packet)

    with pytest.raises(CliError):
        bus.write_offset(motor=3, offset=100)

    # One settle per attempt: the write fails AND does not land, so the read-back
    # never agrees and the VERIFIED retry runs its course. The invariant is that
    # the best-effort relock settles every time — never that it happens once.
    assert sleeps == [_EEPROM_SETTLE_SECONDS] * _OFFSET_WRITE_ATTEMPTS


def test_feetech_write_goal_position_does_not_settle(monkeypatch):
    """A RAM write with no Lock dance (write_goal_position) never pays the EEPROM settle."""
    import arm101.hardware.bus as bus_module

    sleeps: list[float] = []
    monkeypatch.setattr(bus_module.time, "sleep", lambda seconds: sleeps.append(seconds))

    packet = _SequencedPacket(read_outcomes=[(0, 0, 0)])
    bus = _open_feetech(packet)

    bus.write_goal_position(motor=3, position=100)

    assert sleeps == []


def test_feetech_clear_overload_does_not_settle(monkeypatch):
    """clear_overload (addr 40, RAM) never touches the Lock register at all."""
    import arm101.hardware.bus as bus_module

    sleeps: list[float] = []
    monkeypatch.setattr(bus_module.time, "sleep", lambda seconds: sleeps.append(seconds))

    packet = _SequencedPacket(read_outcomes=[(0, 0, 0)])
    bus = _open_feetech(packet)

    bus.clear_overload(motor=3)

    assert sleeps == []
    assert ADDR_LOCK not in {addr for _motor, addr, _val in packet.writes}
