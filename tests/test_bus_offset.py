"""Tests for the STS3215 encoder-offset primitives (``Ofs`` / ``Homing_Offset``, addr 31).

TDD: written before ``read_offset`` / ``write_offset`` / ``encode_offset`` /
``decode_offset`` existed, and they drive the implementation.

Why this register matters (issue #35): ``elbow_flex``'s encoder WRAPS inside its
physical travel — driven far enough it crosses the raw 4095->0 seam and reads
back near zero — so its encoder value is not monotonic with joint angle and
every position comparison in the codebase (arrival checks, ``clamp_goal``,
``ReachMap`` ranges) is wrong for it. Writing addr 31 shifts the joint's zero so
the seam lands in the arc the joint CANNOT reach, which makes the linear-axis
assumption true rather than merely assumed.

Three hardware failures are pinned here, each with its own test:

1. **Sign-magnitude, not two's complement.** The offset's sign lives in BIT 11
   (``(sign << 11) | magnitude``), so ``-1073`` goes on the wire as ``3121``,
   NOT as ``0xFBCF``. A two's-complement encoder would silently write a
   ~64000-tick garbage offset.
2. **The EEPROM Lock dance.** PR #21 of this repo exists because id/baud writes
   silently REVERTED on the next power-cycle when the Lock register (addr 55)
   was never opened. Addr 31 is in the same EEPROM region and carries the same
   hazard, so the unlock -> write -> relock order is asserted on the recorded
   register-write sequence, not merely on the end state.
3. **The LeRobot trap.** LeRobot's ``write_calibration`` writes
   ``Min_Position_Limit`` (addr 9) and ``Max_Position_Limit`` (addr 11)
   alongside the homing offset. In servo mode those CLAMP goals — narrowing the
   very reachable set the re-zero is meant to recover. ``write_offset`` writes
   addr 31 and nothing else, and that is pinned explicitly.

See ``docs/spikes/sts3215-offset-register.md`` for the sourced research.
"""

from __future__ import annotations

import pytest

from arm101.cli._errors import EXIT_ENV_ERROR, EXIT_USER_ERROR, CliError
from arm101.hardware.bus import (
    ADDR_HOMING_OFFSET,
    ADDR_LOCK,
    ADDR_TORQUE_ENABLE,
    OFFSET_MAX_MAGNITUDE,
    OFFSET_SIGN_BIT,
    FakeBus,
    FeetechBus,
    OverloadError,
    decode_offset,
    encode_offset,
)

#: Position-limit registers LeRobot's write_calibration also writes. Nothing in
#: this repo may ever write them — in servo mode they clamp goals.
ADDR_MIN_POSITION_LIMIT = 9
ADDR_MAX_POSITION_LIMIT = 11

#: A real, seam-evicting offset for elbow_flex — and the one our follower is
#: actually carrying. Its seam lands at raw 1073, strictly inside the measured
#: unreachable arc, so it does the job; a *fresh* re-zero would write the arc's
#: midpoint instead (see arm_spec.REZERO_ARCS, and tests/test_rezero.py, which
#: derives that midpoint rather than typing it — the arc gets re-measured).
#: Used throughout so the codec tests read as the real scenario. What matters
#: here is the WIRE ENCODING, which is the same shape for either number.
ELBOW_FLEX_OFFSET = 1073

#: ``encode_offset(-1073)`` -> ``2048 + 1073``. Spelled out because getting this
#: number wrong is the whole point of the codec tests.
ELBOW_FLEX_OFFSET_NEGATIVE_WIRE = 3121


# ---------------------------------------------------------------------------
# Recording packet-handler stubs (FeetechBus wire-level assertions)
# ---------------------------------------------------------------------------


class _RecordingPacket:
    """Records every 1- and 2-byte register write into ONE ordered event list.

    One list, not two, because the assertion that matters is the *interleaving*:
    torque-off (1 byte, addr 40) then unlock (1 byte, addr 55) then the offset
    (2 bytes, addr 31) then relock (1 byte, addr 55). Separate per-width lists
    could not express that order, and it is exactly the order the EEPROM write
    depends on.

    Parameters
    ----------
    fail_addrs:
        Addresses whose write should report a comms failure (``result=1``).
    error_addrs:
        Addresses whose write should report a servo status *error* byte
        (e.g. ``{40: 32}`` for a motor latched in overload).
    reads:
        Canned ``addr -> value`` map for :meth:`read2ByteTxRx`.
    """

    def __init__(
        self,
        fail_addrs: "set[int] | None" = None,
        error_addrs: "dict[int, int] | None" = None,
        reads: "dict[int, int] | None" = None,
    ) -> None:
        self.writes: list[tuple[int, int, int]] = []
        self.reads: list[tuple[int, int]] = []
        self._fail_addrs = set(fail_addrs or set())
        self._error_addrs = dict(error_addrs or {})
        self._canned = dict(reads or {})

    def _result(self, addr: int) -> "tuple[int, int]":
        return (1 if addr in self._fail_addrs else 0, self._error_addrs.get(addr, 0))

    def write1ByteTxRx(self, port, motor, addr, val):  # noqa: N802 - SDK spelling
        self.writes.append((motor, addr, val))
        return self._result(addr)

    def write2ByteTxRx(self, port, motor, addr, val):  # noqa: N802 - SDK spelling
        self.writes.append((motor, addr, val))
        return self._result(addr)

    def read2ByteTxRx(self, port, motor, addr):  # noqa: N802 - SDK spelling
        self.reads.append((motor, addr))
        result, error = self._result(addr)
        return self._canned.get(addr, 0), result, error

    def read1ByteTxRx(self, port, motor, addr):  # noqa: N802 - SDK spelling
        self.reads.append((motor, addr))
        result, error = self._result(addr)
        return self._canned.get(addr, 0), result, error


def _open_feetech(packet: _RecordingPacket) -> FeetechBus:
    """A FeetechBus wired to *packet*, marked open, with no serial port involved."""
    bus = FeetechBus(port="/dev/ttyUSB_fake")
    bus._packet_handler = packet
    bus._port_handler = object()
    bus._open = True
    return bus


# ===========================================================================
# 1. The codec — sign-magnitude on bit 11 (pure, hardware-free)
# ===========================================================================


def test_offset_constants_match_the_datasheet():
    """Address 31, sign bit 11, max magnitude 2047 — the three facts everything rests on."""
    assert ADDR_HOMING_OFFSET == 31
    assert OFFSET_SIGN_BIT == 11
    assert OFFSET_MAX_MAGNITUDE == (1 << 11) - 1 == 2047


@pytest.mark.parametrize("offset", [0, 1, 2, 1073, 2046, 2047])
def test_encode_offset_positive_is_the_magnitude_itself(offset):
    """A positive offset goes on the wire unchanged — the sign bit stays clear."""
    assert encode_offset(offset) == offset
    assert encode_offset(offset) & (1 << OFFSET_SIGN_BIT) == 0


@pytest.mark.parametrize(
    ("offset", "wire"),
    [
        (-1, 2049),  # 2048 | 1
        (-1073, ELBOW_FLEX_OFFSET_NEGATIVE_WIRE),  # 2048 | 1073
        (-2047, 4095),  # 2048 | 2047 — the widest negative
    ],
)
def test_encode_offset_negative_sets_bit_11(offset, wire):
    """A negative offset is 2048 + |offset|: sign in bit 11, magnitude below it."""
    assert encode_offset(offset) == wire


def test_encode_offset_is_not_twos_complement():
    """The regression this whole module exists to prevent.

    Two's complement would put -1 on the wire as 0xFFFF (65535) and -1073 as
    0xFBCF (64463). The STS3215 would read those as an absurd magnitude and the
    joint's reported position would be garbage. Sign-magnitude gives 2049 and
    3121.
    """
    assert encode_offset(-1) == 2049
    assert encode_offset(-1) != 0xFFFF
    assert encode_offset(-1073) == 3121
    assert encode_offset(-1073) != 0xFBCF


@pytest.mark.parametrize(
    ("wire", "offset"),
    [
        (0, 0),
        (1, 1),
        (1073, 1073),
        (2047, 2047),
        (2049, -1),
        (ELBOW_FLEX_OFFSET_NEGATIVE_WIRE, -1073),
        (4095, -2047),
    ],
)
def test_decode_offset_reads_sign_magnitude(wire, offset):
    """Bit 11 set means negative; the low 11 bits are the magnitude."""
    assert decode_offset(wire) == offset


def test_decode_offset_negative_zero_is_zero():
    """Wire 2048 is "negative zero" — a real encoding the servo can hold; it decodes to 0.

    The encoding is therefore NOT injective on the wire (0 and 2048 both mean an
    offset of zero), which is why the round-trip property is stated as
    ``decode(encode(x)) == x`` and never the other way round.
    """
    assert decode_offset(2048) == 0
    assert encode_offset(0) == 0  # we always write the canonical +0


def test_decode_offset_ignores_bits_above_the_12th():
    """A 2-byte read returns 16 bits; only the low 12 are the register.

    Mirrors ``read_position``'s ``& 0x0FFF``. Without this mask a stray high bit
    from a noisy read would decode into a nonsense magnitude rather than being
    discarded.
    """
    assert decode_offset(0xF000 | ELBOW_FLEX_OFFSET_NEGATIVE_WIRE) == -1073
    assert decode_offset(0x1000 | 5) == 5


def test_offset_round_trip_across_the_entire_range():
    """decode(encode(x)) == x for every representable offset, -2047 .. +2047."""
    for offset in range(-OFFSET_MAX_MAGNITUDE, OFFSET_MAX_MAGNITUDE + 1):
        assert decode_offset(encode_offset(offset)) == offset


def test_every_encoded_offset_fits_the_12_bit_register():
    """No representable offset can ever overflow into bits 12-15."""
    for offset in range(-OFFSET_MAX_MAGNITUDE, OFFSET_MAX_MAGNITUDE + 1):
        assert 0 <= encode_offset(offset) <= 0xFFF


@pytest.mark.parametrize("offset", [2048, -2048, 2073, -2073, 4096, -100000])
def test_encode_offset_rejects_unrepresentable_magnitudes(offset):
    """|offset| > 2047 cannot be encoded — reject loudly, never truncate.

    Truncating would corrupt the magnitude straight into the sign bit: 2048
    silently becomes "-0". LeRobot issue #3193 is a real SO-101 hitting exactly
    this (``ValueError: Magnitude 2073 exceeds 2047``); 2073 is in the parameter
    list for that reason.
    """
    with pytest.raises(CliError) as exc:
        encode_offset(offset)
    assert exc.value.code == EXIT_USER_ERROR
    assert "2047" in exc.value.message


def test_encode_offset_boundaries_are_inclusive():
    """+/-2047 encode; +/-2048 do not. The fence is exactly where the sign bit starts."""
    assert encode_offset(2047) == 2047
    assert encode_offset(-2047) == 4095
    for over in (2048, -2048):
        with pytest.raises(CliError):
            encode_offset(over)


# ===========================================================================
# 2. FeetechBus.read_offset — decode what the servo reports
# ===========================================================================


def test_feetech_read_offset_reads_address_31_as_two_bytes():
    """read_offset issues exactly one 2-byte read of addr 31."""
    packet = _RecordingPacket(reads={ADDR_HOMING_OFFSET: ELBOW_FLEX_OFFSET})
    bus = _open_feetech(packet)

    assert bus.read_offset(motor=3) == ELBOW_FLEX_OFFSET
    assert packet.reads == [(3, ADDR_HOMING_OFFSET)]


def test_feetech_read_offset_decodes_a_negative_offset():
    """A servo holding -1073 reports 3121 on the wire; read_offset must return -1073.

    Returning the raw 3121 would make the joint look like it had a large
    POSITIVE offset — the sign error that silently mis-frames every subsequent
    position comparison.
    """
    packet = _RecordingPacket(reads={ADDR_HOMING_OFFSET: ELBOW_FLEX_OFFSET_NEGATIVE_WIRE})
    bus = _open_feetech(packet)

    assert bus.read_offset(motor=3) == -1073


def test_feetech_read_offset_raises_cli_error_on_comms_failure():
    """A failed read is a CliError (EXIT_ENV_ERROR), never a traceback or a bogus 0."""
    packet = _RecordingPacket(fail_addrs={ADDR_HOMING_OFFSET})
    bus = _open_feetech(packet)

    with pytest.raises(CliError) as exc:
        bus.read_offset(motor=3)
    assert exc.value.code == EXIT_ENV_ERROR


def test_feetech_read_offset_requires_an_open_bus():
    bus = FeetechBus(port="/dev/ttyUSB_fake")
    with pytest.raises(CliError) as exc:
        bus.read_offset(motor=3)
    assert exc.value.code == EXIT_ENV_ERROR


# ===========================================================================
# 3. FeetechBus.write_offset — the register-write SEQUENCE
# ===========================================================================


def test_write_offset_emits_torque_off_unlock_write_relock_in_that_order():
    """The whole contract, asserted as one exact wire sequence.

    * torque-off (addr 40) FIRST — with torque on, the servo instantly
      re-interprets its own position against the new offset and could LURCH
      toward its standing goal. Inferred, not documented; treated as a hard
      safety rule.
    * unlock (addr 55 = 0) BEFORE the EEPROM write — without it the value
      updates live and then REVERTS on the next power-cycle (PR #21).
    * the offset (addr 31) — the only data register touched.
    * relock (addr 55 = 1) LAST — write-protection restored.
    """
    packet = _RecordingPacket()
    bus = _open_feetech(packet)

    bus.write_offset(motor=3, offset=ELBOW_FLEX_OFFSET)

    assert packet.writes == [
        (3, ADDR_TORQUE_ENABLE, 0),  # torque off — before anything else
        (3, ADDR_LOCK, 0),  # unlock EEPROM
        (3, ADDR_HOMING_OFFSET, ELBOW_FLEX_OFFSET),  # the offset itself
        (3, ADDR_LOCK, 1),  # relock EEPROM
    ]


def test_write_offset_puts_the_sign_magnitude_encoding_on_the_wire():
    """A negative offset reaches addr 31 as 2048 + |offset|, not as two's complement."""
    packet = _RecordingPacket()
    bus = _open_feetech(packet)

    bus.write_offset(motor=3, offset=-ELBOW_FLEX_OFFSET)

    offset_writes = [w for w in packet.writes if w[1] == ADDR_HOMING_OFFSET]
    assert offset_writes == [(3, ADDR_HOMING_OFFSET, ELBOW_FLEX_OFFSET_NEGATIVE_WIRE)]


def test_write_offset_never_touches_the_position_limit_registers():
    """The LeRobot trap, pinned.

    LeRobot's ``write_calibration`` writes Min_Position_Limit (9) and
    Max_Position_Limit (11) alongside the homing offset. In servo mode the
    firmware CLAMPS goals to that window — narrowing the reachable set the
    re-zero exists to recover. Our factory limits are the wide-open 0/4095 and
    must stay that way.
    """
    packet = _RecordingPacket()
    bus = _open_feetech(packet)

    bus.write_offset(motor=3, offset=ELBOW_FLEX_OFFSET)

    touched = {addr for _motor, addr, _val in packet.writes}
    assert ADDR_MIN_POSITION_LIMIT not in touched
    assert ADDR_MAX_POSITION_LIMIT not in touched
    # Stronger: an exact allow-list. Any NEW register this method learns to
    # write must be justified here first.
    assert touched == {ADDR_TORQUE_ENABLE, ADDR_LOCK, ADDR_HOMING_OFFSET}


def test_write_offset_out_of_range_writes_nothing_at_all():
    """A rejected offset must not disable torque, and must not open the EEPROM.

    Validation happens BEFORE any wire traffic, so a bad ``--offset`` leaves the
    arm exactly as it found it — torque still on, Lock still closed.
    """
    packet = _RecordingPacket()
    bus = _open_feetech(packet)

    with pytest.raises(CliError) as exc:
        bus.write_offset(motor=3, offset=2048)

    assert exc.value.code == EXIT_USER_ERROR
    assert packet.writes == []


def test_write_offset_failed_eeprom_write_still_relocks():
    """A failing addr-31 write must NEVER strand the motor at Lock=0.

    An EEPROM left unlocked is a motor whose id, baud and offset can be
    clobbered by any subsequent stray write — the state PR #21 was written to
    avoid. The original error still propagates.
    """
    packet = _RecordingPacket(fail_addrs={ADDR_HOMING_OFFSET})
    bus = _open_feetech(packet)

    with pytest.raises(CliError) as exc:
        bus.write_offset(motor=3, offset=ELBOW_FLEX_OFFSET)

    assert exc.value.code == EXIT_ENV_ERROR
    assert packet.writes == [
        (3, ADDR_TORQUE_ENABLE, 0),
        (3, ADDR_LOCK, 0),
        (3, ADDR_HOMING_OFFSET, ELBOW_FLEX_OFFSET),  # failed
        (3, ADDR_LOCK, 1),  # best-effort relock — the motor is never left open
    ]


def test_write_offset_relock_failure_preserves_the_original_error():
    """If the relock ALSO fails, the caller still learns why the OFFSET write failed.

    The re-lock is best-effort: letting its error replace the offset write's
    would hide the real fault behind a secondary one, and the operator would go
    hunting for a Lock problem that is not the problem.
    """

    class _OffsetAndRelockFail(_RecordingPacket):
        """Unlock (55 <- 0) succeeds; the offset write and the RELOCK (55 <- 1) fail."""

        def _result(self, addr: int) -> "tuple[int, int]":
            return (0, 0)

        def write1ByteTxRx(self, port, motor, addr, val):  # noqa: N802 - SDK spelling
            self.writes.append((motor, addr, val))
            failed = addr == ADDR_LOCK and val == 1  # the relock, not the unlock
            return (1 if failed else 0), 0

        def write2ByteTxRx(self, port, motor, addr, val):  # noqa: N802 - SDK spelling
            self.writes.append((motor, addr, val))
            return (1 if addr == ADDR_HOMING_OFFSET else 0), 0

    packet = _OffsetAndRelockFail()
    bus = _open_feetech(packet)

    with pytest.raises(CliError) as exc:
        bus.write_offset(motor=3, offset=ELBOW_FLEX_OFFSET)

    # The surfaced error is the OFFSET write's, not the relock's.
    assert "offset" in exc.value.message.lower()
    assert "re-lock" not in exc.value.message.lower()
    # The relock was still attempted — best-effort means tried, not skipped.
    assert packet.writes[-1] == (3, ADDR_LOCK, 1)


def test_write_offset_unlock_failure_never_writes_the_offset():
    """If the EEPROM cannot be unlocked, the offset write is not attempted.

    Writing it anyway would produce the PR #21 failure mode by construction: a
    value that reads back correctly and then vanishes on the next power-cycle.
    """
    packet = _RecordingPacket(fail_addrs={ADDR_LOCK})
    bus = _open_feetech(packet)

    with pytest.raises(CliError):
        bus.write_offset(motor=3, offset=ELBOW_FLEX_OFFSET)

    assert packet.writes == [
        (3, ADDR_TORQUE_ENABLE, 0),  # torque came off
        (3, ADDR_LOCK, 0),  # unlock attempted, and failed
    ]


def test_write_offset_torque_off_failure_never_opens_the_eeprom():
    """If torque cannot be disabled, nothing is unlocked and nothing is written.

    The safety rule is not advisory: an offset written to a torqued servo may
    make it lurch. If we cannot prove the motor is limp, we do not proceed.
    """
    packet = _RecordingPacket(fail_addrs={ADDR_TORQUE_ENABLE})
    bus = _open_feetech(packet)

    with pytest.raises(CliError):
        bus.write_offset(motor=3, offset=ELBOW_FLEX_OFFSET)

    assert packet.writes == [(3, ADDR_TORQUE_ENABLE, 0)]


def test_write_offset_on_an_overloaded_motor_raises_overload_error_and_writes_no_eeprom():
    """A motor latched in overload tags every response with bit 5 (0x20).

    The torque-off write surfaces that as :class:`OverloadError` — and because
    it happens before the unlock, the EEPROM is never opened. The caller (t8)
    must ``clear_overload`` first.
    """
    packet = _RecordingPacket(error_addrs={ADDR_TORQUE_ENABLE: 32})
    bus = _open_feetech(packet)

    with pytest.raises(OverloadError):
        bus.write_offset(motor=3, offset=ELBOW_FLEX_OFFSET)

    touched = {addr for _m, addr, _v in packet.writes}
    assert ADDR_LOCK not in touched
    assert ADDR_HOMING_OFFSET not in touched


def test_feetech_write_offset_requires_an_open_bus():
    bus = FeetechBus(port="/dev/ttyUSB_fake")
    with pytest.raises(CliError) as exc:
        bus.write_offset(motor=3, offset=0)
    assert exc.value.code == EXIT_ENV_ERROR


# ===========================================================================
# 4. FeetechBus.read_info — the offset is visible, and it is SIGNED
# ===========================================================================


def test_feetech_info_registers_includes_homing_offset():
    """addr 31 (2 bytes) is in the read-only snapshot map.

    If it is not, ``arm read`` silently reports 0 on real hardware while the
    servo actually holds a nonzero offset — the register that most needs to be
    inspectable would be the one register you cannot see.
    """
    assert FeetechBus._INFO_REGISTERS.get("homing_offset") == (31, 2)


def test_feetech_read_info_reports_the_offset_decoded_not_raw():
    """``read_info["homing_offset"]`` is the SIGNED offset, never the wire value.

    The wire encoding must not escape ``bus.py``. A -1073 offset surfacing as
    "3121" in ``arm read`` would read as a plausible positive offset and mislead
    the human doing the re-zero.
    """
    packet = _RecordingPacket(reads={ADDR_HOMING_OFFSET: ELBOW_FLEX_OFFSET_NEGATIVE_WIRE})
    bus = _open_feetech(packet)

    info = bus.read_info(motor=3)

    assert info["homing_offset"] == -1073
    assert info["homing_offset"] == bus.read_offset(motor=3)  # one number, one meaning


# ===========================================================================
# 5. FakeBus — the offset register and its effect on present_position
# ===========================================================================


def test_fakebus_offset_defaults_to_zero():
    bus = FakeBus()
    bus.open()
    assert bus.read_offset(1) == 0
    assert bus.read_info(1)["homing_offset"] == 0


def test_fakebus_offset_round_trips_through_write_then_read():
    bus = FakeBus()
    bus.open()

    bus.write_offset(3, -ELBOW_FLEX_OFFSET)

    assert bus.read_offset(3) == -ELBOW_FLEX_OFFSET
    assert bus.read_info(3)["homing_offset"] == -ELBOW_FLEX_OFFSET


def test_fakebus_offset_can_be_seeded_from_the_constructor():
    bus = FakeBus(offsets={3: 1073})
    bus.open()
    assert bus.read_offset(3) == 1073
    assert bus.read_offset(1) == 0  # unseeded motors read the factory 0


def test_fakebus_write_offset_records_the_same_sequence_as_the_real_bus():
    """The fake's register ledger mirrors FeetechBus's wire sequence exactly.

    If the fake skipped the Lock dance, every test built on it would "prove"
    a write that on hardware would silently revert on the next power-cycle.
    """
    bus = FakeBus()
    bus.open()

    bus.write_offset(3, -ELBOW_FLEX_OFFSET)

    assert [(w["motor"], w["addr"], w["value"]) for w in bus.register_writes] == [
        (3, ADDR_TORQUE_ENABLE, 0),
        (3, ADDR_LOCK, 0),
        (3, ADDR_HOMING_OFFSET, ELBOW_FLEX_OFFSET_NEGATIVE_WIRE),  # sign-magnitude on the wire
        (3, ADDR_LOCK, 1),
    ]
    assert bus.offset_writes == [{"motor": 3, "offset": -ELBOW_FLEX_OFFSET}]
    # The torque-off funnels through enable_torque, so it shows up there too —
    # one operation, one wire byte, one ledger entry per view.
    assert bus.torque_writes == [{"motor": 3, "on": False}]


def test_fakebus_write_offset_never_touches_the_position_limits():
    """The LeRobot trap, pinned on the fake as well as on the real bus."""
    bus = FakeBus()
    bus.open()

    bus.write_offset(3, ELBOW_FLEX_OFFSET)

    touched = {w["addr"] for w in bus.register_writes}
    assert ADDR_MIN_POSITION_LIMIT not in touched
    assert ADDR_MAX_POSITION_LIMIT not in touched
    assert touched == {ADDR_TORQUE_ENABLE, ADDR_LOCK, ADDR_HOMING_OFFSET}


def test_fakebus_write_offset_relocks_the_eeprom():
    """Lock ends closed (1) — and read_lock agrees with the ledger."""
    bus = FakeBus()
    bus.open()
    assert bus.read_lock(3) == 0  # factory default in the fake

    bus.write_offset(3, ELBOW_FLEX_OFFSET)

    assert bus.read_lock(3) == 1
    lock_values = [w["value"] for w in bus.register_writes if w["addr"] == ADDR_LOCK]
    assert lock_values == [0, 1]  # opened, then closed


def test_fakebus_write_offset_rejects_out_of_range_without_touching_the_bus():
    bus = FakeBus()
    bus.open()

    with pytest.raises(CliError) as exc:
        bus.write_offset(3, -2048)

    assert exc.value.code == EXIT_USER_ERROR
    assert bus.register_writes == []
    assert bus.offset_writes == []
    assert bus.read_offset(3) == 0  # unchanged


def test_fakebus_write_offset_requires_an_open_bus():
    bus = FakeBus()
    with pytest.raises(CliError) as exc:
        bus.write_offset(3, 0)
    assert exc.value.code == EXIT_ENV_ERROR


def test_fakebus_failed_offset_write_still_relocks():
    """The overload seam fires on the EEPROM write; the fake still relocks.

    Mirrors ``test_write_offset_failed_eeprom_write_still_relocks`` on the real
    bus, so a caller's recovery path can be tested against the fake and mean
    something.
    """
    bus = FakeBus()
    bus.open()
    bus.fail_with_overload_on_op(2)  # op 1 = the torque-off; op 2 = the offset write

    with pytest.raises(OverloadError):
        bus.write_offset(3, ELBOW_FLEX_OFFSET)

    assert [(w["addr"], w["value"]) for w in bus.register_writes] == [
        (ADDR_TORQUE_ENABLE, 0),
        (ADDR_LOCK, 0),
        (ADDR_LOCK, 1),  # relocked even though the offset never landed
    ]

    # The servo is still latched, so recover the way a real caller must before
    # reading anything back (clear_overload also disarms the fake's seam).
    bus.clear_overload(3)

    assert bus.read_lock(3) == 1  # EEPROM closed again — never stranded open
    assert bus.read_offset(3) == 0  # the write did not take effect


# ---------------------------------------------------------------------------
# 5b. The offset's EFFECT: Present_Position = Actual_Position - Homing_Offset
# ---------------------------------------------------------------------------


def test_fakebus_present_position_shifts_by_the_offset():
    """The exact issue-#35 arithmetic, simulated.

    ``elbow_flex`` rests at raw 126 (past its wrap). With H = 1073 the servo
    reports ``(126 - 1073) mod 4096 == 3149`` — and the seam it used to cross
    mid-travel now sits at raw 1073, strictly inside the arc it cannot reach
    (``arm_spec.REZERO_ARCS``, measured 2026-07-12). Evicted, which is the goal.
    """
    bus = FakeBus(positions={3: 126})
    bus.open()

    assert bus.read_position(3) == 126  # no offset yet: raw == reported

    bus.write_offset(3, ELBOW_FLEX_OFFSET)

    assert bus.read_position(3) == 3149
    assert bus.read_info(3)["present_position"] == 3149


def test_fakebus_present_position_and_read_info_agree_under_an_offset():
    """read_position and read_info's present_position are the SAME register (addr 56).

    They must therefore never disagree. A fake in which one applies the offset
    and the other does not would let a test "prove" behaviour no servo can
    exhibit — the lesson ``tests/_fakes.py`` was written to enforce.
    """
    bus = FakeBus(positions={1: 2000, 2: 500})
    bus.open()
    bus.write_offset(1, 800)
    bus.write_offset(2, -800)

    for motor in (1, 2):
        assert bus.read_position(motor) == bus.read_info(motor)["present_position"]

    assert bus.read_position(1) == 1200  # 2000 - 800
    assert bus.read_position(2) == 1300  # 500 - (-800)


def test_fakebus_present_position_wraps_modulo_4096():
    """The reported position stays inside [0, 4095] — the seam MOVES, it does not vanish."""
    bus = FakeBus(positions={1: 100})
    bus.open()

    bus.write_offset(1, 500)

    assert bus.read_position(1) == 3696  # (100 - 500) mod 4096


def test_fakebus_zero_offset_is_the_identity():
    """With no offset the fake reports the encoder byte-for-byte.

    Load-bearing: existing fixtures seed positions outside [0, 4095] (e.g. 5000)
    and must keep reading back exactly what they were given.
    """
    bus = FakeBus(positions={1: 5000})
    bus.open()

    assert bus.read_position(1) == 5000
    assert bus.read_info(1)["present_position"] == 5000


def test_fakebus_unwrapped_seam_models_the_pessimistic_firmware_reading():
    """``offset_wraps=False`` models the reading that would make the re-zero USELESS.

    The spike (docs/spikes/sts3215-offset-register.md, section 4) could not find
    a primary source for whether the corrected position is reduced modulo 4096
    (seam MOVES to raw == Ofs — the fix works) or reported as a plain signed
    subtraction (seam stays pinned at the physical 4095->0 rollover — the fix
    achieves nothing). Every source points at the modular reading and LeRobot
    ships it, so it is the fake's DEFAULT — but the assumption is unproven, so
    the other branch is modelled too rather than being silently assumed away.
    Hardware test step 10 settles it; if it settles the wrong way, this is the
    behaviour t8 must build against.
    """
    bus = FakeBus(positions={1: 100}, offset_wraps=False)
    bus.open()

    bus.write_offset(1, 500)

    assert bus.read_position(1) == -400  # 100 - 500, unwrapped — no modular reduction


def test_fakebus_read_arm_style_snapshot_is_read_only():
    """Reading the offset writes nothing — ``arm read`` must never mutate the servo."""
    bus = FakeBus(offsets={3: 1073})
    bus.open()

    bus.read_offset(3)
    bus.read_info(3)
    bus.read_position(3)

    assert bus.register_writes == []
    assert bus.offset_writes == []


# ===========================================================================
# 6. ServoModelBus — the motion sim stays honest under an offset
# ===========================================================================


def test_servo_model_bus_reports_the_offset_shifted_position_from_both_reads():
    """``ServoModelBus`` overrides ``read_info``; it must not lose the offset doing so.

    ``read_position`` (inherited) and ``read_info["present_position"]``
    (overridden, and driven by the motion sim) are the SAME register on the
    wire. If only one of them applied the offset, a test could pass against a
    bus that cannot exist.
    """
    from tests._fakes import ServoModelBus

    bus = ServoModelBus(positions={1: 2000})
    bus.open()
    bus.write_offset(1, 800)

    assert bus.true_position(1) == 2000  # the shaft has not moved
    assert bus.read_position(1) == 1200  # 2000 - 800, as the servo would report
    assert bus.read_info(1)["present_position"] == 1200


def test_servo_model_bus_goals_are_commanded_in_the_corrected_frame():
    """A goal written after a re-zero is in the SAME frame the servo now reports in.

    ``Present = Actual - Ofs`` implies ``Actual = Goal + Ofs``: commanding 1500
    on a joint offset by 800 must park the physical shaft at raw 2300, and the
    joint must then REPORT 1500. Goal and feedback living in different frames is
    the failure mode that would make a re-zero worse than useless, so the fake
    refuses to model it.
    """
    from tests._fakes import ServoModelBus

    bus = ServoModelBus(positions={1: 2000}, ticks_per_poll=1000)
    bus.open()
    bus.write_offset(1, 800)

    bus.write_goal_position(1, 1500)  # corrected frame
    for _ in range(5):  # polls = simulated time; let the shaft arrive
        reported = bus.read_info(1)["present_position"]

    assert bus.true_position(1) == 2300  # raw shaft: 1500 + 800
    assert reported == 1500  # what the servo reports: back in the corrected frame
