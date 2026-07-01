"""Tests for the bus-layer overload classification + torque-limit support.

Covers (arm101.hardware.bus):

* ``is_overload(error_byte)`` — bit 5 (0x20) classification helper.
* ``OverloadError`` — a ``CliError`` subtype carrying ``.motor``/``.error_byte``,
  raised INSTEAD OF the generic ``CliError`` wherever a bus read/write's SDK
  status error byte flags an overload.
* ``read_torque_limit`` / ``write_torque_limit`` (STS3215 RAM addr 48) and
  ``clear_overload`` (Torque_Enable=0, addr 40) on both ``FeetechBus`` and
  ``FakeBus``.
* ``FakeBus``'s overload-simulation seam (``overload_after_ops`` /
  ``fail_with_overload_on_op``) and its per-address write ledger
  (``register_writes``), used here to prove no bus operation ever writes the
  EEPROM protection registers at addr 34/35/36.

TDD: written before the corresponding bus.py changes; they must fail against
the current code and drive the implementation.
"""

from __future__ import annotations


class _ScriptedPacket:
    """Minimal scservo_sdk-shaped packet-handler stub with a scripted outcome.

    Every 1/2-byte read/write returns the same ``(result, error)`` (for
    writes) or ``(read_value, result, error)`` (for reads) pair, and is
    recorded in ``.writes`` / ``.reads`` in call order for assertions.
    Defaults to an always-succeeding stub (``result=0, error=0``); pass
    *result*/*error* to script a comms failure or a status-error byte
    (e.g. ``error=32`` to simulate an overload).
    """

    def __init__(self, result: int = 0, error: int = 0, read_value: int = 0) -> None:
        self.result = result
        self.error = error
        self.read_value = read_value
        self.writes: list[tuple[int, int, int]] = []
        self.reads: list[tuple[int, int]] = []

    def write1ByteTxRx(self, port, motor, addr, val):
        self.writes.append((motor, addr, val))
        return self.result, self.error

    def write2ByteTxRx(self, port, motor, addr, val):
        self.writes.append((motor, addr, val))
        return self.result, self.error

    def read1ByteTxRx(self, port, motor, addr):
        self.reads.append((motor, addr))
        return self.read_value, self.result, self.error

    def read2ByteTxRx(self, port, motor, addr):
        self.reads.append((motor, addr))
        return self.read_value, self.result, self.error

    def ping(self, port, motor):
        return 0, self.result, self.error


def _make_open_feetech_bus(packet: "_ScriptedPacket"):
    """Return a FeetechBus with its internals patched to *packet*, pre-opened."""
    from arm101.hardware.bus import FeetechBus

    bus = FeetechBus(port="/dev/ttyUSB0")
    bus._packet_handler = packet
    bus._port_handler = object()
    bus._open = True
    return bus


# ---------------------------------------------------------------------------
# 1. is_overload() — bit 5 (0x20) classification
# ---------------------------------------------------------------------------


def test_is_overload_classifies_bit_5():
    from arm101.hardware.bus import is_overload

    cases = [
        (0, False),
        (1, False),
        (2, False),
        (16, False),
        (32, True),  # bit 5 alone
        (33, True),  # bit 5 + bit 0
        (96, True),  # bit 5 + bit 6
        (64, False),  # bit 6 only
        (255, True),  # all bits set, including bit 5
    ]
    for error_byte, expected in cases:
        assert is_overload(error_byte) is expected, error_byte


# ---------------------------------------------------------------------------
# 2. OverloadError — CliError subtype
# ---------------------------------------------------------------------------


def test_overload_error_is_a_cli_error_subtype_carrying_motor_and_error_byte():
    from arm101.cli._errors import EXIT_ENV_ERROR, CliError
    from arm101.hardware.bus import OverloadError

    err = OverloadError(motor=3, error_byte=32)

    assert isinstance(err, CliError)
    assert err.code == EXIT_ENV_ERROR
    assert err.motor == 3
    assert err.error_byte == 32
    assert err.message
    assert err.remediation


def test_overload_error_caught_by_bare_except_cli_error():
    """An OverloadError must still be caught by an unmodified `except CliError`."""
    from arm101.cli._errors import CliError
    from arm101.hardware.bus import OverloadError

    caught = None
    try:
        raise OverloadError(motor=2, error_byte=32)
    except CliError as exc:
        caught = exc
    assert isinstance(caught, OverloadError)


def test_overload_error_custom_message_is_preserved():
    from arm101.hardware.bus import OverloadError

    err = OverloadError(motor=1, error_byte=32, message="custom overload text")
    assert err.message == "custom overload text"


# ---------------------------------------------------------------------------
# 3. FeetechBus — overload vs. plain CliError at the checked call sites
# ---------------------------------------------------------------------------


def test_feetech_write_goal_position_overload_raises_overload_error():
    from arm101.hardware.bus import OverloadError

    packet = _ScriptedPacket(result=0, error=32)
    bus = _make_open_feetech_bus(packet)

    import pytest

    with pytest.raises(OverloadError) as exc:
        bus.write_goal_position(motor=6, position=2048)
    assert exc.value.motor == 6
    assert exc.value.error_byte == 32


def test_feetech_enable_torque_overload_raises_overload_error():
    from arm101.hardware.bus import OverloadError

    packet = _ScriptedPacket(result=0, error=32)
    bus = _make_open_feetech_bus(packet)

    import pytest

    with pytest.raises(OverloadError) as exc:
        bus.enable_torque(motor=6, on=True)
    assert exc.value.motor == 6
    assert exc.value.error_byte == 32


def test_feetech_read_info_overload_raises_overload_error():
    from arm101.hardware.bus import OverloadError

    packet = _ScriptedPacket(result=0, error=32)
    bus = _make_open_feetech_bus(packet)

    import pytest

    with pytest.raises(OverloadError):
        bus.read_info(motor=6)


def test_feetech_write_acceleration_overload_raises_overload_error():
    from arm101.hardware.bus import OverloadError

    packet = _ScriptedPacket(result=0, error=32)
    bus = _make_open_feetech_bus(packet)

    import pytest

    with pytest.raises(OverloadError):
        bus.write_acceleration(motor=6, value=50)


def test_feetech_write_goal_speed_overload_raises_overload_error():
    from arm101.hardware.bus import OverloadError

    packet = _ScriptedPacket(result=0, error=32)
    bus = _make_open_feetech_bus(packet)

    import pytest

    with pytest.raises(OverloadError):
        bus.write_goal_speed(motor=6, value=100)


def test_feetech_read_lock_overload_raises_overload_error():
    from arm101.hardware.bus import OverloadError

    packet = _ScriptedPacket(result=0, error=32)
    bus = _make_open_feetech_bus(packet)

    import pytest

    with pytest.raises(OverloadError):
        bus.read_lock(motor=6)


def test_feetech_read_position_overload_raises_overload_error():
    from arm101.hardware.bus import OverloadError

    packet = _ScriptedPacket(result=0, error=32)
    bus = _make_open_feetech_bus(packet)

    import pytest

    with pytest.raises(OverloadError):
        bus.read_position(motor=6)


def test_feetech_write_goal_position_non_overload_error_raises_plain_cli_error():
    """A non-overload status error bit still raises the generic CliError."""
    from arm101.cli._errors import EXIT_ENV_ERROR, CliError
    from arm101.hardware.bus import OverloadError

    packet = _ScriptedPacket(result=0, error=1)  # some other status error bit
    bus = _make_open_feetech_bus(packet)

    import pytest

    with pytest.raises(CliError) as exc:
        bus.write_goal_position(motor=6, position=2048)
    assert not isinstance(exc.value, OverloadError)
    assert exc.value.code == EXIT_ENV_ERROR


def test_feetech_write_goal_position_comm_failure_raises_plain_cli_error():
    """A comms failure (nonzero result, no status error byte) is not an overload."""
    from arm101.cli._errors import CliError
    from arm101.hardware.bus import OverloadError

    packet = _ScriptedPacket(result=1, error=0)
    bus = _make_open_feetech_bus(packet)

    import pytest

    with pytest.raises(CliError) as exc:
        bus.write_goal_position(motor=6, position=2048)
    assert not isinstance(exc.value, OverloadError)


# ---------------------------------------------------------------------------
# 4. Torque_Limit (addr 48) — FeetechBus
# ---------------------------------------------------------------------------


def test_feetech_read_torque_limit_reads_address_48():
    packet = _ScriptedPacket(read_value=850)
    bus = _make_open_feetech_bus(packet)

    value = bus.read_torque_limit(motor=4)

    assert value == 850
    assert packet.reads == [(4, 48)]


def test_feetech_write_torque_limit_writes_address_48():
    packet = _ScriptedPacket()
    bus = _make_open_feetech_bus(packet)

    bus.write_torque_limit(motor=4, value=600)

    assert packet.writes == [(4, 48, 600)]


def test_feetech_write_torque_limit_out_of_range_raises_user_error():
    from arm101.cli._errors import EXIT_USER_ERROR, CliError

    packet = _ScriptedPacket()
    bus = _make_open_feetech_bus(packet)

    import pytest

    with pytest.raises(CliError) as exc:
        bus.write_torque_limit(motor=4, value=1001)
    assert exc.value.code == EXIT_USER_ERROR

    with pytest.raises(CliError) as exc:
        bus.write_torque_limit(motor=4, value=-1)
    assert exc.value.code == EXIT_USER_ERROR

    assert packet.writes == []  # rejected before touching the bus


def test_feetech_write_torque_limit_overload_raises_overload_error():
    from arm101.hardware.bus import OverloadError

    packet = _ScriptedPacket(result=0, error=32)
    bus = _make_open_feetech_bus(packet)

    import pytest

    with pytest.raises(OverloadError):
        bus.write_torque_limit(motor=4, value=500)


# ---------------------------------------------------------------------------
# 5. clear_overload — FeetechBus
# ---------------------------------------------------------------------------


def test_feetech_clear_overload_disables_torque_at_address_40():
    packet = _ScriptedPacket()
    bus = _make_open_feetech_bus(packet)

    bus.clear_overload(motor=5)

    assert packet.writes == [(5, 40, 0)]


# ---------------------------------------------------------------------------
# 6. FakeBus — torque_limit round-trip
# ---------------------------------------------------------------------------


def test_fakebus_torque_limit_default_1000():
    from arm101.hardware.bus import FakeBus

    bus = FakeBus()
    bus.open()
    assert bus.read_torque_limit(1) == 1000


def test_fakebus_torque_limit_round_trip():
    from arm101.hardware.bus import FakeBus

    bus = FakeBus()
    bus.open()

    bus.write_torque_limit(motor=3, value=500)

    assert bus.read_torque_limit(3) == 500
    assert bus.read_torque_limit(1) == 1000  # other motors stay at the default


def test_fakebus_torque_limit_constructor_seed():
    from arm101.hardware.bus import FakeBus

    bus = FakeBus(torque_limits={2: 750})
    bus.open()
    assert bus.read_torque_limit(2) == 750


def test_fakebus_write_torque_limit_out_of_range_raises_user_error():
    import pytest

    from arm101.cli._errors import EXIT_USER_ERROR, CliError
    from arm101.hardware.bus import FakeBus

    bus = FakeBus()
    bus.open()

    with pytest.raises(CliError) as exc:
        bus.write_torque_limit(motor=1, value=1001)
    assert exc.value.code == EXIT_USER_ERROR

    with pytest.raises(CliError) as exc:
        bus.write_torque_limit(motor=1, value=-1)
    assert exc.value.code == EXIT_USER_ERROR


def test_fakebus_write_torque_limit_not_open_raises_env_error():
    import pytest

    from arm101.cli._errors import EXIT_ENV_ERROR, CliError
    from arm101.hardware.bus import FakeBus

    bus = FakeBus()
    with pytest.raises(CliError) as exc:
        bus.write_torque_limit(motor=1, value=500)
    assert exc.value.code == EXIT_ENV_ERROR


def test_fakebus_read_torque_limit_not_open_raises_env_error():
    import pytest

    from arm101.cli._errors import EXIT_ENV_ERROR, CliError
    from arm101.hardware.bus import FakeBus

    bus = FakeBus()
    with pytest.raises(CliError) as exc:
        bus.read_torque_limit(1)
    assert exc.value.code == EXIT_ENV_ERROR


# ---------------------------------------------------------------------------
# 7. FakeBus — clear_overload
# ---------------------------------------------------------------------------


def test_fakebus_clear_overload_records_torque_off_and_addr_40():
    from arm101.hardware.bus import FakeBus

    bus = FakeBus()
    bus.open()

    bus.enable_torque(motor=1, on=True)
    bus.clear_overload(motor=1)

    assert bus.torque_writes[-1] == {"motor": 1, "on": False}
    assert bus.register_writes[-1] == {"motor": 1, "addr": 40, "value": 0}


# ---------------------------------------------------------------------------
# 8. FakeBus — overload-simulation seam
# ---------------------------------------------------------------------------


def test_fakebus_overload_seam_fires_on_nth_op():
    import pytest

    from arm101.hardware.bus import FakeBus, OverloadError

    bus = FakeBus().fail_with_overload_on_op(3)
    bus.open()

    bus.read_position(1)  # op 1 — fine
    bus.read_info(1)  # op 2 — fine
    with pytest.raises(OverloadError) as exc:
        bus.read_position(1)  # op 3 — overload

    assert exc.value.motor == 1
    assert exc.value.error_byte == 32


def test_fakebus_overload_seam_latches_until_cleared():
    import pytest

    from arm101.hardware.bus import FakeBus, OverloadError

    bus = FakeBus().fail_with_overload_on_op(1)
    bus.open()

    with pytest.raises(OverloadError):
        bus.read_position(1)
    with pytest.raises(OverloadError):
        bus.read_position(1)  # still latched — the fault persists

    bus.clear_overload(1)  # recovery action: disarms the simulated fault too

    assert bus.read_position(1) == 2048  # resumes normal operation


def test_fakebus_overload_after_ops_attribute_can_be_set_directly():
    import pytest

    from arm101.hardware.bus import FakeBus, OverloadError

    bus = FakeBus()
    bus.open()
    bus.overload_after_ops = 2

    bus.read_position(1)  # op 1 — fine
    with pytest.raises(OverloadError):
        bus.read_position(1)  # op 2 — overload


def test_fakebus_default_has_no_overload_seam():
    from arm101.hardware.bus import FakeBus

    bus = FakeBus()
    bus.open()
    for _ in range(10):
        bus.read_position(1)  # never raises when overload_after_ops is unset


# ---------------------------------------------------------------------------
# 9. FakeBus — writes-by-address ledger proves protection registers are safe
# ---------------------------------------------------------------------------


def test_fakebus_never_writes_eeprom_protection_registers():
    """No bus write operation ever touches EEPROM addr 34/35/36.

    Those are Protective_Torque (34), Protection_Time (35), and
    Overload_Torque (36) — firmware fault-detection thresholds this bus
    intentionally never programs. Verified across the whole write surface
    via FakeBus's per-address ledger, not just the new torque-limit/overload
    methods.
    """
    from arm101.hardware.bus import FakeBus

    bus = FakeBus()
    bus.open()

    bus.write_id_baudrate(motor=1, new_id=2, baudrate=1_000_000)
    bus.write_baudrate(motor=2, baudrate=500_000)
    bus.enable_torque(motor=2, on=True)
    bus.write_goal_position(motor=2, position=2048)
    bus.write_acceleration(motor=2, value=50)
    bus.write_goal_speed(motor=2, value=200)
    bus.write_torque_limit(motor=2, value=800)
    bus.clear_overload(motor=2)

    touched_addrs = {entry["addr"] for entry in bus.register_writes}

    assert touched_addrs.isdisjoint({34, 35, 36})
    assert touched_addrs  # sanity: the ledger is not vacuously empty
    assert len(bus.register_writes) >= 8


def test_fakebus_register_writes_records_torque_limit_write():
    from arm101.hardware.bus import FakeBus

    bus = FakeBus()
    bus.open()

    bus.write_torque_limit(motor=7, value=333)

    assert {"motor": 7, "addr": 48, "value": 333} in bus.register_writes


# ---------------------------------------------------------------------------
# 10. MotorBus interface — new abstract methods on both implementations
# ---------------------------------------------------------------------------


def test_motor_bus_declares_new_methods_abstract():
    from arm101.hardware.bus import MotorBus

    assert "read_torque_limit" in MotorBus.__abstractmethods__
    assert "write_torque_limit" in MotorBus.__abstractmethods__
    assert "clear_overload" in MotorBus.__abstractmethods__


def test_motor_bus_interface_includes_torque_limit_and_clear_overload():
    from arm101.hardware.bus import FakeBus, FeetechBus, MotorBus

    fake = FakeBus()
    real = FeetechBus(port="/dev/ttyUSB0")
    for bus in (fake, real):
        assert isinstance(bus, MotorBus)
        assert hasattr(bus, "read_torque_limit")
        assert hasattr(bus, "write_torque_limit")
        assert hasattr(bus, "clear_overload")
