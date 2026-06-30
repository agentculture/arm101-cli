"""Tests for arm101.hardware.bus — MotorBus interface, FeetechBus (lazy-import),
and FakeBus (in-memory).

TDD: these tests were written before bus.py existed and drive the implementation.
"""

from __future__ import annotations

import sys

# ---------------------------------------------------------------------------
# 1. Zero-dep import guarantees
# ---------------------------------------------------------------------------


def test_import_arm101_cli_zero_deps():
    """import arm101.cli must work with no third-party packages installed."""
    # If the import failed, this module would have already errored at collection
    # time.  Belt-and-suspenders: confirm the module is present in sys.modules.
    import arm101.cli  # noqa: F401

    assert "arm101.cli" in sys.modules


def test_import_arm101_hardware_bus_zero_deps():
    """import arm101.hardware.bus must work with no third-party packages installed."""
    import arm101.hardware.bus  # noqa: F401

    assert "arm101.hardware.bus" in sys.modules


def test_sdk_not_imported_at_module_level():
    """scservo_sdk must NOT be imported as a side-effect of importing the bus module."""
    import arm101.hardware.bus  # noqa: F401

    assert "scservo_sdk" not in sys.modules


# ---------------------------------------------------------------------------
# 2. FeetechBus raises CliError(EXIT_ENV_ERROR) when SDK is absent
# ---------------------------------------------------------------------------


def test_feetech_bus_open_without_sdk_raises_cli_error(monkeypatch):
    """Opening a FeetechBus when scservo_sdk is absent must raise CliError(2).

    Simulates the SDK being absent (rather than relying on the test environment
    not having it) so the lazy-import-failure path is exercised whether or not
    the optional ``[seeed]`` extra happens to be installed.
    """
    import importlib

    import pytest

    from arm101.cli._errors import EXIT_ENV_ERROR, CliError
    from arm101.hardware.bus import FeetechBus

    real_import_module = importlib.import_module

    def _fake_import_module(name, *args, **kwargs):
        if name == "scservo_sdk":
            raise ModuleNotFoundError("No module named 'scservo_sdk'")
        return real_import_module(name, *args, **kwargs)

    monkeypatch.setattr(importlib, "import_module", _fake_import_module)

    bus = FeetechBus(port="/dev/ttyUSB0")
    with pytest.raises(CliError) as exc_info:
        bus.open()

    err = exc_info.value
    assert err.code == EXIT_ENV_ERROR
    # Remediation should mention pip install
    assert "pip install" in err.remediation.lower() or "pip install" in err.message.lower()


def test_feetech_bus_read_without_open_raises_cli_error():
    """Calling read_position without open() must raise CliError(2)."""
    from arm101.cli._errors import EXIT_ENV_ERROR, CliError
    from arm101.hardware.bus import FeetechBus

    bus = FeetechBus(port="/dev/ttyUSB0")
    import pytest

    with pytest.raises(CliError) as exc_info:
        bus.read_position(1)

    assert exc_info.value.code == EXIT_ENV_ERROR


# ---------------------------------------------------------------------------
# 3. FakeBus — in-memory round-trips
# ---------------------------------------------------------------------------


def test_fakebus_default_positions():
    """FakeBus with no arguments returns sane default positions."""
    from arm101.hardware.bus import FakeBus

    bus = FakeBus()
    bus.open()
    pos = bus.read_position(1)
    assert isinstance(pos, int)
    assert 0 <= pos <= 4095  # 12-bit encoder range


def test_fakebus_preset_positions():
    """FakeBus accepts a motor→position dict and returns those values."""
    from arm101.hardware.bus import FakeBus

    bus = FakeBus(positions={1: 100, 2: 2000, 6: 4095})
    bus.open()

    assert bus.read_position(1) == 100
    assert bus.read_position(2) == 2000
    assert bus.read_position(6) == 4095


def test_fakebus_default_for_unknown_motor():
    """FakeBus returns a sane default for motors not in the preset dict."""
    from arm101.hardware.bus import FakeBus

    bus = FakeBus(positions={1: 100})
    bus.open()
    pos = bus.read_position(3)
    assert isinstance(pos, int)
    assert 0 <= pos <= 4095


def test_fakebus_records_write_id_baudrate():
    """FakeBus records every write_id_baudrate call for inspection."""
    from arm101.hardware.bus import FakeBus

    bus = FakeBus()
    bus.open()

    bus.write_id_baudrate(motor=1, new_id=2, baudrate=1000000)
    bus.write_id_baudrate(motor=3, new_id=4, baudrate=115200)

    assert len(bus.eeprom_writes) == 2
    assert bus.eeprom_writes[0] == {"motor": 1, "new_id": 2, "baudrate": 1000000}
    assert bus.eeprom_writes[1] == {"motor": 3, "new_id": 4, "baudrate": 115200}


def test_fakebus_eeprom_writes_empty_on_init():
    """FakeBus starts with no recorded writes."""
    from arm101.hardware.bus import FakeBus

    bus = FakeBus()
    assert bus.eeprom_writes == []


def test_fakebus_context_manager():
    """FakeBus works as a context manager (with statement)."""
    from arm101.hardware.bus import FakeBus

    with FakeBus(positions={1: 512}) as bus:
        assert bus.read_position(1) == 512


def test_fakebus_close():
    """FakeBus close() does not raise."""
    from arm101.hardware.bus import FakeBus

    bus = FakeBus()
    bus.open()
    bus.close()  # should not raise


# ---------------------------------------------------------------------------
# 4. Interface contract — both classes expose the same surface
# ---------------------------------------------------------------------------


def test_motor_bus_interface_on_fakebus():
    """FakeBus exposes the full MotorBus interface including new motion primitives."""
    from arm101.hardware.bus import FakeBus, MotorBus

    bus = FakeBus()
    assert isinstance(bus, MotorBus)
    assert hasattr(bus, "read_position")
    assert hasattr(bus, "write_id_baudrate")
    assert hasattr(bus, "enable_torque")
    assert hasattr(bus, "write_goal_position")
    assert hasattr(bus, "read_lock")
    assert hasattr(bus, "open")
    assert hasattr(bus, "close")


def test_motor_bus_interface_on_feetech_bus():
    """FeetechBus exposes the full MotorBus interface including new motion primitives."""
    from arm101.hardware.bus import FeetechBus, MotorBus

    bus = FeetechBus(port="/dev/ttyUSB0")
    assert isinstance(bus, MotorBus)
    assert hasattr(bus, "read_position")
    assert hasattr(bus, "write_id_baudrate")
    assert hasattr(bus, "enable_torque")
    assert hasattr(bus, "write_goal_position")
    assert hasattr(bus, "read_lock")
    assert hasattr(bus, "open")
    assert hasattr(bus, "close")


def test_feetech_info_registers_includes_lock_register():
    """read_info() must read the EEPROM Lock register (addr 55) on real hardware.

    build_plan() surfaces motor_snapshot.lock_register from read_info(); if addr 55
    is not in _INFO_REGISTERS the field silently defaults to 0 on real hardware
    (only FakeBus injects it), so the plan would misreport the lock state.
    """
    from arm101.hardware.bus import FeetechBus

    assert FeetechBus._INFO_REGISTERS.get("lock_register") == (55, 1)


def test_fakebus_records_enable_torque():
    """FakeBus records every enable_torque call in torque_writes."""
    from arm101.hardware.bus import FakeBus

    bus = FakeBus()
    bus.open()

    bus.enable_torque(motor=1, on=True)
    bus.enable_torque(motor=1, on=False)

    assert len(bus.torque_writes) == 2
    assert bus.torque_writes[0] == {"motor": 1, "on": True}
    assert bus.torque_writes[1] == {"motor": 1, "on": False}


def test_fakebus_torque_writes_empty_on_init():
    """FakeBus starts with no recorded torque writes."""
    from arm101.hardware.bus import FakeBus

    bus = FakeBus()
    assert bus.torque_writes == []


def test_fakebus_enable_torque_not_open_raises():
    """enable_torque raises CliError(EXIT_ENV_ERROR) when bus is not open."""
    import pytest

    from arm101.cli._errors import EXIT_ENV_ERROR, CliError
    from arm101.hardware.bus import FakeBus

    bus = FakeBus()
    with pytest.raises(CliError) as exc:
        bus.enable_torque(motor=1, on=True)
    assert exc.value.code == EXIT_ENV_ERROR


def test_fakebus_records_write_goal_position():
    """FakeBus records every write_goal_position call in position_writes."""
    from arm101.hardware.bus import FakeBus

    bus = FakeBus()
    bus.open()

    bus.write_goal_position(motor=1, position=2048)
    bus.write_goal_position(motor=2, position=0)

    assert len(bus.position_writes) == 2
    assert bus.position_writes[0] == {"motor": 1, "position": 2048}
    assert bus.position_writes[1] == {"motor": 2, "position": 0}


def test_fakebus_position_writes_empty_on_init():
    """FakeBus starts with no recorded position writes."""
    from arm101.hardware.bus import FakeBus

    bus = FakeBus()
    assert bus.position_writes == []


def test_fakebus_write_goal_position_not_open_raises():
    """write_goal_position raises CliError(EXIT_ENV_ERROR) when bus is not open."""
    import pytest

    from arm101.cli._errors import EXIT_ENV_ERROR, CliError
    from arm101.hardware.bus import FakeBus

    bus = FakeBus()
    with pytest.raises(CliError) as exc:
        bus.write_goal_position(motor=1, position=2048)
    assert exc.value.code == EXIT_ENV_ERROR


def test_fakebus_write_goal_position_out_of_range_raises():
    """write_goal_position raises CliError(EXIT_USER_ERROR) for out-of-range position."""
    import pytest

    from arm101.cli._errors import EXIT_USER_ERROR, CliError
    from arm101.hardware.bus import FakeBus

    bus = FakeBus()
    bus.open()

    with pytest.raises(CliError) as exc:
        bus.write_goal_position(motor=1, position=9999)
    assert exc.value.code == EXIT_USER_ERROR

    with pytest.raises(CliError) as exc:
        bus.write_goal_position(motor=1, position=-1)
    assert exc.value.code == EXIT_USER_ERROR


def test_fakebus_write_goal_position_boundary_values():
    """write_goal_position accepts boundary values 0 and 4095."""
    from arm101.hardware.bus import FakeBus

    bus = FakeBus()
    bus.open()

    bus.write_goal_position(motor=1, position=0)
    bus.write_goal_position(motor=1, position=4095)

    assert bus.position_writes[0]["position"] == 0
    assert bus.position_writes[1]["position"] == 4095


def test_fakebus_torque_and_position_ordering():
    """Records in torque_writes and position_writes preserve call order.

    Tests for center-motor can assert: torque-on appears before position write,
    and torque-off (relax) appears after position write.
    """
    from arm101.hardware.bus import FakeBus

    bus = FakeBus()
    bus.open()

    bus.enable_torque(motor=1, on=True)
    bus.write_goal_position(motor=1, position=2048)
    bus.enable_torque(motor=1, on=False)

    assert bus.torque_writes[0]["on"] is True
    assert bus.torque_writes[1]["on"] is False
    assert bus.position_writes[0]["position"] == 2048
    # Ordering cross-check: first torque write (enable) then position, then relax.
    # The lists record independently; we rely on the above sequence being the only
    # valid path through center_motor's code.
    assert len(bus.torque_writes) == 2
    assert len(bus.position_writes) == 1


# ---------------------------------------------------------------------------
# 6. FeetechBus.write_id_baudrate write order (Qodo #2)
# ---------------------------------------------------------------------------


def test_write_id_baudrate_writes_baud_before_id():
    """Baud register (addr 6) is written BEFORE the id register (addr 5).

    Writing the id first changes the device address mid-call, so a subsequent
    baud write aimed at the old id would hit an unreachable device. Both writes
    must therefore target the current id, with the id change happening last.
    """
    from arm101.hardware.bus import FeetechBus

    class _RecordingPacket:
        def __init__(self):
            self.writes = []

        def write1ByteTxRx(self, port, motor, addr, val):
            self.writes.append((motor, addr, val))
            return 0, 0  # result, error → success

    bus = FeetechBus(port="/dev/ttyUSB0")
    rec = _RecordingPacket()
    bus._packet_handler = rec
    bus._port_handler = object()
    bus._open = True

    bus.write_id_baudrate(motor=1, new_id=2, baudrate=1_000_000)

    # EEPROM data writes: baud (6) BEFORE id (5), both addressed to the current
    # id (1) so the id change is the last op on the old address.
    data_writes = [(m, a, v) for (m, a, v) in rec.writes if a in (5, 6)]
    addrs = [a for _m, a, _v in data_writes]
    assert addrs == [6, 5]  # baud (6) first, id (5) last
    assert all(m == 1 for m, _a, _v in data_writes)
    id_write = next(w for w in data_writes if w[1] == 5)
    assert id_write[2] == 2  # the id register write carried the new id

    # The EEPROM Lock register (addr 55) is opened BEFORE the writes and restored
    # AFTER — without this the id write does not persist across a power-cycle.
    # The relock is addressed to the NEW id because the id write moved the
    # device address.
    assert rec.writes[0] == (1, 55, 0)  # unlock at current id, first
    assert rec.writes[-1] == (2, 55, 1)  # relock at new id, last


# ---------------------------------------------------------------------------
# 7. FeetechBus.read_lock — Lock register (address 55)
# ---------------------------------------------------------------------------


def test_feetech_bus_read_lock_reads_address_55():
    """read_lock reads the Lock register at address 55 (1 byte)."""
    from arm101.hardware.bus import FeetechBus

    class _RecordingPacket:
        def __init__(self):
            self.reads = []

        def read1ByteTxRx(self, port, motor, addr):
            self.reads.append((motor, addr))
            return 1, 0, 0  # value=1 (locked), result=0, error=0

    bus = FeetechBus(port="/dev/ttyUSB0")
    rec = _RecordingPacket()
    bus._packet_handler = rec
    bus._port_handler = object()
    bus._open = True

    value = bus.read_lock(motor=1)

    assert value == 1
    assert rec.reads == [(1, 55)]


# ---------------------------------------------------------------------------
# 8. FakeBus — lock_register support
# ---------------------------------------------------------------------------


def test_fakebus_read_info_includes_lock_register_default():
    """FakeBus.read_info() includes 'lock_register' key, defaulting to 0."""
    from arm101.hardware.bus import FakeBus

    bus = FakeBus()
    bus.open()
    info = bus.read_info(1)
    assert "lock_register" in info
    assert info["lock_register"] == 0


def test_fakebus_read_info_lock_register_non_default():
    """FakeBus.read_info() reflects a non-default lock_register value."""
    from arm101.hardware.bus import FakeBus

    bus = FakeBus(lock_register=1)
    bus.open()
    info = bus.read_info(1)
    assert info["lock_register"] == 1


def test_fakebus_read_lock_returns_configured_value():
    """FakeBus.read_lock() returns the configured lock_register value."""
    from arm101.hardware.bus import FakeBus

    bus = FakeBus(lock_register=1)
    bus.open()
    assert bus.read_lock(1) == 1

    bus2 = FakeBus()
    bus2.open()
    assert bus2.read_lock(1) == 0


# ---------------------------------------------------------------------------
# 9. write_baudrate — baud-only EEPROM write (no ID change)
# ---------------------------------------------------------------------------


def test_fakebus_records_write_baudrate():
    """FakeBus records every write_baudrate call in baud_writes."""
    from arm101.hardware.bus import FakeBus

    bus = FakeBus()
    bus.open()

    bus.write_baudrate(motor=1, baudrate=1_000_000)
    bus.write_baudrate(motor=3, baudrate=500_000)

    assert len(bus.baud_writes) == 2
    assert bus.baud_writes[0] == {"motor": 1, "baudrate": 1_000_000}
    assert bus.baud_writes[1] == {"motor": 3, "baudrate": 500_000}


def test_fakebus_baud_writes_empty_on_init():
    """FakeBus starts with no recorded baud writes."""
    from arm101.hardware.bus import FakeBus

    bus = FakeBus()
    assert bus.baud_writes == []


def test_fakebus_write_baudrate_not_open_raises():
    """write_baudrate raises CliError(EXIT_ENV_ERROR) when bus is not open."""
    import pytest

    from arm101.cli._errors import EXIT_ENV_ERROR, CliError
    from arm101.hardware.bus import FakeBus

    bus = FakeBus()
    with pytest.raises(CliError) as exc:
        bus.write_baudrate(motor=1, baudrate=1_000_000)
    assert exc.value.code == EXIT_ENV_ERROR


def test_fakebus_write_baudrate_bad_baud_raises_cli_error():
    """FakeBus.write_baudrate rejects an unsupported baud rate (mirrors FeetechBus)."""
    import pytest

    from arm101.cli._errors import EXIT_ENV_ERROR, CliError
    from arm101.hardware.bus import FakeBus

    bus = FakeBus()
    bus.open()
    with pytest.raises(CliError) as exc:
        bus.write_baudrate(motor=1, baudrate=999_999)  # not in BAUD_MAP
    assert exc.value.code == EXIT_ENV_ERROR
    assert "999999" in exc.value.message
    # The invalid call must not be recorded.
    assert bus.baud_writes == []


def test_feetech_write_baudrate_bad_baud_raises_cli_error():
    """FeetechBus.write_baudrate raises CliError(EXIT_ENV_ERROR) for an unsupported baud rate."""
    import pytest

    from arm101.cli._errors import EXIT_ENV_ERROR, CliError
    from arm101.hardware.bus import FeetechBus

    bus = FeetechBus(port="/dev/ttyUSB0")
    # Inject a fake open state; the bad-baud check happens before the SDK call.
    bus._packet_handler = object()
    bus._port_handler = object()
    bus._open = True

    with pytest.raises(CliError) as exc:
        bus.write_baudrate(motor=1, baudrate=999_999)
    assert exc.value.code == EXIT_ENV_ERROR
    assert "999999" in exc.value.message


def test_feetech_write_baudrate_writes_only_baud_register():
    """FeetechBus.write_baudrate writes only register addr 6 (baud), not addr 5 (id)."""
    from arm101.hardware.bus import FeetechBus

    class _RecordingPacket:
        def __init__(self):
            self.writes = []

        def write1ByteTxRx(self, port, motor, addr, val):
            self.writes.append((motor, addr, val))
            return 0, 0  # result, error → success

    class _StubPort:
        """Minimal port-handler stub: write_baudrate now calls setBaudRate()
        when the target baud differs from the bus's current one (Bug A fix),
        so a bare ``object()`` no longer suffices here."""

        def setBaudRate(self, baudrate):
            return True

    bus = FeetechBus(port="/dev/ttyUSB0")
    rec = _RecordingPacket()
    bus._packet_handler = rec
    bus._port_handler = _StubPort()
    bus._open = True

    bus.write_baudrate(motor=2, baudrate=500_000)

    # Unlock (addr 55 = 0) → baud (addr 6) → relock (addr 55 = 1); all at the
    # same id (the id register, addr 5, is never written). The unlock/relock
    # make the baud change persist across a power-cycle.
    assert rec.writes == [
        (2, 55, 0),  # unlock EEPROM
        (2, 6, 1),  # Baud_Rate register (BAUD_MAP[500_000] == 1)
        (2, 55, 1),  # re-lock EEPROM
    ]
    # The id register (addr 5) is never touched.
    assert all(addr != 5 for _motor, addr, _val in rec.writes)


# ---------------------------------------------------------------------------
# 10. write_baudrate / write_id_baudrate — Lock-register exception safety
#     and host-baud-switch-before-relock (Qodo findings 2 & 3)
# ---------------------------------------------------------------------------


def test_feetech_write_baudrate_switches_host_baud_before_relock():
    """A baud change to a DIFFERENT rate switches the host port BEFORE relocking.

    The motor applies a differing baud immediately; if the relock were sent
    at the stale host baud it would never reach the now-retuned motor (Qodo
    finding 2 — "relock uses old baud"). Asserts the call order is: unlock ->
    write baud -> host setBaudRate -> relock, and that ``self._baudrate`` is
    updated to match.
    """
    from arm101.hardware.bus import FeetechBus

    events = []

    class _RecordingPacket:
        def write1ByteTxRx(self, port, motor, addr, val):
            events.append(("write", motor, addr, val))
            return 0, 0  # result, error → success

    class _RecordingPort:
        def setBaudRate(self, baudrate):
            events.append(("setBaudRate", baudrate))
            return True

    bus = FeetechBus(port="/dev/ttyUSB0", baudrate=1_000_000)
    bus._packet_handler = _RecordingPacket()
    bus._port_handler = _RecordingPort()
    bus._open = True

    bus.write_baudrate(motor=2, baudrate=500_000)

    assert events == [
        ("write", 2, 55, 0),  # unlock
        ("write", 2, 6, 1),  # Baud_Rate register (BAUD_MAP[500_000] == 1)
        ("setBaudRate", 500_000),  # host port switches to match the motor
        ("write", 2, 55, 1),  # relock — sent at the NEW host baud, last
    ]
    assert bus._baudrate == 500_000


def test_feetech_write_baudrate_failed_write_still_relocks_without_host_switch():
    """A failing baud write (addr 6) still gets a best-effort relock at *motor*,
    and the host baud is never switched — the motor's baud never actually
    changed, so there is nothing to follow (Qodo finding 3 — "EEPROM unlock
    not rolled back").
    """
    import pytest

    from arm101.cli._errors import CliError
    from arm101.hardware.bus import FeetechBus

    class _FailingPacket:
        def __init__(self):
            self.writes = []

        def write1ByteTxRx(self, port, motor, addr, val):
            self.writes.append((motor, addr, val))
            if addr == 6:
                return 1, 0  # simulate a comm failure on the baud write
            return 0, 0

    class _RecordingPort:
        def __init__(self):
            self.baud_calls = []

        def setBaudRate(self, baudrate):
            self.baud_calls.append(baudrate)
            return True

    bus = FeetechBus(port="/dev/ttyUSB0", baudrate=1_000_000)
    rec = _FailingPacket()
    port = _RecordingPort()
    bus._packet_handler = rec
    bus._port_handler = port
    bus._open = True

    with pytest.raises(CliError):
        bus.write_baudrate(motor=2, baudrate=500_000)

    assert rec.writes == [
        (2, 55, 0),  # unlock
        (2, 6, 1),  # failed baud write (BAUD_MAP[500_000] == 1)
        (2, 55, 1),  # best-effort relock at the unchanged motor id
    ]
    assert port.baud_calls == []  # host never switched
    assert bus._baudrate == 1_000_000  # unchanged


def test_write_id_baudrate_baud_write_failure_relocks_at_original_motor():
    """If the baud write (addr 6) fails, the ID write never ran, so the
    best-effort relock targets the ORIGINAL ``motor`` id — never ``new_id``.
    """
    import pytest

    from arm101.cli._errors import CliError
    from arm101.hardware.bus import FeetechBus

    class _FailingPacket:
        def __init__(self):
            self.writes = []

        def write1ByteTxRx(self, port, motor, addr, val):
            self.writes.append((motor, addr, val))
            if addr == 6:
                return 1, 0  # simulate a comm failure on the baud write
            return 0, 0

    bus = FeetechBus(port="/dev/ttyUSB0")
    rec = _FailingPacket()
    bus._packet_handler = rec
    bus._port_handler = object()
    bus._open = True

    with pytest.raises(CliError):
        bus.write_id_baudrate(motor=1, new_id=2, baudrate=1_000_000)

    assert rec.writes == [
        (1, 55, 0),  # unlock at the current id
        (1, 6, 0),  # failed baud write (BAUD_MAP[1_000_000] == 0)
        (1, 55, 1),  # best-effort relock at the ORIGINAL id — the id write never ran
    ]


def test_write_id_baudrate_id_write_failure_relocks_at_original_motor():
    """If the ID write (addr 5) fails after a successful baud write, the
    device address never moved, so the best-effort relock still targets the
    ORIGINAL ``motor`` id — never ``new_id``.
    """
    import pytest

    from arm101.cli._errors import CliError
    from arm101.hardware.bus import FeetechBus

    class _FailingPacket:
        def __init__(self):
            self.writes = []

        def write1ByteTxRx(self, port, motor, addr, val):
            self.writes.append((motor, addr, val))
            if addr == 5:
                return 1, 0  # simulate a comm failure on the id write
            return 0, 0

    bus = FeetechBus(port="/dev/ttyUSB0")
    rec = _FailingPacket()
    bus._packet_handler = rec
    bus._port_handler = object()
    bus._open = True

    with pytest.raises(CliError):
        bus.write_id_baudrate(motor=1, new_id=2, baudrate=1_000_000)

    assert rec.writes == [
        (1, 55, 0),  # unlock at the current id
        (1, 6, 0),  # baud write succeeds (BAUD_MAP[1_000_000] == 0)
        (1, 5, 2),  # failed id write (attempted new id 2)
        (1, 55, 1),  # best-effort relock at the ORIGINAL id — the id write never committed
    ]
