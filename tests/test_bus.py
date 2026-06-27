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
    """FakeBus exposes the MotorBus interface (read_position, write_id_baudrate, open, close)."""
    from arm101.hardware.bus import FakeBus, MotorBus

    bus = FakeBus()
    assert isinstance(bus, MotorBus)
    assert hasattr(bus, "read_position")
    assert hasattr(bus, "write_id_baudrate")
    assert hasattr(bus, "open")
    assert hasattr(bus, "close")


def test_motor_bus_interface_on_feetech_bus():
    """FeetechBus exposes the MotorBus interface."""
    from arm101.hardware.bus import FeetechBus, MotorBus

    bus = FeetechBus(port="/dev/ttyUSB0")
    assert isinstance(bus, MotorBus)
    assert hasattr(bus, "read_position")
    assert hasattr(bus, "write_id_baudrate")
    assert hasattr(bus, "open")
    assert hasattr(bus, "close")
