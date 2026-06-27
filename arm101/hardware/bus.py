"""MotorBus interface — Feetech STS3215 adapter with lazy SDK import + in-memory FakeBus.

Zero third-party imports at module load time.  The real Feetech SDK
(``scservo_sdk``) is lazy-imported only inside :meth:`FeetechBus.open`; if the
package is absent a :class:`~arm101.cli._errors.CliError` with
``code == EXIT_ENV_ERROR`` is raised with a ``pip install`` remediation hint.

FakeBus implements the same interface entirely in-memory and records every
``write_id_baudrate`` call in :attr:`FakeBus.eeprom_writes` so downstream verbs
(calibrate, setup-motors) can drive hardware interactions without physical
hardware.
"""

from __future__ import annotations

import abc
from types import TracebackType
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    pass  # No runtime imports needed for type hints here

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

#: Default encoder tick for motors not specified in a FakeBus positions dict.
_DEFAULT_POSITION: int = 2048  # mid-range of 12-bit (0–4095) encoder

#: Feetech SDK top-level module name.
_SDK_MODULE = "scservo_sdk"

#: Install hint shown in CliError remediation when the SDK is absent.
_SDK_INSTALL_HINT = "pip install 'arm101[seeed]'  # installs the Feetech scservo_sdk SDK"


# ---------------------------------------------------------------------------
# Abstract interface
# ---------------------------------------------------------------------------


class MotorBus(abc.ABC):
    """Abstract interface for a Feetech STS3215 motor bus.

    Both :class:`FeetechBus` (real serial bus) and :class:`FakeBus`
    (in-memory stub) implement this interface so all downstream verbs can be
    written against ``MotorBus`` and tested with ``FakeBus`` without hardware.

    Lifecycle::

        bus = FeetechBus("/dev/ttyUSB0")  # or FakeBus()
        bus.open()
        try:
            pos = bus.read_position(1)
        finally:
            bus.close()

    Or use as a context manager::

        with FakeBus() as bus:
            pos = bus.read_position(1)
    """

    @abc.abstractmethod
    def open(self) -> None:
        """Open the bus (acquire hardware / initialise SDK).

        Raises
        ------
        CliError(EXIT_ENV_ERROR)
            If the Feetech SDK is absent (real bus) or any environment
            prerequisite is missing.
        """

    @abc.abstractmethod
    def close(self) -> None:
        """Release the bus / hardware resources."""

    @abc.abstractmethod
    def read_position(self, motor: int) -> int:
        """Return the raw encoder position for *motor* (0–4095 ticks).

        Parameters
        ----------
        motor:
            Motor ID (1-indexed, matching the Feetech servo ID).

        Returns
        -------
        int
            Raw 12-bit encoder tick in ``[0, 4095]``.

        Raises
        ------
        CliError(EXIT_ENV_ERROR)
            If the bus has not been opened or a read error occurs.
        """

    @abc.abstractmethod
    def write_id_baudrate(self, motor: int, new_id: int, baudrate: int) -> None:
        """Write the servo ID and baudrate to the motor's EEPROM.

        Parameters
        ----------
        motor:
            Current motor ID used to address the servo on the bus.
        new_id:
            New servo ID to programme into EEPROM.
        baudrate:
            Baud rate to programme into EEPROM (e.g. 1 000 000).

        Raises
        ------
        CliError(EXIT_ENV_ERROR)
            If the bus has not been opened or the write fails.
        """

    @abc.abstractmethod
    def scan(self, ids: "list[int] | None" = None) -> "list[int]":
        """Ping *ids* and return the sorted list of motor IDs that respond.

        Parameters
        ----------
        ids:
            Candidate IDs to ping.  Defaults to ``1..12`` (an SO-101 follower
            plus leader) when ``None``.

        Returns
        -------
        list[int]
            Sorted IDs that answered a ping.  Empty if nothing responded.

        Raises
        ------
        CliError(EXIT_ENV_ERROR)
            If the bus has not been opened.
        """

    @abc.abstractmethod
    def read_info(self, motor: int) -> "dict[str, int]":
        """Return a full read-only register snapshot for *motor*.

        Keys (all raw register values): ``id``, ``model``, ``firmware_major``,
        ``firmware_minor``, ``baud_index``, ``min_angle``, ``max_angle``,
        ``torque_enable``, ``present_position``, ``present_speed``,
        ``present_load``, ``present_voltage`` (units of 0.1 V),
        ``present_temperature`` (deg C).

        This performs only reads — no torque, motion, or EEPROM writes.

        Raises
        ------
        CliError(EXIT_ENV_ERROR)
            If the bus has not been opened or a read fails.
        """

    # ------------------------------------------------------------------
    # Context-manager helpers (shared implementation)
    # ------------------------------------------------------------------

    def __enter__(self) -> "MotorBus":
        self.open()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: TracebackType | None,
    ) -> None:
        self.close()


# ---------------------------------------------------------------------------
# Real implementation — lazy-imports scservo_sdk on open()
# ---------------------------------------------------------------------------


class FeetechBus(MotorBus):
    """Real Feetech STS3215 bus over a serial port.

    The ``scservo_sdk`` package is imported lazily inside :meth:`open` so that
    ``import arm101.hardware.bus`` succeeds in environments that lack the SDK.
    Absence of the SDK is surfaced as a :class:`~arm101.cli._errors.CliError`
    with exit code ``EXIT_ENV_ERROR`` and a ``pip install`` remediation hint —
    never as an ``ImportError`` traceback.

    Parameters
    ----------
    port:
        Serial device path (e.g. ``"/dev/ttyUSB0"``).
    baudrate:
        Communication baud rate.  Defaults to ``1_000_000`` (STS3215 default).
    """

    def __init__(self, port: str, baudrate: int = 1_000_000) -> None:
        self._port = port
        self._baudrate = baudrate
        self._port_handler: object = None
        self._packet_handler: object = None
        self._open = False

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _require_open(self) -> None:
        """Raise CliError if the bus is not open."""
        from arm101.cli._errors import EXIT_ENV_ERROR, CliError

        if not self._open:
            raise CliError(
                code=EXIT_ENV_ERROR,
                message="Bus is not open; call open() before reading or writing.",
                remediation="Call FeetechBus.open() first, or use the bus as a context manager.",
            )

    def _import_sdk(self) -> object:
        """Lazy-import scservo_sdk; raise CliError if absent."""
        from arm101.cli._errors import EXIT_ENV_ERROR, CliError

        try:
            import importlib

            sdk = importlib.import_module(_SDK_MODULE)
        except ModuleNotFoundError:
            raise CliError(
                code=EXIT_ENV_ERROR,
                message=(
                    f"The Feetech SDK ({_SDK_MODULE!r}) is not installed. "
                    "Physical motor communication is unavailable."
                ),
                remediation=_SDK_INSTALL_HINT,
            ) from None
        return sdk

    # ------------------------------------------------------------------
    # MotorBus interface
    # ------------------------------------------------------------------

    def open(self) -> None:
        """Open the serial port and initialise the SDK.

        Raises
        ------
        CliError(EXIT_ENV_ERROR)
            If ``scservo_sdk`` is not installed.
        CliError(EXIT_ENV_ERROR)
            If the port cannot be opened.
        """
        from arm101.cli._errors import EXIT_ENV_ERROR, CliError

        sdk = self._import_sdk()  # raises CliError if absent

        port_handler = sdk.PortHandler(self._port)  # type: ignore[attr-defined]
        packet_handler = sdk.PacketHandler(0)  # type: ignore[attr-defined]

        # The SDK's PortHandler may *raise* (pyserial SerialException — port
        # missing, busy, or permission denied) rather than returning False, so
        # both signals are funnelled into a single CliError. Callers that probe
        # several ports (e.g. calibrate-motor) rely on this being a CliError to
        # skip a busy device cleanly instead of crashing on a raw traceback.
        try:
            opened = port_handler.openPort()
        except Exception as exc:  # noqa: BLE001 - SDK raises pyserial errors
            opened = False
            open_detail: str | None = str(exc)
        else:
            open_detail = None
        if not opened:
            suffix = f" ({open_detail})" if open_detail else ""
            raise CliError(
                code=EXIT_ENV_ERROR,
                message=f"Failed to open serial port {self._port!r}{suffix}.",
                remediation=(
                    "Check that the device is connected, the port path is correct, and "
                    "it is not already in use by another process. You may also need to "
                    "add your user to the 'dialout' group: sudo usermod -aG dialout $USER"
                ),
            )

        try:
            baud_ok = port_handler.setBaudRate(self._baudrate)
        except Exception as exc:  # noqa: BLE001 - SDK raises pyserial errors
            port_handler.closePort()
            raise CliError(
                code=EXIT_ENV_ERROR,
                message=f"Failed to set baud rate {self._baudrate} on {self._port!r} ({exc}).",
                remediation="Verify the baud rate matches your servo configuration.",
            ) from exc
        if not baud_ok:
            port_handler.closePort()
            raise CliError(
                code=EXIT_ENV_ERROR,
                message=f"Failed to set baud rate {self._baudrate} on {self._port!r}.",
                remediation="Verify the baud rate matches your servo configuration.",
            )

        self._port_handler = port_handler
        self._packet_handler = packet_handler
        self._open = True

    def close(self) -> None:
        """Close the serial port if open."""
        if self._open and self._port_handler is not None:
            self._port_handler.closePort()  # type: ignore[attr-defined]
        self._open = False
        self._port_handler = None
        self._packet_handler = None

    def read_position(self, motor: int) -> int:
        """Read the present-position register (address 56, 2 bytes) for *motor*.

        Returns
        -------
        int
            Raw 12-bit encoder tick in ``[0, 4095]``.
        """
        from arm101.cli._errors import EXIT_ENV_ERROR, CliError

        self._require_open()

        # STS3215 present-position address = 56, 2 bytes.
        _ADDR_PRESENT_POSITION = 56

        value, result, error = self._packet_handler.read2ByteTxRx(  # type: ignore[union-attr]
            self._port_handler, motor, _ADDR_PRESENT_POSITION
        )
        if result != 0 or error != 0:  # any non-zero comm result or servo error means failure
            raise CliError(
                code=EXIT_ENV_ERROR,
                message=f"Read position failed for motor {motor}: result={result}, error={error}.",
                remediation="Check wiring, power, and that the motor ID is correct.",
            )
        return int(value) & 0x0FFF  # mask to 12 bits

    def write_id_baudrate(self, motor: int, new_id: int, baudrate: int) -> None:
        """Write servo ID and baud-rate to the motor's EEPROM.

        STS3215 EEPROM registers: ID = address 5 (1 byte), Baud_Rate = address 6 (1 byte).
        The baud-rate register uses a Feetech-specific index (not the raw bps value).
        """
        from arm101.cli._errors import EXIT_ENV_ERROR, CliError

        self._require_open()

        # Baud-rate index mapping (Feetech STS3215 datasheet).
        _BAUD_MAP: dict[int, int] = {
            1_000_000: 0,
            500_000: 1,
            250_000: 2,
            128_000: 3,
            115_200: 4,
            76_800: 5,
            57_600: 6,
            38_400: 7,
        }
        baud_index = _BAUD_MAP.get(baudrate)
        if baud_index is None:
            raise CliError(
                code=EXIT_ENV_ERROR,
                message=f"Unsupported baud rate {baudrate}. Supported: {sorted(_BAUD_MAP)}.",
                remediation="Choose a baud rate from the supported list.",
            )

        _ADDR_ID = 5
        _ADDR_BAUD = 6

        for addr, val, label in (
            (_ADDR_ID, new_id, "ID"),
            (_ADDR_BAUD, baud_index, "Baud_Rate"),
        ):
            result, error = self._packet_handler.write1ByteTxRx(  # type: ignore[union-attr]
                self._port_handler, motor, addr, val
            )
            if result != 0 or error != 0:
                raise CliError(
                    code=EXIT_ENV_ERROR,
                    message=(
                        f"Write {label} failed for motor {motor}: "
                        f"result={result}, error={error}."
                    ),
                    remediation="Check wiring, power, and that the motor ID is correct.",
                )

    # ------------------------------------------------------------------
    # Read-only introspection (no torque / motion / EEPROM writes)
    # ------------------------------------------------------------------

    #: STS3215 read-only register map: name -> (address, byte-length).
    _INFO_REGISTERS: "dict[str, tuple[int, int]]" = {
        "firmware_major": (0, 1),
        "firmware_minor": (1, 1),
        "model": (3, 2),
        "id": (5, 1),
        "baud_index": (6, 1),
        "min_angle": (9, 2),
        "max_angle": (11, 2),
        "torque_enable": (40, 1),
        "present_position": (56, 2),
        "present_speed": (58, 2),
        "present_load": (60, 2),
        "present_voltage": (62, 1),
        "present_temperature": (63, 1),
    }

    def _read_register(self, motor: int, addr: int, length: int) -> int:
        """Read a 1- or 2-byte register; raise CliError on a comms failure."""
        from arm101.cli._errors import EXIT_ENV_ERROR, CliError

        if length == 1:
            value, result, error = self._packet_handler.read1ByteTxRx(  # type: ignore[union-attr]
                self._port_handler, motor, addr
            )
        else:
            value, result, error = self._packet_handler.read2ByteTxRx(  # type: ignore[union-attr]
                self._port_handler, motor, addr
            )
        if result != 0 or error != 0:
            raise CliError(
                code=EXIT_ENV_ERROR,
                message=(
                    f"Read of register {addr} failed for motor {motor}: "
                    f"result={result}, error={error}."
                ),
                remediation="Check wiring, power, and that the motor ID is correct.",
            )
        return int(value)

    def scan(self, ids: "list[int] | None" = None) -> "list[int]":
        """Ping candidate *ids* and return those that respond (read-only)."""
        self._require_open()
        candidates = list(ids) if ids is not None else list(range(1, 13))
        found: list[int] = []
        for sid in candidates:
            _model, result, _error = self._packet_handler.ping(  # type: ignore[union-attr]
                self._port_handler, sid
            )
            if result == 0:  # COMM_SUCCESS
                found.append(sid)
        return sorted(found)

    def read_info(self, motor: int) -> "dict[str, int]":
        """Return a full read-only register snapshot for *motor*."""
        self._require_open()
        return {
            name: self._read_register(motor, addr, length)
            for name, (addr, length) in self._INFO_REGISTERS.items()
        }


# ---------------------------------------------------------------------------
# In-memory FakeBus — for tests and offline development
# ---------------------------------------------------------------------------


class FakeBus(MotorBus):
    """In-memory motor bus for tests and offline development.

    No hardware or third-party packages required.

    Parameters
    ----------
    positions:
        Optional mapping of motor-id → initial encoder position.  Motors not
        present in the dict return :data:`_DEFAULT_POSITION` (2048).

    Attributes
    ----------
    eeprom_writes:
        List of dicts, one per :meth:`write_id_baudrate` call::

            {"motor": int, "new_id": int, "baudrate": int}

        Downstream tests (setup-motors etc.) inspect this list to assert that
        writes happened at the expected time (e.g. only after operator
        confirmation).
    """

    def __init__(
        self,
        positions: dict[int, int] | None = None,
        ids: "list[int] | None" = None,
        info: "dict[int, dict[str, int]] | None" = None,
    ) -> None:
        self._positions: dict[int, int] = dict(positions) if positions else {}
        # Motor IDs the fake bus reports from scan(). Defaults to whatever the
        # positions dict mentions, else a single factory motor at id 1.
        if ids is not None:
            self._ids: list[int] = sorted(ids)
        elif self._positions:
            self._ids = sorted(self._positions)
        else:
            self._ids = [1]
        # Optional per-motor register overrides for read_info().
        self._info_overrides: dict[int, dict[str, int]] = dict(info) if info else {}
        self.eeprom_writes: list[dict[str, int]] = []
        self._open = False

    # ------------------------------------------------------------------
    # MotorBus interface
    # ------------------------------------------------------------------

    def open(self) -> None:
        """Mark the bus as open (no-op for the fake)."""
        self._open = True

    def close(self) -> None:
        """Mark the bus as closed (no-op for the fake)."""
        self._open = False

    def read_position(self, motor: int) -> int:
        """Return the preset position for *motor*, defaulting to 2048.

        Raises
        ------
        CliError(EXIT_ENV_ERROR)
            If the bus has not been opened.
        """
        from arm101.cli._errors import EXIT_ENV_ERROR, CliError

        if not self._open:
            raise CliError(
                code=EXIT_ENV_ERROR,
                message="FakeBus is not open; call open() first.",
                remediation="Call FakeBus.open() or use it as a context manager.",
            )
        return self._positions.get(motor, _DEFAULT_POSITION)

    def write_id_baudrate(self, motor: int, new_id: int, baudrate: int) -> None:
        """Record an EEPROM write in :attr:`eeprom_writes`.

        Raises
        ------
        CliError(EXIT_ENV_ERROR)
            If the bus has not been opened.
        """
        from arm101.cli._errors import EXIT_ENV_ERROR, CliError

        if not self._open:
            raise CliError(
                code=EXIT_ENV_ERROR,
                message="FakeBus is not open; call open() first.",
                remediation="Call FakeBus.open() or use it as a context manager.",
            )
        self.eeprom_writes.append({"motor": motor, "new_id": new_id, "baudrate": baudrate})

    def scan(self, ids: "list[int] | None" = None) -> "list[int]":
        """Return the configured present IDs (optionally filtered by *ids*)."""
        self._require_open_fake()
        if ids is None:
            return list(self._ids)
        wanted = set(ids)
        return [i for i in self._ids if i in wanted]

    def read_info(self, motor: int) -> "dict[str, int]":
        """Return a canned read-only register snapshot for *motor*.

        Defaults mimic a factory STS3215 (model 777, firmware 3.10, baud 1 Mbps,
        12.0 V, 38 deg C); ``present_position`` reflects the positions dict and
        anything in the ``info`` constructor override wins.
        """
        self._require_open_fake()
        snapshot: dict[str, int] = {
            "firmware_major": 3,
            "firmware_minor": 10,
            "model": 777,
            "id": motor,
            "baud_index": 0,
            "min_angle": 0,
            "max_angle": 4095,
            "torque_enable": 0,
            "present_position": self._positions.get(motor, _DEFAULT_POSITION),
            "present_speed": 0,
            "present_load": 0,
            "present_voltage": 120,
            "present_temperature": 38,
        }
        snapshot.update(self._info_overrides.get(motor, {}))
        return snapshot

    def _require_open_fake(self) -> None:
        from arm101.cli._errors import EXIT_ENV_ERROR, CliError

        if not self._open:
            raise CliError(
                code=EXIT_ENV_ERROR,
                message="FakeBus is not open; call open() first.",
                remediation="Call FakeBus.open() or use it as a context manager.",
            )
