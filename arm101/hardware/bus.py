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
_SDK_INSTALL_HINT = "pip install 'arm101[hardware]'  # installs the Feetech scservo_sdk extra"


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

        if not port_handler.openPort():
            raise CliError(
                code=EXIT_ENV_ERROR,
                message=f"Failed to open serial port {self._port!r}.",
                remediation=(
                    "Check that the device is connected and the port path is correct. "
                    "You may need to add your user to the 'dialout' group: "
                    "sudo usermod -aG dialout $USER"
                ),
            )

        if not port_handler.setBaudRate(self._baudrate):
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
        if result != 0 or error != 0:  # COMM_SUCCESS = 0
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

    def __init__(self, positions: dict[int, int] | None = None) -> None:
        self._positions: dict[int, int] = dict(positions) if positions else {}
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
