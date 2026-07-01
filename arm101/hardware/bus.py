"""MotorBus interface — Feetech STS3215 adapter with lazy SDK import + in-memory FakeBus.

Zero third-party imports at module load time.  The real Feetech SDK
(``scservo_sdk``) is lazy-imported only inside :meth:`FeetechBus.open`; if the
package is absent a :class:`~arm101.cli._errors.CliError` with
``code == EXIT_ENV_ERROR`` is raised with a ``pip install`` remediation hint.

FakeBus implements the same interface entirely in-memory and records every
write call (``write_id_baudrate``, ``enable_torque``, ``write_goal_position``)
in the corresponding list attribute so downstream verbs (calibrate, center,
setup-motors) can drive hardware interactions without physical hardware.
"""

from __future__ import annotations

import abc
import contextlib
from types import TracebackType
from typing import TYPE_CHECKING

from arm101.cli._errors import EXIT_ENV_ERROR, CliError

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
#: The PyPI distribution is ``arm101-cli`` (the ``arm101`` import package / the
#: ``arm101`` console script are NOT the installable name), so the extra is
#: ``arm101-cli[seeed]``.
_SDK_INSTALL_HINT = "pip install 'arm101-cli[seeed]'  # installs the Feetech scservo_sdk SDK"

#: Shared "SDK missing" CliError message, so the module-level pre-flight
#: (:func:`require_sdk`) and the per-instance lazy import
#: (:meth:`FeetechBus._import_sdk`) surface the same wording.
_SDK_MISSING_MSG = (
    f"The Feetech SDK ({_SDK_MODULE!r}) is not installed. "
    "Physical motor communication is unavailable."
)


def sdk_available() -> bool:
    """Return ``True`` iff the optional Feetech SDK (``scservo_sdk``) is importable.

    Uses :func:`importlib.util.find_spec` so it never actually imports the SDK
    (no side effects) — a cheap pre-flight for real-hardware code paths that
    want to fail with a clear install error *before* opening a bus, rather than
    degrading a missing SDK into a misleading "no servo answered" diagnosis.
    """
    import importlib.util

    return importlib.util.find_spec(_SDK_MODULE) is not None


def require_sdk() -> None:
    """Raise ``CliError(EXIT_ENV_ERROR)`` with an install hint if the SDK is absent.

    The counterpart to :func:`sdk_available` for callers that want to hard-stop
    a hardware-dependent command up front (e.g. ``doctor --probe``).
    """
    from arm101.cli._errors import EXIT_ENV_ERROR, CliError

    if not sdk_available():
        raise CliError(
            code=EXIT_ENV_ERROR,
            message=_SDK_MISSING_MSG,
            remediation=_SDK_INSTALL_HINT,
        )


#: Repeated CliError strings, extracted so the same literal is not duplicated
#: across methods (SonarCloud python:S1192).
_REMEDIATION_CHECK_WIRING = "Check wiring, power, and that the motor ID is correct."
_REMEDIATION_CHOOSE_BAUD = "Choose a baud rate from the supported list."
_FAKEBUS_NOT_OPEN_MSG = "FakeBus is not open; call open() first."
_FAKEBUS_NOT_OPEN_REMEDIATION = "Call FakeBus.open() or use it as a context manager."

# ---------------------------------------------------------------------------
# Baud-rate mapping — hoisted to module scope so callers can validate/enumerate
# ---------------------------------------------------------------------------

#: Feetech STS3215 baud-rate index mapping (bps → EEPROM index value).
#: Hoisted to module scope so both :meth:`FeetechBus.write_id_baudrate` and
#: callers (e.g. ``setup-motors``) can validate or enumerate supported rates
#: without having to open a bus or duplicate the table.
BAUD_MAP: dict[int, int] = {
    1_000_000: 0,
    500_000: 1,
    250_000: 2,
    128_000: 3,
    115_200: 4,
    76_800: 5,
    57_600: 6,
    38_400: 7,
}

#: Reverse of :data:`BAUD_MAP` — EEPROM index → bps.  Use this to render a
#: motor's ``baud_index`` register value as a human-readable speed string.
BAUD_INDEX_TO_BPS: dict[int, int] = {v: k for k, v in BAUD_MAP.items()}


# ---------------------------------------------------------------------------
# Overload classification — STS3215 status error byte, bit 5 (0x20)
# ---------------------------------------------------------------------------

#: Status-error-byte bit that flags an overload (Feetech STS3215 datasheet:
#: bit 5 of the packet error byte = "Overload Error" — load exceeded the
#: servo's Torque_Limit). Hoisted to module scope so :func:`is_overload`,
#: :class:`OverloadError`, and :class:`FakeBus`'s overload-simulation seam
#: (which raises with exactly this value) all agree on one source of truth.
_OVERLOAD_BIT: int = 0x20  # bit 5 == 32


def is_overload(error_byte: int) -> bool:
    """Return ``True`` iff *error_byte* (an STS3215 status error byte) flags an overload.

    Bit 5 (``0x20`` / ``32``) of the Feetech STS3215 status/error byte is the
    Overload Error flag: the servo's load exceeded its Torque_Limit. This is
    the single source of truth for that bit — bus code checks it here rather
    than inlining ``error_byte & 0x20`` at each call site.

    Parameters
    ----------
    error_byte:
        The raw status error byte as returned by the Feetech SDK's
        ``read*TxRx`` / ``write*TxRx`` calls (the ``error`` return value, NOT
        the communication ``result`` code).

    Returns
    -------
    bool
        ``True`` if bit 5 is set — including when other bits are also set —
        ``False`` otherwise, including for ``error_byte == 0``.
    """
    return bool(error_byte & _OVERLOAD_BIT)


#: STS3215 Present_Load (register 60) encodes **direction in bit 10 (0x400 /
#: 1024)** and magnitude in bits 0-9 (0x3FF / 0-1023). A load in the "negative"
#: direction therefore reads as a raw value >= 1024, which would swamp any
#: sensible contact threshold if compared raw. Mask to the magnitude before any
#: threshold comparison. Single source of truth so every caller agrees.
_LOAD_MAGNITUDE_MASK: int = 0x3FF  # bits 0-9; bit 10 (0x400) is the direction sign


def load_magnitude(present_load: int) -> int:
    """Return the direction-independent magnitude of an STS3215 ``present_load``.

    The Present_Load register (address 60) carries the load *direction* in
    bit 10 (``0x400`` / 1024) and the magnitude (0-1023) in bits 0-9. A load in
    the negative direction thus reads as a raw value ``>= 1024`` — so comparing
    the raw register value against a contact threshold (a few hundred) yields a
    spurious "contact" the instant load points the other way. Callers that
    threshold on load (e.g. the gentle-move contact check) must compare *this*
    magnitude, not the raw value.

    Parameters
    ----------
    present_load:
        The raw Present_Load register value as returned by ``read_info``.

    Returns
    -------
    int
        The load magnitude in the range ``[0, 1023]`` (bit 10 direction sign
        masked off).
    """
    return present_load & _LOAD_MAGNITUDE_MASK


#: Shared remediation text for :class:`OverloadError`.
_REMEDIATION_OVERLOAD = (
    "The servo latched an overload fault (status error bit 5 / 0x20): load "
    "exceeded its Torque_Limit. Call clear_overload(motor) to disable torque, "
    "relieve the mechanical load, then re-enable torque before retrying."
)


class OverloadError(CliError):
    """A :class:`CliError` subtype for a servo-reported overload (status bit 5, ``0x20``).

    Bus read/write paths raise this INSTEAD OF the generic :class:`CliError`
    whenever the SDK's returned status error byte satisfies :func:`is_overload`
    — a comms failure (nonzero ``result``) or any other status error bit still
    raises the plain :class:`CliError` as before. Because ``OverloadError`` IS
    a ``CliError`` (``code == EXIT_ENV_ERROR`` always), any existing
    ``except CliError:`` handler keeps working unmodified; only a caller that
    wants to react specifically to an overload needs to add
    ``except OverloadError:``.

    Parameters
    ----------
    motor:
        Motor ID that reported the overload.
    error_byte:
        The raw status error byte (bit 5 set); stored verbatim so a caller
        can inspect any other bits that were set alongside it.
    message:
        Human-readable message. Defaults to a generic description naming
        *motor* and *error_byte*.
    remediation:
        Remediation hint. Defaults to :data:`_REMEDIATION_OVERLOAD`.

    Attributes
    ----------
    motor: int
    error_byte: int
    """

    def __init__(
        self,
        motor: int,
        error_byte: int,
        message: "str | None" = None,
        remediation: str = _REMEDIATION_OVERLOAD,
    ) -> None:
        self.motor = motor
        self.error_byte = error_byte
        if message is None:
            message = f"Motor {motor} reported an overload (status error byte={error_byte})."
        super().__init__(code=EXIT_ENV_ERROR, message=message, remediation=remediation)


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

    @abc.abstractmethod
    def enable_torque(self, motor: int, on: bool) -> None:
        """Enable or disable torque for *motor*.

        Parameters
        ----------
        motor:
            Motor ID (1-indexed, matching the Feetech servo ID).
        on:
            ``True`` to enable torque, ``False`` to relax it.

        Raises
        ------
        CliError(EXIT_ENV_ERROR)
            If the bus has not been opened or the write fails.
        """

    @abc.abstractmethod
    def write_goal_position(self, motor: int, position: int) -> None:
        """Write the goal position for *motor*.

        Parameters
        ----------
        motor:
            Motor ID (1-indexed, matching the Feetech servo ID).
        position:
            Target encoder tick in ``[0, 4095]``.

        Raises
        ------
        CliError(EXIT_USER_ERROR)
            If *position* is outside ``[0, 4095]``.
        CliError(EXIT_ENV_ERROR)
            If the bus has not been opened or the write fails.
        """

    @abc.abstractmethod
    def write_baudrate(self, motor: int, baudrate: int) -> None:
        """Write only the baud-rate register to the motor's EEPROM (addr 6).

        Changes the motor's EEPROM baud rate without touching the servo ID
        register.  On tested STS3215 firmware (3.10) the change takes effect
        **immediately** — to keep talking to the motor the caller must reopen
        the port at the new baud (older firmware may instead defer to the next
        power-up, in which case it still answers at the old baud this session).

        Parameters
        ----------
        motor:
            Current motor ID used to address the servo on the bus.
        baudrate:
            Baud rate to programme into EEPROM (e.g. 1 000 000).

        Raises
        ------
        CliError(EXIT_ENV_ERROR)
            If the bus has not been opened, the baudrate is not in
            :data:`BAUD_MAP`, or the write fails.
        """

    @abc.abstractmethod
    def read_lock(self, motor: int) -> int:
        """Return the STS3215 Lock register value for *motor*.

        Parameters
        ----------
        motor:
            Motor ID (1-indexed, matching the Feetech servo ID).

        Returns
        -------
        int
            Lock register value (0=unlocked, 1=locked).

        Raises
        ------
        CliError(EXIT_ENV_ERROR)
            If the bus has not been opened or a read error occurs.
        """

    @abc.abstractmethod
    def write_acceleration(self, motor: int, value: int) -> None:
        """Write the goal acceleration for *motor*.

        Parameters
        ----------
        motor:
            Motor ID (1-indexed, matching the Feetech servo ID).
        value:
            Acceleration in ``[0, 254]`` (STS3215 Acceleration register units).

        Raises
        ------
        CliError(EXIT_USER_ERROR)
            If *value* is outside ``[0, 254]``.
        CliError(EXIT_ENV_ERROR)
            If the bus has not been opened or the write fails.
        """

    @abc.abstractmethod
    def write_goal_speed(self, motor: int, value: int) -> None:
        """Write the goal (running) speed for *motor*.

        Parameters
        ----------
        motor:
            Motor ID (1-indexed, matching the Feetech servo ID).
        value:
            Speed in ``[0, 4095]`` (STS3215 Goal/Running Speed register units).

        Raises
        ------
        CliError(EXIT_USER_ERROR)
            If *value* is outside ``[0, 4095]``.
        CliError(EXIT_ENV_ERROR)
            If the bus has not been opened or the write fails.
        """

    @abc.abstractmethod
    def read_torque_limit(self, motor: int) -> int:
        """Return the RAM Torque_Limit register value for *motor* (STS3215 addr 48, 2 bytes).

        Torque_Limit caps the maximum torque (``[0, 1000]``, units of 0.1%
        of rated torque) the servo is permitted to apply. It is a RAM
        register, distinct from the EEPROM Protective_Torque /
        Protection_Time / Overload_Torque registers at addr 34-36 — those
        firmware fault-detection thresholds are never written by this bus.

        Raises
        ------
        CliError(EXIT_ENV_ERROR)
            If the bus has not been opened or a read error occurs.
        OverloadError
            If the servo's returned status error byte reports an overload
            (a :class:`CliError` subtype; see :func:`is_overload`).
        """

    @abc.abstractmethod
    def write_torque_limit(self, motor: int, value: int) -> None:
        """Write the RAM Torque_Limit register for *motor* (STS3215 addr 48, 2 bytes).

        Parameters
        ----------
        motor:
            Motor ID (1-indexed, matching the Feetech servo ID).
        value:
            Torque limit in ``[0, 1000]`` (units of 0.1% of rated torque).

        Raises
        ------
        CliError(EXIT_USER_ERROR)
            If *value* is outside ``[0, 1000]``.
        CliError(EXIT_ENV_ERROR)
            If the bus has not been opened or the write fails.
        OverloadError
            If the servo's returned status error byte reports an overload
            (a :class:`CliError` subtype; see :func:`is_overload`).
        """

    @abc.abstractmethod
    def clear_overload(self, motor: int) -> None:
        """Disable torque for *motor* (Torque_Enable=0, addr 40) to clear a latched overload.

        STS3215 latches an overload status until torque is explicitly
        disabled; this is the documented recovery action for status error
        bit 5 (``0x20`` — see :func:`is_overload`). Equivalent in effect to
        ``enable_torque(motor, False)``, exposed under its own name so
        overload-recovery call sites read clearly.

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

    def _status_error(
        self,
        motor: int,
        result: int,
        error: int,
        action: str,
        remediation: str = _REMEDIATION_CHECK_WIRING,
    ) -> CliError:
        """Build (but do not raise) the CliError for a failed ``result``/``error`` pair.

        *action* is a short present-tense failure description WITHOUT the
        trailing "result=…, error=…" suffix — this method appends it, e.g.
        ``self._status_error(motor, result, error, f"Read position failed for motor {motor}")``.

        If *error* has the overload bit set (:func:`is_overload`), returns an
        :class:`OverloadError` instead of the generic :class:`CliError`, so a
        caller further up the stack can ``except OverloadError`` distinctly
        from every other bus failure. A comms failure (nonzero *result*) with
        no status error byte, or any status error byte other than the
        overload bit, still returns the plain :class:`CliError`.

        Callers must still ``raise`` the returned exception — this method
        composes naturally with ``raise self._status_error(...)``.
        """
        if error != 0 and is_overload(error):
            return OverloadError(
                motor=motor,
                error_byte=error,
                message=f"{action}: motor {motor} reported an overload (error={error}).",
            )
        return CliError(
            code=EXIT_ENV_ERROR,
            message=f"{action}: result={result}, error={error}.",
            remediation=remediation,
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
                message=_SDK_MISSING_MSG,
                remediation=_SDK_INSTALL_HINT,
            ) from None
        return sdk

    def _set_lock(self, motor: int, locked: bool) -> None:
        """Write the STS3215 EEPROM Lock register (addr 55) for *motor*.

        ``locked=False`` (Lock=0) MUST precede any EEPROM register write (id at
        addr 5, baud at addr 6) for that write to *persist*: on fw 3.10 a write
        while Lock=1 updates the live register but is NOT committed to EEPROM, so
        the value reverts to the stored one on the next power-up. Re-lock
        (``locked=True``) afterwards to restore write-protection.
        """
        self._require_open()
        _ADDR_LOCK = 55
        result, error = self._packet_handler.write1ByteTxRx(  # type: ignore[union-attr]
            self._port_handler, motor, _ADDR_LOCK, 1 if locked else 0
        )
        if result != 0 or error != 0:
            state = "re-lock" if locked else "unlock"
            raise self._status_error(
                motor, result, error, f"Failed to {state} EEPROM for motor {motor}"
            )

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
        self._require_open()

        # STS3215 present-position address = 56, 2 bytes.
        _ADDR_PRESENT_POSITION = 56

        value, result, error = self._packet_handler.read2ByteTxRx(  # type: ignore[union-attr]
            self._port_handler, motor, _ADDR_PRESENT_POSITION
        )
        if result != 0 or error != 0:  # any non-zero comm result or servo error means failure
            raise self._status_error(
                motor, result, error, f"Read position failed for motor {motor}"
            )
        return int(value) & 0x0FFF  # mask to 12 bits

    def write_id_baudrate(self, motor: int, new_id: int, baudrate: int) -> None:
        """Write servo ID and baud-rate to the motor's EEPROM.

        STS3215 EEPROM registers: ID = address 5 (1 byte), Baud_Rate = address 6 (1 byte).
        The baud-rate register uses a Feetech-specific index (not the raw bps value).

        Order matters: the **baud-rate is written first, the ID last**, both
        addressed at the motor's *current* id (``motor``).  Writing the ID first
        would change the device's address mid-call, so the subsequent baud write
        — still aimed at the old id — would hit a now-unreachable device and
        fail.

        Caveat (verified on fw 3.10): the STS3215 applies a baud change
        **immediately**, so this baud-first order is only safe when *baudrate*
        equals the current comms baud (the ``setup-motors`` default, 1 000 000 —
        the motor is already there, so nothing switches).  A *differing*
        ``baudrate`` would switch the motor mid-call and make the following ID
        write fail; reassigning id and baud together to a new baud needs a
        reopen between the two writes (not yet implemented).

        Exception safety: the EEPROM is unlocked before the writes; if either
        write fails, a best-effort re-lock is attempted before the original
        ``CliError`` propagates, so a failed call never strands the motor at
        Lock=0. The re-lock targets the NEW id only once the ID write has
        actually succeeded — if the baud write or the ID write itself fails,
        the device address never moved, so the re-lock (best-effort or final)
        is addressed to the original *motor* id instead.
        """
        from arm101.cli._errors import EXIT_ENV_ERROR, CliError

        self._require_open()

        # Baud-rate index mapping (Feetech STS3215 datasheet) — use the
        # module-level BAUD_MAP so there is a single source of truth.
        baud_index = BAUD_MAP.get(baudrate)
        if baud_index is None:
            raise CliError(
                code=EXIT_ENV_ERROR,
                message=f"Unsupported baud rate {baudrate}. Supported: {sorted(BAUD_MAP)}.",
                remediation=_REMEDIATION_CHOOSE_BAUD,
            )

        _ADDR_ID = 5
        _ADDR_BAUD = 6

        # Unlock the EEPROM so the id/baud writes COMMIT. On STS3215 fw 3.10 a
        # write while Lock=1 updates the live register but is NOT persisted to
        # EEPROM — the value reverts to the stored one on the next power-up.
        # Without this, an assigned id silently reverts to the factory default
        # when the motor is power-cycled (verified on hardware).
        self._set_lock(motor, False)
        # Starts at the current id; becomes new_id only once the ID write
        # (addr 5) itself has succeeded — until then the device is still
        # listening at `motor`, on a failure path or otherwise.
        relock_target = motor
        try:
            for addr, val, label in (
                (_ADDR_BAUD, baud_index, "Baud_Rate"),  # baud first (motor still at current id)
                (_ADDR_ID, new_id, "ID"),  # change id last — final op on the old address
            ):
                result, error = self._packet_handler.write1ByteTxRx(  # type: ignore[union-attr]
                    self._port_handler, motor, addr, val
                )
                if result != 0 or error != 0:
                    raise self._status_error(
                        motor, result, error, f"Write {label} failed for motor {motor}"
                    )
                if addr == _ADDR_ID:
                    relock_target = new_id
        except BaseException:
            # Best-effort re-lock so a failed write never strands the motor at
            # Lock=0; if the re-lock itself fails, preserve the ORIGINAL error.
            with contextlib.suppress(Exception):
                self._set_lock(relock_target, True)
            raise
        else:
            # Re-lock to restore write-protection. The ID write changed the device
            # address, so the relock is addressed to the NEW id.
            self._set_lock(relock_target, True)

    def write_baudrate(self, motor: int, baudrate: int) -> None:
        """Write only the baud-rate register (addr 6) to the motor's EEPROM.

        Unlike :meth:`write_id_baudrate`, this does **not** touch the ID
        register (addr 5) — only the baud-rate index is written.

        STS3215 Baud_Rate EEPROM register: address 6 (1 byte, Feetech index).
        On tested firmware (3.10) the new baud takes effect immediately. Once
        the write succeeds, this method switches the *host* port to match
        (``self._port_handler.setBaudRate``, mirroring :meth:`open`) before
        re-locking, so the re-lock — sent over the same serial connection —
        reaches the motor at the baud it is now actually listening on, instead
        of the stale one.

        Exception safety: the EEPROM is unlocked before the write; if the
        write fails, a best-effort re-lock is attempted (at the unchanged
        *motor* id, over the still-unswitched host baud) before the original
        ``CliError`` propagates, so a failed call never strands the motor at
        Lock=0.
        """
        from arm101.cli._errors import EXIT_ENV_ERROR, CliError

        self._require_open()

        baud_index = BAUD_MAP.get(baudrate)
        if baud_index is None:
            raise CliError(
                code=EXIT_ENV_ERROR,
                message=f"Unsupported baud rate {baudrate}. Supported: {sorted(BAUD_MAP)}.",
                remediation=_REMEDIATION_CHOOSE_BAUD,
            )

        _ADDR_BAUD = 6

        # Unlock EEPROM so the baud write persists across a power-cycle, then
        # re-lock (see write_id_baudrate / _set_lock). The id is unchanged here,
        # so both lock writes are addressed to the same *motor* id.
        self._set_lock(motor, False)
        try:
            result, error = self._packet_handler.write1ByteTxRx(  # type: ignore[union-attr]
                self._port_handler, motor, _ADDR_BAUD, baud_index
            )
            if result != 0 or error != 0:
                raise self._status_error(
                    motor, result, error, f"Write Baud_Rate failed for motor {motor}"
                )
            if baudrate != self._baudrate:
                # The motor already applies the new baud; switch the host port
                # to match (mirrors open()) so the relock below — and anything
                # sent afterwards — actually reaches the motor.
                self._port_handler.setBaudRate(baudrate)  # type: ignore[union-attr]
                self._baudrate = baudrate
        except BaseException:
            # Best-effort re-lock so a failed write never strands the motor at
            # Lock=0; if the re-lock itself fails, preserve the ORIGINAL error.
            with contextlib.suppress(Exception):
                self._set_lock(motor, True)
            raise
        else:
            self._set_lock(motor, True)

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
        # EEPROM write-lock flag (addr 55). Surfaced in the center-motor plan
        # snapshot so a locked motor (Lock=1) is visible before a gated write;
        # without this, build_plan would default lock_register to 0 on real
        # hardware even though the motor reports it.
        "lock_register": (55, 1),
    }

    def _read_register(self, motor: int, addr: int, length: int) -> int:
        """Read a 1- or 2-byte register; raise CliError on a comms failure."""
        if length == 1:
            value, result, error = self._packet_handler.read1ByteTxRx(  # type: ignore[union-attr]
                self._port_handler, motor, addr
            )
        else:
            value, result, error = self._packet_handler.read2ByteTxRx(  # type: ignore[union-attr]
                self._port_handler, motor, addr
            )
        if result != 0 or error != 0:
            raise self._status_error(
                motor, result, error, f"Read of register {addr} failed for motor {motor}"
            )
        return int(value)

    def scan(self, ids: "list[int] | None" = None) -> "list[int]":
        """Ping candidate *ids* and return those that respond (read-only).

        With no *ids*, sweeps the full valid Feetech id space (1–253) so a motor
        that was previously re-id'd above the SO-101's 1–12 range is still found
        — important for ``set-motor-id``, whose whole job is to fix a motor's id.
        This SDK build has no ``broadcastPing``, so detection is an exhaustive
        unicast ping sweep; it is a one-time pre-assembly step, not latency
        sensitive. Pass an explicit *ids* list to narrow it.
        """
        self._require_open()
        candidates = list(ids) if ids is not None else list(range(1, 254))
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

    def enable_torque(self, motor: int, on: bool) -> None:
        """Enable or disable torque for *motor*.

        STS3215 Torque_Enable register: address 40, 1 byte.
        Write 1 to enable, 0 to disable.
        """
        self._require_open()

        _ADDR_TORQUE_ENABLE = 40

        result, error = self._packet_handler.write1ByteTxRx(  # type: ignore[union-attr]
            self._port_handler, motor, _ADDR_TORQUE_ENABLE, 1 if on else 0
        )
        if result != 0 or error != 0:
            state = "enable" if on else "disable"
            raise self._status_error(
                motor, result, error, f"Failed to {state} torque for motor {motor}"
            )

    def write_goal_position(self, motor: int, position: int) -> None:
        """Write the goal position for *motor*.

        STS3215 Goal_Position register: address 42, 2 bytes.
        Valid range: ``[0, 4095]`` (12-bit encoder).
        """
        from arm101.cli._errors import EXIT_USER_ERROR, CliError

        self._require_open()

        if not (0 <= position <= 4095):
            raise CliError(
                code=EXIT_USER_ERROR,
                message=(
                    f"Goal position {position} is out of range; "
                    "valid range is 0–4095 (12-bit encoder)."
                ),
                remediation="Pass a --position value between 0 and 4095.",
            )

        _ADDR_GOAL_POSITION = 42

        result, error = self._packet_handler.write2ByteTxRx(  # type: ignore[union-attr]
            self._port_handler, motor, _ADDR_GOAL_POSITION, position
        )
        if result != 0 or error != 0:
            raise self._status_error(
                motor, result, error, f"Write goal position failed for motor {motor}"
            )

    def read_lock(self, motor: int) -> int:
        """Read the STS3215 Lock register (address 55, 1 byte) for *motor*.

        Returns
        -------
        int
            Lock register value (0=unlocked, 1=locked).
        """
        self._require_open()

        _ADDR_LOCK = 55

        value, result, error = self._packet_handler.read1ByteTxRx(  # type: ignore[union-attr]
            self._port_handler, motor, _ADDR_LOCK
        )
        if result != 0 or error != 0:
            raise self._status_error(
                motor, result, error, f"Read lock register failed for motor {motor}"
            )
        return int(value)

    def write_acceleration(self, motor: int, value: int) -> None:
        """Write the goal acceleration for *motor*.

        STS3215 Acceleration register: address 41, 1 byte.
        Valid range: ``[0, 254]``.
        """
        from arm101.cli._errors import EXIT_USER_ERROR, CliError

        self._require_open()

        if not (0 <= value <= 254):
            raise CliError(
                code=EXIT_USER_ERROR,
                message=(f"Acceleration {value} is out of range; valid range is 0–254."),
                remediation="Pass an --acceleration value between 0 and 254.",
            )

        _ADDR_ACCELERATION = 41

        result, error = self._packet_handler.write1ByteTxRx(  # type: ignore[union-attr]
            self._port_handler, motor, _ADDR_ACCELERATION, value
        )
        if result != 0 or error != 0:
            raise self._status_error(
                motor, result, error, f"Write acceleration failed for motor {motor}"
            )

    def write_goal_speed(self, motor: int, value: int) -> None:
        """Write the goal (running) speed for *motor*.

        STS3215 Goal/Running Speed register: address 46, 2 bytes.
        Valid range: ``[0, 4095]``.
        """
        from arm101.cli._errors import EXIT_USER_ERROR, CliError

        self._require_open()

        if not (0 <= value <= 4095):
            raise CliError(
                code=EXIT_USER_ERROR,
                message=(f"Goal speed {value} is out of range; valid range is 0–4095."),
                remediation="Pass a --speed value between 0 and 4095.",
            )

        _ADDR_GOAL_SPEED = 46

        result, error = self._packet_handler.write2ByteTxRx(  # type: ignore[union-attr]
            self._port_handler, motor, _ADDR_GOAL_SPEED, value
        )
        if result != 0 or error != 0:
            raise self._status_error(
                motor, result, error, f"Write goal speed failed for motor {motor}"
            )

    def read_torque_limit(self, motor: int) -> int:
        """Read the RAM Torque_Limit register (address 48, 2 bytes) for *motor*.

        Torque_Limit caps the maximum torque (``[0, 1000]``, units of 0.1%
        of rated torque) the servo may apply. Distinct from the EEPROM
        Protective_Torque / Protection_Time / Overload_Torque registers
        (addr 34-36), which this bus never writes.
        """
        self._require_open()

        _ADDR_TORQUE_LIMIT = 48

        return self._read_register(motor, _ADDR_TORQUE_LIMIT, 2)

    def write_torque_limit(self, motor: int, value: int) -> None:
        """Write the RAM Torque_Limit register (address 48, 2 bytes) for *motor*.

        Valid range: ``[0, 1000]`` (units of 0.1% of rated torque). This is a
        RAM register — unlike :meth:`write_id_baudrate` / :meth:`write_baudrate`
        it does NOT go through the Lock (addr 55) unlock/relock dance, and it
        never touches the EEPROM protection registers at addr 34-36.
        """
        from arm101.cli._errors import EXIT_USER_ERROR, CliError

        self._require_open()

        if not (0 <= value <= 1000):
            raise CliError(
                code=EXIT_USER_ERROR,
                message=(f"Torque limit {value} is out of range; valid range is 0–1000."),
                remediation="Pass a --torque-limit value between 0 and 1000.",
            )

        _ADDR_TORQUE_LIMIT = 48

        result, error = self._packet_handler.write2ByteTxRx(  # type: ignore[union-attr]
            self._port_handler, motor, _ADDR_TORQUE_LIMIT, value
        )
        if result != 0 or error != 0:
            raise self._status_error(
                motor, result, error, f"Write torque limit failed for motor {motor}"
            )

    def clear_overload(self, motor: int) -> None:
        """Disable torque for *motor* (Torque_Enable=0, addr 40) to clear a latched overload.

        Equivalent to ``enable_torque(motor, False)``; exposed under its own
        name so overload-recovery call sites read clearly.
        """
        self.enable_torque(motor, False)


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
    torque_writes:
        List of dicts, one per :meth:`enable_torque` call, in call order::

            {"motor": int, "on": bool}

        Tests for ``center-motor`` inspect this to assert that torque was
        enabled *before* motion and relaxed *after* (ordering guarantee).
    position_writes:
        List of dicts, one per :meth:`write_goal_position` call, in call order::

            {"motor": int, "position": int}

        Tests for ``center-motor`` inspect this to confirm the commanded tick.
    baud_writes:
        List of dicts, one per :meth:`write_baudrate` call, in call order::

            {"motor": int, "baudrate": int}

        Tests for ``set-baudrate`` inspect this to assert the baud was written
        without altering the motor ID (no :attr:`eeprom_writes` entry).
    accel_writes:
        List of dicts, one per :meth:`write_acceleration` call, in call order::

            {"motor": int, "value": int}

        Tests for ``compliant_move`` inspect this to confirm the commanded
        acceleration and that it was written before the goal-speed/position.
    speed_writes:
        List of dicts, one per :meth:`write_goal_speed` call, in call order::

            {"motor": int, "value": int}

        Tests for ``compliant_move`` inspect this to confirm the commanded
        goal speed and that it was written before torque/position.
    torque_limit_writes:
        List of dicts, one per :meth:`write_torque_limit` call, in call order::

            {"motor": int, "value": int}
    register_writes:
        List of dicts, one per ACTUAL register write across every write
        method above (plus :meth:`write_torque_limit` / :meth:`clear_overload`),
        keyed by raw address::

            {"motor": int, "addr": int, "value": int}

        A superset ledger — every feature-specific ``*_writes`` list above
        also gets an entry here. Tests use it to assert, across the WHOLE
        write surface, that no code path ever writes the EEPROM protection
        registers at addr 34 (Protective_Torque), 35 (Protection_Time), or
        36 (Overload_Torque).
    overload_after_ops:
        ``int | None``. When set, the running count of read/write register
        operations (see :meth:`_tick_and_maybe_overload`) reaching this value
        makes every call from then on raise :class:`OverloadError`
        (``error_byte=32``) instead of performing its normal effect —
        simulating a servo that has latched an overload fault mid-move. May
        be set directly or via the :meth:`fail_with_overload_on_op`
        convenience. :meth:`clear_overload` disarms it (sets it back to
        ``None``), mirroring the real recovery action. ``scan()`` and
        :meth:`clear_overload` itself are exempt from the counter.
    """

    def __init__(
        self,
        positions: dict[int, int] | None = None,
        ids: "list[int] | None" = None,
        info: "dict[int, dict[str, int]] | None" = None,
        lock_register: int = 0,
        torque_limits: "dict[int, int] | None" = None,
        overload_after_ops: "int | None" = None,
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
        self.lock_register = lock_register
        self.eeprom_writes: list[dict[str, int]] = []
        self.torque_writes: list[dict] = []
        self.position_writes: list[dict] = []
        self.baud_writes: list[dict[str, int]] = []
        self.accel_writes: list[dict] = []
        self.speed_writes: list[dict] = []
        self._torque_limits: dict[int, int] = dict(torque_limits) if torque_limits else {}
        self.torque_limit_writes: list[dict[str, int]] = []
        self.register_writes: list[dict[str, int]] = []
        self.overload_after_ops: "int | None" = overload_after_ops
        self._op_count: int = 0
        self._open = False

    # ------------------------------------------------------------------
    # Overload-simulation seam + per-address write ledger (test helpers)
    # ------------------------------------------------------------------

    def fail_with_overload_on_op(self, n: int) -> "FakeBus":
        """Arm the overload-simulation seam: from the *n*-th bus operation onward.

        Equivalent to setting ``self.overload_after_ops = n`` directly; this
        is a fluent convenience for test setup::

            bus = FakeBus().fail_with_overload_on_op(3)
            bus.open()
            bus.read_position(1)   # op 1 — OK
            bus.read_info(1)       # op 2 — OK
            bus.read_position(1)   # op 3 — raises OverloadError(error_byte=32)

        *n* is 1-indexed and counts every register read/write call (see
        :meth:`_tick_and_maybe_overload`); ``scan()`` and
        :meth:`clear_overload` do not count. Returns ``self`` so it can be
        chained directly onto the constructor call.
        """
        self.overload_after_ops = n
        return self

    def _tick_and_maybe_overload(self, motor: int) -> None:
        """Advance the bus-operation counter; raise OverloadError if the seam is armed.

        Called near the top of every register read/write method (after the
        "bus is open" check, before the operation's own effect) so
        :attr:`overload_after_ops` — set directly or via
        :meth:`fail_with_overload_on_op` — governs the whole read/write
        surface uniformly. ``scan()`` (a ping sweep, not a register access)
        and :meth:`clear_overload` (the recovery action — see its docstring)
        are deliberately exempt.

        Once the running count reaches :attr:`overload_after_ops`, every
        subsequent call raises too, mimicking a servo that LATCHES the
        overload status until torque is explicitly disabled. The real
        recovery path is: catch ``OverloadError``, call
        :meth:`clear_overload`, which also disarms this seam.
        """
        self._op_count += 1
        if self.overload_after_ops is not None and self._op_count >= self.overload_after_ops:
            raise OverloadError(
                motor=motor,
                error_byte=_OVERLOAD_BIT,
                message=(
                    f"FakeBus: simulated overload on motor {motor} "
                    f"(operation #{self._op_count})."
                ),
            )

    def _record_write(self, motor: int, addr: int, value: int) -> None:
        """Append a ``{"motor", "addr", "value"}`` entry to :attr:`register_writes`.

        Called by every FakeBus write method — the pre-existing
        write_id_baudrate / write_baudrate / enable_torque /
        write_goal_position / write_acceleration / write_goal_speed, plus the
        new write_torque_limit / clear_overload — so a test can assert,
        across the WHOLE write surface, which raw register addresses were
        ever touched (e.g. to prove no code path writes the EEPROM
        protection registers at addr 34/35/36).
        """
        self.register_writes.append({"motor": motor, "addr": addr, "value": value})

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
        OverloadError
            If the overload-simulation seam is armed and this call is the
            Nth (or later) operation; see :meth:`fail_with_overload_on_op`.
        """
        from arm101.cli._errors import EXIT_ENV_ERROR, CliError

        if not self._open:
            raise CliError(
                code=EXIT_ENV_ERROR,
                message=_FAKEBUS_NOT_OPEN_MSG,
                remediation=_FAKEBUS_NOT_OPEN_REMEDIATION,
            )
        self._tick_and_maybe_overload(motor)
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
                message=_FAKEBUS_NOT_OPEN_MSG,
                remediation=_FAKEBUS_NOT_OPEN_REMEDIATION,
            )
        self._tick_and_maybe_overload(motor)
        self.eeprom_writes.append({"motor": motor, "new_id": new_id, "baudrate": baudrate})
        # Mirrors FeetechBus's write order (baud addr 6, then id addr 5).
        self._record_write(motor, 6, BAUD_MAP.get(baudrate, baudrate))
        self._record_write(motor, 5, new_id)

    def write_baudrate(self, motor: int, baudrate: int) -> None:
        """Record a baud-rate EEPROM write in :attr:`baud_writes`.

        Mirrors the :meth:`MotorBus.write_baudrate` contract: an unsupported
        baud rate is rejected (matching :class:`FeetechBus`) so a value that
        would fail on real hardware also fails against the fake, rather than
        being silently recorded.

        Raises
        ------
        CliError(EXIT_ENV_ERROR)
            If the bus has not been opened, or *baudrate* is not in
            :data:`BAUD_MAP`.
        """
        from arm101.cli._errors import EXIT_ENV_ERROR, CliError

        self._require_open_fake()
        if baudrate not in BAUD_MAP:
            raise CliError(
                code=EXIT_ENV_ERROR,
                message=f"Unsupported baud rate {baudrate}. Supported: {sorted(BAUD_MAP)}.",
                remediation=_REMEDIATION_CHOOSE_BAUD,
            )
        self._tick_and_maybe_overload(motor)
        self.baud_writes.append({"motor": motor, "baudrate": baudrate})
        self._record_write(motor, 6, BAUD_MAP[baudrate])

    def enable_torque(self, motor: int, on: bool) -> None:
        """Record a torque enable/disable call in :attr:`torque_writes`.

        Raises
        ------
        CliError(EXIT_ENV_ERROR)
            If the bus has not been opened.
        """
        self._require_open_fake()
        self._tick_and_maybe_overload(motor)
        self.torque_writes.append({"motor": motor, "on": on})
        self._record_write(motor, 40, 1 if on else 0)

    def write_goal_position(self, motor: int, position: int) -> None:
        """Record a goal-position write in :attr:`position_writes`.

        Raises
        ------
        CliError(EXIT_USER_ERROR)
            If *position* is outside ``[0, 4095]``.
        CliError(EXIT_ENV_ERROR)
            If the bus has not been opened.
        """
        from arm101.cli._errors import EXIT_USER_ERROR, CliError

        self._require_open_fake()
        self._tick_and_maybe_overload(motor)

        if not (0 <= position <= 4095):
            raise CliError(
                code=EXIT_USER_ERROR,
                message=(
                    f"Goal position {position} is out of range; "
                    "valid range is 0–4095 (12-bit encoder)."
                ),
                remediation="Pass a --position value between 0 and 4095.",
            )
        self.position_writes.append({"motor": motor, "position": position})
        self._record_write(motor, 42, position)

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
        self._tick_and_maybe_overload(motor)
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
            "lock_register": self.lock_register,
        }
        snapshot.update(self._info_overrides.get(motor, {}))
        return snapshot

    def read_lock(self, motor: int) -> int:
        """Return the configured lock register value for *motor*.

        Raises
        ------
        CliError(EXIT_ENV_ERROR)
            If the bus has not been opened.
        """
        self._require_open_fake()
        self._tick_and_maybe_overload(motor)
        return self.lock_register

    def write_acceleration(self, motor: int, value: int) -> None:
        """Record an acceleration write in :attr:`accel_writes`.

        Raises
        ------
        CliError(EXIT_USER_ERROR)
            If *value* is outside ``[0, 254]``.
        CliError(EXIT_ENV_ERROR)
            If the bus has not been opened.
        """
        from arm101.cli._errors import EXIT_USER_ERROR, CliError

        self._require_open_fake()
        self._tick_and_maybe_overload(motor)

        if not (0 <= value <= 254):
            raise CliError(
                code=EXIT_USER_ERROR,
                message=(f"Acceleration {value} is out of range; valid range is 0–254."),
                remediation="Pass an --acceleration value between 0 and 254.",
            )
        self.accel_writes.append({"motor": motor, "value": value})
        self._record_write(motor, 41, value)

    def write_goal_speed(self, motor: int, value: int) -> None:
        """Record a goal-speed write in :attr:`speed_writes`.

        Raises
        ------
        CliError(EXIT_USER_ERROR)
            If *value* is outside ``[0, 4095]``.
        CliError(EXIT_ENV_ERROR)
            If the bus has not been opened.
        """
        from arm101.cli._errors import EXIT_USER_ERROR, CliError

        self._require_open_fake()
        self._tick_and_maybe_overload(motor)

        if not (0 <= value <= 4095):
            raise CliError(
                code=EXIT_USER_ERROR,
                message=(f"Goal speed {value} is out of range; valid range is 0–4095."),
                remediation="Pass a --speed value between 0 and 4095.",
            )
        self.speed_writes.append({"motor": motor, "value": value})
        self._record_write(motor, 46, value)

    def read_torque_limit(self, motor: int) -> int:
        """Return the configured Torque_Limit for *motor* (default 1000).

        Raises
        ------
        CliError(EXIT_ENV_ERROR)
            If the bus has not been opened.
        OverloadError
            If the overload-simulation seam is armed and this call is the
            Nth (or later) operation; see :meth:`fail_with_overload_on_op`.
        """
        self._require_open_fake()
        self._tick_and_maybe_overload(motor)
        return self._torque_limits.get(motor, 1000)

    def write_torque_limit(self, motor: int, value: int) -> None:
        """Record a Torque_Limit write in :attr:`torque_limit_writes`; update the round-trip value.

        Raises
        ------
        CliError(EXIT_USER_ERROR)
            If *value* is outside ``[0, 1000]``.
        CliError(EXIT_ENV_ERROR)
            If the bus has not been opened.
        OverloadError
            If the overload-simulation seam is armed and this call is the
            Nth (or later) operation; see :meth:`fail_with_overload_on_op`.
        """
        from arm101.cli._errors import EXIT_USER_ERROR, CliError

        self._require_open_fake()
        self._tick_and_maybe_overload(motor)

        if not (0 <= value <= 1000):
            raise CliError(
                code=EXIT_USER_ERROR,
                message=(f"Torque limit {value} is out of range; valid range is 0–1000."),
                remediation="Pass a --torque-limit value between 0 and 1000.",
            )
        self._torque_limits[motor] = value
        self.torque_limit_writes.append({"motor": motor, "value": value})
        self._record_write(motor, 48, value)

    def clear_overload(self, motor: int) -> None:
        """Disable torque for *motor* (Torque_Enable=0, addr 40) to clear a latched overload.

        Deliberately exempt from :meth:`_tick_and_maybe_overload` — on real
        hardware, disabling torque is the standard, always-available
        recovery action for a latched overload (bit 5 / 0x20 of the status
        error byte; see :func:`is_overload`), so this call must never itself
        raise ``OverloadError``. On success it also DISARMS the
        overload-simulation seam (``overload_after_ops = None``), letting a
        test drive the full catch-and-recover flow: arm the seam, hit
        ``OverloadError``, call ``clear_overload()``, then resume with
        normal (non-raising) calls.

        Raises
        ------
        CliError(EXIT_ENV_ERROR)
            If the bus has not been opened.
        """
        self._require_open_fake()
        self.torque_writes.append({"motor": motor, "on": False})
        self._record_write(motor, 40, 0)
        self.overload_after_ops = None

    def _require_open_fake(self) -> None:
        from arm101.cli._errors import EXIT_ENV_ERROR, CliError

        if not self._open:
            raise CliError(
                code=EXIT_ENV_ERROR,
                message=_FAKEBUS_NOT_OPEN_MSG,
                remediation=_FAKEBUS_NOT_OPEN_REMEDIATION,
            )
