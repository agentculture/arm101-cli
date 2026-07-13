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
import time
from types import TracebackType
from typing import TYPE_CHECKING, Callable

from arm101.cli._errors import EXIT_ENV_ERROR, EXIT_USER_ERROR, CliError

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
# Read-retry policy — a dropped/corrupt packet is normal; a read is idempotent
# ---------------------------------------------------------------------------

#: Bounded retry count for :class:`FeetechBus` reads (see
#: :meth:`FeetechBus._retry_read`). A read costs nothing to repeat, and a
#: dropped/corrupt packet on an otherwise healthy bus turned out to be common
#: enough that every ad-hoc probe script written during this session's live
#: hardware run ended up hand-rolling exactly this loop — 3 is the number
#: those scripts converged on independently.
#:
#: NEVER applied BLIND to writes. :meth:`FeetechBus.write_id_baudrate` and
#: :meth:`FeetechBus.write_baudrate` retry nothing, because a "failed" write
#: may in fact have landed — this is precisely how issue #21's Lock-register
#: bug and #38's id-transfer-window bug arose. A read is safe to repeat; a
#: write is not safe to repeat blind.
#:
#: :meth:`FeetechBus.write_offset` is the one write that DOES retry, and the
#: distinction is the whole point: it **reads the register back** and retries
#: only what that read proves is absent. See :data:`_OFFSET_WRITE_ATTEMPTS`.
_READ_RETRY_ATTEMPTS: int = 3

#: Bounded attempts for :meth:`FeetechBus.write_offset` — each one **verified**
#: by reading addr 31 back, never a blind repeat.
#:
#: Measured on the follower, 2026-07-13: an offset write returns ``result=-6``
#: (RX timeout) often, and in every observed case the servo had **applied** it —
#: only the ACK was lost. Treating that as a failure aborted probes that had
#: worked and left a joint wearing a borrowed calibration. A failure in the
#: torque-off *before* the write (``result=-7`` observed) never landed. The
#: status byte cannot tell those apart; the register can.
#:
#: So: attempt, then ASK. Matching read -> done, however ugly the status was.
#: Non-matching read -> the write demonstrably did not land, so repeating it is
#: not a blind retry but a verified one. 3 attempts, matching the read policy.
_OFFSET_WRITE_ATTEMPTS: int = 3

#: Bounded attempts for **IDEMPOTENT RAM** writes — goal position, torque enable,
#: torque limit, acceleration, goal speed, the EEPROM Lock byte.
#:
#: Why these may be repeated when id/baud may NOT: repeating them changes nothing.
#: Writing goal=2048 twice leaves the servo wanting 2048, exactly as once did.
#: Writing id=4 twice does *not* leave the servo at id 4 — the second packet is
#: addressed to a motor that has already moved, which is precisely how issue #38's
#: id-transfer-window bug arose. Idempotence, not RAM-vs-EEPROM, is the dividing
#: line, and it is why these need no read-back to be safe (unlike
#: :data:`_OFFSET_WRITE_ATTEMPTS`, whose register is not idempotent to re-apply
#: blind on a servo whose state we do not know).
#:
#: Why they need it at all (hardware, 2026-07-13): this bus drops write ACKs at a
#: rate that is nowhere near negligible — a goal-position write failed with
#: ``result=-7`` mid-probe, and a follow-up sweep found the SAME write succeeding
#: at every delay from 0 to 0.6 s, so it was a dropped packet, not a busy servo.
#: Reads never showed it because reads have been retried all along; every drop on
#: a write surfaced as a hard failure and aborted the run. An unretried write on a
#: lossy bus is not a safety property — it is an outage.
_IDEMPOTENT_WRITE_ATTEMPTS: int = 3

#: Delay between read-retry attempts. Short enough to stay invisible to a
#: caller polling at ``gentle_move``'s ~25 ms cadence (``_DEFAULT_POLL_INTERVAL``
#: in ``arm101/hardware/gentle.py``), long enough to let a transiently busy bus
#: clear before the next attempt.
_READ_RETRY_DELAY_SECONDS: float = 0.05

#: How many times :meth:`MotorBus.clear_overload` re-checks that the servo has actually
#: dropped its overload latch after the torque-disable that clears it, and how long it
#: waits between checks. The latch is NOT instant: measured on the follower, a read issued
#: immediately after a successful torque-disable still came back tagged ``error=32``, while
#: the same read ~250 ms later was clean. Six checks at 50 ms covers that with room to
#: spare; a motor still latched after ~300 ms is a real fault, not a slow one.
_OVERLOAD_CLEAR_ATTEMPTS: int = 6
_OVERLOAD_CLEAR_DELAY_SECONDS: float = 0.05

# ---------------------------------------------------------------------------
# EEPROM write settle — a write can leave the servo briefly unable to answer
# ---------------------------------------------------------------------------

#: Delay applied after CLOSING the EEPROM Lock (addr 55 -> 1) — the last step
#: of every EEPROM write dance (:meth:`FeetechBus.write_id_baudrate`,
#: :meth:`FeetechBus.write_baudrate`, :meth:`FeetechBus.write_offset`) —
#: before the next bus call is allowed to proceed.
#:
#: Measured live on hardware, the same session as the packet-drop finding
#: above: a ``write_offset(3, 0)`` landed correctly (verified independently —
#: the servo genuinely held offset 0 and position 3387), yet a
#: ``read_position`` issued roughly 0.2 s later returned a silently-WRONG
#: ``0``. A re-read moments later returned the correct 3387, repeatedly and
#: stably. Unlike a dropped packet — which fails loudly and is what the
#: read-retry policy above exists for — this failure mode returns a
#: *plausible* value: position 0 looks exactly like a real reading, and
#: nothing downstream can tell it apart from the truth by inspecting it alone.
#:
#: 0.2 s was observed NOT to be enough; this constant is a deliberately
#: conservative margin past that observed failure point, not a proven-
#: sufficient bound — the true recovery latency is only bracketed between
#: "0.2 s: still wrong" and "a few tenths of a second later: correct and
#: stable". Kept small on purpose: it only costs anything on the already-rare
#: EEPROM-write path (id/baud/offset), never on the poll-rate reads
#: ``gentle_move`` issues roughly every 25 ms.
_EEPROM_SETTLE_SECONDS: float = 0.3

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


# ---------------------------------------------------------------------------
# Register addresses shared across methods (single source of truth)
# ---------------------------------------------------------------------------

#: STS3215 ``Torque_Enable`` — SRAM, addr 40, 1 byte. Write 0 to relax, 1 to hold.
ADDR_TORQUE_ENABLE: int = 40

#: STS3215 ``Lock`` — EEPROM write-protect flag, addr 55, 1 byte. It must be
#: opened (0) around any EEPROM write and closed (1) afterwards; see
#: :meth:`FeetechBus._set_lock` and PR #21.
ADDR_LOCK: int = 55

#: STS3215 ``Ofs`` (Feetech) / ``Homing_Offset`` (LeRobot) — **EEPROM**, addr 31,
#: 2 bytes (31 = OFS_L, 32 = OFS_H). The servo subtracts it from the raw encoder
#: count before reporting: ``Present_Position = Actual_Position - Homing_Offset``.
#:
#: This is the register that fixes issue #35. ``elbow_flex``'s encoder WRAPS
#: inside its physical travel — driven far enough it crosses the raw 4095->0
#: seam and reads back near zero — so its reported position is not monotonic
#: with joint angle, and every position comparison in this codebase (arrival
#: checks, ``clamp_goal``, ``ReachMap`` ranges) is silently wrong for it.
#: Writing an offset shifts the joint's zero so the seam falls in the arc the
#: joint *cannot reach*, which makes the linear-axis assumption TRUE rather than
#: merely assumed.
#:
#: It sits in the same EEPROM region as ID (5) and Baud_Rate (6) — the exact
#: registers that PR #21 proved revert on power-cycle when the Lock (addr 55) is
#: left closed — so it carries the identical persistence hazard.
#:
#: Research: ``docs/spikes/sts3215-offset-register.md`` (triple-sourced against
#: Feetech's SMS_STS.h, Feetech's Python SDK, and LeRobot's feetech/tables.py).
ADDR_HOMING_OFFSET: int = 31


# ---------------------------------------------------------------------------
# Encoder-offset codec — SIGN-MAGNITUDE on bit 11 (NOT two's complement)
# ---------------------------------------------------------------------------

#: Bit index of the SIGN in the offset register's encoding. The wire value is
#: ``(sign << 11) | magnitude`` — sign-magnitude, **not** two's complement.
#: (LeRobot: ``"Homing_Offset": 11`` in ``STS_SMS_SERIES_ENCODINGS_TABLE``.)
OFFSET_SIGN_BIT: int = 11

#: The sign bit itself (2048). Set = negative.
_OFFSET_SIGN_MASK: int = 1 << OFFSET_SIGN_BIT

#: Bits 0-10 — the magnitude field (0-2047).
_OFFSET_MAGNITUDE_MASK: int = _OFFSET_SIGN_MASK - 1

#: Widest magnitude the encoding can hold: ``(1 << 11) - 1`` = **2047**. An
#: offset outside ``[-2047, +2047]`` is not representable — a real SO-101 hit
#: exactly this (LeRobot issue #3193: ``ValueError: Magnitude 2073 exceeds
#: 2047``) — so it is REJECTED, never truncated. Truncation would spill the
#: magnitude straight into the sign bit and turn +2048 into "-0".
OFFSET_MAX_MAGNITUDE: int = _OFFSET_MAGNITUDE_MASK

#: The offset register occupies the low 12 bits of a 2-byte read; bits 12-15 are
#: not part of it. Mirrors :meth:`FeetechBus.read_position`'s ``& 0x0FFF``.
_OFFSET_REGISTER_MASK: int = 0xFFF

#: One full turn of the 12-bit magnetic encoder. The corrected position is
#: reduced modulo this, which is what makes the seam *move* rather than merely
#: be relabelled (see :meth:`FakeBus._reported_position` for the caveat).
ENCODER_RESOLUTION: int = 4096


def encode_offset(offset: int) -> int:
    """Encode a signed *offset* into the STS3215's sign-magnitude wire value.

    The offset register (:data:`ADDR_HOMING_OFFSET`) is **sign-magnitude with
    the sign on bit 11**, not two's complement::

        wire = (sign << 11) | magnitude

    So ``+1073`` goes on the wire as ``1073``, and ``-1073`` as
    ``2048 + 1073 == 3121``. A two's-complement encoder — the obvious wrong
    guess, and the one every stdlib instinct pushes you toward — would send
    ``-1073`` as ``0xFBCF`` (64463); the servo would read that as a nonsense
    magnitude and every position it reported afterwards would be garbage. That
    is why this function exists at module scope with its own round-trip test
    rather than being three inline lines inside :meth:`FeetechBus.write_offset`.

    The bit index is deliberately **not** a parameter. Bit 11 belongs to this
    register alone: ``Present_Position`` is sign-magnitude on bit **15**, and
    ``Present_Load`` carries a *direction* flag on bit **10**
    (:func:`load_magnitude`). A general-purpose codec would invite applying the
    wrong width to the wrong register.

    Parameters
    ----------
    offset:
        Signed offset in ticks, ``[-2047, +2047]``.

    Returns
    -------
    int
        The 12-bit wire value to write to addr 31.

    Raises
    ------
    CliError(EXIT_USER_ERROR)
        If ``abs(offset) > 2047`` (:data:`OFFSET_MAX_MAGNITUDE`). Rejected
        loudly rather than truncated: ``2048`` truncated to 11 bits is ``0``
        with the sign bit set — i.e. the servo would be told "negative zero"
        when the caller asked for a two-thousand-tick shift, and the joint's
        frame would be silently wrong forever after.
    """
    magnitude = abs(offset)
    if magnitude > OFFSET_MAX_MAGNITUDE:
        raise CliError(
            code=EXIT_USER_ERROR,
            message=(
                f"Encoder offset {offset} is out of range: magnitude {magnitude} "
                f"exceeds {OFFSET_MAX_MAGNITUDE} (the STS3215 offset register is "
                f"sign-magnitude on bit {OFFSET_SIGN_BIT}, so it can only hold "
                f"-{OFFSET_MAX_MAGNITUDE}..+{OFFSET_MAX_MAGNITUDE})."
            ),
            remediation=(
                f"Pass an offset between -{OFFSET_MAX_MAGNITUDE} and "
                f"+{OFFSET_MAX_MAGNITUDE}. Any seam placement in 0..2047 is "
                "reachable; only raw 2048 is not expressible."
            ),
        )
    if offset < 0:
        return _OFFSET_SIGN_MASK | magnitude
    return magnitude


def decode_offset(raw: int) -> int:
    """Decode a raw addr-31 register read into a signed offset.

    The inverse of :func:`encode_offset`. Bit 11 set means negative; bits 0-10
    are the magnitude. Bits 12-15 of the 2-byte read are not part of the
    register and are masked off (mirroring :meth:`FeetechBus.read_position`'s
    ``& 0x0FFF``), so a stray high bit from a noisy read is discarded rather
    than decoded into a nonsense magnitude.

    Returning the raw value instead of decoding is the failure this guards: a
    servo holding ``-1073`` reports ``3121``, which reads as an entirely
    plausible *positive* offset. Nothing downstream could tell the difference,
    and every position comparison in the joint's frame would be off by 2146
    ticks.

    Note the encoding is not injective: wire ``0`` and wire ``2048``
    ("negative zero") both decode to ``0``. Both are values a servo can
    genuinely hold, so the round-trip property is ``decode(encode(x)) == x`` —
    never the reverse. :func:`encode_offset` always emits the canonical ``+0``.
    """
    value = int(raw) & _OFFSET_REGISTER_MASK
    magnitude = value & _OFFSET_MAGNITUDE_MASK
    if value & _OFFSET_SIGN_MASK:
        return -magnitude
    return magnitude


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

    @abc.abstractmethod
    def read_offset(self, motor: int) -> int:
        """Return *motor*'s encoder offset (``Ofs``/``Homing_Offset``, addr 31) as a SIGNED int.

        The value is decoded from the register's sign-magnitude encoding
        (:func:`decode_offset`), so a servo holding ``-1073`` — which reports
        ``3121`` on the wire — returns ``-1073``. The wire encoding never
        escapes this module.

        Read-only: this touches no other register and commands no motion.

        Returns
        -------
        int
            Signed offset in encoder ticks, ``[-2047, +2047]``. ``0`` on a
            factory servo.

        Raises
        ------
        CliError(EXIT_ENV_ERROR)
            If the bus has not been opened or the read fails.
        """

    @abc.abstractmethod
    def write_offset(self, motor: int, offset: int) -> None:
        """Write *motor*'s encoder offset to EEPROM (``Ofs``/``Homing_Offset``, addr 31).

        This re-zeros the joint: afterwards the servo reports
        ``Present_Position = Actual_Position - offset``. It is the fix for
        issue #35 — see :data:`ADDR_HOMING_OFFSET`.

        **This is a persistent EEPROM write.** Implementations MUST:

        1. Reject an unrepresentable *offset* BEFORE touching the bus
           (:func:`encode_offset`), so a bad value has no side effects at all.
        2. Disable torque (addr 40) FIRST. With torque on, the servo instantly
           re-interprets its own position against the new offset and could
           LURCH toward its standing goal. (Inferred, not documented — treated
           as a hard safety rule.)
        3. Open the EEPROM Lock (addr 55 -> 0), write ONLY addr 31, then
           re-lock (addr 55 -> 1) — including on the failure path, so a failed
           write never strands the motor unlocked. Skipping the unlock makes
           the write REVERT on the next power-cycle (PR #21).
        4. Write **address 31 and nothing else**. In particular NOT
           ``Min_Position_Limit`` (9) / ``Max_Position_Limit`` (11), which
           LeRobot's ``write_calibration`` also writes: in servo mode those
           CLAMP goals, and narrowing them would shrink the very reachable set
           the re-zero exists to recover.

        Parameters
        ----------
        motor:
            Motor ID (1-indexed, matching the Feetech servo ID).
        offset:
            Signed offset in encoder ticks, ``[-2047, +2047]``.

        Raises
        ------
        CliError(EXIT_USER_ERROR)
            If ``abs(offset) > 2047`` — before any register is touched.
        CliError(EXIT_ENV_ERROR)
            If the bus has not been opened, or any of the writes fail.
        OverloadError
            If the motor is latched in overload (it tags every response with
            status bit 5, including the torque-off write's). The EEPROM is not
            opened in that case; call :meth:`clear_overload` first.
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

    def _sdk_read(
        self,
        fn: "Callable[..., tuple[int, int, int]]",
        motor: int,
        addr: int,
        action: str,
    ) -> "tuple[int, int, int]":
        """Call an SDK read function; convert any bare exception it raises into a CliError.

        *fn* is one of ``self._packet_handler.read1ByteTxRx`` /
        ``read2ByteTxRx``. On a healthy exchange it returns
        ``(value, result, error)`` and this method is a passthrough. On a
        SHORT or CORRUPT packet the vendor SDK can instead *raise* — observed
        live on this session's hardware, mid-session, on a bus that was
        otherwise perfectly healthy::

            File ".../scservo_sdk/protocol_packet_handler.py", line 326, in read2ByteTxRx
                data_read = SCS_MAKEWORD(data[0], data[1]) if (result == COMM_SUCCESS) else 0
                                          ~~~~^^^
            IndexError: list index out of range

        The SDK checks ``result == COMM_SUCCESS`` and only THEN indexes
        ``data[0]``/``data[1]`` — so a response buffer shorter than 2 bytes
        raises ``IndexError`` from *inside* ``read2ByteTxRx``, before this
        bus's own ``result != 0 or error != 0`` check downstream ever gets a
        chance to run. Nothing about the bus itself is wrong; the packet was
        simply dropped or truncated in flight — the read1ByteTxRx path has the
        identical shape and the identical exposure.

        This repo's one hard rule (``arm101/cli/_errors.py`` — no Python
        traceback ever leaks to stderr) applies just as much to an exception
        the *vendor SDK* raises as to one this codebase raises itself, so any
        exception that is not already a :class:`CliError` is caught here and
        converted into one. The caller (:meth:`_retry_read`) treats it exactly
        like a comms failure and retries — a dropped/corrupt packet on a
        healthy bus is transient, and a read is idempotent, so retrying costs
        nothing wrong.

        Raises
        ------
        CliError(EXIT_ENV_ERROR)
            If *fn* raises anything other than a :class:`CliError`.
        """
        from arm101.cli._errors import EXIT_ENV_ERROR, CliError

        try:
            return fn(self._port_handler, motor, addr)
        except CliError:
            raise
        except Exception as exc:  # noqa: BLE001 - the SDK can raise ~anything internally
            raise CliError(
                code=EXIT_ENV_ERROR,
                message=(
                    f"{action}: the SDK raised {type(exc).__name__} ({exc}) reading "
                    f"register {addr} on motor {motor} — the packet was dropped or "
                    "corrupt."
                ),
                remediation=(
                    "A single dropped/corrupt packet is normal on an otherwise "
                    "healthy bus; the read is idempotent, so it is safe to retry."
                ),
            ) from exc

    def _retry_read(self, attempt: "object") -> int:
        """Call *attempt* (a zero-arg callable returning ``int``) with bounded retry.

        A read is idempotent, so retrying a dropped/corrupt packet costs
        nothing wrong — every ad-hoc probe script written during this
        session's live hardware run ended up hand-rolling exactly this loop
        (see :data:`_READ_RETRY_ATTEMPTS`). Only the LAST failure ever
        propagates; every earlier one is swallowed after a short delay
        (:data:`_READ_RETRY_DELAY_SECONDS`).

        :class:`OverloadError` is the one failure NEVER retried: it is a
        real, latched servo fault (status error bit 5), not a dropped packet,
        and callers (``gentle_move``, ``arm rezero``) rely on catching it
        *immediately* to run their own overload recovery — silently eating an
        attempt or two on it here would only delay that recovery, and the
        latch will not clear itself between attempts anyway.

        Raises
        ------
        OverloadError
            Immediately, on the first occurrence — never retried.
        CliError(EXIT_ENV_ERROR)
            If every attempt fails for any other reason.
        """
        last_error: "CliError | None" = None
        for attempt_number in range(1, _READ_RETRY_ATTEMPTS + 1):
            try:
                return attempt()  # type: ignore[operator]
            except OverloadError:
                raise
            except CliError as exc:
                last_error = exc
                if attempt_number < _READ_RETRY_ATTEMPTS:
                    time.sleep(_READ_RETRY_DELAY_SECONDS)
        assert (
            last_error is not None
        )  # pragma: no cover - loop always assigns before falling through
        raise last_error

    def _retry_idempotent_write(self, attempt: "object", motor: int, action: str) -> None:
        """Call *attempt* (a zero-arg callable returning ``(result, error)``) with bounded retry.

        For writes whose SECOND application is indistinguishable from their first:
        goal position, torque enable, torque limit, acceleration, goal speed, the
        Lock byte. See :data:`_IDEMPOTENT_WRITE_ATTEMPTS` for why idempotence — not
        RAM-vs-EEPROM — is the line, and why id/baud writes still retry nothing.

        This exists because the bus really does drop write ACKs (hardware,
        2026-07-13). Reads have been retried since :data:`_READ_RETRY_ATTEMPTS`
        landed, which is exactly why nobody noticed: every drop on a *write*
        surfaced as a hard failure and killed the run.

        Raises
        ------
        OverloadError
            Immediately, on the first occurrence — never retried. A latched servo
            is a real fault, not a dropped packet, and it will not clear itself
            between attempts. Callers rely on seeing it at once.
        CliError(EXIT_ENV_ERROR)
            If every attempt fails for any other reason. Only the LAST failure
            propagates.
        """
        last_error: "CliError | None" = None
        for attempt_number in range(1, _IDEMPOTENT_WRITE_ATTEMPTS + 1):
            result, error = attempt()  # type: ignore[operator, misc]
            if result == 0 and error == 0:
                return
            exc = self._status_error(motor, result, error, action)
            if isinstance(exc, OverloadError):
                raise exc
            last_error = exc
            if attempt_number < _IDEMPOTENT_WRITE_ATTEMPTS:
                time.sleep(_READ_RETRY_DELAY_SECONDS)
        assert (
            last_error is not None
        )  # pragma: no cover - loop always assigns before falling through
        raise last_error

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

        Re-locking also SETTLES (:data:`_EEPROM_SETTLE_SECONDS`) before
        returning — see that constant for the hardware measurement motivating
        it. A caller that turns around and reads a register straight back can
        otherwise catch the servo mid-recovery from the EEPROM write and be
        handed a plausible-looking but WRONG value (observed: a position
        register that genuinely held 3387 read back as 0 roughly 0.2 s after
        the write that changed nothing about it). The settle applies only to
        the CLOSING write (``locked=True``) — the one that marks the end of an
        EEPROM write dance, success or best-effort failure path alike — never
        to the opening unlock, which is followed by more EEPROM traffic, not
        by a caller's read.
        """
        self._require_open()
        result, error = self._packet_handler.write1ByteTxRx(  # type: ignore[union-attr]
            self._port_handler, motor, ADDR_LOCK, 1 if locked else 0
        )
        if result != 0 or error != 0:
            state = "re-lock" if locked else "unlock"
            raise self._status_error(
                motor, result, error, f"Failed to {state} EEPROM for motor {motor}"
            )
        if locked:
            time.sleep(_EEPROM_SETTLE_SECONDS)

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

        The value the servo reports is already offset-corrected:
        ``Present_Position = Actual_Position - Homing_Offset`` (see
        :data:`ADDR_HOMING_OFFSET`). This method reads it as-is — it does not
        add the offset back, because the corrected frame IS the frame goals are
        commanded in.

        A SHORT/CORRUPT packet on the wire (the SDK's ``read2ByteTxRx`` can
        raise a bare ``IndexError`` rather than returning a clean failed
        ``result`` — see :meth:`_sdk_read`) or a comms failure is retried up
        to :data:`_READ_RETRY_ATTEMPTS` times before raising (see
        :meth:`_retry_read`); :class:`OverloadError` is the one failure that
        is never retried.

        Returns
        -------
        int
            12-bit encoder tick in ``[0, 4095]``, as reported.
        """
        self._require_open()

        # STS3215 present-position address = 56, 2 bytes.
        _ADDR_PRESENT_POSITION = 56

        def _attempt() -> int:
            action = f"Read position failed for motor {motor}"
            value, result, error = self._sdk_read(
                self._packet_handler.read2ByteTxRx,  # type: ignore[union-attr]
                motor,
                _ADDR_PRESENT_POSITION,
                action,
            )
            if result != 0 or error != 0:  # non-zero comm result or servo error = failure
                raise self._status_error(motor, result, error, action)
            # Mask to 12 bits. DELIBERATELY unchanged by the offset work (issue #35):
            # Present_Position is itself sign-magnitude on bit 15, so this mask would
            # SWALLOW a negative reading. That only matters if the firmware reports the
            # corrected position as an unwrapped signed subtraction rather than reducing
            # it modulo 4096 — the one unproven assumption behind the whole re-zero
            # (docs/spikes/sts3215-offset-register.md §4). Every source points at the
            # modular reading, in which case the value is always in [0, 4095] and this
            # mask is a no-op. If the hardware test (step 10) says otherwise, the re-zero
            # does not work at all and THIS mask is the next thing to fix.
            return int(value) & 0x0FFF

        return self._retry_read(_attempt)

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
        # EEPROM encoder offset (addr 31). Surfaced so `arm read` can SHOW the
        # re-zero before anyone writes it (issue #35) — the register that most
        # needs inspecting would otherwise be the one register you cannot see.
        # NOTE: read_info DECODES this one (see below); every other entry here
        # is the raw register value.
        "homing_offset": (ADDR_HOMING_OFFSET, 2),
    }

    def _read_register(self, motor: int, addr: int, length: int) -> int:
        """Read a 1- or 2-byte register with bounded retry; raise CliError on failure.

        Retries up to :data:`_READ_RETRY_ATTEMPTS` times on ANY read failure —
        a comms-level ``result``/``error`` failure, or a bare exception the
        SDK raised internally (:meth:`_sdk_read` — e.g. the ``IndexError`` a
        short/corrupt packet triggers inside ``read2ByteTxRx``) — except
        :class:`OverloadError`, which is a latched servo fault rather than a
        dropped packet and is never retried (:meth:`_retry_read`).
        """

        def _attempt() -> int:
            reader = (
                self._packet_handler.read1ByteTxRx  # type: ignore[union-attr]
                if length == 1
                else self._packet_handler.read2ByteTxRx  # type: ignore[union-attr]
            )
            action = f"Read of register {addr} failed for motor {motor}"
            value, result, error = self._sdk_read(reader, motor, addr, action)
            if result != 0 or error != 0:
                raise self._status_error(motor, result, error, action)
            return int(value)

        return self._retry_read(_attempt)

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
        """Return a full read-only register snapshot for *motor*.

        Every value is the raw register **except** ``homing_offset``, which is
        decoded from its sign-magnitude wire form (:func:`decode_offset`) into a
        signed int. That single exception is deliberate: the raw form is a trap.
        A servo holding an offset of ``-1073`` reports ``3121``, which reads as
        a perfectly plausible *positive* offset — so handing the raw value to
        callers would let ``arm read`` display a confident, wrong number to the
        human deciding whether to re-zero the joint. ``read_info["homing_offset"]``
        and :meth:`read_offset` therefore return the same one number with the
        same one meaning, and the wire encoding never leaves this module.

        (Contrast ``present_load``, which IS raw, direction bit and all — see
        :func:`load_magnitude`. The difference is that a raw load is merely
        *large*; a raw offset is *plausible*.)
        """
        self._require_open()
        snapshot = {
            name: self._read_register(motor, addr, length)
            for name, (addr, length) in self._INFO_REGISTERS.items()
        }
        snapshot["homing_offset"] = decode_offset(snapshot["homing_offset"])
        return snapshot

    def enable_torque(self, motor: int, on: bool) -> None:
        """Enable or disable torque for *motor*.

        STS3215 Torque_Enable register: address 40, 1 byte.
        Write 1 to enable, 0 to disable.
        """
        self._require_open()

        state = "enable" if on else "disable"

        def _attempt() -> "tuple[int, int]":
            return self._packet_handler.write1ByteTxRx(  # type: ignore[union-attr]
                self._port_handler, motor, ADDR_TORQUE_ENABLE, 1 if on else 0
            )

        # Idempotent: torque on is torque on, however many times we say it.
        self._retry_idempotent_write(_attempt, motor, f"Failed to {state} torque for motor {motor}")

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

        def _attempt() -> "tuple[int, int]":
            return self._packet_handler.write2ByteTxRx(  # type: ignore[union-attr]
                self._port_handler, motor, _ADDR_GOAL_POSITION, position
            )

        # Idempotent: the servo ends up wanting `position` whether we say it once
        # or twice. On a bus that drops write ACKs, saying it once is an outage.
        self._retry_idempotent_write(
            _attempt, motor, f"Write goal position failed for motor {motor}"
        )

    def read_lock(self, motor: int) -> int:
        """Read the STS3215 Lock register (address 55, 1 byte) for *motor*.

        Bounded-retried like every other read on this bus — see
        :meth:`_read_register` / :meth:`_retry_read`.

        Returns
        -------
        int
            Lock register value (0=unlocked, 1=locked).
        """
        self._require_open()

        def _attempt() -> int:
            action = f"Read lock register failed for motor {motor}"
            value, result, error = self._sdk_read(
                self._packet_handler.read1ByteTxRx,  # type: ignore[union-attr]
                motor,
                ADDR_LOCK,
                action,
            )
            if result != 0 or error != 0:
                raise self._status_error(motor, result, error, action)
            return int(value)

        return self._retry_read(_attempt)

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

        def _attempt() -> "tuple[int, int]":
            return self._packet_handler.write1ByteTxRx(  # type: ignore[union-attr]
                self._port_handler, motor, _ADDR_ACCELERATION, value
            )

        # Idempotent: the register ends up holding `value` either way.
        self._retry_idempotent_write(
            _attempt, motor, f"Write acceleration failed for motor {motor}"
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

        def _attempt() -> "tuple[int, int]":
            return self._packet_handler.write2ByteTxRx(  # type: ignore[union-attr]
                self._port_handler, motor, _ADDR_GOAL_SPEED, value
            )

        # Idempotent: the register ends up holding `value` either way.
        self._retry_idempotent_write(_attempt, motor, f"Write goal speed failed for motor {motor}")

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

        def _attempt() -> "tuple[int, int]":
            return self._packet_handler.write2ByteTxRx(  # type: ignore[union-attr]
                self._port_handler, motor, _ADDR_TORQUE_LIMIT, value
            )

        # Idempotent: the register ends up holding `value` either way.
        self._retry_idempotent_write(
            _attempt, motor, f"Write torque limit failed for motor {motor}"
        )

    def clear_overload(self, motor: int) -> None:
        """Disable torque for *motor* (Torque_Enable=0, addr 40) to clear a latched overload.

        Overload-TOLERANT, unlike :meth:`enable_torque`. While a motor is
        latched in overload the servo tags *every* packet response with the
        overload bit (``0x20``) — including the response to this very
        torque-disable write — so routing through ``enable_torque(False)``
        would re-raise :class:`OverloadError` and defeat the recovery it is
        meant to perform. Since disabling torque is precisely how the latch is
        cleared, an overload bit on THIS write's response is expected and
        treated as success. A genuine comms failure (nonzero ``result``) or any
        *other* status error bit still raises.

        **VERIFIED, because the latch does not clear instantly.** The write returning
        cleanly does not mean the servo has dropped the flag: measured on the follower
        (2026-07-13, ``wrist_flex``), a read issued straight after a successful
        torque-disable STILL came back tagged ``error=32``, and the same read after
        ~250 ms came back clean. So this polls the servo until the overload bit is
        actually gone rather than trusting the write — the same rule ``write_offset``
        already follows: *the register is the arbiter, not the status byte.*

        Assuming it had worked was not harmless. ``gentle_move`` clears the latch and
        then immediately re-reads the joint's position; that read hit a servo that was
        still latched, was suppressed, and the position was lost. Worse, the NEXT probe
        began on a motor that had never really recovered — which is exactly how
        ``wrist_flex``'s high-end measurement came back as 5 ticks of travel and a
        TORQUE_LIMITED verdict that described the leftover fault, not the joint.

        Raises
        ------
        CliError(EXIT_ENV_ERROR)
            If the latch is still set after every attempt. A motor that will not clear
            is a real fault, and saying so is better than handing back a servo that
            silently poisons every reading taken after it.
        """
        self._require_open()

        result, error = self._packet_handler.write1ByteTxRx(  # type: ignore[union-attr]
            self._port_handler, motor, ADDR_TORQUE_ENABLE, 0
        )
        # Mask off the overload bit — it is the very flag we are clearing.
        residual = error & ~_OVERLOAD_BIT
        if result != 0 or residual != 0:
            raise self._status_error(
                motor, result, residual, f"Failed to clear overload for motor {motor}"
            )

        for attempt_number in range(1, _OVERLOAD_CLEAR_ATTEMPTS + 1):
            _, probe_result, probe_error = self._packet_handler.read1ByteTxRx(  # type: ignore
                self._port_handler, motor, ADDR_TORQUE_ENABLE
            )
            if probe_result == 0 and not is_overload(probe_error):
                return
            if attempt_number < _OVERLOAD_CLEAR_ATTEMPTS:
                time.sleep(_OVERLOAD_CLEAR_DELAY_SECONDS)

        raise self._status_error(
            motor,
            0,
            _OVERLOAD_BIT,
            f"Motor {motor} is still latched in overload after a torque-disable and "
            f"{_OVERLOAD_CLEAR_ATTEMPTS} checks over "
            f"{_OVERLOAD_CLEAR_ATTEMPTS * _OVERLOAD_CLEAR_DELAY_SECONDS:.2f}s",
        )

    # ------------------------------------------------------------------
    # Encoder offset (Ofs / Homing_Offset) — EEPROM addr 31
    # ------------------------------------------------------------------

    def read_offset(self, motor: int) -> int:
        """Read *motor*'s encoder offset (addr 31, 2 bytes) and decode it to a signed int.

        Read-only — the counterpart to :meth:`write_offset` that lets a human
        (or ``arm read``) inspect the current re-zero without risking one. See
        :data:`ADDR_HOMING_OFFSET` for what the register does and why issue #35
        needs it.
        """
        self._require_open()
        return decode_offset(self._read_register(motor, ADDR_HOMING_OFFSET, 2))

    def write_offset(self, motor: int, offset: int) -> None:
        """Write *motor*'s encoder offset to EEPROM (addr 31, 2 bytes).

        The contract is :meth:`MotorBus.write_offset`; this is how it is met.

        The exact wire sequence, and why each step is where it is::

            (40, 0)     torque OFF
            (55, 0)     unlock EEPROM
            (31, wire)  the offset — SIGN-MAGNITUDE encoded, and nothing else
            (55, 1)     re-lock EEPROM

        **Validation first, wire traffic second.** :func:`encode_offset` runs
        before anything is sent, so an out-of-range offset leaves the arm
        exactly as it found it: torque still on, EEPROM still locked. Rejecting
        after the torque-off would silently relax a joint as a side effect of a
        *failed* call.

        **Torque off first.** With torque enabled the servo instantly
        re-interprets its own present position against the newly written offset;
        with a standing goal still latched, its position error jumps by the
        offset in one step and it can LURCH. This is inferred rather than
        documented, and is treated as a hard safety rule — enforced here in the
        primitive, not left to the caller to remember. (It also means a motor
        latched in overload raises :class:`OverloadError` from this very write,
        BEFORE the EEPROM is opened — call :meth:`clear_overload` first.)

        **The Lock dance is mandatory, not decorative.** PR #21 of this repo
        exists because id/baud writes appeared to work, read back correctly, and
        then silently REVERTED on the next power-cycle: on fw 3.10 a write while
        Lock=1 updates the live register but is never committed to EEPROM.
        Addr 31 is in the same EEPROM region and fails the same way. The re-lock
        is attempted on the failure path too (best-effort, preserving the
        original error), because a motor stranded at Lock=0 is a motor whose id,
        baud and offset any stray write can clobber.

        **Only addr 31.** LeRobot's ``write_calibration`` also writes
        ``Min_Position_Limit`` (9) and ``Max_Position_Limit`` (11). In servo
        mode the firmware CLAMPS goals to that window — so copying LeRobot
        wholesale would narrow the reachable set that this re-zero exists to
        recover. Our factory limits are the wide-open 0/4095 and stay that way.
        ``tests/test_bus_offset.py`` pins the write surface to an exact
        allow-list, so adding a register here is a deliberate act, not a slip.

        **A failed write is not a write that failed to land — VERIFY, then retry.**
        Measured on the follower, 2026-07-13: an offset write to addr 31 returns
        ``result=-6`` (RX timeout) with real frequency, and every single time the
        servo had *applied* it — the packet went out, the EEPROM took it, only the
        ACK was lost. Reporting that as a failure aborted a probe that had in fact
        worked, and left the joint wearing a borrowed calibration. Meanwhile a
        failure in the torque-off *before* the write (``result=-7`` observed) never
        landed at all. The two are indistinguishable from the status byte and mean
        exactly opposite things.

        So the honest state after a bad status is **unknown**, not failed — and a
        READ settles it. A read is idempotent and already retried
        (:data:`_READ_RETRY_ATTEMPTS`), so it costs nothing to ask. That is what
        turns a blind retry (unsafe: a "failed" write may have landed, and
        re-applying it is a second EEPROM write on a servo whose state we do not
        know) into a **verified** one (safe: we have just read that it did *not*
        land). Only what verification proves absent is retried.
        """
        self._require_open()

        # Encode BEFORE any wire traffic: a rejected offset must have no
        # side effects whatsoever (no torque-off, no unlock).
        wire = encode_offset(offset)

        last_error: "CliError | None" = None
        for _attempt in range(_OFFSET_WRITE_ATTEMPTS):
            try:
                self._write_offset_once(motor, wire)
            except OverloadError:
                # A latched motor is not a flaky bus. Retrying pushes a servo
                # that is already in fault; the caller must clear_overload first.
                raise
            except CliError as exc:
                last_error = exc

            # THE ARBITER. Not the status byte — the register itself.
            try:
                if self.read_offset(motor) == offset:
                    return
            except CliError as exc:  # the bus is too far gone to even ask
                last_error = exc

        raise last_error or self._status_error(
            motor, 0, 0, f"Write encoder offset failed for motor {motor}"
        )

    def _write_offset_once(self, motor: int, wire: int) -> None:
        """One torque-off -> unlock -> write -> re-lock attempt. No verification."""
        # Safety: the servo must be limp before its frame of reference moves
        # under it. Raises (incl. OverloadError) if the motor will not relax —
        # in which case we have not unlocked anything and never will.
        self.enable_torque(motor, False)

        # Persistence: without the unlock the write reverts on power-cycle.
        self._set_lock(motor, False)
        try:
            result, error = self._packet_handler.write2ByteTxRx(  # type: ignore[union-attr]
                self._port_handler, motor, ADDR_HOMING_OFFSET, wire
            )
            if result != 0 or error != 0:
                raise self._status_error(
                    motor, result, error, f"Write encoder offset failed for motor {motor}"
                )
        except BaseException:
            # Best-effort re-lock so a failed write never strands the motor at
            # Lock=0; if the re-lock itself fails, preserve the ORIGINAL error —
            # the operator needs to know why the OFFSET write failed, not to be
            # sent hunting for a Lock problem that is a symptom, not the cause.
            with contextlib.suppress(Exception):
                self._set_lock(motor, True)
            raise
        else:
            self._set_lock(motor, True)


# ---------------------------------------------------------------------------
# In-memory FakeBus — for tests and offline development
# ---------------------------------------------------------------------------


class FakeBus(MotorBus):
    """In-memory motor bus for tests and offline development.

    No hardware or third-party packages required.

    Parameters
    ----------
    positions:
        Optional mapping of motor-id → initial **actual** encoder position (the
        raw magnetic count on the shaft).  Motors not present in the dict use
        :data:`_DEFAULT_POSITION` (2048).  What the bus *reports* is this minus
        the motor's ``Homing_Offset`` — see :meth:`_reported_position`; with the
        default zero offset the two are identical.
    offsets:
        Optional mapping of motor-id → **signed** encoder offset (``Ofs`` /
        ``Homing_Offset``, addr 31).  Motors not present read ``0`` (the factory
        value).  Seeds a servo that has already been re-zeroed, without having
        to drive :meth:`write_offset` first.
    offset_wraps:
        Whether the corrected position is reduced modulo 4096 (the seam *moves*)
        or reported as a plain signed subtraction (the seam *stays*).  Defaults
        to ``True``; see :meth:`_reported_position` for why this is a switch and
        not a hard-coded assumption.

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
    offset_writes:
        List of dicts, one per :meth:`write_offset` call, in call order::

            {"motor": int, "offset": int}

        The **signed** offset, as the caller asked for it.  The sign-magnitude
        *wire* value lands in :attr:`register_writes` at addr 31, so a test can
        pin both the intent and the encoding.
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
        offsets: "dict[int, int] | None" = None,
        offset_wraps: bool = True,
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
        # Signed encoder offsets (addr 31). Motors absent from the dict hold the
        # factory 0, i.e. reported position == actual position.
        self._offsets: dict[int, int] = dict(offsets) if offsets else {}
        self.offset_wraps: bool = offset_wraps
        self.offset_writes: list[dict[str, int]] = []
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

    def _set_lock_fake(self, motor: int, locked: bool) -> None:
        """Model the EEPROM Lock register (addr 55) write, and record it.

        The fake performs the unlock -> write -> re-lock dance for real (as a
        recorded register write and a mutation of :attr:`lock_register`) rather
        than skipping it, because a fake that omitted the dance would let a test
        "prove" an EEPROM write that on hardware reads back correctly and then
        silently REVERTS on the next power-cycle — the exact failure PR #21 was
        written to kill.
        """
        self.lock_register = 1 if locked else 0
        self._record_write(motor, ADDR_LOCK, self.lock_register)

    # ------------------------------------------------------------------
    # The encoder offset's EFFECT on what the servo reports
    # ------------------------------------------------------------------

    def _reported_position(self, motor: int, actual: int) -> int:
        """Apply *motor*'s ``Homing_Offset`` to an *actual* count, as the servo does.

        ``Present_Position = Actual_Position - Homing_Offset``. This is the
        single funnel through which every position this fake REPORTS passes
        (:meth:`read_position` and :meth:`read_info` alike), because on the wire
        they are the same register (addr 56). A fake in which one of them
        applied the offset and the other did not would model a servo that does
        not exist — and would let a test pass against behaviour no hardware can
        produce. (The same lesson as ``clear_overload``/``enable_torque``: one
        wire operation, one overridable method.)

        With no offset written this is the exact identity — not
        ``actual % 4096`` — so a fixture that seeds a position outside
        ``[0, 4095]`` still reads back byte-for-byte what it was given.

        **The modulo is the fake's one unproven assumption, and it is a switch.**
        With ``offset_wraps=True`` (the default) the corrected value is reduced
        modulo 4096, so the 4095->0 seam RELOCATES to ``raw == offset`` — which
        is the entire premise of the issue-#35 re-zero. The alternative reading
        (``offset_wraps=False``) is a plain signed subtraction: the offset then
        merely *relabels* positions, the discontinuity stays pinned to the
        physical angle where the magnet rolls over, and the re-zero achieves
        nothing. Every source and LeRobot's shipped SO-101 calibration imply the
        modular reading, but no primary Feetech source states the firmware's
        formula (see ``docs/spikes/sts3215-offset-register.md`` §4), so the
        other branch is modelled rather than assumed away. Hardware test step 10
        settles it.
        """
        offset = self._offsets.get(motor, 0)
        if offset == 0:
            return actual
        reported = actual - offset
        return reported % ENCODER_RESOLUTION if self.offset_wraps else reported

    def _actual_position(self, motor: int, reported: int) -> int:
        """Inverse of :meth:`_reported_position`: corrected frame -> raw encoder count.

        ``Actual = Present + Homing_Offset``. Needed because a *goal* is
        commanded in the same corrected frame the servo reports in — if goals
        and feedback lived in different frames the re-zero would be worse than
        useless — so a fake that models the shaft in raw counts has to convert
        an incoming goal back. Used by ``ServoModelBus``; harmless (identity) at
        the default zero offset.
        """
        offset = self._offsets.get(motor, 0)
        if offset == 0:
            return reported
        actual = reported + offset
        return actual % ENCODER_RESOLUTION if self.offset_wraps else actual

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
        """Return the position *motor* REPORTS: its actual count minus its offset.

        Defaults to 2048 (:data:`_DEFAULT_POSITION`) with no offset written, in
        which case reported == actual. See :meth:`_reported_position`.

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
        return self._reported_position(motor, self._positions.get(motor, _DEFAULT_POSITION))

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
        self._record_write(motor, ADDR_TORQUE_ENABLE, 1 if on else 0)

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
        12.0 V, 38 deg C); ``present_position`` reflects the positions dict
        (shifted by any written offset, exactly as the servo reports it — see
        :meth:`_reported_position`), ``homing_offset`` is the **signed** offset
        (matching :meth:`FeetechBus.read_info`, which decodes it), and anything
        in the ``info`` constructor override wins.
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
            "present_position": self._reported_position(
                motor, self._positions.get(motor, _DEFAULT_POSITION)
            ),
            "present_speed": 0,
            "present_load": 0,
            "present_voltage": 120,
            "present_temperature": 38,
            "lock_register": self.lock_register,
            "homing_offset": self._offsets.get(motor, 0),
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

        Routed through :meth:`enable_torque` **on purpose**, because on the wire
        the two are the SAME operation: :class:`FeetechBus` implements both as a
        single ``write1ByteTxRx(motor, 40, 0)`` and they differ only in how they
        treat the *response* — ``clear_overload`` masks off the overload bit
        (0x20), ``enable_torque`` raises :class:`OverloadError` on it. A fake in
        which a subclass could intercept ``enable_torque`` and silently MISS the
        identical byte written by ``clear_overload`` would model a bus that does
        not exist, and would let a test "prove" a torque-disable never happened
        when on real hardware it did. Every torque-disable this fake performs
        therefore passes through one overridable method.

        Overload tolerance is preserved: an :class:`OverloadError` out of
        ``enable_torque`` — whether from the simulation seam
        (:meth:`_tick_and_maybe_overload`) or from a subclass modelling a servo
        latched in overload — is caught and treated as SUCCESS, exactly as
        ``FeetechBus.clear_overload`` masks bit 5 off its own response. The write
        genuinely lands on the wire in that case; the overload bit is merely the
        latch being reported one last time on the way out, and disabling torque
        is precisely what clears it. Any OTHER failure (a comms error, a
        subclass modelling a dead port) still propagates — the caller must be
        able to learn that the motor did NOT go limp.

        On completion it also DISARMS the overload-simulation seam
        (``overload_after_ops = None``), letting a test drive the full
        catch-and-recover flow: arm the seam, hit ``OverloadError``, call
        ``clear_overload()``, then resume with normal (non-raising) calls.

        Raises
        ------
        CliError(EXIT_ENV_ERROR)
            If the bus has not been opened.
        """
        self._require_open_fake()
        try:
            self.enable_torque(motor, False)
        except OverloadError:
            # The latch rides on the response to the very write that clears it.
            # The byte DID land — record it, mirroring FeetechBus's masking.
            self.torque_writes.append({"motor": motor, "on": False})
            self._record_write(motor, ADDR_TORQUE_ENABLE, 0)
        self.overload_after_ops = None

    # ------------------------------------------------------------------
    # Encoder offset (Ofs / Homing_Offset) — EEPROM addr 31
    # ------------------------------------------------------------------

    def read_offset(self, motor: int) -> int:
        """Return the signed encoder offset for *motor* (0 if never written).

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
        return self._offsets.get(motor, 0)

    def write_offset(self, motor: int, offset: int) -> None:
        """Record an encoder-offset write, and apply its effect to reported positions.

        Faithfully reproduces :meth:`FeetechBus.write_offset`'s wire sequence —
        torque-off (addr 40) -> unlock (55 -> 0) -> offset (31) -> re-lock
        (55 -> 1) — into :attr:`register_writes`, with the SIGN-MAGNITUDE wire
        value at addr 31, so a test asserting the sequence against the fake is
        asserting something true of the hardware. Three properties are load-bearing
        and each has a test:

        * The offset is validated (:func:`encode_offset`) **before any wire
          traffic**, so a rejected offset does not leave the joint limp.
        * The torque-off goes through :meth:`enable_torque` — one wire byte, one
          overridable method — so a subclass that intercepts torque writes sees
          this one too.
        * A failed EEPROM write still re-locks, so the fake can never "prove" a
          recovery path that leaves a real motor stranded at Lock=0.

        Once written, the offset changes what :meth:`read_position` and
        :meth:`read_info` REPORT (``Present = Actual - Ofs``); the simulated
        shaft itself does not move. See :meth:`_reported_position`.

        Raises
        ------
        CliError(EXIT_USER_ERROR)
            If ``abs(offset) > 2047`` — before anything is written.
        CliError(EXIT_ENV_ERROR)
            If the bus has not been opened.
        OverloadError
            If the overload-simulation seam is armed; on the torque-off it
            fires before the EEPROM is opened, on the offset write it fires
            after — and the re-lock still happens.
        """
        self._require_open_fake()

        # Validate first: a rejected offset must have zero side effects.
        wire = encode_offset(offset)

        # Safety rule, enforced not merely documented: the servo must be limp
        # before its frame of reference shifts under it.
        self.enable_torque(motor, False)

        self._set_lock_fake(motor, False)
        try:
            self._tick_and_maybe_overload(motor)  # the EEPROM write itself
            self._offsets[motor] = offset
            self.offset_writes.append({"motor": motor, "offset": offset})
            self._record_write(motor, ADDR_HOMING_OFFSET, wire)
        except BaseException:
            with contextlib.suppress(Exception):
                self._set_lock_fake(motor, True)
            raise
        else:
            self._set_lock_fake(motor, True)

    def _require_open_fake(self) -> None:
        from arm101.cli._errors import EXIT_ENV_ERROR, CliError

        if not self._open:
            raise CliError(
                code=EXIT_ENV_ERROR,
                message=_FAKEBUS_NOT_OPEN_MSG,
                remediation=_FAKEBUS_NOT_OPEN_REMEDIATION,
            )
