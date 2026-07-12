"""Unit tests for :mod:`arm101.hardware.safety` — the torque_guard primitive.

Every test drives a :class:`~arm101.hardware.bus.FakeBus` (or a purpose-built
subclass of it); none needs hardware. The failure these tests exist to prevent
is concrete: an ``arm explore`` run died on an unhandled ``SerialException``
and left all six motors energised, holding against gravity at ~50 C, until a
human re-opened the bus by hand and disabled torque.
"""

from __future__ import annotations

import pytest

from arm101.cli._errors import EXIT_USER_ERROR, CliError
from arm101.hardware.bus import FakeBus, OverloadError
from arm101.hardware.gentle import gentle_move
from arm101.hardware.safety import (
    ReleaseReport,
    TorqueGuard,
    release_torque,
    torque_guard,
)

ARM_MOTORS = (1, 2, 3, 4, 5, 6)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _open_bus(**kwargs: object) -> FakeBus:
    """An opened FakeBus carrying the six SO-101 follower motors."""
    bus = FakeBus(ids=list(ARM_MOTORS), **kwargs)  # type: ignore[arg-type]
    bus.open()
    return bus


def _de_energised(bus: FakeBus) -> list[int]:
    """Motors that received a torque-disable write, in call order."""
    return [w["motor"] for w in bus.torque_writes if w["on"] is False]


class _BrokenReleaseBus(FakeBus):
    """A FakeBus whose de-energise write fails for specific motors.

    Models the real hazard: the bus that just threw is the very bus the release
    has to talk to, and a serial-level failure is NOT a ``CliError`` — pyserial
    raises ``SerialException`` straight out of the SDK's ``write1ByteTxRx``. The
    default *exc* is therefore a plain :class:`RuntimeError`, standing in for
    that non-CliError case.
    """

    def __init__(
        self, broken: set[int], exc: BaseException | None = None, **kwargs: object
    ) -> None:
        super().__init__(**kwargs)  # type: ignore[arg-type]
        self._broken = broken
        self._exc = exc if exc is not None else RuntimeError("serial port went away")

    def clear_overload(self, motor: int) -> None:
        if motor in self._broken:
            raise self._exc
        super().clear_overload(motor)


class _LatchedOverloadBus(FakeBus):
    """A servo latched in overload: every ``enable_torque`` response carries bit 5.

    Mirrors real STS3215 behaviour — while latched, the servo tags *every*
    packet with the overload bit, including the response to the very
    torque-disable write that would clear it. ``clear_overload`` masks that bit
    off and still de-energises; ``enable_torque`` re-raises. This bus proves the
    guard reaches for the right primitive.
    """

    def enable_torque(self, motor: int, on: bool) -> None:
        raise OverloadError(motor=motor, error_byte=0x20)


# ---------------------------------------------------------------------------
# HOLD ON SUCCESS — a clean exit leaves torque exactly as the verb left it
# ---------------------------------------------------------------------------


def test_clean_exit_does_not_touch_torque() -> None:
    """A normal return must leave torque exactly as the guarded block left it."""
    bus = _open_bus()
    with torque_guard(bus, ARM_MOTORS):
        pass
    assert bus.torque_writes == []
    assert bus.register_writes == []


def test_clean_exit_preserves_gentle_move_stop_and_hold() -> None:
    """gentle_move's stop-and-hold contract survives the guard on the happy path.

    The whole reason the guard releases on ABNORMAL exit only: a gripper that
    has closed on an object must not drop it when the verb returns cleanly.
    """
    bus = _open_bus(positions={m: 2048 for m in ARM_MOTORS})
    with torque_guard(bus, ARM_MOTORS):
        result = gentle_move(bus, 6, 2148, min_angle=0, max_angle=4095, allow_motion=True)

    assert result["contacted"] is False
    assert {"motor": 6, "on": True} in bus.torque_writes
    assert _de_energised(bus) == []  # still holding — nothing was released


def test_clean_exit_reports_no_release() -> None:
    """The guard records that it did not release on a clean exit."""
    bus = _open_bus()
    with torque_guard(bus, ARM_MOTORS) as guard:
        pass
    assert guard.report is None


def test_return_from_the_guarded_block_is_a_clean_exit() -> None:
    """`return` out of the with-block is a normal exit, not an abnormal one."""
    bus = _open_bus()

    def verb() -> str:
        with torque_guard(bus, ARM_MOTORS):
            return "held"
        raise AssertionError("unreachable")  # pragma: no cover

    assert verb() == "held"
    assert _de_energised(bus) == []


# ---------------------------------------------------------------------------
# RELEASE ON ABNORMAL — an exception propagating out of the block
# ---------------------------------------------------------------------------


def test_exception_releases_every_owned_motor() -> None:
    """An exception escaping the block de-energises every motor the guard owns."""
    bus = _open_bus()
    boom = RuntimeError("a second process opened /dev/ttyACM0")

    with pytest.raises(RuntimeError) as excinfo:
        with torque_guard(bus, ARM_MOTORS):
            raise boom

    assert excinfo.value is boom  # the ORIGINAL error, unmasked
    assert _de_energised(bus) == list(ARM_MOTORS)


def test_keyboard_interrupt_releases_every_owned_motor() -> None:
    """SIGINT is an abnormal exit: Ctrl-C must never leave the arm energised.

    ``KeyboardInterrupt`` is a ``BaseException``, not an ``Exception`` — a guard
    that caught only ``Exception`` would sail straight past the single most
    likely way a human stops a runaway arm.
    """
    bus = _open_bus()

    with pytest.raises(KeyboardInterrupt):
        with torque_guard(bus, ARM_MOTORS):
            raise KeyboardInterrupt

    assert _de_energised(bus) == list(ARM_MOTORS)


def test_system_exit_releases_every_owned_motor() -> None:
    """SystemExit is also abnormal — the process is leaving; the arm must not stay hot."""
    bus = _open_bus()

    with pytest.raises(SystemExit):
        with torque_guard(bus, ARM_MOTORS):
            raise SystemExit(2)

    assert _de_energised(bus) == list(ARM_MOTORS)


def test_cli_error_releases_and_still_propagates() -> None:
    """A CliError from a verb releases torque and reaches the CLI error contract intact."""
    bus = _open_bus()

    with pytest.raises(CliError) as excinfo:
        with torque_guard(bus, ARM_MOTORS):
            raise CliError(code=EXIT_USER_ERROR, message="bad flag", remediation="fix it")

    assert excinfo.value.message == "bad flag"
    assert _de_energised(bus) == list(ARM_MOTORS)


def test_abnormal_exit_populates_the_report() -> None:
    """The guard exposes what it released, for a caller that wants to say so."""
    bus = _open_bus()
    guard = TorqueGuard(bus, ARM_MOTORS)

    with pytest.raises(RuntimeError):
        with guard:
            raise RuntimeError("boom")

    report = guard.report
    assert isinstance(report, ReleaseReport)
    assert report.attempted == ARM_MOTORS
    assert report.released == ARM_MOTORS
    assert report.failed == ()
    assert report.complete is True


# ---------------------------------------------------------------------------
# PER-MOTOR INDEPENDENCE — the release must survive its own failure
# ---------------------------------------------------------------------------


def test_a_failing_motor_does_not_abort_the_sweep() -> None:
    """A bus that raises on motor 1's release still de-energises motors 2..6."""
    bus = _BrokenReleaseBus(broken={1}, ids=list(ARM_MOTORS))
    bus.open()
    guard = TorqueGuard(bus, ARM_MOTORS)

    with pytest.raises(RuntimeError, match="original"):
        with guard:
            raise RuntimeError("original failure")

    assert _de_energised(bus) == [2, 3, 4, 5, 6]
    report = guard.report
    assert report is not None
    assert report.released == (2, 3, 4, 5, 6)
    assert report.failed == (1,)
    assert report.complete is False
    assert "serial port went away" in report.errors[1]


def test_a_motor_failing_mid_sweep_does_not_abort_the_rest() -> None:
    """Failures anywhere in the sweep are isolated — the sweep always runs to the end."""
    bus = _BrokenReleaseBus(broken={3, 5}, ids=list(ARM_MOTORS))
    bus.open()
    guard = TorqueGuard(bus, ARM_MOTORS)

    with pytest.raises(RuntimeError):
        with guard:
            raise RuntimeError("original failure")

    assert _de_energised(bus) == [1, 2, 4, 6]
    report = guard.report
    assert report is not None
    assert report.failed == (3, 5)


def test_a_totally_dead_bus_never_masks_the_original_exception() -> None:
    """When every release fails, the ORIGINAL exception is what propagates.

    The release runs on the bus that just died, so it will often fail outright.
    It must never replace the diagnosis the user actually needs.
    """
    bus = _BrokenReleaseBus(broken=set(ARM_MOTORS), ids=list(ARM_MOTORS))
    bus.open()
    original = RuntimeError("could not open port: device busy")
    guard = TorqueGuard(bus, ARM_MOTORS)

    with pytest.raises(RuntimeError) as excinfo:
        with guard:
            raise original

    assert excinfo.value is original
    report = guard.report
    assert report is not None
    assert report.released == ()
    assert report.failed == ARM_MOTORS
    assert report.complete is False


def test_a_system_exit_from_the_bus_is_never_swallowed() -> None:
    """The one exception the sweep will NOT absorb — an explicit request to exit.

    Nothing in the bus layer can plausibly raise ``SystemExit``, so this costs no
    real safety; swallowing an interpreter-exit request, on the other hand, is
    never right. Pinned so the asymmetry with ``KeyboardInterrupt`` (absorbed,
    see below) is a deliberate choice rather than an accident.
    """
    bus = _BrokenReleaseBus(broken={1}, exc=SystemExit(3), ids=list(ARM_MOTORS))
    bus.open()

    with pytest.raises(SystemExit):
        with torque_guard(bus, ARM_MOTORS):
            raise RuntimeError("original failure")


def test_release_survives_a_base_exception_from_the_bus() -> None:
    """A second Ctrl-C landing inside the sweep must not strand the remaining motors."""
    bus = _BrokenReleaseBus(broken={2}, exc=KeyboardInterrupt(), ids=list(ARM_MOTORS))
    bus.open()
    guard = TorqueGuard(bus, ARM_MOTORS)

    with pytest.raises(RuntimeError):
        with guard:
            raise RuntimeError("original failure")

    assert _de_energised(bus) == [1, 3, 4, 5, 6]
    report = guard.report
    assert report is not None
    assert report.failed == (2,)


def test_a_latched_overloaded_motor_is_still_released() -> None:
    """A motor latched in overload de-energises — the guard uses the tolerant primitive.

    ``enable_torque(m, False)`` raises ``OverloadError`` on a latched servo (the
    overload bit rides on the response to the very write that clears it), which
    is exactly the state a crashed motion verb tends to leave behind.
    """
    bus = _LatchedOverloadBus(ids=list(ARM_MOTORS))
    bus.open()
    guard = TorqueGuard(bus, ARM_MOTORS)

    with pytest.raises(RuntimeError):
        with guard:
            raise RuntimeError("overload storm")

    assert _de_energised(bus) == list(ARM_MOTORS)
    report = guard.report
    assert report is not None
    assert report.failed == ()


# ---------------------------------------------------------------------------
# Ownership
# ---------------------------------------------------------------------------


def test_motors_owned_inside_the_block_are_released() -> None:
    """A verb that energises motors lazily can hand them to the guard as it goes."""
    bus = _open_bus()
    guard = TorqueGuard(bus)

    with pytest.raises(RuntimeError):
        with guard as g:
            g.own(4)
            g.own(5, 6)
            raise RuntimeError("boom")

    assert _de_energised(bus) == [4, 5, 6]


def test_ownership_is_deduplicated_and_ordered() -> None:
    """Each motor is released once, in the order it was first claimed."""
    bus = _open_bus()
    guard = TorqueGuard(bus, (3, 1))
    guard.own(1, 2, 3)

    assert guard.motors == (3, 1, 2)

    with pytest.raises(RuntimeError):
        with guard:
            raise RuntimeError("boom")

    assert _de_energised(bus) == [3, 1, 2]


def test_disown_stops_the_guard_releasing_a_dead_address() -> None:
    """A motor that changed bus address mid-run must not be released at the old id.

    ``arm setup`` writes a new servo id into EEPROM, so the address the guard
    claimed stops answering the moment that write lands. Releasing it would fail
    and make the report cry wolf — "may still be energised" about a motor that is
    limp and merely renamed.
    """
    bus = _open_bus()
    guard = TorqueGuard(bus, (1,))

    with pytest.raises(RuntimeError):
        with guard as g:
            g.own(6)  # the servo now answers here...
            g.disown(1)  # ...and no longer here
            raise RuntimeError("boom")

    assert guard.motors == (6,)
    assert _de_energised(bus) == [6]
    report = guard.report
    assert report is not None
    assert report.complete is True  # no phantom failure for the dead address


def test_disowning_an_unclaimed_motor_is_a_no_op() -> None:
    """Idempotent: disowning what was never owned is an intent already satisfied."""
    bus = _open_bus()
    guard = TorqueGuard(bus, (1, 2))
    guard.disown(5)
    guard.disown(2)
    guard.disown(2)

    assert guard.motors == (1,)


def test_a_guard_that_owns_nothing_is_a_no_op() -> None:
    """No motors owned → nothing to release, and no bus traffic at all."""
    bus = _open_bus()
    guard = TorqueGuard(bus)

    with pytest.raises(RuntimeError):
        with guard:
            raise RuntimeError("boom")

    assert bus.register_writes == []
    report = guard.report
    assert report is not None
    assert report.attempted == ()
    assert report.complete is True


@pytest.mark.parametrize("bad", [0, -1])
def test_a_bad_motor_id_is_rejected_up_front(bad: int) -> None:
    """A typo'd id must fail loudly, not silently make the guard own the wrong motor."""
    bus = _open_bus()
    with pytest.raises(CliError) as excinfo:
        TorqueGuard(bus, (bad,))
    assert excinfo.value.code == EXIT_USER_ERROR


def test_own_rejects_a_bad_motor_id_before_the_block_runs() -> None:
    """The same validation applies to a motor claimed mid-block."""
    bus = _open_bus()
    guard = TorqueGuard(bus, (1,))
    with pytest.raises(CliError):
        guard.own(0)
    assert guard.motors == (1,)  # the bad id never landed


# ---------------------------------------------------------------------------
# The on_release hook
# ---------------------------------------------------------------------------


def test_on_release_is_called_with_the_report_on_abnormal_exit() -> None:
    """The hook lets a CLI verb tell the operator the arm was de-energised."""
    bus = _open_bus()
    seen: list[ReleaseReport] = []

    with pytest.raises(RuntimeError):
        with torque_guard(bus, ARM_MOTORS, on_release=seen.append):
            raise RuntimeError("boom")

    assert len(seen) == 1
    assert seen[0].released == ARM_MOTORS


def test_on_release_is_not_called_on_a_clean_exit() -> None:
    """Nothing was released, so nothing is announced."""
    bus = _open_bus()
    seen: list[ReleaseReport] = []

    with torque_guard(bus, ARM_MOTORS, on_release=seen.append):
        pass

    assert seen == []


def test_a_broken_on_release_hook_never_masks_the_original_exception() -> None:
    """A diagnostic that blows up must not become the error the user sees."""
    bus = _open_bus()
    original = RuntimeError("original failure")

    def explode(_report: ReleaseReport) -> None:
        raise ValueError("broken pipe on stderr")

    with pytest.raises(RuntimeError) as excinfo:
        with torque_guard(bus, ARM_MOTORS, on_release=explode):
            raise original

    assert excinfo.value is original
    assert _de_energised(bus) == list(ARM_MOTORS)  # released before the hook ran


def test_a_second_keyboard_interrupt_from_the_hook_never_replaces_the_original() -> None:
    """A second Ctrl-C landing inside the announcement must not replace the real error.

    ``KeyboardInterrupt`` is a ``BaseException``, not an ``Exception`` — a naive
    ``contextlib.suppress(Exception)`` around the hook call would let this one
    straight through ``__exit__``, discarding the original exception the
    ``with`` block was propagating in favour of whatever the operator's second
    Ctrl-C interrupted. That is the bug: an operator hammering the keys because
    the arm is still moving must never turn "SerialException: device busy" into
    a bare "KeyboardInterrupt" with the real diagnosis gone.
    """
    bus = _open_bus()
    original = RuntimeError("a second process opened /dev/ttyACM0")

    def announce_then_get_interrupted(_report: ReleaseReport) -> None:
        raise KeyboardInterrupt

    with pytest.raises(RuntimeError) as excinfo:
        with torque_guard(bus, ARM_MOTORS, on_release=announce_then_get_interrupted):
            raise original

    assert excinfo.value is original
    assert _de_energised(bus) == list(ARM_MOTORS)  # released before the hook ran


def test_a_system_exit_from_the_hook_propagates() -> None:
    """SystemExit from the hook is the one failure that DOES win — a deliberate asymmetry.

    Mirrors :func:`_release_motor`'s own asymmetry (re-raise ``SystemExit``,
    swallow everything else): nothing in an operator-supplied ``on_release``
    callback can plausibly raise ``SystemExit`` on purpose, and suppressing an
    explicit request for the interpreter to exit is never this guard's call to
    make. Pinned here so the difference from the ``KeyboardInterrupt`` case
    above reads as intentional, not as an inconsistent bug.
    """
    bus = _open_bus()

    def announce_then_exit(_report: ReleaseReport) -> None:
        raise SystemExit(7)

    with pytest.raises(SystemExit):
        with torque_guard(bus, ARM_MOTORS, on_release=announce_then_exit):
            raise RuntimeError("original failure")

    assert _de_energised(bus) == list(ARM_MOTORS)  # released before the hook ran


# ---------------------------------------------------------------------------
# release_torque — the bare sweep, usable without the context manager
# ---------------------------------------------------------------------------


def test_release_torque_de_energises_and_reports() -> None:
    """The sweep is a plain function; the guard is only a lifetime around it."""
    bus = _open_bus()
    report = release_torque(bus, ARM_MOTORS)

    assert _de_energised(bus) == list(ARM_MOTORS)
    assert report.released == ARM_MOTORS
    assert report.complete is True


def test_release_torque_is_idempotent() -> None:
    """Re-releasing an already-limp motor is harmless (Torque_Enable=0 twice)."""
    bus = _open_bus()
    release_torque(bus, (1,))
    report = release_torque(bus, (1,))

    assert _de_energised(bus) == [1, 1]
    assert report.complete is True


def test_report_describe_names_the_stranded_motors() -> None:
    """An incomplete release must SAY so — this is the line a human has to act on."""
    bus = _BrokenReleaseBus(broken={4}, ids=list(ARM_MOTORS))
    bus.open()
    report = release_torque(bus, ARM_MOTORS)

    text = report.describe()
    assert "INCOMPLETE" in text
    assert "4" in text
    assert report.complete is False


def test_report_describe_covers_the_clean_and_empty_cases() -> None:
    """The other two shapes of the operator-facing line."""
    bus = _open_bus()
    assert "Torque released on motors 1, 2" in release_torque(bus, (1, 2)).describe()
    assert "no motors" in ReleaseReport().describe()


def test_report_as_dict_is_json_shaped() -> None:
    """A verb's --json payload needs plain lists/str-keyed dicts, not tuples."""
    bus = _BrokenReleaseBus(broken={1}, ids=list(ARM_MOTORS))
    bus.open()
    payload = release_torque(bus, (1, 2)).as_dict()

    assert payload["attempted"] == [1, 2]
    assert payload["released"] == [2]
    assert payload["failed"] == [1]
    assert payload["complete"] is False
    assert set(payload["errors"]) == {"1"}  # type: ignore[arg-type]


def test_release_never_writes_the_eeprom_protection_registers() -> None:
    """The release touches Torque_Enable (addr 40) and nothing else."""
    bus = _open_bus()
    release_torque(bus, ARM_MOTORS)

    addresses = {w["addr"] for w in bus.register_writes}
    assert addresses == {40}
    assert all(w["value"] == 0 for w in bus.register_writes)
