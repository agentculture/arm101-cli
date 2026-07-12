"""Regression guard for issue #33 — abnormal-exit torque leak (task t1).

``arm explore`` died on an unhandled ``serial.SerialException`` mid-run and
left ALL SIX MOTORS ENERGIZED, holding against gravity at ~50C with nobody
watching. ``gentle_move`` deliberately holds torque on a clean finish or a
detected contact — stop-and-hold is its contract and is CORRECT (see
:mod:`arm101.hardware.gentle`). The gap is at the CLI VERB level: neither
``cmd_arm_flex`` nor ``cmd_arm_explore`` (see ``arm101/cli/_commands/arm.py``)
does anything but ``bus.close()`` in their ``finally`` — and closing the bus
does not touch torque — so ANY abnormal exit (an unhandled exception, a bus
fault, SIGINT) walks away from a powered arm.

This file only reproduces the defect; it does not fix it. Task t2 builds the
``torque_guard`` primitive and task t3 wires it into the motion verbs. Every
test below asserts the DESIRED post-fix behavior — torque disabled on every
motor after an abnormal exit — and is marked ``xfail(strict=True)`` because
that assertion FAILS against today's code. Once t3 lands, these tests start
passing; strict xfail turns that unexpected pass into a hard failure, forcing
t3's author to remove the marker rather than silently leaving a stale xfail
behind.

Scenarios covered (each fails differently, so each gets its own test):

(a) an arbitrary (non-``CliError``, non-``OverloadError``) exception raised
    mid-run from a bus read — standing in for the ``serial.SerialException``
    that actually killed ``arm explore`` on hardware;
(b) a ``KeyboardInterrupt`` (SIGINT) mid-run — ``main()``'s dispatcher only
    catches ``CliError``/``Exception`` (see ``arm101/cli/__init__.py``
    ``_dispatch``), so a ``KeyboardInterrupt`` is a ``BaseException`` that
    is deliberately never swallowed there; any safety net has to run before
    it escapes, not rely on the top-level dispatcher catching it;
(c) a bus fault raised from the RELEASE path itself — the bus that just threw
    the original fault is the same bus any recovery must reuse, and on real
    hardware that release write can fail too (comms are already unhappy).
    One motor's release failing must not stop the rest from being released.

Bus injection seam mirrors tests/test_arm_overload.py: ``arm_cmd._open_bus``
is monkeypatched to hand back a purpose-built :class:`~arm101.hardware.bus.FakeBus`
subclass, driven through the real CLI entry point (:func:`arm101.cli.main`)
rather than a bare handler call, so the test exercises the verb-level gap.
"""

from __future__ import annotations

import sys

import pytest

from arm101.cli import main
from arm101.cli._commands import arm as arm_cmd
from arm101.cli._errors import EXIT_ENV_ERROR, CliError
from arm101.hardware import arm_spec
from arm101.hardware.bus import FakeBus

#: Follower motor ids 1..6, in arm_spec.JOINTS order — the whole arm this
#: regression guards end to end.
_ALL_MOTOR_IDS: "tuple[int, ...]" = tuple(
    arm_spec.joint_ids("follower")[joint] for joint in arm_spec.JOINTS
)

#: xfail reason shared by every test in this file — see the module docstring.
_XFAIL_REASON = "known defect #33: motion verbs leak torque on abnormal exit; fixed by t3"


class _SimulatedBusFault(Exception):
    """Stand-in for an unhandled hardware-layer fault (e.g. ``serial.SerialException``).

    The runtime package has zero third-party dependencies (see
    ``pyproject.toml``), so this test cannot import pyserial's real exception
    type. This plays the same structural role: an exception that is NEITHER
    ``CliError`` NOR ``OverloadError`` — exactly the kind of fault issue #33
    reports escaping a motion verb uncaught.
    """


class _FakeStdin:
    """Scripted stdin controlling ``isatty()`` — mirrors tests/test_arm_overload.py.

    Agent mode (``--apply`` + non-TTY) never actually reads a line, so
    ``readline`` is unused here; kept for parity with the sibling test files'
    ``_FakeStdin``.
    """

    def __init__(self, tty: bool = False) -> None:
        self._tty = tty

    def isatty(self) -> bool:
        return self._tty

    def readline(self) -> str:
        return ""  # EOF


class _TeleportBus(FakeBus):
    """FakeBus whose ``write_goal_position`` teleports the joint straight there.

    Plain :class:`FakeBus` never updates ``present_position`` on a goal write,
    so ``gentle_move``'s travel loop would spin for the full
    ``_MAX_POLLS_PER_MOVE`` ceiling on every move (it never measures arrival).
    Teleporting keeps each move's travel loop short and DETERMINISTIC — this
    file cares about which motors got torque enabled before a fault, not
    about travel fidelity (see tests/_fakes.py's ``ServoModelBus`` for that).
    """

    def write_goal_position(self, motor: int, position: int) -> None:
        super().write_goal_position(motor, position)
        self._positions[motor] = position


class _MidRunFaultBus(_TeleportBus):
    """Raises *fault* the first time ``read_info`` is called at/after *fail_after*.

    Only the FIRST qualifying call raises — this models a single fault event
    (a bus that glitches once), not a permanently wedged one, so a RELEASE
    path (once t3 adds one) gets a fair chance to run its cleanup calls
    against a bus that behaves normally again afterwards.
    """

    def __init__(self, *args, fail_after: int, fault: BaseException, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._fail_after = fail_after
        self._fault = fault
        self._reads = 0
        self._fired = False

    def read_info(self, motor: int) -> "dict[str, int]":
        self._reads += 1
        if not self._fired and self._reads >= self._fail_after:
            self._fired = True
            raise self._fault
        return super().read_info(motor)


class _ReleaseAlsoFaultsBus(_MidRunFaultBus):
    """Like :class:`_MidRunFaultBus`, but the release call for *jam_motor* ALSO raises.

    Models scenario (c): the same bus that just threw the original fault is
    the bus any recovery has to reuse, and a release write can fail too. Only
    the first release attempt for *jam_motor* raises (mirrors
    :class:`_MidRunFaultBus`'s single-fault-event shape); every release
    attempt — successful or not — is recorded in :attr:`release_attempts` so
    a test can prove the guard actually TRIED the jammed motor, not merely
    that it skipped it.
    """

    def __init__(self, *args, jam_motor: int, release_fault: BaseException, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._jam_motor = jam_motor
        self._release_fault = release_fault
        self._release_jam_fired = False
        self.release_attempts: "list[int]" = []

    def enable_torque(self, motor: int, on: bool) -> None:
        if not on:
            self.release_attempts.append(motor)
            if motor == self._jam_motor and not self._release_jam_fired:
                self._release_jam_fired = True
                raise self._release_fault
        super().enable_torque(motor, on)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _patch_bus(monkeypatch, bus: FakeBus) -> None:
    """Patch the arm flex/explore seam so it opens *bus* (mirrors test_arm_overload.py)."""
    bus.open()
    monkeypatch.setattr(arm_cmd, "_open_bus", lambda _port: bus)


def _final_torque_state(bus: FakeBus, motor: int) -> "bool | None":
    """The LAST recorded ``on``/``off`` state for *motor* in ``bus.torque_writes``.

    ``None`` if *motor* was never touched at all — a distinct, equally-bad
    case from "last state was True": either way the motor is not provably
    torque-disabled.
    """
    calls = [w["on"] for w in bus.torque_writes if w["motor"] == motor]
    return calls[-1] if calls else None


def _run_flex_demo(monkeypatch, bus: FakeBus):
    """Drive ``arm101 arm flex --demo --apply`` through the real CLI entry point."""
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(tty=False))
    return main(["arm", "flex", "--demo", "--apply", "--port", "/dev/ttyACM_fake"])


# ---------------------------------------------------------------------------
# (a) an arbitrary exception raised mid-run from a bus read
# ---------------------------------------------------------------------------


@pytest.mark.xfail(strict=True, reason=_XFAIL_REASON)
def test_flex_demo_generic_exception_mid_run_releases_all_torque(monkeypatch) -> None:
    """A non-CliError, non-OverloadError fault mid-sweep must still leave
    every motor 1..6 torque-DISABLED, not energized and unattended.

    fail_after=230 lands inside the THIRD joint's first gentle_move call (of
    six): the first two joints finish their full low/high sweep (torque left
    enabled by gentle_move's stop-and-hold contract — that part is correct),
    the third is interrupted mid-travel (torque already enabled, move never
    finishes), and joints four through six are never touched at all — a
    realistic "died partway through" snapshot, not merely "died on the very
    first op".
    """
    bus = _MidRunFaultBus(
        positions={i: 2048 for i in _ALL_MOTOR_IDS},
        fail_after=230,
        fault=_SimulatedBusFault("simulated serial fault mid-run"),
    )

    _run_flex_demo(monkeypatch, bus)

    for motor in _ALL_MOTOR_IDS:
        assert _final_torque_state(bus, motor) is False, (
            f"motor {motor}: expected torque disabled after an abnormal exit, "
            f"got {_final_torque_state(bus, motor)!r}"
        )


# ---------------------------------------------------------------------------
# (b) KeyboardInterrupt (SIGINT) mid-run
# ---------------------------------------------------------------------------


@pytest.mark.xfail(strict=True, reason=_XFAIL_REASON)
def test_flex_demo_keyboard_interrupt_mid_run_releases_all_torque(monkeypatch) -> None:
    """Ctrl-C mid-sweep must ALSO leave every motor torque-disabled.

    ``main()``'s dispatcher deliberately catches only ``CliError``/``Exception``
    (see ``arm101/cli/__init__.py::_dispatch``) — ``KeyboardInterrupt`` is a
    ``BaseException`` and is never swallowed there, so it propagates all the
    way out of ``main()`` uncaught. Any safety net therefore has to run its
    release BEFORE the interrupt escapes the verb, not rely on the top-level
    dispatcher to catch-and-clean-up after the fact — there is no
    catch-and-clean-up for a KeyboardInterrupt at that layer, by design.
    """
    bus = _MidRunFaultBus(
        positions={i: 2048 for i in _ALL_MOTOR_IDS},
        fail_after=230,
        fault=KeyboardInterrupt(),
    )

    with pytest.raises(KeyboardInterrupt):
        _run_flex_demo(monkeypatch, bus)

    for motor in _ALL_MOTOR_IDS:
        assert _final_torque_state(bus, motor) is False, (
            f"motor {motor}: expected torque disabled after Ctrl-C, "
            f"got {_final_torque_state(bus, motor)!r}"
        )


# ---------------------------------------------------------------------------
# (c) the RELEASE path itself faults for one motor — others must still release
# ---------------------------------------------------------------------------


@pytest.mark.xfail(strict=True, reason=_XFAIL_REASON)
def test_flex_demo_release_path_faults_other_motors_still_released(monkeypatch) -> None:
    """The release write can fail too — one jammed motor must not stop the rest.

    The bus that just threw the ORIGINAL fault is the only bus a recovery
    path can reuse (there is nowhere else to send a torque-disable write), and
    on real hardware a comms link that just faulted staying unhappy for the
    very next write is entirely plausible. ``shoulder_lift`` (motor 2, already
    fully swept by the time the original fault fires at fail_after=230) is
    jammed so ITS release raises — the other five motors must still end up
    torque-disabled, and the guard must still have ATTEMPTED motor 2's release
    (not silently skipped it) even though that attempt failed.
    """
    jam_motor = 2  # shoulder_lift
    bus = _ReleaseAlsoFaultsBus(
        positions={i: 2048 for i in _ALL_MOTOR_IDS},
        fail_after=230,
        fault=_SimulatedBusFault("simulated serial fault mid-run"),
        jam_motor=jam_motor,
        release_fault=CliError(
            code=EXIT_ENV_ERROR,
            message=f"simulated release failure for motor {jam_motor}",
            remediation="retry",
        ),
    )

    _run_flex_demo(monkeypatch, bus)

    assert jam_motor in bus.release_attempts, (
        "the guard must still ATTEMPT to release the jammed motor even though "
        "the write itself fails"
    )
    for motor in _ALL_MOTOR_IDS:
        if motor == jam_motor:
            continue  # this motor's own release is the one that raises; see above
        assert _final_torque_state(bus, motor) is False, (
            f"motor {motor}: expected torque disabled even though motor "
            f"{jam_motor}'s release raised, got {_final_torque_state(bus, motor)!r}"
        )
