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

These tests were written (task t1) against the DEFECT, marked
``xfail(strict=True)``, before any fix existed. Task t2 built the
:func:`~arm101.hardware.safety.torque_guard` primitive and task t3 wired it into
every gated motion verb; the markers came off with t3, which is what strict xfail
is for — an unexpected pass is a hard failure, so the fix cannot land while
leaving a stale xfail behind. Every test below asserts the post-fix behavior:
torque disabled on every motor after an abnormal exit.

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

import argparse
import json
import sys

import pytest

from arm101.cli import main
from arm101.cli._commands import arm as arm_cmd
from arm101.cli._commands import calibrate_motor as cm
from arm101.cli._commands import setup_motors
from arm101.cli._errors import EXIT_ENV_ERROR, CliError
from arm101.hardware import arm_spec
from arm101.hardware.bus import FakeBus

#: Follower motor ids 1..6, in arm_spec.JOINTS order — the whole arm this
#: regression guards end to end.
_ALL_MOTOR_IDS: "tuple[int, ...]" = tuple(
    arm_spec.joint_ids("follower")[joint] for joint in arm_spec.JOINTS
)


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


# ===========================================================================
# t3 — the wiring itself. The three tests above prove the LEAK is closed for
# `arm flex --demo`; the rest of this file proves the other half of the
# contract (a clean run must not release), that the OTHER gated motion verbs
# are wired too, and that a release is never silent.
# ===========================================================================


# ---------------------------------------------------------------------------
# HOLD ON SUCCESS — a clean run performs ZERO release writes
# ---------------------------------------------------------------------------


def test_successful_flex_demo_issues_zero_release_writes(monkeypatch) -> None:
    """A demo sweep that COMPLETES must leave torque exactly as it left it.

    This is the other half of the contract, and it is not a nicety: torque is
    released ONLY on an abnormal exit, so a successful ``gentle_move``'s
    deliberate stop-and-hold survives byte-for-byte. A guard that also released
    on the happy path would make a gripper drop whatever it had just closed on
    the instant the command returned. The whole sweep therefore ends with every
    joint still energized — and every ``on`` in the ledger is ``True``.
    """
    bus = _TeleportBus(positions={i: 2048 for i in _ALL_MOTOR_IDS})

    assert _run_flex_demo(monkeypatch, bus) == 0

    assert bus.torque_writes, "the sweep must have energized the joints at all"
    assert not any(
        w["on"] is False for w in bus.torque_writes
    ), f"a clean run released torque: {bus.torque_writes}"
    for motor in _ALL_MOTOR_IDS:
        assert _final_torque_state(bus, motor) is True


def test_successful_single_joint_flex_keeps_holding(monkeypatch) -> None:
    """The single-joint move is guarded too — and still holds on success."""
    bus = _TeleportBus(positions={i: 2048 for i in _ALL_MOTOR_IDS})
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(tty=False))

    code = main(
        [
            "arm",
            "flex",
            "shoulder_pan",
            "--to",
            "2148",
            "--gentle",
            "--apply",
            "--port",
            "/dev/ttyACM_fake",
        ]
    )

    assert code == 0
    assert _final_torque_state(bus, 1) is True
    assert not any(w["on"] is False for w in bus.torque_writes)


# ---------------------------------------------------------------------------
# arm explore — the verb the incident actually happened on
# ---------------------------------------------------------------------------


def test_explore_abnormal_exit_releases_every_joint(monkeypatch, tmp_path) -> None:
    """The verb from issue #33 itself: a mid-flood-fill fault must safe the arm.

    ``explore`` energizes its joints PROGRESSIVELY — the flood-fill lights one
    joint per probe and limps it again afterwards, while the escape search holds
    several perturbed at once — and the engine exposes no per-move callback, so
    the verb cannot know which joints are live when a fault lands. It does not
    need to: it owns all six the moment motion becomes possible. The incident
    left ALL SIX energized, so all six is exactly the right claim.

    ``fail_after=20`` lands inside the first probe's ``gentle_move`` (the 6
    grid-spec reads and 6 thermal-guard reads come first), i.e. with joint 1
    already energized and joints 2-6 not yet touched.
    """
    bus = _MidRunFaultBus(
        positions={i: 2048 for i in _ALL_MOTOR_IDS},
        fail_after=20,
        fault=_SimulatedBusFault("a second process opened /dev/ttyACM0"),
    )
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(tty=False))

    main(
        [
            "arm",
            "explore",
            "--apply",
            "--port",
            "/dev/ttyACM_fake",
            "--map",
            str(tmp_path / "reach.map.json"),
            "--max-moves",
            "5",
        ]
    )

    for motor in _ALL_MOTOR_IDS:
        assert _final_torque_state(bus, motor) is False, (
            f"motor {motor}: arm explore left it energized after an abnormal exit "
            f"— this is issue #33 verbatim (got {_final_torque_state(bus, motor)!r})"
        )


# ---------------------------------------------------------------------------
# The release must never be SILENT — the operator has to learn the arm was safed
# ---------------------------------------------------------------------------


def test_a_release_is_announced_on_stderr(monkeypatch, capsys) -> None:
    """A release fires mid-unwind, so the verb never reaches its result line.

    Without the announcement the de-energising would be completely silent and
    the human would be left staring at a bus error with no idea whether the arm
    they cannot see is still holding itself up. Diagnostics go to stderr, and no
    result is emitted at all on this path — so stdout must stay empty.
    """
    bus = _MidRunFaultBus(
        positions={i: 2048 for i in _ALL_MOTOR_IDS},
        fail_after=230,
        fault=_SimulatedBusFault("simulated serial fault mid-run"),
    )

    _run_flex_demo(monkeypatch, bus)

    captured = capsys.readouterr()
    assert "Torque released on motors 1, 2, 3, 4, 5, 6" in captured.err
    assert "Torque released" not in captured.out  # results/diagnostics never mix


def test_an_incomplete_release_says_so_loudly(monkeypatch, capsys) -> None:
    """A motor the release could NOT reach may still be hot — and must be NAMED.

    This is the one outcome a human has to act on, so it is the one the report
    refuses to soften: "attempted" is not "released".
    """
    bus = _ReleaseAlsoFaultsBus(
        positions={i: 2048 for i in _ALL_MOTOR_IDS},
        fail_after=230,
        fault=_SimulatedBusFault("simulated serial fault mid-run"),
        jam_motor=2,
        release_fault=CliError(
            code=EXIT_ENV_ERROR,
            message="simulated release failure for motor 2",
            remediation="retry",
        ),
    )

    _run_flex_demo(monkeypatch, bus)

    err = capsys.readouterr().err
    assert "INCOMPLETE" in err
    assert "motors 2" in err
    assert "may still be energised" in err


def test_the_release_is_json_under_the_json_flag(monkeypatch, capsys) -> None:
    """``--json`` keeps the same stdout/stderr split — and speaks JSON on both.

    An agent parsing stderr must not have to fish a sentence out from between
    JSON documents, so the announcement is emitted as a structured record
    (:meth:`ReleaseReport.as_dict`) rather than prose.
    """
    bus = _MidRunFaultBus(
        positions={i: 2048 for i in _ALL_MOTOR_IDS},
        fail_after=230,
        fault=_SimulatedBusFault("simulated serial fault mid-run"),
    )
    _patch_bus(monkeypatch, bus)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(tty=False))

    main(["arm", "flex", "--demo", "--apply", "--json", "--port", "/dev/ttyACM_fake"])

    captured = capsys.readouterr()
    payloads = [
        json.loads(line)
        for line in captured.err.splitlines()
        if line.startswith("{") and "torque_release" in line
    ]
    assert len(payloads) == 1, f"expected one structured release record, got: {captured.err!r}"

    release = payloads[0]["torque_release"]
    assert release["attempted"] == list(_ALL_MOTOR_IDS)
    assert release["released"] == list(_ALL_MOTOR_IDS)
    assert release["failed"] == []
    assert release["complete"] is True
    assert captured.out == ""  # no result on an abnormal exit — the split holds


# ---------------------------------------------------------------------------
# arm setup / setup-motors — the guard follows the motor across its id change
# ---------------------------------------------------------------------------


class _EepromWriteFaultBus(FakeBus):
    """A FakeBus whose EEPROM id write fails outright (the motor keeps its old id)."""

    def write_id_baudrate(self, motor: int, new_id: int, baudrate: int) -> None:
        raise _SimulatedBusFault("simulated EEPROM write failure")


def _patch_setup_detection(monkeypatch, fake: FakeBus) -> None:
    """Patch the per-motor detection seam so the setup walk opens *fake*."""

    def _open(_port: str) -> FakeBus:
        fake.open()  # the walk closes the bus after each motor
        return fake

    monkeypatch.setattr(cm, "_candidate_ports", lambda: ["/dev/ttyACM_fake"])
    monkeypatch.setattr(cm, "_open_bus", _open)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(tty=False))


def test_setup_releases_the_motor_on_an_abnormal_exit(monkeypatch) -> None:
    """A fault during the EEPROM walk leaves the motor LIMP, not hot.

    ``setup`` never enables torque itself, so on a cold bench the guard is a
    no-op — which is exactly why it is easy to leave out. It matters because the
    motor on the bench need not be cold: a servo left holding by an earlier
    ``arm flex``, or latched in overload from a previous session, is still
    energized when ``setup`` picks it up, and this verb had no path that would
    ever have relaxed it.

    This also doubles as the "no evidence the address moved" side of the
    id-write-failure probe added for the relock-after-landed-id bug below:
    ``_EepromWriteFaultBus`` raises before touching ``_ids`` at all, so
    ``bus.scan([motor_id])`` finds nothing at the new id and the probe
    (correctly) leaves ownership right where it was — the release still
    targets the untouched ``detected_id``, never the id that was never
    reached.
    """
    bus = _EepromWriteFaultBus(ids=[1])
    _patch_setup_detection(monkeypatch, bus)

    with pytest.raises(_SimulatedBusFault):
        setup_motors.cmd_setup_motors(
            argparse.Namespace(
                json=False, port=None, current_id=None, apply=True, baudrate=1_000_000
            )
        )

    # The id write never landed, so the servo is still at its detected id (1) —
    # which is the address the guard claimed, and the one it released.
    assert _final_torque_state(bus, 1) is False
    # And the probe must not have conjured ownership of an id nothing answers
    # to — the never-reached new id (6, the gripper, written first) must be
    # completely untouched.
    assert _final_torque_state(bus, 6) is None


def test_setup_releases_the_new_id_once_the_eeprom_write_has_landed(monkeypatch, capsys) -> None:
    """Writing EEPROM addr 5 MOVES the servo — the guard's claim has to move with it.

    After the id write the motor answers at its new id and the old address is
    dead. A guard still holding the old id would aim its release at a servo that
    no longer exists, fail, and then tell the operator the motor "may still be
    energised" — a false alarm on the one line that must never be ignored. The
    walk therefore hands ownership over (``own(new)`` + ``disown(old)``) the
    moment the write succeeds.

    Forced here by making the after-read report a mismatched id, which is the
    verb's own "the write did not stick" hard failure — an abnormal exit that
    can only happen AFTER the id write.
    """
    # The gripper (id 6) is written first; make its read-back report a foreign id.
    bus = FakeBus(ids=[1], info={6: {"id": 99}})
    _patch_setup_detection(monkeypatch, bus)

    with pytest.raises(CliError, match="did not persist"):
        setup_motors.cmd_setup_motors(
            argparse.Namespace(
                json=False, port=None, current_id=None, apply=True, baudrate=1_000_000
            )
        )

    assert _final_torque_state(bus, 6) is False, "the NEW id must be released"
    assert _final_torque_state(bus, 1) is None, "the dead OLD address must not be written to"

    err = capsys.readouterr().err
    assert "Torque released on motors 6" in err
    assert "INCOMPLETE" not in err  # no false alarm about a stranded motor


# ---------------------------------------------------------------------------
# qodo review bug (PR #38): write_id_baudrate can raise AFTER the id landed
# ---------------------------------------------------------------------------


class _RelockFailsAfterIdLandsBus(FakeBus):
    """Mirrors ``FeetechBus.write_id_baudrate``'s exact failure mode.

    ``write_id_baudrate`` (arm101/hardware/bus.py) writes Baud_Rate then ID
    (addr 5) inside its own try/except; once the ID write itself succeeds it
    falls through to the SUCCESS path, which restores EEPROM write-protection
    via ``self._set_lock(relock_target, True)`` from the `else:` branch —
    OUTSIDE that inner try/except. If THAT re-lock call raises,
    ``write_id_baudrate`` raises right along with it, even though the servo
    has already moved to ``new_id``.

    This bus reproduces exactly that: the id write LANDS (delegated to
    :class:`FakeBus`, so :attr:`FakeBus.eeprom_writes` records it and
    :attr:`_ids` is updated to reflect the moved address — the same signal a
    real bus gives a subsequent ``scan()``), and only THEN does the call
    raise, standing in for the relock failure.
    """

    def write_id_baudrate(self, motor: int, new_id: int, baudrate: int) -> None:
        super().write_id_baudrate(motor, new_id, baudrate)  # the id write itself lands
        self._ids = sorted((set(self._ids) - {motor}) | {new_id})
        raise _SimulatedBusFault(
            f"simulated relock failure for motor {new_id} after its id write landed"
        )


def test_setup_transfers_ownership_when_the_id_write_lands_but_the_relock_fails(
    monkeypatch,
) -> None:
    """The bug this file is named for: a relock failure AFTER the id lands must
    not strand the guard's claim on a dead address.

    Before the fix, ``_process_one_motor`` only moved the guard's claim
    (``own(new)`` / ``disown(old)``) once ``bus.write_id_baudrate`` RETURNED —
    so on this exact fault the guard kept owning the stale ``detected_id`` (1)
    even though the servo had already moved to ``motor_id`` (6, the gripper,
    written first). The abnormal-exit release sweep would then address an id
    nothing answers to any more: the release write would fail (or, on
    :class:`FakeBus`, silently "succeed" against a fictional motor), and the
    operator would be told the motor "did not respond and may still be
    energised" when in truth it is limp and merely renamed.

    The fix probes ``bus.scan([motor_id])`` for evidence the address moved
    before re-raising, and moves the claim only when it finds that evidence —
    which is exactly what this bus is built to provide.
    """
    bus = _RelockFailsAfterIdLandsBus(ids=[1])
    _patch_setup_detection(monkeypatch, bus)

    with pytest.raises(_SimulatedBusFault):
        setup_motors.cmd_setup_motors(
            argparse.Namespace(
                json=False, port=None, current_id=None, apply=True, baudrate=1_000_000
            )
        )

    assert _final_torque_state(bus, 6) is False, "the NEW id must be released"
    assert _final_torque_state(bus, 1) is None, "the dead OLD address must not be written to"
