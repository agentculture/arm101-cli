"""Torque ownership — a guard that de-energises the arm on any abnormal exit.

Pure helper over :mod:`arm101.hardware.bus` — zero third-party dependencies,
and (deliberately) no runtime import of the bus module at all: it only ever
*calls* the ``MotorBus`` it is handed, so it stays trivially unit-testable
against ``FakeBus`` and adds nothing to the import graph of a verb that never
moves a motor.

Why this module exists
----------------------
An ``arm explore`` run died on an unhandled ``serial.SerialException`` — a
second process had opened ``/dev/ttyACM0`` — and **left all six motors
energised**, holding the arm up against gravity at ~50 C, with nobody watching.
Torque stayed on until a human manually re-opened the bus and disabled it. Any
USB hiccup, cable knock, ``Ctrl-C``, or second process on the port reproduces
it, because nothing in the stack ever *owned* torque as a resource: the verbs
turned it on and simply never had a code path that turned it back off.

Note the layer this sits at. :func:`arm101.hardware.gentle.gentle_move`
**deliberately holds torque after a move** — stop-and-hold is its contract, so a
gripper that has closed on an object does not drop it the instant the move
returns. That is correct and is not changed here. The gap was one layer *above*,
at the CLI-verb level, which had no notion of owning the motors it energised.

The contract: HOLD ON SUCCESS, RELEASE ON ABNORMAL
--------------------------------------------------
* A **clean** exit from the guarded block leaves torque *exactly* as the verb
  left it. No bus traffic at all. ``gentle_move``'s stop-and-hold survives, and
  an arm left powered at the end of a successful command is a state the operator
  asked for.
* An **abnormal** exit — any exception propagating out of the block, explicitly
  including ``KeyboardInterrupt`` (SIGINT / Ctrl-C) and ``SystemExit`` — releases
  torque on every motor the guard owns.

Net effect: **a powered arm at process exit is always a deliberate state, never
an accident.**

Design notes
------------
*The release has to survive its own failure.* The bus that just raised
``SerialException`` is the very bus the release must talk to, so the sweep
assumes it will partly fail: each motor is attempted **independently**, failures
are captured per-motor rather than raised, and the sweep always runs to the end
— a bus that refuses motor 1 must still de-energise motors 2..6. And it must
never *mask* the original exception: the failure the operator needs to see is
the ``SerialException``, not some secondary "could not write to a port that no
longer exists" from the cleanup. :meth:`TorqueGuard.__exit__` therefore returns
``None`` on every path — typed that way precisely so there is no value it *could*
return that would suppress the exception — and never raises.

*Why ``clear_overload`` and not ``enable_torque(m, False)``.* Both write
``Torque_Enable = 0`` (STS3215 addr 40) — identical on the wire, identical in
effect. They differ in how they treat the *response*: while a servo is latched
in overload it tags **every** packet with the overload bit (0x20), including the
response to the very torque-disable write that clears the latch, so
``enable_torque`` raises :class:`~arm101.hardware.bus.OverloadError` on exactly
the motor that most needs de-energising. ``clear_overload`` is documented as
overload-*tolerant*: it masks that bit off and reports success. A crashed motion
verb is a prime way to leave a motor latched, so the release reaches for the
primitive that copes with the state it is most likely to find.

*Why the guard does not restore ``Torque_Limit``.* ``gentle_move`` already caps
and restores the RAM ``Torque_Limit`` (addr 48) around each move in its own
``finally`` — that is a different layer, and duplicating it here would mean
issuing an extra register write, on a bus that has just failed, for a register
whose value is **moot on a de-energised motor**: a torque cap only bounds torque
that is being applied, and the whole point of this module is that none is. The
guard therefore writes exactly one register, addr 40, and nothing else.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from types import TracebackType
from typing import TYPE_CHECKING, Callable, Iterable

from arm101.cli._errors import EXIT_USER_ERROR, CliError

if TYPE_CHECKING:
    from arm101.hardware.bus import MotorBus

#: Callback invoked with the :class:`ReleaseReport` the moment a release happens,
#: so a CLI verb can tell the operator the arm was de-energised *before* the
#: original exception finishes unwinding. Never allowed to fail the release.
ReleaseHook = Callable[["ReleaseReport"], None]


def _describe(exc: BaseException) -> str:
    """Render *exc* for a report field — ``"TypeName: message"``, or just the type.

    Bare ``str()`` is not enough: ``str(KeyboardInterrupt())`` is the empty
    string, which would put a blank entry in the report exactly when a human was
    hammering Ctrl-C and most wants to know what happened.
    """
    detail = str(exc)
    return f"{type(exc).__name__}: {detail}" if detail else type(exc).__name__


def _require_motor_id(motor: int) -> None:
    """Raise :class:`CliError` unless *motor* is a usable Feetech servo id.

    Validated when the motor is *claimed*, never at release time. A typo'd id
    that was only caught during the sweep would mean the guard had spent the
    whole run believing it owned a motor it could not release — the exact silent
    hole this module exists to close. Failing at ``own()`` makes it loud, and
    makes it loud *before* anything is energised.
    """
    if not isinstance(motor, int) or isinstance(motor, bool) or motor < 1:
        raise CliError(
            code=EXIT_USER_ERROR,
            message=f"Motor id must be a positive integer, got {motor!r}.",
            remediation="Pass 1-indexed Feetech servo ids (the SO-101 follower uses 1-6).",
        )


def _release_motor(bus: "MotorBus", motor: int) -> str | None:
    """De-energise one motor, best-effort. Return ``None`` on success, else the error text.

    Deliberately swallows almost everything, including ``KeyboardInterrupt``.
    That is not laziness — it is the point:

    * The failure that motivated this module is a **non-**\\ :class:`CliError`:
      pyserial's ``SerialException`` comes straight out of the SDK's
      ``write1ByteTxRx`` without being wrapped. Catching only ``CliError`` (as
      the explore engine's best-effort release does, where the bus is known
      healthy) would sail right past the case this exists for.
    * A **second** Ctrl-C, from an operator impatient with a runaway arm, must
      not strand motors 3..6 because it landed while motor 2 was being released.
      The sweep is a handful of single-byte writes — a few milliseconds — so
      making it uninterruptible costs nothing and is strictly safer. The
      original ``KeyboardInterrupt`` that *started* the release still propagates;
      only one landing *inside* the sweep is absorbed.

    ``SystemExit`` is the one exception re-raised: nothing in the bus layer can
    plausibly raise it, and swallowing an explicit request for the interpreter to
    exit is never right.
    """
    try:
        # Torque_Enable = 0 (addr 40) — same wire write as enable_torque(m, False),
        # but overload-TOLERANT, so a motor latched in overload still releases
        # instead of raising OverloadError back at us. See the module docstring.
        bus.clear_overload(motor)
    except SystemExit:
        raise
    except BaseException as exc:  # noqa: B036 - the release must outlive any bus failure
        return _describe(exc)
    return None


def _invoke_release_hook(hook: ReleaseHook, report: "ReleaseReport") -> None:
    """Call *hook* with *report*, without letting a hook failure replace the real exception.

    ``TorqueGuard.__exit__`` calls this only while an abnormal exit is already
    unwinding the stack — the torque release itself (see :func:`_release_motor`)
    has already happened by the time this runs, so nothing here can cost the
    operator the safety action, only the diagnostic line that announces it.

    Deliberately mirrors :func:`_release_motor`'s asymmetry, for the identical
    reason:

    * ``contextlib.suppress(Exception)`` looks like the obvious tool for "a
      failing diagnostic must not replace the exception being raised" but is
      the *wrong* one: ``KeyboardInterrupt`` and ``SystemExit`` are
      ``BaseException`` subclasses, not ``Exception`` subclasses, so neither is
      caught by it. A second ``Ctrl-C`` — an operator hammering the keys
      because the arm is still moving — landing while ``hook`` is mid-print
      would sail straight through ``suppress(Exception)``, out of
      ``__exit__``, and **replace** the original exception the ``with`` block
      is propagating. That is precisely the failure this function exists to
      close: the module docstring promises the operator always sees the real
      failure, never a secondary one from an announcement that choked.
    * ``SystemExit`` is the one exception let through anyway: nothing in an
      operator-supplied ``on_release`` callback can plausibly raise it on
      purpose, and swallowing an explicit request for the interpreter to exit
      is never this guard's call to make. Re-raising it here also keeps
      SonarCloud's S5754 ("this exception handler should catch a specific
      exception") satisfied for the same reason it is satisfied in
      :func:`_release_motor`: the broad ``except BaseException`` is paired
      with a narrower one that runs first, so the broad catch reads as a
      deliberate choice, not an oversight.

    Never raises except to propagate ``SystemExit``.
    """
    try:
        hook(report)
    except SystemExit:
        raise
    except BaseException:  # noqa: B036 - a failing announcement must not replace the real exception
        pass


@dataclass(frozen=True)
class ReleaseReport:
    """What a release sweep actually managed to do — honest, never assumed.

    A release runs on a bus that has usually just failed, so "we asked for it"
    and "it happened" are different claims and this reports them separately. A
    caller that wants to reassure the operator that the arm is safe must check
    :attr:`complete`, not merely that a release was attempted.

    Attributes
    ----------
    attempted:
        Every motor the guard owned, in the order it was claimed.
    released:
        Motors whose de-energise write the bus accepted.
    failed:
        Motors the bus refused — these may still be **energised**. Loud by
        design: this is the one outcome a human has to act on.
    errors:
        ``motor -> failure text``, for the motors in :attr:`failed` only.
    """

    attempted: tuple[int, ...] = ()
    released: tuple[int, ...] = ()
    failed: tuple[int, ...] = ()
    errors: dict[int, str] = field(default_factory=dict)

    @property
    def complete(self) -> bool:
        """``True`` iff every owned motor is confirmed de-energised."""
        return not self.failed

    def describe(self) -> str:
        """One line for an operator — safe to hand to ``emit_diagnostic``."""
        if not self.attempted:
            return "Torque guard released no motors (none were owned)."
        if self.complete:
            listed = ", ".join(str(m) for m in self.released)
            return f"Torque released on motors {listed} after an abnormal exit."
        stranded = ", ".join(str(m) for m in self.failed)
        return (
            f"Torque release INCOMPLETE: motors {stranded} did not respond and may still "
            "be energised. Power down the arm or re-run once the bus is available."
        )

    def as_dict(self) -> dict[str, object]:
        """JSON-serialisable form, for a verb's ``--json`` payload."""
        return {
            "attempted": list(self.attempted),
            "released": list(self.released),
            "failed": list(self.failed),
            "errors": {str(motor): text for motor, text in self.errors.items()},
            "complete": self.complete,
        }


def release_torque(bus: "MotorBus", motors: Iterable[int]) -> ReleaseReport:
    """De-energise every motor in *motors*, independently, and report what happened.

    The bare sweep, exposed on its own so it is usable without a guard (and
    testable without one). Never raises for a bus failure: each motor is tried
    on its own and a refusal is recorded in the returned :class:`ReleaseReport`
    rather than aborting the motors behind it. Idempotent — writing
    ``Torque_Enable = 0`` to an already-limp motor is a harmless no-op, so a
    caller in doubt should just release again.

    Parameters
    ----------
    bus:
        An **open** :class:`~arm101.hardware.bus.MotorBus` (real or fake). A
        closed bus is not special-cased: the per-motor writes fail and land in
        :attr:`ReleaseReport.failed`, which is the honest answer — the motors
        were not released.
    motors:
        Motor ids to de-energise, in release order.

    Returns
    -------
    ReleaseReport
        Which motors went limp, which did not, and why.
    """
    attempted: list[int] = []
    released: list[int] = []
    failed: list[int] = []
    errors: dict[int, str] = {}

    for motor in motors:
        attempted.append(motor)
        error = _release_motor(bus, motor)
        if error is None:
            released.append(motor)
        else:
            failed.append(motor)
            errors[motor] = error

    return ReleaseReport(
        attempted=tuple(attempted),
        released=tuple(released),
        failed=tuple(failed),
        errors=errors,
    )


class TorqueGuard:
    """A context manager that owns the motors it energised, and lets go if things go wrong.

    Wrap a **whole gated motion verb** in one of these. On a clean exit it does
    nothing at all — no bus traffic, torque exactly as the verb left it, so
    ``gentle_move``'s stop-and-hold and any deliberate holding pose survive. On
    an abnormal exit it de-energises every motor it owns and then gets out of the
    way, letting the original exception continue to propagate untouched.

    See the module docstring for the hardware failure this exists to prevent and
    for why the release is per-motor independent.

    Usage::

        with torque_guard(bus, joint_ids, on_release=announce) as guard:
            gentle_move(bus, motor, target, ..., allow_motion=True)
            guard.own(other_motor)   # claim motors energised later in the run

    Parameters
    ----------
    bus:
        An open :class:`~arm101.hardware.bus.MotorBus` (real or fake).
    motors:
        Motors the guard owns from the outset. Usually every joint the verb
        *may* energise: claiming a motor that never gets energised is free (the
        release is a no-op on a limp motor), whereas failing to claim one that
        does is precisely the bug.
    on_release:
        Optional callback, invoked with the :class:`ReleaseReport` immediately
        after a release — a verb uses it to tell the operator the arm was
        de-energised while the exception is still unwinding. If it raises, the
        failure is suppressed — with the same ``SystemExit``-re-raises,
        everything-else-swallowed asymmetry as the release sweep itself (see
        :func:`_invoke_release_hook`) — so a broken diagnostic must never
        become the error the user sees instead of the real one, not even when
        the hook's own failure is a second ``KeyboardInterrupt``.

    Attributes
    ----------
    report:
        ``None`` until a release happens (so ``None`` *is* the assertion that
        torque was left alone), then the :class:`ReleaseReport` for it.
    """

    def __init__(
        self,
        bus: "MotorBus",
        motors: Iterable[int] = (),
        *,
        on_release: ReleaseHook | None = None,
    ) -> None:
        self._bus = bus
        # dict-as-ordered-set: dedupes while preserving claim order, so the
        # release sweeps motors in roughly the order they were energised.
        self._motors: dict[int, None] = {}
        self._on_release = on_release
        self.report: ReleaseReport | None = None
        self.own(*motors)

    @property
    def motors(self) -> tuple[int, ...]:
        """The motors this guard owns, in claim order."""
        return tuple(self._motors)

    def own(self, *motors: int) -> None:
        """Claim *motors* — the guard will de-energise them on an abnormal exit.

        Every id is validated **before** any is recorded, so a bad id in a batch
        cannot leave the guard half-updated. Claiming a motor twice is a no-op;
        it is still released exactly once.

        Raises
        ------
        CliError(EXIT_USER_ERROR)
            If any id is not a positive integer. See :func:`_require_motor_id`.
        """
        for motor in motors:
            _require_motor_id(motor)
        for motor in motors:
            self._motors.setdefault(motor, None)

    def disown(self, *motors: int) -> None:
        """Stop owning *motors* — the guard will no longer try to release them.

        The narrow counterpart to :meth:`own`, and it exists for exactly one
        situation: a motor that **changes address mid-run**. ``arm setup`` writes
        a new servo id into EEPROM (addr 5), so the moment that write lands the
        device stops answering at the id the guard claimed and starts answering
        at the new one. Keeping the old id owned would make the release sweep
        address a servo that no longer exists — the write would fail, and the
        guard would then tell the operator a motor "did not respond and may
        still be energised" when in truth it is limp and merely renamed. A false
        alarm on a safety report is corrosive: it teaches a human to ignore the
        one line that must never be ignored.

        Unknown ids are ignored (idempotent) — a caller disowning a motor it
        never claimed is expressing an intent the guard already satisfies.

        This is NOT a way to opt a motor out of the safety net for convenience.
        Disown a motor only when it is genuinely unreachable *at that address*;
        if it is still on the bus under a new id, claim the new id with
        :meth:`own` in the same breath.
        """
        for motor in motors:
            self._motors.pop(motor, None)

    def release(self) -> ReleaseReport:
        """De-energise every owned motor now, recording the outcome in :attr:`report`.

        Called automatically on an abnormal exit; exposed for a verb that decides
        mid-run that it wants the arm limp. Never raises for a bus failure.
        """
        self.report = release_torque(self._bus, self.motors)
        return self.report

    def __enter__(self) -> "TorqueGuard":
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: TracebackType | None,
    ) -> None:
        """Release iff the block is unwinding; always let the original exception through.

        The clean/abnormal split is exactly ``exc_type is None`` — which is what
        makes ``KeyboardInterrupt`` and ``SystemExit`` "abnormal" for free
        (Python hands ``__exit__`` any ``BaseException``, not just ``Exception``),
        and equally makes a plain ``return`` out of the guarded block "clean".

        Returns ``None`` on every path, and the return type says so: this guard
        makes the arm safe, it does not decide what the failure *means*.
        ``None`` is falsy, so the original exception always propagates —
        swallowing it would hide a dead bus behind a zero exit code. Typing this
        ``-> None`` rather than ``-> bool`` states that guarantee at the
        signature: there is no value it could return that would suppress
        anything.
        """
        if exc_type is None:
            return  # HOLD ON SUCCESS — torque is exactly as the verb left it.

        report = self.release()
        if self._on_release is not None:
            # A failing diagnostic must not replace the exception being raised
            # — including when the failure is a second KeyboardInterrupt. See
            # _invoke_release_hook for why contextlib.suppress(Exception) is
            # not enough on its own (KeyboardInterrupt/SystemExit are
            # BaseException, not Exception).
            _invoke_release_hook(self._on_release, report)


def torque_guard(
    bus: "MotorBus",
    motors: Iterable[int] = (),
    *,
    on_release: ReleaseHook | None = None,
) -> TorqueGuard:
    """Return a :class:`TorqueGuard` over *motors* — the idiomatic entry point.

    ``with torque_guard(bus, (1, 2, 3, 4, 5, 6)):`` reads as the intent ("for the
    duration of this block, I own these motors"), while the class is there for a
    caller that wants to hold the guard across a wider scope or inspect its
    :attr:`~TorqueGuard.report` afterwards. See :class:`TorqueGuard` for the
    parameters and the full contract.
    """
    return TorqueGuard(bus, motors, on_release=on_release)
