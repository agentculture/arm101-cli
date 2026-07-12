"""A servo whose shaft can ROLL — the model the rolling frame has to be tested against.

Why not :class:`tests._fakes.ServoModelBus`
-------------------------------------------
``ServoModelBus`` converts an incoming goal into a RAW encoder count at write
time and then drives the shaft toward it with a plain ``min``/``max`` clamp in
raw ticks. That is faithful for every move the arm has made so far — none of
them went near the raw 4095->0 rollover — and it is **wrong for exactly the
motion this task exists to enable**: a shaft at raw 4090, commanded 25 ticks
further out, has a raw goal of ``(4090 + 25) % 4096 == 19``, and a linear raw
clamp drives it *downward*, the long way round, 4071 ticks in the wrong
direction.

A real STS3215 does no such thing. Its control loop subtracts in the **reported**
frame::

    error = Goal_Position - Present_Position          (both REPORTED)

and drives the shaft in the direction of that error. The raw count underneath
wraps freely through 4095->0 while it does — the magnet does not care. The only
discontinuity the servo can see is the **reported** one, at ``raw == Ofs``, and
it is precisely that seam a goal command cannot be driven across. Which is the
whole premise of :mod:`arm101.hardware.rolling_frame`: keep the reported seam
half a turn away and the raw seam stops mattering at all.

So this bus models the servo the way the servo is:

* goals are stored in the **reported** frame, exactly as the register holds them;
* the error is a plain subtraction **in that frame**;
* the shaft advances ``ticks_per_poll`` toward it, **modulo 4096**;
* a **torque-off** joint does not move at all — which is what makes the "stale
  goal" hazard visible. Writing the offset register changes what
  ``Present_Position`` means; a ``Goal_Position`` written in the *old* frame now
  names a *different physical angle*, and the instant torque comes back the servo
  drives to it. The rolling frame prevents that by writing a hold-in-place goal
  before it lets go; ``test_a_stale_goal_really_would_lurch`` proves this fake
  would catch it if it didn't.

Time is polls, as everywhere else in this suite: one :meth:`read_info` call is
one poll interval, and the shaft only ever advances on one.

:attr:`net_travel` is the ground-truth odometer — the signed sum of every tick the
shaft has actually turned. It is the yardstick a rolling frame's ``displacement``
is checked against, and it is the one number that survives both seams: the raw
rollover the shaft goes through, and the offset rewrites the frame goes through.
"""

from __future__ import annotations

from arm101.hardware.bus import FakeBus
from arm101.hardware.ticks import ENCODER_TICKS

#: Load reported while the shaft is advancing freely. Well below
#: ``gentle_move``'s contact threshold, so free motion is never mistaken for a wall.
FREE_TRAVEL_LOAD: int = 60


class RollingServoBus(FakeBus):
    """A :class:`~arm101.hardware.bus.FakeBus` whose shaft rolls through the raw seam.

    Parameters
    ----------
    ticks_per_poll:
        Encoder ticks the shaft advances toward its goal per :meth:`read_info`
        call. Default 25.
    travel_load:
        ``present_load`` magnitude while advancing. Default
        :data:`FREE_TRAVEL_LOAD` — free motion, no contact.
    torque:
        Whether the motors start energised. Default ``False``: a servo that has
        just had its EEPROM written is limp, and a test that wants motion should
        have to say so (or call a mover that does).
    """

    def __init__(
        self,
        *args,
        ticks_per_poll: int = 25,
        travel_load: int = FREE_TRAVEL_LOAD,
        torque: bool = False,
        **kwargs,
    ) -> None:
        super().__init__(*args, **kwargs)
        if ticks_per_poll <= 0:
            raise ValueError(f"ticks_per_poll must be positive, got {ticks_per_poll}")
        self.ticks_per_poll = ticks_per_poll
        self.travel_load = travel_load
        #: motor -> goal, **in the reported frame** (what the register holds).
        self._reported_goals: dict[int, int] = {}
        #: motor -> is torque enabled. A limp joint does not chase its goal.
        self._torque: dict[int, bool] = {}
        self._default_torque = torque
        #: motor -> signed sum of every tick the shaft has turned (ground truth).
        self._net_travel: dict[int, int] = {}
        self.poll_log: list[dict[str, int]] = []

    # ------------------------------------------------------------------
    # Ground truth — no bus, no frame, no offset
    # ------------------------------------------------------------------

    def true_raw(self, motor: int) -> int:
        """The shaft's ACTUAL raw encoder count. Ground truth."""
        return self._positions.get(motor, 2048) % ENCODER_TICKS

    def net_travel(self, motor: int) -> int:
        """Signed ticks the shaft has actually turned since the bus was built.

        The odometer a rolling frame's ``displacement`` must agree with. Immune to
        both seams by construction: it is accumulated one small advance at a time,
        from the simulation itself, never reconstructed from two tick readings.
        """
        return self._net_travel.get(motor, 0)

    def torque_on(self, motor: int) -> bool:
        """Is *motor* currently energised?"""
        return self._torque.get(motor, self._default_torque)

    def reported_goal(self, motor: int) -> "int | None":
        """The goal the servo's ``Goal_Position`` register currently holds (reported frame)."""
        return self._reported_goals.get(motor)

    # ------------------------------------------------------------------
    # MotorBus surface
    # ------------------------------------------------------------------

    def enable_torque(self, motor: int, on: bool) -> None:
        """Energise (or relax) *motor*. A relaxed joint does not chase its goal."""
        super().enable_torque(motor, on)
        self._torque[motor] = bool(on)

    def write_goal_position(self, motor: int, position: int) -> None:
        """Record a goal **in the reported frame** — the frame the register lives in.

        ``super()`` rejects anything outside ``[0, 4095]``, which is the servo's
        own bound and the very wall the rolling frame exists to keep the joint
        away from: a goal past the end of the reported scale is not a long move,
        it is an impossible one.
        """
        super().write_goal_position(motor, position)
        self._reported_goals[motor] = position

    def read_info(self, motor: int) -> dict:
        """Advance the simulation one poll interval, then report what it reads."""
        snapshot = super().read_info(motor)
        raw, load = self._advance(motor)
        reported = self._reported_position(motor, raw)
        snapshot["present_position"] = reported
        snapshot["present_load"] = load
        self.poll_log.append({"motor": motor, "present_position": reported, "present_load": load})
        return snapshot

    # ------------------------------------------------------------------
    # The simulation
    # ------------------------------------------------------------------

    def _advance(self, motor: int) -> "tuple[int, int]":
        """Move the shaft one poll interval toward its goal. Returns ``(raw, load)``."""
        raw = self.true_raw(motor)
        goal = self._reported_goals.get(motor)
        if goal is None or not self.torque_on(motor):
            return raw, 0

        # The servo's own arithmetic: a PLAIN subtraction, in the REPORTED frame.
        # It cannot see the raw seam and it cannot cross the reported one.
        error = goal - self._reported_position(motor, raw)
        if error == 0:
            return raw, 0

        direction = 1 if error > 0 else -1
        advance = direction * min(self.ticks_per_poll, abs(error))
        raw = (raw + advance) % ENCODER_TICKS
        self._positions[motor] = raw
        self._net_travel[motor] = self.net_travel(motor) + advance
        return raw, self.travel_load
