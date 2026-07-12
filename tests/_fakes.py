"""Shared test doubles that model the SO-101's servos honestly.

:class:`ServoModelBus` exists because the previous fake bus in
``tests/test_gentle.py`` *encoded the bug as the spec*: its
``write_goal_position`` teleported ``present_position`` straight to the
commanded goal and materialised load on the very same call. Real STS3215
servos refuse to do either — and because the fake did, the whole suite was
structurally blind to ``gentle_move``'s two real defects (measured on the
follower arm, 2026-07-12): sampling ``present_load`` ~1 ms after a goal-write
(before the servo has mechanically responded), and reporting a *commanded*
tick as ``final_position`` while the joint was still 400 ticks away.

The model
---------

**Time is measured in polls.** One :meth:`ServoModelBus.read_info` call is one
poll interval of wall-clock time; the servo advances
:attr:`~ServoModelBus.ticks_per_poll` ticks toward its goal per poll and *only*
on a poll. ``write_goal_position`` records a goal and moves nothing.

That mapping is what makes the fake faithful. On hardware a 400-tick move at
speed 150 takes ~900 ms while the pre-fix loop's ~16 goal-writes complete in
~71 ms: the goal-writes race far ahead of the shaft, and the function returns
long before the joint has gone anywhere. Here, the default
``ticks_per_poll=10`` against ``gentle_move``'s default ``step=25`` reproduces
exactly that ratio — a loop that polls once per commanded step *cannot* keep up
with its own goals, so it returns with the servo at ~40% of the commanded
travel, having watched none of the real motion. A loop that polls until the
joint measurably arrives spends the polls, and sees everything.

(10 ticks/poll is not arbitrary: 400 ticks of real travel in ~900 ms at a
~20 ms poll interval is ~45 polls, i.e. ~9 ticks per poll.)

**Load is a function of state, and it saturates at ``Torque_Limit``.**
Confirmed twice on the bench (2026-07-12): with ``Torque_Limit=300`` the
gripper's ``present_load`` climbed and pinned at exactly 300; with
``Torque_Limit=600`` it pinned at exactly 600. The register cannot read above
the servo's active torque limit. Since :func:`arm101.hardware.gentle.gentle_move`
caps ``Torque_Limit`` to ``_CONTACT_TORQUE_LIMIT`` (500) for the duration of a
move, load can never exceed 500 during one — so **a contact threshold at or
above the active torque limit can never fire**, and every per-joint threshold
must sit strictly below the cap. The fake clamps every reported load to the
motor's currently-written ``Torque_Limit`` so no test can be built against a
signal the hardware is incapable of producing.

The four regimes:

======================  ==================================================
``idle``                at goal / no goal — load ``idle_load`` (0)
``travelling``          advancing freely — load from ``travel_load``, a
                        profile indexed by consecutive travelling polls, so
                        the acceleration transient (wrist_roll peaks ~272 in
                        FREE space) can be modelled and then settle
``contact``             pressing into an obstacle: the servo *creeps* into it
                        while load ramps, then stalls with load saturated at
                        ``Torque_Limit``
``friction_stall``      ``Torque_Limit`` below the joint's own gear friction:
                        the joint cannot move AT ALL, even in free space, and
                        pins its load at the limit
======================  ==================================================

The ``contact`` regime is a compliant contact, not a brick wall, because the
bench recording is not a brick wall (gripper closing on a soft object,
``Torque_Limit=600``)::

    t (ms)  position   load
       133      3076     60     <- travelling freely
       693      3022    320
       809      3014    388     <- crosses the 380 threshold
       954      3011    548
      1073      3001    600     <- stalled, load saturated at Torque_Limit
      1512      3001    600

The joint is still creeping (3022 -> 3001) while load is already well past the
threshold: the stall is gradual, not instant. So an obstacle here has *give* —
the servo penetrates past the obstacle's contact point by
``Torque_Limit // obstacle_stiffness`` ticks, load rising linearly with
penetration, until it can push no further and the load pins at the limit. A
contact rule of "load over threshold AND position not advancing" must tolerate
those few ticks of creep-under-load.

``friction_stall`` is modelled because it was also proven on the bench:
``Torque_Limit=300`` sits *below* the gripper's own gear friction (~320), so the
gripper stalled in FREE SPACE with its load pinned at the limit — indistinguishable
from a real contact by position and load alone. See
``tests/test_gentle.py::test_under_torqued_joint_stalls_in_free_space_like_a_contact``.
"""

from __future__ import annotations

from collections.abc import Sequence

from arm101.hardware.bus import FakeBus

#: Bit 10 of Present_Load (addr 60) is the load *direction*, not magnitude —
#: see :func:`arm101.hardware.bus.load_magnitude`. Only set when
#: ``direction_bit=True``, so a test can prove callers mask it off.
LOAD_DIRECTION_BIT: int = 0x400

#: Widest value the 10-bit Present_Load magnitude field can hold.
MAX_LOAD_MAGNITUDE: int = 0x3FF

#: What :meth:`FakeBus.read_torque_limit` reports for a motor whose
#: Torque_Limit has never been written (mirrors ``FakeBus``'s own default).
DEFAULT_TORQUE_LIMIT: int = 1000

#: Poll state names, recorded in :attr:`ServoModelBus.poll_log`.
IDLE = "idle"
TRAVELLING = "travelling"
CONTACT = "contact"
FRICTION_STALL = "friction_stall"


class ServoModelBus(FakeBus):
    """A :class:`~arm101.hardware.bus.FakeBus` that models servo travel latency.

    See the module docstring for the physics. In short: the servo does NOT
    arrive on the write that commands it, and load exists only while it is
    actually moving or actually pushing against something — the two things the
    old teleporting fake got backwards.

    Parameters
    ----------
    ticks_per_poll:
        Encoder ticks the shaft advances toward its goal per
        :meth:`read_info` call (= per poll interval of simulated time).
        Default 10, calibrated to the measured ~900 ms / 400-tick travel at
        ``gentle_move``'s default speed of 150.
    travel_load:
        Load magnitude while advancing freely. An ``int`` for a flat profile,
        or a sequence indexed by consecutive travelling polls (clamped to the
        last entry) to model the acceleration transient — e.g.
        ``(272, 272, 200, 120, 60)`` for wrist_roll, whose FREE-motion peak of
        272 sits above the current default contact threshold of 250 and must
        not be mistaken for contact.
    idle_load:
        Load magnitude when the joint is at its goal (or has none). Default 0.
    obstacle_stiffness:
        Load units gained per tick of penetration into an obstacle. The servo
        can therefore penetrate ``Torque_Limit // obstacle_stiffness`` ticks
        before its load saturates at the limit and it stops advancing — the
        "gradual stall" the bench recording shows. Default 20 (25 ticks of
        creep at ``gentle_move``'s 500 cap).
    friction_load:
        The load the joint must overcome to move at all (gear friction). If
        the active ``Torque_Limit`` is at or below this, the joint cannot move
        even in free space: it stalls with load pinned at the limit. Default 0
        (frictionless), so it never perturbs a test that does not ask for it.
    direction_bit:
        When ``True``, Present_Load is reported with bit 10 set for loads in
        the negative direction, exactly as the STS3215 does. Default ``False``
        so assertions read cleanly; flip it on to prove a caller masks it.

    Attributes
    ----------
    poll_log:
        One entry per :meth:`read_info` call::

            {"motor", "present_position", "present_load", "state"}

        The measurement record. ``final_position``/``contact_position``/
        ``contact_load`` are only honest if they can be traced back to an
        entry here.
    """

    def __init__(
        self,
        *args,
        ticks_per_poll: int = 10,
        travel_load: "int | Sequence[int]" = 60,
        idle_load: int = 0,
        obstacle_stiffness: int = 20,
        friction_load: int = 0,
        direction_bit: bool = False,
        **kwargs,
    ) -> None:
        super().__init__(*args, **kwargs)
        if ticks_per_poll <= 0:
            raise ValueError(f"ticks_per_poll must be positive, got {ticks_per_poll}")
        if obstacle_stiffness <= 0:
            raise ValueError(f"obstacle_stiffness must be positive, got {obstacle_stiffness}")

        self.ticks_per_poll = ticks_per_poll
        if isinstance(travel_load, int):
            self.travel_load: tuple[int, ...] = (travel_load,)
        else:
            self.travel_load = tuple(travel_load)
        if not self.travel_load:
            raise ValueError("travel_load profile must not be empty")
        self.idle_load = idle_load
        self.obstacle_stiffness = obstacle_stiffness
        self.friction_load = friction_load
        self.direction_bit = direction_bit

        # motor -> most recently COMMANDED goal (the servo has not moved yet).
        self._goals: dict[int, int] = {}
        # motor -> (contact_point, +1 blocks increasing motion / -1 decreasing).
        self._obstacles: dict[int, tuple[int, int]] = {}
        # motor -> consecutive polls spent advancing (indexes travel_load).
        self._travel_polls: dict[int, int] = {}
        self.poll_log: list[dict[str, object]] = []

    # ------------------------------------------------------------------
    # Test-facing introspection ("what is the arm ACTUALLY doing?")
    # ------------------------------------------------------------------

    @property
    def poll_count(self) -> int:
        """How many :meth:`read_info` polls have been taken (= simulated time)."""
        return len(self.poll_log)

    def true_position(self, motor: int) -> int:
        """The simulated shaft's ACTUAL position — ground truth, no bus involved.

        A ``final_position`` that disagrees with this is a lie the caller told
        about the physical arm.
        """
        return self._positions.get(motor, 2048)

    def polled_positions(self, motor: int = 1) -> list[int]:
        """Every ``present_position`` this bus ever REPORTED for *motor*."""
        return [e["present_position"] for e in self.poll_log if e["motor"] == motor]

    def polled_loads(self, motor: int = 1) -> list[int]:
        """Every ``present_load`` this bus ever REPORTED for *motor* (raw register)."""
        return [e["present_load"] for e in self.poll_log if e["motor"] == motor]

    def active_torque_limit(self, motor: int) -> int:
        """The Torque_Limit currently in force — the ceiling on ``present_load``.

        Read straight off the fake's register store rather than via
        :meth:`read_torque_limit`, which would tick the overload-simulation
        counter and corrupt the ``fail_with_overload_on_op`` seam.
        """
        return self._torque_limits.get(motor, DEFAULT_TORQUE_LIMIT)

    def place_obstacle(self, motor: int, position: int) -> "ServoModelBus":
        """Put a compliant obstacle where *motor* first meets resistance at *position*.

        The blocking direction is inferred from where the shaft is right now:
        an obstacle above the current position blocks increasing motion, one
        below blocks decreasing motion. Retreating away from it is always free
        — which is what makes back-off-and-hold testable.

        The servo can still creep ``Torque_Limit // obstacle_stiffness`` ticks
        *past* this point as it compresses the obstacle, load ramping all the
        way, before it stalls with load saturated at the limit. Returns
        ``self`` so it can be chained onto the constructor call.
        """
        current = self.true_position(motor)
        if position == current:
            raise ValueError("obstacle must not sit exactly on the joint's current position")
        self._obstacles[motor] = (position, 1 if position > current else -1)
        return self

    # ------------------------------------------------------------------
    # MotorBus surface
    # ------------------------------------------------------------------

    def write_goal_position(self, motor: int, position: int) -> None:
        """Record a GOAL. The servo does not move — that takes polls (time).

        *position* arrives in the **corrected frame** — the same frame the servo
        reports in — so it is converted back to a raw encoder count for the
        simulation, which runs on actual shaft positions
        (``Actual = Present + Homing_Offset``). Goals and feedback living in
        different frames is precisely the failure that would make an encoder
        re-zero worse than useless, so the fake refuses to model it. At the
        default zero offset this is the identity and nothing changes.
        """
        super().write_goal_position(motor, position)
        self._goals[motor] = self._actual_position(motor, position)

    def read_info(self, motor: int) -> dict:
        """Advance the simulation one poll interval, then report what it reads.

        The advance happens *after* ``super().read_info`` so that an armed
        overload seam (which raises from there) leaves the simulated shaft
        untouched: a read that failed observed nothing.

        The simulated position is in raw encoder counts, so it goes back out
        through the same offset funnel every other reported position uses
        (:meth:`~arm101.hardware.bus.FakeBus._reported_position`). Skipping that
        would make ``read_info``'s ``present_position`` disagree with
        ``read_position`` — two views of ONE register (addr 56) — and a test
        could then pass against a servo that cannot exist.
        """
        snapshot = super().read_info(motor)
        position, load, state = self._poll(motor)
        reported = self._reported_position(motor, position)
        snapshot["present_position"] = reported
        snapshot["present_load"] = load
        self.poll_log.append(
            {
                "motor": motor,
                "present_position": reported,
                "present_load": load,
                "state": state,
            }
        )
        return snapshot

    # ------------------------------------------------------------------
    # The simulation
    # ------------------------------------------------------------------

    def _reachable(self, motor: int, goal: int) -> int:
        """Clamp *goal* to the deepest position the servo can actually reach.

        An obstacle limits travel to its contact point plus the penetration
        its compliance allows at the CURRENT Torque_Limit — lower the limit and
        the joint cannot push as deep, exactly as on hardware.
        """
        obstacle = self._obstacles.get(motor)
        if obstacle is None:
            return goal
        contact_point, blocks = obstacle
        give = self.active_torque_limit(motor) // self.obstacle_stiffness
        if blocks > 0:
            return min(goal, contact_point + give)
        return max(goal, contact_point - give)

    def _poll(self, motor: int) -> tuple[int, int, str]:
        """Advance one poll interval; return ``(position, raw_load, state)``."""
        position = self.true_position(motor)
        goal = self._goals.get(motor, position)
        limit = self.active_torque_limit(motor)

        # Gear friction the servo is not allowed enough torque to overcome:
        # the joint cannot move at all, even through empty air, and pins its
        # load at the limit. Proven on the bench with Torque_Limit=300 against
        # the gripper's ~320 of friction.
        if goal != position and limit <= self.friction_load:
            self._travel_polls[motor] = 0
            sign = 1 if goal > position else -1
            return position, self._encode(motor, limit, sign), FRICTION_STALL

        reachable = self._reachable(motor, goal)
        if reachable > position:
            new_position = min(position + self.ticks_per_poll, reachable)
        elif reachable < position:
            new_position = max(position - self.ticks_per_poll, reachable)
        else:
            new_position = position
        self._positions[motor] = new_position

        advanced = new_position - position
        penetration = self._penetration(motor, new_position)

        if penetration:
            # Creeping into (or stalled against) the obstacle: load ramps with
            # penetration and saturates at Torque_Limit. This can be true while
            # `advanced` is still non-zero — on the bench the stall is gradual.
            self._travel_polls[motor] = 0
            magnitude = self.obstacle_stiffness * penetration
            sign = self._obstacles[motor][1]
            return new_position, self._encode(motor, magnitude, sign), CONTACT

        if advanced:
            self._travel_polls[motor] = self._travel_polls.get(motor, 0) + 1
            index = min(self._travel_polls[motor] - 1, len(self.travel_load) - 1)
            sign = 1 if advanced > 0 else -1
            return new_position, self._encode(motor, self.travel_load[index], sign), TRAVELLING

        self._travel_polls[motor] = 0
        return new_position, self._encode(motor, self.idle_load, 1), IDLE

    def _penetration(self, motor: int, position: int) -> int:
        """Ticks *motor* has pushed past its obstacle's contact point (0 if none)."""
        obstacle = self._obstacles.get(motor)
        if obstacle is None:
            return 0
        contact_point, blocks = obstacle
        return max(0, (position - contact_point) * blocks)

    def _encode(self, motor: int, magnitude: int, sign: int) -> int:
        """Clamp a load magnitude to what the servo could actually report.

        Two ceilings, both real: the 10-bit magnitude field, and — the one that
        matters here — the motor's active Torque_Limit, which ``present_load``
        provably saturates at (limit 300 -> pins at 300; limit 600 -> pins at
        600). Any contact threshold at or above the active limit is therefore
        unfirable, by physics, not by policy.
        """
        magnitude = max(0, min(magnitude, MAX_LOAD_MAGNITUDE, self.active_torque_limit(motor)))
        if self.direction_bit and sign < 0:
            return magnitude | LOAD_DIRECTION_BIT
        return magnitude
