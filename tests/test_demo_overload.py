"""Tests for demo.py propagating gentle_move's graceful overload through the sweep.

Covers (arm101.hardware.demo.demo_sweep):

* ``gentle_move`` no longer raises on a mid-move servo overload — it CATCHES
  ``OverloadError`` internally and RETURNS its result dict with
  ``overloaded=True`` (see tests/test_gentle_overload.py). ``demo_sweep`` must
  treat that returned flag the same way it already treats a returned
  ``contacted=True``: record it on the joint's report, stop the WHOLE sweep
  immediately (no later joint is visited — no ``read_info``, no writes), and
  never raise.
* Happy path (no overload, no contact) is unchanged: every joint report gains
  an ``overloaded=False`` field, and the top-level report gains
  ``aborted_on_overload=False`` / ``overloaded_joint=None``.

TDD: written before the corresponding demo.py changes; they must fail against
the current code (missing ``overloaded`` keys) and drive the implementation.
Builds on the t2/t3 gentle_move overload-return contract already merged into
this branch's base.
"""

from __future__ import annotations

from arm101.hardware.bus import FakeBus, OverloadError
from arm101.hardware.demo import demo_sweep

# ---------------------------------------------------------------------------
# Local test double — NOT in bus.py (per task scope, kept local to this
# file), mirroring the RampLoadBus double already in tests/test_demo.py for
# the analogous "contact" scenario. Raises OverloadError from
# write_goal_position for ONE configured motor, independent of FakeBus's
# global op-counting seam (fail_with_overload_on_op / overload_after_ops) —
# needed so a multi-joint sweep test can arm exactly one joint's overload
# without hand-counting every bus operation the OTHER joints perform first.
# ---------------------------------------------------------------------------


class PerMotorOverloadBus(FakeBus):
    """FakeBus that raises OverloadError from write_goal_position for ONE motor.

    ``overload_motor`` is the motor id that should overload; ``overload_on_write``
    (1-indexed, default 1) is which write_goal_position call FOR THAT MOTOR
    raises. Also tracks every ``read_info`` call (by motor id) in
    :attr:`read_info_calls` so a test can assert a joint AFTER the overloaded
    one was never even read, matching demo_sweep's "no read_info, no writes"
    contract for a skipped joint.
    """

    def __init__(self, *args, overload_motor, overload_on_write: int = 1, **kwargs):
        super().__init__(*args, **kwargs)
        self._overload_motor = overload_motor
        self._overload_on_write = overload_on_write
        self._writes_for_overload_motor = 0
        self.read_info_calls: list[int] = []

    def read_info(self, motor: int) -> dict:
        self.read_info_calls.append(motor)
        return super().read_info(motor)

    def write_goal_position(self, motor: int, position: int) -> None:
        if motor == self._overload_motor:
            self._writes_for_overload_motor += 1
            if self._writes_for_overload_motor >= self._overload_on_write:
                raise OverloadError(motor=motor, error_byte=32)
        super().write_goal_position(motor, position)


# ---------------------------------------------------------------------------
# Happy path: overloaded=False everywhere, existing keys untouched
# ---------------------------------------------------------------------------


def test_demo_sweep_happy_path_sets_overloaded_false_on_every_joint_and_top_level():
    info = {
        1: {"min_angle": 0, "max_angle": 4095, "present_position": 2048},
        2: {"min_angle": 1000, "max_angle": 3000, "present_position": 2000},
        3: {"min_angle": 1800, "max_angle": 2200, "present_position": 2000},
    }
    bus = FakeBus(positions={1: 2048, 2: 2000, 3: 2000}, info=info)
    bus.open()
    joints = {"shoulder": 1, "elbow": 2, "wrist": 3}

    report = demo_sweep(bus, joints, allow_motion=True)

    # New top-level keys.
    assert report["aborted_on_overload"] is False
    assert report["overloaded_joint"] is None
    # Existing top-level keys/behaviour untouched.
    assert report["aborted_on_contact"] is False
    assert report["aborted_joint"] is None
    assert set(report["joints"]) == {"shoulder", "elbow", "wrist"}

    for name in joints:
        joint_report = report["joints"][name]
        assert joint_report["overloaded"] is False
        assert joint_report["contacted"] is False


# ---------------------------------------------------------------------------
# Overload mid-sweep -> clean abort, later joint never visited
# ---------------------------------------------------------------------------


def test_demo_sweep_aborts_cleanly_on_overload_and_skips_later_joints():
    info = {
        1: {"min_angle": 0, "max_angle": 4095, "present_position": 2048},
        2: {"min_angle": 0, "max_angle": 4095, "present_position": 2048},
        3: {"min_angle": 0, "max_angle": 4095, "present_position": 2048},
    }
    bus = PerMotorOverloadBus(
        positions={1: 2048, 2: 2048, 3: 2048},
        info=info,
        overload_motor=2,  # only the elbow (motor 2) overloads
    )
    bus.open()
    joints = {"shoulder": 1, "elbow": 2, "wrist": 3}

    report = demo_sweep(bus, joints, allow_motion=True)

    assert report["aborted_on_overload"] is True
    assert report["overloaded_joint"] == "elbow"
    # The contact-abort flags stay at their "no contact" defaults — this
    # sweep aborted on overload, not contact.
    assert report["aborted_on_contact"] is False
    assert report["aborted_joint"] is None

    assert report["joints"]["shoulder"]["overloaded"] is False
    assert report["joints"]["elbow"]["overloaded"] is True

    # The wrist joint (after the overloaded elbow in sweep order) is absent
    # from the report entirely, and was never even read, let alone written.
    assert "wrist" not in report["joints"]
    assert 3 not in bus.read_info_calls

    motors_written = {w["motor"] for w in bus.position_writes}
    assert 1 in motors_written  # shoulder was swept before the abort
    assert 3 not in motors_written  # wrist was never touched
    motors_read = set(bus.read_info_calls)
    assert 1 in motors_read
    assert 2 in motors_read
    assert 3 not in motors_read


def test_demo_sweep_overload_does_not_raise():
    """Overload is an expected, safe outcome — demo_sweep must NOT raise."""
    info = {1: {"min_angle": 0, "max_angle": 4095, "present_position": 2048}}
    bus = PerMotorOverloadBus(positions={1: 2048}, info=info, overload_motor=1)
    bus.open()

    # No pytest.raises here on purpose — a raise would fail this test.
    report = demo_sweep(bus, {"shoulder": 1}, allow_motion=True)

    assert isinstance(report, dict)
    assert report["aborted_on_overload"] is True
    assert report["overloaded_joint"] == "shoulder"
    assert report["joints"]["shoulder"]["overloaded"] is True


def test_demo_sweep_overload_stops_the_joints_own_second_target_too():
    """The joint that overloads never attempts its SECOND planned target either."""
    info = {1: {"min_angle": 0, "max_angle": 4095, "present_position": 2048}}
    bus = PerMotorOverloadBus(positions={1: 2048}, info=info, overload_motor=1)
    bus.open()

    report = demo_sweep(bus, {"shoulder": 1}, allow_motion=True)

    joint_report = report["joints"]["shoulder"]
    assert joint_report["overloaded"] is True
    # Only the first target was attempted -- the overload aborted before the
    # second (planned_targets has 2 entries; targets_attempted has fewer).
    assert len(joint_report["targets_attempted"]) < len(joint_report["planned_targets"])


# ---------------------------------------------------------------------------
# Single-joint overload via FakeBus's own documented op-counting seam
# (fail_with_overload_on_op), matching the seam's contract as already
# exercised directly against gentle_move in tests/test_gentle_overload.py.
# ---------------------------------------------------------------------------


def test_demo_sweep_single_joint_overload_via_fail_with_overload_on_op_seam():
    info = {1: {"min_angle": 0, "max_angle": 4095, "present_position": 2048}}
    bus = FakeBus(positions={1: 2048}, info=info).fail_with_overload_on_op(8)
    bus.open()

    # Op 1: demo_sweep's own bus.read_info(1) (min/max/present_position).
    # Ops 2-7: gentle_move's setup for the "low" target -- read_torque_limit,
    # write_torque_limit (cap), write_acceleration, write_goal_speed,
    # enable_torque, read_info (start_position). Op 8 is the FIRST
    # write_goal_position inside gentle_move's step loop.
    report = demo_sweep(bus, {"shoulder": 1}, allow_motion=True)

    assert report["aborted_on_overload"] is True
    assert report["overloaded_joint"] == "shoulder"
    assert report["joints"]["shoulder"]["overloaded"] is True
    # clear_overload() (called inside gentle_move's except handler) disarms
    # the seam -- proves the recovery path actually ran, not merely a raise
    # that happened to get swallowed.
    assert bus.overload_after_ops is None
