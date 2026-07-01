"""Tests for arm101.hardware.arm_read — retry-tolerant whole-arm read snapshot.

TDD: these tests were written before arm_read.py existed and drive the
implementation. A local ``FlakyBus`` test double (subclassing
:class:`~arm101.hardware.bus.FakeBus`) simulates transient and permanent
``read_info`` failures so the retry/health logic can be exercised without
hardware.
"""

from __future__ import annotations

from arm101.cli._errors import EXIT_ENV_ERROR, CliError
from arm101.hardware.arm_read import JointReading, read_arm
from arm101.hardware.bus import FakeBus

JOINTS = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}


class FlakyBus(FakeBus):
    """FakeBus that fails ``read_info`` for chosen motors on the first N calls.

    Parameters
    ----------
    fail_counts:
        Mapping of motor id -> number of times that motor's ``read_info``
        should raise before succeeding. Use ``float("inf")`` (or any count
        >= the caller's total attempt budget) to simulate a permanently
        failing motor.
    """

    def __init__(self, *args, fail_counts: dict[int, float] | None = None, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._fail_counts = dict(fail_counts) if fail_counts else {}
        self._fail_calls: dict[int, int] = {}

    def read_info(self, motor: int) -> dict[str, int]:
        budget = self._fail_counts.get(motor, 0)
        made = self._fail_calls.get(motor, 0)
        if made < budget:
            self._fail_calls[motor] = made + 1
            raise CliError(
                code=EXIT_ENV_ERROR,
                message=f"Simulated transient read failure for motor {motor}.",
                remediation="Retry the read.",
            )
        return super().read_info(motor)


# ---------------------------------------------------------------------------
# All-healthy snapshot
# ---------------------------------------------------------------------------


def test_read_arm_all_healthy_returns_six_ok_joints():
    """A fully responsive bus returns six joints, all health 'ok'."""
    bus = FakeBus(positions={i: 1000 + i for i in range(1, 7)})
    bus.open()

    readings = read_arm(bus, JOINTS)

    assert len(readings) == 6
    assert [r.joint for r in readings] == list(JOINTS.keys())
    for r in readings:
        assert isinstance(r, JointReading)
        assert r.health == "ok"
        assert r.position is not None


def test_read_arm_all_healthy_fields_populated_from_read_info():
    """Each ok joint's fields come straight from the FakeBus's canned info."""
    bus = FakeBus(positions={1: 1234})
    bus.open()

    readings = read_arm(bus, {"shoulder_pan": 1})
    r = readings[0]

    info = bus.read_info(1)
    assert r.position == info["present_position"] == 1234
    assert r.load == info["present_load"]
    assert r.speed == info["present_speed"]
    assert r.voltage == info["present_voltage"]
    assert r.temperature == info["present_temperature"]
    assert r.torque == info["torque_enable"]


# ---------------------------------------------------------------------------
# One joint always failing -> failed, others still returned
# ---------------------------------------------------------------------------


def test_read_arm_one_joint_always_failing_marks_failed_others_ok():
    """A joint whose reads keep raising is 'failed' with no data; the rest are 'ok'."""
    bus = FlakyBus(
        positions={i: 1000 + i for i in range(1, 7)},
        fail_counts={3: float("inf")},
    )
    bus.open()

    readings = read_arm(bus, JOINTS, retries=2)

    by_joint = {r.joint: r for r in readings}
    assert len(readings) == 6

    failed = by_joint["elbow_flex"]
    assert failed.motor_id == 3
    assert failed.health == "failed"
    assert failed.position is None
    assert failed.load is None
    assert failed.speed is None
    assert failed.voltage is None
    assert failed.temperature is None
    assert failed.torque is None

    for joint_name, r in by_joint.items():
        if joint_name == "elbow_flex":
            continue
        assert r.health == "ok"
        assert r.position is not None


def test_read_arm_never_raises_even_when_a_joint_always_fails():
    """No exception escapes read_arm even though one joint's reads always raise."""
    bus = FlakyBus(
        positions={i: 1000 + i for i in range(1, 7)},
        fail_counts={6: float("inf")},
    )
    bus.open()

    # Must not raise.
    readings = read_arm(bus, JOINTS, retries=2)
    assert len(readings) == 6


# ---------------------------------------------------------------------------
# One joint fails once then succeeds -> partial
# ---------------------------------------------------------------------------


def test_read_arm_joint_fails_once_then_succeeds_is_partial():
    """A joint that fails once but then succeeds within the retry budget is 'partial'."""
    bus = FlakyBus(
        positions={i: 1000 + i for i in range(1, 7)},
        fail_counts={2: 1},
    )
    bus.open()

    readings = read_arm(bus, JOINTS, retries=2)
    by_joint = {r.joint: r for r in readings}

    partial = by_joint["shoulder_lift"]
    assert partial.motor_id == 2
    assert partial.health == "partial"
    assert partial.position is not None
    assert partial.load is not None

    for joint_name, r in by_joint.items():
        if joint_name == "shoulder_lift":
            continue
        assert r.health == "ok"


def test_read_arm_joint_exhausts_retry_budget_is_failed():
    """A joint that fails (retries + 1) times in a row exhausts the budget -> failed."""
    bus = FlakyBus(
        positions={i: 1000 + i for i in range(1, 7)},
        fail_counts={4: 3},  # retries=2 -> 3 total attempts -> all 3 fail
    )
    bus.open()

    readings = read_arm(bus, JOINTS, retries=2)
    by_joint = {r.joint: r for r in readings}

    assert by_joint["wrist_flex"].health == "failed"
    assert by_joint["wrist_flex"].position is None


# ---------------------------------------------------------------------------
# Field mapping correctness
# ---------------------------------------------------------------------------


def test_field_mapping_present_keys_map_to_joint_reading_fields():
    """present_position/load/speed/voltage/temperature and torque_enable map correctly."""
    info_override = {
        1: {
            "present_position": 111,
            "present_load": 222,
            "present_speed": 333,
            "present_voltage": 44,
            "present_temperature": 55,
            "torque_enable": 1,
        }
    }
    bus = FakeBus(info=info_override)
    bus.open()

    readings = read_arm(bus, {"shoulder_pan": 1})
    r = readings[0]

    assert r.position == 111
    assert r.load == 222
    assert r.speed == 333
    assert r.voltage == 44
    assert r.temperature == 55
    assert r.torque == 1


# ---------------------------------------------------------------------------
# Misc API shape
# ---------------------------------------------------------------------------


def test_read_arm_default_retries_is_two():
    """retries defaults to 2 (a joint failing exactly twice then succeeding is partial)."""
    bus = FlakyBus(positions={1: 5000}, fail_counts={1: 2})
    bus.open()

    readings = read_arm(bus, {"shoulder_pan": 1})
    assert readings[0].health == "partial"
    assert readings[0].position == 5000


def test_read_arm_empty_joints_returns_empty_list():
    """An empty joints mapping returns an empty list without error."""
    bus = FakeBus()
    bus.open()

    assert read_arm(bus, {}) == []


def test_joint_reading_is_a_dataclass_with_expected_fields():
    """JointReading exposes joint, motor_id, health, and the six register fields."""
    r = JointReading(joint="gripper", motor_id=6, health="ok")
    assert r.joint == "gripper"
    assert r.motor_id == 6
    assert r.health == "ok"
    for field in ("position", "load", "speed", "voltage", "temperature", "torque"):
        assert hasattr(r, field)
        assert getattr(r, field) is None
