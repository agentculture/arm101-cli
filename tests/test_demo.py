"""Tests for arm101.hardware.demo — scripted safe-exploration sweep.

TDD: written before demo.py existed. Drives demo_sweep's contract: sweep
every joint through a SAFE FRACTION of its calibrated [min_angle, max_angle]
range via the gentle compliant primitive (arm101.hardware.gentle.gentle_move),
never exceeding bounds, and aborting cleanly (no exception) the moment any
joint reports contact — leaving joints later in the sweep order untouched.
"""

from __future__ import annotations

import pytest

from arm101.cli._errors import EXIT_USER_ERROR, CliError
from arm101.hardware.bus import FakeBus
from arm101.hardware.demo import _DEFAULT_FRACTION, demo_sweep

# ---------------------------------------------------------------------------
# Local test double — NOT in bus.py (per task scope, kept local to this file).
# ---------------------------------------------------------------------------


class RampLoadBus(FakeBus):
    """FakeBus whose present_load ramps PER MOTOR as that motor is driven.

    Mirrors the RampLoadBus double in tests/test_gentle.py, but tracks the
    load increment per motor-id (rather than one global counter) so a test
    can configure exactly ONE joint to ramp into contact while the others
    sweep cleanly — needed to exercise demo_sweep's "abort on contact, leave
    later joints untouched" behaviour.

    ``load_increment_by_motor`` maps motor-id -> per-write load bump; a motor
    absent from the mapping never ramps (present_load stays 0), so it never
    triggers contact.
    """

    def __init__(self, *args, load_increment_by_motor=None, **kwargs):
        super().__init__(*args, **kwargs)
        self._load_increment_by_motor: dict[int, int] = load_increment_by_motor or {}
        self._load_by_motor: dict[int, int] = {}

    def write_goal_position(self, motor: int, position: int) -> None:
        super().write_goal_position(motor, position)
        increment = self._load_increment_by_motor.get(motor, 0)
        self._load_by_motor[motor] = self._load_by_motor.get(motor, 0) + increment

    def read_info(self, motor: int) -> dict:
        info = super().read_info(motor)
        info["present_load"] = self._load_by_motor.get(motor, 0)
        return info


# ---------------------------------------------------------------------------
# allow_motion gate
# ---------------------------------------------------------------------------


def test_demo_sweep_without_allow_motion_raises_and_writes_nothing():
    bus = FakeBus(positions={1: 2048, 2: 2048, 3: 2048})
    # Deliberately do NOT call bus.open() — if demo_sweep touched the bus at
    # all before the gate check, FakeBus would raise its own "not open"
    # CliError instead, proving the gate fires strictly before any bus call.
    joints = {"shoulder": 1, "elbow": 2, "wrist": 3}

    with pytest.raises(CliError) as exc:
        demo_sweep(bus, joints)

    assert exc.value.code == EXIT_USER_ERROR
    assert "allow_motion" in exc.value.remediation or "flag" in exc.value.message


def test_demo_sweep_allow_motion_false_explicit_raises_and_writes_nothing():
    bus = FakeBus(positions={1: 2048})
    joints = {"shoulder": 1}

    with pytest.raises(CliError) as exc:
        demo_sweep(bus, joints, allow_motion=False)

    assert exc.value.code == EXIT_USER_ERROR
    assert bus.position_writes == []
    assert bus.accel_writes == []
    assert bus.speed_writes == []
    assert bus.torque_writes == []


# ---------------------------------------------------------------------------
# Full sweep, no contact
# ---------------------------------------------------------------------------


def test_demo_sweep_full_sweep_no_contact_sweeps_every_joint_within_bounds():
    info = {
        1: {"min_angle": 0, "max_angle": 4095, "present_position": 2048},
        2: {"min_angle": 1000, "max_angle": 3000, "present_position": 2000},
        3: {"min_angle": 1800, "max_angle": 2200, "present_position": 2000},
    }
    bus = FakeBus(positions={1: 2048, 2: 2000, 3: 2000}, info=info)
    bus.open()
    joints = {"shoulder": 1, "elbow": 2, "wrist": 3}

    report = demo_sweep(bus, joints, allow_motion=True)

    assert report["aborted_on_contact"] is False
    assert report["aborted_joint"] is None
    assert set(report["joints"]) == {"shoulder", "elbow", "wrist"}

    for name, motor in joints.items():
        joint_report = report["joints"][name]
        assert joint_report["contacted"] is False
        assert joint_report["motor"] == motor
        # Both planned targets (low, high) were attempted — no early abort.
        assert len(joint_report["targets_attempted"]) == 2

    # Every commanded position write stayed within its OWN motor's bounds —
    # the key safety property: never exceed [min_angle, max_angle].
    bounds = {motor_id: (reg["min_angle"], reg["max_angle"]) for motor_id, reg in info.items()}
    assert bus.position_writes  # the sweep actually wrote something
    for write in bus.position_writes:
        lo, hi = bounds[write["motor"]]
        assert lo <= write["position"] <= hi


def test_demo_sweep_uses_safe_fraction_strictly_within_full_range():
    """Targets stay within a SAFE sub-range, not the full joint range, on a
    joint whose current position sits centred and far from both limits."""
    info = {1: {"min_angle": 0, "max_angle": 4000, "present_position": 2000}}
    bus = FakeBus(positions={1: 2000}, info=info)
    bus.open()

    report = demo_sweep(bus, {"shoulder": 1}, allow_motion=True, fraction=0.4)

    targets = report["joints"]["shoulder"]["targets_attempted"]
    half_span = 0.4 * (4000 - 0) / 2
    expected_low = round(2000 - half_span)
    expected_high = round(2000 + half_span)
    assert targets == [expected_low, expected_high]
    # Strictly inside the full range — proves it is a SAFE sub-range, not a
    # full-range sweep to the calibrated limits.
    assert 0 < expected_low < expected_high < 4000


# ---------------------------------------------------------------------------
# Contact mid-sweep -> clean abort
# ---------------------------------------------------------------------------


def test_demo_sweep_aborts_cleanly_on_contact_and_skips_later_joints():
    info = {
        1: {"min_angle": 0, "max_angle": 4095, "present_position": 2048},
        2: {"min_angle": 0, "max_angle": 4095, "present_position": 2048},
        3: {"min_angle": 0, "max_angle": 4095, "present_position": 2048},
    }
    bus = RampLoadBus(
        positions={1: 2048, 2: 2048, 3: 2048},
        info=info,
        load_increment_by_motor={2: 40},  # only the elbow (motor 2) ramps
    )
    bus.open()
    joints = {"shoulder": 1, "elbow": 2, "wrist": 3}

    report = demo_sweep(bus, joints, allow_motion=True)

    assert report["aborted_on_contact"] is True
    assert report["aborted_joint"] == "elbow"

    assert report["joints"]["shoulder"]["contacted"] is False
    assert report["joints"]["elbow"]["contacted"] is True
    # The wrist joint (after the contacted elbow in sweep order) is absent
    # from the report entirely OR present but never moved — assert the
    # stronger, more useful property: no bus writes for motor 3 at all.
    assert "wrist" not in report["joints"]

    motors_written = {w["motor"] for w in bus.position_writes}
    assert 1 in motors_written  # shoulder was swept before the abort
    assert 2 in motors_written  # elbow was swept (and contacted)
    assert 3 not in motors_written  # wrist was never touched


def test_demo_sweep_contact_does_not_raise():
    """Contact is an expected, safe outcome — demo_sweep must NOT raise."""
    info = {1: {"min_angle": 0, "max_angle": 4095, "present_position": 2048}}
    bus = RampLoadBus(
        positions={1: 2048},
        info=info,
        load_increment_by_motor={1: 40},
    )
    bus.open()

    report = demo_sweep(bus, {"shoulder": 1}, allow_motion=True)

    assert report["aborted_on_contact"] is True
    assert report["joints"]["shoulder"]["contacted"] is True


# ---------------------------------------------------------------------------
# Bounds safety with a tight range
# ---------------------------------------------------------------------------


def test_demo_sweep_clamps_targets_within_a_tight_joint_range():
    """A joint near its limit with a narrow range -> targets clamp, never
    exceeding [min_angle, max_angle]."""
    info = {1: {"min_angle": 2000, "max_angle": 2100, "present_position": 2090}}
    bus = FakeBus(positions={1: 2090}, info=info)
    bus.open()

    report = demo_sweep(bus, {"wrist": 1}, allow_motion=True, fraction=0.4)

    targets = report["joints"]["wrist"]["targets_attempted"]
    for target in targets:
        assert 2000 <= target <= 2100
    # The "high" target would be 2090+20=2110 unclamped -> must clamp to 2100.
    assert targets[1] == 2100

    for write in bus.position_writes:
        assert 2000 <= write["position"] <= 2100


def test_default_fraction_is_conservative():
    """Document the module's safety contract: the default fraction is a
    meaningful sub-range, not the full span (0 < fraction < 1), and sits in
    the conservative 0.3-0.5 band called out in the design."""
    assert 0.3 <= _DEFAULT_FRACTION <= 0.5
