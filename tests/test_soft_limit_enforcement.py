"""The soft limit BINDS: every motion path resolves its bounds through arm_spec.

Task t9 gave ``wrist_roll`` a :class:`~arm101.hardware.arm_spec.SoftLimit` — a
permitted range whose dead arc contains the encoder seam. That made the *table*
correct. It did not make the *arm* safe: nothing read the table. Every move in
the codebase sourced its bounds from the servo's EEPROM
``min_angle``/``max_angle`` registers, which are the factory ``0-4095`` on every
joint of this arm (``docs/hardware-validation-arm-explore.md``), so ``arm flex
wrist_roll --to 4090 --apply`` drove the joint straight into the dead arc and
across the seam — exactly the failure the soft limit exists to prevent. A soft
limit nobody reads is inert data.

(The table's *frame* was a second bug, fixed later and covered in
``tests/test_tick_frames.py``: it shipped in REPORTED ticks and now holds RAW,
which is why :func:`~arm101.hardware.arm_spec.resolve_bounds` takes the servo's
live offset. Every ``FakeBus`` in THIS file holds offset 0, where the two frames
coincide, so the numbers below are unaffected by that fix — which is precisely
why it needed its own suite to catch it.)

These tests pin the fix from BOTH ends:

* the pure resolver (:func:`arm101.hardware.arm_spec.resolve_bounds`) computes
  the INTERSECTION of the EEPROM range with the soft limit — never a
  replacement, so a servo whose EEPROM range is genuinely tighter still wins;
* every call site that turns ``read_info`` into move bounds — ``arm flex``
  (both the compliant and the gentle path), ``arm explore``'s grid, and the
  ``demo_sweep`` choreography — actually goes through it, asserted on the goal
  positions REALLY written to the bus, not on an intermediate return value.

And the spec boundary: the soft limit is SOFTWARE-only. No path here may ever
write it into the servo's EEPROM angle-limit registers (addresses 9 and 11) —
measured/derived ranges are pose- and environment-dependent and are never burnt
into hardware.
"""

from __future__ import annotations

import argparse
import json
import sys

import pytest

from arm101.cli._commands import arm as arm_cmd
from arm101.cli._errors import EXIT_ENV_ERROR, CliError
from arm101.hardware import arm_spec
from arm101.hardware.arm_spec import SOFT_LIMITS, TICK_MAX, TICK_MIN, resolve_bounds
from arm101.hardware.bus import FakeBus
from arm101.hardware.demo import demo_sweep

#: The one joint with a soft limit, and its permitted range — read from the
#: table rather than hard-coded, so a deliberate retune of the width retunes
#: these tests with it, while the *properties* they assert stay pinned.
#:
#: These are **RAW** ticks (:class:`~arm101.hardware.arm_spec.SoftLimit` — a claim
#: about physical angles, immune to a re-zero). Every ``FakeBus`` below holds the
#: default offset 0, where the raw and reported frames coincide, so the goals
#: written to it are these same numbers. The tests that give a servo a NON-zero
#: offset — the state every real SO-101 ships in, and the one where a frame error
#: stops being invisible — live in ``tests/test_tick_frames.py``.
_WRIST_ROLL = "wrist_roll"
_SOFT_MIN = SOFT_LIMITS[_WRIST_ROLL].min_tick
_SOFT_MAX = SOFT_LIMITS[_WRIST_ROLL].max_tick

#: The offset at which reported ticks ARE raw ticks. Named, rather than a bare 0
#: at each call, because "resolve_bounds needs an offset" is the whole point of
#: the frame fix and a literal 0 reads like it does not matter.
_NO_OFFSET = 0

#: A joint with NO soft limit — the regression guard that the fix did not
#: quietly narrow every joint's travel.
_FREE_JOINT = "shoulder_pan"

#: STS3215 EEPROM addresses of the angle-limit registers (bus.py ``_INFO``:
#: ``"min_angle": (9, 2)``, ``"max_angle": (11, 2)``). Nothing may write these.
_EEPROM_ANGLE_LIMIT_ADDRS = (9, 10, 11, 12)


class _FakeStdin:
    """Scripted stdin: non-TTY, so consent resolves to the agent ``--apply`` mode."""

    def __init__(self, tty: bool = False) -> None:
        self._tty = tty

    def isatty(self) -> bool:
        return self._tty

    def readline(self) -> str:
        return ""


def _patch_bus(monkeypatch, fake: FakeBus, port: str = "/dev/ttyACM_fake") -> None:
    fake.open()
    monkeypatch.setattr(arm_cmd, "_candidate_ports", lambda: [port])
    monkeypatch.setattr(arm_cmd, "_open_bus", lambda _p: fake)
    monkeypatch.setattr(sys, "stdin", _FakeStdin())


def _flex_args(
    joint=None,
    to=None,
    demo=False,
    gentle=False,
    threshold=None,
    role="follower",
    port=None,
    apply=True,
    json_mode=False,
):
    return argparse.Namespace(
        joint=joint,
        to=to,
        demo=demo,
        gentle=gentle,
        threshold=threshold,
        role=role,
        port=port,
        apply=apply,
        json=json_mode,
    )


def _goals(fake: FakeBus, motor: int) -> "list[int]":
    """Every goal position ACTUALLY written to *motor* — the ground truth."""
    return [w["position"] for w in fake.position_writes if w["motor"] == motor]


# ===========================================================================
# The pure resolver: intersection, not replacement
# ===========================================================================


def test_resolve_bounds_applies_the_soft_limit_to_a_wrapping_joint() -> None:
    """Factory EEPROM 0-4095 + wrist_roll's soft limit -> the soft limit.

    This is the case that exists on the real arm: read_info returns the
    untouched factory range for every joint, so before this resolver the soft
    limit had no way to reach the motion path at all.
    """
    assert resolve_bounds(_WRIST_ROLL, TICK_MIN, TICK_MAX, _NO_OFFSET) == (_SOFT_MIN, _SOFT_MAX)


def test_resolve_bounds_renders_the_raw_soft_limit_in_the_servos_own_frame() -> None:
    """The frame crossing, in the resolver: a non-zero offset shifts the bounds.

    The soft limit is stored RAW; the bounds this returns are REPORTED, because
    they are compared against ``read_position`` and written as goals. On a servo
    holding the factory offset the two differ by exactly that offset — which is
    the 85-tick error the old table shipped, and the reason this argument exists.
    """
    factory = arm_spec.FACTORY_ENCODER_OFFSET

    assert resolve_bounds(_WRIST_ROLL, TICK_MIN, TICK_MAX, factory) == (
        _SOFT_MIN - factory,
        _SOFT_MAX - factory,
    )


def test_resolve_bounds_leaves_a_joint_without_a_soft_limit_verbatim() -> None:
    """A joint absent from SOFT_LIMITS gets its EEPROM bounds back unchanged.

    The regression guard for the whole change: it would be very easy to
    "helpfully" clamp every joint into wrist_roll's range and silently steal
    travel from five joints that never had a wrap problem. Note the offset is
    ignored for such a joint — with no RAW table entry there is nothing to
    convert, and the EEPROM bounds are already in the frame the caller wants.
    """
    assert resolve_bounds(_FREE_JOINT, TICK_MIN, TICK_MAX, _NO_OFFSET) == (TICK_MIN, TICK_MAX)
    assert resolve_bounds(_FREE_JOINT, 700, 3300, _NO_OFFSET) == (700, 3300)
    assert resolve_bounds(_FREE_JOINT, 700, 3300, arm_spec.FACTORY_ENCODER_OFFSET) == (700, 3300)


def test_resolve_bounds_takes_the_tighter_end_of_each_bound() -> None:
    """INTERSECTION, not replacement — a narrower EEPROM bound still wins.

    The soft limit says "never outside the permitted range". It does NOT say
    "always permit it": if an operator has genuinely narrowed a servo's EEPROM
    angle limits (a calibration, a fixture, a cable-routing constraint), those
    are a real physical constraint and replacing them with the wider soft limit
    would drive the joint somewhere the servo was explicitly configured not to
    go. So each end independently takes the tighter of the two.
    """
    # Both ends tighter in EEPROM -> EEPROM survives untouched.
    assert resolve_bounds(_WRIST_ROLL, 500, 3000, _NO_OFFSET) == (500, 3000)
    # Low end tighter in the soft limit, high end tighter in EEPROM -> one of each.
    assert resolve_bounds(_WRIST_ROLL, 50, 3000, _NO_OFFSET) == (_SOFT_MIN, 3000)
    # High end tighter in the soft limit, low end tighter in EEPROM -> the mirror.
    assert resolve_bounds(_WRIST_ROLL, 500, 4090, _NO_OFFSET) == (500, _SOFT_MAX)


def test_resolve_bounds_is_idempotent() -> None:
    """Resolving an already-resolved range changes nothing.

    Cheap property, but it is what lets a call site route through the resolver
    without having to prove it is the only one on the path. Note it holds only
    when the output is fed back at the SAME offset: the pair that comes out is
    reported, and re-resolving it against a different offset would be comparing
    two different frames — which is a bug, not an idempotence failure.
    """
    once = resolve_bounds(_WRIST_ROLL, TICK_MIN, TICK_MAX, _NO_OFFSET)
    assert resolve_bounds(_WRIST_ROLL, *once, _NO_OFFSET) == once


def test_resolve_bounds_rejects_an_unknown_joint() -> None:
    with pytest.raises(ValueError, match="Unknown joint"):
        resolve_bounds("elbow_twist", TICK_MIN, TICK_MAX, _NO_OFFSET)


def test_resolve_bounds_rejects_an_empty_intersection() -> None:
    """An EEPROM range lying entirely inside the dead arc is a contradiction.

    ``(0, 50)`` for wrist_roll says "only ever go where the soft limit says
    never go". There is no bound this function could return that honours both,
    and silently returning the inverted pair would surface downstream as
    clamp_goal's misleading "min/max were swapped" error. Fail loudly, here.
    """
    with pytest.raises(ValueError, match="no permitted travel"):
        resolve_bounds(_WRIST_ROLL, 0, 50, _NO_OFFSET)


# ===========================================================================
# arm flex — the goal actually written to the bus is soft-limited
# ===========================================================================


def test_flex_clamps_wrist_roll_to_the_soft_max_not_into_the_dead_arc(monkeypatch, capsys) -> None:
    """``arm flex wrist_roll --to 4090 --apply`` writes 3995, not 4090.

    4090 sits inside the dead arc, 5 ticks shy of the seam. The EEPROM says
    4095 is fine; the soft limit says otherwise, and the soft limit is what
    keeps this joint's travel linear.
    """
    fake = FakeBus(positions={5: 2048})
    _patch_bus(monkeypatch, fake)

    arm_cmd.cmd_arm_flex(_flex_args(joint=_WRIST_ROLL, to=4090, json_mode=True))

    assert _goals(fake, 5) == [_SOFT_MAX]
    payload = json.loads(capsys.readouterr().out)
    assert payload["move"]["clamped_target"] == _SOFT_MAX
    assert payload["move"]["was_clamped"] is True


def test_flex_clamps_wrist_roll_to_the_soft_min_at_the_low_end(monkeypatch) -> None:
    """The mirror case: ``--to 5`` is in the dead arc on the other side of the seam."""
    fake = FakeBus(positions={5: 2048})
    _patch_bus(monkeypatch, fake)

    arm_cmd.cmd_arm_flex(_flex_args(joint=_WRIST_ROLL, to=5))

    assert _goals(fake, 5) == [_SOFT_MIN]


def test_flex_gentle_path_is_soft_limited_too(monkeypatch) -> None:
    """The gentle path resolves bounds at the same call site — every write stays inside.

    ``gentle_move`` steps toward the target and may back off on contact; both
    the stepped goals and any retreat write are clamped against the bounds it
    was handed, so soft-limiting that ONE pair covers the whole move.
    """
    fake = FakeBus(positions={5: 3900})
    _patch_bus(monkeypatch, fake)

    arm_cmd.cmd_arm_flex(_flex_args(joint=_WRIST_ROLL, to=4090, gentle=True))

    written = _goals(fake, 5)
    assert written, "the gentle move wrote no goal at all"
    assert max(written) <= _SOFT_MAX
    assert min(written) >= _SOFT_MIN


def test_flex_does_not_narrow_a_joint_without_a_soft_limit(monkeypatch) -> None:
    """shoulder_pan still reaches 4090 — the EEPROM bounds are used verbatim.

    The regression guard, end-to-end: proves the fix did not quietly apply
    wrist_roll's dead arc to joints that have no wrap problem.
    """
    fake = FakeBus(positions={1: 2048})
    _patch_bus(monkeypatch, fake)

    arm_cmd.cmd_arm_flex(_flex_args(joint=_FREE_JOINT, to=4090))

    assert _goals(fake, 1) == [4090]


def test_flex_honours_a_tighter_eeprom_bound_over_the_soft_limit(monkeypatch) -> None:
    """Intersection at the CLI layer: a narrower EEPROM max wins over the soft max."""
    fake = FakeBus(positions={5: 2048}, info={5: {"min_angle": 0, "max_angle": 3000}})
    _patch_bus(monkeypatch, fake)

    arm_cmd.cmd_arm_flex(_flex_args(joint=_WRIST_ROLL, to=4090))

    assert _goals(fake, 5) == [3000]


def test_flex_reports_an_impossible_bound_as_a_cli_error(monkeypatch) -> None:
    """A servo configured entirely inside the dead arc fails with a CliError, not a traceback."""
    fake = FakeBus(positions={5: 20}, info={5: {"min_angle": 0, "max_angle": 50}})
    _patch_bus(monkeypatch, fake)

    with pytest.raises(CliError) as exc:
        arm_cmd.cmd_arm_flex(_flex_args(joint=_WRIST_ROLL, to=30))

    assert exc.value.code == EXIT_ENV_ERROR
    assert _goals(fake, 5) == [], "no goal may be written when the bounds are impossible"


# ===========================================================================
# arm explore — the grid's bounds are the soft-limited range
# ===========================================================================


def test_grid_spec_bounds_are_soft_limited_for_wrist_roll(monkeypatch) -> None:
    """``_build_grid_spec`` seeds wrist_roll's bounds from the resolver, not raw EEPROM.

    The explore engine takes EVERY move bound it ever uses from
    ``GridSpec.bounds`` (flood-fill neighbours and multi-joint escape probes
    both read ``spec.bounds[joint]``), so soft-limiting the grid is what stops
    exploration from walking a cell into the dead arc.
    """
    fake = FakeBus(positions={i: 2048 for i in range(1, 7)})
    fake.open()

    spec = arm_cmd._build_grid_spec(fake, "follower", 512)

    wrist_roll_index = arm_spec.JOINTS.index(_WRIST_ROLL)
    assert spec.bounds[wrist_roll_index] == (_SOFT_MIN, _SOFT_MAX)
    for index, joint in enumerate(arm_spec.JOINTS):
        if joint != _WRIST_ROLL:
            assert spec.bounds[index] == (TICK_MIN, TICK_MAX), f"{joint} was narrowed"


def test_grid_spec_origin_is_pulled_out_of_the_dead_arc(monkeypatch) -> None:
    """A wrist_roll resting AT the seam yields a grid origin inside the permitted range.

    The t9 hardware run found wrist_roll parked at raw tick 4 — sitting on the
    seam. The origin is clamped into the joint's bounds, and now those bounds
    are the soft-limited ones, so the flood-fill starts from a cell it is
    allowed to be in rather than from inside its own dead arc.
    """
    fake = FakeBus(positions={i: 2048 for i in range(1, 7)} | {5: 4})
    fake.open()

    spec = arm_cmd._build_grid_spec(fake, "follower", 512)

    assert spec.origin[arm_spec.JOINTS.index(_WRIST_ROLL)] == _SOFT_MIN


# ===========================================================================
# demo_sweep — the scripted choreography is soft-limited too
# ===========================================================================


def test_demo_sweep_never_targets_the_dead_arc(monkeypatch) -> None:
    """A demo sweep of wrist_roll from near the seam stays inside the permitted range.

    ``demo_sweep`` centres a sub-range on the joint's CURRENT position and
    clamps it against the joint's bounds. Starting at tick 4 with the factory
    EEPROM range, that low target clamped to 0 — straight into the dead arc,
    on the far side of the seam. With the resolver it clamps to 100 instead.
    """
    fake = FakeBus(positions={5: 4})
    fake.open()

    report = demo_sweep(fake, {_WRIST_ROLL: 5}, allow_motion=True)

    joint_report = report["joints"][_WRIST_ROLL]
    assert (joint_report["min_angle"], joint_report["max_angle"]) == (_SOFT_MIN, _SOFT_MAX)
    for goal in _goals(fake, 5):
        assert _SOFT_MIN <= goal <= _SOFT_MAX, f"demo sweep wrote {goal} into the dead arc"


def test_demo_sweep_leaves_an_unlimited_joint_on_its_full_range(monkeypatch) -> None:
    """shoulder_pan's demo sub-range is still computed from the full EEPROM span."""
    fake = FakeBus(positions={1: 2048})
    fake.open()

    report = demo_sweep(fake, {_FREE_JOINT: 1}, allow_motion=True)

    joint_report = report["joints"][_FREE_JOINT]
    assert (joint_report["min_angle"], joint_report["max_angle"]) == (TICK_MIN, TICK_MAX)


# ===========================================================================
# Spec boundary: the soft limit is SOFTWARE-only, never burnt into EEPROM
# ===========================================================================


def test_no_motion_path_writes_the_soft_limit_into_the_servo(monkeypatch) -> None:
    """Not one register write lands on the EEPROM angle-limit addresses (9/11).

    The explicit spec boundary of the whole soft-limit line of work: measured
    and derived ranges are pose- and environment-dependent, so they live in
    ``arm_spec`` and in the reachability map — NEVER burnt into a servo's
    EEPROM, where they would silently outlive the pose that produced them and
    could not be undone by reinstalling the software.
    """
    fake = FakeBus(positions={5: 2048})
    _patch_bus(monkeypatch, fake)

    arm_cmd.cmd_arm_flex(_flex_args(joint=_WRIST_ROLL, to=4090, gentle=True))

    touched = {w["addr"] for w in fake.register_writes}
    assert not touched & set(_EEPROM_ANGLE_LIMIT_ADDRS), (
        "a motion path wrote the servo's EEPROM angle-limit registers: "
        f"{sorted(touched & set(_EEPROM_ANGLE_LIMIT_ADDRS))}"
    )
    assert fake.eeprom_writes == []
