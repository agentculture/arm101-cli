"""Regression tests for overload on the PRE-LOOP recovery paths (Qodo review, PR #24).

`gentle_move`/`demo_sweep` must report an overload (never raise) even when it
strikes on an operation that runs BEFORE the step loop:

- Bug 2: `gentle_move` reads/writes Torque_Limit to cap it before stepping. An
  overload on that cap read/write must be caught and reported (overloaded=True),
  not propagated.
- Bug 3: `demo_sweep` reads each joint's registers up front (min/max/position).
  An overload on that pre-read must abort the sweep cleanly (reported), not raise.

Both use the FakeBus overload seam armed to fire on op 1 — the very first bus op,
which is the pre-loop call in each case.
"""

from __future__ import annotations

from arm101.hardware.bus import FakeBus
from arm101.hardware.demo import demo_sweep
from arm101.hardware.gentle import gentle_move


def test_gentle_move_overload_during_torque_cap_setup_is_reported() -> None:
    # Op 1 in gentle_move is now read_torque_limit (the cap setup, inside the
    # try). Arm the seam there: gentle_move must return overloaded=True, not raise.
    bus = FakeBus(positions={1: 2048}).fail_with_overload_on_op(1)
    bus.open()

    result = gentle_move(bus, 1, 2200, min_angle=0, max_angle=4095, allow_motion=True)

    assert result["overloaded"] is True
    assert result["contacted"] is False


def test_gentle_move_overload_during_cap_does_not_crash_on_restore() -> None:
    # When the cap READ fails, original_torque_limit is never captured — the
    # finally-restore must be guarded and skip it (no crash, still overloaded).
    bus = FakeBus(positions={1: 2048}).fail_with_overload_on_op(1)
    bus.open()

    result = gentle_move(bus, 1, 2200, min_angle=0, max_angle=4095, allow_motion=True)

    assert result["overloaded"] is True  # returned normally, no exception


def test_demo_sweep_overload_on_initial_read_aborts_cleanly() -> None:
    # Op 1 in demo_sweep is the upfront read_info(motor) for the first joint.
    # Arm the seam there: the sweep must report the overload and stop, not raise.
    bus = FakeBus(positions={1: 2048, 2: 2048}).fail_with_overload_on_op(1)
    bus.open()

    report = demo_sweep(bus, {"shoulder_pan": 1, "shoulder_lift": 2}, allow_motion=True)

    assert report["aborted_on_overload"] is True
    assert report["overloaded_joint"] == "shoulder_pan"
    # first joint recorded as overloaded; the second joint is never visited
    assert report["joints"]["shoulder_pan"]["overloaded"] is True
    assert "shoulder_lift" not in report["joints"]
