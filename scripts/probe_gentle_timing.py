#!/usr/bin/env python3
"""Timing probe that catches ``gentle_move()`` returning before the arm arrives.

What this proves
-----------------
:func:`arm101.hardware.gentle.gentle_move` steps a servo's goal position in
small increments and treats ``current`` as the tick it just *commanded* —
never a value it read back off the motor. Its stepping loop exits the instant
the last goal-write has been issued, not when the joint has mechanically
arrived. On a real SO-101 follower that gap is large: a 400-tick
``wrist_roll`` (motor id 5) move was measured returning in ~71ms while the
joint was still sitting at its start position; the real ~900ms of travel,
and the load spike that comes with it, happened entirely after the function
had already stopped watching.

This script reproduces that gap directly, without going through any CLI verb:

1. Read the joint's ``present_position`` before the move (``start_position``).
2. Call ``gentle_move()`` once, timing the call.
3. The INSTANT the call returns, read the joint again
   (``position_immediately_after_call``). On the buggy code this equals
   ``start_position`` (or is very close to it) even though
   ``result["final_position"]`` claims the joint reached the target.
4. Keep polling ``present_position``/``present_load`` for a while longer
   (default 1.5s at 25ms intervals) to show the joint continuing to travel,
   and load rising well past typical free-motion levels, entirely after
   ``gentle_move()`` already returned.

How to read the output
-----------------------
The report prints a table of post-return polling samples and then a
``DIAGNOSIS`` section that states the two damning facts explicitly:

* (a) whether the call returned before the joint had measurably moved
  (``position_immediately_after_call`` close to ``start_position`` rather
  than to the claimed ``final_position``);
* (b) how long real travel continued *after* the call returned, and the peak
  load observed during that unwatched travel.

On the PRE-FIX code (the bug this task documents), expect (a) to be true and
(b) to show several hundred milliseconds of continued travel. On a FIXED
``gentle_move`` that measures arrival, ``position_immediately_after_call``
should already be at (or within tolerance of) the target, and post-return
polling should show little to no further travel.

Usage
-----
    uv run python scripts/probe_gentle_timing.py --port /dev/ttyACM1
    uv run python scripts/probe_gentle_timing.py --json > probe_output.json

This drives real hardware: the named motor moves ``--travel`` ticks
(default 400) from wherever it currently is. Torque is always released
(``enable_torque(motor, False)``) before the script exits, even on error —
run it on an arm that is clear to move and safe to go limp afterwards.
"""

from __future__ import annotations

import argparse
import contextlib
import sys
import time
from typing import TYPE_CHECKING

from arm101.cli._commands.calibrate_motor import _open_bus
from arm101.cli._errors import EXIT_USER_ERROR, CliError
from arm101.cli._output import emit_error, emit_result
from arm101.hardware.bus import load_magnitude
from arm101.hardware.gentle import gentle_move

if TYPE_CHECKING:
    from arm101.hardware.bus import MotorBus

#: Reference bench default — the SO-101 follower in prior hardware sessions.
_DEFAULT_PORT = "/dev/ttyACM1"

#: wrist_roll is motor id 5 on both follower and leader (see
#: arm101.hardware.arm_spec) and is the joint the bug was first measured on.
_DEFAULT_MOTOR = 5

#: Ticks of travel commanded from wherever the joint currently sits. Signed:
#: a negative value drives the joint the other way.
_DEFAULT_TRAVEL = 400

#: How long to keep polling after gentle_move() returns, and how often.
#: 1.5s / 25ms comfortably covers the ~900ms of real travel measured on
#: wrist_roll with room to spare.
_DEFAULT_POLL_DURATION_S = 1.5
_DEFAULT_POLL_INTERVAL_S = 0.025

#: Extra headroom (ticks) added on both sides of [start, target] when
#: deriving min_angle/max_angle for the gentle_move() call, so the move
#: itself is never clamped by this probe's own bounds.
_BOUNDS_MARGIN_TICKS = 200

#: Ticks within which this probe considers the joint to have "arrived" at
#: the target, purely for the diagnosis printed below — independent of
#: whatever tolerance (if any) gentle_move() itself uses internally.
_ARRIVAL_TOLERANCE_TICKS = 20


def _parse_args(argv: "list[str] | None") -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="probe_gentle_timing.py",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--port",
        default=_DEFAULT_PORT,
        help=f"Serial port the arm is connected on (default: {_DEFAULT_PORT}).",
    )
    parser.add_argument(
        "--motor",
        type=int,
        default=_DEFAULT_MOTOR,
        help=f"Motor id to drive (default: {_DEFAULT_MOTOR}, wrist_roll).",
    )
    parser.add_argument(
        "--travel",
        type=int,
        default=_DEFAULT_TRAVEL,
        help=(
            "Signed encoder ticks to command from the joint's current "
            f"position (default: {_DEFAULT_TRAVEL})."
        ),
    )
    parser.add_argument(
        "--poll-duration",
        type=float,
        default=_DEFAULT_POLL_DURATION_S,
        help=(
            "Seconds to keep polling after gentle_move() returns "
            f"(default: {_DEFAULT_POLL_DURATION_S})."
        ),
    )
    parser.add_argument(
        "--poll-interval",
        type=float,
        default=_DEFAULT_POLL_INTERVAL_S,
        help=f"Seconds between post-return polls (default: {_DEFAULT_POLL_INTERVAL_S}).",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Emit a single JSON object instead of a human-readable report.",
    )
    return parser.parse_args(argv)


def _poll_after_return(
    bus: "MotorBus", motor: int, duration_s: float, interval_s: float
) -> "list[dict[str, object]]":
    """Poll ``present_position``/``present_load`` for *duration_s* seconds.

    Returns one sample per poll, each ``t_ms`` measured from the moment
    polling started (i.e. from just after ``gentle_move()`` returned).
    """
    samples: "list[dict[str, object]]" = []
    t_start = time.perf_counter()
    while True:
        elapsed = time.perf_counter() - t_start
        if elapsed > duration_s:
            break
        info = bus.read_info(motor)
        samples.append(
            {
                "t_ms": round(elapsed * 1000, 1),
                "present_position": info["present_position"],
                "present_load": load_magnitude(info["present_load"]),
            }
        )
        time.sleep(interval_s)
    return samples


def _diagnose(
    *,
    start_position: int,
    target: int,
    position_immediately_after_call: int,
    samples: "list[dict[str, object]]",
) -> "dict[str, object]":
    """Derive the two damning facts from the raw measurements.

    Returns a dict with ``returned_before_arrival`` (bool),
    ``real_travel_ms`` (float, or ``None`` if the joint never got within
    :data:`_ARRIVAL_TOLERANCE_TICKS` of the target during polling), and
    ``peak_load``/``peak_load_t_ms`` for the highest load observed while
    polling.
    """
    returned_before_arrival = (
        abs(position_immediately_after_call - target) > _ARRIVAL_TOLERANCE_TICKS
    )

    real_travel_ms: "float | None" = None
    for sample in samples:
        if abs(int(sample["present_position"]) - target) <= _ARRIVAL_TOLERANCE_TICKS:
            real_travel_ms = float(sample["t_ms"])
            break

    peak_load = 0
    peak_load_t_ms = 0.0
    for sample in samples:
        load = int(sample["present_load"])
        if load > peak_load:
            peak_load = load
            peak_load_t_ms = float(sample["t_ms"])

    return {
        "returned_before_arrival": returned_before_arrival,
        "arrival_tolerance_ticks": _ARRIVAL_TOLERANCE_TICKS,
        "real_travel_ms": real_travel_ms,
        "peak_load": peak_load,
        "peak_load_t_ms": peak_load_t_ms,
    }


def run_probe(
    port: str,
    motor: int,
    travel: int,
    poll_duration_s: float,
    poll_interval_s: float,
) -> "dict[str, object]":
    """Drive one ``gentle_move`` and return the full measurement report as a dict."""
    bus = _open_bus(port)
    try:
        start_info = bus.read_info(motor)
        start_position = start_info["present_position"]
        target = start_position + travel
        low = min(start_position, target) - _BOUNDS_MARGIN_TICKS
        high = max(start_position, target) + _BOUNDS_MARGIN_TICKS

        t0 = time.perf_counter()
        result = gentle_move(
            bus,
            motor=motor,
            target=target,
            min_angle=low,
            max_angle=high,
            allow_motion=True,
        )
        call_elapsed_ms = round((time.perf_counter() - t0) * 1000, 1)

        after_call = bus.read_info(motor)
        position_immediately_after_call = after_call["present_position"]
        load_immediately_after_call = load_magnitude(after_call["present_load"])

        samples = _poll_after_return(bus, motor, poll_duration_s, poll_interval_s)
    finally:
        # Best-effort: always release torque, even if the move or polling
        # raised. A CliError here (comms already broken) must not mask
        # whatever the probe body already found.
        with contextlib.suppress(CliError):
            bus.enable_torque(motor, False)
        with contextlib.suppress(CliError):
            bus.close()

    diagnosis = _diagnose(
        start_position=start_position,
        target=target,
        position_immediately_after_call=position_immediately_after_call,
        samples=samples,
    )

    return {
        "port": port,
        "motor": motor,
        "travel": travel,
        "start_position": start_position,
        "requested_target": target,
        "call_elapsed_ms": call_elapsed_ms,
        "gentle_move_result": result,
        "position_immediately_after_call": position_immediately_after_call,
        "load_immediately_after_call": load_immediately_after_call,
        "poll_samples": samples,
        "diagnosis": diagnosis,
    }


def _format_report(report: "dict[str, object]") -> str:
    samples = report["poll_samples"]
    result = report["gentle_move_result"]
    diagnosis = report["diagnosis"]

    interval_ms = samples[1]["t_ms"] - samples[0]["t_ms"] if len(samples) > 1 else None
    poll_window_ms = samples[-1]["t_ms"] if samples else 0.0

    lines: "list[str]" = []
    lines.append("=== gentle_move timing probe ===")
    lines.append(
        f"port={report['port']} motor={report['motor']} travel={report['travel']:+d} ticks"
    )
    lines.append("")
    lines.append(f"start_position                   = {report['start_position']}")
    lines.append(f"requested_target                 = {report['requested_target']}")
    lines.append(f"call elapsed                     = {report['call_elapsed_ms']} ms")
    lines.append(f"result.final_position (claimed)  = {result['final_position']}")
    lines.append(f"position immediately after call  = {report['position_immediately_after_call']}")
    lines.append(f"load immediately after call      = {report['load_immediately_after_call']}")
    lines.append("")

    interval_label = f"~{interval_ms}" if interval_ms is not None else "?"
    lines.append(
        f"--- polling every {interval_label}ms for up to {poll_window_ms}ms "
        "after the call returned ---"
    )
    lines.append(f"{'t_ms':>8}  {'present_position':>16}  {'present_load':>12}")
    for sample in samples:
        lines.append(
            f"{sample['t_ms']:>8}  {sample['present_position']:>16}  {sample['present_load']:>12}"
        )
    lines.append("")

    lines.append("=== DIAGNOSIS ===")
    if diagnosis["returned_before_arrival"]:
        lines.append(
            f"(a) gentle_move() returned in {report['call_elapsed_ms']}ms — BEFORE the joint "
            "had measurably moved (position immediately after call is still close to "
            "start_position, not the claimed final_position)."
        )
    else:
        lines.append(
            f"(a) gentle_move() returned in {report['call_elapsed_ms']}ms and the joint was "
            "ALREADY at (or within tolerance of) the target when the call returned — "
            "no gap observed."
        )

    if diagnosis["real_travel_ms"] is not None:
        lines.append(
            f"(b) real travel continued for ~{diagnosis['real_travel_ms']}ms AFTER the call "
            f"returned, peaking at load={diagnosis['peak_load']} "
            f"(t={diagnosis['peak_load_t_ms']}ms into polling) — entirely unobserved by the "
            "function that claims to watch for contact."
        )
    else:
        lines.append(
            "(b) the joint had NOT reached the target (within "
            f"{diagnosis['arrival_tolerance_ticks']} ticks) by the end of the "
            f"{poll_window_ms}ms polling window; peak load observed while polling was "
            f"{diagnosis['peak_load']} (t={diagnosis['peak_load_t_ms']}ms into polling). "
            "Consider a longer --poll-duration."
        )

    return "\n".join(lines)


def main(argv: "list[str] | None" = None) -> int:
    args = _parse_args(argv)

    try:
        report = run_probe(
            port=args.port,
            motor=args.motor,
            travel=args.travel,
            poll_duration_s=args.poll_duration,
            poll_interval_s=args.poll_interval,
        )
    except CliError as err:
        emit_error(err, json_mode=args.json)
        return err.code
    except Exception as err:  # noqa: BLE001 - last-resort; never leak a traceback
        wrapped = CliError(
            code=EXIT_USER_ERROR,
            message=f"unexpected: {err.__class__.__name__}: {err}",
            remediation="re-run with the same flags; if it persists, check the wiring/port.",
        )
        emit_error(wrapped, json_mode=args.json)
        return wrapped.code

    if args.json:
        emit_result(report, json_mode=True)
    else:
        emit_result(_format_report(report), json_mode=False)
    return 0


if __name__ == "__main__":
    sys.exit(main())
