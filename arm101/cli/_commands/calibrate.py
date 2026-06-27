"""``arm101 calibrate`` — interactive per-joint min/mid/max capture.

Walks the operator through three arm poses (centred/rest, minimum, maximum),
reads raw encoder ticks from the motor bus after each pose, and persists a
calibration :class:`~arm101.hardware.profiles.Profile` for the named arm id.

Bus injection seam
------------------
The module-level :func:`_open_bus` factory is monkeypatched in tests to inject
a :class:`~arm101.hardware.bus.FakeBus` without physical hardware::

    monkeypatch.setattr(calibrate, "_open_bus", lambda args: fake_bus)

Do NOT add a ``--fake`` flag to the production parser; testability lives in
this seam, not in a runtime flag.
"""

from __future__ import annotations

import argparse

from arm101.cli._errors import CliError  # noqa: F401 — re-exported for type hints
from arm101.cli._output import emit_diagnostic, emit_result
from arm101.hardware.bus import FeetechBus, MotorBus
from arm101.hardware.profiles import (
    JOINTS,
    JointCalibration,
    Profile,
    profile_path,
    save,
)

# ---------------------------------------------------------------------------
# Joint → motor-id mapping  (SO-101 hardware wiring)
# ---------------------------------------------------------------------------

_JOINT_MOTOR: dict[str, int] = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}

_DEFAULT_PORT = "/dev/ttyACM0"


# ---------------------------------------------------------------------------
# Bus injection seam (monkeypatched in tests)
# ---------------------------------------------------------------------------


def _open_bus(args: argparse.Namespace) -> MotorBus:
    """Open and return a :class:`~arm101.hardware.bus.FeetechBus`.

    Tests monkeypatch this function to inject a
    :class:`~arm101.hardware.bus.FakeBus` without physical hardware.

    Raises
    ------
    CliError(EXIT_ENV_ERROR)
        Propagated from :meth:`FeetechBus.open` when ``scservo_sdk`` is absent
        or the serial port cannot be opened.
    """
    port = getattr(args, "port", None) or _DEFAULT_PORT
    bus = FeetechBus(port)
    bus.open()
    return bus


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------


def _read_all_joints(bus: MotorBus) -> dict[str, int]:
    """Read the current encoder position for every joint in :data:`JOINTS` order."""
    return {joint: bus.read_position(_JOINT_MOTOR[joint]) for joint in JOINTS}


# ---------------------------------------------------------------------------
# Command handler
# ---------------------------------------------------------------------------


def cmd_calibrate(args: argparse.Namespace) -> None:
    """Interactive calibration: centred → minimum → maximum pose capture.

    Operator prompts are written to stderr via :func:`emit_diagnostic`.
    The final summary (table or JSON) is written to stdout via
    :func:`emit_result`.  The stdout/stderr split is maintained in both text
    and ``--json`` modes.

    Raises
    ------
    CliError
        Propagated from :func:`_open_bus` or bus read methods; never swallowed.
    """
    json_mode = bool(getattr(args, "json", False))

    bus = _open_bus(args)
    try:
        # --- Round 1: centred/rest pose → mid ---
        emit_diagnostic("Move arm to centered/rest pose, then press Enter...")
        input()
        mid_pos = _read_all_joints(bus)

        # --- Round 2: minimum/fully-closed → min ---
        emit_diagnostic("Move arm to MINIMUM/fully-closed position, then press Enter...")
        input()
        min_pos = _read_all_joints(bus)

        # --- Round 3: maximum/fully-open → max ---
        emit_diagnostic("Move arm to MAXIMUM/fully-open position, then press Enter...")
        input()
        max_pos = _read_all_joints(bus)
    finally:
        bus.close()

    # Build Profile — sort each (mid, min, max) trio so the invariant
    # ``min <= mid <= max`` holds regardless of which direction the joint moved.
    joints_cal: dict[str, JointCalibration] = {}
    for joint in JOINTS:
        trio = sorted([mid_pos[joint], min_pos[joint], max_pos[joint]])
        joints_cal[joint] = JointCalibration(min=trio[0], mid=trio[1], max=trio[2])

    profile = Profile(joints=joints_cal)
    save(profile, args.id)

    # --- Emit result ---
    path_str = str(profile_path(args.id))

    if json_mode:
        payload: dict = {
            "id": args.id,
            "joints": {
                j: {"min": c.min, "mid": c.mid, "max": c.max} for j, c in joints_cal.items()
            },
            "path": path_str,
        }
        emit_result(payload, json_mode=True)
    else:
        lines = [f"Calibration saved: {path_str}", ""]
        lines.append(f"{'Joint':<16} {'min':>6} {'mid':>6} {'max':>6}")
        lines.append("-" * 38)
        for joint in JOINTS:
            c = joints_cal[joint]
            lines.append(f"{joint:<16} {c.min:>6} {c.mid:>6} {c.max:>6}")
        emit_result("\n".join(lines), json_mode=False)


# ---------------------------------------------------------------------------
# Registration
# ---------------------------------------------------------------------------


def register(sub: "argparse._SubParsersAction") -> None:
    """Register the ``calibrate`` subcommand on *sub*."""
    p = sub.add_parser(
        "calibrate",
        help="Interactively capture per-joint min/mid/max positions and save a profile.",
    )
    p.add_argument(
        "id",
        help=(
            "Arm identifier (mirrors lerobot --robot.id). "
            "Used as the calibration profile filename."
        ),
    )
    p.add_argument(
        "--port",
        default=None,
        help=f"Serial port for the motor bus (default: {_DEFAULT_PORT}).",
    )
    p.add_argument(
        "--json",
        action="store_true",
        help="Emit structured JSON to stdout.",
    )
    p.set_defaults(func=cmd_calibrate)
