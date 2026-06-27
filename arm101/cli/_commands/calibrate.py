"""``arm101 calibrate`` — interactive per-joint min/mid/max capture.

Walks the operator through three arm poses (centered/rest, minimum, maximum),
reads raw encoder ticks from the motor bus after each pose, and persists a
calibration :class:`~arm101.hardware.profiles.Profile` for the named arm id.

Three consent modes
-------------------
1. **interactive** (TTY): walks through three poses (centered/rest → minimum →
   maximum), reads all 6 joints after each via the motor bus, then saves the
   profile to disk.  Prompts go to stderr; the summary goes to stdout.
2. **dry_run** (non-TTY, no ``--apply``): emits a read-only description of what
   *would* happen — the profile id, the 6 joints, the three poses, and the profile
   path.  No bus is opened; zero profile files are written.  Safe to run from an
   agent or a pipe.
3. **agent** (non-TTY + ``--apply``): raises a clean
   ``CliError(EXIT_USER_ERROR)`` because full-arm pose calibration requires
   physical arm poses that cannot be captured headlessly.

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
import sys

from arm101.cli._consent import resolve_consent
from arm101.cli._errors import EXIT_ENV_ERROR, EXIT_USER_ERROR, CliError
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


def _capture_pose(label: str, bus: MotorBus) -> dict[str, int]:
    """Prompt the operator to move the arm to *label*, wait for Enter, then read all joints.

    Raises
    ------
    CliError(EXIT_ENV_ERROR)
        If stdin returns EOF (empty string) before the operator presses Enter.
        No profile is written when this occurs.
    """
    emit_diagnostic(f"Move arm to {label}, then press Enter...")
    line = sys.stdin.readline()
    if line == "":
        raise CliError(
            code=EXIT_ENV_ERROR,
            message=f"stdin closed unexpectedly before capturing {label!r} pose.",
            remediation=(
                "Provide an interactive terminal so all three poses can be "
                "captured (centered/rest, minimum, maximum)."
            ),
        )
    return _read_all_joints(bus)


# ---------------------------------------------------------------------------
# Command handler
# ---------------------------------------------------------------------------


def cmd_calibrate(args: argparse.Namespace) -> None:
    """Interactive calibration: centered → minimum → maximum pose capture.

    Routes through :func:`~arm101.cli._consent.resolve_consent` (1-step tier,
    no plan hash required) and branches on the resolved mode:

    * **interactive** (TTY): walks all three poses; saves profile.
    * **dry_run** (non-TTY, no ``--apply``): prints a read-only preview; no bus,
      no write.
    * **agent** (non-TTY + ``--apply``): raises :class:`CliError`
      (:data:`~arm101.cli._errors.EXIT_USER_ERROR`) — physical pose capture
      cannot be automated headlessly.

    Operator prompts are written to stderr via :func:`emit_diagnostic`.
    The final summary (table or JSON) is written to stdout via
    :func:`emit_result`.  The stdout/stderr split is maintained in both text
    and ``--json`` modes.

    Raises
    ------
    CliError
        Propagated from :func:`_open_bus` or bus read methods; never swallowed.
        Also raised by :func:`_capture_pose` on EOF and by agent-mode rejection.
    """
    json_mode = bool(getattr(args, "json", False))

    mode = resolve_consent(args, verb="calibrate", require_plan_hash=False)

    # --- dry_run: emit read-only preview, zero writes, no bus ---
    if mode == "dry_run":
        path_str = str(profile_path(args.id))
        if json_mode:
            emit_result(
                {
                    "id": args.id,
                    "joints": list(JOINTS),
                    "poses": ["centered/rest", "minimum", "maximum"],
                    "path": path_str,
                    "would_write": False,
                },
                json_mode=True,
            )
        else:
            lines = [
                f"## Dry-run preview: calibrate {args.id}",
                "",
                f"Profile id  : {args.id}",
                f"Profile path: {path_str}",
                "",
                "Joints that would be captured:",
            ]
            for joint in JOINTS:
                lines.append(f"  - {joint}")
            lines += [
                "",
                "Poses (in order):",
                "  1. centered/rest pose",
                "  2. minimum / fully-closed position",
                "  3. maximum / fully-open position",
                "",
                "No bus opened; no profile written.",
                "Run in an interactive terminal (TTY) to perform calibration.",
            ]
            emit_result("\n".join(lines), json_mode=False)
        return

    # --- agent: --apply is not supported for calibrate ---
    if mode == "agent":
        raise CliError(
            code=EXIT_USER_ERROR,
            message=(
                "calibrate: --apply is not supported in non-TTY (agent) mode. "
                "Full-arm pose calibration requires physical arm poses that cannot "
                "be captured headlessly."
            ),
            remediation=(
                "Run 'arm101 calibrate <id>' in an interactive terminal (TTY) to "
                "capture poses, or omit --apply for a read-only dry-run preview."
            ),
        )

    # --- interactive mode: open bus, capture 3 poses (EOF-safe), close in finally ---
    bus = _open_bus(args)
    try:
        mid_pos = _capture_pose("centered/rest pose", bus)
        min_pos = _capture_pose("MINIMUM/fully-closed position", bus)
        max_pos = _capture_pose("MAXIMUM/fully-open position", bus)
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
        "--apply",
        action="store_true",
        default=False,
        help=(
            "Non-TTY agent flag. NOT SUPPORTED for calibrate: physical pose capture "
            "needs a human at a terminal. In non-TTY (piped/agent) mode this flag "
            "exits 1; under a TTY it is ignored and interactive capture proceeds."
        ),
    )
    p.add_argument(
        "--json",
        action="store_true",
        help="Emit structured JSON to stdout.",
    )
    p.set_defaults(func=cmd_calibrate)
