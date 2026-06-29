"""Calibration Profile schema and XDG-based persistence for the SO-101 arm.

Units and clamp contract (safety contract for motion verbs)
-----------------------------------------------------------
Each joint stores calibration values as **raw STS3215 encoder ticks** — 12-bit
unsigned integers in the range **0–4095**.  Three values are recorded per joint:

``min``
    Lower travel limit (minimum encoder tick the joint may reach).
``mid``
    Centered/rest position (the tick the joint returns to when homed).
``max``
    Upper travel limit (maximum encoder tick the joint may reach).

Invariant: ``0 <= min <= mid <= max <= 4095``

This invariant is the clamp contract that a future motion verb MUST enforce
before issuing any move command.  Loading a profile does NOT re-validate the
invariant (the profile is assumed correct when saved); callers that drive
hardware are responsible for clamping target ticks into ``[min, max]`` before
transmission.

Persistence
-----------
Profiles are serialised to JSON files at::

    $XDG_CONFIG_HOME/arm101/calibrations/<id>.json

If ``XDG_CONFIG_HOME`` is not set the path falls back to::

    ~/.config/arm101/calibrations/<id>.json

The directory is created automatically on first save.  Profile IDs are
user-supplied strings; they must be valid filename components (no path
separators).
"""

from __future__ import annotations

import json
import os
import pathlib
import re
from dataclasses import asdict, dataclass
from typing import Dict

from arm101.cli._errors import EXIT_ENV_ERROR, EXIT_USER_ERROR, CliError

#: Allowed profile-id shape: a filename component only — leading alphanumeric,
#: then alphanumerics / dot / underscore / hyphen. Path separators and ``..``
#: are rejected so a crafted id cannot escape the calibrations directory.
_VALID_PROFILE_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")

# ---------------------------------------------------------------------------
# Public constants
# ---------------------------------------------------------------------------

#: Canonical joint names for the SO-101, in hardware order.
JOINTS: tuple[str, ...] = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


# ---------------------------------------------------------------------------
# Data model
# ---------------------------------------------------------------------------


@dataclass
class JointCalibration:
    """Raw STS3215 encoder tick limits for a single joint.

    All values are integers in ``[0, 4095]`` and must satisfy
    ``min <= mid <= max``.
    """

    min: int
    mid: int
    max: int


@dataclass
class Profile:
    """Calibration profile for the SO-101: per-joint min/mid/max encoder ticks.

    ``joints`` maps each name in :data:`JOINTS` to its :class:`JointCalibration`.
    """

    joints: Dict[str, JointCalibration]


# ---------------------------------------------------------------------------
# Path helpers
# ---------------------------------------------------------------------------


def profile_path(id: str) -> pathlib.Path:
    """Return the filesystem path for calibration profile *id*.

    Honours ``XDG_CONFIG_HOME``; falls back to ``~/.config`` when unset.

    The *id* must be a single filename component (validated against
    :data:`_VALID_PROFILE_ID`); path separators, ``..``, and empty strings are
    rejected so a crafted id cannot read or overwrite files outside the
    calibrations directory. Both :func:`save` and :func:`load` route through
    here, so every caller is protected.

    Raises
    ------
    CliError(EXIT_USER_ERROR)
        If *id* is not a safe filename component.
    """
    if not _VALID_PROFILE_ID.match(id) or ".." in id:
        raise CliError(
            code=EXIT_USER_ERROR,
            message=f"Invalid profile id {id!r}.",
            remediation=(
                "Use only letters, digits, '.', '_' or '-' (starting with a letter or "
                "digit); path separators and '..' are not allowed."
            ),
        )
    xdg = os.environ.get("XDG_CONFIG_HOME")
    if xdg:
        base = pathlib.Path(xdg)
    else:
        base = pathlib.Path.home() / ".config"
    return base / "arm101" / "calibrations" / f"{id}.json"


# ---------------------------------------------------------------------------
# Serialisation helpers
# ---------------------------------------------------------------------------


def _profile_to_dict(profile: Profile) -> dict:
    return {name: asdict(cal) for name, cal in profile.joints.items()}


def _dict_to_profile(data: dict) -> Profile:
    joints = {}
    for name in JOINTS:
        entry = data[name]
        joints[name] = JointCalibration(
            min=int(entry["min"]),
            mid=int(entry["mid"]),
            max=int(entry["max"]),
        )
    return Profile(joints=joints)


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def save(profile: Profile, id: str) -> None:
    """Persist *profile* to disk under the given *id*.

    Creates parent directories if they do not exist.  Any existing profile
    with the same *id* is overwritten atomically on POSIX systems.
    """
    path = profile_path(id)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(_profile_to_dict(profile), indent=2)
    path.write_text(payload, encoding="utf-8")


def load(id: str) -> Profile:
    """Load and return the calibration profile stored under *id*.

    Raises
    ------
    CliError(EXIT_USER_ERROR)
        If no profile with the given *id* exists.  This is treated as a
        user-input error because the caller supplied the id; the remediation
        message explains how to create one.
    """
    path = profile_path(id)
    if not path.exists():
        raise CliError(
            code=EXIT_USER_ERROR,
            message=f"Calibration profile {id!r} not found at {path}",
            remediation=(
                f"Run `arm101 calibrate {id}` to create the profile, "
                "or list available profiles with `arm101 calibrate --list`."
            ),
        )
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise CliError(
            code=EXIT_ENV_ERROR,
            message=f"Failed to read calibration profile {id!r}: {exc}",
            remediation=(f"Check that {path} is a valid JSON file and is readable."),
        ) from exc
    try:
        return _dict_to_profile(data)
    except (KeyError, ValueError, TypeError) as exc:
        raise CliError(
            code=EXIT_ENV_ERROR,
            message=f"Calibration profile {id!r} has unexpected format: {exc}",
            remediation=(f"Delete {path} and re-run `arm101 calibrate {id}` to rebuild it."),
        ) from exc
