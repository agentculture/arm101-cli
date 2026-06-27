"""Motor catalog — per-motor identity + spec records for SO-101 assembly.

Before an SO-101 (and its leader) is assembled, each Feetech servo is connected
to the computer **one at a time** and identified: which physical motor it is
(its label, e.g. ``F1``..``F6`` for the follower, ``L1``..``L6`` for the
leader), and its hardware spec — **servo model**, **gear ratio**, and the
**joint** it drives.  The ``arm101 calibrate-motor`` verb captures these and
persists them here so the inventory survives across sessions.

Each :class:`MotorEntry` records the three human-supplied spec fields alongside
the hardware facts read back from the servo at registration time (its EEPROM
id, model number, firmware, and the serial port used) — a read-only snapshot,
never a motor write.

Persistence
-----------
The catalog is a single JSON object keyed by label, written to::

    $XDG_CONFIG_HOME/arm101/motors.json

falling back to ``~/.config/arm101/motors.json`` when ``XDG_CONFIG_HOME`` is
unset.  Re-registering a label overwrites that entry (idempotent upsert); other
entries are preserved.
"""

from __future__ import annotations

import datetime as _dt
import json
import os
import pathlib
from dataclasses import asdict, dataclass

from arm101.cli._errors import EXIT_ENV_ERROR, CliError

# ---------------------------------------------------------------------------
# Data model
# ---------------------------------------------------------------------------


@dataclass
class MotorEntry:
    """One catalogued motor: human spec fields + hardware facts read at register."""

    label: str  # logical motor label, e.g. "F1", "L2"
    servo_model: str  # e.g. "ST-3215-C044(7.4V)"
    gear_ratio: str  # e.g. "1:191"
    joint: str  # corresponding joint, e.g. "shoulder_pan" / "L1"
    detected_id: int  # servo EEPROM ID read at registration
    detected_model: int  # servo model number read at registration (e.g. 777)
    port: str  # serial port used (e.g. "/dev/ttyACM1")
    recorded: str = ""  # ISO-8601 date stamped at save time


# ---------------------------------------------------------------------------
# Path helpers
# ---------------------------------------------------------------------------


def catalog_path() -> pathlib.Path:
    """Return the catalog file path, honouring ``XDG_CONFIG_HOME``."""
    xdg = os.environ.get("XDG_CONFIG_HOME")
    base = pathlib.Path(xdg) if xdg else pathlib.Path.home() / ".config"
    return base / "arm101" / "motors.json"


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def load_catalog() -> dict[str, MotorEntry]:
    """Load the full catalog as a ``label -> MotorEntry`` mapping (empty if none)."""
    path = catalog_path()
    if not path.exists():
        return {}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise CliError(
            code=EXIT_ENV_ERROR,
            message=f"Failed to read motor catalog: {exc}",
            remediation=f"Check that {path} is valid JSON and readable, or delete it.",
        ) from exc
    if not isinstance(data, dict):
        raise CliError(
            code=EXIT_ENV_ERROR,
            message=f"Motor catalog root must be a JSON object, got {type(data).__name__}.",
            remediation=f"Delete {path} and re-register the motors.",
        )
    out: dict[str, MotorEntry] = {}
    for label, entry in data.items():
        try:
            out[label] = MotorEntry(**entry)
        except TypeError as exc:
            raise CliError(
                code=EXIT_ENV_ERROR,
                message=f"Motor catalog entry {label!r} has unexpected format: {exc}",
                remediation=f"Delete {path} and re-register the motors.",
            ) from exc
    return out


def save_entry(entry: MotorEntry) -> MotorEntry:
    """Upsert *entry* into the catalog (by label) and return the saved record.

    Stamps ``recorded`` with today's date when the caller left it blank.
    """
    if not entry.recorded:
        entry.recorded = _dt.date.today().isoformat()
    catalog = load_catalog()
    catalog[entry.label] = entry
    path = catalog_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {label: asdict(e) for label, e in sorted(catalog.items())}
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return entry
