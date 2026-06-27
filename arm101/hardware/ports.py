"""Serial-port enumeration for the SO-101 arm (stdlib-only, Linux-first).

Public API
----------
enumerate_ports(_roots=None) -> list[str]
    Return a sorted list of candidate serial-port paths.

    Parameters
    ----------
    _roots : sequence of str, optional
        Glob patterns to search.  Defaults to the canonical Linux patterns:
        ``/dev/ttyACM*``, ``/dev/ttyUSB*``, ``/dev/serial/by-id/*``.
        Pass a list pointing at a fake ``/dev`` tree in tests — this is the
        sole testability hook; no other changes to the module are needed.

    Raises
    ------
    CliError(EXIT_ENV_ERROR)
        On macOS (``sys.platform == "darwin"``) or Windows
        (``sys.platform.startswith("win")``), because the Linux ``/dev``
        paths do not exist there.  The caller receives a clean error with a
        remediation hint rather than a silently empty list.
"""

from __future__ import annotations

import glob
import sys
from typing import Sequence

from arm101.cli._errors import EXIT_ENV_ERROR, CliError

# Default glob patterns for Linux serial ports.
# Tests may override this list by passing ``_roots`` to ``enumerate_ports()``.
_DEFAULT_ROOTS: list[str] = [
    "/dev/ttyACM*",
    "/dev/ttyUSB*",
    "/dev/serial/by-id/*",
]


def enumerate_ports(
    _roots: Sequence[str] | None = None,
) -> list[str]:
    """Return a sorted list of candidate serial-port paths on the current host.

    On Linux, globs ``/dev/ttyACM*``, ``/dev/ttyUSB*``, and
    ``/dev/serial/by-id/*`` (or the caller-supplied *_roots*) and returns all
    matches sorted lexicographically.  An empty match is a clean ``[]`` — it
    is not an error (no arm connected is a normal state in CI).

    On macOS or Windows, raises :class:`~arm101.cli._errors.CliError` with
    ``code=EXIT_ENV_ERROR`` because the Linux ``/dev`` device-file convention
    does not apply.

    Parameters
    ----------
    _roots:
        Iterable of glob patterns to search.  Supply a list pointing at a
        temp directory in tests so enumeration never touches real hardware.
        Defaults to :data:`_DEFAULT_ROOTS`.

    Returns
    -------
    list[str]
        Sorted list of matching paths (strings, not ``Path`` objects).

    Raises
    ------
    CliError
        When called on macOS or Windows.
    """
    platform = sys.platform

    if platform == "darwin":
        raise CliError(
            code=EXIT_ENV_ERROR,
            message=("serial enumeration unsupported on darwin (macOS) for now"),
            remediation=(
                "Install the [mac] optional extra once it is available, or "
                "enumerate ports manually with `ls /dev/tty.*` in a terminal."
            ),
        )

    if platform.startswith("win"):
        raise CliError(
            code=EXIT_ENV_ERROR,
            message=(f"serial enumeration unsupported on {platform} (Windows) for now"),
            remediation=(
                "Install the [win] optional extra once it is available, or "
                "open Device Manager and check 'Ports (COM & LPT)' manually."
            ),
        )

    patterns = list(_roots) if _roots is not None else _DEFAULT_ROOTS
    matches: list[str] = []
    for pattern in patterns:
        matches.extend(glob.glob(pattern))

    return sorted(matches)
