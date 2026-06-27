"""``arm101-cli find-port`` — enumerate or interactively detect a serial port.

Default mode (no flags): lists all candidate serial ports non-interactively,
one per line.  Agent-safe; always exits 0 even when no ports are found.

``--detect`` mode: interactive disconnect-diff that mirrors ``lerobot-find-port``.
Requires a TTY.  Prompts the operator to unplug the arm, then resolves the
single port that disappeared.  Diagnostics (prompts, progress) go to stderr;
the resolved port goes to stdout.
"""

from __future__ import annotations

import argparse
import sys

from arm101.cli._errors import EXIT_ENV_ERROR, EXIT_USER_ERROR, CliError
from arm101.cli._output import emit_diagnostic, emit_result
from arm101.hardware import ports


def cmd_find_port(args: argparse.Namespace) -> None:
    """Handler for ``find-port``."""
    json_mode = bool(getattr(args, "json", False))
    detect = bool(getattr(args, "detect", False))

    if detect:
        _run_detect(json_mode=json_mode)
    else:
        _run_enumerate(json_mode=json_mode)


def _run_enumerate(*, json_mode: bool) -> None:
    """Default non-interactive enumeration."""
    found = ports.enumerate_ports()

    if json_mode:
        emit_result({"ports": found, "count": len(found)}, json_mode=True)
        return

    if not found:
        emit_result("no candidate serial ports found", json_mode=False)
        return

    emit_result("\n".join(found), json_mode=False)


def _run_detect(*, json_mode: bool) -> None:
    """Interactive disconnect-diff.  Requires a real TTY on stdin."""
    if not sys.stdin.isatty():
        raise CliError(
            code=EXIT_ENV_ERROR,
            message="--detect requires an interactive terminal (TTY) on stdin",
            remediation=(
                "Run without --detect to list ports non-interactively "
                "(agent-safe, always exits 0)."
            ),
        )

    # Step 1: snapshot before
    before = set(ports.enumerate_ports())

    # Step 2: prompt operator — diagnostic goes to stderr, never stdout
    emit_diagnostic("Unplug the arm USB cable, then press Enter...")
    sys.stdin.readline()

    # Step 3: snapshot after
    after = set(ports.enumerate_ports())

    # Step 4: diff
    disappeared = sorted(before - after)
    n = len(disappeared)

    if n != 1:
        raise CliError(
            code=EXIT_USER_ERROR,
            message=(
                f"expected exactly one port to disappear, saw {n} "
                f"({', '.join(disappeared) if disappeared else 'none'})"
            ),
            remediation=(
                "Unplug only the arm's USB cable and try again.  "
                "If multiple ports changed, disconnect other USB devices first."
            ),
        )

    port = disappeared[0]
    if json_mode:
        emit_result({"port": port}, json_mode=True)
    else:
        emit_result(port, json_mode=False)


def register(sub: argparse._SubParsersAction) -> None:
    p = sub.add_parser(
        "find-port",
        help="List candidate serial ports or interactively detect the arm's port.",
    )
    p.add_argument("--json", action="store_true", help="Emit structured JSON.")
    p.add_argument(
        "--detect",
        action="store_true",
        help=(
            "Interactive mode: snapshot ports, prompt to unplug the arm, "
            "then resolve the single disappeared port.  Requires a TTY."
        ),
    )
    p.set_defaults(func=cmd_find_port)
