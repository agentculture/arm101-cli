"""Isolated hardware layer for the SO-101 arm.

Everything that talks to physical hardware — serial-port enumeration and
Feetech STS3215 motor I/O — lives under this package so the introspection CLI
keeps importing with **zero third-party runtime dependencies**.

Rules for this package:

* The top-level CLI (``arm101.cli``) must never import a submodule here at
  module load time. Hardware verbs import what they need *inside their handler*
  (lazy import) so ``python -c "import arm101.cli"`` works in an environment with
  no third-party packages installed.
* Third-party hardware libraries (e.g. the Feetech SDK) are declared as the
  optional ``[hardware]`` install extra and lazy-imported only when a bus is
  actually opened. Absence of the SDK surfaces as a ``CliError`` (exit 2,
  environment error), never an ``ImportError`` traceback.
* Serial-port enumeration is stdlib-only and Linux-first; unsupported platforms
  raise a clean ``CliError`` rather than silently finding nothing.
"""

from __future__ import annotations
