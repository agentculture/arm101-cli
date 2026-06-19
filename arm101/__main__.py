"""Entry point for ``python -m arm101``."""

from __future__ import annotations

import sys

from arm101.cli import main

if __name__ == "__main__":
    sys.exit(main())
