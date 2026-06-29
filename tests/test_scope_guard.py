"""Scope-guard tests: assert that DEFERRED features are genuinely absent (t7, #17).

Three guards:

B1. Zero runtime dependencies — pyproject.toml [project].dependencies must be [].
    (No new third-party runtime dep was introduced during the arm_spec refactor.)

B2. No XDG arm-profile loader in arm_spec.py — the module defines its values as
    inline Python constants and does NOT read an external config file.
    (The user-editable XDG profile is Layer 3, not yet built.)

B3. Calibration is NOT gear-aware — calibrate.py does not consume gear ratios for
    its range/position math.
    (Gear-corrected calibration math is a follow-up feature, not yet built.)
"""

from __future__ import annotations

import pathlib
import tomllib

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

_REPO_ROOT = pathlib.Path(__file__).parent.parent
_PYPROJECT = _REPO_ROOT / "pyproject.toml"
_ARM_SPEC = _REPO_ROOT / "arm101" / "hardware" / "arm_spec.py"
_CALIBRATE = _REPO_ROOT / "arm101" / "cli" / "_commands" / "calibrate.py"


# ---------------------------------------------------------------------------
# B1 — Zero runtime deps
# ---------------------------------------------------------------------------


def test_no_runtime_dependencies():
    """[project].dependencies in pyproject.toml must be exactly [] (empty list).

    The arm101-cli runtime package intentionally has zero third-party
    dependencies.  This guard fires if anyone adds a runtime dep — the
    correct place for a new dep is [dependency-groups].dev (test/lint tools)
    or [project.optional-dependencies].seeed/mac/win (hardware extras).
    """
    data = tomllib.loads(_PYPROJECT.read_text(encoding="utf-8"))
    deps = data.get("project", {}).get("dependencies", [])
    assert deps == [], (
        f"[project].dependencies in pyproject.toml must be [] (empty), "
        f"but found: {deps!r}.\n"
        "Hardware libraries belong in [project.optional-dependencies].seeed; "
        "test/lint tools belong in [dependency-groups].dev."
    )


# ---------------------------------------------------------------------------
# B2 — No XDG arm-profile loader in arm_spec.py
# ---------------------------------------------------------------------------

# Tokens that would indicate arm_spec.py reads an external config file.
_FILE_IO_PATTERNS: list[str] = [
    "open(",
    "json.load",
    "yaml",
    "tomllib",
    ".config",
    "XDG_CONFIG_HOME",
    "Path.home()",
]


def test_arm_spec_has_no_external_config_reader():
    """arm_spec.py must define its values as inline Python constants only.

    It must NOT open, load, or reference any external config file, XDG path,
    or home-directory path.  The user-editable XDG arm-profile loader is
    deferred (Layer 3) and must not be present yet.
    """
    source = _ARM_SPEC.read_text(encoding="utf-8")
    violations: list[str] = []

    for pattern in _FILE_IO_PATTERNS:
        for lineno, line in enumerate(source.splitlines(), start=1):
            if pattern in line:
                # Allow comments and docstrings that merely *mention* XDG for
                # documentation purposes, but flag actual code.
                stripped = line.lstrip()
                if not stripped.startswith("#") and not stripped.startswith('"""'):
                    violations.append(f"  line {lineno}: {line.rstrip()!r}  (matched {pattern!r})")
                    break  # one hit per pattern is enough

    assert violations == [], (
        "arm_spec.py contains external-config-read patterns that should not be "
        "present until the XDG profile loader (Layer 3) is built:\n" + "\n".join(violations)
    )


# ---------------------------------------------------------------------------
# B3 — Calibrate is not gear-aware
# ---------------------------------------------------------------------------

# Any of these strings appearing in calibrate.py source would indicate that
# gear-ratio logic has been introduced into the calibration range/position math.
_GEAR_PATTERNS: list[str] = [
    "gear_ratio",
    "motor_spec(",
    "role_motors(",
]


def test_calibrate_is_not_gear_aware():
    """calibrate.py must NOT reference gear ratios, motor_spec(), or role_motors().

    Gear-corrected calibration math (e.g. converting raw ticks via gear_ratio
    before saving the profile) is a deferred feature.  This guard catches an
    accidental early introduction of that logic.

    Allowed: calibrate.py imports arm_spec (to derive joint_ids) — that import
    is already present and correct.  What is forbidden is any reference to the
    gear_ratio field or the MotorSpec-returning accessors (motor_spec /
    role_motors), which would indicate the deferred gear-math has been wired in.
    """
    source = _CALIBRATE.read_text(encoding="utf-8")
    violations: list[tuple[int, str, str]] = []

    for pattern in _GEAR_PATTERNS:
        for lineno, line in enumerate(source.splitlines(), start=1):
            if pattern in line:
                stripped = line.lstrip()
                if not stripped.startswith("#"):
                    violations.append((lineno, pattern, line.rstrip()))

    assert violations == [], (
        "calibrate.py references gear-aware symbols — deferred gear-corrected "
        "calibration math must not be introduced yet:\n"
        + "\n".join(f"  line {n}: {text!r}  (matched {pat!r})" for n, pat, text in violations)
    )
