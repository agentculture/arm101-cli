"""Tests for arm101.hardware.ports — serial-port enumeration.

Design: enumerate_ports() accepts an optional ``_roots`` parameter (a sequence of
glob-pattern strings) so tests can point it at a fake /dev tree under tmp_path
without touching real hardware or /dev.

Coverage targets:
  - Linux: fake /dev tree with ttyACM, ttyUSB, and by-id entries → sorted results
  - Linux: empty dev tree → []
  - macOS: raises CliError(EXIT_ENV_ERROR)
  - Windows: raises CliError(EXIT_ENV_ERROR)
  - Import-clean assertion: module imports without error and has no third-party imports
"""

from __future__ import annotations

import sys

import pytest

from arm101.cli._errors import EXIT_ENV_ERROR, CliError

# ---------------------------------------------------------------------------
# Import-clean assertion
# ---------------------------------------------------------------------------


def test_module_imports_cleanly() -> None:
    """arm101.hardware.ports must be importable with zero third-party deps."""
    import arm101.hardware.ports as ports_mod  # noqa: F401

    # Verify the public API exists
    assert callable(ports_mod.enumerate_ports)


def test_no_third_party_imports() -> None:
    """The ports module must not depend on any third-party package.

    We check by inspecting the module's globals for suspicious imports.
    stdlib modules are allowed; anything outside stdlib is not.
    """
    import arm101.hardware.ports as ports_mod

    # Collect the names of all modules imported at module level
    stdlib_prefixes = {
        "arm101",
        "_io",
        "abc",
        "ast",
        "builtins",
        "collections",
        "contextlib",
        "dataclasses",
        "enum",
        "functools",
        "glob",
        "importlib",
        "inspect",
        "io",
        "itertools",
        "json",
        "logging",
        "math",
        "os",
        "pathlib",
        "platform",
        "re",
        "shutil",
        "signal",
        "socket",
        "struct",
        "subprocess",
        "sys",
        "textwrap",
        "threading",
        "time",
        "traceback",
        "types",
        "typing",
        "unittest",
        "warnings",
    }

    import types

    for name, obj in vars(ports_mod).items():
        if isinstance(obj, types.ModuleType):
            top_level = obj.__name__.split(".")[0]
            assert (
                top_level in stdlib_prefixes or top_level == "arm101"
            ), f"Third-party module detected in arm101.hardware.ports: {obj.__name__!r}"


# ---------------------------------------------------------------------------
# Linux: fake /dev tree
# ---------------------------------------------------------------------------


def _make_fake_dev(tmp_path):
    """Create a fake /dev-like tree with ttyACM, ttyUSB, and serial/by-id entries."""
    # /dev/ttyACM0, /dev/ttyACM1
    acm0 = tmp_path / "ttyACM0"
    acm0.touch()
    acm1 = tmp_path / "ttyACM1"
    acm1.touch()

    # /dev/ttyUSB0
    usb0 = tmp_path / "ttyUSB0"
    usb0.touch()

    # /dev/serial/by-id/<link>
    by_id_dir = tmp_path / "serial" / "by-id"
    by_id_dir.mkdir(parents=True)
    by_id_entry = by_id_dir / "usb-Feetech_STS3215-if00-port0"
    by_id_entry.touch()

    return tmp_path


def test_linux_fake_dev_returns_sorted_list(tmp_path, monkeypatch):
    """On Linux, enumerate_ports() with a fake /dev tree returns a sorted list."""
    monkeypatch.setattr(sys, "platform", "linux")
    fake_dev = _make_fake_dev(tmp_path)

    import arm101.hardware.ports as ports_mod

    roots = [
        str(fake_dev / "ttyACM*"),
        str(fake_dev / "ttyUSB*"),
        str(fake_dev / "serial" / "by-id" / "*"),
    ]
    result = ports_mod.enumerate_ports(_roots=roots)

    assert isinstance(result, list)
    # Should contain all four fake entries
    assert len(result) == 4

    # Must be sorted
    assert result == sorted(result)

    # Spot-check expected entries
    result_names = [p.split("/")[-1] for p in result]
    assert "ttyACM0" in result_names
    assert "ttyACM1" in result_names
    assert "ttyUSB0" in result_names
    assert "usb-Feetech_STS3215-if00-port0" in result_names


def test_linux_fake_dev_sorted_order(tmp_path, monkeypatch):
    """Results are lexicographically sorted (full path, not just basename)."""
    monkeypatch.setattr(sys, "platform", "linux")
    fake_dev = _make_fake_dev(tmp_path)

    import arm101.hardware.ports as ports_mod

    roots = [
        str(fake_dev / "ttyACM*"),
        str(fake_dev / "ttyUSB*"),
        str(fake_dev / "serial" / "by-id" / "*"),
    ]
    result = ports_mod.enumerate_ports(_roots=roots)
    assert result == sorted(result), "Results must be lexicographically sorted"


def test_linux_empty_dev_returns_empty_list(tmp_path, monkeypatch):
    """On Linux with no matching devices, enumerate_ports() returns []."""
    monkeypatch.setattr(sys, "platform", "linux")

    import arm101.hardware.ports as ports_mod

    # Point at an empty dir — no ttyACM/ttyUSB/by-id entries
    roots = [
        str(tmp_path / "ttyACM*"),
        str(tmp_path / "ttyUSB*"),
        str(tmp_path / "serial" / "by-id" / "*"),
    ]
    result = ports_mod.enumerate_ports(_roots=roots)
    assert result == []


# ---------------------------------------------------------------------------
# macOS: must raise CliError(EXIT_ENV_ERROR)
# ---------------------------------------------------------------------------


def test_macos_raises_cli_error(monkeypatch):
    """On macOS, enumerate_ports() raises CliError with EXIT_ENV_ERROR."""
    monkeypatch.setattr(sys, "platform", "darwin")

    import importlib

    import arm101.hardware.ports as ports_mod

    importlib.reload(ports_mod)  # ensure platform check re-evaluates

    with pytest.raises(CliError) as exc_info:
        ports_mod.enumerate_ports()

    err = exc_info.value
    assert err.code == EXIT_ENV_ERROR
    assert "darwin" in err.message.lower() or "mac" in err.message.lower()
    assert err.remediation  # must have a non-empty remediation hint


# ---------------------------------------------------------------------------
# Windows: must raise CliError(EXIT_ENV_ERROR)
# ---------------------------------------------------------------------------


def test_windows_raises_cli_error(monkeypatch):
    """On Windows, enumerate_ports() raises CliError with EXIT_ENV_ERROR."""
    monkeypatch.setattr(sys, "platform", "win32")

    import importlib

    import arm101.hardware.ports as ports_mod

    importlib.reload(ports_mod)

    with pytest.raises(CliError) as exc_info:
        ports_mod.enumerate_ports()

    err = exc_info.value
    assert err.code == EXIT_ENV_ERROR
    assert "win" in err.message.lower() or "windows" in err.message.lower()
    assert err.remediation  # must have a non-empty remediation hint


# ---------------------------------------------------------------------------
# Default roots (smoke-test that default call signature is correct)
# ---------------------------------------------------------------------------


def test_default_roots_used_on_linux(monkeypatch):
    """On Linux, calling enumerate_ports() without _roots uses real /dev globs.

    We just check the call doesn't raise and returns a list. We do NOT assert
    specific content (no real hardware required in CI).
    """
    monkeypatch.setattr(sys, "platform", "linux")

    import arm101.hardware.ports as ports_mod

    result = ports_mod.enumerate_ports()
    assert isinstance(result, list)
    assert result == sorted(result)
