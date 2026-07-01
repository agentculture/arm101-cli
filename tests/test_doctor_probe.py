"""Tests for ``arm101 doctor --probe`` — the multi-baud probe path (t7).

These tests monkeypatch ``arm101.cli._commands.doctor.probe_bus`` so no real
hardware/serial is ever touched; the canned :class:`ProbeReport` objects are
built from the real ``arm101.hardware.baud_probe`` dataclasses.

The plain (no ``--probe``) identity-diagnosis path is exercised elsewhere in
``tests/test_cli_introspection.py`` and must stay untouched by this work —
the regression tests at the bottom of this file re-assert that here too.
"""

from __future__ import annotations

import json

import pytest

from arm101.cli import main
from arm101.cli._commands import doctor
from arm101.cli._errors import EXIT_ENV_ERROR
from arm101.hardware import bus as bus_module
from arm101.hardware.baud_probe import ProbeRecord, ProbeReport


@pytest.fixture(autouse=True)
def _sdk_present(monkeypatch):
    """Default every probe test to 'SDK installed' so ``_run_probe``'s
    ``require_sdk()`` pre-flight passes and the canned ``probe_bus`` is reached.

    This exercises the *real* ``require_sdk`` (not a stub); the dedicated
    missing-SDK test flips ``sdk_available`` to ``False`` to assert the other
    branch. Without this, the probe tests would fail on any host (e.g. CI) that
    lacks the optional ``scservo_sdk`` extra.
    """
    monkeypatch.setattr(bus_module, "sdk_available", lambda: True)


# ---------------------------------------------------------------------------
# Canned reports
# ---------------------------------------------------------------------------


def _mixed_report(port: str = "/dev/ttyFAKE") -> ProbeReport:
    """A report with one SUCCESS id and one CORRUPT id — not fully silent."""
    return ProbeReport(
        port=port,
        records=[
            ProbeRecord(1_000_000, 1, "SUCCESS", "id+model coherent (model=777)"),
            ProbeRecord(500_000, 2, "CORRUPT", "incoherent read: reported id=9, model=0"),
            ProbeRecord(1_000_000, 2, "TIMEOUT", "not present in scan()"),
        ],
    )


def _silent_report(port: str = "/dev/ttyFAKE") -> ProbeReport:
    """A fully-silent report — every record TIMEOUT."""
    return ProbeReport(
        port=port,
        records=[
            ProbeRecord(1_000_000, 1, "TIMEOUT", "not present in scan()"),
            ProbeRecord(500_000, 1, "TIMEOUT", "not present in scan()"),
        ],
    )


# ---------------------------------------------------------------------------
# --probe --port (text mode)
# ---------------------------------------------------------------------------


def test_probe_text_reports_port_and_classifications(monkeypatch, capsys) -> None:
    report = _mixed_report()
    monkeypatch.setattr(doctor, "probe_bus", lambda port: report)

    rc = main(["doctor", "--probe", "--port", "/dev/ttyFAKE"])

    assert rc == 0
    out, err = capsys.readouterr()
    assert err == ""
    assert "/dev/ttyFAKE" in out
    assert "SUCCESS" in out
    assert "CORRUPT" in out
    assert out.strip() == report.summary().strip()


# ---------------------------------------------------------------------------
# --probe --port --json
# ---------------------------------------------------------------------------


def test_probe_json_matches_report_to_dict(monkeypatch, capsys) -> None:
    report = _mixed_report()
    monkeypatch.setattr(doctor, "probe_bus", lambda port: report)

    rc = main(["doctor", "--probe", "--port", "/dev/ttyFAKE", "--json"])

    assert rc == 0
    out, err = capsys.readouterr()
    assert err == ""
    payload = json.loads(out)
    assert payload == report.to_dict()


# ---------------------------------------------------------------------------
# Fully-silent bus -> diagnosed, exit 0 (not a failure)
# ---------------------------------------------------------------------------


def test_probe_fully_silent_bus_is_exit_zero(monkeypatch, capsys) -> None:
    report = _silent_report()
    monkeypatch.setattr(doctor, "probe_bus", lambda port: report)

    rc = main(["doctor", "--probe", "--port", "/dev/ttyFAKE"])

    assert rc == 0
    out, _err = capsys.readouterr()
    assert "no servo answered at any baud" in out


def test_probe_fully_silent_bus_json_flags_fully_silent(monkeypatch, capsys) -> None:
    report = _silent_report()
    monkeypatch.setattr(doctor, "probe_bus", lambda port: report)

    rc = main(["doctor", "--probe", "--port", "/dev/ttyFAKE", "--json"])

    assert rc == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["fully_silent"] is True


# ---------------------------------------------------------------------------
# No --port and no candidate ports -> CliError(EXIT_ENV_ERROR)
# ---------------------------------------------------------------------------


def test_probe_no_port_no_candidates_raises_env_error(monkeypatch, capsys) -> None:
    monkeypatch.setattr(doctor, "_candidate_ports", lambda: [])
    # probe_bus must never be reached when no port resolves.
    monkeypatch.setattr(
        doctor,
        "probe_bus",
        lambda port: pytest.fail("probe_bus should not be called with no resolvable port"),
    )

    rc = main(["doctor", "--probe"])

    assert rc == EXIT_ENV_ERROR
    out, err = capsys.readouterr()
    assert out == ""
    assert err.startswith("error:")
    assert "hint:" in err
    assert "Traceback" not in err


# ---------------------------------------------------------------------------
# Missing Feetech SDK -> clear CliError(EXIT_ENV_ERROR), NOT a "silent bus"
# misdiagnosis (regression for the qodo #22 finding).
# ---------------------------------------------------------------------------


def test_probe_missing_sdk_raises_env_error(monkeypatch, capsys) -> None:
    # Override the autouse "SDK present" default: pretend scservo_sdk is absent.
    monkeypatch.setattr(bus_module, "sdk_available", lambda: False)
    # probe_bus must never run: a missing SDK is diagnosed up front, not
    # degraded into TIMEOUT-everywhere with exit 0.
    monkeypatch.setattr(
        doctor,
        "probe_bus",
        lambda port: pytest.fail("probe_bus must not be reached when the SDK is missing"),
    )

    rc = main(["doctor", "--probe", "--port", "/dev/ttyFAKE"])

    assert rc == EXIT_ENV_ERROR
    out, err = capsys.readouterr()
    assert out == ""
    assert err.startswith("error:")
    assert "not installed" in err
    assert "hint:" in err
    assert "seeed" in err  # the pip-install remediation hint
    assert "Traceback" not in err


def test_probe_no_port_auto_detects_first_candidate(monkeypatch, capsys) -> None:
    report = _mixed_report(port="/dev/ttyAUTO")
    monkeypatch.setattr(doctor, "_candidate_ports", lambda: ["/dev/ttyAUTO", "/dev/ttyOTHER"])
    seen_ports: list[str] = []

    def _fake_probe_bus(port: str) -> ProbeReport:
        seen_ports.append(port)
        return report

    monkeypatch.setattr(doctor, "probe_bus", _fake_probe_bus)

    rc = main(["doctor", "--probe"])

    assert rc == 0
    assert seen_ports == ["/dev/ttyAUTO"]
    out, _err = capsys.readouterr()
    assert "/dev/ttyAUTO" in out


# ---------------------------------------------------------------------------
# Regression: the identity-diagnosis path (no --probe) is unchanged.
# ---------------------------------------------------------------------------


def test_plain_doctor_still_runs_identity_diagnosis(capsys) -> None:
    rc = main(["doctor"])
    assert rc in (0, 1)
    out, err = capsys.readouterr()
    assert "arm101-cli doctor" in out
    assert err == ""


def test_plain_doctor_json_still_runs_identity_diagnosis(capsys) -> None:
    rc = main(["doctor", "--json"])
    assert rc in (0, 1)
    payload = json.loads(capsys.readouterr().out)
    assert isinstance(payload["healthy"], bool)
    assert isinstance(payload["checks"], list)
    assert payload["checks"]
