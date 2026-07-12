"""Retry-tolerant whole-arm read snapshot.

Pure-logic module: reads each joint's live register state from a
:class:`~arm101.hardware.bus.MotorBus`, tolerant of transient RX timeouts via
bounded per-joint retries. A single joint whose reads keep failing is marked
``"failed"`` (no register data) without aborting the rest of the snapshot —
:func:`read_arm` never raises on a per-joint read failure.

Deliberately decoupled from ``arm_spec``: the caller supplies the
joint-name -> motor-id mapping, so this module has no opinion about which
joints exist or what they are called.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Mapping

from arm101.hardware.bus import OverloadError

if TYPE_CHECKING:
    from arm101.hardware.bus import MotorBus

#: Health flag literal values, hoisted so the same string is not duplicated.
_HEALTH_OK = "ok"
_HEALTH_PARTIAL = "partial"
_HEALTH_FAILED = "failed"

#: JointReading field name -> the MotorBus.read_info() key it is sourced from.
_FIELD_MAP: "dict[str, str]" = {
    "position": "present_position",
    "load": "present_load",
    "speed": "present_speed",
    "voltage": "present_voltage",
    "temperature": "present_temperature",
    "torque": "torque_enable",
    "offset": "homing_offset",
}


@dataclass
class JointReading:
    """One joint's read result from a whole-arm snapshot.

    Attributes
    ----------
    joint:
        Joint name as supplied by the caller's ``joints`` mapping.
    motor_id:
        The motor id read for this joint.
    health:
        ``"ok"`` — the first read attempt succeeded.
        ``"partial"`` — a read succeeded only after one or more retries.
        ``"failed"`` — every attempt (``1 + retries``) raised; no register
        data is available and all fields below are ``None``.
    overloaded:
        ``True`` iff every attempt raised and the LAST exception encountered
        was an :class:`~arm101.hardware.bus.OverloadError` (the servo's own
        overload latch, status error bit 5 — see
        :func:`arm101.hardware.bus.is_overload`), so ``health`` is
        ``"failed"`` FOR THIS SPECIFIC REASON rather than a generic comms
        failure. Purely additive/informational — it never changes
        ``health`` or the retry control flow, it only distinguishes WHY a
        failed joint failed. ``False`` on every other outcome (success, or
        a failure whose last exception was not an overload).
    position, load, speed, voltage, temperature, torque:
        Register values mapped from :meth:`MotorBus.read_info`'s
        ``present_position``, ``present_load``, ``present_speed``,
        ``present_voltage``, ``present_temperature``, and ``torque_enable``
        keys respectively. ``None`` when ``health == "failed"``.
    offset:
        The joint's encoder offset (``Ofs`` / ``Homing_Offset``, EEPROM addr
        31), **signed** — the bus decodes it from its sign-magnitude wire form,
        so a servo holding ``-1073`` reads as ``-1073``, not as the raw ``3121``
        (which would look like a plausible *positive* offset).

        Surfaced read-only, for issue #35: ``elbow_flex``'s encoder wraps inside
        its physical travel, and the fix is to re-zero it by writing this
        register. Before anyone writes it, a human has to be able to SEE what it
        currently holds. ``0`` on a factory servo; ``None`` when
        ``health == "failed"`` — a hard ``0`` there would assert "this motor has
        no offset", a claim a failed read has no evidence for.
    """

    joint: str
    motor_id: int
    health: str
    overloaded: bool = False
    position: "int | None" = None
    load: "int | None" = None
    speed: "int | None" = None
    voltage: "int | None" = None
    temperature: "int | None" = None
    torque: "int | None" = None
    offset: "int | None" = None


def _read_joint_with_retries(
    bus: "MotorBus", motor_id: int, retries: int
) -> "tuple[str, dict[str, int] | None, bool]":
    """Read *motor_id* via ``bus.read_info``, retrying on any exception.

    Makes up to ``1 + retries`` attempts. Returns ``(health, info,
    overloaded)`` where ``info`` is the raw ``read_info`` dict on success or
    ``None`` if every attempt raised, and ``overloaded`` is ``True`` iff every
    attempt raised AND the last exception was an
    :class:`~arm101.hardware.bus.OverloadError` — additive-only, it never
    influences ``health``/retry control flow (see :class:`JointReading`).
    """
    total_attempts = 1 + retries
    last_exc: "Exception | None" = None
    for attempt in range(total_attempts):
        try:
            info = bus.read_info(motor_id)
        except Exception as exc:  # noqa: BLE001 - any read failure is retry-worthy
            info = None
            last_exc = exc
        if info is not None:
            health = _HEALTH_OK if attempt == 0 else _HEALTH_PARTIAL
            return health, info, False
    return _HEALTH_FAILED, None, isinstance(last_exc, OverloadError)


def read_arm(
    bus: "MotorBus", joints: "Mapping[str, int]", *, retries: int = 2
) -> "list[JointReading]":
    """Read all joints' live state from *bus*, tolerant of transient failures.

    Parameters
    ----------
    bus:
        An already-:meth:`~arm101.hardware.bus.MotorBus.open`\\ ed bus.
    joints:
        Mapping of joint name -> motor id. Results are returned in this
        mapping's iteration order.
    retries:
        Number of additional read attempts per joint after the first
        failure, before giving up and marking the joint ``"failed"``.

    Returns
    -------
    list[JointReading]
        One entry per entry in *joints*. A joint whose reads keep raising is
        marked ``health="failed"`` with all register fields ``None`` — this
        function never raises on a per-joint read failure, so the other
        joints are always returned.
    """
    results: "list[JointReading]" = []
    for joint, motor_id in joints.items():
        health, info, overloaded = _read_joint_with_retries(bus, motor_id, retries)
        if info is None:
            results.append(
                JointReading(joint=joint, motor_id=motor_id, health=health, overloaded=overloaded)
            )
            continue
        fields = {field: info.get(key) for field, key in _FIELD_MAP.items()}
        results.append(
            JointReading(
                joint=joint, motor_id=motor_id, health=health, overloaded=overloaded, **fields
            )
        )
    return results


def is_complete(readings: "list[JointReading]") -> bool:
    """Return ``True`` iff no reading in *readings* has ``health == "failed"``."""
    return all(r.health != _HEALTH_FAILED for r in readings)
