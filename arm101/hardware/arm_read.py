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
    position, load, speed, voltage, temperature, torque:
        Register values mapped from :meth:`MotorBus.read_info`'s
        ``present_position``, ``present_load``, ``present_speed``,
        ``present_voltage``, ``present_temperature``, and ``torque_enable``
        keys respectively. ``None`` when ``health == "failed"``.
    """

    joint: str
    motor_id: int
    health: str
    position: "int | None" = None
    load: "int | None" = None
    speed: "int | None" = None
    voltage: "int | None" = None
    temperature: "int | None" = None
    torque: "int | None" = None


def _read_joint_with_retries(
    bus: "MotorBus", motor_id: int, retries: int
) -> "tuple[str, dict[str, int] | None]":
    """Read *motor_id* via ``bus.read_info``, retrying on any exception.

    Makes up to ``1 + retries`` attempts. Returns ``(health, info)`` where
    ``info`` is the raw ``read_info`` dict on success or ``None`` if every
    attempt raised.
    """
    total_attempts = 1 + retries
    for attempt in range(total_attempts):
        try:
            info = bus.read_info(motor_id)
        except Exception:  # noqa: BLE001 - any read failure is retry-worthy
            info = None
        if info is not None:
            health = _HEALTH_OK if attempt == 0 else _HEALTH_PARTIAL
            return health, info
    return _HEALTH_FAILED, None


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
        health, info = _read_joint_with_retries(bus, motor_id, retries)
        if info is None:
            results.append(JointReading(joint=joint, motor_id=motor_id, health=health))
            continue
        fields = {field: info.get(key) for field, key in _FIELD_MAP.items()}
        results.append(JointReading(joint=joint, motor_id=motor_id, health=health, **fields))
    return results


def is_complete(readings: "list[JointReading]") -> bool:
    """Return ``True`` iff no reading in *readings* has ``health == "failed"``."""
    return all(r.health != _HEALTH_FAILED for r in readings)
