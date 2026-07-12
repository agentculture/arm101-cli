"""Nothing in ``arm101/`` may write servo address 9 or 11. Enforced, not asserted.

Addresses 9 and 11 are the STS3215's ``Min_Position_Limit`` and
``Max_Position_Limit`` (both 2 bytes, so they occupy the byte range 9-12 —
``FeetechBus._INFO_REGISTERS`` already names them ``min_angle``/``max_angle``).
In servo mode the firmware **CLAMPS every goal position** to that window. Writing
them would narrow the very reachable set this whole line of work exists to
recover — and, being EEPROM, the narrowing would outlive the pose that produced
it: a servo re-installed on another arm, or the software reinstalled from
scratch, would still carry it. Measured and derived ranges belong in
:mod:`~arm101.hardware.arm_spec` and in the reachability map. LeRobot's
``write_calibration`` writes 9 and 11; it must not be ported here.

**Why this is not a source grep.** ``9`` and ``11`` are ordinary numbers — they
appear as motor counts, list indices, bit positions (``OFFSET_SIGN_BIT`` is
literally ``11``). A grep for the literals is noise, and a grep for
``Min_Position_Limit`` proves nothing, because the register that gets written is
named by an *address*, not by a string. And the addresses 9/11 legitimately DO
appear in ``bus.py`` — ``read_info`` reads them, which is how we know the factory
limits are the wide-open 0/4095. A guard that cannot tell a read from a write is
a guard that either fires on the read or never fires at all.

So the guard pins the **boundary where an address becomes a wire write**. Every
byte this package can put into a servo register leaves through exactly one door:
a ``self._packet_handler.writeNByteTxRx(port, id, address, value)`` call in
:mod:`arm101.hardware.bus`. This module parses ``arm101/`` and:

1. asserts that door is the ONLY one — no other module holds a packet handler
   (:func:`test_the_sdk_wire_lives_only_in_bus_py`); then
2. enumerates every write call site through it, statically resolving the address
   argument (through named constants and through ``write_id_baudrate``'s
   ``for addr, val, label in (...)`` loop) to the concrete integers it can hold;
3. **fails closed**: an SDK method it does not recognise, a write handed off by
   reference so its address is invisible, or an address that does not resolve to
   an int constant is a FAILURE, not a silent skip; and
4. asserts the resulting byte spans (address .. address + width - 1, so an
   off-by-one 2-byte write at addr 8 or 10 is caught too) never touch 9-12.

A regression this catches that a per-verb behavioural test would not: someone
ports LeRobot's ``write_calibration`` by adding a ``write_position_limits()``
method to ``FeetechBus``. No existing test calls it, so every existing test still
passes — including ``tests/test_bus_offset.py``'s allow-list, which pins the
write surface of ``write_offset`` only. Here, the new method's
``write2ByteTxRx(..., _ADDR_MIN_LIMIT, ...)`` call site is discovered by the scan
and both the byte-span test and the inventory test fail immediately.

Complemented by a live check: the whole public write surface of ``FeetechBus`` is
driven against a recording packet handler, and the addresses that actually reach
the wire must agree with what the static scan predicted — so the analysis above
is corroborated by execution rather than trusted on its own.
"""

from __future__ import annotations

import ast
import inspect
import pathlib
import time
from typing import NamedTuple

import pytest

from arm101.hardware import bus as bus_module
from arm101.hardware.bus import FeetechBus

# ---------------------------------------------------------------------------
# What the guard forbids — derived from bus.py's own register map
# ---------------------------------------------------------------------------

_ARM101_ROOT = pathlib.Path(__file__).parent.parent / "arm101"
_BUS_MODULE_FILENAME = "bus.py"

#: The two position-limit registers, taken from the map ``read_info`` already
#: uses, so the guard cannot drift away from the addresses the code believes in.
_MIN_LIMIT_ADDR, _MIN_LIMIT_WIDTH = FeetechBus._INFO_REGISTERS["min_angle"]
_MAX_LIMIT_ADDR, _MAX_LIMIT_WIDTH = FeetechBus._INFO_REGISTERS["max_angle"]

#: Every BYTE the two registers occupy: 9-10 and 11-12. A 2-byte write at addr 8
#: or 10 would spill into them just as surely as one aimed squarely at 9.
_FORBIDDEN_BYTES = frozenset(
    list(range(_MIN_LIMIT_ADDR, _MIN_LIMIT_ADDR + _MIN_LIMIT_WIDTH))
    + list(range(_MAX_LIMIT_ADDR, _MAX_LIMIT_ADDR + _MAX_LIMIT_WIDTH))
)

# ---------------------------------------------------------------------------
# The SDK surface: the exact call shapes that put bytes on the wire
# ---------------------------------------------------------------------------

#: The attribute that holds the Feetech SDK packet handler. Every register byte
#: this package sends leaves through a method call on this object.
_HANDLER_ATTR = "_packet_handler"

#: SDK methods that send a packet but write no register byte.
_SDK_READ_METHODS = frozenset({"read1ByteTxRx", "read2ByteTxRx", "read4ByteTxRx", "ping"})

#: SDK write methods -> how many BYTES each one deposits, starting at the address.
_SDK_WRITE_METHODS = {"write1ByteTxRx": 1, "write2ByteTxRx": 2, "write4ByteTxRx": 4}

#: ``fn(port_handler, servo_id, address, value)`` — the address is the third arg.
_ADDRESS_ARG_INDEX = 2

#: Identifiers that would mean a module is talking to the SDK directly.
_SDK_IDENTIFIER_MARKERS = ("TxRx", "PacketHandler", "GroupSync", _HANDLER_ATTR)


class _WireWrite(NamedTuple):
    """One statically discovered register write, and the bytes it can land on."""

    module: str
    lineno: int
    method: str
    addresses: frozenset[int]

    @property
    def width(self) -> int:
        return _SDK_WRITE_METHODS[self.method]

    @property
    def byte_span(self) -> frozenset[int]:
        """Every byte the write deposits — a 2-byte write at addr 8 also touches 9."""
        return frozenset(byte for addr in self.addresses for byte in range(addr, addr + self.width))

    @property
    def where(self) -> str:
        return f"{self.module}:{self.lineno} {self.method}"


# ---------------------------------------------------------------------------
# A very small constant-resolver over the AST
# ---------------------------------------------------------------------------


def _is_int_const(node: ast.AST) -> bool:
    """An ``int`` literal — and not a ``bool``, which Python would happily call an int."""
    return (
        isinstance(node, ast.Constant)
        and isinstance(node.value, int)
        and not isinstance(node.value, bool)
    )


def _bind(name: str, value: ast.AST, env: "dict[str, set[int]]") -> None:
    """Bind *name* to the int(s) *value* can be, if that is knowable statically."""
    if _is_int_const(value):
        env.setdefault(name, set()).add(value.value)  # type: ignore[attr-defined]
    elif isinstance(value, ast.Name) and value.id in env:
        env.setdefault(name, set()).update(env[value.id])


def _int_bindings(node: ast.AST, direct_only: bool = False) -> "dict[str, set[int]]":
    """Names bound to int constants inside *node* (module top level, or a function body).

    Two passes, because ``write_id_baudrate`` binds its address through a loop
    over a literal tuple whose elements are *themselves* named constants::

        _ADDR_ID = 5
        _ADDR_BAUD = 6
        for addr, val, label in ((_ADDR_BAUD, baud_index, "Baud_Rate"),
                                 (_ADDR_ID, new_id, "ID")):
            ... write1ByteTxRx(port, motor, addr, val)

    Pass 1 learns the constants; pass 2 unpacks the loop targets using them, so
    ``addr`` resolves to ``{5, 6}`` — both of which the guard then checks.
    """
    nodes = list(ast.iter_child_nodes(node) if direct_only else ast.walk(node))
    env: dict[str, set[int]] = {}

    for stmt in nodes:
        if isinstance(stmt, ast.Assign):
            pairs = [(target, stmt.value) for target in stmt.targets]
        elif isinstance(stmt, ast.AnnAssign) and stmt.value is not None:
            pairs = [(stmt.target, stmt.value)]
        else:
            continue
        for target, value in pairs:
            if isinstance(target, ast.Name):
                _bind(target.id, value, env)

    for stmt in nodes:
        if not isinstance(stmt, ast.For) or not isinstance(stmt.iter, (ast.Tuple, ast.List)):
            continue
        for item in stmt.iter.elts:
            if isinstance(stmt.target, ast.Name):
                _bind(stmt.target.id, item, env)
            elif isinstance(stmt.target, ast.Tuple) and isinstance(item, (ast.Tuple, ast.List)):
                for slot, element in zip(stmt.target.elts, item.elts):
                    if isinstance(slot, ast.Name):
                        _bind(slot.id, element, env)

    return env


def _resolve_address(
    expr: ast.AST,
    local: "dict[str, set[int]]",
    params: "set[str]",
    module_env: "dict[str, set[int]]",
) -> "set[int] | None":
    """The concrete addresses *expr* can hold, or ``None`` if that is not knowable.

    ``None`` is a FAILURE, never a pass. An address arriving as a function
    parameter is unknowable here by construction — and a wire write whose address
    the caller chooses is exactly the shape a smuggled limit-write would take.
    """
    if _is_int_const(expr):
        return {expr.value}  # type: ignore[attr-defined]
    if isinstance(expr, ast.Name):
        if expr.id in params:
            return None
        for env in (local, module_env):
            if expr.id in env:
                return set(env[expr.id])
    return None


def _link_parents(tree: ast.AST) -> None:
    for parent in ast.walk(tree):
        for child in ast.iter_child_nodes(parent):
            child._guard_parent = parent  # type: ignore[attr-defined]


def _enclosing_function(node: ast.AST) -> "ast.FunctionDef | ast.AsyncFunctionDef | None":
    current = getattr(node, "_guard_parent", None)
    while current is not None:
        if isinstance(current, (ast.FunctionDef, ast.AsyncFunctionDef)):
            return current
        current = getattr(current, "_guard_parent", None)
    return None


def _param_names(fn: "ast.FunctionDef | ast.AsyncFunctionDef") -> "set[str]":
    a = fn.args
    return {
        arg.arg
        for arg in [*a.posonlyargs, *a.args, *a.kwonlyargs, a.vararg, a.kwarg]
        if arg is not None
    }


def _is_packet_handler(node: ast.AST) -> bool:
    """True for the ``self._packet_handler`` sub-expression of an SDK call."""
    return isinstance(node, ast.Attribute) and node.attr == _HANDLER_ATTR


# ---------------------------------------------------------------------------
# The scan
# ---------------------------------------------------------------------------


def _python_files() -> "list[pathlib.Path]":
    return sorted(p for p in _ARM101_ROOT.rglob("*.py") if "__pycache__" not in p.parts)


def _scan_module(path: pathlib.Path) -> "tuple[list[_WireWrite], list[str]]":
    """Every wire write in *path*, plus every reason the scan could not be sure."""
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    _link_parents(tree)
    module_env = _int_bindings(tree, direct_only=True)

    calls_by_func = {
        id(node.func): node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }

    writes: list[_WireWrite] = []
    problems: list[str] = []

    for node in ast.walk(tree):
        if not (isinstance(node, ast.Attribute) and _is_packet_handler(node.value)):
            continue

        method = node.attr
        where = f"{path.name}:{node.lineno} {method}"

        if method in _SDK_READ_METHODS:
            continue  # a read deposits no byte in any register
        if method not in _SDK_WRITE_METHODS:
            problems.append(
                f"{where}: unrecognised SDK method. Classify it in _SDK_READ_METHODS or "
                "_SDK_WRITE_METHODS — until then this guard cannot say whether it writes "
                "a register, so it must not pass."
            )
            continue

        call = calls_by_func.get(id(node))
        if call is None:
            problems.append(
                f"{where}: a write method is referenced without being called here, so its "
                "target address is invisible to this guard. Call it directly (reads may be "
                "passed by reference; writes may not)."
            )
            continue
        if len(call.args) <= _ADDRESS_ARG_INDEX:
            problems.append(
                f"{where}: the address is not the third positional argument, so this guard "
                "cannot read it. Keep the SDK call shape fn(port, id, address, value)."
            )
            continue

        fn = _enclosing_function(node)
        local = _int_bindings(fn) if fn is not None else {}
        params = _param_names(fn) if fn is not None else set()
        addresses = _resolve_address(call.args[_ADDRESS_ARG_INDEX], local, params, module_env)

        if addresses is None:
            problems.append(
                f"{where}: the address argument does not resolve to an int constant "
                "(it comes from a parameter or is computed). A wire write whose address a "
                "caller chooses could target the position-limit registers — name the address "
                "with a constant so this guard can check it."
            )
            continue

        writes.append(_WireWrite(path.name, node.lineno, method, frozenset(addresses)))

    return writes, problems


def _scan_package() -> "tuple[list[_WireWrite], list[str]]":
    writes: list[_WireWrite] = []
    problems: list[str] = []
    for path in _python_files():
        module_writes, module_problems = _scan_module(path)
        writes.extend(module_writes)
        problems.extend(module_problems)
    return writes, problems


_WRITES, _PROBLEMS = _scan_package()

#: Every register address ``arm101`` is CAPABLE of writing. Not "does write in the
#: paths the tests happen to exercise" — capable of, on any path, for any reason.
_WRITABLE_ADDRESSES = frozenset(addr for write in _WRITES for addr in write.addresses)


# ===========================================================================
# 1. The scan is sound: one door, no unknowns, and it actually found something
# ===========================================================================


def test_the_sdk_wire_lives_only_in_bus_py():
    """Only ``bus.py`` may hold an SDK packet handler.

    The static enumeration below is only exhaustive if there is exactly one place
    where an address becomes a wire write. A second module reaching for the SDK
    directly would open a door this guard is not watching — so that is itself the
    failure. (Identifiers are read out of the AST, not grepped, so the docstrings
    and error strings elsewhere that *mention* ``write1ByteTxRx`` are not matches:
    only real code is.)
    """
    offenders: dict[str, set[str]] = {}

    for path in _python_files():
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        identifiers = {
            node.attr if isinstance(node, ast.Attribute) else node.id
            for node in ast.walk(tree)
            if isinstance(node, (ast.Attribute, ast.Name))
        }
        imported = {
            alias.name
            for node in ast.walk(tree)
            if isinstance(node, (ast.Import, ast.ImportFrom))
            for alias in node.names
        } | {node.module or "" for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)}

        hits = {
            name
            for name in identifiers
            if any(marker in name for marker in _SDK_IDENTIFIER_MARKERS)
        }
        hits |= {name for name in imported if name.startswith("scservo_sdk")}

        if hits and path.name != _BUS_MODULE_FILENAME:
            offenders[str(path)] = hits

    assert offenders == {}, (
        "a module outside bus.py talks to the Feetech SDK directly — every register "
        "write must go through FeetechBus, which is the only surface this guard watches:\n"
        + "\n".join(f"  {path}: {sorted(names)}" for path, names in offenders.items())
    )


def test_every_sdk_call_is_classified_and_its_address_is_statically_known():
    """The guard fails CLOSED. Anything it cannot account for is a failure here.

    This is the test that keeps the byte-span test below honest: without it, a
    write whose address the scanner could not resolve would simply not appear in
    the enumeration, and "9 and 11 are not in the list" would be true because the
    list was incomplete.
    """
    assert _PROBLEMS == [], "the wire-write scan could not account for:\n" + "\n".join(
        f"  - {problem}" for problem in _PROBLEMS
    )


def test_the_scan_is_not_vacuous():
    """The scanner really did find the writes — it is not passing on an empty set.

    If ``_packet_handler`` were renamed, or the SDK call shape changed, the scan
    would find nothing and every assertion about "no write targets 9 or 11" would
    become vacuously true. So: it must find writes, and it must find the two
    registers we KNOW are written (torque-enable and the EEPROM lock), reached by
    both the 1-byte and the 2-byte SDK calls.
    """
    assert _WRITES, "no wire writes found at all — the scanner is looking at the wrong shape"
    assert all(write.module == _BUS_MODULE_FILENAME for write in _WRITES)

    assert bus_module.ADDR_TORQUE_ENABLE in _WRITABLE_ADDRESSES
    assert bus_module.ADDR_LOCK in _WRITABLE_ADDRESSES
    assert {write.method for write in _WRITES} >= {"write1ByteTxRx", "write2ByteTxRx"}


def test_the_guard_can_tell_a_read_from_a_write():
    """9 and 11 ARE reachable — for READING. The guard must not confuse the two.

    ``read_info`` reads ``min_angle``/``max_angle`` every time it runs; that is how
    we know this arm's limits are the factory-wide 0/4095. A guard that fired on
    any *mention* of address 9 would fire on that read, and would have been
    deleted long ago. What is forbidden is putting a byte INTO those registers.
    """
    read_addresses = {addr for addr, _width in FeetechBus._INFO_REGISTERS.values()}

    assert _MIN_LIMIT_ADDR in read_addresses
    assert _MAX_LIMIT_ADDR in read_addresses
    assert (_MIN_LIMIT_ADDR, _MIN_LIMIT_WIDTH) == (9, 2)  # STS3215 datasheet: Min_Position_Limit
    assert (_MAX_LIMIT_ADDR, _MAX_LIMIT_WIDTH) == (11, 2)  # STS3215 datasheet: Max_Position_Limit
    assert _FORBIDDEN_BYTES == {9, 10, 11, 12}


# ===========================================================================
# 2. The guard itself
# ===========================================================================


def test_no_wire_write_can_land_on_the_position_limit_registers():
    """Not one register byte this package can send may fall in 9-12.

    Falsified by a single such write, anywhere in ``arm101/``, for any reason —
    a new ``write_position_limits()`` method, a ported ``write_calibration``, or a
    2-byte write aimed one address short of 9 and spilling into it.
    """
    trespassers = [write for write in _WRITES if write.byte_span & _FORBIDDEN_BYTES]

    assert trespassers == [], (
        "a code path writes the servo's position-limit registers "
        f"(bytes {sorted(_FORBIDDEN_BYTES)} — Min/Max_Position_Limit). In servo mode the "
        "firmware CLAMPS goal positions to that window, so this narrows the reachable set "
        "permanently, in EEPROM. Ranges live in arm_spec and in the reachability map:\n"
        + "\n".join(
            f"  {write.where} -> addresses {sorted(write.addresses)}" for write in trespassers
        )
    )


def test_the_writable_register_inventory_is_exactly_this():
    """The complete list of registers ``arm101`` can write. Widening it is a decision.

    An exact set, not a subset check — a NEW register write fails here even if it
    is perfectly harmless, which is the point: the next person to add one has to
    look at this list, see 9/11 named as the thing being kept out, and say why
    theirs belongs. Update the set deliberately; never to make a red test green.
    """
    expected = {
        5,  # ID                (EEPROM, write_id_baudrate)
        6,  # Baud_Rate         (EEPROM, write_id_baudrate / write_baudrate)
        31,  # Homing_Offset    (EEPROM, write_offset — the re-zero, issue #35)
        40,  # Torque_Enable    (RAM,    enable_torque / clear_overload)
        41,  # Acceleration     (RAM,    write_acceleration)
        42,  # Goal_Position    (RAM,    write_goal_position)
        46,  # Goal_Speed       (RAM,    write_goal_speed)
        48,  # Torque_Limit     (RAM,    write_torque_limit)
        55,  # Lock             (EEPROM, the unlock/relock dance around every EEPROM write)
    }

    assert _WRITABLE_ADDRESSES == expected, (
        "the set of servo registers arm101 can write has changed.\n"
        f"  added:   {sorted(_WRITABLE_ADDRESSES - expected)}\n"
        f"  removed: {sorted(expected - _WRITABLE_ADDRESSES)}\n"
        "If you added a register write, justify it here. Addresses 9-12 "
        "(Min/Max_Position_Limit) are never justifiable — see this module's docstring."
    )
    assert not (_WRITABLE_ADDRESSES & _FORBIDDEN_BYTES)


# ===========================================================================
# 3. Corroboration: what actually reaches the wire, when the bus is driven
# ===========================================================================


class _RecordingPacket:
    """A packet handler that records the (motor, addr, value) of every write."""

    def __init__(self) -> None:
        self.writes: list[tuple[int, int, int]] = []
        self.reads: list[tuple[int, int]] = []

    def write1ByteTxRx(self, port, motor, addr, val):  # noqa: N802 - SDK spelling
        self.writes.append((motor, addr, val))
        return 0, 0

    def write2ByteTxRx(self, port, motor, addr, val):  # noqa: N802 - SDK spelling
        self.writes.append((motor, addr, val))
        return 0, 0

    def read1ByteTxRx(self, port, motor, addr):  # noqa: N802 - SDK spelling
        self.reads.append((motor, addr))
        return 0, 0, 0

    def read2ByteTxRx(self, port, motor, addr):  # noqa: N802 - SDK spelling
        self.reads.append((motor, addr))
        return 0, 0, 0

    def ping(self, port, motor):
        return 777, 0, 0  # model, result=COMM_SUCCESS, error


class _RecordingPort:
    def setBaudRate(self, baudrate):  # noqa: N802 - SDK spelling
        self.baudrate = baudrate


#: Every public method of the bus, with arguments that make it run. Reads are in
#: here too: they must be *seen* to write nothing.
_CALL_TABLE: "dict[str, tuple]" = {
    "read_position": (1,),
    "read_info": (1,),
    "read_lock": (1,),
    "read_offset": (1,),
    "read_torque_limit": (1,),
    "scan": ([1],),
    "enable_torque": (1, True),
    "clear_overload": (1,),
    "write_goal_position": (1, 2048),
    "write_acceleration": (1, 100),
    "write_goal_speed": (1, 400),
    "write_torque_limit": (1, 480),
    "write_offset": (1, 100),
    "write_baudrate": (1, 1_000_000),
    "write_id_baudrate": (1, 2, 1_000_000),
}

#: Public methods deliberately NOT driven: neither addresses a register.
#: ``open`` imports and initialises the real SDK; ``close`` only shuts the port.
_NOT_DRIVEN = {"open", "close"}


@pytest.fixture()
def wired_bus(monkeypatch) -> "tuple[FeetechBus, _RecordingPacket]":
    """A FeetechBus wired to a recording packet handler, with no serial port involved."""
    monkeypatch.setattr(time, "sleep", lambda _seconds: None)  # skip the EEPROM settle

    packet = _RecordingPacket()
    bus = FeetechBus(port="/dev/ttyUSB_fake")
    bus._packet_handler = packet
    bus._port_handler = _RecordingPort()
    bus._open = True
    return bus, packet


def test_every_public_bus_method_is_driven_by_the_live_guard():
    """A new bus method must be added to the call table — it cannot arrive unexamined.

    This is what makes the live check below more than a spot-check. Add
    ``write_position_limits()`` to ``FeetechBus`` and this test fails at once,
    naming it: the only ways out are to drive it (and then its addresses are
    checked) or to declare it register-free.
    """
    public = {
        name
        for name, _member in inspect.getmembers(FeetechBus, predicate=inspect.isfunction)
        if not name.startswith("_")
    }

    assert public == set(_CALL_TABLE) | _NOT_DRIVEN, (
        "FeetechBus's public surface has changed and the live write-surface guard does not "
        "cover it.\n"
        f"  not driven: {sorted(public - set(_CALL_TABLE) - _NOT_DRIVEN)}\n"
        f"  gone:       {sorted((set(_CALL_TABLE) | _NOT_DRIVEN) - public)}"
    )


def test_the_live_write_surface_never_touches_the_limit_registers(wired_bus):
    """Drive the WHOLE bus, watch the wire: no byte lands in 9-12.

    The static scan says these are the only addresses that can be written. This
    runs them: every public method, one recording handler, and the addresses that
    actually arrive must (a) miss the limit registers and (b) be a subset of what
    the scan predicted — which is what stops the analysis above from being a story
    about code rather than a fact about it.
    """
    bus, packet = wired_bus

    for name, args in _CALL_TABLE.items():
        getattr(bus, name)(*args)

    touched = {addr for _motor, addr, _value in packet.writes}

    assert not touched & _FORBIDDEN_BYTES, (
        "driving the bus wrote the position-limit registers: "
        f"{sorted(touched & _FORBIDDEN_BYTES)}"
    )
    assert touched <= _WRITABLE_ADDRESSES, (
        "the bus wrote a register the static scan did not predict — the scan is blind to "
        f"some call site: {sorted(touched - _WRITABLE_ADDRESSES)}"
    )
    assert touched, "the recording handler saw no writes at all; the drive did not happen"


def test_the_live_reads_do_touch_the_limit_registers(wired_bus):
    """The counter-example that proves the live guard is watching the right thing.

    ``read_info`` reads addresses 9 and 11 on every call. If the assertion above
    ever passes because the bus stopped talking to those registers ENTIRELY, that
    is a different (and also unwanted) change — reading the limits is how we know
    they are still the factory 0/4095.
    """
    bus, packet = wired_bus

    bus.read_info(1)

    read_addresses = {addr for _motor, addr in packet.reads}
    assert _MIN_LIMIT_ADDR in read_addresses
    assert _MAX_LIMIT_ADDR in read_addresses
    assert not [w for w in packet.writes if w[1] in _FORBIDDEN_BYTES]
