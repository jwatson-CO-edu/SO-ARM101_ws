"""Minimal serial driver for Feetech STS/SCS bus servos (e.g. STS3215, as used on the
physical SO-ARM101).

Implements just enough of the Feetech half-duplex UART protocol to enable torque, write
goal positions, and read present positions. The wire format is a Dynamixel-Protocol-1.0
lookalike:

    0xFF 0xFF <ID> <LEN> <INSTRUCTION> <PARAM 0> ... <PARAM N> <CHECKSUM>

- ID is the servo's bus address (0-253), or BROADCAST_ID for "every servo, no reply".
- LEN is len(params) + 2 (it counts the instruction byte and the checksum byte too).
- CHECKSUM is ~(ID + LEN + INSTRUCTION + sum(PARAMS)) & 0xFF.
- A servo's status/response packet has the same shape, with an error byte where the
  instruction byte was: 0xFF 0xFF <ID> <LEN> <ERROR> <PARAM...> <CHECKSUM>.

Control-table addresses below match the STS3215 memory map used throughout the LeRobot /
SO-ARM10x community. This talks directly to real hardware: if you are driving a different
Feetech model, verify its control table against the datasheet before trusting these
addresses.
"""

import struct
from typing import Dict, Optional

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "pyserial is required to talk to the physical servos (pip install pyserial)"
    ) from exc


# --- Feetech STS control table addresses (STS3215) ----------------------------------------
ADDR_TORQUE_ENABLE = 40  # 1 byte: 0 = free-spin, 1 = holding/moving
ADDR_GOAL_POSITION = 42  # 2 bytes, little-endian ticks
ADDR_PRESENT_POSITION = 56  # 2 bytes, little-endian ticks

# --- Instructions --------------------------------------------------------------------------
INSTR_PING = 0x01
INSTR_READ = 0x02
INSTR_WRITE = 0x03
INSTR_SYNC_WRITE = 0x83  # write the same address on many servos in one packet

HEADER = b"\xff\xff"
BROADCAST_ID = 0xFE  # "all servos"; the bus never sends a reply to this ID

TICKS_PER_REV = 4096  # 12-bit absolute position sensor -> one full servo turn


class FeetechBusError(RuntimeError):
    """Raised for anything from a serial timeout to a servo-reported error byte."""


def _checksum(payload: bytes) -> int:
    """payload is <ID><LEN><INSTRUCTION><PARAMS...> (i.e. everything but header+checksum)."""
    return (~sum(payload)) & 0xFF


class FeetechBus:
    """Half-duplex serial connection to a chain of Feetech STS bus servos.

    Not thread-safe on its own: the wire is half-duplex, so a write and a read (or two
    writes) from different threads can interleave and corrupt each other's packets. Callers
    that use this from more than one thread (as hardware_driver_node.py does) must hold a
    lock around every call into this class.
    """

    def __init__(
        self,
        port: str,
        baud_rate: int = 1_000_000,
        timeout_s: float = 0.05,
        dry_run: bool = False,
    ) -> None:
        self._dry_run = dry_run
        self._port_name = port
        self._serial: Optional[serial.Serial] = None
        if not dry_run:
            self._serial = serial.Serial(port=port, baudrate=baud_rate, timeout=timeout_s)
        # In dry-run mode there's no real servo to echo back a position, so we remember the
        # last commanded tick count per servo id and read that back instead. This lets the
        # rest of the pipeline (pose_commander_node's "current state" for planning the next
        # move, hardware_driver_node's /joint_states) be exercised end-to-end on a bench with
        # no hardware attached, rather than reading back a frozen, meaningless constant.
        self._dry_run_ticks: Dict[int, int] = {}

    def close(self) -> None:
        if self._serial is not None and self._serial.is_open:
            self._serial.close()

    # -- low-level packet I/O ---------------------------------------------------------------

    def _send_packet(self, servo_id: int, instruction: int, params: bytes) -> None:
        length = len(params) + 2
        body = bytes([servo_id, length, instruction]) + params
        packet = HEADER + body + bytes([_checksum(body)])
        if self._dry_run:
            return
        # Half-duplex: our own outgoing bytes would otherwise sit in the input buffer and
        # be mistaken for the start of the servo's reply.
        self._serial.reset_input_buffer()
        self._serial.write(packet)

    def _read_packet(self, expected_param_len: int) -> bytes:
        """Read one status packet and return its parameter bytes (error byte is checked)."""
        header = self._serial.read(2)
        if header != HEADER:
            raise FeetechBusError(f"bad response header on {self._port_name}: {header!r}")
        servo_id_and_len = self._serial.read(2)
        if len(servo_id_and_len) != 2:
            raise FeetechBusError(f"timed out reading response header on {self._port_name}")
        _, length = servo_id_and_len
        rest = self._serial.read(length)
        if len(rest) != length:
            raise FeetechBusError(f"timed out reading response body on {self._port_name}")
        error, *params_and_checksum = rest
        params = bytes(params_and_checksum[:-1])
        if error != 0:
            raise FeetechBusError(f"servo reported error code 0x{error:02x}")
        if len(params) != expected_param_len:
            raise FeetechBusError(
                f"expected {expected_param_len} response bytes, got {len(params)}"
            )
        return params

    # -- public API ---------------------------------------------------------------------------

    def ping(self, servo_id: int) -> bool:
        """Bench/bring-up check: True if the servo answered, False on timeout or bad reply."""
        if self._dry_run:
            return True
        self._send_packet(servo_id, INSTR_PING, b"")
        try:
            self._read_packet(0)
            return True
        except FeetechBusError:
            return False

    def set_torque_enable(self, servo_id: int, enable: bool) -> None:
        """Energize (or free-spin) a servo's motor. Call with True before commanding motion."""
        self._send_packet(servo_id, INSTR_WRITE, bytes([ADDR_TORQUE_ENABLE, 1 if enable else 0]))
        if not self._dry_run:
            self._read_packet(0)

    def read_position_ticks(self, servo_id: int) -> int:
        if self._dry_run:
            # Default to mid-range (2048/4096) before anything has been commanded, matching
            # a servo's typical mechanical center.
            return self._dry_run_ticks.get(servo_id, TICKS_PER_REV // 2)
        self._send_packet(servo_id, INSTR_READ, bytes([ADDR_PRESENT_POSITION, 2]))
        params = self._read_packet(2)
        (ticks,) = struct.unpack("<H", params)
        return ticks

    def write_position_ticks(self, servo_id: int, ticks: int) -> None:
        ticks = max(0, min(TICKS_PER_REV - 1, int(ticks)))
        params = bytes([ADDR_GOAL_POSITION]) + struct.pack("<H", ticks)
        self._send_packet(servo_id, INSTR_WRITE, params)
        if self._dry_run:
            self._dry_run_ticks[servo_id] = ticks
        else:
            self._read_packet(0)

    def sync_write_position_ticks(self, ticks_by_id: Dict[int, int]) -> None:
        """Write goal positions to several servos in a single instruction packet.

        Using SYNC_WRITE (one packet addressed to BROADCAST_ID) instead of one WRITE per
        servo keeps every joint's setpoint update landing within one packet's transmission
        time of each other, which matters for a multi-joint arm: sending five separate WRITE
        packets back-to-back would move each joint a few milliseconds later than the last,
        visibly "shearing" a trajectory waypoint across the joints.
        """
        if not ticks_by_id:
            return
        clamped = {sid: max(0, min(TICKS_PER_REV - 1, int(t))) for sid, t in ticks_by_id.items()}
        data_len = 2  # bytes per servo's position field
        params = bytes([ADDR_GOAL_POSITION, data_len])
        for servo_id, ticks in clamped.items():
            params += bytes([servo_id]) + struct.pack("<H", ticks)
        self._send_packet(BROADCAST_ID, INSTR_SYNC_WRITE, params)
        # SYNC_WRITE addressed to the broadcast ID never gets a status reply, per protocol.
        if self._dry_run:
            self._dry_run_ticks.update(clamped)


def ticks_to_radians(ticks: int, direction: int, offset_rad: float) -> float:
    """Absolute servo tick count -> URDF joint angle in radians.

    Exact inverse of radians_to_ticks (up to the servo's 1-tick quantization, i.e.
    2*pi/4096 rad).
    """
    turns = ticks / TICKS_PER_REV
    return direction * (turns * 2.0 * 3.141592653589793) - offset_rad


def radians_to_ticks(radians: float, direction: int, offset_rad: float) -> int:
    """URDF joint angle in radians -> absolute servo tick count (0..4095).

    `direction` flips sign for a servo mounted so its positive rotation is the URDF joint's
    negative rotation; `offset_rad` shifts so the servo's own zero-tick position lines up
    with the URDF's 0 rad. Both must match the values used by ticks_to_radians for a given
    joint, or position feedback and commanded position will disagree.
    """
    angle = direction * (radians + offset_rad)
    turns = angle / (2.0 * 3.141592653589793)
    # Modulo wraps to the servo's single-turn 0..4095 range; safe here because every joint's
    # full range of motion (see joint_min_rad/joint_max_rad) is well under one revolution.
    return round(turns * TICKS_PER_REV) % TICKS_PER_REV
