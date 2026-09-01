"""
protocol.py -- Python mirror of firmware/ardustep/protocol.h.

THE FROZEN CONTRACT. Every field, order, width and endianness here must match
protocol.h exactly. If you change one, change both in the same commit and bump
PROTO_VERSION.

Layouts (little-endian, packed -- struct format leading '<' disables padding):

    CommandPacket  (PC -> MCU), 18 bytes:
        magic:u8  seq:u8  flags:u8  pos_cmd[3]:i32  spindle:u16  crc:u8
        -> '<BBB iii H B'

    FeedbackPacket (MCU -> PC), 19 bytes:
        magic:u8 seq:u8 status:u8 limits:u8 pos_fb[3]:i32 underruns:u16 crc:u8
        -> '<BBBB iii H B'

    HelloPacket, 4 bytes: magic:u8 seq:u8 protocol:u8 crc:u8
    InfoPacket, 17 bytes: magic:u8 seq:u8 protocol:u8 fw-major:u8 fw-minor:u8
        axes:u8 isr-hz:u32 command-us:u16 max-step-rate:u16 watchdog-ms:u16 crc:u8
"""
import struct

PROTO_VERSION = 2
NUM_AXES = 3
COMMAND_INTERVAL_US = 1000
ISR_HZ = 30000
MAX_STEP_RATE = 15000

CMD_MAGIC = 0xA5
FB_MAGIC = 0x5A
HELLO_MAGIC = 0xC3
INFO_MAGIC = 0x3C

# CommandPacket.flags
FLAG_ENABLE = 0x01
FLAG_ESTOP = 0x02
FLAG_SPINDLE = 0x04
FLAG_CLEAR_FAULT = 0x08

# FeedbackPacket.status
ST_RUNNING = 0x01
ST_ENABLED = 0x02
ST_FAULT = 0x04
ST_UNDERRUN = 0x08

# FeedbackPacket.limits
LIM_X = 0x01
LIM_Y = 0x02
LIM_Z = 0x04

_CMD_FMT = "<BBBiiiHB"   # 18 bytes
_FB_FMT = "<BBBBiiiHB"   # 19 bytes
_HELLO_FMT = "<BBBB"      # 4 bytes
_INFO_FMT = "<BBBBBBIHHHB"  # 17 bytes

CMD_PACKET_LEN = struct.calcsize(_CMD_FMT)
FB_PACKET_LEN = struct.calcsize(_FB_FMT)
HELLO_PACKET_LEN = struct.calcsize(_HELLO_FMT)
INFO_PACKET_LEN = struct.calcsize(_INFO_FMT)
assert CMD_PACKET_LEN == 18, CMD_PACKET_LEN
assert FB_PACKET_LEN == 19, FB_PACKET_LEN
assert HELLO_PACKET_LEN == 4, HELLO_PACKET_LEN
assert INFO_PACKET_LEN == 17, INFO_PACKET_LEN


def crc8(data):
    """CRC-8/SMBus (poly 0x07, init 0x00) -- identical to firmware crc8()."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def pack_command(seq, flags, pos_cmd, spindle=0):
    """Build an 18-byte CommandPacket. pos_cmd is a 3-iterable of int steps."""
    px, py, pz = (int(p) for p in pos_cmd)
    body = struct.pack("<BBBiiiH", CMD_MAGIC, seq & 0xFF, flags & 0xFF,
                       px, py, pz, spindle & 0xFFFF)
    return body + bytes([crc8(body)])


def pack_hello(seq=0, proto_version=PROTO_VERSION):
    body = struct.pack("<BBB", HELLO_MAGIC, seq & 0xFF,
                       proto_version & 0xFF)
    return body + bytes([crc8(body)])


def unpack_feedback(buf):
    """Validate and decode a 19-byte FeedbackPacket.

    Returns a dict, or None if magic/length/CRC fail.
    """
    if len(buf) != FB_PACKET_LEN or buf[0] != FB_MAGIC:
        return None
    if crc8(buf[:-1]) != buf[-1]:
        return None
    magic, seq, status, limits, fx, fy, fz, underruns, _crc = \
        struct.unpack(_FB_FMT, buf)
    return {
        "seq": seq,
        "status": status,
        "limits": limits,
        "pos_fb": (fx, fy, fz),
        "underruns": underruns,
    }


def unpack_info(buf):
    """Validate and decode a startup InfoPacket."""
    if len(buf) != INFO_PACKET_LEN or buf[0] != INFO_MAGIC:
        return None
    if crc8(buf[:-1]) != buf[-1]:
        return None
    (_magic, seq, proto_version, fw_major, fw_minor, axes, isr_hz,
     command_interval_us, max_step_rate, watchdog_ms, _crc) = \
        struct.unpack(_INFO_FMT, buf)
    return {
        "seq": seq,
        "proto_version": proto_version,
        "fw_version": (fw_major, fw_minor),
        "axes": axes,
        "isr_hz": isr_hz,
        "command_interval_us": command_interval_us,
        "max_step_rate": max_step_rate,
        "watchdog_ms": watchdog_ms,
    }


class PacketFramer:
    """Extract fixed-size CRC-protected packets and retain parser counters."""

    def __init__(self, magic, packet_len, unpack):
        self.magic = magic
        self.packet_len = packet_len
        self.unpack = unpack
        self.buffer = bytearray()
        self.crc_failures = 0
        self.resync_events = 0
        self.discarded_bytes = 0

    def feed(self, chunk):
        if chunk:
            self.buffer.extend(chunk)

    def next(self):
        needle = bytes([self.magic])
        while True:
            i = self.buffer.find(needle)
            if i < 0:
                if self.buffer:
                    self.discarded_bytes += len(self.buffer)
                    self.resync_events += 1
                    self.buffer.clear()
                return None
            if i:
                self.discarded_bytes += i
                self.resync_events += 1
                del self.buffer[:i]
            if len(self.buffer) < self.packet_len:
                return None
            frame = bytes(self.buffer[:self.packet_len])
            packet = self.unpack(frame)
            if packet is not None:
                del self.buffer[:self.packet_len]
                return packet
            self.crc_failures += 1
            self.resync_events += 1
            del self.buffer[:1]


class SequenceMatcher:
    """Accept only the reply for the current command, modulo sequence wrap."""

    def __init__(self):
        self.matched = 0
        self.stale = 0
        self.future = 0

    def consider(self, packet, expected):
        distance = (packet["seq"] - expected) & 0xFF
        if distance == 0:
            self.matched += 1
            return packet
        if distance < 128:
            self.future += 1
        else:
            self.stale += 1
        return None


def validate_info(info):
    """Return a human-readable incompatibility, or None when compatible."""
    expected = {
        "proto_version": PROTO_VERSION,
        "axes": NUM_AXES,
        "isr_hz": ISR_HZ,
        "command_interval_us": COMMAND_INTERVAL_US,
        "max_step_rate": MAX_STEP_RATE,
    }
    for key, value in expected.items():
        if info.get(key) != value:
            return "%s mismatch: firmware=%r host=%r" % (
                key, info.get(key), value)
    return None
