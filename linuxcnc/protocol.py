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
"""
import struct

PROTO_VERSION = 1
NUM_AXES = 3

CMD_MAGIC = 0xA5
FB_MAGIC = 0x5A

# CommandPacket.flags
FLAG_ENABLE = 0x01
FLAG_ESTOP = 0x02
FLAG_SPINDLE = 0x04

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

CMD_PACKET_LEN = struct.calcsize(_CMD_FMT)
FB_PACKET_LEN = struct.calcsize(_FB_FMT)
assert CMD_PACKET_LEN == 18, CMD_PACKET_LEN
assert FB_PACKET_LEN == 19, FB_PACKET_LEN


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
