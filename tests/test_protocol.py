import os
import re
import struct
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "linuxcnc"))
import protocol as P  # noqa: E402


def make_feedback(seq, positions=(0, 0, 0), status=0, limits=0, underruns=0):
    body = struct.pack("<BBBBiiiH", P.FB_MAGIC, seq, status, limits,
                       positions[0], positions[1], positions[2], underruns)
    return body + bytes([P.crc8(body)])


def make_info(seq=0x42, version=P.PROTO_VERSION):
    body = struct.pack("<BBBBBBIHHH", P.INFO_MAGIC, seq, version, 2, 0,
                       P.NUM_AXES, P.ISR_HZ, P.COMMAND_INTERVAL_US,
                       P.MAX_STEP_RATE, 50)
    return body + bytes([P.crc8(body)])


class ProtocolTests(unittest.TestCase):
    def test_python_constants_match_firmware_header(self):
        path = os.path.join(ROOT, "firmware", "ardustep", "protocol.h")
        with open(path, encoding="utf-8") as source:
            header = source.read()
        expected = {
            "PROTO_VERSION": P.PROTO_VERSION,
            "NUM_AXES": P.NUM_AXES,
            "CMD_PACKET_LEN": P.CMD_PACKET_LEN,
            "FB_PACKET_LEN": P.FB_PACKET_LEN,
            "HELLO_PACKET_LEN": P.HELLO_PACKET_LEN,
            "INFO_PACKET_LEN": P.INFO_PACKET_LEN,
        }
        for name, value in expected.items():
            match = re.search(r"#define\s+%s\s+(\d+)" % name, header)
            self.assertIsNotNone(match, name)
            self.assertEqual(int(match.group(1)), value, name)

    def test_lengths_and_layout(self):
        self.assertEqual(P.CMD_PACKET_LEN, 18)
        self.assertEqual(P.FB_PACKET_LEN, 19)
        self.assertEqual(P.HELLO_PACKET_LEN, 4)
        self.assertEqual(P.INFO_PACKET_LEN, 17)
        packet = P.pack_command(255, P.FLAG_ENABLE,
                                (-2147483648, 0, 2147483647), 65535)
        fields = struct.unpack("<BBBiiiHB", packet)
        self.assertEqual(fields[:3], (P.CMD_MAGIC, 255, P.FLAG_ENABLE))
        self.assertEqual(fields[3:6], (-2147483648, 0, 2147483647))
        self.assertEqual(fields[6], 65535)
        self.assertEqual(fields[7], P.crc8(packet[:-1]))

    def test_known_crc_vector(self):
        self.assertEqual(P.crc8(b"123456789"), 0xF4)

    def test_feedback_extreme_positions(self):
        frame = make_feedback(7, (-2147483648, 0, 2147483647),
                              P.ST_RUNNING, P.LIM_X | P.LIM_Z, 65535)
        feedback = P.unpack_feedback(frame)
        self.assertEqual(feedback["pos_fb"], (-2147483648, 0, 2147483647))
        self.assertEqual(feedback["underruns"], 65535)

    def test_corrupt_feedback_is_rejected(self):
        frame = bytearray(make_feedback(9))
        frame[5] ^= 0x80
        self.assertIsNone(P.unpack_feedback(bytes(frame)))

    def test_hello_and_info(self):
        hello = P.pack_hello(0x42)
        self.assertEqual(hello[:3], bytes((P.HELLO_MAGIC, 0x42, P.PROTO_VERSION)))
        info = P.unpack_info(make_info())
        self.assertIsNone(P.validate_info(info))
        self.assertEqual(info["fw_version"], (2, 0))

    def test_protocol_version_mismatch(self):
        info = P.unpack_info(make_info(version=P.PROTO_VERSION - 1))
        self.assertIn("proto_version mismatch", P.validate_info(info))


class FramerTests(unittest.TestCase):
    def test_resynchronizes_after_junk_and_bad_crc(self):
        bad = bytearray(make_feedback(4))
        bad[-1] ^= 1
        good = make_feedback(5, (1, 2, 3))
        framer = P.PacketFramer(P.FB_MAGIC, P.FB_PACKET_LEN, P.unpack_feedback)
        framer.feed(b"junk" + bad + b"noise" + good)
        self.assertEqual(framer.next()["seq"], 5)
        self.assertEqual(framer.crc_failures, 1)
        self.assertGreaterEqual(framer.resync_events, 2)
        self.assertGreaterEqual(framer.discarded_bytes, 9)

    def test_partial_packet(self):
        frame = make_feedback(8)
        framer = P.PacketFramer(P.FB_MAGIC, P.FB_PACKET_LEN, P.unpack_feedback)
        framer.feed(frame[:5])
        self.assertIsNone(framer.next())
        framer.feed(frame[5:])
        self.assertEqual(framer.next()["seq"], 8)


if __name__ == "__main__":
    unittest.main()
