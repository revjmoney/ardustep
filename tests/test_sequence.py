import os
import struct
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "linuxcnc"))
import protocol as P  # noqa: E402


class SequenceTests(unittest.TestCase):
    @staticmethod
    def feedback(seq):
        body = struct.pack("<BBBBiiiH", P.FB_MAGIC, seq, 0, 0, 0, 0, 0, 0)
        return body + bytes([P.crc8(body)])

    def test_matching_reply_only(self):
        matcher = P.SequenceMatcher()
        self.assertIsNone(matcher.consider({"seq": 9}, 10))
        self.assertIsNone(matcher.consider({"seq": 11}, 10))
        packet = {"seq": 10, "value": "right"}
        self.assertIs(matcher.consider(packet, 10), packet)
        self.assertEqual((matcher.stale, matcher.future, matcher.matched), (1, 1, 1))

    def test_wraparound_classification(self):
        matcher = P.SequenceMatcher()
        self.assertIsNotNone(matcher.consider({"seq": 0}, 0))
        self.assertIsNone(matcher.consider({"seq": 255}, 0))
        self.assertIsNone(matcher.consider({"seq": 1}, 0))
        self.assertEqual((matcher.stale, matcher.future), (1, 1))

    def test_out_of_order_stream_accepts_only_expected(self):
        framer = P.PacketFramer(P.FB_MAGIC, P.FB_PACKET_LEN, P.unpack_feedback)
        matcher = P.SequenceMatcher()
        framer.feed(self.feedback(254) + self.feedback(0) + self.feedback(255))
        accepted = None
        packet = framer.next()
        while packet is not None:
            accepted = matcher.consider(packet, 255) or accepted
            packet = framer.next()
        self.assertEqual(accepted["seq"], 255)
        self.assertEqual((matcher.stale, matcher.future, matcher.matched), (1, 1, 1))

    def test_ambiguous_half_range_is_classified_stale(self):
        matcher = P.SequenceMatcher()
        matcher.consider({"seq": 128}, 0)
        self.assertEqual(matcher.stale, 1)


if __name__ == "__main__":
    unittest.main()
