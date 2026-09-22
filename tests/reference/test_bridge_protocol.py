import unittest

from bridge_protocol import Frame, crc16_ccitt_false, exchange


class BridgeProtocolTests(unittest.TestCase):
    def test_standard_crc_vector(self):
        self.assertEqual(crc16_ccitt_false(b"123456789"), 0x29B1)

    def test_ping_round_trip(self):
        frame = Frame(0x01, 0x1234)
        self.assertEqual(Frame.decode(frame.encode()), frame)

    def test_exchange_round_trip(self):
        frame = exchange(7, b"\xF6", expected=17)
        self.assertEqual(Frame.decode(frame.encode()), frame)

    def test_crc_corruption_is_rejected(self):
        raw = bytearray(Frame(0x01, 1).encode())
        raw[-1] ^= 0x01
        with self.assertRaisesRegex(ValueError, "CRC mismatch"):
            Frame.decode(bytes(raw))

    def test_oversized_transmit_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "64 bytes"):
            exchange(1, bytes(65), expected=0)


if __name__ == "__main__":
    unittest.main()

