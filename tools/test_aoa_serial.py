import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from aoa_serial import describe_aoa_frame, has_live_aoa_data


class AoaSerialTests(unittest.TestCase):
    def test_raw_frame_shows_all_received_bytes(self):
        self.assertEqual(
            describe_aoa_frame(0x13, b"\xff\xff\xff\xff\x00\x25"),
            "AOA 原始数据: ff ff ff ff 00 25",
        )

    def test_status_frame_shows_rx_and_overflow_counts(self):
        self.assertEqual(
            describe_aoa_frame(0x14, b"\x00\x00\x01\x02\x00\x03"),
            "AOA 接收字节: 258，缓冲溢出: 3",
        )

    def test_other_frames_are_ignored(self):
        self.assertIsNone(describe_aoa_frame(0x10, bytes(20)))
        self.assertIsNone(describe_aoa_frame(0x14, b"\x00"))

    def test_historical_counter_does_not_mean_live_data(self):
        self.assertFalse(has_live_aoa_data(False, 1, 1))
        self.assertTrue(has_live_aoa_data(False, 1, 2))
        self.assertTrue(has_live_aoa_data(True, 1, 1))


if __name__ == "__main__":
    unittest.main()
