import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from range_pan_serial import build_pan_frame, describe_frame, extract_frames


def frame(frame_id, payload):
    data = bytes((0xAA, 0x55, len(payload) + 5, frame_id)) + payload
    return data + bytes((sum(data) & 0xFF,))


class RangePanProtocolTests(unittest.TestCase):
    def test_pan_command_uses_signed_big_endian_degrees(self):
        self.assertEqual(build_pan_frame(-30), frame(0x61, b"\xFF\xE2"))
        self.assertEqual(build_pan_frame(90), frame(0x61, b"\x00\x5A"))
        with self.assertRaises(ValueError):
            build_pan_frame(91)

    def test_stream_parser_resynchronizes_and_waits_for_complete_frame(self):
        valid = frame(0x11, b"\x03\x01\x2C")
        bad = valid[:-1] + bytes((valid[-1] ^ 1,))
        buffer = bytearray(b"noise" + bad + valid[:4])
        self.assertEqual(list(extract_frames(buffer)), [])
        buffer.extend(valid[4:])
        self.assertEqual(list(extract_frames(buffer)), [(0x11, b"\x03\x01\x2C")])
        self.assertEqual(buffer, bytearray())

    def test_distance_and_servo_ack_are_distinct_from_chassis_telemetry(self):
        self.assertEqual(describe_frame(0x11, b"\x03\x01\x2C"), "XURSC 距离: 300 mm")
        self.assertEqual(describe_frame(0x11, b"\x04\x00\x00"), "XURSC 状态: TIMEOUT")
        self.assertEqual(describe_frame(0x12, b"\xFF\xE2"), "SG90 已执行: -30 度")
        self.assertIsNone(describe_frame(0x10, bytes(20)))

    def test_scan_result_includes_signed_angle_and_status(self):
        self.assertEqual(describe_frame(0x15, b"\xFF\xA6\x03\x02\x58"),
                         "SCAN -90 度: 600 mm")
        self.assertEqual(describe_frame(0x15, b"\x00\x1E\x04\x00\x00"),
                         "SCAN 30 度: TIMEOUT")

    def test_avoid_diagnostic_explains_stop_and_sensor_inputs(self):
        payload = bytes((0, 1, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
        description = describe_frame(0x16, payload)
        self.assertIn("WAIT_FRONT", description)
        self.assertIn("AOA_STALE", description)
        self.assertIn("FWD", description)

    def test_avoid_retry_state_is_reported_as_waiting(self):
        payload = bytes((7, 21, 0x0F, 0, 0, 4, 0, 130, 0, 3, 2, 88,
                         0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0))
        description = describe_frame(0x16, payload)
        self.assertIn("RESCAN_WAIT", description)
        self.assertIn("WAIT_RESCAN", description)


if __name__ == "__main__":
    unittest.main()
