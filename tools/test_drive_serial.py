import sys
import unittest
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))

from drive_serial import (
    build_velocity_frame,
    has_valid_telemetry,
    main,
    send_pulse,
    wait_for_telemetry,
)


class DriveSerialTests(unittest.TestCase):
    def test_forward_and_reverse_frames_match_controller_protocol(self):
        self.assertEqual(
            build_velocity_frame(100),
            bytes.fromhex("AA 55 0B 50 00 64 00 00 00 00 BE"),
        )
        self.assertEqual(
            build_velocity_frame(-100),
            bytes.fromhex("AA 55 0B 50 FF 9C 00 00 00 00 F5"),
        )

    def test_turn_frames_set_yaw_without_forward_speed(self):
        self.assertEqual(
            build_velocity_frame(0, yaw_mrad_s=300),
            bytes.fromhex("AA 55 0B 50 00 00 00 00 01 2C 87"),
        )
        self.assertEqual(
            build_velocity_frame(0, yaw_mrad_s=-300),
            bytes.fromhex("AA 55 0B 50 00 00 00 00 FE D4 2C"),
        )

    def test_telemetry_requires_complete_valid_frame(self):
        frame = bytes.fromhex("AA 55 19 10") + bytes(20)
        frame += bytes([sum(frame) & 0xFF])
        self.assertTrue(has_valid_telemetry(b"noise" + frame))
        self.assertFalse(has_valid_telemetry(frame[:-1]))
        self.assertFalse(has_valid_telemetry(frame[:-1] + b"\x00"))

    def test_telemetry_can_arrive_in_multiple_reads(self):
        frame = bytes.fromhex("AA 55 19 10") + bytes(20)
        frame += bytes([sum(frame) & 0xFF])
        chunks = iter([b"noise" + frame[:7], frame[7:]])
        self.assertTrue(wait_for_telemetry(lambda _: next(chunks), lambda: 0.0))

    def test_pulse_sends_zero_speed_after_motion(self):
        writes = []
        send_pulse(writes.append, lambda _: None, 100, duration=0.1)
        self.assertEqual(writes[:2], [build_velocity_frame(100)] * 2)
        self.assertEqual(writes[-3:], [build_velocity_frame(0)] * 3)

    def test_pulse_stops_after_interrupted_wait(self):
        writes = []

        def interrupted_sleep(_):
            raise KeyboardInterrupt

        with self.assertRaises(KeyboardInterrupt):
            send_pulse(writes.append, interrupted_sleep, -100, duration=0.1)
        self.assertEqual(writes[-3:], [build_velocity_frame(0)] * 3)

    def test_stop_command_sends_immediately_without_telemetry(self):
        class FakePort:
            def __init__(self, **_):
                self.is_open = False
                self.writes = []

            def open(self):
                self.is_open = True

            def read(self, _):
                raise AssertionError("停车指令不应等待遥测")

            def write(self, data):
                self.writes.append(data)

            def flush(self):
                pass

            def close(self):
                self.is_open = False

        fake_port = FakePort()
        with patch.dict(sys.modules, {"serial": SimpleNamespace(Serial=lambda **_: fake_port)}):
            with redirect_stdout(StringIO()):
                result = main(["stop", "--port", "/dev/fake"])
        self.assertEqual(result, 0)
        self.assertEqual(fake_port.writes, [build_velocity_frame(0)] * 3)


if __name__ == "__main__":
    unittest.main()
