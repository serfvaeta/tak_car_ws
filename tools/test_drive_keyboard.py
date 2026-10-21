import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from drive_serial import build_velocity_frame
from drive_keyboard import DriveState, decode_input_events, run_drive_loop


EVENT = struct.Struct("@llHHi")


class KeyboardDriveTests(unittest.TestCase):
    def test_key_press_and_release_are_decoded_even_when_reads_split(self):
        press = EVENT.pack(0, 0, 1, 103, 1)
        release = EVENT.pack(0, 0, 1, 103, 0)
        events, pending = decode_input_events(b"", press[:7])
        self.assertEqual(events, [])
        self.assertEqual(pending, press[:7])

        events, pending = decode_input_events(pending, press[7:] + release)
        self.assertEqual(events, [("up", 1), ("up", 0)])
        self.assertEqual(pending, b"")

    def test_release_makes_target_exactly_zero(self):
        state = DriveState(speed_mm_s=100, turn_mrad_s=300)
        state.accept("up", 1)
        self.assertEqual(state.target(), (100, 0))
        state.accept("left", 1)
        self.assertEqual(state.target(), (100, 300))
        state.accept("up", 0)
        self.assertEqual(state.target(), (0, 300))
        state.accept("left", 0)
        self.assertEqual(state.target(), (0, 0))

    def test_space_stops_even_if_direction_key_still_held(self):
        state = DriveState(100, 300)
        state.accept("up", 1)
        state.accept("stop", 1)
        state.accept("up", 2)
        self.assertEqual(state.target(), (0, 0))

    def test_loop_sends_motion_then_zero_on_release_and_quit(self):
        events = iter([[('up', 1)], [], [('up', 0)], [('quit', 1)]])
        writes = []
        run_drive_loop(lambda _: next(events), writes.append, DriveState(100, 300))
        self.assertEqual(writes[0], build_velocity_frame(0))
        self.assertEqual(writes[1:3], [build_velocity_frame(100)] * 2)
        self.assertEqual(writes[3], build_velocity_frame(0))
        self.assertEqual(writes[-3:], [build_velocity_frame(0)] * 3)

    def test_loop_attempts_stop_after_keyboard_interrupt(self):
        writes = []

        def interrupted(_):
            raise KeyboardInterrupt

        with self.assertRaises(KeyboardInterrupt):
            run_drive_loop(interrupted, writes.append, DriveState(100, 300))
        self.assertEqual(writes, [build_velocity_frame(0)] * 4)


if __name__ == "__main__":
    unittest.main()
