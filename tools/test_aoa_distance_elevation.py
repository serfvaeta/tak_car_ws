import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from aoa_distance_elevation import extract_measurements, format_measurement


# Captured from the connected base station. Distance=0x11 cm,
# azimuth=0x0003 degrees, elevation=0xFFB5 (-75 degrees).
SAMPLE = bytes.fromhex(
    "ff ff ff ff 00 25 03 67 20 01 01 02 00 00 12 21 "
    "00 00 12 21 00 00 00 11 00 03 ff b5 12 34 ae 57 "
    "ca 00 00 00 2e"
)


class AoaDistanceElevationTests(unittest.TestCase):
    def test_captured_packet_decodes_distance_azimuth_and_elevation(self):
        self.assertEqual(list(extract_measurements(bytearray(SAMPLE))), [(17, 3, -75)])

    def test_negative_azimuth_is_signed(self):
        packet = bytearray(SAMPLE)
        packet[24:26] = b"\xff\xd9"  # -39 degrees
        packet[-1] = 0x0B  # XOR checksum after replacing 00 03 with FF D9
        self.assertEqual(list(extract_measurements(packet)), [(17, -39, -75)])

    def test_output_does_not_claim_front_or_back_from_azimuth(self):
        self.assertEqual(
            format_measurement(17, -39, -75),
            "距离: 17 cm  方位角: -39°（右侧角度，前后未知）  仰角: -75°",
        )
        self.assertEqual(
            format_measurement(17, 3, -75),
            "距离: 17 cm  方位角: +3°（左侧角度，前后未知）  仰角: -75°",
        )
        self.assertEqual(
            format_measurement(17, 0, -75),
            "距离: 17 cm  方位角: 0°（左右居中，前后未知）  仰角: -75°",
        )

    def test_packet_can_span_multiple_usb_chunks(self):
        buffer = bytearray(SAMPLE[:3])
        self.assertEqual(list(extract_measurements(buffer)), [])
        buffer.extend(SAMPLE[3:18])
        self.assertEqual(list(extract_measurements(buffer)), [])
        buffer.extend(SAMPLE[18:])
        self.assertEqual(list(extract_measurements(buffer)), [(17, 3, -75)])

    def test_bad_checksum_is_ignored_and_next_packet_recovers(self):
        bad = SAMPLE[:-1] + bytes((SAMPLE[-1] ^ 1,))
        buffer = bytearray(b"noise" + bad + SAMPLE)
        self.assertEqual(list(extract_measurements(buffer)), [(17, 3, -75)])

    def test_other_command_is_not_reported_as_location(self):
        other = bytearray(SAMPLE)
        other[9] = 2
        other[-1] ^= 3
        self.assertEqual(list(extract_measurements(other)), [])


if __name__ == "__main__":
    unittest.main()
