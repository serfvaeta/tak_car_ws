"""持续显示单基站的距离、方位角和仰角；按 Ctrl+C 退出。"""

import argparse
import sys

from range_pan_serial import BAUDRATE, extract_frames


AOA_HEADER = b"\xff\xff\xff\xff"
AOA_LOCATION_LENGTH = 37


def format_measurement(distance_cm: int, azimuth_deg: int, elevation_deg: int) -> str:
    return f"距离: {distance_cm} cm  方位角: {azimuth_deg}°  仰角: {elevation_deg}°"


def extract_measurements(buffer: bytearray):
    """从连续字节流中取出校验正确的 (距离 cm, 方位角 °, 仰角 °)。"""
    while True:
        start = buffer.find(AOA_HEADER)
        if start < 0:
            if len(buffer) > 3:
                del buffer[:-3]
            return
        if start:
            del buffer[:start]
        if len(buffer) < 6:
            return
        if int.from_bytes(buffer[4:6], "big") != AOA_LOCATION_LENGTH:
            del buffer[0]
            continue
        if len(buffer) < AOA_LOCATION_LENGTH:
            return
        packet = bytes(buffer[:AOA_LOCATION_LENGTH])
        if sum_xor(packet) != 0:
            del buffer[0]
            continue
        del buffer[:AOA_LOCATION_LENGTH]
        if packet[8:10] != b"\x20\x01":
            continue
        distance_cm = int.from_bytes(packet[20:24], "big")
        azimuth_deg = int.from_bytes(packet[24:26], "big", signed=True)
        elevation_deg = int.from_bytes(packet[26:28], "big", signed=True)
        yield distance_cm, azimuth_deg, elevation_deg


def sum_xor(data: bytes) -> int:
    result = 0
    for byte in data:
        result ^= byte
    return result


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="USB-COM 串口，例如 /dev/ttyACM0")
    args = parser.parse_args(argv)

    try:
        import serial
    except ImportError:
        print("缺少 pyserial；请安装 python3-serial", file=sys.stderr)
        return 1

    port = serial.Serial(
        port=None, baudrate=BAUDRATE, timeout=0.1, write_timeout=0.2,
        exclusive=True, rtscts=False, dsrdtr=False,
    )
    port.dtr = False
    port.rts = False
    port.port = args.port
    try:
        port.open()
        usb_buffer = bytearray()
        aoa_buffer = bytearray()
        print("正在监听基站定位数据；按 Ctrl+C 退出", flush=True)
        while True:
            usb_buffer.extend(port.read(128))
            for frame_id, payload in extract_frames(usb_buffer):
                if frame_id != 0x13:
                    continue
                aoa_buffer.extend(payload)
                for distance_cm, azimuth_deg, elevation_deg in extract_measurements(aoa_buffer):
                    print(format_measurement(distance_cm, azimuth_deg, elevation_deg), flush=True)
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f"串口读取失败：{exc}", file=sys.stderr)
        return 1
    finally:
        if port.is_open:
            port.close()


if __name__ == "__main__":
    raise SystemExit(main())
