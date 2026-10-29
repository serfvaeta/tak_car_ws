"""只读监听 H60 USB-COM 转发的单基站串口数据。"""

import argparse
import sys
import time

from range_pan_serial import extract_frames


def describe_aoa_frame(frame_id: int, payload: bytes) -> str | None:
    if frame_id == 0x13 and payload:
        return f"AOA 原始数据: {payload.hex(' ')}"
    if frame_id == 0x14 and len(payload) == 6:
        received = int.from_bytes(payload[:4], "big")
        dropped = int.from_bytes(payload[4:], "big")
        return f"AOA 接收字节: {received}，缓冲溢出: {dropped}"
    return None


def has_live_aoa_data(raw_seen: bool, first_received: int | None, last_received: int) -> bool:
    return raw_seen or (first_received is not None and last_received > first_received)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="USB-COM 串口，例如 /dev/ttyACM0")
    parser.add_argument("--seconds", type=float, default=8.0, help="监听时长，默认 8 秒")
    args = parser.parse_args(argv)
    if args.seconds <= 0:
        parser.error("--seconds 必须大于 0")

    try:
        import serial
    except ImportError:
        print("缺少 pyserial；请安装 python3-serial", file=sys.stderr)
        return 1

    port = serial.Serial(
        port=None, baudrate=230400, timeout=0.1, write_timeout=0.2,
        exclusive=True, rtscts=False, dsrdtr=False,
    )
    port.dtr = False
    port.rts = False
    port.port = args.port
    try:
        port.open()
        buffer = bytearray()
        deadline = time.monotonic() + args.seconds
        status_seen = False
        raw_seen = False
        first_received = None
        last_received = 0
        while time.monotonic() < deadline:
            buffer.extend(port.read(128))
            for frame_id, payload in extract_frames(buffer):
                description = describe_aoa_frame(frame_id, payload)
                if description is None:
                    continue
                print(description)
                if frame_id == 0x13:
                    raw_seen = True
                elif frame_id == 0x14:
                    status_seen = True
                    last_received = int.from_bytes(payload[:4], "big")
                    if first_received is None:
                        first_received = last_received
        if not status_seen:
            print("没有收到 AOA 状态帧；请确认已烧录接收诊断固件", file=sys.stderr)
            return 1
        if not has_live_aoa_data(raw_seen, first_received, last_received):
            print("监听期间未收到新的基站字节；请检查供电、TX→PD6 和信标连接", file=sys.stderr)
            return 2
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"串口读取失败：{exc}", file=sys.stderr)
        return 1
    finally:
        if port.is_open:
            port.close()


if __name__ == "__main__":
    raise SystemExit(main())
