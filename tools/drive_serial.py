"""Send a short speed command through the OpenCTR H60 USB-COM port.

The controller has no command-loss watchdog. If the cable comes loose after
a motion frame, this program cannot deliver its stop frame. Keep the wheels
off the ground and the board's power switch within reach while testing.
"""

import argparse
import math
import struct
import sys
import time


BAUDRATE = 230400
INTERVAL = 0.05
TELEMETRY_LENGTH = 25


def build_velocity_frame(vx_mm_s: int, vy_mm_s: int = 0, yaw_mrad_s: int = 0) -> bytes:
    """Encode X/Y/yaw target speeds as three signed big-endian 16-bit values."""
    data = bytes((0xAA, 0x55, 0x0B, 0x50)) + struct.pack(
        ">hhh", vx_mm_s, vy_mm_s, yaw_mrad_s
    )
    return data + bytes((sum(data) & 0xFF,))


def has_valid_telemetry(data: bytes) -> bool:
    """Recognize the controller's 25-byte, ID 0x10 telemetry frame."""
    for index in range(len(data) - TELEMETRY_LENGTH + 1):
        frame = data[index : index + TELEMETRY_LENGTH]
        if (
            frame[:4] == b"\xAA\x55\x19\x10"
            and (sum(frame[:-1]) & 0xFF) == frame[-1]
        ):
            return True
    return False


def wait_for_telemetry(read, monotonic, timeout: float = 2.0) -> bool:
    """Require a valid telemetry frame before sending any motion command."""
    deadline = monotonic() + timeout
    buffer = b""
    while monotonic() < deadline:
        buffer = (buffer + read(256))[-512:]
        if has_valid_telemetry(buffer):
            return True
    return False


def send_pulse(write, sleep, vx_mm_s: int, duration: float) -> None:
    """Repeat a short motion command and attempt three stop frames on exit."""
    move = build_velocity_frame(vx_mm_s)
    stop = build_velocity_frame(0)
    stop_error = None
    try:
        for _ in range(math.ceil(duration / INTERVAL)):
            write(move)
            sleep(INTERVAL)
    finally:
        for _ in range(3):
            try:
                write(stop)
            except Exception as exc:
                stop_error = exc
        if stop_error is not None:
            raise RuntimeError("停车帧未能可靠发送；请关闭小车电源") from stop_error


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("direction", choices=("forward", "backward", "stop"))
    parser.add_argument("--port", help="通信 Type-C 串口，如 /dev/ttyACM0")
    parser.add_argument("--speed", type=int, default=100, help="速度 mm/s，默认 100")
    parser.add_argument("--duration", type=float, default=0.3, help="运动时长秒，默认 0.3")
    parser.add_argument("--dry-run", action="store_true", help="只显示指令，不打开串口")
    args = parser.parse_args(argv)

    if not 20 <= args.speed <= 200:
        parser.error("--speed 只能在 20 到 200 mm/s 之间")
    if not 0.05 <= args.duration <= 1.0:
        parser.error("--duration 只能在 0.05 到 1.0 秒之间")
    if not args.dry_run and not args.port:
        parser.error("实际发送时必须指定 --port")

    vx = args.speed if args.direction == "forward" else -args.speed
    if args.direction == "stop":
        vx = 0
    if args.dry_run:
        print("运动帧:", build_velocity_frame(vx).hex(" ").upper())
        print("停车帧:", build_velocity_frame(0).hex(" ").upper())
        return 0

    try:
        import serial
    except ImportError:
        print("缺少 pyserial；请安装 python3-serial 或 pip install pyserial", file=sys.stderr)
        return 1

    port = serial.Serial(
        port=None,
        baudrate=BAUDRATE,
        timeout=0.1,
        write_timeout=0.2,
        exclusive=True,
        rtscts=False,
        dsrdtr=False,
    )
    # Set control lines before opening; the board manual says not to use RTS/DTR.
    port.dtr = False
    port.rts = False
    port.port = args.port
    try:
        port.open()
        if vx != 0 and not wait_for_telemetry(port.read, time.monotonic):
            raise RuntimeError("未收到有效的小车遥测；请确认使用 USB-COM 而非 USB-DEBUG")
        if vx == 0:
            for _ in range(3):
                port.write(build_velocity_frame(0))
        else:
            send_pulse(port.write, time.sleep, vx, args.duration)
        port.flush()
        print("已发送停车指令")
        return 0
    except KeyboardInterrupt:
        print("已中断；程序已尝试发送停车指令", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"串口控制失败：{exc}", file=sys.stderr)
        return 1
    finally:
        if port.is_open:
            port.close()


if __name__ == "__main__":
    raise SystemExit(main())
